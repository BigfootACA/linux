// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nanosic 803 keyboard MCU
 *
 * The MCU sits behind the magnetic pogo pins of tablets such as the Xiaomi
 * Pad 8 Pro and multiplexes the detachable keyboard, its Precision Touchpad,
 * a consumer-control page and a handful of vendor status reports onto a
 * single I2C slave.
 *
 * Only the framing is proprietary: the payloads are genuine HID input
 * reports, so this is a HID transport driver.  The report descriptors are
 * not readable from the device, they are carried here verbatim as recovered
 * from the vendor driver.
 *
 * The MCU has no flash-resident application.  Every reset leaves it in a
 * small ROM bootloader which has to be fed the RAM image from
 * nanosic/MCU_Upgrade.bin before any report is produced.
 *
 * Copyright (c) 2026 Junhao Xie <bigfoot@radxa.com>
 */

#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/hid.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

/*
 * Every transfer, in both directions, starts with a channel selector byte.
 * The slave address is the same for both channels.
 */
#define NANOSIC_CHAN_APP		0x4c
#define NANOSIC_CHAN_BOOT		0x5c

/* Bootloader opcodes, first byte after the channel selector. */
#define NANOSIC_BOOT_READ_ID		0x04
#define NANOSIC_BOOT_READ_STATUS	0x05
#define NANOSIC_BOOT_WRITE_HEADER	0x06
#define NANOSIC_BOOT_WRITE_DATA		0x07
#define NANOSIC_BOOT_START		0x08
#define NANOSIC_BOOT_RUN		0x09

#define NANOSIC_BOOT_ID			0xc8

/* One bootloader command is always a full 512 byte payload. */
#define NANOSIC_BOOT_FRAME_LEN		512
#define NANOSIC_BOOT_DATA_OFF		9
#define NANOSIC_BOOT_DATA_MAX		(NANOSIC_BOOT_FRAME_LEN - NANOSIC_BOOT_DATA_OFF)

/*
 * The firmware file is a container: the loadable image starts at a fixed
 * offset and opens with a 64 byte header describing where the RAM code goes.
 */
#define NANOSIC_FW_NAME			"nanosic/MCU_Upgrade.bin"
#define NANOSIC_FW_IMAGE_OFF		0x7fc0
#define NANOSIC_FW_HDR_LEN		64
#define NANOSIC_FW_HDR_IMAGE_SIZE	48

/* Application channel: one interrupt drains one fixed size frame. */
#define NANOSIC_FRAME_LEN		68
#define NANOSIC_FRAME_MAGIC		0x57
#define NANOSIC_FRAME_DATA_OFF		3

/* Application channel command frames are always this long. */
#define NANOSIC_CMD_LEN			66

/* Command/response device ids, mirrored between request and reply. */
#define NANOSIC_DEV_HOST		0x18	/* the 803x itself */
#define NANOSIC_DEV_KEYBOARD		0x38	/* the MCU in the keyboard */

#define NANOSIC_CMD_VERSION		0x01
#define NANOSIC_CMD_BACKLIGHT		0x23
#define NANOSIC_CMD_PM_NOTIFY		0x25
#define NANOSIC_CMD_LEDS		0x2e
#define NANOSIC_CMD_KEYPAD_STATE	0x30
#define NANOSIC_CMD_HALL_STATE		0xa1

/*
 * The keyboard backlight is set as a percentage rather than as a raw PWM
 * value, and the indicator LEDs come as one bitmap whose unused upper bits
 * the MCU expects to be set.
 */
#define NANOSIC_BACKLIGHT_MAX		100
#define NANOSIC_LEDS_RESERVED		0xfc
#define NANOSIC_LEDS_CAPS_LOCK		BIT(0)

/* Sub-report ids carried inside a frame. */
#define NANOSIC_ID_MIN			0x02
#define NANOSIC_ID_MAX			0x26

#define NANOSIC_ID_MOUSE		0x02
#define NANOSIC_ID_KEYBOARD		0x05
#define NANOSIC_ID_CONSUMER		0x06
#define NANOSIC_ID_TOUCHPAD		0x19
#define NANOSIC_ID_VENDOR_SHORT		0x22
#define NANOSIC_ID_VENDOR_LONG		0x23
#define NANOSIC_ID_VENDOR_TAIL1		0x24
#define NANOSIC_ID_VENDOR_TAIL2		0x26

/* A report that runs to the end of the frame and terminates the walk. */
#define NANOSIC_LEN_REST		0xff

/* Attach status bits of the 0x38/0xa2 vendor report. */
#define NANOSIC_ATTACH_CONNECTED	BIT(0)
#define NANOSIC_ATTACH_POWER		BIT(1)
#define NANOSIC_ATTACH_POGO		BIT(6)
#define NANOSIC_ATTACH_TIMEOUT		msecs_to_jiffies(500)

#define NANOSIC_ATTACH_REPORTED		(NANOSIC_ATTACH_CONNECTED | \
					 NANOSIC_ATTACH_POWER | \
					 NANOSIC_ATTACH_POGO)

/*
 * The part of a touchpad finger collection that actually lays out report
 * bytes.  The firmware spells the three collections slightly differently --
 * only the first names Usage (Finger) -- but the fields are identical, so
 * only the preamble differs.
 *
 * The firmware spends two bytes of each collection on Usage (Tip Pressure)
 * over a 0..65535 range, yet only ever reports the constant 32, whatever the
 * finger does.  Advertising a pressure axis that wide makes userspace derive
 * a touch-down threshold from the range -- roughly a tenth of it -- which the
 * constant never reaches, so no touch is considered to have landed and the
 * touchpad looks dead.  The bytes still have to be accounted for, so they are
 * declared constant and the tip switch is left to say when a finger is down.
 */
#define NANOSIC_RDESC_FINGER						\
	0x09, 0x42,		/*     Usage (Tip Switch)	*/	\
	0x15, 0x00,		/*     Logical Minimum (0)	*/	\
	0x25, 0x01,		/*     Logical Maximum (1)	*/	\
	0x75, 0x01,		/*     Report Size (1)		*/	\
	0x95, 0x01,		/*     Report Count (1)		*/	\
	0x81, 0x02,		/*     Input (Data,Var,Abs)	*/	\
	0x09, 0x32,		/*     Usage (In Range)		*/	\
	0x81, 0x02,		/*     Input (Data,Var,Abs)	*/	\
	0x09, 0x47,		/*     Usage (Confidence)	*/	\
	0x81, 0x02,		/*     Input (Data,Var,Abs)	*/	\
	0x95, 0x05,		/*     Report Count (5)		*/	\
	0x81, 0x03,		/*     Input (Cnst,Var,Abs)	*/	\
	0x75, 0x08,		/*     Report Size (8)		*/	\
	0x09, 0x51,		/*     Usage (Contact Id)	*/	\
	0x95, 0x01,		/*     Report Count (1)		*/	\
	0x81, 0x02,		/*     Input (Data,Var,Abs)	*/	\
	/* Tip Pressure, constant; see above. */				\
	0x75, 0x10,		/*     Report Size (16)		*/	\
	0x95, 0x01,		/*     Report Count (1)		*/	\
	0x81, 0x03,		/*     Input (Cnst,Var,Abs)	*/	\
	0x05, 0x01,		/*     Usage Page (Gen Desktop)	*/	\
	0x15, 0x00,		/*     Logical Minimum (0)	*/	\
	0x26, 0x7f, 0x0c,	/*     Logical Maximum (3199)	*/	\
	0x75, 0x10,		/*     Report Size (16)		*/	\
	0x55, 0x0f,		/*     Unit Exponent (-1)	*/	\
	0x65, 0x11,		/*     Unit (cm)		*/	\
	0x09, 0x30,		/*     Usage (X)		*/	\
	0x35, 0x00,		/*     Physical Minimum (0)	*/	\
	0x46, 0x5e, 0x00,	/*     Physical Maximum (94)	*/	\
	0x81, 0x02,		/*     Input (Data,Var,Abs)	*/	\
	0x09, 0x31,		/*     Usage (Y)		*/	\
	0x26, 0x57, 0x08,	/*     Logical Maximum (2135)	*/	\
	0x46, 0x2a, 0x00,	/*     Physical Maximum (42)	*/	\
	0x81, 0x02,		/*     Input (Data,Var,Abs)	*/	\
	0xc0			/*   End Collection		*/

/*
 * HID report descriptors, byte for byte as shipped by the vendor driver.
 *
 * The vendor registers four separate HID devices, one per descriptor.  Each
 * descriptor is a complete top-level application collection and the four
 * report ids are distinct, so concatenating them yields one valid descriptor
 * and HID core splits it back into the right input devices by itself.  That
 * also lets hid-multitouch claim the touchpad collection, which is plain
 * Windows Precision Touchpad compliant.
 */
static const u8 nanosic_rdesc[] = {
	/* Keyboard, report id 0x05: boot protocol, 6 usages + 5 LEDs */
	0x05, 0x01,		/* Usage Page (Generic Desktop)	*/
	0x09, 0x06,		/* Usage (Keyboard)		*/
	0xa1, 0x01,		/* Collection (Application)	*/
	0x85, 0x05,		/*   Report ID (5)		*/
	0x05, 0x07,		/*   Usage Page (Keyboard)	*/
	0x19, 0xe0,		/*   Usage Minimum (0xe0)	*/
	0x29, 0xe7,		/*   Usage Maximum (0xe7)	*/
	0x15, 0x00,		/*   Logical Minimum (0)	*/
	0x25, 0x01,		/*   Logical Maximum (1)	*/
	0x75, 0x01,		/*   Report Size (1)		*/
	0x95, 0x08,		/*   Report Count (8)		*/
	0x81, 0x02,		/*   Input (Data,Var,Abs)	*/
	0x81, 0x03,		/*   Input (Cnst,Var,Abs)	*/
	0x95, 0x05,		/*   Report Count (5)		*/
	0x05, 0x08,		/*   Usage Page (LEDs)		*/
	0x19, 0x01,		/*   Usage Minimum (1)		*/
	0x29, 0x05,		/*   Usage Maximum (5)		*/
	0x91, 0x02,		/*   Output (Data,Var,Abs)	*/
	0x95, 0x01,		/*   Report Count (1)		*/
	0x75, 0x03,		/*   Report Size (3)		*/
	0x91, 0x01,		/*   Output (Cnst,Ary,Abs)	*/
	0x95, 0x06,		/*   Report Count (6)		*/
	0x75, 0x08,		/*   Report Size (8)		*/
	0x15, 0x00,		/*   Logical Minimum (0)	*/
	0x26, 0xa4, 0x00,	/*   Logical Maximum (164)	*/
	0x05, 0x07,		/*   Usage Page (Keyboard)	*/
	0x19, 0x00,		/*   Usage Minimum (0)		*/
	0x2a, 0xa4, 0x00,	/*   Usage Maximum (164)	*/
	0x81, 0x00,		/*   Input (Data,Ary,Abs)	*/
	0xc0,			/* End Collection		*/

	/* Mouse, report id 0x02: 5 buttons + 16 bit relative X/Y/wheel */
	0x05, 0x01,		/* Usage Page (Generic Desktop)	*/
	0x09, 0x02,		/* Usage (Mouse)		*/
	0xa1, 0x01,		/* Collection (Application)	*/
	0x85, 0x02,		/*   Report ID (2)		*/
	0x09, 0x01,		/*   Usage (Pointer)		*/
	0xa1, 0x00,		/*   Collection (Physical)	*/
	0x05, 0x09,		/*     Usage Page (Button)	*/
	0x19, 0x01,		/*     Usage Minimum (1)	*/
	0x29, 0x05,		/*     Usage Maximum (5)	*/
	0x15, 0x00,		/*     Logical Minimum (0)	*/
	0x25, 0x01,		/*     Logical Maximum (1)	*/
	0x95, 0x05,		/*     Report Count (5)		*/
	0x75, 0x01,		/*     Report Size (1)		*/
	0x81, 0x02,		/*     Input (Data,Var,Abs)	*/
	0x95, 0x01,		/*     Report Count (1)		*/
	0x75, 0x03,		/*     Report Size (3)		*/
	0x81, 0x01,		/*     Input (Cnst,Ary,Abs)	*/
	0x05, 0x01,		/*     Usage Page (Gen Desktop)	*/
	0x09, 0x30,		/*     Usage (X)		*/
	0x09, 0x31,		/*     Usage (Y)		*/
	0x09, 0x38,		/*     Usage (Wheel)		*/
	0x16, 0x00, 0x80,	/*     Logical Minimum (-32768)	*/
	0x26, 0xff, 0x7f,	/*     Logical Maximum (32767)	*/
	0x75, 0x10,		/*     Report Size (16)		*/
	0x95, 0x03,		/*     Report Count (3)		*/
	0x81, 0x06,		/*     Input (Data,Var,Rel)	*/
	0xc0,			/*   End Collection		*/
	0xc0,			/* End Collection		*/

	/*
	 * Touchpad, report id 0x19: Windows Precision Touchpad, three
	 * fingers, 3200x2136 logical units.  The vendor certification
	 * feature report at the end is what makes HID core tag the whole
	 * device as HID_GROUP_MULTITOUCH_WIN_8.
	 *
	 * The firmware declares two buttons here, but the pad has a single
	 * integrated click and never reports the second bit.  Precision
	 * Touchpad numbers the external left and right buttons 2 and 3, so
	 * hid-multitouch folds usage 2 back onto BTN_LEFT and userspace ends
	 * up with a touchpad that has neither a right button nor, because
	 * two buttons were counted, the clickpad property.  Declaring the one
	 * button that exists gets INPUT_PROP_BUTTONPAD set instead.
	 */
	0x05, 0x0d,		/* Usage Page (Digitizer)	*/
	0x09, 0x05,		/* Usage (Touch Pad)		*/
	0xa1, 0x01,		/* Collection (Application)	*/
	0x85, 0x19,		/*   Report ID (25)		*/
	0x15, 0x00,		/*   Logical Minimum (0)	*/
	0x25, 0x01,		/*   Logical Maximum (1)	*/
	0x35, 0x00,		/*   Physical Minimum (0)	*/
	0x45, 0x01,		/*   Physical Maximum (1)	*/
	0x75, 0x01,		/*   Report Size (1)		*/
	0x95, 0x01,		/*   Report Count (1)		*/
	0x05, 0x09,		/*   Usage Page (Button)	*/
	0x09, 0x01,		/*   Usage (Button 1)		*/
	0x81, 0x02,		/*   Input (Data,Var,Abs)	*/
	0x95, 0x07,		/*   Report Count (7)		*/
	0x81, 0x01,		/*   Input (Cnst,Ary,Abs)	*/

	0x05, 0x0d,		/*   Usage Page (Digitizer)	*/
	0x09, 0x22,		/*   Usage (Finger)		*/
	0xa1, 0x02,		/*   Collection (Logical)	*/
	NANOSIC_RDESC_FINGER,

	/* The firmware leaves these two collections without a usage. */
	0xa1, 0x02,		/*   Collection (Logical)	*/
	0x05, 0x0d,		/*   Usage Page (Digitizer)	*/
	NANOSIC_RDESC_FINGER,

	0xa1, 0x02,		/*   Collection (Logical)	*/
	0x05, 0x0d,		/*   Usage Page (Digitizer)	*/
	NANOSIC_RDESC_FINGER,

	0x05, 0x0d,		/*   Usage Page (Digitizer)	*/
	0x09, 0x54,		/*   Usage (Contact Count)	*/
	0x95, 0x01,		/*   Report Count (1)		*/
	0x75, 0x08,		/*   Report Size (8)		*/
	0x15, 0x00,		/*   Logical Minimum (0)	*/
	0x25, 0x08,		/*   Logical Maximum (8)	*/
	0x81, 0x02,		/*   Input (Data,Var,Abs)	*/
	0x09, 0x55,		/*   Usage (Contact Count Max)	*/
	0xb1, 0x02,		/*   Feature (Data,Var,Abs)	*/
	0x06, 0x00, 0xff,	/*   Usage Page (Vendor)	*/
	0x09, 0xc5,		/*   Usage (0xc5)		*/
	0x15, 0x00,		/*   Logical Minimum (0)	*/
	0x26, 0xff, 0x00,	/*   Logical Maximum (255)	*/
	0x75, 0x08,		/*   Report Size (8)		*/
	0x96, 0x00, 0x01,	/*   Report Count (256)		*/
	0xb1, 0x02,		/*   Feature (Data,Var,Abs)	*/
	0xc0,			/* End Collection		*/

	/* Consumer control, report id 0x06: one 16 bit usage */
	0x05, 0x0c,		/* Usage Page (Consumer)	*/
	0x09, 0x01,		/* Usage (Consumer Control)	*/
	0xa1, 0x01,		/* Collection (Application)	*/
	0x85, 0x06,		/*   Report ID (6)		*/
	0x15, 0x00,		/*   Logical Minimum (0)	*/
	0x26, 0x80, 0x03,	/*   Logical Maximum (896)	*/
	0x19, 0x00,		/*   Usage Minimum (0)		*/
	0x2a, 0x80, 0x03,	/*   Usage Maximum (896)	*/
	0x75, 0x10,		/*   Report Size (16)		*/
	0x95, 0x01,		/*   Report Count (1)		*/
	0x81, 0x00,		/*   Input (Data,Ary,Abs)	*/
	0xc0,			/* End Collection		*/
};

/*
 * Sub-report length, including the leading report id byte.  Index is
 * id - NANOSIC_ID_MIN; a zero entry means the id is not implemented and
 * ends the walk, as it does in the vendor parser.
 */
static const u8 nanosic_report_len[NANOSIC_ID_MAX - NANOSIC_ID_MIN + 1] = {
	[NANOSIC_ID_MOUSE - NANOSIC_ID_MIN]	  = 8,
	[NANOSIC_ID_KEYBOARD - NANOSIC_ID_MIN]	  = 9,
	[NANOSIC_ID_CONSUMER - NANOSIC_ID_MIN]	  = 5,
	[NANOSIC_ID_TOUCHPAD - NANOSIC_ID_MIN]	  = 27,
	[NANOSIC_ID_VENDOR_SHORT - NANOSIC_ID_MIN] = 16,
	[NANOSIC_ID_VENDOR_LONG - NANOSIC_ID_MIN]  = 32,
	[NANOSIC_ID_VENDOR_TAIL1 - NANOSIC_ID_MIN] = NANOSIC_LEN_REST,
	[NANOSIC_ID_VENDOR_TAIL2 - NANOSIC_ID_MIN] = NANOSIC_LEN_REST,
};

struct nanosic {
	struct i2c_client *client;
	struct hid_device *hid;
	struct input_dev *wake_input;
	struct completion attach_known;
	struct led_classdev backlight;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *sleep_gpio;
	struct gpio_desc *wake_gpio;

	struct regulator *vdd;
	struct regulator *dvdd;

	int wake_irq;

	/* Serialises the two-message read against the one-message write. */
	struct mutex io_lock;

	/* Set once the HID device is live and reports may be forwarded. */
	bool ready;

	/* Gates the wake doorbell, which also rings during normal typing. */
	bool suspended;

	u16 product_id;
	u8 attach_state;

	/* Last state handed to the MCU, resent to defeat its idle timeout. */
	struct delayed_work led_work;
	u8 leds;
	u8 backlight_level;

	/* Bounce buffers, too big for the stack. */
	u8 rx[NANOSIC_FRAME_LEN];
	u8 tx[NANOSIC_BOOT_FRAME_LEN + 1];
};

/*
 * Both channels use the same shapes: a read is a one byte write of the
 * channel selector followed by a read, a write is the selector prepended to
 * the payload.
 */
static int nanosic_read(struct nanosic *nano, u8 chan, u8 *buf, u16 len)
{
	struct i2c_client *client = nano->client;
	u8 sel = chan;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &sel,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg))
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int nanosic_write(struct nanosic *nano, u8 chan, const u8 *payload,
			 u16 len)
{
	struct i2c_client *client = nano->client;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = len + 1,
		.buf = nano->tx,
	};
	int ret;

	if (len > NANOSIC_BOOT_FRAME_LEN)
		return -EINVAL;

	nano->tx[0] = chan;
	memcpy(nano->tx + 1, payload, len);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)
		return ret < 0 ? ret : -EIO;

	return 0;
}

/*
 * Application channel command frame.  Layout, with the checksum rule
 * verified against every frame the vendor driver emits:
 *
 *	[0]	 0x32			frame magic
 *	[1]	 0x00
 *	[2],[3]	 complement pair summing to 0x7f, [3] = 0x30 + nargs
 *	[4]	 0x80
 *	[5]	 device id
 *	[6]	 command
 *	[7]	 argument count
 *	[8..]	 arguments
 *	[n]	 (sum of [5..n-1] - 1) & 0xff
 *	[..65]	 zero padding
 *
 * The [3] = 0x30 + nargs relation is inferred: the vendor only ever sends
 * 0x4f/0x30 with no argument and 0x4e/0x31 with one.
 */
static int nanosic_send_cmd(struct nanosic *nano, u8 devid, u8 cmd,
			    const u8 *args, u8 nargs)
{
	u8 buf[NANOSIC_CMD_LEN] = {};
	u8 sum = 0;
	unsigned int i, n;
	int ret;

	if (nargs > NANOSIC_CMD_LEN - 9)
		return -EINVAL;

	buf[0] = 0x32;
	buf[1] = 0x00;
	buf[3] = 0x30 + nargs;
	buf[2] = 0x7f - buf[3];
	buf[4] = 0x80;
	buf[5] = devid;
	buf[6] = cmd;
	buf[7] = nargs;
	memcpy(&buf[8], args, nargs);

	n = 8 + nargs;
	for (i = 5; i < n; i++)
		sum += buf[i];
	buf[n] = sum - 1;

	mutex_lock(&nano->io_lock);
	ret = nanosic_write(nano, NANOSIC_CHAN_APP, buf, sizeof(buf));
	mutex_unlock(&nano->io_lock);

	return ret;
}

/* --------------------------------------------------------------------- */
/* Bootloader                                                            */
/* --------------------------------------------------------------------- */

/*
 * Every bootloader write is followed by a fixed delay.  That delay is the
 * whole flow-control protocol: the status byte the vendor reads between
 * steps is never actually examined.
 */
static int nanosic_boot_write(struct nanosic *nano, const u8 *payload, u16 len)
{
	int ret;

	ret = nanosic_write(nano, NANOSIC_CHAN_BOOT, payload, len);
	msleep(3);

	return ret;
}

static int nanosic_boot_cmd_read(struct nanosic *nano, u8 opcode, u8 *val)
{
	int ret;

	ret = nanosic_boot_write(nano, &opcode, 1);
	if (ret)
		return ret;

	return nanosic_read(nano, NANOSIC_CHAN_BOOT, val, 1);
}

/*
 * The ROM loader will not accept the next command until the previous one has
 * been picked up.  Reading the status byte is what gives it the time; the
 * value itself is not documented and the vendor never looks at it either.
 */
static int nanosic_boot_sync(struct nanosic *nano)
{
	u8 status;

	return nanosic_boot_cmd_read(nano, NANOSIC_BOOT_READ_STATUS, &status);
}

/* Sum of the image, taken a little endian word at a time. */
static u32 nanosic_word_sum(const u8 *data, u32 len)
{
	u32 sum = 0;

	for (; len >= sizeof(u32); len -= sizeof(u32), data += sizeof(u32))
		sum += get_unaligned_le32(data);

	return sum;
}

/*
 * The descriptor the loader wants is not the header as it sits in the file:
 * the fields are reordered, the placement of the RAM code is spelled out
 * rather than implied, and the whole thing is covered by two checksums.
 *
 *	[0]	 0x06
 *	[1..7]	 copied verbatim from the file header
 *	[8..9]	 chip id
 *	[10..13] source, [14..17] destination, [18..21] size of the RAM code
 *	[22..31] entry point, product id, vendor id, version
 *	[53..56] sum of the RAM code
 *	[64]	 sum of everything above it
 *
 * The image checksum at [57..60] and the image size at [49..52] stay zero:
 * this transfer only ever carries RAM code.  Everything is little endian.
 */
#define NANOSIC_BOOT_HDR_LEN		65
#define NANOSIC_CHIP_ID			0x8140
#define NANOSIC_RAM_CODE_SRC		0x2345
#define NANOSIC_RAM_CODE_DST		0x40

static int nanosic_boot_write_header(struct nanosic *nano, const u8 *hdr)
{
	u8 frame[NANOSIC_BOOT_HDR_LEN] = {};
	u32 ram_size;
	u8 sum = 0;
	int ret, i;

	ram_size = get_unaligned_le32(hdr + NANOSIC_FW_HDR_IMAGE_SIZE);

	frame[0] = NANOSIC_BOOT_WRITE_HEADER;
	memcpy(frame + 1, hdr, 7);
	put_unaligned_le16(NANOSIC_CHIP_ID, frame + 8);
	put_unaligned_le32(NANOSIC_RAM_CODE_SRC, frame + 10);
	put_unaligned_le32(NANOSIC_RAM_CODE_DST, frame + 14);
	put_unaligned_le32(ram_size, frame + 18);
	memcpy(frame + 22, hdr + 21, 10);
	put_unaligned_le32(nanosic_word_sum(hdr + NANOSIC_FW_HDR_LEN, ram_size),
			   frame + 53);

	for (i = 1; i < NANOSIC_BOOT_HDR_LEN - 1; i++)
		sum += frame[i];
	frame[NANOSIC_BOOT_HDR_LEN - 1] = sum;

	ret = nanosic_boot_write(nano, frame, sizeof(frame));
	if (ret)
		return ret;

	return 0;
}

/*
 * Body transfer.  Each block is a full 512 byte command carrying at most
 * 503 bytes of image:
 *
 *	[0]	 0x07
 *	[1..4]	 byte offset within the image, big endian
 *	[5..8]	 number of image bytes in this block, big endian
 *	[9..]	 the image bytes, zero padded
 */
static int nanosic_boot_write_body(struct nanosic *nano, const u8 *body,
				   u32 size)
{
	u8 *frame;
	u32 off;
	int ret = 0;

	frame = kzalloc(NANOSIC_BOOT_FRAME_LEN, GFP_KERNEL);
	if (!frame)
		return -ENOMEM;

	for (off = 0; off < size; ) {
		u32 chunk = min_t(u32, NANOSIC_BOOT_DATA_MAX, size - off);

		memset(frame, 0, NANOSIC_BOOT_FRAME_LEN);
		frame[0] = NANOSIC_BOOT_WRITE_DATA;
		put_unaligned_be32(off, &frame[1]);
		put_unaligned_be32(chunk, &frame[5]);
		memcpy(&frame[NANOSIC_BOOT_DATA_OFF], body + off, chunk);

		ret = nanosic_boot_write(nano, frame, NANOSIC_BOOT_FRAME_LEN);
		if (ret) {
			dev_err(&nano->client->dev,
				"firmware block at %u failed: %d\n", off, ret);
			break;
		}

		off += chunk;
	}

	kfree(frame);

	return ret;
}

/*
 * A freshly reset MCU answers the boot id query.  If it does not, the ROM
 * loader is not listening and there is no point pushing an image at it.
 */
static int nanosic_read_boot_id(struct nanosic *nano)
{
	struct device *dev = &nano->client->dev;
	u8 id;
	int ret;

	ret = nanosic_boot_cmd_read(nano, NANOSIC_BOOT_READ_ID, &id);
	if (ret) {
		dev_err(dev, "no answer to the boot id query: %d\n", ret);
		return ret;
	}

	if (id != NANOSIC_BOOT_ID) {
		dev_err(dev, "bad boot id 0x%02x, expected 0x%02x\n",
			id, NANOSIC_BOOT_ID);
		return -ENODEV;
	}

	dev_info(dev, "ROM loader answered, boot id 0x%02x\n", id);

	return 0;
}

static int nanosic_load_image(struct nanosic *nano, const u8 *image,
			      size_t len)
{
	struct device *dev = &nano->client->dev;
	u32 body_size;
	u8 op;
	int ret;

	if (len < NANOSIC_FW_HDR_LEN)
		return -EINVAL;

	body_size = get_unaligned_le32(image + NANOSIC_FW_HDR_IMAGE_SIZE);
	if (body_size != len - NANOSIC_FW_HDR_LEN) {
		dev_err(dev, "image size %u does not match the payload %zu\n",
			body_size, len - NANOSIC_FW_HDR_LEN);
		return -EINVAL;
	}

	nanosic_boot_sync(nano);

	ret = nanosic_boot_write_header(nano, image);
	if (ret) {
		dev_err(dev, "image header rejected: %d\n", ret);
		return ret;
	}

	nanosic_boot_sync(nano);

	ret = nanosic_boot_write_body(nano, image + NANOSIC_FW_HDR_LEN,
				      body_size);
	if (ret)
		return ret;

	op = NANOSIC_BOOT_START;
	ret = nanosic_boot_write(nano, &op, 1);
	if (ret)
		return ret;

	nanosic_boot_sync(nano);

	op = NANOSIC_BOOT_RUN;
	ret = nanosic_boot_write(nano, &op, 1);
	if (ret)
		return ret;

	dev_dbg(dev, "loaded %u bytes of RAM code\n", body_size);

	return 0;
}

static int nanosic_load_firmware(struct nanosic *nano)
{
	struct device *dev = &nano->client->dev;
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, NANOSIC_FW_NAME, dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot load %s\n",
				     NANOSIC_FW_NAME);

	/*
	 * The file is a vendor container; only the tail is the loadable
	 * image, and it always starts at the same offset.
	 */
	if (fw->size <= NANOSIC_FW_IMAGE_OFF + NANOSIC_FW_HDR_LEN) {
		dev_err(dev, "%s is too small (%zu bytes)\n",
			NANOSIC_FW_NAME, fw->size);
		ret = -EINVAL;
		goto out;
	}

	ret = nanosic_load_image(nano, fw->data + NANOSIC_FW_IMAGE_OFF,
				 fw->size - NANOSIC_FW_IMAGE_OFF);
out:
	release_firmware(fw);

	return ret;
}

/* --------------------------------------------------------------------- */
/* Power and reset                                                       */
/* --------------------------------------------------------------------- */

static void nanosic_regulator_off(void *data)
{
	regulator_disable(data);
}

static int nanosic_enable(struct device *dev, struct regulator *reg)
{
	int ret;

	ret = regulator_enable(reg);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, nanosic_regulator_off, reg);
}

/*
 * Take the MCU out of reset and bring the sleep line up.  The two long
 * delays are the vendor's; the MCU needs the first one to get its ROM
 * loader listening and the second before it accepts the sleep level.
 */
static void nanosic_reset(struct nanosic *nano)
{
	gpiod_direction_output(nano->reset_gpio, 1);
	gpiod_direction_output(nano->sleep_gpio, 0);
	msleep(2);

	gpiod_set_value_cansleep(nano->reset_gpio, 0);
	msleep(500);

	gpiod_set_value_cansleep(nano->sleep_gpio, 1);
	msleep(50);
}

/* --------------------------------------------------------------------- */
/* Vendor status reports                                                 */
/* --------------------------------------------------------------------- */

static int nanosic_leds_apply(struct nanosic *nano);

/*
 * The four vendor report ids all carry the same reply envelope, which
 * mirrors bytes [5] and [6] of the request:
 *
 *	d[0] device id, d[1] 0x80, d[2] command, then command specific data
 *
 * where d is the sub-report plus two.
 */
static void nanosic_vendor_report(struct nanosic *nano, const u8 *report,
				  unsigned int len)
{
	struct device *dev = &nano->client->dev;
	const u8 *d = report + 2;
	bool attach_changed;
	u8 attach;

	if (len < 2 + 3)
		return;
	len -= 2;

	if (d[1] != 0x80)
		return;

	if (d[0] == NANOSIC_DEV_HOST && d[2] == NANOSIC_CMD_VERSION) {
		if (len < 25)
			return;
		dev_info(dev, "host MCU version %.20s\n", &d[5]);
		return;
	}

	if (d[0] != NANOSIC_DEV_KEYBOARD)
		return;

	switch (d[2]) {
	case NANOSIC_CMD_VERSION:
		if (len < 11)
			return;
		nano->product_id = get_unaligned_le16(&d[9]);
		dev_info(dev,
			 "keyboard %04x touchpad %04x, product id %04x\n",
			 get_unaligned_le16(&d[4]),
			 get_unaligned_le16(&d[6]), nano->product_id);
		break;

	case 0xa2:
		/*
		 * Attach, pogo power and holster state in one bitmap.  The
		 * firmware calls the whole byte its hall status and flips
		 * bits in it that say nothing about whether the keyboard is
		 * there -- attaching one reports 0x03 and then 0x23 -- so
		 * compare only the bits that get reported, or the same line
		 * is logged twice every time.
		 */
		if (len < 8)
			return;
		attach = d[7] & NANOSIC_ATTACH_REPORTED;

		/*
		 * The keyboard is the only thing that can tell laptop mode
		 * from tablet mode apart.  The folio hall sensors cannot: a
		 * cover folded out of the way and no cover at all both read
		 * as no magnet, and a bare tablet is the shipping
		 * configuration.  The input core drops the report if the
		 * state has not moved, so this can run on every update.
		 */
		if (nano->wake_input) {
			input_report_switch(nano->wake_input, SW_TABLET_MODE,
					    !(attach & NANOSIC_ATTACH_CONNECTED));
			input_sync(nano->wake_input);
		}

		attach_changed = nano->attach_state != attach;
		nano->attach_state = attach;
		complete_all(&nano->attach_known);

		if (!attach_changed)
			break;
		dev_info(dev, "keyboard %s%s%s\n",
			 attach & NANOSIC_ATTACH_CONNECTED ? "attached" :
							     "detached",
			 attach & NANOSIC_ATTACH_POWER ? ", powered" : "",
			 attach & NANOSIC_ATTACH_POGO ? ", pogo" : "");

		/*
		 * A keyboard that was detached with its caps lock indicator
		 * lit comes back with it still lit, so the indicator would
		 * disagree with the state the input layer thinks it is in.
		 * Push what we have, which also puts the backlight back where
		 * userspace left it.
		 */
		if (attach & NANOSIC_ATTACH_CONNECTED)
			nanosic_leds_apply(nano);
		break;

	case 0xa4:
		/* Hall sensor of the folio cover, 0xff means no reading. */
		if (len < 7 || d[6] == 0xff)
			return;
		dev_dbg(dev, "cover hall %u\n", d[6]);
		break;

	case NANOSIC_CMD_KEYPAD_STATE:
		break;

	default:
		dev_dbg(dev, "unhandled vendor reply 0x%02x\n", d[2]);
		break;
	}
}

/* --------------------------------------------------------------------- */
/* Frame parsing                                                         */
/* --------------------------------------------------------------------- */

static bool nanosic_is_hid_report(u8 id)
{
	return id == NANOSIC_ID_MOUSE || id == NANOSIC_ID_KEYBOARD ||
	       id == NANOSIC_ID_CONSUMER || id == NANOSIC_ID_TOUCHPAD;
}

/*
 * A frame batches however many sub-reports the MCU had queued.  Walk them
 * on the frame's own bounds rather than on the vendor's byte counter, which
 * can wrap and run off the end of the buffer.
 */
static void nanosic_parse_frame(struct nanosic *nano, const u8 *frame)
{
	struct device *dev = &nano->client->dev;
	const u8 *p = frame + NANOSIC_FRAME_DATA_OFF;
	const u8 *end = frame + NANOSIC_FRAME_LEN;

	if ((frame[0] & 0x7f) != NANOSIC_FRAME_MAGIC) {
		dev_dbg(dev, "bad frame magic 0x%02x\n", frame[0]);
		return;
	}

	/*
	 * The third byte is one of four values the vendor accepts without
	 * distinguishing them; anything else means the frame is not ours.
	 */
	switch (frame[2]) {
	case 0x39:
	case 0x4a:
	case 0x5b:
	case 0x6c:
		break;
	default:
		dev_dbg(dev, "bad frame type 0x%02x\n", frame[2]);
		return;
	}

	while (p < end) {
		unsigned int len;
		bool last = false;
		u8 id = p[0];

		if (id < NANOSIC_ID_MIN || id > NANOSIC_ID_MAX)
			break;

		len = nanosic_report_len[id - NANOSIC_ID_MIN];
		if (!len)
			break;

		if (len == NANOSIC_LEN_REST) {
			len = end - p;
			last = true;
		}

		if (p + len > end)
			break;

		if (nanosic_is_hid_report(id))
			hid_input_report(nano->hid, HID_INPUT_REPORT,
					 (u8 *)p, len, 1);
		else
			nanosic_vendor_report(nano, p, len);

		p += len;

		if (last)
			break;
	}
}

static irqreturn_t nanosic_irq(int irq, void *data)
{
	struct nanosic *nano = data;
	int ret;

	if (!nano->ready)
		return IRQ_HANDLED;

	mutex_lock(&nano->io_lock);
	ret = nanosic_read(nano, NANOSIC_CHAN_APP, nano->rx, sizeof(nano->rx));
	mutex_unlock(&nano->io_lock);

	if (ret) {
		dev_dbg(&nano->client->dev, "frame read failed: %d\n", ret);
		return IRQ_HANDLED;
	}

	nanosic_parse_frame(nano, nano->rx);

	return IRQ_HANDLED;
}

/*
 * The wake line is a plain doorbell: it never carries data and needs no
 * acknowledge.  The MCU rings it for every key it has to deliver rather than
 * only for the ones that arrive while the system is down, so gate the key
 * report on having actually been suspended.  Reporting it unconditionally
 * buries userspace in KEY_WAKEUP on every keystroke, and the keystroke
 * itself already arrives over the normal HID path.
 */
static irqreturn_t nanosic_wake_irq(int irq, void *data)
{
	struct nanosic *nano = data;

	pm_wakeup_event(&nano->client->dev, 1000);

	if (!READ_ONCE(nano->suspended))
		return IRQ_HANDLED;

	WRITE_ONCE(nano->suspended, false);

	if (!nano->wake_input)
		return IRQ_HANDLED;

	input_report_key(nano->wake_input, KEY_WAKEUP, 1);
	input_sync(nano->wake_input);
	input_report_key(nano->wake_input, KEY_WAKEUP, 0);
	input_sync(nano->wake_input);

	return IRQ_HANDLED;
}

/* --------------------------------------------------------------------- */
/* HID transport                                                         */
/* --------------------------------------------------------------------- */

static int nanosic_hid_parse(struct hid_device *hid)
{
	return hid_parse_report(hid, nanosic_rdesc, sizeof(nanosic_rdesc));
}

static int nanosic_hid_start(struct hid_device *hid)
{
	return 0;
}

static void nanosic_hid_stop(struct hid_device *hid)
{
}

static int nanosic_hid_open(struct hid_device *hid)
{
	return 0;
}

static void nanosic_hid_close(struct hid_device *hid)
{
}

/*
 * Feature reports would need vendor commands that are not known, so refuse
 * them rather than make something up; hid-multitouch falls back to the
 * descriptor's own maxima when the contact count feature cannot be read.
 */
static int nanosic_hid_raw_request(struct hid_device *hid, unsigned char id,
				   u8 *buf, size_t len,
				   unsigned char type, int reqtype)
{
	return -EOPNOTSUPP;
}

/*
 * Both indicators are extinguished by the MCU itself once the keyboard has
 * been idle for fifteen seconds, and pressing a key afterwards does not
 * bring them back.  That would make a write to the LED class device look
 * like it had been silently undone, so keep resending the state while
 * anything is lit and leave idle dimming to whoever owns the policy.
 */
#define NANOSIC_LED_REFRESH_MS		10000

static int nanosic_leds_send(struct nanosic *nano)
{
	int ret;

	ret = nanosic_send_cmd(nano, NANOSIC_DEV_KEYBOARD, NANOSIC_CMD_LEDS,
			       &nano->leds, 1);
	if (ret)
		return ret;

	return nanosic_send_cmd(nano, NANOSIC_DEV_KEYBOARD,
				NANOSIC_CMD_BACKLIGHT, &nano->backlight_level,
				1);
}

static bool nanosic_leds_lit(struct nanosic *nano)
{
	return nano->backlight_level || nano->leds != NANOSIC_LEDS_RESERVED;
}

static void nanosic_led_refresh(struct work_struct *work)
{
	struct nanosic *nano = container_of(to_delayed_work(work),
					    struct nanosic, led_work);

	nanosic_leds_send(nano);

	schedule_delayed_work(&nano->led_work,
			      msecs_to_jiffies(NANOSIC_LED_REFRESH_MS));
}

static int nanosic_leds_apply(struct nanosic *nano)
{
	if (nanosic_leds_lit(nano))
		mod_delayed_work(system_wq, &nano->led_work,
				 msecs_to_jiffies(NANOSIC_LED_REFRESH_MS));
	else
		cancel_delayed_work(&nano->led_work);

	return nanosic_leds_send(nano);
}

/*
 * The keyboard collection declares the five standard LED usages, but Caps
 * Lock is the only indicator the hardware has, and it is not driven by the
 * report itself: the MCU wants a vendor command carrying the indicator
 * bitmap, with the bits it does not implement left set.
 */
static int nanosic_hid_output_report(struct hid_device *hid, u8 *buf,
				     size_t len)
{
	struct nanosic *nano = hid->driver_data;
	u8 leds = NANOSIC_LEDS_RESERVED;
	int ret;

	if (len < 2 || buf[0] != NANOSIC_ID_KEYBOARD)
		return -EINVAL;

	/* HID LED usage 0x02, Caps Lock. */
	if (buf[1] & BIT(1))
		leds |= NANOSIC_LEDS_CAPS_LOCK;

	nano->leds = leds;

	ret = nanosic_leds_apply(nano);

	return ret ? ret : len;
}

static int nanosic_backlight_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct nanosic *nano = container_of(cdev, struct nanosic, backlight);

	nano->backlight_level = brightness;

	return nanosic_leds_apply(nano);
}

static int nanosic_register_backlight(struct nanosic *nano)
{
	struct device *dev = &nano->client->dev;

	nano->backlight.name = "nanosic::kbd_backlight";
	nano->backlight.max_brightness = NANOSIC_BACKLIGHT_MAX;
	nano->backlight.brightness_set_blocking = nanosic_backlight_set;
	nano->backlight.flags = LED_CORE_SUSPENDRESUME;

	return devm_led_classdev_register(dev, &nano->backlight);
}

static const struct hid_ll_driver nanosic_hid_ll_driver = {
	.parse = nanosic_hid_parse,
	.start = nanosic_hid_start,
	.stop = nanosic_hid_stop,
	.open = nanosic_hid_open,
	.close = nanosic_hid_close,
	.raw_request = nanosic_hid_raw_request,
	.output_report = nanosic_hid_output_report,
};

static void nanosic_hid_destroy(void *data)
{
	hid_destroy_device(data);
}

static int nanosic_register_hid(struct nanosic *nano)
{
	struct device *dev = &nano->client->dev;
	struct hid_device *hid;
	int ret;

	hid = hid_allocate_device();
	if (IS_ERR(hid))
		return PTR_ERR(hid);

	hid->driver_data = nano;
	hid->ll_driver = &nanosic_hid_ll_driver;
	hid->dev.parent = dev;
	hid->bus = BUS_I2C;
	hid->vendor = 0x15d9;
	hid->product = 0x00a1;
	hid->version = 0x0100;

	strscpy(hid->name, "Nanosic Keyboard");
	strscpy(hid->phys, dev_name(dev));

	ret = hid_add_device(hid);
	if (ret) {
		hid_destroy_device(hid);
		return ret;
	}

	nano->hid = hid;

	return devm_add_action_or_reset(dev, nanosic_hid_destroy, hid);
}

static int nanosic_register_wake_input(struct nanosic *nano)
{
	struct device *dev = &nano->client->dev;
	struct input_dev *input;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "Nanosic Keyboard Wakeup";
	input->phys = dev_name(dev);
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x0827;
	input->id.product = 0x0094;

	input_set_capability(input, EV_KEY, KEY_WAKEUP);
	input_set_capability(input, EV_SW, SW_TABLET_MODE);

	if (!(nano->attach_state & NANOSIC_ATTACH_CONNECTED))
		__set_bit(SW_TABLET_MODE, input->sw);

	nano->wake_input = input;

	return input_register_device(input);
}

/* --------------------------------------------------------------------- */
/* Probe                                                                 */
/* --------------------------------------------------------------------- */

static int nanosic_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct nanosic *nano;
	int ret;

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "no data interrupt\n");

	nano = devm_kzalloc(dev, sizeof(*nano), GFP_KERNEL);
	if (!nano)
		return -ENOMEM;

	nano->client = client;
	init_completion(&nano->attach_known);
	i2c_set_clientdata(client, nano);

	ret = devm_mutex_init(dev, &nano->io_lock);
	if (ret)
		return ret;

	nano->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(nano->vdd))
		return dev_err_probe(dev, PTR_ERR(nano->vdd), "no vdd\n");

	/*
	 * The 1.1V digital rail is optional: on some boards it is not the
	 * AP's to vote for, and it is already up by the time we probe.
	 */
	nano->dvdd = devm_regulator_get_optional(dev, "dvdd");
	if (IS_ERR(nano->dvdd)) {
		if (PTR_ERR(nano->dvdd) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(nano->dvdd),
					     "no dvdd\n");
		nano->dvdd = NULL;
	}

	/* Left as found so that nothing is driven before the rails are up. */
	nano->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(nano->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(nano->reset_gpio),
				     "no reset gpio\n");

	nano->sleep_gpio = devm_gpiod_get(dev, "sleep", GPIOD_ASIS);
	if (IS_ERR(nano->sleep_gpio))
		return dev_err_probe(dev, PTR_ERR(nano->sleep_gpio),
				     "no sleep gpio\n");

	nano->wake_gpio = devm_gpiod_get_optional(dev, "wakeup", GPIOD_IN);
	if (IS_ERR(nano->wake_gpio))
		return dev_err_probe(dev, PTR_ERR(nano->wake_gpio),
				     "no wakeup gpio\n");

	ret = nanosic_enable(dev, nano->vdd);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable vdd\n");

	if (nano->dvdd) {
		ret = nanosic_enable(dev, nano->dvdd);
		if (ret)
			return dev_err_probe(dev, ret, "cannot enable dvdd\n");
	}

	nanosic_reset(nano);

	ret = nanosic_read_boot_id(nano);
	if (ret)
		return ret;

	ret = nanosic_load_firmware(nano);
	if (ret)
		return ret;

	/* The application firmware needs this long before it answers. */
	msleep(200);

	ret = nanosic_register_hid(nano);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register HID device\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, nanosic_irq,
					IRQF_ONESHOT, "nanosic", nano);
	if (ret)
		return dev_err_probe(dev, ret, "cannot claim data irq\n");

	if (nano->wake_gpio) {
		nano->wake_irq = gpiod_to_irq(nano->wake_gpio);
		if (nano->wake_irq > 0) {
			ret = devm_request_threaded_irq(dev, nano->wake_irq,
							NULL,
							nanosic_wake_irq,
							IRQF_ONESHOT |
							IRQF_TRIGGER_RISING,
							"nanosic-wake", nano);
			if (ret)
				return dev_err_probe(dev, ret,
						     "cannot claim wake irq\n");
		}
	}

	device_init_wakeup(dev, true);

	nano->leds = NANOSIC_LEDS_RESERVED;

	ret = devm_delayed_work_autocancel(dev, &nano->led_work,
					   nanosic_led_refresh);
	if (ret)
		return ret;

	ret = nanosic_register_backlight(nano);
	if (ret)
		return dev_err_probe(dev, ret,
				     "cannot register the backlight\n");

	/* From here on the interrupt handler may talk to HID core. */
	nano->ready = true;

	/*
	 * Ask for the versions and the attach state.  The answers arrive
	 * asynchronously as vendor reports; the product id in particular is
	 * only ever reported, never readable on demand.
	 */
	nanosic_send_cmd(nano, NANOSIC_DEV_HOST, NANOSIC_CMD_VERSION, NULL, 0);
	msleep(10);
	nanosic_send_cmd(nano, NANOSIC_DEV_KEYBOARD, NANOSIC_CMD_HALL_STATE,
			 (const u8 []){ 0x01 }, 1);

	if (!wait_for_completion_timeout(&nano->attach_known,
					 NANOSIC_ATTACH_TIMEOUT))
		dev_warn(dev, "no attach state from the MCU; assuming no keyboard\n");

	ret = nanosic_register_wake_input(nano);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register wake input\n");

	msleep(10);
	nanosic_send_cmd(nano, NANOSIC_DEV_KEYBOARD, NANOSIC_CMD_VERSION,
			 NULL, 0);

	return 0;
}

static void nanosic_remove(struct i2c_client *client)
{
	struct nanosic *nano = i2c_get_clientdata(client);

	nano->ready = false;
}

static int nanosic_suspend(struct device *dev)
{
	struct nanosic *nano = dev_get_drvdata(dev);
	u8 off = 0x00;

	/* Tell the MCU the screen is going away, then park the sleep line. */
	cancel_delayed_work_sync(&nano->led_work);
	nanosic_send_cmd(nano, NANOSIC_DEV_KEYBOARD, NANOSIC_CMD_PM_NOTIFY,
			 &off, 1);
	gpiod_set_value_cansleep(nano->sleep_gpio, 0);

	if (device_may_wakeup(dev) && nano->wake_irq > 0)
		enable_irq_wake(nano->wake_irq);

	WRITE_ONCE(nano->suspended, true);

	return 0;
}

static int nanosic_resume(struct device *dev)
{
	struct nanosic *nano = dev_get_drvdata(dev);
	u8 on = 0x01;

	if (device_may_wakeup(dev) && nano->wake_irq > 0)
		disable_irq_wake(nano->wake_irq);

	/*
	 * Interrupts are unmasked before this runs, so a doorbell that really
	 * did wake us has already reported itself; this only covers the case
	 * where something else brought the system back.
	 */
	WRITE_ONCE(nano->suspended, false);

	gpiod_set_value_cansleep(nano->sleep_gpio, 1);
	msleep(50);

	nanosic_send_cmd(nano, NANOSIC_DEV_KEYBOARD, NANOSIC_CMD_PM_NOTIFY,
			 &on, 1);

	/* The MCU comes back with both indicators dark. */
	nanosic_leds_apply(nano);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nanosic_pm_ops, nanosic_suspend,
				nanosic_resume);

static const struct of_device_id nanosic_of_match[] = {
	{ .compatible = "nanosic,803" },
	{ }
};
MODULE_DEVICE_TABLE(of, nanosic_of_match);

static const struct i2c_device_id nanosic_i2c_id[] = {
	{ "nanosic-803" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nanosic_i2c_id);

static struct i2c_driver nanosic_driver = {
	.driver = {
		.name = "nanosic",
		.of_match_table = nanosic_of_match,
		.pm = pm_sleep_ptr(&nanosic_pm_ops),
	},
	.probe = nanosic_probe,
	.remove = nanosic_remove,
	.id_table = nanosic_i2c_id,
};
module_i2c_driver(nanosic_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@radxa.com>");
MODULE_DESCRIPTION("Nanosic 803 keyboard MCU");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(NANOSIC_FW_NAME);
