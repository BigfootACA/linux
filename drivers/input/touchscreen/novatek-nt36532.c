// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36532 SPI touchscreen driver
 *
 * Found on the Xiaomi Pad 8 Pro (codename "piano"), where the NT36532 is the
 * touch half of the TDDI controller that also drives the display.
 *
 * Copyright (c) 2026 BigfootACA <bigfoot@classfun.cn>
 *
 * The vendor driver for this device runs the controller in Xiaomi's "touch host
 * processing" mode, where the kernel only forwards raw capacitance frames to a
 * proprietary userspace daemon and never reports input events itself. The
 * shipped firmware has no other mode: the classic Novatek point report at the
 * start of the event buffer is left untouched, and nothing in the frame carries
 * coordinates the firmware worked out on its own. This driver therefore finds
 * the fingers in the raw frame itself.
 */

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#include <drm/drm_panel.h>

/* The first byte of a transfer carries the register offset and the direction */
#define NVT_SPI_WRITE_MASK(a)		((a) | BIT(7))
#define NVT_SPI_READ_MASK(a)		((a) & GENMASK(6, 0))
/* Reads are preceded by one dummy byte once the offset has been shifted out */
#define NVT_DUMMY_BYTES			1

#define NVT_PAGE_SET_CMD		0xff

/* Hardware registers, addressed through the paging scheme */
#define NVT_CHIP_VER_TRIM_ADDR		0x1fb104
/* Bit 0 is set on a single die part and clear on a cascaded one */
#define NVT_ENB_CASC_ADDR		0x1fb12c
#define NVT_ENB_CASC_SINGLE		BIT(0)
#define NVT_SWRST_SIF_ADDR		0x1fb43e

/* Bootloader registers driving the firmware download */
#define NVT_G_ILM_CHECKSUM_ADDR		0x1fb500
#define NVT_G_DLM_CHECKSUM_ADDR		0x1fb504
#define NVT_BOOT_RDY_ADDR		0x1fb50d
#define NVT_ILM_LENGTH_ADDR		0x1fb518
#define NVT_R_ILM_CHECKSUM_ADDR		0x1fb520
#define NVT_R_DLM_CHECKSUM_ADDR		0x1fb524
#define NVT_ILM_DES_ADDR		0x1fb528
#define NVT_DLM_DES_ADDR		0x1fb52c
#define NVT_DLM_LENGTH_ADDR		0x1fb530
#define NVT_BLD_CRC_FLAG_ADDR		0x1fb533
#define NVT_BLD_CRC_ILM_OK		BIT(0)
#define NVT_BLD_CRC_DLM_OK		BIT(1)
#define NVT_BLD_CRC_DONE		BIT(2)
/*
 * A cascaded part has the primary die mirror everything the host writes into
 * the secondary one.  The register self clears once that queue has drained.
 */
#define NVT_TX_AUTO_COPY_ADDR		0x1fc925
#define NVT_TX_AUTO_COPY_START		0x56

#define NVT_CMD_BOOTLOADER_RESET	0x69

/*
 * Offsets inside the firmware event buffer.  The buffer itself sits at a
 * different address depending on whether the part is cascaded.
 */
#define NVT_EVENT_BUF_ADDR_SINGLE	0x125800
#define NVT_EVENT_BUF_ADDR_CASCADE	0x11c400
#define NVT_EVENT_MAP_HOST_CMD		0x50
#define NVT_EVENT_MAP_RESET_COMPLETE	0x60
#define NVT_EVENT_MAP_FWINFO		0x78

#define NVT_HOST_CMD_DEEP_SLEEP		0x11
#define NVT_HOST_CMD_CRC_ENABLE		0xae

/* Firmware reset progress, reported at NVT_EVENT_MAP_RESET_COMPLETE */
#define NVT_RESET_STATE_INIT		0xa0
#define NVT_RESET_STATE_REK_FINISH	0xa2
#define NVT_RESET_STATE_MAX		0xaf

#define NVT_MAX_FINGERS			10

/*
 * The event buffer is followed by the raw frame the firmware produces once per
 * scan: a header, then one signed 16 bit mutual capacitance sample per node in
 * row major order.
 */
#define NVT_EVENT_BUF_LEN		256
#define NVT_FRAME_HDR_LEN		64
#define NVT_FRAME_CRC_LEN		8
#define NVT_FRAME_CRC_LEN_BAR		16
#define NVT_FRAME_NUM_COL		48
#define NVT_FRAME_NUM_ROW		49
#define NVT_FRAME_DATA_TYPE		56
/* The only type this driver knows: fingers, without an attached stylus */
#define NVT_FRAME_TYPE_FINGER		3

#define NVT_MAX_COLS			64
#define NVT_MAX_ROWS			48
#define NVT_MAX_CELLS			(NVT_MAX_COLS * NVT_MAX_ROWS)
#define NVT_FRAME_BUF_LEN		(NVT_FRAME_HDR_LEN + \
					 NVT_MAX_CELLS * 2 + 1 + NVT_DUMMY_BYTES)

/*
 * A finger lifts a node by well over a thousand counts while the frame to frame
 * noise stays within ten, so the two thresholds only have to be far enough
 * apart to keep the edge of a contact from flickering.
 */
#define NVT_TOUCH_ON			200
#define NVT_TOUCH_OFF			60
/* Nodes further from the baseline than this are assumed to be covered */
#define NVT_BASELINE_LIMIT		40

/* How far a contact may travel between two frames and stay the same finger */
#define NVT_TRACK_DIST			400

#define NVT_PRESSURE_MAX		4095

/* Enough for the largest transfer this driver issues */
#define NVT_XFER_LEN			128
/*
 * Payload of a single SRAM write. The controller keeps incrementing its own
 * address pointer while chip select stays low, so this can be anything; the
 * vendor driver uses 63 KiB, which not every SPI controller can do in one go.
 */
#define NVT_FW_XFER_LEN			4096

/* Firmware images end on a 4 KiB boundary marked by one of these tags */
#define NVT_FW_SECTOR_SIZE		4096
#define NVT_FW_END_TAG_LEN		3
/* The version byte is followed by its own complement, one sector from the end */
#define NVT_FW_VER_OFFSET		NVT_FW_SECTOR_SIZE

/*
 * Firmware header layout. ILM and DLM come first in 12 byte entries with their
 * CRCs gathered separately, everything else follows in 16 byte entries.
 */
#define NVT_FW_HDR_ILM_DLM_STRIDE	12
#define NVT_FW_HDR_ILM_DLM_COUNT	2
#define NVT_FW_HDR_CRC_OFFSET		0x18
#define NVT_FW_HDR_FLAGS_OFFSET		0x20
#define NVT_FW_HDR_FLAGS_CASCADE	BIT(1)
#define NVT_FW_HDR_OVERLAY_OFFSET	0x28
#define NVT_FW_HDR_OVERLAY_VALID	BIT(4)
#define NVT_FW_HDR_OVERLAY_COUNT	GENMASK(3, 0)
#define NVT_FW_HDR_SECTIONS_OFFSET	0x30
#define NVT_FW_HDR_SECTION_STRIDE	0x10
#define NVT_FW_MAX_PARTITIONS		64

struct nvt_fw_partition {
	u32 sram_addr;
	u32 bin_addr;
	u32 size;
	u32 crc;
};

struct nvt_fw_image {
	struct nvt_fw_partition part[NVT_FW_MAX_PARTITIONS];
	unsigned int count;
	bool cascade;
};

struct nvt_contact {
	bool active;
	u32 x;
	u32 y;
};

struct nvt_blob {
	u32 weight;
	u32 wcol;
	u32 wrow;
	u32 cells;
	s32 peak;
	u32 x;
	u32 y;
};

struct nvt_ts {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct drm_panel_follower panel_follower;
	const char **fw_names;
	unsigned int fw_count;
	/* Picked from the cascade strap once the controller has been probed */
	u32 event_buf;
	bool cascade;
	bool running;
	/* Sensor grid, reported by the firmware once it is up */
	unsigned int cols;
	unsigned int rows;
	/* Serialises the shared transfer buffers below */
	struct mutex lock;
	u8 *tx_buf;
	u8 *rx_buf;
	u8 *frame_tx;
	u8 *frame_rx;
	/* Touch detection state, only used from the interrupt thread */
	bool baseline_valid;
	s16 *baseline;
	s16 *diff;
	u16 *stack;
	unsigned long *covered;
	struct nvt_contact contacts[NVT_MAX_FINGERS];
};

static const char * const nvt_supply_names[] = {
	"vdd",		/* AVDD, analog supply */
	"vddio",	/* digital and interface supply */
};

/*
 * Caller holds ts->lock and provides a DMA capable buffer whose first byte is
 * the register offset. Used for the firmware download, which moves far more
 * data than the shared transfer buffers can hold.
 */
static int nvt_spi_write_raw(struct nvt_ts *ts, u8 *buf, size_t len)
{
	struct spi_transfer xfer = { };

	buf[0] = NVT_SPI_WRITE_MASK(buf[0]);

	xfer.tx_buf = buf;
	xfer.len = len;

	return spi_sync_transfer(ts->spi, &xfer, 1);
}

/* Caller holds ts->lock */
static int nvt_spi_write(struct nvt_ts *ts, u8 offset, const u8 *data,
			 size_t count)
{
	size_t len = count + 1;

	if (len > NVT_XFER_LEN)
		return -EINVAL;

	ts->tx_buf[0] = offset;
	memcpy(&ts->tx_buf[1], data, count);

	return nvt_spi_write_raw(ts, ts->tx_buf, len);
}

/* Caller holds ts->lock */
static int nvt_spi_read(struct nvt_ts *ts, u8 offset, u8 *data, size_t count)
{
	struct spi_transfer xfer = { };
	size_t len = count + 1 + NVT_DUMMY_BYTES;
	int ret;

	if (len > NVT_XFER_LEN)
		return -EINVAL;

	memset(ts->tx_buf, 0, len);
	ts->tx_buf[0] = NVT_SPI_READ_MASK(offset);

	xfer.tx_buf = ts->tx_buf;
	xfer.rx_buf = ts->rx_buf;
	xfer.len = len;

	ret = spi_sync_transfer(ts->spi, &xfer, 1);
	if (ret)
		return ret;

	/* Skip the offset echo and the dummy byte */
	memcpy(data, &ts->rx_buf[1 + NVT_DUMMY_BYTES], count);

	return 0;
}

/*
 * Registers are reached by first pointing the controller at a 128 byte window
 * and then addressing the offset within it.
 */
static int nvt_set_page(struct nvt_ts *ts, u32 addr)
{
	u8 page[2] = { (addr >> 15) & 0xff, (addr >> 7) & 0xff };

	return nvt_spi_write(ts, NVT_PAGE_SET_CMD, page, sizeof(page));
}

static int nvt_write_addr(struct nvt_ts *ts, u32 addr, u8 val)
{
	int ret;

	ret = nvt_set_page(ts, addr);
	if (ret)
		return ret;

	return nvt_spi_write(ts, addr & GENMASK(6, 0), &val, 1);
}

static int nvt_read_addr(struct nvt_ts *ts, u32 addr, u8 *data, size_t count)
{
	int ret;

	ret = nvt_set_page(ts, addr);
	if (ret)
		return ret;

	return nvt_spi_read(ts, addr & GENMASK(6, 0), data, count);
}

static int nvt_bootloader_reset(struct nvt_ts *ts)
{
	int ret;

	ret = nvt_write_addr(ts, NVT_SWRST_SIF_ADDR, NVT_CMD_BOOTLOADER_RESET);
	if (ret)
		return ret;

	/* Wait tBRST2FR before touching the controller again */
	usleep_range(5000, 6000);

	return 0;
}

/*
 * The two dies of a cascaded part share one event buffer, but it lives at a
 * different address than on a single die part. Everything else the download
 * touches is at the same place either way.
 */
static int nvt_detect_cascade(struct nvt_ts *ts)
{
	u8 val;
	int ret;

	ret = nvt_read_addr(ts, NVT_ENB_CASC_ADDR, &val, 1);
	if (ret)
		return ret;

	/* Note the inverted sense: the bit is set on a single die part */
	ts->cascade = !(val & NVT_ENB_CASC_SINGLE);
	if (ts->cascade)
		ts->event_buf = NVT_EVENT_BUF_ADDR_CASCADE;
	else
		ts->event_buf = NVT_EVENT_BUF_ADDR_SINGLE;

	dev_dbg(&ts->spi->dev, "%s die, event buffer at %#x\n",
		ts->cascade ? "cascaded" : "single", ts->event_buf);

	return 0;
}

/*
 * Images are padded to a 4 KiB boundary that is tagged with "NVT" or "MOD",
 * and carry a version byte followed by its complement one sector earlier.
 */
static int nvt_fw_check_tail(struct nvt_ts *ts, const struct firmware *fw)
{
	size_t i, len;

	for (i = fw->size / NVT_FW_SECTOR_SIZE; i > 0; i--) {
		const u8 *tag = &fw->data[i * NVT_FW_SECTOR_SIZE -
					  NVT_FW_END_TAG_LEN];

		if (memcmp(tag, "NVT", NVT_FW_END_TAG_LEN) &&
		    memcmp(tag, "MOD", NVT_FW_END_TAG_LEN))
			continue;

		len = i * NVT_FW_SECTOR_SIZE;
		if (len < NVT_FW_VER_OFFSET + 2)
			break;

		if ((fw->data[len - NVT_FW_VER_OFFSET] +
		     fw->data[len - NVT_FW_VER_OFFSET + 1]) != 0xff)
			break;

		dev_dbg(&ts->spi->dev, "firmware version %u\n",
			fw->data[len - NVT_FW_VER_OFFSET]);

		return 0;
	}

	dev_err(&ts->spi->dev, "not a valid NT36532 firmware image\n");

	return -EINVAL;
}

static int nvt_fw_add_partition(struct nvt_ts *ts, struct nvt_fw_image *img,
				const struct firmware *fw, u32 sram_addr,
				u32 size, u32 bin_addr, u32 crc)
{
	struct nvt_fw_partition *part;

	if (img->count >= NVT_FW_MAX_PARTITIONS) {
		dev_err(&ts->spi->dev, "too many firmware partitions\n");
		return -EINVAL;
	}

	if (size > fw->size || bin_addr > fw->size - size) {
		dev_err(&ts->spi->dev,
			"partition %u spans %#x..%#x, past the %zu byte image\n",
			img->count, bin_addr, bin_addr + size, fw->size);
		return -EINVAL;
	}

	part = &img->part[img->count++];
	part->sram_addr = sram_addr;
	part->size = size;
	part->bin_addr = bin_addr;
	part->crc = crc;

	return 0;
}

static int nvt_fw_parse(struct nvt_ts *ts, const struct firmware *fw,
			struct nvt_fw_image *img)
{
	unsigned int sections, overlays, i;
	u32 end, pos;
	int ret;

	memset(img, 0, sizeof(*img));

	if (fw->size < NVT_FW_HDR_SECTIONS_OFFSET + NVT_FW_HDR_SECTION_STRIDE)
		return -EINVAL;

	ret = nvt_fw_check_tail(ts, fw);
	if (ret)
		return ret;

	/* ILM starts right behind the header, so its offset is the header size */
	end = get_unaligned_le32(fw->data);
	if (end > fw->size)
		return -EINVAL;

	/* The header describes itself as the first entry with a zero bin address */
	for (pos = NVT_FW_HDR_SECTIONS_OFFSET; pos + NVT_FW_HDR_SECTION_STRIDE <= end;
	     pos += NVT_FW_HDR_SECTION_STRIDE) {
		if (get_unaligned_le32(&fw->data[pos + 4]) &&
		    !get_unaligned_le32(&fw->data[pos + 8])) {
			end = pos + NVT_FW_HDR_SECTION_STRIDE;
			break;
		}
	}

	sections = (end - NVT_FW_HDR_SECTIONS_OFFSET) / NVT_FW_HDR_SECTION_STRIDE;

	/*
	 * A cascaded image carries a second copy of the header for the second
	 * die, described by an entry just past the doubled header.
	 */
	img->cascade = fw->data[NVT_FW_HDR_FLAGS_OFFSET] & NVT_FW_HDR_FLAGS_CASCADE;
	if (img->cascade) {
		sections++;
		end *= 2;
		if (end > fw->size)
			return -EINVAL;
	}

	overlays = fw->data[NVT_FW_HDR_OVERLAY_OFFSET] & NVT_FW_HDR_OVERLAY_VALID ?
		   fw->data[NVT_FW_HDR_OVERLAY_OFFSET] & NVT_FW_HDR_OVERLAY_COUNT : 0;

	/* ILM and DLM come first, in narrower entries with the CRCs split off */
	for (i = 0; i < NVT_FW_HDR_ILM_DLM_COUNT; i++) {
		const u8 *e = &fw->data[i * NVT_FW_HDR_ILM_DLM_STRIDE];

		ret = nvt_fw_add_partition(ts, img, fw,
					   get_unaligned_le32(&e[4]),
					   get_unaligned_le32(&e[8]),
					   get_unaligned_le32(&e[0]),
					   get_unaligned_le32(&fw->data[NVT_FW_HDR_CRC_OFFSET + i * 4]));
		if (ret)
			return ret;
	}

	for (i = 0; i < sections; i++) {
		const u8 *e;

		pos = NVT_FW_HDR_SECTIONS_OFFSET + i * NVT_FW_HDR_SECTION_STRIDE;
		/* The trailing entry describes the second die's header copy */
		if (img->cascade && i == sections - 1)
			pos = end - NVT_FW_HDR_SECTION_STRIDE;

		if (pos + NVT_FW_HDR_SECTION_STRIDE > fw->size)
			return -EINVAL;

		e = &fw->data[pos];
		ret = nvt_fw_add_partition(ts, img, fw,
					   get_unaligned_le32(&e[0]),
					   get_unaligned_le32(&e[4]),
					   get_unaligned_le32(&e[8]),
					   get_unaligned_le32(&e[12]));
		if (ret)
			return ret;
	}

	if (overlays)
		dev_warn(&ts->spi->dev,
			 "ignoring %u overlay sections, none were expected\n",
			 overlays);

	return 0;
}

/*
 * Point the bootloader's CRC engine at one image bank. The destination, the
 * length and the golden checksum all live in the same 128 byte window, so a
 * single page set covers all three writes.
 */
static int nvt_fw_setup_crc_bank(struct nvt_ts *ts,
				 const struct nvt_fw_partition *part,
				 u32 des_addr, u32 length_addr, u32 crc_addr)
{
	u8 buf[4];
	int ret;

	ret = nvt_set_page(ts, des_addr);
	if (ret)
		return ret;

	put_unaligned_le32(part->sram_addr, buf);
	ret = nvt_spi_write(ts, des_addr & GENMASK(6, 0), buf, 3);
	if (ret)
		return ret;

	put_unaligned_le32(part->size, buf);
	ret = nvt_spi_write(ts, length_addr & GENMASK(6, 0), buf, 3);
	if (ret)
		return ret;

	put_unaligned_le32(part->crc, buf);

	return nvt_spi_write(ts, crc_addr & GENMASK(6, 0), buf, 4);
}

static int nvt_fw_write_sram(struct nvt_ts *ts, u8 *buf, const u8 *data,
			     u32 sram_addr, u32 size)
{
	int ret;

	while (size) {
		u32 chunk = min_t(u32, size, NVT_FW_XFER_LEN);

		ret = nvt_set_page(ts, sram_addr);
		if (ret)
			return ret;

		buf[0] = sram_addr & GENMASK(6, 0);
		memcpy(&buf[1], data, chunk);

		ret = nvt_spi_write_raw(ts, buf, chunk + 1);
		if (ret)
			return ret;

		sram_addr += chunk;
		data += chunk;
		size -= chunk;
	}

	return 0;
}

static int nvt_fw_wait_auto_copy(struct nvt_ts *ts)
{
	u8 val;
	int i, ret;

	for (i = 0; i < 200; i++) {
		ret = nvt_read_addr(ts, NVT_TX_AUTO_COPY_ADDR, &val, 1);
		if (ret)
			return ret;

		if (!val)
			return 0;

		usleep_range(1000, 1100);
	}

	dev_err(&ts->spi->dev, "the second die never drained (%#02x)\n", val);

	return -ETIMEDOUT;
}

/* Clear the reset progress byte and ask the firmware to verify itself */
static int nvt_fw_crc_enable(struct nvt_ts *ts)
{
	static const u8 clear[6] = { };
	static const u8 cmd[2] = { NVT_HOST_CMD_CRC_ENABLE, 0x00 };
	int ret;

	ret = nvt_set_page(ts, ts->event_buf);
	if (ret)
		return ret;

	ret = nvt_spi_write(ts, NVT_EVENT_MAP_RESET_COMPLETE, clear,
			    sizeof(clear));
	if (ret)
		return ret;

	return nvt_spi_write(ts, NVT_EVENT_MAP_HOST_CMD, cmd, sizeof(cmd));
}

static void nvt_fw_report_crc(struct nvt_ts *ts)
{
	u8 flags, golden[4], result[4];

	if (nvt_read_addr(ts, NVT_BLD_CRC_FLAG_ADDR, &flags, 1))
		return;

	dev_err(&ts->spi->dev, "bootloader CRC %#02x: done %u, ILM %u, DLM %u\n",
		flags, !!(flags & NVT_BLD_CRC_DONE),
		!!(flags & NVT_BLD_CRC_ILM_OK), !!(flags & NVT_BLD_CRC_DLM_OK));

	if (!nvt_read_addr(ts, NVT_G_ILM_CHECKSUM_ADDR, golden, 4) &&
	    !nvt_read_addr(ts, NVT_R_ILM_CHECKSUM_ADDR, result, 4))
		dev_err(&ts->spi->dev, "ILM checksum golden %*phN result %*phN\n",
			4, golden, 4, result);

	if (!nvt_read_addr(ts, NVT_G_DLM_CHECKSUM_ADDR, golden, 4) &&
	    !nvt_read_addr(ts, NVT_R_DLM_CHECKSUM_ADDR, result, 4))
		dev_err(&ts->spi->dev, "DLM checksum golden %*phN result %*phN\n",
			4, golden, 4, result);
}

static int nvt_wait_fw_state(struct nvt_ts *ts, u8 min_state, int tries);

/*
 * The controller has no non volatile storage for its firmware, so the host has
 * to push it into SRAM again every time the shared reset line has been pulsed.
 */
static int nvt_fw_download(struct nvt_ts *ts, const struct firmware *fw,
			   const struct nvt_fw_image *img, u8 *buf)
{
	unsigned int i;
	int ret;

	ret = nvt_bootloader_reset(ts);
	if (ret)
		return ret;

	ret = nvt_fw_setup_crc_bank(ts, &img->part[0], NVT_ILM_DES_ADDR,
				    NVT_ILM_LENGTH_ADDR, NVT_G_ILM_CHECKSUM_ADDR);
	if (ret)
		return ret;

	ret = nvt_fw_setup_crc_bank(ts, &img->part[1], NVT_DLM_DES_ADDR,
				    NVT_DLM_LENGTH_ADDR, NVT_G_DLM_CHECKSUM_ADDR);
	if (ret)
		return ret;

	if (img->cascade) {
		ret = nvt_write_addr(ts, NVT_TX_AUTO_COPY_ADDR,
				     NVT_TX_AUTO_COPY_START);
		if (ret)
			return ret;
	}

	for (i = 0; i < img->count; i++) {
		const struct nvt_fw_partition *part = &img->part[i];

		/* Reserved partitions are described with a zero length */
		if (!part->size)
			continue;

		/*
		 * The recorded size is the offset of the last byte, so one more
		 * byte than that has to be written.
		 */
		ret = nvt_fw_write_sram(ts, buf, &fw->data[part->bin_addr],
					part->sram_addr, part->size + 1);
		if (ret)
			return ret;
	}

	if (img->cascade) {
		ret = nvt_fw_wait_auto_copy(ts);
		if (ret)
			return ret;
	}

	ret = nvt_fw_crc_enable(ts);
	if (ret)
		return ret;

	/* Let the MCU out of reset, then give it a moment to check itself */
	ret = nvt_write_addr(ts, NVT_BOOT_RDY_ADDR, 1);
	if (ret)
		return ret;

	usleep_range(5000, 6000);

	return nvt_wait_fw_state(ts, NVT_RESET_STATE_INIT, 11);
}

/*
 * Two images means one per panel source. The controller cannot tell them
 * apart on its own: the project ID its firmware reports is a constant baked
 * into the image, not something read back from the hardware. So the same
 * source strap the panel driver uses has to make the call.
 */
static int nvt_fw_strap_index(struct nvt_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	struct gpio_desc *strap;
	int val;

	/* The panel driver reads the same strap, so only borrow it */
	strap = gpiod_get(dev, "lcd-id", GPIOD_IN);
	if (IS_ERR(strap)) {
		dev_err(dev, "two firmware images listed but no lcd-id strap: %pe\n",
			strap);
		return PTR_ERR(strap);
	}

	val = gpiod_get_value_cansleep(strap);
	gpiod_put(strap);

	if (val < 0)
		return val;

	dev_dbg(dev, "panel source strap reads %d\n", val);

	return val ? 1 : 0;
}

/* Caller holds ts->lock */
static int nvt_fw_load(struct nvt_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	const struct firmware *fw;
	struct nvt_fw_image *img;
	const char *name;
	unsigned int i;
	int ret;
	u8 *buf;

	/* Without an image listed, assume the firmware is already resident */
	if (!ts->fw_count)
		return 0;

	if (ts->fw_count == 1) {
		name = ts->fw_names[0];
	} else {
		ret = nvt_fw_strap_index(ts);
		if (ret < 0)
			return ret;

		name = ts->fw_names[ret];
	}

	ret = request_firmware(&fw, name, dev);
	if (ret) {
		dev_err(dev, "failed to request %s: %d\n", name, ret);
		return ret;
	}

	img = kmalloc(sizeof(*img), GFP_KERNEL);
	buf = kmalloc(NVT_FW_XFER_LEN + 1, GFP_KERNEL);
	if (!img || !buf) {
		ret = -ENOMEM;
		goto out;
	}

	ret = nvt_fw_parse(ts, fw, img);
	if (ret)
		goto out;

	if (img->cascade != ts->cascade) {
		dev_err(dev, "%s is built for a %s die\n", name,
			img->cascade ? "cascaded" : "single");
		ret = -ENODEV;
		goto out;
	}

	for (i = 0; i < 3; i++) {
		ret = nvt_fw_download(ts, fw, img, buf);
		if (!ret) {
			dev_dbg(dev, "loaded %s\n", name);
			goto out;
		}

		dev_dbg(dev, "%s download attempt %u failed: %d\n",
			name, i + 1, ret);
	}

	dev_err(dev, "failed to download %s\n", name);
	nvt_fw_report_crc(ts);

out:
	kfree(buf);
	kfree(img);
	release_firmware(fw);

	return ret;
}

static int nvt_wait_fw_state(struct nvt_ts *ts, u8 min_state, int tries)
{
	u8 state[5] = { };
	int i, ret;

	for (i = 0; i < tries; i++) {
		ret = nvt_read_addr(ts, ts->event_buf | NVT_EVENT_MAP_RESET_COMPLETE,
				    state, sizeof(state));
		if (ret)
			return ret;

		if (state[0] >= min_state && state[0] <= NVT_RESET_STATE_MAX)
			return 0;

		usleep_range(10000, 11000);
	}

	dev_dbg(&ts->spi->dev, "firmware did not reach state %#02x (%#02x)\n",
		min_state, state[0]);

	return -ETIMEDOUT;
}

static int nvt_check_chip_id(struct nvt_ts *ts)
{
	/* Only the last three bytes of the trim ID identify the controller */
	static const u8 nt36532_id[] = { 0x32, 0x65, 0x03 };
	u8 id[6];
	int i, ret;

	for (i = 0; i < 5; i++) {
		ret = nvt_bootloader_reset(ts);
		if (ret)
			return ret;

		ret = nvt_set_page(ts, NVT_CHIP_VER_TRIM_ADDR);
		if (ret)
			return ret;

		/*
		 * The controller only latches the trim values once the address
		 * pointer has been walked over them, so prime it with a dummy
		 * write of the same length as the read that follows.
		 */
		memset(id, 0, sizeof(id));
		ret = nvt_spi_write(ts, NVT_CHIP_VER_TRIM_ADDR & GENMASK(6, 0),
				    id, sizeof(id));
		if (ret)
			return ret;

		ret = nvt_spi_read(ts, NVT_CHIP_VER_TRIM_ADDR & GENMASK(6, 0),
				   id, sizeof(id));
		if (ret)
			return ret;

		if (!memcmp(&id[3], nt36532_id, sizeof(nt36532_id)))
			return 0;

		dev_dbg(&ts->spi->dev, "trim id %*ph\n", (int)sizeof(id), id);
		msleep(10);
	}

	dev_err(&ts->spi->dev, "not an NT36532, trim id %*ph\n",
		(int)sizeof(id), id);

	return -ENODEV;
}

static int nvt_get_fw_info(struct nvt_ts *ts)
{
	u8 buf[38];
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = nvt_read_addr(ts, ts->event_buf | NVT_EVENT_MAP_FWINFO,
				    buf, sizeof(buf));
		if (ret)
			return ret;

		/* The firmware version is followed by its own complement */
		if ((buf[0] + buf[1]) == 0xff) {
			dev_dbg(&ts->spi->dev,
				"fw version %u, %ux%u sensors, fw type %u, pid %#06x\n",
				buf[0], buf[2], buf[3], buf[13],
				get_unaligned_le16(&buf[34]));
			dev_dbg(&ts->spi->dev, "fw info %*phN\n",
				(int)sizeof(buf), buf);

			if (!buf[2] || buf[2] > NVT_MAX_COLS ||
			    !buf[3] || buf[3] > NVT_MAX_ROWS) {
				dev_err(&ts->spi->dev,
					"unsupported sensor grid %ux%u\n",
					buf[2], buf[3]);
				return -EINVAL;
			}

			ts->cols = buf[2];
			ts->rows = buf[3];

			return 0;
		}
	}

	dev_err(&ts->spi->dev, "firmware info is broken (0x%02x 0x%02x)\n",
		buf[0], buf[1]);

	return -EIO;
}

/*
 * The frame sits right behind the event buffer and is far larger than anything
 * else this driver reads, so it gets its own pair of buffers. The controller
 * keeps incrementing its address pointer for as long as chip select stays low,
 * which is what lets a single transfer walk out of the 128 byte window the
 * paging scheme otherwise limits an access to.
 *
 * Caller holds ts->lock
 */
static int nvt_read_raw_frame(struct nvt_ts *ts, size_t count)
{
	struct spi_transfer xfer = { };
	u32 addr = ts->event_buf + NVT_EVENT_BUF_LEN;
	size_t len = count + 1 + NVT_DUMMY_BYTES;
	int ret;

	if (len > NVT_FRAME_BUF_LEN)
		return -EINVAL;

	ret = nvt_set_page(ts, addr);
	if (ret)
		return ret;

	memset(ts->frame_tx, 0, len);
	ts->frame_tx[0] = NVT_SPI_READ_MASK(addr);

	xfer.tx_buf = ts->frame_tx;
	xfer.rx_buf = ts->frame_rx;
	xfer.len = len;

	return spi_sync_transfer(ts->spi, &xfer, 1);
}

/*
 * The header repeats the length of the region the firmware ran its checksum
 * over, once plain and once complemented. That is not a checksum of the samples
 * themselves, but it is enough to notice a frame that was read while the
 * firmware was still filling it in.
 */
static bool nvt_frame_valid(struct nvt_ts *ts, const u8 *frame)
{
	u32 len = get_unaligned_le32(&frame[NVT_FRAME_CRC_LEN]);
	u32 bar = get_unaligned_le32(&frame[NVT_FRAME_CRC_LEN_BAR]);

	if (len != ~bar) {
		dev_dbg(&ts->spi->dev, "torn frame header\n");
		return false;
	}

	if (frame[NVT_FRAME_DATA_TYPE] != NVT_FRAME_TYPE_FINGER) {
		dev_dbg_ratelimited(&ts->spi->dev, "frame data type %u\n",
				    frame[NVT_FRAME_DATA_TYPE]);
		return false;
	}

	if (frame[NVT_FRAME_NUM_COL] != ts->cols ||
	    frame[NVT_FRAME_NUM_ROW] != ts->rows) {
		dev_dbg(&ts->spi->dev, "frame is %ux%u, expected %ux%u\n",
			frame[NVT_FRAME_NUM_COL], frame[NVT_FRAME_NUM_ROW],
			ts->cols, ts->rows);
		return false;
	}

	return true;
}

/*
 * Nodes nobody is touching drift with temperature, so they are nudged one count
 * per frame towards what was just measured. Anything that moved far enough to
 * be a finger, and everything a contact was found on, is left alone.
 */
static void nvt_track_baseline(struct nvt_ts *ts, unsigned int cells)
{
	unsigned int i;

	for (i = 0; i < cells; i++) {
		if (test_bit(i, ts->covered) || abs(ts->diff[i]) >= NVT_BASELINE_LIMIT)
			continue;

		if (ts->diff[i] > 0)
			ts->baseline[i]++;
		else if (ts->diff[i] < 0)
			ts->baseline[i]--;
	}
}

/*
 * A contact is a group of neighbouring nodes that all rose above the lower
 * threshold and that contains at least one node above the upper one. Nodes are
 * collected with an explicit stack because the frame can be larger than what a
 * recursive walk should put on the interrupt thread's stack.
 */
static unsigned int nvt_find_contacts(struct nvt_ts *ts, struct nvt_blob *blobs)
{
	static const int dcol[] = { -1, 1, 0, 0 };
	static const int drow[] = { 0, 0, -1, 1 };
	unsigned int cells = ts->cols * ts->rows;
	unsigned int found = 0;
	unsigned int i;

	bitmap_zero(ts->covered, NVT_MAX_CELLS);

	for (i = 0; i < cells && found < NVT_MAX_FINGERS; i++) {
		struct nvt_blob *blob = &blobs[found];
		unsigned int top = 0;

		if (ts->diff[i] < NVT_TOUCH_ON || test_bit(i, ts->covered))
			continue;

		memset(blob, 0, sizeof(*blob));
		__set_bit(i, ts->covered);
		ts->stack[top++] = i;

		while (top) {
			unsigned int cell = ts->stack[--top];
			unsigned int col = cell % ts->cols;
			unsigned int row = cell / ts->cols;
			u32 weight = ts->diff[cell] - NVT_TOUCH_OFF;
			unsigned int k;

			blob->weight += weight;
			blob->wcol += weight * col;
			blob->wrow += weight * row;
			blob->cells++;
			blob->peak = max_t(s32, blob->peak, ts->diff[cell]);

			for (k = 0; k < ARRAY_SIZE(dcol); k++) {
				int ncol = col + dcol[k];
				int nrow = row + drow[k];
				unsigned int next;

				if (ncol < 0 || ncol >= (int)ts->cols ||
				    nrow < 0 || nrow >= (int)ts->rows)
					continue;

				next = nrow * ts->cols + ncol;
				if (test_bit(next, ts->covered) ||
				    ts->diff[next] < NVT_TOUCH_OFF)
					continue;

				__set_bit(next, ts->covered);
				ts->stack[top++] = next;
			}
		}

		/*
		 * A single node above the upper threshold with nothing around
		 * it is noise rather than a finger.
		 */
		if (blob->weight && blob->cells > 1)
			found++;
	}

	return found;
}

/*
 * Slots have to survive across frames or userspace sees every finger lift and
 * land again on each scan, so each contact is handed the slot of the nearest
 * contact of the previous frame that is close enough to be the same finger.
 */
static void nvt_assign_slots(struct nvt_ts *ts, const struct nvt_blob *blobs,
			     unsigned int count, int *slots)
{
	DECLARE_BITMAP(taken, NVT_MAX_FINGERS);
	unsigned int i, slot;

	bitmap_zero(taken, NVT_MAX_FINGERS);

	for (i = 0; i < count; i++) {
		u64 best = (u64)NVT_TRACK_DIST * NVT_TRACK_DIST;
		int match = -1;

		slots[i] = -1;

		for (slot = 0; slot < NVT_MAX_FINGERS; slot++) {
			s64 dx, dy, dist;

			if (!ts->contacts[slot].active || test_bit(slot, taken))
				continue;

			dx = (s64)blobs[i].x - ts->contacts[slot].x;
			dy = (s64)blobs[i].y - ts->contacts[slot].y;
			dist = dx * dx + dy * dy;
			if (dist > best)
				continue;

			best = dist;
			match = slot;
		}

		if (match < 0)
			continue;

		slots[i] = match;
		__set_bit(match, taken);
	}

	for (i = 0; i < count; i++) {
		if (slots[i] >= 0)
			continue;

		slot = find_first_zero_bit(taken, NVT_MAX_FINGERS);
		if (slot >= NVT_MAX_FINGERS)
			continue;

		slots[i] = slot;
		__set_bit(slot, taken);
	}
}

static void nvt_report_frame(struct nvt_ts *ts, const u8 *frame)
{
	struct nvt_blob blobs[NVT_MAX_FINGERS];
	int slots[NVT_MAX_FINGERS];
	unsigned int cells = ts->cols * ts->rows;
	unsigned int pitch = (ts->prop.max_x + 1) / ts->cols;
	unsigned int count, i;
	bool live[NVT_MAX_FINGERS] = { };

	for (i = 0; i < cells; i++) {
		s16 raw = get_unaligned_le16(&frame[NVT_FRAME_HDR_LEN + i * 2]);

		if (!ts->baseline_valid)
			ts->baseline[i] = raw;

		ts->diff[i] = raw - ts->baseline[i];
	}

	/*
	 * The first frame only establishes what an untouched panel looks like.
	 * A finger already on the glass ends up in the baseline and is missed
	 * until it is lifted, which is the same behaviour every other self
	 * calibrating controller has.
	 */
	if (!ts->baseline_valid) {
		ts->baseline_valid = true;
		return;
	}

	count = nvt_find_contacts(ts, blobs);

	for (i = 0; i < count; i++) {
		struct nvt_blob *blob = &blobs[i];

		/*
		 * The nodes are laid out along the panel, so a column picks
		 * out X and a row picks out Y. Weighting by how far each node
		 * moved puts the contact between them.
		 *
		 * Each node senses the middle of the strip it covers rather
		 * than its edge, so the centroid is offset by half a node
		 * before it is scaled. Stretching the outermost nodes out to
		 * the edges of the panel instead would buy a little reach at
		 * the borders at the cost of a percent or two of error
		 * everywhere else.
		 */
		blob->x = div_u64((u64)(2 * blob->wcol + blob->weight) *
				  (ts->prop.max_x + 1),
				  (u64)blob->weight * 2 * ts->cols);
		blob->y = div_u64((u64)(2 * blob->wrow + blob->weight) *
				  (ts->prop.max_y + 1),
				  (u64)blob->weight * 2 * ts->rows);

		blob->x = min(blob->x, ts->prop.max_x);
		blob->y = min(blob->y, ts->prop.max_y);
	}

	nvt_assign_slots(ts, blobs, count, slots);

	for (i = 0; i < count; i++) {
		struct nvt_blob *blob = &blobs[i];

		if (slots[i] < 0)
			continue;

		input_mt_slot(ts->input, slots[i]);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, blob->x, blob->y,
				       true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
				 int_sqrt(blob->cells) * pitch);
		input_report_abs(ts->input, ABS_MT_PRESSURE,
				 min(blob->peak, NVT_PRESSURE_MAX));

		ts->contacts[slots[i]].x = blob->x;
		ts->contacts[slots[i]].y = blob->y;
		live[slots[i]] = true;
	}

	for (i = 0; i < NVT_MAX_FINGERS; i++) {
		if (!live[i] && ts->contacts[i].active) {
			input_mt_slot(ts->input, i);
			input_mt_report_slot_inactive(ts->input);
		}

		ts->contacts[i].active = live[i];
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	nvt_track_baseline(ts, cells);
}

static irqreturn_t nvt_ts_irq(int irq, void *dev_id)
{
	struct nvt_ts *ts = dev_id;
	size_t len = NVT_FRAME_HDR_LEN + ts->cols * ts->rows * 2;
	const u8 *frame = &ts->frame_rx[1 + NVT_DUMMY_BYTES];
	int ret;

	mutex_lock(&ts->lock);
	ret = nvt_read_raw_frame(ts, len);
	mutex_unlock(&ts->lock);

	if (ret) {
		dev_err_ratelimited(&ts->spi->dev,
				    "failed to read the frame: %d\n", ret);
		return IRQ_HANDLED;
	}

	if (!nvt_frame_valid(ts, frame))
		return IRQ_HANDLED;

	nvt_report_frame(ts, frame);

	return IRQ_HANDLED;
}

/*
 * The touch and display halves share one reset line, which the panel driver
 * owns. Every display power cycle therefore drops the touch firmware back into
 * its bootloader, so the controller has to be brought up again afterwards.
 *
 * The die is also held in reset until the panel is first prepared, which is why
 * even the initial probing of the controller happens from here rather than from
 * probe().
 */
static int nvt_ts_start(struct nvt_ts *ts)
{
	int ret;

	if (ts->running)
		return 0;

	mutex_lock(&ts->lock);

	ret = nvt_check_chip_id(ts);
	if (ret)
		goto out;

	ret = nvt_detect_cascade(ts);
	if (ret)
		goto out;

	ret = nvt_fw_load(ts);
	if (ret)
		goto out;

	/* The firmware was just downloaded again, so re-read what it reports */
	ret = nvt_get_fw_info(ts);
	if (ret)
		goto out;

out:
	mutex_unlock(&ts->lock);
	if (ret)
		return ret;

	/* The panel was just powered, so whatever was measured before is stale */
	ts->baseline_valid = false;
	memset(ts->contacts, 0, sizeof(ts->contacts));

	ts->running = true;
	enable_irq(ts->spi->irq);

	return 0;
}

static void nvt_ts_stop(struct nvt_ts *ts)
{
	int ret;

	if (!ts->running)
		return;

	disable_irq(ts->spi->irq);
	ts->running = false;

	mutex_lock(&ts->lock);
	ret = nvt_write_addr(ts, ts->event_buf | NVT_EVENT_MAP_HOST_CMD,
			     NVT_HOST_CMD_DEEP_SLEEP);
	mutex_unlock(&ts->lock);
	if (ret)
		dev_err(&ts->spi->dev, "failed to enter deep sleep: %d\n", ret);

	/* Drop any contact that was still down */
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static int nvt_ts_panel_prepared(struct drm_panel_follower *follower)
{
	struct nvt_ts *ts = container_of(follower, struct nvt_ts, panel_follower);

	return nvt_ts_start(ts);
}

static int nvt_ts_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct nvt_ts *ts = container_of(follower, struct nvt_ts, panel_follower);

	nvt_ts_stop(ts);

	return 0;
}

static const struct drm_panel_follower_funcs nvt_ts_panel_follower_funcs = {
	.panel_prepared = nvt_ts_panel_prepared,
	.panel_unpreparing = nvt_ts_panel_unpreparing,
};

static int nvt_ts_suspend(struct device *dev)
{
	struct nvt_ts *ts = dev_get_drvdata(dev);

	/* When following a panel, the panel callbacks already did this */
	if (drm_is_panel_follower(dev))
		return 0;

	nvt_ts_stop(ts);

	return 0;
}

static int nvt_ts_resume(struct device *dev)
{
	struct nvt_ts *ts = dev_get_drvdata(dev);

	if (drm_is_panel_follower(dev))
		return 0;

	return nvt_ts_start(ts);
}

static DEFINE_SIMPLE_DEV_PM_OPS(nvt_ts_pm_ops, nvt_ts_suspend, nvt_ts_resume);

static int nvt_ts_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct nvt_ts *ts;
	int ret;

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt assigned\n");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up SPI\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	/*
	 * Keep the SPI buffers in their own DMA safe allocations rather than
	 * inside the driver data.
	 */
	ts->tx_buf = devm_kzalloc(dev, NVT_XFER_LEN, GFP_KERNEL);
	ts->rx_buf = devm_kzalloc(dev, NVT_XFER_LEN, GFP_KERNEL);
	ts->frame_tx = devm_kzalloc(dev, NVT_FRAME_BUF_LEN, GFP_KERNEL);
	ts->frame_rx = devm_kzalloc(dev, NVT_FRAME_BUF_LEN, GFP_KERNEL);
	if (!ts->tx_buf || !ts->rx_buf || !ts->frame_tx || !ts->frame_rx)
		return -ENOMEM;

	ts->baseline = devm_kcalloc(dev, NVT_MAX_CELLS, sizeof(*ts->baseline),
				    GFP_KERNEL);
	ts->diff = devm_kcalloc(dev, NVT_MAX_CELLS, sizeof(*ts->diff),
				GFP_KERNEL);
	ts->stack = devm_kcalloc(dev, NVT_MAX_CELLS, sizeof(*ts->stack),
				 GFP_KERNEL);
	ts->covered = devm_bitmap_zalloc(dev, NVT_MAX_CELLS, GFP_KERNEL);
	if (!ts->baseline || !ts->diff || !ts->stack || !ts->covered)
		return -ENOMEM;

	ts->spi = spi;
	mutex_init(&ts->lock);
	spi_set_drvdata(spi, ts);

	/*
	 * The controller has no firmware of its own. Leaving the property out
	 * assumes something else already put an image in place; one entry names
	 * it outright, and two name one image per panel source, picked apart by
	 * the source strap.
	 */
	if (device_property_present(dev, "firmware-name")) {
		ret = device_property_read_string_array(dev, "firmware-name",
						       NULL, 0);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "failed to count firmware-name\n");
		if (ret > 2)
			return dev_err_probe(dev, -EINVAL,
					     "at most two firmware images are supported, got %d\n",
					     ret);

		ts->fw_count = ret;
	}

	if (ts->fw_count) {
		ts->fw_names = devm_kcalloc(dev, ts->fw_count,
					    sizeof(*ts->fw_names), GFP_KERNEL);
		if (!ts->fw_names)
			return -ENOMEM;

		ret = device_property_read_string_array(dev, "firmware-name",
						       ts->fw_names,
						       ts->fw_count);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "failed to read firmware-name\n");
	}

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(nvt_supply_names),
					     nvt_supply_names);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = "Novatek NT36532 Touchscreen";
	ts->input->phys = "input/ts";
	ts->input->id.bustype = BUS_SPI;

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 0, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 0, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0,
			     NVT_PRESSURE_MAX, 0, 0);

	touchscreen_parse_properties(ts->input, true, &ts->prop);
	if (!ts->prop.max_x || !ts->prop.max_y)
		return dev_err_probe(dev, -EINVAL,
				     "touchscreen-size-x/y are required\n");

	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0,
			     max(ts->prop.max_x, ts->prop.max_y), 0, 0);

	ret = input_mt_init_slots(ts->input, NVT_MAX_FINGERS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init MT slots\n");

	ret = input_register_device(ts->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input\n");

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, nvt_ts_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"nt36532", ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	if (drm_is_panel_follower(dev)) {
		ts->panel_follower.funcs = &nvt_ts_panel_follower_funcs;

		/*
		 * The controller comes up with the panel, so leave the IRQ
		 * masked until the panel driver tells us it is powered.
		 */
		ret = devm_drm_panel_add_follower(dev, &ts->panel_follower);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to follow the panel\n");
	} else {
		ret = nvt_ts_start(ts);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct of_device_id nvt_ts_of_match[] = {
	{ .compatible = "novatek,nt36532" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nvt_ts_of_match);

static const struct spi_device_id nvt_ts_spi_ids[] = {
	{ "nt36532" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, nvt_ts_spi_ids);

static struct spi_driver nvt_ts_driver = {
	.probe = nvt_ts_probe,
	.id_table = nvt_ts_spi_ids,
	.driver = {
		.name = "novatek-nt36532",
		.of_match_table = nvt_ts_of_match,
		.pm = pm_sleep_ptr(&nvt_ts_pm_ops),
	},
};
module_spi_driver(nvt_ts_driver);

MODULE_AUTHOR("BigfootACA <bigfoot@classfun.cn>");
MODULE_DESCRIPTION("Novatek NT36532 SPI touchscreen driver");
MODULE_LICENSE("GPL");
