// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Sensing Hub IIO bridge for the Xiaomi Piano sensor topology.
 *
 * The physical I2C/I3C buses remain owned by the ADSP.  This driver consumes
 * calibrated sensor events from the QSH client QMI service and presents them
 * through the standard IIO ABI.
 *
 * The protobuf field layouts are from Qualcomm's public sensinghub project,
 * commit 6d7e56b85a518a514585efddec5e4a349e13d37d (BSD-3-Clause-Clear),
 * notably sns_client.proto, sns_suid.proto, sns_std.proto,
 * sns_std_sensor.proto, and sns_std_type.proto.  The QMI service envelope is
 * described by Qualcomm sns_client_api_v01 IDL revision 1.7.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/soc/qcom/qmi.h>
#include <linux/spinlock.h>
#include <linux/timekeeping.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define QSH_QMI_SERVICE			400
#define QSH_QMI_VERSION			1
#define QSH_QMI_INSTANCE		0

#define QSH_QMI_REQUEST			0x20
#define QSH_QMI_REPORT_IND		0x21
#define QSH_QMI_JUMBO_REPORT_IND	0x22

#define QSH_REQUEST_MAX			1000
#define QSH_REQUEST_ENCODED_MAX		1009
#define QSH_REPORT_MAX			1000
#define QSH_JUMBO_REPORT_MAX		10240
#define QSH_QMI_RX_MAX			(QSH_JUMBO_REPORT_MAX + 32)

#define QSH_CLIENT_DISABLE_REQ		10
#define QSH_STD_ATTR_REQ		1
#define QSH_STD_ATTR_EVENT		128
#define QSH_SUID_REQ			512
#define QSH_SUID_EVENT			768
#define QSH_SUID_DISCOVERY_DONE_EVENT	769
#define QSH_STD_SENSOR_CONFIG		513
#define QSH_STD_ON_CHANGE_CONFIG	514
#define QSH_PROXIMITY_EVENT		769
#define QSH_STD_SENSOR_EVENT		1025

#define QSH_SUID_SERVICE_LOW		0xababababababababULL
#define QSH_SUID_SERVICE_HIGH		0xababababababababULL

#define QSH_TICKS_PER_SECOND		19200000U
#define QSH_REQUEST_TIMEOUT		(5 * HZ)
#define QSH_DIRECT_TIMEOUT		(3 * HZ)
#define QSH_RESTORE_RETRY		HZ

#define QSH_MAX_CHANNELS		6
#define QSH_MAX_RATES			32
#define QSH_SCALE_MICRO			1000000U
#define QSH_SCALE_MILLI			1000U

enum qsh_sensor_id {
	QSH_ACCEL,
	QSH_GYRO,
	QSH_TEMPERATURE,
	QSH_MAGNETOMETER,
	QSH_FRONT_LIGHT,
	QSH_PROXIMITY,
	QSH_CCT,
	QSH_REAR_LIGHT,
	QSH_FLICKER,
	QSH_SENSOR_COUNT,
};

struct qsh_suid {
	u64 low;
	u64 high;
};

struct qsh_qmi_request {
	u32 payload_len;
	u8 payload[QSH_REQUEST_MAX];
	u8 use_jumbo_valid;
	u8 use_jumbo;
};

struct qsh_qmi_response {
	struct qmi_response_type_v01 response;
	u8 client_id_valid;
	u64 client_id;
	u8 result_valid;
	u32 result;
};

struct qsh_qmi_report {
	u64 client_id;
	u32 payload_len;
	u8 payload[QSH_REPORT_MAX];
};

struct qsh_qmi_jumbo_report {
	u64 client_id;
	u32 payload_len;
	u8 payload[QSH_JUMBO_REPORT_MAX];
};

struct qsh_pb_writer {
	u8 *data;
	size_t length;
	size_t capacity;
};

struct qsh_pb_reader {
	const u8 *position;
	const u8 *end;
};

struct qsh_clock_mapping {
	bool valid;
	s64 offset_ns;
};

struct qsh_hub;

struct qsh_sensor {
	struct qsh_hub *hub;
	struct iio_dev *indio_dev;
	const char *name;
	const char *datatype;
	struct qsh_suid suid;
	u32 sample_rate_micro_hz;
	u32 fixed_multiplier;
	u32 scale_denominator;
	int rates[QSH_MAX_RATES * 2];
	u8 rate_count;
	u8 channel_count;
	bool on_change;
	bool buffered;

	/* Serializes enable, disable, direct reads, and rate changes. */
	struct mutex op_lock;
	struct completion first_sample;
	/* Protects the latest sample and timestamp mapping. */
	spinlock_t sample_lock;
	s32 latest[QSH_MAX_CHANNELS];
	s64 latest_timestamp_ns;
	bool sample_valid;
	bool suid_valid;
	bool attributes_received;
	bool attributes_valid;
	bool rates_initialized;
	bool buffer_enabled;
	bool service_enabled;
	bool direct_enabled;

	unsigned long scan_masks[2];
};

struct qsh_hub {
	struct device *dev;
	struct qmi_handle qmi;
	struct sockaddr_qrtr server;
	/* Protects the server address and sensor service state flags. */
	struct mutex state_lock;
	/* Allows only one synchronous QMI transaction at a time. */
	struct mutex request_lock;
	struct delayed_work service_work;
	bool connected;
	bool removing;
	bool client_id_valid;
	u64 client_id;
	u32 generation;
	/* Protects the shared QSH clock-to-AP timestamp mappings. */
	spinlock_t timestamp_lock;
	struct qsh_clock_mapping timestamp_mappings[CLOCK_TAI + 1];
	struct qsh_sensor *sensors[QSH_SENSOR_COUNT];
	atomic_t malformed_events;
};

struct qsh_sensor_template {
	const char *name;
	const char *datatype;
	const struct iio_chan_spec *channels;
	u32 sample_rate_micro_hz;
	u32 fixed_multiplier;
	u32 scale_denominator;
	u8 channel_count;
	bool on_change;
	bool buffered;
};

#define QSH_SCAN_TYPE_S32					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 32,					\
		.storagebits = 32,				\
		.endianness = IIO_LE,				\
	}

#define QSH_STREAM_AXIS_CHANNEL(_type, _modifier, _index)		\
	{							\
		.type = (_type),					\
		.modified = 1,					\
		.channel2 = (_modifier),				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
		.info_mask_shared_by_type_available =			\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
		.scan_index = (_index),					\
		QSH_SCAN_TYPE_S32,					\
	}

#define QSH_ON_CHANGE_CHANNEL(_type, _index)			\
	{							\
		.type = (_type),					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = (_index),					\
		QSH_SCAN_TYPE_S32,					\
	}

#define QSH_STREAM_SINGLE_CHANNEL(_type)				\
	{							\
		.type = (_type),					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
		.info_mask_shared_by_type_available =			\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
		.scan_index = 0,					\
		QSH_SCAN_TYPE_S32,					\
	}

#define QSH_ON_CHANGE_SINGLE_CHANNEL(_type) \
	QSH_ON_CHANGE_CHANNEL(_type, 0)

static const struct iio_chan_spec qsh_accel_channels[] = {
	QSH_STREAM_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_X, 0),
	QSH_STREAM_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_Y, 1),
	QSH_STREAM_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec qsh_gyro_channels[] = {
	QSH_STREAM_AXIS_CHANNEL(IIO_ANGL_VEL, IIO_MOD_X, 0),
	QSH_STREAM_AXIS_CHANNEL(IIO_ANGL_VEL, IIO_MOD_Y, 1),
	QSH_STREAM_AXIS_CHANNEL(IIO_ANGL_VEL, IIO_MOD_Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec qsh_temperature_channels[] = {
	QSH_STREAM_SINGLE_CHANNEL(IIO_TEMP),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec qsh_magnetometer_channels[] = {
	QSH_STREAM_AXIS_CHANNEL(IIO_MAGN, IIO_MOD_X, 0),
	QSH_STREAM_AXIS_CHANNEL(IIO_MAGN, IIO_MOD_Y, 1),
	QSH_STREAM_AXIS_CHANNEL(IIO_MAGN, IIO_MOD_Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec qsh_light_channels[] = {
	QSH_ON_CHANGE_CHANNEL(IIO_LIGHT, 0),
	QSH_ON_CHANGE_CHANNEL(IIO_INTENSITY, 1),
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static const struct iio_chan_spec qsh_proximity_channels[] = {
	QSH_ON_CHANGE_CHANNEL(IIO_PROXIMITY, 0),
	QSH_ON_CHANGE_CHANNEL(IIO_INTENSITY, 1),
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static const struct iio_chan_spec qsh_cct_channels[] = {
	QSH_ON_CHANGE_SINGLE_CHANNEL(IIO_COLORTEMP),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec qsh_flicker_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY),
		.scan_index = -1,
	},
};

static const struct qsh_sensor_template qsh_piano_sensors[] = {
	[QSH_ACCEL] = {
		.name = "lsm6dso_accel",
		.datatype = "accel",
		.channels = qsh_accel_channels,
		.sample_rate_micro_hz = 26000000,
		.fixed_multiplier = QSH_SCALE_MICRO,
		.scale_denominator = QSH_SCALE_MICRO,
		.channel_count = 3,
		.buffered = true,
	},
	[QSH_GYRO] = {
		.name = "lsm6dso_gyro",
		.datatype = "gyro",
		.channels = qsh_gyro_channels,
		.sample_rate_micro_hz = 26000000,
		.fixed_multiplier = QSH_SCALE_MICRO,
		.scale_denominator = QSH_SCALE_MICRO,
		.channel_count = 3,
		.buffered = true,
	},
	[QSH_TEMPERATURE] = {
		.name = "lsm6dso_temp",
		.datatype = "sensor_temperature",
		.channels = qsh_temperature_channels,
		.sample_rate_micro_hz = 1000000,
		.fixed_multiplier = QSH_SCALE_MILLI,
		/* IIO temperature values are expressed in millidegrees Celsius. */
		.scale_denominator = 1,
		.channel_count = 1,
		.buffered = true,
	},
	[QSH_MAGNETOMETER] = {
		.name = "qmc630x_magn",
		.datatype = "mag",
		.channels = qsh_magnetometer_channels,
		.sample_rate_micro_hz = 10000000,
		/* QSH reports microtesla; the IIO magnetometer ABI uses gauss. */
		.fixed_multiplier = 10000,
		.scale_denominator = QSH_SCALE_MICRO,
		.channel_count = 3,
		.buffered = true,
	},
	[QSH_FRONT_LIGHT] = {
		.name = "stk3bcx_als",
		.datatype = "ambient_light",
		.channels = qsh_light_channels,
		.sample_rate_micro_hz = 5000000,
		.fixed_multiplier = QSH_SCALE_MILLI,
		.scale_denominator = QSH_SCALE_MILLI,
		.channel_count = 2,
		.on_change = true,
		.buffered = true,
	},
	[QSH_PROXIMITY] = {
		.name = "stk3bcx_proximity",
		.datatype = "proximity",
		.channels = qsh_proximity_channels,
		.sample_rate_micro_hz = 5000000,
		.fixed_multiplier = 1,
		.scale_denominator = 1,
		.channel_count = 2,
		.on_change = true,
		.buffered = true,
	},
	[QSH_CCT] = {
		.name = "stk3bcx_cct",
		.datatype = "cct_front",
		.channels = qsh_cct_channels,
		.sample_rate_micro_hz = 5000000,
		.fixed_multiplier = QSH_SCALE_MILLI,
		.scale_denominator = QSH_SCALE_MILLI,
		.channel_count = 1,
		.on_change = true,
		.buffered = true,
	},
	[QSH_REAR_LIGHT] = {
		.name = "sip1328_als",
		.datatype = "ambient_light_back",
		.channels = qsh_light_channels,
		.sample_rate_micro_hz = 5000000,
		.fixed_multiplier = QSH_SCALE_MILLI,
		.scale_denominator = QSH_SCALE_MILLI,
		.channel_count = 2,
		.on_change = true,
		.buffered = true,
	},
	[QSH_FLICKER] = {
		.name = "sip1328_flicker",
		.datatype = "flicker",
		.channels = qsh_flicker_channels,
		.sample_rate_micro_hz = 5000000,
		.fixed_multiplier = QSH_SCALE_MILLI,
		.scale_denominator = QSH_SCALE_MILLI,
		.channel_count = 1,
		.on_change = true,
		.buffered = false,
	},
};

static const struct qmi_elem_info qsh_request_ei[] = {
	{
		.data_type = QMI_DATA_LEN,
		.elem_len = 1,
		.elem_size = sizeof(u16),
		.array_type = NO_ARRAY,
		.tlv_type = 0x01,
		.offset = offsetof(struct qsh_qmi_request, payload_len),
	}, {
		.data_type = QMI_UNSIGNED_1_BYTE,
		.elem_len = QSH_REQUEST_MAX,
		.elem_size = sizeof(u8),
		.array_type = VAR_LEN_ARRAY,
		.tlv_type = 0x01,
		.offset = offsetof(struct qsh_qmi_request, payload),
	}, {
		.data_type = QMI_OPT_FLAG,
		.elem_len = 1,
		.elem_size = sizeof(u8),
		.array_type = NO_ARRAY,
		.tlv_type = 0x10,
		.offset = offsetof(struct qsh_qmi_request, use_jumbo_valid),
	}, {
		.data_type = QMI_UNSIGNED_1_BYTE,
		.elem_len = 1,
		.elem_size = sizeof(u8),
		.array_type = NO_ARRAY,
		.tlv_type = 0x10,
		.offset = offsetof(struct qsh_qmi_request, use_jumbo),
	}, { .data_type = QMI_EOTI },
};

static const struct qmi_elem_info qsh_response_ei[] = {
	{
		.data_type = QMI_STRUCT,
		.elem_len = 1,
		.elem_size = sizeof(struct qmi_response_type_v01),
		.array_type = NO_ARRAY,
		.tlv_type = 0x02,
		.offset = offsetof(struct qsh_qmi_response, response),
		.ei_array = qmi_response_type_v01_ei,
	}, {
		.data_type = QMI_OPT_FLAG,
		.elem_len = 1,
		.elem_size = sizeof(u8),
		.array_type = NO_ARRAY,
		.tlv_type = 0x10,
		.offset = offsetof(struct qsh_qmi_response, client_id_valid),
	}, {
		.data_type = QMI_UNSIGNED_8_BYTE,
		.elem_len = 1,
		.elem_size = sizeof(u64),
		.array_type = NO_ARRAY,
		.tlv_type = 0x10,
		.offset = offsetof(struct qsh_qmi_response, client_id),
	}, {
		.data_type = QMI_OPT_FLAG,
		.elem_len = 1,
		.elem_size = sizeof(u8),
		.array_type = NO_ARRAY,
		.tlv_type = 0x11,
		.offset = offsetof(struct qsh_qmi_response, result_valid),
	}, {
		.data_type = QMI_UNSIGNED_4_BYTE,
		.elem_len = 1,
		.elem_size = sizeof(u32),
		.array_type = NO_ARRAY,
		.tlv_type = 0x11,
		.offset = offsetof(struct qsh_qmi_response, result),
	}, { .data_type = QMI_EOTI },
};

#define QSH_REPORT_EI(_type, _maximum) \
static const struct qmi_elem_info qsh_##_type##_ei[] = { \
	{ \
		.data_type = QMI_UNSIGNED_8_BYTE, \
		.elem_len = 1, \
		.elem_size = sizeof(u64), \
		.array_type = NO_ARRAY, \
		.tlv_type = 0x01, \
		.offset = offsetof(struct qsh_qmi_##_type, client_id), \
	}, { \
		.data_type = QMI_DATA_LEN, \
		.elem_len = 1, \
		.elem_size = sizeof(u16), \
		.array_type = NO_ARRAY, \
		.tlv_type = 0x02, \
		.offset = offsetof(struct qsh_qmi_##_type, payload_len), \
	}, { \
		.data_type = QMI_UNSIGNED_1_BYTE, \
		.elem_len = (_maximum), \
		.elem_size = sizeof(u8), \
		.array_type = VAR_LEN_ARRAY, \
		.tlv_type = 0x02, \
		.offset = offsetof(struct qsh_qmi_##_type, payload), \
	}, { .data_type = QMI_EOTI }, \
}

QSH_REPORT_EI(report, QSH_REPORT_MAX);
QSH_REPORT_EI(jumbo_report, QSH_JUMBO_REPORT_MAX);

static bool qsh_pb_put(struct qsh_pb_writer *writer, const void *data,
		       size_t length)
{
	if (length > writer->capacity - writer->length)
		return false;

	memcpy(writer->data + writer->length, data, length);
	writer->length += length;
	return true;
}

static bool qsh_pb_put_varint(struct qsh_pb_writer *writer, u64 value)
{
	u8 byte;

	do {
		byte = value & 0x7f;
		value >>= 7;
		if (value)
			byte |= 0x80;
		if (!qsh_pb_put(writer, &byte, sizeof(byte)))
			return false;
	} while (value);

	return true;
}

static bool qsh_pb_put_key(struct qsh_pb_writer *writer, u32 field, u8 wire)
{
	return qsh_pb_put_varint(writer, ((u64)field << 3) | wire);
}

static bool qsh_pb_put_fixed32(struct qsh_pb_writer *writer, u32 field,
			       u32 value)
{
	__le32 encoded = cpu_to_le32(value);

	return qsh_pb_put_key(writer, field, 5) &&
	       qsh_pb_put(writer, &encoded, sizeof(encoded));
}

static bool qsh_pb_put_fixed64(struct qsh_pb_writer *writer, u32 field,
			       u64 value)
{
	__le64 encoded = cpu_to_le64(value);

	return qsh_pb_put_key(writer, field, 1) &&
	       qsh_pb_put(writer, &encoded, sizeof(encoded));
}

static bool qsh_pb_put_u32(struct qsh_pb_writer *writer, u32 field, u32 value)
{
	return qsh_pb_put_key(writer, field, 0) &&
	       qsh_pb_put_varint(writer, value);
}

static bool qsh_pb_put_bytes(struct qsh_pb_writer *writer, u32 field,
			     const void *data, size_t length)
{
	return qsh_pb_put_key(writer, field, 2) &&
	       qsh_pb_put_varint(writer, length) &&
	       qsh_pb_put(writer, data, length);
}

static bool qsh_pb_get_varint(struct qsh_pb_reader *reader, u64 *value)
{
	u64 result = 0;
	unsigned int shift;
	u8 byte;

	for (shift = 0; shift < 64; shift += 7) {
		if (reader->position == reader->end)
			return false;
		byte = *reader->position++;
		if (shift == 63 && (byte & 0xfe))
			return false;
		result |= (u64)(byte & 0x7f) << shift;
		if (!(byte & 0x80)) {
			*value = result;
			return true;
		}
	}

	return false;
}

static bool qsh_pb_get_key(struct qsh_pb_reader *reader, u32 *field, u8 *wire)
{
	u64 key;

	if (!qsh_pb_get_varint(reader, &key) || !key || key > U32_MAX)
		return false;
	*field = key >> 3;
	*wire = key & 7;
	return *wire == 0 || *wire == 1 || *wire == 2 || *wire == 5;
}

static bool qsh_pb_get_fixed32(struct qsh_pb_reader *reader, u32 *value)
{
	if (reader->end - reader->position < sizeof(__le32))
		return false;
	*value = get_unaligned_le32(reader->position);
	reader->position += sizeof(__le32);
	return true;
}

static bool qsh_pb_get_fixed64(struct qsh_pb_reader *reader, u64 *value)
{
	if (reader->end - reader->position < sizeof(__le64))
		return false;
	*value = get_unaligned_le64(reader->position);
	reader->position += sizeof(__le64);
	return true;
}

static bool qsh_pb_get_bytes(struct qsh_pb_reader *reader,
			     struct qsh_pb_reader *subreader)
{
	u64 length;

	if (!qsh_pb_get_varint(reader, &length) ||
	    length > reader->end - reader->position)
		return false;
	subreader->position = reader->position;
	subreader->end = reader->position + length;
	reader->position += length;
	return true;
}

static bool qsh_pb_skip(struct qsh_pb_reader *reader, u8 wire)
{
	struct qsh_pb_reader unused;
	u64 value;

	switch (wire) {
	case 0:
		return qsh_pb_get_varint(reader, &value);
	case 1:
		if (reader->end - reader->position < sizeof(u64))
			return false;
		reader->position += sizeof(u64);
		return true;
	case 2:
		return qsh_pb_get_bytes(reader, &unused);
	case 5:
		if (reader->end - reader->position < sizeof(u32))
			return false;
		reader->position += sizeof(u32);
		return true;
	default:
		return false;
	}
}

static u32 qsh_frequency_to_float(u32 frequency_micro_hz)
{
	u64 significand;
	int exponent = 0;
	u64 lower = QSH_SCALE_MICRO;

	if (!frequency_micro_hz)
		return 0;

	if (frequency_micro_hz >= QSH_SCALE_MICRO) {
		while (exponent < 30 &&
		       frequency_micro_hz >= (lower << 1)) {
			lower <<= 1;
			exponent++;
		}
	} else {
		while (exponent > -30 && frequency_micro_hz < lower) {
			lower >>= 1;
			exponent--;
		}
	}

	significand = DIV_ROUND_CLOSEST_ULL((u64)frequency_micro_hz <<
					     (23 - exponent), QSH_SCALE_MICRO);

	if (significand >= BIT_ULL(24)) {
		significand >>= 1;
		exponent++;
	}

	return ((exponent + 127) << 23) |
	       ((u32)significand & GENMASK(22, 0));
}

static bool qsh_build_client_request(const struct qsh_suid *destination,
				     u32 message_id, const u8 *payload,
				     size_t payload_len, bool batching,
				     bool disable,
				     struct qsh_qmi_request *request)
{
	u8 suid_data[32], suspend_data[16], batch_data[24];
	u8 std_request_data[64];
	struct qsh_pb_writer suid = { suid_data, 0, sizeof(suid_data) };
	struct qsh_pb_writer suspend = {
		suspend_data, 0, sizeof(suspend_data)
	};
	struct qsh_pb_writer batch = { batch_data, 0, sizeof(batch_data) };
	struct qsh_pb_writer std_request = {
		std_request_data, 0, sizeof(std_request_data)
	};
	struct qsh_pb_writer output = {
		request->payload, 0, sizeof(request->payload)
	};

	if (!qsh_pb_put_fixed64(&suid, 1, destination->low) ||
	    !qsh_pb_put_fixed64(&suid, 2, destination->high) ||
	    !qsh_pb_put_u32(&suspend, 1, 1) || /* APSS */
	    !qsh_pb_put_u32(&suspend, 2, 1)) /* no-wakeup delivery */
		return false;

	if (batching) {
		if (!qsh_pb_put_u32(&batch, 1, 0) ||
		    (disable && !qsh_pb_put_u32(&batch, 2, U32_MAX)) ||
		    !qsh_pb_put_bytes(&std_request, 1, batch.data,
				      batch.length))
			return false;
	}
	if (payload &&
	    !qsh_pb_put_bytes(&std_request, 2, payload, payload_len))
		return false;

	if (!qsh_pb_put_bytes(&output, 1, suid.data, suid.length) ||
	    !qsh_pb_put_fixed32(&output, 2, message_id) ||
	    !qsh_pb_put_bytes(&output, 3, suspend.data, suspend.length) ||
	    !qsh_pb_put_bytes(&output, 4, std_request.data,
			      std_request.length) ||
	    !qsh_pb_put_u32(&output, 7, 1)) /* SNS_TECH_SENSORS */
		return false;

	request->payload_len = output.length;
	request->use_jumbo_valid = 1;
	request->use_jumbo = 1;
	return true;
}

static int qsh_transact(struct qsh_hub *hub,
			struct qsh_qmi_request *request, u32 generation,
			const char *description, bool allow_removing)
{
	struct qsh_qmi_response response = {};
	struct sockaddr_qrtr server;
	struct qmi_txn transaction;
	int ret;

	mutex_lock(&hub->request_lock);
	mutex_lock(&hub->state_lock);
	if (!hub->connected || hub->generation != generation ||
	    (hub->removing && !allow_removing)) {
		mutex_unlock(&hub->state_lock);
		ret = -ENOTCONN;
		goto out_unlock;
	}
	server = hub->server;
	mutex_unlock(&hub->state_lock);

	ret = qmi_txn_init(&hub->qmi, &transaction, qsh_response_ei, &response);
	if (ret < 0)
		goto out_unlock;

	ret = qmi_send_request(&hub->qmi, &server, &transaction,
			       QSH_QMI_REQUEST, QSH_REQUEST_ENCODED_MAX,
			       qsh_request_ei, request);
	if (ret < 0) {
		qmi_txn_cancel(&transaction);
		goto out_unlock;
	}

	ret = qmi_txn_wait(&transaction, QSH_REQUEST_TIMEOUT);
	if (ret < 0)
		goto out_unlock;
	if (response.response.result != QMI_RESULT_SUCCESS_V01) {
		ret = -EREMOTEIO;
		goto out_unlock;
	}

	mutex_lock(&hub->state_lock);
	if (!hub->connected || hub->generation != generation ||
	    hub->server.sq_node != server.sq_node ||
	    hub->server.sq_port != server.sq_port) {
		ret = -ENOTCONN;
	} else if (!response.client_id_valid) {
		ret = -EPROTO;
	} else if (hub->client_id_valid &&
		   hub->client_id != response.client_id) {
		ret = -EPROTO;
	} else {
		hub->client_id = response.client_id;
		hub->client_id_valid = true;
	}
	mutex_unlock(&hub->state_lock);
	if (ret < 0)
		goto out_unlock;

	if (response.result_valid && response.result) {
		dev_dbg(hub->dev, "%s rejected: %u\n", description,
			response.result);
		ret = -EREMOTEIO;
		goto out_unlock;
	}

	ret = 0;
out_unlock:
	mutex_unlock(&hub->request_lock);
	return ret;
}

static int qsh_send_sensor_request(struct qsh_sensor *sensor, bool enable)
{
	struct qsh_hub *hub = sensor->hub;
	struct qsh_qmi_request *request;
	struct qsh_suid suid;
	u8 config_data[16];
	struct qsh_pb_writer config = {
		config_data, 0, sizeof(config_data)
	};
	u32 generation;
	u32 message_id;
	u32 sample_rate;
	int ret;

	mutex_lock(&hub->state_lock);
	if (!hub->connected) {
		mutex_unlock(&hub->state_lock);
		return -ENOTCONN;
	}
	if (!sensor->suid_valid ||
	    (enable && !sensor->attributes_received)) {
		mutex_unlock(&hub->state_lock);
		return -EAGAIN;
	}
	if (enable && !sensor->attributes_valid) {
		mutex_unlock(&hub->state_lock);
		return -ENODEV;
	}
	suid = sensor->suid;
	generation = hub->generation;
	mutex_unlock(&hub->state_lock);

	request = kzalloc_obj(*request, GFP_KERNEL);
	if (!request)
		return -ENOMEM;

	message_id = enable ?
		(sensor->on_change ? QSH_STD_ON_CHANGE_CONFIG :
		 QSH_STD_SENSOR_CONFIG) : QSH_CLIENT_DISABLE_REQ;
	if (enable && !sensor->on_change) {
		sample_rate = qsh_frequency_to_float(sensor->sample_rate_micro_hz);
		if (!qsh_pb_put_fixed32(&config, 1, sample_rate)) {
			ret = -EOVERFLOW;
			goto out_free;
		}
	}

	if (!qsh_build_client_request(&suid, message_id,
			      enable && !sensor->on_change ? config.data : NULL,
			      enable && !sensor->on_change ? config.length : 0,
			      true, !enable, request)) {
		ret = -EOVERFLOW;
		goto out_free;
	}

	ret = qsh_transact(hub, request, generation, sensor->name, !enable);
out_free:
	kfree(request);
	return ret;
}

static int qsh_send_suid_request(struct qsh_sensor *sensor)
{
	static const struct qsh_suid suid_service = {
		.low = QSH_SUID_SERVICE_LOW,
		.high = QSH_SUID_SERVICE_HIGH,
	};
	struct qsh_hub *hub = sensor->hub;
	struct qsh_qmi_request *request;
	u8 payload_data[96];
	struct qsh_pb_writer payload = {
		payload_data, 0, sizeof(payload_data)
	};
	u32 generation;
	int ret;

	mutex_lock(&hub->state_lock);
	if (!hub->connected || hub->removing) {
		mutex_unlock(&hub->state_lock);
		return -ENOTCONN;
	}
	generation = hub->generation;
	mutex_unlock(&hub->state_lock);

	request = kzalloc_obj(*request, GFP_KERNEL);
	if (!request)
		return -ENOMEM;

	if (!qsh_pb_put_bytes(&payload, 1, sensor->datatype,
			      strlen(sensor->datatype)) ||
	    !qsh_pb_put_u32(&payload, 2, 0) || /* one-shot lookup */
	    !qsh_pb_put_u32(&payload, 3, 1) || /* default sensor only */
	    !qsh_build_client_request(&suid_service, QSH_SUID_REQ,
				      payload.data, payload.length, false,
				      false, request)) {
		ret = -EOVERFLOW;
		goto out_free;
	}

	ret = qsh_transact(hub, request, generation, sensor->datatype, false);
out_free:
	kfree(request);
	return ret;
}

static int qsh_send_attribute_request(struct qsh_sensor *sensor)
{
	struct qsh_hub *hub = sensor->hub;
	struct qsh_qmi_request *request;
	struct qsh_suid suid;
	u32 generation;
	int ret;

	mutex_lock(&hub->state_lock);
	if (!hub->connected || !sensor->suid_valid || hub->removing) {
		mutex_unlock(&hub->state_lock);
		return -EAGAIN;
	}
	suid = sensor->suid;
	generation = hub->generation;
	mutex_unlock(&hub->state_lock);

	request = kzalloc_obj(*request, GFP_KERNEL);
	if (!request)
		return -ENOMEM;
	if (!qsh_build_client_request(&suid, QSH_STD_ATTR_REQ, NULL, 0, false,
				      false, request)) {
		ret = -EOVERFLOW;
		goto out_free;
	}

	ret = qsh_transact(hub, request, generation, sensor->name, false);
out_free:
	kfree(request);
	return ret;
}

static struct qsh_sensor *qsh_find_sensor(struct qsh_hub *hub,
					  const struct qsh_suid *suid)
{
	unsigned int index;

	lockdep_assert_held(&hub->state_lock);

	for (index = 0; index < QSH_SENSOR_COUNT; index++) {
		struct qsh_sensor *sensor = hub->sensors[index];

		if (sensor && sensor->suid_valid &&
		    sensor->suid.low == suid->low &&
		    sensor->suid.high == suid->high)
			return sensor;
	}

	return NULL;
}

static bool qsh_suid_is_service(const struct qsh_suid *suid)
{
	return suid->low == QSH_SUID_SERVICE_LOW &&
	       suid->high == QSH_SUID_SERVICE_HIGH;
}

static bool qsh_pb_string_equal(struct qsh_pb_reader reader, const char *string)
{
	size_t length = strlen(string);

	return reader.end - reader.position == length &&
	       !memcmp(reader.position, string, length);
}

static bool qsh_parse_suid(struct qsh_pb_reader reader, struct qsh_suid *suid)
{
	u32 field;
	u8 wire;
	u64 value;
	bool have_low = false, have_high = false;

	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (field == 1 && wire == 1) {
			if (!qsh_pb_get_fixed64(&reader, &value))
				return false;
			suid->low = value;
			have_low = true;
		} else if (field == 2 && wire == 1) {
			if (!qsh_pb_get_fixed64(&reader, &value))
				return false;
			suid->high = value;
			have_high = true;
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	return have_low && have_high;
}

static bool qsh_float_to_fixed(u32 bits, u32 multiplier, s32 *output)
{
	u32 exponent = FIELD_GET(GENMASK(30, 23), bits);
	u32 mantissa = bits & GENMASK(22, 0);
	u64 value;
	int shift;
	bool negative = bits & BIT(31);

	if (exponent == 0xff)
		return false;
	if (!exponent) {
		*output = 0;
		return true;
	}

	mantissa |= BIT(23);
	value = (u64)mantissa * multiplier;
	shift = (int)exponent - 127 - 23;
	if (shift >= 0) {
		if (shift >= 31 || value > ((u64)S32_MAX >> shift))
			value = negative ? (u64)S32_MAX + 1 : S32_MAX;
		else
			value <<= shift;
	} else if (-shift >= 64) {
		value = 0;
	} else {
		value = DIV_ROUND_CLOSEST_ULL(value, BIT_ULL(-shift));
	}

	if (negative) {
		if (value >= (u64)S32_MAX + 1)
			*output = S32_MIN;
		else
			*output = -(s32)value;
	} else {
		*output = min_t(u64, value, S32_MAX);
	}

	return true;
}

struct qsh_attribute_result {
	u32 rates[QSH_MAX_RATES];
	u8 rate_count;
	u8 stream_type;
	bool available;
	bool have_available;
	bool have_stream_type;
};

static bool qsh_parse_attribute_value(struct qsh_pb_reader reader, u32 id,
				      struct qsh_attribute_result *result,
				      unsigned int depth);

static bool qsh_parse_attribute_data(struct qsh_pb_reader reader, u32 id,
				     struct qsh_attribute_result *result,
				     unsigned int depth)
{
	struct qsh_pb_reader subreader;
	u32 field, bits;
	u8 wire;
	u64 value;
	s32 rate;

	if (depth > 4)
		return false;

	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (field == 1 && wire == 2) {
			if (!qsh_pb_get_bytes(&reader, &subreader) ||
			    !qsh_parse_attribute_value(subreader, id, result,
						       depth + 1))
				return false;
		} else if (id == 3 && field == 5 && wire == 0) {
			if (!qsh_pb_get_varint(&reader, &value) || value > 1)
				return false;
			result->available = value;
			result->have_available = true;
		} else if (id == 6 && field == 3 && wire == 5) {
			if (!qsh_pb_get_fixed32(&reader, &bits) ||
			    !qsh_float_to_fixed(bits, QSH_SCALE_MICRO, &rate) ||
			    rate <= 0 || result->rate_count == QSH_MAX_RATES)
				return false;
			result->rates[result->rate_count++] = rate;
		} else if (id == 16 && field == 4 && wire == 1) {
			if (!qsh_pb_get_fixed64(&reader, &value) || value > 2)
				return false;
			result->stream_type = value;
			result->have_stream_type = true;
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	return true;
}

static bool qsh_parse_attribute_value(struct qsh_pb_reader reader, u32 id,
				      struct qsh_attribute_result *result,
				      unsigned int depth)
{
	struct qsh_pb_reader subreader;
	u32 field;
	u8 wire;

	if (depth > 4)
		return false;

	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (field == 1 && wire == 2) {
			if (!qsh_pb_get_bytes(&reader, &subreader) ||
			    !qsh_parse_attribute_data(subreader, id, result, depth))
				return false;
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	return true;
}

static bool qsh_parse_attribute(struct qsh_pb_reader reader,
				struct qsh_attribute_result *result)
{
	struct qsh_pb_reader scan = reader;
	struct qsh_pb_reader value_reader = {};
	u32 field, id = U32_MAX;
	u8 wire;
	u64 value;
	bool have_value = false;

	while (scan.position < scan.end) {
		if (!qsh_pb_get_key(&scan, &field, &wire))
			return false;
		if (field == 1 && wire == 0) {
			if (!qsh_pb_get_varint(&scan, &value) || value > S32_MAX)
				return false;
			id = value;
		} else if (!qsh_pb_skip(&scan, wire)) {
			return false;
		}
	}

	if (id != 3 && id != 6 && id != 16)
		return true;

	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (field == 2 && wire == 2) {
			if (!qsh_pb_get_bytes(&reader, &value_reader))
				return false;
			have_value = true;
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	return have_value &&
	       qsh_parse_attribute_value(value_reader, id, result, 0);
}

static void qsh_normalize_rates(struct qsh_attribute_result *result)
{
	unsigned int count = 0;
	unsigned int index;

	for (index = 0; index < result->rate_count; index++) {
		u32 rate = result->rates[index];
		unsigned int insert = count;

		while (insert && result->rates[insert - 1] > rate) {
			result->rates[insert] = result->rates[insert - 1];
			insert--;
		}
		if ((insert && result->rates[insert - 1] == rate) ||
		    (insert < count && result->rates[insert] == rate))
			continue;
		result->rates[insert] = rate;
		count++;
	}

	result->rate_count = count;
}

static void qsh_store_attributes(struct qsh_sensor *sensor,
				 const struct qsh_attribute_result *result)
{
	struct qsh_hub *hub = sensor->hub;
	bool mismatch = false;
	bool ready;
	unsigned int index;
	bool found = false;
	u8 expected_stream_type = sensor->on_change ? 1 : 0;

	ready = result->have_available && result->available &&
		result->have_stream_type &&
		result->stream_type == expected_stream_type &&
		(sensor->on_change || result->rate_count);

	mutex_lock(&hub->state_lock);
	if (!hub->connected || !sensor->suid_valid || hub->removing) {
		mutex_unlock(&hub->state_lock);
		return;
	}

	if (ready && !sensor->on_change) {
		if (!sensor->rates_initialized) {
			for (index = 0; index < result->rate_count; index++) {
				sensor->rates[index * 2] =
					result->rates[index] / QSH_SCALE_MICRO;
				sensor->rates[index * 2 + 1] =
					result->rates[index] % QSH_SCALE_MICRO;
			}
			sensor->rate_count = result->rate_count;
			sensor->rates_initialized = true;
		} else if (sensor->rate_count != result->rate_count) {
			mismatch = true;
		} else {
			for (index = 0; index < result->rate_count; index++) {
				u32 rate = sensor->rates[index * 2] *
					   QSH_SCALE_MICRO +
					   sensor->rates[index * 2 + 1];

				if (rate != result->rates[index]) {
					mismatch = true;
					break;
				}
			}
		}

		for (index = 0; index < result->rate_count; index++)
			found |= sensor->sample_rate_micro_hz ==
				 result->rates[index];
		if (!found)
			sensor->sample_rate_micro_hz = result->rates[0];
	}

	sensor->attributes_received = true;
	sensor->attributes_valid = ready && !mismatch;
	mutex_unlock(&hub->state_lock);

	if (mismatch)
		dev_warn_ratelimited(hub->dev,
				     "%s rates changed across service restart\n",
				     sensor->name);
}

static bool qsh_parse_attribute_event(struct qsh_sensor *sensor,
				      struct qsh_pb_reader reader)
{
	struct qsh_attribute_result result = {};
	struct qsh_pb_reader attribute;
	u32 field;
	u8 wire;

	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (field == 1 && wire == 2) {
			if (!qsh_pb_get_bytes(&reader, &attribute) ||
			    !qsh_parse_attribute(attribute, &result))
				return false;
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	qsh_normalize_rates(&result);
	qsh_store_attributes(sensor, &result);
	return true;
}

static bool qsh_parse_standard_sample(struct qsh_sensor *sensor,
				      struct qsh_pb_reader reader,
				      s32 *values, u32 *status)
{
	struct qsh_pb_reader packed;
	u32 field, bits;
	u8 wire;
	u64 value;
	unsigned int count = 0;
	bool have_status = false;

	*status = 0;
	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (field == 1 && wire == 5) {
			if (!qsh_pb_get_fixed32(&reader, &bits))
				return false;
			if (count < sensor->channel_count &&
			    !qsh_float_to_fixed(bits, sensor->fixed_multiplier,
						&values[count]))
				return false;
			count++;
		} else if (field == 1 && wire == 2) {
			if (!qsh_pb_get_bytes(&reader, &packed) ||
			    (packed.end - packed.position) % sizeof(u32))
				return false;
			while (packed.position < packed.end) {
				if (!qsh_pb_get_fixed32(&packed, &bits))
					return false;
				if (count < sensor->channel_count &&
				    !qsh_float_to_fixed(bits,
							sensor->fixed_multiplier,
							&values[count]))
					return false;
				count++;
			}
		} else if (field == 2 && wire == 0) {
			if (!qsh_pb_get_varint(&reader, &value) || value > 3)
				return false;
			*status = value;
			have_status = true;
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	return count >= sensor->channel_count && have_status;
}

static bool qsh_parse_proximity_sample(struct qsh_pb_reader reader,
				       s32 *values, u32 *status)
{
	u32 field;
	u8 wire;
	u64 value;
	bool have_state = false, have_adc = false, have_status = false;

	*status = 0;
	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (wire == 0 && field >= 1 && field <= 3) {
			if (!qsh_pb_get_varint(&reader, &value) || value > U32_MAX)
				return false;
			if (field == 1) {
				values[0] = value ? 1 : 0;
				have_state = true;
			} else if (field == 2) {
				values[1] = min_t(u64, value, S32_MAX);
				have_adc = true;
			} else {
				if (value > 3)
					return false;
				*status = value;
				have_status = true;
			}
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	return have_state && have_adc && have_status;
}

static u64 qsh_ticks_to_ns(u64 ticks)
{
	return mul_u64_u32_div(ticks, NSEC_PER_SEC, QSH_TICKS_PER_SECOND);
}

static s64 qsh_map_timestamp(struct qsh_clock_mapping *mapping,
			     u64 timestamp, s64 now)
{
	s64 result;
	u64 timestamp_ns = qsh_ticks_to_ns(timestamp);

	if (!mapping->valid) {
		mapping->offset_ns = now - timestamp_ns;
		mapping->valid = true;
		return now;
	}

	result = mapping->offset_ns + timestamp_ns;
	if (result > now + 30LL * NSEC_PER_SEC ||
	    result < now - 30LL * NSEC_PER_SEC) {
		mapping->offset_ns = now - timestamp_ns;
		return now;
	}

	return result;
}

static s64 qsh_sensor_timestamp(struct qsh_sensor *sensor, u64 timestamp)
{
	struct qsh_hub *hub = sensor->hub;
	clockid_t clock = iio_device_get_clock(sensor->indio_dev);
	unsigned long flags;
	s64 now = iio_get_time_ns(sensor->indio_dev);
	s64 result;

	if (WARN_ON_ONCE(clock < 0 || clock > CLOCK_TAI))
		return now;

	spin_lock_irqsave(&hub->timestamp_lock, flags);
	result = qsh_map_timestamp(&hub->timestamp_mappings[clock], timestamp,
				   now);
	spin_unlock_irqrestore(&hub->timestamp_lock, flags);

	return result;
}

static void qsh_publish_sample(struct qsh_sensor *sensor, const s32 *values,
			       u64 timestamp)
{
	__le32 scan[QSH_MAX_CHANNELS];
	unsigned long flags;
	s64 timestamp_ns;
	bool push;
	unsigned int index;

	timestamp_ns = qsh_sensor_timestamp(sensor, timestamp);
	spin_lock_irqsave(&sensor->sample_lock, flags);
	for (index = 0; index < sensor->channel_count; index++) {
		sensor->latest[index] = values[index];
		scan[index] = cpu_to_le32(values[index]);
	}
	sensor->latest_timestamp_ns = timestamp_ns;
	sensor->sample_valid = true;
	spin_unlock_irqrestore(&sensor->sample_lock, flags);
	mutex_lock(&sensor->hub->state_lock);
	push = sensor->buffer_enabled;
	mutex_unlock(&sensor->hub->state_lock);

	complete_all(&sensor->first_sample);
	if (push)
		iio_push_to_buffers_with_ts_unaligned(sensor->indio_dev, scan,
						      sensor->channel_count *
						      sizeof(scan[0]),
						      timestamp_ns);
}

static bool qsh_parse_suid_event_payload(struct qsh_hub *hub,
					 struct qsh_pb_reader reader)
{
	struct qsh_pb_reader datatype = {};
	struct qsh_pb_reader subreader;
	struct qsh_suid suid = {};
	struct qsh_sensor *sensor = NULL;
	u32 field;
	u8 wire;
	unsigned int index;
	bool have_datatype = false, have_suid = false;

	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (field == 1 && wire == 2) {
			if (!qsh_pb_get_bytes(&reader, &datatype))
				return false;
			have_datatype = true;
		} else if (field == 2 && wire == 2) {
			if (!qsh_pb_get_bytes(&reader, &subreader))
				return false;
			if (!have_suid) {
				if (!qsh_parse_suid(subreader, &suid))
					return false;
				have_suid = true;
			}
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	if (!have_datatype)
		return false;
	if (!have_suid)
		return true;

	for (index = 0; index < QSH_SENSOR_COUNT; index++) {
		if (qsh_pb_string_equal(datatype,
					hub->sensors[index]->datatype)) {
			sensor = hub->sensors[index];
			break;
		}
	}
	if (!sensor)
		return true;

	mutex_lock(&hub->state_lock);
	if (hub->connected && !hub->removing) {
		if (!sensor->suid_valid || sensor->suid.low != suid.low ||
		    sensor->suid.high != suid.high) {
			sensor->suid = suid;
			sensor->suid_valid = true;
			sensor->attributes_received = false;
			sensor->attributes_valid = false;
		}
	}
	mutex_unlock(&hub->state_lock);
	mod_delayed_work(system_wq, &hub->service_work, 0);

	return true;
}

static bool qsh_parse_suid_client_event(struct qsh_hub *hub,
					struct qsh_pb_reader reader)
{
	struct qsh_pb_reader payload = {};
	u32 field, message_id = 0;
	u8 wire;
	u64 timestamp;
	bool have_message = false, have_timestamp = false, have_payload = false;

	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (field == 1 && wire == 5) {
			if (!qsh_pb_get_fixed32(&reader, &message_id))
				return false;
			have_message = true;
		} else if (field == 2 && wire == 1) {
			if (!qsh_pb_get_fixed64(&reader, &timestamp))
				return false;
			have_timestamp = true;
		} else if (field == 3 && wire == 2) {
			if (!qsh_pb_get_bytes(&reader, &payload))
				return false;
			have_payload = true;
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	if (!have_message || !have_timestamp || !have_payload)
		return false;
	if (message_id == QSH_SUID_DISCOVERY_DONE_EVENT)
		return true;
	if (message_id != QSH_SUID_EVENT)
		return true;

	return qsh_parse_suid_event_payload(hub, payload);
}

static bool qsh_parse_client_event(struct qsh_hub *hub,
				   struct qsh_sensor *sensor,
				   struct qsh_pb_reader reader)
{
	struct qsh_pb_reader payload = {};
	u32 field, message_id = 0;
	u8 wire;
	u64 timestamp = 0;
	s32 values[QSH_MAX_CHANNELS] = {};
	u32 status;
	bool active;
	bool have_message = false, have_timestamp = false, have_payload = false;

	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (field == 1 && wire == 5) {
			if (!qsh_pb_get_fixed32(&reader, &message_id))
				return false;
			have_message = true;
		} else if (field == 2 && wire == 1) {
			if (!qsh_pb_get_fixed64(&reader, &timestamp))
				return false;
			have_timestamp = true;
		} else if (field == 3 && wire == 2) {
			if (!qsh_pb_get_bytes(&reader, &payload))
				return false;
			have_payload = true;
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	if (!have_message || !have_timestamp || !have_payload)
		return false;
	if (message_id == QSH_STD_ATTR_EVENT) {
		if (!qsh_parse_attribute_event(sensor, payload))
			return false;
		mod_delayed_work(system_wq, &hub->service_work, 0);
		return true;
	}

	mutex_lock(&hub->state_lock);
	active = sensor->service_enabled;
	mutex_unlock(&hub->state_lock);
	if (!active)
		return true;

	if (sensor == hub->sensors[QSH_PROXIMITY] &&
	    message_id == QSH_PROXIMITY_EVENT) {
		if (!qsh_parse_proximity_sample(payload, values, &status))
			return false;
		if (!status)
			return true;
		qsh_publish_sample(sensor, values, timestamp);
		return true;
	}

	if (message_id == QSH_STD_SENSOR_EVENT) {
		if (sensor == hub->sensors[QSH_PROXIMITY])
			return true;
		if (!qsh_parse_standard_sample(sensor, payload, values, &status))
			return false;
		if (!status)
			return true;
		qsh_publish_sample(sensor, values, timestamp);
		return true;
	}

	/* Includes configuration/calibration events and vendor flicker waves. */
	return true;
}

static bool qsh_parse_event_message(struct qsh_hub *hub, const u8 *data,
				    size_t length)
{
	struct qsh_pb_reader reader = { data, data + length };
	struct qsh_pb_reader scan = reader;
	struct qsh_pb_reader subreader;
	struct qsh_suid suid = {};
	struct qsh_sensor *sensor = NULL;
	u32 field;
	u8 wire;
	bool have_suid = false;

	/* Protobuf field order is not significant, so locate the SUID first. */
	while (scan.position < scan.end) {
		if (!qsh_pb_get_key(&scan, &field, &wire))
			return false;
		if (field == 1 && wire == 2) {
			if (!qsh_pb_get_bytes(&scan, &subreader) ||
			    !qsh_parse_suid(subreader, &suid))
				return false;
			have_suid = true;
		} else if (!qsh_pb_skip(&scan, wire)) {
			return false;
		}
	}
	if (!have_suid)
		return false;
	if (!qsh_suid_is_service(&suid)) {
		mutex_lock(&hub->state_lock);
		sensor = qsh_find_sensor(hub, &suid);
		mutex_unlock(&hub->state_lock);
	}

	while (reader.position < reader.end) {
		if (!qsh_pb_get_key(&reader, &field, &wire))
			return false;
		if (field == 2 && wire == 2) {
			if (!qsh_pb_get_bytes(&reader, &subreader))
				return false;
			if (qsh_suid_is_service(&suid)) {
				if (!qsh_parse_suid_client_event(hub, subreader))
					return false;
			} else if (sensor &&
				   !qsh_parse_client_event(hub, sensor, subreader)) {
				return false;
			}
		} else if (!qsh_pb_skip(&reader, wire)) {
			return false;
		}
	}

	return true;
}

static bool qsh_validate_indication(struct qsh_hub *hub,
				    const struct sockaddr_qrtr *source,
				    u64 client_id)
{
	bool valid = false;

	mutex_lock(&hub->state_lock);
	if (source && hub->connected && !hub->removing &&
	    source->sq_node == hub->server.sq_node &&
	    source->sq_port == hub->server.sq_port &&
	    (!hub->client_id_valid || hub->client_id == client_id)) {
		hub->client_id = client_id;
		hub->client_id_valid = true;
		valid = true;
	}
	mutex_unlock(&hub->state_lock);

	return valid;
}

static void qsh_handle_report(struct qsh_hub *hub, const u8 *payload,
			      u32 payload_len, u32 maximum)
{
	if (payload_len > maximum ||
	    !qsh_parse_event_message(hub, payload, payload_len)) {
		if (atomic_inc_return(&hub->malformed_events) <= 5)
			dev_warn(hub->dev, "malformed QSH event payload (%u bytes)\n",
				 payload_len);
	}
}

static void qsh_report_indication(struct qmi_handle *qmi,
				  struct sockaddr_qrtr *source,
				  struct qmi_txn *transaction,
				  const void *decoded)
{
	struct qsh_hub *hub = container_of(qmi, struct qsh_hub, qmi);
	const struct qsh_qmi_report *report = decoded;

	if (!qsh_validate_indication(hub, source, report->client_id))
		return;
	qsh_handle_report(hub, report->payload, report->payload_len,
			  QSH_REPORT_MAX);
}

static void qsh_jumbo_report_indication(struct qmi_handle *qmi,
					struct sockaddr_qrtr *source,
					struct qmi_txn *transaction,
					const void *decoded)
{
	struct qsh_hub *hub = container_of(qmi, struct qsh_hub, qmi);
	const struct qsh_qmi_jumbo_report *report = decoded;

	if (!qsh_validate_indication(hub, source, report->client_id))
		return;
	qsh_handle_report(hub, report->payload, report->payload_len,
			  QSH_JUMBO_REPORT_MAX);
}

static const struct qmi_msg_handler qsh_qmi_handlers[] = {
	{
		.type = QMI_INDICATION,
		.msg_id = QSH_QMI_REPORT_IND,
		.ei = qsh_report_ei,
		.decoded_size = sizeof(struct qsh_qmi_report),
		.fn = qsh_report_indication,
	}, {
		.type = QMI_INDICATION,
		.msg_id = QSH_QMI_JUMBO_REPORT_IND,
		.ei = qsh_jumbo_report_ei,
		.decoded_size = sizeof(struct qsh_qmi_jumbo_report),
		.fn = qsh_jumbo_report_indication,
	}, {}
};

static void qsh_service_work(struct work_struct *work)
{
	struct qsh_hub *hub = container_of(to_delayed_work(work), struct qsh_hub,
					   service_work);
	bool retry = false;
	unsigned int index;

	for (index = 0; index < QSH_SENSOR_COUNT; index++) {
		struct qsh_sensor *sensor = hub->sensors[index];
		bool need_attributes, need_suid;
		bool cleanup_direct, restore_buffer;
		int ret;

		if (!sensor)
			continue;

		mutex_lock(&hub->state_lock);
		need_suid = hub->connected && !hub->removing &&
			    !sensor->suid_valid;
		need_attributes = hub->connected && !hub->removing &&
				  sensor->suid_valid &&
				  !sensor->attributes_received;
		mutex_unlock(&hub->state_lock);

		if (need_suid) {
			ret = qsh_send_suid_request(sensor);
			if (ret && ret != -ENOTCONN)
				dev_dbg(hub->dev, "%s lookup failed: %d\n",
					sensor->datatype, ret);
			mutex_lock(&hub->state_lock);
			retry |= hub->connected && !hub->removing &&
				 !sensor->suid_valid;
			mutex_unlock(&hub->state_lock);
			continue;
		}

		if (need_attributes) {
			ret = qsh_send_attribute_request(sensor);
			if (ret && ret != -ENOTCONN && ret != -EAGAIN)
				dev_dbg(hub->dev, "%s attribute request failed: %d\n",
					sensor->name, ret);
			mutex_lock(&hub->state_lock);
			retry |= hub->connected && !hub->removing &&
				 sensor->suid_valid &&
				 !sensor->attributes_received;
			mutex_unlock(&hub->state_lock);
			continue;
		}

		mutex_lock(&sensor->op_lock);
		mutex_lock(&hub->state_lock);
		cleanup_direct = sensor->direct_enabled &&
				 sensor->service_enabled &&
				 !sensor->buffer_enabled && hub->connected;
		restore_buffer = sensor->buffer_enabled &&
				 !sensor->service_enabled &&
				 sensor->attributes_valid && hub->connected &&
				 !hub->removing;
		mutex_unlock(&hub->state_lock);

		if (cleanup_direct) {
			ret = qsh_send_sensor_request(sensor, false);
			if (!ret || ret == -ENOTCONN) {
				mutex_lock(&hub->state_lock);
				sensor->direct_enabled = false;
				sensor->service_enabled = false;
				mutex_unlock(&hub->state_lock);
			} else {
				retry = true;
			}
		}

		if (restore_buffer) {
			ret = qsh_send_sensor_request(sensor, true);
			if (!ret) {
				mutex_lock(&hub->state_lock);
				if (sensor->buffer_enabled &&
				    sensor->attributes_valid && hub->connected &&
				    !hub->removing) {
					sensor->service_enabled = true;
				} else {
					retry = true;
				}
				mutex_unlock(&hub->state_lock);
			} else {
				dev_dbg(hub->dev, "%s restore failed: %d\n",
					sensor->name, ret);
				mutex_lock(&hub->state_lock);
				retry |= sensor->buffer_enabled &&
					 !sensor->service_enabled &&
					 hub->connected && !hub->removing;
				mutex_unlock(&hub->state_lock);
			}
		}
		mutex_unlock(&sensor->op_lock);
	}

	mutex_lock(&hub->state_lock);
	if (retry && hub->connected && !hub->removing)
		mod_delayed_work(system_wq, &hub->service_work,
				 QSH_RESTORE_RETRY);
	mutex_unlock(&hub->state_lock);
}

static int qsh_new_server(struct qmi_handle *qmi, struct qmi_service *service)
{
	struct qsh_hub *hub = container_of(qmi, struct qsh_hub, qmi);
	unsigned long flags;
	bool restore = false;
	unsigned int index;

	mutex_lock(&hub->state_lock);
	if (!hub->removing) {
		hub->server.sq_family = AF_QIPCRTR;
		hub->server.sq_node = service->node;
		hub->server.sq_port = service->port;
		hub->connected = true;
		hub->client_id_valid = false;
		hub->generation++;
		for (index = 0; index < QSH_SENSOR_COUNT; index++) {
			struct qsh_sensor *sensor = hub->sensors[index];

			if (!sensor)
				continue;
			sensor->suid_valid = false;
			sensor->attributes_received = false;
			sensor->attributes_valid = false;
			sensor->service_enabled = false;
		}
		spin_lock_irqsave(&hub->timestamp_lock, flags);
		memset(hub->timestamp_mappings, 0,
		       sizeof(hub->timestamp_mappings));
		spin_unlock_irqrestore(&hub->timestamp_lock, flags);
		restore = true;
	}
	mutex_unlock(&hub->state_lock);
	if (restore)
		mod_delayed_work(system_wq, &hub->service_work, 0);
	return 0;
}

static void qsh_del_server(struct qmi_handle *qmi, struct qmi_service *service)
{
	struct qsh_hub *hub = container_of(qmi, struct qsh_hub, qmi);
	unsigned long flags;
	unsigned int index;

	mutex_lock(&hub->state_lock);
	if (hub->server.sq_node == service->node &&
	    hub->server.sq_port == service->port) {
		hub->connected = false;
		hub->client_id_valid = false;
		hub->generation++;
		hub->server.sq_node = 0;
		hub->server.sq_port = 0;
		spin_lock_irqsave(&hub->timestamp_lock, flags);
		memset(hub->timestamp_mappings, 0,
		       sizeof(hub->timestamp_mappings));
		spin_unlock_irqrestore(&hub->timestamp_lock, flags);
		for (index = 0; index < QSH_SENSOR_COUNT; index++) {
			struct qsh_sensor *sensor = hub->sensors[index];

			if (!sensor)
				continue;
			sensor->service_enabled = false;
			sensor->direct_enabled = false;
			sensor->suid_valid = false;
			sensor->attributes_received = false;
			sensor->attributes_valid = false;
			spin_lock_irqsave(&sensor->sample_lock, flags);
			sensor->sample_valid = false;
			spin_unlock_irqrestore(&sensor->sample_lock, flags);
			complete_all(&sensor->first_sample);
		}
	}
	mutex_unlock(&hub->state_lock);
}

static const struct qmi_ops qsh_qmi_ops = {
	.new_server = qsh_new_server,
	.del_server = qsh_del_server,
};

static int qsh_buffer_postenable(struct iio_dev *indio_dev)
{
	struct qsh_sensor *sensor = iio_priv(indio_dev);
	struct qsh_hub *hub = sensor->hub;
	int ret;

	mutex_lock(&sensor->op_lock);
	mutex_lock(&hub->state_lock);
	if (!hub->connected || !sensor->suid_valid ||
	    !sensor->attributes_valid || hub->removing) {
		mutex_unlock(&hub->state_lock);
		mutex_unlock(&sensor->op_lock);
		return -EAGAIN;
	}
	sensor->buffer_enabled = true;
	sensor->service_enabled = true;
	mutex_unlock(&hub->state_lock);
	reinit_completion(&sensor->first_sample);
	ret = qsh_send_sensor_request(sensor, true);
	mutex_lock(&hub->state_lock);
	if (ret || !hub->connected || !sensor->attributes_valid ||
	    hub->removing) {
		sensor->buffer_enabled = false;
		sensor->service_enabled = false;
		if (!ret)
			ret = -ENOTCONN;
	}
	mutex_unlock(&hub->state_lock);
	mutex_unlock(&sensor->op_lock);
	return ret;
}

static int qsh_buffer_predisable(struct iio_dev *indio_dev)
{
	struct qsh_sensor *sensor = iio_priv(indio_dev);
	struct qsh_hub *hub = sensor->hub;
	bool send_disable;
	int ret = 0;

	mutex_lock(&sensor->op_lock);
	mutex_lock(&hub->state_lock);
	send_disable = sensor->service_enabled && hub->connected;
	mutex_unlock(&hub->state_lock);
	if (send_disable) {
		ret = qsh_send_sensor_request(sensor, false);
		if (ret == -ENOTCONN)
			ret = 0;
	}
	if (!ret) {
		mutex_lock(&hub->state_lock);
		sensor->buffer_enabled = false;
		sensor->service_enabled = false;
		mutex_unlock(&hub->state_lock);
	}
	mutex_unlock(&sensor->op_lock);
	return ret;
}

static const struct iio_buffer_setup_ops qsh_buffer_ops = {
	.postenable = qsh_buffer_postenable,
	.predisable = qsh_buffer_predisable,
};

static int qsh_read_sample(struct qsh_sensor *sensor, unsigned int channel,
			   int *value)
{
	struct qsh_hub *hub = sensor->hub;
	unsigned long flags;
	bool buffer_enabled;
	bool send_disable = false;
	s64 now;
	int ret;

	now = iio_get_time_ns(sensor->indio_dev);
	mutex_lock(&hub->state_lock);
	buffer_enabled = sensor->buffer_enabled;
	mutex_unlock(&hub->state_lock);
	spin_lock_irqsave(&sensor->sample_lock, flags);
	if (sensor->sample_valid &&
	    (buffer_enabled ||
	     (now >= sensor->latest_timestamp_ns &&
	      now - sensor->latest_timestamp_ns < 250 * NSEC_PER_MSEC))) {
		*value = sensor->latest[channel];
		spin_unlock_irqrestore(&sensor->sample_lock, flags);
		return IIO_VAL_INT;
	}
	spin_unlock_irqrestore(&sensor->sample_lock, flags);

	mutex_lock(&sensor->op_lock);
	mutex_lock(&hub->state_lock);
	buffer_enabled = sensor->buffer_enabled;
	if (!buffer_enabled && !sensor->direct_enabled) {
		sensor->direct_enabled = true;
		sensor->service_enabled = true;
	} else if (!buffer_enabled) {
		buffer_enabled = true;
	}
	mutex_unlock(&hub->state_lock);
	if (buffer_enabled) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	spin_lock_irqsave(&sensor->sample_lock, flags);
	sensor->sample_valid = false;
	spin_unlock_irqrestore(&sensor->sample_lock, flags);
	reinit_completion(&sensor->first_sample);
	ret = qsh_send_sensor_request(sensor, true);
	if (ret) {
		mutex_lock(&hub->state_lock);
		sensor->direct_enabled = false;
		sensor->service_enabled = false;
		mutex_unlock(&hub->state_lock);
		goto out_unlock;
	}
	send_disable = true;
	ret = wait_for_completion_interruptible_timeout(&sensor->first_sample,
							QSH_DIRECT_TIMEOUT);
	if (ret == 0)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = 0;
	if (!ret) {
		spin_lock_irqsave(&sensor->sample_lock, flags);
		if (sensor->sample_valid)
			*value = sensor->latest[channel];
		else
			ret = -ENODATA;
		spin_unlock_irqrestore(&sensor->sample_lock, flags);
	}
	if (send_disable) {
		int disable_ret = qsh_send_sensor_request(sensor, false);

		if (disable_ret >= 0 || disable_ret == -ENOTCONN) {
			mutex_lock(&hub->state_lock);
			sensor->direct_enabled = false;
			sensor->service_enabled = false;
			mutex_unlock(&hub->state_lock);
		} else {
			ret = disable_ret;
			mod_delayed_work(system_wq, &hub->service_work,
					 QSH_RESTORE_RETRY);
		}
	}
out_unlock:
	mutex_unlock(&sensor->op_lock);
	return ret ?: IIO_VAL_INT;
}

static int qsh_read_raw(struct iio_dev *indio_dev,
			const struct iio_chan_spec *channel,
			int *value, int *value2, long mask)
{
	struct qsh_sensor *sensor = iio_priv(indio_dev);
	u32 frequency;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (channel->scan_index < 0 ||
		    channel->scan_index >= sensor->channel_count)
			return -EINVAL;
		return qsh_read_sample(sensor, channel->scan_index, value);
	case IIO_CHAN_INFO_SCALE:
		*value = 0;
		if (sensor->scale_denominator == QSH_SCALE_MICRO) {
			*value2 = 1;
			return IIO_VAL_INT_PLUS_MICRO;
		}
		if (sensor->scale_denominator == QSH_SCALE_MILLI) {
			*value2 = 1000;
			return IIO_VAL_INT_PLUS_MICRO;
		}
		*value = 1;
		*value2 = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (sensor->on_change)
			return -EINVAL;
		mutex_lock(&sensor->op_lock);
		frequency = sensor->sample_rate_micro_hz;
		mutex_unlock(&sensor->op_lock);
		*value = frequency / QSH_SCALE_MICRO;
		*value2 = frequency % QSH_SCALE_MICRO;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_FREQUENCY: {
		int ret;

		if (sensor != sensor->hub->sensors[QSH_FLICKER])
			return -EINVAL;
		ret = qsh_read_sample(sensor, 0, value);
		if (ret < 0)
			return ret;
		if (*value < 0)
			return -ERANGE;
		*value2 = (*value % QSH_SCALE_MILLI) * QSH_SCALE_MILLI;
		*value /= QSH_SCALE_MILLI;
		return IIO_VAL_INT_PLUS_MICRO;
	}
	default:
		return -EINVAL;
	}
}

static int qsh_write_raw(struct iio_dev *indio_dev,
			 const struct iio_chan_spec *channel,
			 int value, int value2, long mask)
{
	struct qsh_sensor *sensor = iio_priv(indio_dev);
	struct qsh_hub *hub = sensor->hub;
	u64 frequency;
	int ret = 0;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ || sensor->on_change)
		return -EINVAL;
	if (value < 0 || value2 < 0 || value2 >= QSH_SCALE_MICRO)
		return -EINVAL;
	frequency = (u64)value * QSH_SCALE_MICRO + value2;

	mutex_lock(&sensor->op_lock);
	mutex_lock(&hub->state_lock);
	if (!sensor->rates_initialized) {
		ret = -EAGAIN;
	} else if (sensor->buffer_enabled || sensor->direct_enabled) {
		ret = -EBUSY;
	} else {
		unsigned int index;
		bool supported = false;

		for (index = 0; index < sensor->rate_count; index++)
			supported |= sensor->rates[index * 2] == value &&
				     sensor->rates[index * 2 + 1] == value2;
		if (supported)
			sensor->sample_rate_micro_hz = frequency;
		else
			ret = -EINVAL;
	}
	mutex_unlock(&hub->state_lock);
	mutex_unlock(&sensor->op_lock);
	return ret;
}

static int qsh_read_avail(struct iio_dev *indio_dev,
			  const struct iio_chan_spec *channel,
			  const int **values, int *type, int *length,
			  long mask)
{
	struct qsh_sensor *sensor = iio_priv(indio_dev);
	struct qsh_hub *hub = sensor->hub;
	int ret = 0;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ || sensor->on_change)
		return -EINVAL;

	mutex_lock(&hub->state_lock);
	if (!sensor->rates_initialized) {
		ret = -EAGAIN;
	} else {
		*values = sensor->rates;
		*type = IIO_VAL_INT_PLUS_MICRO;
		*length = sensor->rate_count * 2;
		ret = IIO_AVAIL_LIST;
	}
	mutex_unlock(&hub->state_lock);

	return ret;
}

static const struct iio_info qsh_iio_info = {
	.read_raw = qsh_read_raw,
	.write_raw = qsh_write_raw,
	.read_avail = qsh_read_avail,
};

static int qsh_register_sensor(struct qsh_hub *hub, unsigned int index)
{
	const struct qsh_sensor_template *template = &qsh_piano_sensors[index];
	struct iio_dev *indio_dev;
	struct qsh_sensor *sensor;
	int ret;

	indio_dev = devm_iio_device_alloc(hub->dev, sizeof(*sensor));
	if (!indio_dev)
		return -ENOMEM;
	sensor = iio_priv(indio_dev);
	sensor->hub = hub;
	sensor->indio_dev = indio_dev;
	sensor->name = template->name;
	sensor->datatype = template->datatype;
	sensor->sample_rate_micro_hz = template->sample_rate_micro_hz;
	sensor->fixed_multiplier = template->fixed_multiplier;
	sensor->scale_denominator = template->scale_denominator;
	sensor->channel_count = template->channel_count;
	sensor->on_change = template->on_change;
	sensor->buffered = template->buffered;
	mutex_init(&sensor->op_lock);
	init_completion(&sensor->first_sample);
	spin_lock_init(&sensor->sample_lock);

	indio_dev->name = sensor->name;
	indio_dev->info = &qsh_iio_info;
	indio_dev->channels = template->channels;
	indio_dev->num_channels = template->buffered ?
		template->channel_count + 1 : 1;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (template->buffered) {
		sensor->scan_masks[0] = GENMASK(sensor->channel_count - 1, 0);
		indio_dev->available_scan_masks = sensor->scan_masks;
		ret = devm_iio_kfifo_buffer_setup(hub->dev, indio_dev,
						  &qsh_buffer_ops);
		if (ret)
			return ret;
	}

	hub->sensors[index] = sensor;
	ret = devm_iio_device_register(hub->dev, indio_dev);
	if (ret) {
		hub->sensors[index] = NULL;
		return ret;
	}

	return 0;
}

static int qsh_start(struct device *dev)
{
	struct qsh_hub *hub;
	unsigned int index;
	int ret;

	hub = devm_kzalloc(dev, sizeof(*hub), GFP_KERNEL);
	if (!hub)
		return -ENOMEM;
	hub->dev = dev;
	mutex_init(&hub->state_lock);
	mutex_init(&hub->request_lock);
	spin_lock_init(&hub->timestamp_lock);
	INIT_DELAYED_WORK(&hub->service_work, qsh_service_work);
	atomic_set(&hub->malformed_events, 0);
	dev_set_drvdata(dev, hub);

	for (index = 0; index < QSH_SENSOR_COUNT; index++) {
		ret = qsh_register_sensor(hub, index);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to register sensor %u\n", index);
	}

	ret = qmi_handle_init(&hub->qmi, QSH_QMI_RX_MAX, &qsh_qmi_ops,
			      qsh_qmi_handlers);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to initialize QMI client\n");

	ret = qmi_add_lookup(&hub->qmi, QSH_QMI_SERVICE, QSH_QMI_VERSION,
			     QSH_QMI_INSTANCE);
	if (ret) {
		qmi_handle_release(&hub->qmi);
		return dev_err_probe(dev, ret,
				     "failed to look up QSH service\n");
	}

	dev_info(dev, "registered %u QSH-backed IIO sensors\n",
		 QSH_SENSOR_COUNT);
	return 0;
}

static void qsh_stop(struct qsh_hub *hub)
{
	unsigned int index;

	mutex_lock(&hub->state_lock);
	hub->removing = true;
	mutex_unlock(&hub->state_lock);
	cancel_delayed_work_sync(&hub->service_work);

	for (index = 0; index < QSH_SENSOR_COUNT; index++) {
		struct qsh_sensor *sensor = hub->sensors[index];
		bool disable;

		if (!sensor)
			continue;
		mutex_lock(&sensor->op_lock);
		mutex_lock(&hub->state_lock);
		disable = sensor->service_enabled && hub->connected;
		sensor->buffer_enabled = false;
		sensor->service_enabled = false;
		mutex_unlock(&hub->state_lock);
		if (disable)
			qsh_send_sensor_request(sensor, false);
		mutex_unlock(&sensor->op_lock);
	}

	qmi_handle_release(&hub->qmi);
}

static struct device *qsh_root_device;

static int __init qsh_init(void)
{
	int ret;

	/* The datatype-to-IIO topology below is currently Piano-specific. */
	if (!of_machine_is_compatible("xiaomi,piano"))
		return 0;

	qsh_root_device = root_device_register("qcom-sensing-hub");
	if (IS_ERR(qsh_root_device))
		return PTR_ERR(qsh_root_device);

	ret = qsh_start(qsh_root_device);
	if (ret) {
		root_device_unregister(qsh_root_device);
		qsh_root_device = NULL;
	}

	return ret;
}
module_init(qsh_init);

static void __exit qsh_exit(void)
{
	if (!qsh_root_device)
		return;

	qsh_stop(dev_get_drvdata(qsh_root_device));
	root_device_unregister(qsh_root_device);
}
module_exit(qsh_exit);

MODULE_AUTHOR("BigfootACA <bigfoot@classfun.cn>");
MODULE_DESCRIPTION("Qualcomm Sensing Hub IIO bridge");
MODULE_LICENSE("GPL");
