/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024 Junhao Xie <bigfoot@classfun.cn>
 */

#ifndef MFD_PHOTONICAT_PMU_H
#define MFD_PHOTONICAT_PMU_H

#include <linux/notifier.h>
#include <linux/types.h>

struct pcat_pmu;
struct pcat_request;

struct pcat_data_cmd_date_time {
	u16 year;
	u8 month;
	u8 day;
	u8 hour;
	u8 minute;
	u8 second;
} __packed;

struct pcat_data_cmd_led_setup {
	u16 on_time;
	u16 down_time;
	u16 repeat;
} __packed;

struct pcat_data_cmd_startup_time {
	struct pcat_data_cmd_date_time time;
	union {
		u8 value;
		struct {
			bool year : 1;
			bool month : 1;
			bool day : 1;
			bool hour : 1;
			bool minute : 1;
			bool second : 1;
		} __packed;
	} match;
} __packed;

struct pcat_data_cmd_status {
	u16 battery_microvolt;
	u16 charger_microvolt;
	u16 gpio_input;
	u16 gpio_output;
	struct pcat_data_cmd_date_time time;
	u16 reserved;
	u8 temp;
} __packed;

struct pcat_data_cmd_watchdog {
	u8 startup_timeout;
	u8 shutdown_timeout;
	u8 running_timeout;
} __packed;

struct pcat_data_foot {
	u8 need_ack;
	u16 crc16;
	u8 magic_end;
} __packed;

struct pcat_data_head {
	u8 magic_head;
	u8 source;
	u8 dest;
	u16 frame_id;
	u16 length;
	u16 command;
} __packed;

struct pcat_data {
	struct pcat_pmu *pmu;
	struct pcat_data_head *head;
	struct pcat_data_foot *foot;
	void *data;
	size_t size;
};

enum pcat_boot_reason {
	PCAT_BOOT_BUTTON	= 0x00,
	PCAT_BOOT_ALARM		= 0x01,
	PCAT_BOOT_CAR_MODE	= 0x02,
	PCAT_BOOT_LOW_CHARGE	= 0x03,
	PCAT_BOOT_NO_BATTERY	= 0x03,
};

enum pcat_shutdown_reason {
	PCAT_SHUTDOWN_BUTTON	= 0x00,
	PCAT_SHUTDOWN_POWER_LOW	= 0x01,
	PCAT_SHUTDOWN_UPGRADE	= 0x02,
	PCAT_SHUTDOWN_OTHER	= 0x03,
};

enum pcat_pmu_cmd {
	PCAT_CMD_HEARTBEAT			= 0x01,
	PCAT_CMD_HEARTBEAT_ACK			= 0x02,
	PCAT_CMD_PMU_HW_VERSION_GET		= 0x03,
	PCAT_CMD_PMU_HW_VERSION_GET_ACK		= 0x04,
	PCAT_CMD_PMU_FW_VERSION_GET		= 0x05,
	PCAT_CMD_PMU_FW_VERSION_GET_ACK		= 0x06,
	PCAT_CMD_STATUS_REPORT			= 0x07,
	PCAT_CMD_STATUS_REPORT_ACK		= 0x08,
	PCAT_CMD_DATE_TIME_SYNC			= 0x09,
	PCAT_CMD_DATE_TIME_SYNC_ACK		= 0x0A,
	PCAT_CMD_SCHEDULE_STARTUP_TIME_SET	= 0x0B,
	PCAT_CMD_SCHEDULE_STARTUP_TIME_SET_ACK	= 0x0C,
	PCAT_CMD_PMU_REQUEST_SHUTDOWN		= 0x0D,
	PCAT_CMD_PMU_REQUEST_SHUTDOWN_ACK	= 0x0E,
	PCAT_CMD_HOST_REQUEST_SHUTDOWN		= 0x0F,
	PCAT_CMD_HOST_REQUEST_SHUTDOWN_ACK	= 0x10,
	PCAT_CMD_PMU_REQUEST_FACTORY_RESET	= 0x11,
	PCAT_CMD_PMU_REQUEST_FACTORY_RESET_ACK	= 0x12,
	PCAT_CMD_WATCHDOG_TIMEOUT_SET		= 0x13,
	PCAT_CMD_WATCHDOG_TIMEOUT_SET_ACK	= 0x14,
	PCAT_CMD_CHARGER_ON_AUTO_START		= 0x15,
	PCAT_CMD_CHARGER_ON_AUTO_START_ACK	= 0x16,
	PCAT_CMD_VOLTAGE_THRESHOLD_SET		= 0x17,
	PCAT_CMD_VOLTAGE_THRESHOLD_SET_ACK	= 0x18,
	PCAT_CMD_NET_STATUS_LED_SETUP		= 0x19,
	PCAT_CMD_NET_STATUS_LED_SETUP_ACK	= 0x1A,
	PCAT_CMD_POWER_ON_EVENT_GET		= 0x1B,
	PCAT_CMD_POWER_ON_EVENT_GET_ACK		= 0x1C,
};

int pcat_pmu_send(struct pcat_pmu *pmu, enum pcat_pmu_cmd cmd,
		  const void *data, size_t len);
int pcat_pmu_execute(struct pcat_request *request);
int pcat_pmu_write_data(struct pcat_pmu *pmu, enum pcat_pmu_cmd cmd,
			const void *data, size_t size);
int pcat_pmu_read_string(struct pcat_pmu *pmu, enum pcat_pmu_cmd cmd,
			 char *str, size_t len);
int pcat_pmu_write_u8(struct pcat_pmu *pmu, enum pcat_pmu_cmd cmd, u8 value);
int pcat_pmu_register_notify(struct pcat_pmu *pmu,
			     struct notifier_block *nb);
void pcat_pmu_unregister_notify(struct pcat_pmu *pmu,
				struct notifier_block *nb);

#endif
