// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Junhao Xie <bigfoot@classfun.cn>
 */

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/crc16.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/mfd/photonicat-pmu.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>

#define PCAT_ADDR_CPU		0x01
#define PCAT_ADDR_PMU		0x81
#define PCAT_ADDR_CPU_ALL	0x80
#define PCAT_ADDR_PMU_ALL	0xFE
#define PCAT_ADDR_ALL		0xFF

#define PCAT_MAGIC_HEAD	0xA5
#define PCAT_MAGIC_END	0x5A

struct pcat_request {
	struct pcat_pmu *pmu;
	u16 frame_id;
	struct completion received;
	struct pcat_request_request {
		u16 cmd;
		u16 want;
		const void *data;
		size_t size;
	} request;
	struct pcat_request_reply {
		struct pcat_data_head head;
		struct pcat_data_foot foot;
		void *data;
		size_t size;
	} reply;
};

struct pcat_pmu {
	struct device *dev;
	struct serdev_device *serdev;
	atomic_t frame;
	char buffer[8192];
	size_t length;
	struct pcat_request *reply;
	spinlock_t bus_lock;
	struct mutex reply_lock;
	struct mutex status_lock;
	struct completion first_status;
	struct blocking_notifier_head notifier_list;
};

static int pcat_pmu_raw_write(struct pcat_pmu *pmu, u16 frame_id,
			      enum pcat_pmu_cmd cmd, bool need_ack,
			      const void *data, size_t len)
{
	int ret;
	struct pcat_data_head head;
	struct pcat_data_foot foot;

	head.magic_head = PCAT_MAGIC_HEAD;
	head.source = PCAT_ADDR_CPU;
	head.dest = PCAT_ADDR_PMU;
	head.frame_id = frame_id;
	head.length = len + sizeof(struct pcat_data_foot) - 1;
	head.command = cmd;

	ret = serdev_device_write_buf(pmu->serdev, (u8 *)&head, sizeof(head));
	if (ret < 0) {
		dev_err(pmu->dev, "failed to write frame head: %d", ret);
		return ret;
	}

	ret = serdev_device_write_buf(pmu->serdev, data, len);
	if (ret < 0) {
		dev_err(pmu->dev, "failed to write frame body: %d", ret);
		return ret;
	}

	foot.need_ack = need_ack;
	foot.crc16 = 0;
	foot.magic_end = PCAT_MAGIC_END;
	foot.crc16 = crc16(0xFFFF, (u8 *)&head + 1, sizeof(head) - 1);
	foot.crc16 = crc16(foot.crc16, data, len);
	foot.crc16 = crc16(foot.crc16, (u8 *)&foot, 1);

	ret = serdev_device_write_buf(pmu->serdev, (u8 *)&foot, sizeof(foot));
	if (ret < 0)
		dev_err(pmu->dev, "failed to send frame foot: %d", ret);

	return ret;
}

/**
 * pcat_pmu_send - send a frame to PMU
 * @pmu: Photonicat PMU structure for the device we are communicating with
 * @cmd: command id
 * @data: frame payload we are sending
 * @len: actual size of @data
 *
 * Function returns 0 on success and negative error code on failure
 */
int pcat_pmu_send(struct pcat_pmu *pmu, enum pcat_pmu_cmd cmd,
		  const void *data, size_t len)
{
	u16 frame_id = atomic_inc_return(&pmu->frame);

	return pcat_pmu_raw_write(pmu, frame_id, cmd, false, data, len);
}
EXPORT_SYMBOL_GPL(pcat_pmu_send);

/**
 * pcat_pmu_execute - send a frame to PMU and wait for response
 * @request: the request structure for send frame and response
 *
 * Function returns 0 on success and negative error code on failure
 */
int pcat_pmu_execute(struct pcat_request *request)
{
	int ret = 0, retries = 0;
	unsigned long flags;
	struct pcat_pmu *pmu = request->pmu;
	struct pcat_request_request *req = &request->request;
	struct pcat_request_reply *reply = &request->reply;

	init_completion(&request->received);
	memset(reply, 0, sizeof(request->reply));

	mutex_lock(&pmu->reply_lock);
	if (request->frame_id == 0)
		request->frame_id = atomic_inc_return(&pmu->frame);
	pmu->reply = request;
	mutex_unlock(&pmu->reply_lock);

	if (req->want == 0)
		req->want = req->cmd + 1;

	dev_dbg(pmu->dev, "frame 0x%04X execute cmd 0x%02X\n",
		request->frame_id, req->cmd);

	while (1) {
		spin_lock_irqsave(&pmu->bus_lock, flags);
		ret = pcat_pmu_raw_write(pmu, request->frame_id, req->cmd,
					 true, req->data, req->size);
		spin_unlock_irqrestore(&pmu->bus_lock, flags);
		if (ret < 0) {
			dev_err(pmu->dev,
				"frame 0x%04X write 0x%02X cmd failed: %d\n",
				request->frame_id, req->cmd, ret);
			goto fail;
		}
		dev_dbg(pmu->dev, "frame 0x%04X waiting response for 0x%02X\n",
			request->frame_id, req->cmd);
		if (!wait_for_completion_timeout(&request->received, HZ)) {
			if (retries < 3) {
				retries++;
				continue;
			} else {
				dev_warn(pmu->dev,
					 "frame 0x%04X cmd 0x%02X timeout\n",
					 request->frame_id, req->cmd);
				ret = -ETIMEDOUT;
				goto fail;
			}
		}
		break;
	}
	dev_dbg(pmu->dev, "frame 0x%04X got response 0x%02X\n",
		request->frame_id, reply->head.command);

	return 0;
fail:
	mutex_lock(&pmu->reply_lock);
	pmu->reply = NULL;
	mutex_unlock(&pmu->reply_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(pcat_pmu_execute);

/**
 * pcat_pmu_write_data - send data to PMU and wait for ack
 * @pmu: Photonicat PMU structure for the device we are communicating with
 * @cmd: command id
 * @data: frame payload we are sending
 * @len: actual size of @data
 *
 * Function returns 0 on success and negative error code on failure
 */
int pcat_pmu_write_data(struct pcat_pmu *pmu, enum pcat_pmu_cmd cmd,
			const void *data, size_t size)
{
	int ret;
	struct pcat_request request = {
		.pmu = pmu,
		.request.cmd = cmd,
		.request.data = data,
		.request.size = size,
	};
	ret = pcat_pmu_execute(&request);
	if (request.reply.data)
		devm_kfree(pmu->dev, request.reply.data);
	return ret;
}
EXPORT_SYMBOL_GPL(pcat_pmu_write_data);

/**
 * pcat_pmu_read_string - send string read request to PMU and wait for response
 * @pmu: Photonicat PMU structure for the device we are communicating with
 * @cmd: command id
 * @str: buffer for received string
 * @len: buffer size of @str
 *
 * Function returns 0 on success and negative error code on failure
 */
int pcat_pmu_read_string(struct pcat_pmu *pmu, enum pcat_pmu_cmd cmd,
			 char *str, size_t len)
{
	int ret;
	struct pcat_request request = {
		.pmu = pmu,
		.request.cmd = cmd,
	};
	memset(str, 0, len);
	ret = pcat_pmu_execute(&request);
	if (request.reply.data) {
		memcpy(str, request.reply.data,
		       min(len - 1, request.reply.size));
		devm_kfree(pmu->dev, request.reply.data);
	};
	return ret;
}
EXPORT_SYMBOL_GPL(pcat_pmu_read_string);

/**
 * pcat_pmu_write_u8 - send unsigned 8-bit number to PMU and wait for ack
 * @pmu: Photonicat PMU structure for the device we are communicating with
 * @cmd: command id
 * @v: the unsigned 8-bit number to send
 *
 * Function returns 0 on success and negative error code on failure
 */
int pcat_pmu_write_u8(struct pcat_pmu *pmu, enum pcat_pmu_cmd cmd, u8 v)
{
	return pcat_pmu_write_data(pmu, cmd, &v, sizeof(v));
}
EXPORT_SYMBOL_GPL(pcat_pmu_write_u8);

static bool pcat_process_reply(struct pcat_data *p)
{
	bool processed = false;
	struct pcat_pmu *pmu = p->pmu;
	struct device *dev = pmu->dev;
	struct pcat_request *request;
	struct pcat_request_request *req;
	struct pcat_request_reply *reply;

	mutex_lock(&pmu->reply_lock);
	request = pmu->reply;
	if (!request)
		goto skip;

	req = &request->request;
	reply = &request->reply;
	if (request->frame_id != p->head->frame_id) {
		dev_dbg_ratelimited(dev, "skip mismatch frame %04X != %04X",
				    request->frame_id, p->head->frame_id);
		goto skip;
	}

	if (req->want == 0)
		req->want = req->cmd + 1;

	if (req->want != p->head->command) {
		dev_dbg_ratelimited(
			dev, "frame %04X skip mismatch command %02X != %02X",
			request->frame_id, req->want, p->head->command);
		goto skip;
	}

	if (completion_done(&request->received)) {
		dev_dbg_ratelimited(dev, "frame %04X skip done completion",
				    request->frame_id);
		goto skip;
	}

	memcpy(&reply->head, p->head, sizeof(struct pcat_data_head));
	memcpy(&reply->foot, p->foot, sizeof(struct pcat_data_foot));
	if (p->data && p->size > 0) {
		reply->data = devm_kzalloc(pmu->dev, p->size + 1, GFP_KERNEL);
		if (pmu->reply->reply.data) {
			memcpy(reply->data, p->data, p->size);
			reply->size = p->size;
		}
	}

	complete(&request->received);
	pmu->reply = NULL;
	processed = true;

skip:
	mutex_unlock(&pmu->reply_lock);
	return processed;
}

static int pcat_process_data(struct pcat_pmu *pmu, const u8 *data, size_t len)
{
	int ret;
	u16 crc16_calc;
	size_t data_size = 0;
	struct pcat_data frame;

	memset(&frame, 0, sizeof(frame));
	frame.pmu = pmu;
	if (len < sizeof(struct pcat_data_head)) {
		dev_dbg_ratelimited(pmu->dev, "head too small %zu < %zu\n", len,
				    sizeof(struct pcat_data_head));
		return -EAGAIN;
	}

	frame.head = (struct pcat_data_head *)data;
	if (frame.head->magic_head != PCAT_MAGIC_HEAD) {
		dev_dbg_ratelimited(pmu->dev, "bad head magic %02X\n",
				    frame.head->magic_head);
		return -EBADMSG;
	}

	if (frame.head->source != PCAT_ADDR_PMU) {
		dev_dbg_ratelimited(pmu->dev, "unknown data source %02X\n",
				    frame.head->source);
		return 0;
	}
	if (frame.head->dest != PCAT_ADDR_CPU &&
	    frame.head->dest != PCAT_ADDR_CPU_ALL &&
	    frame.head->dest != PCAT_ADDR_ALL) {
		dev_dbg_ratelimited(pmu->dev, "not data destination %02X\n",
				    frame.head->dest);
		return 0;
	}
	if (frame.head->length < sizeof(struct pcat_data_foot) - 1 ||
	    frame.head->length >= U16_MAX - 4) {
		dev_dbg_ratelimited(pmu->dev, "invalid length %d\n",
				    frame.head->length);
		return -EBADMSG;
	}
	data_size = sizeof(struct pcat_data_head) + frame.head->length + 1;
	if (data_size > len) {
		dev_dbg_ratelimited(pmu->dev, "data too small %zu > %zu\n",
				    data_size, len);
		return -EAGAIN;
	}
	frame.data = (u8 *)data + sizeof(struct pcat_data_head);
	frame.size = frame.head->length + 1 - sizeof(struct pcat_data_foot);
	frame.foot = (struct pcat_data_foot *)(data + frame.size +
					       sizeof(struct pcat_data_head));
	if (frame.foot->magic_end != PCAT_MAGIC_END) {
		dev_dbg_ratelimited(pmu->dev, "bad foot magic %02X\n",
				    frame.foot->magic_end);
		return -EBADMSG;
	}
	crc16_calc = crc16(0xFFFF, data + 1, frame.head->length + 6);
	if (frame.foot->crc16 != crc16_calc) {
		dev_warn_ratelimited(pmu->dev, "crc16 mismatch %04X != %04X\n",
				     frame.foot->crc16, crc16_calc);
		return -EBADMSG;
	}

	if (pcat_process_reply(&frame))
		return 0;

	ret = blocking_notifier_call_chain(&pmu->notifier_list,
					   frame.head->command, &frame);
	if (ret == NOTIFY_DONE && frame.foot->need_ack) {
		pcat_pmu_raw_write(pmu, frame.head->frame_id,
				   frame.head->command + 1, false, NULL, 0);
	}

	return 0;
}

static size_t pcat_pmu_receive_buf(struct serdev_device *serdev,
				   const unsigned char *buf, size_t size)
{
	struct device *dev = &serdev->dev;
	struct pcat_pmu *pmu = dev_get_drvdata(dev);
	size_t processed = size;
	int ret;
	size_t new_len = pmu->length + size;

	if (!pmu || !buf || size <= 0)
		return 0;
	if (new_len > sizeof(pmu->buffer)) {
		new_len = sizeof(pmu->buffer);
		processed = new_len - pmu->length;
	}
	if (pmu->length)
		dev_dbg(pmu->dev, "got remaining message at %zu size %zu (%zu)",
			pmu->length, processed, new_len);
	memcpy(pmu->buffer + pmu->length, buf, processed);
	pmu->length = new_len;
	ret = pcat_process_data(pmu, pmu->buffer, pmu->length);
	if (ret != -EAGAIN)
		pmu->length = 0;
	else
		dev_dbg(pmu->dev, "got partial message %zu", pmu->length);
	return processed;
}

static const struct serdev_device_ops pcat_pmu_serdev_device_ops = {
	.receive_buf = pcat_pmu_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

int pcat_pmu_register_notify(struct pcat_pmu *pmu, struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pmu->notifier_list, nb);
}
EXPORT_SYMBOL_GPL(pcat_pmu_register_notify);

void pcat_pmu_unregister_notify(struct pcat_pmu *pmu, struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&pmu->notifier_list, nb);
}
EXPORT_SYMBOL_GPL(pcat_pmu_unregister_notify);

static const struct mfd_cell photonicat_pmu_cells[] = {
	{ .name = "photonicat-hwmon", },
	{ .name = "photonicat-leds", },
	{ .name = "photonicat-poweroff", },
	{ .name = "photonicat-rtc", },
	{ .name = "photonicat-supply", },
	{ .name = "photonicat-watchdog", },
};

static int pcat_pmu_probe(struct serdev_device *serdev)
{
	int ret;
	u32 baudrate;
	char buffer[64];
	struct pcat_pmu *pmu = NULL;
	struct device *dev = &serdev->dev;

	pmu = devm_kzalloc(dev, sizeof(*pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;

	pmu->dev = dev;
	pmu->serdev = serdev;
	spin_lock_init(&pmu->bus_lock);
	mutex_init(&pmu->reply_lock);
	init_completion(&pmu->first_status);

	if (of_property_read_u32(dev->of_node, "current-speed", &baudrate))
		baudrate = 115200;

	serdev_device_set_client_ops(serdev, &pcat_pmu_serdev_device_ops);
	ret = devm_serdev_device_open(dev, serdev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to open serdev\n");

	serdev_device_set_baudrate(serdev, baudrate);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	dev_set_drvdata(dev, pmu);

	/* Disable watchdog on boot */
	pcat_pmu_write_data(pmu, PCAT_CMD_WATCHDOG_TIMEOUT_SET,
			    &(struct pcat_data_cmd_watchdog){ 60, 60, 0 }, 3);

	/* Read hardware version */
	pcat_pmu_read_string(pmu, PCAT_CMD_PMU_HW_VERSION_GET,
			     buffer, sizeof(buffer));
	if (buffer[0])
		dev_dbg(dev, "PMU Hardware version: %s\n", buffer);

	/* Read firmware version */
	pcat_pmu_read_string(pmu, PCAT_CMD_PMU_FW_VERSION_GET,
			     buffer, sizeof(buffer));
	if (buffer[0])
		dev_dbg(dev, "PMU Firmware version: %s\n", buffer);

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
				    photonicat_pmu_cells,
				    ARRAY_SIZE(photonicat_pmu_cells),
				    NULL, 0, NULL);
}

static const struct of_device_id pcat_pmu_dt_ids[] = {
	{ .compatible = "ariaboard,photonicat-pmu", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pcat_pmu_dt_ids);

static struct serdev_device_driver pcat_pmu_driver = {
	.driver = {
		.name = "photonicat-pmu",
		.of_match_table = pcat_pmu_dt_ids,
	},
	.probe = pcat_pmu_probe,
};
module_serdev_device_driver(pcat_pmu_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@classfun.cn>");
MODULE_DESCRIPTION("Photonicat Power Management Unit");
MODULE_LICENSE("GPL");
