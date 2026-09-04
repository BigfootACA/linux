// SPDX-License-Identifier: GPL-2.0-only
/*
 * HLOS control shim for TrustZone-managed Xiaomi side fingerprint sensors.
 *
 * Sensor transactions are owned by secure firmware. This driver only owns
 * the non-secure power, reset and interrupt signals and retains the small
 * part of Xiaomi's userspace ABI needed to control them.
 */

#include <linux/atomic.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netlink.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/poll.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <net/net_namespace.h>
#include <net/sock.h>

#define XIAOMI_FP_NAME			"xiaomi-fp"
#define XIAOMI_FP_ID_NAME		"mifp_id"
#define XIAOMI_FP_VENDOR_NAME_LEN	30
#define XIAOMI_FP_SUPPLY_NAME_LEN	10
#define XIAOMI_FP_NETLINK_PROTOCOL	25
#define XIAOMI_FP_NETLINK_MSG_LEN	16
#define XIAOMI_FP_NETLINK_IRQ		1
#define XIAOMI_FP_WAKE_MS		2000
#define XIAOMI_FP_RESET_MS		10
#define XIAOMI_FP_RESET_MAX_MS		1000

/* Existing Xiaomi mi_fp ABI. Keep these command encodings unchanged. */
#define XIAOMI_FP_IOC_MAGIC		'g'
#define XIAOMI_FP_IOC_INIT		_IOR(XIAOMI_FP_IOC_MAGIC, 0, u8)
#define XIAOMI_FP_IOC_EXIT		_IO(XIAOMI_FP_IOC_MAGIC, 1)
#define XIAOMI_FP_IOC_RESET		_IO(XIAOMI_FP_IOC_MAGIC, 2)
#define XIAOMI_FP_IOC_ENABLE_IRQ		_IO(XIAOMI_FP_IOC_MAGIC, 3)
#define XIAOMI_FP_IOC_DISABLE_IRQ	_IO(XIAOMI_FP_IOC_MAGIC, 4)
#define XIAOMI_FP_IOC_ENABLE_SPI_CLK	_IOW(XIAOMI_FP_IOC_MAGIC, 5, u32)
#define XIAOMI_FP_IOC_DISABLE_SPI_CLK	_IO(XIAOMI_FP_IOC_MAGIC, 6)
#define XIAOMI_FP_IOC_ENABLE_POWER	_IOW(XIAOMI_FP_IOC_MAGIC, 7, char)
#define XIAOMI_FP_IOC_DISABLE_POWER	_IOW(XIAOMI_FP_IOC_MAGIC, 8, char)
#define XIAOMI_FP_IOC_ENTER_SLEEP	_IO(XIAOMI_FP_IOC_MAGIC, 10)
#define XIAOMI_FP_IOC_GET_FW_INFO	_IOR(XIAOMI_FP_IOC_MAGIC, 11, u8)
#define XIAOMI_FP_IOC_REMOVE		_IO(XIAOMI_FP_IOC_MAGIC, 12)
#define XIAOMI_FP_IOC_CHIP_INFO		_IOW(XIAOMI_FP_IOC_MAGIC, 13, \
					     struct xiaomi_fp_chip_info)
#define XIAOMI_FP_IOC_REQUEST_RESOURCE	_IO(XIAOMI_FP_IOC_MAGIC, 37)
#define XIAOMI_FP_IOC_RELEASE_RESOURCE	_IO(XIAOMI_FP_IOC_MAGIC, 38)
#define XIAOMI_FP_IOC_DEV_INFO		_IOR(XIAOMI_FP_IOC_MAGIC, 40, char)
#define XIAOMI_FP_IOC_RESET_OUT_LOW	_IO(XIAOMI_FP_IOC_MAGIC, 41)
#define XIAOMI_FP_IOC_RESET_TIME_MS	_IOW(XIAOMI_FP_IOC_MAGIC, 44, u32)

struct xiaomi_fp_chip_info {
	u8 vendor_id;
	u8 mode;
	u8 operation;
	u8 reserved[5];
};

struct xiaomi_fp {
	struct device *dev;
	struct gpio_desc *irq_gpio;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;
	struct miscdevice miscdev;
	struct miscdevice id_miscdev;
	/* Serialize HLOS resource state transitions. */
	struct mutex lock;
	wait_queue_head_t irq_wait;
	atomic64_t irq_sequence;
	atomic_t fingerdown;
	struct sock *netlink_sock;
	u32 netlink_portid;
	unsigned int reset_low_ms;
	unsigned int reset_high_ms;
	int irq;
	bool irq_requested;
	bool irq_enabled;
	bool powered;
	bool dead;
	char vendor_name[XIAOMI_FP_VENDOR_NAME_LEN];
};

struct xiaomi_fp_file {
	struct xiaomi_fp *fp;
	u64 irq_sequence;
	bool control;
};

/* Raw netlink does not provide a driver-private callback context. */
static DEFINE_MUTEX(xiaomi_fp_netlink_lock);
static struct xiaomi_fp *xiaomi_fp_netlink_owner;

static void xiaomi_fp_netlink_receive(struct sk_buff *skb)
{
	struct xiaomi_fp *fp = READ_ONCE(xiaomi_fp_netlink_owner);
	struct nlmsghdr *nlh;

	if (!fp || skb->len < nlmsg_total_size(0))
		return;

	nlh = nlmsg_hdr(skb);
	if (!nlmsg_ok(nlh, skb->len))
		return;

	WRITE_ONCE(fp->netlink_portid, NETLINK_CB(skb).portid);
}

static int xiaomi_fp_netlink_start(struct xiaomi_fp *fp)
{
	struct netlink_kernel_cfg cfg = {
		.input = xiaomi_fp_netlink_receive,
	};
	int ret = 0;

	mutex_lock(&xiaomi_fp_netlink_lock);
	if (xiaomi_fp_netlink_owner) {
		ret = -EBUSY;
		goto out_unlock;
	}

	fp->netlink_sock = netlink_kernel_create(&init_net,
						 XIAOMI_FP_NETLINK_PROTOCOL,
						 &cfg);
	if (!fp->netlink_sock) {
		ret = -EADDRINUSE;
		goto out_unlock;
	}

	WRITE_ONCE(xiaomi_fp_netlink_owner, fp);

out_unlock:
	mutex_unlock(&xiaomi_fp_netlink_lock);
	return ret;
}

static void xiaomi_fp_netlink_stop(struct xiaomi_fp *fp)
{
	mutex_lock(&xiaomi_fp_netlink_lock);
	if (xiaomi_fp_netlink_owner == fp) {
		netlink_kernel_release(fp->netlink_sock);
		fp->netlink_sock = NULL;
		WRITE_ONCE(xiaomi_fp_netlink_owner, NULL);
	}
	mutex_unlock(&xiaomi_fp_netlink_lock);
}

static void xiaomi_fp_netlink_send_irq(struct xiaomi_fp *fp)
{
	struct nlmsghdr *nlh;
	struct sk_buff *skb;
	u32 portid;

	portid = READ_ONCE(fp->netlink_portid);
	if (!fp->netlink_sock || !portid)
		return;

	skb = nlmsg_new(XIAOMI_FP_NETLINK_MSG_LEN, GFP_ATOMIC);
	if (!skb)
		return;

	nlh = nlmsg_put(skb, 0, 0, 0, XIAOMI_FP_NETLINK_MSG_LEN, 0);
	if (!nlh) {
		kfree_skb(skb);
		return;
	}

	memset(nlmsg_data(nlh), 0, XIAOMI_FP_NETLINK_MSG_LEN);
	*(u8 *)nlmsg_data(nlh) = XIAOMI_FP_NETLINK_IRQ;
	netlink_unicast(fp->netlink_sock, skb, portid, MSG_DONTWAIT);
}

static int xiaomi_fp_set_power_locked(struct xiaomi_fp *fp, bool enable)
{
	int ret;

	if (fp->powered == enable)
		return 0;

	/* The board regulator constraint owns the voltage selection. */
	if (enable)
		ret = regulator_enable(fp->vdd);
	else
		ret = regulator_disable(fp->vdd);
	if (ret)
		return ret;

	usleep_range(1000, 1100);
	fp->powered = enable;
	return 0;
}

static void xiaomi_fp_reset_locked(struct xiaomi_fp *fp,
				   unsigned int low_ms,
				   unsigned int high_ms)
{
	gpiod_set_value_cansleep(fp->reset_gpio, 1);
	msleep(low_ms);
	gpiod_set_value_cansleep(fp->reset_gpio, 0);
	msleep(high_ms);
}

static int xiaomi_fp_request_irq_locked(struct xiaomi_fp *fp)
{
	if (fp->irq_requested)
		return -EBUSY;

	fp->irq_requested = true;
	return 0;
}

static int xiaomi_fp_release_irq_locked(struct xiaomi_fp *fp)
{
	if (!fp->irq_requested)
		return -EINVAL;

	if (fp->irq_enabled)
		disable_irq(fp->irq);
	fp->irq_enabled = false;
	fp->irq_requested = false;
	return 0;
}

static int xiaomi_fp_set_irq_locked(struct xiaomi_fp *fp, bool enable)
{
	if (!fp->irq_requested)
		return -ENXIO;

	if (fp->irq_enabled == enable)
		return 0;

	if (enable)
		enable_irq(fp->irq);
	else
		disable_irq(fp->irq);
	fp->irq_enabled = enable;
	return 0;
}

static irqreturn_t xiaomi_fp_irq_thread(int irq, void *data)
{
	struct xiaomi_fp *fp = data;

	atomic64_inc(&fp->irq_sequence);
	wake_up_interruptible(&fp->irq_wait);
	sysfs_notify(&fp->dev->kobj, NULL, "irq");
	pm_wakeup_event(fp->dev, XIAOMI_FP_WAKE_MS);
	xiaomi_fp_netlink_send_irq(fp);

	return IRQ_HANDLED;
}

static int xiaomi_fp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct xiaomi_fp_file *ctx;
	struct xiaomi_fp *fp;

	fp = dev_get_drvdata(miscdev->parent);
	if (!fp || READ_ONCE(fp->dead))
		return -ENODEV;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;

	ctx->fp = fp;
	ctx->irq_sequence = atomic64_read(&fp->irq_sequence);
	ctx->control = miscdev == &fp->miscdev;
	file->private_data = ctx;

	return nonseekable_open(inode, file);
}

static int xiaomi_fp_release(struct inode *inode, struct file *file)
{
	struct xiaomi_fp_file *ctx = file->private_data;
	struct xiaomi_fp *fp = ctx->fp;

	if (ctx->control && !READ_ONCE(fp->dead)) {
		mutex_lock(&fp->lock);
		xiaomi_fp_release_irq_locked(fp);
		mutex_unlock(&fp->lock);
	}

	kfree(ctx);
	return 0;
}

static bool xiaomi_fp_irq_pending(struct xiaomi_fp_file *ctx)
{
	return atomic64_read(&ctx->fp->irq_sequence) != ctx->irq_sequence ||
	       READ_ONCE(ctx->fp->dead);
}

static ssize_t xiaomi_fp_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct xiaomi_fp_file *ctx = file->private_data;
	struct xiaomi_fp *fp = ctx->fp;
	u8 event = XIAOMI_FP_NETLINK_IRQ;
	u64 sequence;
	int ret;

	if (count < sizeof(event))
		return -EINVAL;

	sequence = atomic64_read(&fp->irq_sequence);
	if (sequence == ctx->irq_sequence) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(fp->irq_wait,
					       xiaomi_fp_irq_pending(ctx));
		if (ret)
			return ret;
	}

	if (READ_ONCE(fp->dead))
		return -ENODEV;

	if (copy_to_user(buf, &event, sizeof(event)))
		return -EFAULT;

	ctx->irq_sequence = atomic64_read(&fp->irq_sequence);
	return sizeof(event);
}

static __poll_t xiaomi_fp_poll(struct file *file, poll_table *wait)
{
	struct xiaomi_fp_file *ctx = file->private_data;
	struct xiaomi_fp *fp = ctx->fp;
	__poll_t mask = 0;

	poll_wait(file, &fp->irq_wait, wait);
	if (atomic64_read(&fp->irq_sequence) != ctx->irq_sequence)
		mask |= EPOLLIN | EPOLLRDNORM;
	if (READ_ONCE(fp->dead))
		mask |= EPOLLERR | EPOLLHUP;

	return mask;
}

static int xiaomi_fp_get_supply_name(unsigned long arg, char *name)
{
	if (copy_from_user(name, (void __user *)arg,
			   XIAOMI_FP_SUPPLY_NAME_LEN))
		return -EFAULT;

	name[XIAOMI_FP_SUPPLY_NAME_LEN - 1] = '\0';
	return 0;
}

static long xiaomi_fp_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct xiaomi_fp_file *ctx = file->private_data;
	struct xiaomi_fp *fp = ctx->fp;
	u32 reset_time[2];
	char supply[XIAOMI_FP_SUPPLY_NAME_LEN];
	u8 value;
	int ret = 0;

	if (_IOC_TYPE(cmd) != XIAOMI_FP_IOC_MAGIC)
		return -ENOTTY;
	if (READ_ONCE(fp->dead))
		return -ENODEV;

	switch (cmd) {
	case XIAOMI_FP_IOC_INIT:
		if (!fp->netlink_sock)
			return -EOPNOTSUPP;
		value = XIAOMI_FP_NETLINK_PROTOCOL;
		if (copy_to_user((void __user *)arg, &value, sizeof(value)))
			return -EFAULT;
		break;

	case XIAOMI_FP_IOC_EXIT:
		mutex_lock(&fp->lock);
		ret = xiaomi_fp_set_irq_locked(fp, false);
		if (ret == -ENXIO)
			ret = 0;
		WRITE_ONCE(fp->netlink_portid, 0);
		mutex_unlock(&fp->lock);
		break;

	case XIAOMI_FP_IOC_RESET:
		mutex_lock(&fp->lock);
		xiaomi_fp_reset_locked(fp, fp->reset_low_ms,
				       fp->reset_high_ms);
		mutex_unlock(&fp->lock);
		break;

	case XIAOMI_FP_IOC_RESET_TIME_MS:
		/* The vendor encoding says u32 but its payload is two u32 values. */
		if (copy_from_user(reset_time, (void __user *)arg,
				   sizeof(reset_time)))
			return -EFAULT;
		if (!reset_time[0] || !reset_time[1] ||
		    reset_time[0] > XIAOMI_FP_RESET_MAX_MS ||
		    reset_time[1] > XIAOMI_FP_RESET_MAX_MS)
			return -EINVAL;
		mutex_lock(&fp->lock);
		xiaomi_fp_reset_locked(fp, reset_time[0], reset_time[1]);
		mutex_unlock(&fp->lock);
		break;

	case XIAOMI_FP_IOC_RESET_OUT_LOW:
		mutex_lock(&fp->lock);
		gpiod_set_value_cansleep(fp->reset_gpio, 1);
		mutex_unlock(&fp->lock);
		break;

	case XIAOMI_FP_IOC_ENABLE_POWER:
	case XIAOMI_FP_IOC_DISABLE_POWER:
		ret = xiaomi_fp_get_supply_name(arg, supply);
		if (ret)
			return ret;
		if (strcmp(supply, "vreg3v3"))
			return -ENODEV;
		mutex_lock(&fp->lock);
		ret = xiaomi_fp_set_power_locked(fp,
						 cmd == XIAOMI_FP_IOC_ENABLE_POWER);
		mutex_unlock(&fp->lock);
		break;

	case XIAOMI_FP_IOC_REQUEST_RESOURCE:
		mutex_lock(&fp->lock);
		ret = xiaomi_fp_request_irq_locked(fp);
		mutex_unlock(&fp->lock);
		break;

	case XIAOMI_FP_IOC_RELEASE_RESOURCE:
		mutex_lock(&fp->lock);
		ret = xiaomi_fp_release_irq_locked(fp);
		mutex_unlock(&fp->lock);
		break;

	case XIAOMI_FP_IOC_ENABLE_IRQ:
	case XIAOMI_FP_IOC_DISABLE_IRQ:
		mutex_lock(&fp->lock);
		ret = xiaomi_fp_set_irq_locked(fp,
					       cmd == XIAOMI_FP_IOC_ENABLE_IRQ);
		mutex_unlock(&fp->lock);
		break;

	case XIAOMI_FP_IOC_DEV_INFO:
		/* The vendor encoding says char but userspace expects 30 bytes. */
		if (copy_to_user((void __user *)arg, fp->vendor_name,
				 XIAOMI_FP_VENDOR_NAME_LEN))
			return -EFAULT;
		break;

	case XIAOMI_FP_IOC_GET_FW_INFO:
		value = 0;
		if (copy_to_user((void __user *)arg, &value, sizeof(value)))
			return -EFAULT;
		break;

	case XIAOMI_FP_IOC_ENABLE_SPI_CLK:
	case XIAOMI_FP_IOC_DISABLE_SPI_CLK:
	case XIAOMI_FP_IOC_ENTER_SLEEP:
	case XIAOMI_FP_IOC_REMOVE:
	case XIAOMI_FP_IOC_CHIP_INFO:
		/* QCOM secure firmware owns the SPI data path. */
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static const struct file_operations xiaomi_fp_fops = {
	.owner = THIS_MODULE,
	.open = xiaomi_fp_open,
	.release = xiaomi_fp_release,
	.read = xiaomi_fp_read,
	.poll = xiaomi_fp_poll,
	.unlocked_ioctl = xiaomi_fp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_ptr_ioctl,
#endif
};

static ssize_t irq_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct xiaomi_fp *fp = dev_get_drvdata(dev);
	int value;

	value = gpiod_get_value_cansleep(fp->irq_gpio);
	if (value < 0)
		return value;

	return sysfs_emit(buf, "%d\n", value);
}

static ssize_t irq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	return count;
}
static DEVICE_ATTR_RW(irq);

static ssize_t fingerdown_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct xiaomi_fp *fp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", atomic_read(&fp->fingerdown));
}

static ssize_t fingerdown_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct xiaomi_fp *fp = dev_get_drvdata(dev);
	bool value;
	int ret;

	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;

	atomic_set(&fp->fingerdown, value);
	if (value)
		sysfs_notify(&dev->kobj, NULL, "fingerdown");

	return count;
}
static DEVICE_ATTR_RW(fingerdown);

static struct attribute *xiaomi_fp_attrs[] = {
	&dev_attr_irq.attr,
	&dev_attr_fingerdown.attr,
	NULL,
};
ATTRIBUTE_GROUPS(xiaomi_fp);

static int xiaomi_fp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *vendor_name;
	struct xiaomi_fp *fp;
	int ret;

	fp = devm_kzalloc(dev, sizeof(*fp), GFP_KERNEL);
	if (!fp)
		return -ENOMEM;

	fp->dev = dev;
	fp->reset_low_ms = XIAOMI_FP_RESET_MS;
	fp->reset_high_ms = XIAOMI_FP_RESET_MS;
	mutex_init(&fp->lock);
	init_waitqueue_head(&fp->irq_wait);
	atomic64_set(&fp->irq_sequence, 0);
	atomic_set(&fp->fingerdown, 0);
	platform_set_drvdata(pdev, fp);

	fp->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(fp->vdd))
		return dev_err_probe(dev, PTR_ERR(fp->vdd),
				     "failed to get vdd supply\n");

	fp->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(fp->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(fp->reset_gpio),
				     "failed to get reset GPIO\n");

	fp->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(fp->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(fp->irq_gpio),
				     "failed to get IRQ GPIO\n");

	fp->irq = gpiod_to_irq(fp->irq_gpio);
	if (fp->irq < 0)
		return dev_err_probe(dev, fp->irq,
				     "failed to map IRQ GPIO\n");

	ret = devm_request_threaded_irq(dev, fp->irq, NULL,
					xiaomi_fp_irq_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT |
					IRQF_NO_AUTOEN,
					"xiaomi-fp-irq", fp);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	if (!device_property_read_string(dev, "xiaomi,vendor-name",
					 &vendor_name))
		strscpy(fp->vendor_name, vendor_name, sizeof(fp->vendor_name));

	fp->miscdev.minor = MISC_DYNAMIC_MINOR;
	fp->miscdev.name = XIAOMI_FP_NAME;
	fp->miscdev.fops = &xiaomi_fp_fops;
	fp->miscdev.parent = dev;

	fp->id_miscdev.minor = MISC_DYNAMIC_MINOR;
	fp->id_miscdev.name = XIAOMI_FP_ID_NAME;
	fp->id_miscdev.fops = &xiaomi_fp_fops;
	fp->id_miscdev.parent = dev;

	ret = misc_register(&fp->miscdev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register control device\n");

	ret = misc_register(&fp->id_miscdev);
	if (ret) {
		misc_deregister(&fp->miscdev);
		return dev_err_probe(dev, ret,
				     "failed to register identity device\n");
	}

	ret = xiaomi_fp_netlink_start(fp);
	if (ret)
		dev_warn(dev, "netlink protocol 25 unavailable: %d\n", ret);

	if (device_property_read_bool(dev, "wakeup-source")) {
		device_init_wakeup(dev, true);
		ret = dev_pm_set_wake_irq(dev, fp->irq);
		if (ret)
			dev_warn(dev, "failed to configure wake IRQ: %d\n", ret);
	}

	dev_info(dev, "TrustZone fingerprint control shim ready\n");
	return 0;
}

static void xiaomi_fp_remove(struct platform_device *pdev)
{
	struct xiaomi_fp *fp = platform_get_drvdata(pdev);

	WRITE_ONCE(fp->dead, true);
	misc_deregister(&fp->id_miscdev);
	misc_deregister(&fp->miscdev);

	mutex_lock(&fp->lock);
	if (fp->irq_requested)
		xiaomi_fp_release_irq_locked(fp);
	gpiod_set_value_cansleep(fp->reset_gpio, 1);
	if (fp->powered)
		xiaomi_fp_set_power_locked(fp, false);
	mutex_unlock(&fp->lock);

	wake_up_interruptible(&fp->irq_wait);
	xiaomi_fp_netlink_stop(fp);
	dev_pm_clear_wake_irq(&pdev->dev);
	device_init_wakeup(&pdev->dev, false);
}

static const struct of_device_id xiaomi_fp_of_match[] = {
	{ .compatible = "xiaomi,xiaomi-fp" },
	{ }
};
MODULE_DEVICE_TABLE(of, xiaomi_fp_of_match);

static struct platform_driver xiaomi_fp_driver = {
	.probe = xiaomi_fp_probe,
	.remove = xiaomi_fp_remove,
	.driver = {
		.name = XIAOMI_FP_NAME,
		.of_match_table = xiaomi_fp_of_match,
		.dev_groups = xiaomi_fp_groups,
	},
};
module_platform_driver(xiaomi_fp_driver);

MODULE_AUTHOR("BigfootACA");
MODULE_DESCRIPTION("Xiaomi TrustZone side fingerprint control shim");
MODULE_LICENSE("GPL");
