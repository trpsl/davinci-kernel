/*
 * TEE driver for goodix fingerprint sensor
 * Copyright (C) 2016 Goodix
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)     KBUILD_MODNAME ": " fmt
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/pm_wakeup.h>
#include <drm/drm_bridge.h>
#include <drm/drm_notifier.h>
#include "gf_spi.h"

#define VER_MAJOR   1
#define VER_MINOR   2
#define PATCH_LEVEL 1

#define WAKELOCK_HOLD_TIME 2000 /* in ms */
#define FP_UNLOCK_REJECTION_TIMEOUT (WAKELOCK_HOLD_TIME - 500)

#define GF_SPIDEV_NAME			"goodix,fingerprint"
/*device name after register in charater*/
#define GF_DEV_NAME				"goodix_fp"
#define GF_INPUT_NAME			"uinput-goodix" /*"goodix_fp" */

#define CHRD_DRIVER_NAME		"goodix_fp_spi"
#define CLASS_NAME				"goodix_fp"

#define N_SPI_MINORS			32  /* ... up to 256 */


static int SPIDEV_MAJOR;

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct wakeup_source fp_wakelock;
static struct gf_dev gf;

static void gf_enable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		pr_warn("IRQ has been enabled.\n");
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	}
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
	if (gf_dev->irq_enabled) {
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->irq);
	} else {
		pr_warn("IRQ has been disabled.\n");
	}
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	int retval = 0;
	u8 netlink_route = NETLINK_TEST;
	struct gf_ioc_chip_info info;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC) {
		return -ENODEV;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (retval) {
		return -EFAULT;
	}

	if (gf_dev->device_available == 0) {
		if ((cmd == GF_IOC_ENABLE_POWER) || (cmd == GF_IOC_DISABLE_POWER)) {
			pr_debug("power cmd\n");
		} else {
			pr_debug("get cmd %d, but sensor is power off currently.\n", _IOC_NR(cmd));
			return -ENODEV;
		}
	}

	switch (cmd) {
		case GF_IOC_INIT:
			pr_debug("%s GF_IOC_INIT\n", __func__);

			if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
				retval = -EFAULT;
				break;
			}
			break;
		case GF_IOC_EXIT:
			pr_debug("%s GF_IOC_EXIT\n", __func__);
			break;
		case GF_IOC_DISABLE_IRQ:
			pr_debug("%s GF_IOC_DISABLE_IRQ\n", __func__);
			gf_disable_irq(gf_dev);
			break;
		case GF_IOC_ENABLE_IRQ:
			pr_debug("%s GF_IOC_ENABLE_IRQ\n", __func__);
			gf_enable_irq(gf_dev);
			break;
		case GF_IOC_RESET:
			pr_debug("%s GF_IOC_RESET.\n", __func__);
			gf_hw_reset(gf_dev, 3);
			break;
		case GF_IOC_ENABLE_POWER:
			pr_debug("%s GF_IOC_ENABLE_POWER\n", __func__);
			if (gf_dev->device_available == 1) {
				pr_debug("Sensor has already powered-on.\n");
			} else {
				gf_power_on(gf_dev);
			}
			gf_dev->device_available = 1;
			break;
		case GF_IOC_DISABLE_POWER:
			pr_debug("%s GF_IOC_DISABLE_POWER\n", __func__);
			if (gf_dev->device_available == 0) {
				pr_debug("Sensor has already powered-off.\n");
			} else {
				gf_power_off(gf_dev);
			}
			gf_dev->device_available = 0;
			break;
		case GF_IOC_ENTER_SLEEP_MODE:
			pr_debug("%s GF_IOC_ENTER_SLEEP_MODE\n", __func__);
			break;
		case GF_IOC_GET_FW_INFO:
			pr_debug("%s GF_IOC_GET_FW_INFO\n", __func__);
			break;
		case GF_IOC_REMOVE:
			pr_debug("%s GF_IOC_REMOVE\n", __func__);
			break;
		case GF_IOC_CHIP_INFO:
			pr_debug("%s GF_IOC_CHIP_INFO\n", __func__);
			if (copy_from_user(&info, (struct gf_ioc_chip_info *)arg,
						sizeof(struct gf_ioc_chip_info))) {
				retval = -EFAULT;
				break;
			}
			pr_debug("vendor_id : 0x%x\n", info.vendor_id);
			pr_debug("mode : 0x%x\n", info.mode);
			pr_debug("operation: 0x%x\n", info.operation);
			break;
		default:
			pr_debug("unsupport cmd:0x%x\n", cmd);
			break;
	}
	return retval;
}

static long gf_compat_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}

static void notification_work(struct work_struct *work)
{
	pr_debug("%s unblank\n", __func__);
	dsi_bridge_interface_enable(FP_UNLOCK_REJECTION_TIMEOUT);
}

static irqreturn_t gf_irq(int irq, void *handle)
{
	char temp[4] = { 0x0 };
	temp[0] = GF_NET_EVENT_IRQ;

	pr_debug("%s enter\n", __func__);
	__pm_wakeup_event(&fp_wakelock, WAKELOCK_HOLD_TIME);
	sendnlmsg(temp);

	return IRQ_HANDLED;
}

static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;
	int rc = 0;
	int err = 0;

	mutex_lock(&device_list_lock);
	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			pr_debug("Found\n");
			status = 0;
			break;
		}
	}

	if (status == 0) {
		rc = gpio_request(gf_dev->pwr_gpio, "goodix_pwr");

		if (rc) {
			dev_err(&gf_dev->spi->dev, "Failed to request PWR GPIO. rc = %d\n", rc);
			mutex_unlock(&device_list_lock);
			err = -EPERM;
			goto open_error1;
		}

		rc = gpio_request(gf_dev->reset_gpio, "gpio-reset");

		if (rc) {
			dev_err(&gf_dev->spi->dev, "Failed to request RESET GPIO. rc = %d\n", rc);
			mutex_unlock(&device_list_lock);
			err = -EPERM;
			goto open_error1;
		}

		gpio_direction_output(gf_dev->reset_gpio, 0);
		rc = gpio_request(gf_dev->irq_gpio, "gpio-irq");

		if (rc) {
			dev_err(&gf_dev->spi->dev, "Failed to request IRQ GPIO. rc = %d\n", rc);
			mutex_unlock(&device_list_lock);
			err = -EPERM;
			goto open_error2;
		}

		gpio_direction_input(gf_dev->irq_gpio);
		rc = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						"gf", gf_dev);

		if (!rc) {
			enable_irq_wake(gf_dev->irq);
			gf_dev->irq_enabled = 1;
			gf_disable_irq(gf_dev);
		} else {
			err = -EPERM;
			goto open_error3;
		}

		gf_dev->users++;
		filp->private_data = gf_dev;
		nonseekable_open(inode, filp);
		pr_debug("Succeed to open device. irq = %d\n", gf_dev->irq);
	} else {
		pr_debug("No device for minor %d\n", iminor(inode));
	}

	mutex_unlock(&device_list_lock);
	return status;

open_error3:
	gpio_free(gf_dev->irq_gpio);

open_error2:
	gpio_free(gf_dev->reset_gpio);

open_error1:
	return err;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = 0;

	pr_debug("%s\n", __func__);
	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;
	/*
	 *Disable fp_vdd_vreg regulator
	 */
	gf_dev->users --;

	if (!gf_dev->users) {
		pr_debug("disble_irq. irq = %d\n", gf_dev->irq);
		gf_disable_irq(gf_dev);
		/*power off the sensor*/
		gf_dev->device_available = 0;
		free_irq(gf_dev->irq, gf_dev);
		gpio_free(gf_dev->irq_gpio);
		gpio_free(gf_dev->reset_gpio);
		gf_power_off(gf_dev);
		gpio_free(gf_dev->pwr_gpio);
	}

	mutex_unlock(&device_list_lock);
	return status;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.unlocked_ioctl = gf_ioctl,
	.compat_ioctl = gf_compat_ioctl,
	.open = gf_open,
	.release = gf_release,
};

static int goodix_fb_state_chg_callback(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct gf_dev *gf_dev;
	struct fb_event *evdata = data;
	unsigned int blank;
	char temp[4] = { 0x0 };

	if (val != DRM_EVENT_BLANK) {
		return 0;
	}

	pr_debug("[info] %s go to the goodix_fb_state_chg_callback value = %d\n",
		 __func__, (int)val);
	gf_dev = container_of(nb, struct gf_dev, notifier);

	if (evdata && evdata->data && val == DRM_EVENT_BLANK && gf_dev) {
		blank = *(int *)(evdata->data);

		switch (blank) {
			case DRM_BLANK_POWERDOWN:
				if (gf_dev->device_available == 1) {
					gf_dev->fb_black = 1;
					gf_dev->wait_finger_down = true;
					temp[0] = GF_NET_EVENT_FB_BLACK;
					sendnlmsg(temp);
				}
				break;
			case DRM_BLANK_UNBLANK:
				if (gf_dev->device_available == 1) {
					gf_dev->fb_black = 0;
					temp[0] = GF_NET_EVENT_FB_UNBLACK;
					sendnlmsg(temp);
				}
				break;
			default:
				pr_debug("%s defalut\n", __func__);
				break;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block goodix_noti_block = {
	.notifier_call = goodix_fb_state_chg_callback,
};

static struct class *gf_class;
static int gf_probe(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;
	int status = -EINVAL;
	unsigned long minor;

	/* Initialize the driver data */
	INIT_LIST_HEAD(&gf_dev->device_entry);
	gf_dev->spi = pdev;
	gf_dev->irq_gpio = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->pwr_gpio = -EINVAL;
	gf_dev->device_available = 0;
	gf_dev->fb_black = 0;
	gf_dev->wait_finger_down = false;
	INIT_WORK(&gf_dev->work, notification_work);

	if (gf_parse_dts(gf_dev)) {
		goto error_hw;
	}

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);

	if (minor < N_SPI_MINORS) {
		struct device *dev;
		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
					gf_dev, GF_DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&gf_dev->spi->dev, "no minor number available!\n");
		status = -ENODEV;
		mutex_unlock(&device_list_lock);
		goto error_hw;
	}

	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	} else {
		gf_dev->devt = 0;
	}

	mutex_unlock(&device_list_lock);

	if (status == 0) {
		/*input device subsystem */
		gf_dev->input = input_allocate_device();

		if (gf_dev->input == NULL) {
			pr_err("%s, failed to allocate input device\n", __func__);
			status = -ENOMEM;
			goto error_dev;
		}

		gf_dev->input->name = GF_INPUT_NAME;
		status = input_register_device(gf_dev->input);

		if (status) {
			pr_err("failed to register input device\n");
			goto error_input;
		}
	}

	gf_dev->notifier = goodix_noti_block;
	drm_register_client(&gf_dev->notifier);
	gf_dev->irq = gf_irq_num(gf_dev);
	wakeup_source_init(&fp_wakelock, "fp_wakelock");
	pr_debug("version V%d.%d.%02d\n", VER_MAJOR, VER_MINOR, PATCH_LEVEL);
	return status;
	input_unregister_device(gf_dev->input);

error_input:
	if (gf_dev->input != NULL) {
		input_free_device(gf_dev->input);
	}

error_dev:
	if (gf_dev->devt != 0) {
		pr_debug("Err: status = %d\n", status);
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(gf_class, gf_dev->devt);
		clear_bit(MINOR(gf_dev->devt), minors);
		mutex_unlock(&device_list_lock);
	}

error_hw:
	gf_cleanup(gf_dev);
	gf_dev->device_available = 0;
	return status;
}

static int gf_remove(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;

	wakeup_source_trash(&fp_wakelock);

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq) {
		free_irq(gf_dev->irq, gf_dev);
	}

	if (gf_dev->input != NULL) {
		input_unregister_device(gf_dev->input);
	}

	input_free_device(gf_dev->input);
	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);

	if (gf_dev->users == 0) {
		gf_cleanup(gf_dev);
	}

	drm_unregister_client(&gf_dev->notifier);
	mutex_unlock(&device_list_lock);
	return 0;
}

static struct of_device_id gx_match_table[] = {
	{ .compatible = GF_SPIDEV_NAME },
	{},
};

static struct platform_driver gf_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gx_match_table,
	},
	.probe = gf_probe,
	.remove = gf_remove,
};

static int __init gf_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);

	if (status < 0) {
		pr_warn("Failed to register char device!\n");
		return status;
	}

	SPIDEV_MAJOR = status;
	gf_class = class_create(THIS_MODULE, CLASS_NAME);

	if (IS_ERR(gf_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("Failed to create class.\n");
		return PTR_ERR(gf_class);
	}

	status = platform_driver_register(&gf_driver);
	if (status < 0) {
		class_destroy(gf_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("Failed to register SPI driver.\n");
	}

	netlink_init();
	pr_debug("status = 0x%x\n", status);
	return 0;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
	netlink_exit();
	platform_driver_unregister(&gf_driver);
	class_destroy(gf_class);
	unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_AUTHOR("Jandy Gou, <gouqingsong@goodix.com>");
MODULE_DESCRIPTION("goodix fingerprint sensor device driver");
MODULE_LICENSE("GPL");
