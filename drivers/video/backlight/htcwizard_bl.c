/*
 *  Backlight Driver using the htcwizcpld interface.
 *
 *  Copyright (c) 2008 Angelo Arrifano <miknix@gmail.com>
 *
 *  Based on drivers/video/backlight/hp680_bl.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/backlight.h>
#include <linux/i2c/htcwizard-cpld.h>


static int htcwizbl_set_brightness(struct backlight_device *bl_dev)
{
	htcwizcpld_bl_set(bl_dev->props.brightness);
	return 0;
}

static int htcwizbl_get_brightness(struct backlight_device *bl_dev)
{
	return htcwizcpld_bl_get();
}

static struct backlight_ops htcwizbl_ops = {
	.get_brightness = htcwizbl_get_brightness,
	.update_status  = htcwizbl_set_brightness,
};

static int __init htcwizbl_probe(struct platform_device *pdev)
{
	struct backlight_device *bl_dev;

	bl_dev = backlight_device_register ("htcwiz-bl", &pdev->dev, NULL,
		    &htcwizbl_ops);
	/* FIXME:
	if (IS_ERR(bl_dev))
		return PTR_ERR(bl_dev);*/

	platform_set_drvdata(pdev, bl_dev);

	bl_dev->props.max_brightness = HTCWIZCPLD_BL_LMAX;
	bl_dev->props.brightness = htcwizcpld_bl_get();

	return 0;
}

static int htcwizbl_remove(struct platform_device *pdev)
{
	struct backlight_device *bl_dev = platform_get_drvdata(pdev);

	backlight_device_unregister(bl_dev);

	return 0;
}

static struct platform_driver htcwizbl_driver = {
	.probe		= htcwizbl_probe,
	.remove		= htcwizbl_remove,
	.driver		= {
		.name	= "htcwiz-bl",
	},
};

static struct platform_device *htcwizbl_device;

static int __init htcwizbl_init(void)
{
	int ret;

	ret = platform_driver_register(&htcwizbl_driver);
	if (!ret) {
		htcwizbl_device = platform_device_alloc("htcwiz-bl", -1);
		if (!htcwizbl_device)
			return -ENOMEM;

		ret = platform_device_add(htcwizbl_device);

		if (ret) {
			platform_device_put(htcwizbl_device);
			platform_driver_unregister(&htcwizbl_driver);
		}
	}
	return ret;
}

static void __exit htcwizbl_exit(void)
{
	platform_device_unregister(htcwizbl_device);
 	platform_driver_unregister(&htcwizbl_driver);
}

module_init(htcwizbl_init);
module_exit(htcwizbl_exit);

MODULE_AUTHOR("Angelo Arrifano <miknix@gmail.com>");
MODULE_DESCRIPTION("Backlight driver for HTC Wizard CPLD Interface");
MODULE_LICENSE("GPL");
