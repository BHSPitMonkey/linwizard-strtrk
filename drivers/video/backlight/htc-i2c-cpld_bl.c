/*
 *  Backlight Driver using the HTC I2C CPLD interface.
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
#include <linux/i2c/htc-i2c-cpld.h>


static int htci2ccpldbl_set_brightness(struct backlight_device *bl_dev)
{
	htci2ccpld_bl_set(bl_dev->props.brightness);
	return 0;
}

static int htci2ccpldbl_get_brightness(struct backlight_device *bl_dev)
{
	return htci2ccpld_bl_get();
}

static struct backlight_ops htci2ccpldbl_ops = {
	.get_brightness = htci2ccpldbl_get_brightness,
	.update_status  = htci2ccpldbl_set_brightness,
};

static int __init htci2ccpldbl_probe(struct platform_device *pdev)
{
	struct backlight_device *bl_dev;

	bl_dev = backlight_device_register ("generic-bl", &pdev->dev, NULL,
		    &htci2ccpldbl_ops);
	/* FIXME:
	if (IS_ERR(bl_dev))
		return PTR_ERR(bl_dev);*/

	platform_set_drvdata(pdev, bl_dev);

	bl_dev->props.max_brightness = HTCI2CCPLD_BL_LMAX;
	bl_dev->props.brightness = htci2ccpld_bl_get();

	return 0;
}

static int htci2ccpldbl_remove(struct platform_device *pdev)
{
	struct backlight_device *bl_dev = platform_get_drvdata(pdev);

	backlight_device_unregister(bl_dev);

	return 0;
}

static struct platform_driver htci2ccpldbl_driver = {
	.probe		= htci2ccpldbl_probe,
	.remove		= htci2ccpldbl_remove,
	.driver		= {
		.name	= "htc-i2c-cpld-bl",
	},
};

static struct platform_device *htci2ccpldbl_device;

static int __init htci2ccpldbl_init(void)
{
	int ret;

	ret = platform_driver_register(&htci2ccpldbl_driver);
	if (!ret) {
		htci2ccpldbl_device = platform_device_alloc("htc-i2c-cpld-bl", -1);
		if (!htci2ccpldbl_device)
			return -ENOMEM;

		ret = platform_device_add(htci2ccpldbl_device);

		if (ret) {
			platform_device_put(htci2ccpldbl_device);
			platform_driver_unregister(&htci2ccpldbl_driver);
		}
	}
	return ret;
}

static void __exit htci2ccpldbl_exit(void)
{
	platform_device_unregister(htci2ccpldbl_device);
 	platform_driver_unregister(&htci2ccpldbl_driver);
}

module_init(htci2ccpldbl_init);
module_exit(htci2ccpldbl_exit);

MODULE_AUTHOR("Angelo Arrifano <miknix@gmail.com>");
MODULE_DESCRIPTION("Backlight driver using the HTC I2C CPLD I2C");
MODULE_LICENSE("GPL");
