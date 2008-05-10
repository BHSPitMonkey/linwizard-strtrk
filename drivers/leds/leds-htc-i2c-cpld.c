/*
 * LEDs driver for the htc-i2c-cpld interface.
 *
 * Copyright 2008 (C) Justin Verel <justin@marverinc.nl>
 *
 *	based on leds-tosa.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <linux/i2c/htc-i2c-cpld.h>

static void htci2ccpldled_amber_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(LED_ORANGE, 1);	
}

static void htci2ccpldled_green_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(LED_LGREEN, 1);
}

static struct led_classdev tosa_amber_led = {
	.name			= "tosa:amber:charge",
	.default_trigger	= "sharpsl-charge",
	.brightness_set		= htci2ccpldled_amber_set,
};

static struct led_classdev tosa_green_led = {
	.name			= "tosa:green:mail",
	.default_trigger	= "nand-disk",
	.brightness_set		= htci2ccpldled_green_set,
};

static int htci2ccpldled_probe(struct platform_device *pdev)
{
	int ret;

	ret = led_classdev_register(&pdev->dev, &tosa_amber_led);
	if (ret < 0)
		return ret;

	ret = led_classdev_register(&pdev->dev, &tosa_green_led);
	if (ret < 0)
		led_classdev_unregister(&tosa_amber_led);

	return ret;
}

static int htci2ccpldled_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&tosa_amber_led);
	led_classdev_unregister(&tosa_green_led);
        led_classdev_unregister(&tosa_red_led);

	return 0;
}

static struct platform_driver htci2ccpldled_driver = {
	.probe		= htci2ccpldled_probe,
	.remove		= htci2ccpldled_remove,
	.driver		= {
		.name		= "tosa-led",
		.owner		= THIS_MODULE,
	},
};

static int __init htci2ccpldled_init(void)
{
	return platform_driver_register(&htci2ccpldled_driver);
}

static void __exit htci2ccpldled_exit(void)
{
 	platform_driver_unregister(&htci2ccpldled_driver);
}

module_init(htci2ccpldled_init);
module_exit(htci2ccpldled_exit);

MODULE_AUTHOR("Justin Verel <justin@marverinc.nl>");
MODULE_DESCRIPTION("HTC I2C CPLD LED driver");
MODULE_LICENSE("GPL");
