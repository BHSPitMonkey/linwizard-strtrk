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

static void htci2ccpldled_orange_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(LED_ORANGE, value);	
}

static void htci2ccpldled_lgreen_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(LED_LGREEN, value);
}

static struct led_classdev htci2ccpldled_orange_led = {
	.name			= "htci2ccpld:orange",
	.brightness_set		= htci2ccpldled_orange_set,
};

static struct led_classdev htci2ccpldled_lgreen_led = {
	.name			= "htci2ccpld:lgreen",
	.brightness_set		= htci2ccpldled_lgreen_set,
};

static int htci2ccpldled_probe(struct platform_device *pdev)
{
	int ret = 0;
	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_orange_led);
	if (ret < 0)
		return ret;

	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_lgreen_led);
	if (ret < 0)
		led_classdev_unregister(&htci2ccpldled_orange_led);

	return ret;
}

static int htci2ccpldled_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&htci2ccpldled_orange_led);
	led_classdev_unregister(&htci2ccpldled_lgreen_led);

	return 0;
}

#ifdef CONFIG_PM
static int htci2ccpldled_suspend(struct platform_device *dev, pm_message_t state)
{
	led_classdev_suspend(&htci2ccpldled_orange_led);
	led_classdev_suspend(&htci2ccpldled_lgreen_led);
	return 0;
}

static int htci2ccpldled_resume(struct platform_device *dev)
{
	led_classdev_resume(&htci2ccpldled_orange_led);
	led_classdev_resume(&htci2ccpldled_lgreen_led);
	return 0;
}
#endif


static struct platform_driver htci2ccpldled_driver = {
	.probe		= htci2ccpldled_probe,
	.remove		= htci2ccpldled_remove,
#ifdef CONFIG_PM
	.suspend    = htci2ccpldled_suspend,
	.resume		= htci2ccpldled_resume,
#endif
	.driver		= {
		.name		= "htc-i2c-cpld-led",
		.owner		= THIS_MODULE,
	},
};

static int __init htci2ccpldled_init(void)
{
	printk("HTC I2C CPLD LED driver\n");
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
