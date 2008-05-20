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

/* Enable if building for htc herald */
/* #define HTC_IS_HERALD */

static void htci2ccpldled_orange_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(HTCI2CCPLD_LED_ORANGE, value);	
}

static void htci2ccpldled_red_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(HTCI2CCPLD_LED_RED, value);	
}

static void htci2ccpldled_lgreen_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(HTCI2CCPLD_LED_LGREEN, value);
}

static void htci2ccpldled_rgreen_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(HTCI2CCPLD_LED_RGREEN, value);
}

static void htci2ccpldled_vibrator_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_vibrator_set(value);
}

static void htci2ccpldled_camera_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(HTCI2CCPLD_LED_CAMERA, value);
}

static void htci2ccpldled_dpad_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(HTCI2CCPLD_LED_DPAD, value);
}

static void htci2ccpldled_kb_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(HTCI2CCPLD_LED_KB, value);
}

#ifdef HTC_IS_HERALD
static void htci2ccpldled_fn_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	htci2ccpld_led_set(HTCI2CCPLD_FN, value);
}
#endif

static struct led_classdev htci2ccpldled_orange_led = {
	.name			= "htci2ccpld:orange",
	.brightness_set		= htci2ccpldled_orange_set,
};

static struct led_classdev htci2ccpldled_lgreen_led = {
	.name			= "htci2ccpld:lgreen",
	.brightness_set		= htci2ccpldled_lgreen_set,
};

static struct led_classdev htci2ccpldled_red_led = {
	.name			= "htci2ccpld:red",
	.brightness_set		= htci2ccpldled_red_set,
};

static struct led_classdev htci2ccpldled_rgreen_led = {
	.name			= "htci2ccpld:rgreen",
	.default_trigger = "heartbeat",
	.brightness_set		= htci2ccpldled_rgreen_set,
};

static struct led_classdev htci2ccpldled_vibrator = {
	.name			= "htci2ccpld:vibrator",
	.brightness_set		= htci2ccpldled_vibrator_set,
};

static struct led_classdev htci2ccpldled_camera = {
	.name			= "htci2ccpld:camera",
	.brightness_set		= htci2ccpldled_camera_set,
};

static struct led_classdev htci2ccpldled_dpad = {
	.name			= "htci2ccpld:dpad",
	.brightness_set		= htci2ccpldled_dpad_set,
};

static struct led_classdev htci2ccpldled_kb = {
	.name			= "htci2ccpld:kb",
	.brightness_set		= htci2ccpldled_kb_set,
};

#ifdef HTC_IS_HERALD
static struct led_classdev htci2ccpldled_fn = {
	.name			= "htci2ccpld:fnkey",
	.brightness_set		= htci2ccpldled_fn_set,
};
#endif

static int htci2ccpldled_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_orange_led);
	if (ret < 0)
		return ret;

	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_lgreen_led);
	if (ret < 0)
		goto undo_orange;
	
	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_rgreen_led);
	if (ret < 0)
		goto undo_lgreen;

	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_red_led);
	if (ret < 0)
		goto undo_rgreen;

	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_vibrator);
	if (ret < 0)
		goto undo_red;

	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_camera);
	if (ret < 0)
		goto undo_vibr;

	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_dpad);
	if (ret < 0)
		goto undo_cam;

	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_kb);
	if (ret < 0)
		goto undo_dpad;

#ifdef HTC_IS_HERALD
	ret = led_classdev_register(&pdev->dev, &htci2ccpldled_fn);
	if (ret < 0)
		goto undo_kb;
#endif

	return ret;

#ifdef HTC_IS_HERALD
undo_kb:
		led_classdev_unregister(&htci2ccpldled_kb);
#endif
undo_dpad:
		led_classdev_unregister(&htci2ccpldled_dpad);
undo_cam:
		led_classdev_unregister(&htci2ccpldled_camera);
undo_vibr:
		led_classdev_unregister(&htci2ccpldled_vibrator);
undo_red:
		led_classdev_unregister(&htci2ccpldled_red_led);
undo_rgreen:
		led_classdev_unregister(&htci2ccpldled_rgreen_led);
undo_lgreen:
		led_classdev_unregister(&htci2ccpldled_lgreen_led);
undo_orange:
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
	printk("htc-i2c-cpld-led: HTC I2C CPLD LED driver\n");
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
