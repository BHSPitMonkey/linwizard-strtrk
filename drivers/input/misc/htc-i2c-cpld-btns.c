/*
 *  htc-i2c-cpld buttons interface driver.
 *  Provides support for directional buttons.
 *
 *  Copyright (C) 2008  Angelo Arrifano <miknix@gmail.com>
 *
 *  Based on drivers/mfd/htc-bbkeys.c by Kevin O'Connor
 *  Based on drivers/input/misc/cobalt_btns.c by Yoichi Yuasa
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <linux/i2c/htc-i2c-cpld.h>

#define HAS_KEY(F,K) ((F | K) == status)

static const unsigned short htci2ccpldbtns_map[] = {
	KEY_LEFT,
	KEY_UP,
	KEY_DOWN,
	KEY_RIGHT,
	KEY_ENTER
};

struct buttons_dev {
	unsigned short keymap[ARRAY_SIZE(htci2ccpldbtns_map)];
	int count[ARRAY_SIZE(htci2ccpldbtns_map)];
	int irq;
	struct input_dev *input;
};

static inline void
htci2ccpldbtns_report_key(struct buttons_dev *bdev, int key)
{
	input_report_key(bdev->input, bdev->keymap[key], 1);
	input_sync(bdev->input);
	input_report_key(bdev->input, bdev->keymap[key], 0);
	input_sync(bdev->input);
}

static irqreturn_t htci2ccpldbtns_irq(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct device *dev = &pdev->dev;
	struct buttons_dev *bdev = dev_get_drvdata(dev);
	int status = 0;
	
	status = htci2ccpld_btn_get(HTCI2CCPLD_BTN_CHIP_DKB);
	status |= htci2ccpld_btn_get(HTCI2CCPLD_BTN_CHIP_DPAD);
	
	if (HAS_KEY(status, HTCI2CCPLD_BTN_LEFT))
		htci2ccpldbtns_report_key(bdev, 0);

	if (HAS_KEY(status, HTCI2CCPLD_BTN_UP))
		htci2ccpldbtns_report_key(bdev, 1);

	if (HAS_KEY(status, HTCI2CCPLD_BTN_DOWN))
		htci2ccpldbtns_report_key(bdev, 2);
		
	if (HAS_KEY(status, HTCI2CCPLD_BTN_RIGHT))
		htci2ccpldbtns_report_key(bdev, 3);

	if (HAS_KEY(status, HTCI2CCPLD_BTN_ENTER))
		htci2ccpldbtns_report_key(bdev, 4);

	return IRQ_HANDLED;
}

static int __devinit htci2ccpldbtns_probe(struct platform_device *pdev)
{
	struct buttons_dev *bdev;
	struct input_dev *input;
	struct resource *res;
	int error, i;

	bdev = kzalloc(sizeof(struct buttons_dev), GFP_KERNEL);
	input = input_allocate_device();
	if (!bdev || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	platform_set_drvdata(pdev, bdev);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		error = -EBUSY;
		goto err_free_mem;
	}

	bdev->irq = res->start;
	bdev->input = input;
	memcpy(bdev->keymap, htci2ccpldbtns_map, sizeof(bdev->keymap));

	input->name = "HTC I2C CPLD BTNS";
	input->keycode = bdev->keymap;
	input->keycodemax = ARRAY_SIZE(bdev->keymap);
	input->keycodesize = sizeof(unsigned short);

	set_bit(EV_KEY, input->evbit);
	for (i = 0; i < ARRAY_SIZE(htci2ccpldbtns_map); i++)
		set_bit(bdev->keymap[i], input->keybit);

	error = input_register_device(input);
	if (error)
		goto err_free_mem;

	error = request_irq(bdev->irq, htci2ccpldbtns_irq,
	                  IRQF_TRIGGER_FALLING | IRQF_SAMPLE_RANDOM,
							pdev->name, pdev);
	if (error)
		goto err_unreg;

	printk("htc-i2c-cpld-btns: HTC I2C CPLD Buttons\n");
	printk("htc-i2c-cpld-btns: Using IRQ: %d\n", bdev->irq);

	return 0;

 err_unreg:
	input_unregister_device(input);
 err_free_mem:
	input_free_device(input);
	kfree(bdev);
	dev_set_drvdata(&pdev->dev, NULL);
	return error;
}

static int __devexit htci2ccpldbtns_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct buttons_dev *bdev = dev_get_drvdata(dev);

	input_unregister_device(bdev->input);
	input_free_device(bdev->input);
	free_irq(bdev->irq, pdev);
	kfree(bdev);
	dev_set_drvdata(dev, NULL);

	return 0;
}

static struct platform_driver htci2ccpldbtns_driver = {
	.probe	= htci2ccpldbtns_probe,
	.remove	= __devexit_p(htci2ccpldbtns_remove),
	.driver	= {
		.name	= "htc-i2c-cpld-btns",
		.owner	= THIS_MODULE,
	},
};

static int __init htci2ccpldbtns_init(void)
{
	return platform_driver_register(&htci2ccpldbtns_driver);
}

static void __exit htci2ccpldbtns_exit(void)
{
	platform_driver_unregister(&htci2ccpldbtns_driver);
}

module_init(htci2ccpldbtns_init);
module_exit(htci2ccpldbtns_exit);

MODULE_AUTHOR("Angelo Arrifano <miknix@gmail.com>");
MODULE_LICENSE("GPL");
