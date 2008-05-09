/*
 *  htcwizard-cpld.c - Chip driver for a unknown cpld found on htc wizards.
 *  This device is accessible by i2c bus and allows controlling leds,
 *  backlight, etc..
 *  It also exports directional pad events as standard input.
 *
 *  Copyright (C) 2008 Angelo Arrifano <miknix@gmail.com>
 *
 *  Based on i2c/chips/eeprom.c
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/backlight.h>

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x03, 0x04, 0x05, 0x06, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1(htcwizard_cpld);

/* 
 * Chip structures
 */

static struct s_htcwizcpld_chip4_data {
	struct i2c_client client;
	struct backlight_ops bl_data;
	struct backlight_device *bl_dev;
} htcwizcpld_chip4_data;


/*
 * Backlight callbacks
 */
static int htcwizcpld_chip4_get_brightness(struct backlight_device *bl_dev)
{
	
	return 0;
}

static int htcwizcpld_chip4_set_status(struct backlight_device *bl_dev)
{
	u8 cmd = 0xF0;

	switch (bl_dev->props.brightness) {
	case 1:
		cmd |=  0x08;
		cmd &= ~0x06;
		break;
	case 2:
		cmd |=  0x0C;
		cmd &= ~0x02;
		break;
	case 3:
		cmd |=  0x0A;
		cmd &= ~0x04;
		break;
	case 4:
		cmd |=  0x0E;
		break;
	}
	i2c_smbus_read_byte_data(&htcwizcpld_chip4_data.client, cmd);
	
	return 0;
}


/*
 * Driver handling
 */
static int htcwizcpld_attach_adapter(struct i2c_adapter *adapter);
static int htcwizcpld_detach_client(struct i2c_client *client);

static struct i2c_driver htcwizcpld_driver = {
	.driver = {
		.name	= "htcwizcpld",
	},
	.attach_adapter	= htcwizcpld_attach_adapter,
	.detach_client	= htcwizcpld_detach_client,
};

static int htcwizcpld_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct s_htcwizcpld_chip4_data *chip4_data = &htcwizcpld_chip4_data;
	struct i2c_client *client;
	int err;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
		printk("htcwizcpld: Unsuported I2C BUS!\n");
		return -ENODEV;
	}

	if (address == 0x04)
		goto chip4;

	return -ENODEV;

chip4:
	client = &chip4_data->client;
	i2c_set_clientdata(client, chip4_data);
	client->addr = address;
	client->adapter = adapter;
	client->driver = &htcwizcpld_driver;
	strlcpy(client->name, "htcwizcpld", I2C_NAME_SIZE);

	chip4_data->bl_data.get_brightness = htcwizcpld_chip4_get_brightness;
	chip4_data->bl_data.update_status = htcwizcpld_chip4_set_status;
	chip4_data->bl_dev = backlight_device_register("htcwizcpld_bl", NULL, 
						 NULL, &chip4_data->bl_data);
	//FIXME: Check success of code above!!
	chip4_data->bl_dev->props.max_brightness = 4;

	if ((err = i2c_attach_client(client))) {
		//FIXME: Free allocated data!!
		printk("htcwizcpld: Cannot attach client!\n");
		return err;
	}

	return 0;
}

static int htcwizcpld_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, htcwizcpld_detect);
}

static int htcwizcpld_detach_client(struct i2c_client *client)
{
	//FIXME: OMG WE ARE GOING TO DIE!!
	i2c_detach_client(client);

	return 0;
}

static int __init htcwizcpld_init(void)
{
	return i2c_add_driver(&htcwizcpld_driver);
}

static void __exit htcwizcpld_exit(void)
{
	i2c_del_driver(&htcwizcpld_driver);
}

module_init(htcwizcpld_init);
module_exit(htcwizcpld_exit);

MODULE_AUTHOR("Angelo Arrifano <miknix@gmail.com>");
MODULE_DESCRIPTION("HTC I2C CPLD Driver");
MODULE_LICENSE("GPL");

