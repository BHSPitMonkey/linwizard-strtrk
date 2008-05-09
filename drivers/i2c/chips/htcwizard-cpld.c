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

#include <linux/i2c/htcwizard-cpld.h>

#define HTCWIZCPLD_BL_OFF_MASK0 0xF1
#define HTCWIZCPLD_BL_L4_MASK1  0x0E
#define HTCWIZCPLD_BL_L3_MASK1  0x0A
#define HTCWIZCPLD_BL_L2_MASK1  0x0C
#define HTCWIZCPLD_BL_L1_MASK1  0x08

#define HTCWIZCPLD_LCD_WHITE_MASK0 0x0F
#define HTCWIZCPLD_LCD_FB_MASK1    0xF0

#define HTCWIZCPLD_LLED_OFF_MASK0   0xDF
#define HTCWIZCPLD_LLED_GREEN_MASK1 0x20

#define HTCWIZCPLD_RLED_OFF_MASK0    0xF1
#define HTCWIZCPLD_RLED_GREEN_MASK1  0x08
#define HTCWIZCPLD_RLED_RED_MASK1    0x04
#define HTCWIZCPLD_RLED_ORANGE_MASK1 0x0C

#define HTCWIZCPLD_CHIP_RST(C,M) \
	htcwizcpld_chip##C.cmd &= M

#define HTCWIZCPLD_CHIP_SET(C,M) \
	htcwizcpld_chip##C.cmd |= M


/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x03, 0x04, 0x05, 0x06, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1(htcwizard_cpld);

/* 
 * Chip structures
 */

struct htcwizcpld_chip_data {
	struct i2c_client client;
	u8 cmd;
};

static struct htcwizcpld_chip_data htcwizcpld_chip3;
static struct htcwizcpld_chip_data htcwizcpld_chip4;
static struct htcwizcpld_chip_data htcwizcpld_chip5;
static struct htcwizcpld_chip_data htcwizcpld_chip6;



/*
 * Chip management
 */

static void htcwizcpld_chip_update(struct htcwizcpld_chip_data *chip)
{
	i2c_smbus_read_byte_data(&chip->client, chip->cmd);
}

static int htcwizcpld_chip3_init(void)
{
	return 0;
}

static int htcwizcpld_chip4_init(void)
{
	/* Enable FB output on LCD */
	HTCWIZCPLD_CHIP_SET(4, HTCWIZCPLD_LCD_FB_MASK1);

	/* Set backlight level to 3 */
	HTCWIZCPLD_CHIP_RST(4, HTCWIZCPLD_BL_OFF_MASK0);
	HTCWIZCPLD_CHIP_SET(4, HTCWIZCPLD_BL_L3_MASK1);

	htcwizcpld_chip_update(&htcwizcpld_chip4);
	return 0;
}

static int htcwizcpld_chip5_init(void)
{
	/* Turn off left led and turn on right green led */
	HTCWIZCPLD_CHIP_RST(5, HTCWIZCPLD_LLED_OFF_MASK0);
	HTCWIZCPLD_CHIP_RST(5, HTCWIZCPLD_RLED_OFF_MASK0);
	HTCWIZCPLD_CHIP_SET(5, HTCWIZCPLD_RLED_GREEN_MASK1);

	htcwizcpld_chip_update(&htcwizcpld_chip5);
	return 0;
}

static int htcwizcpld_chip6_init(void)
{
	return 0;
}

/*
 * Function Interface
 */

int htcwizcpld_bl_set(int value)
{
	HTCWIZCPLD_CHIP_RST(4, HTCWIZCPLD_BL_OFF_MASK0);

	switch (value) {
		case 1:
			HTCWIZCPLD_CHIP_SET(4, HTCWIZCPLD_BL_L1_MASK1);
			break;
		case 2:
			HTCWIZCPLD_CHIP_SET(4, HTCWIZCPLD_BL_L2_MASK1);
			break;
		case 3:
			HTCWIZCPLD_CHIP_SET(4, HTCWIZCPLD_BL_L3_MASK1);
			break;
		case 4:
			HTCWIZCPLD_CHIP_SET(4, HTCWIZCPLD_BL_L4_MASK1);
			break;
	}
	
	htcwizcpld_chip_update(&htcwizcpld_chip4);
	return 0;
}
EXPORT_SYMBOL(htcwizcpld_bl_set);

int htcwizcpld_bl_get(void)
{
	return 0;
}
EXPORT_SYMBOL(htcwizcpld_bl_get);

int htcwizcpld_led_set(enum htcwizcpld_led_type led, bool value)
{

	return 0;
}
EXPORT_SYMBOL(htcwizcpld_led_set);


int htcwizcpld_led_get(enum htcwizcpld_led_type led)
{
	return 0;
}
EXPORT_SYMBOL(htcwizcpld_led_get);

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
	struct i2c_client *client = NULL;
	int ret;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BYTE_DATA))
		return -ENODEV;

	switch (address) {
		case 0x03:
			client = &htcwizcpld_chip3.client;
			i2c_set_clientdata(client, &htcwizcpld_chip3);
			strlcpy(client->name, "htcwizcpld_chip3", I2C_NAME_SIZE);
			break;
		case 0x04:
			client = &htcwizcpld_chip4.client;
			i2c_set_clientdata(client, &htcwizcpld_chip4);
			strlcpy(client->name, "htcwizcpld_chip4", I2C_NAME_SIZE);
			break;
		case 0x05:
			client = &htcwizcpld_chip5.client;
			i2c_set_clientdata(client, &htcwizcpld_chip5);
			strlcpy(client->name, "htcwizcpld_chip5", I2C_NAME_SIZE);
			break;
		case 0x06:
			client = &htcwizcpld_chip6.client;
			i2c_set_clientdata(client, &htcwizcpld_chip6);
			strlcpy(client->name, "htcwizcpld_chip6", I2C_NAME_SIZE);
			break;
		default:
			return -ENODEV;
	}

	client->addr = address;
	client->adapter = adapter;
	client->driver = &htcwizcpld_driver;

	if ((ret = i2c_attach_client(client))) {
		printk("htcwizcpld: Cannot attach client!\n");
		return ret;
	}

	switch (address) {
		case 0x03:
			ret = htcwizcpld_chip3_init();
			break;
		case 0x04:
			ret = htcwizcpld_chip4_init();
			break;
		case 0x05:
			ret = htcwizcpld_chip5_init();
			break;
		case 0x06:
			ret = htcwizcpld_chip6_init();
			break;
	}

	if (ret) {
		i2c_detach_client(client);
		return ret;
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

