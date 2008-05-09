/*
 *  htc-i2c-cpld.c
 *  Chip driver for a unknown cpld found on HTC boards with omap850.
 *  The cpld is located on the i2c bus and controls backlight, leds,
 *  rumble and other power devices. The cpld also returns buttons status
 *  of the directional pads found on this HTC devices.
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

#include <linux/i2c/htc-i2c-cpld.h>

#define HTCI2CCPLD_BL_OFF_MASK0 0xF1
#define HTCI2CCPLD_BL_L4_MASK1  0x0E
#define HTCI2CCPLD_BL_L3_MASK1  0x0A
#define HTCI2CCPLD_BL_L2_MASK1  0x0C
#define HTCI2CCPLD_BL_L1_MASK1  0x08

#define HTCI2CCPLD_LCD_WHITE_MASK0 0x0F
#define HTCI2CCPLD_LCD_FB_MASK1    0xF0

#define HTCI2CCPLD_LLED_OFF_MASK0   0xDF
#define HTCI2CCPLD_LLED_GREEN_MASK1 0x20

#define HTCI2CCPLD_RLED_OFF_MASK0    0xF1
#define HTCI2CCPLD_RLED_GREEN_MASK1  0x08
#define HTCI2CCPLD_RLED_RED_MASK1    0x04
#define HTCI2CCPLD_RLED_ORANGE_MASK1 0x0C

#define HTCI2CCPLD_RUMBLE_OFF_MASK0 0XF7
#define HTCI2CCPLD_RUMBLE_ON_MASK1  0X08 

#define HTCI2CCPLD_CHIP_RST(C,M) \
	htci2ccpld_chip##C.cmd &= M

#define HTCI2CCPLD_CHIP_SET(C,M) \
	htci2ccpld_chip##C.cmd |= M

#define HTCI2CCPLD_CHIP_CHK(C,M) \
	htci2ccpld_chip##C.cmd == (htci2ccpld_chip##C.cmd | M)

/* Addresses to scan */
static const unsigned short normal_i2c[] = {
	0x03, 0x04, 0x05, 0x06, I2C_CLIENT_END
};

I2C_CLIENT_INSMOD_1(htc_i2c_cpld);

/* 
 * Chip structures
 */

struct htci2ccpld_chip_data {
	struct i2c_client client;
	u8 cmd;
};

/*
 * FIXME: Depending on the bus driver
 * the chip count can start in 1
 */
static struct htci2ccpld_chip_data htci2ccpld_chip3;
static struct htci2ccpld_chip_data htci2ccpld_chip4;
static struct htci2ccpld_chip_data htci2ccpld_chip5;
static struct htci2ccpld_chip_data htci2ccpld_chip6;

/*
 * Chip management
 */

static void htci2ccpld_chip_update(struct htci2ccpld_chip_data *chip)
{
	i2c_smbus_read_byte_data(&chip->client, chip->cmd);
}

static int htci2ccpld_chip3_init(void)
{
	return 0;
}

static int htci2ccpld_chip4_init(void)
{
	/* Enable FB output on LCD */
	HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_LCD_FB_MASK1);

	/* Set backlight level to 3 */
	HTCI2CCPLD_CHIP_RST(4, HTCI2CCPLD_BL_OFF_MASK0);
	HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_BL_L3_MASK1);

	htci2ccpld_chip_update(&htci2ccpld_chip4);
	return 0;
}

static int htci2ccpld_chip5_init(void)
{
	/* Turn off left led and turn on right green led */
	HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_LLED_OFF_MASK0);
	HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_RLED_OFF_MASK0);
	HTCI2CCPLD_CHIP_SET(5, HTCI2CCPLD_RLED_GREEN_MASK1);

	htci2ccpld_chip_update(&htci2ccpld_chip5);
	return 0;
}

static int htci2ccpld_chip6_init(void)
{
	return 0;
}

/*
 * Interface functions
 */

void htci2ccpld_rumble_set(bool status)
{
	HTCI2CCPLD_CHIP_RST(6, HTCI2CCPLD_RUMBLE_OFF_MASK0);
	if (status)
		HTCI2CCPLD_CHIP_SET(6, HTCI2CCPLD_RUMBLE_ON_MASK1);
}
EXPORT_SYMBOL(htci2ccpld_rumble_set);


bool htci2ccpld_rumble_get(void)
{
	return HTCI2CCPLD_CHIP_CHK(6, HTCI2CCPLD_RUMBLE_ON_MASK1);
}
EXPORT_SYMBOL(htci2ccpld_rumble_get);


void htci2ccpld_bl_set(int value)
{
	HTCI2CCPLD_CHIP_RST(4, HTCI2CCPLD_BL_OFF_MASK0);

	switch (value) {
		case 1:
			HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_BL_L1_MASK1);
			break;
		case 2:
			HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_BL_L2_MASK1);
			break;
		case 3:
			HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_BL_L3_MASK1);
			break;
		case 4:
			HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_BL_L4_MASK1);
			break;
	}
	
	htci2ccpld_chip_update(&htci2ccpld_chip4);
}
EXPORT_SYMBOL(htci2ccpld_bl_set);


int htci2ccpld_bl_get(void)
{
	if (HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_BL_L1_MASK1));
		return 1;
	if (HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_BL_L2_MASK1));
		return 2;
	if (HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_BL_L3_MASK1));
		return 3;
	if (HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_BL_L4_MASK1));
		return 4;

	return 0;
}
EXPORT_SYMBOL(htci2ccpld_bl_get);

void htci2ccpld_led_set(enum htci2ccpld_led_type led, bool value)
{
	switch (led) {
		case LED_RED:
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_RLED_OFF_MASK0);
			if (value)
				HTCI2CCPLD_CHIP_SET(5, HTCI2CCPLD_RLED_RED_MASK1);
			break;
		case LED_ORANGE:
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_RLED_OFF_MASK0);
			if (value)
				HTCI2CCPLD_CHIP_SET(5, HTCI2CCPLD_RLED_ORANGE_MASK1);
			break;
		case LED_RGREEN:
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_RLED_OFF_MASK0);
			if (value)
				HTCI2CCPLD_CHIP_SET(5, HTCI2CCPLD_RLED_GREEN_MASK1);
			break;
		case LED_LGREEN:
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_LLED_OFF_MASK0);
			if (value)
				HTCI2CCPLD_CHIP_SET(5, HTCI2CCPLD_LLED_GREEN_MASK1);
		default:
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_RLED_OFF_MASK0);
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_LLED_OFF_MASK0);
	}
}
EXPORT_SYMBOL(htci2ccpld_led_set);


bool htci2ccpld_led_get(enum htci2ccpld_led_type led)
{
	switch (led) {
		case LED_RED:
			return HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_RLED_RED_MASK1);
			break;
		case LED_ORANGE:
			return HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_RLED_ORANGE_MASK1);
			break;
		case LED_RGREEN:
			return HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_RLED_GREEN_MASK1);
			break;
		case LED_LGREEN:
			return HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_LLED_GREEN_MASK1);
			break;
	}
	return 0;
}
EXPORT_SYMBOL(htci2ccpld_led_get);

/*
 * Driver handling
 */
static int htci2ccpld_attach_adapter(struct i2c_adapter *adapter);
static int htci2ccpld_detach_client(struct i2c_client *client);

static struct i2c_driver htci2ccpld_driver = {
	.driver = {
		.name	= "htci2ccpld",
	},
	.attach_adapter	= htci2ccpld_attach_adapter,
	.detach_client	= htci2ccpld_detach_client,
};

static int htci2ccpld_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *client = NULL;
	int ret;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BYTE_DATA))
		return -ENODEV;

	switch (address) {
		case 0x03:
			client = &htci2ccpld_chip3.client;
			i2c_set_clientdata(client, &htci2ccpld_chip3);
			strlcpy(client->name, "htci2ccpld_chip3", I2C_NAME_SIZE);
			break;
		case 0x04:
			client = &htci2ccpld_chip4.client;
			i2c_set_clientdata(client, &htci2ccpld_chip4);
			strlcpy(client->name, "htci2ccpld_chip4", I2C_NAME_SIZE);
			break;
		case 0x05:
			client = &htci2ccpld_chip5.client;
			i2c_set_clientdata(client, &htci2ccpld_chip5);
			strlcpy(client->name, "htci2ccpld_chip5", I2C_NAME_SIZE);
			break;
		case 0x06:
			client = &htci2ccpld_chip6.client;
			i2c_set_clientdata(client, &htci2ccpld_chip6);
			strlcpy(client->name, "htci2ccpld_chip6", I2C_NAME_SIZE);
			break;
		default:
			return -ENODEV;
	}

	client->addr = address;
	client->adapter = adapter;
	client->driver = &htci2ccpld_driver;

	if ((ret = i2c_attach_client(client))) {
		printk("htci2ccpld: Cannot attach client!\n");
		return ret;
	}

	switch (address) {
		case 0x03:
			ret = htci2ccpld_chip3_init();
			break;
		case 0x04:
			ret = htci2ccpld_chip4_init();
			break;
		case 0x05:
			ret = htci2ccpld_chip5_init();
			break;
		case 0x06:
			ret = htci2ccpld_chip6_init();
			break;
	}

	if (ret) {
		i2c_detach_client(client);
		return ret;
	}

	return 0;
}

static int htci2ccpld_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, htci2ccpld_detect);
}

static int htci2ccpld_detach_client(struct i2c_client *client)
{
	i2c_detach_client(client);

	return 0;
}

static int __init htci2ccpld_init(void)
{
	return i2c_add_driver(&htci2ccpld_driver);
}

static void __exit htci2ccpld_exit(void)
{
	i2c_del_driver(&htci2ccpld_driver);
}

module_init(htci2ccpld_init);
module_exit(htci2ccpld_exit);

MODULE_AUTHOR("Angelo Arrifano <miknix@gmail.com>");
MODULE_DESCRIPTION("HTC I2C CPLD Driver");
MODULE_LICENSE("GPL");

