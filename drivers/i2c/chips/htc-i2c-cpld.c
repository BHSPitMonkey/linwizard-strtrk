/*
 *  htc-i2c-cpld.c
 *  Chip driver for a unknown cpld found on HTC boards with omap850.
 *  The cpld is located on the i2c bus and controls backlight, leds,
 *  vibrator and other power devices. The cpld also returns buttons status
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

/* TODO:
 * HTC Herald: Chip6 Bit4 enabled Fn LED.
 */

/* Enable if building for htc herald */
/* #define HTC_IS_HERALD */

#define HTCI2CCPLD_BL_MASK   0x0E
#define HTCI2CCPLD_BL_L4_CMD 0x0E
#define HTCI2CCPLD_BL_L3_CMD 0x0A
#define HTCI2CCPLD_BL_L2_CMD 0x0C
#define HTCI2CCPLD_BL_L1_CMD 0x08

#ifdef HTC_IS_HERALD
/* On Herald chip4 bit5 enables CAPS LED */
# define HTCI2CCPLD_LCD_MASK   0xD0
# define HTCI2CCPLD_LCD_FB_CMD 0xD0
#else
# define HTCI2CCPLD_LCD_MASK   0xF0
# define HTCI2CCPLD_LCD_FB_CMD 0xF0
#endif

#define HTCI2CCPLD_LLED_MASK      0x20
#define HTCI2CCPLD_LLED_GREEN_CMD 0x20

#define HTCI2CCPLD_RLED_MASK       0x0E
#define HTCI2CCPLD_RLED_GREEN_CMD  0x08
#define HTCI2CCPLD_RLED_RED_CMD    0x04
#define HTCI2CCPLD_RLED_ORANGE_CMD 0x0C

#define HTCI2CCPLD_RUMBLE_MASK 0X08
#define HTCI2CCPLD_RUMBLE_CMD  0X08 

#define HTCI2CCPLD_CHIP_RST(C,M) \
	(htci2ccpld_chip##C.cmd &= ~M)

#define HTCI2CCPLD_CHIP_SET(C,D) \
	(htci2ccpld_chip##C.cmd |= D)

#define HTCI2CCPLD_CHIP_CHK(C,D,M) \
	(((htci2ccpld_chip##C.cmd & M) ^ D) == 0)

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

static void htci2ccpld_chip_update(struct htci2ccpld_chip_data *chip);

/*
 * Interface functions
 */

void htci2ccpld_vibrator_set(bool status)
{
	HTCI2CCPLD_CHIP_RST(6, HTCI2CCPLD_RUMBLE_MASK);
	if (status)
		HTCI2CCPLD_CHIP_SET(6, HTCI2CCPLD_RUMBLE_CMD);

	htci2ccpld_chip_update(&htci2ccpld_chip6);
}
EXPORT_SYMBOL(htci2ccpld_vibrator_set);


bool htci2ccpld_vibrator_get(void)
{
	return HTCI2CCPLD_CHIP_CHK(6, HTCI2CCPLD_RUMBLE_CMD,
	                              HTCI2CCPLD_RUMBLE_MASK);
}
EXPORT_SYMBOL(htci2ccpld_vibrator_get);


void htci2ccpld_bl_set(int value)
{
	HTCI2CCPLD_CHIP_RST(4, HTCI2CCPLD_BL_MASK);

	switch (value) {
		case 1:
			HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_BL_L1_CMD);
			break;
		case 2:
			HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_BL_L2_CMD);
			break;
		case 3:
			HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_BL_L3_CMD);
			break;
		case 4:
			HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_BL_L4_CMD);
			break;
	}
	
	htci2ccpld_chip_update(&htci2ccpld_chip4);
}
EXPORT_SYMBOL(htci2ccpld_bl_set);


int htci2ccpld_bl_get(void)
{
	if (HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_BL_L1_CMD, HTCI2CCPLD_BL_MASK))
		return 1;
	if (HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_BL_L2_CMD, HTCI2CCPLD_BL_MASK))
		return 2;
	if (HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_BL_L3_CMD, HTCI2CCPLD_BL_MASK))
		return 3;
	if (HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_BL_L4_CMD, HTCI2CCPLD_BL_MASK))
		return 4;

	return 0;
}
EXPORT_SYMBOL(htci2ccpld_bl_get);

void htci2ccpld_led_set(enum htci2ccpld_led_type led, bool value)
{
	switch (led) {
		case LED_RED:
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_RLED_MASK);
			if (value)
				HTCI2CCPLD_CHIP_SET(5, HTCI2CCPLD_RLED_RED_CMD);
			break;
		case LED_ORANGE:
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_RLED_MASK);
			if (value)
				HTCI2CCPLD_CHIP_SET(5, HTCI2CCPLD_RLED_ORANGE_CMD);
			break;
		case LED_RGREEN:
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_RLED_MASK);
			if (value)
				HTCI2CCPLD_CHIP_SET(5, HTCI2CCPLD_RLED_GREEN_CMD);
			break;
		case LED_LGREEN:
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_LLED_MASK);
			if (value)
				HTCI2CCPLD_CHIP_SET(5, HTCI2CCPLD_LLED_GREEN_CMD);
			break;
		default:
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_RLED_MASK);
			HTCI2CCPLD_CHIP_RST(5, HTCI2CCPLD_LLED_MASK);
	}

	htci2ccpld_chip_update(&htci2ccpld_chip5);
}
EXPORT_SYMBOL(htci2ccpld_led_set);


bool htci2ccpld_led_get(enum htci2ccpld_led_type led)
{
	switch (led) {
		case LED_RED:
			return HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_RLED_RED_CMD,
			                              HTCI2CCPLD_RLED_MASK);
			break;
		case LED_ORANGE:
			return HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_RLED_ORANGE_CMD,
			                              HTCI2CCPLD_RLED_MASK);
			break;
		case LED_RGREEN:
			return HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_RLED_GREEN_CMD,
			                              HTCI2CCPLD_RLED_MASK);
			break;
		case LED_LGREEN:
			return HTCI2CCPLD_CHIP_CHK(4, HTCI2CCPLD_LLED_GREEN_CMD,
			                              HTCI2CCPLD_LLED_MASK);
			break;
	}
	return 0;
}
EXPORT_SYMBOL(htci2ccpld_led_get);


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
	HTCI2CCPLD_CHIP_SET(4, HTCI2CCPLD_LCD_FB_CMD);

	/* Set backlight level to 3 */
	htci2ccpld_bl_set(1);
	
	return 0;
}

static int htci2ccpld_chip5_init(void)
{
	/* Turn off left led and turn on right green led */
	htci2ccpld_led_set(LED_LGREEN, 0);
	htci2ccpld_led_set(LED_RGREEN, 1);

	return 0;
}

static int htci2ccpld_chip6_init(void)
{
	return 0;
}


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

	printk("htc-i2c-cpld: Detected chip %x\n", address);

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

