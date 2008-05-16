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
#include <linux/spinlock.h>

#include <linux/i2c/htc-i2c-cpld.h>

/* Some macros to easy bit handling */
#define HTCI2CCPLD_CHIP_RST_GRP(X) HTCI2CCPLD_CHIP_RST_GRP_E(X)
#define HTCI2CCPLD_CHIP_SET(X)     HTCI2CCPLD_CHIP_SET_E(X)
#define HTCI2CCPLD_CHIP_ISSET(X)    HTCI2CCPLD_CHIP_ISSET_E(X)
#define HTCI2CCPLD_CHIP_ISNSET(X,Y) HTCI2CCPLD_CHIP_ISNSET_E(X,Y)

#define HTCI2CCPLD_CHIP_RST_GRP_E(C,B,M) \
	(htci2ccpld_chip##C.cmd &= ~M)

#define HTCI2CCPLD_CHIP_SET_E(C,B,M) \
	(htci2ccpld_chip##C.cmd |= B)

#define HTCI2CCPLD_CHIP_ISSET_E(C,B,M) \
	(((htci2ccpld_chip##C.cmd & M) ^ B) == 0)

#define HTCI2CCPLD_CHIP_ISNSET_E(C,B,M,V) \
	((V | B) != V)

/* Enable if building for htc herald */
/* #define HTC_IS_HERALD */

/*     function                 chip, bit , group mask */
#define HTCI2CCPLD_BIT_DKB_DOWN    3, 0x10, 0xF0
#define HTCI2CCPLD_BIT_DKB_LEFT    3, 0x20, 0xF0
#define HTCI2CCPLD_BIT_DKB_UP      3, 0x40, 0xF0
#define HTCI2CCPLD_BIT_DKB_RIGHT   3, 0x80, 0xF0

#define HTCI2CCPLD_BIT_DPAD_BL     3, 0x01, 0x01

#define HTCI2CCPLD_BIT_SOUND       3, 0x08, 0x08

#define HTCI2CCPLD_BIT_DPAD_ENTER  4, 0x08, 0xF8
#define HTCI2CCPLD_BIT_DPAD_DOWN   4, 0x10, 0xF8
#define HTCI2CCPLD_BIT_DPAD_LEFT   4, 0x20, 0xF8
#define HTCI2CCPLD_BIT_DPAD_UP     4, 0x40, 0xF8
#define HTCI2CCPLD_BIT_DPAD_RIGHT  4, 0x80, 0xF8

#define HTCI2CCPLD_BIT_BL_L4       4, 0x0E, 0x0E
#define HTCI2CCPLD_BIT_BL_L3       4, 0x0A, 0x0E
#define HTCI2CCPLD_BIT_BL_L2       4, 0x0C, 0x0E
#define HTCI2CCPLD_BIT_BL_L1       4, 0x08, 0x0E

#define HTCI2CCPLD_BIT_KB_BL       4, 0x01, 0x01

#ifdef HTC_IS_HERALD
/* On Herald chip4 bit5 enables CAPS LED */
# define HTCI2CCPLD_BIT_LCD_FB     4, 0xD0, 0xD0
#else
# define HTCI2CCPLD_BIT_LCD_FB     4, 0xF0, 0xF0
#endif

/* This one doesnt turn on when usb is off */
#define HTCI2CCPLD_BIT_LLED_GREEN  5, 0x20, 0x20

/* We force the charge bit to enable leds
   even with usb off */
#define HTCI2CCPLD_BIT_RLED_GREEN  5, 0x18, 0x1E
#define HTCI2CCPLD_BIT_RLED_RED    5, 0x14, 0x1E
#define HTCI2CCPLD_BIT_RLED_ORANGE 5, 0x1C, 0x1E

#define HTCI2CCPLD_BIT_DPAD_OFF    5, 0x01, 0x01

#ifdef HTC_IS_HERALD
# define HTCI2CCPLD_BIT_LED_FN     6, 0x10, 0x10
#endif

#define HTCI2CCPLD_BIT_VIBRATOR    6, 0x08, 0x08

#define HTCI2CCPLD_BIT_LED_CAMERA  6, 0x20, 0x20

#define HTCI2CCPLD_BIT_DALL_OFF    6, 0x02, 0x02

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
	spinlock_t lock;
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

static u8 htci2ccpld_chip_update(struct htci2ccpld_chip_data *chip);

/*
 * Interface functions
 */

void htci2ccpld_vibrator_set(bool status)
{
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_VIBRATOR);
	if (status)
		HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_VIBRATOR);

	htci2ccpld_chip_update(&htci2ccpld_chip6);
}
EXPORT_SYMBOL(htci2ccpld_vibrator_set);


bool htci2ccpld_vibrator_get(void)
{
	return HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_VIBRATOR);
}
EXPORT_SYMBOL(htci2ccpld_vibrator_get);


void htci2ccpld_bl_set(int value)
{
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_BL_L1);

	switch (value) {
		case 1:
			HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_BL_L1);
			break;
		case 2:
			HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_BL_L2);
			break;
		case 3:
			HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_BL_L3);
			break;
		case 4:
			HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_BL_L4);
			break;
	}
	
	htci2ccpld_chip_update(&htci2ccpld_chip4);
}
EXPORT_SYMBOL(htci2ccpld_bl_set);


int htci2ccpld_bl_get(void)
{
	if (HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_BL_L1))
		return 1;
	if (HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_BL_L2))
		return 2;
	if (HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_BL_L3))
		return 3;
	if (HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_BL_L4))
		return 4;

	return 0;
}
EXPORT_SYMBOL(htci2ccpld_bl_get);

void htci2ccpld_led_set(enum htci2ccpld_led_type led, bool value)
{
	switch (led) {
		case HTCI2CCPLD_LED_RED:
			HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_RLED_RED);
			if (value)
				HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_RLED_RED);
			break;
		case HTCI2CCPLD_LED_ORANGE:
			HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_RLED_ORANGE);
			if (value)
				HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_RLED_ORANGE);
			break;
		case HTCI2CCPLD_LED_RGREEN:
			HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_RLED_GREEN);
			if (value)
				HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_RLED_GREEN);
			break;
		case HTCI2CCPLD_LED_LGREEN:
			HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_LLED_GREEN);
			if (value)
				HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_LLED_GREEN);
			break;
		case HTCI2CCPLD_LED_CAMERA:
			HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_LED_CAMERA);
			if (value)
				HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_LED_CAMERA);
			break;
		case HTCI2CCPLD_LED_KB:
			HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_KB_BL);
			if (value)
				HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_KB_BL);
			break;
		case HTCI2CCPLD_LED_DPAD:
			HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_DPAD_BL);
			if (value)
				HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_DPAD_BL);
			break;
		case HTCI2CCPLD_LED_FN:
#ifdef HTC_IS_HERALD
			HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_LED_FN);
			if (value)
				HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_LED_FN);
#endif
			break;
		default:
			HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_RLED_GREEN);
			HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_LLED_GREEN);
	}

	htci2ccpld_chip_update(&htci2ccpld_chip5);
}
EXPORT_SYMBOL(htci2ccpld_led_set);


bool htci2ccpld_led_get(enum htci2ccpld_led_type led)
{
	switch (led) {
		case HTCI2CCPLD_LED_RED:
			return HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_RLED_RED);
			break;
		case HTCI2CCPLD_LED_ORANGE:
			return HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_RLED_ORANGE);
			break;
		case HTCI2CCPLD_LED_RGREEN:
			return HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_RLED_GREEN);
			break;
		case HTCI2CCPLD_LED_LGREEN:
			return HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_LLED_GREEN);
			break;
		case HTCI2CCPLD_LED_CAMERA:
			return HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_LED_CAMERA);
			break;
		case HTCI2CCPLD_LED_KB:
			return HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_KB_BL);
			break;
		case HTCI2CCPLD_LED_DPAD:
			return HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_DPAD_BL);
			break;
		case HTCI2CCPLD_LED_FN:
#ifdef HTC_IS_HERALD
			return HTCI2CCPLD_CHIP_ISSET(HTCI2CCPLD_BIT_LED_FN);
#endif
			break;
	}
	return 0;
}
EXPORT_SYMBOL(htci2ccpld_led_get);


int htci2ccpld_btn_get(enum htci2ccpld_btn_chip bchip)
{
	u8 val;
	int status = 0;

	if (bchip == HTCI2CCPLD_BTN_CHIP_DKB) {
		val = htci2ccpld_chip_update(&htci2ccpld_chip3);
		if (HTCI2CCPLD_CHIP_ISNSET(HTCI2CCPLD_BIT_DKB_UP, val))
			status |= HTCI2CCPLD_BTN_UP;

		if (HTCI2CCPLD_CHIP_ISNSET(HTCI2CCPLD_BIT_DKB_DOWN, val))
			status |= HTCI2CCPLD_BTN_DOWN;

		if (HTCI2CCPLD_CHIP_ISNSET(HTCI2CCPLD_BIT_DKB_LEFT, val))
			status |= HTCI2CCPLD_BTN_LEFT;

		if (HTCI2CCPLD_CHIP_ISNSET(HTCI2CCPLD_BIT_DKB_RIGHT, val))
			status |= HTCI2CCPLD_BTN_RIGHT;
		
		/* Check for unhandled keys */
		if ((! status) && (val < 0xff))
			printk("htc-i2c-cpld: Unhandled event %x on chip 3.\n", val);

		return status;
	}
	
	val = htci2ccpld_chip_update(&htci2ccpld_chip4);
	
	if (HTCI2CCPLD_CHIP_ISNSET(HTCI2CCPLD_BIT_DPAD_ENTER, val))
		status |= HTCI2CCPLD_BTN_ENTER;

	if (HTCI2CCPLD_CHIP_ISNSET(HTCI2CCPLD_BIT_DPAD_UP, val))
		status |= HTCI2CCPLD_BTN_UP;

	if (HTCI2CCPLD_CHIP_ISNSET(HTCI2CCPLD_BIT_DPAD_DOWN, val))
		status |= HTCI2CCPLD_BTN_DOWN;

	if (HTCI2CCPLD_CHIP_ISNSET(HTCI2CCPLD_BIT_DPAD_LEFT, val))
		status |= HTCI2CCPLD_BTN_LEFT;

	if (HTCI2CCPLD_CHIP_ISNSET(HTCI2CCPLD_BIT_DPAD_RIGHT, val))
		status |= HTCI2CCPLD_BTN_RIGHT;

	/* Check for unhandled keys */
	if ((! status) && (val < 0xff))
		printk("htc-i2c-cpld: Unhandled event %x on chip 4.\n", val);

	return status;
}
EXPORT_SYMBOL(htci2ccpld_btn_get);


/*
 * Chip management
 */

static u8 htci2ccpld_chip_update(struct htci2ccpld_chip_data *chip)
{
	return i2c_smbus_read_byte_data(&chip->client, chip->cmd);
}

static int htci2ccpld_chip3_init(void)
{
	/* Enable all, this will mess someone */
	htci2ccpld_chip3.cmd = 0xff;

	/* Disable sound */
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_SOUND);

	/* Disable DPAD backlight */
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_DPAD_BL);

	htci2ccpld_chip_update(&htci2ccpld_chip3);

	return 0;
}

static int htci2ccpld_chip4_init(void)
{
	/* Enable FB output on LCD */
	HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_LCD_FB);

	/* Set backlight level to 3 */
	htci2ccpld_bl_set(1);
	
	return 0;
}

static int htci2ccpld_chip5_init(void)
{
	/* Enable all, this will mess someone */
	htci2ccpld_chip5.cmd = 0xff;
	
	/* Turn off all known leds */
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_LLED_GREEN);
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_RLED_GREEN);

	/* Turn on right green led */
	HTCI2CCPLD_CHIP_SET(HTCI2CCPLD_BIT_RLED_GREEN);

	/* Enable DPAD */
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_DPAD_OFF);

	htci2ccpld_chip_update(&htci2ccpld_chip5);

	return 0;
}

static int htci2ccpld_chip6_init(void)
{
	/* Enable all, this will mess someone */
	htci2ccpld_chip6.cmd = 0xff;

	/* Disable camera led */
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_LED_CAMERA);

	/* Disable vibrator */
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_VIBRATOR);

	/* Enable dkb and dpad */
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_DALL_OFF);

#ifdef HTC_IS_HERALD
	/* Disable Fn LED */
	HTCI2CCPLD_CHIP_RST_GRP(HTCI2CCPLD_BIT_LED_FN);
#endif
	htci2ccpld_chip_update(&htci2ccpld_chip6);

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

