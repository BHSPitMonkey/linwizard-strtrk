#ifndef __LINUX_I2C_HTCI2CCPLD_H
#define __LINUX_I2C_HTCI2CCPLD_H

#define HTCI2CCPLD_BL_LMAX 4

/* FIXME: Change LED_ to HTCI2CCPLD_LED
   to prevent conflicts */
enum htci2ccpld_led_type {
	HTCI2CCPLD_LED_RED,
	HTCI2CCPLD_LED_ORANGE,
	HTCI2CCPLD_LED_RGREEN,
	HTCI2CCPLD_LED_LGREEN,
	HTCI2CCPLD_LED_FN,
	HTCI2CCPLD_LED_CAMERA,
	HTCI2CCPLD_LED_DPAD,
	HTCI2CCPLD_LED_KB,
};

enum htci2ccpld_btn_chip {
	HTCI2CCPLD_BTN_CHIP_DPAD,
	HTCI2CCPLD_BTN_CHIP_DKB,
};

enum htci2ccpld_btn {
	HTCI2CCPLD_BTN_LEFT  = 0x01,
	HTCI2CCPLD_BTN_RIGHT = 0x02,
	HTCI2CCPLD_BTN_UP    = 0x04,
	HTCI2CCPLD_BTN_DOWN  = 0x08,
	HTCI2CCPLD_BTN_ENTER = 0x10,
};

extern int htci2ccpld_btn_get(enum htci2ccpld_btn_chip bchip);

extern void htci2ccpld_bl_set(int value);
extern int htci2ccpld_bl_get(void);

extern void htci2ccpld_led_set(enum htci2ccpld_led_type led, bool value);
extern bool htci2ccpld_led_get(enum htci2ccpld_led_type led);

extern void htci2ccpld_vibrator_set(bool status);
bool htci2ccpld_vibrator_get(void);

#endif /* __LINUX_I2C_HTCI2CCPLD_H */
