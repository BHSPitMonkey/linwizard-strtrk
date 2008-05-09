#ifndef __LINUX_I2C_HTCI2CCPLD_H
#define __LINUX_I2C_HTCI2CCPLD_H

#define HTCI2CCPLD_BL_LMAX 4

enum htci2ccpld_led_type {
	LED_RED,
	LED_ORANGE,
	LED_RGREEN,
	LED_LGREEN,
};

extern void htci2ccpld_bl_set(int value);
extern int htci2ccpld_bl_get(void);

extern void htci2ccpld_led_set(enum htci2ccpld_led_type led, bool value);
extern bool htci2ccpld_led_get(enum htci2ccpld_led_type led);

extern void htci2ccpld_rumble_set(bool status);
bool htci2ccpld_rumble_get(void);

#endif /* __LINUX_I2C_HTCI2CCPLD_H */
