#ifndef __LINUX_I2C_HTCWIZCPLD_H
#define __LINUX_I2C_HTCWIZCPLD_H

#define HTCWIZCPLD_BL_LMAX 4

enum htcwizcpld_led_type {
	LED_RED,
	LED_ORANGE,
	LED_LGREEN,
	LED_RGREEN,
};

extern int htcwizcpld_bl_set(int value);
extern int htcwizcpld_bl_get(void);

extern int htcwizcpld_led_set(enum htcwizcpld_led_type led, bool value);
extern int htcwizcpld_led_get(enum htcwizcpld_led_type led);

#endif /* __LINUX_I2C_HTCWIZCPLD_H */
