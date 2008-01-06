/*
 * Modified from board-perseus2.c and htcwizard.c
 *
 * HTC Wizard init stuff
 * Copyright (C) 2006 Unai Uribarri
 * Copyright (C) 2008 linwizard.sourceforge.net
 *
 * This  program is  free  software; you  can  redistribute it  and/or
 * modify  it under the  terms of  the GNU  General Public  License as
 * published by the Free Software  Foundation; either version 2 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT  ANY  WARRANTY;  without   even  the  implied  warranty  of
 * MERCHANTABILITY or  FITNESS FOR A PARTICULAR PURPOSE.   See the GNU
 * General Public License for more details.
 * 
 * You should have  received a copy of the  GNU General Public License
 * along  with  this program;  if  not,  write  to the  Free  Software
 * Foundation,  Inc.,  51 Franklin  Street,  Fifth  Floor, Boston,  MA
 * 02110-1301, USA. 
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/arch/omap850.h>
#include <asm/page.h>
#include <asm/memory.h>
#include <asm/arch/common.h>
#include <asm/arch/board.h>
#include <asm/arch/io.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/keypad.h>

#include <linux/delay.h>

/* Keyboard definition */

static int htc_wizard_keymap[] = {
	KEY(0,0,KEY_RECORD),
	KEY(0,1,KEY_CAMERA),
	KEY(0,2,KEY_PHONE),
	KEY(0,3,KEY_VOLUMEUP),
	KEY(0,4,KEY_F2),
	KEY(0,5,KEY_MAIL),
	KEY(0,6,KEY_DIRECTORY),
	KEY(1,0,KEY_MENU),
	KEY(1,1,KEY_COMMA),
	KEY(1,2,KEY_M),
	KEY(1,3,KEY_K),
	KEY(1,4,KEY_OK),
	KEY(1,5,KEY_I),
	KEY(1,6,KEY_U),
	KEY(2,0,KEY_RED), // FIX ME
	KEY(2,1,KEY_TAB),
	KEY(2,2,KEY_N),
	KEY(2,3,KEY_J),
	KEY(2,4,KEY_ENTER),
	KEY(2,5,KEY_H),
	KEY(2,6,KEY_Z),
	KEY(3,0,KEY_SPACE),
	KEY(3,1,KEY_L),
	KEY(3,2,KEY_B),
	KEY(3,3,KEY_V),
	KEY(3,4,KEY_BACKSPACE),
	KEY(3,5,KEY_G),
	KEY(3,6,KEY_T),
	KEY(4,0,KEY_CAPSLOCK),
	KEY(4,1,KEY_C),
	KEY(4,2,KEY_F),
	KEY(4,3,KEY_R),
	KEY(4,4,KEY_O),
	KEY(4,5,KEY_E),
	KEY(4,6,KEY_D),
	KEY(5,0,KEY_X),
	KEY(5,1,KEY_Y),
	KEY(5,2,KEY_S),
	KEY(5,3,KEY_W),
	KEY(5,4,KEY_P),
	KEY(5,5,KEY_Q),
	KEY(5,6,KEY_A),
	KEY(6,0,KEY_CONNECT),
	KEY(6,2,KEY_CANCEL),
	KEY(6,3,KEY_VOLUMEDOWN),
	KEY(6,4,KEY_F1),
	KEY(6,5,KEY_WWW),
	KEY(6,6,KEY_CALENDAR),
	0
};

struct omap_kp_platform_data kp_data = {
	.rows	= 8,
	.cols	= 8,
	.delay = 9,
	.keymap = htc_wizard_keymap,
};

static struct resource kp_resources[] = {
	[0] = {
		.start	= INT_850_MPUIO_KEYPAD,
		.end	= INT_850_MPUIO_KEYPAD,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device kp_device = {
	.name		= "omap-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &kp_data,
	},
	.num_resources	= ARRAY_SIZE(kp_resources),
	.resource	= kp_resources,
};


/* LCD Device resources */
static struct platform_device lcd_device = {
	.name		= "lcd_htcwizard",
	.id		= -1,
};


static struct platform_device *devices[] __initdata = {
/* 	&gsm_device, */
	&kp_device,
	&lcd_device,
};

/* Init functions from here on */

static void __init htcwizard_map_io(void)
{
	omap1_map_common_io();
	printk("htcwizard_map_io done.\n");
}

static void __init htcwizard_init_irq(void)
{
	printk("htcwizard_init_irq.\n");
	omap1_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
}

static void __init htcwizard_disable_watchdog()
{
  /* Disable watchdog if running */
  if (omap_readl(OMAP_WDT_TIMER_MODE) & 0x8000) {
    /*
     * disable a potentially running watchdog timer before
     * it kills us.
     */
    printk("OMAP850 Watchdog seems to be activated, disabling it for now.\n");
    omap_writel(0xF5, OMAP_WDT_TIMER_MODE);
    omap_writel(0xA0, OMAP_WDT_TIMER_MODE);
  }
}

static void __init htcwizard_init(void)
{
  printk("HTC Wizard init.\n");
  platform_add_devices(devices, ARRAY_SIZE(devices));
  
  htcwizard_disable_watchdog();
}

MACHINE_START(OMAP_HTCWIZARD, "HTC Wizard")
        /* Maintainer: Unai Uribarri <unaiur@gmail.com> */
		  /* Maintainer: linwizard.sourceforge.net */
        .phys_io        = 0xfff00000,
        .io_pg_offst    = ((0xfef00000) >> 18) & 0xfffc,
        .boot_params    = 0x10000100,
        .map_io         = htcwizard_map_io,
        .init_irq       = htcwizard_init_irq,
        .init_machine   = htcwizard_init,
        .timer          = &omap_timer,
MACHINE_END
