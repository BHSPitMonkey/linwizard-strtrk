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

#ifdef CONFIG_EFB_DEBUG
#include <asm/arch/efb.h>
#endif

#include <asm/arch/io.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/keypad.h>
#include <asm/arch/spi100k.h>

#include <linux/spi/spi.h>
#include <linux/spi/tsc2046.h>

#include <linux/delay.h>

#define ADS7846_PENDOWN_GPIO 76

#define HTCWIZARD_GPIO_DM 35
#define HTCWIZARD_GPIO_DP 36

static struct omap_lcd_config htcwizard_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_usb_config htcwizard_usb_config __initdata = {
	.otg		= 0,
	.register_host	= 0,
	.register_dev	= 1,
	.hmc_mode	= 4,
	.pins[0]	= 2,
};

static struct omap_mmc_config htcwizard_mmc_config __initdata =
{
	.mmc[0] = {
		.enabled = 1,
		.nomux = 1,
		.wire4 = 1,
		.power_pin = -1,
		.switch_pin = -1,
	}
};

static struct omap_board_config_kernel htcwizard_config[] = 
{
	{ OMAP_TAG_LCD, &htcwizard_lcd_config },
	{ OMAP_TAG_USB, &htcwizard_usb_config },
	{ OMAP_TAG_MMC, &htcwizard_mmc_config },
};
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
	KEY(1,4,KEY_SLASH), /* FIXME: Should be KEY_OK */
	KEY(1,5,KEY_I),
	KEY(1,6,KEY_U),
	/* Handle red button as alt key,
	 * and map keys on defkeymap */
	KEY(2,0,KEY_LEFTALT),
	KEY(2,1,KEY_TAB),
	KEY(2,2,KEY_N),
	KEY(2,3,KEY_J),
	KEY(2,4,KEY_ENTER),
	KEY(2,5,KEY_H),
	KEY(2,6,KEY_Y),
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
	KEY(5,1,KEY_Z),
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
	.rows	= 7,
	.cols	= 7,
	.delay = 10,
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

/*
 * Touchscreen
 */
static int ads7846_get_pendown_state(void)
{
	return !omap_get_gpio_datain(76);
}

static struct tsc2046_platform_data htcwizard_ts_platform_data __initdata = {
        .ts_x_plate_ohm    = 419,
        .dav_gpio          = 76,
        .gpio_debounce     = 0,
	.ts_max_pressure   = 10000,
        .ts_touch_pressure = 5000,
};

static struct spi_board_info htcwizard_spi_board_info[] __initdata = { 
	{
 	       .modalias               = "tsc2046",
 	       .platform_data          = &htcwizard_ts_platform_data,
	        .max_speed_hz           = 120000 /* max sample rate at 3V */
         	                                * 26 /* command + data + overhead */,
	        .bus_num                = 2,
		.chip_select            = 0,
	} 
};

/*
 * Init functions from here on
 */
static void __init htcwizard_map_io(void)
{
	omap1_map_common_io();
	printk("htcwizard_map_io done.\n");

#ifdef CONFIG_EFB_DEBUG
	efb_enable();
#endif
}

static void __init htcwizard_disable_watchdog(void)
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

/* TSC2046 init from board-nokia770.c */
static void ads7846_dev_init(void)
{
	if (omap_request_gpio(ADS7846_PENDOWN_GPIO) < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");
}

static void __init htcwizard_usb_enable(void)
{
	unsigned int tries = 20;
	printk("trying to enable USB.\n");

	/* force USB_EN GPIO to 0 */
	do {
		omap_set_gpio_direction(33, 0); /* output */
		omap_set_gpio_dataout(33, 0); /* low */
		--tries;
	} while(omap_get_gpio_datain(33) && tries);
	if (tries) {
		printk("unable to reset USB_EN GPIO after 20 tries.\n");
		printk("I will try to continue anyway: USB may not be available.\n");
	}
	printk("USB_EN to 0 after %i tries.\n", tries);

	omap_set_gpio_dataout(73, 0);
	
	omap_set_gpio_direction(HTCWIZARD_GPIO_DM, 1); /* input */
	
	/* get uart control from GSM */
	
	/* select GPIO35 for D_MCLK_OUT */
	/* select GPIO36 for D_CRESET */
	omap_writel(omap_readl(OMAP730_IO_CONF_3) & 0xffffffcc, OMAP730_IO_CONF_3);
	omap_writel(omap_readl(OMAP730_IO_CONF_3) | 0x000000cc, OMAP730_IO_CONF_3);
	

	omap_set_gpio_direction(HTCWIZARD_GPIO_DP, 1); /* input */

	/* select D_DM, D_DP for D_DM and disable PE_DM control */
	omap_writel(omap_readl(OMAP730_IO_CONF_2) & 0xff1fffff, OMAP730_IO_CONF_2);
	omap_writel(omap_readl(OMAP730_IO_CONF_2) | 0x00100000, OMAP730_IO_CONF_2);
	mdelay(100);

	/* select USB_VBUSI for D_VBUSI, enable PE_VIBUSI pull enable control  */
	omap_writel(omap_readl(OMAP730_IO_CONF_2) & 0xf1ffffff, OMAP730_IO_CONF_2);
	omap_writel(omap_readl(OMAP730_IO_CONF_2) | 0x01000000, OMAP730_IO_CONF_2);

	/* set USB_VBUS_CTRL */
	omap_writel(omap_readl(OMAP730_MODE_1) | (1 << 25), OMAP730_MODE_1);
}

static void __init htcwizard_usb_otg(void)
{
	/* clock configuration */
	omap_writew(omap_readw(ULPD_SOFT_REQ) | (1 << 8) | SOFT_USB_CLK_REQ, ULPD_SOFT_REQ);

	//  clk_enable(&l3_ocpi_ck);
	omap_writew(omap_readw(ARM_IDLECT3) | (1 << 0), ARM_IDLECT3);

	/* pin muxing */
	omap_writel(omap_readl(OMAP730_MODE_1) & ~(1 <<  2), OMAP730_MODE_1);
	omap_writel(omap_readl(OMAP730_MODE_1) & ~(1 <<  3), OMAP730_MODE_1);
	omap_writel(omap_readl(OMAP730_MODE_1) |  (1 << 15), OMAP730_MODE_1);
	omap_writel(omap_readl(OMAP730_MODE_1) |  (1 << 23), OMAP730_MODE_1);
	omap_writel(omap_readl(OMAP730_MODE_1) |  (1 << 26), OMAP730_MODE_1);
	omap_writel(omap_readl(OMAP730_MODE_1) |  (1 << 25), OMAP730_MODE_1);
	omap_writel(omap_readl(OMAP730_MODE_1) & ~(1 << 24), OMAP730_MODE_1);
	omap_writel(omap_readl(OMAP730_MODE_1) & ~(1 << 10), OMAP730_MODE_1);
	omap_writel(omap_readl(OMAP730_MODE_1) & ~(1 << 11), OMAP730_MODE_1);
}

static void __init htcwizard_spi_mux(void)
{
	/* Setup MUX config for SPI */
	omap_writel(omap_readl(OMAP850_IO_CONF_6) |  0x00088880, OMAP850_IO_CONF_6);
	omap_writel(omap_readl(OMAP850_IO_CONF_6) & ~0x00077770, OMAP850_IO_CONF_6);

	omap_writel(omap_readl(OMAP850_IO_CONF_8) |  0x01000000, OMAP850_IO_CONF_8);
	omap_writel(omap_readl(OMAP850_IO_CONF_8) & ~0x10110000, OMAP850_IO_CONF_8);

	omap_writel(omap_readl(OMAP850_IO_CONF_9) |  0x00000010, OMAP850_IO_CONF_9);
	omap_writel(omap_readl(OMAP850_IO_CONF_9) & ~0x00000001, OMAP850_IO_CONF_9);

	/* configure spi setup registers */
	omap_writew(0xfffe, OMAP850_SPI2_BASE + 0x02);
	omap_writew(0x0000, OMAP850_SPI2_BASE + 0x08);
	omap_writew(0x7ff8, OMAP850_SPI2_BASE + 0x0e);
}
static void __init htcwizard_i2c_init(void)
{
	/* Set pin mux for I2C */
	omap_writel(omap_readl(OMAP850_IO_CONF_5) & ~0x000000FF, OMAP850_IO_CONF_5);
	omap_register_i2c_bus(1, 100, NULL, 0);
}

static void __init htcwizard_init(void)
{
  printk("HTC Wizard init.\n");
  efb_putstr("HTC init");
  omap_board_config = htcwizard_config;
  omap_board_config_size = ARRAY_SIZE(htcwizard_config);
  platform_add_devices(devices, ARRAY_SIZE(devices));

  htcwizard_disable_watchdog();

  htcwizard_usb_otg();
  htcwizard_usb_enable();
  htcwizard_spi_mux();
  htcwizard_i2c_init();
  htcwizard_mmc_init();

  /* For testing.. Disable for now */
  spi_register_board_info(htcwizard_spi_board_info,
			ARRAY_SIZE(htcwizard_spi_board_info));
  
  ads7846_dev_init(); 

}

static void __init htcwizard_init_irq(void)
{
	printk("htcwizard_init_irq.\n");
	omap1_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
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
