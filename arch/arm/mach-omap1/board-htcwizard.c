/* This is a (rough) modification of board-htcwizard.c in order to
 * make it support the HTC Star Trek (htcstrtrk).
 *
 * Modified from board-htcwizard.c, which was in turn modified from
 * board-perseus2.c and htcwizard.c
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
#include <linux/bootmem.h>
#include <linux/gpio_keys.h>

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
#include <linux/i2c-gpio.h>

#include <linux/delay.h>

#include <linux/i2c.h>
#include <linux/i2c/tps65010.h>

#define HTCWIZARD_GPIO_DM 35
#define HTCWIZARD_GPIO_DP 36

#define HTCWIZARD_GIRQ_BTNS 141



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

/* GSM device */
#define WIZARD_GSM_PHYS_START          0x13c00000 /* end of RAM */
#define WIZARD_GSM_PHYS_SIZE           0x00400000 /* 4.0 MB code/data/fifo */

static struct resource wizard_gsm_resources[] = {
	{       /* GSM DSP MMU */
                .start          = IO_ADDRESS(OMAP850_DSP_MMU_BASE),
                .end            = IO_ADDRESS(OMAP850_DSP_MMU_BASE) + 0x54,
                .flags          = IORESOURCE_MEM,
        },
        {       /* GSM software interrupt */
                .start          = INT_850_ICR,
                .flags          = IORESOURCE_IRQ,
        },
        {       /* GSM radio interrupt */
                .start          = INT_850_DBB_RF_EN,
                .flags          = IORESOURCE_IRQ,
        },
};

static struct platform_device gsm_device = {
        .name           = "wizard-gsm",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(wizard_gsm_resources),
        .resource       = wizard_gsm_resources,
};

/* Keyboard definition for HTC StarTrek */
/* Inserted hackishly by BHSPitMonkey */

static int htc_startrek_keymap[] = {
/** Still need to figure out KEY_RECORD on a less broken unit than mine. */
	KEY(0,0,KEY_A),	// 5
	KEY(0,1,KEY_RECORD),	// Record
	KEY(0,2,KEY_NEXTSONG),	// External Next
	KEY(0,3,KEY_PREVIOUSSONG),	// External Prev
	KEY(0,5,KEY_F1),	// Left Softkey
	KEY(1,0,KEY_KP0),	// 0
	KEY(1,1,KEY_H),		// Home key - FIXME: Should be better
	KEY(1,2,KEY_PLAYPAUSE),	// External Play
	KEY(1,3,KEY_VOLUMEDOWN),	// Volume down
	KEY(1,5,KEY_F2),	// Right Softkey
	KEY(2,1,KEY_BACKSPACE),	// Backspace
	KEY(2,2,KEY_CAMERA),	// Camera
	KEY(2,3,KEY_VOLUMEUP),	// Volume up
	KEY(2,5,KEY_KPASTERISK),	// Star * / T9
	KEY(3,0,KEY_R),	// 1
	KEY(3,1,KEY_T),	// 3
	KEY(3,2,KEY_KP7),	// 7
	KEY(3,3,KEY_KP9),	// 9
	KEY(3,5,KEY_SPACE),	// Pound # / Space
	KEY(4,0,KEY_O),	// 2
	KEY(4,1,KEY_S),	// 4
	KEY(4,2,KEY_X),	// 6
	KEY(4,3,KEY_KP8),	// 8
	0
};

struct omap_kp_platform_data kp_data = {
	.rows	= 7,
	.cols	= 7,
	.delay = 20,
	.keymap = htc_startrek_keymap,
};


/*
 * GPIO Keys
 */

static struct gpio_keys_button htc_strtrk_button_table[] = {
	{KEY_ESC,        70,    0, "Hangup button"},
//	{KEY_CAMERA,     GPIO91_MAGICIAN_KEY_CAMERA,    0, "Camera button"},
	{KEY_UP,         136,        0, "Up button"},
	{KEY_DOWN,       137,      0, "Down button"},
	{KEY_LEFT,       139,      0, "Left button"},
	{KEY_RIGHT,      19,     0, "Right button"},
	{KEY_KPENTER,    76,     0, "Action button"},
//	{KEY_RECORD,     GPIO98_MAGICIAN_KEY_RECORD,    0, "Record button"},
//	{KEY_VOLUMEUP,   GPIO100_MAGICIAN_KEY_VOL_UP,   0, "Volume up"},
//	{KEY_VOLUMEDOWN, GPIO101_MAGICIAN_KEY_VOL_DOWN, 0, "Volume down"},
//	{KEY_PHONE,      GPIO102_MAGICIAN_KEY_PHONE,    0, "Phone button"}
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons  = htc_strtrk_button_table,
	.nbuttons = ARRAY_SIZE(htc_strtrk_button_table),
};

static struct platform_device gpio_keys = {
	.name = "gpio-keys",
	.dev  = {
		.platform_data = &gpio_keys_data,
	},
	.id   = -1,
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

#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)

static struct i2c_gpio_platform_data i2cgpio_device_data = {
	.sda_pin		= 69,
	.scl_pin		= 70,
	.udelay			= 2,		/* ~100 kHz */
};

static struct platform_device i2cgpio_device = {
	.name			= "i2c-gpio",
	.id			= 0,
	.dev	 = {
		.platform_data	= &i2cgpio_device_data,
	},
};

#endif

static struct platform_device led_device = {
	.name		= "htc-i2c-cpld-led",
	.id		= -1,
};

static struct resource btns_resources[] = {
	[0] = {
		.start	= OMAP_GPIO_IRQ(HTCWIZARD_GIRQ_BTNS),
		.end	= OMAP_GPIO_IRQ(HTCWIZARD_GIRQ_BTNS),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device btns_device = {
	.name		= "htc-i2c-cpld-btns",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(btns_resources),
	.resource	= btns_resources,
};

static struct platform_device *devices[] __initdata = {
 	&gsm_device,
	&kp_device,
	&lcd_device,
#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
	&i2cgpio_device,
#endif
	&led_device,
	&btns_device,
	&gpio_keys,
};

/*
 * Touchscreen
 
static struct tsc2046_platform_data htcwizard_ts_platform_data __initdata = {
        .ts_x_plate_ohm    = 496, 
        .dav_gpio          = 76,
        .gpio_debounce     = 0,
	.ts_max_pressure   = 100000,
        .ts_touch_pressure = 5000, 
};

static struct spi_board_info htcwizard_spi_board_info[] __initdata = { 
	{
 	       .modalias               = "tsc2046",
 	       .platform_data          = &htcwizard_ts_platform_data,
	       .max_speed_hz           = 2500000,
	       .bus_num                = 2,
	       .chip_select            = 1,
	       .irq		       = OMAP_GPIO_IRQ(76),
	} 
}; */
/* LCD register definition (maybe to move somewhere else */
#define       OMAP_LCDC_CONTROL               (0xfffec000 + 0x00)
#define       OMAP_LCDC_STATUS                (0xfffec000 + 0x10)
#define       OMAP_DMA_LCD_CCR                (0xfffee300 + 0xc2)
#define       OMAP_DMA_LCD_CTRL               (0xfffee300 + 0xc4)
#define       OMAP_LCDC_CTRL_LCD_EN           (1 << 0)
#define       OMAP_LCDC_STAT_DONE             (1 << 0)
static void htcwizard_lcd_init(void)
{
        u32 reg;
        /* disable controller if active */
        reg = omap_readl(OMAP_LCDC_CONTROL);
        if (reg & OMAP_LCDC_CTRL_LCD_EN) {
                reg &= ~OMAP_LCDC_CTRL_LCD_EN;
                omap_writel(reg, OMAP_LCDC_CONTROL);

                /* wait for end of frame */
                while (!(omap_readl(OMAP_LCDC_STATUS) & OMAP_LCDC_STAT_DONE));

                /* turn off DMA */
                reg = omap_readw(OMAP_DMA_LCD_CCR);
                reg &= ~(1 << 7);
                omap_writew(reg, OMAP_DMA_LCD_CCR);

                reg = omap_readw(OMAP_DMA_LCD_CTRL);
                reg &= ~(1 << 8);
                omap_writew(reg, OMAP_DMA_LCD_CTRL);
        }
}


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
	htcwizard_lcd_init();

	/* Reserve GSM Memory */
	reserve_bootmem(WIZARD_GSM_PHYS_START, WIZARD_GSM_PHYS_SIZE, 0);
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
#if defined(CONFIG_I2C_OMAP) || defined(CONFIG_I2C_OMAP_MODULE)
	/* Use I2C_SCK and I2_SDA */
	omap_writel(omap_readl(OMAP850_IO_CONF_5) & ~0x000000FF, OMAP850_IO_CONF_5);
	omap_register_i2c_bus(1, 100, NULL, 0);
#else
	/* Set GPIOS 70 and 69 */
	omap_writel(omap_readl(OMAP850_IO_CONF_5) |  0x000000CC, OMAP850_IO_CONF_5);
	omap_writel(omap_readl(OMAP850_IO_CONF_5) & ~0x00000033, OMAP850_IO_CONF_5);
#endif
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
//  spi_register_board_info(htcwizard_spi_board_info,
//			ARRAY_SIZE(htcwizard_spi_board_info));
  /* omap_free_gpio(76); */
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
