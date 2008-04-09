/*
 * linux/arch/arm/mach-omap2/board-apollon.c
 *
 * Copyright (C) 2005,2006 Samsung Electronics
 * Author: Kyungmin Park <kyungmin.park@samsung.com>
 *
 * Modified from mach-omap2/board-h4.c
 *
 * Code for apollon OMAP2 board. Should work on many OMAP2 systems where
 * the bootloader passes the board-specific data to the kernel.
 * Do not put any board specific code to this file; create a new machine
 * type if you need custom low-level initializations.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/onenand.h>
#include <linux/delay.h>
#include <linux/leds.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>

#include <asm/arch/gpio.h>
#include <asm/arch/led.h>
#include <asm/arch/mux.h>
#include <asm/arch/usb.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/gpmc.h>
#include "prcm-regs.h"

/* LED & Switch macros */
#define LED0_GPIO13		13
#define LED1_GPIO14		14
#define LED2_GPIO15		15
#define LED3_GPIO92		92
#define LED4_GPIO93		93

#define APOLLON_FLASH_CS	0
#define APOLLON_ETH_CS		1

#define APOLLON_FLASH_CS	0
#define APOLLON_ETH_CS		1

static struct mtd_partition apollon_partitions[] = {
	{
		.name		= "X-Loader + U-Boot",
		.offset		= 0,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
	},
	{
		.name		= "rootfs",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_16M,
	},
	{
		.name		= "filesystem00",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_32M,
	},
	{
		.name		= "filesystem01",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data apollon_flash_data = {
	.parts		= apollon_partitions,
	.nr_parts	= ARRAY_SIZE(apollon_partitions),
};

static struct resource apollon_flash_resource[] = {
	[0] = {
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device apollon_onenand_device = {
	.name		= "onenand",
	.id		= -1,
	.dev		= {
		.platform_data	= &apollon_flash_data,
	},
	.num_resources	= ARRAY_SIZE(apollon_flash_resource),
	.resource	= apollon_flash_resource,
};

static void __init apollon_flash_init(void)
{
	unsigned long base;

	if (gpmc_cs_request(APOLLON_FLASH_CS, SZ_128K, &base) < 0) {
		printk(KERN_ERR "Cannot request OneNAND GPMC CS\n");
		return;
	}
	apollon_flash_resource[0].start = base;
	apollon_flash_resource[0].end   = base + SZ_128K - 1;
}

static struct resource apollon_smc91x_resources[] = {
	[0] = {
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start	= OMAP_GPIO_IRQ(APOLLON_ETHR_GPIO_IRQ),
		.end	= OMAP_GPIO_IRQ(APOLLON_ETHR_GPIO_IRQ),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device apollon_smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(apollon_smc91x_resources),
	.resource	= apollon_smc91x_resources,
};

static struct platform_device apollon_lcd_device = {
	.name		= "apollon_lcd",
	.id		= -1,
};

static struct omap_led_config apollon_led_config[] = {
	{
		.cdev	= {
			.name	= "apollon:led0",
		},
		.gpio	= LED0_GPIO13,
	},
	{
		.cdev	= {
			.name	= "apollon:led1",
		},
		.gpio	= LED1_GPIO14,
	},
	{
		.cdev	= {
			.name	= "apollon:led2",
		},
		.gpio	= LED2_GPIO15,
	},
};

static struct omap_led_platform_data apollon_led_data = {
	.nr_leds	= ARRAY_SIZE(apollon_led_config),
	.leds		= apollon_led_config,
};

static struct platform_device apollon_led_device = {
	.name		= "omap-led",
	.id		= -1,
	.dev		= {
		.platform_data	= &apollon_led_data,
	},
};

static struct platform_device *apollon_devices[] __initdata = {
	&apollon_onenand_device,
	&apollon_smc91x_device,
	&apollon_lcd_device,
	&apollon_led_device,
};

static inline void __init apollon_init_smc91x(void)
{
	unsigned long base;

	/* Make sure CS1 timings are correct */
	GPMC_CONFIG1_1 = 0x00011203;
	GPMC_CONFIG2_1 = 0x001f1f01;
	GPMC_CONFIG3_1 = 0x00080803;
	GPMC_CONFIG4_1 = 0x1c091c09;
	GPMC_CONFIG5_1 = 0x041f1f1f;
	GPMC_CONFIG6_1 = 0x000004c4;

	if (gpmc_cs_request(APOLLON_ETH_CS, SZ_16M, &base) < 0) {
		printk(KERN_ERR "Failed to request GPMC CS for smc91x\n");
		return;
	}
	apollon_smc91x_resources[0].start = base + 0x300;
	apollon_smc91x_resources[0].end   = base + 0x30f;
	udelay(100);

	omap_cfg_reg(W4__24XX_GPIO74);
	if (omap_request_gpio(APOLLON_ETHR_GPIO_IRQ) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc91x IRQ\n",
			APOLLON_ETHR_GPIO_IRQ);
		gpmc_cs_free(APOLLON_ETH_CS);
		return;
	}
	omap_set_gpio_direction(APOLLON_ETHR_GPIO_IRQ, 1);
}

static void __init omap_apollon_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	apollon_init_smc91x();
}

static struct tsc210x_config tsc_platform_data = {
	.use_internal		= 1,
	.monitor		= TSC_TEMP,
	.mclk			= "sys_clkout",
};

static struct spi_board_info apollon_spi_board_info[] = {
	[0] = {
		.modalias	= "tsc2101",
		.irq		= OMAP_GPIO_IRQ(85),
		.bus_num	= 1,
		.chip_select	= 0,
		.mode		= SPI_MODE_1,
		.max_speed_hz	= 6000000,
		.platform_data	= &tsc_platform_data,
	},
};

static struct omap_uart_config apollon_uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (0 << 1) | (0 << 2),
};

static struct omap_mmc_config apollon_mmc_config __initdata = {
	.mmc [0] = {
		.enabled 	= 1,
		.wire4		= 1,
	/* Use internal loop-back in MMC/SDIO Module Input Clock selection */
		.internal_clock	= 1,
		.wp_pin		= -1,
		.power_pin	= -1,
	/* Note: If you want to detect card feature, please assign 37 */
		.switch_pin	= -1,
	},
};

static struct omap_usb_config apollon_usb_config __initdata = {
	.register_dev	= 1,
	.hmc_mode	= 0x14,	/* 0:dev 1:host1 2:disable */

	.pins[0]	= 6,
};

static struct omap_lcd_config apollon_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_board_config_kernel apollon_config[] __initdata = {
	{ OMAP_TAG_UART,	&apollon_uart_config },
	{ OMAP_TAG_MMC,		&apollon_mmc_config },
	{ OMAP_TAG_USB,		&apollon_usb_config },
	{ OMAP_TAG_LCD,		&apollon_lcd_config },
};

static void __init apollon_led_init(void)
{
	/* LED0 - AA10 */
	omap_cfg_reg(AA10_242X_GPIO13);
	omap_request_gpio(LED0_GPIO13);
	omap_set_gpio_direction(LED0_GPIO13, 0);
	omap_set_gpio_dataout(LED0_GPIO13, 0);
	/* LED1  - AA6 */
	omap_cfg_reg(AA6_242X_GPIO14);
	omap_request_gpio(LED1_GPIO14);
	omap_set_gpio_direction(LED1_GPIO14, 0);
	omap_set_gpio_dataout(LED1_GPIO14, 0);
	/* LED2  - AA4 */
	omap_cfg_reg(AA4_242X_GPIO15);
	omap_request_gpio(LED2_GPIO15);
	omap_set_gpio_direction(LED2_GPIO15, 0);
	omap_set_gpio_dataout(LED2_GPIO15, 0);
#ifdef CONFIG_MACH_OMAP_APOLLON_PLUS
	/* LED3 - M15 */
	omap_cfg_reg(M15_24XX_GPIO92);
	omap_request_gpio(LED3_GPIO92);
	omap_set_gpio_direction(LED3_GPIO92, 0);
	omap_set_gpio_dataout(LED3_GPIO92, 0);
	/* LED4 - P20 */
	omap_cfg_reg(P20_24XX_GPIO93);
	omap_request_gpio(LED4_GPIO93);
	omap_set_gpio_direction(LED4_GPIO93, 0);
	omap_set_gpio_dataout(LED4_GPIO93, 0);
#endif
}

static void __init apollon_usb_init(void)
{
	/* USB device */
	/* DEVICE_SUSPEND */
	omap_cfg_reg(P21_242X_GPIO12);
	omap_request_gpio(12);
	omap_set_gpio_direction(12, 0);		/* OUT */
	omap_set_gpio_dataout(12, 0);
}

static void __init apollon_tsc_init(void)
{
	/* TSC2101 */
	omap_cfg_reg(N15_24XX_GPIO85);
	omap_request_gpio(85);
	omap_set_gpio_direction(85, 1);

	omap_cfg_reg(W14_24XX_SYS_CLKOUT);	/* mclk */
}

static void __init apollon_usb_init(void)
{
	/* USB device */
	/* DEVICE_SUSPEND */
	omap_cfg_reg(P21_242X_GPIO12);
	omap_request_gpio(12);
	omap_set_gpio_direction(12, 0);		/* OUT */
	omap_set_gpio_dataout(12, 0);
}

static void __init omap_apollon_init(void)
{
	apollon_led_init();
	apollon_sw_init();
	apollon_flash_init();
	apollon_usb_init();

	/* REVISIT: where's the correct place */
	omap_cfg_reg(W19_24XX_SYS_NIRQ);

	/* Use Interal loop-back in MMC/SDIO Module Input Clock selection */
	CONTROL_DEVCONF |= (1 << 24);

	/*
 	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on
	 * if not needed.
	 */
	platform_add_devices(apollon_devices, ARRAY_SIZE(apollon_devices));
	omap_board_config = apollon_config;
	omap_board_config_size = ARRAY_SIZE(apollon_config);
	omap_serial_init();

	spi_register_board_info(apollon_spi_board_info,
				ARRAY_SIZE(apollon_spi_board_info));
}

static void __init omap_apollon_map_io(void)
{
	omap2_map_common_io();
}

MACHINE_START(OMAP_APOLLON, "OMAP24xx Apollon")
	/* Maintainer: Kyungmin Park <kyungmin.park@samsung.com> */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_apollon_map_io,
	.init_irq	= omap_apollon_init_irq,
	.init_machine	= omap_apollon_init,
	.timer		= &omap_timer,
MACHINE_END
