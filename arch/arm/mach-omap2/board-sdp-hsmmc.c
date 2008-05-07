/*
 * linux/arch/arm/mach-omap2/board-sdp-hsmmc.c
 *
 * Copyright (C) 2007 Texas Instruments
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c/twl4030.h>
#include <asm/hardware.h>
#include <asm/arch/mmc.h>
#include <asm/arch/board.h>

#if defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

#define VMMC1_DEV_GRP		0x27
#define P1_DEV_GRP		0x20
#define VMMC1_DEDICATED		0x2A
#define VSEL_3V			0x02
#define VSEL_18V		0x00
#define TWL_GPIO_PUPDCTR1	0x13
#define TWL_GPIO_IMR1A		0x1C
#define TWL_GPIO_ISR1A		0x19
#define LDO_CLR			0x00
#define VSEL_S2_CLR		0x40
#define GPIO_0_BIT_POS		(1 << 0)
#define MMC1_CD_IRQ		0
#define MMC2_CD_IRQ		1

#define OMAP2_CONTROL_DEVCONF0	0x48002274
#define OMAP2_CONTROL_DEVCONF1	0x490022E8

#define OMAP2_CONTROL_DEVCONF0_LBCLK	(1 << 24)
#define OMAP2_CONTROL_DEVCONF1_ACTOV	(1 << 31)

#define OMAP2_CONTROL_PBIAS_VMODE	(1 << 0)
#define OMAP2_CONTROL_PBIAS_PWRDNZ	(1 << 1)
#define OMAP2_CONTROL_PBIAS_SCTRL	(1 << 2)

static int sdp_mmc_card_detect(int irq)
{
	return twl4030_get_gpio_datain(irq - IH_TWL4030_GPIO_BASE);
}

/*
 * MMC Slot Initialization.
 */
static int sdp_mmc_late_init(struct device *dev)
{
	int ret = 0;

	/*
	 * Configure TWL4030 GPIO parameters for MMC hotplug irq
	 */
	ret = twl4030_request_gpio(MMC1_CD_IRQ);
	if (ret)
		goto err;

	ret = twl4030_set_gpio_edge_ctrl(MMC1_CD_IRQ,
			TWL4030_GPIO_EDGE_RISING | TWL4030_GPIO_EDGE_FALLING);
	if (ret)
		goto err;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x02,
						TWL_GPIO_PUPDCTR1);
	if (ret)
		goto err;

	ret = twl4030_set_gpio_debounce(MMC1_CD_IRQ, TWL4030_GPIO_IS_ENABLE);
	if (ret)
		goto err;

	return ret;

err:
	dev_err(dev, "Failed to configure TWL4030 GPIO IRQ\n");
	return ret;
}

static void sdp_mmc_cleanup(struct device *dev)
{
	int ret = 0;

	ret = twl4030_free_gpio(MMC1_CD_IRQ);
	if (ret)
		dev_err(dev, "Failed to configure TWL4030 GPIO IRQ\n");
}

#ifdef CONFIG_PM

/*
 * To mask and unmask MMC Card Detect Interrupt
 * mask : 1
 * unmask : 0
 */
static int mask_cd_interrupt(int mask)
{
	u8 reg = 0, ret = 0;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg, TWL_GPIO_IMR1A);
	if (ret)
		goto err;

	reg = (mask == 1) ? (reg | GPIO_0_BIT_POS) : (reg & ~GPIO_0_BIT_POS);

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, reg, TWL_GPIO_IMR1A);
	if (ret)
		goto err;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg, TWL_GPIO_ISR1A);
	if (ret)
		goto err;

	reg = (mask == 1) ? (reg | GPIO_0_BIT_POS) : (reg & ~GPIO_0_BIT_POS);

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, reg, TWL_GPIO_ISR1A);
	if (ret)
		goto err;

err:
	return ret;
}

static int sdp_mmc_suspend(struct device *dev, int slot)
{
	int ret = 0;

	disable_irq(TWL4030_GPIO_IRQ_NO(MMC1_CD_IRQ));
	ret = mask_cd_interrupt(1);

	return ret;
}

static int sdp_mmc_resume(struct device *dev, int slot)
{
	int ret = 0;

	enable_irq(TWL4030_GPIO_IRQ_NO(MMC1_CD_IRQ));
	ret = mask_cd_interrupt(0);

	return ret;
}

#endif

static int sdp_mmc_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	u32 vdd_sel = 0, devconf = 0, reg = 0;
	int ret = 0;

	/* REVISIT: Using address directly till the control.h defines
	 * are settled.
	 */
#if defined(CONFIG_ARCH_OMAP2430)
	#define OMAP2_CONTROL_PBIAS 0x490024A0
#else
	#define OMAP2_CONTROL_PBIAS 0x48002520
#endif

	if (power_on) {
		if (cpu_is_omap24xx())
			devconf = omap_readl(OMAP2_CONTROL_DEVCONF1);
		else
			devconf = omap_readl(OMAP2_CONTROL_DEVCONF0);

		switch (1 << vdd) {
		case MMC_VDD_33_34:
		case MMC_VDD_32_33:
			vdd_sel = VSEL_3V;
			if (cpu_is_omap24xx())
				devconf |= OMAP2_CONTROL_DEVCONF1_ACTOV;
			break;
		case MMC_VDD_165_195:
			vdd_sel = VSEL_18V;
			if (cpu_is_omap24xx())
				devconf &= ~OMAP2_CONTROL_DEVCONF1_ACTOV;
		}

		if (cpu_is_omap24xx())
			omap_writel(devconf, OMAP2_CONTROL_DEVCONF1);
		else
			omap_writel(devconf | OMAP2_CONTROL_DEVCONF0_LBCLK,
				    OMAP2_CONTROL_DEVCONF0);

		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg |= OMAP2_CONTROL_PBIAS_SCTRL;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg &= ~OMAP2_CONTROL_PBIAS_PWRDNZ;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						P1_DEV_GRP, VMMC1_DEV_GRP);
		if (ret)
			goto err;

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						vdd_sel, VMMC1_DEDICATED);
		if (ret)
			goto err;

		msleep(100);
		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg |= (OMAP2_CONTROL_PBIAS_SCTRL |
			OMAP2_CONTROL_PBIAS_PWRDNZ);
		if (vdd_sel == VSEL_18V)
			reg &= ~OMAP2_CONTROL_PBIAS_VMODE;
		else
			reg |= OMAP2_CONTROL_PBIAS_VMODE;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		return ret;

	} else {
		/* Power OFF */

		/* For MMC1, Toggle PBIAS before every power up sequence */
		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg &= ~OMAP2_CONTROL_PBIAS_PWRDNZ;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						LDO_CLR, VMMC1_DEV_GRP);
		if (ret)
			goto err;

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						VSEL_S2_CLR, VMMC1_DEDICATED);
		if (ret)
			goto err;

		/* 100ms delay required for PBIAS configuration */
		msleep(100);
		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg |= (OMAP2_CONTROL_PBIAS_VMODE |
			OMAP2_CONTROL_PBIAS_PWRDNZ |
			OMAP2_CONTROL_PBIAS_SCTRL);
		omap_writel(reg, OMAP2_CONTROL_PBIAS);
	}

	return 0;

err:
	return 1;
}

static struct omap_mmc_platform_data sdp_mmc_data = {
	.nr_slots			= 1,
	.switch_slot			= NULL,
	.init				= sdp_mmc_late_init,
	.cleanup			= sdp_mmc_cleanup,
#ifdef CONFIG_PM
	.suspend			= sdp_mmc_suspend,
	.resume				= sdp_mmc_resume,
#endif
	.slots[0] = {
		.set_power		= sdp_mmc_set_power,
		.set_bus_mode		= NULL,
		.get_ro			= NULL,
		.get_cover_state	= NULL,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 |
						MMC_VDD_165_195,
		.name			= "first slot",

		.card_detect_irq        = TWL4030_GPIO_IRQ_NO(MMC1_CD_IRQ),
		.card_detect            = sdp_mmc_card_detect,
	},
};

void __init sdp_mmc_init(void)
{
	omap_set_mmc_info(1, &sdp_mmc_data);
}

#else

void __init sdp_mmc_init(void)
{

}

#endif
