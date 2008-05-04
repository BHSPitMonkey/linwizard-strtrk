/*
 * linux/arch/arm/mach-omap1/board-htcwizard-mmc.c
 *
 * Copyright (C) 2007 Instituto Nokia de Tecnologia - INdT
 * Author: Felipe Balbi <felipe.lima@indt.org.br>
 *
 * This code is based on linux/arch/arm/mach-omap2/board-n800-mmc.c, which is:
 * Copyright (C) 2006 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/arch/mmc.h>
#include <asm/arch/gpio.h>

#include <linux/delay.h>

#ifdef CONFIG_MMC_OMAP
static int slot_cover_open;
static struct device *mmc_device;

static int htcwizard_mmc_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Set slot %d power: %s (vdd %d)\n", slot + 1,
		power_on ? "on" : "off", vdd);
#endif
	if (slot != 0) {
		dev_err(dev, "No such slot %d\n", slot + 1);
		return -ENODEV;
	}

	return 0;
}

static int htcwizard_mmc_set_bus_mode(struct device *dev, int slot, int bus_mode)
{
#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Set slot %d bus_mode %s\n", slot + 1,
		bus_mode == MMC_BUSMODE_OPENDRAIN ? "open-drain" : "push-pull");
#endif
	if (slot != 0) {
		dev_err(dev, "No such slot %d\n", slot + 1);
		return -ENODEV;
	}

	/* Treated on upper level */

	return bus_mode;
}

static int htcwizard_mmc_get_cover_state(struct device *dev, int slot)
{
	BUG_ON(slot != 0);

	return slot_cover_open;
}

static int htcwizard_mmc_late_init(struct device *dev)
{
	int ret = 0;

	mmc_device = dev;

	return ret;
}

static void htcwizard_mmc_cleanup(struct device *dev)
{
}

static struct omap_mmc_platform_data htcwizard_mmc_data = {
	.nr_slots                       = 1,
	.switch_slot                    = NULL,
	.init                           = htcwizard_mmc_late_init,
	.cleanup                        = htcwizard_mmc_cleanup,
	.slots[0]       = {
		.set_power              = htcwizard_mmc_set_power,
		.set_bus_mode           = htcwizard_mmc_set_bus_mode,
		.get_ro                 = NULL,
		.get_cover_state        = htcwizard_mmc_get_cover_state,
		.ocr_mask               = MMC_VDD_28_29 | MMC_VDD_30_31 |
					  MMC_VDD_32_33 | MMC_VDD_33_34,
		.name                   = "mmcblk",
	},
};

#endif

void __init htcwizard_mmc_init(void)
{
	/* Set MUX config for SDMC, having or not mmc driver */
	omap_writel(omap_readl(OMAP850_IO_CONF_2) & ~0x0008FF00, OMAP850_IO_CONF_2);
#ifdef CONFIG_MMC_OMAP
	omap_set_mmc_info(1, &htcwizard_mmc_data);
#endif
}
