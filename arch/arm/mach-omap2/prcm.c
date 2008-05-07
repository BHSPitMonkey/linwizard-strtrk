/*
 * linux/arch/arm/mach-omap2/prcm.c
 *
 * OMAP 24xx Power Reset and Clock Management (PRCM) functions
 *
 * Copyright (C) 2005 Nokia Corporation
 *
 * Written by Tony Lindgren <tony.lindgren@nokia.com>
 *
 * Some pieces of code Copyright (C) 2005 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>

#include "prm.h"
#include "prm-regbits-24xx.h"

extern void omap2_clk_prepare_for_reboot(void);

u32 omap_prcm_get_reset_sources(void)
{
	/* XXX This presumably needs modification for 34XX */
	return prm_read_mod_reg(WKUP_MOD, RM_RSTST) & 0x7f;
}
EXPORT_SYMBOL(omap_prcm_get_reset_sources);

/* Resets clock rates and reboots the system. Only called from system.h */
void omap_prcm_arch_reset(char mode)
{
	s16 prcm_offs;
	omap2_clk_prepare_for_reboot();

	if (cpu_is_omap24xx()) {
		prcm_offs = WKUP_MOD;
	} else if (cpu_is_omap34xx()) {
		prcm_offs = OMAP3430_GR_MOD;
	} else {
		WARN_ON(1);
	}

	prm_set_mod_reg_bits(OMAP_RST_DPLL3, prcm_offs, RM_RSTCTRL);
}
