/*
 * linux/arch/arm/mach-omap2/mux.c
 *
 * OMAP1 pin multiplexing configurations
 *
 * Copyright (C) 2003 - 2005 Nokia Corporation
 *
 * Written by Tony Lindgren <tony.lindgren@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/spinlock.h>

#include <asm/arch/mux.h>

#ifdef CONFIG_OMAP_MUX

struct pin_config __initdata_or_module omap24xx_pins[] = {
/*
 *            description             mux       mux   pull  pull  debug
 *                                    offset    mode  ena   type
 */

/* 24xx I2C */
OMAP2_MUX_CFG("M19_24XX_I2C1_SCL",    0x111,    0,    0,    0,    1)
OMAP2_MUX_CFG("L15_24XX_I2C1_SDA",    0x112,    0,    0,    0,    1)
OMAP2_MUX_CFG("J15_24XX_I2C2_SCL",    0x113,    0,    0,    0,    1)
OMAP2_MUX_CFG("H19_24XX_I2C2_SDA",    0x114,    0,    0,    0,    1)
OMAP2_MUX_CFG("W19_24XX_SYS_NIRQ",    0x12c,    0,    1,    1,    1)
};

int __init omap2_mux_init(void)
{
	omap_mux_register(omap24xx_pins, ARRAY_SIZE(omap24xx_pins));
	return 0;
}

#endif