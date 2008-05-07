/*
 * linux/arch/arm/plat-omap/sram.c
 *
 * OMAP SRAM detection and management
 *
 * Copyright (C) 2005 Nokia Corporation
 * Written by Tony Lindgren <tony@atomide.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#undef DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/tlb.h>
#include <asm/io.h>
#include <asm/cacheflush.h>

#include <asm/mach/map.h>

#include <asm/arch/sram.h>
#include <asm/arch/board.h>
#include <asm/arch/cpu.h>

#include <asm/arch/control.h>

#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)
# include "../mach-omap2/prm.h"
# include "../mach-omap2/cm.h"
# include "../mach-omap2/sdrc.h"
#endif

#ifdef CONFIG_ARCH_OMAP850
/* This is safer on wizard */
# define OMAP1_SRAM_PA		0x20000000
# define OMAP1_SRAM_VA		0xd0000000
#else
# define OMAP1_SRAM_PA		0x20000000
# define OMAP1_SRAM_VA		VMALLOC_END
#endif
#define OMAP2_SRAM_PA		0x40200000
#define OMAP2_SRAM_PUB_PA	0x4020f800
#define OMAP2_SRAM_VA		VMALLOC_END
#define OMAP2_SRAM_PUB_VA	(VMALLOC_END + 0x800)
#define OMAP3_SRAM_PA           0x40200000
#define OMAP3_SRAM_VA           0xd7000000
#define OMAP3_SRAM_PUB_PA       0x40208000
#define OMAP3_SRAM_PUB_VA       0xd7008000

#if defined(CONFIG_ARCH_OMAP24XX) || defined(CONFIG_ARCH_OMAP34XX)
#define SRAM_BOOTLOADER_SZ	0x00
#elif defined(CONFIG_EFB_DEBUG)
/* reserve memory for frame buffer too ... */
#define SRAM_BOOTLOADER_SZ (0x1020 + 320 * 240 * 2)
#else
#define SRAM_BOOTLOADER_SZ	0x80
#endif

#define OMAP24XX_VA_REQINFOPERM0	IO_ADDRESS(0x68005048)
#define OMAP24XX_VA_READPERM0		IO_ADDRESS(0x68005050)
#define OMAP24XX_VA_WRITEPERM0		IO_ADDRESS(0x68005058)

#define OMAP34XX_VA_REQINFOPERM0	IO_ADDRESS(0x68012848)
#define OMAP34XX_VA_READPERM0		IO_ADDRESS(0x68012850)
#define OMAP34XX_VA_WRITEPERM0		IO_ADDRESS(0x68012858)
#define OMAP34XX_VA_ADDR_MATCH2		IO_ADDRESS(0x68012880)
#define OMAP34XX_VA_SMS_RG_ATT0		IO_ADDRESS(0x6C000048)
#define OMAP34XX_VA_CONTROL_STAT	IO_ADDRESS(0x480022F0)

#define GP_DEVICE		0x300

#define ROUND_DOWN(value,boundary)	((value) & (~((boundary)-1)))

static unsigned long omap_sram_start;
static unsigned long omap_sram_base;
static unsigned long omap_sram_size;
static unsigned long omap_sram_ceil;

extern unsigned long omapfb_reserve_sram(unsigned long sram_pstart,
					 unsigned long sram_vstart,
					 unsigned long sram_size,
					 unsigned long pstart_avail,
					 unsigned long size_avail);

/* Global symbols in sram-fn.S to be patched with omap_sram_patch_va() */
extern void *omap24xx_sdi_cm_clksel2_pll;
extern void *omap24xx_sdi_sdrc_dlla_ctrl;
extern void *omap24xx_sdi_prcm_voltctrl;
extern void *omap24xx_sdi_timer_32ksynct_cr;
extern void *omap24xx_srs_cm_clksel2_pll;
extern void *omap24xx_srs_sdrc_dlla_ctrl;
extern void *omap24xx_srs_sdrc_rfr_ctrl;
extern void *omap24xx_srs_prcm_voltctrl;
extern void *omap24xx_srs_timer_32ksynct;
extern void *omap24xx_ssp_set_config;
extern void *omap24xx_ssp_pll_ctl;
extern void *omap24xx_ssp_pll_stat;
extern void *omap24xx_ssp_pll_div;
extern void *omap24xx_ssp_sdrc_rfr;
extern void *omap24xx_ssp_dlla_ctrl;


/*
 * Depending on the target RAMFS firewall setup, the public usable amount of
 * SRAM varies.  The default accessible size for all device types is 2k. A GP
 * device allows ARM11 but not other initiators for full size. This
 * functionality seems ok until some nice security API happens.
 */
static int is_sram_locked(void)
{
	int type = 0;

	if (cpu_is_omap242x())
		type = system_rev & OMAP2_DEVICETYPE_MASK;

	if (type == GP_DEVICE) {
		/* RAMFW: R/W access to all initiators for all qualifier sets */
		if (cpu_is_omap242x()) {
			__raw_writel(0xFF, OMAP24XX_VA_REQINFOPERM0); /* all q-vects */
			__raw_writel(0xCFDE, OMAP24XX_VA_READPERM0);  /* all i-read */
			__raw_writel(0xCFDE, OMAP24XX_VA_WRITEPERM0); /* all i-write */
		}
		if (cpu_is_omap34xx()) {
			__raw_writel(0xFFFF, OMAP34XX_VA_REQINFOPERM0); /* all q-vects */
			__raw_writel(0xFFFF, OMAP34XX_VA_READPERM0);  /* all i-read */
			__raw_writel(0xFFFF, OMAP34XX_VA_WRITEPERM0); /* all i-write */
			__raw_writel(0x0, OMAP34XX_VA_ADDR_MATCH2);
			__raw_writel(0xFFFFFFFF, OMAP34XX_VA_SMS_RG_ATT0);
		}
		return 0;
	} else
		return 1; /* assume locked with no PPA or security driver */
}

/*
 * The amount of SRAM depends on the core type.
 * Note that we cannot try to test for SRAM here because writes
 * to secure SRAM will hang the system. Also the SRAM is not
 * yet mapped at this point.
 */
void __init omap_detect_sram(void)
{
	unsigned long reserved;

	if (cpu_class_is_omap2()) {
		if (is_sram_locked()) {
			if (cpu_is_omap34xx()) {
				omap_sram_base = OMAP3_SRAM_PUB_VA;
				omap_sram_start = OMAP3_SRAM_PUB_PA;
				omap_sram_size = 0x8000; /* 32K */
			} else {
				omap_sram_base = OMAP2_SRAM_PUB_VA;
				omap_sram_start = OMAP2_SRAM_PUB_PA;
				omap_sram_size = 0x800; /* 2K */
			}
		} else {
			if (cpu_is_omap34xx()) {
				omap_sram_base = OMAP3_SRAM_VA;
				omap_sram_start = OMAP3_SRAM_PA;
				omap_sram_size = 0x10000; /* 64K */
			} else {
				omap_sram_base = OMAP2_SRAM_VA;
				omap_sram_start = OMAP2_SRAM_PA;
				if (cpu_is_omap242x())
					omap_sram_size = 0xa0000; /* 640K */
				else if (cpu_is_omap243x())
					omap_sram_size = 0x10000; /* 64K */
			}
		}
	} else {
		omap_sram_base = OMAP1_SRAM_VA;
		omap_sram_start = OMAP1_SRAM_PA;

		if (cpu_is_omap730() || cpu_is_omap850 ())
			omap_sram_size = 0x32000;	/* 200K */
		else if (cpu_is_omap15xx())
			omap_sram_size = 0x30000;	/* 192K */
		else if (cpu_is_omap1610() || cpu_is_omap1621() ||
		     cpu_is_omap1710())
			omap_sram_size = 0x4000;	/* 16K */
		else if (cpu_is_omap1611())
			omap_sram_size = 0x3e800;	/* 250K */
		else {
			printk(KERN_ERR "Could not detect SRAM size\n");
			omap_sram_size = 0x4000;
		}
	}
	reserved = omapfb_reserve_sram(omap_sram_start, omap_sram_base,
				       omap_sram_size,
				       omap_sram_start + SRAM_BOOTLOADER_SZ,
				       omap_sram_size - SRAM_BOOTLOADER_SZ);
	omap_sram_size -= reserved;
	omap_sram_ceil = omap_sram_base + omap_sram_size;
}

static struct map_desc omap_sram_io_desc[] __initdata = {
	{	/* .length gets filled in at runtime */
		.virtual	= OMAP1_SRAM_VA,
		.pfn		= __phys_to_pfn(OMAP1_SRAM_PA),
		.type		= MT_MEMORY
	}
};

/*
 * Note that we cannot use ioremap for SRAM, as clock init needs SRAM early.
 */
void __init omap_map_sram(void)
{
	unsigned long base;

	if (omap_sram_size == 0)
		return;

	if (cpu_is_omap24xx()) {
		omap_sram_io_desc[0].virtual = OMAP2_SRAM_VA;

		base = OMAP2_SRAM_PA;
		base = ROUND_DOWN(base, PAGE_SIZE);
		omap_sram_io_desc[0].pfn = __phys_to_pfn(base);
	}

	if (cpu_is_omap34xx()) {
		omap_sram_io_desc[0].virtual = OMAP3_SRAM_VA;
		base = OMAP3_SRAM_PA;
		base = ROUND_DOWN(base, PAGE_SIZE);
		omap_sram_io_desc[0].pfn = __phys_to_pfn(base);
	}

	omap_sram_io_desc[0].length = 1024 * 1024;	/* Use section desc */
	iotable_init(omap_sram_io_desc, ARRAY_SIZE(omap_sram_io_desc));

	printk(KERN_INFO "SRAM: Mapped pa 0x%08lx to va 0x%08lx size: 0x%lx\n",
	__pfn_to_phys(omap_sram_io_desc[0].pfn),
	omap_sram_io_desc[0].virtual,
	       omap_sram_io_desc[0].length);

	/*
	 * Normally devicemaps_init() would flush caches and tlb after
	 * mdesc->map_io(), but since we're called from map_io(), we
	 * must do it here.
	 */
	local_flush_tlb_all();
	flush_cache_all();

	/*
	 * Looks like we need to preserve some bootloader code at the
	 * beginning of SRAM for jumping to flash for reboot to work...
	 */
	memset((void *)omap_sram_base + SRAM_BOOTLOADER_SZ, 0,
	       omap_sram_size - SRAM_BOOTLOADER_SZ);
}

void * omap_sram_push(void * start, unsigned long size)
{
	if (size > (omap_sram_ceil - (omap_sram_base + SRAM_BOOTLOADER_SZ))) {
		printk(KERN_ERR "Not enough space in SRAM\n");
		return NULL;
	}

	omap_sram_ceil -= size;
	omap_sram_ceil = ROUND_DOWN(omap_sram_ceil, sizeof(void *));
	memcpy((void *)omap_sram_ceil, start, size);
	flush_icache_range((unsigned long)start, (unsigned long)(start + size));

	return (void *)omap_sram_ceil;
}

/**
 * omap_sram_patch_va - patch a virtual address into SRAM code
 * @srcfn: original start address (in DRAM) of function to patch
 * @srcd: original address (in DRAM) of location to patch
 * @sramfn: start address (in SRAM) of function to patch
 * @d: virtual address to insert
 *
 * Replace a location in SRAM containing a magic number
 * (SRAM_VA_MAGIC) with a caller-specified virtual address.  Used to
 * dynamically patch SRAM code at runtime for multiboot, since some
 * register addresses change depending on the OMAP chip in use.
 * Returns 1 upon success, 0 upon failure.
 */
int omap_sram_patch_va(void *srcfn, void *srcd, void *sramfn, void __iomem *d)
{
	unsigned long sram_addr;
	long offs;

	offs = (unsigned long)srcd - (unsigned long)srcfn;
	sram_addr = (unsigned long)sramfn + offs;

#ifdef CONFIG_OMAP_DEBUG_SRAM_PATCH
	if (offs < 0) {
		printk(KERN_ERR "sram: patch address 0x%0lx < function start "
		       "address 0x%0lx\n", (unsigned long)srcd,
		       (unsigned long)srcfn);
		WARN_ON(1);
		return 0;
	}

	/*
	 * REVISIT: We should probably pass in the function's size also,
	 * so we can verify that the address to patch exists within
	 * the function
	 */
	if (sram_addr > omap_sram_base + omap_sram_size ||
	    sram_addr < omap_sram_base + SRAM_BOOTLOADER_SZ) {
		printk(KERN_ERR "sram: invalid patch address 0x%0lx\n",
		       sram_addr);
		WARN_ON(1);
		return 0;
	}

	if (*(typeof(SRAM_VA_MAGIC) *)sram_addr != SRAM_VA_MAGIC) {
		printk(KERN_ERR "sram: will not patch address 0x%0lx: "
		       "no magic\n", sram_addr);
		WARN_ON(1);
		return 0;
	}
#endif /* CONFIG_OMAP_DEBUG_SRAM_PATCH */

	pr_debug("sram: patching 0x%0lx with 0x%0lx\n", sram_addr,
		 (unsigned long)d);

	*(unsigned long *)sram_addr = (unsigned long)d;

	return 1;
}


static void omap_sram_error(void)
{
	panic("Uninitialized SRAM function\n");
}

#ifdef CONFIG_ARCH_OMAP1

static void (*_omap_sram_reprogram_clock)(u32 dpllctl, u32 ckctl);

void omap_sram_reprogram_clock(u32 dpllctl, u32 ckctl)
{
	if (!_omap_sram_reprogram_clock)
		omap_sram_error();

	return _omap_sram_reprogram_clock(dpllctl, ckctl);
}

int __init omap1_sram_init(void)
{
	_omap_sram_reprogram_clock =
			omap_sram_push(omap1_sram_reprogram_clock,
					omap1_sram_reprogram_clock_sz);

	return 0;
}

#else
#define omap1_sram_init()	do {} while (0)
#endif

#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)

static void (*_omap2_sram_ddr_init)(u32 *slow_dll_ctrl, u32 fast_dll_ctrl,
			      u32 base_cs, u32 force_unlock);

void omap2_sram_ddr_init(u32 *slow_dll_ctrl, u32 fast_dll_ctrl,
		   u32 base_cs, u32 force_unlock)
{
	if (!_omap2_sram_ddr_init)
		omap_sram_error();

	return _omap2_sram_ddr_init(slow_dll_ctrl, fast_dll_ctrl,
				    base_cs, force_unlock);
}

static void (*_omap2_sram_reprogram_sdrc)(u32 perf_level, u32 dll_val,
					  u32 mem_type);

void omap2_sram_reprogram_sdrc(u32 perf_level, u32 dll_val, u32 mem_type)
{
	if (!_omap2_sram_reprogram_sdrc)
		omap_sram_error();

	return _omap2_sram_reprogram_sdrc(perf_level, dll_val, mem_type);
}

static u32 (*_omap2_set_prcm)(u32 dpll_ctrl_val, u32 sdrc_rfr_val, int bypass);

u32 omap2_set_prcm(u32 dpll_ctrl_val, u32 sdrc_rfr_val, int bypass)
{
	if (!_omap2_set_prcm)
		omap_sram_error();

	return _omap2_set_prcm(dpll_ctrl_val, sdrc_rfr_val, bypass);
}
#endif

#ifdef CONFIG_ARCH_OMAP2
int __init omap24xx_sram_init(void)
{
	_omap2_sram_ddr_init = omap_sram_push(omap24xx_sram_ddr_init,
					omap24xx_sram_ddr_init_sz);

	/* Patch in the correct register addresses for multiboot */
	omap_sram_patch_va(omap24xx_sram_ddr_init, &omap24xx_sdi_cm_clksel2_pll,
			   _omap2_sram_ddr_init,
			   OMAP_CM_REGADDR(PLL_MOD, CM_CLKSEL2));
	omap_sram_patch_va(omap24xx_sram_ddr_init, &omap24xx_sdi_sdrc_dlla_ctrl,
			   _omap2_sram_ddr_init,
			   OMAP_SDRC_REGADDR(SDRC_DLLA_CTRL));
	omap_sram_patch_va(omap24xx_sram_ddr_init, &omap24xx_sdi_prcm_voltctrl,
			   _omap2_sram_ddr_init, OMAP24XX_PRCM_VOLTCTRL);
	omap_sram_patch_va(omap24xx_sram_ddr_init,
			   &omap24xx_sdi_timer_32ksynct_cr,
			   _omap2_sram_ddr_init,
			   (void __iomem *)IO_ADDRESS(OMAP2_32KSYNCT_BASE + 0x010));

	_omap2_sram_reprogram_sdrc = omap_sram_push(omap24xx_sram_reprogram_sdrc,
						    omap24xx_sram_reprogram_sdrc_sz);

	omap_sram_patch_va(omap24xx_sram_reprogram_sdrc,
			   &omap24xx_srs_cm_clksel2_pll,
			   _omap2_sram_reprogram_sdrc,
			   OMAP_CM_REGADDR(PLL_MOD, CM_CLKSEL2));
	omap_sram_patch_va(omap24xx_sram_reprogram_sdrc,
			   &omap24xx_srs_sdrc_dlla_ctrl,
			   _omap2_sram_reprogram_sdrc,
			   OMAP_SDRC_REGADDR(SDRC_DLLA_CTRL));
	omap_sram_patch_va(omap24xx_sram_reprogram_sdrc,
			   &omap24xx_srs_sdrc_rfr_ctrl,
			   _omap2_sram_reprogram_sdrc,
			   OMAP_SDRC_REGADDR(SDRC_RFR_CTRL_0));
	omap_sram_patch_va(omap24xx_sram_reprogram_sdrc,
			   &omap24xx_srs_prcm_voltctrl,
			   _omap2_sram_reprogram_sdrc,
			   OMAP24XX_PRCM_VOLTCTRL);
	omap_sram_patch_va(omap24xx_sram_reprogram_sdrc,
			   &omap24xx_srs_timer_32ksynct,
			   _omap2_sram_reprogram_sdrc,
			   (void __iomem *)IO_ADDRESS(OMAP2_32KSYNCT_BASE + 0x010));

	_omap2_set_prcm = omap_sram_push(omap24xx_sram_set_prcm,
					 omap24xx_sram_set_prcm_sz);

	omap_sram_patch_va(omap24xx_sram_set_prcm, &omap24xx_ssp_set_config,
			   _omap2_set_prcm,
			   OMAP24XX_PRCM_CLKCFG_CTRL);
	omap_sram_patch_va(omap24xx_sram_set_prcm, &omap24xx_ssp_pll_ctl,
			   _omap2_set_prcm,
			   OMAP_CM_REGADDR(PLL_MOD, CM_CLKEN));
	omap_sram_patch_va(omap24xx_sram_set_prcm, &omap24xx_ssp_pll_stat,
			   _omap2_set_prcm,
			   OMAP_CM_REGADDR(PLL_MOD, CM_IDLEST));
	omap_sram_patch_va(omap24xx_sram_set_prcm, &omap24xx_ssp_pll_div,
			   _omap2_set_prcm,
			   OMAP_CM_REGADDR(PLL_MOD, CM_CLKSEL1));
	omap_sram_patch_va(omap24xx_sram_set_prcm, &omap24xx_ssp_sdrc_rfr,
			   _omap2_set_prcm,
			   OMAP_SDRC_REGADDR(SDRC_RFR_CTRL_0));
	omap_sram_patch_va(omap24xx_sram_set_prcm, &omap24xx_ssp_dlla_ctrl,
			   _omap2_set_prcm,
			   OMAP_SDRC_REGADDR(SDRC_DLLA_CTRL));

	return 0;
}
#else
static inline int omap24xx_sram_init(void)
{
	return 0;
}
#endif

int __init omap_sram_init(void)
{
	omap_detect_sram();
	omap_map_sram();

	if (!(cpu_class_is_omap2()))
		omap1_sram_init();
	else if (cpu_is_omap24xx())
		omap24xx_sram_init();

	return 0;
}
