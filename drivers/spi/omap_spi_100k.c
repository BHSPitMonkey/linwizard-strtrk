/*
 * OMAP850 SPI 100k controller driver
 * Author: Fabrice Crohas <fcrohas@gmail.com>
 * from original omap1_mcspi driver  
 *
 * Copyright (C) 2005, 2006 Nokia Corporation
 * Author:	Samuel Ortiz <samuel.ortiz@nokia.com> and
 *		Juha Yrjola <juha.yrjola@nokia.com>
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
#define VERBOSE
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <linux/spi/spi.h>

#include <asm/arch/clock.h>


#define OMAP1_SPI100K_MAX_FREQ		48000000

#define OMAP1_SPI100K_REVISION		0x00
#define OMAP1_SPI100K_SYSCONFIG		0x10
#define OMAP1_SPI100K_SYSSTATUS		0x14
#define OMAP1_SPI100K_IRQSTATUS		0x18
#define OMAP1_SPI100K_IRQENABLE		0x1c
#define OMAP1_SPI100K_WAKEUPENABLE	0x20
#define OMAP1_SPI100K_SYST		0x24
#define OMAP1_SPI100K_MODULCTRL		0x28

/* per-channel banks, 0x14 bytes each, first is: */
#define OMAP1_SPI100K_CHCONF0		0x2c
#define OMAP1_SPI100K_CHSTAT0		0x00
#define OMAP1_SPI100K_CHCTRL0		0x34
#define OMAP1_SPI100K_TX0			0x0A
#define OMAP1_SPI100K_RX0			0x0C
#define OMAP1_SPI100K_CHSTAT_RXS		0x06
#define OMAP1_SPI100K_CHSTAT_TXS		0x02

/* per-register bitmasks: */

#define OMAP1_SPI100K_SYSCONFIG_AUTOIDLE	(1 << 0)
#define OMAP1_SPI100K_SYSCONFIG_SOFTRESET	(1 << 1)

#define OMAP1_SPI100K_SYSSTATUS_RESETDONE	(1 << 0)

#define OMAP1_SPI100K_MODULCTRL_SINGLE	(1 << 0)
#define OMAP1_SPI100K_MODULCTRL_MS	(1 << 2)
#define OMAP1_SPI100K_MODULCTRL_STEST	(1 << 3)

#define OMAP1_SPI100K_CHCONF_PHA		(1 << 0)
#define OMAP1_SPI100K_CHCONF_POL		(1 << 1)
#define OMAP1_SPI100K_CHCONF_CLKD_MASK	(0x0f << 2)
#define OMAP1_SPI100K_CHCONF_EPOL		(1 << 6)
#define OMAP1_SPI100K_CHCONF_WL_MASK	(0x1f << 7)
#define OMAP1_SPI100K_CHCONF_TRM_RX_ONLY	(0x01 << 12)
#define OMAP1_SPI100K_CHCONF_TRM_TX_ONLY	(0x02 << 12)
#define OMAP1_SPI100K_CHCONF_TRM_MASK	(0x03 << 12)
#define OMAP1_SPI100K_CHCONF_DMAW		(1 << 14)
#define OMAP1_SPI100K_CHCONF_DMAR		(1 << 15)
#define OMAP1_SPI100K_CHCONF_DPE0		(1 << 16)
#define OMAP1_SPI100K_CHCONF_DPE1		(1 << 17)
#define OMAP1_SPI100K_CHCONF_IS		(1 << 18)
#define OMAP1_SPI100K_CHCONF_TURBO	(1 << 19)
#define OMAP1_SPI100K_CHCONF_FORCE	(1 << 20)

#define OMAP1_SPI100K_CHSTAT_EOT		(1 << 2)

#define OMAP1_SPI100K_CHCTRL_EN		(1 << 0)


/* use PIO for small transfers, avoiding DMA setup/teardown overhead and
 * cache operations; better heuristics consider wordsize and bitrate.
 */
#define DMA_MIN_BYTES			8


struct omap1_spi100k {
	struct work_struct	work;
	/* lock protects queue and registers */
	spinlock_t		lock;
	struct list_head	msg_queue;
	struct spi_master	*master;
	struct clk		*ick;
	struct clk		*fck;
	/* Virtual base address of the controller */
	void __iomem		*base;
};

struct omap1_spi100k_cs {
	void __iomem		*base;
	int			word_len;
};

static struct workqueue_struct *omap1_spi100k_wq;

#define MOD_REG_BIT(val, mask, set) do { \
	if (set) \
		val |= mask; \
	else \
		val &= ~mask; \
} while (0)

static inline void spi100k_write_reg(struct spi_master *master,
		int idx, u32 val)
{
	struct omap1_spi100k *spi100k = spi_master_get_devdata(master);

	__raw_writel(val, spi100k->base + idx);
}

static inline u32 spi100k_read_reg(struct spi_master *master, int idx)
{
	struct omap1_spi100k *spi100k = spi_master_get_devdata(master);

	return __raw_readl(spi100k->base + idx);
}

static inline void spi100k_write_cs_reg(const struct spi_device *spi,
		int idx, u32 val)
{
	struct omap1_spi100k_cs	*cs = spi->controller_state;

	__raw_writel(val, cs->base +  idx);
}

static inline u32 spi100k_read_cs_reg(const struct spi_device *spi, int idx)
{
	struct omap1_spi100k_cs	*cs = spi->controller_state;

	return __raw_readl(cs->base + idx);
}

static void omap1_spi100k_set_enable(const struct spi_device *spi, int enable)
{
	u32 l;

	l = enable ? OMAP1_SPI100K_CHCTRL_EN : 0;
	spi100k_write_cs_reg(spi, OMAP1_SPI100K_CHCTRL0, l);
}

static void omap1_spi100k_force_cs(struct spi_device *spi, int cs_active)
{
	u32 l;

	l = spi100k_read_cs_reg(spi, OMAP1_SPI100K_CHCONF0);
	MOD_REG_BIT(l, OMAP1_SPI100K_CHCONF_FORCE, cs_active);
	spi100k_write_cs_reg(spi, OMAP1_SPI100K_CHCONF0, l);
}

static void omap1_spi100k_set_master_mode(struct spi_master *master)
{
	u32 l;

	/* setup when switching from (reset default) slave mode
	 * to single-channel master mode
	 */
	l = spi100k_read_reg(master, OMAP1_SPI100K_MODULCTRL);
	MOD_REG_BIT(l, OMAP1_SPI100K_MODULCTRL_STEST, 0);
	MOD_REG_BIT(l, OMAP1_SPI100K_MODULCTRL_MS, 0);
	MOD_REG_BIT(l, OMAP1_SPI100K_MODULCTRL_SINGLE, 1);
	spi100k_write_reg(master, OMAP1_SPI100K_MODULCTRL, l);
}

static int spi100k_wait_for_reg_bit(void __iomem *reg, unsigned long bit)
{
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(1000);
	while (!(__raw_readl(reg) & bit)) {
		if (time_after(jiffies, timeout))
			return -1;
		cpu_relax();
	}
	return 0;
}

static unsigned
omap1_spi100k_txrx_pio(struct spi_device *spi, struct spi_transfer *xfer)
{
	struct omap1_spi100k	*spi100k;
	struct omap1_spi100k_cs	*cs = spi->controller_state;
	unsigned int		count, c;
	u32			l;
	void __iomem		*base = cs->base;
	void __iomem		*tx_reg;
	void __iomem		*rx_reg;
	void __iomem		*chstat_reg;
	int			word_len;

	spi100k = spi_master_get_devdata(spi->master);
	count = xfer->len;
	c = count;
	word_len = cs->word_len;

	l = spi100k_read_cs_reg(spi, OMAP1_SPI100K_CHCONF0);
	l &= ~OMAP1_SPI100K_CHCONF_TRM_MASK;

	/* We store the pre-calculated register addresses on stack to speed
	 * up the transfer loop. */
	tx_reg		= base + OMAP1_SPI100K_TX0;
	rx_reg		= base + OMAP1_SPI100K_RX0;
	chstat_reg	= base + OMAP1_SPI100K_CHSTAT0;

	if (word_len <= 8) {
		u8		*rx;
		const u8	*tx;

		rx = xfer->rx_buf;
		tx = xfer->tx_buf;

		do {
			c -= 1;
			if (tx != NULL) {
				if (spi100k_wait_for_reg_bit(chstat_reg,
						OMAP1_SPI100K_CHSTAT_TXS) < 0) {
					dev_err(&spi->dev, "TXS timed out\n");
					goto out;
				}
#ifdef VERBOSE
				dev_dbg(&spi->dev, "write-%d %02x\n",
						word_len, *tx);
#endif
				__raw_writel(*tx++, tx_reg);
			}
			if (rx != NULL) {
				if (spi100k_wait_for_reg_bit(chstat_reg,
						OMAP1_SPI100K_CHSTAT_RXS) < 0) {
					dev_err(&spi->dev, "RXS timed out\n");
					goto out;
				}
				/* prevent last RX_ONLY read from triggering
				 * more word i/o: switch to rx+tx
				 */
				if (c == 0 && tx == NULL)
					spi100k_write_cs_reg(spi,
							OMAP1_SPI100K_CHCONF0, l);
				*rx++ = __raw_readl(rx_reg);
#ifdef VERBOSE
				dev_dbg(&spi->dev, "read-%d %02x\n",
						word_len, *(rx - 1));
#endif
			}
		} while (c);
	} else if (word_len <= 16) {
		u16		*rx;
		const u16	*tx;

		rx = xfer->rx_buf;
		tx = xfer->tx_buf;
		do {
			c -= 2;
			if (tx != NULL) {
				if (spi100k_wait_for_reg_bit(chstat_reg,
						OMAP1_SPI100K_CHSTAT_TXS) < 0) {
					dev_err(&spi->dev, "TXS timed out\n");
					goto out;
				}
#ifdef VERBOSE
				dev_dbg(&spi->dev, "write-%d %04x\n",
						word_len, *tx);
#endif
				__raw_writel(*tx++, tx_reg);
			}
			if (rx != NULL) {
				if (spi100k_wait_for_reg_bit(chstat_reg,
						OMAP1_SPI100K_CHSTAT_RXS) < 0) {
					dev_err(&spi->dev, "RXS timed out\n");
					goto out;
				}
				/* prevent last RX_ONLY read from triggering
				 * more word i/o: switch to rx+tx
				 */
				if (c == 0 && tx == NULL)
					spi100k_write_cs_reg(spi,
							OMAP1_SPI100K_CHCONF0, l);
				*rx++ = __raw_readl(rx_reg);
#ifdef VERBOSE
				dev_dbg(&spi->dev, "read-%d %04x\n",
						word_len, *(rx - 1));
#endif
			}
		} while (c);
	} else if (word_len <= 32) {
		u32		*rx;
		const u32	*tx;

		rx = xfer->rx_buf;
		tx = xfer->tx_buf;
		do {
			c -= 4;
			if (tx != NULL) {
				if (spi100k_wait_for_reg_bit(chstat_reg,
						OMAP1_SPI100K_CHSTAT_TXS) < 0) {
					dev_err(&spi->dev, "TXS timed out\n");
					goto out;
				}
#ifdef VERBOSE
				dev_dbg(&spi->dev, "write-%d %04x\n",
						word_len, *tx);
#endif
				__raw_writel(*tx++, tx_reg);
			}
			if (rx != NULL) {
				if (spi100k_wait_for_reg_bit(chstat_reg,
						OMAP1_SPI100K_CHSTAT_RXS) < 0) {
					dev_err(&spi->dev, "RXS timed out\n");
					goto out;
				}
				/* prevent last RX_ONLY read from triggering
				 * more word i/o: switch to rx+tx
				 */
				if (c == 0 && tx == NULL)
					spi100k_write_cs_reg(spi,
							OMAP1_SPI100K_CHCONF0, l);
				*rx++ = __raw_readl(rx_reg);
#ifdef VERBOSE
				dev_dbg(&spi->dev, "read-%d %04x\n",
						word_len, *(rx - 1));
#endif
			}
		} while (c);
	}

	/* for TX_ONLY mode, be sure all words have shifted out */
	if (xfer->rx_buf == NULL) {
		if (spi100k_wait_for_reg_bit(chstat_reg,
				OMAP1_SPI100K_CHSTAT_TXS) < 0) {
			dev_err(&spi->dev, "TXS timed out\n");
		} else if (spi100k_wait_for_reg_bit(chstat_reg,
				OMAP1_SPI100K_CHSTAT_EOT) < 0)
			dev_err(&spi->dev, "EOT timed out\n");
	}
out:
	return count - c;
}

/* called only when no transfer is active to this device */
static int omap1_spi100k_setup_transfer(struct spi_device *spi,
		struct spi_transfer *t)
{
	struct omap1_spi100k_cs *cs = spi->controller_state;
	struct omap1_spi100k *spi100k;
	u32 l = 0, div = 0;
	u8 word_len = spi->bits_per_word;

	spi100k = spi_master_get_devdata(spi->master);

	if (t != NULL && t->bits_per_word)
		word_len = t->bits_per_word;

	cs->word_len = word_len;

	if (spi->max_speed_hz) {
		while (div <= 15 && (OMAP1_SPI100K_MAX_FREQ / (1 << div))
					> spi->max_speed_hz)
			div++;
	} else
		div = 15;

	l = spi100k_read_cs_reg(spi, OMAP1_SPI100K_CHCONF0);

	/* standard 4-wire master mode:  SCK, MOSI/out, MISO/in, nCS
	 * REVISIT: this controller could support SPI_3WIRE mode.
	 */
	l &= ~(OMAP1_SPI100K_CHCONF_IS|OMAP1_SPI100K_CHCONF_DPE1);
	l |= OMAP1_SPI100K_CHCONF_DPE0;

	/* wordlength */
	l &= ~OMAP1_SPI100K_CHCONF_WL_MASK;
	l |= (word_len - 1) << 7;

	/* set chipselect polarity; manage with FORCE */
	if (!(spi->mode & SPI_CS_HIGH))
		l |= OMAP1_SPI100K_CHCONF_EPOL;	/* active-low; normal */
	else
		l &= ~OMAP1_SPI100K_CHCONF_EPOL;

	/* set clock divisor */
	l &= ~OMAP1_SPI100K_CHCONF_CLKD_MASK;
	l |= div << 2;

	/* set SPI mode 0..3 */
	if (spi->mode & SPI_CPOL)
		l |= OMAP1_SPI100K_CHCONF_POL;
	else
		l &= ~OMAP1_SPI100K_CHCONF_POL;
	if (spi->mode & SPI_CPHA)
		l |= OMAP1_SPI100K_CHCONF_PHA;
	else
		l &= ~OMAP1_SPI100K_CHCONF_PHA;

	spi100k_write_cs_reg(spi, OMAP1_SPI100K_CHCONF0, l);

	dev_dbg(&spi->dev, "setup: speed %d, sample %s edge, clk %s\n",
			OMAP1_SPI100K_MAX_FREQ / (1 << div),
			(spi->mode & SPI_CPHA) ? "trailing" : "leading",
			(spi->mode & SPI_CPOL) ? "inverted" : "normal");

	return 0;
}

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)

static int omap1_spi100k_setup(struct spi_device *spi)
{
	int			ret;
	struct omap1_spi100k	*spi100k;
	struct omap1_spi100k_cs	*cs = spi->controller_state;

	if (spi->mode & ~MODEBITS) {
		dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}

	if (spi->bits_per_word == 0)
		spi->bits_per_word = 8;
	else if (spi->bits_per_word < 4 || spi->bits_per_word > 32) {
		dev_dbg(&spi->dev, "setup: unsupported %d bit words\n",
			spi->bits_per_word);
		return -EINVAL;
	}

	spi100k = spi_master_get_devdata(spi->master);

	if (!cs) {
		cs = kzalloc(sizeof *cs, GFP_KERNEL);
		if (!cs)
			return -ENOMEM;
		cs->base = spi100k->base + spi->chip_select * 0x14;
		spi->controller_state = cs;
	}


	clk_enable(spi100k->ick);
	clk_enable(spi100k->fck);
	ret = omap1_spi100k_setup_transfer(spi, NULL);
	clk_disable(spi100k->fck);
	clk_disable(spi100k->ick);

	return ret;
}

static void omap1_spi100k_cleanup(struct spi_device *spi)
{
	struct omap1_spi100k	*spi100k;
	spi100k = spi_master_get_devdata(spi->master);
}

static void omap1_spi100k_work(struct work_struct *work)
{
	struct omap1_spi100k	*spi100k;

	spi100k = container_of(work, struct omap1_spi100k, work);
	spin_lock_irq(&spi100k->lock);

	clk_enable(spi100k->ick);
	clk_enable(spi100k->fck);

	/* We only enable one channel at a time -- the one whose message is
	 * at the head of the queue -- although this controller would gladly
	 * arbitrate among multiple channels.  This corresponds to "single
	 * channel" master mode.  As a side effect, we need to manage the
	 * chipselect with the FORCE bit ... CS != channel enable.
	 */
	while (!list_empty(&spi100k->msg_queue)) {
		struct spi_message		*m;
		struct spi_device		*spi;
		struct spi_transfer		*t = NULL;
		int				cs_active = 0;
		struct omap1_spi100k_cs		*cs;
		int				par_override = 0;
		int				status = 0;
		u32				chconf;

		m = container_of(spi100k->msg_queue.next, struct spi_message,
				 queue);

		list_del_init(&m->queue);
		spin_unlock_irq(&spi100k->lock);

		spi = m->spi;
		cs = spi->controller_state;

		omap1_spi100k_set_enable(spi, 1);
		list_for_each_entry(t, &m->transfers, transfer_list) {
			if (t->tx_buf == NULL && t->rx_buf == NULL && t->len) {
				status = -EINVAL;
				break;
			}
			if (par_override || t->speed_hz || t->bits_per_word) {
				par_override = 1;
				status = omap1_spi100k_setup_transfer(spi, t);
				if (status < 0)
					break;
				if (!t->speed_hz && !t->bits_per_word)
					par_override = 0;
			}

			if (!cs_active) {
				omap1_spi100k_force_cs(spi, 1);
				cs_active = 1;
			}

			chconf = spi100k_read_cs_reg(spi, OMAP1_SPI100K_CHCONF0);
			chconf &= ~OMAP1_SPI100K_CHCONF_TRM_MASK;
			if (t->tx_buf == NULL)
				chconf |= OMAP1_SPI100K_CHCONF_TRM_RX_ONLY;
			else if (t->rx_buf == NULL)
				chconf |= OMAP1_SPI100K_CHCONF_TRM_TX_ONLY;
			spi100k_write_cs_reg(spi, OMAP1_SPI100K_CHCONF0, chconf);

			if (t->len) {
				unsigned	count;

				/* RX_ONLY mode needs dummy data in TX reg */
				if (t->tx_buf == NULL)
					__raw_writel(0, cs->base
							+ OMAP1_SPI100K_TX0);

				count = omap1_spi100k_txrx_pio(spi, t);
				m->actual_length += count;

				if (count != t->len) {
					status = -EIO;
					break;
				}
			}

			if (t->delay_usecs)
				udelay(t->delay_usecs);

			/* ignore the "leave it on after last xfer" hint */
			if (t->cs_change) {
				omap1_spi100k_force_cs(spi, 0);
				cs_active = 0;
			}
		}

		/* Restore defaults if they were overriden */
		if (par_override) {
			par_override = 0;
			status = omap1_spi100k_setup_transfer(spi, NULL);
		}

		if (cs_active)
			omap1_spi100k_force_cs(spi, 0);

		omap1_spi100k_set_enable(spi, 0);

		m->status = status;
		m->complete(m->context);

		spin_lock_irq(&spi100k->lock);
	}

	clk_disable(spi100k->fck);
	clk_disable(spi100k->ick);

	spin_unlock_irq(&spi100k->lock);
}

static int omap1_spi100k_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct omap1_spi100k	*spi100k;
	unsigned long		flags;
	struct spi_transfer	*t;

	m->actual_length = 0;
	m->status = 0;

	/* reject invalid messages and transfers */
	if (list_empty(&m->transfers) || !m->complete)
		return -EINVAL;
	list_for_each_entry(t, &m->transfers, transfer_list) {
		const void	*tx_buf = t->tx_buf;
		void		*rx_buf = t->rx_buf;
		unsigned	len = t->len;

		if (t->speed_hz > OMAP1_SPI100K_MAX_FREQ
				|| (len && !(rx_buf || tx_buf))
				|| (t->bits_per_word &&
					(  t->bits_per_word < 4
					|| t->bits_per_word > 32))) {
			dev_dbg(&spi->dev, "transfer: %d Hz, %d %s%s, %d bpw\n",
					t->speed_hz,
					len,
					tx_buf ? "tx" : "",
					rx_buf ? "rx" : "",
					t->bits_per_word);
			return -EINVAL;
		}
		if (t->speed_hz && t->speed_hz < OMAP1_SPI100K_MAX_FREQ/(1<<16)) {
			dev_dbg(&spi->dev, "%d Hz max exceeds %d\n",
					t->speed_hz,
					OMAP1_SPI100K_MAX_FREQ/(1<<16));
			return -EINVAL;
		}

	}

	spi100k = spi_master_get_devdata(spi->master);

	spin_lock_irqsave(&spi100k->lock, flags);
	list_add_tail(&m->queue, &spi100k->msg_queue);
	queue_work(omap1_spi100k_wq, &spi100k->work);
	spin_unlock_irqrestore(&spi100k->lock, flags);

	return 0;
}

static int __init omap1_spi100k_reset(struct omap1_spi100k *spi100k)
{
	struct spi_master	*master = spi100k->master;
	u32			tmp;

	clk_enable(spi100k->ick);
	clk_enable(spi100k->fck);

	spi100k_write_reg(master, OMAP1_SPI100K_SYSCONFIG,
			OMAP1_SPI100K_SYSCONFIG_SOFTRESET);
	do {
		tmp = spi100k_read_reg(master, OMAP1_SPI100K_SYSSTATUS);
	} while (!(tmp & OMAP1_SPI100K_SYSSTATUS_RESETDONE));

	spi100k_write_reg(master, OMAP1_SPI100K_SYSCONFIG,
			/* (3 << 8) | (2 << 3) | */
			OMAP1_SPI100K_SYSCONFIG_AUTOIDLE);

	omap1_spi100k_set_master_mode(master);

	clk_disable(spi100k->fck);
	clk_disable(spi100k->ick);
	return 0;
}

static int __init omap1_spi100k_probe(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct omap1_spi100k	*spi100k;
	struct resource		*r;
	int			status = 0, i;
	unsigned		num_chipselect = 1;

	if ( !pdev->id)
		return -EINVAL;

	master = spi_alloc_master(&pdev->dev, sizeof *spi100k);
	if (master == NULL) {
		dev_dbg(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	if (pdev->id != -1)
		master->bus_num = pdev->id;

	master->setup = omap1_spi100k_setup;
	master->transfer = omap1_spi100k_transfer;
	master->cleanup = omap1_spi100k_cleanup;
	master->num_chipselect = num_chipselect;

	dev_set_drvdata(&pdev->dev, master);

	spi100k = spi_master_get_devdata(master);
	spi100k->master = master;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		printk("SPI: Get resource FAILED\n");
		status = -ENODEV;
		goto err1;
	}
	if (!request_mem_region(r->start, (r->end - r->start) + 1,
			pdev->dev.bus_id)) {
		printk("SPI: Request mem region FAILED\n");
		status = -EBUSY;
		goto err1;
	}

	spi100k->base = (void __iomem *) io_p2v(r->start);

	INIT_WORK(&spi100k->work, omap1_spi100k_work);

	spin_lock_init(&spi100k->lock);
	INIT_LIST_HEAD(&spi100k->msg_queue);

	spi100k->ick = clk_get(&pdev->dev, "spi100k_ick");
	if (IS_ERR(spi100k->ick)) {
		printk("SPI: cant get ick\n");
		dev_dbg(&pdev->dev, "can't get spi100k_ick\n");
		status = PTR_ERR(spi100k->ick);
		goto err1a;
	}
	spi100k->fck = clk_get(&pdev->dev, "spi100k_fck");
	if (IS_ERR(spi100k->fck)) {
		printk("SPI: Gcan get fck\n");
		dev_dbg(&pdev->dev, "can't get spi100k_fck\n");
		status = PTR_ERR(spi100k->fck);
		goto err2;
	}

	//if (omap1_spi100k_reset(spi100k) < 0)
	//	goto err3;

	status = spi_register_master(master);
	if (status < 0)
		goto err3;

	return status;

err3:
	clk_put(spi100k->fck);
err2:
	clk_put(spi100k->ick);
err1a:
	release_mem_region(r->start, (r->end - r->start) + 1);
err1:
	spi_master_put(master);
	return status;
}

static int __exit omap1_spi100k_remove(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct omap1_spi100k	*spi100k;
	struct resource		*r;

	master = dev_get_drvdata(&pdev->dev);
	spi100k = spi_master_get_devdata(master);

	clk_put(spi100k->fck);
	clk_put(spi100k->ick);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, (r->end - r->start) + 1);

	spi_unregister_master(master);

	return 0;
}

static struct platform_driver omap1_spi100k_driver = {
	.driver = {
		.name =		"omap1_spi100k",
		.owner =	THIS_MODULE,
	},
	.remove =	__exit_p(omap1_spi100k_remove),
};


static int __init omap1_spi100k_init(void)
{
	omap1_spi100k_wq = create_singlethread_workqueue(
				omap1_spi100k_driver.driver.name);
	if (omap1_spi100k_wq == NULL)
		return -1;
	return platform_driver_probe(&omap1_spi100k_driver, omap1_spi100k_probe);
}
subsys_initcall(omap1_spi100k_init);

static void __exit omap1_spi100k_exit(void)
{
	platform_driver_unregister(&omap1_spi100k_driver);

	destroy_workqueue(omap1_spi100k_wq);
}
module_exit(omap1_spi100k_exit);

MODULE_LICENSE("GPL");

