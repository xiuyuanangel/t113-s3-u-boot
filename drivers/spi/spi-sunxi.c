/*
 * (C) Copyright 2017 Whitebox Systems / Northend Systems B.V.
 * S.J.R. van Schaik <stephan@whiteboxsystems.nl>
 * M.B.W. Wajer <merlijn@whiteboxsystems.nl>
 *
 * (C) Copyright 2017 Olimex Ltd..
 * Stefan Mavrodiev <stefan@olimex.com>
 *
 * Based on linux spi driver. Original copyright follows:
 * linux/drivers/spi/spi-sun4i.c
 *
 * Copyright (C) 2012 - 2014 Allwinner Tech
 * Pan Nan <pannan@allwinnertech.com>
 *
 * Copyright (C) 2014 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <log.h>
#include <spi.h>
#include <errno.h>
#include <fdt_support.h>
#include <reset.h>
#include <wait_bit.h>
#include <asm/global_data.h>
#include <dm/device_compat.h>
#include <linux/bitops.h>

#include <asm/bitops.h>
#include <asm/gpio.h>
#include <asm/arch-sunxi/gpio.h>
#include <asm/io.h>

#include <linux/iopoll.h>
#include <linux/log2.h>

DECLARE_GLOBAL_DATA_PTR;

/* sun4i spi registers */
#define SUN4I_RXDATA_REG		0x00
#define SUN4I_TXDATA_REG		0x04
#define SUN4I_CTL_REG			0x08
#define SUN4I_CLK_CTL_REG		0x1c
#define SUN4I_BURST_CNT_REG		0x20
#define SUN4I_XMIT_CNT_REG		0x24
#define SUN4I_FIFO_STA_REG		0x28

/* sun6i spi registers */
#define SUN6I_GBL_CTL_REG		0x04
#define SUN6I_TFR_CTL_REG		0x08
#define SUN6I_INT_CTL_REG		0x10
#define SUN6I_INT_STA_REG		0x14
#define SUN6I_FIFO_CTL_REG		0x18
#define SUN6I_FIFO_STA_REG		0x1c
#define SUN6I_CLK_CTL_REG		0x24
#define SUN6I_BURST_CNT_REG		0x30
#define SUN6I_XMIT_CNT_REG		0x34
#define SUN6I_BURST_CTL_REG		0x38
#define SUN6I_TXDATA_REG		0x200
#define SUN6I_RXDATA_REG		0x300

/* sun spi bits */
#define SUN4I_CTL_ENABLE		BIT(0)
#define SUN4I_CTL_MASTER		BIT(1)
#define SUN4I_CLK_CTL_CDR2_MASK		0xff
#define SUN4I_CLK_CTL_CDR2(div)		((div) & SUN4I_CLK_CTL_CDR2_MASK)
#define SUN4I_CLK_CTL_CDR1_MASK		0xf
#define SUN4I_CLK_CTL_CDR1(div)		(((div) & SUN4I_CLK_CTL_CDR1_MASK) << 8)
#define SUN4I_CLK_CTL_DRS		BIT(12)
#define SUN4I_MAX_XFER_SIZE		0xffffff
#define SUN4I_BURST_CNT(cnt)		((cnt) & SUN4I_MAX_XFER_SIZE)
#define SUN4I_XMIT_CNT(cnt)		((cnt) & SUN4I_MAX_XFER_SIZE)
#define SUN4I_FIFO_STA_RF_CNT_BITS	0

#define SUN4I_SPI_MAX_RATE		100000000
#define SUN4I_SPI_MIN_RATE		3000
#define SUN4I_SPI_DEFAULT_RATE		25000000
#define SUN4I_SPI_TIMEOUT_US		1000000

#define SPI_REG(priv, reg)		((priv)->base + \
					(priv)->variant->regs[reg])
#define SPI_BIT(priv, bit)		((priv)->variant->bits[bit])
#define SPI_CS(priv, cs)		(((cs) << SPI_BIT(priv, SPI_TCR_CS_SEL)) & \
					SPI_BIT(priv, SPI_TCR_CS_MASK))


/* sun spi register set */
enum sun4i_spi_regs {
	SPI_GCR,
	SPI_TCR,
    SPI_IER,
    SPI_ISR,
	SPI_FCR,
	SPI_FSR,
	SPI_CCR,
	SPI_BC,
	SPI_TC,
	SPI_BCTL,
	SPI_TXD,
	SPI_RXD,
};

/* sun spi register bits */
enum sun4i_spi_bits {
	SPI_GCR_TP,
	SPI_GCR_SRST,
	SPI_TCR_CPHA,
	SPI_TCR_CPOL,
	SPI_TCR_CS_ACTIVE_LOW,
	SPI_TCR_CS_SEL,
	SPI_TCR_CS_MASK,
	SPI_TCR_XCH,
	SPI_TCR_CS_MANUAL,
	SPI_TCR_CS_LEVEL,
	SPI_FCR_TF_RST,
	SPI_FCR_RF_RST,
	SPI_FSR_RF_CNT_MASK,
    SPI_BCTL_QUAD_EN,
    SPI_BCTL_DUAL_EN,
};

struct sun4i_spi_variant {
	const unsigned long *regs;
	const u32 *bits;
	u32 fifo_depth;
	bool has_soft_reset;
	bool has_burst_ctl;
};

struct sun4i_spi_plat {
	struct sun4i_spi_variant *variant;
//	u32 base;
	void __iomem* base;
	u32 max_hz;
};

struct sun4i_spi_priv {
	struct sun4i_spi_variant *variant;
	void __iomem* base;
	u32 freq;
	u32 mode;

	const u8 *tx_buf;
	u8 *rx_buf;
};

static inline void sun4i_spi_drain_fifo(struct sun4i_spi_priv *priv, int len)
{
	u8 byte;

	while (len--) {
		byte = readb(SPI_REG(priv, SPI_RXD));
		if (priv->rx_buf)
			*priv->rx_buf++ = byte;
	}
}

static inline void sun4i_spi_fill_fifo(struct sun4i_spi_priv *priv, int len)
{
	u8 byte;

	while (len--) {
		byte = priv->tx_buf ? *priv->tx_buf++ : 0;
		writeb(byte, SPI_REG(priv, SPI_TXD));
	}
}

static void sun4i_spi_set_cs(struct udevice *bus, u8 cs, bool enable)
{
	struct sun4i_spi_priv *priv = dev_get_priv(bus->parent);
	u32 reg;

	reg = readl(SPI_REG(priv, SPI_TCR));

	reg &= ~SPI_BIT(priv, SPI_TCR_CS_MASK);
	reg |= SPI_CS(priv, cs);

	if (enable)
		reg &= ~SPI_BIT(priv, SPI_TCR_CS_LEVEL);
	else
		reg |= SPI_BIT(priv, SPI_TCR_CS_LEVEL);

	writel(reg, SPI_REG(priv, SPI_TCR));
}

static int sun4i_spi_parse_pins(struct udevice *dev)
{
	const void *fdt = gd->fdt_blob;
	const char *pin_name;
	const fdt32_t *list;
	u32 phandle;
	int drive, pull = 0, pin, i;
	int offset;
	int size;

	list = fdt_getprop(fdt, dev_of_offset(dev), "pinctrl-0", &size);
	if (!list) {
		printf("WARNING: sun4i_spi: cannot find pinctrl-0 node\n");
		return -EINVAL;
	}

	while (size) {
		phandle = fdt32_to_cpu(*list++);
		size -= sizeof(*list);

		offset = fdt_node_offset_by_phandle(fdt, phandle);
		if (offset < 0)
			return offset;

		drive = fdt_getprop_u32_default_node(fdt, offset, 0,
						     "drive-strength", 0);
		if (drive) {
			if (drive <= 10)
				drive = 0;
			else if (drive <= 20)
				drive = 1;
			else if (drive <= 30)
				drive = 2;
			else
				drive = 3;
		} else {
			drive = fdt_getprop_u32_default_node(fdt, offset, 0,
							     "allwinner,drive",
							      0);
			drive = min(drive, 3);
		}

		if (fdt_get_property(fdt, offset, "bias-disable", NULL))
			pull = 0;
		else if (fdt_get_property(fdt, offset, "bias-pull-up", NULL))
			pull = 1;
		else if (fdt_get_property(fdt, offset, "bias-pull-down", NULL))
			pull = 2;
		else
			pull = fdt_getprop_u32_default_node(fdt, offset, 0,
							    "allwinner,pull",
							     0);
		pull = min(pull, 2);

		for (i = 0; ; i++) {
			pin_name = fdt_stringlist_get(fdt, offset,
						      "pins", i, NULL);
			if (!pin_name) {
				pin_name = fdt_stringlist_get(fdt, offset,
							      "allwinner,pins",
							       i, NULL);
				if (!pin_name)
					break;
			}

			pin = sunxi_name_to_gpio(pin_name);
			if (pin < 0)
				break;

			if (IS_ENABLED(CONFIG_MACH_SUN20IW1)){
				sunxi_gpio_set_cfgpin(pin, SUN20I_GPC_SPI0);
            }
            else if (IS_ENABLED(CONFIG_MACH_SUN50I))
				sunxi_gpio_set_cfgpin(pin, SUN50I_GPC_SPI0);
			else
				sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SPI0);
			sunxi_gpio_set_drv(pin, drive);
			sunxi_gpio_set_pull(pin, pull);
		}
	}
    writel(0x222222ff, 0x02000000 + 0x60);
	return 0;
}

static inline int sun4i_spi_set_clock(struct udevice *dev)
{
	const void *fdt = gd->fdt_blob;
	const fdt32_t *list;
	u32 phandle;
	int gate_off, reset_off;
	int offset;
	int size;
    void __iomem * ccu_base;

	list = fdt_getprop(fdt, dev_of_offset(dev), "clocks", &size);
	if (!list) {
		printf("WARNING: sun4i_spi: cannot find spi0_clocks node\n");
		return -EINVAL;
	}
	while (size) {
		phandle = fdt32_to_cpu(*list++);
		size -= sizeof(*list);

		offset = fdt_node_offset_by_phandle(fdt, phandle);
		if (offset < 0)
			return offset;
        ccu_base = (void *)fdt_getprop_u32_default_node(fdt, offset, 0, "ccu_base", 0);
        if (!ccu_base) {
		    printf("WARNING: sun4i_spi: cannot find ccu_base node\n");
    		return -EINVAL;
        }

        gate_off = fdt_getprop_u32_default_node(fdt, offset, 0,
						     "spi0_clk_gate_off", 0);
		if (!gate_off) {
		    printf("WARNING: sun4i_spi: cannot find spi0_clk_gate_off node\n");
    		return -EINVAL;
	    }
		reset_off = fdt_getprop_u32_default_node(fdt, offset, 0,
						     "spi0_clk_reset_off", 0);
		if (!reset_off) {
		    printf("WARNING: sun4i_spi: cannot find spi0_clk_reset_off node\n");
    		return -EINVAL;
	    }
    }
    /* Deassert spi0 reset */
    clrbits_le32(ccu_base + reset_off, (1 << 16));
    setbits_le32(ccu_base + reset_off, (1 << 16));
    /* Open the spi0 bus gate */
    setbits_le32(ccu_base + reset_off, (1 << 0));
    /* Open the spi0 clock gate */
    setbits_le32(ccu_base + gate_off, (1 << 31));
    /* Select pll-periph0 for spi0 clk */
    clrsetbits_le32(ccu_base + gate_off, 0x3 << 24, 0x1 << 24);
    /* Set clock pre divide ratio, divided by 1 */
    clrsetbits_le32(ccu_base + gate_off, 0x3 << 8, 0 << 0);
    /* Set clock divide ratio, divided by 3 */
    clrsetbits_le32(ccu_base + gate_off, 0xf << 0, (0x6-1) << 0);

    return 0;
}

static int sun4i_spi_claim_bus(struct udevice *dev)
{
	struct sun4i_spi_priv *priv = dev_get_priv(dev->parent);

	writel(SUN4I_CTL_ENABLE | SUN4I_CTL_MASTER | SPI_BIT(priv, SPI_GCR_TP),
            SPI_REG(priv, SPI_GCR));

	if (priv->variant->has_soft_reset)
		setbits_le32(SPI_REG(priv, SPI_GCR),
			     SPI_BIT(priv, SPI_GCR_SRST));

    writel(SPI_BIT(priv, SPI_TCR_CS_MANUAL) |
            SPI_BIT(priv, SPI_TCR_CS_ACTIVE_LOW), SPI_REG(priv, SPI_TCR));

	return 0;
}

static int sun4i_spi_release_bus(struct udevice *dev)
{
	struct sun4i_spi_priv *priv = dev_get_priv(dev->parent);

	clrbits_le32(SPI_REG(priv, SPI_GCR), SUN4I_CTL_ENABLE);

//	sun4i_spi_set_clock(dev, false);

	return 0;
}

static int sun4i_spi_set_speed(struct udevice *dev, uint speed)
{
	struct sun4i_spi_plat *plat = dev_get_plat(dev);
	struct sun4i_spi_priv *priv = dev_get_priv(dev);
	unsigned int div;
	u32 reg;

	if (speed > plat->max_hz)
		speed = plat->max_hz;

	if (speed < SUN4I_SPI_MIN_RATE)
		speed = SUN4I_SPI_MIN_RATE;
	/*
	 * Setup clock divider.
	 *
	 * We have two choices there. Either we can use the clock
	 * divide rate 1, which is calculated thanks to this formula:
	 * SPI_CLK = MOD_CLK / (2 ^ (cdr + 1))
	 * Or we can use CDR2, which is calculated with the formula:
	 * SPI_CLK = MOD_CLK / (2 * (cdr + 1))
	 * Whether we use the former or the latter is set through the
	 * DRS bit.
	 *
	 * First try CDR2, and if we can't reach the expected
	 * frequency, fall back to CDR1.
	 */

	div = SUN4I_SPI_MAX_RATE / (2 * speed);
	reg = readl(SPI_REG(priv, SPI_CCR));

	if (div <= (SUN4I_CLK_CTL_CDR2_MASK + 1)) {
		if (div > 0)
			div--;

		reg &= ~(SUN4I_CLK_CTL_CDR2_MASK | SUN4I_CLK_CTL_DRS);
		reg |= SUN4I_CLK_CTL_CDR2(div) | SUN4I_CLK_CTL_DRS;
	} else {
		div = ilog2(SUN4I_SPI_MAX_RATE) - ilog2(speed);
		reg &= ~((SUN4I_CLK_CTL_CDR1_MASK << 8) | SUN4I_CLK_CTL_DRS);
		reg |= SUN4I_CLK_CTL_CDR1(div);
	}

	priv->freq = speed;
	writel(reg, SPI_REG(priv, SPI_CCR));

	return 0;
}

static int sun4i_spi_set_mode(struct udevice *dev, uint mode)
{
	struct sun4i_spi_priv *priv = dev_get_priv(dev);

	priv->mode = mode;

	if (mode & SPI_CPOL)
        setbits_le32(SPI_REG(priv, SPI_TCR), SPI_BIT(priv, SPI_TCR_CPOL));
    else
        clrbits_le32(SPI_REG(priv, SPI_TCR), SPI_BIT(priv, SPI_TCR_CPOL));

	if (mode & SPI_CPHA)
        setbits_le32(SPI_REG(priv, SPI_TCR), SPI_BIT(priv, SPI_TCR_CPHA));
    else
        clrbits_le32(SPI_REG(priv, SPI_TCR), SPI_BIT(priv, SPI_TCR_CPHA));

	if (mode & (SPI_TX_QUAD | SPI_RX_QUAD))
        setbits_le32(SPI_REG(priv, SPI_BCTL), SPI_BIT(priv, SPI_BCTL_QUAD_EN));
    else
        clrbits_le32(SPI_REG(priv, SPI_BCTL), SPI_BIT(priv, SPI_BCTL_QUAD_EN));

    if (mode & (SPI_TX_DUAL | SPI_RX_DUAL))
        setbits_le32(SPI_REG(priv, SPI_BCTL), SPI_BIT(priv, SPI_BCTL_DUAL_EN));
    else
        clrbits_le32(SPI_REG(priv, SPI_BCTL), SPI_BIT(priv, SPI_BCTL_DUAL_EN));
//printf("%s %d -- %x\n",__FILE__,__LINE__,mode);
//printf("%s %d -- %x\n",__FILE__,__LINE__,readl(SPI_REG(priv, SPI_BCTL)));
	return 0;
}

static int sun4i_spi_transfer(struct udevice *dev, unsigned int bitlen, const void *dout, void *din, unsigned long flags)
{
	struct sun4i_spi_priv *priv = dev_get_priv(dev->parent);
	struct dm_spi_slave_plat *slave_plat = dev_get_parent_plat(dev);

	u32 len = bitlen / 8;
	u8 nbytes;
    u32 cnt = 0x100000;

	priv->tx_buf = dout;
	priv->rx_buf = din;

	if (bitlen % 8) {
		debug("%s: non byte-aligned SPI transfer.\n", __func__);
		return -ENAVAIL;
	}

	if (flags & SPI_XFER_BEGIN)
		sun4i_spi_set_cs(dev, slave_plat->cs, true);

	/* Reset FIFOs */
	setbits_le32(SPI_REG(priv, SPI_FCR), SPI_BIT(priv, SPI_FCR_RF_RST) |
		     SPI_BIT(priv, SPI_FCR_TF_RST));

	/* Reset FIFOs */
	setbits_le32(SPI_REG(priv, SPI_FCR), SPI_BIT(priv, SPI_FCR_RF_RST) |
		     SPI_BIT(priv, SPI_FCR_TF_RST));

	while (len) {
		/* Setup the transfer now... */
		nbytes = min(len, (priv->variant->fifo_depth - 1));

		/* Setup the counters */
		writel(SUN4I_BURST_CNT(nbytes), SPI_REG(priv, SPI_BC));
		writel(SUN4I_XMIT_CNT(nbytes), SPI_REG(priv, SPI_TC));

		if (priv->variant->has_burst_ctl)
            clrsetbits_le32(SPI_REG(priv, SPI_BCTL), SUN4I_MAX_XFER_SIZE, SUN4I_BURST_CNT(nbytes));
        
		/* Fill the TX FIFO */
		sun4i_spi_fill_fifo(priv, nbytes);

		/* Start the transfer */
		setbits_le32(SPI_REG(priv, SPI_TCR),
			     SPI_BIT(priv, SPI_TCR_XCH));

		/* Wait till RX FIFO to be empty */
		while((readl(SPI_REG(priv, SPI_FSR)) &
                (SPI_BIT(priv, SPI_FSR_RF_CNT_MASK)>>SUN4I_FIFO_STA_RF_CNT_BITS))
                 < nbytes)if(!cnt--)return -ETIMEDOUT;

		/* Drain the RX FIFO */
		sun4i_spi_drain_fifo(priv, nbytes);

		len -= nbytes;
	}

	if (flags & SPI_XFER_END)
		sun4i_spi_set_cs(dev, slave_plat->cs, false);

	return 0;
}

static const struct dm_spi_ops sun4i_spi_ops = {
    .xfer			= sun4i_spi_transfer,
	.set_speed		= sun4i_spi_set_speed,
	.set_mode		= sun4i_spi_set_mode,
	.claim_bus		= sun4i_spi_claim_bus,
	.release_bus		= sun4i_spi_release_bus,
};

static int sun4i_spi_probe(struct udevice *bus)
{
	struct sun4i_spi_plat *plat = dev_get_plat(bus);
	struct sun4i_spi_priv *priv = dev_get_priv(bus);

	priv->variant = plat->variant;
	priv->base = plat->base;
	priv->freq = plat->max_hz;

    sun4i_spi_parse_pins(bus);

	sun4i_spi_set_clock(bus);

	return 0;
}

static int sun4i_spi_of_to_plat(struct udevice *bus)
{
	struct sun4i_spi_plat *plat = dev_get_plat(bus);
	int node = dev_of_offset(bus);

	plat->base = (void *)dev_read_addr(bus);
	plat->variant = (struct sun4i_spi_variant *)dev_get_driver_data(bus);
	plat->max_hz = fdtdec_get_int(gd->fdt_blob, node,
				      "spi-max-frequency",
				      SUN4I_SPI_DEFAULT_RATE);

	if (plat->max_hz > SUN4I_SPI_MAX_RATE)
		plat->max_hz = SUN4I_SPI_MAX_RATE;

	return 0;
}

static const unsigned long sun4i_spi_regs[] = {
	[SPI_GCR]		= SUN4I_CTL_REG,
	[SPI_TCR]		= SUN4I_CTL_REG,
	[SPI_FCR]		= SUN4I_CTL_REG,
	[SPI_FSR]		= SUN4I_FIFO_STA_REG,
	[SPI_CCR]		= SUN4I_CLK_CTL_REG,
	[SPI_BC]		= SUN4I_BURST_CNT_REG,
	[SPI_TC]		= SUN4I_XMIT_CNT_REG,
	[SPI_TXD]		= SUN4I_TXDATA_REG,
	[SPI_RXD]		= SUN4I_RXDATA_REG,
};

static const u32 sun4i_spi_bits[] = {
	[SPI_GCR_TP]		= BIT(18),
	[SPI_TCR_CPHA]		= BIT(2),
	[SPI_TCR_CPOL]		= BIT(3),
	[SPI_TCR_CS_ACTIVE_LOW] = BIT(4),
	[SPI_TCR_XCH]		= BIT(10),
	[SPI_TCR_CS_SEL]	= 12,
	[SPI_TCR_CS_MASK]	= 0x3000,
	[SPI_TCR_CS_MANUAL]	= BIT(16),
	[SPI_TCR_CS_LEVEL]	= BIT(17),
	[SPI_FCR_TF_RST]	= BIT(8),
	[SPI_FCR_RF_RST]	= BIT(9),
	[SPI_FSR_RF_CNT_MASK]	= GENMASK(6, 0),
};

static const unsigned long sun6i_spi_regs[] = {
	[SPI_GCR]		= SUN6I_GBL_CTL_REG,
	[SPI_TCR]		= SUN6I_TFR_CTL_REG,
	[SPI_IER]		= SUN6I_INT_CTL_REG,
	[SPI_ISR]		= SUN6I_INT_STA_REG,
	[SPI_FCR]		= SUN6I_FIFO_CTL_REG,
	[SPI_FSR]		= SUN6I_FIFO_STA_REG,
	[SPI_CCR]		= SUN6I_CLK_CTL_REG,
	[SPI_BC]		= SUN6I_BURST_CNT_REG,
	[SPI_TC]		= SUN6I_XMIT_CNT_REG,
	[SPI_BCTL]		= SUN6I_BURST_CTL_REG,
	[SPI_TXD]		= SUN6I_TXDATA_REG,
	[SPI_RXD]		= SUN6I_RXDATA_REG,
};

static const u32 sun6i_spi_bits[] = {
	[SPI_GCR_TP]		= BIT(7),
	[SPI_GCR_SRST]		= BIT(31),
	[SPI_TCR_CPHA]		= BIT(0),
	[SPI_TCR_CPOL]		= BIT(1),
	[SPI_TCR_CS_ACTIVE_LOW] = BIT(2),
	[SPI_TCR_CS_SEL]	= 4,
	[SPI_TCR_CS_MASK]	= 0x30,
	[SPI_TCR_CS_MANUAL]	= BIT(6),
	[SPI_TCR_CS_LEVEL]	= BIT(7),
	[SPI_TCR_XCH]		= BIT(31),
	[SPI_FCR_RF_RST]	= BIT(15),
	[SPI_FCR_TF_RST]	= BIT(31),
	[SPI_FSR_RF_CNT_MASK]	= GENMASK(7, 0),
	[SPI_BCTL_QUAD_EN]	= BIT(29),
	[SPI_BCTL_DUAL_EN]	= BIT(28),
};

static const struct sun4i_spi_variant sun4i_a10_spi_variant = {
	.regs			= sun4i_spi_regs,
	.bits			= sun4i_spi_bits,
	.fifo_depth		= 64,
};

static const struct sun4i_spi_variant sun6i_a31_spi_variant = {
	.regs			= sun6i_spi_regs,
	.bits			= sun6i_spi_bits,
	.fifo_depth		= 128,
	.has_soft_reset		= true,
	.has_burst_ctl		= true,
};

static const struct sun4i_spi_variant sun8i_h3_spi_variant = {
	.regs			= sun6i_spi_regs,
	.bits			= sun6i_spi_bits,
	.fifo_depth		= 64,
	.has_soft_reset		= true,
	.has_burst_ctl		= true,
};

static const struct udevice_id sun4i_spi_ids[] = {
	{
	  .compatible = "allwinner,sun4i-a10-spi",
	  .data = (ulong)&sun4i_a10_spi_variant,
	},
	{
	  .compatible = "allwinner,sun6i-a31-spi",
	  .data = (ulong)&sun6i_a31_spi_variant,
	},
	{
	  .compatible = "allwinner,sun8i-h3-spi",
	  .data = (ulong)&sun8i_h3_spi_variant,
	},
	{
	  .compatible = "allwinner,sun20i-spi",
	  .data = (ulong)&sun8i_h3_spi_variant,
	},
	{ /* sentinel */ }
};

U_BOOT_DRIVER(sun4i_spi) = {
	.name	= "sun4i_spi",
	.id	= UCLASS_SPI,
	.of_match	= sun4i_spi_ids,
	.ops	= &sun4i_spi_ops,
	.of_to_plat	= sun4i_spi_of_to_plat,
	.plat_auto	= sizeof(struct sun4i_spi_plat),
	.priv_auto	= sizeof(struct sun4i_spi_priv),
	.probe	= sun4i_spi_probe,
};
