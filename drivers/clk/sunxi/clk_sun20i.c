// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (C) 2021 Jernej Skrabec <jernej.skrabec@siol.net>
 */

#include <common.h>
#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <clk/sunxi.h>
#include <linux/bitops.h>

static struct ccu_clk_gate d1s_gates[] = {
	[0]		= GATE(0x940, BIT(31)),
	[1]		= GATE(0x940, BIT(24)),
};

static struct ccu_reset d1s_resets[] = {
	[0]		= RESET(0x96c, BIT(16)),
};

static const struct ccu_desc d1s_ccu_desc = {
	.gates = d1s_gates,
	.resets = d1s_resets,
};

static int d1s_clk_bind(struct udevice *dev)
{
	return sunxi_reset_bind(dev, ARRAY_SIZE(d1s_resets));
}

static const struct udevice_id d1s_ccu_ids[] = {
	{ .compatible = "allwinner,sun20i-d1s-ccu",
	  .data = (ulong)&d1s_ccu_desc },
	{ }
};

U_BOOT_DRIVER(clk_sun20i_d1s) = {
	.name		= "sun20i_d1s_ccu",
	.id		= UCLASS_CLK,
	.of_match	= d1s_ccu_ids,
	.priv_auto	= sizeof(struct ccu_priv),
	.ops		= &sunxi_clk_ops,
	.probe		= sunxi_clk_probe,
	.bind		= d1s_clk_bind,
};
