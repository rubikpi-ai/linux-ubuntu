// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) "clk: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "clk-debug.h"
#include "common.h"

static struct measure_clk_data debug_mux_priv = {
	.ctl_reg = 0x72038,
	.status_reg = 0x7203c,
	.xo_div4_cbcr = 0x6e008,
};

static const char *const apss_cc_debug_mux_parent_names[] = {
	"measure_only_apcs_cl1_l3_clk",
	"measure_only_apcs_cl0_clk",
	"measure_only_apcs_cl1_clk",
	"measure_only_apcs_cl0_l3_clk",
};

static int apss_cc_debug_mux_sels[] = {
	0x61,		/* measure_only_apcs_cl1_l3_clk */
	0x21,		/* measure_only_apcs_cl0_clk */
	0x25,		/* measure_only_apcs_cl1_clk */
	0x41,		/* measure_only_apcs_cl0_l3_clk */
};

static int apss_cc_debug_mux_pre_divs[] = {
	0x4,		/* measure_only_apcs_cl1_l3_clk */
	0x8,		/* measure_only_apcs_cl0_clk */
	0x8,		/* measure_only_apcs_cl1_clk */
	0x4,		/* measure_only_apcs_cl0_l3_clk */
};

static struct clk_debug_mux apss_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x18,
	.post_div_offset = 0x18,
	.cbcr_offset = 0x0,
	.src_sel_mask = 0x7f0,
	.src_sel_shift = 4,
	.post_div_mask = 0x7800,
	.post_div_shift = 11,
	.post_div_val = 1,
	.mux_sels = apss_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(apss_cc_debug_mux_sels),
	.pre_div_vals = apss_cc_debug_mux_pre_divs,
	.hw.init = &(const struct clk_init_data){
		.name = "apss_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = apss_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(apss_cc_debug_mux_parent_names),
	},
};

static const char *const gcc_debug_mux_parent_names[] = {
	"apss_cc_debug_mux",
	"mc_cc_debug_mux",
};

static int gcc_debug_mux_sels[] = {
	0x175,		/* apss_cc_debug_mux */
	0x149,		/* mc_cc_debug_mux */
};

static struct clk_debug_mux gcc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x72024,
	.post_div_offset = 0x6e000,
	.cbcr_offset = 0x6e004,
	.src_sel_mask = 0x1fff,
	.src_sel_shift = 0,
	.post_div_mask = 0xf,
	.post_div_shift = 0,
	.post_div_val = 2,
	.mux_sels = gcc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(gcc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "gcc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = gcc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(gcc_debug_mux_parent_names),
	},
};

static const char *const mc_cc_debug_mux_parent_names[] = {
	"measure_only_mccc_clk",
};

static struct clk_debug_mux mc_cc_debug_mux = {
	.period_offset = 0x50,
	.hw.init = &(struct clk_init_data){
		.name = "mc_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = mc_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(mc_cc_debug_mux_parent_names),
	},
};

static struct mux_regmap_names mux_list[] = {
	{ .mux = &mc_cc_debug_mux, .regmap_name = "qcom,mccc" },
	{ .mux = &apss_cc_debug_mux, .regmap_name = "qcom,apsscc" },
	{ .mux = &gcc_debug_mux, .regmap_name = "qcom,gcc" },
};

static struct clk_fixed_rate measure_only_mccc_clk = {
	.fixed_rate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_mccc_clk",
		.ops = &clk_fixed_rate_measure_ops,
	},
};

static struct clk_fixed_rate measure_only_apcs_cl0_clk = {
	.fixed_rate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_apcs_cl0_clk",
		.ops = &clk_fixed_rate_measure_ops,
	},
};

static struct clk_fixed_rate measure_only_apcs_cl1_clk = {
	.fixed_rate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_apcs_cl1_clk",
		.ops = &clk_fixed_rate_measure_ops,
	},
};

static struct clk_fixed_rate measure_only_apcs_cl0_l3_clk = {
	.fixed_rate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_apcs_cl0_l3_clk",
		.ops = &clk_fixed_rate_measure_ops,
	},
};

static struct clk_fixed_rate measure_only_apcs_cl1_l3_clk = {
	.fixed_rate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_apcs_cl1_l3_clk",
		.ops = &clk_fixed_rate_measure_ops,
	},
};

static struct clk_hw *debugcc_measure_hws[] = {
	&measure_only_mccc_clk.hw,
	&measure_only_apcs_cl0_clk.hw,
	&measure_only_apcs_cl1_clk.hw,
	&measure_only_apcs_cl0_l3_clk.hw,
	&measure_only_apcs_cl1_l3_clk.hw,
};

static const struct of_device_id clk_debug_match_table[] = {
	{ .compatible = "qcom,measure-debugcc-sa8775p" },
	{ }
};

static int clk_debug_measure_probe(struct platform_device *pdev)
{
	struct clk *clk;
	int ret, i;

	clk = devm_clk_get(&pdev->dev, "xo_clk_src");
	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Unable to get xo clock\n");
		return PTR_ERR(clk);
	}

	debug_mux_priv.cxo = clk;

	for (i = 0; i < ARRAY_SIZE(mux_list); i++) {
		if (IS_ERR_OR_NULL(mux_list[i].mux->regmap)) {
			ret = map_debug_bases(pdev, mux_list[i].regmap_name,
					      mux_list[i].mux);
			if (ret == -EBADR)
				continue;
			else if (ret)
				return ret;
		}
	}

	for (i = 0; i < ARRAY_SIZE(debugcc_measure_hws); i++) {
		clk = devm_clk_register(&pdev->dev, debugcc_measure_hws[i]);
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Unable to register (%d), err:(%ld)\n",
				i, PTR_ERR(clk));
			return PTR_ERR(clk);
		}
	}

	for (i = 0; i < ARRAY_SIZE(mux_list); i++) {
		ret = devm_clk_register_debug_mux(&pdev->dev, mux_list[i].mux);
		if (ret) {
			dev_err(&pdev->dev, "Unable to register mux clk %d, err:(%d)\n", i, ret);
			return ret;
		}
	}

	ret = clk_debug_measure_register(&gcc_debug_mux.hw);
	if (ret) {
		dev_err(&pdev->dev, "Could not register Measure clocks\n");
		return ret;
	}

	dev_info(&pdev->dev, "Registered debug measure clocks\n");

	return ret;
}

static struct platform_driver clk_debug_driver = {
	.probe = clk_debug_measure_probe,
	.driver = {
		.name = "measure-debugcc",
		.of_match_table = clk_debug_match_table,
	},
};

static int __init clk_debug_measure_init(void)
{
	return platform_driver_register(&clk_debug_driver);
}
fs_initcall(clk_debug_measure_init);

MODULE_DESCRIPTION("QTI DEBUG CC MEASURE Driver");
MODULE_LICENSE("GPL");
