// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Otto Pflüger
 *
 * Based on gcc-msm8953.c:
 *   Copyright 2021, The Linux Foundation. All rights reserved.
 * with parts taken from gcc-qcs404.c:
 *   Copyright 2018, The Linux Foundation. All rights reserved.
 * and gcc-msm8939.c:
 *   Copyright 2020 Linaro Limited
 * adapted with data from clock-gcc-8952.c in Qualcomm's msm-4.9 release:
 *   Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#include <dt-bindings/clock/qcom,gcc-msm8917.h>

#include "clk-alpha-pll.h"
#include "clk-branch.h"
#include "clk-pll.h"
#include "clk-rcg.h"
#include "common.h"
#include "gdsc.h"
#include "reset.h"

enum {
	DT_XO,
	DT_SLEEP_CLK,
	DT_DSI0PLL,
	DT_DSI0PLL_BYTE,
};

enum {
	P_XO,
	P_SLEEP_CLK,
	P_GPLL0,
	P_GPLL3,
	P_GPLL4,
	P_GPLL6,
	P_DSI0PLL,
	P_DSI0PLL_BYTE,
};

static struct clk_alpha_pll gpll0_sleep_clk_src = {
	.offset = 0x21000,
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_DEFAULT],
	.clkr = {
		.enable_reg = 0x45008,
		.enable_mask = BIT(23),
		.enable_is_inverted = true,
		.hw.init = &(struct clk_init_data){
			.name = "gpll0_sleep_clk_src",
			.parent_data = &(const struct clk_parent_data) {
				.index = DT_XO,
			},
			.num_parents = 1,
			.ops = &clk_branch_simple_ops,
		},
	},
};

static struct clk_alpha_pll gpll0_early = {
	.offset = 0x21000,
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_DEFAULT],
	.clkr = {
		.enable_reg = 0x45000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gpll0_early",
			.parent_hws = (const struct clk_hw*[]){
				&gpll0_sleep_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_fixed_ops,
		},
	},
};

static struct clk_alpha_pll_postdiv gpll0 = {
	.offset = 0x21000,
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_DEFAULT],
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gpll0",
		.parent_hws = (const struct clk_hw*[]){
			&gpll0_early.clkr.hw,
		},
		.num_parents = 1,
		.ops = &clk_alpha_pll_postdiv_ro_ops,
	},
};

static const struct pll_vco gpll3_p_vco[] = {
	{ 700000000, 1400000000, 0 },
};

static const struct alpha_pll_config gpll3_early_config = {
	.l = 63,
	.config_ctl_val = 0x4001055b,
	.early_output_mask = 0,
	.post_div_mask = GENMASK(11, 8),
	.post_div_val = BIT(8),
};

static struct clk_alpha_pll gpll3_early = {
	.offset = 0x22000,
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_DEFAULT],
	.vco_table = gpll3_p_vco,
	.num_vco = ARRAY_SIZE(gpll3_p_vco),
	.flags = SUPPORTS_DYNAMIC_UPDATE,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "gpll3_early",
			.parent_data = &(const struct clk_parent_data) {
				.index = DT_XO,
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_ops,
		},
	},
};

static struct clk_alpha_pll_postdiv gpll3 = {
	.offset = 0x22000,
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_DEFAULT],
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gpll3",
		.parent_hws = (const struct clk_hw*[]){
			&gpll3_early.clkr.hw,
		},
		.num_parents = 1,
		.ops = &clk_alpha_pll_postdiv_ops,
		.flags = CLK_SET_RATE_PARENT,
	},
};

static struct clk_alpha_pll gpll4_early = {
	.offset = 0x24000,
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_DEFAULT],
	.clkr = {
		.enable_reg = 0x45000,
		.enable_mask = BIT(5),
		.hw.init = &(struct clk_init_data){
			.name = "gpll4_early",
			.parent_data = &(const struct clk_parent_data) {
				.index = DT_XO,
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_fixed_ops,
		},
	},
};

static struct clk_alpha_pll_postdiv gpll4 = {
	.offset = 0x24000,
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_DEFAULT],
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gpll4",
		.parent_hws = (const struct clk_hw*[]){
			&gpll4_early.clkr.hw,
		},
		.num_parents = 1,
		.ops = &clk_alpha_pll_postdiv_ro_ops,
	},
};

static struct clk_pll gpll6_early = {
	.l_reg = 0x37004,
	.m_reg = 0x37008,
	.n_reg = 0x3700c,
	.config_reg = 0x37014,
	.mode_reg = 0x37000,
	.status_reg = 0x3701c,
	.status_bit = 17,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gpll6_early",
		.parent_data = &(const struct clk_parent_data) {
			.index = DT_XO,
		},
		.num_parents = 1,
		.ops = &clk_pll_ops,
	},
};

static struct clk_regmap gpll6 = {
	.enable_reg = 0x45000,
	.enable_mask = BIT(7),
	.hw.init = &(struct clk_init_data){
		.name = "gpll6",
		.parent_hws = (const struct clk_hw*[]){
			&gpll6_early.clkr.hw,
		},
		.num_parents = 1,
		.ops = &clk_pll_vote_ops,
	},
};

static const struct parent_map gcc_xo_gpll0_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0, 1 },
};

static const struct parent_map gcc_xo_gpll0_out_aux_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0, 2 },
};

static const struct clk_parent_data gcc_xo_gpll0_data[] = {
	{ .index = DT_XO },
	{ .hw = &gpll0.clkr.hw },
};

static const struct parent_map gcc_xo_gpll0_gpll6_sleep_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0, 1 },
	{ P_GPLL6, 2 },
	{ P_SLEEP_CLK, 6 },
};

static const struct clk_parent_data gcc_xo_gpll0_gpll6_sleep_data[] = {
	{ .index = DT_XO },
	{ .hw = &gpll0.clkr.hw },
	{ .hw = &gpll6.hw },
	{ .index = DT_SLEEP_CLK },
};

static const struct parent_map gcc_xo_gpll0_gpll6_gpll4_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0, 1 },
	{ P_GPLL6, 2 },
	{ P_GPLL4, 3 },
};

static const struct clk_parent_data gcc_xo_gpll0_gpll6_gpll4_data[] = {
	{ .index = DT_XO },
	{ .hw = &gpll0.clkr.hw },
	{ .hw = &gpll6.hw },
	{ .hw = &gpll4.clkr.hw },
};

static const struct freq_tbl ftbl_apss_ahb_clk_src[] = {
	F(19200000, P_XO, 1, 0, 0),
	F(50000000, P_GPLL0, 16, 0, 0),
	F(100000000, P_GPLL0, 8, 0, 0),
	F(133330000, P_GPLL0, 6, 0, 0),
	{ }
};

static struct clk_rcg2 apss_ahb_clk_src = {
	.cmd_rcgr = 0x46000,
	.hid_width = 5,
	.freq_tbl = ftbl_apss_ahb_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "apss_ahb_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_blsp_i2c_apps_clk_src[] = {
	F(19200000, P_XO, 1, 0, 0),
	F(50000000, P_GPLL0, 16, 0, 0),
	{ }
};

static struct clk_rcg2 blsp1_qup2_i2c_apps_clk_src = {
	.cmd_rcgr = 0x03000,
	.hid_width = 5,
	.freq_tbl = ftbl_blsp_i2c_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp1_qup2_i2c_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp1_qup3_i2c_apps_clk_src = {
	.cmd_rcgr = 0x04000,
	.hid_width = 5,
	.freq_tbl = ftbl_blsp_i2c_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp1_qup3_i2c_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp1_qup4_i2c_apps_clk_src = {
	.cmd_rcgr = 0x05000,
	.hid_width = 5,
	.freq_tbl = ftbl_blsp_i2c_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp1_qup4_i2c_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp2_qup1_i2c_apps_clk_src = {
	.cmd_rcgr = 0x0c00c,
	.hid_width = 5,
	.freq_tbl = ftbl_blsp_i2c_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp2_qup1_i2c_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp2_qup2_i2c_apps_clk_src = {
	.cmd_rcgr = 0x0d000,
	.hid_width = 5,
	.freq_tbl = ftbl_blsp_i2c_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp2_qup2_i2c_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp2_qup3_i2c_apps_clk_src = {
	.cmd_rcgr = 0x0f000,
	.hid_width = 5,
	.freq_tbl = ftbl_blsp_i2c_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp2_qup3_i2c_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_blsp_spi_apps_clk_src[] = {
	F(960000, P_XO, 10, 1, 2),
	F(4800000, P_XO, 4, 0, 0),
	F(9600000, P_XO, 2, 0, 0),
	F(16000000, P_GPLL0, 10, 1, 5),
	F(19200000, P_XO, 1, 0, 0),
	F(25000000, P_GPLL0, 16, 1, 2),
	F(50000000, P_GPLL0, 16, 0, 0),
	{ }
};

static struct clk_rcg2 blsp1_qup2_spi_apps_clk_src = {
	.cmd_rcgr = 0x03014,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_blsp_spi_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp1_qup2_spi_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp1_qup3_spi_apps_clk_src = {
	.cmd_rcgr = 0x04024,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_blsp_spi_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp1_qup3_spi_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp1_qup4_spi_apps_clk_src = {
	.cmd_rcgr = 0x05024,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_blsp_spi_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp1_qup4_spi_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp2_qup1_spi_apps_clk_src = {
	.cmd_rcgr = 0x0c024,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_blsp_spi_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp2_qup1_spi_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp2_qup2_spi_apps_clk_src = {
	.cmd_rcgr = 0x0d014,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_blsp_spi_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp2_qup2_spi_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp2_qup3_spi_apps_clk_src = {
	.cmd_rcgr = 0x0f024,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_blsp_spi_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp2_qup3_spi_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_blsp_uart_apps_clk_src[] = {
	F(3686400, P_GPLL0, 1, 72, 15625),
	F(7372800, P_GPLL0, 1, 144, 15625),
	F(14745600, P_GPLL0, 1, 288, 15625),
	F(16000000, P_GPLL0, 10, 1, 5),
	F(19200000, P_XO, 1, 0, 0),
	F(24000000, P_GPLL0, 1, 3, 100),
	F(25000000, P_GPLL0, 16, 1, 2),
	F(32000000, P_GPLL0, 1, 1, 25),
	F(40000000, P_GPLL0, 1, 1, 20),
	F(46400000, P_GPLL0, 1, 29, 500),
	F(48000000, P_GPLL0, 1, 3, 50),
	F(51200000, P_GPLL0, 1, 8, 125),
	F(56000000, P_GPLL0, 1, 7, 100),
	F(58982400, P_GPLL0, 1, 1152, 15625),
	F(60000000, P_GPLL0, 1, 3, 40),
	F(64000000, P_GPLL0, 1, 2, 25),
	{ }
};

static struct clk_rcg2 blsp1_uart1_apps_clk_src = {
	.cmd_rcgr = 0x02044,
	.hid_width = 5,
	.mnd_width = 16,
	.freq_tbl = ftbl_blsp_uart_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp1_uart1_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp1_uart2_apps_clk_src = {
	.cmd_rcgr = 0x03034,
	.hid_width = 5,
	.mnd_width = 16,
	.freq_tbl = ftbl_blsp_uart_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp1_uart2_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp2_uart1_apps_clk_src = {
	.cmd_rcgr = 0x0c044,
	.hid_width = 5,
	.mnd_width = 16,
	.freq_tbl = ftbl_blsp_uart_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp2_uart1_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 blsp2_uart2_apps_clk_src = {
	.cmd_rcgr = 0x0d034,
	.hid_width = 5,
	.mnd_width = 16,
	.freq_tbl = ftbl_blsp_uart_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "blsp2_uart2_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct parent_map gcc_byte0_map[] = {
	{ P_XO, 0 },
	{ P_DSI0PLL_BYTE, 1 },
};

static const struct clk_parent_data gcc_byte_data[] = {
	{ .index = DT_XO },
	{ .index = DT_DSI0PLL_BYTE },
};

static struct clk_rcg2 byte0_clk_src = {
	.cmd_rcgr = 0x4d044,
	.hid_width = 5,
	.parent_map = gcc_byte0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "byte0_clk_src",
		.parent_data = gcc_byte_data,
		.num_parents = ARRAY_SIZE(gcc_byte_data),
		.ops = &clk_byte2_ops,
		.flags = CLK_SET_RATE_PARENT,
	}
};

static const struct freq_tbl ftbl_camss_gp_clk_src[] = {
	F(100000000, P_GPLL0, 8, 0, 0),
	F(160000000, P_GPLL0, 5, 0, 0),
	F(200000000, P_GPLL0, 4, 0, 0),
	{ }
};

static struct clk_rcg2 camss_gp0_clk_src = {
	.cmd_rcgr = 0x54000,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_camss_gp_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_sleep_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "camss_gp0_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_sleep_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_sleep_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 camss_gp1_clk_src = {
	.cmd_rcgr = 0x55000,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_camss_gp_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_sleep_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "camss_gp1_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_sleep_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_sleep_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_camss_top_ahb_clk_src[] = {
	F(40000000, P_GPLL0, 10, 1, 2),
	F(61540000, P_GPLL0, 13, 0, 0),
	F(80000000, P_GPLL0, 10, 0, 0),
	{ }
};

static struct clk_rcg2 camss_top_ahb_clk_src = {
	.cmd_rcgr = 0x5a000,
	.hid_width = 5,
	.freq_tbl = ftbl_camss_top_ahb_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "camss_top_ahb_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_cci_clk_src[] = {
	F(19200000, P_XO, 1, 0, 0),
	F(37500000, P_GPLL0, 1, 3, 64),
	{ }
};

static struct clk_rcg2 cci_clk_src = {
	.cmd_rcgr = 0x51000,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_cci_clk_src,
	.parent_map = gcc_xo_gpll0_out_aux_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "cci_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct parent_map gcc_cpp_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0, 1 },
	{ P_GPLL6, 3 },
};

static const struct clk_parent_data gcc_cpp_data[] = {
	{ .index = DT_XO },
	{ .hw = &gpll0.clkr.hw },
	{ .hw = &gpll6.hw },
};

static const struct freq_tbl ftbl_cpp_clk_src[] = {
	F(133330000, P_GPLL0, 6, 0, 0),
	F(160000000, P_GPLL0, 5, 0, 0),
	F(266670000, P_GPLL0, 3, 0, 0),
	F(308570000, P_GPLL0, 3.5, 0, 0),
	F(320000000, P_GPLL0, 2.5, 0, 0),
	F(360000000, P_GPLL6, 3, 0, 0),
	{ }
};

static struct clk_rcg2 cpp_clk_src = {
	.cmd_rcgr = 0x58018,
	.hid_width = 5,
	.freq_tbl = ftbl_cpp_clk_src,
	.parent_map = gcc_cpp_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "cpp_clk_src",
		.parent_data = gcc_cpp_data,
		.num_parents = ARRAY_SIZE(gcc_cpp_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_crypto_clk_src[] = {
	F(50000000, P_GPLL0, 16, 0, 0),
	F(80000000, P_GPLL0, 10, 0, 0),
	F(100000000, P_GPLL0, 8, 0, 0),
	F(160000000, P_GPLL0, 5, 0, 0),
	{ }
};

static struct clk_rcg2 crypto_clk_src = {
	.cmd_rcgr = 0x16004,
	.hid_width = 5,
	.freq_tbl = ftbl_crypto_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "crypto_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_csi_clk_src[] = {
	F(100000000, P_GPLL0, 8, 0, 0),
	F(160000000, P_GPLL0, 5, 0, 0),
	F(200000000, P_GPLL0, 4, 0, 0),
	{ }
};

static struct clk_rcg2 csi0_clk_src = {
	.cmd_rcgr = 0x4e020,
	.hid_width = 5,
	.freq_tbl = ftbl_csi_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "csi0_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 csi1_clk_src = {
	.cmd_rcgr = 0x4f020,
	.hid_width = 5,
	.freq_tbl = ftbl_csi_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "csi1_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 csi2_clk_src = {
	.cmd_rcgr = 0x3c020,
	.hid_width = 5,
	.freq_tbl = ftbl_csi_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "csi2_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_csi_phytimer_clk_src[] = {
	F(100000000, P_GPLL0, 8, 0, 0),
	F(160000000, P_GPLL0, 5, 0, 0),
	F(200000000, P_GPLL0, 4, 0, 0),
	F(266670000, P_GPLL0, 3, 0, 0),
	{ }
};

static struct clk_rcg2 csi0phytimer_clk_src = {
	.cmd_rcgr = 0x4e000,
	.hid_width = 5,
	.freq_tbl = ftbl_csi_phytimer_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "csi0phytimer_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 csi1phytimer_clk_src = {
	.cmd_rcgr = 0x4f000,
	.hid_width = 5,
	.freq_tbl = ftbl_csi_phytimer_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "csi1phytimer_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_esc0_1_clk_src[] = {
	F(19200000, P_XO, 1, 0, 0),
	{ }
};

static struct clk_rcg2 esc0_clk_src = {
	.cmd_rcgr = 0x4d05c,
	.hid_width = 5,
	.freq_tbl = ftbl_esc0_1_clk_src,
	.parent_map = gcc_xo_gpll0_out_aux_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "esc0_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct parent_map gcc_gfx3d_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0, 1 },
	{ P_GPLL3, 2 },
	{ P_GPLL6, 3 },
};

static const struct parent_map gcc_gfx3d_map_qm215[] = {
	{ P_XO, 0 },
	{ P_GPLL0, 5 },
	{ P_GPLL3, 2 },
	{ P_GPLL6, 6 },
};

static const struct clk_parent_data gcc_gfx3d_data[] = {
	{ .index = DT_XO },
	{ .hw = &gpll0.clkr.hw },
	{ .hw = &gpll3.clkr.hw },
	{ .hw = &gpll6.hw },
};

static const struct freq_tbl ftbl_gfx3d_clk_src[] = {
	F(19200000, P_XO, 1, 0, 0),
	F(50000000, P_GPLL0, 16, 0, 0),
	F(80000000, P_GPLL0, 10, 0, 0),
	F(100000000, P_GPLL0, 8, 0, 0),
	F(160000000, P_GPLL0, 5, 0, 0),
	F(200000000, P_GPLL0, 4, 0, 0),
	F(228570000, P_GPLL0, 3.5, 0, 0),
	F(240000000, P_GPLL6, 4.5, 0, 0),
	F(266670000, P_GPLL0, 3, 0, 0),
	F(270000000, P_GPLL6, 4, 0, 0),
	F(320000000, P_GPLL0, 2.5, 0, 0),
	F(400000000, P_GPLL0, 2, 0, 0),
	F(465000000, P_GPLL3, 1, 0, 0),
	F(484800000, P_GPLL3, 1, 0, 0),
	F(500000000, P_GPLL3, 1, 0, 0),
	F(523200000, P_GPLL3, 1, 0, 0),
	F(550000000, P_GPLL3, 1, 0, 0),
	F(598000000, P_GPLL3, 1, 0, 0),
	{ }
};

static struct clk_rcg2 gfx3d_clk_src = {
	.cmd_rcgr = 0x59000,
	.hid_width = 5,
	.freq_tbl = ftbl_gfx3d_clk_src,
	.parent_map = gcc_gfx3d_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "gfx3d_clk_src",
		.parent_data = gcc_gfx3d_data,
		.num_parents = ARRAY_SIZE(gcc_gfx3d_data),
		.ops = &clk_rcg2_ops,
		.flags = CLK_SET_RATE_PARENT | CLK_OPS_PARENT_ENABLE,
	}
};

static const struct freq_tbl ftbl_gp_clk_src[] = {
	F(19200000, P_XO, 1, 0, 0),
	{ }
};

static struct clk_rcg2 gp1_clk_src = {
	.cmd_rcgr = 0x08004,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_gp_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_sleep_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "gp1_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_sleep_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_sleep_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 gp2_clk_src = {
	.cmd_rcgr = 0x09004,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_gp_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_sleep_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "gp2_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_sleep_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_sleep_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 gp3_clk_src = {
	.cmd_rcgr = 0x0a004,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_gp_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_sleep_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "gp3_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_sleep_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_sleep_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_jpeg0_clk_src[] = {
	F(133330000, P_GPLL0, 6, 0, 0),
	F(266670000, P_GPLL0, 3, 0, 0),
	F(320000000, P_GPLL0, 2.5, 0, 0),
	{ }
};

static struct clk_rcg2 jpeg0_clk_src = {
	.cmd_rcgr = 0x57000,
	.hid_width = 5,
	.freq_tbl = ftbl_jpeg0_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "jpeg0_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_mclk_clk_src[] = {
	F(19200000, P_XO, 1, 0, 0),
	F(24000000, P_GPLL6, 1, 1, 45),
	F(66667000, P_GPLL0, 12, 0, 0),
	{ }
};

static struct clk_rcg2 mclk0_clk_src = {
	.cmd_rcgr = 0x52000,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_mclk_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_sleep_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "mclk0_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_sleep_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_sleep_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 mclk1_clk_src = {
	.cmd_rcgr = 0x53000,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_mclk_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_sleep_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "mclk1_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_sleep_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_sleep_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 mclk2_clk_src = {
	.cmd_rcgr = 0x5c000,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_mclk_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_sleep_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "mclk2_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_sleep_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_sleep_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_mdp_clk_src[] = {
	F(50000000, P_GPLL0, 16, 0, 0),
	F(80000000, P_GPLL0, 10, 0, 0),
	F(100000000, P_GPLL0, 8, 0, 0),
	F(145450000, P_GPLL0, 5.5, 0, 0),
	F(160000000, P_GPLL0, 5, 0, 0),
	F(177780000, P_GPLL0, 4.5, 0, 0),
	F(200000000, P_GPLL0, 4, 0, 0),
	F(266670000, P_GPLL0, 3, 0, 0),
	F(320000000, P_GPLL0, 2.5, 0, 0),
	{ }
};

static struct clk_rcg2 mdp_clk_src = {
	.cmd_rcgr = 0x4d014,
	.hid_width = 5,
	.freq_tbl = ftbl_mdp_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "mdp_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct parent_map gcc_pclk_map[] = {
	{ P_XO, 0 },
	{ P_DSI0PLL, 1 },
};

static const struct clk_parent_data gcc_pclk_data[] = {
	{ .index = DT_XO },
	{ .index = DT_DSI0PLL },
};

static struct clk_rcg2 pclk0_clk_src = {
	.cmd_rcgr = 0x4d000,
	.hid_width = 5,
	.mnd_width = 8,
	.parent_map = gcc_pclk_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "pclk0_clk_src",
		.parent_data = gcc_pclk_data,
		.num_parents = ARRAY_SIZE(gcc_pclk_data),
		.ops = &clk_pixel_ops,
		.flags = CLK_SET_RATE_PARENT,
	}
};

static const struct freq_tbl ftbl_pdm2_clk_src[] = {
	F(64000000, P_GPLL0, 12.5, 0, 0),
	{ }
};

static struct clk_rcg2 pdm2_clk_src = {
	.cmd_rcgr = 0x44010,
	.hid_width = 5,
	.freq_tbl = ftbl_pdm2_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "pdm2_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_sdcc1_ice_core_clk_src[] = {
	F(100000000, P_GPLL0, 8, 0, 0),
	F(200000000, P_GPLL0, 4, 0, 0),
	{ }
};

static struct clk_rcg2 sdcc1_ice_core_clk_src = {
	.cmd_rcgr = 0x5d000,
	.hid_width = 5,
	.freq_tbl = ftbl_sdcc1_ice_core_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "sdcc1_ice_core_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct parent_map gcc_sdcc1_apps_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0, 1 },
	{ P_GPLL4, 2 },
};

static const struct clk_parent_data gcc_sdcc1_apss_data[] = {
	{ .index = DT_XO },
	{ .hw = &gpll0.clkr.hw },
	{ .hw = &gpll4.clkr.hw },
};

static const struct freq_tbl ftbl_sdcc1_apps_clk_src[] = {
	F(144000, P_XO, 16, 3, 25),
	F(400000, P_XO, 12, 1, 4),
	F(20000000, P_GPLL0, 10, 1, 4),
	F(25000000, P_GPLL0, 16, 1, 2),
	F(50000000, P_GPLL0, 16, 0, 0),
	F(100000000, P_GPLL0, 8, 0, 0),
	F(177770000, P_GPLL0, 4.5, 0, 0),
	F(192000000, P_GPLL4, 6, 0, 0),
	F(200000000, P_GPLL0, 4, 0, 0),
	F(384000000, P_GPLL4, 3, 0, 0),
	{ }
};

static struct clk_rcg2 sdcc1_apps_clk_src = {
	.cmd_rcgr = 0x42004,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_sdcc1_apps_clk_src,
	.parent_map = gcc_sdcc1_apps_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "sdcc1_apps_clk_src",
		.parent_data = gcc_sdcc1_apss_data,
		.num_parents = ARRAY_SIZE(gcc_sdcc1_apss_data),
		.ops = &clk_rcg2_floor_ops,
	}
};

static const struct freq_tbl ftbl_sdcc2_apps_clk_src[] = {
	F(144000, P_XO, 16, 3, 25),
	F(400000, P_XO, 12, 1, 4),
	F(20000000, P_GPLL0, 10, 1, 4),
	F(25000000, P_GPLL0, 16, 1, 2),
	F(50000000, P_GPLL0, 16, 0, 0),
	F(100000000, P_GPLL0, 8, 0, 0),
	F(177770000, P_GPLL0, 4.5, 0, 0),
	F(200000000, P_GPLL0, 4, 0, 0),
	{ }
};

static struct clk_rcg2 sdcc2_apps_clk_src = {
	.cmd_rcgr = 0x43004,
	.hid_width = 5,
	.mnd_width = 8,
	.freq_tbl = ftbl_sdcc2_apps_clk_src,
	.parent_map = gcc_xo_gpll0_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "sdcc2_apps_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_floor_ops,
	}
};

static const struct freq_tbl ftbl_usb_hs_system_clk_src[] = {
	F(80000000, P_GPLL0, 10, 0, 0),
	F(100000000, P_GPLL0, 8, 0, 0),
	F(133330000, P_GPLL0, 6, 0, 0),
	F(177780000, P_GPLL0, 4.5, 0, 0),
	{ }
};

static struct clk_rcg2 usb_hs_system_clk_src = {
	.cmd_rcgr = 0x41010,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_usb_hs_system_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "usb_hs_system_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_vcodec0_clk_src[] = {
	F(133330000, P_GPLL0, 6, 0, 0),
	F(180000000, P_GPLL6, 6, 0, 0),
	F(228570000, P_GPLL0, 3.5, 0, 0),
	F(266670000, P_GPLL0, 3, 0, 0),
	F(308570000, P_GPLL6, 3.5, 0, 0),
	F(329140000, P_GPLL4, 3.5, 0, 0),
	F(360000000, P_GPLL6, 3, 0, 0),
	{ }
};

static struct clk_rcg2 vcodec0_clk_src = {
	.cmd_rcgr = 0x4c000,
	.hid_width = 5,
	.freq_tbl = ftbl_vcodec0_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_gpll4_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "vcodec0_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_gpll4_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_gpll4_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_vfe_clk_src[] = {
	F(50000000, P_GPLL0, 16, 0, 0),
	F(80000000, P_GPLL0, 10, 0, 0),
	F(100000000, P_GPLL0, 8, 0, 0),
	F(133330000, P_GPLL0, 6, 0, 0),
	F(160000000, P_GPLL0, 5, 0, 0),
	F(200000000, P_GPLL0, 4, 0, 0),
	F(266670000, P_GPLL0, 3, 0, 0),
	F(308570000, P_GPLL6, 3.5, 0, 0),
	F(320000000, P_GPLL0, 2.5, 0, 0),
	F(329140000, P_GPLL4, 3.5, 0, 0),
	F(360000000, P_GPLL6, 3, 0, 0),
	{ }
};

static struct clk_rcg2 vfe0_clk_src = {
	.cmd_rcgr = 0x58000,
	.hid_width = 5,
	.freq_tbl = ftbl_vfe_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_gpll4_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "vfe0_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_gpll4_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_gpll4_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_rcg2 vfe1_clk_src = {
	.cmd_rcgr = 0x58054,
	.hid_width = 5,
	.freq_tbl = ftbl_vfe_clk_src,
	.parent_map = gcc_xo_gpll0_gpll6_gpll4_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "vfe1_clk_src",
		.parent_data = gcc_xo_gpll0_gpll6_gpll4_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_gpll6_gpll4_data),
		.ops = &clk_rcg2_ops,
	}
};

static const struct freq_tbl ftbl_vsync_clk_src[] = {
	F(19200000, P_XO, 1, 0, 0),
	{ }
};

static struct clk_rcg2 vsync_clk_src = {
	.cmd_rcgr = 0x4d02c,
	.hid_width = 5,
	.freq_tbl = ftbl_vsync_clk_src,
	.parent_map = gcc_xo_gpll0_out_aux_map,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "vsync_clk_src",
		.parent_data = gcc_xo_gpll0_data,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_data),
		.ops = &clk_rcg2_ops,
	}
};

static struct clk_branch gcc_apss_tcu_clk = {
	.halt_reg = 0x12018,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(1),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_apss_tcu_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_bimc_gfx_clk = {
	.halt_reg = 0x59034,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x59034,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_bimc_gfx_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_bimc_gpu_clk = {
	.halt_reg = 0x59030,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x59030,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_bimc_gpu_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_blsp1_ahb_clk = {
	.halt_reg = 0x01008,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(10),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp1_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_blsp2_ahb_clk = {
	.halt_reg = 0x0b008,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(20),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp2_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_blsp1_qup2_i2c_apps_clk = {
	.halt_reg = 0x03010,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x03010,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp1_qup2_i2c_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp1_qup2_i2c_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp1_qup3_i2c_apps_clk = {
	.halt_reg = 0x04020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x04020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp1_qup3_i2c_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp1_qup3_i2c_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp1_qup4_i2c_apps_clk = {
	.halt_reg = 0x05020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x05020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp1_qup4_i2c_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp1_qup4_i2c_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp2_qup1_i2c_apps_clk = {
	.halt_reg = 0x0c008,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0c008,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp2_qup1_i2c_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp2_qup1_i2c_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp2_qup2_i2c_apps_clk = {
	.halt_reg = 0x0d010,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0d010,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp2_qup2_i2c_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp2_qup2_i2c_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp2_qup3_i2c_apps_clk = {
	.halt_reg = 0x0f020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0f020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp2_qup3_i2c_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp2_qup3_i2c_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp1_qup2_spi_apps_clk = {
	.halt_reg = 0x0300c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0300c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp1_qup2_spi_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp1_qup2_spi_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp1_qup3_spi_apps_clk = {
	.halt_reg = 0x0401c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0401c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp1_qup3_spi_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp1_qup3_spi_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp1_qup4_spi_apps_clk = {
	.halt_reg = 0x0501c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0501c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp1_qup4_spi_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp1_qup4_spi_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp2_qup1_spi_apps_clk = {
	.halt_reg = 0x0c004,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0c004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp2_qup1_spi_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp2_qup1_spi_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp2_qup2_spi_apps_clk = {
	.halt_reg = 0x0d00c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0d00c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp2_qup2_spi_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp2_qup2_spi_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp2_qup3_spi_apps_clk = {
	.halt_reg = 0x0f01c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0f01c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp2_qup3_spi_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp2_qup3_spi_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp1_uart1_apps_clk = {
	.halt_reg = 0x0203c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0203c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp1_uart1_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp1_uart1_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp1_uart2_apps_clk = {
	.halt_reg = 0x0302c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0302c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp1_uart2_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp1_uart2_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp2_uart1_apps_clk = {
	.halt_reg = 0x0c03c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0c03c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp2_uart1_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp2_uart1_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_blsp2_uart2_apps_clk = {
	.halt_reg = 0x0d02c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0d02c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_blsp2_uart2_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&blsp2_uart2_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_boot_rom_ahb_clk = {
	.halt_reg = 0x1300c,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(7),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_boot_rom_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_camss_ahb_clk = {
	.halt_reg = 0x56004,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x56004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_camss_cci_ahb_clk = {
	.halt_reg = 0x5101c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x5101c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_cci_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_cci_clk = {
	.halt_reg = 0x51018,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x51018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_cci_clk",
			.parent_hws = (const struct clk_hw*[]){
				&cci_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_cpp_ahb_clk = {
	.halt_reg = 0x58040,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x58040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_cpp_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_cpp_clk = {
	.halt_reg = 0x5803c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x5803c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_cpp_clk",
			.parent_hws = (const struct clk_hw*[]){
				&cpp_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi0_ahb_clk = {
	.halt_reg = 0x4e040,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4e040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi0_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi1_ahb_clk = {
	.halt_reg = 0x4f040,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4f040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi1_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi2_ahb_clk = {
	.halt_reg = 0x3c040,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x3c040,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi2_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi0_clk = {
	.halt_reg = 0x4e03c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4e03c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi1_clk = {
	.halt_reg = 0x4f03c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4f03c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi1_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi1_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi2_clk = {
	.halt_reg = 0x3c03c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x3c03c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi2_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi2_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi0phy_clk = {
	.halt_reg = 0x4e048,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4e048,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi0phy_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi1phy_clk = {
	.halt_reg = 0x4f048,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4f048,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi1phy_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi1_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi2phy_clk = {
	.halt_reg = 0x3c048,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x3c048,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi2phy_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi2_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi0phytimer_clk = {
	.halt_reg = 0x4e01c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4e01c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi0phytimer_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi0phytimer_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi1phytimer_clk = {
	.halt_reg = 0x4f01c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4f01c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi1phytimer_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi1phytimer_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi0pix_clk = {
	.halt_reg = 0x4e058,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4e058,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi0pix_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi1pix_clk = {
	.halt_reg = 0x4f058,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4f058,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi1pix_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi1_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi2pix_clk = {
	.halt_reg = 0x3c058,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x3c058,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi2pix_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi2_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi0rdi_clk = {
	.halt_reg = 0x4e050,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4e050,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi0rdi_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi1rdi_clk = {
	.halt_reg = 0x4f050,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4f050,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi1rdi_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi1_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi2rdi_clk = {
	.halt_reg = 0x3c050,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x3c050,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi2rdi_clk",
			.parent_hws = (const struct clk_hw*[]){
				&csi2_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi_vfe0_clk = {
	.halt_reg = 0x58050,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x58050,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi_vfe0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&vfe0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_csi_vfe1_clk = {
	.halt_reg = 0x58074,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x58074,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_csi_vfe1_clk",
			.parent_hws = (const struct clk_hw*[]){
				&vfe1_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_gp0_clk = {
	.halt_reg = 0x54018,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x54018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_gp0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_gp0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_gp1_clk = {
	.halt_reg = 0x55018,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x55018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_gp1_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_gp1_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_ispif_ahb_clk = {
	.halt_reg = 0x50004,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x50004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_ispif_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_jpeg0_clk = {
	.halt_reg = 0x57020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x57020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_jpeg0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&jpeg0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_jpeg_ahb_clk = {
	.halt_reg = 0x57024,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x57024,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_jpeg_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_jpeg_axi_clk = {
	.halt_reg = 0x57028,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x57028,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_jpeg_axi_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_camss_mclk0_clk = {
	.halt_reg = 0x52018,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x52018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_mclk0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&mclk0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_mclk1_clk = {
	.halt_reg = 0x53018,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x53018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_mclk1_clk",
			.parent_hws = (const struct clk_hw*[]){
				&mclk1_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_mclk2_clk = {
	.halt_reg = 0x5c018,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x5c018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_mclk2_clk",
			.parent_hws = (const struct clk_hw*[]){
				&mclk2_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_micro_ahb_clk = {
	.halt_reg = 0x5600c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x5600c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_micro_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_top_ahb_clk = {
	.halt_reg = 0x5a014,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x5a014,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_top_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_vfe0_ahb_clk = {
	.halt_reg = 0x58044,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x58044,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_vfe0_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_vfe0_axi_clk = {
	.halt_reg = 0x58048,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x58048,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_vfe0_axi_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_camss_vfe0_clk = {
	.halt_reg = 0x58038,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x58038,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_vfe0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&vfe0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_vfe1_ahb_clk = {
	.halt_reg = 0x58060,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x58060,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_vfe1_ahb_clk",
			.parent_hws = (const struct clk_hw*[]){
				&camss_top_ahb_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_camss_vfe1_axi_clk = {
	.halt_reg = 0x58068,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x58068,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_vfe1_axi_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_camss_vfe1_clk = {
	.halt_reg = 0x5805c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x5805c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_camss_vfe1_clk",
			.parent_hws = (const struct clk_hw*[]){
				&vfe1_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_cpp_tbu_clk = {
	.halt_reg = 0x12040,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(14),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_cpp_tbu_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_crypto_ahb_clk = {
	.halt_reg = 0x16024,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_crypto_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_crypto_axi_clk = {
	.halt_reg = 0x16020,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(1),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_crypto_axi_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_crypto_clk = {
	.halt_reg = 0x1601c,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_crypto_clk",
			.parent_hws = (const struct clk_hw*[]){
				&crypto_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_dcc_clk = {
	.halt_reg = 0x77004,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x77004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_dcc_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_gfx_tbu_clk = {
	.halt_reg = 0x12010,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(3),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_gfx_tbu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_gfx_tcu_clk = {
	.halt_reg = 0x12020,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(2),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_gfx_tcu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_gtcu_ahb_clk = {
	.halt_reg = 0x12044,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(13),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_gtcu_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_gp1_clk = {
	.halt_reg = 0x08000,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x08000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_gp1_clk",
			.parent_hws = (const struct clk_hw*[]){
				&gp1_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_gp2_clk = {
	.halt_reg = 0x09000,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x09000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_gp2_clk",
			.parent_hws = (const struct clk_hw*[]){
				&gp2_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_gp3_clk = {
	.halt_reg = 0x0a000,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x0a000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_gp3_clk",
			.parent_hws = (const struct clk_hw*[]){
				&gp3_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_jpeg_tbu_clk = {
	.halt_reg = 0x12034,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(10),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_jpeg_tbu_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_mdp_tbu_clk = {
	.halt_reg = 0x1201c,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(4),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_mdp_tbu_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_mdss_ahb_clk = {
	.halt_reg = 0x4d07c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d07c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_mdss_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_mdss_axi_clk = {
	.halt_reg = 0x4d080,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d080,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_mdss_axi_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_mdss_byte0_clk = {
	.halt_reg = 0x4d094,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d094,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_mdss_byte0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&byte0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_mdss_esc0_clk = {
	.halt_reg = 0x4d098,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d098,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_mdss_esc0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&esc0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_mdss_mdp_clk = {
	.halt_reg = 0x4d088,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d088,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_mdss_mdp_clk",
			.parent_hws = (const struct clk_hw*[]){
				&mdp_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_mdss_pclk0_clk = {
	.halt_reg = 0x4d084,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d084,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_mdss_pclk0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&pclk0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_mdss_vsync_clk = {
	.halt_reg = 0x4d090,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4d090,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_mdss_vsync_clk",
			.parent_hws = (const struct clk_hw*[]){
				&vsync_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_mss_cfg_ahb_clk = {
	.halt_reg = 0x49000,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x49000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_mss_cfg_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_mss_q6_bimc_axi_clk = {
	.halt_reg = 0x49004,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x49004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_mss_q6_bimc_axi_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_oxili_ahb_clk = {
	.halt_reg = 0x59028,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x59028,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_oxili_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_oxili_gfx3d_clk = {
	.halt_reg = 0x59020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x59020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_oxili_gfx3d_clk",
			.parent_hws = (const struct clk_hw*[]){
				&gfx3d_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_pdm2_clk = {
	.halt_reg = 0x4400c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4400c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_pdm2_clk",
			.parent_hws = (const struct clk_hw*[]){
				&pdm2_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_pdm_ahb_clk = {
	.halt_reg = 0x44004,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x44004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_pdm_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_prng_ahb_clk = {
	.halt_reg = 0x13004,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(8),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_prng_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_qdss_dap_clk = {
	.halt_reg = 0x29084,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(11),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_qdss_dap_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_sdcc1_ice_core_clk = {
	.halt_reg = 0x5d014,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x5d014,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_sdcc1_ice_core_clk",
			.parent_hws = (const struct clk_hw*[]){
				&sdcc1_ice_core_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_sdcc1_ahb_clk = {
	.halt_reg = 0x4201c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4201c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_sdcc1_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_sdcc2_ahb_clk = {
	.halt_reg = 0x4301c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4301c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_sdcc2_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_sdcc1_apps_clk = {
	.halt_reg = 0x42018,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x42018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_sdcc1_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&sdcc1_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_sdcc2_apps_clk = {
	.halt_reg = 0x43018,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x43018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_sdcc2_apps_clk",
			.parent_hws = (const struct clk_hw*[]){
				&sdcc2_apps_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_smmu_cfg_clk = {
	.halt_reg = 0x12038,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(12),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_smmu_cfg_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_usb2a_phy_sleep_clk = {
	.halt_reg = 0x4102c,
	.clkr = {
		.enable_reg = 0x4102c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_usb2a_phy_sleep_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_usb_hs_ahb_clk = {
	.halt_reg = 0x41008,
	.clkr = {
		.enable_reg = 0x41008,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_usb_hs_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_usb_hs_phy_cfg_ahb_clk = {
	.halt_reg = 0x41030,
	.clkr = {
		.enable_reg = 0x41030,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_usb_hs_phy_cfg_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_usb_hs_system_clk = {
	.halt_reg = 0x41004,
	.clkr = {
		.enable_reg = 0x41004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_usb_hs_system_clk",
			.parent_hws = (const struct clk_hw*[]){
				&usb_hs_system_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_venus0_ahb_clk = {
	.halt_reg = 0x4c020,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4c020,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_venus0_ahb_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_venus0_axi_clk = {
	.halt_reg = 0x4c024,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4c024,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_venus0_axi_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_venus0_core0_vcodec0_clk = {
	.halt_reg = 0x4c02c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4c02c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_venus0_core0_vcodec0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&vcodec0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_venus0_vcodec0_clk = {
	.halt_reg = 0x4c01c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x4c01c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_venus0_vcodec0_clk",
			.parent_hws = (const struct clk_hw*[]){
				&vcodec0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		}
	}
};

static struct clk_branch gcc_venus_tbu_clk = {
	.halt_reg = 0x12014,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(5),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_venus_tbu_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_vfe1_tbu_clk = {
	.halt_reg = 0x12090,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(17),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_vfe1_tbu_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct clk_branch gcc_vfe_tbu_clk = {
	.halt_reg = 0x1203c,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x4500c,
		.enable_mask = BIT(9),
		.hw.init = &(struct clk_init_data) {
			.name = "gcc_vfe_tbu_clk",
			.ops = &clk_branch2_ops,
		}
	}
};

static struct gdsc venus_gdsc = {
	.gdscr = 0x4c018,
	.cxcs = (unsigned int []){ 0x4c024, 0x4c01c },
	.cxc_count = 2,
	.pd = {
		.name = "venus_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
};

static struct gdsc venus_core0_gdsc = {
	.gdscr = 0x4c028,
	.cxcs = (unsigned int []){ 0x4c02c },
	.cxc_count = 1,
	.pd = {
		.name = "venus_core0",
	},
	.flags = HW_CTRL,
	.pwrsts = PWRSTS_OFF_ON,
};

static struct gdsc mdss_gdsc = {
	.gdscr = 0x4d078,
	.cxcs = (unsigned int []){ 0x4d080, 0x4d088 },
	.cxc_count = 2,
	.pd = {
		.name = "mdss_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
};

static struct gdsc jpeg_gdsc = {
	.gdscr = 0x5701c,
	.cxcs = (unsigned int []){ 0x57020, 0x57028 },
	.cxc_count = 2,
	.pd = {
		.name = "jpeg_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
};

static struct gdsc vfe0_gdsc = {
	.gdscr = 0x58034,
	.cxcs = (unsigned int []){ 0x58038, 0x58048, 0x5600c, 0x58050 },
	.cxc_count = 4,
	.pd = {
		.name = "vfe0_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
};

static struct gdsc vfe1_gdsc = {
	.gdscr = 0x5806c,
	.cxcs = (unsigned int []){ 0x5805c, 0x58068, 0x5600c, 0x58074 },
	.cxc_count = 4,
	.pd = {
		.name = "vfe1_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
};

static struct gdsc oxili_gx_gdsc = {
	.gdscr = 0x5901c,
	.clamp_io_ctrl = 0x5b00c,
	.cxcs = (unsigned int []){ 0x59000, 0x59020 },
	.cxc_count = 2,
	.pd = {
		.name = "oxili_gx_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = CLAMP_IO,
};

static struct gdsc cpp_gdsc = {
	.gdscr = 0x58078,
	.cxcs = (unsigned int []){ 0x5803c, 0x58064 },
	.cxc_count = 2,
	.pd = {
		.name = "cpp_gdsc",
	},
	.flags = ALWAYS_ON,
	.pwrsts = PWRSTS_OFF_ON,
};

static struct clk_regmap *gcc_msm8917_clocks[] = {
	[GPLL0] = &gpll0.clkr,
	[GPLL0_EARLY] = &gpll0_early.clkr,
	[GPLL0_SLEEP_CLK_SRC] = &gpll0_sleep_clk_src.clkr,
	[GPLL3] = &gpll3.clkr,
	[GPLL3_EARLY] = &gpll3_early.clkr,
	[GPLL4] = &gpll4.clkr,
	[GPLL4_EARLY] = &gpll4_early.clkr,
	[GPLL6] = &gpll6,
	[GPLL6_EARLY] = &gpll6_early.clkr,
	[APSS_AHB_CLK_SRC] = &apss_ahb_clk_src.clkr,
	[BLSP1_QUP2_I2C_APPS_CLK_SRC] = &blsp1_qup2_i2c_apps_clk_src.clkr,
	[BLSP1_QUP2_SPI_APPS_CLK_SRC] = &blsp1_qup2_spi_apps_clk_src.clkr,
	[BLSP1_QUP3_I2C_APPS_CLK_SRC] = &blsp1_qup3_i2c_apps_clk_src.clkr,
	[BLSP1_QUP3_SPI_APPS_CLK_SRC] = &blsp1_qup3_spi_apps_clk_src.clkr,
	[BLSP1_QUP4_I2C_APPS_CLK_SRC] = &blsp1_qup4_i2c_apps_clk_src.clkr,
	[BLSP1_QUP4_SPI_APPS_CLK_SRC] = &blsp1_qup4_spi_apps_clk_src.clkr,
	[BLSP1_UART1_APPS_CLK_SRC] = &blsp1_uart1_apps_clk_src.clkr,
	[BLSP1_UART2_APPS_CLK_SRC] = &blsp1_uart2_apps_clk_src.clkr,
	[BLSP2_QUP1_I2C_APPS_CLK_SRC] = &blsp2_qup1_i2c_apps_clk_src.clkr,
	[BLSP2_QUP1_SPI_APPS_CLK_SRC] = &blsp2_qup1_spi_apps_clk_src.clkr,
	[BLSP2_QUP2_I2C_APPS_CLK_SRC] = &blsp2_qup2_i2c_apps_clk_src.clkr,
	[BLSP2_QUP2_SPI_APPS_CLK_SRC] = &blsp2_qup2_spi_apps_clk_src.clkr,
	[BLSP2_QUP3_I2C_APPS_CLK_SRC] = &blsp2_qup3_i2c_apps_clk_src.clkr,
	[BLSP2_QUP3_SPI_APPS_CLK_SRC] = &blsp2_qup3_spi_apps_clk_src.clkr,
	[BLSP2_UART1_APPS_CLK_SRC] = &blsp2_uart1_apps_clk_src.clkr,
	[BLSP2_UART2_APPS_CLK_SRC] = &blsp2_uart2_apps_clk_src.clkr,
	[BYTE0_CLK_SRC] = &byte0_clk_src.clkr,
	[CAMSS_GP0_CLK_SRC] = &camss_gp0_clk_src.clkr,
	[CAMSS_GP1_CLK_SRC] = &camss_gp1_clk_src.clkr,
	[CAMSS_TOP_AHB_CLK_SRC] = &camss_top_ahb_clk_src.clkr,
	[CCI_CLK_SRC] = &cci_clk_src.clkr,
	[CPP_CLK_SRC] = &cpp_clk_src.clkr,
	[CRYPTO_CLK_SRC] = &crypto_clk_src.clkr,
	[CSI0PHYTIMER_CLK_SRC] = &csi0phytimer_clk_src.clkr,
	[CSI0_CLK_SRC] = &csi0_clk_src.clkr,
	[CSI1PHYTIMER_CLK_SRC] = &csi1phytimer_clk_src.clkr,
	[CSI1_CLK_SRC] = &csi1_clk_src.clkr,
	[CSI2_CLK_SRC] = &csi2_clk_src.clkr,
	[ESC0_CLK_SRC] = &esc0_clk_src.clkr,
	[GFX3D_CLK_SRC] = &gfx3d_clk_src.clkr,
	[GP1_CLK_SRC] = &gp1_clk_src.clkr,
	[GP2_CLK_SRC] = &gp2_clk_src.clkr,
	[GP3_CLK_SRC] = &gp3_clk_src.clkr,
	[JPEG0_CLK_SRC] = &jpeg0_clk_src.clkr,
	[MCLK0_CLK_SRC] = &mclk0_clk_src.clkr,
	[MCLK1_CLK_SRC] = &mclk1_clk_src.clkr,
	[MCLK2_CLK_SRC] = &mclk2_clk_src.clkr,
	[MDP_CLK_SRC] = &mdp_clk_src.clkr,
	[PCLK0_CLK_SRC] = &pclk0_clk_src.clkr,
	[PDM2_CLK_SRC] = &pdm2_clk_src.clkr,
	[SDCC1_APPS_CLK_SRC] = &sdcc1_apps_clk_src.clkr,
	[SDCC1_ICE_CORE_CLK_SRC] = &sdcc1_ice_core_clk_src.clkr,
	[SDCC2_APPS_CLK_SRC] = &sdcc2_apps_clk_src.clkr,
	[USB_HS_SYSTEM_CLK_SRC] = &usb_hs_system_clk_src.clkr,
	[VCODEC0_CLK_SRC] = &vcodec0_clk_src.clkr,
	[VFE0_CLK_SRC] = &vfe0_clk_src.clkr,
	[VFE1_CLK_SRC] = &vfe1_clk_src.clkr,
	[VSYNC_CLK_SRC] = &vsync_clk_src.clkr,
	[GCC_APSS_TCU_CLK] = &gcc_apss_tcu_clk.clkr,
	[GCC_BIMC_GFX_CLK] = &gcc_bimc_gfx_clk.clkr,
	[GCC_BIMC_GPU_CLK] = &gcc_bimc_gpu_clk.clkr,
	[GCC_BLSP1_AHB_CLK] = &gcc_blsp1_ahb_clk.clkr,
	[GCC_BLSP1_QUP2_I2C_APPS_CLK] = &gcc_blsp1_qup2_i2c_apps_clk.clkr,
	[GCC_BLSP1_QUP2_SPI_APPS_CLK] = &gcc_blsp1_qup2_spi_apps_clk.clkr,
	[GCC_BLSP1_QUP3_I2C_APPS_CLK] = &gcc_blsp1_qup3_i2c_apps_clk.clkr,
	[GCC_BLSP1_QUP3_SPI_APPS_CLK] = &gcc_blsp1_qup3_spi_apps_clk.clkr,
	[GCC_BLSP1_QUP4_I2C_APPS_CLK] = &gcc_blsp1_qup4_i2c_apps_clk.clkr,
	[GCC_BLSP1_QUP4_SPI_APPS_CLK] = &gcc_blsp1_qup4_spi_apps_clk.clkr,
	[GCC_BLSP1_UART1_APPS_CLK] = &gcc_blsp1_uart1_apps_clk.clkr,
	[GCC_BLSP1_UART2_APPS_CLK] = &gcc_blsp1_uart2_apps_clk.clkr,
	[GCC_BLSP2_AHB_CLK] = &gcc_blsp2_ahb_clk.clkr,
	[GCC_BLSP2_QUP1_I2C_APPS_CLK] = &gcc_blsp2_qup1_i2c_apps_clk.clkr,
	[GCC_BLSP2_QUP1_SPI_APPS_CLK] = &gcc_blsp2_qup1_spi_apps_clk.clkr,
	[GCC_BLSP2_QUP2_I2C_APPS_CLK] = &gcc_blsp2_qup2_i2c_apps_clk.clkr,
	[GCC_BLSP2_QUP2_SPI_APPS_CLK] = &gcc_blsp2_qup2_spi_apps_clk.clkr,
	[GCC_BLSP2_QUP3_I2C_APPS_CLK] = &gcc_blsp2_qup3_i2c_apps_clk.clkr,
	[GCC_BLSP2_QUP3_SPI_APPS_CLK] = &gcc_blsp2_qup3_spi_apps_clk.clkr,
	[GCC_BLSP2_UART1_APPS_CLK] = &gcc_blsp2_uart1_apps_clk.clkr,
	[GCC_BLSP2_UART2_APPS_CLK] = &gcc_blsp2_uart2_apps_clk.clkr,
	[GCC_BOOT_ROM_AHB_CLK] = &gcc_boot_rom_ahb_clk.clkr,
	[GCC_CAMSS_AHB_CLK] = &gcc_camss_ahb_clk.clkr,
	[GCC_CAMSS_CCI_AHB_CLK] = &gcc_camss_cci_ahb_clk.clkr,
	[GCC_CAMSS_CCI_CLK] = &gcc_camss_cci_clk.clkr,
	[GCC_CAMSS_CPP_AHB_CLK] = &gcc_camss_cpp_ahb_clk.clkr,
	[GCC_CAMSS_CPP_CLK] = &gcc_camss_cpp_clk.clkr,
	[GCC_CAMSS_CSI0PHYTIMER_CLK] = &gcc_camss_csi0phytimer_clk.clkr,
	[GCC_CAMSS_CSI0PHY_CLK] = &gcc_camss_csi0phy_clk.clkr,
	[GCC_CAMSS_CSI0PIX_CLK] = &gcc_camss_csi0pix_clk.clkr,
	[GCC_CAMSS_CSI0RDI_CLK] = &gcc_camss_csi0rdi_clk.clkr,
	[GCC_CAMSS_CSI0_AHB_CLK] = &gcc_camss_csi0_ahb_clk.clkr,
	[GCC_CAMSS_CSI0_CLK] = &gcc_camss_csi0_clk.clkr,
	[GCC_CAMSS_CSI1PHYTIMER_CLK] = &gcc_camss_csi1phytimer_clk.clkr,
	[GCC_CAMSS_CSI1PHY_CLK] = &gcc_camss_csi1phy_clk.clkr,
	[GCC_CAMSS_CSI1PIX_CLK] = &gcc_camss_csi1pix_clk.clkr,
	[GCC_CAMSS_CSI1RDI_CLK] = &gcc_camss_csi1rdi_clk.clkr,
	[GCC_CAMSS_CSI1_AHB_CLK] = &gcc_camss_csi1_ahb_clk.clkr,
	[GCC_CAMSS_CSI1_CLK] = &gcc_camss_csi1_clk.clkr,
	[GCC_CAMSS_CSI2PHY_CLK] = &gcc_camss_csi2phy_clk.clkr,
	[GCC_CAMSS_CSI2PIX_CLK] = &gcc_camss_csi2pix_clk.clkr,
	[GCC_CAMSS_CSI2RDI_CLK] = &gcc_camss_csi2rdi_clk.clkr,
	[GCC_CAMSS_CSI2_AHB_CLK] = &gcc_camss_csi2_ahb_clk.clkr,
	[GCC_CAMSS_CSI2_CLK] = &gcc_camss_csi2_clk.clkr,
	[GCC_CAMSS_CSI_VFE0_CLK] = &gcc_camss_csi_vfe0_clk.clkr,
	[GCC_CAMSS_CSI_VFE1_CLK] = &gcc_camss_csi_vfe1_clk.clkr,
	[GCC_CAMSS_GP0_CLK] = &gcc_camss_gp0_clk.clkr,
	[GCC_CAMSS_GP1_CLK] = &gcc_camss_gp1_clk.clkr,
	[GCC_CAMSS_ISPIF_AHB_CLK] = &gcc_camss_ispif_ahb_clk.clkr,
	[GCC_CAMSS_JPEG0_CLK] = &gcc_camss_jpeg0_clk.clkr,
	[GCC_CAMSS_JPEG_AHB_CLK] = &gcc_camss_jpeg_ahb_clk.clkr,
	[GCC_CAMSS_JPEG_AXI_CLK] = &gcc_camss_jpeg_axi_clk.clkr,
	[GCC_CAMSS_MCLK0_CLK] = &gcc_camss_mclk0_clk.clkr,
	[GCC_CAMSS_MCLK1_CLK] = &gcc_camss_mclk1_clk.clkr,
	[GCC_CAMSS_MCLK2_CLK] = &gcc_camss_mclk2_clk.clkr,
	[GCC_CAMSS_MICRO_AHB_CLK] = &gcc_camss_micro_ahb_clk.clkr,
	[GCC_CAMSS_TOP_AHB_CLK] = &gcc_camss_top_ahb_clk.clkr,
	[GCC_CAMSS_VFE0_AHB_CLK] = &gcc_camss_vfe0_ahb_clk.clkr,
	[GCC_CAMSS_VFE0_AXI_CLK] = &gcc_camss_vfe0_axi_clk.clkr,
	[GCC_CAMSS_VFE0_CLK] = &gcc_camss_vfe0_clk.clkr,
	[GCC_CAMSS_VFE1_AHB_CLK] = &gcc_camss_vfe1_ahb_clk.clkr,
	[GCC_CAMSS_VFE1_AXI_CLK] = &gcc_camss_vfe1_axi_clk.clkr,
	[GCC_CAMSS_VFE1_CLK] = &gcc_camss_vfe1_clk.clkr,
	[GCC_CPP_TBU_CLK] = &gcc_cpp_tbu_clk.clkr,
	[GCC_CRYPTO_AHB_CLK] = &gcc_crypto_ahb_clk.clkr,
	[GCC_CRYPTO_AXI_CLK] = &gcc_crypto_axi_clk.clkr,
	[GCC_CRYPTO_CLK] = &gcc_crypto_clk.clkr,
	[GCC_DCC_CLK] = &gcc_dcc_clk.clkr,
	[GCC_GFX_TBU_CLK] = &gcc_gfx_tbu_clk.clkr,
	[GCC_GFX_TCU_CLK] = &gcc_gfx_tcu_clk.clkr,
	[GCC_GP1_CLK] = &gcc_gp1_clk.clkr,
	[GCC_GP2_CLK] = &gcc_gp2_clk.clkr,
	[GCC_GP3_CLK] = &gcc_gp3_clk.clkr,
	[GCC_GTCU_AHB_CLK] = &gcc_gtcu_ahb_clk.clkr,
	[GCC_JPEG_TBU_CLK] = &gcc_jpeg_tbu_clk.clkr,
	[GCC_MDP_TBU_CLK] = &gcc_mdp_tbu_clk.clkr,
	[GCC_MDSS_AHB_CLK] = &gcc_mdss_ahb_clk.clkr,
	[GCC_MDSS_AXI_CLK] = &gcc_mdss_axi_clk.clkr,
	[GCC_MDSS_BYTE0_CLK] = &gcc_mdss_byte0_clk.clkr,
	[GCC_MDSS_ESC0_CLK] = &gcc_mdss_esc0_clk.clkr,
	[GCC_MDSS_MDP_CLK] = &gcc_mdss_mdp_clk.clkr,
	[GCC_MDSS_PCLK0_CLK] = &gcc_mdss_pclk0_clk.clkr,
	[GCC_MDSS_VSYNC_CLK] = &gcc_mdss_vsync_clk.clkr,
	[GCC_MSS_CFG_AHB_CLK] = &gcc_mss_cfg_ahb_clk.clkr,
	[GCC_MSS_Q6_BIMC_AXI_CLK] = &gcc_mss_q6_bimc_axi_clk.clkr,
	[GCC_OXILI_AHB_CLK] = &gcc_oxili_ahb_clk.clkr,
	[GCC_OXILI_GFX3D_CLK] = &gcc_oxili_gfx3d_clk.clkr,
	[GCC_PDM2_CLK] = &gcc_pdm2_clk.clkr,
	[GCC_PDM_AHB_CLK] = &gcc_pdm_ahb_clk.clkr,
	[GCC_PRNG_AHB_CLK] = &gcc_prng_ahb_clk.clkr,
	[GCC_QDSS_DAP_CLK] = &gcc_qdss_dap_clk.clkr,
	[GCC_SDCC1_AHB_CLK] = &gcc_sdcc1_ahb_clk.clkr,
	[GCC_SDCC1_APPS_CLK] = &gcc_sdcc1_apps_clk.clkr,
	[GCC_SDCC1_ICE_CORE_CLK] = &gcc_sdcc1_ice_core_clk.clkr,
	[GCC_SDCC2_AHB_CLK] = &gcc_sdcc2_ahb_clk.clkr,
	[GCC_SDCC2_APPS_CLK] = &gcc_sdcc2_apps_clk.clkr,
	[GCC_SMMU_CFG_CLK] = &gcc_smmu_cfg_clk.clkr,
	[GCC_USB2A_PHY_SLEEP_CLK] = &gcc_usb2a_phy_sleep_clk.clkr,
	[GCC_USB_HS_AHB_CLK] = &gcc_usb_hs_ahb_clk.clkr,
	[GCC_USB_HS_PHY_CFG_AHB_CLK] = &gcc_usb_hs_phy_cfg_ahb_clk.clkr,
	[GCC_USB_HS_SYSTEM_CLK] = &gcc_usb_hs_system_clk.clkr,
	[GCC_VENUS0_AHB_CLK] = &gcc_venus0_ahb_clk.clkr,
	[GCC_VENUS0_AXI_CLK] = &gcc_venus0_axi_clk.clkr,
	[GCC_VENUS0_CORE0_VCODEC0_CLK] = &gcc_venus0_core0_vcodec0_clk.clkr,
	[GCC_VENUS0_VCODEC0_CLK] = &gcc_venus0_vcodec0_clk.clkr,
	[GCC_VENUS_TBU_CLK] = &gcc_venus_tbu_clk.clkr,
	[GCC_VFE1_TBU_CLK] = &gcc_vfe1_tbu_clk.clkr,
	[GCC_VFE_TBU_CLK] = &gcc_vfe_tbu_clk.clkr,
};

static const struct qcom_reset_map gcc_msm8917_resets[] = {
	[GCC_CAMSS_MICRO_BCR]		= { 0x56008 },
	[GCC_MSS_BCR]			= { 0x71000 },
	[GCC_QUSB2_PHY_BCR]		= { 0x4103c },
	[GCC_USB_HS_BCR]		= { 0x41000 },
	[GCC_USB2_HS_PHY_ONLY_BCR]	= { 0x41034 },
};

static const struct regmap_config gcc_msm8917_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x80000,
	.fast_io	= true,
};

static struct gdsc *gcc_msm8917_gdscs[] = {
	[CPP_GDSC] = &cpp_gdsc,
	[JPEG_GDSC] = &jpeg_gdsc,
	[MDSS_GDSC] = &mdss_gdsc,
	[OXILI_GX_GDSC] = &oxili_gx_gdsc,
	[VENUS_CORE0_GDSC] = &venus_core0_gdsc,
	[VENUS_GDSC] = &venus_gdsc,
	[VFE0_GDSC] = &vfe0_gdsc,
	[VFE1_GDSC] = &vfe1_gdsc,
};

static const struct qcom_cc_desc gcc_msm8917_desc = {
	.config = &gcc_msm8917_regmap_config,
	.clks = gcc_msm8917_clocks,
	.num_clks = ARRAY_SIZE(gcc_msm8917_clocks),
	.resets = gcc_msm8917_resets,
	.num_resets = ARRAY_SIZE(gcc_msm8917_resets),
	.gdscs = gcc_msm8917_gdscs,
	.num_gdscs = ARRAY_SIZE(gcc_msm8917_gdscs),
};

static const struct qcom_cc_desc gcc_qm215_desc = {
	.config = &gcc_msm8917_regmap_config,
	.clks = gcc_msm8917_clocks,
	.num_clks = ARRAY_SIZE(gcc_msm8917_clocks),
	.resets = gcc_msm8917_resets,
	.num_resets = ARRAY_SIZE(gcc_msm8917_resets),
	.gdscs = gcc_msm8917_gdscs,
	.num_gdscs = ARRAY_SIZE(gcc_msm8917_gdscs),
};

static int gcc_msm8917_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	const struct qcom_cc_desc *gcc_desc;

	gcc_desc = of_device_get_match_data(&pdev->dev);

	if (gcc_desc == &gcc_qm215_desc)
		gfx3d_clk_src.parent_map = gcc_gfx3d_map_qm215;

	regmap = qcom_cc_map(pdev, gcc_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	clk_alpha_pll_configure(&gpll3_early, regmap, &gpll3_early_config);

	return qcom_cc_really_probe(&pdev->dev, gcc_desc, regmap);
}

static const struct of_device_id gcc_msm8917_match_table[] = {
	{ .compatible = "qcom,gcc-msm8917", .data = &gcc_msm8917_desc },
	{ .compatible = "qcom,gcc-qm215", .data = &gcc_qm215_desc },
	{},
};

static struct platform_driver gcc_msm8917_driver = {
	.probe = gcc_msm8917_probe,
	.driver = {
		.name = "gcc-msm8917",
		.of_match_table = gcc_msm8917_match_table,
	},
};

static int __init gcc_msm8917_init(void)
{
	return platform_driver_register(&gcc_msm8917_driver);
}
core_initcall(gcc_msm8917_init);

static void __exit gcc_msm8917_exit(void)
{
	platform_driver_unregister(&gcc_msm8917_driver);
}
module_exit(gcc_msm8917_exit);

MODULE_DESCRIPTION("Qualcomm GCC MSM8917 Driver");
MODULE_LICENSE("GPL");
