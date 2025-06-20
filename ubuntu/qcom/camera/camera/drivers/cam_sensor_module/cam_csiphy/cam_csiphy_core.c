// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>

#include <dt-bindings/msm-camera.h>

#include "cam_compat.h"
#include "cam_csiphy_core.h"
#include "cam_csiphy_dev.h"
#include "cam_csiphy_soc.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "cam_mem_mgr.h"
#include "cam_cpas_api.h"
#include "cam_compat.h"

#define SCM_SVC_CAMERASS 0x18
#define SECURE_SYSCALL_ID 0x6
#define SECURE_SYSCALL_ID_2 0x7

#define LANE_MASK_2PH 0x1F
#define LANE_MASK_3PH 0x7

/* Size of CPAS_SEC_LANE_CP_CTRL register mask */
#define SEC_LANE_CP_REG_LEN 32
/*
 * PHY index at which CPAS_SEC_LANE_CP_CTRL register mask
 * changes depending on PHY HW version
 */
#define CAM_MAX_PHYS_PER_CP_CTRL_REG 4

static DEFINE_MUTEX(active_csiphy_cnt_mutex);
static DEFINE_MUTEX(main_aon_selection);

static int csiphy_onthego_reg_count;
static unsigned int csiphy_onthego_regs[150];
module_param_array(csiphy_onthego_regs, uint, &csiphy_onthego_reg_count, 0644);
MODULE_PARM_DESC(csiphy_onthego_regs, "Functionality to let csiphy registers program on the fly");

struct g_csiphy_data {
	void __iomem *base_address;
	uint8_t is_3phase;
	uint32_t cpas_handle;
	bool is_configured_for_main;
	uint64_t data_rate_aux_mask;
	uint32_t aon_cam_id;
	struct cam_csiphy_aon_sel_params_t *aon_sel_param;
};

static struct g_csiphy_data g_phy_data[MAX_CSIPHY] = {0};
static int active_csiphy_hw_cnt;

void cam_csiphy_update_auxiliary_mask(struct csiphy_device *csiphy_dev)
{
	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "Invalid param");
		return;
	}

	if (!g_phy_data[csiphy_dev->soc_info.index].is_3phase) {
		CAM_INFO_RATE_LIMIT(CAM_CSIPHY, "2PH Sensor is connected to the PHY");
		return;
	}

	g_phy_data[csiphy_dev->soc_info.index].data_rate_aux_mask |=
			BIT_ULL(csiphy_dev->curr_data_rate_idx);

	/* check if userland has provided a buffer for data rate aux mask */
	if (csiphy_dev->aux_params.aux_mem_update_en) {

		*csiphy_dev->aux_params.aux_config_ptr =
			g_phy_data[csiphy_dev->soc_info.index].data_rate_aux_mask;
	}

	CAM_DBG(CAM_CSIPHY,
		"CSIPHY:%u configuring aux settings curr_data_rate_idx: %u curr_data_rate: %llu curr_aux_mask: 0x%lx updated in memory: %s",
		csiphy_dev->soc_info.index, csiphy_dev->curr_data_rate_idx,
		csiphy_dev->current_data_rate,
		g_phy_data[csiphy_dev->soc_info.index].data_rate_aux_mask,
		CAM_BOOL_TO_YESNO(csiphy_dev->aux_params.aux_mem_update_en));
}

int32_t cam_csiphy_get_instance_offset(struct csiphy_device *csiphy_dev, int32_t dev_handle)
{
	int32_t i = 0;

	if ((csiphy_dev->acquire_count >
		csiphy_dev->session_max_device_support) ||
		(csiphy_dev->acquire_count < 0)) {
		CAM_ERR(CAM_CSIPHY,
			"Invalid acquire count: %d, Max supported device for session: %u",
			csiphy_dev->acquire_count,
			csiphy_dev->session_max_device_support);
		return -EINVAL;
	}

	for (i = 0; i < csiphy_dev->acquire_count; i++) {
		if (dev_handle ==
			csiphy_dev->csiphy_info[i].hdl_data.device_hdl)
			break;
	}

	return i;
}

static int cam_csiphy_cpas_ops(
	uint32_t cpas_handle, bool start)
{
	int rc = 0;
	struct cam_ahb_vote ahb_vote;
	struct cam_axi_vote axi_vote = {0};

	if (start) {
		ahb_vote.type = CAM_VOTE_ABSOLUTE;
		ahb_vote.vote.level = CAM_LOWSVS_D1_VOTE;
		axi_vote.num_paths = 1;
		axi_vote.axi_path[0].path_data_type =
			CAM_AXI_PATH_DATA_ALL;
		axi_vote.axi_path[0].transac_type =
			CAM_AXI_TRANSACTION_WRITE;
		axi_vote.axi_path[0].camnoc_bw =
			CAM_CPAS_DEFAULT_AXI_BW;
		axi_vote.axi_path[0].mnoc_ab_bw =
			CAM_CPAS_DEFAULT_AXI_BW;
		axi_vote.axi_path[0].mnoc_ib_bw =
			CAM_CPAS_DEFAULT_AXI_BW;

		rc = cam_cpas_start(cpas_handle,
			&ahb_vote, &axi_vote);
		if (rc < 0) {
			CAM_ERR(CAM_CSIPHY, "voting CPAS: %d", rc);
			return rc;
		}
		CAM_DBG(CAM_CSIPHY, "CPAS START");
	} else {
		rc = cam_cpas_stop(cpas_handle);
		if (rc < 0) {
			CAM_ERR(CAM_CSIPHY, "de-voting CPAS: %d", rc);
			return rc;
		}
		CAM_DBG(CAM_CSIPHY, "CPAS STOPPED");
	}

	return rc;
}

static void cam_csiphy_reset_phyconfig_param(struct csiphy_device *csiphy_dev,
	int32_t index)
{
	CAM_DBG(CAM_CSIPHY, "Resetting phyconfig param at index: %d", index);
	csiphy_dev->csiphy_info[index].lane_cnt = 0;
	csiphy_dev->csiphy_info[index].lane_assign = 0;
	csiphy_dev->csiphy_info[index].lane_enable = 0;
	csiphy_dev->csiphy_info[index].settle_time = 0;
	csiphy_dev->csiphy_info[index].data_rate = 0;
	csiphy_dev->csiphy_info[index].secure_mode = 0;
	csiphy_dev->csiphy_info[index].mipi_flags = 0;
	csiphy_dev->csiphy_info[index].hdl_data.device_hdl = -1;
	csiphy_dev->csiphy_info[index].csiphy_3phase = -1;
	csiphy_dev->csiphy_info[index].conn_csid_idx = -1;
	csiphy_dev->csiphy_info[index].use_hw_client_voting = false;
	csiphy_dev->csiphy_info[index].is_drv_config_en = false;
}

static inline void cam_csiphy_apply_onthego_reg_values(void __iomem *csiphybase, uint8_t csiphy_idx)
{
	int                                                      i;

	CAM_DBG(CAM_CSIPHY, "csiphy: %d, onthego_reg_count: %d",
		csiphy_idx,
		csiphy_onthego_reg_count);

	if (csiphy_onthego_reg_count % 3)
		csiphy_onthego_reg_count -= (csiphy_onthego_reg_count % 3);

	for (i = 0; i < csiphy_onthego_reg_count; i += 3) {
		cam_io_w_mb(csiphy_onthego_regs[i+1],
			csiphybase + csiphy_onthego_regs[i]);

		if (csiphy_onthego_regs[i+2])
			usleep_range(csiphy_onthego_regs[i+2], csiphy_onthego_regs[i+2] + 5);

		CAM_INFO(CAM_CSIPHY, "Offset: 0x%x, Val: 0x%x Delay(us): %u",
			csiphy_onthego_regs[i],
			cam_io_r_mb(csiphybase + csiphy_onthego_regs[i]),
			csiphy_onthego_regs[i+2]);
	}
}

static inline int cam_csiphy_release_from_reset_state(struct csiphy_device *csiphy_dev,
	void __iomem *csiphybase, int32_t instance)
{
	int                                                  i;
	struct csiphy_reg_parms_t                           *csiphy_reg;
	struct csiphy_reg_t                                 *csiphy_reset_release_reg;
	bool                                                 config_found = false;

	if (!csiphy_dev || !csiphybase) {
		CAM_ERR(CAM_CSIPHY, "Invalid input params: csiphy_dev: %s, csiphybase: %s",
			CAM_IS_NULL_TO_STR(csiphy_dev),
			CAM_IS_NULL_TO_STR(csiphybase));
		return -EINVAL;
	}

	CAM_DBG(CAM_CSIPHY, "Csiphy idx: %d", csiphy_dev->soc_info.index);

	csiphy_reg = csiphy_dev->ctrl_reg->csiphy_reg;
	for (i = 0; i < csiphy_reg->csiphy_reset_exit_array_size; i++) {
		csiphy_reset_release_reg = &csiphy_dev->ctrl_reg->csiphy_reset_exit_regs[i];

		switch (csiphy_reset_release_reg->csiphy_param_type) {
		case CSIPHY_2PH_REGS:
			if (!g_phy_data[csiphy_dev->soc_info.index].is_3phase &&
				!csiphy_dev->combo_mode &&
				!csiphy_dev->cphy_dphy_combo_mode) {
				cam_io_w_mb(csiphy_reset_release_reg->reg_data,
					csiphybase + csiphy_reset_release_reg->reg_addr);
				config_found = true;
			}
			break;
		case CSIPHY_3PH_REGS:
			if (g_phy_data[csiphy_dev->soc_info.index].is_3phase &&
				!csiphy_dev->combo_mode  &&
				!csiphy_dev->cphy_dphy_combo_mode) {
				cam_io_w_mb(csiphy_reset_release_reg->reg_data,
					csiphybase + csiphy_reset_release_reg->reg_addr);
				config_found = true;
			}
			break;
		case CSIPHY_2PH_COMBO_REGS:
			if (!csiphy_dev->csiphy_info[instance].csiphy_3phase &&
					csiphy_dev->combo_mode &&
					!csiphy_dev->cphy_dphy_combo_mode) {
				cam_io_w_mb(csiphy_reset_release_reg->reg_data,
					csiphybase + csiphy_reset_release_reg->reg_addr);
				config_found = true;
			}
			break;
		case CSIPHY_3PH_COMBO_REGS:
			if (csiphy_dev->csiphy_info[instance].csiphy_3phase &&
					csiphy_dev->combo_mode &&
					!csiphy_dev->cphy_dphy_combo_mode) {
				cam_io_w_mb(csiphy_reset_release_reg->reg_data,
					csiphybase + csiphy_reset_release_reg->reg_addr);
				config_found = true;
			}
			break;
		case CSIPHY_2PH_3PH_COMBO_REGS:
			if (!csiphy_dev->combo_mode && csiphy_dev->cphy_dphy_combo_mode) {
				cam_io_w_mb(csiphy_reset_release_reg->reg_data,
					csiphybase + csiphy_reset_release_reg->reg_addr);
				config_found = true;
			}
			break;
		default:
			CAM_ERR(CAM_CSIPHY, "Invalid combination");
			return -EINVAL;
			break;
		}

		if (config_found) {
			if (csiphy_reset_release_reg->delay) {
				usleep_range(csiphy_reset_release_reg->delay,
					csiphy_reset_release_reg->delay + 5);
			}

			break;
		}

	}
	return 0;
}

static int __csiphy_prgm_common_data(uint32_t phy_idx, struct csiphy_reg_t *csiphy_common_reg,
	uint32_t size, bool reset)
{
	int i = 0;
	void __iomem *csiphybase = NULL;
	uint8_t is_3phase;

	csiphybase = g_phy_data[phy_idx].base_address;
	is_3phase = g_phy_data[phy_idx].is_3phase;

	if (!csiphybase) {
		CAM_DBG(CAM_CSIPHY, "CSIPHY: %d is not available in platform", phy_idx);
		return 0;
	}

	CAM_DBG(CAM_CSIPHY, "common_data_array_size: %d, is_3phase: %d, reset: %s",
		size, is_3phase, (reset ? "true" : "false"));

	for (i = 0; i < size; i++) {
		switch (csiphy_common_reg[i].csiphy_param_type) {
		case CSIPHY_DEFAULT_PARAMS:
			cam_io_w_mb(reset ? 0x00 : csiphy_common_reg[i].reg_data,
				csiphybase + csiphy_common_reg[i].reg_addr);
			break;
		case CSIPHY_2PH_REGS:
			if (!is_3phase) {
				cam_io_w_mb(reset ? 0x00 : csiphy_common_reg[i].reg_data,
					csiphybase + csiphy_common_reg[i].reg_addr);
			}
			break;
		case CSIPHY_3PH_REGS:
			if (is_3phase) {
				cam_io_w_mb(reset ? 0x00 : csiphy_common_reg[i].reg_data,
					csiphybase + csiphy_common_reg[i].reg_addr);
			}
			break;
		default:
			break;
		}

		if (csiphy_common_reg[i].delay > 0)
			usleep_range(csiphy_common_reg[i].delay, csiphy_common_reg[i].delay + 5);
	}

	return 0;
}

void cam_csiphy_query_cap(struct csiphy_device *csiphy_dev,
	struct cam_csiphy_query_cap *csiphy_cap)
{
	struct cam_hw_soc_info *soc_info = &csiphy_dev->soc_info;

	csiphy_cap->slot_info = soc_info->index;
	csiphy_cap->version = csiphy_dev->hw_version;
	csiphy_cap->clk_lane = csiphy_dev->clk_lane;
	csiphy_cap->reserved = csiphy_dev->is_aggregator_rx;
}

int cam_csiphy_dump_status_reg(struct csiphy_device *csiphy_dev)
{
	struct cam_hw_soc_info *soc_info;
	void __iomem *phybase = NULL;
	void __iomem *lane0_offset = 0;
	void __iomem *lane1_offset = 0;
	void __iomem *lane2_offset = 0;
	void __iomem *lane3_offset = 0;
	void __iomem *clk_offset = 0;
	struct csiphy_reg_parms_t *csiphy_reg;
	struct cam_cphy_dphy_status_reg_params_t *status_regs;
	int i = 0;

	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "Null csiphy_dev");
		return -EINVAL;
	}

	soc_info = &csiphy_dev->soc_info;
	if (!soc_info) {
		CAM_ERR(CAM_CSIPHY, "Null soc_info");
		return -EINVAL;
	}

	if (!g_phy_data[soc_info->index].base_address) {
		CAM_ERR(CAM_CSIPHY, "Invalid cphy_idx: %d", soc_info->index);
		return -EINVAL;
	}

	csiphy_reg = csiphy_dev->ctrl_reg->csiphy_reg;
	status_regs = csiphy_reg->status_reg_params;
	phybase = g_phy_data[soc_info->index].base_address;

	if (!status_regs) {
		CAM_ERR(CAM_CSIPHY, "2ph/3ph status offset not set");
		return -EINVAL;
	}

	if (g_phy_data[soc_info->index].is_3phase) {
		CAM_INFO(CAM_CSIPHY, "Dumping 3ph status regs");
		lane0_offset = phybase + status_regs->csiphy_3ph_status0_offset;
		lane1_offset =
			lane0_offset + csiphy_reg->size_offset_betn_lanes;
		lane2_offset =
			lane1_offset + csiphy_reg->size_offset_betn_lanes;

		for (i = 0; i < status_regs->csiphy_3ph_status_size; i++) {
			CAM_INFO(CAM_CSIPHY,
				"PHY: %d, Status%u. Ln0: 0x%x, Ln1: 0x%x, Ln2: 0x%x",
				soc_info->index, i,
				cam_io_r(lane0_offset + (i * 4)),
				cam_io_r(lane1_offset + (i * 4)),
				cam_io_r(lane2_offset + (i * 4)));
		}
	} else {
		CAM_INFO(CAM_CSIPHY, "Dumping 2ph status regs");
		lane0_offset = phybase + status_regs->csiphy_2ph_status0_offset;
		lane1_offset =
			lane0_offset + csiphy_reg->size_offset_betn_lanes;
		lane2_offset =
			lane1_offset + csiphy_reg->size_offset_betn_lanes;
		lane3_offset =
			lane2_offset + csiphy_reg->size_offset_betn_lanes;
		clk_offset =
			lane3_offset + (csiphy_reg->size_offset_betn_lanes / 2);

		for (i = 0; i < status_regs->csiphy_2ph_status_size; i++) {
			CAM_INFO(CAM_CSIPHY,
				"PHY: %d, Status%u. "
				"CLK_LN: 0x%x Ln0: 0x%x, Ln1: 0x%x, Ln2: 0x%x, Ln3: 0x%x",
				soc_info->index, i,
				cam_io_r(clk_offset + (i * 4)),
				cam_io_r(lane0_offset + (i * 4)),
				cam_io_r(lane1_offset + (i * 4)),
				cam_io_r(lane2_offset + (i * 4)),
				cam_io_r(lane3_offset + (i * 4)));
		}
	}
	return 0;
}

void cam_csiphy_reset(struct csiphy_device *csiphy_dev)
{
	int32_t  i;
	void __iomem *base = NULL;
	uint32_t size =
		csiphy_dev->ctrl_reg->csiphy_reg->csiphy_reset_enter_array_size;
	struct cam_hw_soc_info *soc_info = &csiphy_dev->soc_info;

	base = soc_info->reg_map[0].mem_base;

	for (i = 0; i < size; i++) {
		cam_io_w_mb(
			csiphy_dev->ctrl_reg->csiphy_reset_enter_regs[i].reg_data,
			base +
			csiphy_dev->ctrl_reg->csiphy_reset_enter_regs[i].reg_addr);
		if (csiphy_dev->ctrl_reg->csiphy_reset_enter_regs[i].delay > 0)
			usleep_range(
			csiphy_dev->ctrl_reg->csiphy_reset_enter_regs[i].delay,
			csiphy_dev->ctrl_reg->csiphy_reset_enter_regs[i].delay
			+ 5);
	}

	if (csiphy_dev->en_lane_status_reg_dump) {
		CAM_INFO(CAM_CSIPHY, "Status Reg Dump after phy reset");
		cam_csiphy_dump_status_reg(csiphy_dev);
	}
}

static void cam_csiphy_program_common_registers(
	struct csiphy_device *csiphy_dev,
	bool reset,
	enum cam_csiphy_common_reg_program selection)
{
	int csiphy_idx;
	uint32_t size;
	struct csiphy_reg_t *csiphy_common_reg;

	csiphy_common_reg = csiphy_dev->ctrl_reg->csiphy_common_reg;
	size = csiphy_dev->ctrl_reg->csiphy_reg->csiphy_common_reg_array_size;

	if (selection == CAM_CSIPHY_PRGM_ALL) {
		if (active_csiphy_hw_cnt < 0 || active_csiphy_hw_cnt >= MAX_CSIPHY) {
			CAM_WARN(CAM_CSIPHY,
				"MisMatched in active phy hw: %d and Max supported: %d",
				active_csiphy_hw_cnt, MAX_CSIPHY);
			return;
		}

		if (active_csiphy_hw_cnt) {
			CAM_DBG(CAM_CSIPHY,
				"Common Settings already applied for all PHYs. active_phy_cnt: %d",
				active_csiphy_hw_cnt);
			return;
		}

		CAM_DBG(CAM_CSIPHY, "Is reset required: %d", reset);

		for (csiphy_idx = 0; csiphy_idx < MAX_CSIPHY; csiphy_idx++)
			__csiphy_prgm_common_data(csiphy_idx, csiphy_common_reg, size, reset);
	} else {
		__csiphy_prgm_common_data(csiphy_dev->soc_info.index,
			csiphy_common_reg, size, reset);
	}
}

static int cam_csiphy_update_secure_info(struct csiphy_device *csiphy_dev, int32_t index)
{
	uint64_t lane_assign_bitmask = 0;
	uint16_t lane_assign;
	uint32_t bit_offset_bet_phys_in_cp_ctrl;
	uint8_t lane_cnt;
	uint32_t cpas_version;
	int rc;

	if (csiphy_dev->domain_id_security) {
		CAM_DBG(CAM_CSIPHY, "Target supports domain ID security, skipping legacy update");
		return 0;
	}

	lane_assign = csiphy_dev->csiphy_info[index].lane_assign;
	lane_cnt = csiphy_dev->csiphy_info[index].lane_cnt;

	while (lane_cnt--) {
		lane_assign_bitmask |= (1 << (lane_assign & 0xF));
		lane_assign >>= 4;
	}

	rc = cam_cpas_get_cpas_hw_version(&cpas_version);
	if (rc) {
		CAM_ERR(CAM_CPAS, "Failed while getting CPAS Version");
		return rc;
	}

	switch (cpas_version) {
	case CAM_CPAS_TITAN_640_V200:
	case CAM_CPAS_TITAN_665_V100:
	case CAM_CPAS_TITAN_770_V100:
		bit_offset_bet_phys_in_cp_ctrl =
			CAM_CSIPHY_MAX_DPHY_LANES + CAM_CSIPHY_MAX_CPHY_LANES + 1;
		break;
	case CAM_CPAS_TITAN_580_V100:
	case CAM_CPAS_TITAN_680_V100:
	case CAM_CPAS_TITAN_780_V100:
		bit_offset_bet_phys_in_cp_ctrl =
			(csiphy_dev->soc_info.index < CAM_MAX_PHYS_PER_CP_CTRL_REG) ?
				(CAM_CSIPHY_MAX_DPHY_LANES + CAM_CSIPHY_MAX_CPHY_LANES) :
				(CAM_CSIPHY_MAX_DPHY_LANES + CAM_CSIPHY_MAX_CPHY_LANES + 1);
		break;
	default:
		bit_offset_bet_phys_in_cp_ctrl =
			CAM_CSIPHY_MAX_DPHY_LANES + CAM_CSIPHY_MAX_CPHY_LANES;
		break;
	}

	if (csiphy_dev->soc_info.index < CAM_MAX_PHYS_PER_CP_CTRL_REG)
		csiphy_dev->csiphy_info[index].csiphy_cpas_cp_reg_mask =
			lane_assign_bitmask <<
				((csiphy_dev->soc_info.index * bit_offset_bet_phys_in_cp_ctrl) +
				(!csiphy_dev->csiphy_info[index].csiphy_3phase) *
				(CAM_CSIPHY_MAX_CPHY_LANES));
	else
		csiphy_dev->csiphy_info[index].csiphy_cpas_cp_reg_mask =
			lane_assign_bitmask <<
				((csiphy_dev->soc_info.index - CAM_MAX_PHYS_PER_CP_CTRL_REG) *
					bit_offset_bet_phys_in_cp_ctrl +
				SEC_LANE_CP_REG_LEN +
				(!csiphy_dev->csiphy_info[index].csiphy_3phase) *
					CAM_CSIPHY_MAX_CPHY_LANES);

	CAM_DBG(CAM_CSIPHY, "CSIPHY_idx:%d, cp_reg_mask:0x%llx",
		csiphy_dev->soc_info.index,
		csiphy_dev->csiphy_info[index].csiphy_cpas_cp_reg_mask);

	return 0;
}

static int cam_csiphy_get_lane_enable(
	struct csiphy_device *csiphy, int index,
	uint16_t lane_assign, uint32_t *lane_enable)
{
	uint32_t lane_select = 0;

	if (csiphy->csiphy_info[index].csiphy_3phase) {
		CAM_DBG(CAM_CSIPHY, "LaneEnable for CPHY");
		switch (lane_assign & 0xF) {
		case 0x0:
			lane_select |= CPHY_LANE_0;
			break;
		case 0x1:
			lane_select |= CPHY_LANE_1;
			break;
		case 0x2:
			lane_select |= CPHY_LANE_2;
			break;
		default:
			CAM_ERR(CAM_CSIPHY,
				"Wrong lane configuration for CPHY : %d",
				lane_assign);
			*lane_enable = 0;
			return -EINVAL;
		}
	} else {
		CAM_DBG(CAM_CSIPHY, "LaneEnable for DPHY");
		switch (lane_assign & 0xF) {
		case 0x0:
			lane_select |= DPHY_LANE_0;
			lane_select |= DPHY_CLK_LN;
			break;
		case 0x1:
			lane_select |= DPHY_LANE_1;
			lane_select |= DPHY_CLK_LN;
			break;
		case 0x2:
			lane_select |= DPHY_LANE_2;
			if (csiphy->combo_mode)
				lane_select |= DPHY_LANE_3;
			else
				lane_select |= DPHY_CLK_LN;
			break;
		case 0x3:
			if (csiphy->combo_mode) {
				CAM_ERR(CAM_CSIPHY,
					"Wrong lane configuration for DPHYCombo: %d",
					lane_assign);
				*lane_enable = 0;
				return -EINVAL;
			}
			lane_select |= DPHY_LANE_3;
			lane_select |= DPHY_CLK_LN;
			break;
		default:
			CAM_ERR(CAM_CSIPHY,
				"Wrong lane configuration for DPHY: %d",
				lane_assign);
			*lane_enable = 0;
			return -EINVAL;
		}
	}

	*lane_enable = lane_select;
	CAM_DBG(CAM_CSIPHY, "Lane_select: 0x%x", lane_select);

	return 0;
}

static int cam_csiphy_sanitize_lane_cnt(
	struct csiphy_device *csiphy_dev,
	int32_t index, uint8_t lane_cnt)
{
	uint8_t max_supported_lanes = 0;

	if (csiphy_dev->combo_mode) {
		if (csiphy_dev->csiphy_info[index].csiphy_3phase)
			max_supported_lanes = 1;
		else
			max_supported_lanes = 2;
	} else if (csiphy_dev->cphy_dphy_combo_mode) {
		/* 2DPHY + 1CPHY or 2CPHY + 1DPHY */
		if (csiphy_dev->csiphy_info[index].csiphy_3phase)
			max_supported_lanes = 2;
		else
			max_supported_lanes = 2;
	} else {
		/* Mission Mode */
		if (csiphy_dev->csiphy_info[index].csiphy_3phase)
			max_supported_lanes = 3;
		else
			max_supported_lanes = 4;
	}

	if (lane_cnt <= 0 || lane_cnt > max_supported_lanes) {
		CAM_ERR(CAM_CSIPHY,
			"wrong lane_cnt configuration: expected max lane_cnt: %u received lane_cnt: %u",
			max_supported_lanes, lane_cnt);
		return -EINVAL;
	}

	return 0;
}

static int __cam_csiphy_parse_lane_info_cmd_buf(
	int32_t dev_handle,
	struct csiphy_device *csiphy_dev,
	struct cam_cmd_buf_desc *cmd_desc)
{
	int index, rc = 0;
	uint8_t lane_cnt = 0;
	uint32_t lane_enable = 0;
	uint16_t lane_assign = 0, preamble_en = 0;
	uintptr_t generic_ptr;
	uint32_t *cmd_buf = NULL;
	size_t len;
	rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle,
		&generic_ptr, &len);
	if (rc < 0) {
		CAM_ERR(CAM_CSIPHY,
			"Failed to get cmd buf mem address : %d", rc);
		return rc;
	}

	cmd_buf = (uint32_t *)generic_ptr;
	cmd_buf += cmd_desc->offset / 4;
	index = cam_csiphy_get_instance_offset(csiphy_dev, dev_handle);
	if (index < 0 || index >= csiphy_dev->session_max_device_support) {
		CAM_ERR(CAM_CSIPHY, "index in invalid: %d", index);
		cam_mem_put_cpu_buf(cmd_desc->mem_handle);
		return -EINVAL;
	}

	if (cmd_desc->meta_data == CAM_CSIPHY_PACKET_META_LANE_INFO_V2) {
		struct cam_csiphy_info_v2 *cam_cmd_csiphy_info_v2 = NULL;

		if ((len < sizeof(struct cam_csiphy_info_v2)) ||
			(cmd_desc->offset > (len - sizeof(struct cam_csiphy_info_v2)))) {
			CAM_ERR(CAM_CSIPHY,
				"Not enough buffer provided for cam_csiphy_info");
			rc = -EINVAL;
			cam_mem_put_cpu_buf(cmd_desc->mem_handle);
			return rc;
		}

		cam_cmd_csiphy_info_v2 = (struct cam_csiphy_info_v2 *)cmd_buf;
		rc = cam_csiphy_sanitize_lane_cnt(csiphy_dev, index,
			cam_cmd_csiphy_info_v2->lane_cnt);
		if (rc) {
			CAM_ERR(CAM_CSIPHY, "Wrong configuration lane_cnt: %u",
				cam_cmd_csiphy_info_v2->lane_cnt);
			cam_mem_put_cpu_buf(cmd_desc->mem_handle);
			return rc;
		}

		preamble_en = (cam_cmd_csiphy_info_v2->mipi_flags &
			PREAMBLE_PATTEN_CAL_MASK);

		csiphy_dev->csiphy_info[index].lane_cnt =
			cam_cmd_csiphy_info_v2->lane_cnt;
		csiphy_dev->csiphy_info[index].lane_assign =
			cam_cmd_csiphy_info_v2->lane_assign;
		csiphy_dev->csiphy_info[index].settle_time =
			cam_cmd_csiphy_info_v2->settle_time;
		csiphy_dev->csiphy_info[index].data_rate =
			cam_cmd_csiphy_info_v2->data_rate;
		csiphy_dev->csiphy_info[index].secure_mode =
			cam_cmd_csiphy_info_v2->secure_mode;
		csiphy_dev->csiphy_info[index].mipi_flags =
			(cam_cmd_csiphy_info_v2->mipi_flags & SKEW_CAL_MASK);
		csiphy_dev->csiphy_info[index].channel_type =
			cam_cmd_csiphy_info_v2->channel_type;
	} else if (cmd_desc->meta_data == CAM_CSIPHY_PACKET_META_LANE_INFO) {
		struct cam_csiphy_info *cam_cmd_csiphy_info = NULL;

		if ((len < sizeof(struct cam_csiphy_info)) ||
			(cmd_desc->offset > (len - sizeof(struct cam_csiphy_info)))) {
			CAM_ERR(CAM_CSIPHY,
				"Not enough buffer provided for cam_csiphy_info");
			rc = -EINVAL;
			cam_mem_put_cpu_buf(cmd_desc->mem_handle);
			return rc;
		}

		cam_cmd_csiphy_info = (struct cam_csiphy_info *)cmd_buf;
		rc = cam_csiphy_sanitize_lane_cnt(csiphy_dev, index,
			cam_cmd_csiphy_info->lane_cnt);
		if (rc) {
			CAM_ERR(CAM_CSIPHY, "Wrong configuration lane_cnt: %u",
				cam_cmd_csiphy_info->lane_cnt);
			cam_mem_put_cpu_buf(cmd_desc->mem_handle);
			return rc;
		}

		preamble_en = (cam_cmd_csiphy_info->mipi_flags &
			PREAMBLE_PATTEN_CAL_MASK);

		csiphy_dev->csiphy_info[index].lane_cnt =
			cam_cmd_csiphy_info->lane_cnt;
		csiphy_dev->csiphy_info[index].lane_assign =
			cam_cmd_csiphy_info->lane_assign;
		csiphy_dev->csiphy_info[index].settle_time =
			cam_cmd_csiphy_info->settle_time;
		csiphy_dev->csiphy_info[index].data_rate =
			cam_cmd_csiphy_info->data_rate;
		csiphy_dev->csiphy_info[index].secure_mode =
			cam_cmd_csiphy_info->secure_mode;
		csiphy_dev->csiphy_info[index].mipi_flags =
			(cam_cmd_csiphy_info->mipi_flags & SKEW_CAL_MASK);
		csiphy_dev->csiphy_info[index].channel_type =
			CAM_CSIPHY_DATARATE_SHORT_CHANNEL;
	}

	/* Cannot support CPHY combo mode with One sensor setting
	 * preamble enable and second/third sensor is without
	 * preamble enable.
	 */
	if (csiphy_dev->preamble_enable && !preamble_en &&
		csiphy_dev->combo_mode &&
		!csiphy_dev->cphy_dphy_combo_mode) {
		CAM_ERR(CAM_CSIPHY,
			"Cannot support %s combo mode with differnt preamble settings",
			(csiphy_dev->csiphy_info[index].csiphy_3phase ?
			"CPHY" : "DPHY"));
		cam_mem_put_cpu_buf(cmd_desc->mem_handle);
		return -EINVAL;
	}

	csiphy_dev->preamble_enable = preamble_en;
	lane_assign = csiphy_dev->csiphy_info[index].lane_assign;
	lane_cnt = csiphy_dev->csiphy_info[index].lane_cnt;

	while (lane_cnt--) {
		rc = cam_csiphy_get_lane_enable(csiphy_dev, index,
			(lane_assign & 0xF), &lane_enable);
		if (rc) {
			CAM_ERR(CAM_CSIPHY, "Wrong lane configuration: %d",
				csiphy_dev->csiphy_info[index].lane_assign);
			if ((csiphy_dev->combo_mode) ||
				(csiphy_dev->cphy_dphy_combo_mode)) {
				CAM_DBG(CAM_CSIPHY,
					"Resetting error to zero for other devices to configure");
				rc = 0;
			}
			lane_enable = 0;
			csiphy_dev->csiphy_info[index].lane_enable = lane_enable;
			goto reset_settings;
		}
		csiphy_dev->csiphy_info[index].lane_enable |= lane_enable;
		lane_assign >>= 4;
	}

	if (csiphy_dev->csiphy_info[index].secure_mode == 1) {
		rc = cam_csiphy_update_secure_info(csiphy_dev, index);
		if (rc) {
			CAM_ERR(CAM_CSIPHY,
				"Secure info configuration failed for index: %d", index);
			goto reset_settings;
		}
	}

	CAM_DBG(CAM_CSIPHY,
		"phy version:%d, phy_idx: %d, preamble_en: %u",
		csiphy_dev->hw_version,
		csiphy_dev->soc_info.index,
		csiphy_dev->preamble_enable);
	CAM_DBG(CAM_CSIPHY,
		"3phase:%d, combo mode:%d, secure mode:%d",
		csiphy_dev->csiphy_info[index].csiphy_3phase,
		csiphy_dev->combo_mode,
		csiphy_dev->csiphy_info[index].secure_mode);
	CAM_DBG(CAM_CSIPHY,
		"lane_cnt: 0x%x, lane_assign: 0x%x, lane_enable: 0x%x, settle time:%llu, datarate:%llu",
		csiphy_dev->csiphy_info[index].lane_cnt,
		csiphy_dev->csiphy_info[index].lane_assign,
		csiphy_dev->csiphy_info[index].lane_enable,
		csiphy_dev->csiphy_info[index].settle_time,
		csiphy_dev->csiphy_info[index].data_rate);

	cam_mem_put_cpu_buf(cmd_desc->mem_handle);
	return rc;

reset_settings:
	cam_csiphy_reset_phyconfig_param(csiphy_dev, index);
	cam_mem_put_cpu_buf(cmd_desc->mem_handle);
	return rc;
}

static int __cam_csiphy_handle_cdr_sweep_info(
	struct csiphy_device               *csiphy_dev,
	struct cam_csiphy_cdr_sweep_params *cdr_sweep_params)
{
	if (cdr_sweep_params->tolerance_op_type != CAM_CSIPHY_CDR_ADD_TOLERANCE &&
		cdr_sweep_params->tolerance_op_type != CAM_CSIPHY_CDR_SUB_TOLERANCE) {
		CAM_ERR(CAM_CSIPHY, "Invalid tolerance op type: %u",
			cdr_sweep_params->tolerance_op_type);
		return -EINVAL;
	}

	csiphy_dev->cdr_params.cdr_tolerance = cdr_sweep_params->cdr_tolerance;
	csiphy_dev->cdr_params.tolerance_op_type = cdr_sweep_params->tolerance_op_type;
	csiphy_dev->cdr_params.cdr_config_ptr =
		(uint32_t *)&cdr_sweep_params->configured_cdr;
	csiphy_dev->cdr_params.cdr_sweep_enabled = true;

	CAM_DBG(CAM_CSIPHY,
		"CSIPHY:%u cdr sweep with tolerance: %u op_type: %u cpu_addr: %pK enabled",
		csiphy_dev->soc_info.index, csiphy_dev->cdr_params.cdr_tolerance,
		csiphy_dev->cdr_params.tolerance_op_type,
		csiphy_dev->cdr_params.cdr_config_ptr);

	return 0;
}

static int __cam_csiphy_handle_aux_mem_buffer(
	struct csiphy_device                  *csiphy_dev,
	struct cam_csiphy_aux_settings_params *aux_setting_params)
{
	if (aux_setting_params->data_rate_aux_mask)
		g_phy_data[csiphy_dev->soc_info.index].data_rate_aux_mask |=
			aux_setting_params->data_rate_aux_mask;

	csiphy_dev->aux_params.aux_mem_update_en = true;
	csiphy_dev->aux_params.aux_config_ptr =
		(uint32_t *)&aux_setting_params->data_rate_aux_mask;

	CAM_DBG(CAM_CSIPHY,
		"CSIPHY:%u aux setting buffer provided addr: %pK provided_mask: 0x%llx current_mask :0x%llx",
		csiphy_dev->soc_info.index, csiphy_dev->aux_params.aux_config_ptr,
		aux_setting_params->data_rate_aux_mask,
		g_phy_data[csiphy_dev->soc_info.index].data_rate_aux_mask);

	return 0;
}

static int32_t __cam_csiphy_generic_blob_handler(void *user_data,
	uint32_t blob_type, uint32_t blob_size, uint8_t *blob_data)
{
	int rc = 0;
	struct csiphy_device *csiphy_dev =
		(struct csiphy_device *)user_data;

	if (!blob_data || (blob_size == 0)) {
		CAM_ERR(CAM_CSIPHY, "Invalid blob info %pK %u", blob_data,
			blob_size);
		return -EINVAL;
	}

	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "Invalid user data");
		return -EINVAL;
	}

	switch (blob_type) {
	case CAM_CSIPHY_GENERIC_BLOB_TYPE_CDR_CONFIG: {
		struct cam_csiphy_cdr_sweep_params *cdr_sweep_params =
			(struct cam_csiphy_cdr_sweep_params *)blob_data;

		if (blob_size < sizeof(struct cam_csiphy_cdr_sweep_params)) {
			CAM_ERR(CAM_CSIPHY, "Invalid blob size expected: 0x%x actual: 0x%x",
				sizeof(struct cam_csiphy_cdr_sweep_params), blob_size);
			return -EINVAL;
		}

		rc = __cam_csiphy_handle_cdr_sweep_info(csiphy_dev, cdr_sweep_params);
		break;
	}
	case CAM_CSIPHY_GENERIC_BLOB_TYPE_AUX_CONFIG: {
		struct cam_csiphy_aux_settings_params *aux_setting_params =
			(struct cam_csiphy_aux_settings_params *)blob_data;

		if (blob_size < sizeof(struct cam_csiphy_aux_settings_params)) {
			CAM_ERR(CAM_CSIPHY, "Invalid blob size expected: 0x%x actual: 0x%x",
				sizeof(struct cam_csiphy_aux_settings_params), blob_size);
			return -EINVAL;
		}

		rc = __cam_csiphy_handle_aux_mem_buffer(csiphy_dev, aux_setting_params);
		break;
	}
	default:
		CAM_WARN(CAM_CSIPHY, "Invalid blob type %d", blob_type);
		break;
	}

	return rc;
}

int32_t cam_cmd_buf_parser(struct csiphy_device *csiphy_dev,
	struct cam_config_dev_cmd *cfg_dev)
{
	int                      rc = 0, i;
	uintptr_t                generic_pkt_ptr;
	struct cam_packet       *csl_packet = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	size_t                   len, remain_len;
	uint32_t                 cmd_buf_type;


	if (!cfg_dev || !csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "Invalid Args");
		return -EINVAL;
	}

	rc = cam_mem_get_cpu_buf((int32_t) cfg_dev->packet_handle,
		&generic_pkt_ptr, &len);
	if (rc < 0) {
		CAM_ERR(CAM_CSIPHY, "Failed to get packet Mem address: %d", rc);
		return rc;
	}

	remain_len = len;
	if ((sizeof(struct cam_packet) > len) ||
		((size_t)cfg_dev->offset >= len - sizeof(struct cam_packet))) {
		CAM_ERR(CAM_CSIPHY,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len);
		cam_mem_put_cpu_buf(cfg_dev->packet_handle);
		rc = -EINVAL;
		return rc;
	}

	remain_len -= (size_t)cfg_dev->offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_ptr + (uint32_t)cfg_dev->offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_CSIPHY, "Invalid packet params");
		cam_mem_put_cpu_buf(cfg_dev->packet_handle);
		rc = -EINVAL;
		return rc;
	}

	if (csl_packet->num_cmd_buf)
		cmd_desc = (struct cam_cmd_buf_desc *)
			((uint32_t *)&csl_packet->payload_flex +
			csl_packet->cmd_buf_offset / 4);
	else {
		CAM_ERR(CAM_CSIPHY, "num_cmd_buffer = %d", csl_packet->num_cmd_buf);
		cam_mem_put_cpu_buf(cfg_dev->packet_handle);
		rc = -EINVAL;
		return rc;
	}

	CAM_DBG(CAM_CSIPHY, "CSIPHY:%u num cmd buffers received: %u",
		csiphy_dev->soc_info.index, csl_packet->num_cmd_buf);

	for (i = 0; i < csl_packet->num_cmd_buf; i++) {
		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc) {
			cam_mem_put_cpu_buf(cfg_dev->packet_handle);
			return rc;
		}

		cmd_buf_type = cmd_desc[i].meta_data;

		CAM_DBG(CAM_CSIPHY, "CSIPHY:%u cmd_buffer_%d type: %u",
			csiphy_dev->soc_info.index, i, cmd_buf_type);

		switch (cmd_buf_type) {
		case CAM_CSIPHY_PACKET_META_LANE_INFO:
			rc = __cam_csiphy_parse_lane_info_cmd_buf(
				cfg_dev->dev_handle, csiphy_dev, &cmd_desc[i]);
			break;
		case CAM_CSIPHY_PACKET_META_LANE_INFO_V2:
			rc = __cam_csiphy_parse_lane_info_cmd_buf(
				cfg_dev->dev_handle, csiphy_dev, &cmd_desc[i]);
			break;
		case CAM_CSIPHY_PACKET_META_GENERIC_BLOB:
			rc =  cam_packet_util_process_generic_cmd_buffer(&cmd_desc[i],
				__cam_csiphy_generic_blob_handler, csiphy_dev);
			break;
		default:
			CAM_WARN(CAM_CSIPHY,
				"Invalid meta type: %u", cmd_buf_type);
			break;
		}

		if (rc)
			break;
	}
	cam_mem_put_cpu_buf(cfg_dev->packet_handle);
	return rc;
}

void cam_csiphy_cphy_irq_config(struct csiphy_device *csiphy_dev)
{
	int32_t                        i;
	struct csiphy_reg_t           *csiphy_irq_reg;
	uint32_t num_of_irq_status_regs = 0;

	void __iomem *csiphybase =
		csiphy_dev->soc_info.reg_map[0].mem_base;

	num_of_irq_status_regs =
		csiphy_dev->ctrl_reg->csiphy_reg->csiphy_interrupt_status_size;

	for (i = 0; i < num_of_irq_status_regs; i++) {
		csiphy_irq_reg = &csiphy_dev->ctrl_reg->csiphy_irq_reg[i];
		cam_io_w_mb(csiphy_irq_reg->reg_data,
			csiphybase + csiphy_irq_reg->reg_addr);

		if (csiphy_irq_reg->delay)
			usleep_range(csiphy_irq_reg->delay,
				csiphy_irq_reg->delay + 5);
	}
}

void cam_csiphy_cphy_irq_disable(struct csiphy_device *csiphy_dev)
{
	int32_t i;
	void __iomem *csiphybase =
		csiphy_dev->soc_info.reg_map[0].mem_base;
	uint32_t num_of_irq_status_regs = 0;

	num_of_irq_status_regs =
		csiphy_dev->ctrl_reg->csiphy_reg->csiphy_interrupt_status_size;

	for (i = 0; i < num_of_irq_status_regs; i++)
		cam_io_w_mb(0x0, csiphybase +
			csiphy_dev->ctrl_reg->csiphy_irq_reg[i].reg_addr);
}

irqreturn_t cam_csiphy_irq(int irq_num, void *data)
{
	struct csiphy_device *csiphy_dev =
		(struct csiphy_device *)data;
	struct csiphy_reg_parms_t *csiphy_reg = NULL;
	void __iomem *base = NULL;

	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "Invalid Args");
		return IRQ_NONE;
	}

	base = csiphy_dev->soc_info.reg_map[0].mem_base;
	csiphy_reg = csiphy_dev->ctrl_reg->csiphy_reg;

	if (csiphy_dev->en_common_status_reg_dump) {
		cam_csiphy_common_status_reg_dump(csiphy_dev);
		cam_io_w_mb(0x1, base + csiphy_reg->mipi_csiphy_glbl_irq_cmd_addr);
		cam_io_w_mb(0x0, base + csiphy_reg->mipi_csiphy_glbl_irq_cmd_addr);
	}

	return IRQ_HANDLED;
}

static inline void __cam_csiphy_compute_cdr_value(
	int32_t *cdr_val, struct csiphy_device *csiphy_device)
{
	if (csiphy_device->cdr_params.tolerance_op_type ==
		CAM_CSIPHY_CDR_ADD_TOLERANCE)
		*cdr_val += csiphy_device->cdr_params.cdr_tolerance;
	else
		*cdr_val -= csiphy_device->cdr_params.cdr_tolerance;
}

static int cam_csiphy_cphy_data_rate_config(struct csiphy_device *csiphy_device, int32_t idx,
	uint8_t datarate_variant_idx)
{
	int i;
	unsigned int data_rate_idx;
	uint64_t required_phy_data_rate;
	void __iomem *csiphybase;
	ssize_t num_data_rates;
	struct data_rate_settings_t *settings_table;
	uint64_t intermediate_var = 0;
	uint16_t settle_cnt = 0;
	uint32_t reg_addr, reg_data, reg_param_type;
	int32_t  delay;
	struct csiphy_reg_t *config_params;
	uint8_t csiphy_index = 0;
	uint32_t channel_type;

	if ((csiphy_device == NULL) || (csiphy_device->ctrl_reg == NULL)) {
		CAM_ERR(CAM_CSIPHY, "Device is NULL");
		return -EINVAL;
	}

	if (csiphy_device->ctrl_reg->data_rates_settings_table == NULL) {
		CAM_DBG(CAM_CSIPHY,
			"Data rate specific register table not available");
		return 0;
	}

	if (csiphy_device->soc_info.index >= MAX_CSIPHY) {
		CAM_ERR(CAM_CSIPHY, "soc_info.index:%d >= MAX_CSIPHY:%d ",
			csiphy_device->soc_info.index, MAX_CSIPHY);
		return -EINVAL;
	}

	required_phy_data_rate = csiphy_device->csiphy_info[idx].data_rate;
	csiphybase = csiphy_device->soc_info.reg_map[0].mem_base;
	settings_table = csiphy_device->ctrl_reg->data_rates_settings_table;
	num_data_rates = settings_table->num_data_rate_settings;

	intermediate_var = csiphy_device->csiphy_info[idx].settle_time;
	do_div(intermediate_var, 200000000);
	settle_cnt = intermediate_var;
	csiphy_index = csiphy_device->soc_info.index;
	channel_type = csiphy_device->csiphy_info[idx].channel_type;

	CAM_DBG(CAM_CSIPHY, "required data rate : %llu", required_phy_data_rate);
	for (data_rate_idx = 0; data_rate_idx < num_data_rates; data_rate_idx++) {
		struct data_rate_reg_info_t *drate_settings = settings_table->data_rate_settings;
		uint64_t supported_phy_bw = drate_settings[data_rate_idx].bandwidth;
		ssize_t  num_reg_entries = drate_settings[data_rate_idx].data_rate_reg_array_size;
		struct csiphy_reg_t **drate_reg_array =
			drate_settings[data_rate_idx].data_rate_reg_array[csiphy_index];

		if ((required_phy_data_rate > supported_phy_bw) &&
			(data_rate_idx < (num_data_rates - 1))) {
			CAM_DBG(CAM_CSIPHY,
				"Skipping table [%d] with BW: %llu, Required data_rate: %llu",
				data_rate_idx, supported_phy_bw, required_phy_data_rate);
			continue;
		}

		CAM_DBG(CAM_CSIPHY, "table[%d] BW : %llu Selected",
			data_rate_idx, supported_phy_bw);

		if (datarate_variant_idx >= CAM_CSIPHY_MAX_DATARATE_VARIANTS) {
			CAM_ERR(CAM_CSIPHY, "Datarate variant Idx: %u can not exceed %u",
				datarate_variant_idx, CAM_CSIPHY_MAX_DATARATE_VARIANTS-1);
			return -EINVAL;
		}

		config_params = drate_reg_array[datarate_variant_idx];
		if (!config_params) {
			CAM_ERR(CAM_CSIPHY, "Datarate settings are null. datarate variant idx: %u",
				datarate_variant_idx);
			return -EINVAL;
		}

		for (i = 0; i < num_reg_entries; i++) {
			reg_addr = config_params[i].reg_addr;
			reg_data = config_params[i].reg_data;
			reg_param_type = config_params[i].csiphy_param_type;
			delay = config_params[i].delay;
			CAM_DBG(CAM_CSIPHY,
				"param_type: %d writing reg : %x val : %x delay: %dus",
				reg_param_type, reg_addr, reg_data,
				delay);
			switch (reg_param_type) {
			case CSIPHY_DEFAULT_PARAMS:
				cam_io_w_mb(reg_data,
					csiphybase + reg_addr);
			break;
			case CSIPHY_SHORT_CHANNEL_PARAMS:
				if (channel_type == CAM_CSIPHY_DATARATE_SHORT_CHANNEL)
					cam_io_w_mb(reg_data,
						csiphybase + reg_addr);
			break;
			case CSIPHY_STANDARD_CHANNEL_PARAMS:
				if (channel_type == CAM_CSIPHY_DATARATE_STANDARD_CHANNEL)
					cam_io_w_mb(reg_data,
						csiphybase + reg_addr);
			break;
			case CSIPHY_SETTLE_CNT_LOWER_BYTE:
				cam_io_w_mb(settle_cnt & 0xFF,
					csiphybase + reg_addr);
			break;
			case CSIPHY_SETTLE_CNT_HIGHER_BYTE:
				cam_io_w_mb((settle_cnt >> 8) & 0xFF,
					csiphybase + reg_addr);
			break;
			case CSIPHY_AUXILIARY_SETTING: {
				uint32_t phy_idx = csiphy_device->soc_info.index;

				if (g_phy_data[phy_idx].data_rate_aux_mask &
					BIT_ULL(data_rate_idx)) {
					cam_io_w_mb(reg_data, csiphybase + reg_addr);
					CAM_DBG(CAM_CSIPHY,
						"CSIPHY: %u configuring new aux setting reg_addr: 0x%x reg_val: 0x%x",
						csiphy_device->soc_info.index, reg_addr, reg_data);
				}
			}
			break;
			default:
				CAM_DBG(CAM_CSIPHY, "Do Nothing");
			break;
			}

			if ((reg_param_type ==
				(CSIPHY_CDR_LN_SETTINGS | CSIPHY_SHORT_CHANNEL_PARAMS) &&
				(channel_type == CAM_CSIPHY_DATARATE_SHORT_CHANNEL)) ||
				(reg_param_type ==
				(CSIPHY_CDR_LN_SETTINGS | CSIPHY_STANDARD_CHANNEL_PARAMS) &&
				(channel_type == CAM_CSIPHY_DATARATE_STANDARD_CHANNEL)) ||
				reg_param_type == CSIPHY_CDR_LN_SETTINGS) {
				int32_t cdr_val = reg_data;
				struct cam_csiphy_dev_cdr_sweep_params *cdr_params =
					&csiphy_device->cdr_params;

				if (cdr_params->cdr_sweep_enabled) {
					__cam_csiphy_compute_cdr_value(&cdr_val, csiphy_device);

					if (cdr_val < 0) {
						CAM_ERR(CAM_CSIPHY,
							"CSIPHY: %u invalid CDR tolerance computation, default: 0x%x tolerance: 0x%x op_type: 0x%x",
							csiphy_device->soc_info.index,
							reg_data, cdr_params->cdr_tolerance,
							cdr_params->tolerance_op_type);
						return -EINVAL;
					}

					/* Update userland on configured values */
					*csiphy_device->cdr_params.cdr_config_ptr = cdr_val;
				}

				cam_io_w_mb(cdr_val, csiphybase + reg_addr);
				CAM_DBG(CAM_CSIPHY,
					"CSIPHY: %u CDR reg_addr: 0x%x reg_val: 0x%x sweep test: %s",
					csiphy_device->soc_info.index,
					reg_addr, cdr_val,
					CAM_BOOL_TO_YESNO(cdr_params->cdr_sweep_enabled));
			}

			if (delay > 0)
				usleep_range(delay, delay + 5);
		}

		csiphy_device->curr_data_rate_idx = data_rate_idx;
		break;
	}

	return 0;
}

static int __cam_csiphy_prgm_bist_reg(struct csiphy_device *csiphy_dev, bool is_3phase)
{
	int i;
	int bist_arr_size;
	struct csiphy_reg_t *csiphy_common_reg;
	void __iomem *csiphybase;

	csiphybase = csiphy_dev->soc_info.reg_map[0].mem_base;

	if (is_3phase) {
		csiphy_common_reg = csiphy_dev->ctrl_reg->csiphy_bist_reg->bist_3ph_settings_arry;
		bist_arr_size = csiphy_dev->ctrl_reg->csiphy_bist_reg->num_3ph_bist_settings;
	} else {
		csiphy_common_reg = csiphy_dev->ctrl_reg->csiphy_bist_reg->bist_2ph_settings_arry;
		bist_arr_size = csiphy_dev->ctrl_reg->csiphy_bist_reg->num_2ph_bist_settings;
	}

	if (!csiphy_common_reg) {
		CAM_DBG(CAM_CSIPHY, "No Bist Settings available for %s",
			(is_3phase) ? "CPHY" : "DPHY");
		return 0;
	}

	for (i = 0; i < bist_arr_size; i++) {
		cam_io_w_mb(csiphy_common_reg[i].reg_data,
			csiphybase + csiphy_common_reg[i].reg_addr);

		if (csiphy_common_reg[i].delay)
			usleep_range(csiphy_common_reg[i].delay, csiphy_common_reg[i].delay + 5);
	}

	return 0;
}

#ifdef CONFIG_SECURE_CAMERA
static int cam_csiphy_program_secure_mode(struct csiphy_device *csiphy_dev,
	bool protect, int32_t offset)
{
	int rc = 0;

	if (csiphy_dev->domain_id_security) {
		if (!csiphy_dev->csiphy_info[offset].secure_info_updated) {
			CAM_ERR(CAM_CSIPHY,
				"PHY[%u] domain id info not updated, aborting secure call",
				csiphy_dev->soc_info.index);

			return -EINVAL;
		}

		rc = cam_cpas_enable_clks_for_domain_id(true);
		if (rc) {
			CAM_ERR(CAM_CSIPHY, "Failed to enable the Domain ID clocks");
			return rc;
		}
	}

	rc = cam_csiphy_notify_secure_mode(csiphy_dev, protect, offset);

	if (csiphy_dev->domain_id_security) {
		if (cam_cpas_enable_clks_for_domain_id(false))
			CAM_ERR(CAM_CSIPHY, "Failed to disable the Domain ID clocks");

		if (!protect)
			csiphy_dev->csiphy_info[offset].secure_info_updated = false;
	}

	return rc;
}
#endif

int32_t cam_csiphy_config_dev(struct csiphy_device *csiphy_dev,
	int32_t dev_handle, uint8_t datarate_variant_idx)
{
	int32_t      rc = 0;
	uint32_t     lane_enable = 0;
	uint16_t     i = 0, cfg_size = 0;
	uint16_t     settle_cnt = 0;
	uint8_t      skew_cal_enable = 0;
	uint64_t     intermediate_var;
	int          index;
	void __iomem *csiphybase;
	struct csiphy_reg_t *reg_array;
	struct csiphy_reg_parms_t *csiphy_reg;
	bool         is_3phase = false;

	csiphybase = csiphy_dev->soc_info.reg_map[0].mem_base;

	CAM_DBG(CAM_CSIPHY, "ENTER");
	if (!csiphybase) {
		CAM_ERR(CAM_CSIPHY, "csiphybase NULL");
		return -EINVAL;
	}

	index = cam_csiphy_get_instance_offset(csiphy_dev, dev_handle);
	if (index < 0 || index >= csiphy_dev->session_max_device_support) {
		CAM_ERR(CAM_CSIPHY, "index is invalid: %d", index);
		return -EINVAL;
	}

	CAM_DBG(CAM_CSIPHY,
		"Index: %d: expected dev_hdl: 0x%x : derived dev_hdl: 0x%x",
		index, dev_handle,
		csiphy_dev->csiphy_info[index].hdl_data.device_hdl);

	if (csiphy_dev->csiphy_info[index].csiphy_3phase)
		is_3phase = true;

	csiphy_reg = csiphy_dev->ctrl_reg->csiphy_reg;
	if (csiphy_dev->combo_mode) {
		/* for CPHY(3Phase) or DPHY(2Phase) combo mode selection */
		if (is_3phase) {
			/* CPHY combo mode */
			if (csiphy_dev->ctrl_reg->csiphy_3ph_combo_reg) {
				reg_array = csiphy_dev->ctrl_reg->csiphy_3ph_combo_reg;
				cfg_size = csiphy_reg->csiphy_3ph_combo_config_array_size;
			} else {
				CAM_WARN(CAM_CSIPHY, "CPHY combo mode reg settings not found");
				reg_array = csiphy_dev->ctrl_reg->csiphy_3ph_reg;
				cfg_size = csiphy_reg->csiphy_3ph_config_array_size;
			}
		} else {
			/* DPHY combo mode*/
			if (csiphy_dev->ctrl_reg->csiphy_2ph_combo_mode_reg) {
				reg_array = csiphy_dev->ctrl_reg->csiphy_2ph_combo_mode_reg;
				cfg_size = csiphy_reg->csiphy_2ph_combo_config_array_size;
			} else {
				CAM_WARN(CAM_CSIPHY, "DPHY combo mode reg settings not found");
				reg_array = csiphy_dev->ctrl_reg->csiphy_2ph_reg;
				cfg_size = csiphy_reg->csiphy_2ph_config_array_size;
			}

		}
	} else if (csiphy_dev->cphy_dphy_combo_mode) {
		/* for CPHY and DPHY combo mode selection */
		if (csiphy_dev->ctrl_reg->csiphy_2ph_3ph_mode_reg) {
			reg_array = csiphy_dev->ctrl_reg->csiphy_2ph_3ph_mode_reg;
			cfg_size = csiphy_reg->csiphy_2ph_3ph_config_array_size;
		} else {
			reg_array = csiphy_dev->ctrl_reg->csiphy_3ph_reg;
			cfg_size = csiphy_reg->csiphy_3ph_config_array_size;
			CAM_WARN(CAM_CSIPHY,
					"Unsupported configuration, Falling back to CPHY mission mode");
		}
	} else {
		/* for CPHY(3Phase) or DPHY(2Phase) Non combe mode selection */
		if (is_3phase) {
			CAM_DBG(CAM_CSIPHY,
				"3phase Non combo mode reg array selected");
			reg_array = csiphy_dev->ctrl_reg->csiphy_3ph_reg;
			cfg_size = csiphy_reg->csiphy_3ph_config_array_size;
		} else {
			CAM_DBG(CAM_CSIPHY,
				"2PHASE Non combo mode reg array selected");
			reg_array = csiphy_dev->ctrl_reg->csiphy_2ph_reg;
			cfg_size = csiphy_reg->csiphy_2ph_config_array_size;
		}
	}

	lane_enable = csiphy_dev->csiphy_info[index].lane_enable;

	if (csiphy_dev->csiphy_info[index].csiphy_3phase) {
		rc = cam_csiphy_cphy_data_rate_config(csiphy_dev, index, datarate_variant_idx);
		if (rc) {
			CAM_ERR(CAM_CSIPHY,
				"Date rate specific configuration failed rc: %d",
				rc);
			return rc;
		}
	}

	csiphy_dev->lanes_enabled = lane_enable;
	intermediate_var = csiphy_dev->csiphy_info[index].settle_time;
	do_div(intermediate_var, 200000000);
	settle_cnt = intermediate_var;
	skew_cal_enable = csiphy_dev->csiphy_info[index].mipi_flags;

	for (i = 0; i < cfg_size; i++) {
		switch (reg_array[i].csiphy_param_type) {
		case CSIPHY_LANE_ENABLE:
			cam_io_w_mb(lane_enable, csiphybase + reg_array[i].reg_addr);
		break;
		case CSIPHY_DEFAULT_PARAMS:
			cam_io_w_mb(reg_array[i].reg_data, csiphybase + reg_array[i].reg_addr);
		break;
		case CSIPHY_SETTLE_CNT_LOWER_BYTE:
			cam_io_w_mb(settle_cnt & 0xFF, csiphybase + reg_array[i].reg_addr);
		break;
		case CSIPHY_SETTLE_CNT_HIGHER_BYTE:
			cam_io_w_mb((settle_cnt >> 8) & 0xFF, csiphybase + reg_array[i].reg_addr);
		break;
		case CSIPHY_SKEW_CAL:
		if (skew_cal_enable)
			cam_io_w_mb(reg_array[i].reg_data, csiphybase + reg_array[i].reg_addr);
		else
			cam_io_w_mb(0x00, csiphybase + reg_array[i].reg_addr);
		break;
		default:
			CAM_DBG(CAM_CSIPHY, "Do Nothing");
		break;
		}

		if (reg_array[i].delay > 0)
			usleep_range(reg_array[i].delay, reg_array[i].delay + 5);
	}

	if (csiphy_dev->preamble_enable)
		__cam_csiphy_prgm_bist_reg(csiphy_dev, is_3phase);

	cam_csiphy_cphy_irq_config(csiphy_dev);

	CAM_DBG(CAM_CSIPHY, "EXIT");
	return rc;
}

void cam_csiphy_shutdown(struct csiphy_device *csiphy_dev)
{
	struct cam_hw_soc_info *soc_info;
	struct cam_csiphy_param *param;
	int32_t i = 0;
	int rc = 0;

	if (csiphy_dev->csiphy_state == CAM_CSIPHY_INIT)
		return;

	if (!csiphy_dev->acquire_count && !csiphy_dev->start_dev_count)
		return;

	if (csiphy_dev->acquire_count >= CSIPHY_MAX_INSTANCES_PER_PHY) {
		CAM_WARN(CAM_CSIPHY, "acquire count is invalid: %u",
			csiphy_dev->acquire_count);
		csiphy_dev->acquire_count =
			CSIPHY_MAX_INSTANCES_PER_PHY;
	}

	if ((csiphy_dev->csiphy_state == CAM_CSIPHY_START) ||
		csiphy_dev->start_dev_count) {
		soc_info = &csiphy_dev->soc_info;

		for (i = 0; i < csiphy_dev->acquire_count; i++) {
			param = &csiphy_dev->csiphy_info[i];

#ifdef CONFIG_SECURE_CAMERA
			if (param->secure_mode)
				cam_csiphy_program_secure_mode(csiphy_dev,
					CAM_SECURE_MODE_NON_SECURE, i);
#endif
			param->secure_mode = CAM_SECURE_MODE_NON_SECURE;

			if (soc_info->is_clk_drv_en && param->use_hw_client_voting) {
				rc = cam_soc_util_set_src_clk_rate(soc_info, param->conn_csid_idx,
					0, 0);
				if (rc) {
					CAM_ERR(CAM_CSIPHY,
						"[%d] Failed in setting clk rate for %d",
						soc_info->index, param->conn_csid_idx);
				} else {
					rc = cam_soc_util_cesta_channel_switch(param->conn_csid_idx,
						"csiphy_shutdown");
					if (rc) {
						CAM_ERR(CAM_CSIPHY,
							"Failed to apply power states for cesta client:%d rc:%d",
							param->conn_csid_idx, rc);
					}
				}
			}

			cam_csiphy_reset_phyconfig_param(csiphy_dev, i);
		}

		if ((csiphy_dev->prgm_cmn_reg_across_csiphy) &&
			(active_csiphy_hw_cnt > 0)) {
			mutex_lock(&active_csiphy_cnt_mutex);
			active_csiphy_hw_cnt--;
			mutex_unlock(&active_csiphy_cnt_mutex);

			cam_csiphy_program_common_registers(csiphy_dev, true,
				CAM_CSIPHY_PRGM_ALL);
		}

		cam_csiphy_reset(csiphy_dev);
		cam_soc_util_disable_platform_resource(soc_info, CAM_CLK_SW_CLIENT_IDX, true, true);
		if (g_phy_data[soc_info->index].aon_cam_id == NOT_AON_CAM)
			cam_cpas_stop(csiphy_dev->cpas_handle);

		csiphy_dev->csiphy_state = CAM_CSIPHY_ACQUIRE;
	}

	if (csiphy_dev->csiphy_state == CAM_CSIPHY_ACQUIRE) {
		for (i = 0; i < csiphy_dev->acquire_count; i++) {
			if (csiphy_dev->csiphy_info[i].hdl_data.device_hdl
				!= -1)
				cam_destroy_device_hdl(
				csiphy_dev->csiphy_info[i]
				.hdl_data.device_hdl);
			csiphy_dev->csiphy_info[i].hdl_data.device_hdl = -1;
			csiphy_dev->csiphy_info[i].hdl_data.session_hdl = -1;
		}
	}

	csiphy_dev->ref_count = 0;
	for (i = 0; i < csiphy_dev->session_max_device_support; i++) {
		csiphy_dev->lanes_assigned[i].lane_assign = -1;
		csiphy_dev->lanes_assigned[i].lane_assign_cnt = 0;
	}
	csiphy_dev->lanes_enabled = 0x0;
	csiphy_dev->acquire_count = 0;
	csiphy_dev->start_dev_count = 0;
	csiphy_dev->csiphy_state = CAM_CSIPHY_INIT;
}

static int32_t cam_csiphy_external_cmd(struct csiphy_device *csiphy_dev,
	struct cam_config_dev_cmd *p_submit_cmd)
{
	struct cam_csiphy_info cam_cmd_csiphy_info;
	int32_t rc = 0;
	int32_t  index = -1;

	if (copy_from_user(&cam_cmd_csiphy_info,
		u64_to_user_ptr(p_submit_cmd->packet_handle),
		sizeof(struct cam_csiphy_info))) {
		CAM_ERR(CAM_CSIPHY, "failed to copy cam_csiphy_info\n");
		rc = -EFAULT;
	} else {
		index = cam_csiphy_get_instance_offset(csiphy_dev,
			p_submit_cmd->dev_handle);
		if (index < 0 ||
			index >= csiphy_dev->session_max_device_support) {
			CAM_ERR(CAM_CSIPHY, "index is invalid: %d", index);
			return -EINVAL;
		}

		csiphy_dev->csiphy_info[index].lane_cnt =
			cam_cmd_csiphy_info.lane_cnt;
		csiphy_dev->csiphy_info[index].lane_assign =
			cam_cmd_csiphy_info.lane_assign;
		csiphy_dev->csiphy_info[index].settle_time =
			cam_cmd_csiphy_info.settle_time;
		csiphy_dev->csiphy_info[index].data_rate =
			cam_cmd_csiphy_info.data_rate;
		CAM_DBG(CAM_CSIPHY,
			"%s CONFIG_DEV_EXT settle_time= %lld lane_cnt=%d",
			__func__,
			csiphy_dev->csiphy_info[index].settle_time,
			csiphy_dev->csiphy_info[index].lane_cnt);
	}

	return rc;
}

static int cam_csiphy_update_lane_selection(struct csiphy_device *csiphy, int index, bool enable)
{
	uint32_t lane_enable;
	void __iomem *base_address;
	int32_t lane_reg_addr;
	int32_t delay;

	base_address = csiphy->soc_info.reg_map[0].mem_base;
	lane_reg_addr = csiphy->ctrl_reg->csiphy_lane_config_reg->reg_addr;
	delay = csiphy->ctrl_reg->csiphy_lane_config_reg->delay;

	lane_enable = cam_io_r(base_address + lane_reg_addr);

	if (enable)
		lane_enable |= csiphy->csiphy_info[index].lane_enable;
	else
		lane_enable &= ~csiphy->csiphy_info[index].lane_enable;

	csiphy->lanes_enabled = lane_enable;

	CAM_INFO(CAM_CSIPHY, "lane_reg_addr: 0x%x, lane_assign: 0x%x, lane_enable: 0x%x, delay: %d",
		lane_reg_addr, csiphy->csiphy_info[index].lane_assign, lane_enable, delay);

	cam_io_w_mb(lane_enable, base_address + lane_reg_addr);

	if (delay)
		usleep_range(delay, delay + 5);

	return 0;
}

static void cam_csiphy_update_lane_assign_info(
	struct csiphy_device *csiphy, int index, bool enable)
{
	int i = 0;

	CAM_DBG(CAM_CSIPHY,
		"acquire_cnt: %d phy_idx: %d offset: %d lane_assign: 0x%x, lanes_enabled: 0x%x max_dev_support: %d",
		csiphy->acquire_count,
		csiphy->soc_info.index,
		index,
		csiphy->csiphy_info[index].lane_assign,
		csiphy->lanes_enabled,
		csiphy->session_max_device_support);

	if (enable) {
		for (i = 0; i < csiphy->session_max_device_support; i++) {
			if (csiphy->lanes_assigned[i].lane_assign
				== csiphy->csiphy_info[index].lane_assign){
				csiphy->lanes_assigned[i].lane_assign_cnt++;
				break;
			}
		}
		if (i == csiphy->session_max_device_support) {
			for (i = 0; i < csiphy->session_max_device_support; i++) {
				if (csiphy->lanes_assigned[i].lane_assign_cnt == 0) {
					csiphy->lanes_assigned[i].lane_assign =
						csiphy->csiphy_info[index].lane_assign;
					csiphy->lanes_assigned[i].lane_assign_cnt++;
					break;
				}
			}
		}
	} else {
		for (i = 0; i < csiphy->session_max_device_support; i++) {
			if (csiphy->lanes_assigned[i].lane_assign
				== csiphy->csiphy_info[index].lane_assign) {
				csiphy->lanes_assigned[i].lane_assign_cnt--;
				if (csiphy->lanes_assigned[i].lane_assign_cnt == 0) {
					csiphy->lanes_assigned[i].lane_assign = -1;
					cam_csiphy_update_lane_selection(csiphy, index, false);
				}
				break;
			}
		}
	}


	if (i < csiphy->session_max_device_support && i >= 0) {
		CAM_DBG(CAM_CSIPHY,
			"lane_assign_cnt: 0%d, lane_assign: 0x%x, lanes_enabled: 0x%x",
			csiphy->lanes_assigned[i].lane_assign_cnt,
			csiphy->lanes_assigned[i].lane_assign,
			csiphy->lanes_enabled);
	} else {
		CAM_ERR(CAM_CSIPHY,
			"Array out of bounds: > i: %d", i);
	}
}

static int __csiphy_cpas_configure_for_main_or_aon(
	bool get_access, uint32_t phy_idx,
	struct cam_csiphy_aon_sel_params_t *aon_sel_params)
{
	int rc = 0;
	uint32_t aon_config = 0;
	uint32_t cpas_handle = g_phy_data[phy_idx].cpas_handle;

	if (g_phy_data[phy_idx].aon_cam_id == NOT_AON_CAM) {
		CAM_ERR(CAM_CSIPHY, "Not an AON Camera");
		return -EINVAL;
	}

	if (!aon_sel_params->aon_cam_sel_offset[g_phy_data[phy_idx].aon_cam_id]) {
		CAM_ERR(CAM_CSIPHY, "Mux register offset can not be 0. AON_Cam_ID: %u",
			g_phy_data[phy_idx].aon_cam_id);
		return -EINVAL;
	}

	if (get_access == g_phy_data[phy_idx].is_configured_for_main) {
		CAM_DBG(CAM_CSIPHY, "Already Configured/Released for %s",
			get_access ? "Main" : "AON");
		return 0;
	}

	if (get_access) {
		rc = cam_csiphy_cpas_ops(cpas_handle, true);
		if (rc) {
			CAM_ERR(CAM_CSIPHY, "voting CPAS: %d failed", rc);
			return rc;
		}
	}

	cam_cpas_reg_read(cpas_handle, CAM_CPAS_REGBASE_CPASTOP,
		aon_sel_params->aon_cam_sel_offset[g_phy_data[phy_idx].aon_cam_id],
		true, &aon_config);

	if (get_access && !g_phy_data[phy_idx].is_configured_for_main) {
		aon_config &= ~(aon_sel_params->cam_sel_mask |
			aon_sel_params->mclk_sel_mask);
		CAM_INFO(CAM_CSIPHY,
			"Selecting MainCamera over AON Camera");
		g_phy_data[phy_idx].is_configured_for_main = true;
	} else if (!get_access && g_phy_data[phy_idx].is_configured_for_main) {
		aon_config |= (aon_sel_params->cam_sel_mask |
			aon_sel_params->mclk_sel_mask);
		CAM_INFO(CAM_CSIPHY,
			"Releasing MainCamera to AON Camera");
		g_phy_data[phy_idx].is_configured_for_main = false;
	}

	CAM_DBG(CAM_CSIPHY, "value of aon_config = %u", aon_config);
	rc = cam_cpas_reg_write(cpas_handle, CAM_CPAS_REGBASE_CPASTOP,
		aon_sel_params->aon_cam_sel_offset[g_phy_data[phy_idx].aon_cam_id],
		true, aon_config);
	if (rc)
		CAM_ERR(CAM_CSIPHY, "CPAS AON sel register write failed");

	if (!get_access) {
		rc = cam_csiphy_cpas_ops(cpas_handle, false);
		if (rc) {
			CAM_ERR(CAM_CSIPHY, "voting CPAS: %d failed", rc);
			return rc;
		}
	}

	return rc;
}

int cam_csiphy_util_update_aon_registration(uint32_t phy_idx, uint32_t aon_cam_id)
{
	/* aon support enable for the sensor associated with phy idx*/
	if (phy_idx >= MAX_CSIPHY) {
		CAM_ERR(CAM_CSIPHY,
			"Invalid PHY index: %u", phy_idx);
		return -EINVAL;
	}

	if (!g_phy_data[phy_idx].base_address) {
		CAM_ERR(CAM_CSIPHY, "Invalid PHY idx: %d from Sensor user", phy_idx);
		return -EINVAL;
	}

	g_phy_data[phy_idx].aon_cam_id = aon_cam_id;

	return 0;
}

int cam_csiphy_util_update_aon_ops(
	bool get_access, uint32_t phy_idx)
{
	struct cam_csiphy_aon_sel_params_t *aon_sel_params;
	int rc = 0;

	if (phy_idx >= MAX_CSIPHY) {
		CAM_ERR(CAM_CSIPHY, "Null device");
		return -ENODEV;
	}

	if (!g_phy_data[phy_idx].base_address) {
		CAM_ERR(CAM_CSIPHY, "phy_idx: %d is not supported", phy_idx);
		return -EINVAL;
	}

	if (!g_phy_data[phy_idx].aon_sel_param) {
		CAM_ERR(CAM_CSIPHY, "AON select parameters are null");
		return -EINVAL;
	}

	aon_sel_params = g_phy_data[phy_idx].aon_sel_param;

	mutex_lock(&main_aon_selection);

	CAM_DBG(CAM_CSIPHY, "PHY idx: %d, AON_support %s", phy_idx,
		(get_access) ? "enable" : "disable");

	rc = __csiphy_cpas_configure_for_main_or_aon(
			get_access, phy_idx, aon_sel_params);
	if (rc)
		CAM_ERR(CAM_CSIPHY, "Configuration for AON ops failed: rc: %d", rc);

	mutex_unlock(&main_aon_selection);
	return rc;
}

static void __cam_csiphy_read_2phase_bist_counter_status(
	struct csiphy_device *csiphy_dev, uint32_t *counter)
{
	int i = 0, lane_count;
	int bist_status_arr_size =
		csiphy_dev->ctrl_reg->csiphy_bist_reg->number_of_counters;
	uint32_t base_offset = 0;
	void __iomem *phy_base = NULL;
	uint32_t val = 0;
	uint32_t offset_betwn_lane = 0;
	struct bist_reg_settings_t *bist_reg = NULL;

	phy_base = csiphy_dev->soc_info.reg_map[0].mem_base;
	bist_reg = csiphy_dev->ctrl_reg->csiphy_bist_reg;
	offset_betwn_lane =
		csiphy_dev->ctrl_reg->csiphy_reg->size_offset_betn_lanes;

	for (i = 0; i < bist_status_arr_size; i++) {
		base_offset = bist_reg->bist_counter_2ph_base_offset + (0x4 * i);
		val = 0;
		for (lane_count = 0; lane_count < CAM_CSIPHY_MAX_DPHY_LANES; lane_count++) {
			CAM_DBG(CAM_CSIPHY, "value to be read from addr: 0x%x is 0x%x",
				base_offset + (lane_count * offset_betwn_lane),
				(cam_io_r(phy_base + base_offset + (lane_count * offset_betwn_lane))));
			val |= ((cam_io_r(phy_base + base_offset + (lane_count * offset_betwn_lane))));
		}
		*counter |= (val << (i * 8));
		CAM_DBG(CAM_CSIPHY, "COUNTER VALUE is 0x%x", *counter);
	}

	return;
}

static void __cam_csiphy_get_2phase_pattern_status(
	struct csiphy_device *csiphy_dev)
{
	int i;
	int bist_status_arr_size = csiphy_dev->ctrl_reg->csiphy_bist_reg->num_status_reg;
	struct csiphy_reg_t *csiphy_common_reg = NULL;
	void __iomem *csiphybase;
	uint32_t status = 0;
	uint32_t counter = 0;
	struct bist_reg_settings_t *bist_reg;

	CAM_DBG(CAM_CSIPHY, "ENTER");
	csiphybase = csiphy_dev->soc_info.reg_map[0].mem_base;
	bist_reg = csiphy_dev->ctrl_reg->csiphy_bist_reg;

	/* This loop is to read every lane status value
	 * in case if loop breaks with only last lane.
	 */
	for (i = 0; i < bist_status_arr_size; i++) {
		csiphy_common_reg = &bist_reg->bist_status_arr[i];
		switch (csiphy_common_reg->csiphy_param_type) {
		case CSIPHY_2PH_REGS:
			status |= cam_io_r(csiphybase + csiphy_common_reg->reg_addr);
		break;
		}
	}

	/* Read the Counter value for possible corrupted headers */
	__cam_csiphy_read_2phase_bist_counter_status(csiphy_dev, &counter);

	if ((status & PREAMBLE_PATTERN_BIST_DONE) &&
		(status & bist_reg->error_status_val_2ph)) {
		/**
		 * This condition happen when CSIPHY try to read status after sensor
		 * streamoff. In this case error bit is set due to postamble detection
		 * which is bit(4). In this scenraio this is not consider as an error.
		 * We need to check for status2/3 counter value to determine if there are
		 * more header that is corrupted than 2. Counter always shows value of 2
		 * with postamble packet.
		 *
		 */
		if (counter <= PREAMBLE_MAX_ERR_COUNT_ALLOWED) {
			CAM_INFO(CAM_CSIPHY,
				"PN9 Pattern rxced succesfully:: counter value: 0x%x, Status0: 0x%x",
				counter, status);
		} else {
			CAM_INFO(CAM_CSIPHY,
				"PN9 Pattern is corrupted:: counter value: 0x%x, Status0: 0x%x",
				counter, status);
		}
	} else if ((status & PREAMBLE_PATTERN_BIST_DONE) &&
		!(status & bist_reg->error_status_val_2ph)) {
		/**
		 * This condition happen when CSIPHY try to read status with some counter
		 * value is set to check against. In this case error bit is not expected
		 * to be set.
		 */
		CAM_INFO(CAM_CSIPHY,
			"PN9 Pattern rxced succesfully:: counter value: 0x%x, Status0: 0x%x",
			counter, status);
	} else {
		CAM_INFO(CAM_CSIPHY,
			"PN9 Pattern is corrupted:: counter value: 0x%x Status0: 0x%x",
			counter, status);
	}

	return;
}

static void __cam_csiphy_2ph_status_checker_ops(
	struct csiphy_device *csiphy_dev, bool set)
{
	int i = 0;
	void __iomem *csiphybase = NULL;
	uint32_t base_offset = 0;
	uint32_t read_back_value;
	uint32_t lane_offset = 0;
	uint32_t offset_betwn_lane = 0;
	struct bist_reg_settings_t *bist_reg = NULL;

	csiphybase = csiphy_dev->soc_info.reg_map[0].mem_base;
	bist_reg = csiphy_dev->ctrl_reg->csiphy_bist_reg;
	base_offset = bist_reg->set_status_update_2ph_base_offset;
	offset_betwn_lane =
		csiphy_dev->ctrl_reg->csiphy_reg->size_offset_betn_lanes;

	/* Set checker bit to read the correct status1 value */
	for (i = 0; i < CAM_CSIPHY_MAX_DPHY_LANES; i++) {
		lane_offset = base_offset + (i * offset_betwn_lane);
		read_back_value = cam_io_r(csiphybase + lane_offset);
		set ? (read_back_value |= PREAMBLE_PATTERN_SET_CHECKER) :
			(read_back_value &= ~PREAMBLE_PATTERN_SET_CHECKER);
		cam_io_w_mb(read_back_value, csiphybase + lane_offset);
	}
}

static void __cam_csiphy_read_3phase_bist_counter_status(
	struct csiphy_device *csiphy_dev, uint32_t *counter)
{
	int i = 0, lane_count;
	int bist_status_arr_size =
		csiphy_dev->ctrl_reg->csiphy_bist_reg->number_of_counters;
	uint32_t base_offset = 0;
	void __iomem *phy_base = NULL;
	uint32_t val = 0;
	uint32_t offset_betwn_lane = 0;
	struct bist_reg_settings_t *bist_reg = NULL;

	phy_base = csiphy_dev->soc_info.reg_map[0].mem_base;
	bist_reg = csiphy_dev->ctrl_reg->csiphy_bist_reg;
	offset_betwn_lane =
		csiphy_dev->ctrl_reg->csiphy_reg->size_offset_betn_lanes;

	for (i = 0; i < bist_status_arr_size; i++) {
		base_offset = bist_reg->bist_counter_3ph_base_offset + (0x4 * i);
		val = 0;
		for (lane_count = 0; lane_count < CAM_CSIPHY_MAX_CPHY_LANES; lane_count++) {
			CAM_DBG(CAM_CSIPHY, "value to be read from addr: 0x%x is 0x%x",
				base_offset + (lane_count * offset_betwn_lane),
				(cam_io_r(phy_base + base_offset + (lane_count * offset_betwn_lane))));
			val |= ((cam_io_r(phy_base + base_offset + (lane_count * offset_betwn_lane))));
		}

		*counter |= (val << (i * 8));
		CAM_DBG(CAM_CSIPHY, "COUNTER VALUE is 0x%x", *counter);
	}

	return;
}

static void __cam_csiphy_get_3phase_pattern_status(
	struct csiphy_device *csiphy_dev)
{
	int i = 0;
	int bist_status_arr_size =
		csiphy_dev->ctrl_reg->csiphy_bist_reg->num_status_reg;
	struct csiphy_reg_t *csiphy_common_reg = NULL;
	void __iomem *csiphybase = NULL;
	uint32_t base_offset = 0;
	uint32_t lane_offset = 0;
	uint32_t status1 = 0, status0 = 0;
	uint32_t counter = 0;
	uint32_t offset_betwn_lane = 0;
	struct bist_reg_settings_t *bist_reg = NULL;

	CAM_DBG(CAM_CSIPHY, "ENTER");
	csiphybase = csiphy_dev->soc_info.reg_map[0].mem_base;
	bist_reg = csiphy_dev->ctrl_reg->csiphy_bist_reg;
	base_offset = bist_reg->bist_sensor_data_3ph_status_base_offset;
	offset_betwn_lane =
		csiphy_dev->ctrl_reg->csiphy_reg->size_offset_betn_lanes;

	/* This loop is to read every lane status value
	 * in case if loop breaks with only last lane.
	 */
	for (i = 0; i < bist_status_arr_size; i++) {
		csiphy_common_reg = &bist_reg->bist_status_arr[i];
		switch (csiphy_common_reg->csiphy_param_type) {
		case CSIPHY_3PH_REGS:
			status1 |= cam_io_r(csiphybase + csiphy_common_reg->reg_addr);
		break;
		}
	}

	/* Read Status0 value to detect sensor related communication */
	for (i = 0; i < CAM_CSIPHY_MAX_CPHY_LANES; i++) {
		lane_offset = base_offset + (i * offset_betwn_lane);
		status0 |= cam_io_r(csiphybase + lane_offset);
	}


	/* Read the Counter value for possible corrupted headers */
	__cam_csiphy_read_3phase_bist_counter_status(csiphy_dev, &counter);

	if ((status1 & PREAMBLE_PATTERN_BIST_DONE) &&
		(status1 & bist_reg->error_status_val_3ph)) {
		/**
		 * This condition happen when CSIPHY try to read status after sensor
		 * streamoff. In this case error bit is set due to postamble detection
		 * which is bit(4). In this scenraio this is not consider as an error.
		 * We need to check for status2/3 counter value to determine if there are
		 * more header that is corrupted than 2. Counter always shows value of 2
		 * with postamble packet.
		 *
		 */
		if (counter <= PREAMBLE_MAX_ERR_COUNT_ALLOWED) {
			CAM_INFO(CAM_CSIPHY,
				"PN9 Pattern rxced succesfully after sensor streamoff:: counter value: 0x%x Status1: 0x%x Status0: 0x%x",
				counter, status1, status0);
		} else {
			CAM_INFO(CAM_CSIPHY,
				"PN9 Pattern is corrupted:: counter value: 0x%x Status1: 0x%x Status0: 0x%x",
				counter, status1, status0);
		}
	} else if ((status1 & PREAMBLE_PATTERN_BIST_DONE) &&
		!(status1 & bist_reg->error_status_val_3ph)) {
		/**
		 * This condition happen when CSIPHY try to read status with some counter
		 * value is set to check against. In this case error bit is not expected
		 * to be set.
		 */
		CAM_INFO(CAM_CSIPHY,
			"PN9 Pattern rxced succesfully before sensor streamoff:: counter value: 0x%x Status1: 0x%x Status0: 0x%x",
			counter, status1, status0);
	} else {
		CAM_INFO(CAM_CSIPHY,
			"PN9 Pattern is corrupted:: counter value: 0x%x Status1: 0x%x Status0: 0x%x",
			counter, status1, status0);
	}

	return;
}

static void __cam_csiphy_3ph_status_checker_ops(
	struct csiphy_device *csiphy_dev, bool set)
{
	int i = 0;
	void __iomem *csiphybase = NULL;
	uint32_t base_offset = 0;
	uint32_t read_back_value;
	uint32_t lane_offset = 0;
	uint32_t offset_betwn_lane = 0;

	csiphybase = csiphy_dev->soc_info.reg_map[0].mem_base;
	base_offset =
		csiphy_dev->ctrl_reg->csiphy_bist_reg->set_status_update_3ph_base_offset;
	offset_betwn_lane =
		csiphy_dev->ctrl_reg->csiphy_reg->size_offset_betn_lanes;
	/* Set checker bit to read the correct status1 value */
	for (i = 0; i < CAM_CSIPHY_MAX_CPHY_LANES; i++) {
		lane_offset = base_offset + (i * offset_betwn_lane);
		read_back_value = cam_io_r(csiphybase + lane_offset);
		set ? (read_back_value |= PREAMBLE_PATTERN_SET_CHECKER) :
			(read_back_value &= ~PREAMBLE_PATTERN_SET_CHECKER);
		cam_io_w_mb(read_back_value, csiphybase + lane_offset);
	}
}

static void __cam_csiphy_get_preamble_status(
	struct csiphy_device *csiphy_dev, int offset)
{
	bool is_3phase = false;

	is_3phase = csiphy_dev->csiphy_info[offset].csiphy_3phase;

	if (is_3phase) {
		__cam_csiphy_3ph_status_checker_ops(csiphy_dev, true);
		__cam_csiphy_get_3phase_pattern_status(csiphy_dev);
		__cam_csiphy_3ph_status_checker_ops(csiphy_dev, false);
	} else {
		__cam_csiphy_2ph_status_checker_ops(csiphy_dev, true);
		__cam_csiphy_get_2phase_pattern_status(csiphy_dev);
		__cam_csiphy_2ph_status_checker_ops(csiphy_dev, false);
	}
	return;
}

int32_t cam_csiphy_core_cfg(void *phy_dev,
			void *arg)
{
	struct cam_control   *cmd = (struct cam_control *)arg;
	struct csiphy_device *csiphy_dev = (struct csiphy_device *)phy_dev;
	struct cam_cphy_dphy_status_reg_params_t *status_reg_ptr;
	struct csiphy_reg_parms_t *csiphy_reg;
	struct cam_hw_soc_info *soc_info;
	uint32_t      cphy_trio_status;
	void __iomem *csiphybase;
	int32_t              rc = 0;

	if (!csiphy_dev || !cmd) {
		CAM_ERR(CAM_CSIPHY, "Invalid input args");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_CSIPHY, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	soc_info = &csiphy_dev->soc_info;
	if (!soc_info) {
		CAM_ERR(CAM_CSIPHY, "Null Soc_info");
		return -EINVAL;
	}

	if (!g_phy_data[soc_info->index].base_address) {
		CAM_ERR(CAM_CSIPHY, "CSIPHY hw is not avaialble at index: %d",
			soc_info->index);
		return -EINVAL;
	}

	csiphybase = soc_info->reg_map[0].mem_base;
	csiphy_reg = csiphy_dev->ctrl_reg->csiphy_reg;
	status_reg_ptr = csiphy_reg->status_reg_params;

	CAM_DBG(CAM_CSIPHY, "Opcode received: %d", cmd->op_code);
	mutex_lock(&csiphy_dev->mutex);
	switch (cmd->op_code) {
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev csiphy_acq_dev;
		struct cam_csiphy_acquire_dev_info csiphy_acq_params;
		int index;
		struct cam_create_dev_hdl bridge_params;

		CAM_DBG(CAM_CSIPHY, "ACQUIRE_CNT: %d COMBO_MODE: %d",
			csiphy_dev->acquire_count,
			csiphy_dev->combo_mode);
		if ((csiphy_dev->csiphy_state == CAM_CSIPHY_START) &&
			(csiphy_dev->combo_mode == 0) &&
			(csiphy_dev->acquire_count > 0) && (csiphy_dev->is_aggregator_rx == 0)) {
			CAM_ERR(CAM_CSIPHY,
				"NonComboMode does not support multiple acquire: Acquire_count: %d",
				csiphy_dev->acquire_count);
			rc = -EINVAL;
			goto release_mutex;
		}

		if ((csiphy_dev->acquire_count) &&
			(csiphy_dev->acquire_count >=
			csiphy_dev->session_max_device_support)) {
			CAM_ERR(CAM_CSIPHY,
				"Max acquires are allowed in combo mode: %d",
				csiphy_dev->session_max_device_support);
			rc = -EINVAL;
			goto release_mutex;
		}

		rc = copy_from_user(&csiphy_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(csiphy_acq_dev));
		if (rc < 0) {
			CAM_ERR(CAM_CSIPHY, "Failed copying from User");
			goto release_mutex;
		}

		csiphy_acq_params.combo_mode = 0;

		if (copy_from_user(&csiphy_acq_params,
			u64_to_user_ptr(csiphy_acq_dev.info_handle),
			sizeof(csiphy_acq_params))) {
			CAM_ERR(CAM_CSIPHY,
				"Failed copying from User");
			goto release_mutex;
		}

		if (csiphy_acq_params.combo_mode &&
			csiphy_acq_params.cphy_dphy_combo_mode) {
			CAM_ERR(CAM_CSIPHY,
				"Cannot support both Combo_mode and cphy_dphy_combo_mode");
			rc = -EINVAL;
			goto release_mutex;
		}

		if (csiphy_acq_params.combo_mode == 1) {
			CAM_DBG(CAM_CSIPHY, "combo mode stream detected");
			csiphy_dev->combo_mode = 1;
			if (csiphy_acq_params.csiphy_3phase) {
				CAM_DBG(CAM_CSIPHY, "3Phase ComboMode");
				csiphy_dev->session_max_device_support =
					CSIPHY_MAX_INSTANCES_PER_PHY;
			} else {
				csiphy_dev->session_max_device_support =
					CSIPHY_MAX_INSTANCES_PER_PHY - 1;
				CAM_DBG(CAM_CSIPHY, "2Phase ComboMode");
			}
		}
		if (csiphy_acq_params.cphy_dphy_combo_mode == 1) {
			CAM_DBG(CAM_CSIPHY,
				"cphy_dphy_combo_mode stream detected");
			csiphy_dev->cphy_dphy_combo_mode = 1;
			csiphy_dev->session_max_device_support =
				CSIPHY_MAX_INSTANCES_PER_PHY - 1;
		}

		if (!csiphy_acq_params.combo_mode &&
			!csiphy_acq_params.cphy_dphy_combo_mode) {
			CAM_DBG(CAM_CSIPHY, "Non Combo Mode stream");
			csiphy_dev->session_max_device_support = 1;
		}

		if (csiphy_dev->is_aggregator_rx)
			csiphy_dev->session_max_device_support =
				CSIPHY_MAX_INSTANCES_PER_AGGREG_RX_PHY;

		bridge_params.ops = NULL;
		bridge_params.session_hdl = csiphy_acq_dev.session_handle;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = csiphy_dev;
		bridge_params.dev_id = CAM_CSIPHY;
		csiphy_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		if (csiphy_acq_dev.device_handle <= 0) {
			rc = -EFAULT;
			CAM_ERR(CAM_CSIPHY, "Can not create device handle");
			goto release_mutex;
		}

		for (index = 0; index < csiphy_dev->session_max_device_support; index++) {
			if (csiphy_dev->csiphy_info[index].hdl_data.device_hdl == -1)
				break;
		}

		if (index >= csiphy_dev->session_max_device_support) {
			CAM_ERR(CAM_CSIPHY, "Index is invalid: %d", index);
			goto release_mutex;
		}

		csiphy_dev->csiphy_info[index].hdl_data.device_hdl =
			csiphy_acq_dev.device_handle;
		csiphy_dev->csiphy_info[index].hdl_data.session_hdl =
			csiphy_acq_dev.session_handle;
		csiphy_dev->csiphy_info[index].csiphy_3phase =
			csiphy_acq_params.csiphy_3phase;
		csiphy_dev->csiphy_info[index].conn_csid_idx = -1;
		csiphy_dev->csiphy_info[index].use_hw_client_voting = false;
		csiphy_dev->csiphy_info[index].is_drv_config_en = false;

		CAM_DBG(CAM_CSIPHY, "Add dev_handle:0x%x at index: %d ",
			csiphy_dev->csiphy_info[index].hdl_data.device_hdl,
			index);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
				&csiphy_acq_dev,
				sizeof(struct cam_sensor_acquire_dev))) {
			CAM_ERR(CAM_CSIPHY, "Failed copying from User");
			rc = -EINVAL;
			goto release_mutex;
		}

		if (!csiphy_dev->acquire_count) {
			g_phy_data[soc_info->index].is_3phase = csiphy_acq_params.csiphy_3phase;
			CAM_DBG(CAM_CSIPHY,
				"g_csiphy data is updated for index: %d is_3phase: %u",
				soc_info->index,
				g_phy_data[soc_info->index].is_3phase);
		}

		if (g_phy_data[soc_info->index].aon_cam_id != NOT_AON_CAM) {
			rc = cam_csiphy_util_update_aon_ops(true, soc_info->index);
			if (rc) {
				CAM_ERR(CAM_CSIPHY,
					"Error in setting up AON operation for phy_idx: %d, rc: %d",
					soc_info->index, rc);
				goto release_mutex;
			}
		}

		csiphy_dev->acquire_count++;

		if (csiphy_dev->csiphy_state == CAM_CSIPHY_INIT)
			csiphy_dev->csiphy_state = CAM_CSIPHY_ACQUIRE;

		CAM_INFO(CAM_CSIPHY,
			"CAM_ACQUIRE_DEV: %u Type: %s acquire_count: %d combo: %u cphy+dphy combo: %u",
			soc_info->index,
			g_phy_data[soc_info->index].is_3phase ? "CPHY" : "DPHY",
			csiphy_dev->acquire_count,
			csiphy_dev->combo_mode,
			csiphy_dev->cphy_dphy_combo_mode);
	}
		break;
	case CAM_QUERY_CAP: {
		struct cam_csiphy_query_cap csiphy_cap = {0};

		cam_csiphy_query_cap(csiphy_dev, &csiphy_cap);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&csiphy_cap, sizeof(struct cam_csiphy_query_cap))) {
			CAM_ERR(CAM_CSIPHY, "Failed copying from User");
			rc = -EINVAL;
			goto release_mutex;
		}
	}
		break;
	case CAM_STOP_DEV: {
		int32_t offset, rc = 0;
		struct cam_start_stop_dev_cmd config;
		struct cam_csiphy_param *param;

		rc = copy_from_user(&config, (void __user *)cmd->handle,
					sizeof(config));
		if (rc < 0) {
			CAM_ERR(CAM_CSIPHY, "Failed copying from User");
			goto release_mutex;
		}

		if (csiphy_dev->csiphy_state != CAM_CSIPHY_START) {
			CAM_ERR(CAM_CSIPHY, "Csiphy:%d Not in right state to stop : %d",
				soc_info->index, csiphy_dev->csiphy_state);
			goto release_mutex;
		}

		offset = cam_csiphy_get_instance_offset(csiphy_dev,
			config.dev_handle);
		if (offset < 0 ||
			offset >= csiphy_dev->session_max_device_support) {
			CAM_ERR(CAM_CSIPHY, "Index is invalid: %d", offset);
			goto release_mutex;
		}

		param = &csiphy_dev->csiphy_info[offset];

		cam_csiphy_update_lane_assign_info(csiphy_dev, offset, false);

		if (--csiphy_dev->start_dev_count) {
#ifdef CONFIG_SECURE_CAMERA
			if (param->secure_mode)
				cam_csiphy_program_secure_mode(csiphy_dev,
					CAM_SECURE_MODE_NON_SECURE, offset);
#endif
			param->secure_mode = CAM_SECURE_MODE_NON_SECURE;
			param->csiphy_cpas_cp_reg_mask = 0;

			if (soc_info->is_clk_drv_en && param->use_hw_client_voting) {
				rc = cam_soc_util_set_src_clk_rate(soc_info, param->conn_csid_idx,
					0, 0);
				if (rc) {
					CAM_ERR(CAM_CSIPHY,
						"[%d] Failed in setting clk rate for %d",
						soc_info->index, param->conn_csid_idx);
				} else {
					rc = cam_soc_util_cesta_channel_switch(param->conn_csid_idx,
						"csiphy_stop_dev");
					if (rc) {
						CAM_ERR(CAM_CSIPHY,
							"Failed to apply power states for cesta client:%d rc:%d",
							param->conn_csid_idx, rc);
					}
				}
			}

			CAM_INFO(CAM_CSIPHY,
				"CAM_STOP_PHYDEV: %d, CSID:%d, Type: %s, dev_cnt: %u, slot: %d, Datarate: %llu, Settletime: %llu",
				soc_info->index, param->conn_csid_idx,
				g_phy_data[soc_info->index].is_3phase ? "CPHY" : "DPHY",
				csiphy_dev->start_dev_count, offset, param->data_rate,
				param->settle_time);

			goto release_mutex;
		}

#ifdef CONFIG_SECURE_CAMERA
		if (param->secure_mode)
			cam_csiphy_program_secure_mode(csiphy_dev, CAM_SECURE_MODE_NON_SECURE,
				offset);
#endif
		param->secure_mode = CAM_SECURE_MODE_NON_SECURE;
		param->csiphy_cpas_cp_reg_mask = 0x0;

		if (csiphy_dev->prgm_cmn_reg_across_csiphy) {
			mutex_lock(&active_csiphy_cnt_mutex);
			active_csiphy_hw_cnt--;
			mutex_unlock(&active_csiphy_cnt_mutex);

			cam_csiphy_program_common_registers(csiphy_dev, true,
				CAM_CSIPHY_PRGM_ALL);
		} else {
			cam_csiphy_program_common_registers(csiphy_dev, true,
				CAM_CSIPHY_PRGM_INDVDL);
		}

		if (csiphy_dev->preamble_enable)
			__cam_csiphy_get_preamble_status(csiphy_dev, offset);

		cam_csiphy_update_lane_selection(csiphy_dev, offset, false);

		rc = cam_csiphy_disable_hw(csiphy_dev, offset);
		if (rc < 0)
			CAM_ERR(CAM_CSIPHY, "Failed in csiphy release");

		if (!g_phy_data[soc_info->index].is_configured_for_main) {
			if (cam_csiphy_cpas_ops(csiphy_dev->cpas_handle, false)) {
				CAM_ERR(CAM_CSIPHY, "Failed in de-voting CPAS");
				rc = -EFAULT;
			}
		}

		csiphy_dev->csiphy_state = CAM_CSIPHY_ACQUIRE;

		CAM_INFO(CAM_CSIPHY,
			"CAM_STOP_PHYDEV: %u, CSID:%d, Type: %s, slot: %d, Datarate: %llu, Settletime: %llu",
			soc_info->index, param->conn_csid_idx,
			g_phy_data[soc_info->index].is_3phase ? "CPHY" : "DPHY",
			offset, param->data_rate, param->settle_time);
	}
		break;
	case CAM_RELEASE_DEV: {
		int32_t offset;
		struct cam_release_dev_cmd release;

		if (!csiphy_dev->acquire_count) {
			CAM_ERR(CAM_CSIPHY, "No valid devices to release");
			rc = -EINVAL;
			goto release_mutex;
		}

		if (copy_from_user(&release,
			u64_to_user_ptr(cmd->handle),
			sizeof(release))) {
			rc = -EFAULT;
			goto release_mutex;
		}

		offset = cam_csiphy_get_instance_offset(csiphy_dev,
			release.dev_handle);
		if (offset < 0 ||
			offset >= csiphy_dev->session_max_device_support) {
			CAM_ERR(CAM_CSIPHY, "index is invalid: %d", offset);
			goto release_mutex;
		}

#ifdef CONFIG_SECURE_CAMERA
		if (csiphy_dev->csiphy_info[offset].secure_mode)
			cam_csiphy_program_secure_mode(
				csiphy_dev,
				CAM_SECURE_MODE_NON_SECURE, offset);
#endif
		csiphy_dev->csiphy_info[offset].secure_mode =
			CAM_SECURE_MODE_NON_SECURE;

		csiphy_dev->csiphy_cpas_cp_reg_mask[offset] = 0x0;
		csiphy_dev->preamble_enable = 0;

		rc = cam_destroy_device_hdl(release.dev_handle);
		if (rc < 0)
			CAM_ERR(CAM_CSIPHY, "destroying the device hdl");
		csiphy_dev->csiphy_info[offset].hdl_data.device_hdl = -1;
		csiphy_dev->csiphy_info[offset].hdl_data.session_hdl = -1;

		cam_csiphy_reset_phyconfig_param(csiphy_dev, offset);

		if (csiphy_dev->acquire_count) {
			csiphy_dev->acquire_count--;
			CAM_DBG(CAM_CSIPHY, "Acquire_cnt: %d",
				csiphy_dev->acquire_count);
		}

		if (csiphy_dev->acquire_count == 0) {
			CAM_DBG(CAM_CSIPHY, "All PHY devices released");
			if (g_phy_data[soc_info->index].aon_cam_id != NOT_AON_CAM) {
				rc = cam_csiphy_util_update_aon_ops(false, soc_info->index);
				if (rc) {
					CAM_WARN(CAM_CSIPHY,
						"Error in releasing AON operation for phy_idx: %d, rc: %d",
						csiphy_dev->soc_info.index, rc);
					rc = 0;
				}
			}
			csiphy_dev->combo_mode = 0;
			csiphy_dev->csiphy_state = CAM_CSIPHY_INIT;
		}

		if (csiphy_dev->cdr_params.cdr_sweep_enabled)
			memset(&csiphy_dev->cdr_params, 0x0,
				sizeof(struct cam_csiphy_dev_cdr_sweep_params));

		if (csiphy_dev->aux_params.aux_mem_update_en)
			memset(&csiphy_dev->aux_params, 0x0,
				sizeof(struct cam_csiphy_dev_aux_setting_params));

		CAM_DBG(CAM_CSIPHY, "CAM_RELEASE_PHYDEV: %u Type: %s",
			soc_info->index,
			g_phy_data[soc_info->index].is_3phase ? "CPHY" : "DPHY");

		if (csiphy_dev->acquire_count < csiphy_dev->start_dev_count) {
			rc = -EINVAL;
			CAM_ERR(CAM_CSIPHY,
				"PHYDEV %u streamon count:%u bigger than acquire count:%u, missing stream offs",
				soc_info->index, csiphy_dev->start_dev_count,
				csiphy_dev->acquire_count);
		}

		break;
	}
	case CAM_CONFIG_DEV: {
		struct cam_config_dev_cmd config;

		CAM_DBG(CAM_CSIPHY, "CONFIG_DEV Called");

		if (copy_from_user(&config, u64_to_user_ptr(cmd->handle), sizeof(config))) {
			CAM_ERR(CAM_CSIPHY, "Couldn't copy the Entire Config From User");
			rc = -EFAULT;
		} else {
			rc = cam_cmd_buf_parser(csiphy_dev, &config);
			if (rc < 0) {
				CAM_ERR(CAM_CSIPHY, "Fail in cmd buf parser");
				goto release_mutex;
			}
		}
		break;
	}
	case CAM_START_DEV: {
		struct cam_csiphy_param *param;
		struct cam_start_stop_dev_cmd config;
		int32_t i, offset;
		int clk_vote_level_high = -1;
		int clk_vote_level_low = -1;
		uint8_t data_rate_variant_idx = 0;
		unsigned long clk_rate = 0;

		CAM_DBG(CAM_CSIPHY, "START_DEV Called");
		rc = copy_from_user(&config, (void __user *)cmd->handle,
			sizeof(config));
		if (rc < 0) {
			CAM_ERR(CAM_CSIPHY, "Failed copying from User");
			goto release_mutex;
		}

		if ((csiphy_dev->csiphy_state == CAM_CSIPHY_START) &&
			(csiphy_dev->start_dev_count >
			csiphy_dev->session_max_device_support)) {
			CAM_ERR(CAM_CSIPHY,
				"Invalid start count: %d, Max supported devices: %u",
				csiphy_dev->start_dev_count,
				csiphy_dev->session_max_device_support);
			rc = -EINVAL;
			goto release_mutex;
		}

		offset = cam_csiphy_get_instance_offset(csiphy_dev,
			config.dev_handle);
		if (offset < 0 ||
			offset >= csiphy_dev->session_max_device_support) {
			CAM_ERR(CAM_CSIPHY, "index is invalid: %d", offset);
			goto release_mutex;
		}

		param = &csiphy_dev->csiphy_info[offset];

		rc = cam_cpas_query_drv_enable(NULL, &soc_info->is_clk_drv_en);
		if (rc) {
			CAM_ERR(CAM_CSIPHY, "Failed to query DRV enable rc: %d", rc);
			goto release_mutex;
		}

		cam_csiphy_update_lane_assign_info(csiphy_dev, offset, true);

		if (csiphy_dev->start_dev_count) {
			if (csiphy_dev->is_aggregator_rx) {
				if ((csiphy_dev->lanes_enabled
					& csiphy_dev->csiphy_info[offset].lane_enable)
					== csiphy_dev->csiphy_info[offset].lane_enable) {
					csiphy_dev->start_dev_count++;
					CAM_INFO(CAM_CSIPHY,
						"CAM_START_PHYDEV: %d dev_cnt: %u, slot: %d",
						soc_info->index,
						csiphy_dev->start_dev_count,
						offset);
					goto release_mutex;
				}
			}
			clk_vote_level_high =
				csiphy_dev->ctrl_reg->getclockvoting(csiphy_dev, offset);
			clk_vote_level_low = clk_vote_level_high;

			CAM_DBG(CAM_CSIPHY,
				"CSIPHY[%d] is_clk_drv_en[%d] conn_csid_idx[%d] use_hw_client_voting[%d] is_drv_config_en[%d]",
				csiphy_dev->soc_info.index, soc_info->is_clk_drv_en,
				param->conn_csid_idx, param->use_hw_client_voting,
				param->is_drv_config_en);

			if (soc_info->is_clk_drv_en && param->use_hw_client_voting) {
				if (param->is_drv_config_en)
					clk_vote_level_low = soc_info->lowest_clk_level;

				rc = cam_soc_util_set_clk_rate_level(&csiphy_dev->soc_info,
					param->conn_csid_idx, clk_vote_level_high,
					clk_vote_level_low, false);
				if (rc) {
					CAM_ERR(CAM_CSIPHY,
						"Failed to set the req clk_rate level[high low]: [%s %s] cesta_client_idx: %d rc: %d",
						cam_soc_util_get_string_from_level(
						clk_vote_level_high),
						cam_soc_util_get_string_from_level(
						clk_vote_level_low), param->conn_csid_idx, rc);
					rc = -EINVAL;
					goto release_mutex;
				}

				rc = cam_soc_util_cesta_channel_switch(param->conn_csid_idx,
					"csiphy_combo");
				if (rc) {
					CAM_ERR(CAM_CSIPHY,
						"Failed to apply power states for crm client:%d rc:%d",
						param->conn_csid_idx, rc);
					rc = 0;
				}
			} else {
				rc = cam_soc_util_set_clk_rate_level(&csiphy_dev->soc_info,
					CAM_CLK_SW_CLIENT_IDX, clk_vote_level_high, 0, false);
				if (rc) {
					CAM_WARN(CAM_CSIPHY,
						"Failed to set the req clk_rate level: %s",
						cam_soc_util_get_string_from_level(
						clk_vote_level_high));
					rc = -EINVAL;
					goto release_mutex;

				}

				for (i = 0; i < csiphy_dev->soc_info.num_clk;
									i++) {
					if (i ==
					csiphy_dev->soc_info.src_clk_idx) {
						CAM_DBG(CAM_CSIPHY,
						"Skipping call back for src"
						" clk %s",
						csiphy_dev->soc_info.clk_name[
									i]);
						continue;
					}
					clk_rate =
					cam_soc_util_get_clk_rate_applied(
						&csiphy_dev->soc_info, i,
						false, clk_vote_level_high);
					if (clk_rate > 0) {
						cam_subdev_notify_message(
						CAM_TFE_DEVICE_TYPE,
					CAM_SUBDEV_MESSAGE_CLOCK_UPDATE,
						(void *)(&clk_rate));
					}
				}
			}

#ifdef CONFIG_SECURE_CAMERA
			if (csiphy_dev->csiphy_info[offset].secure_mode == 1) {
				if (!cam_cpas_is_feature_supported(
					CAM_CPAS_SECURE_CAMERA_ENABLE,
					CAM_CPAS_HW_IDX_ANY, NULL)) {
					CAM_ERR(CAM_CSIPHY,
						"sec_cam: camera fuse bit not set");
					goto release_mutex;
				}

				rc = cam_csiphy_program_secure_mode(csiphy_dev,
					CAM_SECURE_MODE_SECURE, offset);
				if (rc) {
					csiphy_dev->csiphy_info[offset]
						.secure_mode =
						CAM_SECURE_MODE_NON_SECURE;
					CAM_ERR(CAM_CSIPHY,
						"sec_cam: notify failed: rc: %d",
						rc);
					goto release_mutex;
				}
			}
#endif

			if (csiphy_dev->csiphy_info[offset].csiphy_3phase) {
				rc = cam_csiphy_cphy_data_rate_config(
					csiphy_dev, offset, data_rate_variant_idx);
				if (rc) {
					CAM_ERR(CAM_CSIPHY,
						"Data rate specific configuration failed rc: %d",
						rc);
					goto release_mutex;
				}
			}

			rc = cam_csiphy_update_lane_selection(csiphy_dev, offset, true);
			if (rc) {
				CAM_ERR(CAM_CSIPHY, "Update enable lane failed, rc: %d", rc);
				goto release_mutex;
			}

			if (csiphy_dev->en_full_phy_reg_dump)
				cam_csiphy_reg_dump(&csiphy_dev->soc_info);

			csiphy_dev->start_dev_count++;

			CAM_INFO(CAM_CSIPHY,
				"CAM_START_PHYDEV: %d, CSID:%d, Type: %s, dev_cnt: %u, slot: %d, combo: %u, cphy+dphy: %u, skew_en: %d, sec_mode: %d, Datarate: %llu, Settletime: %llu",
				soc_info->index,
				csiphy_dev->csiphy_info[offset].conn_csid_idx,
				g_phy_data[soc_info->index].is_3phase ? "CPHY" : "DPHY",
				csiphy_dev->start_dev_count,
				offset,
				csiphy_dev->combo_mode,
				csiphy_dev->cphy_dphy_combo_mode,
				csiphy_dev->csiphy_info[offset].mipi_flags,
				csiphy_dev->csiphy_info[offset].secure_mode,
				csiphy_dev->csiphy_info[offset].data_rate,
				csiphy_dev->csiphy_info[offset].settle_time);

			goto release_mutex;
		}

		if (!g_phy_data[soc_info->index].is_configured_for_main) {
			if (cam_csiphy_cpas_ops(csiphy_dev->cpas_handle, true)) {
				rc = -EFAULT;
				CAM_ERR(CAM_CSIPHY, "voting CPAS: %d", rc);
				goto release_mutex;
			}
		}

#ifdef CONFIG_SECURE_CAMERA
		if (csiphy_dev->csiphy_info[offset].secure_mode == 1) {
			if (!cam_cpas_is_feature_supported(
					CAM_CPAS_SECURE_CAMERA_ENABLE,
					CAM_CPAS_HW_IDX_ANY, NULL)) {
				CAM_ERR(CAM_CSIPHY,
					"sec_cam: camera fuse bit not set");
				rc = -EINVAL;
				goto cpas_stop;
			}

			rc = cam_csiphy_program_secure_mode(
				csiphy_dev,
				CAM_SECURE_MODE_SECURE, offset);
			if (rc) {
				csiphy_dev->csiphy_info[offset].secure_mode =
					CAM_SECURE_MODE_NON_SECURE;
				goto cpas_stop;
			}
		}
#endif

		rc = cam_csiphy_enable_hw(csiphy_dev, offset);
		if (rc != 0) {
			CAM_ERR(CAM_CSIPHY, "cam_csiphy_enable_hw failed");
			goto cpas_stop;
		}

		rc = cam_csiphy_update_lane_selection(csiphy_dev, offset, true);
		if (rc) {
			CAM_ERR(CAM_CSIPHY, "Update enable lane failed, rc: %d", rc);
			goto cpas_stop;
		}

		if (csiphy_dev->prgm_cmn_reg_across_csiphy) {
			cam_csiphy_program_common_registers(csiphy_dev, false, CAM_CSIPHY_PRGM_ALL);
			mutex_lock(&active_csiphy_cnt_mutex);
			active_csiphy_hw_cnt++;
			mutex_unlock(&active_csiphy_cnt_mutex);
		} else {
			cam_csiphy_program_common_registers(csiphy_dev, false,
				CAM_CSIPHY_PRGM_INDVDL);
		}

		rc = cam_csiphy_config_dev(csiphy_dev, config.dev_handle, data_rate_variant_idx);
		if (rc < 0) {
			CAM_ERR(CAM_CSIPHY, "cam_csiphy_config_dev failed");
			cam_csiphy_disable_hw(csiphy_dev, offset);
			goto hw_cnt_decrement;
		}

		if (csiphy_onthego_reg_count)
			cam_csiphy_apply_onthego_reg_values(csiphybase, soc_info->index);

		cam_csiphy_release_from_reset_state(csiphy_dev, csiphybase, offset);

		if (g_phy_data[soc_info->index].is_3phase && status_reg_ptr) {
			for (i = 0; i < CAM_CSIPHY_MAX_CPHY_LANES; i++) {
				if (status_reg_ptr->cphy_lane_status[i]) {
					cphy_trio_status = cam_io_r_mb(csiphybase +
						status_reg_ptr->cphy_lane_status[i]);

					cphy_trio_status &= 0x1F;
					if (cphy_trio_status == 0 || cphy_trio_status == 8) {
						CAM_DBG(CAM_CSIPHY,
							"Reg_offset: 0x%x, cphy_trio%d_status = 0x%x",
							status_reg_ptr->cphy_lane_status[i],
							i, cphy_trio_status);
					} else {
						CAM_WARN(CAM_CSIPHY,
							"Reg_offset: 0x%x, Cphy_trio%d_status = 0x%x",
							status_reg_ptr->cphy_lane_status[i],
							i, cphy_trio_status);
					}
				}
			}
		}

		if (csiphy_dev->en_full_phy_reg_dump)
			cam_csiphy_reg_dump(&csiphy_dev->soc_info);

		if (csiphy_dev->en_lane_status_reg_dump) {
			usleep_range(50000, 50005);
			CAM_INFO(CAM_CSIPHY, "Status Reg Dump after config");
			cam_csiphy_dump_status_reg(csiphy_dev);
		}

		csiphy_dev->start_dev_count++;
		csiphy_dev->csiphy_state = CAM_CSIPHY_START;

		CAM_INFO(CAM_CSIPHY,
			"CAM_START_PHYDEV: %d, CSID:%d, Type: %s, dev_cnt: %u, slot: %d, combo: %u, cphy+dphy: %u, skew_en: %d, sec_mode: %d, Datarate: %llu, Settletime: %llu",
			soc_info->index,
			csiphy_dev->csiphy_info[offset].conn_csid_idx,
			g_phy_data[soc_info->index].is_3phase ? "CPHY" : "DPHY",
			csiphy_dev->start_dev_count,
			offset,
			csiphy_dev->combo_mode,
			csiphy_dev->cphy_dphy_combo_mode,
			csiphy_dev->csiphy_info[offset].mipi_flags,
			csiphy_dev->csiphy_info[offset].secure_mode,
			csiphy_dev->csiphy_info[offset].data_rate,
			csiphy_dev->csiphy_info[offset].settle_time);
	}
		break;
	case CAM_CONFIG_DEV_EXTERNAL: {
		struct cam_config_dev_cmd submit_cmd;

		if (copy_from_user(&submit_cmd,
			u64_to_user_ptr(cmd->handle),
			sizeof(struct cam_config_dev_cmd))) {
			CAM_ERR(CAM_CSIPHY, "failed copy config ext\n");
			rc = -EFAULT;
			goto release_mutex;
		} else {
			rc = cam_csiphy_external_cmd(csiphy_dev, &submit_cmd);
			if (rc) {
				CAM_ERR(CAM_CSIPHY,
					"exteranal command configuration failed rc: %d",
					rc);
				goto release_mutex;
			}
		}
		break;
	}
	default:
		CAM_ERR(CAM_CSIPHY, "Invalid Opcode: %d", cmd->op_code);
		rc = -EINVAL;
		goto release_mutex;
	}

	mutex_unlock(&csiphy_dev->mutex);
	return rc;

hw_cnt_decrement:
	if (csiphy_dev->prgm_cmn_reg_across_csiphy) {
		mutex_lock(&active_csiphy_cnt_mutex);
		active_csiphy_hw_cnt--;
		mutex_unlock(&active_csiphy_cnt_mutex);
	}

cpas_stop:
	if (cam_csiphy_cpas_ops(csiphy_dev->cpas_handle, false))
		CAM_ERR(CAM_CSIPHY, "cpas stop failed");
release_mutex:
	mutex_unlock(&csiphy_dev->mutex);

	return rc;
}

int cam_csiphy_register_baseaddress(struct csiphy_device *csiphy_dev)
{
	int phy_idx;

	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "Data is NULL");
		return -EINVAL;
	}

	if (csiphy_dev->soc_info.index >= MAX_CSIPHY) {
		CAM_ERR(CAM_CSIPHY, "Invalid soc index: %u Max soc index: %u",
			csiphy_dev->soc_info.index, MAX_CSIPHY);
		return -EINVAL;
	}

	phy_idx = csiphy_dev->soc_info.index;
	g_phy_data[phy_idx].base_address =
		csiphy_dev->soc_info.reg_map[0].mem_base;
	g_phy_data[phy_idx].cpas_handle =
		csiphy_dev->cpas_handle;
	g_phy_data[phy_idx].aon_sel_param =
		csiphy_dev->ctrl_reg->csiphy_reg->aon_sel_params;
	g_phy_data[phy_idx].aon_cam_id = NOT_AON_CAM;
	g_phy_data[phy_idx].is_configured_for_main = false;
	g_phy_data[phy_idx].data_rate_aux_mask = 0;

	return 0;
}
