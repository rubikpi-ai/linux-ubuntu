// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/slab.h>
#include "cam_io_util.h"
#include "cam_cdm_util.h"
#include "cam_vfe_hw_intf.h"
#include "cam_vfe_top.h"
#include "cam_vfe_top_ver4.h"
#include "cam_debug_util.h"
#include "cam_vfe_soc.h"
#include "cam_trace.h"
#include "cam_isp_hw_mgr_intf.h"
#include "cam_irq_controller.h"
#include "cam_tasklet_util.h"
#include "cam_cdm_intf_api.h"

#define CAM_SHIFT_TOP_CORE_VER_4_CFG_DSP_EN            8
#define CAM_VFE_CAMIF_IRQ_SOF_DEBUG_CNT_MAX            2
#define CAM_VFE_LEN_LOG_BUF                            256

struct cam_vfe_top_ver4_common_data {
	struct cam_hw_intf                         *hw_intf;
	struct cam_vfe_top_ver4_reg_offset_common  *common_reg;
	struct cam_vfe_top_ver4_hw_info            *hw_info;
};

struct cam_vfe_top_ver4_perf_counter_cfg {
	uint32_t perf_counter_val;
	bool     dump_counter;
};

struct cam_vfe_top_ver4_prim_sof_ts_reg_addr {
	void __iomem  *curr0_ts_addr;
	void __iomem  *curr1_ts_addr;
};

struct cam_vfe_top_ver4_priv {
	struct cam_vfe_top_ver4_common_data          common_data;
	struct cam_vfe_top_priv_common               top_common;
	atomic_t                                     overflow_pending;
	uint8_t                                      log_buf[CAM_VFE_LEN_LOG_BUF];
	uint32_t                                     sof_cnt;
	struct cam_vfe_top_ver4_perf_counter_cfg     perf_counters[CAM_VFE_PERF_CNT_MAX];
	struct cam_vfe_top_ver4_prim_sof_ts_reg_addr sof_ts_reg_addr;
	bool                                         enable_ife_frame_irqs;
};

enum cam_vfe_top_ver4_fsm_state {
	VFE_TOP_VER4_FSM_SOF = 0,
	VFE_TOP_VER4_FSM_EPOCH,
	VFE_TOP_VER4_FSM_EOF,
	VFE_TOP_VER4_FSM_MAX,
};

struct cam_vfe_mux_ver4_data {
	void __iomem                                *mem_base;
	struct cam_hw_soc_info                      *soc_info;
	struct cam_hw_intf                          *hw_intf;
	struct cam_vfe_top_ver4_reg_offset_common   *common_reg;
	struct cam_vfe_top_common_cfg                cam_common_cfg;
	struct cam_vfe_ver4_path_reg_data           *reg_data;
	struct cam_vfe_top_ver4_priv                *top_priv;

	cam_hw_mgr_event_cb_func             event_cb;
	void                                *priv;
	int                                  irq_err_handle;
	int                                  frame_irq_handle;
	void                                *vfe_irq_controller;
	struct cam_vfe_top_irq_evt_payload   evt_payload[CAM_VFE_CAMIF_EVT_MAX];
	struct list_head                     free_payload_list;
	spinlock_t                           spin_lock;

	enum cam_isp_hw_sync_mode          sync_mode;
	uint32_t                           dsp_mode;
	uint32_t                           pix_pattern;
	uint32_t                           first_pixel;
	uint32_t                           first_line;
	uint32_t                           last_pixel;
	uint32_t                           last_line;
	uint32_t                           hbi_value;
	uint32_t                           vbi_value;
	uint32_t                           irq_debug_cnt;
	uint32_t                           camif_debug;
	uint32_t                           horizontal_bin;
	uint32_t                           qcfa_bin;
	uint32_t                           dual_hw_idx;
	uint32_t                           is_dual;
	uint32_t                           epoch_factor;
	struct timespec64                  sof_ts;
	struct timespec64                  epoch_ts;
	struct timespec64                  eof_ts;
	struct timespec64                  error_ts;
	enum cam_vfe_top_ver4_fsm_state    fsm_state;
	uint32_t                           n_frame_irqs;
	bool                               is_fe_enabled;
	bool                               is_offline;
	bool                               is_lite;
	bool                               is_pixel_path;
	bool                               sfe_binned_epoch_cfg;
	bool                               enable_sof_irq_debug;
	bool                               handle_camif_irq;
	uint32_t                           hw_ctxt_mask;
};

static inline int cam_vfe_top_ver4_get_hw_ctxt_from_irq_status(
	struct cam_vfe_mux_ver4_data *vfe_priv,
	uint32_t irq_status)
{
	int i;

	for (i = 0; i < CAM_ISP_MULTI_CTXT_MAX; i++)
		if (irq_status & vfe_priv->reg_data->frm_irq_hw_ctxt_mask[i])
			break;

	if (i < CAM_ISP_MULTI_CTXT_MAX)
		return i;

	return -1;
}

static int cam_vfe_top_ver4_get_path_port_map(struct cam_vfe_top_ver4_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_isp_hw_path_port_map *arg = cmd_args;
	struct cam_vfe_top_ver4_hw_info *hw_info = top_priv->common_data.hw_info;
	int i;

	for (i = 0; i < hw_info->num_path_port_map; i++) {
		arg->entry[i][0] = hw_info->path_port_map[i][0];
		arg->entry[i][1] = hw_info->path_port_map[i][1];
	}
	arg->num_entries = hw_info->num_path_port_map;

	return 0;
}

static int cam_vfe_top_ver4_pdaf_lcr_config(struct cam_vfe_top_ver4_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_vfe_top_ver4_hw_info  *hw_info;
	struct cam_isp_hw_get_cmd_update *cdm_args = NULL;
	struct cam_cdm_utils_ops         *cdm_util_ops = NULL;
	uint32_t                          i;
	uint32_t                          reg_val_idx = 0;
	uint32_t                          num_reg_vals;
	uint32_t                          reg_val_pair[4];
	struct cam_isp_lcr_rdi_cfg_args  *cfg_args;
	size_t                            size;

	if (!cmd_args || !top_priv) {
		CAM_ERR(CAM_ISP, "Error, Invalid args");
		return -EINVAL;
	}

	cdm_args = (struct cam_isp_hw_get_cmd_update *)cmd_args;
	if (!cdm_args->res) {
		CAM_ERR(CAM_ISP, "VFE:%u Error, Invalid res", top_priv->top_common.hw_idx);
		return -EINVAL;
	}

	hw_info = top_priv->common_data.hw_info;
	if (!hw_info->num_pdaf_lcr_res || !hw_info->pdaf_lcr_res_mask) {
		CAM_DBG(CAM_ISP, "VFE:%u PDAF LCR is not supported", top_priv->top_common.hw_idx);
		return 0;
	}

	cfg_args = (struct cam_isp_lcr_rdi_cfg_args *)cdm_args->data;
	cdm_util_ops =
		(struct cam_cdm_utils_ops *)cdm_args->res->cdm_ops;
	if (!cdm_util_ops) {
		CAM_ERR(CAM_ISP, "VFE:%u Invalid CDM ops", top_priv->top_common.hw_idx);
		return -EINVAL;
	}

	for (i = 0; i < hw_info->num_pdaf_lcr_res; i++)
		if (cfg_args->ife_src_res_id == hw_info->pdaf_lcr_res_mask[i].res_id)
			break;

	if (i == hw_info->num_pdaf_lcr_res) {
		CAM_ERR(CAM_ISP, "VFE:%u Res :%d src_res :%u is not supported for mux",
			top_priv->top_common.hw_idx, cfg_args->rdi_lcr_cfg->res_id,
			cfg_args->ife_src_res_id);
		return -EINVAL;
	}

	if (cfg_args->is_init)
		num_reg_vals = 2;
	else
		num_reg_vals = 1;

	size = cdm_util_ops->cdm_required_size_reg_random(num_reg_vals);
	/* since cdm returns dwords, we need to convert it into bytes */
	if ((size * 4) > cdm_args->cmd.size) {
		CAM_ERR(CAM_ISP, "VFE:%u buf size:%d is not sufficient, expected: %d",
			top_priv->top_common.hw_idx, cdm_args->cmd.size, (size*4));
		return -EINVAL;
	}

	if (cfg_args->is_init) {
		reg_val_pair[reg_val_idx++] = hw_info->common_reg->pdaf_input_cfg_1;
		reg_val_pair[reg_val_idx++] = 0;
	}

	reg_val_pair[reg_val_idx++] = hw_info->common_reg->pdaf_input_cfg_0;
	reg_val_pair[reg_val_idx] = hw_info->pdaf_lcr_res_mask[i].val;
	cdm_util_ops->cdm_write_regrandom(cdm_args->cmd.cmd_buf_addr,
		num_reg_vals, reg_val_pair);
	cdm_args->cmd.used_bytes = size * 4;

	return 0;
}


int cam_vfe_top_acquire_resource_update_res(
	struct cam_isp_resource_node  *vfe_full_res,
	void                          *acquire_param)
{
	struct cam_vfe_mux_ver4_data      *res_data;
	struct cam_vfe_acquire_args       *acquire_data;

	res_data  = (struct cam_vfe_mux_ver4_data *)
		vfe_full_res->res_priv;
	acquire_data = (struct cam_vfe_acquire_args *)acquire_param;

	res_data->sync_mode      = acquire_data->vfe_in.sync_mode;
	res_data->event_cb       = acquire_data->event_cb;
	res_data->priv           = acquire_data->priv;

	if (!res_data->is_pixel_path)
		goto config_done;

	res_data->pix_pattern    = acquire_data->vfe_in.in_port->test_pattern;
	res_data->dsp_mode       = acquire_data->vfe_in.in_port->dsp_mode;
	res_data->first_pixel    = acquire_data->vfe_in.in_port->left_start;
	res_data->last_pixel     = acquire_data->vfe_in.in_port->left_stop;
	res_data->first_line     = acquire_data->vfe_in.in_port->line_start;
	res_data->last_line      = acquire_data->vfe_in.in_port->line_stop;
	res_data->is_fe_enabled  = acquire_data->vfe_in.is_fe_enabled;
	res_data->is_offline     = acquire_data->vfe_in.is_offline;
	res_data->is_dual        = acquire_data->vfe_in.is_dual;
	res_data->qcfa_bin       = acquire_data->vfe_in.in_port->qcfa_bin;
	res_data->horizontal_bin =
		acquire_data->vfe_in.in_port->horizontal_bin;
	res_data->sfe_binned_epoch_cfg =
		acquire_data->vfe_in.in_port->sfe_binned_epoch_cfg;
	res_data->handle_camif_irq	 = acquire_data->vfe_in.handle_camif_irq;

	if (res_data->is_dual)
		res_data->dual_hw_idx = acquire_data->vfe_in.dual_hw_idx;

config_done:
	CAM_DBG(CAM_ISP,
		"VFE:%d Res:[id:%d name:%s] dsp_mode:%d is_dual:%d dual_hw_idx:%d handle_camif_irq:%d",
		vfe_full_res->hw_intf->hw_idx,
		vfe_full_res->res_id,
		vfe_full_res->res_name,
		res_data->dsp_mode,
		res_data->is_dual, res_data->dual_hw_idx,
		res_data->handle_camif_irq);

	return 0;
}

static int cam_vfe_top_ver4_update_res(struct cam_vfe_top_ver4_priv *top_priv,
	void *reserve_args, uint32_t arg_size)
{
	struct cam_vfe_acquire_args             *args;
	struct cam_vfe_resource_update          *res_update_args;
	struct cam_vfe_hw_vfe_in_acquire_args   *acquire_args;
	uint32_t i;
	int rc = -EINVAL;

	if (!top_priv || !reserve_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	res_update_args = (struct cam_vfe_resource_update *)reserve_args;
	args = (struct cam_vfe_acquire_args *)res_update_args->vfe_acquire;
	acquire_args = &args->vfe_in;

	for (i = 0; i < top_priv->top_common.num_mux; i++) {
		if (top_priv->top_common.mux_rsrc[i].res_id ==
			acquire_args->res_id) {

			if (acquire_args->res_id == CAM_ISP_HW_VFE_IN_CAMIF) {
				rc = cam_vfe_top_acquire_resource_update_res(
					&top_priv->top_common.mux_rsrc[i],
					args);
				if (rc)
					break;
			}

			if (acquire_args->res_id >= CAM_ISP_HW_VFE_IN_RDI0 &&
				acquire_args->res_id < CAM_ISP_HW_VFE_IN_MAX) {
				rc = cam_vfe_top_acquire_resource_update_res(
					&top_priv->top_common.mux_rsrc[i],
					args);
				if (rc)
					break;
			}

			top_priv->top_common.mux_rsrc[i].tasklet_info =
				args->tasklet;
			top_priv->top_common.mux_rsrc[i].is_per_port_acquire =
				false;

			acquire_args->rsrc_node =
				&top_priv->top_common.mux_rsrc[i];

			rc = 0;
			break;
		}
	}
	return rc;
}

static int cam_vfe_ver4_err_irq_top_half(
	uint32_t                               evt_id,
	struct cam_irq_th_payload             *th_payload);

static int cam_vfe_top_ver4_enable_irq(
	struct cam_vfe_top_ver4_priv *top_priv,
	void *res_irq_mask)
{
	struct cam_isp_resource_node   *vfe_res;
	struct cam_vfe_mux_ver4_data   *rsrc_data;
	int                             i, rc = 0;
	struct cam_vfe_res_irq_info    *irq_args;
	uint32_t                        err_irq_mask[CAM_IFE_IRQ_REGISTERS_MAX];

	if (!res_irq_mask) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	irq_args = (struct cam_vfe_res_irq_info *)res_irq_mask;
	for (i = 0; i < irq_args->num_res; i++) {
		vfe_res = irq_args->node_res[i];

		if (vfe_res->res_state != CAM_ISP_RESOURCE_STATE_STREAMING) {
			/* possible reason can be irqs are already disabled */
			CAM_DBG(CAM_ISP, "Error, Invalid camif res res_state:%d",
				vfe_res->res_state);
			return 0;
		}

		rsrc_data = (struct cam_vfe_mux_ver4_data *)vfe_res->res_priv;

		/* Perport works with csid only.
		 * No need to subscribe frame irq at ife side, it is already taken care from csid
		 */
		err_irq_mask[CAM_IFE_IRQ_CAMIF_REG_STATUS0] = rsrc_data->reg_data->error_irq_mask;

		if (!vfe_res->is_per_port_acquire && !rsrc_data->irq_err_handle &&
			!vfe_res->is_per_port_start) {
			rsrc_data->irq_err_handle = cam_irq_controller_subscribe_irq(
				rsrc_data->vfe_irq_controller,
				CAM_IRQ_PRIORITY_0,
				err_irq_mask,
				vfe_res,
				cam_vfe_ver4_err_irq_top_half,
				vfe_res->bottom_half_handler,
				vfe_res->tasklet_info,
				&tasklet_bh_api,
				CAM_IRQ_EVT_GROUP_0);

			if (rsrc_data->irq_err_handle < 1) {
				CAM_ERR(CAM_ISP, "VFE:%u Error IRQ handle subscribe failure",
					vfe_res->hw_intf->hw_idx);
				rc = -ENOMEM;
				rsrc_data->irq_err_handle = 0;
			}
		} else {
			if (rsrc_data->irq_err_handle) {
				rc = cam_irq_controller_update_irq(
					rsrc_data->vfe_irq_controller,
					rsrc_data->irq_err_handle,
					irq_args->enable_irq,
					err_irq_mask);

				if (rc) {
					CAM_ERR(CAM_ISP, "Error IRQ handle update failure");
					rc = -ENOMEM;
				}
			} else {
				CAM_ERR(CAM_ISP, "Frame IRQ Err handle not found");
			}
		}

		CAM_DBG(CAM_ISP, "VFE:%d Res: %s irq enable update done err_irq_mask:0x%x",
			vfe_res->hw_intf->hw_idx,
			vfe_res->res_name, err_irq_mask[CAM_IFE_IRQ_CAMIF_REG_STATUS0]);
		}

	return rc;
}

static int cam_vfe_top_ver4_mux_get_base(struct cam_vfe_top_ver4_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	uint32_t                          size = 0;
	uint32_t                          mem_base = 0;
	struct cam_isp_hw_get_cmd_update *cdm_args  = cmd_args;
	struct cam_cdm_utils_ops         *cdm_util_ops = NULL;
	struct cam_vfe_soc_private       *soc_private;

	if (arg_size != sizeof(struct cam_isp_hw_get_cmd_update)) {
		CAM_ERR(CAM_ISP, "Error, Invalid cmd size");
		return -EINVAL;
	}

	if (!cdm_args || !cdm_args->res || !top_priv ||
		!top_priv->top_common.soc_info) {
		CAM_ERR(CAM_ISP, "Error, Invalid args");
		return -EINVAL;
	}

	soc_private = top_priv->top_common.soc_info->soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_ISP, "VFE:%u soc_private is null", top_priv->top_common.hw_idx);
		return -EINVAL;
	}

	cdm_util_ops =
		(struct cam_cdm_utils_ops *)cdm_args->res->cdm_ops;

	if (!cdm_util_ops) {
		CAM_ERR(CAM_ISP, "VFE:%u Invalid CDM ops", top_priv->top_common.hw_idx);
		return -EINVAL;
	}

	size = cdm_util_ops->cdm_required_size_changebase();
	/* since cdm returns dwords, we need to convert it into bytes */
	if ((size * 4) > cdm_args->cmd.size) {
		CAM_ERR(CAM_ISP, "VFE:%u buf size:%d is not sufficient, expected: %d",
			top_priv->top_common.hw_idx, cdm_args->cmd.size, size);
		return -EINVAL;
	}

	mem_base = CAM_SOC_GET_REG_MAP_CAM_BASE(
		top_priv->top_common.soc_info, VFE_CORE_BASE_IDX);
	if (mem_base == -1) {
		CAM_ERR(CAM_ISP, "failed to get mem_base, index: %d num_reg_map: %u",
			VFE_CORE_BASE_IDX, top_priv->top_common.soc_info->num_reg_map);
		return -EINVAL;
	}

	if (cdm_args->cdm_id == CAM_CDM_RT) {
		if (!soc_private->rt_wrapper_base) {
			CAM_ERR(CAM_ISP, "VFE:%u rt_wrapper_base_addr is null",
				top_priv->top_common.hw_idx);
			return -EINVAL;
		}

		mem_base -= soc_private->rt_wrapper_base;
	}

	CAM_DBG(CAM_ISP, "core %u mem_base 0x%x, cdm_id: %u",
		top_priv->top_common.soc_info->index, mem_base,
		cdm_args->cdm_id);

	cdm_util_ops->cdm_write_changebase(cdm_args->cmd.cmd_buf_addr, mem_base);
	cdm_args->cmd.used_bytes = (size * 4);

	return 0;
}

static int cam_vfe_top_fs_update(
	struct cam_vfe_top_ver4_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_vfe_fe_update_args *cmd_update = cmd_args;

	if (cmd_update->node_res->process_cmd)
		return cmd_update->node_res->process_cmd(cmd_update->node_res,
			CAM_ISP_HW_CMD_FE_UPDATE_IN_RD, cmd_args, arg_size);

	return 0;
}

static int cam_vfe_top_ver4_set_primary_sof_timer_reg_addr(
	struct cam_vfe_top_ver4_priv *top_priv, void *cmd_args)
{
	struct cam_ife_csid_ts_reg_addr *sof_ts_addr_update_args;

	if (!cmd_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid args");
		return -EINVAL;
	}

	sof_ts_addr_update_args = (struct cam_ife_csid_ts_reg_addr *)cmd_args;

	if (!sof_ts_addr_update_args->curr0_ts_addr ||
		!sof_ts_addr_update_args->curr1_ts_addr) {
		CAM_ERR(CAM_ISP, "Invalid SOF Qtimer address: curr0: 0x%pK, curr1: 0x%pK",
		sof_ts_addr_update_args->curr0_ts_addr,
		sof_ts_addr_update_args->curr1_ts_addr);
		return -EINVAL;
	}

	top_priv->sof_ts_reg_addr.curr0_ts_addr =
		sof_ts_addr_update_args->curr0_ts_addr;
	top_priv->sof_ts_reg_addr.curr1_ts_addr =
		sof_ts_addr_update_args->curr1_ts_addr;

	return 0;
}

static void cam_vfe_top_ver4_check_module_status(
	uint32_t num_reg, uint32_t *reg_val,
	struct cam_vfe_top_ver4_debug_reg_info (*status_list)[][8])
{
	bool found = false;
	uint32_t i, j, val = 0;
	size_t len = 0;
	uint8_t log_buf[1024];

	if (!status_list)
		return;

	for (i = 0; i < num_reg; i++) {
		/* Check for ideal values */
		if ((reg_val[i] == 0) || (reg_val[i] == 0x55555555))
			continue;

		for (j = 0; j < 8; j++) {
			val = reg_val[i] >> (*status_list)[i][j].shift;
			val &= 0xF;
			if (val == 0 || val == 5)
				continue;

			CAM_INFO_BUF(CAM_ISP, log_buf, 1024, &len, "%s [I:%u V:%u R:%u]",
				(*status_list)[i][j].clc_name,
				((val >> 2) & 1), ((val >> 1) & 1), (val & 1));
			found = true;
		}
		if (found)
			CAM_INFO_RATE_LIMIT(CAM_ISP, "Check config for Debug%u - %s", i, log_buf);
		len = 0;
		found = false;
		memset(log_buf, 0, sizeof(uint8_t)*1024);
	}
}

static void cam_vfe_top_ver4_print_pdaf_violation_info(
	struct cam_vfe_mux_ver4_data *vfe_priv)
{
	struct cam_vfe_top_ver4_priv        *top_priv;
	struct cam_hw_soc_info              *soc_info;
	struct cam_vfe_top_ver4_common_data *common_data;
	void __iomem                        *base;
	uint32_t                             val = 0;
	uint32_t                             i = 0;

	top_priv    =  vfe_priv->top_priv;
	common_data = &top_priv->common_data;
	soc_info    =  top_priv->top_common.soc_info;
	base        =  soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base;
	val         =  cam_io_r(base +
			    common_data->common_reg->pdaf_violation_status),

	CAM_INFO(CAM_ISP, "VFE[%u] PDAF HW Violation status 0x%x",
	     soc_info->index, val);

	for (i = 0; i < common_data->hw_info->num_pdaf_violation_errors; i++) {
		if (common_data->hw_info->pdaf_violation_desc[i].bitmask &
			val) {
			CAM_ERR(CAM_ISP, "%s",
				common_data->hw_info->pdaf_violation_desc[i].desc);

		}
	}
}

static void cam_vfe_top_ver4_print_ipp_violation_info(
	struct cam_vfe_top_ver4_priv *top_priv)
{
	struct cam_hw_soc_info              *soc_info;
	struct cam_vfe_top_ver4_common_data *common_data;
	void __iomem                        *base;
	uint32_t                             val = 0;

	common_data = &top_priv->common_data;
	soc_info    =  top_priv->top_common.soc_info;
	base        =  soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base;
	val         =  cam_io_r(base +
			    common_data->common_reg->ipp_violation_status),

	CAM_INFO(CAM_ISP, "VFE[%u] IPP Violation status 0x%x",
	     soc_info->index, val);

	if (common_data->hw_info->ipp_module_desc)
		CAM_ERR(CAM_ISP, "VFE[%u] IPP Violation Module id: [%u %s]",
			soc_info->index,
			common_data->hw_info->ipp_module_desc[val].id,
			common_data->hw_info->ipp_module_desc[val].desc);

}

static void cam_vfe_top_ver4_print_top_irq_error(
	struct cam_vfe_mux_ver4_data *vfe_priv,
	uint32_t irq_status)
{
	uint32_t                                    i = 0;
	struct cam_vfe_top_ver4_priv               *top_priv;
	struct cam_vfe_top_ver4_common_data        *common_data;

	top_priv    =  vfe_priv->top_priv;
	common_data = &top_priv->common_data;

	for (i = 0; i < common_data->hw_info->num_top_errors; i++) {
		if (common_data->hw_info->top_err_desc[i].bitmask &
			irq_status) {
			CAM_ERR(CAM_ISP, "%s %s",
				common_data->hw_info->top_err_desc[i].err_name,
				common_data->hw_info->top_err_desc[i].desc);

		}
	}

	if (irq_status & vfe_priv->reg_data->ipp_violation_mask)
		cam_vfe_top_ver4_print_ipp_violation_info(top_priv);

	if (irq_status & vfe_priv->reg_data->pdaf_violation_mask)
		cam_vfe_top_ver4_print_pdaf_violation_info(vfe_priv);
}

static void cam_vfe_top_dump_perf_counters(
	const char *event,
	const char *res_name,
	struct cam_vfe_top_ver4_priv *top_priv)
{
	int i;
	void __iomem                              *mem_base;
	struct cam_hw_soc_info                    *soc_info;
	struct cam_vfe_top_ver4_reg_offset_common *common_reg;

	soc_info = top_priv->top_common.soc_info;
	common_reg = top_priv->common_data.common_reg;
	mem_base = soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base;

	for (i = 0; i < top_priv->common_data.common_reg->num_perf_counters; i++) {
		if (top_priv->perf_counters[i].dump_counter) {
			CAM_INFO(CAM_ISP,
				"VFE [%u] on %s %s counter: %d pixel_cnt: %d line_cnt: %d stall_cnt: %d always_cnt: %d status: 0x%x",
				top_priv->common_data.hw_intf->hw_idx, res_name, event, (i + 1),
				cam_io_r_mb(mem_base +
					common_reg->perf_count_reg[i].perf_pix_count),
				cam_io_r_mb(mem_base +
					common_reg->perf_count_reg[i].perf_line_count),
				cam_io_r_mb(mem_base +
					common_reg->perf_count_reg[i].perf_stall_count),
				cam_io_r_mb(mem_base +
					common_reg->perf_count_reg[i].perf_always_count),
				cam_io_r_mb(mem_base +
					common_reg->perf_count_reg[i].perf_count_status));
		}
	}
}

static void cam_vfe_top_ver4_print_debug_reg_status(
	struct cam_vfe_top_ver4_priv *top_priv)
{
	struct cam_vfe_top_ver4_reg_offset_common  *common_reg;
	uint32_t                                    val = 0;
	uint32_t                                    num_reg =  0;
	uint32_t                                    i = 0, j;
	size_t                                      len = 0;
	uint8_t                                    *log_buf;
	uint32_t                                   reg_val[CAM_VFE_TOP_DBG_REG_MAX] = {0};
	struct cam_hw_soc_info                     *soc_info;
	void __iomem                               *base;

	soc_info   =  top_priv->top_common.soc_info;
	common_reg =  top_priv->common_data.common_reg;
	num_reg    =  common_reg->num_top_debug_reg;
	base       =  soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base;
	log_buf    =  top_priv->log_buf;

	while (i < num_reg) {
		for(j = 0; j < 4 && i < num_reg; j++, i++) {
			val = cam_io_r(base +
				common_reg->top_debug[i]);
			reg_val[i] = val;
			CAM_INFO_BUF(CAM_ISP, log_buf, CAM_VFE_LEN_LOG_BUF, &len,
				"VFE[%u] status %2d : 0x%08x", soc_info->index, i, val);
		}
		CAM_INFO(CAM_ISP, "VFE[%u]: Top Debug Status: %s", soc_info->index, log_buf);
		len = 0;
	}

	cam_vfe_top_ver4_check_module_status(num_reg, reg_val,
		top_priv->common_data.hw_info->debug_reg_info);
	cam_vfe_top_dump_perf_counters("ERROR", "", top_priv);
}

int cam_vfe_top_ver4_dump_timestamps(
	struct cam_vfe_top_ver4_priv *top_priv,
	int  res_id)
{
	uint32_t                           i;
	struct cam_vfe_mux_ver4_data      *vfe_priv = NULL;
	struct cam_isp_resource_node      *res = NULL;
	struct cam_isp_resource_node      *camif_res = NULL;
	struct timespec64                  ts;

	for (i = 0; i < top_priv->top_common.num_mux; i++) {

		res = &top_priv->top_common.mux_rsrc[i];

		if (!res || !res->res_priv) {
			CAM_ERR_RATE_LIMIT(CAM_ISP, "VFE[%u] Invalid Resource",
					top_priv->common_data.hw_intf->hw_idx);
			return -EINVAL;
		}

		vfe_priv  = res->res_priv;

		if (!vfe_priv->frame_irq_handle)
			continue;

		if (vfe_priv->is_pixel_path) {
			camif_res = res;
			if (res->res_id == res_id)
				break;
		} else {
			if (res->is_rdi_primary_res && res->res_id == res_id) {
				break;
			} else if (!res->is_rdi_primary_res && camif_res) {
				vfe_priv  = camif_res->res_priv;
				break;
			}
		}
	}

	if (i ==  top_priv->top_common.num_mux || !vfe_priv) {
		CAM_ERR_RATE_LIMIT(CAM_ISP, "VFE[%u] invalid res_id %d",
			top_priv->common_data.hw_intf->hw_idx, res_id);
		return 0;
	}

	ktime_get_boottime_ts64(&ts);

	CAM_INFO(CAM_ISP,
		"VFE[%u] res: %u current_ts: %lld:%lld epoch_factor: %u%%",
		vfe_priv->hw_intf->hw_idx, res_id, ts.tv_sec, ts.tv_nsec,
		vfe_priv->epoch_factor);

	CAM_INFO(CAM_ISP,
		"VFE[%u] CAMIF Error timestamp:[%lld.%09lld] SOF timestamp:[%lld.%09lld] EPOCH timestamp:[%lld.%09lld] EOF timestamp:[%lld.%09lld]",
		vfe_priv->hw_intf->hw_idx, vfe_priv->error_ts.tv_sec,
		vfe_priv->error_ts.tv_nsec,
		vfe_priv->sof_ts.tv_sec,
		vfe_priv->sof_ts.tv_nsec,
		vfe_priv->epoch_ts.tv_sec,
		vfe_priv->epoch_ts.tv_nsec,
		vfe_priv->eof_ts.tv_sec,
		vfe_priv->eof_ts.tv_nsec);

	return 0;
}

static int cam_vfe_top_ver4_print_overflow_debug_info(
	struct cam_vfe_top_ver4_priv *top_priv, void *cmd_args)
{
	struct cam_vfe_top_ver4_common_data *common_data;
	struct cam_hw_soc_info              *soc_info;
	struct cam_vfe_soc_private *soc_private = NULL;
	uint32_t                             violation_status = 0, bus_overflow_status = 0, tmp;
	uint32_t                             i = 0;
	int                                  res_id;
	struct cam_isp_hw_overflow_info     *overflow_info = NULL;

	overflow_info = (struct cam_isp_hw_overflow_info *)cmd_args;
	res_id = overflow_info->res_id;

	common_data = &top_priv->common_data;
	soc_info = top_priv->top_common.soc_info;
	soc_private = soc_info->soc_private;

	bus_overflow_status = cam_io_r(soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base +
		common_data->common_reg->bus_overflow_status);
	violation_status = cam_io_r(soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base +
		common_data->common_reg->bus_violation_status);

	if (soc_private->is_ife_lite)
		CAM_ERR(CAM_ISP,
			"VFE[%u] sof_cnt:%d src_clk:%lu overflow:%s violation:%s",
			soc_info->index, top_priv->sof_cnt,
			soc_info->applied_src_clk_rates.sw_client,
			CAM_BOOL_TO_YESNO(bus_overflow_status),
			CAM_BOOL_TO_YESNO(violation_status));
	else
		CAM_ERR(CAM_ISP,
			"VFE[%u] sof_cnt:%d src_clk sw_client:%lu hw_client:[%lu %lu] overflow:%s violation:%s",
			soc_info->index, top_priv->sof_cnt,
			soc_info->applied_src_clk_rates.sw_client,
			soc_info->applied_src_clk_rates.hw_client[soc_info->index].high,
			soc_info->applied_src_clk_rates.hw_client[soc_info->index].low,
			CAM_BOOL_TO_YESNO(bus_overflow_status),
			CAM_BOOL_TO_YESNO(violation_status));

	if (bus_overflow_status) {
		overflow_info->is_bus_overflow = true;
		CAM_INFO(CAM_ISP, "VFE[%u] Bus overflow status: 0x%x",
			soc_info->index, bus_overflow_status);
	}

	tmp = bus_overflow_status;
	while (tmp) {
		if (tmp & 0x1)
			CAM_ERR(CAM_ISP, "VFE[%u] Bus Overflow %s",
				soc_info->index, common_data->hw_info->wr_client_desc[i].desc);
		tmp = tmp >> 1;
		i++;
	}

	cam_vfe_top_ver4_dump_timestamps(top_priv, res_id);
	cam_cpas_dump_camnoc_buff_fill_info(soc_private->cpas_handle);
	if (bus_overflow_status)
		cam_cpas_log_votes(false);

	if (violation_status)
		CAM_INFO(CAM_ISP, "VFE[%u] Bus violation status: 0x%x",
			soc_info->index, violation_status);

	i = 0;
	tmp = violation_status;
	while (tmp) {
		if (tmp & 0x1)
			CAM_ERR(CAM_ISP, "VFE[%u] Bus Violation %s",
				soc_info->index, common_data->hw_info->wr_client_desc[i].desc);
		tmp = tmp >> 1;
		i++;
	}

	cam_vfe_top_ver4_print_debug_reg_status(top_priv);

	return 0;
}

static int cam_vfe_core_config_control(
	struct cam_vfe_top_ver4_priv *top_priv,
	 void *cmd_args, uint32_t arg_size)
{
	struct cam_vfe_core_config_args *vfe_core_cfg = cmd_args;
	struct cam_isp_resource_node *rsrc_node = vfe_core_cfg->node_res;
	struct cam_vfe_mux_ver4_data *vfe_priv = rsrc_node->res_priv;

	vfe_priv->cam_common_cfg.vid_ds16_r2pd =
		vfe_core_cfg->core_config.vid_ds16_r2pd;
	vfe_priv->cam_common_cfg.vid_ds4_r2pd =
		vfe_core_cfg->core_config.vid_ds4_r2pd;
	vfe_priv->cam_common_cfg.disp_ds16_r2pd =
		vfe_core_cfg->core_config.disp_ds16_r2pd;
	vfe_priv->cam_common_cfg.disp_ds4_r2pd =
		vfe_core_cfg->core_config.disp_ds4_r2pd;
	vfe_priv->cam_common_cfg.dsp_streaming_tap_point =
		vfe_core_cfg->core_config.dsp_streaming_tap_point;
	vfe_priv->cam_common_cfg.ihist_src_sel =
		vfe_core_cfg->core_config.ihist_src_sel;
	vfe_priv->cam_common_cfg.input_pp_fmt =
		vfe_core_cfg->core_config.core_cfg_flag
			& CAM_ISP_PARAM_CORE_CFG_PP_FORMAT;
	vfe_priv->cam_common_cfg.hdr_mux_sel_pp =
		vfe_core_cfg->core_config.core_cfg_flag
			& CAM_ISP_PARAM_CORE_CFG_HDR_MUX_SEL;

	return 0;
}

static int cam_vfe_init_config_update(
	void *cmd_args, uint32_t arg_size)
{
	struct cam_isp_hw_init_config_update *init_cfg = cmd_args;
	struct cam_isp_resource_node *rsrc_node = init_cfg->node_res;
	struct cam_vfe_mux_ver4_data *mux_data = rsrc_node->res_priv;

	if (arg_size != sizeof(struct cam_isp_hw_init_config_update)) {
		CAM_ERR(CAM_ISP, "VFE:%u Invalid args size expected: %zu actual: %zu",
			rsrc_node->hw_intf->hw_idx, sizeof(struct cam_isp_hw_init_config_update),
			arg_size);
		return -EINVAL;
	}

	mux_data->epoch_factor =
		init_cfg->init_config->epoch_cfg.epoch_factor;

	CAM_DBG(CAM_ISP,
		"VFE:%u Init Update for res_name: %s epoch_factor: %u%%",
		rsrc_node->hw_intf->hw_idx, rsrc_node->res_name, mux_data->epoch_factor);

	return 0;
}

static int cam_vfe_top_ver4_mux_get_reg_update(
	struct cam_vfe_top_ver4_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	CAM_ERR(CAM_ISP, "VFE:%u Invalid request, Add RUP in CSID",
		top_priv->common_data.hw_intf->hw_idx);
	return -EINVAL;
}

static int cam_vfe_top_ver4_get_data(
	struct cam_vfe_top_ver4_priv *top_priv,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_isp_resource_node  *res = cmd_args;

	if (res->process_cmd)
		return res->process_cmd(res,
			CAM_ISP_HW_CMD_CAMIF_DATA, cmd_args, arg_size);

	return -EINVAL;
}

int cam_vfe_top_ver4_get_hw_caps(void *device_priv, void *args, uint32_t arg_size)
{
	struct cam_vfe_hw_get_hw_cap *vfe_cap_info = NULL;
	struct cam_vfe_top_ver4_priv *vfe_top_prv = NULL;
	struct cam_vfe_soc_private *soc_priv = NULL;

	if (!device_priv || !args) {
		CAM_ERR(CAM_ISP, "Invalid arguments device_priv:%p, args:%p", device_priv, args);
		return -EINVAL;
	}

	vfe_cap_info = args;
	vfe_top_prv = device_priv;

	if (!vfe_top_prv->top_common.soc_info) {
		CAM_ERR(CAM_ISP, "soc info is null");
		return -EFAULT;
	}

	soc_priv = (struct cam_vfe_soc_private *)vfe_top_prv->top_common.soc_info->soc_private;

	vfe_cap_info->is_lite = soc_priv->is_ife_lite;
	vfe_cap_info->incr    = (vfe_top_prv->top_common.hw_version) & 0x00ffff;
	vfe_cap_info->minor   = ((vfe_top_prv->top_common.hw_version) >> 16) & 0x0fff;
	vfe_cap_info->major   = (vfe_top_prv->top_common.hw_version) >> 28;

	return 0;
}

int cam_vfe_top_ver4_init_hw(void *device_priv,
	void *init_hw_args, uint32_t arg_size)
{
	struct cam_vfe_top_ver4_priv   *top_priv = device_priv;
	struct cam_vfe_top_ver4_common_data common_data = top_priv->common_data;

	top_priv->top_common.applied_clk_rate = 0;

	top_priv->top_common.hw_version = cam_io_r_mb(
		top_priv->top_common.soc_info->reg_map[0].mem_base +
		common_data.common_reg->hw_version);
	CAM_DBG(CAM_ISP, "VFE:%u hw-version:0x%x",
		top_priv->top_common.hw_idx,
		top_priv->top_common.hw_version);

	return 0;
}

int cam_vfe_top_ver4_reset(void *device_priv,
	void *reset_core_args, uint32_t arg_size)
{
	CAM_DBG(CAM_ISP, "Reset not supported");
	return 0;
}

int cam_vfe_top_acquire_resource(
	struct cam_isp_resource_node  *vfe_full_res,
	void                          *acquire_param)
{
	struct cam_vfe_mux_ver4_data      *res_data;
	struct cam_vfe_acquire_args       *acquire_data;
	int                                    rc = 0;

	res_data  = (struct cam_vfe_mux_ver4_data *)
		vfe_full_res->res_priv;
	acquire_data = (struct cam_vfe_acquire_args *)acquire_param;

	res_data->sync_mode      = acquire_data->vfe_in.sync_mode;
	res_data->event_cb       = acquire_data->event_cb;
	res_data->priv           = acquire_data->priv;

	if (!res_data->is_pixel_path)
		goto config_done;

	res_data->pix_pattern    = acquire_data->vfe_in.in_port->test_pattern;
	res_data->dsp_mode       = acquire_data->vfe_in.in_port->dsp_mode;
	res_data->first_pixel    = acquire_data->vfe_in.in_port->left_start;
	res_data->last_pixel     = acquire_data->vfe_in.in_port->left_stop;
	res_data->first_line     = acquire_data->vfe_in.in_port->line_start;
	res_data->last_line      = acquire_data->vfe_in.in_port->line_stop;
	res_data->is_fe_enabled  = acquire_data->vfe_in.is_fe_enabled;
	res_data->is_offline     = acquire_data->vfe_in.is_offline;
	res_data->is_dual        = acquire_data->vfe_in.is_dual;
	res_data->qcfa_bin       = acquire_data->vfe_in.in_port->qcfa_bin;
	res_data->horizontal_bin =
		acquire_data->vfe_in.in_port->horizontal_bin;
	res_data->vbi_value      = 0;
	res_data->hbi_value      = 0;
	res_data->sfe_binned_epoch_cfg =
		acquire_data->vfe_in.in_port->sfe_binned_epoch_cfg;
	res_data->handle_camif_irq   = acquire_data->vfe_in.handle_camif_irq;

	if (res_data->is_dual)
		res_data->dual_hw_idx = acquire_data->vfe_in.dual_hw_idx;

config_done:
	CAM_DBG(CAM_ISP,
		"VFE:%u Res:[id:%d name:%s] dsp_mode:%d is_dual:%d dual_hw_idx:%d",
		vfe_full_res->hw_intf->hw_idx,
		vfe_full_res->res_id,
		vfe_full_res->res_name,
		res_data->dsp_mode,
		res_data->is_dual, res_data->dual_hw_idx);

	return rc;
}

int cam_vfe_top_ver4_reserve(void *device_priv,
	void *reserve_args, uint32_t arg_size)
{
	struct cam_vfe_top_ver4_priv            *top_priv;
	struct cam_vfe_acquire_args             *args;
	struct cam_vfe_hw_vfe_in_acquire_args   *acquire_args;
	struct cam_vfe_mux_ver4_data            *vfe_priv = NULL;
	uint32_t i;
	int rc = -EINVAL;

	if (!device_priv || !reserve_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	top_priv = (struct cam_vfe_top_ver4_priv   *)device_priv;
	args = (struct cam_vfe_acquire_args *)reserve_args;
	acquire_args = &args->vfe_in;

	CAM_DBG(CAM_ISP, "VFE:%u res id %d",
		top_priv->common_data.hw_intf->hw_idx, acquire_args->res_id);


	for (i = 0; i < top_priv->top_common.num_mux; i++) {
		if (top_priv->top_common.mux_rsrc[i].res_id == acquire_args->res_id) {
			vfe_priv = (struct cam_vfe_mux_ver4_data *)
				top_priv->top_common.mux_rsrc[i].res_priv;

			if (top_priv->top_common.mux_rsrc[i].res_state !=
				CAM_ISP_RESOURCE_STATE_AVAILABLE) {
				if (acquire_args->res_id != CAM_ISP_HW_VFE_IN_CAMIF) {
					CAM_ERR(CAM_ISP, "VFE:%d Duplicate acquire for camif",
						top_priv->common_data.hw_intf->hw_idx);
					rc = -EINVAL;
					break;
				}

				if (!vfe_priv->hw_ctxt_mask && !acquire_args->hw_ctxt_mask) {
					rc = -EINVAL;
					goto end;
				}

				if (!(vfe_priv->hw_ctxt_mask & acquire_args->hw_ctxt_mask)) {
					CAM_DBG(CAM_ISP,
						"VFE:%d Update hw ctxt mask: 0x%x for camif curr_mask_val: 0x%x",
						top_priv->common_data.hw_intf->hw_idx,
						acquire_args->hw_ctxt_mask,
						vfe_priv->hw_ctxt_mask);
					vfe_priv->hw_ctxt_mask |= acquire_args->hw_ctxt_mask;
					acquire_args->rsrc_node = &top_priv->top_common.mux_rsrc[i];
					rc = 0;
				} else {
					CAM_ERR(CAM_ISP,
						"VFE:%d Duplicate hw ctxt mask: 0x%x for camif curr_mask_val: 0x%x",
						top_priv->common_data.hw_intf->hw_idx,
						acquire_args->hw_ctxt_mask,
						vfe_priv->hw_ctxt_mask);
					rc = -EINVAL;
				}

				break;
			}

			if (((acquire_args->res_id == CAM_ISP_HW_VFE_IN_CAMIF) ||
				(acquire_args->res_id >= CAM_ISP_HW_VFE_IN_RDI0)) &&
				(acquire_args->res_id < CAM_ISP_HW_VFE_IN_MAX)) {
				rc = cam_vfe_top_acquire_resource(
					&top_priv->top_common.mux_rsrc[i],
					args);
				if (rc)
					break;
			}

			top_priv->top_common.mux_rsrc[i].cdm_ops =
				acquire_args->cdm_ops;
			top_priv->top_common.mux_rsrc[i].tasklet_info =
				args->tasklet;
			vfe_priv->hw_ctxt_mask = acquire_args->hw_ctxt_mask;
			top_priv->top_common.mux_rsrc[i].res_state =
				CAM_ISP_RESOURCE_STATE_RESERVED;
			if (acquire_args->in_port->per_port_en && args->per_port_acquire)
				top_priv->top_common.mux_rsrc[i].is_per_port_acquire = true;
			acquire_args->rsrc_node = &top_priv->top_common.mux_rsrc[i];

			rc = 0;
			break;
		}
	}
end:
	return rc;

}

int cam_vfe_top_ver4_release(void *device_priv,
	void *release_args, uint32_t arg_size)
{
	struct cam_isp_resource_node            *mux_res;
	struct cam_vfe_top_ver4_priv            *top_priv;
	struct cam_vfe_mux_ver4_data            *vfe_priv = NULL;

	if (!device_priv || !release_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	mux_res = (struct cam_isp_resource_node *)release_args;
	top_priv = (struct cam_vfe_top_ver4_priv   *)device_priv;
	vfe_priv = (struct cam_vfe_mux_ver4_data *) mux_res->res_priv;

	CAM_DBG(CAM_ISP, "VFE:%u Resource in state %d",
		top_priv->common_data.hw_intf->hw_idx, mux_res->res_state);
	if (mux_res->res_state < CAM_ISP_RESOURCE_STATE_RESERVED) {
		CAM_ERR(CAM_ISP, "VFE:%u Error, Resource in Invalid res_state :%d",
			top_priv->common_data.hw_intf->hw_idx, mux_res->res_state);
		return -EINVAL;
	}

	memset(&vfe_priv->top_priv->sof_ts_reg_addr, 0,
		sizeof(vfe_priv->top_priv->sof_ts_reg_addr));
	vfe_priv->hw_ctxt_mask = 0;
	mux_res->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;

	return 0;
}

int cam_vfe_top_ver4_start(void *device_priv,
	void *start_args, uint32_t arg_size)
{
	struct cam_vfe_top_ver4_priv     *top_priv;
	struct cam_isp_resource_node     *mux_res;
	struct cam_hw_info               *hw_info = NULL;
	struct cam_hw_soc_info           *soc_info = NULL;
	struct cam_vfe_soc_private       *soc_private = NULL;
	int rc = 0, i;

	if (!device_priv || !start_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	top_priv = (struct cam_vfe_top_ver4_priv *)device_priv;
	soc_info = top_priv->top_common.soc_info;
	soc_private = soc_info->soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_ISP, "VFE:%u Error soc_private NULL",
			top_priv->common_data.hw_intf->hw_idx);
		return -EINVAL;
	}

	mux_res = (struct cam_isp_resource_node *)start_args;
	hw_info = (struct cam_hw_info *)mux_res->hw_intf->hw_priv;

	if (hw_info->hw_state == CAM_HW_STATE_POWER_UP) {
		rc = cam_vfe_top_apply_clock_start_stop(&top_priv->top_common);
		if (rc) {
			CAM_ERR(CAM_ISP,
				"VFE:%u Failed in applying start clock rc:%d",
				hw_info->soc_info.index, rc);
			return rc;
		}

		rc = cam_vfe_top_apply_bw_start_stop(&top_priv->top_common);
		if (rc) {
			CAM_ERR(CAM_ISP,
				"VFE:%u Failed in applying start bw rc:%d",
				hw_info->soc_info.index, rc);
			return rc;
		}

		if (mux_res->start) {
			rc = mux_res->start(mux_res);
		} else {
			CAM_ERR(CAM_ISP,
				"VFE:%u Invalid res id:%d",
				hw_info->soc_info.index, mux_res->res_id);
			rc = -EINVAL;
		}

		/* Perf counter config */
		for (i = 0; i < top_priv->common_data.common_reg->num_perf_counters; i++) {
			if (!top_priv->perf_counters[i].perf_counter_val)
				continue;

			top_priv->perf_counters[i].dump_counter = true;
			cam_io_w_mb(top_priv->perf_counters[i].perf_counter_val,
				soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base +
				top_priv->common_data.common_reg->perf_count_reg[i].perf_count_cfg);
			CAM_DBG(CAM_ISP, "VFE [%u] perf_count_%d: 0x%x",
				hw_info->soc_info.index, (i + 1),
				top_priv->perf_counters[i].perf_counter_val);
		}
	} else {
		CAM_ERR(CAM_ISP, "VFE:%u HW not powered up", hw_info->soc_info.index);
		rc = -EPERM;
	}

	atomic_set(&top_priv->overflow_pending, 0);
	return rc;
}

int cam_vfe_top_ver4_stop(void *device_priv,
	void *stop_args, uint32_t arg_size)
{
	struct cam_vfe_top_ver4_priv            *top_priv;
	struct cam_isp_resource_node            *mux_res;
	struct cam_hw_soc_info                  *soc_info = NULL;
	int i, rc = 0;

	if (!device_priv || !stop_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	top_priv = (struct cam_vfe_top_ver4_priv   *)device_priv;
	soc_info = top_priv->top_common.soc_info;
	mux_res = (struct cam_isp_resource_node *)stop_args;

	if (mux_res->res_id < CAM_ISP_HW_VFE_IN_MAX) {
		rc = mux_res->stop(mux_res);
	} else {
		CAM_ERR(CAM_ISP, "VFE:%u Invalid res id:%d",
			top_priv->common_data.hw_intf->hw_idx, mux_res->res_id);
		return -EINVAL;
	}

	if (!rc) {
		for (i = 0; i < top_priv->top_common.num_mux; i++) {
			if (top_priv->top_common.mux_rsrc[i].res_id ==
				mux_res->res_id) {
				if (!top_priv->top_common.skip_data_rst_on_stop)
					top_priv->top_common.req_clk_rate[i] = 0;
				memset(&top_priv->top_common.req_axi_vote[i],
					0, sizeof(struct cam_axi_vote));
				top_priv->top_common.axi_vote_control[i] =
					CAM_ISP_BW_CONTROL_EXCLUDE;
				break;
			}
		}
	}

	/* Reset perf counters at stream off */
	for (i = 0; i < top_priv->common_data.common_reg->num_perf_counters; i++) {
		if (top_priv->perf_counters[i].dump_counter)
			cam_io_w_mb(0x0,
				soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base +
				top_priv->common_data.common_reg->perf_count_reg[i].perf_count_cfg);
		top_priv->perf_counters[i].dump_counter = false;
	}

	if (top_priv->common_data.hw_info->num_pdaf_lcr_res)
		cam_io_w(1, soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base +
			top_priv->common_data.common_reg->pdaf_input_cfg_1);

	atomic_set(&top_priv->overflow_pending, 0);
	return rc;
}

int cam_vfe_top_ver4_read(void *device_priv,
	void *read_args, uint32_t arg_size)
{
	return -EPERM;
}

int cam_vfe_top_ver4_write(void *device_priv,
	void *write_args, uint32_t arg_size)
{
	return -EPERM;
}

static int cam_vfe_top_apply_fcg_update(
	struct cam_vfe_top_ver4_priv    *top_priv,
	struct cam_isp_hw_fcg_update    *fcg_update,
	struct cam_cdm_utils_ops        *cdm_util_ops)
{
	struct cam_isp_fcg_config_internal             *fcg_config;
	struct cam_isp_ch_ctx_fcg_config_internal      *fcg_ch_ctx;
	struct cam_isp_predict_fcg_config_internal     *fcg_pr;
	struct cam_vfe_top_ver4_hw_info                *hw_info;
	struct cam_vfe_ver4_fcg_module_info            *fcg_module_info;
	uint32_t                                        size, fcg_index_shift;
	uint32_t                                       *reg_val_pair;
	uint32_t                                        num_regval_pairs = 0;
	int                                             rc = 0, i, j = 0;

	if (!top_priv || !fcg_update || (fcg_update->prediction_idx == 0)) {
		CAM_ERR(CAM_ISP, "Invalid args");
		return -EINVAL;
	}

	hw_info = top_priv->common_data.hw_info;
	fcg_config = (struct cam_isp_fcg_config_internal *)fcg_update->data;
	if (!hw_info || !fcg_config) {
		CAM_ERR(CAM_ISP, "Invalid config params");
		return -EINVAL;
	}

	fcg_module_info = hw_info->fcg_module_info;
	if (!fcg_module_info) {
		CAM_ERR(CAM_ISP, "Invalid FCG common data");
		return -EINVAL;
	}

	if (fcg_config->num_ch_ctx > CAM_ISP_MAX_FCG_CH_CTXS) {
		CAM_ERR(CAM_SFE, "out of bounds %d",
			fcg_config->num_ch_ctx);
		return -EINVAL;
	}

	reg_val_pair = kcalloc(fcg_module_info->max_reg_val_pair_size, sizeof(uint32_t),
		GFP_KERNEL);
	if (!reg_val_pair) {
		CAM_ERR(CAM_ISP, "Failed allocating memory for reg val pair");
		return -ENOMEM;
	}

	fcg_index_shift = fcg_module_info->fcg_index_shift;

	for (i = 0, j = 0; i < fcg_config->num_ch_ctx; i++) {
		if (j >= fcg_module_info->max_reg_val_pair_size) {
			CAM_ERR(CAM_ISP, "reg_val_pair %d exceeds the array limit %u",
				j, fcg_module_info->max_reg_val_pair_size);
			rc = -ENOMEM;
			goto free_mem;
		}

		fcg_ch_ctx = &fcg_config->ch_ctx_fcg_configs[i];
		if (!fcg_ch_ctx) {
			CAM_ERR(CAM_ISP, "Failed in FCG channel/context dereference");
			rc = -EINVAL;
			goto free_mem;
		}

		fcg_pr = &fcg_ch_ctx->predicted_fcg_configs[
			fcg_update->prediction_idx - 1];

		/* For VFE/MC_TFE, only PHASE should be enabled */
		if (fcg_ch_ctx->fcg_enable_mask & CAM_ISP_FCG_ENABLE_PHASE) {
			switch (fcg_ch_ctx->fcg_ch_ctx_id) {
			/* Same value as CAM_ISP_FCG_MASK_CH0/1/2 to support both VFE and MC_TFE */
			case CAM_ISP_MULTI_CTXT0_MASK:
				if (hw_info->fcg_mc_supported) {
					CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
						fcg_module_info->fcg_reg_ctxt_sel,
						(fcg_module_info->fcg_reg_ctxt_mask &
						(fcg_ch_ctx->fcg_ch_ctx_id <<
						fcg_module_info->fcg_reg_ctxt_shift)));
					CAM_DBG(CAM_ISP,
						"Program FCG registers for MC_TFE, ch_ctx_id: 0x%x, sel_wr: 0x%x",
						fcg_ch_ctx->fcg_ch_ctx_id,
						(fcg_module_info->fcg_reg_ctxt_mask &
						(fcg_ch_ctx->fcg_ch_ctx_id <<
						fcg_module_info->fcg_reg_ctxt_shift)));
				}
				CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
					fcg_module_info->fcg_phase_index_cfg_0,
					(fcg_pr->phase_index_r |
					(fcg_pr->phase_index_g << fcg_index_shift)));
				CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
					fcg_module_info->fcg_phase_index_cfg_1,
					fcg_pr->phase_index_b);
				CAM_DBG(CAM_ISP,
					"Program FCG registers for IFE/MC_TFE, ch_ctx_id: 0x%x, phase_index_cfg_0: %u, phase_index_cfg_1: %u",
					fcg_ch_ctx->fcg_ch_ctx_id,
					(fcg_pr->phase_index_r |
					(fcg_pr->phase_index_g << fcg_index_shift)),
					fcg_pr->phase_index_b);
				break;
			case CAM_ISP_MULTI_CTXT1_MASK:
			case CAM_ISP_MULTI_CTXT2_MASK:
				if (!hw_info->fcg_mc_supported) {
					CAM_ERR(CAM_ISP,
						"No support for multi context for FCG on ch_ctx_id: 0x%x",
						fcg_ch_ctx->fcg_ch_ctx_id);
					rc = -EINVAL;
					goto free_mem;
				}

				CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
					fcg_module_info->fcg_reg_ctxt_sel,
					(fcg_module_info->fcg_reg_ctxt_mask &
					(fcg_ch_ctx->fcg_ch_ctx_id <<
					fcg_module_info->fcg_reg_ctxt_shift)));
				CAM_DBG(CAM_ISP,
					"Program FCG registers for MC_TFE, ch_ctx_id: 0x%x, sel_wr: 0x%x",
					fcg_ch_ctx->fcg_ch_ctx_id,
					(fcg_module_info->fcg_reg_ctxt_mask &
					(fcg_ch_ctx->fcg_ch_ctx_id <<
					fcg_module_info->fcg_reg_ctxt_shift)));

				CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
					fcg_module_info->fcg_phase_index_cfg_0,
					(fcg_pr->phase_index_r |
					(fcg_pr->phase_index_g << fcg_index_shift)));
				CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
					fcg_module_info->fcg_phase_index_cfg_1,
					fcg_pr->phase_index_b);
				CAM_DBG(CAM_ISP,
					"Program FCG registers for MC_TFE, ch_ctx_id: 0x%x, phase_index_cfg_0: %u, phase_index_cfg_1: %u",
					fcg_ch_ctx->fcg_ch_ctx_id,
					(fcg_pr->phase_index_r |
					(fcg_pr->phase_index_g << fcg_index_shift)),
					fcg_pr->phase_index_b);
				break;
			default:
				CAM_ERR(CAM_ISP, "Unsupported ch_ctx_id: 0x%x",
					fcg_ch_ctx->fcg_ch_ctx_id);
				rc = -EINVAL;
				goto free_mem;
			}
		}
	}

	num_regval_pairs = j / 2;

	if (num_regval_pairs) {
		size = cdm_util_ops->cdm_required_size_reg_random(
			num_regval_pairs);

		if ((size * 4) != fcg_update->cmd_size) {
			CAM_ERR(CAM_ISP,
				"Failed! Buf size:%d is wrong, expected size: %d",
				fcg_update->cmd_size, size * 4);
			rc = -ENOMEM;
			goto free_mem;
		}

		cdm_util_ops->cdm_write_regrandom(
			(uint32_t *)fcg_update->cmd_buf_addr,
			num_regval_pairs, reg_val_pair);
	} else {
		CAM_WARN(CAM_ISP, "No reg val pairs");
	}

free_mem:
	kfree(reg_val_pair);
	return rc;
}

static int cam_vfe_top_get_fcg_buf_size(
	struct cam_vfe_top_ver4_priv    *top_priv,
	struct cam_isp_hw_fcg_get_size  *fcg_get_size,
	struct cam_cdm_utils_ops        *cdm_util_ops)
{
	struct cam_vfe_top_ver4_hw_info                *hw_info;
	struct cam_vfe_ver4_fcg_module_info            *fcg_module_info;
	uint32_t                                        num_types, num_reg_val;

	if (!top_priv) {
		CAM_ERR(CAM_ISP, "Invalid args");
		return -EINVAL;
	}

	hw_info = top_priv->common_data.hw_info;
	if (!hw_info) {
		CAM_ERR(CAM_ISP, "Invalid config params");
		return -EINVAL;
	}

	if (!hw_info->fcg_supported &&
		!hw_info->fcg_mc_supported) {
		fcg_get_size->fcg_supported = false;
		CAM_DBG(CAM_ISP, "FCG is not supported by hardware");
		return 0;
	}

	fcg_module_info = hw_info->fcg_module_info;
	fcg_get_size->fcg_supported = true;
	num_types = fcg_get_size->num_types;
	if (num_types == 0) {
		CAM_ERR(CAM_ISP, "Number of types(STATS/PHASE) requested is empty");
		return -EINVAL;
	}

	num_reg_val = num_types * fcg_module_info->fcg_type_size;

	/* Count for wr_sel register in MC_TFE */
	if (hw_info->fcg_mc_supported)
		num_reg_val += fcg_get_size->num_ctxs;

	fcg_get_size->kmd_size =
		cdm_util_ops->cdm_required_size_reg_random(num_reg_val);
	return 0;
}

static int cam_vfe_top_fcg_config(
	struct cam_vfe_top_ver4_priv    *top_priv,
	void                            *cmd_args,
	uint32_t                         arg_size)
{
	struct cam_isp_hw_fcg_cmd       *fcg_cmd;
	struct cam_cdm_utils_ops        *cdm_util_ops;
	int rc;

	if (arg_size != sizeof(struct cam_isp_hw_fcg_cmd)) {
		CAM_ERR(CAM_ISP, "Invalid cmd size, arg_size: %u, expected size: %u",
			arg_size, sizeof(struct cam_isp_hw_fcg_cmd));
		return -EINVAL;
	}

	fcg_cmd = (struct cam_isp_hw_fcg_cmd *) cmd_args;
	if (!fcg_cmd || !fcg_cmd->res) {
		CAM_ERR(CAM_ISP, "Invalid cmd args");
		return -EINVAL;
	}

	cdm_util_ops =
		(struct cam_cdm_utils_ops *)fcg_cmd->res->cdm_ops;
	if (!cdm_util_ops) {
		CAM_ERR(CAM_ISP, "Invalid CDM ops");
		return -EINVAL;
	}

	if (fcg_cmd->get_size_flag) {
		struct cam_isp_hw_fcg_get_size  *fcg_get_size;

		fcg_get_size = &fcg_cmd->u.fcg_get_size;
		rc = cam_vfe_top_get_fcg_buf_size(top_priv, fcg_get_size, cdm_util_ops);
	} else {
		struct cam_isp_hw_fcg_update    *fcg_update;

		fcg_update = &fcg_cmd->u.fcg_update;
		rc = cam_vfe_top_apply_fcg_update(top_priv, fcg_update, cdm_util_ops);
	}

	return rc;
}

int cam_vfe_top_ver4_process_cmd(void *device_priv, uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size)
{
	int rc = 0;
	struct cam_vfe_top_ver4_priv            *top_priv;
	struct cam_hw_soc_info                  *soc_info;
	struct cam_vfe_soc_private              *soc_private;

	if (!device_priv || !cmd_args) {
		CAM_ERR(CAM_ISP, "Error, Invalid arguments");
		return -EINVAL;
	}

	top_priv = (struct cam_vfe_top_ver4_priv *)device_priv;
	soc_info = top_priv->top_common.soc_info;
	soc_private = soc_info->soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_ISP, "VFE:%u Error soc_private NULL",
			top_priv->common_data.hw_intf->hw_idx);
		return -EINVAL;
	}

	switch (cmd_type) {
	case CAM_ISP_HW_CMD_GET_CHANGE_BASE:
		rc = cam_vfe_top_ver4_mux_get_base(top_priv,
			cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_GET_REG_UPDATE:
		rc = cam_vfe_top_ver4_mux_get_reg_update(top_priv, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_CMD_CAMIF_DATA:
		rc = cam_vfe_top_ver4_get_data(top_priv, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_CMD_CLOCK_UPDATE:
		rc = cam_vfe_top_clock_update(&top_priv->top_common, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_NOTIFY_OVERFLOW:
		atomic_set(&top_priv->overflow_pending, 1);
		rc = cam_vfe_top_ver4_print_overflow_debug_info(top_priv,
			cmd_args);
		break;
	case CAM_ISP_HW_CMD_FE_UPDATE_IN_RD:
		rc = cam_vfe_top_fs_update(top_priv, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_CMD_BW_UPDATE:
		rc = cam_vfe_top_bw_update(soc_private, &top_priv->top_common,
			cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_BW_UPDATE_V2:
		rc = cam_vfe_top_bw_update_v2(soc_private,
			&top_priv->top_common, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_BW_CONTROL:
		rc = cam_vfe_top_bw_control(soc_private, &top_priv->top_common,
			cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_CORE_CONFIG:
		rc = cam_vfe_core_config_control(top_priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_GET_PATH_PORT_MAP:
		rc = cam_vfe_top_ver4_get_path_port_map(top_priv, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_CMD_APPLY_CLK_BW_UPDATE:
		rc = cam_vfe_top_apply_clk_bw_update(&top_priv->top_common, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_INIT_CONFIG_UPDATE:
		rc = cam_vfe_init_config_update(cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_RDI_LCR_CFG:
		rc = cam_vfe_top_ver4_pdaf_lcr_config(top_priv, cmd_args,
			arg_size);
		break;
	case CAM_ISP_HW_CMD_QUERY_CAP: {
		struct cam_isp_hw_cap *ife_cap;

		ife_cap = (struct cam_isp_hw_cap *) cmd_args;
		ife_cap->num_perf_counters =
			top_priv->common_data.common_reg->num_perf_counters;
	}
		break;
	case CAM_ISP_HW_CMD_IFE_DEBUG_CFG: {
		int i;
		uint32_t max_counters = top_priv->common_data.common_reg->num_perf_counters;
		struct cam_vfe_generic_debug_config *debug_cfg;

		debug_cfg = (struct cam_vfe_generic_debug_config *)cmd_args;
		if (debug_cfg->num_counters <= max_counters)
			for (i = 0; i < max_counters; i++)
				top_priv->perf_counters[i].perf_counter_val =
					debug_cfg->vfe_perf_counter_val[i];

		top_priv->enable_ife_frame_irqs = debug_cfg->enable_ife_frame_irqs;
	}
		break;
	case CAM_ISP_HW_CMD_GET_SET_PRIM_SOF_TS_ADDR: {
		struct cam_ife_csid_ts_reg_addr  *sof_addr_args =
			(struct cam_ife_csid_ts_reg_addr *)cmd_args;

		if (sof_addr_args->get_addr) {
			CAM_ERR(CAM_ISP, "VFE does not support get of primary SOF ts addr");
			rc = -EINVAL;
		} else
			rc = cam_vfe_top_ver4_set_primary_sof_timer_reg_addr(top_priv,
				sof_addr_args);
	}
		break;
	case CAM_ISP_HW_CMD_FCG_CONFIG:
		rc = cam_vfe_top_fcg_config(top_priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_UPDATE_VFE_SRC_RES_DATA:
		rc = cam_vfe_top_ver4_update_res(top_priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_UPDATE_VFE_SRC_RES_IRQ_MASK:
		rc = cam_vfe_top_ver4_enable_irq(top_priv, cmd_args);
		break;
	default:
		rc = -EINVAL;
		CAM_ERR(CAM_ISP, "VFE:%u Error, Invalid cmd:%d",
			top_priv->common_data.hw_intf->hw_idx, cmd_type);
		break;
	}

	return rc;
}

static int cam_vfe_get_evt_payload(
	struct cam_vfe_mux_ver4_data           *vfe_priv,
	struct cam_vfe_top_irq_evt_payload    **evt_payload)
{
	int rc = 0;

	spin_lock(&vfe_priv->spin_lock);
	if (list_empty(&vfe_priv->free_payload_list)) {
		CAM_ERR_RATE_LIMIT(CAM_ISP, "No free VFE event payload");
		rc = -ENODEV;
		goto done;
	}

	*evt_payload = list_first_entry(&vfe_priv->free_payload_list,
		struct cam_vfe_top_irq_evt_payload, list);
	list_del_init(&(*evt_payload)->list);
done:
	spin_unlock(&vfe_priv->spin_lock);
	return rc;
}

static int cam_vfe_top_put_evt_payload(
	struct cam_vfe_mux_ver4_data           *vfe_priv,
	struct cam_vfe_top_irq_evt_payload    **evt_payload)
{
	unsigned long flags;

	if (!vfe_priv) {
		CAM_ERR(CAM_ISP, "Invalid param core_info NULL");
		return -EINVAL;
	}
	if (*evt_payload == NULL) {
		CAM_ERR(CAM_ISP, "VFE:%u No payload to put", vfe_priv->hw_intf->hw_idx);
		return -EINVAL;
	}

	spin_lock_irqsave(&vfe_priv->spin_lock, flags);
	list_add_tail(&(*evt_payload)->list, &vfe_priv->free_payload_list);
	*evt_payload = NULL;
	spin_unlock_irqrestore(&vfe_priv->spin_lock, flags);

	CAM_DBG(CAM_ISP, "VFE:%u Done", vfe_priv->hw_intf->hw_idx);
	return 0;
}

static int cam_vfe_handle_irq_top_half(uint32_t evt_id,
	struct cam_irq_th_payload *th_payload)
{
	int32_t                                rc;
	int                                    i;
	struct cam_isp_resource_node          *vfe_res;
	struct cam_vfe_mux_ver4_data          *vfe_priv;
	struct cam_vfe_top_irq_evt_payload    *evt_payload;

	vfe_res = th_payload->handler_priv;
	vfe_priv = vfe_res->res_priv;

	CAM_DBG(CAM_ISP,
		"VFE:%u IRQ status_0: 0x%X status_1: 0x%X",
		vfe_res->hw_intf->hw_idx, th_payload->evt_status_arr[0],
		th_payload->evt_status_arr[1]);

	rc  = cam_vfe_get_evt_payload(vfe_priv, &evt_payload);
	if (rc) {
		CAM_INFO_RATE_LIMIT(CAM_ISP,
		"VFE:%u IRQ status_0: 0x%X status_1: 0x%X",
		vfe_res->hw_intf->hw_idx, th_payload->evt_status_arr[0],
		th_payload->evt_status_arr[1]);
		return rc;
	}

	cam_isp_hw_get_timestamp(&evt_payload->ts);
	evt_payload->reg_val = 0;

	for (i = 0; i < th_payload->num_registers; i++)
		evt_payload->irq_reg_val[i] = th_payload->evt_status_arr[i];

	th_payload->evt_payload_priv = evt_payload;

	if (th_payload->evt_status_arr[vfe_priv->common_reg->frame_timing_irq_reg_idx]
			& vfe_priv->reg_data->sof_irq_mask) {
		if (vfe_priv->top_priv->sof_ts_reg_addr.curr0_ts_addr &&
			vfe_priv->top_priv->sof_ts_reg_addr.curr1_ts_addr) {
			evt_payload->ts.sof_ts =
				cam_io_r_mb(vfe_priv->top_priv->sof_ts_reg_addr.curr1_ts_addr);
			evt_payload->ts.sof_ts = (evt_payload->ts.sof_ts << 32) |
				cam_io_r_mb(vfe_priv->top_priv->sof_ts_reg_addr.curr0_ts_addr);
		}

		trace_cam_log_event("SOF", "TOP_HALF",
		th_payload->evt_status_arr[vfe_priv->common_reg->frame_timing_irq_reg_idx],
		vfe_res->hw_intf->hw_idx);
	}

	if (th_payload->evt_status_arr[vfe_priv->common_reg->frame_timing_irq_reg_idx]
			& vfe_priv->reg_data->epoch0_irq_mask) {
		trace_cam_log_event("EPOCH0", "TOP_HALF",
		th_payload->evt_status_arr[vfe_priv->common_reg->frame_timing_irq_reg_idx],
		vfe_res->hw_intf->hw_idx);
	}

	if (th_payload->evt_status_arr[vfe_priv->common_reg->frame_timing_irq_reg_idx]
			& vfe_priv->reg_data->eof_irq_mask) {
		trace_cam_log_event("EOF", "TOP_HALF",
		th_payload->evt_status_arr[vfe_priv->common_reg->frame_timing_irq_reg_idx],
		vfe_res->hw_intf->hw_idx);
	}

	CAM_DBG(CAM_ISP, "VFE:%u Exit", vfe_res->hw_intf->hw_idx);
	return rc;
}

static void cam_vfe_irq_status_to_event(struct cam_vfe_mux_ver4_data *vfe_priv,
	uint32_t irq_status, bool *sof, bool *epoch, bool *eof)
{
	*sof = (irq_status & vfe_priv->reg_data->sof_irq_mask);
	*epoch = (irq_status & vfe_priv->reg_data->epoch0_irq_mask);
	*eof = (irq_status & vfe_priv->reg_data->eof_irq_mask);
}

static enum cam_vfe_top_ver4_fsm_state cam_vfe_top_ver4_fsm_next_state(
	struct cam_isp_resource_node *res,
	enum cam_vfe_top_ver4_fsm_state state)
{
	switch (state) {
	case VFE_TOP_VER4_FSM_SOF:
		return (res->is_rdi_primary_res) ? VFE_TOP_VER4_FSM_EOF : VFE_TOP_VER4_FSM_EPOCH;
	case VFE_TOP_VER4_FSM_EPOCH:
		return VFE_TOP_VER4_FSM_EOF;
	case VFE_TOP_VER4_FSM_EOF:
		return VFE_TOP_VER4_FSM_SOF;
	default:
		/* set to SOF to recover from incorrect state */
		return VFE_TOP_VER4_FSM_SOF;
	}
}

static const char *cam_vfe_top_ver4_fsm_state_to_string(
	enum cam_vfe_top_ver4_fsm_state state)
{
	switch (state) {
	case VFE_TOP_VER4_FSM_SOF:   return "SOF";
	case VFE_TOP_VER4_FSM_EPOCH: return "EPOCH";
	case VFE_TOP_VER4_FSM_EOF:   return "EOF";
	default:                     return "INVALID";
	}
}

typedef int (*cam_vfe_handle_frame_irq_t)(struct cam_vfe_mux_ver4_data *vfe_priv,
	struct cam_vfe_top_irq_evt_payload *payload,
	struct cam_isp_hw_event_info *evt_info);


static int cam_vfe_handle_sof(struct cam_vfe_mux_ver4_data *vfe_priv,
	struct cam_vfe_top_irq_evt_payload *payload,
	struct cam_isp_hw_event_info *evt_info)
{
	if ((vfe_priv->enable_sof_irq_debug) &&
		(vfe_priv->irq_debug_cnt <=
		CAM_VFE_CAMIF_IRQ_SOF_DEBUG_CNT_MAX)) {
		CAM_INFO_RATE_LIMIT(CAM_ISP, "VFE:%u Received SOF",
			vfe_priv->hw_intf->hw_idx);

		vfe_priv->irq_debug_cnt++;
		if (vfe_priv->irq_debug_cnt ==
			CAM_VFE_CAMIF_IRQ_SOF_DEBUG_CNT_MAX) {
			vfe_priv->enable_sof_irq_debug = false;
			vfe_priv->irq_debug_cnt = 0;
		}
	} else {
		uint32_t frm_irq_status =
			payload->irq_reg_val[vfe_priv->common_reg->frame_timing_irq_reg_idx];
		int hw_ctxt_id = -1;

		if (vfe_priv->reg_data->is_mc_path)
			hw_ctxt_id = cam_vfe_top_ver4_get_hw_ctxt_from_irq_status(vfe_priv,
				frm_irq_status);

		CAM_DBG(CAM_ISP, "VFE:%u is_mc:%s hw_ctxt:%d Received SOF",
			vfe_priv->hw_intf->hw_idx,
			CAM_BOOL_TO_YESNO(vfe_priv->reg_data->is_mc_path), hw_ctxt_id);
		vfe_priv->sof_ts.tv_sec = payload->ts.mono_time.tv_sec;
		vfe_priv->sof_ts.tv_nsec = payload->ts.mono_time.tv_nsec;
	}
	vfe_priv->top_priv->sof_cnt++;

	return 0;
}

static int cam_vfe_handle_epoch(struct cam_vfe_mux_ver4_data *vfe_priv,
	struct cam_vfe_top_irq_evt_payload *payload,
	struct cam_isp_hw_event_info *evt_info)
{
	CAM_DBG(CAM_ISP, "VFE:%u Received EPOCH", vfe_priv->hw_intf->hw_idx);
	evt_info->reg_val = payload->reg_val;
	vfe_priv->epoch_ts.tv_sec = payload->ts.mono_time.tv_sec;
	vfe_priv->epoch_ts.tv_nsec = payload->ts.mono_time.tv_nsec;

	return 0;
}

static int cam_vfe_handle_eof(struct cam_vfe_mux_ver4_data *vfe_priv,
	struct cam_vfe_top_irq_evt_payload *payload,
	struct cam_isp_hw_event_info *evt_info)
{
	uint32_t frm_irq_status =
		payload->irq_reg_val[vfe_priv->common_reg->frame_timing_irq_reg_idx];
	int hw_ctxt_id = -1;

	if (vfe_priv->reg_data->is_mc_path)
		hw_ctxt_id = cam_vfe_top_ver4_get_hw_ctxt_from_irq_status(vfe_priv, frm_irq_status);

	CAM_DBG(CAM_ISP, "VFE:%u is_mc:%s hw_ctxt:%d Received EOF",
		vfe_priv->hw_intf->hw_idx,
		CAM_BOOL_TO_YESNO(vfe_priv->reg_data->is_mc_path), hw_ctxt_id);

	vfe_priv->eof_ts.tv_sec = payload->ts.mono_time.tv_sec;
	vfe_priv->eof_ts.tv_nsec = payload->ts.mono_time.tv_nsec;

	return 0;
}

static int __cam_vfe_handle_frame_timing_irqs(struct cam_isp_resource_node *vfe_res, bool event,
	enum cam_isp_hw_event_type event_type, cam_vfe_handle_frame_irq_t handle_irq_fn,
	struct cam_vfe_top_irq_evt_payload *payload, struct cam_isp_hw_event_info *evt_info)
{
	struct cam_vfe_mux_ver4_data *vfe_priv = vfe_res->res_priv;

	if (!event) {
		CAM_WARN(CAM_ISP, "VFE:%u missed %s", vfe_priv->hw_intf->hw_idx,
			cam_isp_hw_evt_type_to_string(event_type));
	} else {
		handle_irq_fn(vfe_priv, payload, evt_info);
		if (!(vfe_priv->top_priv->enable_ife_frame_irqs)
			&& vfe_priv->event_cb)
			vfe_priv->event_cb(vfe_priv->priv, event_type, evt_info);
	}
	vfe_priv->fsm_state = cam_vfe_top_ver4_fsm_next_state(vfe_res, vfe_priv->fsm_state);

	return 0;
}

static int cam_vfe_handle_frame_timing_irqs(struct cam_isp_resource_node *vfe_res,
	uint32_t irq_status, struct cam_vfe_top_irq_evt_payload *payload,
	struct cam_isp_hw_event_info *evt_info)
{
	struct cam_vfe_mux_ver4_data *vfe_priv = vfe_res->res_priv;
	bool sof, epoch, eof;
	int i, j;

	cam_vfe_irq_status_to_event(vfe_priv, irq_status, &sof, &epoch, &eof);
	CAM_DBG(CAM_ISP, "VFE:%u SOF:%s EPOCH:%s EOF:%s", vfe_priv->hw_intf->hw_idx,
		CAM_BOOL_TO_YESNO(sof), CAM_BOOL_TO_YESNO(epoch), CAM_BOOL_TO_YESNO(eof));

	i = (sof ? 1 : 0) + (epoch ? 1 : 0) + (eof ? 1 : 0);
	j = i;

	if (i == vfe_priv->n_frame_irqs)
		CAM_WARN_RATE_LIMIT(CAM_ISP, "VFE:%u top-half delay", vfe_priv->hw_intf->hw_idx);

	while (i > 0) {
		bool event;
		enum cam_isp_hw_event_type event_type;
		cam_vfe_handle_frame_irq_t handle_irq_fn;

		CAM_DBG_PR2(CAM_ISP, "VFE:%u enter state:%s (%d/%d)", vfe_priv->hw_intf->hw_idx,
			cam_vfe_top_ver4_fsm_state_to_string(vfe_priv->fsm_state), i, j);

		switch (vfe_priv->fsm_state) {
		case VFE_TOP_VER4_FSM_SOF:
			event = sof;
			event_type = CAM_ISP_HW_EVENT_SOF;
			handle_irq_fn = cam_vfe_handle_sof;
			break;
		case VFE_TOP_VER4_FSM_EPOCH:
			event = epoch;
			event_type = CAM_ISP_HW_EVENT_EPOCH;
			handle_irq_fn = cam_vfe_handle_epoch;
			break;
		case VFE_TOP_VER4_FSM_EOF:
			event = eof;
			event_type = CAM_ISP_HW_EVENT_EOF;
			handle_irq_fn = cam_vfe_handle_eof;
			break;
		default:
			CAM_ERR(CAM_ISP, "VFE:%u frame state machine in invalid state",
				vfe_priv->hw_intf->hw_idx);
			return -EINVAL;
		}

		/* consume event */
		if (event)
			i--;

		__cam_vfe_handle_frame_timing_irqs(vfe_res, event, event_type, handle_irq_fn,
			payload, evt_info);

		CAM_DBG_PR2(CAM_ISP, "VFE:%u exit state:%s (%d/%d)", vfe_priv->hw_intf->hw_idx,
			cam_vfe_top_ver4_fsm_state_to_string(vfe_priv->fsm_state), i, j);
	}

	return CAM_VFE_IRQ_STATUS_SUCCESS;
}

static int cam_vfe_handle_irq_bottom_half(void *handler_priv,
	void *evt_payload_priv)
{
	int ret = CAM_VFE_IRQ_STATUS_ERR;
	struct cam_isp_resource_node *vfe_res;
	struct cam_vfe_mux_ver4_data *vfe_priv;
	struct cam_vfe_top_irq_evt_payload *payload;
	struct cam_isp_hw_event_info evt_info;
	struct cam_isp_hw_error_event_info err_evt_info;
	struct cam_isp_sof_ts_data sof_and_boot_time;
	uint32_t irq_status[CAM_IFE_IRQ_REGISTERS_MAX] = {0}, frame_timing_mask;
	struct timespec64 ts;
	int i = 0;

	if (!handler_priv || !evt_payload_priv) {
		CAM_ERR(CAM_ISP,
			"Invalid params handle_priv:%pK, evt_payload_priv:%pK",
			handler_priv, evt_payload_priv);
		return ret;
	}

	vfe_res = handler_priv;
	vfe_priv = vfe_res->res_priv;
	payload = evt_payload_priv;

	if (atomic_read(&vfe_priv->top_priv->overflow_pending)) {
		CAM_INFO(CAM_ISP,
			"VFE:%u Handling overflow, Ignore bottom half",
			vfe_res->hw_intf->hw_idx);
		cam_vfe_top_put_evt_payload(vfe_priv, &payload);
		return IRQ_HANDLED;
	}

	for (i = 0; i < CAM_IFE_IRQ_REGISTERS_MAX; i++)
		irq_status[i] = payload->irq_reg_val[i];

	sof_and_boot_time.boot_time = payload->ts.mono_time;
	sof_and_boot_time.sof_ts = payload->ts.sof_ts;

	evt_info.hw_idx   = vfe_res->hw_intf->hw_idx;
	evt_info.hw_type  = CAM_ISP_HW_TYPE_VFE;
	evt_info.res_id   = vfe_res->res_id;
	evt_info.res_type = vfe_res->res_type;
	evt_info.reg_val = 0;
	evt_info.event_data = &sof_and_boot_time;

	frame_timing_mask = vfe_priv->reg_data->sof_irq_mask |
				vfe_priv->reg_data->epoch0_irq_mask |
				vfe_priv->reg_data->eof_irq_mask;

	if (irq_status[vfe_priv->common_reg->frame_timing_irq_reg_idx] & frame_timing_mask) {
		ret = cam_vfe_handle_frame_timing_irqs(vfe_res,
			irq_status[vfe_priv->common_reg->frame_timing_irq_reg_idx] &
			frame_timing_mask,
			payload, &evt_info);
	}

	if (irq_status[CAM_IFE_IRQ_CAMIF_REG_STATUS0]
		& vfe_priv->reg_data->error_irq_mask) {
		CAM_ERR(CAM_ISP, "VFE:%u Error", evt_info.hw_idx);

		err_evt_info.err_type = CAM_VFE_IRQ_STATUS_VIOLATION;
		evt_info.event_data = (void *)&err_evt_info;
		ktime_get_boottime_ts64(&ts);
		CAM_INFO(CAM_ISP,
			"VFE:%u current monotonic timestamp:[%lld.%09lld]",
			evt_info.hw_idx, ts.tv_sec, ts.tv_nsec);

		if (vfe_priv->event_cb)
			vfe_priv->event_cb(vfe_priv->priv,
				CAM_ISP_HW_EVENT_ERROR, (void *)&evt_info);

		cam_vfe_top_ver4_print_debug_reg_status(vfe_priv->top_priv);

		cam_vfe_top_ver4_print_top_irq_error(vfe_priv,
			irq_status[CAM_IFE_IRQ_CAMIF_REG_STATUS0]);

		ret = CAM_VFE_IRQ_STATUS_ERR;
	}

	if (vfe_priv->camif_debug & CAMIF_DEBUG_ENABLE_SENSOR_DIAG_STATUS) {
		CAM_DBG(CAM_ISP, "VFE:%u VFE_DIAG_SENSOR_STATUS: 0x%X",
			evt_info.hw_idx, vfe_priv->mem_base,
			cam_io_r(vfe_priv->mem_base +
			vfe_priv->common_reg->diag_sensor_status_0));
	}

	/* Perf counter dump */
	if (irq_status[vfe_priv->common_reg->frame_timing_irq_reg_idx] &
		vfe_priv->reg_data->eof_irq_mask)
		cam_vfe_top_dump_perf_counters("EOF", vfe_res->res_name, vfe_priv->top_priv);

	if (irq_status[vfe_priv->common_reg->frame_timing_irq_reg_idx] &
		vfe_priv->reg_data->sof_irq_mask)
		cam_vfe_top_dump_perf_counters("SOF", vfe_res->res_name, vfe_priv->top_priv);

	cam_vfe_top_put_evt_payload(vfe_priv, &payload);

	CAM_DBG(CAM_ISP, "VFE:%u returning status = %d", evt_info.hw_idx, ret);
	return ret;
}

static int cam_vfe_ver4_err_irq_top_half(
	uint32_t                               evt_id,
	struct cam_irq_th_payload             *th_payload)
{
	int32_t                                rc = 0;
	int                                    i;
	struct cam_isp_resource_node          *vfe_res;
	struct cam_vfe_mux_ver4_data          *vfe_priv;
	struct cam_vfe_top_irq_evt_payload    *evt_payload;
	bool                                   error_flag = false;

	vfe_res = th_payload->handler_priv;
	vfe_priv = vfe_res->res_priv;
	/*
	 *  need to handle overflow condition here, otherwise irq storm
	 *  will block everything
	 */
	if ((th_payload->evt_status_arr[0] &
		vfe_priv->reg_data->error_irq_mask)) {
		CAM_ERR(CAM_ISP,
			"VFE:%u Err IRQ status_0: 0x%X",
			vfe_res->hw_intf->hw_idx,
			th_payload->evt_status_arr[0]);
		CAM_ERR(CAM_ISP, "Stopping further IRQ processing from VFE:%u",
			vfe_res->hw_intf->hw_idx);
		cam_irq_controller_disable_all(
			vfe_priv->vfe_irq_controller);
		error_flag = true;
	}

	rc  = cam_vfe_get_evt_payload(vfe_priv, &evt_payload);
	if (rc)
		return rc;

	cam_isp_hw_get_timestamp(&evt_payload->ts);
	if (error_flag) {
		vfe_priv->error_ts.tv_sec =
			evt_payload->ts.mono_time.tv_sec;
		vfe_priv->error_ts.tv_nsec =
			evt_payload->ts.mono_time.tv_nsec;
	}

	for (i = 0; i < th_payload->num_registers; i++)
		evt_payload->irq_reg_val[i] = th_payload->evt_status_arr[i];

	th_payload->evt_payload_priv = evt_payload;

	return rc;
}

static int cam_vfe_resource_start(
	struct cam_isp_resource_node *vfe_res)
{
	struct cam_vfe_mux_ver4_data   *rsrc_data;
	uint32_t                        val = 0, epoch_factor = 50;
	int                             rc = 0;
	uint32_t                        err_irq_mask[CAM_IFE_IRQ_REGISTERS_MAX];
	uint32_t                        irq_mask[CAM_IFE_IRQ_REGISTERS_MAX];

	if (!vfe_res) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	if (vfe_res->res_state != CAM_ISP_RESOURCE_STATE_RESERVED) {
		CAM_ERR(CAM_ISP, "VFE:%u Error, Invalid camif res res_state:%d",
			vfe_res->hw_intf->hw_idx, vfe_res->res_state);
		return -EINVAL;
	}

	memset(err_irq_mask, 0, sizeof(err_irq_mask));
	memset(irq_mask, 0, sizeof(irq_mask));
	rsrc_data = (struct cam_vfe_mux_ver4_data *)vfe_res->res_priv;

	/* config debug status registers */
	cam_io_w_mb(rsrc_data->reg_data->top_debug_cfg_en, rsrc_data->mem_base +
		rsrc_data->common_reg->top_debug_cfg);

	if (rsrc_data->is_lite || !rsrc_data->is_pixel_path)
		goto skip_core_cfg;

	/* IFE top cfg programmed via CDM */
	CAM_DBG(CAM_ISP, "VFE:%u TOP core_cfg0: 0x%x core_cfg1: 0x%x",
		vfe_res->hw_intf->hw_idx,
		cam_io_r_mb(rsrc_data->mem_base +
			rsrc_data->common_reg->core_cfg_0),
		cam_io_r_mb(rsrc_data->mem_base +
			rsrc_data->common_reg->core_cfg_1));

	/* % epoch factor from userland */
	if ((rsrc_data->epoch_factor) && (rsrc_data->epoch_factor <= 100))
		epoch_factor = rsrc_data->epoch_factor;

	val = ((rsrc_data->last_line + rsrc_data->vbi_value) -
		rsrc_data->first_line) * epoch_factor / 100;

	if (val > rsrc_data->last_line)
		val = rsrc_data->last_line;

	if (rsrc_data->horizontal_bin || rsrc_data->qcfa_bin ||
		rsrc_data->sfe_binned_epoch_cfg)
		val >>= 1;

	cam_io_w_mb(val, rsrc_data->mem_base +
				rsrc_data->common_reg->epoch_height_cfg);
	CAM_DBG(CAM_ISP,
		"VFE:%u height [0x%x : 0x%x] vbi_val: 0x%x epoch_factor: %u%% epoch_line_cfg: 0x%x",
		vfe_res->hw_intf->hw_idx, rsrc_data->first_line, rsrc_data->last_line,
		rsrc_data->vbi_value, epoch_factor, val);

skip_core_cfg:
	vfe_res->res_state = CAM_ISP_RESOURCE_STATE_STREAMING;

	/* reset sof count */
	rsrc_data->top_priv->sof_cnt = 0;

	/* disable sof irq debug flag */
	rsrc_data->enable_sof_irq_debug = false;
	rsrc_data->irq_debug_cnt = 0;

	if (rsrc_data->camif_debug &
		CAMIF_DEBUG_ENABLE_SENSOR_DIAG_STATUS) {
		val = cam_io_r_mb(rsrc_data->mem_base +
			rsrc_data->common_reg->diag_config);
		val |= rsrc_data->reg_data->enable_diagnostic_hw;
		cam_io_w_mb(val, rsrc_data->mem_base +
			rsrc_data->common_reg->diag_config);
	}

	if (vfe_res->is_per_port_start) {
		CAM_DBG(CAM_ISP, "Skipping irq subscribe for resources that are not updated");
		goto skip_irq_subscribe;
	}

	/* Skip subscribing to timing irqs in these scenarios:
	 * Debug config is not enabled for IFE frame timing IRQs, and
	 *     1. Resource is dual IFE slave
	 *     2. Resource is not primary RDI
	 *     3. non-sfe use cases, such cases are taken care in CSID.
	 */

	if (!(rsrc_data->top_priv->enable_ife_frame_irqs) &&
		(((rsrc_data->sync_mode == CAM_ISP_HW_SYNC_SLAVE) && rsrc_data->is_dual) ||
		(!rsrc_data->is_pixel_path && !vfe_res->is_rdi_primary_res) ||
		!rsrc_data->handle_camif_irq))
		goto skip_frame_irq_subscribe;

	irq_mask[rsrc_data->common_reg->frame_timing_irq_reg_idx] =
		rsrc_data->reg_data->sof_irq_mask | rsrc_data->reg_data->epoch0_irq_mask |
		rsrc_data->reg_data->eof_irq_mask;

	rsrc_data->n_frame_irqs =
		hweight32(irq_mask[rsrc_data->common_reg->frame_timing_irq_reg_idx]);

	if (!rsrc_data->frame_irq_handle) {
		rsrc_data->frame_irq_handle = cam_irq_controller_subscribe_irq(
			rsrc_data->vfe_irq_controller,
			CAM_IRQ_PRIORITY_1,
			irq_mask,
			vfe_res,
			vfe_res->top_half_handler,
			vfe_res->bottom_half_handler,
			vfe_res->tasklet_info,
			&tasklet_bh_api,
			CAM_IRQ_EVT_GROUP_0);

		if (rsrc_data->frame_irq_handle < 1) {
			CAM_ERR(CAM_ISP, "VFE:%u Frame IRQs handle subscribe failure",
				vfe_res->hw_intf->hw_idx);
			rc = -ENOMEM;
			rsrc_data->frame_irq_handle = 0;
		}
	}

skip_frame_irq_subscribe:
	err_irq_mask[CAM_IFE_IRQ_CAMIF_REG_STATUS0] = rsrc_data->reg_data->error_irq_mask;

	if (!rsrc_data->irq_err_handle) {
		rsrc_data->irq_err_handle = cam_irq_controller_subscribe_irq(
			rsrc_data->vfe_irq_controller,
			CAM_IRQ_PRIORITY_0,
			err_irq_mask,
			vfe_res,
			cam_vfe_ver4_err_irq_top_half,
			vfe_res->bottom_half_handler,
			vfe_res->tasklet_info,
			&tasklet_bh_api,
			CAM_IRQ_EVT_GROUP_0);

		if (rsrc_data->irq_err_handle < 1) {
			CAM_ERR(CAM_ISP, "VFE:%u Error IRQ handle subscribe failure",
				vfe_res->hw_intf->hw_idx);
			rc = -ENOMEM;
			rsrc_data->irq_err_handle = 0;
		}
	}

skip_irq_subscribe:
	rsrc_data->fsm_state = VFE_TOP_VER4_FSM_SOF;

	CAM_DBG(CAM_ISP, "VFE:%u Res: %s Start Done",
		vfe_res->hw_intf->hw_idx,
		vfe_res->res_name);

	return rc;
}

static int cam_vfe_resource_stop(
	struct cam_isp_resource_node *vfe_res)
{
	struct cam_vfe_mux_ver4_data        *vfe_priv;
	struct cam_vfe_top_ver4_priv        *top_priv;
	int                                  rc = 0;
	uint32_t                             val = 0;

	if (!vfe_res) {
		CAM_ERR(CAM_ISP, "Error, Invalid input arguments");
		return -EINVAL;
	}

	if ((vfe_res->res_state == CAM_ISP_RESOURCE_STATE_RESERVED) ||
		(vfe_res->res_state == CAM_ISP_RESOURCE_STATE_AVAILABLE))
		return 0;

	vfe_priv = (struct cam_vfe_mux_ver4_data *)vfe_res->res_priv;
	top_priv = vfe_priv->top_priv;

	if (vfe_priv->is_lite || !vfe_priv->is_pixel_path)
		goto skip_core_decfg;

	if ((vfe_priv->dsp_mode >= CAM_ISP_DSP_MODE_ONE_WAY) &&
		(vfe_priv->dsp_mode <= CAM_ISP_DSP_MODE_ROUND)) {
		val = cam_io_r_mb(vfe_priv->mem_base +
			vfe_priv->common_reg->core_cfg_0);
		val &= (~(1 << CAM_SHIFT_TOP_CORE_VER_4_CFG_DSP_EN));
		cam_io_w_mb(val, vfe_priv->mem_base +
			vfe_priv->common_reg->core_cfg_0);
	}

skip_core_decfg:
	if (vfe_res->res_state == CAM_ISP_RESOURCE_STATE_STREAMING)
		vfe_res->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;

	val = cam_io_r_mb(vfe_priv->mem_base +
		vfe_priv->common_reg->diag_config);
	if (val & vfe_priv->reg_data->enable_diagnostic_hw) {
		val &= ~vfe_priv->reg_data->enable_diagnostic_hw;
		cam_io_w_mb(val, vfe_priv->mem_base +
			vfe_priv->common_reg->diag_config);
	}

	if (vfe_priv->frame_irq_handle) {
		cam_irq_controller_unsubscribe_irq(
			vfe_priv->vfe_irq_controller,
			vfe_priv->frame_irq_handle);
		vfe_priv->frame_irq_handle = 0;
	}
	vfe_priv->n_frame_irqs = 0;

	if (vfe_priv->irq_err_handle) {
		cam_irq_controller_unsubscribe_irq(
			vfe_priv->vfe_irq_controller,
			vfe_priv->irq_err_handle);
		vfe_priv->irq_err_handle = 0;
	}

	/* Skip epoch factor reset for internal recovery */
	if (!top_priv->top_common.skip_data_rst_on_stop)
		vfe_priv->epoch_factor = 0;

	CAM_DBG(CAM_ISP, "VFE:%u Res: %s Stopped",
		vfe_res->hw_intf->hw_idx,
		vfe_res->res_name);

	return rc;
}

static int cam_vfe_resource_init(
	struct cam_isp_resource_node *vfe_res,
	void *init_args, uint32_t arg_size)
{
	struct cam_vfe_mux_ver4_data          *rsrc_data;
	struct cam_hw_soc_info                *soc_info;
	int                                    rc = 0;

	if (!vfe_res) {
		CAM_ERR(CAM_ISP, "Error Invalid input arguments");
		return -EINVAL;
	}

	rsrc_data = vfe_res->res_priv;
	soc_info = rsrc_data->soc_info;

	if ((rsrc_data->dsp_mode >= CAM_ISP_DSP_MODE_ONE_WAY) &&
		(rsrc_data->dsp_mode <= CAM_ISP_DSP_MODE_ROUND)) {
		rc = cam_vfe_soc_enable_clk(soc_info, CAM_VFE_DSP_CLK_NAME);
		if (rc)
			CAM_ERR(CAM_ISP,
				"VFE:%u failed to enable dsp clk, rc = %d",
				vfe_res->hw_intf->hw_idx, rc);
	}

	rsrc_data->sof_ts.tv_sec = 0;
	rsrc_data->sof_ts.tv_nsec = 0;
	rsrc_data->epoch_ts.tv_sec = 0;
	rsrc_data->epoch_ts.tv_nsec = 0;
	rsrc_data->eof_ts.tv_sec = 0;
	rsrc_data->eof_ts.tv_nsec = 0;
	rsrc_data->error_ts.tv_sec = 0;
	rsrc_data->error_ts.tv_nsec = 0;

	CAM_DBG(CAM_ISP, "VFE:%u Res: %s Init Done",
		vfe_res->hw_intf->hw_idx,
		vfe_res->res_name);

	return rc;
}

static int cam_vfe_resource_deinit(
	struct cam_isp_resource_node        *vfe_res,
	void *deinit_args, uint32_t arg_size)
{
	struct cam_vfe_mux_ver4_data          *rsrc_data;
	struct cam_hw_soc_info                *soc_info;
	int                                    rc = 0;

	if (!vfe_res) {
		CAM_ERR(CAM_ISP, "Error Invalid input arguments");
		return -EINVAL;
	}

	rsrc_data = vfe_res->res_priv;
	soc_info = rsrc_data->soc_info;

	if ((rsrc_data->dsp_mode >= CAM_ISP_DSP_MODE_ONE_WAY) &&
		(rsrc_data->dsp_mode <= CAM_ISP_DSP_MODE_ROUND)) {
		rc = cam_vfe_soc_disable_clk(soc_info, CAM_VFE_DSP_CLK_NAME);
		if (rc)
			CAM_ERR(CAM_ISP, "VFE:%u failed to disable dsp clk",
				vfe_res->hw_intf->hw_idx);
	}

	CAM_DBG(CAM_ISP, "VFE:%u Res: %s DeInit Done",
		vfe_res->hw_intf->hw_idx,
		vfe_res->res_name);
	return rc;
}

int cam_vfe_res_mux_init(
	struct cam_vfe_top_ver4_priv  *top_priv,
	struct cam_hw_intf            *hw_intf,
	struct cam_hw_soc_info        *soc_info,
	void                          *vfe_hw_info,
	struct cam_isp_resource_node  *vfe_res,
	void                          *vfe_irq_controller)
{
	struct cam_vfe_mux_ver4_data           *vfe_priv = NULL;
	struct cam_vfe_ver4_path_hw_info       *hw_info = vfe_hw_info;
	struct cam_vfe_soc_private    *soc_priv = soc_info->soc_private;
	int i;

	vfe_priv = kzalloc(sizeof(struct cam_vfe_mux_ver4_data),
		GFP_KERNEL);
	if (!vfe_priv)
		return -ENOMEM;

	vfe_res->res_priv     = vfe_priv;
	vfe_priv->mem_base    = soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base;
	vfe_priv->common_reg  = hw_info->common_reg;
	vfe_priv->reg_data    = hw_info->reg_data;
	vfe_priv->hw_intf     = hw_intf;
	vfe_priv->is_lite     = soc_priv->is_ife_lite;
	vfe_priv->soc_info    = soc_info;
	vfe_priv->vfe_irq_controller = vfe_irq_controller;
	vfe_priv->is_pixel_path = (vfe_res->res_id == CAM_ISP_HW_VFE_IN_CAMIF);
	vfe_priv->top_priv     = top_priv;

	vfe_res->init                = cam_vfe_resource_init;
	vfe_res->deinit              = cam_vfe_resource_deinit;
	vfe_res->start               = cam_vfe_resource_start;
	vfe_res->stop                = cam_vfe_resource_stop;
	vfe_res->top_half_handler    = cam_vfe_handle_irq_top_half;
	vfe_res->bottom_half_handler = cam_vfe_handle_irq_bottom_half;

	spin_lock_init(&vfe_priv->spin_lock);
	INIT_LIST_HEAD(&vfe_priv->free_payload_list);
	for (i = 0; i < CAM_VFE_CAMIF_EVT_MAX; i++) {
		INIT_LIST_HEAD(&vfe_priv->evt_payload[i].list);
		list_add_tail(&vfe_priv->evt_payload[i].list,
			&vfe_priv->free_payload_list);
	}
	return 0;
}

int cam_vfe_res_mux_deinit(
	struct cam_isp_resource_node  *vfe_res)
{
	struct cam_vfe_mux_ver4_data *vfe_priv;
	int i = 0;

	if (!vfe_res) {
		CAM_ERR(CAM_ISP, "Error, VFE Node Resource is NULL %pK", vfe_res);
		return -ENODEV;
	}

	vfe_priv = vfe_res->res_priv;

	vfe_res->init                = NULL;
	vfe_res->deinit              = NULL;
	vfe_res->start               = NULL;
	vfe_res->stop                = NULL;
	vfe_res->process_cmd         = NULL;
	vfe_res->top_half_handler    = NULL;
	vfe_res->bottom_half_handler = NULL;
	vfe_res->res_priv            = NULL;

	if (!vfe_priv) {
		CAM_ERR(CAM_ISP, "VFE:%u vfe_priv is NULL %pK", vfe_res->hw_intf->hw_idx, vfe_priv);
		return -ENODEV;
	}

	INIT_LIST_HEAD(&vfe_priv->free_payload_list);
	for (i = 0; i < CAM_VFE_CAMIF_EVT_MAX; i++)
		INIT_LIST_HEAD(&vfe_priv->evt_payload[i].list);
	kfree(vfe_priv);

	return 0;
}

int cam_vfe_top_ver4_init(
	struct cam_hw_soc_info                 *soc_info,
	struct cam_hw_intf                     *hw_intf,
	void                                   *top_hw_info,
	void                                   *vfe_irq_controller,
	struct cam_vfe_top                    **vfe_top_ptr)
{
	int i, j, rc = 0;
	struct cam_vfe_top_ver4_priv           *top_priv = NULL;
	struct cam_vfe_top_ver4_hw_info        *hw_info = top_hw_info;
	struct cam_vfe_top                     *vfe_top;

	vfe_top = kzalloc(sizeof(struct cam_vfe_top), GFP_KERNEL);
	if (!vfe_top) {
		CAM_DBG(CAM_ISP, "VFE:%u Error, Failed to alloc for vfe_top", hw_intf->hw_idx);
		rc = -ENOMEM;
		goto end;
	}

	top_priv = kzalloc(sizeof(struct cam_vfe_top_ver4_priv),
		GFP_KERNEL);
	if (!top_priv) {
		CAM_DBG(CAM_ISP, "VFE:%u Error, Failed to alloc for vfe_top_priv", hw_intf->hw_idx);
		rc = -ENOMEM;
		goto free_vfe_top;
	}

	vfe_top->top_priv = top_priv;
	top_priv->top_common.applied_clk_rate = 0;

	if (hw_info->num_mux > CAM_VFE_TOP_MUX_MAX) {
		CAM_ERR(CAM_ISP, "VFE:%u Invalid number of input rsrc: %d, max: %d",
			hw_intf->hw_idx, hw_info->num_mux, CAM_VFE_TOP_MUX_MAX);
		rc = -EINVAL;
		goto free_top_priv;
	}

	top_priv->top_common.num_mux = hw_info->num_mux;

	for (i = 0, j = 0; i < top_priv->top_common.num_mux &&
		j < hw_info->num_rdi; i++) {
		top_priv->top_common.mux_rsrc[i].res_type =
			CAM_ISP_RESOURCE_VFE_IN;
		top_priv->top_common.mux_rsrc[i].hw_intf = hw_intf;
		top_priv->top_common.mux_rsrc[i].res_state =
			CAM_ISP_RESOURCE_STATE_AVAILABLE;
		top_priv->top_common.req_clk_rate[i] = 0;

		if (hw_info->mux_type[i] == CAM_VFE_CAMIF_VER_4_0) {
			top_priv->top_common.mux_rsrc[i].res_id =
				CAM_ISP_HW_VFE_IN_CAMIF;

			rc = cam_vfe_res_mux_init(top_priv,
				hw_intf, soc_info,
				&hw_info->vfe_full_hw_info,
				&top_priv->top_common.mux_rsrc[i],
				vfe_irq_controller);
			scnprintf(top_priv->top_common.mux_rsrc[i].res_name,
				CAM_ISP_RES_NAME_LEN, "CAMIF");
		} else if (hw_info->mux_type[i] ==
			CAM_VFE_PDLIB_VER_1_0) {
			/* set the PDLIB resource id */
			top_priv->top_common.mux_rsrc[i].res_id =
				CAM_ISP_HW_VFE_IN_PDLIB;

			rc = cam_vfe_res_mux_init(top_priv,
				hw_intf, soc_info,
				&hw_info->pdlib_hw_info,
				&top_priv->top_common.mux_rsrc[i],
				vfe_irq_controller);
			scnprintf(top_priv->top_common.mux_rsrc[i].res_name,
				CAM_ISP_RES_NAME_LEN, "PDLIB");
		} else if (hw_info->mux_type[i] ==
			CAM_VFE_RDI_VER_1_0) {
			/* set the RDI resource id */
			top_priv->top_common.mux_rsrc[i].res_id =
				CAM_ISP_HW_VFE_IN_RDI0 + j;

			scnprintf(top_priv->top_common.mux_rsrc[i].res_name,
				CAM_ISP_RES_NAME_LEN, "RDI_%d", j);
			rc = cam_vfe_res_mux_init(top_priv,
				hw_intf, soc_info,
				&hw_info->rdi_hw_info[j++],
				&top_priv->top_common.mux_rsrc[i],
				vfe_irq_controller);
		} else {
			CAM_WARN(CAM_ISP, "VFE:%u Invalid mux type: %u",
				hw_intf->hw_idx, hw_info->mux_type[i]);
		}
		if (rc)
			goto deinit_resources;
	}


	vfe_top->hw_ops.get_hw_caps = cam_vfe_top_ver4_get_hw_caps;
	vfe_top->hw_ops.init        = cam_vfe_top_ver4_init_hw;
	vfe_top->hw_ops.reset       = cam_vfe_top_ver4_reset;
	vfe_top->hw_ops.reserve     = cam_vfe_top_ver4_reserve;
	vfe_top->hw_ops.release     = cam_vfe_top_ver4_release;
	vfe_top->hw_ops.start       = cam_vfe_top_ver4_start;
	vfe_top->hw_ops.stop        = cam_vfe_top_ver4_stop;
	vfe_top->hw_ops.read        = cam_vfe_top_ver4_read;
	vfe_top->hw_ops.write       = cam_vfe_top_ver4_write;
	vfe_top->hw_ops.process_cmd = cam_vfe_top_ver4_process_cmd;
	*vfe_top_ptr = vfe_top;

	top_priv->common_data.hw_info      = hw_info;
	top_priv->top_common.soc_info     = soc_info;
	top_priv->common_data.hw_intf      = hw_intf;
	top_priv->top_common.hw_idx        = hw_intf->hw_idx;
	top_priv->common_data.common_reg   = hw_info->common_reg;

	if (top_priv->common_data.common_reg->num_perf_counters > CAM_VFE_PERF_CNT_MAX) {
		CAM_ERR(CAM_ISP, "Invalid number of perf counters: %d max: %d",
			top_priv->common_data.common_reg->num_perf_counters,
			CAM_VFE_PERF_CNT_MAX);
		rc = -EINVAL;
		goto deinit_resources;
	}

	return rc;

deinit_resources:

	for (--i; i >= 0; i--) {
		if (hw_info->mux_type[i] == CAM_VFE_CAMIF_VER_4_0) {
			if (cam_vfe_res_mux_deinit(
				&top_priv->top_common.mux_rsrc[i]))
				CAM_ERR(CAM_ISP, "VFE:%u Camif Deinit failed", hw_intf->hw_idx);
		} else {
			if (cam_vfe_res_mux_deinit(
				&top_priv->top_common.mux_rsrc[i]))
				CAM_ERR(CAM_ISP,
					"VFE:%u Camif lite res id %d Deinit failed",
					hw_intf->hw_idx, top_priv->top_common.mux_rsrc[i]
					.res_id);
		}
		top_priv->top_common.mux_rsrc[i].res_state =
			CAM_ISP_RESOURCE_STATE_UNAVAILABLE;
	}


free_top_priv:
	kfree(vfe_top->top_priv);
free_vfe_top:
	kfree(vfe_top);
end:
	return rc;
}

int cam_vfe_top_ver4_deinit(struct cam_vfe_top  **vfe_top_ptr)
{
	int i, rc = 0;
	struct cam_vfe_top_ver4_priv           *top_priv = NULL;
	struct cam_vfe_top                     *vfe_top;

	if (!vfe_top_ptr) {
		CAM_ERR(CAM_ISP, "Error, Invalid input");
		return -EINVAL;
	}

	vfe_top = *vfe_top_ptr;
	if (!vfe_top) {
		CAM_ERR(CAM_ISP, "Error, vfe_top NULL");
		return -ENODEV;
	}

	top_priv = vfe_top->top_priv;
	if (!top_priv) {
		CAM_ERR(CAM_ISP, "Error, vfe_top_priv NULL");
		rc = -ENODEV;
		goto free_vfe_top;
	}

	for (i = 0; i < top_priv->top_common.num_mux; i++) {
		top_priv->top_common.mux_rsrc[i].res_state =
			CAM_ISP_RESOURCE_STATE_UNAVAILABLE;
		rc = cam_vfe_res_mux_deinit(&top_priv->top_common.mux_rsrc[i]);
		if (rc)
			CAM_ERR(CAM_ISP, "VFE:%u Mux[%d] deinit failed rc=%d",
				top_priv->common_data.hw_intf->hw_idx, i, rc);
	}

	kfree(vfe_top->top_priv);

free_vfe_top:
	kfree(vfe_top);
	*vfe_top_ptr = NULL;

	return rc;
}
