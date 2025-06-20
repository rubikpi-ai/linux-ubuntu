// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */


#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <media/cam_isp.h>

#include "cam_io_util.h"
#include "cam_debug_util.h"
#include "cam_cdm_util.h"
#include "cam_hw_intf.h"
#include "cam_ife_hw_mgr.h"
#include "cam_vfe_hw_intf.h"
#include "cam_irq_controller.h"
#include "cam_tasklet_util.h"
#include "cam_vfe_bus.h"
#include "cam_vfe_bus_ver3.h"
#include "cam_vfe_core.h"
#include "cam_vfe_soc.h"
#include "cam_debug_util.h"
#include "cam_cpas_api.h"
#include "cam_trace.h"
#include "cam_smmu_api.h"
#include "cam_common_util.h"
#include "cam_compat.h"

static const char drv_name[] = "vfe_bus";

#define CAM_VFE_BUS_VER3_IRQ_REG0                0
#define CAM_VFE_BUS_VER3_IRQ_REG1                1

#define CAM_VFE_BUS_VER3_PAYLOAD_MAX             256

#define CAM_VFE_RDI_BUS_DEFAULT_WIDTH               0xFFFF
#define CAM_VFE_RDI_BUS_DEFAULT_STRIDE              0xFFFF

#define MAX_BUF_UPDATE_REG_NUM   \
	((sizeof(struct cam_vfe_bus_ver3_reg_offset_bus_client) +  \
	sizeof(struct cam_vfe_bus_ver3_reg_offset_ubwc_client))/4)
#define MAX_REG_VAL_PAIR_SIZE    \
	(MAX_BUF_UPDATE_REG_NUM * 2 * CAM_PACKET_MAX_PLANES)

static uint32_t bus_error_irq_mask[2] = {
	0xD0000000,
	0x00000000,
};

enum cam_vfe_bus_wr_wm_mode {
	CAM_VFE_WM_LINE_BASED_MODE,
        CAM_VFE_WM_FRAME_BASED_MODE,
        CAM_VFE_WM_INDEX_BASED_MODE,
        CAM_VFE_WM_MODE_MAX,
};

enum cam_vfe_bus_ver3_packer_format {
	PACKER_FMT_VER3_PLAIN_128,
	PACKER_FMT_VER3_PLAIN_8,
	PACKER_FMT_VER3_PLAIN_8_ODD_EVEN,
	PACKER_FMT_VER3_PLAIN_8_LSB_MSB_10,
	PACKER_FMT_VER3_PLAIN_8_LSB_MSB_10_ODD_EVEN,
	PACKER_FMT_VER3_PLAIN_16_10BPP,
	PACKER_FMT_VER3_PLAIN_16_12BPP,
	PACKER_FMT_VER3_PLAIN_16_14BPP,
	PACKER_FMT_VER3_PLAIN_16_16BPP,
	PACKER_FMT_VER3_PLAIN_32,
	PACKER_FMT_VER3_PLAIN_64,
	PACKER_FMT_VER3_TP_10,
	PACKER_FMT_VER3_MIPI10,
	PACKER_FMT_VER3_MIPI12,
	PACKER_FMT_VER3_MIPI14,
	PACKER_FMT_VER3_MIPI20,
	PACKER_FMT_VER3_PLAIN32_20BPP,
	PACKER_FMT_VER3_MAX,
};

struct cam_vfe_bus_irq_violation_type {
	bool hwpd_violation;
};

struct cam_vfe_bus_ver3_comp_grp_acquire_args {
	enum cam_vfe_bus_ver3_comp_grp_type  comp_grp_id;
	uint64_t                             composite_mask;
};

struct cam_vfe_bus_ver3_common_data {
	uint32_t                                    core_index;
	void __iomem                               *mem_base;
	struct cam_hw_intf                         *hw_intf;
	void                                       *bus_irq_controller;
	void                                       *vfe_irq_controller;
	void                                       *buf_done_controller;
	void                                       *priv;
	struct cam_hw_soc_info                     *soc_info;
	struct cam_vfe_bus_ver3_reg_offset_common  *common_reg;
	struct cam_cdm_utils_ops                   *cdm_util_ops;
	uint32_t                                    io_buf_update[
		MAX_REG_VAL_PAIR_SIZE];

	struct cam_vfe_bus_irq_evt_payload          evt_payload[
		CAM_VFE_BUS_VER3_PAYLOAD_MAX];
	struct list_head                            free_payload_list;
	spinlock_t                                  spin_lock;
	struct mutex                                bus_mutex;
	uint32_t                                    secure_mode;
	uint32_t                                    num_sec_out;
	uint32_t                                    addr_no_sync;
	uint32_t                                    supported_irq;
	bool                                        comp_config_needed;
	bool                                        is_lite;
	bool                                        hw_init;
	bool                                        support_consumed_addr;
	bool                                        disable_ubwc_comp;
	bool                                        init_irq_subscribed;
	bool                                        disable_mmu_prefetch;
	cam_hw_mgr_event_cb_func                    event_cb;
	int                        rup_irq_handle[CAM_VFE_BUS_VER3_SRC_GRP_MAX];
	uint32_t                                    pack_align_shift;
	uint32_t                                    max_bw_counter_limit;
};

struct cam_vfe_bus_ver3_wm_resource_data {
	uint32_t             index;
	struct cam_vfe_bus_ver3_common_data            *common_data;
	struct cam_vfe_bus_ver3_reg_offset_bus_client  *hw_regs;

	bool                 init_cfg_done;
	bool                 hfr_cfg_done;

	uint32_t             offset;
	uint32_t             width;
	uint32_t             height;
	uint32_t             stride;
	uint32_t             format;
	enum cam_vfe_bus_ver3_packer_format pack_fmt;

	uint32_t             burst_len;

	uint32_t             en_ubwc;
	bool                 ubwc_updated;
	uint32_t             packer_cfg;
	uint32_t             h_init;
	uint32_t             ubwc_meta_addr;
	uint32_t             ubwc_meta_cfg;
	uint32_t             ubwc_mode_cfg;
	uint32_t             ubwc_stats_ctrl;
	uint32_t             ubwc_ctrl_2;

	uint32_t             irq_subsample_period;
	uint32_t             irq_subsample_pattern;
	uint32_t             framedrop_period;
	uint32_t             framedrop_pattern;

	uint32_t             en_cfg;
	uint32_t             is_dual;

	uint32_t             ubwc_lossy_threshold_0;
	uint32_t             ubwc_lossy_threshold_1;
	uint32_t             ubwc_offset_lossy_variance;
	uint32_t             ubwc_bandwidth_limit;
	uint32_t             acquired_width;
	uint32_t             acquired_height;
	uint32_t             default_line_based;
	bool                 use_wm_pack;
	bool                 update_wm_format;
	enum cam_vfe_bus_wr_wm_mode wm_mode;
};

struct cam_vfe_bus_ver3_comp_grp_data {
	enum cam_vfe_bus_ver3_comp_grp_type          comp_grp_type;
	struct cam_vfe_bus_ver3_common_data         *common_data;

	uint64_t                                     composite_mask;
	uint32_t                                     comp_done_mask;
	uint32_t                                     is_master;
	uint32_t                                     is_dual;
	uint32_t                                     dual_slave_core;
	uint32_t                                     intra_client_mask;
	uint32_t                                     addr_sync_mode;

	uint32_t                                     acquire_dev_cnt;
	uint32_t                                     irq_trigger_cnt;
	uint32_t                                     ubwc_static_ctrl;
};

struct cam_vfe_bus_ver3_vfe_out_data {
	uint32_t                              out_type;
	uint32_t                              source_group;
	struct cam_vfe_bus_ver3_common_data  *common_data;
	struct cam_vfe_bus_ver3_priv         *bus_priv;

	uint32_t                         num_wm;
	struct cam_isp_resource_node    *wm_res;

	struct cam_isp_resource_node    *comp_grp;
	enum cam_isp_hw_sync_mode        dual_comp_sync_mode;
	uint32_t                         dual_hw_alternate_vfe_id;
	struct list_head                 vfe_out_list;

	uint32_t                         is_master;
	uint32_t                         is_dual;

	uint32_t                         format;
	uint32_t                         max_width;
	uint32_t                         max_height;
	uint32_t                         secure_mode;
	void                            *priv;
	uint32_t                        *mid;
	uint32_t                         num_mid;
	bool                             limiter_enabled;
	bool                             mc_based;
	bool                             cntxt_cfg_except;
	uint32_t                         dst_hw_ctxt_id_mask;
	uint64_t                         pid_mask;
};

struct cam_vfe_bus_ver3_priv {
	struct cam_vfe_bus_ver3_common_data    common_data;
	uint32_t                               num_client;
	uint32_t                               num_out;
	uint32_t                               num_comp_grp;
	uint32_t                               top_irq_shift;

	struct cam_isp_resource_node          *bus_client;
	struct cam_isp_resource_node          *comp_grp;
	struct cam_isp_resource_node          *vfe_out;
	uint32_t  vfe_out_map_outtype[CAM_VFE_BUS_VER3_VFE_OUT_MAX];
	struct list_head                       free_comp_grp;
	struct list_head                       used_comp_grp;
	int                                    bus_irq_handle;
	int                                    rup_irq_handle;
	int                                    error_irq_handle;
	void                                  *tasklet_info;
	uint32_t                               max_out_res;
	uint32_t                               num_cons_err;
	struct cam_vfe_constraint_error_info  *constraint_error_list;
	struct cam_vfe_bus_ver3_hw_info       *bus_hw_info;
};

static void cam_vfe_bus_ver3_unsubscribe_init_irq(
	struct cam_vfe_bus_ver3_priv          *bus_priv);

static int cam_vfe_bus_ver3_subscribe_init_irq(
	struct cam_vfe_bus_ver3_priv          *bus_priv);

static int cam_vfe_bus_ver3_process_cmd(
	struct cam_isp_resource_node *priv,
	uint32_t cmd_type, void *cmd_args, uint32_t arg_size);

static int cam_vfe_bus_ver3_config_ubwc_regs(
	struct cam_vfe_bus_ver3_wm_resource_data *wm_data);

static int cam_vfe_bus_ver3_get_evt_payload(
	struct cam_vfe_bus_ver3_common_data  *common_data,
	struct cam_vfe_bus_irq_evt_payload  **evt_payload)
{
	int rc;

	spin_lock(&common_data->spin_lock);

	if (!common_data->hw_init) {
		*evt_payload = NULL;
		CAM_ERR_RATE_LIMIT(CAM_ISP, "VFE:%u Bus uninitialized",
			common_data->core_index);
		rc = -EPERM;
		goto done;
	}

	if (list_empty(&common_data->free_payload_list)) {
		*evt_payload = NULL;
		CAM_ERR_RATE_LIMIT(CAM_ISP, "VFE:%u No free BUS event payload",
				common_data->core_index);
		rc = -ENODEV;
		goto done;
	}

	*evt_payload = list_first_entry(&common_data->free_payload_list,
		struct cam_vfe_bus_irq_evt_payload, list);
	list_del_init(&(*evt_payload)->list);
	rc = 0;
done:
	spin_unlock(&common_data->spin_lock);
	return rc;
}

static int cam_vfe_bus_ver3_put_evt_payload(
	struct cam_vfe_bus_ver3_common_data     *common_data,
	struct cam_vfe_bus_irq_evt_payload     **evt_payload)
{
	unsigned long flags;

	if (!common_data) {
		CAM_ERR(CAM_ISP, "Invalid param common_data NULL");
		return -EINVAL;
	}

	if (*evt_payload == NULL) {
		CAM_ERR(CAM_ISP, "VFE:%u No payload to put", common_data->core_index);
		return -EINVAL;
	}

	spin_lock_irqsave(&common_data->spin_lock, flags);
	if (common_data->hw_init)
		list_add_tail(&(*evt_payload)->list,
			&common_data->free_payload_list);
	spin_unlock_irqrestore(&common_data->spin_lock, flags);

	*evt_payload = NULL;

	CAM_DBG(CAM_ISP, "VFE:%u Done", common_data->core_index);
	return 0;
}

static bool cam_vfe_bus_ver3_can_be_secure(uint32_t out_type)
{
	switch (out_type) {
	case CAM_VFE_BUS_VER3_VFE_OUT_FULL:
	case CAM_VFE_BUS_VER3_VFE_OUT_DS4:
	case CAM_VFE_BUS_VER3_VFE_OUT_DS16:
	case CAM_VFE_BUS_VER3_VFE_OUT_RAW_DUMP:
	case CAM_VFE_BUS_VER3_VFE_OUT_RDI0:
	case CAM_VFE_BUS_VER3_VFE_OUT_RDI1:
	case CAM_VFE_BUS_VER3_VFE_OUT_RDI2:
	case CAM_VFE_BUS_VER3_VFE_OUT_RDI3:
	case CAM_VFE_BUS_VER3_VFE_OUT_FULL_DISP:
	case CAM_VFE_BUS_VER3_VFE_OUT_DS4_DISP:
	case CAM_VFE_BUS_VER3_VFE_OUT_DS16_DISP:
	case CAM_VFE_BUS_VER3_VFE_OUT_2PD:
	case CAM_VFE_BUS_VER3_VFE_OUT_LCR:
	case CAM_VFE_BUS_VER3_VFE_OUT_PDAF:
	case CAM_VFE_BUS_VER3_VFE_OUT_SPARSE_PD:
	case CAM_VFE_BUS_VER3_VFE_OUT_PREPROCESS_2PD:
	case CAM_VFE_BUS_VER3_VFE_OUT_PDAF_PARSED:
	case CAM_VFE_BUS_VER3_VFE_OUT_PREPROCESS_RAW:
	case CAM_VFE_BUS_VER3_VFE_OUT_RDI4:
	case CAM_VFE_BUS_VER3_VFE_OUT_RDI5:
	case CAM_VFE_BUS_VER3_VFE_OUT_AI_OUT_1:
	case CAM_VFE_BUS_VER3_VFE_OUT_AI_OUT_2:
	case CAM_VFE_BUS_VER3_VFE_OUT_IR:
		return true;

	case CAM_VFE_BUS_VER3_VFE_OUT_FD:
		return cam_secure_get_vfe_fd_port_config();

	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_HDR_BE:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_HDR_BHIST:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_TL_BG:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_BF:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_AWB_BG:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_BHIST:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_RS:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_CS:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_IHIST:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_CAF:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_BAYER_RS:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_ALSC:
	case CAM_VFE_BUS_VER3_VFE_OUT_AWB_BFW:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_AEC_BE:
	case CAM_VFE_BUS_VER3_VFE_OUT_LTM_STATS:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_GTM_BHIST:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_BG:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_IR_BG:
	case CAM_VFE_BUS_VER3_VFE_OUT_STATS_IR_BHIST:
	default:
		return false;
	}
}

static enum cam_vfe_bus_ver3_vfe_out_type
	cam_vfe_bus_ver3_get_out_res_id_and_index(
	struct cam_vfe_bus_ver3_priv  *bus_priv,
	uint32_t res_type, uint32_t  *index)
{
	uint32_t  vfe_out_type;

	switch (res_type) {
	case CAM_ISP_IFE_OUT_RES_FULL:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_FULL;
		break;
	case CAM_ISP_IFE_OUT_RES_DS4:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_DS4;
		break;
	case CAM_ISP_IFE_OUT_RES_DS16:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_DS16;
		break;
	case CAM_ISP_IFE_OUT_RES_FD:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_FD;
		break;
	case CAM_ISP_IFE_OUT_RES_RAW_DUMP:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_RAW_DUMP;
		break;
	case CAM_ISP_IFE_OUT_RES_2PD:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_2PD;
		break;
	case CAM_ISP_IFE_OUT_RES_RDI_0:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_RDI0;
		break;
	case CAM_ISP_IFE_OUT_RES_RDI_1:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_RDI1;
		break;
	case CAM_ISP_IFE_OUT_RES_RDI_2:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_RDI2;
		break;
	case CAM_ISP_IFE_OUT_RES_RDI_3:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_RDI3;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_HDR_BE:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_HDR_BE;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_HDR_BHIST:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_HDR_BHIST;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_TL_BG:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_TL_BG;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_BF:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_BF;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_AWB_BG:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_AWB_BG;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_BHIST:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_BHIST;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_RS:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_RS;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_CS:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_CS;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_IHIST:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_IHIST;
		break;
	case CAM_ISP_IFE_OUT_RES_FULL_DISP:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_FULL_DISP;
		break;
	case CAM_ISP_IFE_OUT_RES_DS4_DISP:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_DS4_DISP;
		break;
	case CAM_ISP_IFE_OUT_RES_DS16_DISP:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_DS16_DISP;
		break;
	case CAM_ISP_IFE_OUT_RES_LCR:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_LCR;
		break;
	case CAM_ISP_IFE_OUT_RES_AWB_BFW:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_AWB_BFW;
		break;
	case CAM_ISP_IFE_OUT_RES_SPARSE_PD:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_SPARSE_PD;
		break;
	case CAM_ISP_IFE_OUT_RES_PREPROCESS_2PD:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_PREPROCESS_2PD;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_AEC_BE:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_AEC_BE;
		break;
	case CAM_ISP_IFE_OUT_RES_LTM_STATS:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_LTM_STATS;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_GTM_BHIST:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_GTM_BHIST;
		break;
	case CAM_ISP_IFE_LITE_OUT_RES_STATS_BG:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_BG;
		break;
	case CAM_ISP_IFE_LITE_OUT_RES_PREPROCESS_RAW:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_PREPROCESS_RAW;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_CAF:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_CAF;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_BAYER_RS:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_BAYER_RS;
		break;
	case CAM_ISP_IFE_OUT_RES_PDAF_PARSED_DATA:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_PDAF_PARSED;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_ALSC:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_ALSC;
		break;
	case CAM_ISP_IFE_OUT_RES_RDI_4:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_RDI4;
		break;
	case CAM_ISP_IFE_OUT_RES_RDI_5:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_RDI5;
		break;
	case CAM_ISP_IFE_OUT_RES_AI_OUT_1:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_AI_OUT_1;
		break;
	case CAM_ISP_IFE_OUT_RES_AI_OUT_2:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_AI_OUT_2;
		break;
	case CAM_ISP_IFE_OUT_RES_IR:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_IR;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_IR_BG:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_IR_BG;
		break;
	case CAM_ISP_IFE_OUT_RES_STATS_IR_BHIST:
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_STATS_IR_BHIST;
		break;
	default:
		CAM_WARN(CAM_ISP, "VFE:%u Invalid isp res id: %d , assigning max",
			bus_priv->common_data.core_index, res_type);
		vfe_out_type = CAM_VFE_BUS_VER3_VFE_OUT_MAX;
		*index = CAM_VFE_BUS_VER3_VFE_OUT_MAX;
		return vfe_out_type;
	}
	*index = bus_priv->vfe_out_map_outtype[vfe_out_type];

	return vfe_out_type;
}

static enum cam_vfe_bus_ver3_packer_format
	cam_vfe_bus_ver3_get_packer_fmt(uint32_t out_fmt, int wm_index)
{
	switch (out_fmt) {
	case CAM_FORMAT_MIPI_RAW_6:
	case CAM_FORMAT_MIPI_RAW_16:
	case CAM_FORMAT_PLAIN16_8:
	case CAM_FORMAT_PLAIN128:
	case CAM_FORMAT_PD8:
		return PACKER_FMT_VER3_PLAIN_128;
	case CAM_FORMAT_MIPI_RAW_8:
	case CAM_FORMAT_PLAIN8:
		return PACKER_FMT_VER3_PLAIN_8;
	case CAM_FORMAT_MIPI_RAW_10:
		return PACKER_FMT_VER3_MIPI10;
	case CAM_FORMAT_MIPI_RAW_12:
		return PACKER_FMT_VER3_MIPI12;
	case CAM_FORMAT_MIPI_RAW_14:
		return PACKER_FMT_VER3_MIPI14;
	case CAM_FORMAT_MIPI_RAW_20:
		return PACKER_FMT_VER3_MIPI20;
	case CAM_FORMAT_NV21:
		if ((wm_index == 1) || (wm_index == 3) || (wm_index == 5))
			return PACKER_FMT_VER3_PLAIN_8_LSB_MSB_10_ODD_EVEN;
		fallthrough;
	case CAM_FORMAT_NV12:
	case CAM_FORMAT_UBWC_NV12:
	case CAM_FORMAT_UBWC_NV12_4R:
	case CAM_FORMAT_Y_ONLY:
		return PACKER_FMT_VER3_PLAIN_8_LSB_MSB_10;
	case CAM_FORMAT_PLAIN16_10:
		return PACKER_FMT_VER3_PLAIN_16_10BPP;
	case CAM_FORMAT_PLAIN16_12:
		return PACKER_FMT_VER3_PLAIN_16_12BPP;
	case CAM_FORMAT_PLAIN16_14:
		return PACKER_FMT_VER3_PLAIN_16_14BPP;
	case CAM_FORMAT_PLAIN16_16:
		return PACKER_FMT_VER3_PLAIN_16_16BPP;
	case CAM_FORMAT_PLAIN32:
	case CAM_FORMAT_ARGB:
		return PACKER_FMT_VER3_PLAIN_32;
	case CAM_FORMAT_PLAIN32_20:
		return PACKER_FMT_VER3_PLAIN32_20BPP;
	case CAM_FORMAT_PLAIN64:
	case CAM_FORMAT_ARGB_16:
	case CAM_FORMAT_PD10:
		return PACKER_FMT_VER3_PLAIN_64;
	case CAM_FORMAT_UBWC_TP10:
	case CAM_FORMAT_TP10:
		return PACKER_FMT_VER3_TP_10;
	default:
		return PACKER_FMT_VER3_MAX;
	}
	return PACKER_FMT_VER3_MAX;
}

static int cam_vfe_bus_ver3_handle_rup_top_half(uint32_t evt_id,
	struct cam_irq_th_payload *th_payload)
{
	int32_t                                     rc;
	int                                         i;
	struct cam_isp_resource_node               *vfe_out = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data       *rsrc_data = NULL;
	struct cam_vfe_bus_irq_evt_payload         *evt_payload;
	uint32_t irq_status;

	vfe_out = th_payload->handler_priv;
	if (!vfe_out) {
		CAM_ERR_RATE_LIMIT(CAM_ISP, "No resource");
		return -ENODEV;
	}

	rsrc_data = vfe_out->res_priv;

	CAM_DBG(CAM_ISP, "VFE:%u Bus IRQ status_0: 0x%X",
		rsrc_data->common_data->core_index,
		th_payload->evt_status_arr[0]);

	rc  = cam_vfe_bus_ver3_get_evt_payload(rsrc_data->common_data,
		&evt_payload);

	if (rc) {
		CAM_INFO_RATE_LIMIT(CAM_ISP,
			"VFE:%u Bus IRQ status_0: 0x%X",
			rsrc_data->common_data->core_index,
			th_payload->evt_status_arr[0]);
		return rc;
	}

	evt_payload->core_index = rsrc_data->common_data->core_index;
	evt_payload->evt_id  = evt_id;
	for (i = 0; i < th_payload->num_registers; i++)
		evt_payload->irq_reg_val[i] = th_payload->evt_status_arr[i];

	irq_status =
		th_payload->evt_status_arr[CAM_IFE_IRQ_BUS_VER3_REG_STATUS0];

	trace_cam_log_event("RUP", "RUP_IRQ", irq_status, 0);

	th_payload->evt_payload_priv = evt_payload;

	return rc;
}

static void cam_vfe_bus_ver3_print_constraint_errors(
	struct cam_vfe_bus_ver3_priv *bus_priv,
	uint8_t *wm_name,
	uint32_t constraint_errors)
{
	uint32_t i;

	CAM_INFO(CAM_ISP, "VFE:%u Constraint violation bitflags: 0x%X",
		bus_priv->common_data.core_index, constraint_errors);

	for (i = 0; i < bus_priv->num_cons_err; i++) {
		if (bus_priv->constraint_error_list[i].bitmask &
			constraint_errors) {
			CAM_INFO(CAM_ISP, "VFE:%u WM:%s %s",
				bus_priv->common_data.core_index, wm_name,
				bus_priv->constraint_error_list[i].error_description);
		}
	}
}

static void cam_vfe_bus_ver3_get_constraint_errors(
	struct cam_vfe_bus_ver3_priv *bus_priv)
{
	uint32_t i, j, constraint_errors;
	uint8_t *wm_name = NULL;
	struct cam_isp_resource_node              *out_rsrc_node = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data      *out_rsrc_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data  *wm_data   = NULL;

	for (i = 0; i < bus_priv->num_out; i++) {
		out_rsrc_node = &bus_priv->vfe_out[i];
		if (!out_rsrc_node || !out_rsrc_node->res_priv) {
			CAM_DBG(CAM_ISP,
				"VFE:%u out:%d out rsrc node or data is NULL",
				bus_priv->common_data.core_index, i);
			continue;
		}
		out_rsrc_data = out_rsrc_node->res_priv;
		for (j = 0; j < out_rsrc_data->num_wm; j++) {
			wm_data = out_rsrc_data->wm_res[j].res_priv;
			wm_name = out_rsrc_data->wm_res[j].res_name;
			if (wm_data) {
				constraint_errors = cam_io_r_mb(
					bus_priv->common_data.mem_base +
					wm_data->hw_regs->debug_status_1);
				if (!constraint_errors)
					continue;

				cam_vfe_bus_ver3_print_constraint_errors(
					bus_priv, wm_name, constraint_errors);
			}
		}
	}
}

static int cam_vfe_bus_ver3_handle_rup_bottom_half(void *handler_priv,
	void *evt_payload_priv)
{
	int                                   ret = CAM_VFE_IRQ_STATUS_ERR;
	struct cam_vfe_bus_irq_evt_payload   *payload;
	struct cam_isp_resource_node         *vfe_out = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data *rsrc_data = NULL;
	struct cam_isp_hw_event_info          evt_info;
	uint32_t                              irq_status;

	if (!handler_priv || !evt_payload_priv) {
		CAM_ERR(CAM_ISP, "Invalid params");
		return ret;
	}

	payload = evt_payload_priv;
	vfe_out = handler_priv;
	rsrc_data = vfe_out->res_priv;

	if (!rsrc_data->common_data->event_cb) {
		CAM_ERR(CAM_ISP, "VFE:%u Callback to HW MGR not found", vfe_out->hw_intf->hw_idx);
		return ret;
	}

	irq_status = payload->irq_reg_val[CAM_IFE_IRQ_BUS_VER3_REG_STATUS0];

	evt_info.hw_type  = CAM_ISP_HW_TYPE_VFE;
	evt_info.hw_idx = rsrc_data->common_data->core_index;
	evt_info.res_type = CAM_ISP_RESOURCE_VFE_IN;

	if (!rsrc_data->common_data->is_lite) {
		if (irq_status & 0x1) {
			CAM_DBG(CAM_ISP, "VFE:%u Received CAMIF RUP",
				evt_info.hw_idx);
			evt_info.res_id = CAM_ISP_HW_VFE_IN_CAMIF;
			rsrc_data->common_data->event_cb(
				rsrc_data->priv, CAM_ISP_HW_EVENT_REG_UPDATE,
				(void *)&evt_info);
		}

		if (irq_status & 0x2) {
			CAM_DBG(CAM_ISP, "VFE:%u Received PDLIB RUP",
				evt_info.hw_idx);
			evt_info.res_id = CAM_ISP_HW_VFE_IN_PDLIB;
			rsrc_data->common_data->event_cb(
				rsrc_data->priv, CAM_ISP_HW_EVENT_REG_UPDATE,
				(void *)&evt_info);
		}

		if (irq_status & 0x4)
			CAM_DBG(CAM_ISP, "VFE:%u Received LCR RUP",
				evt_info.hw_idx);

		if (irq_status & 0x8) {
			CAM_DBG(CAM_ISP, "VFE:%u Received RDI0 RUP",
				evt_info.hw_idx);
			evt_info.res_id = CAM_ISP_HW_VFE_IN_RDI0;
			rsrc_data->common_data->event_cb(
				rsrc_data->priv, CAM_ISP_HW_EVENT_REG_UPDATE,
				(void *)&evt_info);
		}

		if (irq_status & 0x10) {
			CAM_DBG(CAM_ISP, "VFE:%u Received RDI1 RUP",
				evt_info.hw_idx);
			evt_info.res_id = CAM_ISP_HW_VFE_IN_RDI1;
			rsrc_data->common_data->event_cb(
				rsrc_data->priv, CAM_ISP_HW_EVENT_REG_UPDATE,
				(void *)&evt_info);
		}

		if (irq_status & 0x20) {
			CAM_DBG(CAM_ISP, "VFE:%u Received RDI2 RUP",
				evt_info.hw_idx);
			evt_info.res_id = CAM_ISP_HW_VFE_IN_RDI2;
			rsrc_data->common_data->event_cb(
				rsrc_data->priv, CAM_ISP_HW_EVENT_REG_UPDATE,
				(void *)&evt_info);
		}
	} else {
		if (irq_status & 0x1) {
			CAM_DBG(CAM_ISP, "VFE:%u Received RDI0 RUP",
				evt_info.hw_idx);
			evt_info.res_id = CAM_ISP_HW_VFE_IN_RDI0;
			rsrc_data->common_data->event_cb(
				rsrc_data->priv, CAM_ISP_HW_EVENT_REG_UPDATE,
				(void *)&evt_info);
		}

		if (irq_status & 0x2) {
			CAM_DBG(CAM_ISP, "VFE:%u Received RDI1 RUP",
				evt_info.hw_idx);
			evt_info.res_id = CAM_ISP_HW_VFE_IN_RDI1;
			rsrc_data->common_data->event_cb(
				rsrc_data->priv, CAM_ISP_HW_EVENT_REG_UPDATE,
				(void *)&evt_info);
		}

		if (irq_status & 0x4) {
			CAM_DBG(CAM_ISP, "VFE:%u Received RDI2 RUP",
				evt_info.hw_idx);
			evt_info.res_id = CAM_ISP_HW_VFE_IN_RDI2;
			rsrc_data->common_data->event_cb(
				rsrc_data->priv, CAM_ISP_HW_EVENT_REG_UPDATE,
				(void *)&evt_info);
		}

		if (irq_status & 0x8) {
			CAM_DBG(CAM_ISP, "VFE:%u Received RDI3 RUP",
				evt_info.hw_idx);
			evt_info.res_id = CAM_ISP_HW_VFE_IN_RDI3;
			rsrc_data->common_data->event_cb(
				rsrc_data->priv, CAM_ISP_HW_EVENT_REG_UPDATE,
				(void *)&evt_info);
		}
	}

	ret = CAM_VFE_IRQ_STATUS_SUCCESS;

	CAM_DBG(CAM_ISP,
		"VFE:%u Bus RUP IRQ status_0:0x%X rc:%d",
		evt_info.hw_idx, CAM_ISP_HW_EVENT_REG_UPDATE, irq_status, ret);

	cam_vfe_bus_ver3_put_evt_payload(rsrc_data->common_data, &payload);

	return ret;
}

static inline void cam_vfe_bus_ver3_config_frame_based_rdi_wm(
	struct cam_vfe_bus_ver3_wm_resource_data  *rsrc_data)
{
	rsrc_data->width = CAM_VFE_RDI_BUS_DEFAULT_WIDTH;
	rsrc_data->height = 0;
	rsrc_data->stride = CAM_VFE_RDI_BUS_DEFAULT_STRIDE;
	rsrc_data->en_cfg = (0x1 << 16) | 0x1;
}

static int cam_vfe_bus_ver3_config_rdi_wm(
	struct cam_vfe_bus_ver3_wm_resource_data  *rsrc_data)
{

	rsrc_data->pack_fmt = PACKER_FMT_VER3_PLAIN_128;

	if (rsrc_data->wm_mode == CAM_VFE_WM_FRAME_BASED_MODE)
		cam_vfe_bus_ver3_config_frame_based_rdi_wm(rsrc_data);
	else if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
		 rsrc_data->en_cfg = 0x1;
	else {
		CAM_WARN(CAM_ISP, "No index mode %d is supported for VFE: %u  WM: %u",
			rsrc_data->wm_mode,
			rsrc_data->common_data->core_index,
			rsrc_data->index);
		return 0;
	}

	switch (rsrc_data->format) {
	case CAM_FORMAT_MIPI_RAW_10:
		if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
			rsrc_data->width = ALIGNUP((rsrc_data->width * 5) / 4, 16) / 16;

		if (rsrc_data->use_wm_pack) {
			rsrc_data->pack_fmt = PACKER_FMT_VER3_MIPI10;
			if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
				rsrc_data->width = ALIGNUP((rsrc_data->acquired_width), 8);
		}
		break;
	case CAM_FORMAT_MIPI_RAW_6:
		if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
			rsrc_data->width =
				ALIGNUP((rsrc_data->width * 3) / 4, 16) / 16;
		break;
	case CAM_FORMAT_MIPI_RAW_8:
	case CAM_FORMAT_YUV422:
		if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
			rsrc_data->width =
				ALIGNUP(rsrc_data->width, 16) / 16;
		break;
	case CAM_FORMAT_MIPI_RAW_12:
		if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
			rsrc_data->width =
				ALIGNUP((rsrc_data->width * 3) / 2, 16) / 16;

		if (rsrc_data->use_wm_pack) {
			rsrc_data->pack_fmt = PACKER_FMT_VER3_MIPI12;
			if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
				rsrc_data->width = ALIGNUP((rsrc_data->acquired_width), 8);
		}
		break;
	case CAM_FORMAT_MIPI_RAW_14:
		if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
			rsrc_data->width =
				ALIGNUP((rsrc_data->width * 7) / 2, 16) / 16;

		if (rsrc_data->use_wm_pack) {
			rsrc_data->pack_fmt = PACKER_FMT_VER3_MIPI14;
			if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
				rsrc_data->width = ALIGNUP((rsrc_data->acquired_width), 8);
		}
		break;
	case CAM_FORMAT_MIPI_RAW_16:
		if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
			rsrc_data->width =
				ALIGNUP((rsrc_data->width * 2), 16) / 16;
		break;
	case CAM_FORMAT_MIPI_RAW_20:
		if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
			rsrc_data->width =
				ALIGNUP((rsrc_data->width * 5) / 2, 16) / 16;

		if (rsrc_data->use_wm_pack)
			rsrc_data->pack_fmt = PACKER_FMT_VER3_MIPI20;
		break;
	case CAM_FORMAT_PLAIN128:
		if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
			rsrc_data->width =
				ALIGNUP((rsrc_data->width * 16), 16) / 16;
		break;
	case CAM_FORMAT_PLAIN32_20:
		if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
			rsrc_data->width =
				ALIGNUP((rsrc_data->width * 4), 16) / 16;
		break;
	case CAM_FORMAT_PLAIN8:
		rsrc_data->en_cfg = 0x1;
		rsrc_data->stride = rsrc_data->width * 2;
		break;
	case CAM_FORMAT_PLAIN16_10:
	case CAM_FORMAT_PLAIN16_12:
	case CAM_FORMAT_PLAIN16_14:
	case CAM_FORMAT_PLAIN16_16:
		rsrc_data->width =
			ALIGNUP(rsrc_data->width * 2, 16) / 16;
		rsrc_data->en_cfg = 0x1;

		if (rsrc_data->use_wm_pack) {
			rsrc_data->pack_fmt = cam_vfe_bus_ver3_get_packer_fmt(rsrc_data->format,
				rsrc_data->index);
			/* LSB aligned */
			rsrc_data->pack_fmt |= (1 <<
				rsrc_data->common_data->pack_align_shift);

			if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
				rsrc_data->width = ALIGNUP((rsrc_data->acquired_width), 8);
		}
		break;
	case CAM_FORMAT_PLAIN64:
		rsrc_data->width =
			ALIGNUP(rsrc_data->width * 8, 16) / 16;
		rsrc_data->en_cfg = 0x1;
		break;
	case CAM_FORMAT_YUV422_10:
		if (rsrc_data->wm_mode == CAM_VFE_WM_LINE_BASED_MODE)
			rsrc_data->width =
				ALIGNUP((rsrc_data->width * 5) / 4, 16) / 16;
		break;
	default:
		CAM_ERR(CAM_ISP, "VFE:%u Unsupported RDI format %d",
			rsrc_data->common_data->core_index, rsrc_data->format);
		return -EINVAL;
	}

	return 0;
}

static int cam_vfe_bus_ver3_res_update_config_wm(
	struct cam_vfe_bus_ver3_priv           *ver3_bus_priv,
	enum cam_vfe_bus_ver3_vfe_out_type      vfe_out_res_id,
	enum cam_vfe_bus_plane_type             plane,
	struct cam_isp_resource_node           *wm_res,
	enum cam_vfe_bus_ver3_comp_grp_type    *comp_grp_id,
	char                                   *wm_mode,
	int32_t                                 wm_mode_size)
{
	int32_t rc = 0;
	struct cam_vfe_bus_ver3_wm_resource_data  *rsrc_data = NULL;

	rsrc_data = wm_res->res_priv;

	if (((vfe_out_res_id >= CAM_VFE_BUS_VER3_VFE_OUT_RDI0) &&
		(vfe_out_res_id <= CAM_VFE_BUS_VER3_VFE_OUT_RDI5))) {
		if (rsrc_data->default_line_based)
			rsrc_data->wm_mode = CAM_VFE_WM_LINE_BASED_MODE;
		else
			rsrc_data->wm_mode = CAM_VFE_WM_FRAME_BASED_MODE;

		rc = cam_vfe_bus_ver3_config_rdi_wm(rsrc_data);
		if (rc)
			return rc;
	} else if (vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_RAW_DUMP ||
			(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_IR)) {

		rsrc_data->stride = rsrc_data->width;
		rsrc_data->en_cfg = 0x1;
		/* LSB aligned */
		rsrc_data->pack_fmt |= (1 <<
				ver3_bus_priv->common_data.pack_align_shift);

	} else if ((vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_FULL) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_AI_OUT_1) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_AI_OUT_2) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_FD) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_FULL_DISP)) {

		switch (rsrc_data->format) {
		case CAM_FORMAT_UBWC_NV12_4R:
			rsrc_data->en_ubwc = 1;
			switch (plane) {
			case PLANE_C:
				rsrc_data->height /= 2;
				break;
			case PLANE_Y:
				break;
			default:
				CAM_ERR(CAM_ISP, "VFE:%u Invalid plane %d",
					wm_res->hw_intf->hw_idx, plane);
				return -EINVAL;
			}
			break;
		case CAM_FORMAT_UBWC_NV12:
			rsrc_data->en_ubwc = 1;
			fallthrough;
			/* Fall through for NV12 */
		case CAM_FORMAT_NV21:
		case CAM_FORMAT_NV12:
		case CAM_FORMAT_Y_ONLY:
			switch (plane) {
			case PLANE_C:
				rsrc_data->height /= 2;
				break;
			case PLANE_Y:
				break;
			default:
				CAM_ERR(CAM_ISP, "VFE:%u Invalid plane %d",
					wm_res->hw_intf->hw_idx, plane);
				return -EINVAL;
			}
			break;
		case CAM_FORMAT_UBWC_TP10:
			rsrc_data->en_ubwc = 1;
			switch (plane) {
			case PLANE_C:
				rsrc_data->height /= 2;
				break;
			case PLANE_Y:
				break;
			default:
				CAM_ERR(CAM_ISP, "VFE:%u Invalid plane %d",
					wm_res->hw_intf->hw_idx, plane);
				return -EINVAL;
			}
			break;
		case CAM_FORMAT_TP10:
			switch (plane) {
			case PLANE_C:
				rsrc_data->height /= 2;
				break;
			case PLANE_Y:
				break;
			default:
				CAM_ERR(CAM_ISP, "VFE:%u Invalid plane %d",
					wm_res->hw_intf->hw_idx, plane);
				return -EINVAL;
			}
			break;
		case CAM_FORMAT_PLAIN16_10:
			switch (plane) {
			case PLANE_C:
				rsrc_data->height /= 2;
				break;
			case PLANE_Y:
				break;
			default:
				CAM_ERR(CAM_ISP, "VFE:%u Invalid plane %d",
					wm_res->hw_intf->hw_idx, plane);
				return -EINVAL;
			}
			break;
		default:
			if (wm_res->is_per_port_acquire) {
				rsrc_data->height /= 2;
				break;
			}
			CAM_ERR(CAM_ISP, "VFE:%u Invalid format %d out_type:%d",
				wm_res->hw_intf->hw_idx, rsrc_data->format, vfe_out_res_id);
			return -EINVAL;
		}
		rsrc_data->en_cfg = 0x1;

	} else if ((vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_STATS_BF) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_STATS_CAF)) {

		rsrc_data->en_cfg = (0x1 << 16) | 0x1;

	} else if (((vfe_out_res_id >= CAM_VFE_BUS_VER3_VFE_OUT_STATS_HDR_BE) &&
		(vfe_out_res_id <= CAM_VFE_BUS_VER3_VFE_OUT_STATS_IHIST)) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_STATS_BAYER_RS)) {

		rsrc_data->width = 0;
		rsrc_data->height = 0;
		rsrc_data->stride = 1;
		rsrc_data->en_cfg = (0x1 << 16) | 0x1;

	} else if (vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_2PD) {

		rsrc_data->width = 0;
		rsrc_data->height = 0;
		rsrc_data->stride = 1;
		rsrc_data->en_cfg = (0x1 << 16) | 0x1;

	} else if (vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_LCR) {
		switch (rsrc_data->format) {
		case CAM_FORMAT_PLAIN16_16:
			rsrc_data->stride = ALIGNUP(rsrc_data->width * 2, 8);
			rsrc_data->en_cfg = 0x1;
			/* LSB aligned */
			rsrc_data->pack_fmt |= (1 <<
				ver3_bus_priv->common_data.pack_align_shift);

			break;
		default:
			if (wm_res->is_per_port_acquire) {
				rsrc_data->stride = ALIGNUP(rsrc_data->width * 2, 8);
				rsrc_data->en_cfg = 0x1;
				/* LSB aligned */
				rsrc_data->pack_fmt |= (1 <<
					ver3_bus_priv->common_data.pack_align_shift);
				break;
			}
			CAM_ERR(CAM_ISP, "VFE:%u Invalid format %d out_type:%d",
				wm_res->hw_intf->hw_idx, rsrc_data->format, vfe_out_res_id);
			return -EINVAL;
		}

	} else if (vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_SPARSE_PD) {
		switch (rsrc_data->format) {
		case CAM_FORMAT_PLAIN8:
			rsrc_data->stride = ALIGNUP(rsrc_data->width * 2, 8);
			rsrc_data->en_cfg = 0x1;
			break;
		case CAM_FORMAT_PLAIN16_8:
		case CAM_FORMAT_PLAIN16_10:
		case CAM_FORMAT_PLAIN16_12:
		case CAM_FORMAT_PLAIN16_14:
			rsrc_data->stride = ALIGNUP(rsrc_data->width * 2, 8);
			rsrc_data->en_cfg = 0x1;
			/* LSB aligned */
			rsrc_data->pack_fmt |= (1 <<
				ver3_bus_priv->common_data.pack_align_shift);
			break;
		default:
			if (wm_res->is_per_port_acquire) {
				rsrc_data->stride = ALIGNUP(rsrc_data->width * 2, 8);
				rsrc_data->en_cfg = 0x1;
				break;
			}
			CAM_ERR(CAM_ISP, "VFE:%u Invalid format %d out_type:%d",
				wm_res->hw_intf->hw_idx, rsrc_data->format, vfe_out_res_id);
			return -EINVAL;
		}

	} else if ((vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_DS4) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_DS16) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_DS4_DISP) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_DS16_DISP)) {

		rsrc_data->height = rsrc_data->height / 2;
		rsrc_data->width  = rsrc_data->width / 2;
		rsrc_data->en_cfg = 0x1;

	} else if ((vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_AWB_BFW) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_STATS_ALSC)) {
		switch (rsrc_data->format) {
		case CAM_FORMAT_PLAIN64:
			rsrc_data->width = 0;
			rsrc_data->height = 0;
			rsrc_data->stride = 1;
			rsrc_data->en_cfg = (0x1 << 16) | 0x1;
			break;
		default:
			if (wm_res->is_per_port_acquire) {
				rsrc_data->width = 0;
				rsrc_data->height = 0;
				rsrc_data->stride = 1;
				rsrc_data->en_cfg = (0x1 << 16) | 0x1;
				break;
			}
			CAM_ERR(CAM_ISP, "VFE:%u Invalid format %d out_type:%d",
				wm_res->hw_intf->hw_idx, rsrc_data->format, vfe_out_res_id);
			return -EINVAL;
		}

	} else if (vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_PREPROCESS_2PD) {
		switch (rsrc_data->format) {
		case CAM_FORMAT_PLAIN16_16:
			rsrc_data->stride = ALIGNUP(rsrc_data->width * 2, 8);
			rsrc_data->en_cfg = 0x1;
			break;
		default:
			if (wm_res->is_per_port_acquire) {
				rsrc_data->stride = ALIGNUP(rsrc_data->width * 2, 8);
				rsrc_data->en_cfg = 0x1;
				break;
			}
			CAM_ERR(CAM_ISP, "VFE:%u Invalid format %d out_type:%d",
				wm_res->hw_intf->hw_idx, rsrc_data->format, vfe_out_res_id);
			return -EINVAL;
		}

	} else if (vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_LTM_STATS) {
		switch (rsrc_data->format) {
		case CAM_FORMAT_PLAIN32:
			rsrc_data->stride = ALIGNUP(rsrc_data->width * 4, 16);
			rsrc_data->en_cfg = 0x1;
			break;
		default:
			if (wm_res->is_per_port_acquire) {
				rsrc_data->stride = ALIGNUP(rsrc_data->width * 4, 16);
				rsrc_data->en_cfg = 0x1;
				break;
			}
			CAM_ERR(CAM_ISP, "VFE:%u Invalid format %d out_type:%d",
				wm_res->hw_intf->hw_idx, rsrc_data->format, vfe_out_res_id);
			return -EINVAL;
		}

	} else if ((vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_STATS_AEC_BE) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_STATS_GTM_BHIST) ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_STATS_IR_BHIST)  ||
		(vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_STATS_IR_BG)) {
		rsrc_data->width = 0;
		rsrc_data->height = 0;
		rsrc_data->stride = 1;
		rsrc_data->en_cfg = (0x1 << 16) | 0x1;
	} else if (vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_PREPROCESS_RAW) {
		switch (rsrc_data->format) {
		case CAM_FORMAT_MIPI_RAW_8:
		case CAM_FORMAT_MIPI_RAW_10:
		case CAM_FORMAT_MIPI_RAW_12:
		case CAM_FORMAT_PLAIN8:
		case CAM_FORMAT_PLAIN16_8:
		case CAM_FORMAT_PLAIN16_10:
		case CAM_FORMAT_PLAIN16_12:
		case CAM_FORMAT_PLAIN16_14:
			rsrc_data->width = 0;
			rsrc_data->height = 0;
			rsrc_data->stride = 1;
			rsrc_data->en_cfg = (0x1 << 16) | 0x1;
			break;
		default:
			if (wm_res->is_per_port_acquire) {
				rsrc_data->width = 0;
				rsrc_data->height = 0;
				rsrc_data->stride = 1;
				rsrc_data->en_cfg = (0x1 << 16) | 0x1;
				break;
			}
			CAM_ERR(CAM_ISP, "VFE:%u Invalid format %d out_type:%d",
				wm_res->hw_intf->hw_idx, rsrc_data->format, vfe_out_res_id);
			return -EINVAL;
		}

	} else if (vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_STATS_BG) {
		switch (rsrc_data->format) {
		case CAM_FORMAT_PLAIN64:
			rsrc_data->width = 0;
			rsrc_data->height = 0;
			rsrc_data->stride = 1;
			rsrc_data->en_cfg = (0x1 << 16) | 0x1;
			break;
		default:
			if (wm_res->is_per_port_acquire) {
				rsrc_data->width = 0;
				rsrc_data->height = 0;
				rsrc_data->stride = 1;
				rsrc_data->en_cfg = (0x1 << 16) | 0x1;
				break;
			}
			CAM_ERR(CAM_ISP, "VFE:%u Invalid format %d out_type:%d",
				 wm_res->hw_intf->hw_idx, rsrc_data->format, vfe_out_res_id);
			return -EINVAL;
		}

	} else if (vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_PDAF_PARSED) {
		switch (rsrc_data->format) {
		case CAM_FORMAT_PLAIN16_16:
			rsrc_data->stride = ALIGNUP(rsrc_data->width * 2, 8);
			rsrc_data->en_cfg = 0x1;
			/* LSB aligned */
			rsrc_data->pack_fmt |= (1 <<
				ver3_bus_priv->common_data.pack_align_shift);

			break;
		default:
			if (wm_res->is_per_port_acquire) {
				rsrc_data->stride = ALIGNUP(rsrc_data->width * 2, 8);
				rsrc_data->en_cfg = 0x1;
				/* LSB aligned */
				rsrc_data->pack_fmt |= (1 <<
					ver3_bus_priv->common_data.pack_align_shift);
				break;
			}
			CAM_ERR(CAM_ISP, "VFE:%u Invalid format %d out_type:%d",
				wm_res->hw_intf->hw_idx, rsrc_data->format, vfe_out_res_id);
			return -EINVAL;
		}

	} else {
		CAM_ERR(CAM_ISP, "VFE:%u Invalid out_type:%d requested",
			wm_res->hw_intf->hw_idx, vfe_out_res_id);
		return -EINVAL;
	}

	*comp_grp_id = rsrc_data->hw_regs->comp_group;

	switch (rsrc_data->en_cfg) {
	case 0x1:
		strscpy(wm_mode, "line-based", sizeof(wm_mode));
		break;
	case ((0x1 << 16) | 0x1):
		strscpy(wm_mode, "frame-based", sizeof(wm_mode));
		break;
	case ((0x2 << 16) | 0x1):
		strscpy(wm_mode, "index-based", sizeof(wm_mode));
		break;
	}

	return rc;
}

static bool cam_vfe_bus_ver3_match_comp_grp(
	struct cam_vfe_bus_ver3_priv           *ver3_bus_priv,
	struct cam_isp_resource_node          **comp_grp,
	uint32_t                                comp_grp_id)
{
	struct cam_vfe_bus_ver3_comp_grp_data  *rsrc_data = NULL;
	struct cam_isp_resource_node           *comp_grp_local = NULL;

	if (!list_empty(&ver3_bus_priv->used_comp_grp)) {
		list_for_each_entry(comp_grp_local,
			&ver3_bus_priv->used_comp_grp, list) {
			rsrc_data = comp_grp_local->res_priv;
			if (rsrc_data->comp_grp_type == comp_grp_id) {
				/* Match found */
				*comp_grp = comp_grp_local;
				return true;
			}
		}
	}

	list_for_each_entry(comp_grp_local,
		&ver3_bus_priv->free_comp_grp, list) {
		rsrc_data = comp_grp_local->res_priv;
		if (rsrc_data->comp_grp_type == comp_grp_id) {
			/* Match found */
			*comp_grp = comp_grp_local;
			list_del(&comp_grp_local->list);
			list_add_tail(&comp_grp_local->list,
			&ver3_bus_priv->used_comp_grp);
			return false;
		}
	}

	*comp_grp = NULL;
	return false;
}

static int cam_vfe_bus_ver3_acquire_wm(
	struct cam_vfe_bus_ver3_priv           *ver3_bus_priv,
	struct cam_vfe_hw_vfe_out_acquire_args *out_acq_args,
	void                                   *tasklet,
	enum cam_vfe_bus_ver3_vfe_out_type      vfe_out_res_id,
	enum cam_vfe_bus_plane_type             plane,
	struct cam_isp_resource_node           *wm_res,
	enum cam_vfe_bus_ver3_comp_grp_type   *comp_grp_id,
	bool                                   is_per_port_acquire,
	bool                                   update_only)
{
	int32_t wm_idx = 0, rc;
	struct cam_vfe_bus_ver3_wm_resource_data  *rsrc_data = NULL;
	char wm_mode[50] = {'\0'};

	if (wm_res->res_state != CAM_ISP_RESOURCE_STATE_AVAILABLE) {
		CAM_ERR(CAM_ISP, "VFE:%u WM:%d not available state:%d",
			wm_res->hw_intf->hw_idx, wm_idx, wm_res->res_state);
		return -EALREADY;
	}

	rsrc_data = wm_res->res_priv;
	wm_idx = rsrc_data->index;
	rsrc_data->format = out_acq_args->out_port_info->format;
	rsrc_data->use_wm_pack = out_acq_args->use_wm_pack;
	rsrc_data->pack_fmt = cam_vfe_bus_ver3_get_packer_fmt(rsrc_data->format,
		wm_idx);

	rsrc_data->width = out_acq_args->out_port_info->width;
	rsrc_data->height = out_acq_args->out_port_info->height;
	rsrc_data->acquired_width = out_acq_args->out_port_info->width;
	rsrc_data->acquired_height = out_acq_args->out_port_info->height;
	rsrc_data->is_dual = out_acq_args->is_dual;

	/* Set WM offset value to default */
	rsrc_data->offset  = 0;
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d width %d height %d per_port_en %d",
		wm_res->hw_intf->hw_idx, rsrc_data->index,
		rsrc_data->width, rsrc_data->height, is_per_port_acquire);

	if (is_per_port_acquire)
		wm_res->is_per_port_acquire = true;

	rc = cam_vfe_bus_ver3_res_update_config_wm(ver3_bus_priv, vfe_out_res_id,
		plane, wm_res, comp_grp_id, wm_mode, sizeof(wm_mode));
		if (rc)
			return rc;

	wm_res->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
	wm_res->tasklet_info = tasklet;

	CAM_DBG(CAM_ISP,
		"VFE:%u WM:%d %s processed width:%d height:%d stride:%d format:0x%X en_ubwc:%d %s",
		rsrc_data->common_data->core_index, rsrc_data->index,
		wm_res->res_name, rsrc_data->width, rsrc_data->height,
		rsrc_data->stride, rsrc_data->format, rsrc_data->en_ubwc,
		wm_mode);
	return 0;
}

static int cam_vfe_bus_ver3_release_wm(void   *bus_priv,
	struct cam_isp_resource_node     *wm_res)
{
	struct cam_vfe_bus_ver3_wm_resource_data   *rsrc_data =
		wm_res->res_priv;

	rsrc_data->offset = 0;
	rsrc_data->width = 0;
	rsrc_data->height = 0;
	rsrc_data->stride = 0;
	rsrc_data->format = 0;
	rsrc_data->pack_fmt = 0;
	rsrc_data->burst_len = 0;
	rsrc_data->irq_subsample_period = 0;
	rsrc_data->irq_subsample_pattern = 0;
	rsrc_data->framedrop_period = 0;
	rsrc_data->framedrop_pattern = 0;
	rsrc_data->packer_cfg = 0;
	rsrc_data->en_ubwc = 0;
	rsrc_data->h_init = 0;
	rsrc_data->ubwc_meta_addr = 0;
	rsrc_data->ubwc_meta_cfg = 0;
	rsrc_data->ubwc_mode_cfg = 0;
	rsrc_data->ubwc_stats_ctrl = 0;
	rsrc_data->ubwc_ctrl_2 = 0;
	rsrc_data->init_cfg_done = false;
	rsrc_data->hfr_cfg_done = false;
	rsrc_data->ubwc_updated = false;
	rsrc_data->en_cfg = 0;
	rsrc_data->is_dual = 0;

	rsrc_data->ubwc_lossy_threshold_0 = 0;
	rsrc_data->ubwc_lossy_threshold_1 = 0;
	rsrc_data->ubwc_offset_lossy_variance = 0;
	rsrc_data->ubwc_bandwidth_limit = 0;
	wm_res->tasklet_info = NULL;
	wm_res->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;

	CAM_DBG(CAM_ISP, "VFE:%u Release WM:%d %s",
		rsrc_data->common_data->core_index, rsrc_data->index,
		wm_res->res_name);

	return 0;
}

static int cam_vfe_bus_ver3_start_wm(struct cam_isp_resource_node *wm_res)
{
	const uint32_t enable_debug_status_1 = 11 << 8;
	int val = 0;
	struct cam_vfe_bus_ver3_wm_resource_data   *rsrc_data =
		wm_res->res_priv;
	struct cam_vfe_bus_ver3_common_data        *common_data =
		rsrc_data->common_data;
	struct cam_vfe_bus_ver3_reg_offset_ubwc_client *ubwc_regs;
	bool disable_ubwc_comp = rsrc_data->common_data->disable_ubwc_comp;

	ubwc_regs = (struct cam_vfe_bus_ver3_reg_offset_ubwc_client *)
		rsrc_data->hw_regs->ubwc_regs;

	cam_io_w(0xf, common_data->mem_base + rsrc_data->hw_regs->burst_limit);

	cam_io_w((rsrc_data->height << 16) | rsrc_data->width,
		common_data->mem_base + rsrc_data->hw_regs->image_cfg_0);
	cam_io_w(rsrc_data->pack_fmt,
		common_data->mem_base + rsrc_data->hw_regs->packer_cfg);

	/* enable ubwc if needed*/
	if (rsrc_data->en_ubwc) {
		if (!ubwc_regs) {
			CAM_ERR(CAM_ISP,
				"ubwc_regs is NULL, VFE:%u WM:%d en_ubwc:%d",
				rsrc_data->common_data->core_index,
				rsrc_data->index, rsrc_data->en_ubwc);
			return -EINVAL;
		}

		if (rsrc_data->ubwc_updated) {
			cam_vfe_bus_ver3_config_ubwc_regs(rsrc_data);
			rsrc_data->ubwc_updated = false;
		}

		val = cam_io_r_mb(common_data->mem_base + ubwc_regs->mode_cfg);
		val |= 0x1;
		if (disable_ubwc_comp) {
			val &= ~ubwc_regs->ubwc_comp_en_bit;
			CAM_DBG(CAM_ISP,
				"Force disable UBWC compression, VFE:%u WM:%d ubwc_mode_cfg: 0x%x",
				rsrc_data->common_data->core_index,
				rsrc_data->index, val);
		}
		cam_io_w_mb(val, common_data->mem_base + ubwc_regs->mode_cfg);
	}

	/* Validate for debugfs and mmu reg info for targets that don't list it */
	if (!(common_data->disable_mmu_prefetch) &&
		(rsrc_data->hw_regs->mmu_prefetch_cfg)) {
		cam_io_w_mb(1, common_data->mem_base +
			rsrc_data->hw_regs->mmu_prefetch_cfg);
		cam_io_w_mb(0xFFFFFFFF, common_data->mem_base +
			rsrc_data->hw_regs->mmu_prefetch_max_offset);
		CAM_DBG(CAM_ISP, "VFE: %u WM: %d MMU prefetch enabled",
			rsrc_data->common_data->core_index,
			rsrc_data->index);
	}

	/* Enable WM */
	cam_io_w_mb(rsrc_data->en_cfg, common_data->mem_base +
		rsrc_data->hw_regs->cfg);

	/* Enable constraint error detection */
	cam_io_w_mb(enable_debug_status_1,
		common_data->mem_base +
		rsrc_data->hw_regs->debug_status_cfg);

	CAM_DBG(CAM_ISP,
		"Start VFE:%u WM:%d %s offset:0x%X en_cfg:0x%X width:%d height:%d",
		rsrc_data->common_data->core_index, rsrc_data->index,
		wm_res->res_name, (uint32_t) rsrc_data->hw_regs->cfg,
		rsrc_data->en_cfg, rsrc_data->width, rsrc_data->height);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d pk_fmt:%d stride:%d burst len:%d",
		rsrc_data->common_data->core_index, rsrc_data->index, rsrc_data->pack_fmt,
		rsrc_data->stride, 0xF);

	wm_res->res_state = CAM_ISP_RESOURCE_STATE_STREAMING;

	return 0;
}

static int cam_vfe_bus_ver3_stop_wm(struct cam_isp_resource_node *wm_res)
{
	struct cam_vfe_bus_ver3_wm_resource_data   *rsrc_data =
		wm_res->res_priv;
	struct cam_vfe_bus_ver3_common_data        *common_data =
		rsrc_data->common_data;

	/* Disable WM */
	cam_io_w_mb(0x0, common_data->mem_base + rsrc_data->hw_regs->cfg);
	CAM_DBG(CAM_ISP, "Stop VFE:%u WM:%d %s",
		rsrc_data->common_data->core_index, rsrc_data->index,
		wm_res->res_name);

	wm_res->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
	rsrc_data->init_cfg_done = false;
	rsrc_data->hfr_cfg_done = false;
	rsrc_data->ubwc_updated = false;
	rsrc_data->update_wm_format = false;

	return 0;
}

static int cam_vfe_bus_ver3_handle_wm_done_top_half(uint32_t evt_id,
	struct cam_irq_th_payload *th_payload)
{
	return -EPERM;
}

static int cam_vfe_bus_ver3_handle_wm_done_bottom_half(void *wm_node,
	void *evt_payload_priv)
{
	return -EPERM;
}

static int cam_vfe_bus_ver3_init_wm_resource(uint32_t index,
	struct cam_vfe_bus_ver3_priv    *ver3_bus_priv,
	struct cam_vfe_bus_ver3_hw_info *ver3_hw_info,
	struct cam_isp_resource_node    *wm_res,
	struct cam_isp_resource_node    **comp_grp,
	uint8_t                         *wm_name,
	uint32_t                         line_based_config)
{
	struct cam_vfe_bus_ver3_wm_resource_data *rsrc_data;

	rsrc_data = kzalloc(sizeof(struct cam_vfe_bus_ver3_wm_resource_data),
		GFP_KERNEL);
	if (!rsrc_data) {
		CAM_DBG(CAM_ISP, "VFE:%u Failed to alloc for WM res priv",
			ver3_bus_priv->common_data.hw_intf->hw_idx);
		return -ENOMEM;
	}
	wm_res->res_priv = rsrc_data;

	rsrc_data->index = index;
	rsrc_data->default_line_based = line_based_config;
	rsrc_data->hw_regs = &ver3_hw_info->bus_client_reg[index];
	rsrc_data->common_data = &ver3_bus_priv->common_data;
	*comp_grp = &ver3_bus_priv->comp_grp[(&ver3_hw_info->bus_client_reg[index])->comp_group];

	wm_res->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;
	INIT_LIST_HEAD(&wm_res->list);

	wm_res->start = cam_vfe_bus_ver3_start_wm;
	wm_res->stop = cam_vfe_bus_ver3_stop_wm;
	wm_res->top_half_handler = cam_vfe_bus_ver3_handle_wm_done_top_half;
	wm_res->bottom_half_handler =
		cam_vfe_bus_ver3_handle_wm_done_bottom_half;
	wm_res->hw_intf = ver3_bus_priv->common_data.hw_intf;

	if (wm_name)
		scnprintf(wm_res->res_name, CAM_ISP_RES_NAME_LEN,
			"%s", wm_name);
	return 0;
}

static int cam_vfe_bus_ver3_deinit_wm_resource(
	struct cam_isp_resource_node    *wm_res)
{
	struct cam_vfe_bus_ver3_wm_resource_data *rsrc_data;

	wm_res->res_state = CAM_ISP_RESOURCE_STATE_UNAVAILABLE;
	INIT_LIST_HEAD(&wm_res->list);

	wm_res->start = NULL;
	wm_res->stop = NULL;
	wm_res->top_half_handler = NULL;
	wm_res->bottom_half_handler = NULL;
	wm_res->hw_intf = NULL;

	rsrc_data = wm_res->res_priv;
	wm_res->res_priv = NULL;
	if (!rsrc_data)
		return -ENOMEM;
	kfree(rsrc_data);

	return 0;
}

static int cam_vfe_bus_ver3_acquire_comp_grp(
	struct cam_vfe_bus_ver3_priv         *ver3_bus_priv,
	void                                *tasklet,
	uint32_t                             is_dual,
	uint32_t                             is_master,
	struct cam_isp_resource_node       **comp_grp,
	struct cam_vfe_bus_ver3_comp_grp_acquire_args *comp_acq_args,
	bool                                 is_per_port_acquire,
	bool                                 update_only)
{
	int rc = 0;
	struct cam_isp_resource_node           *comp_grp_local = NULL;
	struct cam_vfe_bus_ver3_comp_grp_data  *rsrc_data = NULL;
	bool previously_acquired = false;

	/* Check if matching comp_grp has already been acquired */
	previously_acquired = cam_vfe_bus_ver3_match_comp_grp(
		ver3_bus_priv, &comp_grp_local, comp_acq_args->comp_grp_id);

	if (!comp_grp_local) {
		CAM_ERR(CAM_ISP, "Invalid comp_grp:%d",
			comp_acq_args->comp_grp_id);
		return -ENODEV;
	}

	rsrc_data = comp_grp_local->res_priv;

	if (!previously_acquired) {
		rsrc_data->intra_client_mask = 0x1;
		comp_grp_local->tasklet_info = tasklet;
		comp_grp_local->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;

		rsrc_data->is_master = is_master;
		rsrc_data->is_dual = is_dual;

		if (is_master)
			rsrc_data->addr_sync_mode = 0;
		else
			rsrc_data->addr_sync_mode = 1;

	} else {
		rsrc_data = comp_grp_local->res_priv;
		/* Do not support runtime change in composite mask */
		if (!update_only && comp_grp_local->res_state ==
			CAM_ISP_RESOURCE_STATE_STREAMING) {
			CAM_ERR(CAM_ISP, "Invalid State %d comp_grp:%u",
				comp_grp_local->res_state,
				rsrc_data->comp_grp_type);
			return -EBUSY;
		}
	}

	if (is_per_port_acquire)
		comp_grp_local->is_per_port_acquire = true;

	CAM_DBG(CAM_ISP, "Acquire VFE:%d comp_grp:%u is_per_port_acquire :%d",
		rsrc_data->common_data->core_index,
		rsrc_data->comp_grp_type, comp_grp_local->is_per_port_acquire);

	if (!update_only)
		rsrc_data->acquire_dev_cnt++;
	rsrc_data->composite_mask |= comp_acq_args->composite_mask;
	*comp_grp = comp_grp_local;

	return rc;
}

static int cam_vfe_bus_ver3_release_comp_grp(
	struct cam_vfe_bus_ver3_priv         *ver3_bus_priv,
	struct cam_isp_resource_node         *in_comp_grp)
{
	struct cam_isp_resource_node           *comp_grp = NULL;
	struct cam_vfe_bus_ver3_comp_grp_data  *in_rsrc_data = NULL;
	int match_found = 0;

	if (!in_comp_grp) {
		CAM_ERR(CAM_ISP, "Invalid Params comp_grp %pK", in_comp_grp);
		return -EINVAL;
	}

	if (in_comp_grp->res_state == CAM_ISP_RESOURCE_STATE_AVAILABLE) {
		CAM_ERR(CAM_ISP, "VFE:%u Already released comp_grp",
			ver3_bus_priv->common_data.core_index);
		return 0;
	}

	if (in_comp_grp->res_state == CAM_ISP_RESOURCE_STATE_STREAMING) {
		CAM_ERR(CAM_ISP, "VFE:%u Invalid State %d",
			ver3_bus_priv->common_data.core_index, in_comp_grp->res_state);
		return -EBUSY;
	}

	in_rsrc_data = in_comp_grp->res_priv;
	CAM_DBG(CAM_ISP, "Release VFE:%u comp_grp:%u",
		ver3_bus_priv->common_data.core_index,
		in_rsrc_data->comp_grp_type);

	list_for_each_entry(comp_grp, &ver3_bus_priv->used_comp_grp, list) {
		if (comp_grp == in_comp_grp) {
			match_found = 1;
			break;
		}
	}

	if (!match_found) {
		CAM_ERR(CAM_ISP, "Could not find comp_grp:%u",
			in_rsrc_data->comp_grp_type);
		return -ENODEV;
	}

	in_rsrc_data->acquire_dev_cnt--;
	if (in_rsrc_data->acquire_dev_cnt == 0) {
		list_del(&comp_grp->list);
		in_rsrc_data->dual_slave_core = CAM_VFE_BUS_VER3_VFE_CORE_MAX;
		in_rsrc_data->addr_sync_mode = 0;
		in_rsrc_data->composite_mask = 0;

		in_comp_grp->tasklet_info = NULL;
		in_comp_grp->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;

		list_add_tail(&comp_grp->list, &ver3_bus_priv->free_comp_grp);
	}

	return 0;
}

static int cam_vfe_bus_ver3_start_comp_grp(
	struct cam_vfe_bus_ver3_vfe_out_data *vfe_out_data, uint32_t *bus_irq_reg_mask)
{
	int rc = 0;
	uint32_t val;
	struct cam_isp_resource_node *comp_grp = vfe_out_data->comp_grp;
	struct cam_vfe_bus_ver3_comp_grp_data *rsrc_data =
		vfe_out_data->comp_grp->res_priv;
	struct cam_vfe_bus_ver3_common_data *common_data = rsrc_data->common_data;

	CAM_DBG(CAM_ISP,
		"Start VFE:%u comp_grp:%d streaming state:%d comp_mask:0x%X",
		rsrc_data->common_data->core_index,
		rsrc_data->comp_grp_type, comp_grp->res_state,
		rsrc_data->composite_mask);

	if (comp_grp->res_state == CAM_ISP_RESOURCE_STATE_STREAMING) {
		bus_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_REG0] = rsrc_data->comp_done_mask;
		CAM_DBG(CAM_ISP, "Already Start Done VFE:%u comp_grp:%d bus_irq_mask_0: 0x%X",
			rsrc_data->common_data->core_index,
			rsrc_data->comp_grp_type,
			bus_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_REG0]);
		return 0;
	}

	if (!common_data->comp_config_needed)
		goto skip_comp_cfg;

	if (rsrc_data->is_dual) {
		if (rsrc_data->is_master) {
			val = cam_io_r_mb(common_data->mem_base +
				common_data->common_reg->comp_cfg_0);

			val |= (0x1 << (rsrc_data->comp_grp_type + 14));

			cam_io_w_mb(val, common_data->mem_base +
				common_data->common_reg->comp_cfg_0);

			val = cam_io_r_mb(common_data->mem_base +
				common_data->common_reg->comp_cfg_1);

			val |= (0x1 << rsrc_data->comp_grp_type);

			cam_io_w_mb(val, common_data->mem_base +
				common_data->common_reg->comp_cfg_1);
		} else {
			val = cam_io_r_mb(common_data->mem_base +
				common_data->common_reg->comp_cfg_0);

			val |= (0x1 << rsrc_data->comp_grp_type);
			val |= (0x1 << (rsrc_data->comp_grp_type + 14));

			cam_io_w_mb(val, common_data->mem_base +
				common_data->common_reg->comp_cfg_0);

			val = cam_io_r_mb(common_data->mem_base +
				common_data->common_reg->comp_cfg_1);

			val |= (0x1 << rsrc_data->comp_grp_type);

			cam_io_w_mb(val, common_data->mem_base +
				common_data->common_reg->comp_cfg_1);
		}
	}

skip_comp_cfg:

	if (rsrc_data->ubwc_static_ctrl) {
		val = cam_io_r_mb(common_data->mem_base +
			common_data->common_reg->ubwc_static_ctrl);
		val |= rsrc_data->ubwc_static_ctrl;
		cam_io_w_mb(val, common_data->mem_base +
			common_data->common_reg->ubwc_static_ctrl);
	}

	bus_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_REG0] = rsrc_data->comp_done_mask;

	CAM_DBG(CAM_ISP, "Start Done VFE:%u comp_grp:%d bus_irq_mask_0: 0x%X",
		rsrc_data->common_data->core_index,
		rsrc_data->comp_grp_type,
		bus_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_REG0]);

	comp_grp->res_state = CAM_ISP_RESOURCE_STATE_STREAMING;

	return rc;
}

static int cam_vfe_bus_ver3_stop_comp_grp(
	struct cam_isp_resource_node          *comp_grp)
{
	comp_grp->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;

	return 0;
}

static int cam_vfe_bus_ver3_handle_comp_done_top_half(uint32_t evt_id,
	struct cam_irq_th_payload *th_payload)
{
	return -EPERM;
}

static int cam_vfe_bus_ver3_handle_comp_done_bottom_half(
	void                *handler_priv,
	void                *evt_payload_priv,
	uint32_t            *comp_grp_id)
{
	int rc = CAM_VFE_IRQ_STATUS_ERR;
	struct cam_vfe_bus_ver3_vfe_out_data  *vfe_out  = handler_priv;
	struct cam_isp_resource_node          *comp_grp = vfe_out->comp_grp;
	struct cam_vfe_bus_irq_evt_payload    *evt_payload = evt_payload_priv;
	struct cam_vfe_bus_ver3_comp_grp_data *rsrc_data = comp_grp->res_priv;
	uint32_t                              *cam_ife_irq_regs;
	uint32_t                               status_0;

	if (!evt_payload || !rsrc_data) {
		CAM_ERR(CAM_ISP, "Either evt_payload or rsrc_data is invalid");
		return rc;
	}
	if (rsrc_data->is_dual && (!rsrc_data->is_master)) {
		CAM_ERR(CAM_ISP, "VFE:%u Invalid comp_grp:%u is_master:%u",
			rsrc_data->common_data->core_index, rsrc_data->comp_grp_type,
			rsrc_data->is_master);
		return rc;
	}

	cam_ife_irq_regs = evt_payload->irq_reg_val;
	status_0 = cam_ife_irq_regs[CAM_IFE_IRQ_BUS_VER3_REG_STATUS0];

	if (status_0 & rsrc_data->comp_done_mask) {
		evt_payload->evt_id = CAM_ISP_HW_EVENT_DONE;
		rc = CAM_VFE_IRQ_STATUS_SUCCESS;
	}

	CAM_DBG(CAM_ISP, "VFE:%u comp_grp:%d Bus IRQ status_0: 0x%X rc:%d",
		rsrc_data->common_data->core_index, rsrc_data->comp_grp_type,
		status_0, rc);

	*comp_grp_id = rsrc_data->comp_grp_type;

	return rc;
}

static int cam_vfe_bus_ver3_init_comp_grp(uint32_t index,
	struct cam_hw_soc_info          *soc_info,
	struct cam_vfe_bus_ver3_priv    *ver3_bus_priv,
	struct cam_vfe_bus_ver3_hw_info *ver3_hw_info,
	struct cam_isp_resource_node    *comp_grp)
{
	struct cam_vfe_bus_ver3_comp_grp_data *rsrc_data = NULL;
	struct cam_vfe_soc_private *vfe_soc_private = soc_info->soc_private;
	int ddr_type = 0;

	rsrc_data = kzalloc(sizeof(struct cam_vfe_bus_ver3_comp_grp_data),
		GFP_KERNEL);
	if (!rsrc_data)
		return -ENOMEM;

	comp_grp->res_priv = rsrc_data;

	comp_grp->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;
	INIT_LIST_HEAD(&comp_grp->list);

	rsrc_data->comp_grp_type   = index;
	rsrc_data->common_data     = &ver3_bus_priv->common_data;
	rsrc_data->dual_slave_core = CAM_VFE_BUS_VER3_VFE_CORE_MAX;
	rsrc_data->comp_done_mask = ver3_hw_info->comp_done_mask[index];

	if (rsrc_data->comp_grp_type != CAM_VFE_BUS_VER3_COMP_GRP_0 &&
		rsrc_data->comp_grp_type != CAM_VFE_BUS_VER3_COMP_GRP_1)
		rsrc_data->ubwc_static_ctrl = 0;
	else {
		ddr_type = cam_get_ddr_type();
		if ((ddr_type == DDR_TYPE_LPDDR5) ||
			(ddr_type == DDR_TYPE_LPDDR5X))
			rsrc_data->ubwc_static_ctrl =
				vfe_soc_private->ubwc_static_ctrl[1];
		else
			rsrc_data->ubwc_static_ctrl =
				vfe_soc_private->ubwc_static_ctrl[0];
	}

	list_add_tail(&comp_grp->list, &ver3_bus_priv->free_comp_grp);

	comp_grp->top_half_handler = cam_vfe_bus_ver3_handle_comp_done_top_half;
	comp_grp->hw_intf = ver3_bus_priv->common_data.hw_intf;

	return 0;
}

static int cam_vfe_bus_ver3_deinit_comp_grp(
	struct cam_isp_resource_node    *comp_grp)
{
	struct cam_vfe_bus_ver3_comp_grp_data *rsrc_data =
		comp_grp->res_priv;

	comp_grp->start = NULL;
	comp_grp->stop = NULL;
	comp_grp->top_half_handler = NULL;
	comp_grp->bottom_half_handler = NULL;
	comp_grp->hw_intf = NULL;

	list_del_init(&comp_grp->list);
	comp_grp->res_state = CAM_ISP_RESOURCE_STATE_UNAVAILABLE;

	comp_grp->res_priv = NULL;

	if (!rsrc_data) {
		CAM_ERR(CAM_ISP, "comp_grp_priv is NULL");
		return -ENODEV;
	}
	kfree(rsrc_data);

	return 0;
}

static int cam_vfe_bus_ver3_get_secure_mode(void *priv, void *cmd_args,
	uint32_t arg_size)
{
	struct cam_isp_hw_get_cmd_update      *secure_mode = cmd_args;
	struct cam_vfe_bus_ver3_vfe_out_data  *rsrc_data;
	uint32_t                              *mode;

	rsrc_data = (struct cam_vfe_bus_ver3_vfe_out_data *)
		secure_mode->res->res_priv;
	mode = (uint32_t *)secure_mode->data;
	*mode = (rsrc_data->secure_mode == CAM_SECURE_MODE_SECURE) ?
		true : false;

	return 0;
}

static int cam_vfe_bus_ver3_update_acquire_vfe_out(void *bus_priv, void *acquire_args,
	uint32_t args_size, bool update_only)
{
	int                                     rc = -ENODEV;
	int                                     i;
	enum cam_vfe_bus_ver3_vfe_out_type      vfe_out_res_id;
	struct cam_vfe_bus_ver3_priv           *ver3_bus_priv = bus_priv;
	struct cam_vfe_acquire_args            *acq_args = acquire_args;
	struct cam_vfe_hw_vfe_out_acquire_args *out_acquire_args;
	struct cam_isp_resource_node           *rsrc_node = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data   *rsrc_data = NULL;
	uint32_t                                secure_caps = 0, mode;
	struct cam_vfe_bus_ver3_comp_grp_acquire_args comp_acq_args = {0};
	uint32_t       outmap_index = CAM_VFE_BUS_VER3_VFE_OUT_MAX;
	uint32_t       out_port_res_type;
	bool is_per_port_acquire = false;

	if (!bus_priv || !acquire_args) {
		CAM_ERR(CAM_ISP, "Invalid Param");
		return -EINVAL;
	}

	out_acquire_args = &acq_args->vfe_out;

	if (acq_args->per_port_acquire) {
		out_port_res_type = out_acquire_args->vfe_res_out_id;
		is_per_port_acquire = true;
	} else {
		out_port_res_type = out_acquire_args->out_port_info->acquired_res_type;
	}

	CAM_DBG(CAM_ISP, "VFE:%u Acquire out_type:0x%X use_hw_ctxt:%s hw_ctx_id:0x%x",
		ver3_bus_priv->common_data.core_index,
		out_port_res_type,
		CAM_BOOL_TO_YESNO(out_acquire_args->use_hw_ctxt),
		out_acquire_args->out_port_info->hw_context_id);

	vfe_out_res_id = cam_vfe_bus_ver3_get_out_res_id_and_index(
				ver3_bus_priv,
				out_port_res_type,
				&outmap_index);
	if ((vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_MAX) ||
		(outmap_index >= ver3_bus_priv->num_out)) {
		CAM_WARN(CAM_ISP,
			"VFE:%u target does not support req res id :0x%x outtype:%d index:%d",
			ver3_bus_priv->common_data.core_index,
			out_acquire_args->out_port_info->acquired_res_type,
			vfe_out_res_id, outmap_index);
		return -ENODEV;
	}

	rsrc_node = &ver3_bus_priv->vfe_out[outmap_index];
	rsrc_data = rsrc_node->res_priv;
	if (!update_only &&
		rsrc_node->res_state != CAM_ISP_RESOURCE_STATE_AVAILABLE) {
		if ((rsrc_data->mc_based || rsrc_data->cntxt_cfg_except) &&
			out_acquire_args->use_hw_ctxt &&
			!(rsrc_data->dst_hw_ctxt_id_mask &
			out_acquire_args->out_port_info->hw_context_id)) {
			rsrc_data->dst_hw_ctxt_id_mask |=
				out_acquire_args->out_port_info->hw_context_id;
			out_acquire_args->rsrc_node = rsrc_node;
			rc = 0;
			goto end;
		} else {
			CAM_ERR(CAM_ISP,
				"VFE:%u out_type:0x%X mc_cap:%s cntxt_except:%s resource not available state:%d dst_hw_ctxt_id_mask:0x%x req_ctxt_id:0x%x",
				ver3_bus_priv->common_data.core_index,
				vfe_out_res_id, CAM_BOOL_TO_YESNO(rsrc_data->mc_based),
				CAM_BOOL_TO_YESNO(rsrc_data->cntxt_cfg_except),
				rsrc_node->res_state, rsrc_data->dst_hw_ctxt_id_mask,
				out_acquire_args->out_port_info->hw_context_id);
			rc = -EBUSY;
			goto end;
		}
	}

	rsrc_data->common_data->event_cb = acq_args->event_cb;
	rsrc_data->common_data->priv = acq_args->priv;
	rsrc_data->common_data->disable_ubwc_comp =
		out_acquire_args->disable_ubwc_comp;
	rsrc_data->priv = acq_args->priv;
	rsrc_data->bus_priv = ver3_bus_priv;
	rsrc_data->limiter_enabled = false;
	comp_acq_args.composite_mask = (1ULL << vfe_out_res_id);

	if (out_acquire_args->use_hw_ctxt)
		rsrc_data->dst_hw_ctxt_id_mask |= out_acquire_args->out_port_info->hw_context_id;

	/* for some hw versions, buf done is not received from vfe but
	 * from IP external to VFE. In such case, we get the controller
	 * from hw manager and assign it here
	 */
	if (!(ver3_bus_priv->common_data.supported_irq &
			CAM_VFE_HW_IRQ_CAP_BUF_DONE))
		rsrc_data->common_data->buf_done_controller =
			acq_args->buf_done_controller;

	secure_caps = cam_vfe_bus_ver3_can_be_secure(
		rsrc_data->out_type);
	mode = out_acquire_args->out_port_info->secure_mode;
	mutex_lock(&rsrc_data->common_data->bus_mutex);
	if (secure_caps) {
		if (!rsrc_data->common_data->num_sec_out) {
			rsrc_data->secure_mode = mode;
			rsrc_data->common_data->secure_mode = mode;
		} else {
			if (mode == rsrc_data->common_data->secure_mode) {
				rsrc_data->secure_mode =
					rsrc_data->common_data->secure_mode;
			} else {
				rc = -EINVAL;
				CAM_ERR_RATE_LIMIT(CAM_ISP,
					"VFE:%u Mismatch: Acquire mode[%d], drvr mode[%d]",
					ver3_bus_priv->common_data.core_index,
					rsrc_data->common_data->secure_mode,
					mode);
				mutex_unlock(
					&rsrc_data->common_data->bus_mutex);
				return rc;
			}
		}
		rsrc_data->common_data->num_sec_out++;
	}
	mutex_unlock(&rsrc_data->common_data->bus_mutex);

	ver3_bus_priv->tasklet_info = acq_args->tasklet;
	rsrc_node->is_rdi_primary_res = false;
	rsrc_node->res_id = out_port_res_type;
	rsrc_node->tasklet_info = acq_args->tasklet;
	rsrc_node->cdm_ops = out_acquire_args->cdm_ops;
	rsrc_data->common_data->cdm_util_ops = out_acquire_args->cdm_ops;
	rsrc_data->format = out_acquire_args->out_port_info->format;

	if ((rsrc_data->out_type == CAM_VFE_BUS_VER3_VFE_OUT_FD) &&
		(rsrc_data->format == CAM_FORMAT_Y_ONLY))
		rsrc_data->num_wm = 1;

	/* Acquire WM and retrieve COMP GRP ID */
	for (i = 0; i < rsrc_data->num_wm; i++) {
		rc = cam_vfe_bus_ver3_acquire_wm(ver3_bus_priv,
			out_acquire_args,
			acq_args->tasklet,
			vfe_out_res_id,
			i,
			&rsrc_data->wm_res[i],
			&comp_acq_args.comp_grp_id,
			is_per_port_acquire,
			update_only);
		if (rc) {
			CAM_ERR(CAM_ISP,
				"Failed to acquire WM VFE:%u out_type:%d rc:%d",
				rsrc_data->common_data->core_index,
				vfe_out_res_id, rc);
			goto release_wm;
		}
	}

	/* Acquire composite group using COMP GRP ID */
	rc = cam_vfe_bus_ver3_acquire_comp_grp(ver3_bus_priv,
		acq_args->tasklet,
		out_acquire_args->is_dual,
		out_acquire_args->is_master,
		&rsrc_data->comp_grp,
		&comp_acq_args,
		is_per_port_acquire,
		update_only);
	if (rc) {
		CAM_ERR(CAM_ISP,
			"Failed to acquire comp_grp VFE:%u out_typp:%d rc:%d",
			rsrc_data->common_data->core_index,
			vfe_out_res_id, rc);
		goto release_wm;
	}

	rsrc_data->is_dual = out_acquire_args->is_dual;
	rsrc_data->is_master = out_acquire_args->is_master;
	if (!update_only)
		rsrc_node->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
	rsrc_node->is_per_port_acquire = is_per_port_acquire;
	out_acquire_args->rsrc_node = rsrc_node;
	out_acquire_args->comp_grp_id = comp_acq_args.comp_grp_id;

	CAM_DBG(CAM_ISP, "VFE:%u Acquire successful", rsrc_data->common_data->core_index);
	return rc;

release_wm:
	for (i--; i >= 0; i--)
		cam_vfe_bus_ver3_release_wm(ver3_bus_priv,
			&rsrc_data->wm_res[i]);

end:
	return rc;
}

static int cam_vfe_bus_ver3_acquire_vfe_out(void *bus_priv,
	void *acquire_args, uint32_t args_size)
{
	return cam_vfe_bus_ver3_update_acquire_vfe_out(bus_priv, acquire_args,
			args_size, false);
}

static int cam_vfe_bus_ver3_release_vfe_out(void *bus_priv, void *release_args,
	uint32_t args_size)
{
	uint32_t i;
	struct cam_isp_resource_node          *vfe_out = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data  *rsrc_data = NULL;
	uint32_t                               secure_caps = 0;

	if (!bus_priv || !release_args) {
		CAM_ERR(CAM_ISP, "Invalid input bus_priv %pK release_args %pK",
			bus_priv, release_args);
		return -EINVAL;
	}

	vfe_out = release_args;
	rsrc_data = vfe_out->res_priv;

	if (vfe_out->res_state != CAM_ISP_RESOURCE_STATE_RESERVED) {
		CAM_ERR(CAM_ISP,
			"Invalid resource state:%d VFE:%u out_type:0x%X",
			vfe_out->res_state, rsrc_data->common_data->core_index,
			vfe_out->res_id);
	}

	for (i = 0; i < rsrc_data->num_wm; i++)
		cam_vfe_bus_ver3_release_wm(bus_priv, &rsrc_data->wm_res[i]);

	if ((rsrc_data->out_type == CAM_VFE_BUS_VER3_VFE_OUT_FD) &&
		(rsrc_data->format == CAM_FORMAT_Y_ONLY))
		rsrc_data->num_wm = 2;

	if (rsrc_data->comp_grp)
		cam_vfe_bus_ver3_release_comp_grp(bus_priv,
			rsrc_data->comp_grp);


	vfe_out->tasklet_info = NULL;
	vfe_out->cdm_ops = NULL;
	rsrc_data->dst_hw_ctxt_id_mask = 0;

	secure_caps = cam_vfe_bus_ver3_can_be_secure(rsrc_data->out_type);
	mutex_lock(&rsrc_data->common_data->bus_mutex);
	if (secure_caps) {
		if (rsrc_data->secure_mode ==
			rsrc_data->common_data->secure_mode) {
			rsrc_data->common_data->num_sec_out--;
			rsrc_data->secure_mode =
				CAM_SECURE_MODE_NON_SECURE;
		} else {
			/*
			 * The validity of the mode is properly
			 * checked while acquiring the output port.
			 * not expected to reach here, unless there is
			 * some corruption.
			 */
			CAM_ERR(CAM_ISP, "VFE:%u driver[%d],resource[%d] mismatch",
				rsrc_data->common_data->core_index,
				rsrc_data->common_data->secure_mode,
				rsrc_data->secure_mode);
		}

		if (!rsrc_data->common_data->num_sec_out)
			rsrc_data->common_data->secure_mode =
				CAM_SECURE_MODE_NON_SECURE;
	}
	mutex_unlock(&rsrc_data->common_data->bus_mutex);

	if (vfe_out->res_state == CAM_ISP_RESOURCE_STATE_RESERVED)
		vfe_out->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;

	return 0;
}

static int cam_vfe_bus_ver3_start_vfe_out(
	struct cam_isp_resource_node          *vfe_out)
{
	int rc = 0, i;
	struct cam_vfe_bus_ver3_vfe_out_data  *rsrc_data = NULL;
	struct cam_vfe_bus_ver3_common_data   *common_data = NULL;
	uint32_t source_group = 0;
	uint32_t bus_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_MAX];
	uint32_t rup_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_MAX];

	if (!vfe_out) {
		CAM_ERR(CAM_ISP, "Invalid input");
		return -EINVAL;
	}

	rsrc_data = vfe_out->res_priv;
	common_data = rsrc_data->common_data;
	source_group = rsrc_data->source_group;

	CAM_DBG(CAM_ISP, "Start VFE:%u out_type:0x%X",
		rsrc_data->common_data->core_index, rsrc_data->out_type);

	if (vfe_out->res_state != CAM_ISP_RESOURCE_STATE_RESERVED) {
		CAM_ERR(CAM_ISP,
			"Invalid resource state:%d VFE:%u out_type:0x%X",
			vfe_out->res_state, rsrc_data->common_data->core_index,
			rsrc_data->out_type);
		return -EACCES;
	}

	/* subscribe when first out rsrc is streamed on */
	if (!rsrc_data->common_data->init_irq_subscribed) {
		rc = cam_vfe_bus_ver3_subscribe_init_irq(rsrc_data->bus_priv);
		if (rc)
			return rc;
	}

	for (i = 0; i < rsrc_data->num_wm; i++) {
		rc = cam_vfe_bus_ver3_start_wm(&rsrc_data->wm_res[i]);
		if (rc)
			return rc;
	}

	memset(bus_irq_reg_mask, 0, sizeof(bus_irq_reg_mask));
	rc = cam_vfe_bus_ver3_start_comp_grp(rsrc_data,
			bus_irq_reg_mask);

	if (rsrc_data->is_dual && !rsrc_data->is_master)
		goto end;

	if (vfe_out->is_per_port_start) {
		CAM_DBG(CAM_ISP, "Skipping irq subscribe for resources that are not updated");
		goto end;
	}

	vfe_out->irq_handle = cam_irq_controller_subscribe_irq(
		common_data->buf_done_controller,
		CAM_IRQ_PRIORITY_1,
		bus_irq_reg_mask,
		vfe_out,
		vfe_out->top_half_handler,
		vfe_out->bottom_half_handler,
		vfe_out->tasklet_info,
		&tasklet_bh_api,
		CAM_IRQ_EVT_GROUP_0);

	if (vfe_out->irq_handle < 1) {
		CAM_ERR(CAM_ISP, "Subscribe IRQ failed for VFE out_res %d, VFE:%u",
			vfe_out->res_id, rsrc_data->common_data->core_index);
		vfe_out->irq_handle = 0;
		return -EFAULT;
	}

	if ((common_data->is_lite || source_group > CAM_VFE_BUS_VER3_SRC_GRP_0)
		&& !vfe_out->is_rdi_primary_res)
		goto end;

	if ((common_data->supported_irq & CAM_VFE_HW_IRQ_CAP_RUP) &&
		(!common_data->rup_irq_handle[source_group])) {
		memset(rup_irq_reg_mask, 0, sizeof(rup_irq_reg_mask));
		rup_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_REG0] |=
			0x1 << source_group;

		CAM_DBG(CAM_ISP,
			"VFE:%u out_type:0x%X bus_irq_mask_0:0x%X for RUP",
			rsrc_data->common_data->core_index, rsrc_data->out_type,
			rup_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_REG0]);

		common_data->rup_irq_handle[source_group] =
			cam_irq_controller_subscribe_irq(
				common_data->bus_irq_controller,
				CAM_IRQ_PRIORITY_0,
				rup_irq_reg_mask,
				vfe_out,
				cam_vfe_bus_ver3_handle_rup_top_half,
				cam_vfe_bus_ver3_handle_rup_bottom_half,
				vfe_out->tasklet_info,
				&tasklet_bh_api,
				CAM_IRQ_EVT_GROUP_1);

		if (common_data->rup_irq_handle[source_group] < 1) {
			CAM_ERR(CAM_ISP, "VFE:%u Failed to subscribe RUP IRQ",
				rsrc_data->common_data->core_index);
			common_data->rup_irq_handle[source_group] = 0;
			return -EFAULT;
		}
	}

end:
	vfe_out->res_state = CAM_ISP_RESOURCE_STATE_STREAMING;
	return rc;
}

static int cam_vfe_bus_ver3_stop_vfe_out(
	struct cam_isp_resource_node          *vfe_out)
{
	int rc = 0, i;
	struct cam_vfe_bus_ver3_vfe_out_data  *rsrc_data = NULL;
	struct cam_vfe_bus_ver3_common_data   *common_data = NULL;

	if (!vfe_out) {
		CAM_ERR(CAM_ISP, "Invalid input");
		return -EINVAL;
	}

	rsrc_data = vfe_out->res_priv;
	common_data = rsrc_data->common_data;

	if (vfe_out->res_state == CAM_ISP_RESOURCE_STATE_AVAILABLE ||
		vfe_out->res_state == CAM_ISP_RESOURCE_STATE_RESERVED) {
		CAM_DBG(CAM_ISP, "Stop VFE:%u out_type:0x%X state:%d",
			rsrc_data->common_data->core_index, rsrc_data->out_type,
			vfe_out->res_state);
		return rc;
	}

	rc = cam_vfe_bus_ver3_stop_comp_grp(rsrc_data->comp_grp);

	for (i = 0; i < rsrc_data->num_wm; i++)
		rc = cam_vfe_bus_ver3_stop_wm(&rsrc_data->wm_res[i]);

	if (common_data->rup_irq_handle[rsrc_data->source_group]) {
		rc = cam_irq_controller_unsubscribe_irq(
			common_data->bus_irq_controller,
			common_data->rup_irq_handle[rsrc_data->source_group]);
		common_data->rup_irq_handle[rsrc_data->source_group] = 0;
	}

	if (vfe_out->irq_handle) {
		rc = cam_irq_controller_unsubscribe_irq(
			common_data->buf_done_controller,
			vfe_out->irq_handle);
		vfe_out->irq_handle = 0;
	}

	if (rsrc_data->common_data->init_irq_subscribed)
		cam_vfe_bus_ver3_unsubscribe_init_irq(rsrc_data->bus_priv);

	vfe_out->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
	return rc;
}

static int cam_vfe_bus_ver3_handle_vfe_out_done_top_half(uint32_t evt_id,
	struct cam_irq_th_payload *th_payload)
{
	int32_t                                     rc;
	int                                         i;
	struct cam_isp_resource_node               *vfe_out = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data       *rsrc_data = NULL;
	struct cam_vfe_bus_irq_evt_payload         *evt_payload;
	struct cam_vfe_bus_ver3_comp_grp_data      *resource_data;
	uint32_t                                    status_0;
	struct cam_vfe_bus_ver3_wm_resource_data   *wm_rsrc_data = NULL;

	vfe_out = th_payload->handler_priv;
	if (!vfe_out) {
		CAM_ERR_RATE_LIMIT(CAM_ISP, "No resource");
		return -ENODEV;
	}

	rsrc_data = vfe_out->res_priv;
	resource_data = rsrc_data->comp_grp->res_priv;

	rc  = cam_vfe_bus_ver3_get_evt_payload(rsrc_data->common_data,
		&evt_payload);

	if (rc) {
		for (i = 0; i < th_payload->num_registers; i++)
			CAM_INFO_RATE_LIMIT(CAM_ISP,
				"VFE:%u Bus IRQ status_%d: 0x%X",
				rsrc_data->common_data->core_index, i,
				th_payload->evt_status_arr[i]);
		return rc;
	}

	cam_isp_hw_get_timestamp(&evt_payload->ts);

	evt_payload->core_index = rsrc_data->common_data->core_index;
	evt_payload->evt_id = evt_id;

	for (i = 0; i < th_payload->num_registers; i++) {
		evt_payload->irq_reg_val[i] = th_payload->evt_status_arr[i];
		CAM_DBG(CAM_ISP, "VFE:%u Bus IRQ status_%d: 0x%X",
			rsrc_data->common_data->core_index, i,
			th_payload->evt_status_arr[i]);
	}

	th_payload->evt_payload_priv = evt_payload;

	status_0 = th_payload->evt_status_arr[CAM_IFE_IRQ_BUS_VER3_REG_STATUS0];

	if (status_0 & resource_data->comp_done_mask) {
		wm_rsrc_data = rsrc_data->wm_res[PLANE_Y].res_priv;
		evt_payload->last_consumed_addr = cam_io_r_mb(
			wm_rsrc_data->common_data->mem_base +
			wm_rsrc_data->hw_regs->addr_status_0);
		trace_cam_log_event("bufdone", "bufdone_IRQ",
			status_0, resource_data->comp_grp_type);
	}

	if (status_0 & 0x1)
		trace_cam_log_event("UnexpectedRUP", "RUP_IRQ", status_0, 40);

	CAM_DBG(CAM_ISP, "VFE:%u Exit", rsrc_data->common_data->core_index);
	return rc;
}

static int cam_vfe_bus_ver3_handle_vfe_out_done_bottom_half(
	void                *handler_priv,
	void                *evt_payload_priv)
{
	int                                    rc = -EINVAL;
	struct cam_isp_resource_node          *vfe_out = handler_priv;
	struct cam_vfe_bus_ver3_vfe_out_data  *rsrc_data = vfe_out->res_priv;
	struct cam_vfe_bus_irq_evt_payload    *evt_payload = evt_payload_priv;
	struct cam_isp_hw_event_info           evt_info;
	struct cam_isp_hw_bufdone_event_info   bufdone_evt_info = {0};
	void                                  *ctx = NULL;
	uint32_t                               evt_id = 0;
	uint32_t                               comp_grp_id = 0;

	if (!rsrc_data) {
		CAM_ERR(CAM_ISP, "Invalid rsrc data pointer, returning from bottom half");
		return rc;
	}

	rc = cam_vfe_bus_ver3_handle_comp_done_bottom_half(
		rsrc_data, evt_payload_priv, &comp_grp_id);
	CAM_DBG(CAM_ISP, "VFE:%u out_type:0x%x comp_grp_id:%d rc:%d",
		rsrc_data->common_data->core_index, rsrc_data->out_type,
		comp_grp_id, rc);

	ctx = rsrc_data->priv;

	switch (rc) {
	case CAM_VFE_IRQ_STATUS_SUCCESS:
		evt_id = evt_payload->evt_id;

		evt_info.res_type = vfe_out->res_type;
		evt_info.hw_idx   = vfe_out->hw_intf->hw_idx;
		evt_info.hw_type  = CAM_ISP_HW_TYPE_VFE;
		evt_info.res_id = vfe_out->res_id;
		bufdone_evt_info.res_id = vfe_out->res_id;
		bufdone_evt_info.comp_grp_id = comp_grp_id;
		bufdone_evt_info.last_consumed_addr = evt_payload->last_consumed_addr;
		evt_info.event_data = (void *)&bufdone_evt_info;

		if (rsrc_data->common_data->event_cb)
			rsrc_data->common_data->event_cb(ctx, evt_id,
				(void *)&evt_info);
		break;
	default:
		break;
	}

	cam_vfe_bus_ver3_put_evt_payload(rsrc_data->common_data, &evt_payload);

	return rc;
}

static int cam_vfe_bus_ver3_init_vfe_out_resource(uint32_t  index,
	struct cam_vfe_bus_ver3_priv                  *ver3_bus_priv,
	struct cam_vfe_bus_ver3_hw_info               *ver3_hw_info)
{
	struct cam_isp_resource_node         *vfe_out = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data *rsrc_data = NULL;
	int rc = 0, i = 0;
	int32_t vfe_out_type =
		ver3_hw_info->vfe_out_hw_info[index].vfe_out_type;

	if (vfe_out_type < 0 ||
		vfe_out_type >= CAM_VFE_BUS_VER3_VFE_OUT_MAX) {
		CAM_ERR(CAM_ISP, "Init VFE Out failed, Invalid type=%d",
			vfe_out_type);
		return -EINVAL;
	}

	ver3_bus_priv->vfe_out_map_outtype[vfe_out_type] = index;
	vfe_out = &ver3_bus_priv->vfe_out[index];
	if (vfe_out->res_state != CAM_ISP_RESOURCE_STATE_UNAVAILABLE ||
		vfe_out->res_priv) {
		CAM_ERR(CAM_ISP, "vfe_out_type %d has already been initialized",
			vfe_out_type);
		return -EFAULT;
	}

	rsrc_data = kzalloc(sizeof(struct cam_vfe_bus_ver3_vfe_out_data),
		GFP_KERNEL);
	if (!rsrc_data) {
		rc = -ENOMEM;
		return rc;
	}

	vfe_out->res_priv = rsrc_data;

	vfe_out->res_type = CAM_ISP_RESOURCE_VFE_OUT;
	vfe_out->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;
	INIT_LIST_HEAD(&vfe_out->list);

	rsrc_data->source_group =
		ver3_hw_info->vfe_out_hw_info[index].source_group;
	rsrc_data->out_type     =
		ver3_hw_info->vfe_out_hw_info[index].vfe_out_type;
	rsrc_data->common_data  = &ver3_bus_priv->common_data;
	rsrc_data->max_width    =
		ver3_hw_info->vfe_out_hw_info[index].max_width;
	rsrc_data->max_height   =
		ver3_hw_info->vfe_out_hw_info[index].max_height;
	rsrc_data->secure_mode  = CAM_SECURE_MODE_NON_SECURE;
	rsrc_data->num_wm       = ver3_hw_info->vfe_out_hw_info[index].num_wm;
	rsrc_data->mc_based = ver3_hw_info->vfe_out_hw_info[index].mc_based;
	rsrc_data->cntxt_cfg_except = ver3_hw_info->vfe_out_hw_info[index].cntxt_cfg_except;

	rsrc_data->wm_res = kzalloc((sizeof(struct cam_isp_resource_node) *
		rsrc_data->num_wm), GFP_KERNEL);
	if (!rsrc_data->wm_res) {
		CAM_ERR(CAM_ISP, "Failed to alloc for wm_res");
		return -ENOMEM;
	}

	for (i = 0; i < rsrc_data->num_wm; i++) {
		rc = cam_vfe_bus_ver3_init_wm_resource(
			ver3_hw_info->vfe_out_hw_info[index].wm_idx[i],
			ver3_bus_priv, ver3_hw_info,
			&rsrc_data->wm_res[i],
			&rsrc_data->comp_grp,
			ver3_hw_info->vfe_out_hw_info[index].name[i],
			ver3_hw_info->vfe_out_hw_info[index].line_based);
		if (rc < 0) {
			CAM_ERR(CAM_ISP, "VFE:%u init WM:%d failed rc:%d",
				ver3_bus_priv->common_data.core_index, i, rc);
			return rc;
		}
	}

	rsrc_data->pid_mask = ver3_hw_info->vfe_out_hw_info[index].pid_mask;

	vfe_out->start = cam_vfe_bus_ver3_start_vfe_out;
	vfe_out->stop = cam_vfe_bus_ver3_stop_vfe_out;
	vfe_out->top_half_handler =
		cam_vfe_bus_ver3_handle_vfe_out_done_top_half;
	vfe_out->bottom_half_handler =
		cam_vfe_bus_ver3_handle_vfe_out_done_bottom_half;
	vfe_out->process_cmd = cam_vfe_bus_ver3_process_cmd;
	vfe_out->hw_intf = ver3_bus_priv->common_data.hw_intf;
	vfe_out->irq_handle = 0;

	rsrc_data->num_mid = ver3_hw_info->vfe_out_hw_info->num_mid;
	rsrc_data->mid = ver3_hw_info->vfe_out_hw_info[index].mid;

	return 0;
}

static int cam_vfe_bus_ver3_deinit_vfe_out_resource(
	struct cam_isp_resource_node    *vfe_out)
{
	struct cam_vfe_bus_ver3_vfe_out_data *rsrc_data = vfe_out->res_priv;
	int rc = 0, i = 0;

	if (vfe_out->res_state == CAM_ISP_RESOURCE_STATE_UNAVAILABLE) {
		/*
		 * This is not error. It can happen if the resource is
		 * never supported in the HW.
		 */
		return 0;
	}

	vfe_out->start = NULL;
	vfe_out->stop = NULL;
	vfe_out->top_half_handler = NULL;
	vfe_out->bottom_half_handler = NULL;
	vfe_out->hw_intf = NULL;
	vfe_out->irq_handle = 0;

	vfe_out->res_state = CAM_ISP_RESOURCE_STATE_UNAVAILABLE;
	INIT_LIST_HEAD(&vfe_out->list);
	vfe_out->res_priv = NULL;

	if (!rsrc_data)
		return -ENOMEM;

	for (i = 0; i < rsrc_data->num_wm; i++) {
		rc = cam_vfe_bus_ver3_deinit_wm_resource(&rsrc_data->wm_res[i]);
		if (rc < 0)
			CAM_ERR(CAM_ISP,
				"VFE:%u deinit WM:%d failed rc:%d",
				rsrc_data->common_data->core_index, i, rc);
	}

	rsrc_data->wm_res = NULL;
	rsrc_data->comp_grp = NULL;
	kfree(rsrc_data);

	return 0;
}

static void cam_vfe_bus_ver3_print_wm_info(
	struct cam_vfe_bus_ver3_wm_resource_data  *wm_data,
	struct cam_vfe_bus_ver3_common_data  *common_data,
	uint8_t *wm_name)
{
	uint32_t addr_status0, addr_status1, addr_status2, addr_status3, limiter;

	addr_status0 = cam_io_r_mb(common_data->mem_base +
		wm_data->hw_regs->addr_status_0);
	addr_status1 = cam_io_r_mb(common_data->mem_base +
		wm_data->hw_regs->addr_status_1);
	addr_status2 = cam_io_r_mb(common_data->mem_base +
		wm_data->hw_regs->addr_status_2);
	addr_status3 = cam_io_r_mb(common_data->mem_base +
		wm_data->hw_regs->addr_status_3);
	limiter = cam_io_r_mb(common_data->mem_base +
		wm_data->hw_regs->bw_limiter_addr);

	CAM_INFO(CAM_ISP,
		"VFE:%u WM:%d wm_name:%s width:%u height:%u stride:%u x_init:%u en_cfg:%u acquired width:%u height:%u",
		wm_data->common_data->core_index, wm_data->index, wm_name,
		wm_data->width,
		wm_data->height,
		wm_data->stride, wm_data->h_init,
		wm_data->en_cfg,
		wm_data->acquired_width,
		wm_data->acquired_height);
	CAM_INFO(CAM_ISP,
		"VFE:%u WM:%u last consumed address:0x%x last frame addr:0x%x fifo cnt:0x%x current client address:0x%x limiter: 0x%x",
		common_data->hw_intf->hw_idx, wm_data->index,
		addr_status0, addr_status1, addr_status2, addr_status3, limiter);
}

static int cam_vfe_bus_ver3_print_dimensions(
	uint32_t                                   res_id,
	struct cam_vfe_bus_ver3_priv              *bus_priv)
{
	struct cam_isp_resource_node              *rsrc_node = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data      *rsrc_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data  *wm_data   = NULL;
	struct cam_vfe_bus_ver3_common_data  *common_data = NULL;
	int                                        i;
	uint8_t                                *wm_name = NULL;
	enum cam_vfe_bus_ver3_vfe_out_type  vfe_out_res_id =
		CAM_VFE_BUS_VER3_VFE_OUT_MAX;
	uint32_t  outmap_index = CAM_VFE_BUS_VER3_VFE_OUT_MAX;

	if (!bus_priv) {
		CAM_ERR(CAM_ISP, "Invalid bus private data, res_id: %d",
			res_id);
		return -EINVAL;
	}

	vfe_out_res_id = cam_vfe_bus_ver3_get_out_res_id_and_index(bus_priv,
				res_id, &outmap_index);

	if ((vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_MAX) ||
		(outmap_index >= bus_priv->num_out)) {
		CAM_WARN_RATE_LIMIT(CAM_ISP,
			"target does not support req res id :0x%x outtype:%d index:%d",
			res_id,
			vfe_out_res_id, outmap_index);
		return -EINVAL;
	}

	rsrc_node = &bus_priv->vfe_out[outmap_index];
	rsrc_data = rsrc_node->res_priv;
	if (!rsrc_data) {
		CAM_ERR(CAM_ISP, "VFE out data is null, res_id: %d",
			vfe_out_res_id);
		return -EINVAL;
	}

	for (i = 0; i < rsrc_data->num_wm; i++) {
		wm_data = rsrc_data->wm_res[i].res_priv;
		wm_name = rsrc_data->wm_res[i].res_name;
		common_data = rsrc_data->common_data;
		cam_vfe_bus_ver3_print_wm_info(wm_data, common_data, wm_name);
	}
	return 0;
}

static int cam_vfe_bus_ver3_mini_dump(
	struct cam_vfe_bus_ver3_priv *bus_priv,
	void *cmd_args)
{
	struct cam_isp_resource_node              *rsrc_node = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data      *rsrc_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data  *wm        = NULL;
	struct cam_vfe_bus_ver3_mini_dump_data    *md;
	struct cam_vfe_bus_ver3_wm_mini_dump      *md_wm;
	struct cam_hw_mini_dump_args              *md_args;
	struct cam_hw_info                        *hw_info = NULL;
	uint32_t                                   bytes_written = 0;
	uint32_t                                   i, j, k = 0;

	if (!bus_priv || !cmd_args) {
		CAM_ERR(CAM_ISP, "Invalid bus private data");
		return -EINVAL;
	}

	hw_info = (struct cam_hw_info *)bus_priv->common_data.hw_intf->hw_priv;
	md_args = (struct cam_hw_mini_dump_args *)cmd_args;

	if (sizeof(*md) > md_args->len) {
		md_args->bytes_written = 0;
		return 0;
	}

	md = (struct cam_vfe_bus_ver3_mini_dump_data *)md_args->start_addr;
	md->clk_rate = cam_soc_util_get_applied_src_clk(&hw_info->soc_info, true);
	md->hw_idx = bus_priv->common_data.hw_intf->hw_idx;
	md->hw_state = hw_info->hw_state;
	bytes_written += sizeof(*md);
	md->wm = (struct cam_vfe_bus_ver3_wm_mini_dump *)
			((uint8_t *)md_args->start_addr + bytes_written);

	for (i = 0; i < bus_priv->num_out; i++) {
		rsrc_node = &bus_priv->vfe_out[i];
		rsrc_data = rsrc_node->res_priv;
		if (!rsrc_data)
			continue;

		for (j = 0; j < rsrc_data->num_wm; j++) {
			if (bytes_written + sizeof(*md_wm) > md_args->len)
				goto end;

			md_wm = &md->wm[k];
			wm = rsrc_data->wm_res[j].res_priv;
			md_wm->width  = wm->width;
			md_wm->index  = wm->index;
			md_wm->height = wm->height;
			md_wm->stride = wm->stride;
			md_wm->en_cfg = wm->en_cfg;
			md_wm->h_init = wm->h_init;
			md_wm->format = wm->format;
			md_wm->acquired_width = wm->acquired_width;
			md_wm->acquired_height = wm->acquired_height;
			md_wm->state = rsrc_node->res_state;
			scnprintf(md_wm->name, CAM_ISP_RES_NAME_LEN,
				"%s", rsrc_data->wm_res[j].res_name);
			k++;
			bytes_written += sizeof(*md_wm);
		}
	}
end:
	md->num_client = k;
	md_args->bytes_written = bytes_written;
	return 0;
}

static void *cam_vfe_bus_ver3_user_dump_info(
	void *dump_struct, uint8_t *addr_ptr)
{
	struct cam_vfe_bus_ver3_wm_resource_data  *wm = NULL;
	uint32_t                                  *addr;
	uint32_t                                   addr_status0;
	uint32_t                                   addr_status1;
	uint32_t                                   addr_status2;
	uint32_t                                   addr_status3;

	wm = (struct cam_vfe_bus_ver3_wm_resource_data *)dump_struct;

	addr_status0 = cam_io_r_mb(wm->common_data->mem_base +
		wm->hw_regs->addr_status_0);
	addr_status1 = cam_io_r_mb(wm->common_data->mem_base +
		wm->hw_regs->addr_status_1);
	addr_status2 = cam_io_r_mb(wm->common_data->mem_base +
		wm->hw_regs->addr_status_2);
	addr_status3 = cam_io_r_mb(wm->common_data->mem_base +
		wm->hw_regs->addr_status_3);

	addr = (uint32_t *)addr_ptr;

	*addr++ = wm->common_data->hw_intf->hw_idx;
	*addr++ = wm->index;
	*addr++ = addr_status0;
	*addr++ = addr_status1;
	*addr++ = addr_status2;
	*addr++ = addr_status3;

	return addr;
}

static int cam_vfe_bus_ver3_user_dump(
	struct cam_vfe_bus_ver3_priv *bus_priv,
	void *cmd_args)
{
	struct cam_isp_resource_node              *rsrc_node = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data      *rsrc_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data  *wm = NULL;
	struct cam_hw_info                        *hw_info = NULL;
	struct cam_isp_hw_dump_args               *dump_args;
	uint32_t                                   i, j = 0;
	int                                        rc = 0;


	if (!bus_priv || !cmd_args) {
		CAM_ERR(CAM_ISP, "Bus private data NULL");
		return -EINVAL;
	}

	hw_info = (struct cam_hw_info *)bus_priv->common_data.hw_intf->hw_priv;
	dump_args = (struct cam_isp_hw_dump_args *)cmd_args;

	if (hw_info->hw_state == CAM_HW_STATE_POWER_DOWN) {
		CAM_WARN(CAM_ISP,
			"VFE:%u  BUS powered down", bus_priv->common_data.core_index);
		return -EINVAL;
	}

	rc = cam_common_user_dump_helper(dump_args, cam_common_user_dump_clock,
		hw_info, sizeof(uint64_t), "CLK_RATE_PRINT:");

	if (rc) {
		CAM_ERR(CAM_ISP, "VFE:%u BUS VER3: Clock dump failed, rc: %d",
			bus_priv->common_data.core_index, rc);
		return rc;
	}

	for (i = 0; i < bus_priv->num_out; i++) {
		rsrc_node = &bus_priv->vfe_out[i];
		if (!rsrc_node)
			continue;

		if (rsrc_node->res_state < CAM_ISP_RESOURCE_STATE_RESERVED) {
			CAM_DBG(CAM_ISP,
				"VFE:%u BUS VER3: path inactive res ID: %d, continuing",
				bus_priv->common_data.core_index, rsrc_node->res_id);
			continue;
		}

		rsrc_data = rsrc_node->res_priv;
		if (!rsrc_data)
			continue;
		for (j = 0; j < rsrc_data->num_wm; j++) {

			wm = rsrc_data->wm_res[j].res_priv;
			if (!wm)
				continue;

			rc = cam_common_user_dump_helper(dump_args, cam_vfe_bus_ver3_user_dump_info,
				wm, sizeof(uint32_t), "VFE_BUS_CLIENT.%s.%d:",
				rsrc_data->wm_res[j].res_name,
				rsrc_data->common_data->core_index);

			if (rc) {
				CAM_ERR(CAM_ISP, "VFE:%u BUS VER3: Info dump failed, rc: %d",
					bus_priv->common_data.core_index, rc);
				return rc;
			}
		}
	}
	return 0;
}

static int cam_vfe_bus_ver3_handle_bus_irq(uint32_t    evt_id,
	struct cam_irq_th_payload                 *th_payload)
{
	struct cam_vfe_bus_ver3_priv          *bus_priv;
	int rc = 0;

	bus_priv = th_payload->handler_priv;
	CAM_DBG(CAM_ISP, "Enter VFE:%u", bus_priv->common_data.core_index);
	rc = cam_irq_controller_handle_irq(evt_id,
		bus_priv->common_data.bus_irq_controller, CAM_IRQ_EVT_GROUP_0);
	return (rc == IRQ_HANDLED) ? 0 : -EINVAL;
}

static int cam_vfe_bus_ver3_handle_rup_irq(uint32_t     evt_id,
	struct cam_irq_th_payload                 *th_payload)
{
	struct cam_vfe_bus_ver3_priv          *bus_priv;
	int rc = 0;

	bus_priv = th_payload->handler_priv;
	CAM_DBG(CAM_ISP, "Enter, VFE:%u", bus_priv->common_data.core_index);
	rc = cam_irq_controller_handle_irq(evt_id,
		bus_priv->common_data.bus_irq_controller, CAM_IRQ_EVT_GROUP_1);
	return (rc == IRQ_HANDLED) ? 0 : -EINVAL;
}

static int cam_vfe_bus_ver3_handle_err_irq_top_half(uint32_t evt_id,
	struct cam_irq_th_payload *th_payload)
{
	int i = 0, rc = 0;
	struct cam_vfe_bus_ver3_priv *bus_priv =
		th_payload->handler_priv;
	struct cam_vfe_bus_irq_evt_payload *evt_payload;

	CAM_ERR(CAM_ISP, "VFE:%u BUS Err IRQ",
		bus_priv->common_data.core_index);
	for (i = 0; i < th_payload->num_registers; i++) {
		CAM_ERR(CAM_ISP, "VFE:%u BUS IRQ status_%d: 0x%X",
		bus_priv->common_data.core_index, i,
			th_payload->evt_status_arr[i]);
	}
	cam_irq_controller_disable_irq(bus_priv->common_data.bus_irq_controller,
		bus_priv->error_irq_handle);

	rc  = cam_vfe_bus_ver3_get_evt_payload(&bus_priv->common_data,
		&evt_payload);
	if (rc)
		return rc;

	for (i = 0; i < th_payload->num_registers; i++)
		evt_payload->irq_reg_val[i] = th_payload->evt_status_arr[i];

	evt_payload->core_index = bus_priv->common_data.core_index;

	evt_payload->ccif_violation_status = cam_io_r_mb(
		bus_priv->common_data.mem_base +
		bus_priv->common_data.common_reg->ccif_violation_status);

	evt_payload->image_size_violation_status = cam_io_r_mb(
		bus_priv->common_data.mem_base +
		bus_priv->common_data.common_reg->image_size_violation_status);

	th_payload->evt_payload_priv = evt_payload;

	return rc;
}

static void cam_vfe_check_violations(
	char *error_type,
	uint32_t status,
	struct cam_vfe_bus_ver3_priv *bus_priv,
	uint64_t *out_port,
	struct cam_vfe_bus_irq_violation_type *violation_type)
{
	int i, j;
	struct cam_isp_resource_node       *rsrc_node = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data      *rsrc_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data  *wm_data   = NULL;
	struct cam_vfe_bus_ver3_common_data  *common_data = NULL;
	uint8_t                                *wm_name = NULL;

	if (!bus_priv) {
		CAM_ERR(CAM_ISP, "Invalid bus private data");
		return;
	}

	for (i = 0; i < bus_priv->num_out; i++) {
		rsrc_node = &bus_priv->vfe_out[i];
		rsrc_data = rsrc_node->res_priv;
		if (!rsrc_data) {
			CAM_ERR(CAM_ISP, "VFE:%u out data is null, res_id: %d",
				bus_priv->common_data.core_index, i);
			return;
		}

		for (j = 0; j < rsrc_data->num_wm; j++) {
			wm_data = rsrc_data->wm_res[j].res_priv;
			common_data = rsrc_data->common_data;
			wm_name = rsrc_data->wm_res[j].res_name;

			if (status & BIT(wm_data->index)) {
				CAM_INFO(CAM_ISP, "VFE:%u, %s Violation",
					bus_priv->common_data.core_index, error_type);
				cam_vfe_bus_ver3_print_wm_info(wm_data,
					common_data, wm_name);
				*out_port |= BIT_ULL(rsrc_node->res_id & 0xFF);

				/* check what type of violation it is*/
				switch (wm_data->index) {
				case 21:
				case 22:
					if (!strcmp(error_type, "Image Size"))
						violation_type->hwpd_violation = true;
					break;
				default:
					break;
				}
			}
		}
	}
}

static int cam_vfe_bus_ver3_handle_err_irq_bottom_half(
	void *handler_priv, void *evt_payload_priv)
{
	struct cam_vfe_bus_irq_evt_payload *evt_payload = evt_payload_priv;
	struct cam_vfe_bus_ver3_priv *bus_priv = handler_priv;
	struct cam_vfe_bus_ver3_common_data *common_data;
	struct cam_isp_hw_event_info evt_info;
	struct cam_isp_hw_error_event_info err_evt_info;
	struct cam_vfe_bus_irq_violation_type violation_type;
	uint32_t status = 0, image_size_violation = 0, ccif_violation = 0, constraint_violation = 0;
	uint64_t out_port_mask = 0;
	int idx = -1;

	if (!handler_priv || !evt_payload_priv)
		return -EINVAL;

	common_data = &bus_priv->common_data;

	status = evt_payload->irq_reg_val[CAM_IFE_IRQ_BUS_VER3_REG_STATUS0];
	image_size_violation = (status >> 31) & 0x1;
	ccif_violation = (status >> 30) & 0x1;
	constraint_violation = (status >> 28) & 0x1;

	CAM_ERR(CAM_ISP,
		"VFE:%u BUS error image size violation %d CCIF violation %d constraint violation %d",
		bus_priv->common_data.core_index, image_size_violation,
		ccif_violation, constraint_violation);
	CAM_INFO(CAM_ISP,
		"VFE:%u Image Size violation status 0x%X CCIF violation status 0x%X",
		bus_priv->common_data.core_index, evt_payload->image_size_violation_status,
		evt_payload->ccif_violation_status);

	memset(&evt_info, 0, sizeof(evt_info));
	memset(&err_evt_info, 0, sizeof(err_evt_info));
	memset(&violation_type, 0, sizeof(violation_type));

	if (image_size_violation || constraint_violation) {
		status = evt_payload->image_size_violation_status;
		if (!status)
			cam_vfe_bus_ver3_get_constraint_errors(bus_priv);
		else {
			cam_vfe_check_violations("Image Size", status, bus_priv,
				&out_port_mask, &violation_type);
		}
	}

	if (ccif_violation) {
		status = evt_payload->ccif_violation_status;
		cam_vfe_check_violations("CCIF", status, bus_priv,
			&out_port_mask, &violation_type);
	}

	if (image_size_violation && violation_type.hwpd_violation) {
		err_evt_info.err_mask |= CAM_VFE_IRQ_ERR_MASK_HWPD_VIOLATION;
		CAM_DBG(CAM_ISP, "HWPD image size violation");
	}

	cam_vfe_bus_ver3_put_evt_payload(common_data, &evt_payload);

	evt_info.hw_idx = common_data->core_index;
	evt_info.res_type = CAM_ISP_RESOURCE_VFE_OUT;
	evt_info.hw_type = CAM_ISP_HW_TYPE_VFE;
	err_evt_info.err_type = CAM_VFE_IRQ_STATUS_VIOLATION;
	evt_info.event_data = (void *)&err_evt_info;

	if (!common_data->event_cb)
		return 0;

	if (!out_port_mask) {
		/* No valid res_id found */
		evt_info.res_id = CAM_VFE_BUS_VER3_VFE_OUT_MAX;
		common_data->event_cb(common_data->priv, CAM_ISP_HW_EVENT_ERROR,
			(void *)&evt_info);
		return 0;
	}

	while (out_port_mask) {
		idx++;
		if (!(out_port_mask & 0x1)) {
			out_port_mask >>= 1;
			continue;
		}

		evt_info.res_id = CAM_ISP_IFE_OUT_RES_BASE + idx;
		common_data->event_cb(common_data->priv, CAM_ISP_HW_EVENT_ERROR,
			(void *)&evt_info);
		out_port_mask >>= 1;
	}
	return 0;
}

static void cam_vfe_bus_ver3_unsubscribe_init_irq(
	struct cam_vfe_bus_ver3_priv          *bus_priv)
{
	int rc = 0;

	if (bus_priv->error_irq_handle) {
		rc = cam_irq_controller_unsubscribe_irq(
			bus_priv->common_data.bus_irq_controller,
			bus_priv->error_irq_handle);
		if (rc)
			CAM_WARN(CAM_ISP, "VFE:%u failed to unsubscribe error irqs",
				bus_priv->common_data.core_index);

		bus_priv->error_irq_handle = 0;
	}

	if (bus_priv->bus_irq_handle) {
		rc = cam_irq_controller_unsubscribe_irq(
			bus_priv->common_data.vfe_irq_controller,
			bus_priv->bus_irq_handle);
		if (rc)
			CAM_WARN(CAM_ISP, "VFE:%u failed to unsubscribe top irq",
				bus_priv->common_data.core_index);

		bus_priv->bus_irq_handle = 0;
		cam_irq_controller_unregister_dependent(bus_priv->common_data.vfe_irq_controller,
		bus_priv->common_data.bus_irq_controller);
	}

	if (bus_priv->rup_irq_handle) {
		rc = cam_irq_controller_unsubscribe_irq(
			bus_priv->common_data.vfe_irq_controller,
			bus_priv->rup_irq_handle);
		if (rc)
			CAM_WARN(CAM_ISP, "VFE:%u failed to unsubscribe rup done irq",
				bus_priv->common_data.core_index);

		bus_priv->rup_irq_handle = 0;
	}

	bus_priv->common_data.init_irq_subscribed = false;
	CAM_DBG(CAM_ISP, "BUS init irq unsubscribed, VFE:%u", bus_priv->common_data.core_index);
}

static int cam_vfe_bus_ver3_subscribe_init_irq(
	struct cam_vfe_bus_ver3_priv          *bus_priv)
{
	uint32_t top_irq_reg_mask[3] = {0};

	/* Subscribe top IRQ */
	top_irq_reg_mask[0] = (1 << bus_priv->top_irq_shift);

	cam_irq_controller_register_dependent(
		bus_priv->common_data.vfe_irq_controller,
		bus_priv->common_data.bus_irq_controller,
		top_irq_reg_mask);

	bus_priv->bus_irq_handle = cam_irq_controller_subscribe_irq(
		bus_priv->common_data.vfe_irq_controller,
		CAM_IRQ_PRIORITY_4,
		top_irq_reg_mask,
		bus_priv,
		cam_vfe_bus_ver3_handle_bus_irq,
		NULL,
		NULL,
		NULL,
		CAM_IRQ_EVT_GROUP_0);

	if (bus_priv->bus_irq_handle < 1) {
		CAM_ERR(CAM_ISP, "VFE:%u Failed to subscribe BUS (buf_done) IRQ",
			bus_priv->common_data.core_index);
		bus_priv->bus_irq_handle = 0;
		return -EFAULT;
	}


	if (bus_priv->tasklet_info != NULL) {
		bus_priv->error_irq_handle = cam_irq_controller_subscribe_irq(
			bus_priv->common_data.bus_irq_controller,
			CAM_IRQ_PRIORITY_0,
			bus_error_irq_mask,
			bus_priv,
			cam_vfe_bus_ver3_handle_err_irq_top_half,
			cam_vfe_bus_ver3_handle_err_irq_bottom_half,
			bus_priv->tasklet_info,
			&tasklet_bh_api,
			CAM_IRQ_EVT_GROUP_0);

		if (bus_priv->error_irq_handle < 1) {
			CAM_ERR(CAM_ISP, "VFE:%u Failed to subscribe BUS Error IRQ",
				bus_priv->common_data.core_index);
			bus_priv->error_irq_handle = 0;
			return -EFAULT;
		}
	}

	if (bus_priv->common_data.supported_irq & CAM_VFE_HW_IRQ_CAP_RUP) {
		bus_priv->rup_irq_handle = cam_irq_controller_subscribe_irq(
			bus_priv->common_data.vfe_irq_controller,
			CAM_IRQ_PRIORITY_2,
			top_irq_reg_mask,
			bus_priv,
			cam_vfe_bus_ver3_handle_rup_irq,
			NULL,
			NULL,
			NULL,
			CAM_IRQ_EVT_GROUP_0);

		if (bus_priv->rup_irq_handle < 1) {
			CAM_ERR(CAM_ISP, "VFE:%u Failed to subscribe BUS (rup) IRQ",
				bus_priv->common_data.core_index);
			bus_priv->rup_irq_handle = 0;
			return -EFAULT;
		}
	}

	bus_priv->common_data.init_irq_subscribed = true;
	CAM_DBG(CAM_ISP, "VFE:%u BUS irq subscribed", bus_priv->common_data.core_index);
	return 0;
}

static void cam_vfe_bus_ver3_update_ubwc_meta_addr(
	uint32_t *reg_val_pair,
	uint32_t  *j,
	void     *regs,
	dma_addr_t  image_buf)
{
	struct cam_vfe_bus_ver3_reg_offset_ubwc_client *ubwc_regs;
	uint32_t temp = cam_smmu_is_expanded_memory() ?
		CAM_36BIT_INTF_GET_IOVA_BASE(image_buf) : image_buf;

	if (cam_smmu_is_expanded_memory() &&
		CAM_36BIT_INTF_GET_IOVA_OFFSET(image_buf)) {
		CAM_ERR(CAM_ISP, "Error, address not aligned! offset:0x%x",
			CAM_36BIT_INTF_GET_IOVA_OFFSET(image_buf));
	}

	if (!regs || !reg_val_pair || !j) {
		CAM_ERR(CAM_ISP, "Invalid args");
		goto end;
	}

	ubwc_regs = (struct cam_vfe_bus_ver3_reg_offset_ubwc_client *) regs;
	CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, *j, ubwc_regs->meta_addr, temp);

end:
	return;
}

static int cam_vfe_bus_ver3_update_ubwc_regs(
	struct cam_vfe_bus_ver3_wm_resource_data *wm_data,
	uint32_t *reg_val_pair,	uint32_t i, uint32_t *j)
{
	struct cam_vfe_bus_ver3_reg_offset_ubwc_client *ubwc_regs;
	int rc = 0;

	if (!wm_data || !reg_val_pair || !j) {
		CAM_ERR(CAM_ISP, "Invalid args");
		rc = -EINVAL;
		goto end;
	}

	ubwc_regs = (struct cam_vfe_bus_ver3_reg_offset_ubwc_client *)
		wm_data->hw_regs->ubwc_regs;

	CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, *j,
		wm_data->hw_regs->packer_cfg, wm_data->packer_cfg);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d packer cfg 0x%X",
		wm_data->common_data->core_index, wm_data->index, reg_val_pair[*j-1]);

	CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, *j,
		ubwc_regs->meta_cfg, wm_data->ubwc_meta_cfg);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d meta stride 0x%X",
		wm_data->common_data->core_index, wm_data->index, reg_val_pair[*j-1]);

	if (wm_data->common_data->disable_ubwc_comp) {
		wm_data->ubwc_mode_cfg &= ~ubwc_regs->ubwc_comp_en_bit;
		CAM_DBG(CAM_ISP,
			"Force disable UBWC compression on VFE:%u WM:%d",
			wm_data->common_data->core_index, wm_data->index);
	}

	CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, *j,
		ubwc_regs->mode_cfg, wm_data->ubwc_mode_cfg);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d ubwc_mode_cfg 0x%X",
		 wm_data->common_data->core_index, wm_data->index, reg_val_pair[*j-1]);

	CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, *j,
		ubwc_regs->ctrl_2, wm_data->ubwc_ctrl_2);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d ubwc_ctrl_2 0x%X",
		wm_data->common_data->core_index, wm_data->index, reg_val_pair[*j-1]);

	CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, *j,
		ubwc_regs->lossy_thresh0, wm_data->ubwc_lossy_threshold_0);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d lossy_thresh0 0x%X",
		wm_data->common_data->core_index, wm_data->index, reg_val_pair[*j-1]);

	CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, *j,
		ubwc_regs->lossy_thresh1, wm_data->ubwc_lossy_threshold_1);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d lossy_thresh1 0x%X",
		wm_data->common_data->core_index, wm_data->index, reg_val_pair[*j-1]);

	CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, *j,
		ubwc_regs->off_lossy_var, wm_data->ubwc_offset_lossy_variance);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d off_lossy_var 0x%X",
		wm_data->common_data->core_index, wm_data->index, reg_val_pair[*j-1]);

	/*
	 * If limit value >= 0xFFFF, limit configured by
	 * generic limiter blob
	 */
	if (wm_data->ubwc_bandwidth_limit < 0xFFFF) {
		CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, *j,
			ubwc_regs->bw_limit, wm_data->ubwc_bandwidth_limit);
		CAM_DBG(CAM_ISP, "VFE:%u WM:%d ubwc bw limit 0x%X",
			wm_data->common_data->core_index, wm_data->index,
			wm_data->ubwc_bandwidth_limit);
	}

end:
	return rc;
}

static int cam_vfe_bus_ver3_config_ubwc_regs(
	struct cam_vfe_bus_ver3_wm_resource_data *wm_data)
{
	struct cam_vfe_bus_ver3_reg_offset_ubwc_client *ubwc_regs =
		(struct cam_vfe_bus_ver3_reg_offset_ubwc_client *)
		wm_data->hw_regs->ubwc_regs;

	cam_io_w_mb(wm_data->packer_cfg, wm_data->common_data->mem_base +
		wm_data->hw_regs->packer_cfg);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d packer cfg:0x%x",
		wm_data->common_data->core_index, wm_data->index, wm_data->packer_cfg);

	cam_io_w_mb(wm_data->ubwc_meta_cfg,
		wm_data->common_data->mem_base + ubwc_regs->meta_cfg);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d meta stride:0x%x",
		wm_data->common_data->core_index, wm_data->index, wm_data->ubwc_meta_cfg);

	if (wm_data->common_data->disable_ubwc_comp) {
		wm_data->ubwc_mode_cfg &= ~ubwc_regs->ubwc_comp_en_bit;
		CAM_DBG(CAM_ISP,
			"Force disable UBWC compression on VFE:%u WM:%d",
			wm_data->common_data->core_index, wm_data->index);
	}

	cam_io_w_mb(wm_data->ubwc_mode_cfg,
		wm_data->common_data->mem_base + ubwc_regs->mode_cfg);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d ubwc_mode_cfg:0x%x",
		wm_data->common_data->core_index, wm_data->index, wm_data->ubwc_mode_cfg);

	cam_io_w_mb(wm_data->ubwc_ctrl_2,
		wm_data->common_data->mem_base + ubwc_regs->ctrl_2);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d ubwc_ctrl_2:0x%x",
		wm_data->common_data->core_index, wm_data->index, wm_data->ubwc_ctrl_2);

	cam_io_w_mb(wm_data->ubwc_lossy_threshold_0,
		wm_data->common_data->mem_base + ubwc_regs->lossy_thresh0);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d lossy_thresh0: 0x%x",
		wm_data->common_data->core_index, wm_data->index, wm_data->ubwc_lossy_threshold_0);

	cam_io_w_mb(wm_data->ubwc_lossy_threshold_1,
		wm_data->common_data->mem_base + ubwc_regs->lossy_thresh1);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d lossy_thresh0:0x%x",
		wm_data->common_data->core_index, wm_data->index, wm_data->ubwc_lossy_threshold_1);

	cam_io_w_mb(wm_data->ubwc_offset_lossy_variance,
		wm_data->common_data->mem_base + ubwc_regs->off_lossy_var);
	CAM_DBG(CAM_ISP, "VFE:%u WM:%d off_lossy_var:0x%x",
	wm_data->common_data->core_index, wm_data->index, wm_data->ubwc_offset_lossy_variance);

	/*
	 * If limit value >= 0xFFFF, limit configured by
	 * generic limiter blob
	 */
	if (wm_data->ubwc_bandwidth_limit < 0xFFFF) {
		cam_io_w_mb(wm_data->ubwc_bandwidth_limit,
			wm_data->common_data->mem_base + ubwc_regs->bw_limit);
		CAM_DBG(CAM_ISP, "VFE:%u WM:%d ubwc bw limit:0x%x",
			wm_data->common_data->core_index, wm_data->index,
			wm_data->ubwc_bandwidth_limit);
	}

	return 0;
}

static int cam_vfe_bus_ver3_config_wm(void *priv, void *cmd_args,
	uint32_t arg_size)
{
	struct cam_isp_hw_get_cmd_update *update_buf;
	struct cam_vfe_bus_ver3_priv *bus_priv;
	struct cam_vfe_bus_ver3_vfe_out_data *vfe_out_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data *wm_data = NULL;
	struct cam_vfe_bus_ver3_reg_offset_ubwc_client *ubwc_regs;
	uint32_t i, val, iova_addr, iova_offset, stride;
	dma_addr_t iova;

	update_buf = (struct cam_isp_hw_get_cmd_update *) cmd_args;
	bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;

	vfe_out_data = (struct cam_vfe_bus_ver3_vfe_out_data *)
		update_buf->res->res_priv;
	if (!vfe_out_data) {
		CAM_ERR(CAM_ISP, "Invalid data");
		return -EINVAL;
	}

	if (!vfe_out_data->limiter_enabled)
		CAM_WARN(CAM_ISP,
			"Configuring scratch for VFE:%u out_type: %u, with no BW limiter enabled",
			bus_priv->common_data.core_index, vfe_out_data->out_type);

	for (i = 0; i < vfe_out_data->num_wm; i++) {
		wm_data = vfe_out_data->wm_res[i].res_priv;
		ubwc_regs = (struct cam_vfe_bus_ver3_reg_offset_ubwc_client *)
			wm_data->hw_regs->ubwc_regs;

		stride =  update_buf->wm_update->stride;
		val = stride;
		val = ALIGNUP(val, 16);
		if (val != stride &&
			val != wm_data->stride)
			CAM_WARN(CAM_SFE, "VFE:%u Warning stride %u expected %u",
				bus_priv->common_data.core_index, stride, val);

		if (wm_data->stride != val || !wm_data->init_cfg_done) {
			cam_io_w_mb(stride, wm_data->common_data->mem_base +
				wm_data->hw_regs->image_cfg_2);
			wm_data->stride = val;
			CAM_DBG(CAM_ISP, "VFE:%u WM:%d image stride 0x%x",
				bus_priv->common_data.core_index, wm_data->index, stride);
		}

		/* WM Image address */
		iova = update_buf->wm_update->image_buf[i];
		if (cam_smmu_is_expanded_memory()) {
			iova_addr = CAM_36BIT_INTF_GET_IOVA_BASE(iova);
			iova_offset = CAM_36BIT_INTF_GET_IOVA_OFFSET(iova);

			cam_io_w_mb(iova_addr, wm_data->common_data->mem_base +
				wm_data->hw_regs->image_addr);
			cam_io_w_mb(iova_offset, wm_data->common_data->mem_base +
				wm_data->hw_regs->addr_cfg);

			CAM_DBG(CAM_ISP, "VFE:%u WM:%d image address 0x%x 0x%x",
				bus_priv->common_data.core_index, wm_data->index,
				iova_addr, iova_offset);
		} else {
			iova_addr = iova;
			cam_io_w_mb(iova_addr, wm_data->common_data->mem_base +
				wm_data->hw_regs->image_addr);
			CAM_DBG(CAM_ISP, "VFE:%u WM:%d image address 0x%X",
				bus_priv->common_data.core_index, wm_data->index, iova_addr);
		}

		if (wm_data->en_ubwc) {
			if (!wm_data->hw_regs->ubwc_regs) {
				CAM_ERR(CAM_ISP,
					"VFE:%u No UBWC register to configure for WM: %u",
					bus_priv->common_data.core_index, wm_data->index);
				return -EINVAL;
			}

			if (wm_data->ubwc_updated) {
				wm_data->ubwc_updated = false;
				cam_vfe_bus_ver3_config_ubwc_regs(wm_data);
			}

			cam_io_w_mb(iova_addr, wm_data->common_data->mem_base +
				ubwc_regs->meta_addr);
			CAM_DBG(CAM_ISP, "VFE:%u WM:%d meta address 0x%x",
				bus_priv->common_data.core_index, wm_data->index, iova_addr);
		}

		/* enable the WM */
		cam_io_w_mb(wm_data->en_cfg, wm_data->common_data->mem_base +
			wm_data->hw_regs->cfg);
		CAM_DBG(CAM_ISP, "VFE:%u WM:%d en_cfg 0x%x",
			bus_priv->common_data.core_index, wm_data->index, wm_data->en_cfg);
	}

	return 0;
}

static int cam_vfe_bus_ver3_update_wm(void *priv, void *cmd_args,
	uint32_t arg_size)
{
	struct cam_isp_hw_get_cmd_update         *update_buf;
	struct cam_vfe_bus_ver3_priv             *bus_priv;
	struct cam_buf_io_cfg                    *io_cfg;
	struct cam_vfe_bus_ver3_vfe_out_data     *vfe_out_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data *wm_data = NULL;
	struct cam_cdm_utils_ops                       *cdm_util_ops;
	uint32_t *reg_val_pair;
	uint32_t num_regval_pairs = 0;
	uint32_t i, j, size = 0;
	uint32_t frame_inc = 0, val;
	uint32_t iova_addr, iova_offset, image_buf_offset = 0, stride, slice_h;
	dma_addr_t iova;

	update_buf = (struct cam_isp_hw_get_cmd_update *) cmd_args;
	bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;

	vfe_out_data = (struct cam_vfe_bus_ver3_vfe_out_data *)
		update_buf->res->res_priv;
	if (!vfe_out_data || !vfe_out_data->common_data->cdm_util_ops) {
		CAM_ERR(CAM_ISP, "Invalid data");
		return -EINVAL;
	}

	cdm_util_ops = vfe_out_data->common_data->cdm_util_ops;
	if ((update_buf->wm_update->num_buf != vfe_out_data->num_wm) &&
		(!(update_buf->use_scratch_cfg))) {
		CAM_ERR(CAM_ISP,
			"VFE:%u Failed! Invalid number buffers:%d required:%d",
			bus_priv->common_data.core_index, update_buf->wm_update->num_buf,
			vfe_out_data->num_wm);
		return -EINVAL;
	}

	reg_val_pair = &vfe_out_data->common_data->io_buf_update[0];
	if (update_buf->use_scratch_cfg) {
		CAM_DBG(CAM_ISP, "VFE:%u Using scratch for IFE out_type: %u",
			bus_priv->common_data.core_index, vfe_out_data->out_type);

		if (!vfe_out_data->limiter_enabled)
			CAM_WARN(CAM_ISP,
				"Configuring scratch for VFE:%u out_type: %u, with no BW limiter enabled",
				bus_priv->common_data.core_index, vfe_out_data->out_type);
	} else {
		io_cfg = update_buf->wm_update->io_cfg;
	}

	for (i = 0, j = 0; i < vfe_out_data->num_wm; i++) {
		if (j >= (MAX_REG_VAL_PAIR_SIZE - MAX_BUF_UPDATE_REG_NUM * 2)) {
			CAM_ERR(CAM_ISP,
				"VFE:%u reg_val_pair %d exceeds the array limit %zu",
				bus_priv->common_data.core_index, j, MAX_REG_VAL_PAIR_SIZE);
			return -ENOMEM;
		}

		wm_data = vfe_out_data->wm_res[i].res_priv;

		/* Disable frame header in case it was previously enabled */
		if ((wm_data->en_cfg) & (1 << 2))
			wm_data->en_cfg &= ~(1 << 2);

		if (update_buf->wm_update->frame_header &&
			!update_buf->wm_update->fh_enabled &&
			wm_data->hw_regs->frame_header_addr) {

			wm_data->en_cfg |= 1 << 2;
			update_buf->wm_update->fh_enabled = true;
			if (cam_smmu_is_expanded_memory()) {
				iova_addr = CAM_36BIT_INTF_GET_IOVA_BASE(
					update_buf->wm_update->frame_header);
				iova_offset = CAM_36BIT_INTF_GET_IOVA_OFFSET(
					update_buf->wm_update->frame_header);
			} else {
				iova_addr = update_buf->wm_update->frame_header;
				iova_offset = 0;
			}

			if (iova_offset)
				CAM_ERR(CAM_ISP, "VFE:%u Error, address not aligned! offset:0x%x",
					bus_priv->common_data.core_index, iova_offset);
			CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
				wm_data->hw_regs->frame_header_addr, iova_addr);
			CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
				wm_data->hw_regs->frame_header_cfg,
				update_buf->wm_update->local_id);
			CAM_DBG(CAM_ISP,
				"VFE:%u WM: %d en_cfg 0x%x frame_header %pK local_id %u",
				bus_priv->common_data.core_index, wm_data->index, wm_data->en_cfg,
				update_buf->wm_update->frame_header,
				update_buf->wm_update->local_id);
		}

		CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
			wm_data->hw_regs->cfg, wm_data->en_cfg);
		CAM_DBG(CAM_ISP, "VFE:%u WM:%d %s en_cfg 0x%X",
			bus_priv->common_data.core_index, wm_data->index,
			vfe_out_data->wm_res[i].res_name, reg_val_pair[j-1]);

		val = (wm_data->height << 16) | wm_data->width;
		CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
			wm_data->hw_regs->image_cfg_0, val);
		CAM_DBG(CAM_ISP, "VFE:%u WM:%d image height and width 0x%X",
			bus_priv->common_data.core_index, wm_data->index, reg_val_pair[j-1]);

		/* For initial configuration program all bus registers */
		if (update_buf->use_scratch_cfg) {
			stride = update_buf->wm_update->stride;
			slice_h = update_buf->wm_update->slice_height;
		} else {
			stride = io_cfg->planes[i].plane_stride;
			slice_h = io_cfg->planes[i].slice_height;
		}

		val = stride;
		CAM_DBG(CAM_ISP, "VFE:%u val before stride %d",
			bus_priv->common_data.core_index, val);
		val = ALIGNUP(val, 16);
		if (val != stride)
			CAM_DBG(CAM_ISP, "VFE:%u Warning stride %u expected %u",
				bus_priv->common_data.core_index, stride, val);

		if (wm_data->stride != val || !wm_data->init_cfg_done) {
			CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
				wm_data->hw_regs->image_cfg_2,
				stride);
			wm_data->stride = val;
			CAM_DBG(CAM_ISP, "VFE:%u WM:%d image stride 0x%X",
				bus_priv->common_data.core_index, wm_data->index,
				reg_val_pair[j-1]);
		}

		/*
		 * For write master format update case, we check whether
		 * the updated format matches with the io config format,
		 * to help debug the wrong programming on a target who
		 * doesn't support 2 decode formats on CSID.
		 */
		if (wm_data->update_wm_format) {
			if ((!update_buf->use_scratch_cfg) &&
				(io_cfg->format != wm_data->format))
				CAM_WARN(CAM_ISP,
					"VFE:%u format mismatch, wm_data format:%u io_cfg format:%u",
					bus_priv->common_data.core_index,
					wm_data->format,
					io_cfg->format);

			val = wm_data->pack_fmt;
			CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
				wm_data->hw_regs->packer_cfg, val);
			CAM_DBG(CAM_ISP, "VFE:%u WM:%d image pack_fmt %d",
				bus_priv->common_data.core_index,
				wm_data->index,
				reg_val_pair[j-1]);
		}

		if (wm_data->en_ubwc) {
			if (!wm_data->hw_regs->ubwc_regs) {
				CAM_ERR(CAM_ISP,
					"VFE:%u No UBWC register to configure.",
					bus_priv->common_data.core_index);
				return -EINVAL;
			}
			if (wm_data->ubwc_updated) {
				wm_data->ubwc_updated = false;
				cam_vfe_bus_ver3_update_ubwc_regs(
					wm_data, reg_val_pair, i, &j);
			}

			/* UBWC meta address */
			cam_vfe_bus_ver3_update_ubwc_meta_addr(
				reg_val_pair, &j,
				wm_data->hw_regs->ubwc_regs,
				update_buf->wm_update->image_buf[i]);
			CAM_DBG(CAM_ISP, "VFE:%u WM:%d ubwc meta addr 0x%llx",
				bus_priv->common_data.core_index, wm_data->index,
				update_buf->wm_update->image_buf[i]);
		}

		frame_inc = stride * slice_h;
		if (wm_data->en_ubwc) {
			frame_inc = ALIGNUP(stride *
				slice_h, 4096);

			if (!update_buf->use_scratch_cfg) {
				frame_inc += io_cfg->planes[i].meta_size;
				CAM_DBG(CAM_ISP,
					"VFE:%u WM:%d frm %d: ht: %d stride %d meta: %d",
					bus_priv->common_data.core_index, wm_data->index,
					frame_inc, io_cfg->planes[i].slice_height,
					io_cfg->planes[i].plane_stride,
					io_cfg->planes[i].meta_size);
			}
		}

		if (!(wm_data->en_cfg & (0x3 << 16))) {
			CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
				wm_data->hw_regs->image_cfg_1, wm_data->h_init);
			CAM_DBG(CAM_ISP, "VFE:%u WM:%d h_init 0x%X",
				bus_priv->common_data.core_index, wm_data->index,
				reg_val_pair[j-1]);
		}

		if ((wm_data->en_ubwc) && (!update_buf->use_scratch_cfg))
			image_buf_offset = io_cfg->planes[i].meta_size;
		else if (wm_data->en_cfg & (0x3 << 16))
			image_buf_offset = wm_data->offset;
		else
			image_buf_offset = 0;

		/* WM Image address */
		iova = update_buf->wm_update->image_buf[i] +
			image_buf_offset;

		if (cam_smmu_is_expanded_memory()) {
			iova_addr = CAM_36BIT_INTF_GET_IOVA_BASE(iova);
			iova_offset = CAM_36BIT_INTF_GET_IOVA_OFFSET(iova);

			/* Align frame inc to 256 as well */
			frame_inc = CAM_36BIT_INTF_GET_IOVA_BASE(frame_inc);
			CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
				wm_data->hw_regs->image_addr, iova_addr);

			CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
				wm_data->hw_regs->addr_cfg, iova_offset);

			CAM_DBG(CAM_ISP, "VFE:%u WM:%d image address:0x%X image offset: 0x%X",
				bus_priv->common_data.core_index, wm_data->index,
				reg_val_pair[j-3], reg_val_pair[j-1]);
		} else {
			iova_addr = iova;

			CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
				wm_data->hw_regs->image_addr, iova_addr);

			CAM_DBG(CAM_ISP, "VFE:%u WM:%d image address 0x%X",
				bus_priv->common_data.core_index,
				wm_data->index, reg_val_pair[j-1]);
		}

		update_buf->wm_update->image_buf_offset[i] = image_buf_offset;

		CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
			wm_data->hw_regs->frame_incr, frame_inc);
		CAM_DBG(CAM_ISP, "VFE:%u WM:%d frame_inc: %d expanded_mem: %s",
			bus_priv->common_data.core_index, wm_data->index, reg_val_pair[j-1],
			CAM_BOOL_TO_YESNO(cam_smmu_is_expanded_memory));

		/* enable the WM */
		CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
			wm_data->hw_regs->cfg,
			wm_data->en_cfg);

		/* set initial configuration done */
		if (!wm_data->init_cfg_done)
			wm_data->init_cfg_done = true;
	}

	num_regval_pairs = j / 2;
	if (num_regval_pairs) {
		size = cdm_util_ops->cdm_required_size_reg_random(
			num_regval_pairs);

		/* cdm util returns dwords, need to convert to bytes */
		if ((size * 4) > update_buf->cmd.size) {
			CAM_ERR(CAM_ISP,
				"VFE:%u Failed! Buf size:%d insufficient, expected size:%d",
				bus_priv->common_data.core_index, update_buf->cmd.size, size);
			return -ENOMEM;
		}

		cdm_util_ops->cdm_write_regrandom(
			update_buf->cmd.cmd_buf_addr,
			num_regval_pairs, reg_val_pair);

		/* cdm util returns dwords, need to convert to bytes */
		update_buf->cmd.used_bytes = size * 4;
	} else {
		CAM_DBG(CAM_ISP,
			"No reg val pairs. num_wms: %u VFE:%u",
			vfe_out_data->num_wm, bus_priv->common_data.core_index);
		update_buf->cmd.used_bytes = 0;
	}

	return 0;
}

static int cam_vfe_bus_ver3_update_hfr(void *priv, void *cmd_args,
	uint32_t arg_size)
{
	struct cam_isp_hw_get_cmd_update         *update_hfr;
	struct cam_vfe_bus_ver3_priv             *bus_priv;
	struct cam_vfe_bus_ver3_vfe_out_data     *vfe_out_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data *wm_data = NULL;
	struct cam_isp_port_hfr_config           *hfr_cfg = NULL;
	struct cam_cdm_utils_ops                 *cdm_util_ops;
	uint32_t *reg_val_pair;
	uint32_t num_regval_pairs = 0;
	uint32_t  i, j, size = 0;

	update_hfr =  (struct cam_isp_hw_get_cmd_update *) cmd_args;
	bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;

	vfe_out_data = (struct cam_vfe_bus_ver3_vfe_out_data *)
		update_hfr->res->res_priv;

	if (!vfe_out_data || !vfe_out_data->common_data->cdm_util_ops) {
		CAM_ERR(CAM_ISP, "Invalid data");
		return -EINVAL;
	}

	cdm_util_ops = vfe_out_data->common_data->cdm_util_ops;
	reg_val_pair = &vfe_out_data->common_data->io_buf_update[0];
	hfr_cfg = (struct cam_isp_port_hfr_config *)update_hfr->data;

	for (i = 0, j = 0; i < vfe_out_data->num_wm; i++) {
		if (j >= (MAX_REG_VAL_PAIR_SIZE - MAX_BUF_UPDATE_REG_NUM * 2)) {
			CAM_ERR(CAM_ISP,
				"VFE:%u reg_val_pair %d exceeds the array limit %zu",
				bus_priv->common_data.core_index, j, MAX_REG_VAL_PAIR_SIZE);
			return -ENOMEM;
		}

		wm_data = vfe_out_data->wm_res[i].res_priv;

		/* Frame drop config is only applicable to full IFE */
		if (!bus_priv->common_data.is_lite) {
			if ((wm_data->framedrop_pattern !=
				hfr_cfg->framedrop_pattern) ||
				!wm_data->hfr_cfg_done) {
				CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
					wm_data->hw_regs->framedrop_pattern,
					hfr_cfg->framedrop_pattern);
				wm_data->framedrop_pattern = hfr_cfg->framedrop_pattern;
				CAM_DBG(CAM_ISP, "VFE:%u WM:%d framedrop pattern 0x%X",
					bus_priv->common_data.core_index, wm_data->index,
					wm_data->framedrop_pattern);
			}

			if (wm_data->framedrop_period != hfr_cfg->framedrop_period ||
				!wm_data->hfr_cfg_done) {
				CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
					wm_data->hw_regs->framedrop_period,
					hfr_cfg->framedrop_period);
				wm_data->framedrop_period = hfr_cfg->framedrop_period;
				CAM_DBG(CAM_ISP, "VFE:%u WM:%d framedrop period 0x%X",
					bus_priv->common_data.core_index, wm_data->index,
					wm_data->framedrop_period);
			}
		}

		if (wm_data->irq_subsample_period != hfr_cfg->subsample_period
			|| !wm_data->hfr_cfg_done) {
			CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
				wm_data->hw_regs->irq_subsample_period,
				hfr_cfg->subsample_period);
			wm_data->irq_subsample_period =
				hfr_cfg->subsample_period;
			CAM_DBG(CAM_ISP, "VFE:%u WM:%d irq subsample period 0x%X",
				bus_priv->common_data.core_index, wm_data->index,
				wm_data->irq_subsample_period);
		}

		if (wm_data->irq_subsample_pattern != hfr_cfg->subsample_pattern
			|| !wm_data->hfr_cfg_done) {
			CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
				wm_data->hw_regs->irq_subsample_pattern,
				hfr_cfg->subsample_pattern);
			wm_data->irq_subsample_pattern =
				hfr_cfg->subsample_pattern;
			CAM_DBG(CAM_ISP, "VFE:%u WM:%d irq subsample pattern 0x%X",
				bus_priv->common_data.core_index, wm_data->index,
				wm_data->irq_subsample_pattern);
		}

		/* set initial configuration done */
		if (!wm_data->hfr_cfg_done)
			wm_data->hfr_cfg_done = true;
	}

	num_regval_pairs = j / 2;

	if (num_regval_pairs) {
		size = cdm_util_ops->cdm_required_size_reg_random(
			num_regval_pairs);

		/* cdm util returns dwords, need to convert to bytes */
		if ((size * 4) > update_hfr->cmd.size) {
			CAM_ERR(CAM_ISP,
				"VFE:%u Failed! Buf size:%d insufficient, expected size:%d",
				bus_priv->common_data.core_index, update_hfr->cmd.size, size);
			return -ENOMEM;
		}

		cdm_util_ops->cdm_write_regrandom(
			update_hfr->cmd.cmd_buf_addr,
			num_regval_pairs, reg_val_pair);

		/* cdm util returns dwords, need to convert to bytes */
		update_hfr->cmd.used_bytes = size * 4;
	} else {
		update_hfr->cmd.used_bytes = 0;
		CAM_DBG(CAM_ISP,
			"VFE:%u No reg val pairs. num_wms: %u",
			 bus_priv->common_data.core_index, vfe_out_data->num_wm);
	}

	return 0;
}

static int cam_vfe_bus_ver3_update_ubwc_config_v2(void *cmd_args)
{
	struct cam_isp_hw_get_cmd_update         *update_ubwc;
	struct cam_vfe_bus_ver3_vfe_out_data     *vfe_out_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data *wm_data = NULL;
	struct cam_vfe_generic_ubwc_config       *ubwc_generic_cfg = NULL;
	struct cam_vfe_generic_ubwc_plane_config *ubwc_generic_plane_cfg = NULL;
	uint32_t                                  i;
	int                                       rc = 0;

	if (!cmd_args) {
		CAM_ERR(CAM_ISP, "Invalid args");
		rc = -EINVAL;
		goto end;
	}

	update_ubwc =  (struct cam_isp_hw_get_cmd_update *) cmd_args;

	vfe_out_data = (struct cam_vfe_bus_ver3_vfe_out_data *)
		update_ubwc->res->res_priv;

	if (!vfe_out_data || !vfe_out_data->common_data->cdm_util_ops) {
		CAM_ERR(CAM_ISP, "Invalid data");
		rc = -EINVAL;
		goto end;
	}

	ubwc_generic_cfg = (struct cam_vfe_generic_ubwc_config *)
		update_ubwc->data;

	for (i = 0; i < vfe_out_data->num_wm; i++) {

		wm_data = vfe_out_data->wm_res[i].res_priv;
		ubwc_generic_plane_cfg = &ubwc_generic_cfg->ubwc_plane_cfg[i];

		if (!wm_data->hw_regs->ubwc_regs) {
			CAM_ERR(CAM_ISP,
				"VFE:%u No UBWC register to configure.",
				vfe_out_data->common_data->core_index);
			rc = -EINVAL;
			goto end;
		}

		if (!wm_data->en_ubwc) {
			CAM_ERR(CAM_ISP, "VFE:%u UBWC Disabled",
				vfe_out_data->common_data->core_index);
			rc = -EINVAL;
			goto end;
		}

		if (wm_data->packer_cfg !=
			ubwc_generic_plane_cfg->packer_config ||
			!wm_data->init_cfg_done) {
			wm_data->packer_cfg =
				ubwc_generic_plane_cfg->packer_config;
			wm_data->ubwc_updated = true;
		}

		if ((!wm_data->is_dual) && ((wm_data->h_init !=
			ubwc_generic_plane_cfg->h_init) ||
			!wm_data->init_cfg_done)) {
			wm_data->h_init = ubwc_generic_plane_cfg->h_init;
			wm_data->ubwc_updated = true;
		}

		if (wm_data->ubwc_meta_cfg !=
			ubwc_generic_plane_cfg->meta_stride ||
			!wm_data->init_cfg_done) {
			wm_data->ubwc_meta_cfg =
				ubwc_generic_plane_cfg->meta_stride;
			wm_data->ubwc_updated = true;
		}

		if (wm_data->ubwc_mode_cfg !=
			ubwc_generic_plane_cfg->mode_config_0 ||
			!wm_data->init_cfg_done) {
			wm_data->ubwc_mode_cfg =
				ubwc_generic_plane_cfg->mode_config_0;
			wm_data->ubwc_updated = true;
		}

		if (wm_data->ubwc_ctrl_2 !=
			ubwc_generic_plane_cfg->ctrl_2 ||
			!wm_data->init_cfg_done) {
			wm_data->ubwc_ctrl_2 =
				ubwc_generic_plane_cfg->ctrl_2;
			wm_data->ubwc_updated = true;
		}

		if (wm_data->ubwc_lossy_threshold_0 !=
			ubwc_generic_plane_cfg->lossy_threshold_0 ||
			!wm_data->init_cfg_done) {
			wm_data->ubwc_lossy_threshold_0 =
				ubwc_generic_plane_cfg->lossy_threshold_0;
			wm_data->ubwc_updated = true;
		}

		if (wm_data->ubwc_lossy_threshold_1 !=
			ubwc_generic_plane_cfg->lossy_threshold_1 ||
			!wm_data->init_cfg_done) {
			wm_data->ubwc_lossy_threshold_1 =
				ubwc_generic_plane_cfg->lossy_threshold_1;
			wm_data->ubwc_updated = true;
		}

		if (wm_data->ubwc_offset_lossy_variance !=
			ubwc_generic_plane_cfg->lossy_var_offset ||
			!wm_data->init_cfg_done) {
			wm_data->ubwc_offset_lossy_variance =
				ubwc_generic_plane_cfg->lossy_var_offset;
			wm_data->ubwc_updated = true;
		}

		if (wm_data->ubwc_bandwidth_limit !=
			ubwc_generic_plane_cfg->bandwidth_limit ||
			!wm_data->init_cfg_done) {
			wm_data->ubwc_bandwidth_limit =
				ubwc_generic_plane_cfg->bandwidth_limit;
			wm_data->ubwc_updated = true;
		}
	}

end:
	return rc;
}

static int cam_vfe_bus_ver3_update_stripe_cfg(void *priv, void *cmd_args,
	uint32_t arg_size)
{
	struct cam_vfe_bus_ver3_priv                *bus_priv;
	struct cam_isp_hw_dual_isp_update_args      *stripe_args;
	struct cam_vfe_bus_ver3_vfe_out_data        *vfe_out_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data    *wm_data = NULL;
	struct cam_isp_dual_stripe_config           *stripe_config;
	uint32_t outport_id, ports_plane_idx, i;

	bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;
	stripe_args = (struct cam_isp_hw_dual_isp_update_args *)cmd_args;

	vfe_out_data = (struct cam_vfe_bus_ver3_vfe_out_data *)
		stripe_args->res->res_priv;

	if (!vfe_out_data) {
		CAM_ERR(CAM_ISP, "Failed! Invalid data");
		return -EINVAL;
	}

	outport_id = stripe_args->res->res_id & 0xFF;
	if (stripe_args->res->res_id < CAM_ISP_IFE_OUT_RES_BASE ||
		stripe_args->res->res_id >= bus_priv->max_out_res)
		return 0;

	ports_plane_idx = (stripe_args->split_id *
	(stripe_args->dual_cfg->num_ports * CAM_PACKET_MAX_PLANES)) +
	(outport_id * CAM_PACKET_MAX_PLANES);
	for (i = 0; i < vfe_out_data->num_wm; i++) {
		wm_data = vfe_out_data->wm_res[i].res_priv;
		stripe_config = (struct cam_isp_dual_stripe_config  *)
			&stripe_args->dual_cfg->stripes_flex[ports_plane_idx + i];
		wm_data->width = stripe_config->width;

		/*
		 * UMD sends buffer offset address as offset for clients
		 * programmed to operate in frame/index based mode and h_init
		 * value as offset for clients programmed to operate in line
		 * based mode.
		 */

		if (wm_data->en_cfg & (0x3 << 16))
			wm_data->offset = stripe_config->offset;
		else
			wm_data->h_init = stripe_config->offset;

		CAM_DBG(CAM_ISP,
			"VFE:%u out_type:0x%X WM:%d width:%d offset:0x%X h_init:%d",
			vfe_out_data->common_data->core_index, stripe_args->res->res_id,
			wm_data->index, wm_data->width, wm_data->offset, wm_data->h_init);
	}

	return 0;
}

static int cam_vfe_bus_ver3_update_wm_config(
	void                                        *cmd_args)
{
	int                                          i;
	struct cam_isp_hw_get_cmd_update            *wm_config_update;
	struct cam_vfe_bus_ver3_vfe_out_data        *vfe_out_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data    *wm_data = NULL;
	struct cam_isp_vfe_wm_config                *wm_config = NULL;
	enum   cam_vfe_bus_ver3_packer_format        packer_fmt =
		PACKER_FMT_VER3_MAX;

	if (!cmd_args) {
		CAM_ERR(CAM_ISP, "Invalid args");
		return -EINVAL;
	}

	wm_config_update = cmd_args;
	vfe_out_data = wm_config_update->res->res_priv;
	wm_config = (struct cam_isp_vfe_wm_config  *)
		wm_config_update->data;

	if (!vfe_out_data || !vfe_out_data->common_data->cdm_util_ops || !wm_config) {
		CAM_ERR(CAM_ISP, "Invalid data");
		return -EINVAL;
	}

	for (i = 0; i < vfe_out_data->num_wm; i++) {
		wm_data = vfe_out_data->wm_res[i].res_priv;

		if (wm_config->wm_mode >= CAM_VFE_WM_MODE_MAX) {
			CAM_ERR(CAM_ISP, "VFE:%u Invalid wm_mode: 0x%X WM:%d",
				vfe_out_data->common_data->core_index, wm_config->wm_mode,
				wm_data->index);
			return -EINVAL;
		}

		wm_data->en_cfg = ((wm_config->wm_mode << 16) |
			(wm_config->virtual_frame_en << 1) | 0x1);
		wm_data->width  = wm_config->width;

		if (i == PLANE_C)
			wm_data->height = wm_config->height / 2;
		else
			wm_data->height = wm_config->height;

		/*
		 * For RAW10/RAW12/RAW14 sensor mode seamless switch case,
		 * the format may be changed on RDIs, so we update RDIs WM data.
		 */
		wm_data->update_wm_format = false;
		if ((wm_config->packer_format != CAM_FORMAT_BASE) &&
			(wm_data->format != wm_config->packer_format)) {
			wm_data->update_wm_format = true;
			wm_data->format = wm_config->packer_format;
			packer_fmt = cam_vfe_bus_ver3_get_packer_fmt(
				wm_config->packer_format, wm_data->index);

			/* Reconfigure only for valid packer fmt */
			if (packer_fmt != PACKER_FMT_VER3_MAX) {
				if ((vfe_out_data->out_type >= CAM_VFE_BUS_VER3_VFE_OUT_RDI0) &&
					(vfe_out_data->out_type <= CAM_VFE_BUS_VER3_VFE_OUT_RDI5)) {
					if (wm_config->wm_mode != wm_data->wm_mode) {
						wm_data->wm_mode = wm_config->wm_mode;
						cam_vfe_bus_ver3_config_rdi_wm(wm_data);
					}
				}

				/* LSB aligned for plain type format */
				switch (wm_config->packer_format) {
				case CAM_FORMAT_PLAIN16_10:
				case CAM_FORMAT_PLAIN16_12:
				case CAM_FORMAT_PLAIN16_14:
				case CAM_FORMAT_PLAIN16_16:
					packer_fmt |=
						(1 << wm_data->common_data->pack_align_shift);
					wm_data->pack_fmt = packer_fmt;
					break;
				default:
					break;
				}
			} else {
				CAM_ERR(CAM_ISP, "VFE:%u Invalid format:%d",
					vfe_out_data->common_data->core_index,
					wm_config->packer_format);
				return -EINVAL;
			}

			CAM_DBG(CAM_ISP,
				"VFE:%u WM:%d update format:%d pack_fmt:%d",
				vfe_out_data->common_data->core_index, wm_data->index,
				wm_config->packer_format, wm_data->pack_fmt);
		}

		CAM_DBG(CAM_ISP,
			"VFE:%u WM:%d en_cfg:0x%X height:%d width:%d stride:%d",
			vfe_out_data->common_data->core_index, wm_data->index, wm_data->en_cfg,
			wm_data->height, wm_data->width, wm_data->stride);
	}

	return 0;
}

static int cam_vfe_bus_update_bw_limiter(
	void *priv, void *cmd_args, uint32_t arg_size)
{
	struct cam_isp_hw_get_cmd_update         *wm_config_update;
	struct cam_vfe_bus_ver3_vfe_out_data     *vfe_out_data = NULL;
	struct cam_cdm_utils_ops                 *cdm_util_ops;
	struct cam_vfe_bus_ver3_wm_resource_data *wm_data = NULL;
	struct cam_isp_wm_bw_limiter_config      *wm_bw_limit_cfg = NULL;
	uint32_t                                  counter_limit = 0, reg_val = 0;
	uint32_t                                 *reg_val_pair, num_regval_pairs = 0;
	uint32_t                                  i, j, size = 0;
	bool                                      limiter_enabled = false;

	wm_config_update = (struct cam_isp_hw_get_cmd_update *) cmd_args;
	wm_bw_limit_cfg  = (struct cam_isp_wm_bw_limiter_config  *)
			wm_config_update->data;

	vfe_out_data = (struct cam_vfe_bus_ver3_vfe_out_data *)
		wm_config_update->res->res_priv;
	if (!vfe_out_data || !vfe_out_data->common_data->cdm_util_ops) {
		CAM_ERR(CAM_ISP, "Invalid data");
		return -EINVAL;
	}

	cdm_util_ops = vfe_out_data->common_data->cdm_util_ops;
	reg_val_pair = &vfe_out_data->common_data->io_buf_update[0];
	for (i = 0, j = 0; i < vfe_out_data->num_wm; i++) {
		if (j >= (MAX_REG_VAL_PAIR_SIZE - (MAX_BUF_UPDATE_REG_NUM * 2))) {
			CAM_ERR(CAM_ISP,
				"VFE:%u reg_val_pair %d exceeds the array limit %zu for WM idx %d",
				vfe_out_data->common_data->core_index, j,
				MAX_REG_VAL_PAIR_SIZE, i);
			return -ENOMEM;
		}

		/* Num WMs needs to match max planes */
		if (i >= CAM_PACKET_MAX_PLANES) {
			CAM_WARN(CAM_ISP,
				"VFE:%u Num of WMs: %d exceeded max planes",
				vfe_out_data->common_data->core_index, i);
			goto add_reg_pair;
		}

		wm_data = (struct cam_vfe_bus_ver3_wm_resource_data *)
			vfe_out_data->wm_res[i].res_priv;
		if (!wm_data->hw_regs->bw_limiter_addr) {
			CAM_ERR(CAM_ISP,
				"VFE:%u WM: %d %s has no support for bw limiter",
				vfe_out_data->common_data->core_index, wm_data->index,
				vfe_out_data->wm_res[i].res_name);
			return -EINVAL;
		}

		counter_limit = wm_bw_limit_cfg->counter_limit[i];

		/* Validate max counter limit */
		if (counter_limit >
			wm_data->common_data->max_bw_counter_limit) {
			CAM_WARN(CAM_ISP,
				"VFE:%u Invalid counter limit: 0x%x capping to max: 0x%x",
				vfe_out_data->common_data->core_index,
				wm_bw_limit_cfg->counter_limit[i],
				wm_data->common_data->max_bw_counter_limit);
			counter_limit =
				wm_data->common_data->max_bw_counter_limit;
		}

		if (wm_bw_limit_cfg->enable_limiter && counter_limit) {
			reg_val = 1;
			reg_val |= (counter_limit << 1);
			limiter_enabled = true;
		} else {
			reg_val = 0;
		}

		CAM_VFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
			wm_data->hw_regs->bw_limiter_addr, reg_val);
		CAM_DBG(CAM_ISP, "VFE:%u WM: %d for %s bw_limter: 0x%x",
			vfe_out_data->common_data->core_index, wm_data->index,
			vfe_out_data->wm_res[i].res_name,
			reg_val_pair[j-1]);
	}

add_reg_pair:

	num_regval_pairs = j / 2;

	if (num_regval_pairs) {
		size = cdm_util_ops->cdm_required_size_reg_random(
			num_regval_pairs);

		/* cdm util returns dwords, need to convert to bytes */
		if ((size * 4) > wm_config_update->cmd.size) {
			CAM_ERR(CAM_ISP,
				"VFE:%u Failed! Buf size:%d insufficient, expected size:%d",
				vfe_out_data->common_data->core_index,
				wm_config_update->cmd.size, size);
			return -ENOMEM;
		}

		cdm_util_ops->cdm_write_regrandom(
			wm_config_update->cmd.cmd_buf_addr, num_regval_pairs,
			reg_val_pair);

		/* cdm util returns dwords, need to convert to bytes */
		wm_config_update->cmd.used_bytes = size * 4;
	} else {
		CAM_DBG(CAM_ISP,
			"VFE:%u No reg val pairs. num_wms: %u",
			vfe_out_data->common_data->core_index, vfe_out_data->num_wm);
		wm_config_update->cmd.used_bytes = 0;
	}

	vfe_out_data->limiter_enabled = limiter_enabled;
	return 0;
}

static int cam_vfe_bus_ver3_mc_ctxt_sel(
	void *priv, void *cmd_args, uint32_t arg_size)

{
	struct cam_vfe_bus_ver3_priv              *bus_priv;
	struct cam_isp_hw_get_cmd_update          *mc_config;
	struct cam_cdm_utils_ops                  *cdm_util_ops = NULL;
	struct cam_vfe_bus_ver3_reg_offset_common *common_reg;
	uint32_t                                   reg_val[2], ctxt_id = 0;
	uint32_t                                   size = 0;

	if (!priv || !cmd_args) {
		CAM_ERR(CAM_ISP, "Invalid args priv %x cmd_args %x",
			priv, cmd_args);
		return -EINVAL;
	}

	bus_priv = (struct cam_vfe_bus_ver3_priv  *)priv;
	mc_config = (struct cam_isp_hw_get_cmd_update *)cmd_args;
	ctxt_id  = *((uint32_t *)mc_config->data);

	common_reg = bus_priv->common_data.common_reg;
	reg_val[0] = common_reg->ctxt_sel;
	reg_val[1] = ctxt_id << common_reg->mc_write_sel_shift;

	cdm_util_ops = bus_priv->common_data.cdm_util_ops;
	size = cdm_util_ops->cdm_required_size_reg_random(1);

	/* cdm util returns dwords, need to convert to bytes */
	if ((size * 4) > mc_config->cmd.size) {
		CAM_ERR(CAM_ISP,
			"Failed! Buf size:%d insufficient, expected size:%d",
			mc_config->cmd.size, size);
		return -ENOMEM;
	}

	cdm_util_ops->cdm_write_regrandom(
		mc_config->cmd.cmd_buf_addr, 1, reg_val);

	/* cdm util returns dwords, need to convert to bytes */
	mc_config->cmd.used_bytes = size * 4;

	return 0;
}

static int cam_vfe_bus_ver3_irq_inject(
	void *priv, void *cmd_args, uint32_t arg_size)
{
	struct cam_vfe_bus_ver3_priv      *bus_priv = NULL;
	struct cam_hw_soc_info            *soc_info = NULL;
	struct cam_vfe_bus_ver3_hw_info   *bus_hw_info = NULL;
	struct cam_isp_irq_inject_param   *inject_params = NULL;
	struct cam_irq_register_set       *inject_reg = NULL;

	if (!cmd_args) {
		CAM_ERR(CAM_ISP, "Invalid params");
		return -EINVAL;
	}

	bus_priv = (struct cam_vfe_bus_ver3_priv  *)priv;
	soc_info = bus_priv->common_data.soc_info;
	bus_hw_info = (struct cam_vfe_bus_ver3_hw_info *)bus_priv->bus_hw_info;
	inject_params = (struct cam_isp_irq_inject_param *)cmd_args;

	if (inject_params->reg_unit ==
		CAM_ISP_IFE_0_BUS_WR_INPUT_IF_IRQ_SET_0_REG)
		inject_reg = &bus_hw_info->common_reg.irq_reg_info.irq_reg_set[0];
	else if (inject_params->reg_unit ==
		CAM_ISP_IFE_0_BUS_WR_INPUT_IF_IRQ_SET_1_REG)
		inject_reg = &bus_hw_info->common_reg.irq_reg_info.irq_reg_set[1];
	else
		return -EINVAL;

	if (!inject_reg) {
		CAM_INFO(CAM_ISP, "Invalid inject_reg");
		return -EINVAL;
	}

	cam_io_w_mb(inject_params->irq_mask,
		soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base +
		inject_reg->set_reg_offset);
	cam_io_w_mb(0x10, soc_info->reg_map[VFE_CORE_BASE_IDX].mem_base +
		bus_hw_info->common_reg.irq_reg_info.global_irq_cmd_offset);
	CAM_INFO(CAM_ISP, "Injected : irq_mask %#x set_reg_offset %#x",
		inject_params->irq_mask, inject_reg->set_reg_offset);

	return 0;
}

static int cam_vfe_bus_ver3_dump_irq_desc(
	void *priv, void *cmd_args, uint32_t arg_size)
{
	int                                   i, offset = 0;
	int                                   num_irq_desc = 0;
	struct cam_vfe_bus_ver3_priv         *bus_priv = NULL;
	struct cam_vfe_bus_ver3_hw_info      *bus_hw_info = NULL;
	struct cam_isp_irq_inject_param      *inject_params = NULL;
	struct cam_vfe_bus_ver3_err_irq_desc *err_irq_desc = NULL;

	if (!cmd_args) {
		CAM_ERR(CAM_ISP, "Invalid params");
		return -EINVAL;
	}

	bus_priv = (struct cam_vfe_bus_ver3_priv *)priv;
	bus_hw_info = (struct cam_vfe_bus_ver3_hw_info *)bus_priv->bus_hw_info;
	inject_params = (struct cam_isp_irq_inject_param *)cmd_args;

	if (inject_params->reg_unit ==
			CAM_ISP_IFE_0_BUS_WR_INPUT_IF_IRQ_SET_0_REG) {
		err_irq_desc = bus_hw_info->bus_err_desc_0;
		num_irq_desc = bus_hw_info->num_bus_errors_0;
	} else if (inject_params->reg_unit ==
			CAM_ISP_IFE_0_BUS_WR_INPUT_IF_IRQ_SET_1_REG) {
		err_irq_desc = bus_hw_info->bus_err_desc_1;
		num_irq_desc = bus_hw_info->num_bus_errors_1;
	} else
		return -EINVAL;

	offset += scnprintf(inject_params->line_buf + offset,
		LINE_BUFFER_LEN - offset,
		"Printing executable IRQ for hw_type: VFE reg_unit: %d\n",
		inject_params->reg_unit);

	for (i = 0; i < num_irq_desc; i++)
		offset += scnprintf(inject_params->line_buf + offset,
			LINE_BUFFER_LEN - offset, "%#12x : %s - %s\n",
			err_irq_desc[i].bitmask,
			err_irq_desc[i].err_name,
			err_irq_desc[i].desc);

	return 0;
}


static int cam_vfe_bus_ver3_update_res_wm(
	struct cam_vfe_bus_ver3_priv           *ver3_bus_priv,
	struct cam_vfe_hw_vfe_out_acquire_args *out_acq_args,
	void                                   *tasklet,
	enum cam_vfe_bus_ver3_vfe_out_type      vfe_out_res_id,
	enum cam_vfe_bus_plane_type             plane,
	struct cam_isp_resource_node           *wm_res,
	enum cam_vfe_bus_ver3_comp_grp_type   *comp_grp_id)
{
	int32_t wm_idx = 0, rc;
	struct cam_vfe_bus_ver3_wm_resource_data  *rsrc_data = NULL;
	char wm_mode[50];

	memset(wm_mode, '\0', sizeof(wm_mode));

	wm_res->is_per_port_acquire = false;
	rsrc_data = wm_res->res_priv;
	wm_idx = rsrc_data->index;
	rsrc_data->format = out_acq_args->out_port_info->format;
	rsrc_data->use_wm_pack = out_acq_args->use_wm_pack;
	rsrc_data->pack_fmt = cam_vfe_bus_ver3_get_packer_fmt(rsrc_data->format,
		wm_idx);

	rsrc_data->width = out_acq_args->out_port_info->width;
	rsrc_data->height = out_acq_args->out_port_info->height;
	rsrc_data->acquired_width = out_acq_args->out_port_info->width;
	rsrc_data->acquired_height = out_acq_args->out_port_info->height;
	rsrc_data->is_dual = out_acq_args->is_dual;

	/* Set WM offset value to default */
	rsrc_data->offset  = 0;
	CAM_DBG(CAM_ISP, "WM:%d width %d height %d is_per_port:%d",
		rsrc_data->index, rsrc_data->width,
		rsrc_data->height, wm_res->is_per_port_acquire);

	rc = cam_vfe_bus_ver3_res_update_config_wm(ver3_bus_priv, vfe_out_res_id,
		plane, wm_res, comp_grp_id, wm_mode, sizeof(wm_mode));
	if (rc)
		return rc;

	wm_res->tasklet_info = tasklet;

	CAM_DBG(CAM_ISP,
		"VFE:%d WM:%d %s processed width:%d height:%d stride:%d format:0x%X en_ubwc:%d %s",
		rsrc_data->common_data->core_index, rsrc_data->index,
		wm_res->res_name, rsrc_data->width, rsrc_data->height,
		rsrc_data->stride, rsrc_data->format, rsrc_data->en_ubwc,
		wm_mode);
	return 0;
}

static int cam_vfe_bus_ver3_update_res_comp_grp(
	struct cam_vfe_bus_ver3_priv         *ver3_bus_priv,
	void                                *tasklet,
	uint32_t                             is_dual,
	uint32_t                             is_master,
	struct cam_isp_resource_node       **comp_grp,
	struct cam_vfe_bus_ver3_comp_grp_acquire_args *comp_acq_args)
{
	struct cam_isp_resource_node           *comp_grp_local = NULL;
	struct cam_vfe_bus_ver3_comp_grp_data  *rsrc_data = NULL;
	bool previously_acquired = false;

	/* Check if matching comp_grp has already been acquired */
	previously_acquired = cam_vfe_bus_ver3_match_comp_grp(
		ver3_bus_priv, &comp_grp_local, comp_acq_args->comp_grp_id);

	if (!comp_grp_local || !previously_acquired) {
		CAM_ERR(CAM_ISP, "Invalid comp_grp:%d",
			comp_acq_args->comp_grp_id);
		return -ENODEV;
	}

	rsrc_data = comp_grp_local->res_priv;
	/* Do not support runtime change in composite mask */
	CAM_DBG(CAM_ISP, "Update res VFE:%d comp_grp:%u",
		rsrc_data->common_data->core_index, rsrc_data->comp_grp_type);

	rsrc_data->acquire_dev_cnt++;
	rsrc_data->composite_mask |= comp_acq_args->composite_mask;
	*comp_grp = comp_grp_local;
	comp_grp_local->is_per_port_acquire = false;

	return 0;
}

static int cam_vfe_bus_ver3_update_res_vfe_out(void *bus_priv, void *acquire_args,
	uint32_t args_size)
{
	int                                     rc = -ENODEV;
	int                                     i;
	enum cam_vfe_bus_ver3_vfe_out_type      vfe_out_res_id;
	uint32_t                                format;
	struct cam_vfe_bus_ver3_priv           *ver3_bus_priv = bus_priv;
	struct cam_vfe_acquire_args            *acq_args;
	struct cam_vfe_resource_update		   *res_update_args;
	struct cam_vfe_hw_vfe_out_acquire_args *out_acquire_args;
	struct cam_isp_resource_node           *rsrc_node = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data   *rsrc_data = NULL;
	uint32_t                                secure_caps = 0, mode;
	struct cam_vfe_bus_ver3_comp_grp_acquire_args comp_acq_args = {0};
	uint32_t       outmap_index = CAM_VFE_BUS_VER3_VFE_OUT_MAX;

	if (!bus_priv || !acquire_args) {
		CAM_ERR(CAM_ISP, "Invalid Param");
		return -EINVAL;
	}

	res_update_args = (struct cam_vfe_resource_update *)acquire_args;
	acq_args = res_update_args->vfe_acquire;

	out_acquire_args = &acq_args->vfe_out;
	format = out_acquire_args->out_port_info->format;

	CAM_DBG(CAM_ISP, "VFE:%d Acquire out_type:0x%X",
		ver3_bus_priv->common_data.core_index,
		out_acquire_args->out_port_info->acquired_res_type);

	vfe_out_res_id = cam_vfe_bus_ver3_get_out_res_id_and_index(
				ver3_bus_priv,
				out_acquire_args->out_port_info->acquired_res_type,
				&outmap_index);
	if ((vfe_out_res_id == CAM_VFE_BUS_VER3_VFE_OUT_MAX) ||
		(outmap_index >= ver3_bus_priv->num_out)) {
		CAM_WARN(CAM_ISP,
			"target does not support req res id :0x%x outtype:%d index:%d num: %d",
			out_acquire_args->out_port_info->acquired_res_type,
			vfe_out_res_id, outmap_index, ver3_bus_priv->num_out);
		return -ENODEV;
	}

	rsrc_node = &ver3_bus_priv->vfe_out[outmap_index];

	rsrc_data = rsrc_node->res_priv;
	rsrc_data->common_data->event_cb = acq_args->event_cb;
	rsrc_data->common_data->priv = acq_args->priv;
	rsrc_data->common_data->disable_ubwc_comp =
		out_acquire_args->disable_ubwc_comp;
	rsrc_data->priv = acq_args->priv;
	rsrc_data->bus_priv = ver3_bus_priv;
	rsrc_data->limiter_enabled = false;

	comp_acq_args.composite_mask = (1ULL << vfe_out_res_id);

	/* for some hw versions, buf done is not received from vfe but
	 * from IP external to VFE. In such case, we get the controller
	 * from hw manager and assign it here
	 */
	if (!(ver3_bus_priv->common_data.supported_irq &
			CAM_VFE_HW_IRQ_CAP_BUF_DONE))
		rsrc_data->common_data->buf_done_controller =
			acq_args->buf_done_controller;

	secure_caps = cam_vfe_bus_ver3_can_be_secure(
		rsrc_data->out_type);
	mode = out_acquire_args->out_port_info->secure_mode;
	mutex_lock(&rsrc_data->common_data->bus_mutex);
	if (secure_caps) {
		if (!rsrc_data->common_data->num_sec_out) {
			rsrc_data->secure_mode = mode;
			rsrc_data->common_data->secure_mode = mode;
		} else {
			if (mode == rsrc_data->common_data->secure_mode) {
				rsrc_data->secure_mode =
					rsrc_data->common_data->secure_mode;
			} else {
				rc = -EINVAL;
				CAM_ERR_RATE_LIMIT(CAM_ISP,
					"Mismatch: Acquire mode[%d], drvr mode[%d]",
					rsrc_data->common_data->secure_mode,
					mode);
				mutex_unlock(
					&rsrc_data->common_data->bus_mutex);
				return -EINVAL;
			}
		}
		rsrc_data->common_data->num_sec_out++;
	}
	mutex_unlock(&rsrc_data->common_data->bus_mutex);

	ver3_bus_priv->tasklet_info = acq_args->tasklet;
	rsrc_node->res_id = out_acquire_args->out_port_info->acquired_res_type;
	rsrc_node->tasklet_info = acq_args->tasklet;
	rsrc_node->cdm_ops = out_acquire_args->cdm_ops;
	rsrc_data->common_data->event_cb = acq_args->event_cb;

	rsrc_node->tasklet_info = acq_args->tasklet;
	rsrc_data->format = out_acquire_args->out_port_info->format;

	if ((rsrc_data->out_type == CAM_VFE_BUS_VER3_VFE_OUT_FD) &&
		(rsrc_data->format == CAM_FORMAT_Y_ONLY))
		rsrc_data->num_wm = 1;

	/* Update WM params and retrieve COMP GRP ID */
	for (i = 0; i < rsrc_data->num_wm; i++) {
		rc = cam_vfe_bus_ver3_update_res_wm(ver3_bus_priv,
			out_acquire_args,
			acq_args->tasklet,
			vfe_out_res_id,
			i,
			&rsrc_data->wm_res[i],
			&comp_acq_args.comp_grp_id);
		if (rc) {
			CAM_ERR(CAM_ISP,
				"Failed to update resource WM VFE:%d out_type:%d rc:%d",
				rsrc_data->common_data->core_index,
				vfe_out_res_id, rc);
			return -EINVAL;
		}
	}


	/* Update composite group data using COMP GRP ID */
	rc = cam_vfe_bus_ver3_update_res_comp_grp(ver3_bus_priv,
		acq_args->tasklet,
		out_acquire_args->is_dual,
		out_acquire_args->is_master,
		&rsrc_data->comp_grp,
		&comp_acq_args);
	if (rc) {
		CAM_ERR(CAM_ISP,
			"Failed to update resource comp_grp VFE:%d out_typp:%d rc:%d",
			rsrc_data->common_data->core_index,
			vfe_out_res_id, rc);
		return -EINVAL;
	}

	rsrc_node->is_per_port_acquire = false;
	rsrc_data->is_dual = out_acquire_args->is_dual;
	rsrc_data->is_master = out_acquire_args->is_master;
	out_acquire_args->comp_grp_id = comp_acq_args.comp_grp_id;

	out_acquire_args->rsrc_node = rsrc_node;

	CAM_DBG(CAM_ISP, "Update res successful");
	return rc;
}

static int cam_vfe_bus_ver3_enable_irq_vfe_out(void *bus_priv, void *res_irq_mask)
{
	int   i, rc = 0;
	struct cam_vfe_bus_ver3_vfe_out_data  *rsrc_data = NULL;
	struct cam_vfe_bus_ver3_common_data   *common_data = NULL;
	uint32_t source_group = 0;
	struct cam_isp_resource_node          *vfe_out;
	struct cam_vfe_res_irq_info           *irq_args;
	uint32_t bus_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_MAX];
	uint32_t rup_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_MAX];

	if (!res_irq_mask) {
		CAM_ERR(CAM_ISP, "Invalid input");
		return -EINVAL;
	}

	irq_args = (struct cam_vfe_res_irq_info *)res_irq_mask;
	for (i = 0; i < irq_args->num_res; i++) {
		vfe_out = irq_args->node_res[i];
		rsrc_data = vfe_out->res_priv;
		common_data = rsrc_data->common_data;
		source_group = rsrc_data->source_group;

		CAM_DBG(CAM_ISP, "Start VFE:%d out_type:0x%X",
			rsrc_data->common_data->core_index, rsrc_data->out_type);

		if (rsrc_data->is_dual && !rsrc_data->is_master)
			goto end;

		memset(bus_irq_reg_mask, 0, sizeof(bus_irq_reg_mask));
		rc = cam_vfe_bus_ver3_start_comp_grp(rsrc_data, bus_irq_reg_mask);

		memset(rup_irq_reg_mask, 0, sizeof(rup_irq_reg_mask));
		rup_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_REG0] |=
		0x1 << source_group;

		if (!vfe_out->irq_handle && !vfe_out->is_per_port_start) {
			vfe_out->irq_handle = cam_irq_controller_subscribe_irq(
				common_data->buf_done_controller,
				CAM_IRQ_PRIORITY_1,
				bus_irq_reg_mask,
				vfe_out,
				vfe_out->top_half_handler,
				vfe_out->bottom_half_handler,
				vfe_out->tasklet_info,
				&tasklet_bh_api,
				CAM_IRQ_EVT_GROUP_0);

			if (vfe_out->irq_handle < 1) {
				CAM_ERR(CAM_ISP, "Subscribe IRQ failed for VFE out_res %d, VFE:%u",
					vfe_out->res_id, rsrc_data->common_data->core_index);
					vfe_out->irq_handle = 0;
				return -EFAULT;
			}

			if ((common_data->is_lite || source_group > CAM_VFE_BUS_VER3_SRC_GRP_0)
				&& !vfe_out->is_rdi_primary_res)
				goto end;

			if ((common_data->supported_irq & CAM_VFE_HW_IRQ_CAP_RUP) &&
				(!common_data->rup_irq_handle[source_group])) {
				CAM_DBG(CAM_ISP,
					"VFE:%d out_type:0x%X bus_irq_mask_0:0x%X for RUP",
					rsrc_data->common_data->core_index, rsrc_data->out_type,
				rup_irq_reg_mask[CAM_VFE_BUS_VER3_IRQ_REG0]);

				common_data->rup_irq_handle[source_group] =
					cam_irq_controller_subscribe_irq(
						common_data->bus_irq_controller,
						CAM_IRQ_PRIORITY_0,
						rup_irq_reg_mask,
						vfe_out,
						cam_vfe_bus_ver3_handle_rup_top_half,
						cam_vfe_bus_ver3_handle_rup_bottom_half,
						vfe_out->tasklet_info,
						&tasklet_bh_api,
						CAM_IRQ_EVT_GROUP_1);

				if (common_data->rup_irq_handle[source_group] < 1) {
					CAM_ERR(CAM_ISP, "VFE:%u Failed to subscribe RUP IRQ",
						rsrc_data->common_data->core_index);
						common_data->rup_irq_handle[source_group] = 0;
					return -EFAULT;
				}
			}
		} else if (vfe_out->irq_handle) {
			rc = cam_irq_controller_update_irq(
				common_data->buf_done_controller,
				vfe_out->irq_handle,
				irq_args->enable_irq,
				bus_irq_reg_mask);

			if (rc) {
				CAM_ERR(CAM_ISP, "Update IRQ failed for VFE out_res %d",
					vfe_out->res_id);
				return -EFAULT;
			}

			if ((common_data->is_lite || source_group > CAM_VFE_BUS_VER3_SRC_GRP_0))
				goto end;

			if ((common_data->supported_irq & CAM_VFE_HW_IRQ_CAP_RUP)) {
				rc = cam_irq_controller_update_irq(
						common_data->bus_irq_controller,
						common_data->rup_irq_handle[source_group],
						irq_args->enable_irq,
						rup_irq_reg_mask);
				if (rc) {
					CAM_ERR(CAM_ISP, "Update IRQ failed for VFE out_res %d",
						vfe_out->res_id);
					return -EFAULT;
				}
			}
		} else {
			CAM_ERR(CAM_ISP, "VFE out_res irq handle not found");
			return -EINVAL;
		}
	}
end:
	return rc;
}

static int cam_vfe_bus_ver3_start_hw(void *hw_priv,
	void *start_hw_args, uint32_t arg_size)
{
	return cam_vfe_bus_ver3_start_vfe_out(hw_priv);
}

static int cam_vfe_bus_ver3_stop_hw(void *hw_priv,
	void *stop_hw_args, uint32_t arg_size)
{
	return cam_vfe_bus_ver3_stop_vfe_out(hw_priv);
}

static int cam_vfe_bus_ver3_init_hw(void *hw_priv,
	void *init_hw_args, uint32_t arg_size)
{
	struct cam_vfe_bus_ver3_priv    *bus_priv = hw_priv;

	if (!bus_priv) {
		CAM_ERR(CAM_ISP, "Invalid args");
		return -EINVAL;
	}

	if (bus_priv->common_data.hw_init)
		return 0;

	/* We take the controller only if the buf done is supported on vfe side
	 * for some hw, it is taken from IP extenal to VFE like CSID
	 */
	if ((bus_priv->common_data.supported_irq & CAM_VFE_HW_IRQ_CAP_BUF_DONE))
		bus_priv->common_data.buf_done_controller =
			bus_priv->common_data.bus_irq_controller;

	bus_priv->common_data.hw_init = true;

	CAM_DBG(CAM_ISP, "VFE:%u bus-wr hw-version:0x%x",
		bus_priv->common_data.core_index,
		cam_io_r_mb(bus_priv->common_data.mem_base +
			bus_priv->common_data.common_reg->hw_version));

	return 0;
}

static int cam_vfe_bus_ver3_deinit_hw(void *hw_priv,
	void *deinit_hw_args, uint32_t arg_size)
{
	struct cam_vfe_bus_ver3_priv    *bus_priv = hw_priv;
	int                              rc = 0, i;
	unsigned long                    flags;

	if (!bus_priv) {
		CAM_ERR(CAM_ISP, "Error: Invalid args");
		return -EINVAL;
	}

	if (!bus_priv->common_data.hw_init)
		return 0;

	spin_lock_irqsave(&bus_priv->common_data.spin_lock, flags);
	INIT_LIST_HEAD(&bus_priv->common_data.free_payload_list);
	for (i = 0; i < CAM_VFE_BUS_VER3_PAYLOAD_MAX; i++) {
		INIT_LIST_HEAD(&bus_priv->common_data.evt_payload[i].list);
		list_add_tail(&bus_priv->common_data.evt_payload[i].list,
			&bus_priv->common_data.free_payload_list);
	}
	bus_priv->common_data.hw_init = false;
	spin_unlock_irqrestore(&bus_priv->common_data.spin_lock, flags);

	return rc;
}

static int cam_vfe_bus_get_res_for_mid(
	struct cam_vfe_bus_ver3_priv *bus_priv,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_vfe_bus_ver3_vfe_out_data   *out_data = NULL;
	struct cam_isp_hw_get_cmd_update       *cmd_update = cmd_args;
	struct cam_isp_hw_get_res_for_mid       *get_res = NULL;
	uint32_t num_mid = 0, port_mid[CAM_VFE_BUS_VER3_VFE_OUT_MAX] = {0};
	int i, j;
	bool pid_found = false;

	get_res = (struct cam_isp_hw_get_res_for_mid *)cmd_update->data;
	if (!get_res) {
		CAM_ERR(CAM_ISP,
			"invalid get resource for mid paramas");
		return -EINVAL;
	}

	for (i = 0; i < bus_priv->num_out; i++) {
		out_data = (struct cam_vfe_bus_ver3_vfe_out_data   *)
			bus_priv->vfe_out[i].res_priv;

		if (!out_data)
			continue;

		for (j = 0; j < out_data->num_mid; j++) {
			if (out_data->mid[j] == get_res->mid)
				port_mid[num_mid++] = i;
		}
	}

	for (i = 0; i < num_mid; i++) {
		out_data = (struct cam_vfe_bus_ver3_vfe_out_data   *)
			bus_priv->vfe_out[i].res_priv;
		get_res->out_res_id = bus_priv->vfe_out[port_mid[i]].res_id;
		if (out_data->pid_mask & (1 << get_res->pid)) {
			get_res->out_res_id = bus_priv->vfe_out[port_mid[i]].res_id;
			pid_found = true;
			goto end;
		}
	}

	if (!num_mid) {
		CAM_ERR(CAM_ISP,
			"VFE:%u mid:%d does not match with any out resource",
			bus_priv->common_data.core_index, get_res->mid);
		get_res->out_res_id = 0;
		return -EINVAL;
	}

end:
	CAM_INFO(CAM_ISP, "VFE:%u match mid :%d  out resource:0x%x found, is pid found %d",
		bus_priv->common_data.core_index, get_res->mid, bus_priv->vfe_out[i].res_id,
		 pid_found);
	return 0;
}

static int __cam_vfe_bus_ver3_process_cmd(void *priv,
	uint32_t cmd_type, void *cmd_args, uint32_t arg_size)
{
	return cam_vfe_bus_ver3_process_cmd(priv, cmd_type, cmd_args, arg_size);
}

static uint32_t cam_vfe_bus_ver3_get_last_consumed_addr(
	struct cam_vfe_bus_ver3_priv *bus_priv,
	uint32_t res_type)
{
	uint32_t                                  last_consumed_addr = 0;
	struct cam_isp_resource_node             *rsrc_node = NULL;
	struct cam_vfe_bus_ver3_vfe_out_data     *rsrc_data = NULL;
	struct cam_vfe_bus_ver3_wm_resource_data *wm_rsrc_data = NULL;
	enum cam_vfe_bus_ver3_vfe_out_type        res_id;
	uint32_t                                  outmap_index =
		CAM_VFE_BUS_VER3_VFE_OUT_MAX;

	res_id = cam_vfe_bus_ver3_get_out_res_id_and_index(bus_priv,
		res_type, &outmap_index);
	if ((res_id >= CAM_VFE_BUS_VER3_VFE_OUT_MAX) ||
		(outmap_index >= bus_priv->num_out)) {
		CAM_WARN(CAM_ISP,
			"target does not support req res id :0x%x outtype:%d index:%d",
			res_type, res_id, outmap_index);
		return 0;
	}

	rsrc_node = &bus_priv->vfe_out[outmap_index];
	rsrc_data = rsrc_node->res_priv;
	wm_rsrc_data = rsrc_data->wm_res[PLANE_Y].res_priv;
	last_consumed_addr = cam_io_r_mb(
		wm_rsrc_data->common_data->mem_base +
		wm_rsrc_data->hw_regs->addr_status_0);

	CAM_DBG(CAM_ISP, "VFE:%u res_type:0x%x res_id:0x%x last_consumed_addr:0x%x",
		bus_priv->common_data.core_index, res_type, res_id, last_consumed_addr);

	return last_consumed_addr;
}

static int cam_vfe_bus_ver3_process_cmd(
	struct cam_isp_resource_node *priv,
	uint32_t cmd_type, void *cmd_args, uint32_t arg_size)
{
	int rc = -EINVAL;
	struct cam_vfe_bus_ver3_priv		 *bus_priv;
	uint32_t top_mask_0 = 0;
	struct cam_isp_hw_cap *vfe_bus_cap;
	struct cam_isp_hw_done_event_data *done;

	if (!priv || !cmd_args) {
		CAM_ERR_RATE_LIMIT(CAM_ISP, "Invalid input arguments");
		return -EINVAL;
	}

	switch (cmd_type) {
	case CAM_ISP_HW_CMD_WM_UPDATE:
		rc = cam_vfe_bus_ver3_update_acquire_vfe_out(priv, cmd_args,
				arg_size, true);
		break;
	case CAM_ISP_HW_CMD_GET_BUF_UPDATE:
		rc = cam_vfe_bus_ver3_update_wm(priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_GET_HFR_UPDATE:
		rc = cam_vfe_bus_ver3_update_hfr(priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_GET_WM_SECURE_MODE:
		rc = cam_vfe_bus_ver3_get_secure_mode(priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_STRIPE_UPDATE:
		rc = cam_vfe_bus_ver3_update_stripe_cfg(priv,
			cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_STOP_BUS_ERR_IRQ:
		bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;
		if (bus_priv->error_irq_handle) {
			CAM_DBG(CAM_ISP, "VFE:%u Mask off bus error irq handler",
				priv->hw_intf->hw_idx);
			rc = cam_irq_controller_unsubscribe_irq(
				bus_priv->common_data.bus_irq_controller,
				bus_priv->error_irq_handle);
			bus_priv->error_irq_handle = 0;
		}
		break;
	case CAM_ISP_HW_CMD_DUMP_BUS_INFO: {
		struct cam_isp_hw_event_info  *event_info;

		event_info =
			(struct cam_isp_hw_event_info *)cmd_args;
		bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;

		rc = cam_vfe_bus_ver3_print_dimensions(
			event_info->res_id, bus_priv);
		break;
		}
	case CAM_ISP_HW_IFE_BUS_MINI_DUMP: {
		bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;

		rc = cam_vfe_bus_ver3_mini_dump(bus_priv, cmd_args);
		break;
		}
	case CAM_ISP_HW_USER_DUMP: {
		bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;

		rc = cam_vfe_bus_ver3_user_dump(bus_priv, cmd_args);
		break;
	}
	case CAM_ISP_HW_CMD_UBWC_UPDATE_V2:
		rc = cam_vfe_bus_ver3_update_ubwc_config_v2(cmd_args);
		break;
	case CAM_ISP_HW_CMD_WM_CONFIG_UPDATE:
		rc = cam_vfe_bus_ver3_update_wm_config(cmd_args);
		break;
	case CAM_ISP_HW_CMD_UNMASK_BUS_WR_IRQ:
		bus_priv = (struct cam_vfe_bus_ver3_priv *) priv;
		top_mask_0 = cam_io_r_mb(bus_priv->common_data.mem_base +
			bus_priv->common_data.common_reg->top_irq_mask_0);
		top_mask_0 |= (1 << bus_priv->top_irq_shift);
		cam_io_w_mb(top_mask_0, bus_priv->common_data.mem_base +
			bus_priv->common_data.common_reg->top_irq_mask_0);
		break;
	case CAM_ISP_HW_CMD_GET_RES_FOR_MID:
		bus_priv = (struct cam_vfe_bus_ver3_priv *) priv;
		rc = cam_vfe_bus_get_res_for_mid(bus_priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_QUERY_CAP:
		bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;
		vfe_bus_cap = (struct cam_isp_hw_cap *) cmd_args;
		vfe_bus_cap->max_out_res_type = bus_priv->max_out_res;
		vfe_bus_cap->support_consumed_addr =
			bus_priv->common_data.support_consumed_addr;
		break;
	case CAM_ISP_HW_CMD_GET_LAST_CONSUMED_ADDR:
		bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;
		done = (struct cam_isp_hw_done_event_data *) cmd_args;
		done->last_consumed_addr = cam_vfe_bus_ver3_get_last_consumed_addr(
			bus_priv, done->resource_handle);
		if (done->last_consumed_addr)
			rc = 0;
		break;
	case CAM_ISP_HW_CMD_IFE_DEBUG_CFG: {
		struct cam_vfe_generic_debug_config *debug_cfg;

		bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;
		debug_cfg = (struct cam_vfe_generic_debug_config *)cmd_args;
		bus_priv->common_data.disable_mmu_prefetch =
			debug_cfg->disable_ife_mmu_prefetch;

		CAM_DBG(CAM_ISP, "IFE: %u bus WR prefetch %s",
			bus_priv->common_data.core_index,
			bus_priv->common_data.disable_mmu_prefetch ?
			"disabled" : "enabled");
		rc = 0;
	}
		break;
	case CAM_ISP_HW_CMD_BUF_UPDATE:
		rc = cam_vfe_bus_ver3_config_wm(priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_WM_BW_LIMIT_CONFIG:
		rc = cam_vfe_bus_update_bw_limiter(priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_MC_CTXT_SEL:
		rc = cam_vfe_bus_ver3_mc_ctxt_sel(priv, cmd_args, arg_size);
		break;

	case CAM_ISP_HW_CMD_IRQ_INJECTION:
		rc = cam_vfe_bus_ver3_irq_inject(priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_DUMP_IRQ_DESCRIPTION:
		rc = cam_vfe_bus_ver3_dump_irq_desc(priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_UPDATE_VFE_OUT_RES_DATA:
		rc = cam_vfe_bus_ver3_update_res_vfe_out(priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_UPDATE_VFE_OUT_RES_IRQ_MASK:
		rc = cam_vfe_bus_ver3_enable_irq_vfe_out(priv, cmd_args);
		break;
	case CAM_ISP_HW_CMD_GET_NUM_OUT_RES: {
		uint32_t *max_num_out_res;

		max_num_out_res = (uint32_t *) cmd_args;
		bus_priv = (struct cam_vfe_bus_ver3_priv  *) priv;
		*max_num_out_res = bus_priv->num_out;
		break;
	}

	default:
		CAM_ERR_RATE_LIMIT(CAM_ISP, "VFE:%u Invalid camif process command:%d",
			priv->hw_intf->hw_idx, cmd_type);
		break;
	}

	return rc;
}

int cam_vfe_bus_ver3_init(
	struct cam_hw_soc_info               *soc_info,
	struct cam_hw_intf                   *hw_intf,
	void                                 *bus_hw_info,
	void                                 *vfe_irq_controller,
	struct cam_vfe_bus                  **vfe_bus)
{
	int i, rc = 0;
	struct cam_vfe_bus_ver3_priv    *bus_priv = NULL;
	struct cam_vfe_bus              *vfe_bus_local;
	struct cam_vfe_bus_ver3_hw_info *ver3_hw_info = bus_hw_info;
	struct cam_vfe_soc_private      *soc_private = NULL;

	CAM_DBG(CAM_ISP, "Enter");

	if (!soc_info || !hw_intf || !bus_hw_info || !vfe_irq_controller) {
		CAM_ERR(CAM_ISP,
			"Inval_prms soc_info:%pK hw_intf:%pK hw_info%pK",
			soc_info, hw_intf, bus_hw_info);
		CAM_ERR(CAM_ISP, "controller: %pK", vfe_irq_controller);
		rc = -EINVAL;
		goto end;
	}

	soc_private = soc_info->soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_ISP, "Invalid soc_private");
		rc = -ENODEV;
		goto end;
	}

	vfe_bus_local = kzalloc(sizeof(struct cam_vfe_bus), GFP_KERNEL);
	if (!vfe_bus_local) {
		CAM_DBG(CAM_ISP, "Failed to alloc for vfe_bus");
		rc = -ENOMEM;
		goto end;
	}

	bus_priv = kzalloc(sizeof(struct cam_vfe_bus_ver3_priv),
		GFP_KERNEL);
	if (!bus_priv) {
		CAM_DBG(CAM_ISP, "Failed to alloc for vfe_bus_priv");
		rc = -ENOMEM;
		goto free_bus_local;
	}
	vfe_bus_local->bus_priv = bus_priv;

	bus_priv->num_client                     = ver3_hw_info->num_client;
	bus_priv->num_out                        = ver3_hw_info->num_out;
	bus_priv->num_comp_grp                   = ver3_hw_info->num_comp_grp;
	bus_priv->top_irq_shift                  = ver3_hw_info->top_irq_shift;
	bus_priv->max_out_res                    = ver3_hw_info->max_out_res;
	bus_priv->common_data.num_sec_out        = 0;
	bus_priv->common_data.secure_mode        = CAM_SECURE_MODE_NON_SECURE;
	bus_priv->common_data.core_index         = soc_info->index;
	bus_priv->common_data.mem_base           =
		CAM_SOC_GET_REG_MAP_START(soc_info, VFE_CORE_BASE_IDX);
	bus_priv->common_data.hw_intf            = hw_intf;
	bus_priv->common_data.vfe_irq_controller = vfe_irq_controller;
	bus_priv->common_data.common_reg         = &ver3_hw_info->common_reg;
	bus_priv->common_data.hw_init            = false;

	bus_priv->common_data.is_lite = soc_private->is_ife_lite;
	bus_priv->common_data.support_consumed_addr =
		ver3_hw_info->support_consumed_addr;
	bus_priv->common_data.disable_ubwc_comp = false;
	bus_priv->common_data.supported_irq      = ver3_hw_info->supported_irq;
	bus_priv->common_data.comp_config_needed =
		ver3_hw_info->comp_cfg_needed;
	bus_priv->common_data.init_irq_subscribed = false;
	bus_priv->common_data.disable_mmu_prefetch = false;
	bus_priv->common_data.pack_align_shift =
		ver3_hw_info->pack_align_shift;
	bus_priv->common_data.max_bw_counter_limit =
		ver3_hw_info->max_bw_counter_limit;
	bus_priv->num_cons_err = ver3_hw_info->num_cons_err;
	bus_priv->constraint_error_list = ver3_hw_info->constraint_error_list;
	bus_priv->common_data.soc_info = soc_info;
	bus_priv->bus_hw_info = ver3_hw_info;

	if (bus_priv->num_out >= CAM_VFE_BUS_VER3_VFE_OUT_MAX) {
		CAM_ERR(CAM_ISP, "VFE:%u number of vfe out:%d more than max value:%d ",
			bus_priv->common_data.core_index, bus_priv->num_out,
			CAM_VFE_BUS_VER3_VFE_OUT_MAX);
		rc = -EINVAL;
		goto free_bus_priv;
	}

	bus_priv->comp_grp = kzalloc((sizeof(struct cam_isp_resource_node) *
		bus_priv->num_comp_grp), GFP_KERNEL);
	if (!bus_priv->comp_grp) {
		CAM_ERR(CAM_ISP, "VFE:%u Failed to alloc for bus comp groups",
			bus_priv->common_data.core_index);
		rc = -ENOMEM;
		goto free_bus_priv;
	}

	bus_priv->vfe_out = kzalloc((sizeof(struct cam_isp_resource_node) *
		bus_priv->num_out), GFP_KERNEL);
	if (!bus_priv->vfe_out) {
		CAM_ERR(CAM_ISP, "VFE:%u Failed to alloc for bus out res",
			bus_priv->common_data.core_index);
		rc = -ENOMEM;
		goto free_comp_grp;
	}

	for (i = 0; i < CAM_VFE_BUS_VER3_SRC_GRP_MAX; i++)
		bus_priv->common_data.rup_irq_handle[i] = 0;

	mutex_init(&bus_priv->common_data.bus_mutex);

	rc = cam_irq_controller_init(drv_name, bus_priv->common_data.mem_base,
		&ver3_hw_info->common_reg.irq_reg_info,
		&bus_priv->common_data.bus_irq_controller);
	if (rc) {
		CAM_ERR(CAM_ISP, "VFE:%u Init bus_irq_controller failed",
			bus_priv->common_data.core_index);
		goto free_vfe_out;
	}

	INIT_LIST_HEAD(&bus_priv->free_comp_grp);
	INIT_LIST_HEAD(&bus_priv->used_comp_grp);

	for (i = 0; i < bus_priv->num_comp_grp; i++) {
		rc = cam_vfe_bus_ver3_init_comp_grp(i, soc_info,
			bus_priv, bus_hw_info,
			&bus_priv->comp_grp[i]);
		if (rc < 0) {
			CAM_ERR(CAM_ISP, "VFE:%u init comp_grp:%d failed rc:%d",
				bus_priv->common_data.core_index, i, rc);
			goto deinit_comp_grp;
		}
	}

	for (i = 0; i < CAM_VFE_BUS_VER3_VFE_OUT_MAX; i++)
		bus_priv->vfe_out_map_outtype[i] =
			CAM_VFE_BUS_VER3_VFE_OUT_MAX;

	for (i = 0; i < bus_priv->num_out; i++) {
		rc = cam_vfe_bus_ver3_init_vfe_out_resource(i, bus_priv,
			bus_hw_info);
		if (rc < 0) {
			CAM_ERR(CAM_ISP,
				"VFE:%u init out_type:0x%X failed rc:%d",
				bus_priv->common_data.core_index, i, rc);
			goto deinit_vfe_out;
		}
	}

	spin_lock_init(&bus_priv->common_data.spin_lock);
	INIT_LIST_HEAD(&bus_priv->common_data.free_payload_list);
	for (i = 0; i < CAM_VFE_BUS_VER3_PAYLOAD_MAX; i++) {
		INIT_LIST_HEAD(&bus_priv->common_data.evt_payload[i].list);
		list_add_tail(&bus_priv->common_data.evt_payload[i].list,
			&bus_priv->common_data.free_payload_list);
	}

	vfe_bus_local->hw_ops.reserve      = cam_vfe_bus_ver3_acquire_vfe_out;
	vfe_bus_local->hw_ops.release      = cam_vfe_bus_ver3_release_vfe_out;
	vfe_bus_local->hw_ops.start        = cam_vfe_bus_ver3_start_hw;
	vfe_bus_local->hw_ops.stop         = cam_vfe_bus_ver3_stop_hw;
	vfe_bus_local->hw_ops.init         = cam_vfe_bus_ver3_init_hw;
	vfe_bus_local->hw_ops.deinit       = cam_vfe_bus_ver3_deinit_hw;
	vfe_bus_local->top_half_handler    = NULL;
	vfe_bus_local->bottom_half_handler = NULL;
	vfe_bus_local->hw_ops.process_cmd  = __cam_vfe_bus_ver3_process_cmd;

	*vfe_bus = vfe_bus_local;

	CAM_DBG(CAM_ISP, "Exit, VFE:%u", bus_priv->common_data.core_index);
	return rc;

deinit_vfe_out:
	for (--i; i >= 0; i--)
		cam_vfe_bus_ver3_deinit_vfe_out_resource(&bus_priv->vfe_out[i]);

deinit_comp_grp:
	if (i < 0)
		i = bus_priv->num_comp_grp;
	for (--i; i >= 0; i--)
		cam_vfe_bus_ver3_deinit_comp_grp(&bus_priv->comp_grp[i]);

free_vfe_out:
	kfree(bus_priv->vfe_out);

free_comp_grp:
	kfree(bus_priv->comp_grp);

free_bus_priv:
	kfree(vfe_bus_local->bus_priv);

free_bus_local:
	kfree(vfe_bus_local);

end:
	return rc;
}

int cam_vfe_bus_ver3_deinit(
	struct cam_vfe_bus                  **vfe_bus)
{
	int i, rc = 0;
	struct cam_vfe_bus_ver3_priv    *bus_priv = NULL;
	struct cam_vfe_bus              *vfe_bus_local;
	unsigned long                    flags;

	if (!vfe_bus || !*vfe_bus) {
		CAM_ERR(CAM_ISP, "Invalid input");
		return -EINVAL;
	}
	vfe_bus_local = *vfe_bus;

	bus_priv = vfe_bus_local->bus_priv;
	if (!bus_priv) {
		CAM_ERR(CAM_ISP, "bus_priv is NULL");
		rc = -ENODEV;
		goto free_bus_local;
	}

	spin_lock_irqsave(&bus_priv->common_data.spin_lock, flags);
	INIT_LIST_HEAD(&bus_priv->common_data.free_payload_list);
	for (i = 0; i < CAM_VFE_BUS_VER3_PAYLOAD_MAX; i++)
		INIT_LIST_HEAD(&bus_priv->common_data.evt_payload[i].list);
	bus_priv->common_data.hw_init = false;
	bus_priv->common_data.init_irq_subscribed = false;
	spin_unlock_irqrestore(&bus_priv->common_data.spin_lock, flags);

	for (i = 0; i < bus_priv->num_comp_grp; i++) {
		rc = cam_vfe_bus_ver3_deinit_comp_grp(&bus_priv->comp_grp[i]);
		if (rc < 0)
			CAM_ERR(CAM_ISP,
				"VFE:%u deinit comp_grp:%d failed rc:%d",
				bus_priv->common_data.core_index, i, rc);
	}

	for (i = 0; i < bus_priv->num_out; i++) {
		rc = cam_vfe_bus_ver3_deinit_vfe_out_resource(
			&bus_priv->vfe_out[i]);
		if (rc < 0)
			CAM_ERR(CAM_ISP,
				"VFE:%u deinit out_type:0x%X failed rc:%d",
				bus_priv->common_data.core_index, i, rc);
	}

	INIT_LIST_HEAD(&bus_priv->free_comp_grp);
	INIT_LIST_HEAD(&bus_priv->used_comp_grp);

	rc = cam_irq_controller_deinit(
		&bus_priv->common_data.bus_irq_controller);
	if (rc)
		CAM_ERR(CAM_ISP,
			"VFE:%u Deinit BUS IRQ Controller failed rc=%d",
			bus_priv->common_data.core_index, rc);

	kfree(bus_priv->comp_grp);
	kfree(bus_priv->vfe_out);

	mutex_destroy(&bus_priv->common_data.bus_mutex);
	kfree(vfe_bus_local->bus_priv);

free_bus_local:
	kfree(vfe_bus_local);

	*vfe_bus = NULL;

	return rc;
}
