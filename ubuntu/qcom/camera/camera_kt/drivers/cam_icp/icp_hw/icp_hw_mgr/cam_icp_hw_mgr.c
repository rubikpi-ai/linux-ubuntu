// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <media/cam_defs.h>
#include <media/cam_icp.h>
#include <media/cam_cpas.h>

#include "cam_sync_api.h"
#include "cam_packet_util.h"
#include "cam_hw.h"
#include "cam_hw_mgr_intf.h"
#include "cam_icp_hw_mgr_intf.h"
#include "cam_icp_hw_mgr.h"
#include "cam_a5_hw_intf.h"
#include "cam_bps_hw_intf.h"
#include "cam_ipe_hw_intf.h"
#include "cam_smmu_api.h"
#include "cam_mem_mgr.h"
#include "hfi_intf.h"
#include "hfi_reg.h"
#include "hfi_session_defs.h"
#include "hfi_sys_defs.h"
#include "cam_req_mgr_workq.h"
#include "cam_mem_mgr.h"
#include "a5_core.h"
#include "hfi_sys_defs.h"
#include "cam_debug_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"
#include "cam_cpas_api.h"
#include "cam_common_util.h"

#define ICP_WORKQ_TASK_CMD_TYPE 1
#define ICP_WORKQ_TASK_MSG_TYPE 2

#define ICP_DEV_TYPE_TO_CLK_TYPE(dev_type) \
	((dev_type == CAM_ICP_RES_TYPE_BPS) ? ICP_CLK_HW_BPS : ICP_CLK_HW_IPE)

#define ICP_DEVICE_IDLE_TIMEOUT 400

static const struct hfi_ops hfi_a5_ops = {
	.irq_raise = cam_a5_irq_raise,
	.irq_enable = cam_a5_irq_enable,
	.iface_addr = cam_a5_iface_addr,
};

static struct cam_icp_hw_mgr icp_hw_mgr;

static void cam_icp_mgr_process_dbg_buf(unsigned int debug_lvl);

static int cam_icp_dump_io_cfg(struct cam_icp_hw_ctx_data *ctx_data,
	int32_t buf_handle, uint32_t size)
{
	uintptr_t vaddr_ptr;
	uint32_t  *ptr;
	uint32_t  io_size;
	size_t    len;
	int       rc, i;
	char      buf[512];
	int       used = 0;

	rc = cam_mem_get_cpu_buf(buf_handle, &vaddr_ptr, &len);
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to get io_cfg buf address for %d",
			ctx_data->ctx_id);
		return rc;
	}

	io_size = size / sizeof(uint32_t);
	ptr = (uint32_t *)vaddr_ptr;
	for (i = 0; i < io_size; i++) {
		used += snprintf(buf + used,
			sizeof(buf) - used, "0X%08X-", ptr[i]);
		if (!(i % 8)) {
			CAM_DBG(CAM_ICP, "%s: %s", __func__, buf);
			used = 0;
		}
	}
	cam_mem_put_cpu_buf(buf_handle);
	return rc;
}

static const char *cam_icp_dev_type_to_name(
	uint32_t dev_type)
{
	switch (dev_type) {
	case CAM_ICP_RES_TYPE_BPS:
		return "BPS";
	case CAM_ICP_RES_TYPE_IPE_RT:
		return "IPE_RT";
	case CAM_ICP_RES_TYPE_IPE:
		return "IPE";
	default:
		return "Invalid dev type";
	}
}

static int cam_icp_send_ubwc_cfg(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	int rc;
	uint32_t disable_ubwc_comp = 0;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is NULL");
		return -EINVAL;
	}

	disable_ubwc_comp = hw_mgr->disable_ubwc_comp;

	rc = icp_dev_intf->hw_ops.process_cmd(
		icp_dev_intf->hw_priv,
		CAM_ICP_CMD_UBWC_CFG, (void *)&disable_ubwc_comp,
		sizeof(disable_ubwc_comp));
	if (rc)
		CAM_ERR(CAM_ICP, "CAM_ICP_CMD_UBWC_CFG is failed");

	return rc;
}

static void cam_icp_hw_mgr_clk_info_update(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	struct cam_icp_clk_info *hw_mgr_clk_info;

	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS)
		hw_mgr_clk_info = &hw_mgr->clk_info[ICP_CLK_HW_BPS];
	else
		hw_mgr_clk_info = &hw_mgr->clk_info[ICP_CLK_HW_IPE];

	if (hw_mgr_clk_info->base_clk >= ctx_data->clk_info.base_clk)
		hw_mgr_clk_info->base_clk -= ctx_data->clk_info.base_clk;
}

static void cam_icp_hw_mgr_reset_clk_info(struct cam_icp_hw_mgr *hw_mgr)
{
	int i;

	for (i = 0; i < ICP_CLK_HW_MAX; i++) {
		hw_mgr->clk_info[i].base_clk = 0;
		hw_mgr->clk_info[i].curr_clk = ICP_CLK_SVS_HZ;
		hw_mgr->clk_info[i].threshold = ICP_OVER_CLK_THRESHOLD;
		hw_mgr->clk_info[i].over_clked = 0;
		hw_mgr->clk_info[i].uncompressed_bw = CAM_CPAS_DEFAULT_AXI_BW;
		hw_mgr->clk_info[i].compressed_bw = CAM_CPAS_DEFAULT_AXI_BW;
	}
	hw_mgr->icp_default_clk = ICP_CLK_SVS_HZ;
}

static int cam_icp_get_actual_clk_rate_idx(
	struct cam_icp_hw_ctx_data *ctx_data, uint32_t base_clk)
{
	int i;

	for (i = 0; i < CAM_MAX_VOTE; i++)
		if (ctx_data->clk_info.clk_rate[i] >= base_clk)
			return i;

	/*
	 * Caller has to ensure returned index is within array
	 * size bounds while accessing that index.
	 */

	return i;
}

static bool cam_icp_is_over_clk(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *hw_mgr_clk_info)
{
	int base_clk_idx;
	int curr_clk_idx;

	base_clk_idx = cam_icp_get_actual_clk_rate_idx(ctx_data,
		hw_mgr_clk_info->base_clk);

	curr_clk_idx = cam_icp_get_actual_clk_rate_idx(ctx_data,
		hw_mgr_clk_info->curr_clk);

	CAM_DBG(CAM_PERF, "bc_idx = %d cc_idx = %d %d %d",
		base_clk_idx, curr_clk_idx, hw_mgr_clk_info->base_clk,
		hw_mgr_clk_info->curr_clk);

	if (curr_clk_idx > base_clk_idx)
		return true;

	return false;
}

static int cam_icp_get_lower_clk_rate(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, uint32_t base_clk)
{
	int i;

	i = cam_icp_get_actual_clk_rate_idx(ctx_data, base_clk);

	if (i > 0)
		return ctx_data->clk_info.clk_rate[i - 1];

	CAM_DBG(CAM_PERF, "Already clk at lower level");
	return base_clk;
}

static int cam_icp_get_next_clk_rate(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, uint32_t base_clk)
{
	int i;

	i = cam_icp_get_actual_clk_rate_idx(ctx_data, base_clk);

	if (i < CAM_MAX_VOTE - 1)
		return ctx_data->clk_info.clk_rate[i + 1];

	CAM_DBG(CAM_PERF, "Already clk at higher level");

	return base_clk;
}

static int cam_icp_get_actual_clk_rate(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, uint32_t base_clk)
{
	int i;

	for (i = 0; i < CAM_MAX_VOTE; i++)
		if (ctx_data->clk_info.clk_rate[i] >= base_clk)
			return ctx_data->clk_info.clk_rate[i];

	return base_clk;
}

static int cam_icp_supported_clk_rates(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int i;
	struct cam_hw_soc_info *soc_info;
	struct cam_hw_intf *dev_intf = NULL;
	struct cam_hw_info *dev = NULL;

	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS)
		dev_intf = hw_mgr->bps_dev_intf;
	else
		dev_intf = hw_mgr->ipe0_dev_intf;

	if (!dev_intf) {
		CAM_ERR(CAM_ICP, "dev_intf is invalid");
		return -EINVAL;
	}
	dev = (struct cam_hw_info *)dev_intf->hw_priv;
	soc_info = &dev->soc_info;

	for (i = 0; i < CAM_MAX_VOTE; i++) {
		ctx_data->clk_info.clk_rate[i] =
			soc_info->clk_rate[i][soc_info->src_clk_idx];
		CAM_DBG(CAM_PERF, "clk_info[%d] = %d",
			i, ctx_data->clk_info.clk_rate[i]);
	}

	return 0;
}

static int cam_icp_clk_idx_from_req_id(struct cam_icp_hw_ctx_data *ctx_data,
	uint64_t req_id)
{
	struct hfi_frame_process_info *frame_process;
	int i;

	frame_process = &ctx_data->hfi_frame_process;

	for (i = 0; i < CAM_FRAME_CMD_MAX; i++)
		if (frame_process->request_id[i] == req_id)
			return i;

	return 0;
}

static int cam_icp_ctx_clk_info_init(struct cam_icp_hw_ctx_data *ctx_data)
{
	int i;

	ctx_data->clk_info.curr_fc = 0;
	ctx_data->clk_info.base_clk = 0;
	ctx_data->clk_info.uncompressed_bw = 0;
	ctx_data->clk_info.compressed_bw = 0;
	for (i = 0; i < CAM_ICP_MAX_PER_PATH_VOTES; i++) {
		ctx_data->clk_info.axi_path[i].camnoc_bw = 0;
		ctx_data->clk_info.axi_path[i].mnoc_ab_bw = 0;
		ctx_data->clk_info.axi_path[i].mnoc_ib_bw = 0;
	}

	cam_icp_supported_clk_rates(&icp_hw_mgr, ctx_data);

	return 0;
}

static bool cam_icp_frame_pending(struct cam_icp_hw_ctx_data *ctx_data)
{
	return !bitmap_empty(ctx_data->hfi_frame_process.bitmap,
			CAM_FRAME_CMD_MAX);
}

static int cam_icp_ctx_timer_reset(struct cam_icp_hw_ctx_data *ctx_data)
{
	if (ctx_data && ctx_data->watch_dog) {
		ctx_data->watch_dog_reset_counter++;
		CAM_DBG(CAM_PERF, "reset timer : ctx_id = %d, counter=%d",
			ctx_data->ctx_id, ctx_data->watch_dog_reset_counter);
		crm_timer_reset(ctx_data->watch_dog);
	}

	return 0;
}

static void cam_icp_device_timer_reset(struct cam_icp_hw_mgr *hw_mgr,
	int device_index)
{
	if ((device_index >= ICP_CLK_HW_MAX) || (!hw_mgr))
		return;

	if (hw_mgr->clk_info[device_index].watch_dog) {
		CAM_DBG(CAM_PERF, "reset timer : device_index = %d",
			device_index);
		crm_timer_reset(hw_mgr->clk_info[device_index].watch_dog);
		hw_mgr->clk_info[device_index].watch_dog_reset_counter++;
	}
}

static int32_t cam_icp_deinit_idle_clk(void *priv, void *data)
{
	struct cam_icp_hw_mgr *hw_mgr = (struct cam_icp_hw_mgr *)priv;
	struct clk_work_data *task_data = (struct clk_work_data *)data;
	struct cam_icp_clk_info *clk_info =
		(struct cam_icp_clk_info *)task_data->data;
	uint32_t id;
	uint32_t i;
	struct cam_icp_hw_ctx_data *ctx_data;
	struct cam_hw_intf *ipe0_dev_intf = NULL;
	struct cam_hw_intf *ipe1_dev_intf = NULL;
	struct cam_hw_intf *bps_dev_intf = NULL;
	struct cam_hw_intf *dev_intf = NULL;
	struct cam_icp_clk_update_cmd clk_upd_cmd;
	int rc = 0;
	bool busy = false;

	ipe0_dev_intf = hw_mgr->ipe0_dev_intf;
	ipe1_dev_intf = hw_mgr->ipe1_dev_intf;
	bps_dev_intf = hw_mgr->bps_dev_intf;

	clk_info->base_clk = 0;
	clk_info->curr_clk = 0;
	clk_info->over_clked = 0;

	mutex_lock(&hw_mgr->hw_mgr_mutex);

	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		ctx_data = &hw_mgr->ctx_data[i];
		mutex_lock(&ctx_data->ctx_mutex);
		if ((ctx_data->state == CAM_ICP_CTX_STATE_ACQUIRED) &&
			(ICP_DEV_TYPE_TO_CLK_TYPE(
			ctx_data->icp_dev_acquire_info->dev_type)
			== clk_info->hw_type)) {
			busy = cam_icp_frame_pending(ctx_data);
			if (busy) {
				mutex_unlock(&ctx_data->ctx_mutex);
				break;
			}
			cam_icp_ctx_clk_info_init(ctx_data);
		}
		mutex_unlock(&ctx_data->ctx_mutex);
	}

	if (busy) {
		cam_icp_device_timer_reset(hw_mgr, clk_info->hw_type);
		rc = -EBUSY;
		goto done;
	}

	if ((!ipe0_dev_intf) || (!bps_dev_intf)) {
		CAM_ERR(CAM_ICP, "dev intfs are wrong, failed to update clk");
		rc = -EINVAL;
		goto done;
	}

	if (clk_info->hw_type == ICP_CLK_HW_BPS) {
		dev_intf = bps_dev_intf;
		id = CAM_ICP_BPS_CMD_DISABLE_CLK;
	} else if (clk_info->hw_type == ICP_CLK_HW_IPE) {
		dev_intf = ipe0_dev_intf;
		id = CAM_ICP_IPE_CMD_DISABLE_CLK;
	} else {
		CAM_ERR(CAM_ICP, "Error");
		goto done;
	}

	CAM_DBG(CAM_PERF, "Disable %d", clk_info->hw_type);

	clk_upd_cmd.ipe_bps_pc_enable = icp_hw_mgr.ipe_bps_pc_flag;

	dev_intf->hw_ops.process_cmd(dev_intf->hw_priv, id,
		&clk_upd_cmd, sizeof(clk_upd_cmd));

	if (clk_info->hw_type != ICP_CLK_HW_BPS)
		if (ipe1_dev_intf)
			ipe1_dev_intf->hw_ops.process_cmd(
				ipe1_dev_intf->hw_priv, id,
				&clk_upd_cmd, sizeof(clk_upd_cmd));

done:
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	return rc;
}

static int cam_icp_remove_ctx_bw(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0;
	struct cam_hw_intf *dev_intf = NULL;
	uint32_t id;
	uint64_t temp;
	struct cam_icp_clk_info *clk_info;
	struct cam_icp_cpas_vote clk_update;
	int i = 0;
	int device_share_ratio = 1;
	uint64_t total_ab_bw = 0;

	if (!ctx_data->icp_dev_acquire_info) {
		CAM_WARN(CAM_ICP, "NULL acquire info");
		return -EINVAL;
	}

	if ((!hw_mgr->ipe0_dev_intf) || (!hw_mgr->bps_dev_intf)) {
		CAM_ERR(CAM_ICP, "dev intfs are wrong, failed to update clk");
		return -EINVAL;
	}

	CAM_DBG(CAM_PERF,
		"ctx_id = %d ubw = %lld cbw = %lld curr_fc = %u bc = %u",
		ctx_data->ctx_id,
		ctx_data->clk_info.uncompressed_bw,
		ctx_data->clk_info.compressed_bw,
		ctx_data->clk_info.curr_fc, ctx_data->clk_info.base_clk);

	if (!ctx_data->clk_info.bw_included) {
		CAM_DBG(CAM_PERF, "ctx_id = %d BW vote already removed",
			ctx_data->ctx_id);
		return 0;
	}

	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS) {
		dev_intf = hw_mgr->bps_dev_intf;
		clk_info = &hw_mgr->clk_info[ICP_CLK_HW_BPS];
		id = CAM_ICP_BPS_CMD_VOTE_CPAS;
	} else {
		dev_intf = hw_mgr->ipe0_dev_intf;
		clk_info = &hw_mgr->clk_info[ICP_CLK_HW_IPE];
		id = CAM_ICP_IPE_CMD_VOTE_CPAS;
	}

	/*
	 * Since there are 2 devices, we assume the load is evenly shared
	 * between HWs and corresponding AXI paths. So divide total bw by half
	 * to vote on each device
	 */
	if ((ctx_data->icp_dev_acquire_info->dev_type !=
		CAM_ICP_RES_TYPE_BPS) && (hw_mgr->ipe1_dev_intf))
		device_share_ratio = 2;

	if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_V1) {
		clk_update.axi_vote.num_paths = 1;
		if (ctx_data->icp_dev_acquire_info->dev_type ==
			CAM_ICP_RES_TYPE_BPS) {
			clk_update.axi_vote.axi_path[0].path_data_type =
				CAM_BPS_DEFAULT_AXI_PATH;
			clk_update.axi_vote.axi_path[0].transac_type =
				CAM_BPS_DEFAULT_AXI_TRANSAC;
		} else {
			clk_update.axi_vote.axi_path[0].path_data_type =
				CAM_IPE_DEFAULT_AXI_PATH;
			clk_update.axi_vote.axi_path[0].transac_type =
				CAM_IPE_DEFAULT_AXI_TRANSAC;
		}

		clk_info->compressed_bw -= ctx_data->clk_info.compressed_bw;
		clk_info->uncompressed_bw -= ctx_data->clk_info.uncompressed_bw;

		total_ab_bw = clk_info->compressed_bw;

		ctx_data->clk_info.uncompressed_bw = 0;
		ctx_data->clk_info.compressed_bw = 0;
		ctx_data->clk_info.curr_fc = 0;
		ctx_data->clk_info.base_clk = 0;

		clk_update.axi_vote.num_paths = 1;

		temp = clk_info->uncompressed_bw;
		do_div(temp, device_share_ratio);
		clk_update.axi_vote.axi_path[0].camnoc_bw = temp;

		temp = clk_info->compressed_bw;
		do_div(temp, device_share_ratio);
		clk_update.axi_vote.axi_path[0].mnoc_ab_bw = temp;
		clk_update.axi_vote.axi_path[0].mnoc_ib_bw = temp;
		clk_update.axi_vote.axi_path[0].ddr_ab_bw = temp;
		clk_update.axi_vote.axi_path[0].ddr_ib_bw = temp;
	} else {
		int path_index;

		/*
		 * Remove previous vote of this context from hw mgr first.
		 * hw_mgr_clk_info has all valid paths, with each path in its
		 * own index. BW that we wanted to vote now is after removing
		 * current context's vote from hw mgr consolidated vote
		 */
		for (i = 0; i < ctx_data->clk_info.num_paths; i++) {
			if (ctx_data->icp_dev_acquire_info->dev_type ==
				CAM_ICP_RES_TYPE_BPS) {
				/*
				 * By assuming BPS has Read-All, Write-All
				 * votes only.
				 */
				path_index =
				ctx_data->clk_info.axi_path[i].transac_type -
				CAM_AXI_TRANSACTION_READ;
			} else {
				path_index =
				ctx_data->clk_info.axi_path[i].path_data_type -
				CAM_AXI_PATH_DATA_IPE_START_OFFSET;
			}

			if (path_index >= CAM_ICP_MAX_PER_PATH_VOTES) {
				CAM_WARN(CAM_PERF,
				"Invalid path %d, start offset=%d, max=%d",
				ctx_data->clk_info.axi_path[i].path_data_type,
				CAM_AXI_PATH_DATA_IPE_START_OFFSET,
				CAM_ICP_MAX_PER_PATH_VOTES);
				continue;
			}

			clk_info->axi_path[path_index].camnoc_bw -=
				ctx_data->clk_info.axi_path[i].camnoc_bw;
			clk_info->axi_path[path_index].mnoc_ab_bw -=
				ctx_data->clk_info.axi_path[i].mnoc_ab_bw;
			clk_info->axi_path[path_index].mnoc_ib_bw -=
				ctx_data->clk_info.axi_path[i].mnoc_ib_bw;
			clk_info->axi_path[path_index].ddr_ab_bw -=
				ctx_data->clk_info.axi_path[i].ddr_ab_bw;
			clk_info->axi_path[path_index].ddr_ib_bw -=
				ctx_data->clk_info.axi_path[i].ddr_ib_bw;

			total_ab_bw +=
				clk_info->axi_path[path_index].mnoc_ab_bw;

			CAM_DBG(CAM_PERF,
				"Removing ctx bw from path_type: %s, transac_type: %s, camnoc_bw = %lld mnoc_ab_bw = %lld, mnoc_ib_bw = %lld, device: %s",
				cam_cpas_axi_util_path_type_to_string(
				ctx_data->clk_info.axi_path[i].path_data_type),
				cam_cpas_axi_util_trans_type_to_string(
				ctx_data->clk_info.axi_path[i].transac_type),
				ctx_data->clk_info.axi_path[i].camnoc_bw,
				ctx_data->clk_info.axi_path[i].mnoc_ab_bw,
				ctx_data->clk_info.axi_path[i].mnoc_ib_bw,
				cam_icp_dev_type_to_name(
				ctx_data->icp_dev_acquire_info->dev_type));

			CAM_DBG(CAM_PERF,
				"Final HW bw for path_type: %s, transac_type: %s, camnoc_bw = %lld mnoc_ab_bw = %lld, mnoc_ib_bw = %lld, device: %s",
				cam_cpas_axi_util_path_type_to_string(
				clk_info->axi_path[i].path_data_type),
				cam_cpas_axi_util_trans_type_to_string(
				clk_info->axi_path[i].transac_type),
				clk_info->axi_path[i].camnoc_bw,
				clk_info->axi_path[i].mnoc_ab_bw,
				clk_info->axi_path[i].mnoc_ib_bw,
				cam_icp_dev_type_to_name(
				ctx_data->icp_dev_acquire_info->dev_type));
		}

		memset(&ctx_data->clk_info.axi_path[0], 0,
			CAM_ICP_MAX_PER_PATH_VOTES *
			sizeof(struct cam_axi_per_path_bw_vote));
		ctx_data->clk_info.curr_fc = 0;
		ctx_data->clk_info.base_clk = 0;

		clk_update.axi_vote.num_paths = clk_info->num_paths;
		memcpy(&clk_update.axi_vote.axi_path[0],
			&clk_info->axi_path[0],
			clk_update.axi_vote.num_paths *
			sizeof(struct cam_axi_per_path_bw_vote));

		if (device_share_ratio > 1) {
			for (i = 0; i < clk_update.axi_vote.num_paths; i++) {
				do_div(
				clk_update.axi_vote.axi_path[i].camnoc_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].mnoc_ab_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].mnoc_ib_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].ddr_ab_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].ddr_ib_bw,
					device_share_ratio);
			}
		}
	}

	clk_update.axi_vote_valid = true;

	if (total_ab_bw == 0) {
		/* If no more contexts are active, reduce AHB vote to minimum */
		clk_update.ahb_vote.type = CAM_VOTE_ABSOLUTE;
		clk_update.ahb_vote.vote.level = CAM_LOWSVS_VOTE;
		clk_update.ahb_vote_valid = true;
	} else {
		clk_update.ahb_vote_valid = false;
	}

	rc = dev_intf->hw_ops.process_cmd(dev_intf->hw_priv, id,
		&clk_update, sizeof(clk_update));
	if (rc)
		CAM_ERR(CAM_PERF, "Failed in updating cpas vote, rc=%d", rc);

	/*
	 * Vote half bandwidth each on both devices.
	 * Total bw at mnoc - CPAS will take care of adding up.
	 * camnoc clk calculate is more accurate this way.
	 */
	if ((!rc) && (hw_mgr->ipe1_dev_intf) &&
		(ctx_data->icp_dev_acquire_info->dev_type !=
		CAM_ICP_RES_TYPE_BPS)) {
		dev_intf = hw_mgr->ipe1_dev_intf;
		rc = dev_intf->hw_ops.process_cmd(dev_intf->hw_priv,
			id, &clk_update, sizeof(clk_update));
		if (rc)
			CAM_ERR(CAM_PERF,
				"Failed in updating cpas vote for ipe 2, rc=%d",
				rc);
	}

	ctx_data->clk_info.bw_included = false;

	CAM_DBG(CAM_PERF, "X :ctx_id = %d curr_fc = %u bc = %u",
		ctx_data->ctx_id, ctx_data->clk_info.curr_fc,
		ctx_data->clk_info.base_clk);

	return rc;

}


static int32_t cam_icp_ctx_timer(void *priv, void *data)
{
	struct clk_work_data *task_data = (struct clk_work_data *)data;
	struct cam_icp_hw_ctx_data *ctx_data =
		(struct cam_icp_hw_ctx_data *)task_data->data;

	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "ctx_data is NULL, failed to update clk");
		return -EINVAL;
	}

	mutex_lock(&ctx_data->ctx_mutex);

	CAM_DBG(CAM_PERF,
		"ctx_id = %d ubw = %lld cbw = %lld curr_fc = %u bc = %u",
		ctx_data->ctx_id,
		ctx_data->clk_info.uncompressed_bw,
		ctx_data->clk_info.compressed_bw,
		ctx_data->clk_info.curr_fc,
		ctx_data->clk_info.base_clk);

	if ((ctx_data->state != CAM_ICP_CTX_STATE_ACQUIRED) ||
		(ctx_data->watch_dog_reset_counter == 0)) {
		CAM_DBG(CAM_PERF, "state %d, counter=%d",
			ctx_data->state, ctx_data->watch_dog_reset_counter);
		mutex_unlock(&ctx_data->ctx_mutex);
		return 0;
	}

	if (cam_icp_frame_pending(ctx_data)) {
		cam_icp_ctx_timer_reset(ctx_data);
		mutex_unlock(&ctx_data->ctx_mutex);
		return -EBUSY;
	}

	cam_icp_remove_ctx_bw(&icp_hw_mgr, ctx_data);

	mutex_unlock(&ctx_data->ctx_mutex);

	return 0;
}

static void cam_icp_ctx_timer_cb(struct timer_list *timer_data)
{
	unsigned long flags;
	struct crm_workq_task *task;
	struct clk_work_data *task_data;
	struct cam_req_mgr_timer *timer =
		container_of(timer_data, struct cam_req_mgr_timer, sys_timer);

	spin_lock_irqsave(&icp_hw_mgr.hw_mgr_lock, flags);
	task = cam_req_mgr_workq_get_task(icp_hw_mgr.timer_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "no empty task");
		spin_unlock_irqrestore(&icp_hw_mgr.hw_mgr_lock, flags);
		return;
	}

	task_data = (struct clk_work_data *)task->payload;
	task_data->data = timer->parent;
	task_data->type = ICP_WORKQ_TASK_MSG_TYPE;
	task->process_cb = cam_icp_ctx_timer;
	cam_req_mgr_workq_enqueue_task(task, &icp_hw_mgr,
		CRM_TASK_PRIORITY_0);
	spin_unlock_irqrestore(&icp_hw_mgr.hw_mgr_lock, flags);
}

static void cam_icp_device_timer_cb(struct timer_list *timer_data)
{
	unsigned long flags;
	struct crm_workq_task *task;
	struct clk_work_data *task_data;
	struct cam_req_mgr_timer *timer =
		container_of(timer_data, struct cam_req_mgr_timer, sys_timer);

	spin_lock_irqsave(&icp_hw_mgr.hw_mgr_lock, flags);
	task = cam_req_mgr_workq_get_task(icp_hw_mgr.timer_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "no empty task");
		spin_unlock_irqrestore(&icp_hw_mgr.hw_mgr_lock, flags);
		return;
	}

	task_data = (struct clk_work_data *)task->payload;
	task_data->data = timer->parent;
	task_data->type = ICP_WORKQ_TASK_MSG_TYPE;
	task->process_cb = cam_icp_deinit_idle_clk;
	cam_req_mgr_workq_enqueue_task(task, &icp_hw_mgr,
		CRM_TASK_PRIORITY_0);
	spin_unlock_irqrestore(&icp_hw_mgr.hw_mgr_lock, flags);
}

static int cam_icp_clk_info_init(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int i, j;

	for (i = 0; i < ICP_CLK_HW_MAX; i++) {
		hw_mgr->clk_info[i].base_clk = ICP_CLK_SVS_HZ;
		hw_mgr->clk_info[i].curr_clk = ICP_CLK_SVS_HZ;
		hw_mgr->clk_info[i].threshold = ICP_OVER_CLK_THRESHOLD;
		hw_mgr->clk_info[i].over_clked = 0;
		hw_mgr->clk_info[i].uncompressed_bw = CAM_CPAS_DEFAULT_AXI_BW;
		hw_mgr->clk_info[i].compressed_bw = CAM_CPAS_DEFAULT_AXI_BW;
		for (j = 0; j < CAM_ICP_MAX_PER_PATH_VOTES; j++) {
			hw_mgr->clk_info[i].axi_path[j].path_data_type = 0;
			hw_mgr->clk_info[i].axi_path[j].transac_type = 0;
			hw_mgr->clk_info[i].axi_path[j].camnoc_bw = 0;
			hw_mgr->clk_info[i].axi_path[j].mnoc_ab_bw = 0;
			hw_mgr->clk_info[i].axi_path[j].mnoc_ib_bw = 0;
		}

		hw_mgr->clk_info[i].hw_type = i;
		hw_mgr->clk_info[i].watch_dog_reset_counter = 0;
	}

	hw_mgr->icp_default_clk = ICP_CLK_SVS_HZ;

	return 0;
}

static int cam_icp_ctx_timer_start(struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0;

	rc = crm_timer_init(&ctx_data->watch_dog,
		200, ctx_data, &cam_icp_ctx_timer_cb);
	if (rc)
		CAM_ERR(CAM_ICP, "Failed to start timer");

	ctx_data->watch_dog_reset_counter = 0;

	CAM_DBG(CAM_PERF, "start timer : ctx_id = %d", ctx_data->ctx_id);
	return rc;
}

static int cam_icp_device_timer_start(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0;
	int i;

	for (i = 0; i < ICP_CLK_HW_MAX; i++)  {
		if (!hw_mgr->clk_info[i].watch_dog) {
			rc = crm_timer_init(&hw_mgr->clk_info[i].watch_dog,
				ICP_DEVICE_IDLE_TIMEOUT, &hw_mgr->clk_info[i],
				&cam_icp_device_timer_cb);

			if (rc)
				CAM_ERR(CAM_ICP, "Failed to start timer %d", i);

			hw_mgr->clk_info[i].watch_dog_reset_counter = 0;
		}
	}

	return rc;
}

static int cam_icp_ctx_timer_stop(struct cam_icp_hw_ctx_data *ctx_data)
{
	if (ctx_data->watch_dog) {
		CAM_DBG(CAM_PERF, "stop timer : ctx_id = %d", ctx_data->ctx_id);
		ctx_data->watch_dog_reset_counter = 0;
		crm_timer_exit(&ctx_data->watch_dog);
		ctx_data->watch_dog = NULL;
	}

	return 0;
}

static void cam_icp_device_timer_stop(struct cam_icp_hw_mgr *hw_mgr)
{
	if (!hw_mgr->bps_ctxt_cnt &&
		hw_mgr->clk_info[ICP_CLK_HW_BPS].watch_dog) {
		hw_mgr->clk_info[ICP_CLK_HW_BPS].watch_dog_reset_counter = 0;
		crm_timer_exit(&hw_mgr->clk_info[ICP_CLK_HW_BPS].watch_dog);
		hw_mgr->clk_info[ICP_CLK_HW_BPS].watch_dog = NULL;
	}

	if (!hw_mgr->ipe_ctxt_cnt &&
		hw_mgr->clk_info[ICP_CLK_HW_IPE].watch_dog) {
		hw_mgr->clk_info[ICP_CLK_HW_IPE].watch_dog_reset_counter = 0;
		crm_timer_exit(&hw_mgr->clk_info[ICP_CLK_HW_IPE].watch_dog);
		hw_mgr->clk_info[ICP_CLK_HW_IPE].watch_dog = NULL;
	}
}

static uint32_t cam_icp_mgr_calc_base_clk(uint32_t frame_cycles,
	uint64_t budget)
{
	uint64_t base_clk;
	uint64_t mul = 1000000000;

	base_clk = frame_cycles * mul;
	do_div(base_clk, budget);

	CAM_DBG(CAM_PERF, "budget = %lld fc = %d ib = %lld base_clk = %lld",
		budget, frame_cycles,
		(long long)(frame_cycles * mul), base_clk);

	return base_clk;
}

static bool cam_icp_busy_prev_reqs(struct hfi_frame_process_info *frm_process,
	uint64_t req_id)
{
	int i;
	int cnt;

	for (i = 0, cnt = 0; i < CAM_FRAME_CMD_MAX; i++) {
		if (frm_process->request_id[i]) {
			if (frm_process->fw_process_flag[i]) {
				CAM_DBG(CAM_PERF, "r id = %lld busy = %d",
					frm_process->request_id[i],
					frm_process->fw_process_flag[i]);
				cnt++;
			}
		}
	}
	if (cnt > 1)
		return true;

	return false;
}

static int cam_icp_calc_total_clk(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_clk_info *hw_mgr_clk_info, uint32_t dev_type)
{
	int i;
	struct cam_icp_hw_ctx_data *ctx_data;

	hw_mgr_clk_info->base_clk = 0;
	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		ctx_data = &hw_mgr->ctx_data[i];
		if (ctx_data->state == CAM_ICP_CTX_STATE_ACQUIRED &&
			ICP_DEV_TYPE_TO_CLK_TYPE(
			ctx_data->icp_dev_acquire_info->dev_type) ==
			ICP_DEV_TYPE_TO_CLK_TYPE(dev_type))
			hw_mgr_clk_info->base_clk +=
				ctx_data->clk_info.base_clk;
	}

	return 0;
}

static bool cam_icp_update_clk_busy(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *hw_mgr_clk_info,
	struct cam_icp_clk_bw_request *clk_info,
	uint32_t base_clk)
{
	uint32_t next_clk_level;
	uint32_t actual_clk;
	bool rc = false;

	/* 1. if current request frame cycles(fc) are more than previous
	 *      frame fc
	 *      Calculate the new base clock.
	 *      if sum of base clocks are more than next available clk level
	 *       Update clock rate, change curr_clk_rate to sum of base clock
	 *       rates and make over_clked to zero
	 *      else
	 *       Update clock rate to next level, update curr_clk_rate and make
	 *       overclked cnt to zero
	 * 2. if current fc is less than or equal to previous  frame fc
	 *      Still Bump up the clock to next available level
	 *      if it is available, then update clock, make overclk cnt to
	 *      zero. If the clock is already at highest clock rate then
	 *      no need to update the clock
	 */
	ctx_data->clk_info.base_clk = base_clk;
	hw_mgr_clk_info->over_clked = 0;
	if (clk_info->frame_cycles > ctx_data->clk_info.curr_fc) {
		cam_icp_calc_total_clk(hw_mgr, hw_mgr_clk_info,
			ctx_data->icp_dev_acquire_info->dev_type);
		actual_clk = cam_icp_get_actual_clk_rate(hw_mgr,
			ctx_data, base_clk);
		if (hw_mgr_clk_info->base_clk > actual_clk) {
			hw_mgr_clk_info->curr_clk = hw_mgr_clk_info->base_clk;
		} else {
			next_clk_level = cam_icp_get_next_clk_rate(hw_mgr,
				ctx_data, hw_mgr_clk_info->curr_clk);
			hw_mgr_clk_info->curr_clk = next_clk_level;
		}
		rc = true;
	} else {
		next_clk_level =
			cam_icp_get_next_clk_rate(hw_mgr, ctx_data,
			hw_mgr_clk_info->curr_clk);
		if (hw_mgr_clk_info->curr_clk < next_clk_level) {
			hw_mgr_clk_info->curr_clk = next_clk_level;
			rc = true;
		}
	}
	ctx_data->clk_info.curr_fc = clk_info->frame_cycles;

	return rc;
}

static bool cam_icp_update_clk_overclk_free(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *hw_mgr_clk_info,
	struct cam_icp_clk_bw_request *clk_info,
	uint32_t base_clk)
{
	int rc = false;

	/*
	 * In caseof no pending packets case
	 *    1. In caseof overclk cnt is less than threshold, increase
	 *       overclk count and no update in the clock rate
	 *    2. In caseof overclk cnt is greater than or equal to threshold
	 *       then lower clock rate by one level and update hw_mgr current
	 *       clock value.
	 *        a. In case of new clock rate greater than sum of clock
	 *           rates, reset overclk count value to zero if it is
	 *           overclock
	 *        b. if it is less than sum of base clocks then go to next
	 *           level of clock and make overclk count to zero
	 *        c. if it is same as sum of base clock rates update overclock
	 *           cnt to 0
	 */
	if (hw_mgr_clk_info->over_clked < hw_mgr_clk_info->threshold) {
		hw_mgr_clk_info->over_clked++;
		rc = false;
	} else {
		hw_mgr_clk_info->curr_clk =
			cam_icp_get_lower_clk_rate(hw_mgr, ctx_data,
			hw_mgr_clk_info->curr_clk);
		if (hw_mgr_clk_info->curr_clk > hw_mgr_clk_info->base_clk) {
			if (cam_icp_is_over_clk(hw_mgr, ctx_data,
				hw_mgr_clk_info))
				hw_mgr_clk_info->over_clked = 0;
		} else if (hw_mgr_clk_info->curr_clk <
			hw_mgr_clk_info->base_clk) {
			hw_mgr_clk_info->curr_clk =
				cam_icp_get_next_clk_rate(hw_mgr, ctx_data,
				hw_mgr_clk_info->curr_clk);
				hw_mgr_clk_info->over_clked = 0;
		} else if (hw_mgr_clk_info->curr_clk ==
			hw_mgr_clk_info->base_clk) {
			hw_mgr_clk_info->over_clked = 0;
		}
		rc = true;
	}

	return rc;
}

static bool cam_icp_update_clk_free(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *hw_mgr_clk_info,
	struct cam_icp_clk_bw_request *clk_info,
	uint32_t base_clk)
{
	int rc = false;
	bool over_clocked = false;

	ctx_data->clk_info.curr_fc = clk_info->frame_cycles;
	ctx_data->clk_info.base_clk = base_clk;
	cam_icp_calc_total_clk(hw_mgr, hw_mgr_clk_info,
		ctx_data->icp_dev_acquire_info->dev_type);

	/*
	 * Current clock is not always sum of base clocks, due to
	 * clock scales update to next higher or lower levels, it
	 * equals to one of discrete clock values supported by hardware.
	 * So even current clock is higher than sum of base clocks, we
	 * can not consider it is over clocked. if it is greater than
	 * discrete clock level then only it is considered as over clock.
	 * 1. Handle over clock case
	 * 2. If current clock is less than sum of base clocks
	 *    update current clock
	 * 3. If current clock is same as sum of base clocks no action
	 */

	over_clocked = cam_icp_is_over_clk(hw_mgr, ctx_data,
		hw_mgr_clk_info);

	if (hw_mgr_clk_info->curr_clk > hw_mgr_clk_info->base_clk &&
		over_clocked) {
		rc = cam_icp_update_clk_overclk_free(hw_mgr, ctx_data,
			hw_mgr_clk_info, clk_info, base_clk);
	} else if (hw_mgr_clk_info->curr_clk > hw_mgr_clk_info->base_clk) {
		hw_mgr_clk_info->over_clked = 0;
		rc = false;
	}  else if (hw_mgr_clk_info->curr_clk < hw_mgr_clk_info->base_clk) {
		hw_mgr_clk_info->curr_clk = cam_icp_get_actual_clk_rate(hw_mgr,
			ctx_data, hw_mgr_clk_info->base_clk);
		rc = true;
	}

	return rc;
}

static bool cam_icp_debug_clk_update(struct cam_icp_clk_info *hw_mgr_clk_info)
{
	if (icp_hw_mgr.icp_debug_clk &&
		icp_hw_mgr.icp_debug_clk != hw_mgr_clk_info->curr_clk) {
		hw_mgr_clk_info->base_clk = icp_hw_mgr.icp_debug_clk;
		hw_mgr_clk_info->curr_clk = icp_hw_mgr.icp_debug_clk;
		hw_mgr_clk_info->uncompressed_bw = icp_hw_mgr.icp_debug_clk;
		hw_mgr_clk_info->compressed_bw = icp_hw_mgr.icp_debug_clk;
		CAM_DBG(CAM_PERF, "bc = %d cc = %d",
			hw_mgr_clk_info->base_clk, hw_mgr_clk_info->curr_clk);
		return true;
	}

	return false;
}

static bool cam_icp_default_clk_update(struct cam_icp_clk_info *hw_mgr_clk_info)
{
	if (icp_hw_mgr.icp_default_clk != hw_mgr_clk_info->curr_clk) {
		hw_mgr_clk_info->base_clk = icp_hw_mgr.icp_default_clk;
		hw_mgr_clk_info->curr_clk = icp_hw_mgr.icp_default_clk;
		hw_mgr_clk_info->uncompressed_bw = icp_hw_mgr.icp_default_clk;
		hw_mgr_clk_info->compressed_bw = icp_hw_mgr.icp_default_clk;
		CAM_DBG(CAM_PERF, "bc = %d cc = %d",
			hw_mgr_clk_info->base_clk, hw_mgr_clk_info->curr_clk);
		return true;
	}

	return false;
}

static bool cam_icp_update_bw_v2(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *hw_mgr_clk_info,
	struct cam_icp_clk_bw_req_internal_v2 *clk_info,
	bool busy)
{
	uint32_t i, path_index;
	bool update_required = true;

	/*
	 * If current request bandwidth is different from previous frames, then
	 * recalculate bandwidth of all contexts of same hardware and update
	 * voting of bandwidth
	 */

	for (i = 0; i < clk_info->num_paths; i++)
		CAM_DBG(CAM_PERF, "clk_info camnoc = %lld busy = %d",
			clk_info->axi_path[i].camnoc_bw, busy);

	if (clk_info->num_paths == ctx_data->clk_info.num_paths) {
		update_required = false;
		for (i = 0; i < clk_info->num_paths; i++) {
			if ((clk_info->axi_path[i].transac_type ==
			ctx_data->clk_info.axi_path[i].transac_type) &&
			(clk_info->axi_path[i].path_data_type ==
			ctx_data->clk_info.axi_path[i].path_data_type) &&
			(clk_info->axi_path[i].camnoc_bw ==
			ctx_data->clk_info.axi_path[i].camnoc_bw) &&
			(clk_info->axi_path[i].mnoc_ab_bw ==
			ctx_data->clk_info.axi_path[i].mnoc_ab_bw)) {
				continue;
			} else {
				update_required = true;
				break;
			}
		}
	}

	if (!update_required) {
		CAM_DBG(CAM_PERF,
			"Incoming BW hasn't changed, no update required, num_paths=%d",
			clk_info->num_paths);
		return false;
	}

	if (busy) {
		for (i = 0; i < clk_info->num_paths; i++) {
			if (ctx_data->clk_info.axi_path[i].camnoc_bw >
				clk_info->axi_path[i].camnoc_bw)
				return false;
		}
	}

	/*
	 * Remove previous vote of this context from hw mgr first.
	 * hw_mgr_clk_info has all valid paths, with each path in its own index
	 */
	for (i = 0; i < ctx_data->clk_info.num_paths; i++) {
		if (ctx_data->icp_dev_acquire_info->dev_type ==
			CAM_ICP_RES_TYPE_BPS) {
			/* By assuming BPS has Read-All, Write-All votes only */
			path_index =
				ctx_data->clk_info.axi_path[i].transac_type -
				CAM_AXI_TRANSACTION_READ;
		} else {
			path_index =
				ctx_data->clk_info.axi_path[i].path_data_type -
				CAM_AXI_PATH_DATA_IPE_START_OFFSET;
		}

		if (path_index >= CAM_ICP_MAX_PER_PATH_VOTES) {
			CAM_WARN(CAM_PERF,
				"Invalid path %d, start offset=%d, max=%d",
				ctx_data->clk_info.axi_path[i].path_data_type,
				CAM_AXI_PATH_DATA_IPE_START_OFFSET,
				CAM_ICP_MAX_PER_PATH_VOTES);
			continue;
		}

		hw_mgr_clk_info->axi_path[path_index].camnoc_bw -=
			ctx_data->clk_info.axi_path[i].camnoc_bw;
		hw_mgr_clk_info->axi_path[path_index].mnoc_ab_bw -=
			ctx_data->clk_info.axi_path[i].mnoc_ab_bw;
		hw_mgr_clk_info->axi_path[path_index].mnoc_ib_bw -=
			ctx_data->clk_info.axi_path[i].mnoc_ib_bw;
		hw_mgr_clk_info->axi_path[path_index].ddr_ab_bw -=
			ctx_data->clk_info.axi_path[i].ddr_ab_bw;
		hw_mgr_clk_info->axi_path[path_index].ddr_ib_bw -=
			ctx_data->clk_info.axi_path[i].ddr_ib_bw;
	}

	ctx_data->clk_info.num_paths = clk_info->num_paths;

	memcpy(&ctx_data->clk_info.axi_path[0],
		&clk_info->axi_path[0],
		clk_info->num_paths * sizeof(struct cam_axi_per_path_bw_vote));

	/*
	 * Add new vote of this context in hw mgr.
	 * hw_mgr_clk_info has all paths, with each path in its own index
	 */
	for (i = 0; i < ctx_data->clk_info.num_paths; i++) {
		if (ctx_data->icp_dev_acquire_info->dev_type ==
			CAM_ICP_RES_TYPE_BPS) {
			/* By assuming BPS has Read-All, Write-All votes only */
			path_index =
				ctx_data->clk_info.axi_path[i].transac_type -
				CAM_AXI_TRANSACTION_READ;
		} else {
			path_index =
				ctx_data->clk_info.axi_path[i].path_data_type -
				CAM_AXI_PATH_DATA_IPE_START_OFFSET;
		}

		if (path_index >= CAM_ICP_MAX_PER_PATH_VOTES) {
			CAM_WARN(CAM_PERF,
				"Invalid path %d, start offset=%d, max=%d",
				ctx_data->clk_info.axi_path[i].path_data_type,
				CAM_AXI_PATH_DATA_IPE_START_OFFSET,
				CAM_ICP_MAX_PER_PATH_VOTES);
			continue;
		}

		hw_mgr_clk_info->axi_path[path_index].path_data_type =
			ctx_data->clk_info.axi_path[i].path_data_type;
		hw_mgr_clk_info->axi_path[path_index].transac_type =
			ctx_data->clk_info.axi_path[i].transac_type;
		hw_mgr_clk_info->axi_path[path_index].camnoc_bw +=
			ctx_data->clk_info.axi_path[i].camnoc_bw;
		hw_mgr_clk_info->axi_path[path_index].mnoc_ab_bw +=
			ctx_data->clk_info.axi_path[i].mnoc_ab_bw;
		hw_mgr_clk_info->axi_path[path_index].mnoc_ib_bw +=
			ctx_data->clk_info.axi_path[i].mnoc_ib_bw;
		hw_mgr_clk_info->axi_path[path_index].ddr_ab_bw +=
			ctx_data->clk_info.axi_path[i].ddr_ab_bw;
		hw_mgr_clk_info->axi_path[path_index].ddr_ib_bw +=
			ctx_data->clk_info.axi_path[i].ddr_ib_bw;

		CAM_DBG(CAM_PERF,
			"Consolidate Path Vote : Dev[%s] i[%d] path_idx[%d] : [%s %s] [%lld %lld]",
			cam_icp_dev_type_to_name(
			ctx_data->icp_dev_acquire_info->dev_type),
			i, path_index,
			cam_cpas_axi_util_trans_type_to_string(
			hw_mgr_clk_info->axi_path[path_index].transac_type),
			cam_cpas_axi_util_path_type_to_string(
			hw_mgr_clk_info->axi_path[path_index].path_data_type),
			hw_mgr_clk_info->axi_path[path_index].camnoc_bw,
			hw_mgr_clk_info->axi_path[path_index].mnoc_ab_bw);
	}

	ctx_data->clk_info.bw_included = true;

	if (hw_mgr_clk_info->num_paths < ctx_data->clk_info.num_paths)
		hw_mgr_clk_info->num_paths = ctx_data->clk_info.num_paths;

	return true;
}

static bool cam_icp_update_bw(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *hw_mgr_clk_info,
	struct cam_icp_clk_bw_request *clk_info,
	bool busy)
{
	int i;
	struct cam_icp_hw_ctx_data *ctx;

	/*
	 * If current request bandwidth is different from previous frames, then
	 * recalculate bandwidth of all contexts of same hardware and update
	 * voting of bandwidth
	 */
	CAM_DBG(CAM_PERF, "ubw ctx = %lld clk_info ubw = %lld busy = %d",
		ctx_data->clk_info.uncompressed_bw,
		clk_info->uncompressed_bw, busy);

	if ((clk_info->uncompressed_bw == ctx_data->clk_info.uncompressed_bw) &&
		(ctx_data->clk_info.uncompressed_bw ==
		hw_mgr_clk_info->uncompressed_bw)) {
		CAM_DBG(CAM_PERF, "Update not required bw=%lld",
			ctx_data->clk_info.uncompressed_bw);
		return false;
	}

	if (busy &&
		(ctx_data->clk_info.uncompressed_bw >
		clk_info->uncompressed_bw)) {
		CAM_DBG(CAM_PERF,
			"Busy, Update not req existing=%lld, new=%lld",
			ctx_data->clk_info.uncompressed_bw,
			clk_info->uncompressed_bw);
		return false;
	}

	ctx_data->clk_info.uncompressed_bw = clk_info->uncompressed_bw;
	ctx_data->clk_info.compressed_bw = clk_info->compressed_bw;
	hw_mgr_clk_info->uncompressed_bw = 0;
	hw_mgr_clk_info->compressed_bw = 0;
	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		ctx = &hw_mgr->ctx_data[i];
		if (ctx->state == CAM_ICP_CTX_STATE_ACQUIRED &&
			ICP_DEV_TYPE_TO_CLK_TYPE(
			ctx->icp_dev_acquire_info->dev_type) ==
			ICP_DEV_TYPE_TO_CLK_TYPE(
			ctx_data->icp_dev_acquire_info->dev_type)) {
			hw_mgr_clk_info->uncompressed_bw +=
				ctx->clk_info.uncompressed_bw;
			hw_mgr_clk_info->compressed_bw +=
				ctx->clk_info.compressed_bw;
			CAM_DBG(CAM_PERF,
				"Current context=[%lld %lld] Total=[%lld %lld]",
				ctx->clk_info.uncompressed_bw,
				ctx->clk_info.compressed_bw,
				hw_mgr_clk_info->uncompressed_bw,
				hw_mgr_clk_info->compressed_bw);
		}
	}

	ctx_data->clk_info.bw_included = true;

	return true;
}

static bool cam_icp_check_clk_update(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, int idx)
{
	bool busy, rc = false;
	uint32_t base_clk;
	struct cam_icp_clk_bw_request *clk_info;
	struct hfi_frame_process_info *frame_info;
	uint64_t req_id;
	struct cam_icp_clk_info *hw_mgr_clk_info;

	cam_icp_ctx_timer_reset(ctx_data);
	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS) {
		cam_icp_device_timer_reset(hw_mgr, ICP_CLK_HW_BPS);
		hw_mgr_clk_info = &hw_mgr->clk_info[ICP_CLK_HW_BPS];
		CAM_DBG(CAM_PERF, "Reset bps timer");
	} else {
		cam_icp_device_timer_reset(hw_mgr, ICP_CLK_HW_IPE);
		hw_mgr_clk_info = &hw_mgr->clk_info[ICP_CLK_HW_IPE];
		CAM_DBG(CAM_PERF, "Reset ipe timer");
	}

	if (icp_hw_mgr.icp_debug_clk)
		return cam_icp_debug_clk_update(hw_mgr_clk_info);

	/* Check is there any pending frames in this context */
	frame_info = &ctx_data->hfi_frame_process;
	req_id = frame_info->request_id[idx];
	busy = cam_icp_busy_prev_reqs(frame_info, req_id);
	CAM_DBG(CAM_PERF, "busy = %d req_id = %lld", busy, req_id);

	clk_info = &ctx_data->hfi_frame_process.clk_info[idx];
	if (!clk_info->frame_cycles)
		return cam_icp_default_clk_update(hw_mgr_clk_info);

	ctx_data->clk_info.rt_flag = clk_info->rt_flag;

	/* Override base clock to max or calculate base clk rate */
	if (!ctx_data->clk_info.rt_flag &&
		(ctx_data->icp_dev_acquire_info->dev_type !=
		CAM_ICP_RES_TYPE_BPS))
		base_clk = ctx_data->clk_info.clk_rate[CAM_MAX_VOTE-1];
	else
		base_clk = cam_icp_mgr_calc_base_clk(clk_info->frame_cycles,
				clk_info->budget_ns);

	if (busy)
		rc = cam_icp_update_clk_busy(hw_mgr, ctx_data,
			hw_mgr_clk_info, clk_info, base_clk);
	else
		rc = cam_icp_update_clk_free(hw_mgr, ctx_data,
			hw_mgr_clk_info, clk_info, base_clk);

	CAM_DBG(CAM_PERF, "bc = %d cc = %d busy = %d overclk = %d uc = %d",
		hw_mgr_clk_info->base_clk, hw_mgr_clk_info->curr_clk,
		busy, hw_mgr_clk_info->over_clked, rc);

	return rc;
}

static bool cam_icp_check_bw_update(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, int idx)
{
	bool busy, bw_updated = false;
	int i;
	struct cam_icp_clk_bw_request *clk_info;
	struct cam_icp_clk_bw_req_internal_v2 *clk_info_v2;
	struct cam_icp_clk_info *hw_mgr_clk_info;
	struct hfi_frame_process_info *frame_info;
	uint64_t req_id;

	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS)
		hw_mgr_clk_info = &hw_mgr->clk_info[ICP_CLK_HW_BPS];
	else
		hw_mgr_clk_info = &hw_mgr->clk_info[ICP_CLK_HW_IPE];

	frame_info = &ctx_data->hfi_frame_process;
	req_id = frame_info->request_id[idx];
	busy = cam_icp_busy_prev_reqs(frame_info, req_id);

	if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_V1) {
		clk_info = &ctx_data->hfi_frame_process.clk_info[idx];

		CAM_DBG(CAM_PERF,
			"Ctx[%pK][%d] Req[%lld] Current camno=%lld, mnoc=%lld",
			ctx_data, ctx_data->ctx_id, req_id,
			hw_mgr_clk_info->uncompressed_bw,
			hw_mgr_clk_info->compressed_bw);

		bw_updated = cam_icp_update_bw(hw_mgr, ctx_data,
			hw_mgr_clk_info, clk_info, busy);
	} else if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_V2) {
		clk_info_v2 = &ctx_data->hfi_frame_process.clk_info_v2[idx];

		CAM_DBG(CAM_PERF, "index=%d, num_paths=%d, ctx_data=%pK",
			idx, clk_info_v2->num_paths, ctx_data);

		bw_updated = cam_icp_update_bw_v2(hw_mgr, ctx_data,
			hw_mgr_clk_info, clk_info_v2, busy);

		for (i = 0; i < hw_mgr_clk_info->num_paths; i++) {
			CAM_DBG(CAM_PERF,
				"Final path_type: %s, transac_type: %s, camnoc_bw = %lld mnoc_ab_bw = %lld, mnoc_ib_bw = %lld, device: %s",
				cam_cpas_axi_util_path_type_to_string(
				hw_mgr_clk_info->axi_path[i].path_data_type),
				cam_cpas_axi_util_trans_type_to_string(
				hw_mgr_clk_info->axi_path[i].transac_type),
				hw_mgr_clk_info->axi_path[i].camnoc_bw,
				hw_mgr_clk_info->axi_path[i].mnoc_ab_bw,
				hw_mgr_clk_info->axi_path[i].mnoc_ib_bw,
				cam_icp_dev_type_to_name(
				ctx_data->icp_dev_acquire_info->dev_type));
		}
	} else {
		CAM_ERR(CAM_PERF, "Invalid bw config version: %d",
			ctx_data->bw_config_version);
		return false;
	}

	return bw_updated;
}

static int cam_icp_update_clk_rate(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	uint32_t id;
	uint32_t curr_clk_rate;
	struct cam_hw_intf *ipe0_dev_intf = NULL;
	struct cam_hw_intf *ipe1_dev_intf = NULL;
	struct cam_hw_intf *bps_dev_intf = NULL;
	struct cam_hw_intf *icp_dev_intf = NULL;
	struct cam_hw_intf *dev_intf = NULL;
	struct cam_icp_clk_update_cmd clk_upd_cmd;

	ipe0_dev_intf = hw_mgr->ipe0_dev_intf;
	ipe1_dev_intf = hw_mgr->ipe1_dev_intf;
	bps_dev_intf = hw_mgr->bps_dev_intf;
	icp_dev_intf = hw_mgr->icp_dev_intf;


	if ((!ipe0_dev_intf) || (!bps_dev_intf)) {
		CAM_ERR(CAM_ICP, "dev intfs are wrong, failed to update clk");
		return -EINVAL;
	}

	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS) {
		dev_intf = bps_dev_intf;
		curr_clk_rate = hw_mgr->clk_info[ICP_CLK_HW_BPS].curr_clk;
		id = CAM_ICP_BPS_CMD_UPDATE_CLK;
	} else {
		dev_intf = ipe0_dev_intf;
		curr_clk_rate = hw_mgr->clk_info[ICP_CLK_HW_IPE].curr_clk;
		id = CAM_ICP_IPE_CMD_UPDATE_CLK;
	}

	CAM_DBG(CAM_PERF, "clk_rate %u for dev_type %d", curr_clk_rate,
		ctx_data->icp_dev_acquire_info->dev_type);
	clk_upd_cmd.curr_clk_rate = curr_clk_rate;
	clk_upd_cmd.ipe_bps_pc_enable = icp_hw_mgr.ipe_bps_pc_flag;
	clk_upd_cmd.clk_level = -1;

	dev_intf->hw_ops.process_cmd(dev_intf->hw_priv, id,
		&clk_upd_cmd, sizeof(clk_upd_cmd));

	if (ctx_data->icp_dev_acquire_info->dev_type != CAM_ICP_RES_TYPE_BPS) {
		if (ipe1_dev_intf) {
			ipe1_dev_intf->hw_ops.process_cmd(
				ipe1_dev_intf->hw_priv, id,
				&clk_upd_cmd, sizeof(clk_upd_cmd));
		}

		/* update a5 clock */
		CAM_DBG(CAM_PERF, "Update ICP clk to level [%d]",
			clk_upd_cmd.clk_level);
		icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv,
			CAM_ICP_CMD_CLK_UPDATE, &clk_upd_cmd.clk_level,
			sizeof(clk_upd_cmd.clk_level));
	}

	return 0;
}

static int cam_icp_update_cpas_vote(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0;
	uint32_t id;
	uint64_t temp;
	int i = 0;
	struct cam_hw_intf *ipe0_dev_intf = NULL;
	struct cam_hw_intf *ipe1_dev_intf = NULL;
	struct cam_hw_intf *bps_dev_intf = NULL;
	struct cam_hw_intf *dev_intf = NULL;
	struct cam_icp_clk_info *clk_info;
	struct cam_icp_cpas_vote clk_update = {{0}, {0}, 0, 0};
	int device_share_ratio = 1;

	ipe0_dev_intf = hw_mgr->ipe0_dev_intf;
	ipe1_dev_intf = hw_mgr->ipe1_dev_intf;
	bps_dev_intf = hw_mgr->bps_dev_intf;

	if ((!ipe0_dev_intf) || (!bps_dev_intf)) {
		CAM_ERR(CAM_ICP, "dev intfs are wrong, failed to update clk");
		return -EINVAL;
	}

	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS) {
		dev_intf = bps_dev_intf;
		clk_info = &hw_mgr->clk_info[ICP_CLK_HW_BPS];
		id = CAM_ICP_BPS_CMD_VOTE_CPAS;
	} else {
		dev_intf = ipe0_dev_intf;
		clk_info = &hw_mgr->clk_info[ICP_CLK_HW_IPE];
		id = CAM_ICP_IPE_CMD_VOTE_CPAS;
	}

	/*
	 * Since there are 2 devices, we assume the load is evenly shared
	 * between HWs and corresponding AXI paths. So divide total bw by half
	 * to vote on each device
	 */
	if ((ctx_data->icp_dev_acquire_info->dev_type !=
		CAM_ICP_RES_TYPE_BPS) && (ipe1_dev_intf))
		device_share_ratio = 2;

	clk_update.ahb_vote.type = CAM_VOTE_DYNAMIC;
	clk_update.ahb_vote.vote.freq = 0;
	clk_update.ahb_vote_valid = false;

	if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_V1) {
		clk_update.axi_vote.num_paths = 1;
		if (ctx_data->icp_dev_acquire_info->dev_type ==
			CAM_ICP_RES_TYPE_BPS) {
			clk_update.axi_vote.axi_path[0].path_data_type =
				CAM_BPS_DEFAULT_AXI_PATH;
			clk_update.axi_vote.axi_path[0].transac_type =
				CAM_BPS_DEFAULT_AXI_TRANSAC;
		} else {
			clk_update.axi_vote.axi_path[0].path_data_type =
				CAM_IPE_DEFAULT_AXI_PATH;
			clk_update.axi_vote.axi_path[0].transac_type =
				CAM_IPE_DEFAULT_AXI_TRANSAC;
		}

		temp = clk_info->uncompressed_bw;
		do_div(temp, device_share_ratio);
		clk_update.axi_vote.axi_path[0].camnoc_bw = temp;

		temp = clk_info->compressed_bw;
		do_div(temp, device_share_ratio);
		clk_update.axi_vote.axi_path[0].mnoc_ab_bw = temp;
		clk_update.axi_vote.axi_path[0].mnoc_ib_bw = temp;
		clk_update.axi_vote.axi_path[0].ddr_ab_bw = temp;
		clk_update.axi_vote.axi_path[0].ddr_ib_bw = temp;
	} else {
		clk_update.axi_vote.num_paths = clk_info->num_paths;
		memcpy(&clk_update.axi_vote.axi_path[0],
			&clk_info->axi_path[0],
			clk_update.axi_vote.num_paths *
			sizeof(struct cam_axi_per_path_bw_vote));

		if (device_share_ratio > 1) {
			for (i = 0; i < clk_update.axi_vote.num_paths; i++) {
				do_div(
				clk_update.axi_vote.axi_path[i].camnoc_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].mnoc_ab_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].mnoc_ib_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].ddr_ab_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].ddr_ib_bw,
					device_share_ratio);
			}
		}
	}

	clk_update.axi_vote_valid = true;
	rc = dev_intf->hw_ops.process_cmd(dev_intf->hw_priv, id,
		&clk_update, sizeof(clk_update));
	if (rc)
		CAM_ERR(CAM_PERF, "Failed in updating cpas vote, rc=%d", rc);

	/*
	 * Vote half bandwidth each on both devices.
	 * Total bw at mnoc - CPAS will take care of adding up.
	 * camnoc clk calculate is more accurate this way.
	 */
	if ((ctx_data->icp_dev_acquire_info->dev_type !=
		CAM_ICP_RES_TYPE_BPS) && (ipe1_dev_intf)) {
		rc = ipe1_dev_intf->hw_ops.process_cmd(ipe1_dev_intf->hw_priv,
			id, &clk_update, sizeof(clk_update));
		if (rc)
			CAM_ERR(CAM_PERF,
				"Failed in updating cpas vote for ipe 2, rc=%d",
				rc);
	}

	return rc;
}

static int cam_icp_mgr_ipe_bps_clk_update(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, int idx)
{
	int rc = 0;

	if (cam_icp_check_clk_update(hw_mgr, ctx_data, idx))
		rc = cam_icp_update_clk_rate(hw_mgr, ctx_data);

	if (cam_icp_check_bw_update(hw_mgr, ctx_data, idx))
		rc |= cam_icp_update_cpas_vote(hw_mgr, ctx_data);

	return rc;
}

static int cam_icp_mgr_ipe_bps_resume(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	struct cam_hw_intf *ipe0_dev_intf = NULL;
	struct cam_hw_intf *ipe1_dev_intf = NULL;
	struct cam_hw_intf *bps_dev_intf = NULL;
	uint32_t core_info_mask = 0;
	int rc = 0;

	ipe0_dev_intf = hw_mgr->ipe0_dev_intf;
	ipe1_dev_intf = hw_mgr->ipe1_dev_intf;
	bps_dev_intf = hw_mgr->bps_dev_intf;

	if ((!ipe0_dev_intf) || (!bps_dev_intf)) {
		CAM_ERR(CAM_ICP, "dev intfs are wrong, failed to close");
		return -EINVAL;
	}

	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS) {
		if (hw_mgr->bps_ctxt_cnt++)
			goto end;
		if (!hw_mgr->bps_clk_state) {
			bps_dev_intf->hw_ops.init(
				bps_dev_intf->hw_priv, NULL, 0);
			hw_mgr->bps_clk_state = true;
		}
		if (icp_hw_mgr.ipe_bps_pc_flag) {
			bps_dev_intf->hw_ops.process_cmd(
				bps_dev_intf->hw_priv,
				CAM_ICP_BPS_CMD_POWER_RESUME, NULL, 0);
		}
		core_info_mask = ICP_PWR_CLP_BPS;
	} else {
		if (hw_mgr->ipe_ctxt_cnt++)
			goto end;
		if (!hw_mgr->ipe_clk_state)
			ipe0_dev_intf->hw_ops.init(
				ipe0_dev_intf->hw_priv, NULL, 0);
		if (icp_hw_mgr.ipe_bps_pc_flag) {
			ipe0_dev_intf->hw_ops.process_cmd(
				ipe0_dev_intf->hw_priv,
				CAM_ICP_IPE_CMD_POWER_RESUME, NULL, 0);
		}

		if ((icp_hw_mgr.ipe1_enable) &&
			(ipe1_dev_intf) &&
			(!hw_mgr->ipe_clk_state)) {
			ipe1_dev_intf->hw_ops.init(ipe1_dev_intf->hw_priv,
				NULL, 0);

			if (icp_hw_mgr.ipe_bps_pc_flag) {
				ipe1_dev_intf->hw_ops.process_cmd(
					ipe1_dev_intf->hw_priv,
					CAM_ICP_IPE_CMD_POWER_RESUME,
					NULL, 0);
			}
		}
		hw_mgr->ipe_clk_state = true;

		if ((icp_hw_mgr.ipe1_enable) &&
			(ipe1_dev_intf))
			core_info_mask = (ICP_PWR_CLP_IPE0 |
				ICP_PWR_CLP_IPE1);
		else
			core_info_mask = ICP_PWR_CLP_IPE0;
	}

	CAM_DBG(CAM_PERF, "core_info %X", core_info_mask);
	if (icp_hw_mgr.ipe_bps_pc_flag)
		rc = hfi_enable_ipe_bps_pc(true, core_info_mask);
	else
		rc = hfi_enable_ipe_bps_pc(false, core_info_mask);
end:
	return rc;
}

static int cam_icp_mgr_ipe_bps_power_collapse(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, int dev_type)
{
	int rc = 0, dev;
	struct cam_hw_intf *ipe0_dev_intf = NULL;
	struct cam_hw_intf *ipe1_dev_intf = NULL;
	struct cam_hw_intf *bps_dev_intf = NULL;

	ipe0_dev_intf = hw_mgr->ipe0_dev_intf;
	ipe1_dev_intf = hw_mgr->ipe1_dev_intf;
	bps_dev_intf = hw_mgr->bps_dev_intf;

	if ((!ipe0_dev_intf) || (!bps_dev_intf)) {
		CAM_ERR(CAM_ICP, "dev intfs are wrong, failed to close");
		return -EINVAL;
	}

	if (!ctx_data)
		dev = dev_type;
	else
		dev = ctx_data->icp_dev_acquire_info->dev_type;

	if (dev == CAM_ICP_RES_TYPE_BPS) {
		CAM_DBG(CAM_PERF, "bps ctx cnt %d", hw_mgr->bps_ctxt_cnt);
		if (ctx_data)
			--hw_mgr->bps_ctxt_cnt;

		if (hw_mgr->bps_ctxt_cnt)
			goto end;

		if (icp_hw_mgr.ipe_bps_pc_flag &&
			!atomic_read(&hw_mgr->recovery)) {
			rc = bps_dev_intf->hw_ops.process_cmd(
				bps_dev_intf->hw_priv,
				CAM_ICP_BPS_CMD_POWER_COLLAPSE,
				NULL, 0);
		}

		if (hw_mgr->bps_clk_state) {
			bps_dev_intf->hw_ops.deinit
				(bps_dev_intf->hw_priv, NULL, 0);
			hw_mgr->bps_clk_state = false;
		}
	} else {
		CAM_DBG(CAM_PERF, "ipe ctx cnt %d", hw_mgr->ipe_ctxt_cnt);
		if (ctx_data)
			--hw_mgr->ipe_ctxt_cnt;

		if (hw_mgr->ipe_ctxt_cnt)
			goto end;

		if (icp_hw_mgr.ipe_bps_pc_flag &&
			!atomic_read(&hw_mgr->recovery)) {
			rc = ipe0_dev_intf->hw_ops.process_cmd(
				ipe0_dev_intf->hw_priv,
				CAM_ICP_IPE_CMD_POWER_COLLAPSE, NULL, 0);
		}

		if (hw_mgr->ipe_clk_state)
			ipe0_dev_intf->hw_ops.deinit(
				ipe0_dev_intf->hw_priv, NULL, 0);

		if (ipe1_dev_intf) {
			if (icp_hw_mgr.ipe_bps_pc_flag &&
				!atomic_read(&hw_mgr->recovery)) {
				rc = ipe1_dev_intf->hw_ops.process_cmd(
					ipe1_dev_intf->hw_priv,
					CAM_ICP_IPE_CMD_POWER_COLLAPSE,
					NULL, 0);
			}

		if (hw_mgr->ipe_clk_state)
			ipe1_dev_intf->hw_ops.deinit(ipe1_dev_intf->hw_priv,
				NULL, 0);
		}

		hw_mgr->ipe_clk_state = false;
	}

end:
	return rc;
}

static int cam_icp_mgr_ipe_bps_get_gdsc_control(
	struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0;
	struct cam_hw_intf *ipe0_dev_intf = NULL;
	struct cam_hw_intf *ipe1_dev_intf = NULL;
	struct cam_hw_intf *bps_dev_intf = NULL;

	ipe0_dev_intf = hw_mgr->ipe0_dev_intf;
	ipe1_dev_intf = hw_mgr->ipe1_dev_intf;
	bps_dev_intf = hw_mgr->bps_dev_intf;

	if ((!ipe0_dev_intf) || (!bps_dev_intf)) {
		CAM_ERR(CAM_ICP, "dev intfs are wrong");
		return -EINVAL;
	}

	if (icp_hw_mgr.ipe_bps_pc_flag) {
		rc = bps_dev_intf->hw_ops.process_cmd(
			bps_dev_intf->hw_priv,
			CAM_ICP_BPS_CMD_POWER_COLLAPSE,
			NULL, 0);

		rc = ipe0_dev_intf->hw_ops.process_cmd(
			ipe0_dev_intf->hw_priv,
			CAM_ICP_IPE_CMD_POWER_COLLAPSE, NULL, 0);

		if (ipe1_dev_intf) {
			rc = ipe1_dev_intf->hw_ops.process_cmd(
				ipe1_dev_intf->hw_priv,
				CAM_ICP_IPE_CMD_POWER_COLLAPSE,
				NULL, 0);
		}
	}

	return rc;
}

static int cam_icp_set_dbg_default_clk(void *data, u64 val)
{
	icp_hw_mgr.icp_debug_clk = val;
	return 0;
}

static int cam_icp_get_dbg_default_clk(void *data, u64 *val)
{
	*val = icp_hw_mgr.icp_debug_clk;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_icp_debug_default_clk,
	cam_icp_get_dbg_default_clk,
	cam_icp_set_dbg_default_clk, "%16llu");

static int cam_icp_set_icp_dbg_lvl(void *data, u64 val)
{
	icp_hw_mgr.icp_dbg_lvl = val;
	return 0;
}

static int cam_icp_get_icp_dbg_lvl(void *data, u64 *val)
{
	*val = icp_hw_mgr.icp_dbg_lvl;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_icp_debug_fs, cam_icp_get_icp_dbg_lvl,
	cam_icp_set_icp_dbg_lvl, "%08llu");

static int cam_icp_set_icp_dbg_type(void *data, u64 val)
{
	if (val <= NUM_HFI_DEBUG_MODE)
		icp_hw_mgr.icp_debug_type = val;
	return 0;
}

static int cam_icp_get_icp_dbg_type(void *data, u64 *val)
{
	*val = icp_hw_mgr.icp_debug_type;
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(cam_icp_debug_type_fs, cam_icp_get_icp_dbg_type,
	cam_icp_set_icp_dbg_type, "%08llu");

static int cam_icp_set_icp_fw_dump_lvl(void *data, u64 val)
{
	if (val < NUM_HFI_DUMP_LVL)
		icp_hw_mgr.icp_fw_dump_lvl = val;
	return 0;
}

static int cam_icp_get_icp_fw_dump_lvl(void *data, u64 *val)
{
	*val = icp_hw_mgr.icp_fw_dump_lvl;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_icp_debug_fw_dump, cam_icp_get_icp_fw_dump_lvl,
	cam_icp_set_icp_fw_dump_lvl, "%08llu");

static int cam_icp_hw_mgr_create_debugfs_entry(void)
{
	int rc = 0;
	struct dentry *dbgfileptr = NULL;

	dbgfileptr = debugfs_create_dir("camera_icp", NULL);
	if (!dbgfileptr) {
		CAM_ERR(CAM_ICP,"DebugFS could not create directory!");
		rc = -ENOENT;
		goto end;
	}
	/* Store parent inode for cleanup in caller */
	icp_hw_mgr.dentry = dbgfileptr;

	debugfs_create_bool("icp_pc", 0644, icp_hw_mgr.dentry,
		&icp_hw_mgr.icp_pc_flag);

	debugfs_create_bool("ipe_bps_pc", 0644, icp_hw_mgr.dentry,
		&icp_hw_mgr.ipe_bps_pc_flag);

	debugfs_create_file("icp_debug_clk", 0644,
		icp_hw_mgr.dentry, NULL, &cam_icp_debug_default_clk);

	debugfs_create_bool("icp_jtag_debug", 0644,
		icp_hw_mgr.dentry, &icp_hw_mgr.icp_jtag_debug);

	debugfs_create_file("icp_debug_type", 0644,
		icp_hw_mgr.dentry, NULL, &cam_icp_debug_type_fs);

	debugfs_create_file("icp_debug_lvl", 0644,
		icp_hw_mgr.dentry, NULL, &cam_icp_debug_fs);

	debugfs_create_file("icp_fw_dump_lvl", 0644,
		icp_hw_mgr.dentry, NULL, &cam_icp_debug_fw_dump);

	debugfs_create_bool("disable_ubwc_comp", 0644,
		icp_hw_mgr.dentry, &icp_hw_mgr.disable_ubwc_comp);

end:
	/* Set default hang dump lvl */
	icp_hw_mgr.icp_fw_dump_lvl = HFI_FW_DUMP_ON_FAILURE;
	return rc;
}

static int cam_icp_mgr_process_cmd(void *priv, void *data)
{
	int rc;
	struct hfi_cmd_work_data *task_data = NULL;
	struct cam_icp_hw_mgr *hw_mgr;

	if (!data || !priv) {
		CAM_ERR(CAM_ICP, "Invalid params%pK %pK", data, priv);
		return -EINVAL;
	}

	hw_mgr = priv;
	task_data = (struct hfi_cmd_work_data *)data;

	rc = hfi_write_cmd(task_data->data);

	return rc;
}

static int cam_icp_mgr_cleanup_ctx(struct cam_icp_hw_ctx_data *ctx_data)
{
	int i;
	struct hfi_frame_process_info *hfi_frame_process;
	struct cam_hw_done_event_data buf_data;

	hfi_frame_process = &ctx_data->hfi_frame_process;
	for (i = 0; i < CAM_FRAME_CMD_MAX; i++) {
		if (!hfi_frame_process->request_id[i])
			continue;
		buf_data.request_id = hfi_frame_process->request_id[i];
		ctx_data->ctxt_event_cb(ctx_data->context_priv,
			CAM_CTX_EVT_ID_SUCCESS, &buf_data);
		hfi_frame_process->request_id[i] = 0;
		if (ctx_data->hfi_frame_process.in_resource[i] > 0) {
			CAM_DBG(CAM_ICP, "Delete merged sync in object: %d",
				ctx_data->hfi_frame_process.in_resource[i]);
			cam_sync_destroy(
				ctx_data->hfi_frame_process.in_resource[i]);
			ctx_data->hfi_frame_process.in_resource[i] = 0;
		}
		hfi_frame_process->fw_process_flag[i] = false;
		clear_bit(i, ctx_data->hfi_frame_process.bitmap);
	}

	for (i = 0; i < CAM_FRAME_CMD_MAX; i++) {
		if (!hfi_frame_process->in_free_resource[i])
			continue;

		CAM_DBG(CAM_ICP, "Delete merged sync in object: %d",
			ctx_data->hfi_frame_process.in_free_resource[i]);
		cam_sync_destroy(
			ctx_data->hfi_frame_process.in_free_resource[i]);
		ctx_data->hfi_frame_process.in_free_resource[i] = 0;
	}

	return 0;
}

static const char *cam_icp_error_handle_id_to_type(
	uint32_t error_handle)
{
	const char *name = NULL;

	switch (error_handle) {
	case CAMERAICP_SUCCESS:
		name = "SUCCESS";
		break;
	case CAMERAICP_EFAILED:
		name = "EFAILED";
		break;
	case CAMERAICP_ENOMEMORY:
		name = "ENOMEMORY";
		break;
	case CAMERAICP_EBADSTATE:
		name = "EBADSTATE";
		break;
	case CAMERAICP_EBADPARM:
		name = "EBADPARM";
		break;
	case CAMERAICP_EBADITEM:
		name = "EBADITEM";
		break;
	case CAMERAICP_EINVALIDFORMAT:
		name = "EINVALIDFORMAT";
		break;
	case CAMERAICP_EUNSUPPORTED:
		name = "EUNSUPPORTED";
		break;
	case CAMERAICP_EOUTOFBOUND:
		name = "EOUTOFBOUND";
		break;
	case CAMERAICP_ETIMEDOUT:
		name = "ETIMEDOUT";
		break;
	case CAMERAICP_EABORTED:
		name = "EABORTED";
		break;
	case CAMERAICP_EHWVIOLATION:
		name = "EHWVIOLATION";
		break;
	case CAMERAICP_ECDMERROR:
		name = "ECDMERROR";
		break;
	case CAMERAICP_HFI_ERR_COMMAND_SIZE:
		name = "HFI_ERR_COMMAND_SIZE";
		break;
	case CAMERAICP_HFI_ERR_MESSAGE_SIZE:
		name = "HFI_ERR_MESSAGE_SIZE";
		break;
	case CAMERAICP_HFI_QUEUE_EMPTY:
		name = "HFI_QUEUE_EMPTY";
		break;
	case CAMERAICP_HFI_QUEUE_FULL:
		name = "HFI_QUEUE_FULL";
		break;
	default:
		name = NULL;
		break;
	}
	return name;
}

static int cam_icp_mgr_handle_frame_process(uint32_t *msg_ptr, int flag)
{
	int i;
	uint32_t idx;
	uint64_t request_id;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct hfi_msg_ipebps_async_ack *ioconfig_ack = NULL;
	struct hfi_frame_process_info *hfi_frame_process;
	struct cam_hw_done_event_data buf_data;
	uint32_t clk_type;
	uint32_t event_id;

	ioconfig_ack = (struct hfi_msg_ipebps_async_ack *)msg_ptr;
	request_id = ioconfig_ack->user_data2;
	ctx_data = (struct cam_icp_hw_ctx_data *)
		U64_TO_PTR(ioconfig_ack->user_data1);
	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "Invalid Context req %llu", request_id);
		return -EINVAL;
	}

	mutex_lock(&ctx_data->ctx_mutex);
	cam_icp_ctx_timer_reset(ctx_data);
	if (ctx_data->state != CAM_ICP_CTX_STATE_ACQUIRED) {
		CAM_DBG(CAM_ICP, "ctx %u is in %d state",
			ctx_data->ctx_id, ctx_data->state);
		mutex_unlock(&ctx_data->ctx_mutex);
		return 0;
	}

	CAM_DBG(CAM_REQ,
		"ctx_id : %u, request_id :%lld dev_type: %d",
		ctx_data->ctx_id, request_id,
		ctx_data->icp_dev_acquire_info->dev_type);

	clk_type = ICP_DEV_TYPE_TO_CLK_TYPE(
			ctx_data->icp_dev_acquire_info->dev_type);
	cam_icp_device_timer_reset(&icp_hw_mgr, clk_type);

	hfi_frame_process = &ctx_data->hfi_frame_process;
	for (i = 0; i < CAM_FRAME_CMD_MAX; i++)
		if (hfi_frame_process->request_id[i] == request_id)
			break;

	if (i >= CAM_FRAME_CMD_MAX) {
		CAM_ERR(CAM_ICP, "pkt not found in ctx data for req_id =%lld",
			request_id);
		mutex_unlock(&ctx_data->ctx_mutex);
		return -EINVAL;
	}
	idx = i;

	if (flag == ICP_FRAME_PROCESS_FAILURE) {
		buf_data.evt_param = CAM_SYNC_ICP_EVENT_FRAME_PROCESS_FAILURE;
		if (ioconfig_ack->err_type == CAMERAICP_EABORTED) {
			CAM_WARN(CAM_ICP,
				"ctx_id %d req %llu dev %d has been aborted[flushed]",
				ctx_data->ctx_id, request_id,
				ctx_data->icp_dev_acquire_info->dev_type);
			event_id = CAM_CTX_EVT_ID_CANCEL;
		} else {
			CAM_ERR(CAM_ICP,
				"Done with error: %u err_type= [%s] on ctx_id %d dev %d for req %llu",
				ioconfig_ack->err_type,
				cam_icp_error_handle_id_to_type(
				ioconfig_ack->err_type),
				ctx_data->ctx_id,
				ctx_data->icp_dev_acquire_info->dev_type,
				request_id);
			event_id = CAM_CTX_EVT_ID_ERROR;
		}
	} else {
		event_id = CAM_CTX_EVT_ID_SUCCESS;
	}

	buf_data.request_id = hfi_frame_process->request_id[idx];
	ctx_data->ctxt_event_cb(ctx_data->context_priv, event_id, &buf_data);
	hfi_frame_process->request_id[idx] = 0;
	if (ctx_data->hfi_frame_process.in_resource[idx] > 0) {
		CAM_DBG(CAM_ICP, "Delete merged sync in object: %d",
			ctx_data->hfi_frame_process.in_resource[idx]);
		cam_sync_destroy(ctx_data->hfi_frame_process.in_resource[idx]);
		ctx_data->hfi_frame_process.in_resource[idx] = 0;
	}
	clear_bit(idx, ctx_data->hfi_frame_process.bitmap);
	hfi_frame_process->fw_process_flag[idx] = false;
	mutex_unlock(&ctx_data->ctx_mutex);

	return 0;
}

static int cam_icp_mgr_process_msg_frame_process(uint32_t *msg_ptr)
{
	struct hfi_msg_ipebps_async_ack *ioconfig_ack = NULL;
	struct hfi_msg_frame_process_done *frame_done;

	if (!msg_ptr) {
		CAM_ERR(CAM_ICP, "msg ptr is NULL");
		return -EINVAL;
	}

	ioconfig_ack = (struct hfi_msg_ipebps_async_ack *)msg_ptr;
	if (ioconfig_ack->err_type != CAMERAICP_SUCCESS) {
		cam_icp_mgr_handle_frame_process(msg_ptr,
			ICP_FRAME_PROCESS_FAILURE);
		return -EIO;
	}

	frame_done =
		(struct hfi_msg_frame_process_done *)ioconfig_ack->msg_data_flex;
	if (!frame_done) {
		cam_icp_mgr_handle_frame_process(msg_ptr,
			ICP_FRAME_PROCESS_FAILURE);
		return -EINVAL;
	}

	if (frame_done->result)
		return cam_icp_mgr_handle_frame_process(msg_ptr,
			ICP_FRAME_PROCESS_FAILURE);
	else
		return cam_icp_mgr_handle_frame_process(msg_ptr,
			ICP_FRAME_PROCESS_SUCCESS);
}

static int cam_icp_mgr_process_msg_config_io(uint32_t *msg_ptr)
{
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct hfi_msg_ipebps_async_ack *ioconfig_ack = NULL;
	struct hfi_msg_ipe_config *ipe_config_ack = NULL;
	struct hfi_msg_bps_common *bps_config_ack = NULL;

	if (!msg_ptr) {
		CAM_ERR(CAM_ICP, "msg ptr is NULL");
		return -EINVAL;
	}

	ioconfig_ack = (struct hfi_msg_ipebps_async_ack *)msg_ptr;

	if (ioconfig_ack->opcode == HFI_IPEBPS_CMD_OPCODE_IPE_CONFIG_IO) {
		ipe_config_ack =
			(struct hfi_msg_ipe_config *)(ioconfig_ack->msg_data_flex);
		if (ipe_config_ack->rc) {
			CAM_ERR(CAM_ICP, "rc = %d failed with\n"
				"err_no = [%u] err_type = [%s]",
				ipe_config_ack->rc,
				ioconfig_ack->err_type,
				cam_icp_error_handle_id_to_type(
				ioconfig_ack->err_type));

			return -EIO;
		}
		ctx_data = (struct cam_icp_hw_ctx_data *)
			U64_TO_PTR(ioconfig_ack->user_data1);
		if (!ctx_data) {
			CAM_ERR(CAM_ICP, "wrong ctx data from IPE response");
			return -EINVAL;
		}
		ctx_data->scratch_mem_size = ipe_config_ack->scratch_mem_size;
	} else {
		bps_config_ack =
			(struct hfi_msg_bps_common *)(ioconfig_ack->msg_data_flex);
		if (bps_config_ack->rc) {
			CAM_ERR(CAM_ICP, "rc : %u, opcode :%u",
				bps_config_ack->rc, ioconfig_ack->opcode);
			return -EIO;
		}
		ctx_data = (struct cam_icp_hw_ctx_data *)
			U64_TO_PTR(ioconfig_ack->user_data1);
		if (!ctx_data) {
			CAM_ERR(CAM_ICP, "wrong ctx data from BPS response");
			return -EINVAL;
		}
	}
	complete(&ctx_data->wait_complete);

	return 0;
}

static int cam_icp_mgr_process_msg_create_handle(uint32_t *msg_ptr)
{
	struct hfi_msg_create_handle_ack *create_handle_ack = NULL;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	int rc = 0;

	create_handle_ack = (struct hfi_msg_create_handle_ack *)msg_ptr;
	if (!create_handle_ack) {
		CAM_ERR(CAM_ICP, "Invalid create_handle_ack");
		return -EINVAL;
	}

	ctx_data =
		(struct cam_icp_hw_ctx_data *)(uintptr_t)
		create_handle_ack->user_data1;
	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "Invalid ctx_data");
		return -EINVAL;
	}

	if (ctx_data->state == CAM_ICP_CTX_STATE_IN_USE) {
		ctx_data->fw_handle = create_handle_ack->fw_handle;
		CAM_DBG(CAM_ICP, "fw_handle = %x", ctx_data->fw_handle);
	} else {
		CAM_WARN(CAM_ICP,
			"This ctx is no longer in use current state: %d",
			ctx_data->state);
		ctx_data->fw_handle = 0;
		rc = -EPERM;
	}
	complete(&ctx_data->wait_complete);
	return rc;
}

static int cam_icp_mgr_process_msg_ping_ack(uint32_t *msg_ptr)
{
	struct hfi_msg_ping_ack *ping_ack = NULL;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;

	ping_ack = (struct hfi_msg_ping_ack *)msg_ptr;
	if (!ping_ack) {
		CAM_ERR(CAM_ICP, "Empty ping ack message");
		return -EINVAL;
	}

	ctx_data = (struct cam_icp_hw_ctx_data *)
		U64_TO_PTR(ping_ack->user_data);
	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "Invalid ctx_data");
		return -EINVAL;
	}

	if (ctx_data->state == CAM_ICP_CTX_STATE_IN_USE)
		complete(&ctx_data->wait_complete);

	return 0;
}

static int cam_icp_mgr_process_indirect_ack_msg(uint32_t *msg_ptr)
{
	int rc;

	if (!msg_ptr) {
		CAM_ERR(CAM_ICP, "msg ptr is NULL");
		return -EINVAL;
	}

	switch (msg_ptr[ICP_PACKET_OPCODE]) {
	case HFI_IPEBPS_CMD_OPCODE_IPE_CONFIG_IO:
	case HFI_IPEBPS_CMD_OPCODE_BPS_CONFIG_IO:
		CAM_DBG(CAM_ICP, "received IPE/BPS_CONFIG_IO:");
		rc = cam_icp_mgr_process_msg_config_io(msg_ptr);
		if (rc)
			return rc;
		break;

	case HFI_IPEBPS_CMD_OPCODE_IPE_FRAME_PROCESS:
	case HFI_IPEBPS_CMD_OPCODE_BPS_FRAME_PROCESS:
		rc = cam_icp_mgr_process_msg_frame_process(msg_ptr);
		if (rc)
			return rc;
		break;
	default:
		CAM_ERR(CAM_ICP, "Invalid opcode : %u",
			msg_ptr[ICP_PACKET_OPCODE]);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int cam_icp_mgr_process_direct_ack_msg(uint32_t *msg_ptr)
{
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct hfi_msg_ipebps_async_ack *ioconfig_ack = NULL;
	int rc = 0;

	if (!msg_ptr)
		return -EINVAL;

	switch (msg_ptr[ICP_PACKET_OPCODE]) {
	case HFI_IPEBPS_CMD_OPCODE_IPE_ABORT:
	case HFI_IPEBPS_CMD_OPCODE_BPS_ABORT:
		ioconfig_ack = (struct hfi_msg_ipebps_async_ack *)msg_ptr;
		ctx_data = (struct cam_icp_hw_ctx_data *)
			U64_TO_PTR(ioconfig_ack->user_data1);
		if (ctx_data->state != CAM_ICP_CTX_STATE_FREE)
			complete(&ctx_data->wait_complete);
		CAM_DBG(CAM_ICP, "received IPE/BPS/ ABORT: ctx_state =%d",
			ctx_data->state);
		break;
	case HFI_IPEBPS_CMD_OPCODE_IPE_DESTROY:
	case HFI_IPEBPS_CMD_OPCODE_BPS_DESTROY:
		ioconfig_ack = (struct hfi_msg_ipebps_async_ack *)msg_ptr;
		ctx_data = (struct cam_icp_hw_ctx_data *)
			U64_TO_PTR(ioconfig_ack->user_data1);
		if ((ctx_data->state == CAM_ICP_CTX_STATE_RELEASE) ||
			(ctx_data->state == CAM_ICP_CTX_STATE_IN_USE)) {
			complete(&ctx_data->wait_complete);
		}
		CAM_DBG(CAM_ICP, "received IPE/BPS/ DESTROY: ctx_state =%d",
			ctx_data->state);
		break;
	case HFI_IPEBPS_CMD_OPCODE_MEM_MAP:
		ioconfig_ack = (struct hfi_msg_ipebps_async_ack *)msg_ptr;
		ctx_data =
			(struct cam_icp_hw_ctx_data *)ioconfig_ack->user_data1;
		if (ctx_data->state != CAM_ICP_CTX_STATE_FREE)
			complete(&ctx_data->wait_complete);
		CAM_DBG(CAM_ICP, "received IPE/BPS\n"
			"MAP ACK:ctx_state =%d\n"
			"failed with err_no = [%u] err_type = [%s]",
			ctx_data->state,
			ioconfig_ack->err_type,
			cam_icp_error_handle_id_to_type(
			ioconfig_ack->err_type));
		break;
	case HFI_IPEBPS_CMD_OPCODE_MEM_UNMAP:
		ioconfig_ack = (struct hfi_msg_ipebps_async_ack *)msg_ptr;
		ctx_data =
			(struct cam_icp_hw_ctx_data *)ioconfig_ack->user_data1;
		if (ctx_data->state != CAM_ICP_CTX_STATE_FREE)
			complete(&ctx_data->wait_complete);
		CAM_DBG(CAM_ICP,
			"received IPE/BPS UNMAP ACK:ctx_state =%d\n"
			"failed with err_no = [%u] err_type = [%s]",
			ctx_data->state,
			ioconfig_ack->err_type,
			cam_icp_error_handle_id_to_type(
			ioconfig_ack->err_type));
		break;
	default:
		CAM_ERR(CAM_ICP, "Invalid opcode : %u",
			msg_ptr[ICP_PACKET_OPCODE]);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int cam_icp_ipebps_reset(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0;
	struct cam_hw_intf *ipe0_dev_intf;
	struct cam_hw_intf *ipe1_dev_intf;
	struct cam_hw_intf *bps_dev_intf;

	ipe0_dev_intf = hw_mgr->ipe0_dev_intf;
	ipe1_dev_intf = hw_mgr->ipe1_dev_intf;
	bps_dev_intf = hw_mgr->bps_dev_intf;

	if (hw_mgr->bps_ctxt_cnt) {
		rc = bps_dev_intf->hw_ops.process_cmd(
			bps_dev_intf->hw_priv,
			CAM_ICP_BPS_CMD_RESET,
			NULL, 0);
		if (rc)
			CAM_ERR(CAM_ICP, "bps reset failed");
	}

	if (hw_mgr->ipe_ctxt_cnt) {
		rc = ipe0_dev_intf->hw_ops.process_cmd(
			ipe0_dev_intf->hw_priv,
			CAM_ICP_IPE_CMD_RESET,
			NULL, 0);
		if (rc)
			CAM_ERR(CAM_ICP, "ipe0 reset failed");

		if (ipe1_dev_intf) {
			rc = ipe1_dev_intf->hw_ops.process_cmd(
				ipe1_dev_intf->hw_priv,
				CAM_ICP_IPE_CMD_RESET,
				NULL, 0);
			if (rc)
				CAM_ERR(CAM_ICP, "ipe1 reset failed");
		}
	}

	return 0;
}

static int cam_icp_mgr_trigger_recovery(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0;
	struct sfr_buf *sfr_buffer = NULL;

	CAM_DBG(CAM_ICP, "Enter");

	if (atomic_read(&hw_mgr->recovery)) {
		CAM_ERR(CAM_ICP, "Recovery is set");
		return rc;
	}

	sfr_buffer = (struct sfr_buf *)icp_hw_mgr.hfi_mem.sfr_buf.kva;
	CAM_WARN(CAM_ICP, "SFR:%s", sfr_buffer->msg);

	cam_icp_mgr_ipe_bps_get_gdsc_control(hw_mgr);
	cam_icp_ipebps_reset(hw_mgr);

	atomic_set(&hw_mgr->recovery, 1);
	CAM_DBG(CAM_ICP, "Done");
	return rc;
}
static int cam_icp_mgr_process_fatal_error(
	struct cam_icp_hw_mgr *hw_mgr, uint32_t *msg_ptr)
{
	struct hfi_msg_event_notify *event_notify;
	int rc = 0;

	CAM_DBG(CAM_ICP, "Enter");

	event_notify = (struct hfi_msg_event_notify *)msg_ptr;
	if (!event_notify) {
		CAM_ERR(CAM_ICP, "Empty event message");
		return -EINVAL;
	}

	CAM_DBG(CAM_ICP, "evt_id: %u evt_data1: %u evt_data2: %u",
		event_notify->event_id,
		event_notify->event_data1,
		event_notify->event_data2);

	if (event_notify->event_id == HFI_EVENT_SYS_ERROR) {
		CAM_INFO(CAM_ICP, "received HFI_EVENT_SYS_ERROR");
		if (event_notify->event_data1 == HFI_ERR_SYS_FATAL) {
			CAM_ERR(CAM_ICP, "received HFI_ERR_SYS_FATAL");
			BUG();
		}
		rc = cam_icp_mgr_trigger_recovery(hw_mgr);
		cam_icp_mgr_process_dbg_buf(icp_hw_mgr.icp_dbg_lvl);
	}

	return rc;
}

static void cam_icp_mgr_process_dbg_buf(unsigned int debug_lvl)
{
	uint32_t *msg_ptr = NULL, *pkt_ptr = NULL;
	struct hfi_msg_debug *dbg_msg;
	uint32_t read_len, size_processed = 0;
	uint64_t timestamp = 0;
	char *dbg_buf;
	int rc = 0;

	rc = hfi_read_message(icp_hw_mgr.dbg_buf, Q_DBG, &read_len);
	if (rc)
		return;

	msg_ptr = (uint32_t *)icp_hw_mgr.dbg_buf;
	while (true) {
		pkt_ptr = msg_ptr;
		if (pkt_ptr[ICP_PACKET_TYPE] == HFI_MSG_SYS_DEBUG) {
			dbg_msg = (struct hfi_msg_debug *)pkt_ptr;
			dbg_buf = (char *)&dbg_msg->msg_data_flex;
			timestamp = ((((uint64_t)(dbg_msg->timestamp_hi) << 32)
				| dbg_msg->timestamp_lo) >> 16);
			trace_cam_icp_fw_dbg(dbg_buf, timestamp/2);
			if (!debug_lvl)
				CAM_INFO(CAM_ICP, "FW_DBG:%s", dbg_buf);
		}
		size_processed += (pkt_ptr[ICP_PACKET_SIZE] >>
			BYTE_WORD_SHIFT);
		if (size_processed >= read_len)
			return;
		msg_ptr += (pkt_ptr[ICP_PACKET_SIZE] >>
		BYTE_WORD_SHIFT);
		pkt_ptr = NULL;
		dbg_msg = NULL;
		dbg_buf = NULL;
	}
}

static int cam_icp_process_msg_pkt_type(
	struct cam_icp_hw_mgr *hw_mgr,
	uint32_t *msg_ptr,
	uint32_t *msg_processed_len)
{
	int rc = 0;
	int size_processed = 0;

	switch (msg_ptr[ICP_PACKET_TYPE]) {
	case HFI_MSG_SYS_INIT_DONE:
		CAM_DBG(CAM_ICP, "received SYS_INIT_DONE");
		complete(&hw_mgr->icp_complete);
		size_processed = (
			(struct hfi_msg_init_done *)msg_ptr)->size;
		break;

	case HFI_MSG_SYS_PC_PREP_DONE:
		CAM_DBG(CAM_ICP, "HFI_MSG_SYS_PC_PREP_DONE is received\n");
		complete(&hw_mgr->icp_complete);
		size_processed = sizeof(struct hfi_msg_pc_prep_done);
		break;

	case HFI_MSG_SYS_PING_ACK:
		CAM_DBG(CAM_ICP, "received SYS_PING_ACK");
		rc = cam_icp_mgr_process_msg_ping_ack(msg_ptr);
		size_processed = sizeof(struct hfi_msg_ping_ack);
		break;

	case HFI_MSG_IPEBPS_CREATE_HANDLE_ACK:
		CAM_DBG(CAM_ICP, "received IPEBPS_CREATE_HANDLE_ACK");
		rc = cam_icp_mgr_process_msg_create_handle(msg_ptr);
		size_processed = sizeof(struct hfi_msg_create_handle_ack);
		break;

	case HFI_MSG_IPEBPS_ASYNC_COMMAND_INDIRECT_ACK:
		CAM_DBG(CAM_ICP, "received ASYNC_INDIRECT_ACK");
		rc = cam_icp_mgr_process_indirect_ack_msg(msg_ptr);
		size_processed = (
			(struct hfi_msg_ipebps_async_ack *)msg_ptr)->size;
		break;

	case  HFI_MSG_IPEBPS_ASYNC_COMMAND_DIRECT_ACK:
		CAM_DBG(CAM_ICP, "received ASYNC_DIRECT_ACK");
		rc = cam_icp_mgr_process_direct_ack_msg(msg_ptr);
		size_processed = (
			(struct hfi_msg_ipebps_async_ack *)msg_ptr)->size;
		break;

	case HFI_MSG_EVENT_NOTIFY:
		CAM_DBG(CAM_ICP, "received EVENT_NOTIFY");
		size_processed = (
			(struct hfi_msg_event_notify *)msg_ptr)->size;
		rc = cam_icp_mgr_process_fatal_error(hw_mgr, msg_ptr);
		if (rc)
			CAM_ERR(CAM_ICP, "failed in processing evt notify");

		break;

	default:
		CAM_ERR(CAM_ICP, "invalid msg : %u",
			msg_ptr[ICP_PACKET_TYPE]);
		rc = -EINVAL;
		break;
	}

	*msg_processed_len = size_processed;
	return rc;
}

static int32_t cam_icp_mgr_process_msg(void *priv, void *data)
{
	uint32_t read_len, msg_processed_len;
	uint32_t *msg_ptr = NULL;
	struct hfi_msg_work_data *task_data;
	struct cam_icp_hw_mgr *hw_mgr;
	int rc = 0;

	if (!data || !priv) {
		CAM_ERR(CAM_ICP, "Invalid data");
		return -EINVAL;
	}

	task_data = data;
	hw_mgr = priv;

	rc = hfi_read_message(icp_hw_mgr.msg_buf, Q_MSG, &read_len);
	if (rc) {
		CAM_DBG(CAM_ICP, "Unable to read msg q rc %d", rc);
	} else {
		read_len = read_len << BYTE_WORD_SHIFT;
		msg_ptr = (uint32_t *)icp_hw_mgr.msg_buf;
		while (true) {
			cam_icp_process_msg_pkt_type(hw_mgr, msg_ptr,
				&msg_processed_len);

			if (!msg_processed_len) {
				CAM_ERR(CAM_ICP, "Failed to read");
				rc = -EINVAL;
				break;
			}

			read_len -= msg_processed_len;
			if (read_len > 0) {
				msg_ptr += (msg_processed_len >>
				BYTE_WORD_SHIFT);
				msg_processed_len = 0;
			} else {
				break;
			}
		}
	}

	cam_icp_mgr_process_dbg_buf(icp_hw_mgr.icp_dbg_lvl);

	if ((task_data->irq_status & A5_WDT_0) ||
		(task_data->irq_status & A5_WDT_1)) {
		CAM_ERR_RATE_LIMIT(CAM_ICP, "watch dog interrupt from A5");

		rc = cam_icp_mgr_trigger_recovery(hw_mgr);
	}

	return rc;
}

static int32_t cam_icp_hw_mgr_cb(uint32_t irq_status, void *data)
{
	int32_t rc = 0;
	unsigned long flags;
	struct cam_icp_hw_mgr *hw_mgr = data;
	struct crm_workq_task *task;
	struct hfi_msg_work_data *task_data;

	if (!data) {
		CAM_ERR(CAM_ICP, "irq cb data is NULL");
		return rc;
	}

	spin_lock_irqsave(&hw_mgr->hw_mgr_lock, flags);
	task = cam_req_mgr_workq_get_task(icp_hw_mgr.msg_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "no empty task");
		spin_unlock_irqrestore(&hw_mgr->hw_mgr_lock, flags);
		return -ENOMEM;
	}

	task_data = (struct hfi_msg_work_data *)task->payload;
	task_data->data = hw_mgr;
	task_data->irq_status = irq_status;
	task_data->type = ICP_WORKQ_TASK_MSG_TYPE;
	task->process_cb = cam_icp_mgr_process_msg;
	rc = cam_req_mgr_workq_enqueue_task(task, &icp_hw_mgr,
		CRM_TASK_PRIORITY_0);
	spin_unlock_irqrestore(&hw_mgr->hw_mgr_lock, flags);

	return rc;
}

static void cam_icp_free_hfi_mem(void)
{
	int rc;

	cam_smmu_dealloc_firmware(icp_hw_mgr.iommu_hdl);
	rc = cam_mem_mgr_free_memory_region(&icp_hw_mgr.hfi_mem.sec_heap);
	if (rc)
		CAM_ERR(CAM_ICP, "failed to unreserve sec heap");

	cam_smmu_dealloc_qdss(icp_hw_mgr.iommu_hdl);
	cam_mem_mgr_release_mem(&icp_hw_mgr.hfi_mem.qtbl);
	cam_mem_mgr_release_mem(&icp_hw_mgr.hfi_mem.cmd_q);
	cam_mem_mgr_release_mem(&icp_hw_mgr.hfi_mem.msg_q);
	cam_mem_mgr_release_mem(&icp_hw_mgr.hfi_mem.dbg_q);
	cam_mem_mgr_release_mem(&icp_hw_mgr.hfi_mem.sfr_buf);
}

static int cam_icp_alloc_secheap_mem(struct cam_mem_mgr_memory_desc *secheap)
{
	int rc;
	struct cam_mem_mgr_request_desc alloc;
	struct cam_mem_mgr_memory_desc out;
	struct cam_smmu_region_info secheap_info;

	memset(&alloc, 0, sizeof(alloc));
	memset(&out, 0, sizeof(out));

	rc = cam_smmu_get_region_info(icp_hw_mgr.iommu_hdl,
		CAM_SMMU_REGION_SECHEAP,
		&secheap_info);
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to get secheap memory info");
		return rc;
	}

	alloc.size = secheap_info.iova_len;
	alloc.align = 0;
	alloc.flags = 0;
	alloc.smmu_hdl = icp_hw_mgr.iommu_hdl;
	rc = cam_mem_mgr_reserve_memory_region(&alloc,
		CAM_SMMU_REGION_SECHEAP,
		&out);
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to reserve secheap memory");
		return rc;
	}

	*secheap = out;
	CAM_DBG(CAM_ICP, "kva: %llX, iova: %x, hdl: %x, len: %lld",
		out.kva, out.iova, out.mem_handle, out.len);

	return rc;
}

static int cam_icp_alloc_sfr_mem(struct cam_mem_mgr_memory_desc *sfr)
{
	int rc;
	struct cam_mem_mgr_request_desc alloc;
	struct cam_mem_mgr_memory_desc out;

	memset(&alloc, 0, sizeof(alloc));
	memset(&out, 0, sizeof(out));
	alloc.size = SZ_8K;
	alloc.align = 0;
	alloc.flags = CAM_MEM_FLAG_HW_READ_WRITE |
		CAM_MEM_FLAG_HW_SHARED_ACCESS;

	alloc.smmu_hdl = icp_hw_mgr.iommu_hdl;
	rc = cam_mem_mgr_request_mem(&alloc, &out);
	if (rc)
		return rc;

	*sfr = out;
	CAM_DBG(CAM_ICP, "kva: %llX, iova: %x, hdl: %x, len: %lld",
		out.kva, out.iova, out.mem_handle, out.len);

	return rc;
}

static int cam_icp_alloc_shared_mem(struct cam_mem_mgr_memory_desc *qtbl)
{
	int rc;
	struct cam_mem_mgr_request_desc alloc;
	struct cam_mem_mgr_memory_desc out;

	memset(&alloc, 0, sizeof(alloc));
	memset(&out, 0, sizeof(out));
	alloc.size = SZ_1M;
	alloc.align = 0;
	alloc.flags = CAM_MEM_FLAG_HW_READ_WRITE |
		CAM_MEM_FLAG_HW_SHARED_ACCESS;
	alloc.smmu_hdl = icp_hw_mgr.iommu_hdl;
	rc = cam_mem_mgr_request_mem(&alloc, &out);
	if (rc)
		return rc;

	*qtbl = out;
	CAM_DBG(CAM_ICP, "kva: %llX, iova: %x, hdl: %x, len: %lld",
		out.kva, out.iova, out.mem_handle, out.len);

	return rc;
}

static int cam_icp_allocate_fw_mem(void)
{
	int rc;
	uintptr_t kvaddr;
	size_t len;
	dma_addr_t iova;

	rc = cam_smmu_alloc_firmware(icp_hw_mgr.iommu_hdl,
		&iova, &kvaddr, &len);
	if (rc)
		return -ENOMEM;

	icp_hw_mgr.hfi_mem.fw_buf.len = len;
	icp_hw_mgr.hfi_mem.fw_buf.kva = kvaddr;
	icp_hw_mgr.hfi_mem.fw_buf.iova = iova;
	icp_hw_mgr.hfi_mem.fw_buf.smmu_hdl = icp_hw_mgr.iommu_hdl;

	CAM_DBG(CAM_ICP, "kva: %zX, iova: %llx, len: %zu",
		kvaddr, iova, len);

	return rc;
}

static int cam_icp_allocate_qdss_mem(void)
{
	int rc;
	size_t len;
	dma_addr_t iova;

	rc = cam_smmu_alloc_qdss(icp_hw_mgr.iommu_hdl,
		&iova, &len);
	if (rc)
		return rc;

	icp_hw_mgr.hfi_mem.qdss_buf.len = len;
	icp_hw_mgr.hfi_mem.qdss_buf.iova = iova;
	icp_hw_mgr.hfi_mem.qdss_buf.smmu_hdl = icp_hw_mgr.iommu_hdl;

	CAM_DBG(CAM_ICP, "iova: %llx, len: %zu", iova, len);

	return rc;
}

static int cam_icp_get_io_mem_info(void)
{
	int rc;
	size_t len, discard_iova_len;
	dma_addr_t iova, discard_iova_start;

	rc = cam_smmu_get_io_region_info(icp_hw_mgr.iommu_hdl,
		&iova, &len, &discard_iova_start, &discard_iova_len);
	if (rc)
		return rc;

	icp_hw_mgr.hfi_mem.io_mem.iova_len = len;
	icp_hw_mgr.hfi_mem.io_mem.iova_start = iova;
	icp_hw_mgr.hfi_mem.io_mem.discard_iova_start = discard_iova_start;
	icp_hw_mgr.hfi_mem.io_mem.discard_iova_len = discard_iova_len;

	CAM_DBG(CAM_ICP, "iova: %llx, len: %zu discard iova %llx len %llx",
		iova, len, discard_iova_start, discard_iova_len);

	return rc;
}

static int cam_icp_allocate_hfi_mem(void)
{
	int rc;

	rc = cam_smmu_get_region_info(icp_hw_mgr.iommu_hdl,
		CAM_SMMU_REGION_SHARED,
		&icp_hw_mgr.hfi_mem.shmem);
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to get shared memory info");
		return rc;
	}

	rc = cam_icp_allocate_fw_mem();
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to allocate FW memory");
		return rc;
	}

	rc = cam_icp_allocate_qdss_mem();
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to allocate qdss memory");
		goto fw_alloc_failed;
	}

	rc = cam_icp_alloc_shared_mem(&icp_hw_mgr.hfi_mem.qtbl);
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to allocate qtbl memory");
		goto qtbl_alloc_failed;
	}

	rc = cam_icp_alloc_shared_mem(&icp_hw_mgr.hfi_mem.cmd_q);
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to allocate cmd q memory");
		goto cmd_q_alloc_failed;
	}

	rc = cam_icp_alloc_shared_mem(&icp_hw_mgr.hfi_mem.msg_q);
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to allocate msg q memory");
		goto msg_q_alloc_failed;
	}

	rc = cam_icp_alloc_shared_mem(&icp_hw_mgr.hfi_mem.dbg_q);
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to allocate dbg q memory");
		goto dbg_q_alloc_failed;
	}

	rc = cam_icp_alloc_sfr_mem(&icp_hw_mgr.hfi_mem.sfr_buf);
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to allocate sfr buffer");
		goto sfr_buf_alloc_failed;
	}

	rc = cam_icp_alloc_secheap_mem(&icp_hw_mgr.hfi_mem.sec_heap);
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to allocate sec heap memory");
		goto sec_heap_alloc_failed;
	}

	rc = cam_icp_get_io_mem_info();
	if (rc) {
		CAM_ERR(CAM_ICP, "Unable to get I/O region info");
		goto get_io_mem_failed;
	}

	return rc;
get_io_mem_failed:
	cam_mem_mgr_free_memory_region(&icp_hw_mgr.hfi_mem.sec_heap);
sec_heap_alloc_failed:
	cam_mem_mgr_release_mem(&icp_hw_mgr.hfi_mem.sfr_buf);
sfr_buf_alloc_failed:
	cam_mem_mgr_release_mem(&icp_hw_mgr.hfi_mem.dbg_q);
dbg_q_alloc_failed:
	cam_mem_mgr_release_mem(&icp_hw_mgr.hfi_mem.msg_q);
msg_q_alloc_failed:
	cam_mem_mgr_release_mem(&icp_hw_mgr.hfi_mem.cmd_q);
cmd_q_alloc_failed:
	cam_mem_mgr_release_mem(&icp_hw_mgr.hfi_mem.qtbl);
qtbl_alloc_failed:
	cam_smmu_dealloc_qdss(icp_hw_mgr.iommu_hdl);
fw_alloc_failed:
	cam_smmu_dealloc_firmware(icp_hw_mgr.iommu_hdl);
	return rc;
}

static int cam_icp_mgr_get_free_ctx(struct cam_icp_hw_mgr *hw_mgr)
{
	int i = 0;

	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		mutex_lock(&hw_mgr->ctx_data[i].ctx_mutex);
		if (hw_mgr->ctx_data[i].state == CAM_ICP_CTX_STATE_FREE) {
			hw_mgr->ctx_data[i].state = CAM_ICP_CTX_STATE_IN_USE;
			mutex_unlock(&hw_mgr->ctx_data[i].ctx_mutex);
			break;
		}
		mutex_unlock(&hw_mgr->ctx_data[i].ctx_mutex);
	}

	return i;
}

static void cam_icp_mgr_put_ctx(struct cam_icp_hw_ctx_data *ctx_data)
{
	ctx_data->state = CAM_ICP_CTX_STATE_FREE;
}

static int cam_icp_mgr_send_pc_prep(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	unsigned long rem_jiffies;
	int timeout = 5000;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is NULL");
		return -EINVAL;
	}

	reinit_completion(&hw_mgr->icp_complete);
	CAM_DBG(CAM_ICP, "Sending HFI init command");
	rc = icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv,
					CAM_ICP_CMD_PC_PREP, NULL, 0);
	if (rc)
		return rc;

	CAM_DBG(CAM_ICP, "Wait for PC_PREP_DONE Message\n");
	rem_jiffies = wait_for_completion_timeout(&icp_hw_mgr.icp_complete,
		msecs_to_jiffies((timeout)));
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		CAM_ERR(CAM_ICP, "PC_PREP response timed out %d\n", rc);
	}
	CAM_DBG(CAM_ICP, "Done Waiting for PC_PREP Message\n");

	return rc;
}

static int cam_ipe_bps_deint(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *ipe0_dev_intf = NULL;
	struct cam_hw_intf *ipe1_dev_intf = NULL;
	struct cam_hw_intf *bps_dev_intf = NULL;

	ipe0_dev_intf = hw_mgr->ipe0_dev_intf;
	ipe1_dev_intf = hw_mgr->ipe1_dev_intf;
	bps_dev_intf = hw_mgr->bps_dev_intf;
	if ((!ipe0_dev_intf) || (!bps_dev_intf)) {
		CAM_ERR(CAM_ICP, "dev intfs are wrong, failed to close");
		return 0;
	}

	if (ipe1_dev_intf && hw_mgr->ipe_clk_state) {
		ipe1_dev_intf->hw_ops.deinit(ipe1_dev_intf->hw_priv,
				NULL, 0);
	}

	if (hw_mgr->ipe_clk_state)
		ipe0_dev_intf->hw_ops.deinit(ipe0_dev_intf->hw_priv, NULL, 0);
	if (hw_mgr->bps_clk_state)
		bps_dev_intf->hw_ops.deinit(bps_dev_intf->hw_priv, NULL, 0);


	hw_mgr->bps_clk_state = false;
	hw_mgr->ipe_clk_state = false;

	return 0;
}

static int cam_icp_mgr_hw_close_u(void *hw_priv, void *hw_close_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_priv;
	int rc = 0;

	CAM_DBG(CAM_ICP, "UMD calls close");
	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "Null hw mgr");
		return 0;
	}

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	rc = cam_icp_mgr_hw_close(hw_mgr, NULL);
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	return rc;
}

static int cam_icp_mgr_hw_close_k(void *hw_priv, void *hw_close_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_priv;

	CAM_DBG(CAM_ICP, "KMD calls close");
	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "Null hw mgr");
		return 0;
	}

	return cam_icp_mgr_hw_close(hw_mgr, NULL);

}

static int cam_icp_mgr_proc_resume(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;

	if (!icp_dev_intf)
		return -EINVAL;

	return icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv,
						CAM_ICP_CMD_POWER_RESUME,
						&hw_mgr->icp_jtag_debug,
						sizeof(hw_mgr->icp_jtag_debug));
}

static void cam_icp_mgr_proc_suspend(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;

	if (!icp_dev_intf)
		return;

	icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv,
					CAM_ICP_CMD_POWER_COLLAPSE,
					NULL, 0);
}

static int __power_collapse(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0;

	if (!hw_mgr->icp_pc_flag || atomic_read(&hw_mgr->recovery)) {
		cam_icp_mgr_proc_suspend(hw_mgr);
		rc = cam_icp_mgr_hw_close_k(hw_mgr, NULL);
	} else {
		CAM_DBG(CAM_PERF, "Sending PC prep ICP PC enabled");
		rc = cam_icp_mgr_send_pc_prep(hw_mgr);
		cam_icp_mgr_proc_suspend(hw_mgr);
	}

	return rc;
}

static int cam_icp_mgr_icp_power_collapse(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	int rc;

	CAM_DBG(CAM_PERF, "ENTER");

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is NULL");
		return -EINVAL;
	}

	rc = __power_collapse(hw_mgr);

	icp_dev_intf->hw_ops.deinit(icp_dev_intf->hw_priv, NULL, 0);

	CAM_DBG(CAM_PERF, "EXIT");

	return rc;
}

static int cam_icp_mgr_hfi_resume(struct cam_icp_hw_mgr *hw_mgr)
{
	struct hfi_mem_info hfi_mem;

	hfi_mem.qtbl.kva = icp_hw_mgr.hfi_mem.qtbl.kva;
	hfi_mem.qtbl.iova = icp_hw_mgr.hfi_mem.qtbl.iova;
	hfi_mem.qtbl.len = icp_hw_mgr.hfi_mem.qtbl.len;
	CAM_DBG(CAM_ICP, "qtbl kva = %llX IOVA = %X length = %lld\n",
		hfi_mem.qtbl.kva, hfi_mem.qtbl.iova, hfi_mem.qtbl.len);

	hfi_mem.cmd_q.kva = icp_hw_mgr.hfi_mem.cmd_q.kva;
	hfi_mem.cmd_q.iova = icp_hw_mgr.hfi_mem.cmd_q.iova;
	hfi_mem.cmd_q.len = icp_hw_mgr.hfi_mem.cmd_q.len;
	CAM_DBG(CAM_ICP, "cmd_q kva = %llX IOVA = %X length = %lld\n",
		hfi_mem.cmd_q.kva, hfi_mem.cmd_q.iova, hfi_mem.cmd_q.len);

	hfi_mem.msg_q.kva = icp_hw_mgr.hfi_mem.msg_q.kva;
	hfi_mem.msg_q.iova = icp_hw_mgr.hfi_mem.msg_q.iova;
	hfi_mem.msg_q.len = icp_hw_mgr.hfi_mem.msg_q.len;
	CAM_DBG(CAM_ICP, "msg_q kva = %llX IOVA = %X length = %lld\n",
		hfi_mem.msg_q.kva, hfi_mem.msg_q.iova, hfi_mem.msg_q.len);

	hfi_mem.dbg_q.kva = icp_hw_mgr.hfi_mem.dbg_q.kva;
	hfi_mem.dbg_q.iova = icp_hw_mgr.hfi_mem.dbg_q.iova;
	hfi_mem.dbg_q.len = icp_hw_mgr.hfi_mem.dbg_q.len;
	CAM_DBG(CAM_ICP, "dbg_q kva = %llX IOVA = %X length = %lld\n",
		hfi_mem.dbg_q.kva, hfi_mem.dbg_q.iova, hfi_mem.dbg_q.len);

	hfi_mem.sfr_buf.kva = icp_hw_mgr.hfi_mem.sfr_buf.kva;
	hfi_mem.sfr_buf.iova = icp_hw_mgr.hfi_mem.sfr_buf.iova;
	hfi_mem.sfr_buf.len = icp_hw_mgr.hfi_mem.sfr_buf.len;
	CAM_DBG(CAM_ICP, "sfr_buf kva = %llX IOVA = %X length = %lld\n",
		hfi_mem.sfr_buf.kva, hfi_mem.sfr_buf.iova,
		hfi_mem.sfr_buf.len);

	hfi_mem.sec_heap.kva = icp_hw_mgr.hfi_mem.sec_heap.kva;
	hfi_mem.sec_heap.iova = icp_hw_mgr.hfi_mem.sec_heap.iova;
	hfi_mem.sec_heap.len = icp_hw_mgr.hfi_mem.sec_heap.len;
	CAM_DBG(CAM_ICP, "sec_heap kva = %llX IOVA = %X length = %lld\n",
		hfi_mem.sec_heap.kva, hfi_mem.sec_heap.iova,
		hfi_mem.sec_heap.len);

	hfi_mem.shmem.iova = icp_hw_mgr.hfi_mem.shmem.iova_start;
	hfi_mem.shmem.len = icp_hw_mgr.hfi_mem.shmem.iova_len;

	hfi_mem.qdss.iova = icp_hw_mgr.hfi_mem.qdss_buf.iova;
	hfi_mem.qdss.len = icp_hw_mgr.hfi_mem.qdss_buf.len;

	if (icp_hw_mgr.hfi_mem.io_mem.discard_iova_start &&
		icp_hw_mgr.hfi_mem.io_mem.discard_iova_len) {
		/* IO Region 1 */
		hfi_mem.io_mem.iova = icp_hw_mgr.hfi_mem.io_mem.iova_start;
		hfi_mem.io_mem.len =
			icp_hw_mgr.hfi_mem.io_mem.discard_iova_start -
			icp_hw_mgr.hfi_mem.io_mem.iova_start;

		/* IO Region 2 */
		hfi_mem.io_mem2.iova =
			icp_hw_mgr.hfi_mem.io_mem.discard_iova_start +
			icp_hw_mgr.hfi_mem.io_mem.discard_iova_len;
		hfi_mem.io_mem2.len =
			icp_hw_mgr.hfi_mem.io_mem.iova_start +
			icp_hw_mgr.hfi_mem.io_mem.iova_len   -
			hfi_mem.io_mem2.iova;
	} else {
		/* IO Region 1 */
		hfi_mem.io_mem.iova = icp_hw_mgr.hfi_mem.io_mem.iova_start;
		hfi_mem.io_mem.len = icp_hw_mgr.hfi_mem.io_mem.iova_len;

		/* IO Region 2 */
		hfi_mem.io_mem2.iova = 0x0;
		hfi_mem.io_mem2.len = 0x0;
	}

	CAM_DBG(CAM_ICP,
		"IO region1 IOVA = %X length = %lld, IO region2 IOVA = %X length = %lld",
		hfi_mem.io_mem.iova,
		hfi_mem.io_mem.len,
		hfi_mem.io_mem2.iova,
		hfi_mem.io_mem2.len);

	return cam_hfi_resume(&hfi_mem);
}

static int cam_icp_retry_wait_for_abort(
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int retry_cnt = 1;
	unsigned long rem_jiffies;
	int timeout = 1000;

	CAM_WARN(CAM_ICP, "FW timeout in abort ctx: %u retry_left: %d",
		ctx_data->ctx_id, retry_cnt);
	while (retry_cnt > 0) {
		rem_jiffies = wait_for_completion_timeout(
			&ctx_data->wait_complete,
			msecs_to_jiffies((timeout)));
		if (!rem_jiffies) {
			retry_cnt--;
			if (retry_cnt > 0) {
				CAM_WARN(CAM_ICP,
					"FW timeout in abort ctx: %u retry_left: %u",
					ctx_data->ctx_id, retry_cnt);
				continue;
			}
		}

		if (retry_cnt > 0)
			return 0;
	}

	return -ETIMEDOUT;
}

static int cam_icp_mgr_abort_handle_wq(
	void *priv, void *data)
{
	int rc;
	size_t packet_size;
	struct hfi_cmd_work_data   *task_data = NULL;
	struct cam_icp_hw_ctx_data *ctx_data;
	struct hfi_cmd_ipebps_async *abort_cmd;

	if (!data || !priv) {
		CAM_ERR(CAM_ICP, "Invalid params %pK %pK", data, priv);
		return -EINVAL;
	}

	task_data = (struct hfi_cmd_work_data *)data;
	ctx_data =
		(struct cam_icp_hw_ctx_data *)task_data->data;
	packet_size =
		sizeof(struct hfi_cmd_ipebps_async) +
		sizeof(struct hfi_cmd_abort) -
		sizeof(((struct hfi_cmd_ipebps_async *)0)->payload.direct);
	abort_cmd = kzalloc(packet_size, GFP_KERNEL);
	CAM_DBG(CAM_ICP, "abort pkt size = %d", (int) packet_size);
	if (!abort_cmd) {
		rc = -ENOMEM;
		return rc;
	}

	abort_cmd->size = packet_size;
	abort_cmd->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS)
		abort_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_BPS_ABORT;
	else
		abort_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_IPE_ABORT;

	abort_cmd->num_fw_handles = 1;
	abort_cmd->fw_handles_flex[0] = ctx_data->fw_handle;
	abort_cmd->user_data1 = PTR_TO_U64(ctx_data);
	abort_cmd->user_data2 = (uint64_t)0x0;

	rc = hfi_write_cmd(abort_cmd);
	if (rc) {
		kfree(abort_cmd);
		return rc;
	}
	CAM_DBG(CAM_ICP, "fw_handle = %x ctx_data = %pK ctx_id %d",
		ctx_data->fw_handle, ctx_data, ctx_data->ctx_id);

	kfree(abort_cmd);
	return rc;
}

static int cam_icp_mgr_abort_handle(
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0;
	unsigned long rem_jiffies = 0;
	size_t packet_size;
	int timeout = 1000;
	struct hfi_cmd_ipebps_async *abort_cmd;

	packet_size =
		sizeof(struct hfi_cmd_ipebps_async) +
		sizeof(struct hfi_cmd_abort) -
		sizeof(((struct hfi_cmd_ipebps_async *)0)->payload.direct);
	abort_cmd = kzalloc(packet_size, GFP_KERNEL);
	CAM_DBG(CAM_ICP, "abort pkt size = %d", (int) packet_size);
	if (!abort_cmd) {
		rc = -ENOMEM;
		return rc;
	}

	abort_cmd->size = packet_size;
	abort_cmd->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS)
		abort_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_BPS_ABORT;
	else
		abort_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_IPE_ABORT;

	reinit_completion(&ctx_data->wait_complete);
	abort_cmd->num_fw_handles = 1;
	abort_cmd->fw_handles[0] = ctx_data->fw_handle;
	abort_cmd->user_data1 = PTR_TO_U64(ctx_data);
	abort_cmd->user_data2 = (uint64_t)0x0;

	rc = hfi_write_cmd(abort_cmd);
	if (rc) {
		kfree(abort_cmd);
		return rc;
	}
	CAM_DBG(CAM_ICP, "fw_handle = %x ctx_data = %pK",
		ctx_data->fw_handle, ctx_data);
	rem_jiffies = wait_for_completion_timeout(&ctx_data->wait_complete,
			msecs_to_jiffies((timeout)));
	if (!rem_jiffies) {
		rc = cam_icp_retry_wait_for_abort(ctx_data);
		if (rc) {
			CAM_ERR(CAM_ICP,
				"FW timeout/err in abort handle command ctx: %u",
				ctx_data->ctx_id);
			cam_icp_mgr_process_dbg_buf(icp_hw_mgr.icp_dbg_lvl);
			cam_hfi_queue_dump();
		}
	}

	kfree(abort_cmd);
	return rc;
}

static int cam_icp_mgr_destroy_handle(
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0;
	int timeout = 1000;
	unsigned long rem_jiffies;
	size_t packet_size;
	struct hfi_cmd_ipebps_async *destroy_cmd;

	packet_size =
		sizeof(struct hfi_cmd_ipebps_async) +
		sizeof(struct hfi_cmd_abort_destroy) -
		sizeof(((struct hfi_cmd_ipebps_async *)0)->payload.direct);
	destroy_cmd = kzalloc(packet_size, GFP_KERNEL);
	if (!destroy_cmd) {
		rc = -ENOMEM;
		return rc;
	}

	destroy_cmd->size = packet_size;
	destroy_cmd->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS)
		destroy_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_BPS_DESTROY;
	else
		destroy_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_IPE_DESTROY;

	reinit_completion(&ctx_data->wait_complete);
	destroy_cmd->num_fw_handles = 1;
	destroy_cmd->fw_handles_flex[0] = ctx_data->fw_handle;
	destroy_cmd->user_data1 = PTR_TO_U64(ctx_data);
	destroy_cmd->user_data2 = (uint64_t)0x0;

	rc = hfi_write_cmd(destroy_cmd);
	if (rc) {
		kfree(destroy_cmd);
		return rc;
	}
	CAM_DBG(CAM_ICP, "fw_handle = %x ctx_data = %pK",
		ctx_data->fw_handle, ctx_data);
	rem_jiffies = wait_for_completion_timeout(&ctx_data->wait_complete,
			msecs_to_jiffies((timeout)));
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		CAM_ERR(CAM_ICP, "FW response timeout: %d for %u",
			rc, ctx_data->ctx_id);
		cam_icp_mgr_process_dbg_buf(icp_hw_mgr.icp_dbg_lvl);
		cam_hfi_queue_dump();
	}
	kfree(destroy_cmd);
	return rc;
}

static int cam_icp_mgr_release_ctx(struct cam_icp_hw_mgr *hw_mgr, int ctx_id)
{
	int i = 0;

	if (ctx_id >= CAM_ICP_CTX_MAX) {
		CAM_ERR(CAM_ICP, "ctx_id is wrong: %d", ctx_id);
		return -EINVAL;
	}

	mutex_lock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);
	cam_icp_remove_ctx_bw(hw_mgr, &hw_mgr->ctx_data[ctx_id]);
	if (hw_mgr->ctx_data[ctx_id].state !=
		CAM_ICP_CTX_STATE_ACQUIRED) {
		mutex_unlock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);
		CAM_DBG(CAM_ICP,
			"ctx with id: %d not in right state to release: %d",
			ctx_id, hw_mgr->ctx_data[ctx_id].state);
		return 0;
	}
	cam_icp_mgr_ipe_bps_power_collapse(hw_mgr,
		&hw_mgr->ctx_data[ctx_id], 0);
	hw_mgr->ctx_data[ctx_id].state = CAM_ICP_CTX_STATE_RELEASE;
	CAM_DBG(CAM_ICP, "E: ctx_id = %d recovery = %d",
		ctx_id, atomic_read(&hw_mgr->recovery));
	cam_icp_mgr_abort_handle(&hw_mgr->ctx_data[ctx_id]);
	cam_icp_mgr_destroy_handle(&hw_mgr->ctx_data[ctx_id]);
	cam_icp_mgr_cleanup_ctx(&hw_mgr->ctx_data[ctx_id]);

	hw_mgr->ctx_data[ctx_id].fw_handle = 0;
	hw_mgr->ctx_data[ctx_id].scratch_mem_size = 0;
	hw_mgr->ctx_data[ctx_id].last_flush_req = 0;
	for (i = 0; i < CAM_FRAME_CMD_MAX; i++)
		clear_bit(i, hw_mgr->ctx_data[ctx_id].hfi_frame_process.bitmap);
	kfree(hw_mgr->ctx_data[ctx_id].hfi_frame_process.bitmap);
	hw_mgr->ctx_data[ctx_id].hfi_frame_process.bitmap = NULL;
	cam_icp_hw_mgr_clk_info_update(hw_mgr, &hw_mgr->ctx_data[ctx_id]);
	hw_mgr->ctx_data[ctx_id].clk_info.curr_fc = 0;
	hw_mgr->ctx_data[ctx_id].clk_info.base_clk = 0;
	hw_mgr->ctxt_cnt--;
	kfree(hw_mgr->ctx_data[ctx_id].icp_dev_acquire_info);
	hw_mgr->ctx_data[ctx_id].icp_dev_acquire_info = NULL;
	hw_mgr->ctx_data[ctx_id].state = CAM_ICP_CTX_STATE_FREE;
	cam_icp_ctx_timer_stop(&hw_mgr->ctx_data[ctx_id]);
	mutex_unlock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);

	CAM_DBG(CAM_ICP, "X: ctx_id = %d", ctx_id);
	return 0;
}

static void cam_icp_mgr_device_deinit(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = NULL;
	struct cam_hw_intf *ipe0_dev_intf = NULL;
	struct cam_hw_intf *ipe1_dev_intf = NULL;
	struct cam_hw_intf *bps_dev_intf = NULL;

	icp_dev_intf = hw_mgr->icp_dev_intf;
	ipe0_dev_intf = hw_mgr->ipe0_dev_intf;
	ipe1_dev_intf = hw_mgr->ipe1_dev_intf;
	bps_dev_intf = hw_mgr->bps_dev_intf;

	if ((!icp_dev_intf) || (!ipe0_dev_intf) || (!bps_dev_intf)) {
		CAM_ERR(CAM_ICP, "dev intfs are wrong, failed to close");
		return;
	}

	if (ipe1_dev_intf)
		ipe1_dev_intf->hw_ops.deinit(ipe1_dev_intf->hw_priv, NULL, 0);
	ipe0_dev_intf->hw_ops.deinit(ipe0_dev_intf->hw_priv, NULL, 0);
	bps_dev_intf->hw_ops.deinit(bps_dev_intf->hw_priv, NULL, 0);
	icp_dev_intf->hw_ops.deinit(icp_dev_intf->hw_priv, NULL, 0);
	hw_mgr->bps_clk_state = false;
	hw_mgr->ipe_clk_state = false;
}

static int cam_icp_mgr_hw_close(void *hw_priv, void *hw_close_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_priv;
	struct cam_hw_intf *icp_dev_intf = NULL;
	struct cam_icp_set_irq_cb irq_cb;
	struct cam_icp_a5_set_fw_buf_info fw_buf_info;
	int rc = 0;

	CAM_DBG(CAM_ICP, "E");
	if (hw_mgr->fw_download == false) {
		CAM_DBG(CAM_ICP, "hw mgr is already closed");
		return 0;
	}

	icp_dev_intf = hw_mgr->icp_dev_intf;
	if (!icp_dev_intf) {
		CAM_DBG(CAM_ICP, "ICP device interface is NULL");
		return -EINVAL;
	}

	fw_buf_info.kva = 0;
	fw_buf_info.iova = 0;
	fw_buf_info.len = 0;
	rc = icp_dev_intf->hw_ops.process_cmd(
		icp_dev_intf->hw_priv,
		CAM_ICP_CMD_SET_FW_BUF,
		&fw_buf_info,
		sizeof(fw_buf_info));
	if (rc)
		CAM_ERR(CAM_ICP, "nullify the fw buf failed");

	cam_hfi_deinit();

	irq_cb.icp_hw_mgr_cb = NULL;
	irq_cb.data = NULL;
	rc = icp_dev_intf->hw_ops.process_cmd(
		icp_dev_intf->hw_priv,
		CAM_ICP_SET_IRQ_CB,
		&irq_cb, sizeof(irq_cb));
	if (rc)
		CAM_ERR(CAM_ICP, "deregister irq call back failed");

	cam_icp_free_hfi_mem();
	hw_mgr->fw_download = false;

	CAM_DBG(CAM_ICP, "Exit");
	return rc;
}

static int cam_icp_mgr_device_init(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0;
	struct cam_hw_intf *icp_dev_intf = NULL;
	struct cam_hw_intf *ipe0_dev_intf = NULL;
	struct cam_hw_intf *ipe1_dev_intf = NULL;
	struct cam_hw_intf *bps_dev_intf = NULL;

	icp_dev_intf = hw_mgr->icp_dev_intf;
	ipe0_dev_intf = hw_mgr->ipe0_dev_intf;
	ipe1_dev_intf = hw_mgr->ipe1_dev_intf;
	bps_dev_intf = hw_mgr->bps_dev_intf;

	if ((!icp_dev_intf) || (!ipe0_dev_intf) || (!bps_dev_intf)) {
		CAM_ERR(CAM_ICP, "dev intfs are wrong");
		return -EINVAL;
	}

	rc = icp_dev_intf->hw_ops.init(icp_dev_intf->hw_priv, NULL, 0);
	if (rc)
		goto icp_dev_init_failed;

	rc = bps_dev_intf->hw_ops.init(bps_dev_intf->hw_priv, NULL, 0);
	if (rc)
		goto bps_dev_init_failed;

	rc = ipe0_dev_intf->hw_ops.init(ipe0_dev_intf->hw_priv, NULL, 0);
	if (rc)
		goto ipe0_dev_init_failed;

	if (ipe1_dev_intf) {
		rc = ipe1_dev_intf->hw_ops.init(ipe1_dev_intf->hw_priv,
						NULL, 0);
		if (rc)
			goto ipe1_dev_init_failed;
	}

	hw_mgr->bps_clk_state = true;
	hw_mgr->ipe_clk_state = true;

	return rc;
ipe1_dev_init_failed:
	ipe0_dev_intf->hw_ops.deinit(ipe0_dev_intf->hw_priv, NULL, 0);
	hw_mgr->ipe_clk_state = false;
ipe0_dev_init_failed:
	bps_dev_intf->hw_ops.deinit(bps_dev_intf->hw_priv, NULL, 0);
	hw_mgr->bps_clk_state = false;
bps_dev_init_failed:
	icp_dev_intf->hw_ops.deinit(icp_dev_intf->hw_priv, NULL, 0);
icp_dev_init_failed:
	return rc;
}

static int cam_icp_mgr_fw_download(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	struct cam_hw_intf *icp_dev_intf = NULL;
	struct cam_icp_set_irq_cb irq_cb;
	struct cam_icp_a5_set_fw_buf_info fw_buf_info;

	icp_dev_intf = hw_mgr->icp_dev_intf;
	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is invalid");
		return -EINVAL;
	}

	irq_cb.icp_hw_mgr_cb = cam_icp_hw_mgr_cb;
	irq_cb.data = hw_mgr;
	rc = icp_dev_intf->hw_ops.process_cmd(
		icp_dev_intf->hw_priv,
		CAM_ICP_SET_IRQ_CB,
		&irq_cb, sizeof(irq_cb));
	if (rc)
		return rc;

	fw_buf_info.kva = icp_hw_mgr.hfi_mem.fw_buf.kva;
	fw_buf_info.iova = icp_hw_mgr.hfi_mem.fw_buf.iova;
	fw_buf_info.len = icp_hw_mgr.hfi_mem.fw_buf.len;

	rc = icp_dev_intf->hw_ops.process_cmd(
		icp_dev_intf->hw_priv,
		CAM_ICP_CMD_SET_FW_BUF,
		&fw_buf_info, sizeof(fw_buf_info));
	if (rc)
		return rc;

	rc = icp_dev_intf->hw_ops.process_cmd(
		icp_dev_intf->hw_priv,
		CAM_ICP_CMD_FW_DOWNLOAD,
		NULL, 0);
	if (rc)
		return rc;

	return cam_icp_mgr_proc_resume(hw_mgr);
}

static int cam_icp_mgr_hfi_init(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = NULL;
	struct cam_hw_info *icp_dev = NULL;
	struct hfi_mem_info hfi_mem;
	const struct hfi_ops *hfi_ops;

	icp_dev_intf = hw_mgr->icp_dev_intf;
	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is invalid");
		return -EINVAL;
	}
	icp_dev = (struct cam_hw_info *)icp_dev_intf->hw_priv;

	hfi_mem.qtbl.kva = icp_hw_mgr.hfi_mem.qtbl.kva;
	hfi_mem.qtbl.iova = icp_hw_mgr.hfi_mem.qtbl.iova;
	hfi_mem.qtbl.len = icp_hw_mgr.hfi_mem.qtbl.len;

	hfi_mem.cmd_q.kva = icp_hw_mgr.hfi_mem.cmd_q.kva;
	hfi_mem.cmd_q.iova = icp_hw_mgr.hfi_mem.cmd_q.iova;
	hfi_mem.cmd_q.len = icp_hw_mgr.hfi_mem.cmd_q.len;

	hfi_mem.msg_q.kva = icp_hw_mgr.hfi_mem.msg_q.kva;
	hfi_mem.msg_q.iova = icp_hw_mgr.hfi_mem.msg_q.iova;
	hfi_mem.msg_q.len = icp_hw_mgr.hfi_mem.msg_q.len;

	hfi_mem.dbg_q.kva = icp_hw_mgr.hfi_mem.dbg_q.kva;
	hfi_mem.dbg_q.iova = icp_hw_mgr.hfi_mem.dbg_q.iova;
	hfi_mem.dbg_q.len = icp_hw_mgr.hfi_mem.dbg_q.len;

	hfi_mem.sfr_buf.kva = icp_hw_mgr.hfi_mem.sfr_buf.kva;
	hfi_mem.sfr_buf.iova = icp_hw_mgr.hfi_mem.sfr_buf.iova;
	hfi_mem.sfr_buf.len = icp_hw_mgr.hfi_mem.sfr_buf.len;

	hfi_mem.sec_heap.kva = icp_hw_mgr.hfi_mem.sec_heap.kva;
	hfi_mem.sec_heap.iova = icp_hw_mgr.hfi_mem.sec_heap.iova;
	hfi_mem.sec_heap.len = icp_hw_mgr.hfi_mem.sec_heap.len;

	hfi_mem.shmem.iova = icp_hw_mgr.hfi_mem.shmem.iova_start;
	hfi_mem.shmem.len = icp_hw_mgr.hfi_mem.shmem.iova_len;

	hfi_mem.qdss.iova = icp_hw_mgr.hfi_mem.qdss_buf.iova;
	hfi_mem.qdss.len = icp_hw_mgr.hfi_mem.qdss_buf.len;

	if (icp_hw_mgr.hfi_mem.io_mem.discard_iova_start &&
		icp_hw_mgr.hfi_mem.io_mem.discard_iova_len) {
		/* IO Region 1 */
		hfi_mem.io_mem.iova = icp_hw_mgr.hfi_mem.io_mem.iova_start;
		hfi_mem.io_mem.len =
			icp_hw_mgr.hfi_mem.io_mem.discard_iova_start -
			icp_hw_mgr.hfi_mem.io_mem.iova_start;

		/* IO Region 2 */
		hfi_mem.io_mem2.iova =
			icp_hw_mgr.hfi_mem.io_mem.discard_iova_start +
			icp_hw_mgr.hfi_mem.io_mem.discard_iova_len;
		hfi_mem.io_mem2.len =
			icp_hw_mgr.hfi_mem.io_mem.iova_start +
			icp_hw_mgr.hfi_mem.io_mem.iova_len   -
			hfi_mem.io_mem2.iova;
	} else {
		/* IO Region 1 */
		hfi_mem.io_mem.iova = icp_hw_mgr.hfi_mem.io_mem.iova_start;
		hfi_mem.io_mem.len = icp_hw_mgr.hfi_mem.io_mem.iova_len;

		/* IO Region 2 */
		hfi_mem.io_mem2.iova = 0x0;
		hfi_mem.io_mem2.len = 0x0;
	}

	hfi_ops = &hfi_a5_ops;

	return cam_hfi_init(&hfi_mem, hfi_ops, icp_dev, 0);
}

static int cam_icp_mgr_send_fw_init(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	unsigned long rem_jiffies;
	int timeout = 5000;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is NULL");
		return -EINVAL;
	}

	reinit_completion(&hw_mgr->icp_complete);
	CAM_DBG(CAM_ICP, "Sending HFI init command");
	rc = icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv,
					CAM_ICP_SEND_INIT, NULL, 0);
	if (rc)
		return rc;

	rem_jiffies = wait_for_completion_timeout(&icp_hw_mgr.icp_complete,
		msecs_to_jiffies((timeout)));
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		CAM_ERR(CAM_ICP, "FW response timed out %d", rc);
		cam_icp_mgr_process_dbg_buf(icp_hw_mgr.icp_dbg_lvl);
		cam_hfi_queue_dump();
	}
	CAM_DBG(CAM_ICP, "Done Waiting for INIT DONE Message");

	return rc;
}

static int cam_icp_mgr_hw_open_u(void *hw_mgr_priv, void *download_fw_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	int rc = 0;

	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "Null hw mgr");
		return 0;
	}

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	rc = cam_icp_mgr_hw_open(hw_mgr, download_fw_args);
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	return rc;
}

static int cam_icp_mgr_hw_open_k(void *hw_mgr_priv, void *download_fw_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;

	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "Null hw mgr");
		return 0;
	}

	return cam_icp_mgr_hw_open(hw_mgr, download_fw_args);
}

static int cam_icp_mgr_icp_resume(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0;
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	bool downloadFromResume = true;

	CAM_DBG(CAM_ICP, "Enter");

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is NULL");
		return -EINVAL;
	}

	if (hw_mgr->fw_download  == false) {
		CAM_DBG(CAM_ICP, "Downloading FW");
		rc = cam_icp_mgr_hw_open_k(hw_mgr, &downloadFromResume);
		CAM_DBG(CAM_ICP, "FW Download Done Exit");
		return rc;
	}

	rc = icp_dev_intf->hw_ops.init(icp_dev_intf->hw_priv, NULL, 0);
	if (rc)
		return -EINVAL;

	rc = cam_icp_mgr_proc_resume(hw_mgr);
	if (rc)
		goto hw_deinit;

	rc = cam_icp_mgr_hfi_resume(hw_mgr);
	if (rc)
		goto power_collapse;

	CAM_DBG(CAM_ICP, "Exit");
	return rc;

power_collapse:
	__power_collapse(hw_mgr);
hw_deinit:
	icp_dev_intf->hw_ops.deinit(icp_dev_intf->hw_priv, NULL, 0);

	return rc;
}

static int cam_icp_mgr_hw_open(void *hw_mgr_priv, void *download_fw_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	bool icp_pc = false;
	int rc = 0;

	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "hw_mgr is NULL");
		return -EINVAL;
	}

	if (hw_mgr->fw_download) {
		CAM_DBG(CAM_ICP, "FW already downloaded");
		return rc;
	}

	if (!hw_mgr->icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is invalid");
		return -EINVAL;
	}

	rc = cam_icp_allocate_hfi_mem();
	if (rc)
		goto alloc_hfi_mem_failed;

	rc = cam_icp_mgr_device_init(hw_mgr);
	if (rc)
		goto dev_init_fail;

	rc = cam_icp_mgr_fw_download(hw_mgr);
	if (rc)
		goto fw_download_failed;

	rc = cam_icp_mgr_hfi_init(hw_mgr);
	if (rc)
		goto hfi_init_failed;

	rc = cam_icp_mgr_send_fw_init(hw_mgr);
	if (rc)
		goto fw_init_failed;

	hw_mgr->ctxt_cnt = 0;
	hw_mgr->fw_download = true;
	atomic_set(&hw_mgr->recovery, 0);

	CAM_INFO(CAM_ICP, "FW download done successfully");

	rc = cam_ipe_bps_deint(hw_mgr);
	if (download_fw_args)
		icp_pc = *((bool *)download_fw_args);

	if (download_fw_args && icp_pc == true && hw_mgr->icp_pc_flag) {
		rc = cam_ipe_bps_deint(hw_mgr);
		CAM_DBG(CAM_ICP, "deinit all clocks");
	}

	if (download_fw_args && icp_pc == true)
		return rc;

	rc = cam_ipe_bps_deint(hw_mgr);
	rc = cam_icp_mgr_icp_power_collapse(hw_mgr);
	CAM_DBG(CAM_ICP, "deinit all clocks at boot up");

	return rc;

fw_init_failed:
	cam_hfi_deinit();
hfi_init_failed:
	cam_icp_mgr_proc_suspend(hw_mgr);
fw_download_failed:
	cam_icp_mgr_device_deinit(hw_mgr);
dev_init_fail:
	cam_icp_free_hfi_mem();
alloc_hfi_mem_failed:
	return rc;
}

static int cam_icp_mgr_handle_config_err(
	struct cam_hw_config_args *config_args,
	struct cam_icp_hw_ctx_data *ctx_data,
	int idx)
{
	struct cam_hw_done_event_data buf_data;

	buf_data.request_id = *(uint64_t *)config_args->priv;
	buf_data.evt_param = CAM_SYNC_ICP_EVENT_CONFIG_ERR;
	ctx_data->ctxt_event_cb(ctx_data->context_priv, CAM_CTX_EVT_ID_SUCCESS,
		&buf_data);

	ctx_data->hfi_frame_process.request_id[idx] = 0;
	ctx_data->hfi_frame_process.fw_process_flag[idx] = false;
	clear_bit(idx, ctx_data->hfi_frame_process.bitmap);

	return 0;
}

static int cam_icp_mgr_enqueue_config(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_hw_config_args *config_args)
{
	int rc = 0;
	uint64_t request_id = 0;
	struct crm_workq_task *task;
	struct hfi_cmd_work_data *task_data;
	struct hfi_cmd_ipebps_async *hfi_cmd;
	struct cam_hw_update_entry *hw_update_entries;
	struct icp_frame_info *frame_info = NULL;

	frame_info = (struct icp_frame_info *)config_args->priv;
	request_id = frame_info->request_id;
	hw_update_entries = config_args->hw_update_entries;
	CAM_DBG(CAM_ICP, "req_id = %lld %pK", request_id, config_args->priv);

	task = cam_req_mgr_workq_get_task(icp_hw_mgr.cmd_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "no empty task");
		return -ENOMEM;
	}

	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)hw_update_entries->addr;
	hfi_cmd = (struct hfi_cmd_ipebps_async *)hw_update_entries->addr;
	task_data->request_id = request_id;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_process_cmd;
	rc = cam_req_mgr_workq_enqueue_task(task, &icp_hw_mgr,
		CRM_TASK_PRIORITY_0);

	return rc;
}

static int cam_icp_mgr_send_config_io(struct cam_icp_hw_ctx_data *ctx_data,
	uint32_t io_buf_addr)
{
	int rc = 0;
	struct hfi_cmd_work_data *task_data;
	struct hfi_cmd_ipebps_async ioconfig_cmd;
	unsigned long rem_jiffies;
	int timeout = 5000;
	struct crm_workq_task *task;
	uint32_t size_in_words;

	task = cam_req_mgr_workq_get_task(icp_hw_mgr.cmd_work);
	if (!task) {
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"No free task ctx id:%d dev hdl:0x%x session hdl:0x%x dev_type:%d",
			ctx_data->ctx_id, ctx_data->acquire_dev_cmd.dev_handle,
			ctx_data->acquire_dev_cmd.session_handle,
			ctx_data->icp_dev_acquire_info->dev_type);
		return -ENOMEM;
	}

	ioconfig_cmd.size = sizeof(struct hfi_cmd_ipebps_async);
	ioconfig_cmd.pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT;
	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS)
		ioconfig_cmd.opcode = HFI_IPEBPS_CMD_OPCODE_BPS_CONFIG_IO;
	else
		ioconfig_cmd.opcode = HFI_IPEBPS_CMD_OPCODE_IPE_CONFIG_IO;

	reinit_completion(&ctx_data->wait_complete);

	ioconfig_cmd.num_fw_handles = 1;
	ioconfig_cmd.fw_handles_flex[0] = ctx_data->fw_handle;
	ioconfig_cmd.payload.indirect = io_buf_addr;
	ioconfig_cmd.user_data1 = PTR_TO_U64(ctx_data);
	ioconfig_cmd.user_data2 = (uint64_t)0x0;
	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)&ioconfig_cmd;
	task_data->request_id = 0;
	task_data->type = ICP_WORKQ_TASK_MSG_TYPE;
	task->process_cb = cam_icp_mgr_process_cmd;
	size_in_words = (*(uint32_t *)task_data->data) >> 2;
	CAM_DBG(CAM_ICP, "size_in_words %u", size_in_words);
	rc = cam_req_mgr_workq_enqueue_task(task, &icp_hw_mgr,
		CRM_TASK_PRIORITY_0);
	if (rc) {
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"Enqueue task failed ctx id:%d dev hdl:0x%x session hdl:0x%x dev_type:%d",
			ctx_data->ctx_id, ctx_data->acquire_dev_cmd.dev_handle,
			ctx_data->acquire_dev_cmd.session_handle,
			ctx_data->icp_dev_acquire_info->dev_type);
		return rc;
	}

	rem_jiffies = wait_for_completion_timeout(&ctx_data->wait_complete,
		msecs_to_jiffies((timeout)));
	if (!rem_jiffies) {
		/* send specific error for io config failure */
		rc = -EREMOTEIO;
		CAM_ERR(CAM_ICP,
			"FW response timed out %d ctx id:%d dev hdl:0x%x session hdl:0x%x dev_type:%d",
			rc,
			ctx_data->ctx_id, ctx_data->acquire_dev_cmd.dev_handle,
			ctx_data->acquire_dev_cmd.session_handle,
			ctx_data->icp_dev_acquire_info->dev_type);
		cam_icp_mgr_process_dbg_buf(icp_hw_mgr.icp_dbg_lvl);
		cam_hfi_queue_dump();
	}

	return rc;
}

static int cam_icp_mgr_send_recfg_io(struct cam_icp_hw_ctx_data *ctx_data,
	struct hfi_cmd_ipebps_async *ioconfig_cmd, uint64_t req_id)
{
	int rc = 0;
	struct hfi_cmd_work_data *task_data;
	struct crm_workq_task *task;

	task = cam_req_mgr_workq_get_task(icp_hw_mgr.cmd_work);
	if (!task)
		return -ENOMEM;

	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)ioconfig_cmd;
	task_data->request_id = req_id;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_process_cmd;
	rc = cam_req_mgr_workq_enqueue_task(task, &icp_hw_mgr,
		CRM_TASK_PRIORITY_0);
	if (rc)
		return rc;

	return rc;
}

static int cam_icp_mgr_config_hw(void *hw_mgr_priv, void *config_hw_args)
{
	int rc = 0;
	int idx;
	uint64_t req_id;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_hw_config_args *config_args = config_hw_args;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct icp_frame_info *frame_info = NULL;

	if (!hw_mgr || !config_args) {
		CAM_ERR(CAM_ICP, "Invalid arguments %pK %pK",
			hw_mgr, config_args);
		return -EINVAL;
	}

	if (!config_args->num_hw_update_entries) {
		CAM_ERR(CAM_ICP, "No hw update enteries are available");
		return -EINVAL;
	}

	ctx_data = config_args->ctxt_to_hw_map;
	mutex_lock(&hw_mgr->hw_mgr_mutex);
	mutex_lock(&ctx_data->ctx_mutex);
	if (ctx_data->state != CAM_ICP_CTX_STATE_ACQUIRED) {
		mutex_unlock(&ctx_data->ctx_mutex);
		mutex_unlock(&hw_mgr->hw_mgr_mutex);
		CAM_ERR(CAM_ICP, "ctx id :%u is not in use",
			ctx_data->ctx_id);
		return -EINVAL;
	}

	frame_info = (struct icp_frame_info *)config_args->priv;
	req_id = frame_info->request_id;
	idx = cam_icp_clk_idx_from_req_id(ctx_data, req_id);

	cam_icp_mgr_ipe_bps_clk_update(hw_mgr, ctx_data, idx);
	ctx_data->hfi_frame_process.fw_process_flag[idx] = true;
	ctx_data->hfi_frame_process.submit_timestamp[idx] = ktime_get();

	CAM_DBG(CAM_ICP, "req_id %llu, io config %llu", req_id,
		frame_info->io_config);

	if (frame_info->io_config != 0) {
		CAM_INFO(CAM_ICP, "Send recfg io");
		rc = cam_icp_mgr_send_recfg_io(ctx_data,
			&frame_info->hfi_cfg_io_cmd, req_id);
		if (rc)
			CAM_ERR(CAM_ICP, "Fail to send reconfig io cmd");
	}

	if (req_id <= ctx_data->last_flush_req)
		CAM_WARN(CAM_ICP,
			"Anomaly submitting flushed req %llu [last_flush %llu] in ctx %u",
			req_id, ctx_data->last_flush_req, ctx_data->ctx_id);

	rc = cam_icp_mgr_enqueue_config(hw_mgr, config_args);
	if (rc)
		goto config_err;
	CAM_DBG(CAM_REQ,
		"req_id = %lld on ctx_id %u for dev %d queued to FW",
		req_id, ctx_data->ctx_id,
		ctx_data->icp_dev_acquire_info->dev_type);
	mutex_unlock(&ctx_data->ctx_mutex);
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	return 0;
config_err:
	cam_icp_mgr_handle_config_err(config_args, ctx_data, idx);
	mutex_unlock(&ctx_data->ctx_mutex);
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	return rc;
}

static int cam_icp_mgr_prepare_frame_process_cmd(
	struct cam_icp_hw_ctx_data *ctx_data,
	struct hfi_cmd_ipebps_async *hfi_cmd,
	uint64_t request_id,
	uint32_t fw_cmd_buf_iova_addr)
{
	hfi_cmd->size = sizeof(struct hfi_cmd_ipebps_async);
	hfi_cmd->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT;
	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS)
		hfi_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_BPS_FRAME_PROCESS;
	else
		hfi_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_IPE_FRAME_PROCESS;
	hfi_cmd->num_fw_handles = 1;
	hfi_cmd->fw_handles_flex[0] = ctx_data->fw_handle;
	hfi_cmd->payload.indirect = fw_cmd_buf_iova_addr;
	hfi_cmd->user_data1 = PTR_TO_U64(ctx_data);
	hfi_cmd->user_data2 = request_id;

	CAM_DBG(CAM_ICP, "ctx_data : %pK, request_id :%lld cmd_buf %x",
		(void *)ctx_data->context_priv, request_id,
		fw_cmd_buf_iova_addr);

	return 0;
}

static bool cam_icp_mgr_is_valid_inconfig(struct cam_packet *packet)
{
	int i, num_in_map_entries = 0;
	bool in_config_valid = false;
	struct cam_buf_io_cfg *io_cfg_ptr = NULL;

	io_cfg_ptr = (struct cam_buf_io_cfg *) ((uint32_t *) &packet->payload_flex +
					packet->io_configs_offset/4);

	for (i = 0 ; i < packet->num_io_configs; i++)
		if (io_cfg_ptr[i].direction == CAM_BUF_INPUT)
			num_in_map_entries++;

	if (num_in_map_entries <= CAM_MAX_IN_RES) {
		in_config_valid = true;
	} else {
		CAM_ERR(CAM_ICP, "In config entries(%u) more than allowed(%u)",
				num_in_map_entries, CAM_MAX_IN_RES);
	}

	CAM_DBG(CAM_ICP, "number of in_config info: %u %u %u %u",
			packet->num_io_configs, IPE_IO_IMAGES_MAX,
			num_in_map_entries, CAM_MAX_IN_RES);

	return in_config_valid;
}

static bool cam_icp_mgr_is_valid_outconfig(struct cam_packet *packet)
{
	int i, num_out_map_entries = 0;
	bool out_config_valid = false;
	struct cam_buf_io_cfg *io_cfg_ptr = NULL;

	io_cfg_ptr = (struct cam_buf_io_cfg *) ((uint32_t *) &packet->payload_flex +
					packet->io_configs_offset/4);

	for (i = 0 ; i < packet->num_io_configs; i++)
	    if ((io_cfg_ptr[i].direction == CAM_BUF_OUTPUT) ||
		(io_cfg_ptr[i].direction == CAM_BUF_IN_OUT))
			num_out_map_entries++;

	if (num_out_map_entries <= CAM_MAX_OUT_RES) {
		out_config_valid = true;
	} else {
		CAM_ERR(CAM_ICP, "Out config entries(%u) more than allowed(%u)",
				num_out_map_entries, CAM_MAX_OUT_RES);
	}

	CAM_DBG(CAM_ICP, "number of out_config info: %u %u %u %u",
			packet->num_io_configs, IPE_IO_IMAGES_MAX,
			num_out_map_entries, CAM_MAX_OUT_RES);

	return out_config_valid;
}

static int cam_icp_mgr_pkt_validation(struct cam_packet *packet)
{
	if (((packet->header.op_code & 0xff) !=
		CAM_ICP_OPCODE_IPE_UPDATE) &&
		((packet->header.op_code & 0xff) !=
		CAM_ICP_OPCODE_BPS_UPDATE)) {
		CAM_ERR(CAM_ICP, "Invalid Opcode in pkt: %d",
			packet->header.op_code & 0xff);
		return -EINVAL;
	}

	if (!packet->num_io_configs || packet->num_io_configs > IPE_IO_IMAGES_MAX) {
		CAM_ERR(CAM_ICP, "Invalid number of io configs: %d %d",
			IPE_IO_IMAGES_MAX, packet->num_io_configs);
		return -EINVAL;
	}

	if (!packet->num_cmd_buf || packet->num_cmd_buf > CAM_ICP_CTX_MAX_CMD_BUFFERS) {
		CAM_ERR(CAM_ICP, "Invalid number of cmd buffers: %d %d",
			CAM_ICP_CTX_MAX_CMD_BUFFERS, packet->num_cmd_buf);
		return -EINVAL;
	}

	if (!cam_icp_mgr_is_valid_inconfig(packet) ||
		!cam_icp_mgr_is_valid_outconfig(packet)) {
		return -EINVAL;
	}

	CAM_DBG(CAM_ICP, "number of cmd/patch info: %u %u %u %u",
			packet->num_cmd_buf,
			packet->num_io_configs, IPE_IO_IMAGES_MAX,
			packet->num_patches);
	return 0;
}

static int cam_icp_mgr_process_cmd_desc(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_packet *packet, struct cam_icp_hw_ctx_data *ctx_data,
	uint32_t *fw_cmd_buf_iova_addr)
{
	int rc = 0;
	int i;
	int num_cmd_buf = 0;
	dma_addr_t addr;
	size_t len;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	uintptr_t cpu_addr = 0;

	cmd_desc = (struct cam_cmd_buf_desc *)
		((uint32_t *) &packet->payload_flex + packet->cmd_buf_offset/4);
	rc = cam_packet_util_validate_cmd_desc(cmd_desc);
	if (rc)
		return rc;

	*fw_cmd_buf_iova_addr = 0;
	for (i = 0; i < packet->num_cmd_buf; i++, num_cmd_buf++) {
		if (cmd_desc[i].type == CAM_CMD_BUF_FW) {
			rc = cam_mem_get_io_buf(cmd_desc[i].mem_handle,
				hw_mgr->iommu_hdl, &addr, &len);
			if (rc) {
				CAM_ERR(CAM_ICP, "get cmd buf failed %x",
					hw_mgr->iommu_hdl);

				if (num_cmd_buf > 0)
					num_cmd_buf--;
				return rc;
			}
			*fw_cmd_buf_iova_addr = addr;

			if ((cmd_desc[i].offset >= len) ||
				((len - cmd_desc[i].offset) <
				cmd_desc[i].size)){
				CAM_ERR(CAM_ICP,
					"Invalid offset, i: %d offset: %u len: %zu size: %zu",
					i, cmd_desc[i].offset,
					len, cmd_desc[i].size);
				return -EINVAL;
			}

			*fw_cmd_buf_iova_addr =
				(*fw_cmd_buf_iova_addr + cmd_desc[i].offset);

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&cpu_addr, &len);
			if (rc || !cpu_addr) {
				CAM_ERR(CAM_ICP, "get cmd buf failed %x",
					hw_mgr->iommu_hdl);
				*fw_cmd_buf_iova_addr = 0;

				if (num_cmd_buf > 0)
					num_cmd_buf--;
				return rc;
			}
			if ((len <= cmd_desc[i].offset) ||
				(cmd_desc[i].size < cmd_desc[i].length) ||
				((len - cmd_desc[i].offset) <
				cmd_desc[i].length)) {
				CAM_ERR(CAM_ICP, "Invalid offset or length");
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				return -EINVAL;
			}
			cpu_addr = cpu_addr + cmd_desc[i].offset;

			cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
		}
	}

	if (!cpu_addr) {
		CAM_ERR(CAM_ICP, "invalid number of cmd buf");
		return -EINVAL;
	}

	return rc;
}

static int cam_icp_mgr_process_io_cfg(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_packet *packet,
	struct cam_hw_prepare_update_args *prepare_args,
	int32_t index)
{
	int i, j, k, rc = 0;
	struct cam_buf_io_cfg *io_cfg_ptr = NULL;
	int32_t sync_in_obj[CAM_MAX_IN_RES];
	int32_t merged_sync_in_obj;

	io_cfg_ptr = (struct cam_buf_io_cfg *) ((uint32_t *) &packet->payload_flex +
				packet->io_configs_offset/4);
	prepare_args->num_out_map_entries = 0;
	prepare_args->num_in_map_entries = 0;

	for (i = 0, j = 0, k = 0; i < packet->num_io_configs; i++) {
		if (io_cfg_ptr[i].direction == CAM_BUF_INPUT) {
			sync_in_obj[j++] = io_cfg_ptr[i].fence;
			prepare_args->num_in_map_entries++;
		} else if ((io_cfg_ptr[i].direction == CAM_BUF_OUTPUT) ||
		    (io_cfg_ptr[i].direction == CAM_BUF_IN_OUT)) {
			prepare_args->out_map_entries[k++].sync_id =
				io_cfg_ptr[i].fence;
			prepare_args->num_out_map_entries++;
		} else {
		    CAM_ERR(CAM_ICP, "dir: %d, max_out:%u, out %u",
			io_cfg_ptr[i].direction,
			prepare_args->max_out_map_entries,
			prepare_args->num_out_map_entries);
		return -EINVAL;
		}
		CAM_DBG(CAM_REQ,
			"ctx_id: %u req_id: %llu dir[%d]: %u, fence: %u resource_type = %u memh %x",
			ctx_data->ctx_id, packet->header.request_id, i,
			io_cfg_ptr[i].direction, io_cfg_ptr[i].fence,
			io_cfg_ptr[i].resource_type,
			io_cfg_ptr[i].mem_handle[0]);
	}

	if (prepare_args->num_in_map_entries > 1)
		prepare_args->num_in_map_entries =
			cam_common_util_remove_duplicate_arr(
			sync_in_obj, prepare_args->num_in_map_entries);

	if (prepare_args->num_in_map_entries > 1) {
		rc = cam_sync_merge(&sync_in_obj[0],
			prepare_args->num_in_map_entries, &merged_sync_in_obj);
		if (rc) {
			prepare_args->num_out_map_entries = 0;
			prepare_args->num_in_map_entries = 0;
			return rc;
		}

		ctx_data->hfi_frame_process.in_resource[index] =
			merged_sync_in_obj;
		prepare_args->in_map_entries[0].sync_id = merged_sync_in_obj;
		prepare_args->num_in_map_entries = 1;
		CAM_DBG(CAM_REQ, "ctx_id: %u req_id: %llu Merged Sync obj: %d",
			ctx_data->ctx_id, packet->header.request_id,
			merged_sync_in_obj);
	} else if (prepare_args->num_in_map_entries == 1) {
		prepare_args->in_map_entries[0].sync_id = sync_in_obj[0];
		prepare_args->num_in_map_entries = 1;
		ctx_data->hfi_frame_process.in_resource[index] = 0;
	} else {
		CAM_ERR(CAM_ICP, "No input fences");
		prepare_args->num_in_map_entries = 0;
		ctx_data->hfi_frame_process.in_resource[index] = 0;
		rc = -EINVAL;
	}

	return rc;
}

static int cam_icp_process_stream_settings(
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_cmd_mem_regions *cmd_mem_regions,
	bool map_unmap)
{
	int rc = 0, i = 0;
	size_t packet_size, map_cmd_size, len;
	dma_addr_t iova;
	unsigned long rem_jiffies;
	int timeout = 5000;
	struct hfi_cmd_ipe_bps_map  *map_cmd;
	struct hfi_cmd_ipebps_async *async_direct;

	map_cmd_size =
		sizeof(struct hfi_cmd_ipe_bps_map) +
		((cmd_mem_regions->num_regions - 1) *
		sizeof(struct mem_map_region_data));

	map_cmd = kzalloc(map_cmd_size, GFP_KERNEL);
	if (!map_cmd)
		return -ENOMEM;

	for (i = 0; i < cmd_mem_regions->num_regions; i++) {
		rc = cam_mem_get_io_buf(
			cmd_mem_regions->map_info_array_flex[i].mem_handle,
			icp_hw_mgr.iommu_hdl, &iova, &len);
		if (rc) {
			CAM_ERR(CAM_ICP,
				"Failed to get cmd region iova for handle %u",
				cmd_mem_regions->map_info_array_flex[i].mem_handle);
			kfree(map_cmd);
			return -EINVAL;
		}

		map_cmd->mem_map_region_sets_flex[i].start_addr = (uint32_t)iova +
			(cmd_mem_regions->map_info_array_flex[i].offset);
		map_cmd->mem_map_region_sets_flex[i].len = (uint32_t) len;

		CAM_DBG(CAM_ICP, "Region %u mem_handle %d iova %pK len %u",
			(i+1), cmd_mem_regions->map_info_array_flex[i].mem_handle,
			(uint32_t)iova, (uint32_t)len);
	}

	map_cmd->mem_map_request_num = cmd_mem_regions->num_regions;
	map_cmd->user_data = 0;

	packet_size =
		sizeof(struct hfi_cmd_ipebps_async) +
		(sizeof(struct hfi_cmd_ipe_bps_map) +
		((cmd_mem_regions->num_regions - 1) *
		sizeof(struct mem_map_region_data))) -
		sizeof(((struct hfi_cmd_ipebps_async *)0)->payload.direct);

	async_direct = kzalloc(packet_size, GFP_KERNEL);
	if (!async_direct) {
		kfree(map_cmd);
		return -ENOMEM;
	}

	async_direct->size = packet_size;
	async_direct->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
	if (map_unmap)
		async_direct->opcode = HFI_IPEBPS_CMD_OPCODE_MEM_MAP;
	else
		async_direct->opcode = HFI_IPEBPS_CMD_OPCODE_MEM_UNMAP;
	async_direct->num_fw_handles = 1;
	async_direct->fw_handles_flex[0] = ctx_data->fw_handle;
	async_direct->user_data1 = (uint64_t)ctx_data;
	async_direct->user_data2 = (uint64_t)0x0;
	memcpy(async_direct->payload.direct_flex, map_cmd,
		map_cmd_size);

	reinit_completion(&ctx_data->wait_complete);
	rc = hfi_write_cmd(async_direct);
	if (rc) {
		CAM_ERR(CAM_ICP, "hfi write failed  rc %d", rc);
		goto end;
	}

	CAM_DBG(CAM_ICP, "Sent FW %s cmd",
		map_unmap ? "Map" : "Unmap");

	rem_jiffies = wait_for_completion_timeout(&ctx_data->wait_complete,
		msecs_to_jiffies((timeout)));
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		CAM_ERR(CAM_ICP, "FW response timed out %d", rc);
		cam_hfi_queue_dump();
	}

end:
	kfree(map_cmd);
	kfree(async_direct);
	return rc;
}

static int cam_icp_packet_generic_blob_handler(void *user_data,
	uint32_t blob_type, uint32_t blob_size, uint8_t *blob_data)
{
	struct cam_icp_clk_bw_request *soc_req;
	struct cam_icp_clk_bw_request *clk_info;
	struct cam_icp_clk_bw_request_v2 *soc_req_v2;
	struct cam_icp_clk_bw_req_internal_v2 *clk_info_v2;
	struct cam_cmd_mem_regions *cmd_mem_regions;
	struct icp_cmd_generic_blob *blob;
	struct cam_icp_hw_ctx_data *ctx_data;
	uint32_t index;
	size_t io_buf_size, clk_update_size;
	int rc = 0;
	uintptr_t pResource;
	uint32_t i = 0;

	if (!blob_data || (blob_size == 0)) {
		CAM_ERR(CAM_ICP, "Invalid blob info %pK %d", blob_data,
			blob_size);
		return -EINVAL;
	}

	blob = (struct icp_cmd_generic_blob *)user_data;
	ctx_data = blob->ctx;
	index = blob->frame_info_idx;

	switch (blob_type) {
	case CAM_ICP_CMD_GENERIC_BLOB_CLK:
		CAM_WARN_RATE_LIMIT_CUSTOM(CAM_PERF, 300, 1,
			"Using deprecated blob type GENERIC_BLOB_CLK");
		if (blob_size != sizeof(struct cam_icp_clk_bw_request)) {
			CAM_ERR(CAM_ICP, "Mismatch blob size %d expected %lu",
				blob_size,
				sizeof(struct cam_icp_clk_bw_request));
			return -EINVAL;
		}

		if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_UNKNOWN) {
			ctx_data->bw_config_version = CAM_ICP_BW_CONFIG_V1;
		} else if (ctx_data->bw_config_version !=
			CAM_ICP_BW_CONFIG_V1) {
			CAM_ERR(CAM_ICP,
				"Mismatch blob versions %d expected v1 %d, blob_type=%d",
				ctx_data->bw_config_version,
				CAM_ICP_BW_CONFIG_V1, blob_type);
			return -EINVAL;
		}

		clk_info = &ctx_data->hfi_frame_process.clk_info[index];

		soc_req = (struct cam_icp_clk_bw_request *)blob_data;
		*clk_info = *soc_req;
		CAM_DBG(CAM_PERF, "budget:%llu fc: %llu %d BW %lld %lld",
			clk_info->budget_ns, clk_info->frame_cycles,
			clk_info->rt_flag, clk_info->uncompressed_bw,
			clk_info->compressed_bw);
		break;

	case CAM_ICP_CMD_GENERIC_BLOB_CLK_V2:
		if (blob_size < sizeof(struct cam_icp_clk_bw_request_v2)) {
			CAM_ERR(CAM_ICP, "Mismatch blob size %d expected %lu",
				blob_size,
				sizeof(struct cam_icp_clk_bw_request_v2));
			return -EINVAL;
		}

		if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_UNKNOWN) {
			ctx_data->bw_config_version = CAM_ICP_BW_CONFIG_V2;
		} else if (ctx_data->bw_config_version !=
			CAM_ICP_BW_CONFIG_V2) {
			CAM_ERR(CAM_ICP,
				"Mismatch blob versions %d expected v2 %d, blob_type=%d",
				ctx_data->bw_config_version,
				CAM_ICP_BW_CONFIG_V2, blob_type);
			return -EINVAL;
		}

		soc_req_v2 = (struct cam_icp_clk_bw_request_v2 *)blob_data;
		if (soc_req_v2->num_paths > CAM_ICP_MAX_PER_PATH_VOTES) {
			CAM_ERR(CAM_PERF, "Invalid num paths: %d",
				soc_req_v2->num_paths);
			return -EINVAL;
		}

		/* Check for integer overflow */
		if (soc_req_v2->num_paths != 1) {
			if (sizeof(struct cam_axi_per_path_bw_vote) >
				((UINT_MAX -
				sizeof(struct cam_icp_clk_bw_request_v2)) /
				(soc_req_v2->num_paths - 1))) {
				CAM_ERR(CAM_ICP,
					"Size exceeds limit paths:%u size per path:%lu",
					soc_req_v2->num_paths - 1,
					sizeof(
					struct cam_axi_per_path_bw_vote));
				return -EINVAL;
			}
		}

		clk_update_size = sizeof(struct cam_icp_clk_bw_request_v2) +
			((soc_req_v2->num_paths - 1) *
			sizeof(struct cam_axi_per_path_bw_vote));
		if (blob_size < clk_update_size) {
			CAM_ERR(CAM_ICP, "Invalid blob size: %u",
				blob_size);
			return -EINVAL;
		}

		clk_info = &ctx_data->hfi_frame_process.clk_info[index];
		clk_info_v2 = &ctx_data->hfi_frame_process.clk_info_v2[index];

		memcpy(clk_info_v2, soc_req_v2, clk_update_size);

		/* Use v1 structure for clk fields */
		clk_info->budget_ns = clk_info_v2->budget_ns;
		clk_info->frame_cycles = clk_info_v2->frame_cycles;
		clk_info->rt_flag = clk_info_v2->rt_flag;

		CAM_DBG(CAM_PERF,
			"budget=%llu, frame_cycle=%llu, rt_flag=%d, num_paths=%d, clk_update_size=%d, index=%d, ctx_data=%pK",
			clk_info_v2->budget_ns, clk_info_v2->frame_cycles,
			clk_info_v2->rt_flag,
			clk_info_v2->num_paths,
			clk_update_size,
			index,
			ctx_data);

		for (i = 0; i < clk_info_v2->num_paths; i++) {
			CAM_DBG(CAM_PERF,
				"[%d] : path_type=%d, trans_type=%d, camnoc=%lld, mnoc_ab=%lld, mnoc_ib=%lld",
				i,
				clk_info_v2->axi_path_flex[i].path_data_type,
				clk_info_v2->axi_path_flex[i].transac_type,
				clk_info_v2->axi_path_flex[i].camnoc_bw,
				clk_info_v2->axi_path_flex[i].mnoc_ab_bw,
				clk_info_v2->axi_path_flex[i].mnoc_ib_bw);
		}

		break;

	case CAM_ICP_CMD_GENERIC_BLOB_CFG_IO:
		CAM_DBG(CAM_ICP, "CAM_ICP_CMD_GENERIC_BLOB_CFG_IO");
		pResource = *((uint32_t *)blob_data);
		if (copy_from_user(&ctx_data->icp_dev_io_info,
			(void __user *)pResource,
			sizeof(struct cam_icp_acquire_dev_info))) {
			CAM_ERR(CAM_ICP, "Failed in copy from user");
			return -EFAULT;
		}
		CAM_DBG(CAM_ICP, "buf handle %d",
			ctx_data->icp_dev_io_info.io_config_cmd_handle);
		rc = cam_mem_get_io_buf(
			ctx_data->icp_dev_io_info.io_config_cmd_handle,
			icp_hw_mgr.iommu_hdl,
			blob->io_buf_addr, &io_buf_size);
		if (rc)
			CAM_ERR(CAM_ICP, "Failed in blob update");
		else
			CAM_DBG(CAM_ICP, "io buf addr %llu",
				*blob->io_buf_addr);
		break;

	case CAM_ICP_CMD_GENERIC_BLOB_FW_MEM_MAP:
		cmd_mem_regions =
			(struct cam_cmd_mem_regions *)blob_data;
		if (cmd_mem_regions->num_regions <= 0) {
			rc = -EINVAL;
			CAM_ERR(CAM_ICP,
				"Invalid number of regions for FW map %u",
				cmd_mem_regions->num_regions);
		} else {
			CAM_DBG(CAM_ICP,
				"Processing blob for mapping %u regions",
				cmd_mem_regions->num_regions);
			rc = cam_icp_process_stream_settings(ctx_data,
				cmd_mem_regions, true);
		}
		break;

	case CAM_ICP_CMD_GENERIC_BLOB_FW_MEM_UNMAP:
		cmd_mem_regions =
			(struct cam_cmd_mem_regions *)blob_data;
		if (cmd_mem_regions->num_regions <= 0) {
			rc = -EINVAL;
			CAM_ERR(CAM_ICP,
				"Invalid number of regions for FW unmap %u",
				cmd_mem_regions->num_regions);
		} else {
			CAM_DBG(CAM_ICP,
				"Processing blob for unmapping %u regions",
				cmd_mem_regions->num_regions);
			rc = cam_icp_process_stream_settings(ctx_data,
				cmd_mem_regions, false);
		}
		break;

	default:
		CAM_WARN(CAM_ICP, "Invalid blob type %d", blob_type);
		break;
	}
	return rc;
}

static int cam_icp_process_generic_cmd_buffer(
	struct cam_packet *packet,
	struct cam_icp_hw_ctx_data *ctx_data,
	int32_t index,
	dma_addr_t *io_buf_addr)
{
	int i, rc = 0;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct icp_cmd_generic_blob cmd_generic_blob;

	cmd_generic_blob.ctx = ctx_data;
	cmd_generic_blob.frame_info_idx = index;
	cmd_generic_blob.io_buf_addr = io_buf_addr;

	cmd_desc = (struct cam_cmd_buf_desc *)
		((uint32_t *) &packet->payload_flex + packet->cmd_buf_offset/4);
	for (i = 0; i < packet->num_cmd_buf; i++) {
		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc)
			return rc;

		if (!cmd_desc[i].length)
			continue;

		if (cmd_desc[i].meta_data != CAM_ICP_CMD_META_GENERIC_BLOB)
			continue;

		rc = cam_packet_util_process_generic_cmd_buffer(&cmd_desc[i],
			cam_icp_packet_generic_blob_handler, &cmd_generic_blob);
		if (rc)
			CAM_ERR(CAM_ICP, "Failed in processing blobs %d", rc);
	}

	return rc;
}

static int cam_icp_mgr_process_cfg_io_cmd(
	struct cam_icp_hw_ctx_data *ctx_data,
	struct hfi_cmd_ipebps_async *ioconfig_cmd,
	uint64_t request_id,
	uint64_t io_config)
{
	ioconfig_cmd->size = sizeof(struct hfi_cmd_ipebps_async);
	ioconfig_cmd->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT;
	if (ctx_data->icp_dev_acquire_info->dev_type == CAM_ICP_RES_TYPE_BPS)
		ioconfig_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_BPS_CONFIG_IO;
	else
		ioconfig_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_IPE_CONFIG_IO;

	ioconfig_cmd->num_fw_handles = 1;
	ioconfig_cmd->fw_handles_flex[0] = ctx_data->fw_handle;
	ioconfig_cmd->payload.indirect = io_config;
	ioconfig_cmd->user_data1 = PTR_TO_U64(ctx_data);
	ioconfig_cmd->user_data2 = request_id;

	return 0;
}

static int cam_icp_mgr_update_hfi_frame_process(
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_packet *packet,
	struct cam_hw_prepare_update_args *prepare_args,
	int32_t *idx)
{
	int32_t index, rc;
	struct hfi_cmd_ipebps_async *hfi_cmd = NULL;

	index = find_first_zero_bit(ctx_data->hfi_frame_process.bitmap,
		ctx_data->hfi_frame_process.bits);
	if (index < 0 || index >= CAM_FRAME_CMD_MAX) {
		CAM_ERR(CAM_ICP, "request idx is wrong: %d", index);
		return -EINVAL;
	}
	set_bit(index, ctx_data->hfi_frame_process.bitmap);

	ctx_data->hfi_frame_process.request_id[index] =
		packet->header.request_id;
	ctx_data->hfi_frame_process.frame_info[index].request_id =
		packet->header.request_id;
	ctx_data->hfi_frame_process.frame_info[index].io_config = 0;
	rc = cam_icp_process_generic_cmd_buffer(packet, ctx_data, index,
		&ctx_data->hfi_frame_process.frame_info[index].io_config);
	if (rc) {
		clear_bit(index, ctx_data->hfi_frame_process.bitmap);
		ctx_data->hfi_frame_process.request_id[index] = -1;
		return rc;
	}

	if (ctx_data->hfi_frame_process.frame_info[index].io_config)  {
		hfi_cmd = (struct hfi_cmd_ipebps_async *)
		&ctx_data->hfi_frame_process.frame_info[index].hfi_cfg_io_cmd;
		rc = cam_icp_mgr_process_cfg_io_cmd(ctx_data, hfi_cmd,
			packet->header.request_id,
		ctx_data->hfi_frame_process.frame_info[index].io_config);
	}
	*idx = index;

	return rc;
}

static void cam_icp_mgr_print_io_bufs(struct cam_packet *packet,
	int32_t iommu_hdl, int32_t sec_mmu_hdl, uint32_t pf_buf_info,
	bool *mem_found)
{
	dma_addr_t   iova_addr;
	size_t     src_buf_size;
	int        i;
	int        j;
	int        rc = 0;
	int32_t    mmu_hdl;

	struct cam_buf_io_cfg  *io_cfg = NULL;

	if (mem_found)
		*mem_found = false;

	io_cfg = (struct cam_buf_io_cfg *)((uint32_t *)&packet->payload +
		packet->io_configs_offset / 4);

	for (i = 0; i < packet->num_io_configs; i++) {
		for (j = 0; j < CAM_PACKET_MAX_PLANES; j++) {
			if (!io_cfg[i].mem_handle[j])
				break;

			if (GET_FD_FROM_HANDLE(io_cfg[i].mem_handle[j]) ==
				GET_FD_FROM_HANDLE(pf_buf_info)) {
				CAM_INFO(CAM_ICP,
					"Found PF at port: %d mem %x fd: %x",
					io_cfg[i].resource_type,
					io_cfg[i].mem_handle[j],
					pf_buf_info);
				if (mem_found)
					*mem_found = true;
			}

			CAM_INFO(CAM_ICP, "port: %d f: %u format: %d dir %d",
				io_cfg[i].resource_type,
				io_cfg[i].fence,
				io_cfg[i].format,
				io_cfg[i].direction);

			mmu_hdl = cam_mem_is_secure_buf(
				io_cfg[i].mem_handle[j]) ? sec_mmu_hdl :
				iommu_hdl;
			rc = cam_mem_get_io_buf(io_cfg[i].mem_handle[j],
				mmu_hdl, &iova_addr, &src_buf_size);
			if (rc < 0) {
				CAM_ERR(CAM_UTIL,
					"get src buf address fail rc %d", rc);
				continue;
			}
			if ((iova_addr & 0xFFFFFFFF) != iova_addr) {
				CAM_ERR(CAM_ICP, "Invalid mapped address");
				rc = -EINVAL;
				continue;
			}

			CAM_INFO(CAM_ICP,
				"pln %d dir %d w %d h %d s %u sh %u sz %d addr 0x%x off 0x%x memh %x",
				j, io_cfg[i].direction,
				io_cfg[i].planes[j].width,
				io_cfg[i].planes[j].height,
				io_cfg[i].planes[j].plane_stride,
				io_cfg[i].planes[j].slice_height,
				(int32_t)src_buf_size,
				(unsigned int)iova_addr,
				io_cfg[i].offsets[j],
				io_cfg[i].mem_handle[j]);

			iova_addr += io_cfg[i].offsets[j];

		}
	}
	cam_packet_dump_patch_info(packet, icp_hw_mgr.iommu_hdl,
		icp_hw_mgr.iommu_sec_hdl);
}

static int cam_icp_mgr_config_stream_settings(
	void *hw_mgr_priv, void *hw_stream_settings)
{
	int        rc = 0;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct cam_packet *packet = NULL;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct icp_cmd_generic_blob cmd_generic_blob;
	struct cam_hw_stream_setttings *config_args =
		hw_stream_settings;

	if ((!hw_stream_settings) ||
		(!hw_mgr) || (!config_args->packet)) {
		CAM_ERR(CAM_ICP, "Invalid input arguments");
		return -EINVAL;
	}

	ctx_data = config_args->ctxt_to_hw_map;
	mutex_lock(&ctx_data->ctx_mutex);
	packet = config_args->packet;

	cmd_generic_blob.ctx = ctx_data;
	cmd_generic_blob.frame_info_idx = -1;
	cmd_generic_blob.io_buf_addr = NULL;

	cmd_desc = (struct cam_cmd_buf_desc *)
		((uint32_t *) &packet->payload_flex + packet->cmd_buf_offset/4);

	rc = cam_packet_util_validate_cmd_desc(cmd_desc);
	if (rc)
		return rc;

	if (!cmd_desc[0].length ||
		cmd_desc[0].meta_data != CAM_ICP_CMD_META_GENERIC_BLOB) {
		CAM_ERR(CAM_ICP, "Invalid cmd buffer length/metadata");
		rc = -EINVAL;
		goto end;
	}

	rc = cam_packet_util_process_generic_cmd_buffer(&cmd_desc[0],
		cam_icp_packet_generic_blob_handler, &cmd_generic_blob);
	if (rc)
		CAM_ERR(CAM_ICP, "Failed in processing cmd mem blob %d", rc);

end:
	mutex_unlock(&ctx_data->ctx_mutex);
	return rc;
}

static int cam_icp_mgr_prepare_hw_update(void *hw_mgr_priv,
	void *prepare_hw_update_args)
{
	int        rc = 0;
	int32_t    idx;
	uint32_t   fw_cmd_buf_iova_addr;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct cam_packet *packet = NULL;
	struct hfi_cmd_ipebps_async *hfi_cmd = NULL;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_hw_prepare_update_args *prepare_args =
		prepare_hw_update_args;

	if ((!prepare_args) || (!hw_mgr) || (!prepare_args->packet)) {
		CAM_ERR(CAM_ICP, "Invalid args");
		return -EINVAL;
	}

	ctx_data = prepare_args->ctxt_to_hw_map;
	mutex_lock(&ctx_data->ctx_mutex);
	if (ctx_data->state != CAM_ICP_CTX_STATE_ACQUIRED) {
		mutex_unlock(&ctx_data->ctx_mutex);
		CAM_ERR(CAM_ICP, "ctx id: %u is not in use",
			ctx_data->ctx_id);
		return -EINVAL;
	}

	packet = prepare_args->packet;

	if (cam_packet_util_validate_packet(packet, prepare_args->remain_len))
		return -EINVAL;

	rc = cam_icp_mgr_pkt_validation(packet);
	if (rc) {
		mutex_unlock(&ctx_data->ctx_mutex);
		return rc;
	}

	rc = cam_icp_mgr_process_cmd_desc(hw_mgr, packet,
		ctx_data, &fw_cmd_buf_iova_addr);
	if (rc) {
		mutex_unlock(&ctx_data->ctx_mutex);
		return rc;
	}

	prepare_args->pf_data->packet = packet;

	CAM_DBG(CAM_REQ, "req id = %lld for ctx = %u",
		packet->header.request_id, ctx_data->ctx_id);
	/* Update Buffer Address from handles and patch information */
	rc = cam_packet_util_process_patches(packet, hw_mgr->iommu_hdl,
		hw_mgr->iommu_sec_hdl);
	if (rc) {
		mutex_unlock(&ctx_data->ctx_mutex);
		return rc;
	}

	rc = cam_icp_mgr_update_hfi_frame_process(ctx_data, packet,
		prepare_args, &idx);
	if (rc) {
		mutex_unlock(&ctx_data->ctx_mutex);
		return rc;
	}

	rc = cam_icp_mgr_process_io_cfg(hw_mgr, ctx_data,
		packet, prepare_args, idx);
	if (rc) {
		if (ctx_data->hfi_frame_process.in_resource[idx] > 0)
			cam_sync_destroy(
				ctx_data->hfi_frame_process.in_resource[idx]);
		clear_bit(idx, ctx_data->hfi_frame_process.bitmap);
		ctx_data->hfi_frame_process.request_id[idx] = -1;
		mutex_unlock(&ctx_data->ctx_mutex);
		return rc;
	}

	hfi_cmd = (struct hfi_cmd_ipebps_async *)
			&ctx_data->hfi_frame_process.hfi_frame_cmd[idx];
	cam_icp_mgr_prepare_frame_process_cmd(
		ctx_data, hfi_cmd, packet->header.request_id,
		fw_cmd_buf_iova_addr);

	prepare_args->num_hw_update_entries = 1;
	prepare_args->hw_update_entries[0].addr = (uintptr_t)hfi_cmd;
	prepare_args->priv = &ctx_data->hfi_frame_process.frame_info[idx];

	CAM_DBG(CAM_ICP, "X: req id = %lld ctx_id = %u",
		packet->header.request_id, ctx_data->ctx_id);
	mutex_unlock(&ctx_data->ctx_mutex);
	return rc;
}

static int cam_icp_mgr_send_abort_status(struct cam_icp_hw_ctx_data *ctx_data)
{
	struct hfi_frame_process_info *hfi_frame_process;
	int idx;

	mutex_lock(&ctx_data->ctx_mutex);
	hfi_frame_process = &ctx_data->hfi_frame_process;
	for (idx = 0; idx < CAM_FRAME_CMD_MAX; idx++) {
		if (!hfi_frame_process->request_id[idx])
			continue;

		ctx_data->ctxt_event_cb(ctx_data->context_priv,
			CAM_CTX_EVT_ID_CANCEL,
			&hfi_frame_process->request_id[idx]);

		/* now release memory for hfi frame process command */
		hfi_frame_process->request_id[idx] = 0;
		if (ctx_data->hfi_frame_process.in_resource[idx] > 0) {
			CAM_DBG(CAM_ICP, "Delete merged sync in object: %d",
				ctx_data->hfi_frame_process.in_resource[idx]);
			cam_sync_destroy(
				ctx_data->hfi_frame_process.in_resource[idx]);
			ctx_data->hfi_frame_process.in_resource[idx] = 0;
	}
		clear_bit(idx, ctx_data->hfi_frame_process.bitmap);
	}
	mutex_unlock(&ctx_data->ctx_mutex);
	return 0;
}

static int cam_icp_mgr_delete_sync(void *priv, void *data)
{
	struct hfi_cmd_work_data *task_data = NULL;
	struct cam_icp_hw_ctx_data *ctx_data;
	struct hfi_frame_process_info *hfi_frame_process;
	int idx;

	if (!data || !priv) {
		CAM_ERR(CAM_ICP, "Invalid params%pK %pK", data, priv);
		return -EINVAL;
	}

	task_data = (struct hfi_cmd_work_data *)data;
	ctx_data = task_data->data;

	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "Null Context");
		return -EINVAL;
	}

	mutex_lock(&ctx_data->ctx_mutex);
	hfi_frame_process = &ctx_data->hfi_frame_process;
	for (idx = 0; idx < CAM_FRAME_CMD_MAX; idx++) {
		if (!hfi_frame_process->in_free_resource[idx])
			continue;
		//cam_sync_destroy(
			//ctx_data->hfi_frame_process.in_free_resource[idx]);
		ctx_data->hfi_frame_process.in_resource[idx] = 0;
	}
	mutex_unlock(&ctx_data->ctx_mutex);
	return 0;
}

static int cam_icp_mgr_delete_sync_obj(struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0;
	struct crm_workq_task *task;
	struct hfi_cmd_work_data *task_data;

	task = cam_req_mgr_workq_get_task(icp_hw_mgr.cmd_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "no empty task");
		return -ENOMEM;
	}

	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)ctx_data;
	task_data->request_id = 0;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_delete_sync;
	rc = cam_req_mgr_workq_enqueue_task(task, &icp_hw_mgr,
		CRM_TASK_PRIORITY_0);

	return rc;
}

static int cam_icp_mgr_flush_all(struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_hw_flush_args *flush_args)
{
	struct hfi_frame_process_info *hfi_frame_process;
	int idx;
	bool clear_in_resource = false;

	hfi_frame_process = &ctx_data->hfi_frame_process;
	for (idx = 0; idx < CAM_FRAME_CMD_MAX; idx++) {
		if (!hfi_frame_process->request_id[idx])
			continue;

		/* now release memory for hfi frame process command */
		hfi_frame_process->request_id[idx] = 0;
		if (ctx_data->hfi_frame_process.in_resource[idx] > 0) {
			ctx_data->hfi_frame_process.in_free_resource[idx] =
				ctx_data->hfi_frame_process.in_resource[idx];
			ctx_data->hfi_frame_process.in_resource[idx] = 0;
		}
		clear_bit(idx, ctx_data->hfi_frame_process.bitmap);
		clear_in_resource = true;
	}

	if (clear_in_resource)
		cam_icp_mgr_delete_sync_obj(ctx_data);

	return 0;
}

static int cam_icp_mgr_flush_req(struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_hw_flush_args *flush_args)
{
	int64_t request_id;
	struct hfi_frame_process_info *hfi_frame_process;
	int idx;
	bool clear_in_resource = false;

	hfi_frame_process = &ctx_data->hfi_frame_process;
	request_id = *(int64_t *)flush_args->flush_req_pending[0];
	for (idx = 0; idx < CAM_FRAME_CMD_MAX; idx++) {
		if (!hfi_frame_process->request_id[idx])
			continue;

		if (hfi_frame_process->request_id[idx] != request_id)
			continue;

		/* now release memory for hfi frame process command */
		hfi_frame_process->request_id[idx] = 0;
		if (ctx_data->hfi_frame_process.in_resource[idx] > 0) {
			ctx_data->hfi_frame_process.in_free_resource[idx] =
				ctx_data->hfi_frame_process.in_resource[idx];
			ctx_data->hfi_frame_process.in_resource[idx] = 0;
		}
		clear_bit(idx, ctx_data->hfi_frame_process.bitmap);
		clear_in_resource = true;
	}

	if (clear_in_resource)
		cam_icp_mgr_delete_sync_obj(ctx_data);

	return 0;
}

static void cam_icp_mgr_flush_info_dump(
	struct cam_hw_flush_args *flush_args, uint32_t ctx_id)
{
	int i;

	for (i = 0; i < flush_args->num_req_active; i++) {
		CAM_DBG(CAM_ICP, "Flushing active request %lld in ctx %u",
			*(int64_t *)flush_args->flush_req_active[i],
			ctx_id);
	}

	for (i = 0; i < flush_args->num_req_pending; i++) {
		CAM_DBG(CAM_ICP, "Flushing pending request %lld in ctx %u",
			*(int64_t *)flush_args->flush_req_pending[i],
			ctx_id);
	}
}

static int cam_icp_mgr_enqueue_abort(
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int timeout = 1000, rc;
	unsigned long rem_jiffies = 0;
	struct hfi_cmd_work_data *task_data;
	struct crm_workq_task *task;

	task = cam_req_mgr_workq_get_task(icp_hw_mgr.cmd_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "no empty task");
		return -ENOMEM;
	}

	reinit_completion(&ctx_data->wait_complete);
	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)ctx_data;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_abort_handle_wq;
	cam_req_mgr_workq_enqueue_task(task, &icp_hw_mgr,
		CRM_TASK_PRIORITY_0);

	rem_jiffies = wait_for_completion_timeout(&ctx_data->wait_complete,
		msecs_to_jiffies((timeout)));
	if (!rem_jiffies) {
		rc = cam_icp_retry_wait_for_abort(ctx_data);
		if (rc) {
			CAM_ERR(CAM_ICP,
				"FW timeout/err in abort handle command ctx: %u",
				ctx_data->ctx_id);
			cam_icp_mgr_process_dbg_buf(icp_hw_mgr.icp_dbg_lvl);
			cam_hfi_queue_dump();
			return rc;
		}
	}

	CAM_DBG(CAM_ICP, "Abort after flush is success");
	return 0;
}

static int cam_icp_mgr_hw_dump(void *hw_priv, void *hw_dump_args)
{
	int                              rc;
	int                              i;
	size_t                           remain_len;
	uint8_t                         *dst;
	uint32_t                         min_len;
	uint64_t                         diff;
	uint64_t                        *addr, *start;
	struct timespec64                cur_ts;
	struct timespec64                req_ts;
	ktime_t                          cur_time;
	struct cam_hw_intf              *icp_dev_intf;
	struct cam_icp_hw_mgr           *hw_mgr;
	struct cam_hw_dump_args         *dump_args;
	struct cam_icp_hw_ctx_data      *ctx_data;
	struct cam_icp_dump_header      *hdr;
	struct cam_icp_hw_dump_args      icp_dump_args;
	struct hfi_frame_process_info   *frm_process;

	if ((!hw_priv) || (!hw_dump_args)) {
		CAM_ERR(CAM_ICP, "Invalid params %pK %pK",
			hw_priv, hw_dump_args);
		return -EINVAL;
	}

	dump_args = (struct cam_hw_dump_args *)hw_dump_args;
	hw_mgr = hw_priv;
	ctx_data = dump_args->ctxt_to_hw_map;
	CAM_DBG(CAM_ICP, "Req %lld", dump_args->request_id);
	frm_process = &ctx_data->hfi_frame_process;
	for (i = 0; i < CAM_FRAME_CMD_MAX; i++) {
		if ((frm_process->request_id[i] ==
			dump_args->request_id) &&
			frm_process->fw_process_flag[i])
			goto hw_dump;
	}
	return 0;
hw_dump:
	cur_time = ktime_get();
	diff = ktime_us_delta(frm_process->submit_timestamp[i], cur_time);
	cur_ts = ktime_to_timespec64(cur_time);
	req_ts = ktime_to_timespec64(frm_process->submit_timestamp[i]);

	if (diff < CAM_ICP_CTX_RESPONSE_TIME_THRESHOLD) {
		CAM_INFO(CAM_ICP, "No Error req %lld %ld:%06ld %ld:%06ld",
			dump_args->request_id,
			req_ts.tv_sec,
			req_ts.tv_nsec/NSEC_PER_USEC,
			cur_ts.tv_sec,
			cur_ts.tv_nsec/NSEC_PER_USEC);
		return 0;
	}

	CAM_INFO(CAM_ICP, "Error req %lld %ld:%06ld %ld:%06ld",
		dump_args->request_id,
		req_ts.tv_sec,
		req_ts.tv_nsec/NSEC_PER_USEC,
		cur_ts.tv_sec,
		cur_ts.tv_nsec/NSEC_PER_USEC);

	rc  = cam_mem_get_cpu_buf(dump_args->buf_handle,
		&icp_dump_args.cpu_addr, &icp_dump_args.buf_len);
	if (rc) {
		CAM_ERR(CAM_ICP, "Invalid addr %u rc %d",
			dump_args->buf_handle, rc);
		return rc;
	}
	if (icp_dump_args.buf_len <= dump_args->offset) {
		CAM_WARN(CAM_ICP, "dump buffer overshoot len %zu offset %zu",
			icp_dump_args.buf_len, dump_args->offset);
		cam_mem_put_cpu_buf(dump_args->buf_handle);
		return -ENOSPC;
	}

	remain_len = icp_dump_args.buf_len - dump_args->offset;
	min_len = sizeof(struct cam_icp_dump_header) +
			(CAM_ICP_DUMP_NUM_WORDS * sizeof(uint64_t));

	if (remain_len < min_len) {
		CAM_WARN(CAM_ICP, "dump buffer exhaust remain %zu min %u",
			remain_len, min_len);
		cam_mem_put_cpu_buf(dump_args->buf_handle);
		return -ENOSPC;
	}

	dst = (uint8_t *)icp_dump_args.cpu_addr + dump_args->offset;
	hdr = (struct cam_icp_dump_header *)dst;
	scnprintf(hdr->tag, CAM_ICP_DUMP_TAG_MAX_LEN, "ICP_REQ:");
	hdr->word_size = sizeof(uint64_t);
	addr = (uint64_t *)(dst + sizeof(struct cam_icp_dump_header));
	start = addr;
	*addr++ = frm_process->request_id[i];
	*addr++ = req_ts.tv_sec;
	*addr++ = req_ts.tv_nsec/NSEC_PER_USEC;
	*addr++ = cur_ts.tv_sec;
	*addr++ = cur_ts.tv_nsec/NSEC_PER_USEC;
	hdr->size = hdr->word_size * (addr - start);
	dump_args->offset += (hdr->size + sizeof(struct cam_icp_dump_header));
	/* Dumping the fw image*/
	icp_dump_args.offset = dump_args->offset;
	icp_dev_intf = hw_mgr->icp_dev_intf;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is NULL");
		cam_mem_put_cpu_buf(dump_args->buf_handle);
		return -EINVAL;
	}

	rc = icp_dev_intf->hw_ops.process_cmd(
		icp_dev_intf->hw_priv,
		CAM_ICP_CMD_HW_DUMP, &icp_dump_args,
		sizeof(struct cam_icp_hw_dump_args));
	CAM_DBG(CAM_ICP, "Offset before %zu after %zu",
		dump_args->offset, icp_dump_args.offset);
	dump_args->offset = icp_dump_args.offset;

	cam_mem_put_cpu_buf(dump_args->buf_handle);
	return rc;
}

static int cam_icp_mgr_hw_flush(void *hw_priv, void *hw_flush_args)
{
	struct cam_hw_flush_args *flush_args = hw_flush_args;
	struct cam_icp_hw_ctx_data *ctx_data;
	struct cam_icp_hw_mgr *hw_mgr = hw_priv;

	if ((!hw_priv) || (!hw_flush_args)) {
		CAM_ERR(CAM_ICP, "Input params are Null:");
		return -EINVAL;
	}

	ctx_data = flush_args->ctxt_to_hw_map;
	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "Ctx data is NULL");
		return -EINVAL;
	}

	if ((flush_args->flush_type >= CAM_FLUSH_TYPE_MAX) ||
		(flush_args->flush_type < CAM_FLUSH_TYPE_REQ)) {
		CAM_ERR(CAM_ICP, "Invalid lush type: %d",
			flush_args->flush_type);
		return -EINVAL;
	}

	ctx_data->last_flush_req = flush_args->last_flush_req;
	CAM_DBG(CAM_REQ, "ctx_id %d Flush type %d last_flush_req %u",
		ctx_data->ctx_id, flush_args->flush_type,
		ctx_data->last_flush_req);
	switch (flush_args->flush_type) {
	case CAM_FLUSH_TYPE_ALL:
		mutex_lock(&hw_mgr->hw_mgr_mutex);
		if (!atomic_read(&hw_mgr->recovery)
			&& flush_args->num_req_active) {
			mutex_unlock(&hw_mgr->hw_mgr_mutex);
			cam_icp_mgr_flush_info_dump(flush_args,
				ctx_data->ctx_id);
			cam_icp_mgr_enqueue_abort(ctx_data);
		} else {
			mutex_unlock(&hw_mgr->hw_mgr_mutex);
		}
		mutex_lock(&ctx_data->ctx_mutex);
		cam_icp_mgr_flush_all(ctx_data, flush_args);
		mutex_unlock(&ctx_data->ctx_mutex);
		break;
	case CAM_FLUSH_TYPE_REQ:
		mutex_lock(&ctx_data->ctx_mutex);
		if (flush_args->num_req_active) {
			CAM_ERR(CAM_ICP, "Flush request is not supported");
			mutex_unlock(&ctx_data->ctx_mutex);
			return -EINVAL;
		}
		if (flush_args->num_req_pending)
			cam_icp_mgr_flush_req(ctx_data, flush_args);
		mutex_unlock(&ctx_data->ctx_mutex);
		break;
	default:
		CAM_ERR(CAM_ICP, "Invalid flush type: %d",
			flush_args->flush_type);
		return -EINVAL;
	}

	return 0;
}

static int cam_icp_mgr_release_hw(void *hw_mgr_priv, void *release_hw_args)
{
	int rc = 0;
	int ctx_id = 0;
	struct cam_hw_release_args *release_hw = release_hw_args;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;

	if (!release_hw || !hw_mgr) {
		CAM_ERR(CAM_ICP, "Invalid args: %pK %pK", release_hw, hw_mgr);
		return -EINVAL;
	}

	CAM_DBG(CAM_ICP, "Enter recovery set %d",
		atomic_read(&hw_mgr->recovery));
	ctx_data = release_hw->ctxt_to_hw_map;
	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "NULL ctx data");
		return -EINVAL;
	}

	ctx_id = ctx_data->ctx_id;
	if (ctx_id < 0 || ctx_id >= CAM_ICP_CTX_MAX) {
		CAM_ERR(CAM_ICP, "Invalid ctx id: %d", ctx_id);
		return -EINVAL;
	}

	mutex_lock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);
	if (hw_mgr->ctx_data[ctx_id].state != CAM_ICP_CTX_STATE_ACQUIRED) {
		CAM_DBG(CAM_ICP, "ctx is not in use: %d", ctx_id);
		mutex_unlock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);
		return -EINVAL;
	}
	mutex_unlock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	if (!atomic_read(&hw_mgr->recovery) && release_hw->active_req) {
		mutex_unlock(&hw_mgr->hw_mgr_mutex);
		cam_icp_mgr_abort_handle(ctx_data);
		cam_icp_mgr_send_abort_status(ctx_data);
	} else {
		mutex_unlock(&hw_mgr->hw_mgr_mutex);
	}

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	rc = cam_icp_mgr_release_ctx(hw_mgr, ctx_id);
	if (!hw_mgr->ctxt_cnt) {
		CAM_DBG(CAM_ICP, "Last Release");
		cam_icp_mgr_icp_power_collapse(hw_mgr);
		cam_icp_hw_mgr_reset_clk_info(hw_mgr);
		rc = cam_ipe_bps_deint(hw_mgr);
	}
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	if ((!hw_mgr->bps_ctxt_cnt || !hw_mgr->ipe_ctxt_cnt))
		cam_icp_device_timer_stop(hw_mgr);

	CAM_DBG(CAM_ICP, "Release done for ctx_id %d", ctx_id);
	return rc;
}

static int cam_icp_mgr_create_handle(uint32_t dev_type,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	struct hfi_cmd_create_handle create_handle;
	struct hfi_cmd_work_data *task_data;
	unsigned long rem_jiffies;
	int timeout = 5000;
	struct crm_workq_task *task;
	int rc = 0;

	task = cam_req_mgr_workq_get_task(icp_hw_mgr.cmd_work);
	if (!task)
		return -ENOMEM;

	create_handle.size = sizeof(struct hfi_cmd_create_handle);
	create_handle.pkt_type = HFI_CMD_IPEBPS_CREATE_HANDLE;
	create_handle.handle_type = dev_type;
	create_handle.user_data1 = PTR_TO_U64(ctx_data);
	reinit_completion(&ctx_data->wait_complete);
	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)&create_handle;
	task_data->request_id = 0;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_process_cmd;
	rc = cam_req_mgr_workq_enqueue_task(task, &icp_hw_mgr,
		CRM_TASK_PRIORITY_0);
	if (rc)
		return rc;

	rem_jiffies = wait_for_completion_timeout(&ctx_data->wait_complete,
			msecs_to_jiffies((timeout)));
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		CAM_ERR(CAM_ICP, "FW response timed out %d", rc);
		cam_icp_mgr_process_dbg_buf(icp_hw_mgr.icp_dbg_lvl);
		cam_hfi_queue_dump();
	}

	if (ctx_data->fw_handle == 0) {
		CAM_ERR(CAM_ICP, "Invalid handle created");
		rc = -EINVAL;
	}

	return rc;
}

static int cam_icp_mgr_send_ping(struct cam_icp_hw_ctx_data *ctx_data)
{
	struct hfi_cmd_ping_pkt ping_pkt;
	struct hfi_cmd_work_data *task_data;
	unsigned long rem_jiffies;
	int timeout = 5000;
	struct crm_workq_task *task;
	int rc = 0;

	task = cam_req_mgr_workq_get_task(icp_hw_mgr.cmd_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "No free task to send ping command");
		return -ENOMEM;
	}

	ping_pkt.size = sizeof(struct hfi_cmd_ping_pkt);
	ping_pkt.pkt_type = HFI_CMD_SYS_PING;
	ping_pkt.user_data = PTR_TO_U64(ctx_data);
	init_completion(&ctx_data->wait_complete);
	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)&ping_pkt;
	task_data->request_id = 0;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_process_cmd;

	rc = cam_req_mgr_workq_enqueue_task(task, &icp_hw_mgr,
		CRM_TASK_PRIORITY_0);
	if (rc)
		return rc;

	rem_jiffies = wait_for_completion_timeout(&ctx_data->wait_complete,
			msecs_to_jiffies((timeout)));
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		CAM_ERR(CAM_ICP, "FW response timed out %d", rc);
		cam_icp_mgr_process_dbg_buf(icp_hw_mgr.icp_dbg_lvl);
		cam_hfi_queue_dump();
	}

	return rc;
}

static int cam_icp_get_acquire_info(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_hw_acquire_args *args,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int i;
	int acquire_size;
	struct cam_icp_acquire_dev_info icp_dev_acquire_info;
	struct cam_icp_res_info *p_icp_out = NULL;

	if (copy_from_user(&icp_dev_acquire_info,
		(void __user *)args->acquire_info,
		sizeof(struct cam_icp_acquire_dev_info))) {
		CAM_ERR(CAM_ICP, "Failed in acquire");
		return -EFAULT;
	}

	if (icp_dev_acquire_info.secure_mode > CAM_SECURE_MODE_SECURE) {
		CAM_ERR(CAM_ICP, "Invalid mode:%d",
			icp_dev_acquire_info.secure_mode);
		return -EINVAL;
	}

	if ((icp_dev_acquire_info.num_out_res > ICP_MAX_OUTPUT_SUPPORTED) ||
		(icp_dev_acquire_info.num_out_res <= 0)) {
		CAM_ERR(CAM_ICP, "Invalid num of out resources: %u",
			icp_dev_acquire_info.num_out_res);
		return -EINVAL;
	}

	if (icp_dev_acquire_info.dev_type >= CAM_ICP_RES_TYPE_MAX) {
		CAM_ERR(CAM_ICP, "Invalid device type: %d",
			icp_dev_acquire_info.dev_type);
		return -EFAULT;
	}

	acquire_size = sizeof(struct cam_icp_acquire_dev_info) +
		((icp_dev_acquire_info.num_out_res - 1) *
		sizeof(struct cam_icp_res_info));
	ctx_data->icp_dev_acquire_info = kzalloc(acquire_size, GFP_KERNEL);
	if (!ctx_data->icp_dev_acquire_info)
		return -ENOMEM;

	if (copy_from_user(ctx_data->icp_dev_acquire_info,
		(void __user *)args->acquire_info, acquire_size)) {
		CAM_ERR(CAM_ICP, "Failed in acquire: size = %d", acquire_size);
		kfree(ctx_data->icp_dev_acquire_info);
		ctx_data->icp_dev_acquire_info = NULL;
		return -EFAULT;
	}

	CAM_DBG(CAM_ICP, "%x %x %x %x %x %x %x",
		ctx_data->icp_dev_acquire_info->dev_type,
		ctx_data->icp_dev_acquire_info->in_res.format,
		ctx_data->icp_dev_acquire_info->in_res.width,
		ctx_data->icp_dev_acquire_info->in_res.height,
		ctx_data->icp_dev_acquire_info->in_res.fps,
		ctx_data->icp_dev_acquire_info->num_out_res,
		ctx_data->icp_dev_acquire_info->scratch_mem_size);

	p_icp_out = ctx_data->icp_dev_acquire_info->out_res_flex;
	for (i = 0; i < icp_dev_acquire_info.num_out_res; i++)
		CAM_DBG(CAM_ICP, "out[i] %x %x %x %x",
			p_icp_out[i].format,
			p_icp_out[i].width,
			p_icp_out[i].height,
			p_icp_out[i].fps);

	return 0;
}

static uint32_t cam_icp_unify_dev_type(
	uint32_t dev_type)
{
	switch (dev_type) {
	case CAM_ICP_RES_TYPE_BPS:
		return CAM_ICP_RES_TYPE_BPS;
	case CAM_ICP_RES_TYPE_BPS_RT:
		return CAM_ICP_RES_TYPE_BPS;
	case CAM_ICP_RES_TYPE_BPS_SEMI_RT:
		return CAM_ICP_RES_TYPE_BPS;
	case CAM_ICP_RES_TYPE_IPE:
		return CAM_ICP_RES_TYPE_IPE;
	case CAM_ICP_RES_TYPE_IPE_RT:
		return CAM_ICP_RES_TYPE_IPE;
	case CAM_ICP_RES_TYPE_IPE_SEMI_RT:
		return CAM_ICP_RES_TYPE_IPE;
	default:
		return CAM_ICP_RES_TYPE_MAX;
	}
}

static int cam_icp_mgr_acquire_hw(void *hw_mgr_priv, void *acquire_hw_args)
{
	int rc = 0, bitmap_size = 0;
	uint32_t ctx_id = 0, dev_type;
	dma_addr_t io_buf_addr;
	size_t io_buf_size;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct cam_hw_acquire_args *args = acquire_hw_args;
	struct cam_icp_acquire_dev_info *icp_dev_acquire_info;
	struct cam_cmd_mem_regions cmd_mem_region;

	if ((!hw_mgr_priv) || (!acquire_hw_args)) {
		CAM_ERR(CAM_ICP, "Invalid params: %pK %pK", hw_mgr_priv,
			acquire_hw_args);
		return -EINVAL;
	}

	if (args->num_acq > 1) {
		CAM_ERR(CAM_ICP, "number of resources are wrong: %u",
			args->num_acq);
		return -EINVAL;
	}

	CAM_DBG(CAM_ICP, "ENTER");
	mutex_lock(&hw_mgr->hw_mgr_mutex);
	ctx_id = cam_icp_mgr_get_free_ctx(hw_mgr);
	if (ctx_id >= CAM_ICP_CTX_MAX) {
		CAM_ERR(CAM_ICP, "No free ctx space in hw_mgr");
		mutex_unlock(&hw_mgr->hw_mgr_mutex);
		return -ENOSPC;
	}
	ctx_data = &hw_mgr->ctx_data[ctx_id];
	ctx_data->ctx_id = ctx_id;

	mutex_lock(&ctx_data->ctx_mutex);
	rc = cam_icp_get_acquire_info(hw_mgr, args, ctx_data);
	if (rc)
		goto acquire_info_failed;

	icp_dev_acquire_info = ctx_data->icp_dev_acquire_info;
	dev_type = icp_dev_acquire_info->dev_type;
	icp_dev_acquire_info->dev_type =
		cam_icp_unify_dev_type(dev_type);

	CAM_DBG(CAM_ICP, "acquire io buf handle %d",
		icp_dev_acquire_info->io_config_cmd_handle);
	rc = cam_mem_get_io_buf(
		icp_dev_acquire_info->io_config_cmd_handle,
		hw_mgr->iommu_hdl,
		&io_buf_addr, &io_buf_size);
	if (rc) {
		CAM_ERR(CAM_ICP, "unable to get src buf info from io desc");
		goto get_io_buf_failed;
	}

	CAM_DBG(CAM_ICP, "hdl: %d, addr: %pK, size: %zu",
		icp_dev_acquire_info->io_config_cmd_handle,
		(void *)io_buf_addr, io_buf_size);

	if (!hw_mgr->ctxt_cnt) {
		rc = cam_icp_clk_info_init(hw_mgr, ctx_data);
		if (rc)
			goto get_io_buf_failed;

		rc = cam_icp_mgr_icp_resume(hw_mgr);
		if (rc)
			goto get_io_buf_failed;

		if (icp_hw_mgr.icp_debug_type)
			hfi_set_debug_level(icp_hw_mgr.icp_debug_type,
				icp_hw_mgr.icp_dbg_lvl);

		hfi_set_fw_dump_level(icp_hw_mgr.icp_fw_dump_lvl);

		rc = cam_icp_send_ubwc_cfg(hw_mgr);
		if (rc)
			goto ubwc_cfg_failed;
	}


	rc = cam_icp_mgr_ipe_bps_resume(hw_mgr, ctx_data);
	if (rc)
		goto ipe_bps_resume_failed;

	rc = cam_icp_mgr_send_ping(ctx_data);
	if (rc) {
		CAM_ERR(CAM_ICP, "ping ack not received");
		goto send_ping_failed;
	}
	CAM_DBG(CAM_ICP, "ping ack received");

	rc = cam_icp_mgr_create_handle(dev_type,
		ctx_data);
	if (rc) {
		CAM_ERR(CAM_ICP, "create handle failed");
		goto create_handle_failed;
	}

	CAM_DBG(CAM_ICP,
		"created stream handle for dev_type %u",
		dev_type);

	cmd_mem_region.num_regions = 1;
	cmd_mem_region.map_info_array_flex[0].mem_handle =
		icp_dev_acquire_info->io_config_cmd_handle;
	cmd_mem_region.map_info_array_flex[0].offset = 0;
	cmd_mem_region.map_info_array_flex[0].size =
		icp_dev_acquire_info->io_config_cmd_size;
	cmd_mem_region.map_info_array_flex[0].flags = 0;

	rc = cam_icp_process_stream_settings(ctx_data,
		&cmd_mem_region, true);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"sending config io mapping failed rc %d", rc);
		goto send_map_info_failed;
	}

	rc = cam_icp_mgr_send_config_io(ctx_data, io_buf_addr);
	if (rc) {
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"IO Config command failed %d size:%d",
			rc, icp_dev_acquire_info->io_config_cmd_size);
		cam_icp_dump_io_cfg(ctx_data,
			icp_dev_acquire_info->io_config_cmd_handle,
			icp_dev_acquire_info->io_config_cmd_size);
		goto ioconfig_failed;
	}

	rc = cam_icp_process_stream_settings(ctx_data,
		&cmd_mem_region, false);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"sending config io unmapping failed %d", rc);
		goto send_map_info_failed;
	}

	ctx_data->context_priv = args->context_data;
	args->ctxt_to_hw_map = ctx_data;

	bitmap_size = BITS_TO_LONGS(CAM_FRAME_CMD_MAX) * sizeof(long);
	ctx_data->hfi_frame_process.bitmap =
			kzalloc(bitmap_size, GFP_KERNEL);
	if (!ctx_data->hfi_frame_process.bitmap) {
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"hfi frame bitmap failed ctx id:%d dev hdl:0x%x session hdl:0x%x dev type %d",
			ctx_data->ctx_id, ctx_data->acquire_dev_cmd.dev_handle,
			ctx_data->acquire_dev_cmd.session_handle,
			ctx_data->icp_dev_acquire_info->dev_type);
		goto ioconfig_failed;
	}

	ctx_data->hfi_frame_process.bits = bitmap_size * BITS_PER_BYTE;
	hw_mgr->ctx_data[ctx_id].ctxt_event_cb = args->event_cb;
	icp_dev_acquire_info->scratch_mem_size = ctx_data->scratch_mem_size;

	if (copy_to_user((void __user *)args->acquire_info,
		icp_dev_acquire_info,
		sizeof(struct cam_icp_acquire_dev_info))) {
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"copy from user failed ctx id:%d dev hdl:0x%x session hdl:0x%x dev type %d",
			ctx_data->ctx_id, ctx_data->acquire_dev_cmd.dev_handle,
			ctx_data->acquire_dev_cmd.session_handle,
			ctx_data->icp_dev_acquire_info->dev_type);
		goto copy_to_user_failed;
	}

	cam_icp_ctx_clk_info_init(ctx_data);
	ctx_data->state = CAM_ICP_CTX_STATE_ACQUIRED;
	mutex_unlock(&ctx_data->ctx_mutex);
	CAM_DBG(CAM_ICP, "scratch size = %x fw_handle = %x",
			(unsigned int)icp_dev_acquire_info->scratch_mem_size,
			(unsigned int)ctx_data->fw_handle);
	/* Start device timer*/
	if (((hw_mgr->bps_ctxt_cnt == 1) || (hw_mgr->ipe_ctxt_cnt == 1)))
		cam_icp_device_timer_start(hw_mgr);
	/* Start context timer*/
	cam_icp_ctx_timer_start(ctx_data);
	hw_mgr->ctxt_cnt++;
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	CAM_DBG(CAM_ICP, "Acquire Done for ctx_id %u dev type %d",
		ctx_data->ctx_id,
		ctx_data->icp_dev_acquire_info->dev_type);

	return 0;

copy_to_user_failed:
	kfree(ctx_data->hfi_frame_process.bitmap);
	ctx_data->hfi_frame_process.bitmap = NULL;
ioconfig_failed:
	cam_icp_process_stream_settings(ctx_data,
		&cmd_mem_region, false);
send_map_info_failed:
	cam_icp_mgr_destroy_handle(ctx_data);
create_handle_failed:
send_ping_failed:
	cam_icp_mgr_ipe_bps_power_collapse(hw_mgr, ctx_data, 0);
ipe_bps_resume_failed:
ubwc_cfg_failed:
	if (!hw_mgr->ctxt_cnt)
		cam_icp_mgr_icp_power_collapse(hw_mgr);
get_io_buf_failed:
	kfree(hw_mgr->ctx_data[ctx_id].icp_dev_acquire_info);
	hw_mgr->ctx_data[ctx_id].icp_dev_acquire_info = NULL;
acquire_info_failed:
	cam_icp_mgr_put_ctx(ctx_data);
	cam_icp_mgr_process_dbg_buf(icp_hw_mgr.icp_dbg_lvl);
	mutex_unlock(&ctx_data->ctx_mutex);
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	return rc;
}

static int cam_icp_mgr_get_hw_caps(void *hw_mgr_priv, void *hw_caps_args)
{
	int rc = 0;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_query_cap_cmd *query_cap = hw_caps_args;

	if ((!hw_mgr_priv) || (!hw_caps_args)) {
		CAM_ERR(CAM_ICP, "Invalid params: %pK %pK",
			hw_mgr_priv, hw_caps_args);
		return -EINVAL;
	}

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	if (copy_from_user(&icp_hw_mgr.icp_caps,
		u64_to_user_ptr(query_cap->caps_handle),
		sizeof(struct cam_icp_query_cap_cmd))) {
		CAM_ERR(CAM_ICP, "copy_from_user failed");
		rc = -EFAULT;
		goto end;
	}

	rc = hfi_get_hw_caps(&icp_hw_mgr.icp_caps);
	if (rc)
		goto end;

	icp_hw_mgr.icp_caps.dev_iommu_handle.non_secure = hw_mgr->iommu_hdl;
	icp_hw_mgr.icp_caps.dev_iommu_handle.secure = hw_mgr->iommu_sec_hdl;

	if (copy_to_user(u64_to_user_ptr(query_cap->caps_handle),
		&icp_hw_mgr.icp_caps, sizeof(struct cam_icp_query_cap_cmd))) {
		CAM_ERR(CAM_ICP, "copy_to_user failed");
		rc = -EFAULT;
	}
end:
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	return rc;
}

static int cam_icp_mgr_alloc_devs(struct device_node *of_node)
{
	int rc;
	uint32_t num_dev;

	rc = of_property_read_u32(of_node, "num-a5", &num_dev);
	if (rc) {
		CAM_ERR(CAM_ICP, "getting num of a5 failed");
		goto num_a5_failed;
	}

	icp_hw_mgr.devices[CAM_ICP_DEV_A5] = kzalloc(
		sizeof(struct cam_hw_intf *) * num_dev, GFP_KERNEL);
	if (!icp_hw_mgr.devices[CAM_ICP_DEV_A5]) {
		rc = -ENOMEM;
		CAM_ERR(CAM_ICP, "a5 allocation fail: rc = %d", rc);
		goto num_a5_failed;
	}

	rc = of_property_read_u32(of_node, "num-ipe", &num_dev);
	if (rc) {
		CAM_ERR(CAM_ICP, "getting number of ipe dev nodes failed");
		goto num_ipe_failed;
	}

	if (!icp_hw_mgr.ipe1_enable)
		num_dev = 1;

	icp_hw_mgr.devices[CAM_ICP_DEV_IPE] = kcalloc(num_dev,
		sizeof(struct cam_hw_intf *), GFP_KERNEL);
	if (!icp_hw_mgr.devices[CAM_ICP_DEV_IPE]) {
		rc = -ENOMEM;
		CAM_ERR(CAM_ICP, "ipe device allocation fail : rc= %d", rc);
		goto num_ipe_failed;
	}

	rc = of_property_read_u32(of_node, "num-bps", &num_dev);
	if (rc) {
		CAM_ERR(CAM_ICP, "read num bps devices failed");
		goto num_bps_failed;
	}
	icp_hw_mgr.devices[CAM_ICP_DEV_BPS] = kcalloc(num_dev,
		sizeof(struct cam_hw_intf *), GFP_KERNEL);
	if (!icp_hw_mgr.devices[CAM_ICP_DEV_BPS]) {
		rc = -ENOMEM;
		CAM_ERR(CAM_ICP, "bps device allocation fail : rc= %d", rc);
		goto num_bps_failed;
	}

	icp_hw_mgr.ipe_bps_pc_flag = of_property_read_bool(of_node,
		"ipe_bps_pc_en");

	icp_hw_mgr.icp_pc_flag = of_property_read_bool(of_node,
		"icp_pc_en");

	return 0;
num_bps_failed:
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_IPE]);
num_ipe_failed:
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_A5]);
num_a5_failed:
	return rc;
}

static int cam_icp_mgr_init_devs(struct device_node *of_node)
{
	int rc = 0;
	int count, i;
	const char *name = NULL;
	struct device_node *child_node = NULL;
	struct platform_device *child_pdev = NULL;
	struct cam_hw_intf *child_dev_intf = NULL;

	rc = cam_icp_mgr_alloc_devs(of_node);
	if (rc) {
		CAM_ERR(CAM_ICP, "alloc_devs fail : rc = %d", rc);
		return rc;
	}
	count = of_property_count_strings(of_node, "compat-hw-name");
	if (!count) {
		CAM_ERR(CAM_ICP, "no compat hw found in dev tree, cnt = %d",
			count);
		rc = -EINVAL;
		goto compat_hw_name_failed;
	}

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node, "compat-hw-name",
			i, &name);
		if (rc) {
			CAM_ERR(CAM_ICP, "getting dev object name failed");
			goto compat_hw_name_failed;
		}

		child_node = of_find_node_by_name(NULL, name);
		if (!child_node) {
			CAM_ERR(CAM_ICP, "Cannot find node in dtsi %s", name);
			rc = -ENODEV;
			goto compat_hw_name_failed;
		}

		child_pdev = of_find_device_by_node(child_node);
		if (!child_pdev) {
			CAM_ERR(CAM_ICP, "failed to find device on bus %s",
				child_node->name);
			rc = -ENODEV;
			of_node_put(child_node);
			goto compat_hw_name_failed;
		}

		child_dev_intf = (struct cam_hw_intf *)platform_get_drvdata(
			child_pdev);
		if (!child_dev_intf) {
			CAM_ERR(CAM_ICP, "no child device");
			of_node_put(child_node);
			if (!icp_hw_mgr.ipe1_enable)
				continue;
			goto compat_hw_name_failed;
		}
		icp_hw_mgr.devices[child_dev_intf->hw_type]
			[child_dev_intf->hw_idx] = child_dev_intf;

		if (!child_dev_intf->hw_ops.process_cmd)
			goto compat_hw_name_failed;

		of_node_put(child_node);
	}

	icp_hw_mgr.icp_dev_intf = icp_hw_mgr.devices[CAM_ICP_DEV_A5][0];
	icp_hw_mgr.bps_dev_intf = icp_hw_mgr.devices[CAM_ICP_DEV_BPS][0];
	icp_hw_mgr.ipe0_dev_intf = icp_hw_mgr.devices[CAM_ICP_DEV_IPE][0];
	if (icp_hw_mgr.ipe1_enable)
		icp_hw_mgr.ipe1_dev_intf =
			icp_hw_mgr.devices[CAM_ICP_DEV_IPE][1];

	return 0;
compat_hw_name_failed:
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_BPS]);
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_IPE]);
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_A5]);
	return rc;
}

static void cam_req_mgr_process_workq_icp_command_queue(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

static void cam_req_mgr_process_workq_icp_message_queue(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

static void cam_req_mgr_process_workq_icp_timer_queue(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

static int cam_icp_mgr_create_wq(void)
{
	int rc;
	int i;

	rc = cam_req_mgr_workq_create("icp_command_queue", ICP_WORKQ_NUM_TASK,
		&icp_hw_mgr.cmd_work, CRM_WORKQ_USAGE_NON_IRQ, 0,
		cam_req_mgr_process_workq_icp_command_queue);
	if (rc) {
		CAM_ERR(CAM_ICP, "unable to create a command worker");
		goto cmd_work_failed;
	}

	rc = cam_req_mgr_workq_create("icp_message_queue", ICP_WORKQ_NUM_TASK,
		&icp_hw_mgr.msg_work, CRM_WORKQ_USAGE_IRQ, 0,
		cam_req_mgr_process_workq_icp_message_queue);
	if (rc) {
		CAM_ERR(CAM_ICP, "unable to create a message worker");
		goto msg_work_failed;
	}

	rc = cam_req_mgr_workq_create("icp_timer_queue", ICP_WORKQ_NUM_TASK,
		&icp_hw_mgr.timer_work, CRM_WORKQ_USAGE_IRQ, 0,
		cam_req_mgr_process_workq_icp_timer_queue);
	if (rc) {
		CAM_ERR(CAM_ICP, "unable to create a timer worker");
		goto timer_work_failed;
	}

	icp_hw_mgr.cmd_work_data =
		kzalloc(sizeof(struct hfi_cmd_work_data) * ICP_WORKQ_NUM_TASK,
		GFP_KERNEL);
	if (!icp_hw_mgr.cmd_work_data) {
		CAM_ERR(CAM_ICP, "Mem reservation fail for cmd_work_data");
		goto cmd_work_data_failed;
	}
	icp_hw_mgr.msg_work_data =
		kzalloc(sizeof(struct hfi_msg_work_data) * ICP_WORKQ_NUM_TASK,
		GFP_KERNEL);
	if (!icp_hw_mgr.msg_work_data) {
		CAM_ERR(CAM_ICP, "Mem reservation fail for msg_work_data");
		goto msg_work_data_failed;
	}

	icp_hw_mgr.timer_work_data =
		kzalloc(sizeof(struct hfi_msg_work_data) * ICP_WORKQ_NUM_TASK,
		GFP_KERNEL);
	if (!icp_hw_mgr.timer_work_data) {
		CAM_ERR(CAM_ICP, "Mem reservation fail for timer_work_data");
		goto timer_work_data_failed;
	}

	rc = cam_icp_hw_mgr_create_debugfs_entry();
	if (rc)
		goto debugfs_create_failed;

	for (i = 0; i < ICP_WORKQ_NUM_TASK; i++)
		icp_hw_mgr.msg_work->task.pool[i].payload =
				&icp_hw_mgr.msg_work_data[i];

	for (i = 0; i < ICP_WORKQ_NUM_TASK; i++)
		icp_hw_mgr.cmd_work->task.pool[i].payload =
				&icp_hw_mgr.cmd_work_data[i];

	for (i = 0; i < ICP_WORKQ_NUM_TASK; i++)
		icp_hw_mgr.timer_work->task.pool[i].payload =
				&icp_hw_mgr.timer_work_data[i];
	return 0;

debugfs_create_failed:
	kfree(icp_hw_mgr.timer_work_data);
timer_work_data_failed:
	kfree(icp_hw_mgr.msg_work_data);
msg_work_data_failed:
	kfree(icp_hw_mgr.cmd_work_data);
cmd_work_data_failed:
	cam_req_mgr_workq_destroy(&icp_hw_mgr.timer_work);
timer_work_failed:
	cam_req_mgr_workq_destroy(&icp_hw_mgr.msg_work);
msg_work_failed:
	cam_req_mgr_workq_destroy(&icp_hw_mgr.cmd_work);
cmd_work_failed:
	return rc;
}

static void cam_icp_mgr_destroy_wq(void)
{
	cam_req_mgr_workq_destroy(&icp_hw_mgr.timer_work);
	cam_req_mgr_workq_destroy(&icp_hw_mgr.msg_work);
	cam_req_mgr_workq_destroy(&icp_hw_mgr.cmd_work);
}

static int cam_icp_mgr_cmd(void *hw_mgr_priv, void *cmd_args)
{
	int rc = 0;
	struct cam_hw_cmd_args *hw_cmd_args = cmd_args;
	struct cam_icp_hw_mgr  *hw_mgr = hw_mgr_priv;

	if (!hw_mgr_priv || !cmd_args) {
		CAM_ERR(CAM_ICP, "Invalid arguments");
		return -EINVAL;
	}

	switch (hw_cmd_args->cmd_type) {
	case CAM_HW_MGR_CMD_DUMP_PF_INFO:
		cam_icp_mgr_print_io_bufs(
			hw_cmd_args->u.pf_args.pf_data.packet,
			hw_mgr->iommu_hdl,
			hw_mgr->iommu_sec_hdl,
			hw_cmd_args->u.pf_args.buf_info,
			hw_cmd_args->u.pf_args.mem_found);

		break;
	default:
		CAM_ERR(CAM_ICP, "Invalid cmd");
	}

	return rc;
}

int cam_icp_hw_mgr_init(struct device_node *of_node, uint64_t *hw_mgr_hdl,
	int *iommu_hdl)
{
	int i, rc = 0;
	struct cam_hw_mgr_intf *hw_mgr_intf;
	struct cam_cpas_query_cap query;
	uint32_t cam_caps, camera_hw_version;

	hw_mgr_intf = (struct cam_hw_mgr_intf *)hw_mgr_hdl;
	if (!of_node || !hw_mgr_intf) {
		CAM_ERR(CAM_ICP, "Invalid args of_node %pK hw_mgr %pK",
			of_node, hw_mgr_intf);
		return -EINVAL;
	}

	hw_mgr_intf->hw_mgr_priv = &icp_hw_mgr;
	hw_mgr_intf->hw_get_caps = cam_icp_mgr_get_hw_caps;
	hw_mgr_intf->hw_acquire = cam_icp_mgr_acquire_hw;
	hw_mgr_intf->hw_release = cam_icp_mgr_release_hw;
	hw_mgr_intf->hw_prepare_update = cam_icp_mgr_prepare_hw_update;
	hw_mgr_intf->hw_config_stream_settings =
		cam_icp_mgr_config_stream_settings;
	hw_mgr_intf->hw_config = cam_icp_mgr_config_hw;
	hw_mgr_intf->hw_open = cam_icp_mgr_hw_open_u;
	hw_mgr_intf->hw_close = cam_icp_mgr_hw_close_u;
	hw_mgr_intf->hw_flush = cam_icp_mgr_hw_flush;
	hw_mgr_intf->hw_cmd = cam_icp_mgr_cmd;
	hw_mgr_intf->hw_dump = cam_icp_mgr_hw_dump;

	icp_hw_mgr.secure_mode = CAM_SECURE_MODE_NON_SECURE;
	mutex_init(&icp_hw_mgr.hw_mgr_mutex);
	spin_lock_init(&icp_hw_mgr.hw_mgr_lock);

	for (i = 0; i < CAM_ICP_CTX_MAX; i++)
		mutex_init(&icp_hw_mgr.ctx_data[i].ctx_mutex);

	rc = cam_cpas_get_hw_info(&query.camera_family,
			&query.camera_version, &query.cpas_version,
			&cam_caps, NULL);
	if (rc) {
		CAM_ERR(CAM_ICP, "failed to get hw info rc=%d", rc);
		goto destroy_mutex;
	}

	rc = cam_cpas_get_cpas_hw_version(&camera_hw_version);
	if (rc) {
		CAM_ERR(CAM_ICP, "failed to get hw version rc=%d", rc);
		goto destroy_mutex;
	}

	if ((camera_hw_version == CAM_CPAS_TITAN_480_V100) ||
		(camera_hw_version == CAM_CPAS_TITAN_580_V100) ||
		(camera_hw_version == CAM_CPAS_TITAN_570_V200)) {
		if (cam_caps & CPAS_TITAN_480_IPE0_BIT)
			icp_hw_mgr.ipe0_enable = true;
		if (cam_caps & CPAS_BPS_BIT)
			icp_hw_mgr.bps_enable = true;
	} else {
		if (cam_caps & CPAS_IPE0_BIT)
			icp_hw_mgr.ipe0_enable = true;
		if (cam_caps & CPAS_IPE1_BIT)
			icp_hw_mgr.ipe1_enable = true;
		if (cam_caps & CPAS_BPS_BIT)
			icp_hw_mgr.bps_enable = true;
	}

	rc = cam_icp_mgr_init_devs(of_node);
	if (rc) {
		CAM_ERR(CAM_ICP, "cam_icp_mgr_init_devs fail: rc: %d", rc);
		goto destroy_mutex;
	}
	rc = cam_smmu_get_handle("icp", &icp_hw_mgr.iommu_hdl);
	if (rc) {
		CAM_ERR(CAM_ICP, "get mmu handle failed: %d", rc);
		goto icp_get_hdl_failed;
	}

	rc = cam_smmu_get_handle("cam-secure", &icp_hw_mgr.iommu_sec_hdl);
	if (rc) {
		CAM_ERR(CAM_ICP, "get secure mmu handle failed: %d", rc);
		goto secure_hdl_failed;
	}

	rc = cam_icp_mgr_create_wq();
	if (rc) {
		CAM_ERR(CAM_ICP, "cam_icp_mgr_create_wq fail: rc=%d", rc);
		goto icp_wq_create_failed;
	}

	if (iommu_hdl)
		*iommu_hdl = icp_hw_mgr.iommu_hdl;

	init_completion(&icp_hw_mgr.icp_complete);
	return rc;

icp_wq_create_failed:
	cam_smmu_destroy_handle(icp_hw_mgr.iommu_sec_hdl);
	icp_hw_mgr.iommu_sec_hdl = -1;
secure_hdl_failed:
	cam_smmu_destroy_handle(icp_hw_mgr.iommu_hdl);
	icp_hw_mgr.iommu_hdl = -1;
icp_get_hdl_failed:
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_BPS]);
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_IPE]);
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_A5]);
destroy_mutex:
	mutex_destroy(&icp_hw_mgr.hw_mgr_mutex);
	for (i = 0; i < CAM_ICP_CTX_MAX; i++)
		mutex_destroy(&icp_hw_mgr.ctx_data[i].ctx_mutex);

	return rc;
}

void cam_icp_hw_mgr_deinit(void)
{
	int i = 0;

	debugfs_remove_recursive(icp_hw_mgr.dentry);
	icp_hw_mgr.dentry = NULL;
	cam_icp_mgr_destroy_wq();
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_BPS]);
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_IPE]);
	kfree(icp_hw_mgr.devices[CAM_ICP_DEV_A5]);
	mutex_destroy(&icp_hw_mgr.hw_mgr_mutex);
	for (i = 0; i < CAM_ICP_CTX_MAX; i++)
		mutex_destroy(&icp_hw_mgr.ctx_data[i].ctx_mutex);
}
