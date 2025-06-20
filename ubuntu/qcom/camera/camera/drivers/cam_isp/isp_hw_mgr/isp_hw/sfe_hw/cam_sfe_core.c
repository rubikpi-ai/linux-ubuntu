// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/slab.h>
#include <linux/list.h>
#include "cam_tasklet_util.h"
#include "cam_sfe_hw_intf.h"
#include "cam_sfe_soc.h"
#include "cam_sfe_core.h"
#include "cam_debug_util.h"

static const char drv_name[] = "sfe";

int cam_sfe_get_hw_caps(void *device_priv,
	void *get_hw_cap_args, uint32_t arg_size)
{
	return -EPERM;
}

int cam_sfe_reset(void *device_priv,
	void *reset_core_args, uint32_t arg_size)
{
	return -EPERM;
}

int cam_sfe_read(void *device_priv,
	void *read_args, uint32_t arg_size)
{
	return -EPERM;
}

int cam_sfe_write(void *device_priv,
	void *write_args, uint32_t arg_size)
{
	return -EPERM;
}

int cam_sfe_init_hw(void *hw_priv, void *init_hw_args, uint32_t arg_size)
{
	struct cam_hw_info                *sfe_hw = hw_priv;
	struct cam_hw_soc_info            *soc_info = NULL;
	struct cam_sfe_hw_core_info       *core_info = NULL;
	int rc = 0;

	if (!hw_priv) {
		CAM_ERR(CAM_SFE, "Invalid arguments");
		return -EINVAL;
	}

	mutex_lock(&sfe_hw->hw_mutex);
	sfe_hw->open_count++;
	if (sfe_hw->open_count > 1) {
		mutex_unlock(&sfe_hw->hw_mutex);
		CAM_DBG(CAM_SFE, "SFE has already been initialized cnt: %d",
			sfe_hw->open_count);
		return 0;
	}
	mutex_unlock(&sfe_hw->hw_mutex);

	soc_info = &sfe_hw->soc_info;
	core_info = (struct cam_sfe_hw_core_info *)sfe_hw->core_info;

	/* Turn ON Regulators, Clocks and other SOC resources */
	if (!core_info->sfe_hw_info->bus_wr_version &&
		core_info->sfe_hw_info->bus_rd_version)
		rc = cam_sfe_enable_soc_resources(soc_info, true);
	else
		rc = cam_sfe_enable_soc_resources(soc_info, false);
	if (rc) {
		CAM_ERR(CAM_SFE, "Enable SOC failed");
		rc = -EFAULT;
		goto deinit_hw;
	}

	CAM_DBG(CAM_SFE, "SFE SOC resource enabled");

	if (core_info->sfe_top->hw_ops.init) {
		rc = core_info->sfe_top->hw_ops.init(core_info->sfe_top->top_priv, NULL, 0);
		if (rc) {
			CAM_ERR(CAM_SFE, "Top init failed rc=%d", rc);
			goto deinit_hw;
		}
	}

	if (core_info->sfe_hw_info->bus_wr_version &&
		core_info->sfe_bus_wr->hw_ops.init) {
		rc = core_info->sfe_bus_wr->hw_ops.init(core_info->sfe_bus_wr->bus_priv, NULL, 0);
		if (rc) {
			CAM_ERR(CAM_SFE, "Bus WR init failed rc=%d", rc);
			goto deinit_hw;
		}
	}

	if (core_info->sfe_hw_info->bus_rd_version &&
		core_info->sfe_bus_rd->hw_ops.init) {
		rc = core_info->sfe_bus_rd->hw_ops.init(core_info->sfe_bus_rd->bus_priv, NULL, 0);
		if (rc) {
			CAM_ERR(CAM_SFE, "Bus RD init failed rc=%d", rc);
			goto deinit_hw;
		}
	}

	/*
	 * Async Reset as part of power ON
	 * Any register write in bus_wr_init/bus_rd_init
	 * will be cleared by sync reset in CSID.
	 *
	 */

	sfe_hw->hw_state = CAM_HW_STATE_POWER_UP;
	return rc;

deinit_hw:
	cam_sfe_deinit_hw(hw_priv, NULL, 0);
	return rc;
}

int cam_sfe_deinit_hw(void *hw_priv, void *deinit_hw_args, uint32_t arg_size)
{
	struct cam_hw_info                *sfe_hw = hw_priv;
	struct cam_hw_soc_info            *soc_info = NULL;
	struct cam_sfe_hw_core_info       *core_info = NULL;
	int rc = 0;


	if (!hw_priv) {
		CAM_ERR(CAM_SFE, "Invalid arguments");
		return -EINVAL;
	}

	mutex_lock(&sfe_hw->hw_mutex);
	if (!sfe_hw->open_count) {
		mutex_unlock(&sfe_hw->hw_mutex);
		CAM_ERR(CAM_SFE, "Unbalanced deinit");
		return -EFAULT;
	}

	sfe_hw->open_count--;
	if (sfe_hw->open_count) {
		mutex_unlock(&sfe_hw->hw_mutex);
		CAM_DBG(CAM_SFE, "open_cnt non-zero: %d",
			sfe_hw->open_count);
		return 0;
	}
	mutex_unlock(&sfe_hw->hw_mutex);

	soc_info = &sfe_hw->soc_info;
	core_info = (struct cam_sfe_hw_core_info *)sfe_hw->core_info;

	if (core_info->sfe_hw_info->bus_rd_version &&
		core_info->sfe_bus_rd->hw_ops.deinit) {
		rc = core_info->sfe_bus_rd->hw_ops.deinit(core_info->sfe_bus_rd->bus_priv, NULL, 0);
		if (rc)
			CAM_ERR(CAM_SFE, "Bus RD deinit failed rc=%d", rc);
	}

	/* Probe write engine only if it exists - 0x0 is not a valid version */
	if (core_info->sfe_hw_info->bus_wr_version &&
		core_info->sfe_bus_wr->hw_ops.deinit) {
		rc = core_info->sfe_bus_wr->hw_ops.deinit(core_info->sfe_bus_wr->bus_priv, NULL, 0);
		if (rc)
			CAM_ERR(CAM_SFE, "Bus WR deinit failed rc=%d", rc);
	}

	if (core_info->sfe_top->hw_ops.deinit) {
		rc = core_info->sfe_top->hw_ops.deinit(core_info->sfe_top->top_priv, NULL, 0);
		if (rc)
			CAM_ERR(CAM_SFE, "Top deinit failed rc=%d", rc);
	}

	/* Turn OFF Regulators, Clocks and other SOC resources */
	CAM_DBG(CAM_SFE, "Disable SFE SOC resource");
	rc = cam_sfe_disable_soc_resources(soc_info);
	if (rc)
		CAM_ERR(CAM_SFE, "Disable SOC failed");

	sfe_hw->hw_state = CAM_HW_STATE_POWER_DOWN;

	CAM_DBG(CAM_SFE, "SFE deinit done rc: %d", rc);
	return rc;
}

int cam_sfe_test_irq_line(void *hw_priv)
{
	struct cam_hw_info *sfe_hw = hw_priv;
	struct cam_sfe_hw_core_info *core_info;
	int rc;

	if (!hw_priv) {
		CAM_ERR(CAM_SFE, "Invalid arguments");
		return -EINVAL;
	}

	core_info = sfe_hw->core_info;

	rc = cam_sfe_init_hw(sfe_hw, NULL, 0);
	if (rc) {
		CAM_ERR(CAM_SFE, "SFE:%d failed to init hw", sfe_hw->soc_info.index);
		return rc;
	}

	rc = cam_irq_controller_test_irq_line(core_info->sfe_irq_controller, "SFE:%d",
		sfe_hw->soc_info.index);
	if (rc)
		CAM_ERR(CAM_SFE, "failed to test SFE:%d", sfe_hw->soc_info.index);

	rc = cam_sfe_deinit_hw(sfe_hw, NULL, 0);
	if (rc)
		CAM_ERR(CAM_SFE, "SFE:%d failed to deinit hw", sfe_hw->soc_info.index);

	return rc;
}

int cam_sfe_reserve(void *hw_priv, void *reserve_args, uint32_t arg_size)
{
	struct cam_sfe_hw_core_info       *core_info = NULL;
	struct cam_hw_info                *sfe_hw  = hw_priv;
	struct cam_sfe_acquire_args       *acquire;
	int rc = -ENODEV;

	if (!hw_priv || !reserve_args || (arg_size !=
		sizeof(struct cam_sfe_acquire_args))) {
		CAM_ERR(CAM_SFE, "Invalid input arguments");
		return -EINVAL;
	}

	core_info = (struct cam_sfe_hw_core_info *)sfe_hw->core_info;
	acquire = (struct cam_sfe_acquire_args   *)reserve_args;

	CAM_DBG(CAM_SFE, "SFE acquire for res type: %d",
		acquire->rsrc_type);

	mutex_lock(&sfe_hw->hw_mutex);
	if (acquire->rsrc_type == CAM_ISP_RESOURCE_SFE_IN)
		rc = core_info->sfe_top->hw_ops.reserve(
			core_info->sfe_top->top_priv,
			reserve_args, arg_size);
	else if (acquire->rsrc_type == CAM_ISP_RESOURCE_SFE_OUT)
		rc = core_info->sfe_bus_wr->hw_ops.reserve(
			core_info->sfe_bus_wr->bus_priv, acquire,
			sizeof(*acquire));
	else if (acquire->rsrc_type == CAM_ISP_RESOURCE_SFE_RD)
		rc = core_info->sfe_bus_rd->hw_ops.reserve(
			core_info->sfe_bus_rd->bus_priv, acquire,
			sizeof(*acquire));
	else
		CAM_ERR(CAM_SFE, "Invalid SFE res_type: %d",
			acquire->rsrc_type);
	mutex_unlock(&sfe_hw->hw_mutex);

	return rc;
}

int cam_sfe_release(void *hw_priv, void *release_args, uint32_t arg_size)
{
	struct cam_sfe_hw_core_info       *core_info = NULL;
	struct cam_hw_info                *sfe_hw  = hw_priv;
	struct cam_isp_resource_node      *sfe_res;
	int rc = -ENODEV;

	if (!hw_priv || !release_args ||
		(arg_size != sizeof(struct cam_isp_resource_node))) {
		CAM_ERR(CAM_SFE, "Invalid input arguments");
		return -EINVAL;
	}

	core_info = (struct cam_sfe_hw_core_info *) sfe_hw->core_info;
	sfe_res = (struct cam_isp_resource_node  *) release_args;

	CAM_DBG(CAM_SFE, "SFE release for res type: %d",
		sfe_res->res_type);

	mutex_lock(&sfe_hw->hw_mutex);
	if (sfe_res->res_type == CAM_ISP_RESOURCE_SFE_IN)
		rc = core_info->sfe_top->hw_ops.release(
			core_info->sfe_top->top_priv, sfe_res,
			sizeof(struct cam_isp_resource_node));
	else if (sfe_res->res_type == CAM_ISP_RESOURCE_SFE_OUT)
		rc = core_info->sfe_bus_wr->hw_ops.release(
			core_info->sfe_bus_wr->bus_priv, sfe_res,
			sizeof(*sfe_res));
	else if (sfe_res->res_type == CAM_ISP_RESOURCE_SFE_RD)
		rc = core_info->sfe_bus_rd->hw_ops.release(
			core_info->sfe_bus_rd->bus_priv, sfe_res,
			sizeof(*sfe_res));
	else
		CAM_ERR(CAM_SFE, "Invalid SFE res type: %d",
			sfe_res->res_type);
	mutex_unlock(&sfe_hw->hw_mutex);

	return rc;
}

int cam_sfe_start(void *hw_priv, void *start_args, uint32_t arg_size)
{
	struct cam_sfe_hw_core_info       *core_info = NULL;
	struct cam_hw_info                *sfe_hw  = hw_priv;
	struct cam_isp_resource_node      *sfe_res;
	int                                rc;

	if (!hw_priv || !start_args ||
		(arg_size != sizeof(struct cam_isp_resource_node))) {
		CAM_ERR(CAM_SFE, "Invalid input arguments");
		return -EINVAL;
	}

	core_info = (struct cam_sfe_hw_core_info *)sfe_hw->core_info;
	sfe_res = (struct cam_isp_resource_node  *)start_args;
	core_info->tasklet_info = sfe_res->tasklet_info;

	mutex_lock(&sfe_hw->hw_mutex);
	if (sfe_res->res_type == CAM_ISP_RESOURCE_SFE_IN) {
		rc = core_info->sfe_top->hw_ops.start(
			core_info->sfe_top->top_priv, sfe_res,
			sizeof(struct cam_isp_resource_node));
		if (rc)
			CAM_ERR(CAM_SFE, "Failed to start SFE IN rc: %d", rc);
	} else if (sfe_res->res_type == CAM_ISP_RESOURCE_SFE_OUT) {
		rc = core_info->sfe_bus_wr->hw_ops.start(sfe_res, NULL, 0);
		if (rc)
			CAM_ERR(CAM_SFE, "Failed to start SFE BUS WR rc: %d",
				rc);
	} else if (sfe_res->res_type == CAM_ISP_RESOURCE_SFE_RD) {
		rc = core_info->sfe_bus_rd->hw_ops.start(sfe_res,
			NULL, 0);
		if (rc)
			CAM_ERR(CAM_SFE, "Failed to start SFE BUS RD rc: %d",
				rc);
	} else {
		CAM_ERR(CAM_SFE, "Invalid SFE res type:%d",
			sfe_res->res_type);
		rc = -EINVAL;
	}

	mutex_unlock(&sfe_hw->hw_mutex);
	CAM_DBG(CAM_SFE,
		"Start for SFE res type: %u res id: %u res_state: %d rc: %d",
		sfe_res->res_type, sfe_res->res_id,
		sfe_res->res_state, rc);
	return rc;
}

int cam_sfe_stop(void *hw_priv, void *stop_args, uint32_t arg_size)
{
	struct cam_sfe_hw_core_info       *core_info = NULL;
	struct cam_hw_info                *sfe_hw  = hw_priv;
	struct cam_isp_resource_node      *sfe_res;
	int rc = -EINVAL;

	if (!hw_priv || !stop_args ||
		(arg_size != sizeof(struct cam_isp_resource_node))) {
		CAM_ERR(CAM_SFE, "Invalid input arguments");
		return -EINVAL;
	}

	core_info = (struct cam_sfe_hw_core_info *)sfe_hw->core_info;
	sfe_res = (struct cam_isp_resource_node  *)stop_args;
	mutex_lock(&sfe_hw->hw_mutex);

	if (sfe_res->res_type == CAM_ISP_RESOURCE_SFE_IN)
		rc = core_info->sfe_top->hw_ops.stop(
			core_info->sfe_top->top_priv, sfe_res,
			sizeof(struct cam_isp_resource_node));
	else if (sfe_res->res_type == CAM_ISP_RESOURCE_SFE_OUT)
		rc = core_info->sfe_bus_wr->hw_ops.stop(sfe_res, NULL, 0);
	else if (sfe_res->res_type == CAM_ISP_RESOURCE_SFE_RD)
		rc = core_info->sfe_bus_rd->hw_ops.stop(sfe_res, NULL, 0);
	else
		CAM_ERR(CAM_SFE, "Invalid SFE res type: %d", sfe_res->res_type);

	mutex_unlock(&sfe_hw->hw_mutex);
	CAM_DBG(CAM_SFE,
			"Stop for SFE res type: %u res id: %u res_state: %d rc: %d",
			sfe_res->res_type, sfe_res->res_id,
			sfe_res->res_state, rc);

	return rc;
}

int cam_sfe_process_cmd(void *hw_priv, uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_hw_info                *sfe_hw = hw_priv;
	struct cam_hw_soc_info            *soc_info = NULL;
	struct cam_sfe_hw_core_info       *core_info = NULL;
	int rc = 0;

	if (!hw_priv) {
		CAM_ERR(CAM_SFE, "Invalid arguments");
		return -EINVAL;
	}

	soc_info = &sfe_hw->soc_info;
	core_info = (struct cam_sfe_hw_core_info *)sfe_hw->core_info;

	switch (cmd_type) {
	case CAM_ISP_HW_CMD_GET_CHANGE_BASE:
	case CAM_ISP_HW_CMD_CLOCK_UPDATE:
	case CAM_ISP_HW_CMD_BW_UPDATE_V2:
	case CAM_ISP_HW_CMD_BW_CONTROL:
	case CAM_ISP_HW_CMD_CORE_CONFIG:
	case CAM_ISP_HW_NOTIFY_OVERFLOW:
	case CAM_ISP_HW_CMD_APPLY_CLK_BW_UPDATE:
	case CAM_ISP_HW_CMD_FCG_CONFIG:
		rc = core_info->sfe_top->hw_ops.process_cmd(
			core_info->sfe_top->top_priv, cmd_type,
			cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_GET_BUF_UPDATE:
	case CAM_ISP_HW_CMD_BUF_UPDATE:
	case CAM_ISP_HW_CMD_GET_HFR_UPDATE:
	case CAM_ISP_HW_CMD_STRIPE_UPDATE:
	case CAM_ISP_HW_CMD_WM_CONFIG_UPDATE:
	case CAM_ISP_HW_CMD_GET_WM_SECURE_MODE:
	case CAM_ISP_HW_SFE_SYS_CACHE_WM_CONFIG:
	case CAM_ISP_HW_CMD_WM_BW_LIMIT_CONFIG:
	case CAM_ISP_HW_SFE_BUS_MINI_DUMP:
	case CAM_ISP_HW_USER_DUMP:
	case CAM_ISP_HW_CMD_GET_LAST_CONSUMED_ADDR:
		if (core_info->sfe_hw_info->bus_wr_version) {
			rc = core_info->sfe_bus_wr->hw_ops.process_cmd(
				core_info->sfe_bus_wr->bus_priv, cmd_type,
				cmd_args, arg_size);
		}
		break;
	case CAM_ISP_HW_CMD_GET_CLK_THRESHOLDS:
		if (core_info->sfe_bus_rd) {
			rc = core_info->sfe_bus_rd->hw_ops.process_cmd(
				core_info->sfe_bus_rd->bus_priv, cmd_type,
				cmd_args, arg_size);
		}
		break;
	case CAM_ISP_HW_CMD_GET_HFR_UPDATE_RM:
	case CAM_ISP_HW_CMD_GET_BUF_UPDATE_RM:
	case CAM_ISP_HW_CMD_BUF_UPDATE_RM:
	case CAM_ISP_HW_CMD_FE_UPDATE_BUS_RD:
	case CAM_ISP_HW_SFE_SYS_CACHE_RM_CONFIG:
	case CAM_ISP_HW_CMD_RM_ENABLE_DISABLE:
	case CAM_ISP_HW_CMD_GET_RM_SECURE_MODE:
		if (core_info->sfe_hw_info->bus_rd_version) {
			rc = core_info->sfe_bus_rd->hw_ops.process_cmd(
				core_info->sfe_bus_rd->bus_priv, cmd_type,
				cmd_args, arg_size);
		}
		break;
	case  CAM_ISP_HW_CMD_UNMASK_BUS_WR_IRQ:
		/* Not supported */
		break;
	case CAM_ISP_HW_CMD_SET_SFE_DEBUG_CFG:
		/* propagate to SFE top */
		core_info->sfe_top->hw_ops.process_cmd(
			core_info->sfe_top->top_priv, cmd_type,
			cmd_args, arg_size);
		fallthrough;
	case CAM_ISP_HW_CMD_GET_RES_FOR_MID:
	case CAM_ISP_HW_CMD_DUMP_BUS_INFO:
	case CAM_ISP_HW_CMD_IRQ_INJECTION:
	case CAM_ISP_HW_CMD_DUMP_IRQ_DESCRIPTION:
		/* propagate to SFE bus wr */
		if (core_info->sfe_hw_info->bus_wr_version) {
			core_info->sfe_bus_wr->hw_ops.process_cmd(
				core_info->sfe_bus_wr->bus_priv, cmd_type,
				cmd_args, arg_size);
		}
		/* propagate to SFE bus rd */
		if (core_info->sfe_hw_info->bus_rd_version) {
			core_info->sfe_bus_rd->hw_ops.process_cmd(
				core_info->sfe_bus_rd->bus_priv, cmd_type,
				cmd_args, arg_size);
		}
		break;
	case CAM_ISP_HW_CMD_QUERY_CAP:
		/* propagate to SFE top */
		core_info->sfe_top->hw_ops.process_cmd(
			core_info->sfe_top->top_priv, cmd_type,
			cmd_args, arg_size);

		/* propagate to SFE bus wr */
		if (core_info->sfe_hw_info->bus_wr_version) {
			core_info->sfe_bus_wr->hw_ops.process_cmd(
				core_info->sfe_bus_wr->bus_priv, cmd_type,
				cmd_args, arg_size);
		}
		break;
	case CAM_ISP_HW_CMD_QUERY_REGSPACE_DATA:
		*((struct cam_hw_soc_info **)cmd_args) = soc_info;
		rc = 0;
		break;
	default:
		CAM_ERR(CAM_SFE, "Invalid cmd type: %d", cmd_type);
		rc = -EINVAL;
		break;
	}

	return rc;
}

irqreturn_t cam_sfe_irq(int irq_num, void *data)
{
	struct cam_hw_info            *sfe_hw;
	struct cam_sfe_hw_core_info   *core_info;

	if (!data)
		return IRQ_NONE;

	sfe_hw = (struct cam_hw_info *)data;
	core_info = (struct cam_sfe_hw_core_info *)sfe_hw->core_info;

	return cam_irq_controller_handle_irq(irq_num,
		core_info->sfe_irq_controller, CAM_IRQ_EVT_GROUP_0);
}

int cam_sfe_core_init(
	struct cam_sfe_hw_core_info  *core_info,
	struct cam_hw_soc_info       *soc_info,
	struct cam_hw_intf           *hw_intf,
	struct cam_sfe_hw_info       *sfe_hw_info)
{
	int rc = -EINVAL;

	rc = cam_irq_controller_init(drv_name,
		CAM_SOC_GET_REG_MAP_START(soc_info, SFE_CORE_BASE_IDX),
		sfe_hw_info->irq_reg_info, &core_info->sfe_irq_controller);
	if (rc) {
		CAM_ERR(CAM_SFE, "SFE irq controller init failed");
		return rc;
	}

	rc = cam_sfe_top_init(sfe_hw_info->top_version, soc_info, hw_intf,
		sfe_hw_info->top_hw_info, core_info->sfe_irq_controller,
		&core_info->sfe_top);
	if (rc) {
		CAM_ERR(CAM_SFE, "SFE top init failed rc: %d", rc);
		goto deinit_controller;
	}

	/* Probe write engine only if it exists - 0x0 is not a valid version */
	if (sfe_hw_info->bus_wr_version) {
		rc = cam_sfe_bus_init(sfe_hw_info->bus_wr_version, BUS_TYPE_SFE_WR,
			soc_info, hw_intf, sfe_hw_info->bus_wr_hw_info,
			core_info->sfe_irq_controller,
			&core_info->sfe_bus_wr);
		if (rc) {
			CAM_ERR(CAM_SFE, "SFE bus wr init failed rc: %d", rc);
			goto deinit_top;
		}
	}

	/* Probe fetch engine only if it exists - 0x0 is not a valid version */
	if (sfe_hw_info->bus_rd_version) {
		rc = cam_sfe_bus_init(sfe_hw_info->bus_rd_version, BUS_TYPE_SFE_RD,
			soc_info, hw_intf, sfe_hw_info->bus_rd_hw_info,
			core_info->sfe_irq_controller,
			&core_info->sfe_bus_rd);
		if (rc) {
			CAM_ERR(CAM_SFE, "SFE bus rd init failed rc: %d", rc);
			goto deinit_bus_wr;
		}
	}

	spin_lock_init(&core_info->spin_lock);
	CAM_DBG(CAM_SFE, "SFE device [%u] INIT success",
		hw_intf->hw_idx);
	return rc;

deinit_bus_wr:
	if (sfe_hw_info->bus_wr_version)
		cam_sfe_bus_deinit(sfe_hw_info->bus_wr_version,
			BUS_TYPE_SFE_WR, &core_info->sfe_bus_wr);
deinit_top:
	cam_sfe_top_deinit(sfe_hw_info->top_version,
		&core_info->sfe_top);
deinit_controller:
	if (cam_irq_controller_deinit(&core_info->sfe_irq_controller))
		CAM_ERR(CAM_SFE,
			"Error cam_irq_controller_deinit failed rc=%d", rc);

	return rc;
}

int cam_sfe_core_deinit(
	struct cam_sfe_hw_core_info  *core_info,
	struct cam_sfe_hw_info       *sfe_hw_info)
{
	int                rc = -EINVAL;
	unsigned long      flags;

	spin_lock_irqsave(&core_info->spin_lock, flags);

	if (sfe_hw_info->bus_rd_version) {
		rc = cam_sfe_bus_deinit(sfe_hw_info->bus_rd_version,
				BUS_TYPE_SFE_RD, &core_info->sfe_bus_rd);
		if (rc)
			CAM_ERR(CAM_SFE,
				"SFE bus rd deinit failed rc: %d", rc);
	}

	if (sfe_hw_info->bus_wr_version) {
		rc = cam_sfe_bus_deinit(sfe_hw_info->bus_wr_version,
				BUS_TYPE_SFE_WR, &core_info->sfe_bus_wr);
		if (rc)
			CAM_ERR(CAM_SFE,
				"SFE bus wr deinit failed rc: %d", rc);
	}

	rc = cam_sfe_top_deinit(sfe_hw_info->top_version,
		&core_info->sfe_top);
	if (rc)
		CAM_ERR(CAM_SFE,
			"SFE top deinit failed rc: %d", rc);

	rc = cam_irq_controller_deinit(&core_info->sfe_irq_controller);
	if (rc)
		CAM_ERR(CAM_SFE,
			"Error cam_irq_controller_deinit failed rc=%d", rc);

	spin_unlock_irqrestore(&core_info->spin_lock, flags);
	return rc;
}
