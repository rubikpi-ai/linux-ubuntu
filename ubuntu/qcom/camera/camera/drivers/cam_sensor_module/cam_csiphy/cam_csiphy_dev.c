// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "cam_csiphy_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_csiphy_soc.h"
#include "cam_csiphy_core.h"
#include <media/cam_sensor.h>
#include "camera_main.h"
#include <dt-bindings/msm-camera.h>

#define CSIPHY_DEBUGFS_NAME_MAX_SIZE 10
static struct dentry *root_dentry;

static inline void cam_csiphy_trigger_reg_dump(struct csiphy_device *csiphy_dev)
{
	cam_csiphy_common_status_reg_dump(csiphy_dev);

	if (csiphy_dev->en_full_phy_reg_dump)
		cam_csiphy_reg_dump(&csiphy_dev->soc_info);

	if (csiphy_dev->en_lane_status_reg_dump) {
		CAM_INFO(CAM_CSIPHY, "Status Reg Dump on failure");
		cam_csiphy_dump_status_reg(csiphy_dev);
	}
}

static int cam_csiphy_format_secure_phy_lane_info(
	struct csiphy_device *csiphy_dev, int offset, uint64_t *mask)
{
	struct cam_csiphy_param *param;
	uint64_t phy_lane_sel_mask = 0;

	param = &csiphy_dev->csiphy_info[offset];

	if (param->csiphy_3phase) {
		if (param->lane_enable & CPHY_LANE_0)
			phy_lane_sel_mask |= LANE_0_SEL;
		if (param->lane_enable & CPHY_LANE_1)
			phy_lane_sel_mask |= LANE_1_SEL;
		if (param->lane_enable & CPHY_LANE_2)
			phy_lane_sel_mask |= LANE_2_SEL;
		phy_lane_sel_mask <<= CPHY_LANE_SELECTION_SHIFT;
	} else {
		if (param->lane_enable & DPHY_LANE_0)
			phy_lane_sel_mask |= LANE_0_SEL;
		if (param->lane_enable & DPHY_LANE_1)
			phy_lane_sel_mask |= LANE_1_SEL;
		if (param->lane_enable & DPHY_LANE_2)
			phy_lane_sel_mask |= LANE_2_SEL;
		if (param->lane_enable & DPHY_LANE_3)
			phy_lane_sel_mask |= LANE_3_SEL;
		phy_lane_sel_mask <<= DPHY_LANE_SELECTION_SHIFT;
	}
	if (csiphy_dev->soc_info.index > MAX_SUPPORTED_PHY_IDX) {
		CAM_ERR(CAM_CSIPHY, "Invalid PHY index: %u",
			csiphy_dev->soc_info.index);
			return -EINVAL;
	}

	phy_lane_sel_mask |= BIT(csiphy_dev->soc_info.index);
	*mask = phy_lane_sel_mask;

	CAM_DBG(CAM_CSIPHY, "Formatted PHY[%u] phy_lane_sel_mask: 0x%llx",
		csiphy_dev->soc_info.index, *mask);

	return 0;

}

static int cam_csiphy_get_session_index(struct csiphy_device *csiphy_dev,
	uint32_t lane_assign)
{
	int i = 0;
	struct cam_csiphy_param *param;

	for (i = 0; i < CSIPHY_MAX_INSTANCES_PER_PHY; i++) {
		param = &csiphy_dev->csiphy_info[i];

		if (param->lane_assign == lane_assign)
			break;
	}

	return i;
}

static void cam_csiphy_populate_secure_info(
	struct csiphy_device *csiphy_dev, void *data)
{
	int i;
	struct cam_csiphy_secure_info *secure_info =
		(struct cam_csiphy_secure_info *)data;
	struct cam_csiphy_param *param;

	for (i = 0; i < CSIPHY_MAX_INSTANCES_PER_PHY; i++) {
		param = &csiphy_dev->csiphy_info[i];

		if (param->secure_mode &&
			param->lane_assign == secure_info->lane_assign) {

			param->secure_info.cdm_hw_idx_mask = secure_info->cdm_hw_idx_mask;
			param->secure_info.csid_hw_idx_mask = secure_info->csid_hw_idx_mask;
			param->secure_info.vc_mask = secure_info->vc_mask;
			param->secure_info.phy_lane_sel_mask = 0;

			if (!cam_csiphy_format_secure_phy_lane_info(csiphy_dev, i,
				&param->csiphy_phy_lane_sel_mask)) {
				param->secure_info_updated =  true;

				CAM_DBG(CAM_CSIPHY,
					"PHY[%d] secure info, phy_lane_mask: 0x%llx, ife: 0x%x, cdm: 0x%x, vc_mask: 0x%llx",
					csiphy_dev->soc_info.index,
					param->csiphy_phy_lane_sel_mask,
					param->secure_info.csid_hw_idx_mask,
					param->secure_info.cdm_hw_idx_mask,
					param->secure_info.vc_mask);
			} else
				CAM_ERR(CAM_CSIPHY,
					"Error in formatting PHY[%u] phy_lane_sel_mask: 0x%llx",
					csiphy_dev->soc_info.index,
					param->csiphy_phy_lane_sel_mask);

			break;
		}
	}

	if (i == CSIPHY_MAX_INSTANCES_PER_PHY)
		CAM_ERR(CAM_CSIPHY, "No matching secure PHY for a session");

}

static void cam_csiphy_subdev_handle_message(struct v4l2_subdev *sd,
	enum cam_subdev_message_type_t message_type, void *data)
{
	struct csiphy_device *csiphy_dev = v4l2_get_subdevdata(sd);
	uint32_t phy_idx;
	int rc = 0;

	if (!data) {
		CAM_ERR(CAM_CSIPHY, "Empty Payload");
		return;
	}

	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "csiphy_dev ptr is NULL");
		return;
	}

	phy_idx = *(uint32_t *)data;
	if (phy_idx != csiphy_dev->soc_info.index) {
		CAM_DBG(CAM_CSIPHY, "Current HW IDX: %u, Expected IDX: %u",
			csiphy_dev->soc_info.index, phy_idx);
		return;
	}

	switch (message_type) {
	case CAM_SUBDEV_MESSAGE_REG_DUMP: {
		cam_csiphy_trigger_reg_dump(csiphy_dev);
		break;
	}
	case CAM_SUBDEV_MESSAGE_APPLY_CSIPHY_AUX: {
		cam_csiphy_trigger_reg_dump(csiphy_dev);

		if (!csiphy_dev->skip_aux_settings) {
			cam_csiphy_update_auxiliary_mask(csiphy_dev);

			CAM_INFO(CAM_CSIPHY,
				"CSIPHY[%u] updating aux settings for data rate idx: %u",
				csiphy_dev->soc_info.index, csiphy_dev->curr_data_rate_idx);
		}
		break;
	}
	case CAM_SUBDEV_MESSAGE_DOMAIN_ID_SECURE_PARAMS: {
		cam_csiphy_populate_secure_info(csiphy_dev, data);

		break;
	}
	case CAM_SUBDEV_MESSAGE_CONN_CSID_INFO: {
		struct cam_subdev_msg_phy_conn_csid_info *conn_csid_info =
			(struct cam_subdev_msg_phy_conn_csid_info *) data;
		int idx;
		struct cam_csiphy_param *param;

		idx = cam_csiphy_get_session_index(csiphy_dev, conn_csid_info->lane_cfg);
		if (idx >= CSIPHY_MAX_INSTANCES_PER_PHY) {
			CAM_ERR(CAM_CSIPHY, "Phy session not found %d %d",
				csiphy_dev->soc_info.index, conn_csid_info->lane_cfg);
			break;
		}

		param = &csiphy_dev->csiphy_info[idx];
		param->conn_csid_idx = conn_csid_info->core_idx;

		CAM_DBG(CAM_CSIPHY, "PHY: %d, CSID: %d connected", csiphy_dev->soc_info.index,
			param->conn_csid_idx);
		break;
	}
	case CAM_SUBDEV_MESSAGE_DRV_INFO: {
		struct cam_subdev_msg_phy_drv_info *drv_info =
			(struct cam_subdev_msg_phy_drv_info *) data;
		int idx;
		struct cam_csiphy_param *param;

		idx = cam_csiphy_get_session_index(csiphy_dev, drv_info->lane_cfg);
		if (idx >= CSIPHY_MAX_INSTANCES_PER_PHY) {
			CAM_ERR(CAM_CSIPHY, "Phy session not found %d %d",
				csiphy_dev->soc_info.index, drv_info->lane_cfg);
			break;
		}

		param = &csiphy_dev->csiphy_info[idx];
		param->is_drv_config_en = drv_info->is_drv_config_en;
		param->use_hw_client_voting = drv_info->use_hw_client_voting;

		CAM_DBG(CAM_CSIPHY,
			"PHY: %d, CSID: %d DRV info : use hw client %d, enable drv config %d",
			csiphy_dev->soc_info.index, param->conn_csid_idx,
			param->use_hw_client_voting, param->is_drv_config_en);

		break;
	}
	case CAM_SUBDEV_MESSAGE_NOTIFY_HALT_RESUME: {
		int drv_idx;
		struct cam_subdev_msg_phy_halt_resume_info *halt_resume_info =
			(struct cam_subdev_msg_phy_halt_resume_info *) data;
		int idx;
		struct cam_csiphy_param *param;
		unsigned long clk_rate;

		idx = cam_csiphy_get_session_index(csiphy_dev, halt_resume_info->lane_cfg);
		if (idx >= CSIPHY_MAX_INSTANCES_PER_PHY) {
			CAM_ERR(CAM_CSIPHY, "Phy session not found %d %d",
				csiphy_dev->soc_info.index, halt_resume_info->lane_cfg);
			break;
		}

		param = &csiphy_dev->csiphy_info[idx];
		drv_idx = param->conn_csid_idx;

		if (!csiphy_dev->soc_info.is_clk_drv_en || !param->use_hw_client_voting ||
			!param->is_drv_config_en)
			break;

		CAM_DBG(CAM_CSIPHY,
			"PHY: %d, CSID: %d DRV info : use hw client %d, enable drv config %d, op=%s",
			csiphy_dev->soc_info.index, param->conn_csid_idx,
			param->use_hw_client_voting, param->is_drv_config_en,
			(halt_resume_info->csid_state == CAM_SUBDEV_PHY_CSID_HALT) ? "HALT" :
			"RESUME");

		if (halt_resume_info->csid_state == CAM_SUBDEV_PHY_CSID_HALT) {
			clk_rate =
				csiphy_dev->soc_info.applied_src_clk_rates.hw_client[drv_idx].high;

			rc = cam_soc_util_set_src_clk_rate(&csiphy_dev->soc_info,
				CAM_CLK_SW_CLIENT_IDX, clk_rate, 0);
			if (rc)
				CAM_ERR(CAM_CSIPHY,
					"PHY[%d] CSID HALT: csiphy set_rate %ld failed rc: %d",
					phy_idx, clk_rate, rc);
		} else if (halt_resume_info->csid_state == CAM_SUBDEV_PHY_CSID_RESUME) {
			int32_t src_idx = csiphy_dev->soc_info.src_clk_idx;
			uint32_t lowest_clk_level = csiphy_dev->soc_info.lowest_clk_level;

			clk_rate = csiphy_dev->soc_info.clk_rate[lowest_clk_level][src_idx];

			rc = cam_soc_util_set_src_clk_rate(&csiphy_dev->soc_info,
				CAM_CLK_SW_CLIENT_IDX, clk_rate, 0);
			if (rc)
				CAM_ERR(CAM_CSIPHY,
					"PHY[%d] CSID RESUME: csiphy _set_rate %ld failed rc: %d",
					phy_idx, clk_rate, rc);
		} else {
			CAM_ERR(CAM_CSIPHY,
				"CSIPHY:%d Failed to handle CSID halt resume csid_state: %d",
				phy_idx, halt_resume_info->csid_state);
		}
		break;
	}
	default:
		break;
	}
}

static int cam_csiphy_debug_register(struct csiphy_device *csiphy_dev)
{
	struct dentry *dbgfileptr = NULL;
	char debugfs_name[CSIPHY_DEBUGFS_NAME_MAX_SIZE];

	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "null CSIPHY dev ptr");
		return -EINVAL;
	}

	if (!cam_debugfs_available())
		return 0;

	if (!root_dentry) {
		if (cam_debugfs_create_subdir("csiphy", &dbgfileptr)) {
			CAM_ERR(CAM_CSIPHY,
				"Debugfs could not create directory!");
			return -ENOENT;
		}
		root_dentry = dbgfileptr;
	}

	/* Create the CSIPHY directory for this csiphy */
	snprintf(debugfs_name, CSIPHY_DEBUGFS_NAME_MAX_SIZE, "CSIPHY%d",
		csiphy_dev->soc_info.index);
	dbgfileptr = debugfs_create_dir(debugfs_name, root_dentry);
	if (IS_ERR(dbgfileptr)) {
		CAM_ERR(CAM_CSIPHY, "Could not create a debugfs PHY indx subdirectory. rc: %ld",
			dbgfileptr);
		return -ENOENT;
	}

	debugfs_create_bool("en_common_status_reg_dump", 0644,
		dbgfileptr, &csiphy_dev->en_common_status_reg_dump);

	debugfs_create_bool("en_lane_status_reg_dump", 0644,
		dbgfileptr, &csiphy_dev->en_lane_status_reg_dump);

	debugfs_create_bool("en_full_phy_reg_dump", 0644,
		dbgfileptr, &csiphy_dev->en_full_phy_reg_dump);

	debugfs_create_bool("skip_aux_settings", 0644,
		dbgfileptr, &csiphy_dev->skip_aux_settings);

	return 0;
}

static void cam_csiphy_debug_unregister(void)
{
	root_dentry = NULL;
}

static int cam_csiphy_subdev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct csiphy_device *csiphy_dev =
		v4l2_get_subdevdata(sd);

	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "csiphy_dev ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&csiphy_dev->mutex);
	cam_csiphy_shutdown(csiphy_dev);
	mutex_unlock(&csiphy_dev->mutex);

	return 0;
}

static int cam_csiphy_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open();

	if (crm_active) {
		CAM_DBG(CAM_CSIPHY, "CRM is ACTIVE, close should be from CRM");
		return 0;
	}

	return cam_csiphy_subdev_close_internal(sd, fh);
}

static long cam_csiphy_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	struct csiphy_device *csiphy_dev = v4l2_get_subdevdata(sd);
	int rc = 0;

	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "csiphy_dev ptr is NULL");
		return -EINVAL;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_csiphy_core_cfg(csiphy_dev, arg);
		if (rc)
			CAM_ERR(CAM_CSIPHY,
				"Failed in configuring the device: %d", rc);
		break;
	case CAM_SD_SHUTDOWN:
		if (!cam_req_mgr_is_shutdown()) {
			CAM_ERR(CAM_CORE, "SD shouldn't come from user space");
			return 0;
		}

		rc = cam_csiphy_subdev_close_internal(sd, NULL);
		break;
	default:
		CAM_ERR_RATE_LIMIT(CAM_CSIPHY, "Wrong ioctl : %d", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}

#ifdef CONFIG_COMPAT
static long cam_csiphy_subdev_compat_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	int32_t rc = 0;
	struct cam_control cmd_data;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_CSIPHY, "Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	/* All the arguments converted to 64 bit here
	 * Passed to the api in core.c
	 */
	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_csiphy_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc)
			CAM_ERR(CAM_CSIPHY,
				"Failed in subdev_ioctl: %d", rc);
		break;
	default:
		CAM_ERR(CAM_CSIPHY, "Invalid compat ioctl cmd: %d", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_CSIPHY,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}

	return rc;
}
#endif

static struct v4l2_subdev_core_ops csiphy_subdev_core_ops = {
	.ioctl = cam_csiphy_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_csiphy_subdev_compat_ioctl,
#endif
};

static const struct v4l2_subdev_ops csiphy_subdev_ops = {
	.core = &csiphy_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops csiphy_subdev_intern_ops = {
	.close = cam_csiphy_subdev_close,
};

static int cam_csiphy_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct cam_cpas_register_params cpas_parms;
	struct csiphy_device *new_csiphy_dev;
	int32_t               rc = 0;
	struct platform_device *pdev = to_platform_device(dev);
	char wq_name[32];
	int i;

	new_csiphy_dev = devm_kzalloc(&pdev->dev,
		sizeof(struct csiphy_device), GFP_KERNEL);
	if (!new_csiphy_dev)
		return -ENOMEM;

	mutex_init(&new_csiphy_dev->mutex);
	new_csiphy_dev->v4l2_dev_str.pdev = pdev;

	new_csiphy_dev->soc_info.pdev = pdev;
	new_csiphy_dev->soc_info.dev = &pdev->dev;
	new_csiphy_dev->soc_info.dev_name = pdev->name;
	new_csiphy_dev->ref_count = 0;
	new_csiphy_dev->current_data_rate = 0;

	rc = cam_csiphy_parse_dt_info(pdev, new_csiphy_dev);
	if (rc < 0) {
		CAM_ERR(CAM_CSIPHY, "DT parsing failed: %d", rc);
		goto csiphy_no_resource;
	}

	/* validate PHY fuse only for CSIPHY4 */
	if ((new_csiphy_dev->soc_info.index == 4) &&
		!cam_cpas_is_feature_supported(
			CAM_CPAS_CSIPHY_FUSE,
			(1 << new_csiphy_dev->soc_info.index), NULL)) {
		CAM_ERR(CAM_CSIPHY, "PHY%d is not supported",
			new_csiphy_dev->soc_info.index);
		goto csiphy_no_resource;
	}

	if (cam_cpas_query_domain_id_security_support())
		new_csiphy_dev->domain_id_security = true;

	new_csiphy_dev->v4l2_dev_str.internal_ops =
		&csiphy_subdev_intern_ops;
	new_csiphy_dev->v4l2_dev_str.ops =
		&csiphy_subdev_ops;
	snprintf(new_csiphy_dev->device_name,
		CAM_CTX_DEV_NAME_MAX_LENGTH,
		"%s%d", CAMX_CSIPHY_DEV_NAME,
		new_csiphy_dev->soc_info.index);
	new_csiphy_dev->v4l2_dev_str.name =
		new_csiphy_dev->device_name;
	new_csiphy_dev->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	new_csiphy_dev->v4l2_dev_str.ent_function =
		CAM_CSIPHY_DEVICE_TYPE;
	new_csiphy_dev->v4l2_dev_str.msg_cb =
		cam_csiphy_subdev_handle_message;
	new_csiphy_dev->v4l2_dev_str.token =
		new_csiphy_dev;
	new_csiphy_dev->v4l2_dev_str.close_seq_prior =
		CAM_SD_CLOSE_MEDIUM_PRIORITY;

	rc = cam_register_subdev(&(new_csiphy_dev->v4l2_dev_str));
	if (rc < 0) {
		CAM_ERR(CAM_CSIPHY, "cam_register_subdev Failed rc: %d", rc);
		goto csiphy_no_resource;
	}

	platform_set_drvdata(pdev, &(new_csiphy_dev->v4l2_dev_str.sd));

	for (i = 0; i < CSIPHY_MAX_INSTANCES_PER_AGGREG_RX_PHY; i++) {
		new_csiphy_dev->csiphy_info[i].hdl_data.device_hdl = -1;
		new_csiphy_dev->csiphy_info[i].hdl_data.session_hdl = -1;
		new_csiphy_dev->csiphy_info[i].csiphy_3phase = -1;
		new_csiphy_dev->csiphy_info[i].data_rate = 0;
		new_csiphy_dev->csiphy_info[i].settle_time = 0;
		new_csiphy_dev->csiphy_info[i].lane_cnt = 0;
		new_csiphy_dev->csiphy_info[i].lane_assign = 0;
		new_csiphy_dev->csiphy_info[i].lane_enable = 0;
		new_csiphy_dev->csiphy_info[i].mipi_flags = 0;
		new_csiphy_dev->csiphy_info[i].conn_csid_idx = -1;
		new_csiphy_dev->csiphy_info[i].use_hw_client_voting = false;
		new_csiphy_dev->csiphy_info[i].is_drv_config_en = false;
		new_csiphy_dev->lanes_assigned[i].lane_assign = -1;
		new_csiphy_dev->lanes_assigned[i].lane_assign_cnt = 0;
	}

	new_csiphy_dev->ops.get_dev_info = NULL;
	new_csiphy_dev->ops.link_setup = NULL;
	new_csiphy_dev->ops.apply_req = NULL;

	new_csiphy_dev->acquire_count = 0;
	new_csiphy_dev->start_dev_count = 0;
	new_csiphy_dev->preamble_enable = 0;

	if (new_csiphy_dev->is_aggregator_rx)
		new_csiphy_dev->session_max_device_support =
			CSIPHY_MAX_INSTANCES_PER_AGGREG_RX_PHY;

	cpas_parms.cam_cpas_client_cb = NULL;
	cpas_parms.cell_index = new_csiphy_dev->soc_info.index;
	cpas_parms.dev = &pdev->dev;
	cpas_parms.userdata = new_csiphy_dev;

	strscpy(cpas_parms.identifier, "csiphy", CAM_HW_IDENTIFIER_LENGTH);

	rc = cam_cpas_register_client(&cpas_parms);
	if (rc) {
		CAM_ERR(CAM_CSIPHY, "CPAS registration failed rc: %d", rc);
		goto csiphy_unregister_subdev;
	}

	CAM_DBG(CAM_CSIPHY, "CPAS registration successful handle=%d",
		cpas_parms.client_handle);
	new_csiphy_dev->cpas_handle = cpas_parms.client_handle;

	snprintf(wq_name, 32, "%s%d%s", "csiphy",
		new_csiphy_dev->soc_info.index, "_wq");

	rc = cam_csiphy_register_baseaddress(new_csiphy_dev);
	if (rc) {
		CAM_ERR(CAM_CSIPHY, "Failed to register baseaddress, rc: %d", rc);
		goto cpas_unregister;
	}

	CAM_DBG(CAM_CSIPHY, "%s component bound successfully",
		pdev->name);

	cam_csiphy_debug_register(new_csiphy_dev);

	return rc;

cpas_unregister:
	cam_cpas_unregister_client(new_csiphy_dev->cpas_handle);
csiphy_unregister_subdev:
	cam_unregister_subdev(&(new_csiphy_dev->v4l2_dev_str));
csiphy_no_resource:
	mutex_destroy(&new_csiphy_dev->mutex);
	devm_kfree(&pdev->dev, new_csiphy_dev);
	return rc;
}

static void cam_csiphy_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);

	struct v4l2_subdev *subdev = NULL;
	struct csiphy_device *csiphy_dev = NULL;

	subdev = platform_get_drvdata(pdev);

	if (!subdev) {
		CAM_ERR(CAM_CSIPHY, "Error No data in subdev");
		return;
	}

	csiphy_dev = v4l2_get_subdevdata(subdev);

	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "Error No data in csiphy_dev");
		return;
	}

	if (!csiphy_dev) {
		CAM_ERR(CAM_CSIPHY, "csiphy_dev ptr is NULL");
		return;
	}

	cam_csiphy_debug_unregister();
	CAM_INFO(CAM_CSIPHY, "Unbind CSIPHY component");
	cam_cpas_unregister_client(csiphy_dev->cpas_handle);
	cam_csiphy_soc_release(csiphy_dev);
	mutex_lock(&csiphy_dev->mutex);
	cam_csiphy_shutdown(csiphy_dev);
	mutex_unlock(&csiphy_dev->mutex);
	cam_unregister_subdev(&(csiphy_dev->v4l2_dev_str));
	platform_set_drvdata(pdev, NULL);
	v4l2_set_subdevdata(&(csiphy_dev->v4l2_dev_str.sd), NULL);
	devm_kfree(&pdev->dev, csiphy_dev);
}

const static struct component_ops cam_csiphy_component_ops = {
	.bind = cam_csiphy_component_bind,
	.unbind = cam_csiphy_component_unbind,
};

static int32_t cam_csiphy_platform_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_CSIPHY, "Adding CSIPHY component");
	rc = component_add(&pdev->dev, &cam_csiphy_component_ops);
	if (rc)
		CAM_ERR(CAM_CSIPHY, "failed to add component rc: %d", rc);

	return rc;
}


static int32_t cam_csiphy_device_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_csiphy_component_ops);
	return 0;
}

static const struct of_device_id cam_csiphy_dt_match[] = {
	{.compatible = "qcom,csiphy"},
	{}
};

MODULE_DEVICE_TABLE(of, cam_csiphy_dt_match);

struct platform_driver csiphy_driver = {
	.probe = cam_csiphy_platform_probe,
	.remove = cam_csiphy_device_remove,
	.driver = {
		.name = CAMX_CSIPHY_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cam_csiphy_dt_match,
		.suppress_bind_attrs = true,
	},
};

int32_t cam_csiphy_init_module(void)
{
	return platform_driver_register(&csiphy_driver);
}

void cam_csiphy_exit_module(void)
{
	platform_driver_unregister(&csiphy_driver);
}

MODULE_DESCRIPTION("CAM CSIPHY driver");
MODULE_LICENSE("GPL v2");
