// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/iommu.h>
#include <linux/timer.h>
#include <linux/kernel.h>

#include <media/cam_req_mgr.h>
#include "cam_isp_dev.h"
#include "cam_hw_mgr_intf.h"
#include "cam_isp_hw_mgr_intf.h"
#include "cam_node.h"
#include "cam_debug_util.h"
#include "cam_smmu_api.h"
#include "camera_main.h"
#include "cam_common_util.h"
#include "cam_context_utils.h"

static struct cam_isp_dev g_isp_dev;

static int cam_isp_dev_evt_inject_cb(void *inject_args)
{
	struct cam_common_inject_evt_param *inject_params = inject_args;
	int i;

	for (i = 0; i < g_isp_dev.max_context; i++) {
		if (g_isp_dev.ctx[i].dev_hdl == inject_params->dev_hdl) {
			cam_context_add_evt_inject(&g_isp_dev.ctx[i],
				&inject_params->evt_params);
			return 0;
		}
	}

	CAM_ERR(CAM_ISP, "No dev hdl found %d", inject_params->dev_hdl);
	return -ENODEV;
}

static void cam_isp_dev_iommu_fault_handler(struct cam_smmu_pf_info *pf_smmu_info)
{
	int i, rc;
	struct cam_node *node = NULL;
	struct cam_hw_dump_pf_args pf_args = {0};

	if (!pf_smmu_info || !pf_smmu_info->token) {
		CAM_ERR(CAM_ISP, "invalid token in page handler cb");
		return;
	}

	node = (struct cam_node *)pf_smmu_info->token;

	pf_args.pf_smmu_info = pf_smmu_info;

	for (i = 0; i < node->ctx_size; i++) {
		cam_context_dump_pf_info(&(node->ctx_list[i]), &pf_args);
		if (pf_args.pf_context_info.ctx_found)
			/* Faulted ctx found */
			break;
	}

	if (i == node->ctx_size) {
		/* Faulted ctx not found. Report PF to userspace */
		rc = cam_context_send_pf_evt(NULL, &pf_args);
		if (rc)
			CAM_ERR(CAM_ISP,
				"Failed to notify PF event to userspace rc: %d", rc);
	}
}

static void cam_isp_subdev_handle_message(
		struct v4l2_subdev *sd,
		enum cam_subdev_message_type_t message_type,
		void *data)
{
	int i, rc = 0;
	struct cam_node  *node = v4l2_get_subdevdata(sd);

	CAM_DBG(CAM_ISP, "node name %s", node->name);
	for (i = 0; i < node->ctx_size; i++) {
		rc = cam_context_handle_message(&(node->ctx_list[i]), message_type, data);
		if (rc)
			CAM_ERR(CAM_ISP, "Failed to handle message for %s", node->name);
	}
}

static const struct of_device_id cam_isp_dt_match[] = {
	{
		.compatible = "qcom,cam-isp"
	},
	{}
};

static int cam_isp_subdev_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	int rc = 0;
	cam_req_mgr_rwsem_read_op(CAM_SUBDEV_LOCK);
	mutex_lock(&g_isp_dev.isp_mutex);
	if (g_isp_dev.open_cnt >= 1) {
		CAM_ERR_RATE_LIMIT(CAM_ISP, "ISP subdev is already opened");
		rc = -EALREADY;
		goto end;
	}
	g_isp_dev.open_cnt++;
end:
	mutex_unlock(&g_isp_dev.isp_mutex);
	cam_req_mgr_rwsem_read_op(CAM_SUBDEV_UNLOCK);

	return rc;
}

static int cam_isp_subdev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	int rc = 0;
	struct cam_node *node = v4l2_get_subdevdata(sd);

	mutex_lock(&g_isp_dev.isp_mutex);
	if (g_isp_dev.open_cnt <= 0) {
		CAM_DBG(CAM_ISP, "ISP subdev is already closed");
		rc = -EINVAL;
		goto end;
	}

	g_isp_dev.open_cnt--;
	if (!node) {
		CAM_ERR(CAM_ISP, "Node ptr is NULL");
		rc = -EINVAL;
		goto end;
	}

	if (g_isp_dev.open_cnt == 0)
		cam_node_shutdown(node);

end:
	mutex_unlock(&g_isp_dev.isp_mutex);
	return rc;
}

static int cam_isp_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open();

	if (crm_active) {
		CAM_DBG(CAM_ISP, "CRM is ACTIVE, close should be from CRM");
		return 0;
	}
	return cam_isp_subdev_close_internal(sd, fh);
}

static const struct v4l2_subdev_internal_ops cam_isp_subdev_internal_ops = {
	.close = cam_isp_subdev_close,
	.open = cam_isp_subdev_open,
};

static int cam_isp_dev_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int rc = -1;
	int i;
	struct cam_hw_mgr_intf         hw_mgr_intf;
	struct cam_node               *node;
	const char                    *compat_str = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	int iommu_hdl = -1;

	of_property_read_string_index(pdev->dev.of_node, "arch-compat", 0,
		(const char **)&compat_str);

	g_isp_dev.sd.internal_ops = &cam_isp_subdev_internal_ops;
	g_isp_dev.sd.close_seq_prior = CAM_SD_CLOSE_HIGH_PRIORITY;
	/* Initialize the v4l2 subdevice first. (create cam_node) */
	if (strnstr(compat_str, "ife", strlen(compat_str))) {
		rc = cam_subdev_probe(&g_isp_dev.sd, pdev, CAM_ISP_DEV_NAME,
		CAM_IFE_DEVICE_TYPE);
		g_isp_dev.isp_device_type = CAM_IFE_DEVICE_TYPE;
		g_isp_dev.max_context = CAM_CTX_MAX;
	} else if (strnstr(compat_str, "mc_tfe", strlen(compat_str))) {
		rc  = cam_subdev_probe(&g_isp_dev.sd, pdev, CAM_ISP_DEV_NAME,
		CAM_TFE_MC_DEVICE_TYPE);
		g_isp_dev.isp_device_type = CAM_TFE_MC_DEVICE_TYPE;
		g_isp_dev.max_context = CAM_CTX_MAX;
	} else if (strnstr(compat_str, "tfe", strlen(compat_str))) {
		rc = cam_subdev_probe(&g_isp_dev.sd, pdev, CAM_ISP_DEV_NAME,
		CAM_TFE_DEVICE_TYPE);
		g_isp_dev.sd.msg_cb = cam_isp_subdev_handle_message;
		g_isp_dev.isp_device_type = CAM_TFE_DEVICE_TYPE;
		g_isp_dev.max_context = CAM_TFE_CTX_MAX;
	} else  {
		CAM_ERR(CAM_ISP, "Invalid ISP hw type %s", compat_str);
		rc = -EINVAL;
		goto err;
	}

	if (rc) {
		CAM_ERR(CAM_ISP, "ISP cam_subdev_probe failed!");
		goto err;
	}
	node = (struct cam_node *) g_isp_dev.sd.token;

	memset(&hw_mgr_intf, 0, sizeof(hw_mgr_intf));
	g_isp_dev.ctx = kcalloc(g_isp_dev.max_context,
		sizeof(struct cam_context),
		GFP_KERNEL);
	if (!g_isp_dev.ctx) {
		CAM_ERR(CAM_ISP,
			"Mem Allocation failed for ISP base context");
		goto unregister;
	}

	g_isp_dev.ctx_isp = kcalloc(g_isp_dev.max_context,
		sizeof(struct cam_isp_context),
		GFP_KERNEL);
	if (!g_isp_dev.ctx_isp) {
		CAM_ERR(CAM_ISP,
			"Mem Allocation failed for Isp private context");
		kfree(g_isp_dev.ctx);
		g_isp_dev.ctx = NULL;
		goto unregister;
	}

	rc = cam_isp_hw_mgr_init(compat_str, &hw_mgr_intf, &iommu_hdl,
		g_isp_dev.isp_device_type);
	if (rc != 0) {
		CAM_ERR(CAM_ISP, "Can not initialized ISP HW manager!");
		goto free_mem;
	}

	for (i = 0; i < g_isp_dev.max_context; i++) {
		rc = cam_isp_context_init(&g_isp_dev.ctx_isp[i],
			&g_isp_dev.ctx[i],
			&node->crm_node_intf,
			&node->hw_mgr_intf,
			i,
			g_isp_dev.isp_device_type, iommu_hdl);
		if (rc) {
			CAM_ERR(CAM_ISP, "ISP context init failed!");
			goto free_mem;
		}
	}

	if (g_isp_dev.isp_device_type == CAM_IFE_DEVICE_TYPE)
		cam_common_register_evt_inject_cb(cam_isp_dev_evt_inject_cb,
			CAM_COMMON_EVT_INJECT_HW_IFE);
	else
		cam_common_register_evt_inject_cb(cam_isp_dev_evt_inject_cb,
			CAM_COMMON_EVT_INJECT_HW_TFE);

	rc = cam_node_init(node, &hw_mgr_intf, g_isp_dev.ctx,
			g_isp_dev.max_context, CAM_ISP_DEV_NAME);

	if (rc) {
		CAM_ERR(CAM_ISP, "ISP node init failed!");
		goto free_mem;
	}

	node->sd_handler = cam_isp_subdev_close_internal;
	cam_smmu_set_client_page_fault_handler(iommu_hdl,
		cam_isp_dev_iommu_fault_handler, node);

	mutex_init(&g_isp_dev.isp_mutex);

	rc = cam_subdev_register(&g_isp_dev.sd, pdev);
	CAM_DBG(CAM_ISP, "Component bound successfully");

	return rc;

free_mem:
	kfree(g_isp_dev.ctx);
	g_isp_dev.ctx = NULL;
	kfree(g_isp_dev.ctx_isp);
	g_isp_dev.ctx_isp = NULL;

unregister:
	rc = cam_subdev_remove(&g_isp_dev.sd);
err:
	return rc;
}

static void cam_isp_dev_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	int rc = 0;
	int i;
	const char *compat_str = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	of_property_read_string_index(pdev->dev.of_node, "arch-compat", 0,
		(const char **)&compat_str);

	cam_isp_hw_mgr_deinit(compat_str);
	/* clean up resources */
	for (i = 0; i < g_isp_dev.max_context; i++) {
		rc = cam_isp_context_deinit(&g_isp_dev.ctx_isp[i]);
		if (rc)
			CAM_ERR(CAM_ISP, "ISP context %d deinit failed",
				 i);
	}

	kfree(g_isp_dev.ctx);
	g_isp_dev.ctx = NULL;
	kfree(g_isp_dev.ctx_isp);
	g_isp_dev.ctx_isp = NULL;

	rc = cam_unregister_subdev(&g_isp_dev.sd);
	if (rc)
		CAM_ERR(CAM_ISP, "Unregister failed rc: %d", rc);

	rc = cam_subdev_remove(&g_isp_dev.sd);
	if (rc)
		CAM_ERR(CAM_ISP, "Subdev remove rc: %d", rc);

	memset(&g_isp_dev, 0, sizeof(g_isp_dev));
}

const static struct component_ops cam_isp_dev_component_ops = {
	.bind = cam_isp_dev_component_bind,
	.unbind = cam_isp_dev_component_unbind,
};

static int cam_isp_dev_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_isp_dev_component_ops);
	return 0;
}

static int cam_isp_dev_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_ISP, "Adding ISP dev component");
	rc = component_add(&pdev->dev, &cam_isp_dev_component_ops);
	if (rc)
		CAM_ERR(CAM_ISP, "failed to add component rc: %d", rc);

	return rc;
}

struct platform_driver isp_driver = {
	.probe = cam_isp_dev_probe,
	.remove = cam_isp_dev_remove,
	.driver = {
		.name = "cam_isp",
		.owner = THIS_MODULE,
		.of_match_table = cam_isp_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_isp_dev_init_module(void)
{
	return platform_driver_register(&isp_driver);
}

void cam_isp_dev_exit_module(void)
{
	platform_driver_unregister(&isp_driver);
}

MODULE_DESCRIPTION("MSM ISP driver");
MODULE_LICENSE("GPL v2");
