// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/cam_cpas.h>
#include <media/cam_req_mgr.h>

#include <dt-bindings/msm-camera.h>

#include "cam_subdev.h"
#include "cam_cpas_hw_intf.h"
#include "cam_cpas_soc.h"
#include "cam_cpastop_hw.h"
#include "camera_main.h"
#include "cam_cpas_hw.h"

#include <linux/soc/qcom/llcc-qcom.h>
#include "cam_req_mgr_interface.h"

#ifdef CONFIG_DYNAMIC_FD_PORT_CONFIG
#include <linux/IClientEnv.h>
#include <linux/ITrustedCameraDriver.h>
#include <linux/CTrustedCameraDriver.h>
#define CAM_CPAS_ERROR_NOT_ALLOWED 10
#endif

#define CAM_CPAS_DEV_NAME    "cam-cpas"
#define CAM_CPAS_INTF_INITIALIZED() (g_cpas_intf && g_cpas_intf->probe_done)

/**
 * struct cam_cpas_intf : CPAS interface
 *
 * @pdev: Platform device
 * @subdev: Subdev info
 * @hw_intf: CPAS HW interface
 * @hw_caps: CPAS HW capabilities
 * @intf_lock: CPAS interface mutex
 * @open_cnt: CPAS subdev open count
 * @probe_done: Whether CPAS prove completed
 *
 */
struct cam_cpas_intf {
	struct platform_device *pdev;
	struct cam_subdev subdev;
	struct cam_hw_intf *hw_intf;
	struct cam_cpas_hw_caps hw_caps;
	struct mutex intf_lock;
	uint32_t open_cnt;
	bool probe_done;
};

static struct cam_cpas_intf *g_cpas_intf;

const char *cam_cpas_axi_util_path_type_to_string(
	uint32_t path_data_type)
{
	switch (path_data_type) {
	/* IFE Paths */
	case CAM_AXI_PATH_DATA_IFE_LINEAR:
		return "IFE_LINEAR";
	case CAM_AXI_PATH_DATA_IFE_VID:
		return "IFE_VID";
	case CAM_AXI_PATH_DATA_IFE_DISP:
		return "IFE_DISP";
	case CAM_AXI_PATH_DATA_IFE_STATS:
		return "IFE_STATS";
	case CAM_AXI_PATH_DATA_IFE_RDI0:
		return "IFE_RDI0";
	case CAM_AXI_PATH_DATA_IFE_RDI1:
		return "IFE_RDI1";
	case CAM_AXI_PATH_DATA_IFE_RDI2:
		return "IFE_RDI2";
	case CAM_AXI_PATH_DATA_IFE_RDI3:
		return "IFE_RDI3";
	case CAM_AXI_PATH_DATA_IFE_PDAF:
		return "IFE_PDAF";
	case CAM_AXI_PATH_DATA_IFE_PIXEL_RAW:
		return "IFE_PIXEL_RAW";

	/* IPE Paths */
	case CAM_AXI_PATH_DATA_IPE_RD_IN:
		return "IPE_RD_IN";
	case CAM_AXI_PATH_DATA_IPE_RD_REF:
		return "IPE_RD_REF";
	case CAM_AXI_PATH_DATA_IPE_WR_VID:
		return "IPE_WR_VID";
	case CAM_AXI_PATH_DATA_IPE_WR_DISP:
		return "IPE_WR_DISP";
	case CAM_AXI_PATH_DATA_IPE_WR_REF:
		return "IPE_WR_REF";
	case CAM_AXI_PATH_DATA_IPE_WR_APP:
		return "IPE_WR_APP";

	/* OPE Paths */
	case CAM_AXI_PATH_DATA_OPE_RD_IN:
		return "OPE_RD_IN";
	case CAM_AXI_PATH_DATA_OPE_RD_REF:
		return "OPE_RD_REF";
	case CAM_AXI_PATH_DATA_OPE_WR_VID:
		return "OPE_WR_VID";
	case CAM_AXI_PATH_DATA_OPE_WR_DISP:
		return "OPE_WR_DISP";
	case CAM_AXI_PATH_DATA_OPE_WR_REF:
		return "OPE_WR_REF";

	/* SFE Paths */
	case CAM_AXI_PATH_DATA_SFE_NRDI:
		return "SFE_NRDI";
	case CAM_AXI_PATH_DATA_SFE_RDI0:
		return "SFE_RDI0";
	case CAM_AXI_PATH_DATA_SFE_RDI1:
		return "SFE_RDI1";
	case CAM_AXI_PATH_DATA_SFE_RDI2:
		return "SFE_RDI2";
	case CAM_AXI_PATH_DATA_SFE_RDI3:
		return "SFE_RDI3";
	case CAM_AXI_PATH_DATA_SFE_RDI4:
		return "SFE_RDI4";
	case CAM_AXI_PATH_DATA_SFE_STATS:
		return "SFE_STATS";
	case CAM_AXI_PATH_DATA_CRE_RD_IN:
		return "CRE_RD_IN";
	case CAM_AXI_PATH_DATA_CRE_WR_OUT:
		return "CRE_WR_OUT";

	/* OFE Paths */
	case CAM_AXI_PATH_DATA_OFE_RD_EXT:
		return "OFE_RD_EXT";
	case CAM_AXI_PATH_DATA_OFE_RD_INT_PDI:
		return "OFE_RD_INT_PDI";
	case CAM_AXI_PATH_DATA_OFE_RD_INT_HDR:
		return "OFE_RD_INT_HDR";
	case CAM_AXI_PATH_DATA_OFE_WR_VID:
		return "OFE_WR_VID";
	case CAM_AXI_PATH_DATA_OFE_WR_DISP:
		return "OFE_WR_DISP";
	case CAM_AXI_PATH_DATA_OFE_WR_IR:
		return "OFE_WR_IR";
	case CAM_AXI_PATH_DATA_OFE_WR_HDR_LTM:
		return "OFE_WR_HDR_LTM";
	case CAM_AXI_PATH_DATA_OFE_WR_DC4:
		return "OFE_WR_DC4";
	case CAM_AXI_PATH_DATA_OFE_WR_AI:
		return "OFE_WR_AI";
	case CAM_AXI_PATH_DATA_OFE_WR_PDI:
		return "OFE_WR_PDI";
	case CAM_AXI_PATH_DATA_OFE_WR_IDEALRAW:
		return "OFE_WR_IDEALRAW";
	case CAM_AXI_PATH_DATA_OFE_WR_STATS:
		return "OFE_WR_STATS";

	/* Common Paths */
	case CAM_AXI_PATH_DATA_ALL:
		return "DATA_ALL";
	default:
		return "CPAS_PATH_INVALID";
	}
}
EXPORT_SYMBOL(cam_cpas_axi_util_path_type_to_string);

const char *cam_cpas_axi_util_trans_type_to_string(
	uint32_t transac_type)
{
	switch (transac_type) {
	case CAM_AXI_TRANSACTION_READ:
		return "TRANSAC_READ";
	case CAM_AXI_TRANSACTION_WRITE:
		return "TRANSAC_WRITE";
	default:
		return "TRANSAC_INVALID";
	}
}
EXPORT_SYMBOL(cam_cpas_axi_util_trans_type_to_string);

const char *cam_cpas_axi_util_drv_vote_lvl_to_string(
	uint32_t vote_lvl)
{
	switch (vote_lvl) {
	case CAM_CPAS_VOTE_LEVEL_LOW:
		return "VOTE_LVL_LOW";
	case CAM_CPAS_VOTE_LEVEL_HIGH:
		return "VOTE_LVL_HIGH";
	default:
		return "VOTE_LVL_INVALID";
	}
}
EXPORT_SYMBOL(cam_cpas_axi_util_drv_vote_lvl_to_string);

const char *cam_cpas_util_vote_type_to_string(enum cam_cpas_vote_type vote_type)
{
	switch (vote_type) {
	case CAM_CPAS_VOTE_TYPE_HLOS:
		return "VOTE_TYPE_HLOS";
	case CAM_CPAS_VOTE_TYPE_DRV:
		return "VOTE_TYPE_DRV";
	default:
		return "VOTE_TYPE_INVALID";
	}
}
EXPORT_SYMBOL(cam_cpas_util_vote_type_to_string);

int cam_cpas_query_drv_enable(bool *is_ddr_drv_enabled, bool *is_clk_drv_enabled)
{
	struct cam_hw_info *cpas_hw = NULL;
	struct cam_cpas_private_soc *soc_private = NULL;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (!is_ddr_drv_enabled && !is_clk_drv_enabled) {
		CAM_ERR(CAM_CPAS, "invalid input ddr: %pK clk: %pK", is_ddr_drv_enabled,
			is_clk_drv_enabled);
		return -EINVAL;
	}

	cpas_hw = (struct cam_hw_info  *) g_cpas_intf->hw_intf->hw_priv;
	soc_private = (struct cam_cpas_private_soc *) cpas_hw->soc_info.soc_private;

	if (is_ddr_drv_enabled)
		*is_ddr_drv_enabled = soc_private->enable_cam_ddr_drv;

	if (is_clk_drv_enabled)
		*is_clk_drv_enabled = soc_private->enable_cam_clk_drv;

	return 0;
}
EXPORT_SYMBOL(cam_cpas_query_drv_enable);

int cam_cpas_csid_process_resume(uint32_t csid_idx)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_CSID_PROCESS_RESUME, &csid_idx,
			sizeof(uint32_t));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_csid_process_resume);


int cam_cpas_csid_input_core_info_update(int csid_idx, int sfe_idx, bool set_port)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		struct cam_cpas_hw_cmd_csid_input_core_info_update core_info_update;

		core_info_update.csid_idx = csid_idx;
		core_info_update.sfe_idx = sfe_idx;
		core_info_update.set_port = set_port;
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_CSID_INPUT_CORE_INFO_UPDATE, &core_info_update,
			sizeof(core_info_update));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_csid_input_core_info_update);

int cam_cpas_dump_camnoc_buff_fill_info(uint32_t client_handle)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_DUMP_BUFF_FILL_INFO, &client_handle,
			sizeof(uint32_t));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_dump_camnoc_buff_fill_info);

bool cam_cpas_is_part_supported(uint32_t flag, uint32_t hw_map, uint32_t part_info)
{
	int32_t i;
	struct cam_hw_info *cpas_hw = g_cpas_intf->hw_intf->hw_priv;
	struct cam_cpas *cpas_core = NULL;
	struct cam_cpas_subpart_info *cam_subpart_info = NULL;

	mutex_lock(&cpas_hw->hw_mutex);
	cpas_core = cpas_hw->core_info;
	cam_subpart_info = cpas_core->cam_subpart_info;

	if (!cam_subpart_info) {
		CAM_DBG(CAM_CPAS, "Invalid address of cam_subpart_info");
		mutex_unlock(&cpas_hw->hw_mutex);
		return true;
	}

	for (i = 0; i < cam_subpart_info->num_bits; i++) {
		if ((cam_subpart_info->hw_bitmap_mask[i][0] == flag) &&
				(cam_subpart_info->hw_bitmap_mask[i][1] == hw_map)) {
			CAM_DBG(CAM_CPAS, "flag: %u hw_map: %u part_info:0x%x",
					flag, hw_map, part_info);
			mutex_unlock(&cpas_hw->hw_mutex);
			return ((part_info & BIT(i)) == 0);
		}
	}

	mutex_unlock(&cpas_hw->hw_mutex);
	return true;
}

bool cam_cpas_is_feature_supported(uint32_t flag, uint32_t hw_map,
	uint32_t *fuse_val)
{
	struct cam_hw_info *cpas_hw = NULL;
	struct cam_cpas_private_soc *soc_private = NULL;
	bool supported = true;
	int32_t i;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return false;
	}

	cpas_hw = (struct cam_hw_info *) g_cpas_intf->hw_intf->hw_priv;
	soc_private =
		(struct cam_cpas_private_soc *)cpas_hw->soc_info.soc_private;

	if (flag >= CAM_CPAS_FUSE_FEATURE_MAX) {
		CAM_ERR(CAM_CPAS, "Unknown feature flag %x", flag);
		return false;
	}

	supported = cam_cpas_is_part_supported(flag, hw_map, soc_private->part_info);

	for (i = 0; i < soc_private->num_feature_info; i++)
		if (soc_private->feature_info[i].feature == flag)
			break;

	if (i == soc_private->num_feature_info)
		goto end;

	if (soc_private->feature_info[i].type == CAM_CPAS_FEATURE_TYPE_DISABLE
		|| (soc_private->feature_info[i].type ==
		CAM_CPAS_FEATURE_TYPE_ENABLE)) {
		if ((soc_private->feature_info[i].hw_map & hw_map) == hw_map) {
			if (!(supported && soc_private->feature_info[i].enable))
				supported = false;
		}
	} else {
		if (!fuse_val) {
			CAM_ERR(CAM_CPAS,
				"Invalid arg fuse_val");
		} else {
			*fuse_val = soc_private->feature_info[i].value;
		}
	}

end:
	return supported;
}
EXPORT_SYMBOL(cam_cpas_is_feature_supported);

int cam_cpas_get_cpas_hw_version(uint32_t *hw_version)
{
	struct cam_hw_info *cpas_hw = NULL;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (!hw_version) {
		CAM_ERR(CAM_CPAS, "invalid input %pK", hw_version);
		return -EINVAL;
	}

	cpas_hw = (struct cam_hw_info  *) g_cpas_intf->hw_intf->hw_priv;

	*hw_version = cpas_hw->soc_info.hw_version;

	if (*hw_version == CAM_CPAS_TITAN_NONE) {
		CAM_DBG(CAM_CPAS, "Didn't find a valid HW Version %d",
			*hw_version);
	}

	return 0;
}

int cam_cpas_get_hw_info(uint32_t *camera_family,
	struct cam_hw_version *camera_version,
	struct cam_hw_version *cpas_version,
	uint32_t **cam_caps, uint32_t *num_cap_mask,
	struct cam_cpas_fuse_info *cam_fuse_info,
	struct cam_cpas_domain_id_caps *domain_id_info)
{
	struct cam_hw_info              *cpas_hw;
	struct cam_cpas_private_soc     *soc_private;
	struct cam_cpas_domain_id_info   cpas_domain_id;
	int i;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (!camera_family || !camera_version || !cpas_version || !cam_caps || !num_cap_mask) {
		CAM_ERR(CAM_CPAS, "invalid input %pK %pK %pK %pK %pK",
			camera_family, camera_version, cpas_version, cam_caps, num_cap_mask);
		return -EINVAL;
	}

	cpas_hw = g_cpas_intf->hw_intf->hw_priv;
	soc_private = (struct cam_cpas_private_soc *)
		cpas_hw->soc_info.soc_private;

	*camera_family  = g_cpas_intf->hw_caps.camera_family;
	*camera_version = g_cpas_intf->hw_caps.camera_version;
	*cpas_version   = g_cpas_intf->hw_caps.cpas_version;
	*cam_caps       = g_cpas_intf->hw_caps.camera_capability;
	*num_cap_mask   = g_cpas_intf->hw_caps.num_capability_reg;

	if (cam_fuse_info)
		*cam_fuse_info  = g_cpas_intf->hw_caps.fuse_info;
	if (domain_id_info) {
		cpas_domain_id = soc_private->domain_id_info;

		if (!soc_private->domain_id_info.domain_id_supported) {
			domain_id_info->num_mapping = 0;
			domain_id_info->is_supported = 0;
		} else {
			domain_id_info->is_supported = 1;
			domain_id_info->num_mapping =
				soc_private->domain_id_info.num_domain_ids;

			for (i = 0; i < domain_id_info->num_mapping; i++) {
				domain_id_info->entries[i].domain_type =
					cpas_domain_id.domain_id_entries[i].domain_type;
				domain_id_info->entries[i].mapping_id =
					cpas_domain_id.domain_id_entries[i].mapping_id;
			}
		}
	}

	CAM_DBG(CAM_CPAS, "Family %d, version %d.%d cam_caps %d, domain_id: %s",
		*camera_family, camera_version->major,
		camera_version->minor, *cam_caps,
		CAM_BOOL_TO_YESNO(soc_private->domain_id_info.domain_id_supported));

	return 0;
}
EXPORT_SYMBOL(cam_cpas_get_hw_info);

static inline enum cam_cpas_reg_base __cam_cpas_get_internal_reg_base(
	enum cam_cpas_regbase_types reg_base)
{
	switch (reg_base) {
	case CAM_CPAS_REGBASE_CPASTOP:
		return CAM_CPAS_REG_CPASTOP;
	default:
		return CAM_CPAS_REG_MAX;
	}
}

int cam_cpas_reg_write(uint32_t client_handle, enum cam_cpas_regbase_types reg_base,
	uint32_t offset, bool mb, uint32_t value)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		struct cam_cpas_hw_cmd_reg_read_write cmd_reg_write;
		enum cam_cpas_reg_base internal_reg_base;

		internal_reg_base = __cam_cpas_get_internal_reg_base(reg_base);
		if (internal_reg_base >= CAM_CPAS_REG_MAX) {
			CAM_ERR(CAM_CPAS, "Invalid reg base: %d for write ops client: %u",
				reg_base, client_handle);
			return -EINVAL;
		}

		cmd_reg_write.client_handle = client_handle;
		cmd_reg_write.reg_base = internal_reg_base;
		cmd_reg_write.offset = offset;
		cmd_reg_write.value = value;
		cmd_reg_write.mb = mb;

		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_REG_WRITE, &cmd_reg_write,
			sizeof(struct cam_cpas_hw_cmd_reg_read_write));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_reg_write);

int cam_cpas_reg_read(uint32_t client_handle, enum cam_cpas_regbase_types reg_base,
	uint32_t offset, bool mb, uint32_t *value)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (!value) {
		CAM_ERR(CAM_CPAS, "Invalid arg value");
		return -EINVAL;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		struct cam_cpas_hw_cmd_reg_read_write cmd_reg_read;
		enum cam_cpas_reg_base internal_reg_base;

		internal_reg_base = __cam_cpas_get_internal_reg_base(reg_base);
		if (internal_reg_base >= CAM_CPAS_REG_MAX) {
			CAM_ERR(CAM_CPAS, "Invalid reg base: %d for read ops client: %u",
				reg_base, client_handle);
			return -EINVAL;
		}

		cmd_reg_read.client_handle = client_handle;
		cmd_reg_read.reg_base = internal_reg_base;
		cmd_reg_read.offset = offset;
		cmd_reg_read.mb = mb;
		cmd_reg_read.value = 0;

		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_REG_READ, &cmd_reg_read,
			sizeof(struct cam_cpas_hw_cmd_reg_read_write));
		if (rc) {
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
			return rc;
		}

		*value = cmd_reg_read.value;
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_reg_read);

int cam_cpas_update_axi_vote(uint32_t client_handle,
	struct cam_axi_vote *axi_vote)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (!axi_vote) {
		CAM_ERR(CAM_CPAS, "NULL axi vote");
		return -EINVAL;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		struct cam_cpas_hw_cmd_axi_vote cmd_axi_vote;

		cmd_axi_vote.client_handle = client_handle;
		cmd_axi_vote.axi_vote = axi_vote;

		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_AXI_VOTE, &cmd_axi_vote,
			sizeof(struct cam_cpas_hw_cmd_axi_vote));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_update_axi_vote);

int cam_cpas_update_ahb_vote(uint32_t client_handle,
	struct cam_ahb_vote *ahb_vote)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		struct cam_cpas_hw_cmd_ahb_vote cmd_ahb_vote;

		cmd_ahb_vote.client_handle = client_handle;
		cmd_ahb_vote.ahb_vote = ahb_vote;

		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_AHB_VOTE, &cmd_ahb_vote,
			sizeof(struct cam_cpas_hw_cmd_ahb_vote));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_update_ahb_vote);

int cam_cpas_stop(uint32_t client_handle)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.stop) {
		struct cam_cpas_hw_cmd_stop cmd_hw_stop;

		cmd_hw_stop.client_handle = client_handle;

		rc = g_cpas_intf->hw_intf->hw_ops.stop(
			g_cpas_intf->hw_intf->hw_priv, &cmd_hw_stop,
			sizeof(struct cam_cpas_hw_cmd_stop));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in stop, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid stop ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_stop);

int cam_cpas_start(uint32_t client_handle,
	struct cam_ahb_vote *ahb_vote, struct cam_axi_vote *axi_vote)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (!axi_vote) {
		CAM_ERR(CAM_CPAS, "NULL axi vote");
		return -EINVAL;
	}

	if (g_cpas_intf->hw_intf->hw_ops.start) {
		struct cam_cpas_hw_cmd_start cmd_hw_start;

		cmd_hw_start.client_handle = client_handle;
		cmd_hw_start.ahb_vote = ahb_vote;
		cmd_hw_start.axi_vote = axi_vote;

		rc = g_cpas_intf->hw_intf->hw_ops.start(
			g_cpas_intf->hw_intf->hw_priv, &cmd_hw_start,
			sizeof(struct cam_cpas_hw_cmd_start));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in start, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid start ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_start);

void cam_cpas_log_votes(bool ddr_only)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_LOG_VOTE, &ddr_only,
			sizeof(ddr_only));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
	}

}
EXPORT_SYMBOL(cam_cpas_log_votes);

int cam_cpas_select_qos_settings(uint32_t selection_mask)
{
	int rc = 0;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -EBADR;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_SELECT_QOS, &selection_mask,
			sizeof(selection_mask));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EBADR;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_select_qos_settings);

int cam_cpas_enable_tpg_mux_sel(uint32_t tpg_mux_sel)
{
	int rc = 0;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -EBADR;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_TPG_MUX_SEL, &tpg_mux_sel,
			sizeof(tpg_mux_sel));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EBADR;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_enable_tpg_mux_sel);

int cam_cpas_notify_event(const char *identifier_string,
	int32_t identifier_value)
{
	int rc = 0;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -EBADR;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		struct cam_cpas_hw_cmd_notify_event event = { 0 };

		event.identifier_string = identifier_string;
		event.identifier_value = identifier_value;

		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_LOG_EVENT, &event,
			sizeof(event));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EBADR;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_notify_event);

int cam_cpas_unregister_client(uint32_t client_handle)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_UNREGISTER_CLIENT,
			&client_handle, sizeof(uint32_t));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_unregister_client);

int cam_cpas_register_client(
	struct cam_cpas_register_params *register_params)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_REGISTER_CLIENT, register_params,
			sizeof(struct cam_cpas_register_params));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_register_client);

int cam_cpas_get_scid(
	enum cam_sys_cache_config_types type)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_GET_SCID, &type,
			sizeof(type));
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_get_scid);

int cam_cpas_prepare_subpart_info(enum cam_subparts_index idx, uint32_t num_subpart_available,
	uint32_t num_subpart_functional)
{
	struct cam_hw_info *cpas_hw = NULL;
	struct cam_cpas_private_soc *soc_private = NULL;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}
	cpas_hw = (struct cam_hw_info *) g_cpas_intf->hw_intf->hw_priv;

	mutex_lock(&cpas_hw->hw_mutex);
	soc_private = (struct cam_cpas_private_soc *)cpas_hw->soc_info.soc_private;

	if (!soc_private) {
		CAM_ERR(CAM_CPAS, "Invalid soc_private: 0x%x", soc_private);
		mutex_unlock(&cpas_hw->hw_mutex);
		return -EINVAL;
	}

	switch (idx) {
	case CAM_IFE_HW_IDX:
		soc_private->sysfs_info.num_ifes[CAM_CPAS_AVAILABLE_NUM_SUBPARTS] =
			num_subpart_available;
		soc_private->sysfs_info.num_ifes[CAM_CPAS_FUNCTIONAL_NUM_SUBPARTS] =
			num_subpart_functional;
		break;
	case CAM_IFE_LITE_HW_IDX:
		soc_private->sysfs_info.num_ife_lites[CAM_CPAS_AVAILABLE_NUM_SUBPARTS] =
			num_subpart_available;
		soc_private->sysfs_info.num_ife_lites[CAM_CPAS_FUNCTIONAL_NUM_SUBPARTS] =
			num_subpart_functional;
		break;
	case CAM_SFE_HW_IDX:
		soc_private->sysfs_info.num_sfes[CAM_CPAS_AVAILABLE_NUM_SUBPARTS] =
			num_subpart_available;
		soc_private->sysfs_info.num_sfes[CAM_CPAS_FUNCTIONAL_NUM_SUBPARTS] =
			num_subpart_functional;
		break;
	case CAM_CUSTOM_HW_IDX:
		soc_private->sysfs_info.num_custom[CAM_CPAS_AVAILABLE_NUM_SUBPARTS] =
			num_subpart_available;
		soc_private->sysfs_info.num_custom[CAM_CPAS_FUNCTIONAL_NUM_SUBPARTS] =
			num_subpart_functional;
		break;
	default:
		CAM_ERR(CAM_CPAS, "Invalid camera subpart index : %d", idx);
		mutex_unlock(&cpas_hw->hw_mutex);
		return -EINVAL;
	}

	mutex_unlock(&cpas_hw->hw_mutex);
	return 0;
}
EXPORT_SYMBOL(cam_cpas_prepare_subpart_info);

int cam_cpas_activate_llcc(
	enum cam_sys_cache_config_types type)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_ACTIVATE_LLC, &type,
			sizeof(type));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_activate_llcc);

int cam_cpas_deactivate_llcc(
	enum cam_sys_cache_config_types type)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_DEACTIVATE_LLC, &type,
			sizeof(type));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_deactivate_llcc);

int cam_cpas_configure_staling_llcc(
	enum cam_sys_cache_config_types type,
	enum cam_sys_cache_llcc_staling_mode mode_param,
	enum cam_sys_cache_llcc_staling_op_type operation_type,
	uint32_t staling_distance)
{
	int rc;
	struct cam_sys_cache_local_info sys_cache_info;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}
	if (!cam_cpas_is_notif_staling_supported())
		return -EOPNOTSUPP;

	sys_cache_info.mode = mode_param;
	sys_cache_info.op_type = operation_type;
	sys_cache_info.staling_distance
		= staling_distance;
	sys_cache_info.type = type;

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_CONFIGURE_STALING_LLC, &sys_cache_info,
			sizeof(struct cam_sys_cache_local_info));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_configure_staling_llcc);

int cam_cpas_notif_increment_staling_counter(
	enum cam_sys_cache_config_types type)
{
	int rc;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}
	if (!cam_cpas_is_notif_staling_supported())
		return -EOPNOTSUPP;

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_NOTIF_STALL_INC_LLC, &type,
			sizeof(type));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in process_cmd, rc=%d", rc);
	} else {
		CAM_ERR(CAM_CPAS, "Invalid process_cmd ops");
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_notif_increment_staling_counter);

bool cam_cpas_is_notif_staling_supported(void)
{
	#if IS_ENABLED(CONFIG_SPECTRA_LLCC_STALING)
		return true;
	#else
		return false;
	#endif
}
EXPORT_SYMBOL(cam_cpas_is_notif_staling_supported);

bool cam_cpas_query_domain_id_security_support(void)
{
	struct cam_hw_info *cpas_hw = NULL;
	struct cam_cpas_private_soc *soc_private = NULL;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return false;
	}

	cpas_hw = (struct cam_hw_info *) g_cpas_intf->hw_intf->hw_priv;
	soc_private =
		(struct cam_cpas_private_soc *)cpas_hw->soc_info.soc_private;

	return soc_private->domain_id_info.domain_id_supported;
}
EXPORT_SYMBOL(cam_cpas_query_domain_id_security_support);

int cam_cpas_enable_clks_for_domain_id(bool enable)
{
	int rc = 0;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_ENABLE_DISABLE_DOMAIN_ID_CLK, &enable,
			sizeof(enable));
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_enable_clks_for_domain_id);

int cam_cpas_dump_state_monitor_info(struct cam_req_mgr_dump_info *info)
{
	int rc = 0;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -ENODEV;
	}

	if (g_cpas_intf->hw_intf->hw_ops.process_cmd) {
		rc = g_cpas_intf->hw_intf->hw_ops.process_cmd(
			g_cpas_intf->hw_intf->hw_priv,
			CAM_CPAS_HW_CMD_DUMP_STATE_MONITOR_INFO, info,
			sizeof(*info));
	}

	return rc;
}
EXPORT_SYMBOL(cam_cpas_dump_state_monitor_info);

#ifdef CONFIG_DYNAMIC_FD_PORT_CONFIG
static int cam_cpas_handle_fd_port_config(uint32_t is_secure)
{
	int rc = 0;
	struct Object client_env, sc_object;
	struct cam_hw_info *cpas_hw = NULL;
	struct cam_cpas *cpas_core;

	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return -EINVAL;
	}

	cpas_hw = (struct cam_hw_info *) g_cpas_intf->hw_intf->hw_priv;
	if (cpas_hw) {
		cpas_core = (struct cam_cpas *) cpas_hw->core_info;
		mutex_lock(&cpas_hw->hw_mutex);
		if (cpas_core->streamon_clients > 0) {
			CAM_ERR(CAM_CPAS,
				"FD port config can not be updated during the session");
			mutex_unlock(&cpas_hw->hw_mutex);
			return -EINVAL;
		}
	} else {
		CAM_ERR(CAM_CPAS, "cpas_hw handle not initialized");
		return -EINVAL;
	}

	/* Need to vote first before enabling clocks */
	rc = cam_cpas_util_vote_default_ahb_axi(cpas_hw, true);
	if (rc) {
		CAM_ERR(CAM_CPAS,
			"failed to vote for the default ahb/axi clock, rc=%d", rc);
		goto release_mutex;
	}

	rc = cam_cpas_soc_enable_resources(&cpas_hw->soc_info,
		cpas_hw->soc_info.lowest_clk_level);
	if (rc) {
		CAM_ERR(CAM_CPAS, "failed in soc_enable_resources, rc=%d", rc);
		goto remove_default_vote;
	}

	rc = get_client_env_object(&client_env);
	if (rc) {
		CAM_ERR(CAM_CPAS, "Failed getting mink env object, rc: %d", rc);
		goto disable_resources;
	}

	rc = IClientEnv_open(client_env, CTrustedCameraDriver_UID, &sc_object);
	if (rc) {
		CAM_ERR(CAM_CPAS, "Failed getting mink sc_object, rc: %d", rc);
		goto client_release;
	}

	rc = ITrustedCameraDriver_dynamicConfigureFDPort(sc_object, is_secure);
	if (rc) {
		if (rc == CAM_CPAS_ERROR_NOT_ALLOWED) {
			CAM_ERR(CAM_CPAS, "Dynamic FD port config not allowed");
			rc = -EPERM;
		} else {
			CAM_ERR(CAM_CPAS, "Mink secure call failed, rc: %d", rc);
			rc = -EINVAL;
		}
		goto obj_release;
	}

	rc = Object_release(sc_object);
	if (rc) {
		CAM_ERR(CAM_CSIPHY, "Failed releasing secure camera object, rc: %d", rc);
		goto client_release;
	}

	rc = Object_release(client_env);
	if (rc) {
		CAM_ERR(CAM_CSIPHY, "Failed releasing mink env object, rc: %d", rc);
		goto disable_resources;
	}

	rc = cam_cpas_soc_disable_resources(&cpas_hw->soc_info, true, true);
	if (rc) {
		CAM_ERR(CAM_CPAS, "failed in soc_disable_resources, rc=%d", rc);
		goto remove_default_vote;
	}

	rc = cam_cpas_util_vote_default_ahb_axi(cpas_hw, false);
	if (rc)
		CAM_ERR(CAM_CPAS,
			"failed remove the vote on ahb/axi clock, rc=%d", rc);

	mutex_unlock(&cpas_hw->hw_mutex);
	return rc;

obj_release:
	Object_release(sc_object);
client_release:
	Object_release(client_env);
disable_resources:
	cam_cpas_soc_disable_resources(&cpas_hw->soc_info, true, true);
remove_default_vote:
	cam_cpas_util_vote_default_ahb_axi(cpas_hw, false);
release_mutex:
	mutex_unlock(&cpas_hw->hw_mutex);
	return rc;
}
#endif

static int cam_cpas_handle_custom_config_cmd(struct cam_cpas_intf *cpas_intf,
	struct cam_custom_cmd *cmd)
{
	int32_t rc = 0;

	if (!cmd) {
		CAM_ERR(CAM_CPAS, "Invalid input cmd");
		return -EINVAL;
	}

	switch (cmd->cmd_type) {
#ifdef CONFIG_DYNAMIC_FD_PORT_CONFIG
	case CAM_CPAS_CUSTOM_CMD_FD_PORT_CFG: {
		struct cam_cpas_fd_port_config cfg;

		if (cmd->size < sizeof(cfg))
			return -EINVAL;

		rc = copy_from_user(&cfg, u64_to_user_ptr(cmd->handle),
			sizeof(cfg));
		if (rc) {
			CAM_ERR(CAM_CPAS, "Failed in copy from user, rc=%d",
				rc);
			rc = -EINVAL;
			break;
		}

		rc = cam_cpas_handle_fd_port_config(cfg.is_secure);
		break;
	}
#endif
	default:
		CAM_ERR(CAM_CPAS, "Invalid custom command %d for CPAS", cmd->cmd_type);
		rc = -EINVAL;
		break;

	}

	return rc;
}

int cam_cpas_subdev_cmd(struct cam_cpas_intf *cpas_intf,
	struct cam_control *cmd)
{
	int rc = 0;
	uint32_t *camera_capability, num_cap_mask;

	if (!cmd) {
		CAM_ERR(CAM_CPAS, "Invalid input cmd");
		return -EINVAL;
	}

	switch (cmd->op_code) {
	case CAM_QUERY_CAP: {
		struct cam_cpas_query_cap query;

		rc = copy_from_user(&query, u64_to_user_ptr(cmd->handle),
			sizeof(query));
		if (rc) {
			CAM_ERR(CAM_CPAS, "Failed in copy from user, rc=%d",
				rc);
			break;
		}

		rc = cam_cpas_get_hw_info(&query.camera_family,
			&query.camera_version, &query.cpas_version,
			&camera_capability, &num_cap_mask, NULL, NULL);
		if (rc)
			break;

		query.reserved = camera_capability[0];

		rc = copy_to_user(u64_to_user_ptr(cmd->handle), &query,
			sizeof(query));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in copy to user, rc=%d", rc);

		break;
	}
	case CAM_QUERY_CAP_V2: {
		struct cam_cpas_query_cap_v2 query;

		rc = copy_from_user(&query, u64_to_user_ptr(cmd->handle),
			sizeof(query));
		if (rc) {
			CAM_ERR(CAM_CPAS, "Failed in copy from user, rc=%d",
				rc);
			break;
		}

		rc = cam_cpas_get_hw_info(&query.camera_family,
			&query.camera_version, &query.cpas_version,
			&camera_capability, &num_cap_mask,
			&query.fuse_info, NULL);
		if (rc)
			break;

		query.reserved = camera_capability[0];

		rc = copy_to_user(u64_to_user_ptr(cmd->handle), &query,
			sizeof(query));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in copy to user, rc=%d", rc);

		break;
	}
	case CAM_QUERY_CAP_V3: {
		struct cam_cpas_query_cap_v3 query;

		rc = copy_from_user(&query, u64_to_user_ptr(cmd->handle),
			sizeof(query));
		if (rc) {
			CAM_ERR(CAM_CPAS, "Failed in copy from user, rc=%d",
				rc);
			break;
		}

		rc = cam_cpas_get_hw_info(&query.camera_family,
			&query.camera_version, &query.cpas_version,
			&camera_capability, &num_cap_mask, &query.fuse_info,
			&query.domain_id_info);
		if (rc)
			break;

		query.camera_caps = camera_capability[0];

		rc = copy_to_user(u64_to_user_ptr(cmd->handle), &query,
			sizeof(query));
		if (rc)
			CAM_ERR(CAM_CPAS, "Failed in copy to user, rc=%d", rc);

		break;
	}
	case CAM_CUSTOM_DEV_CONFIG: {
		struct cam_custom_cmd custom_cmd;

		rc = copy_from_user(&custom_cmd, u64_to_user_ptr(cmd->handle),
			sizeof(custom_cmd));
		if (rc) {
			CAM_ERR(CAM_CPAS, "Failed in copy from user, rc=%d",
				rc);
			break;
		}
		rc = cam_cpas_handle_custom_config_cmd(cpas_intf, &custom_cmd);
		break;
	}
	case CAM_SD_SHUTDOWN:
		break;
	default:
		CAM_ERR(CAM_CPAS, "Unknown op code %d for CPAS", cmd->op_code);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int cam_cpas_subdev_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_cpas_intf *cpas_intf = v4l2_get_subdevdata(sd);

	if (!cpas_intf || !cpas_intf->probe_done) {
		CAM_ERR(CAM_CPAS, "CPAS not initialized");
		return -ENODEV;
	}

	mutex_lock(&cpas_intf->intf_lock);
	cpas_intf->open_cnt++;
	CAM_DBG(CAM_CPAS, "CPAS Subdev open count %d", cpas_intf->open_cnt);
	mutex_unlock(&cpas_intf->intf_lock);

	return 0;
}

static int __cam_cpas_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_cpas_intf *cpas_intf = v4l2_get_subdevdata(sd);

	if (!cpas_intf || !cpas_intf->probe_done) {
		CAM_ERR(CAM_CPAS, "CPAS not initialized");
		return -ENODEV;
	}

	mutex_lock(&cpas_intf->intf_lock);
	if (cpas_intf->open_cnt <= 0) {
		CAM_WARN(CAM_CPAS, "device already closed, open_cnt: %d", cpas_intf->open_cnt);
		mutex_unlock(&cpas_intf->intf_lock);
		return 0;
	}
	cpas_intf->open_cnt--;
	CAM_DBG(CAM_CPAS, "CPAS Subdev close count %d", cpas_intf->open_cnt);
	mutex_unlock(&cpas_intf->intf_lock);

	return 0;
}

static int cam_cpas_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open();

	if (crm_active) {
		CAM_DBG(CAM_CPAS, "CRM is ACTIVE, close should be from CRM");
		return 0;
	}

	return __cam_cpas_subdev_close(sd, fh);
}

static long cam_cpas_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int32_t rc;
	struct cam_cpas_intf *cpas_intf = v4l2_get_subdevdata(sd);

	if (!cpas_intf || !cpas_intf->probe_done) {
		CAM_ERR(CAM_CPAS, "CPAS not initialized");
		return -ENODEV;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_cpas_subdev_cmd(cpas_intf, (struct cam_control *) arg);
		break;
	case CAM_SD_SHUTDOWN:
		rc = __cam_cpas_subdev_close(sd, NULL);
		break;
	default:
		CAM_ERR_RATE_LIMIT(CAM_CPAS, "Invalid command %d for CPAS!", cmd);
		rc = -EINVAL;
		break;
	}

	return rc;
}

#ifdef CONFIG_COMPAT
static long cam_cpas_subdev_compat_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc;
	struct cam_cpas_intf *cpas_intf = v4l2_get_subdevdata(sd);

	if (!cpas_intf || !cpas_intf->probe_done) {
		CAM_ERR(CAM_CPAS, "CPAS not initialized");
		return -ENODEV;
	}

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_CPAS, "Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_cpas_subdev_cmd(cpas_intf, &cmd_data);
		break;
	default:
		CAM_ERR(CAM_CPAS, "Invalid command %d for CPAS!", cmd);
		rc = -EINVAL;
		break;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_CPAS,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}

	return rc;
}
#endif

static struct v4l2_subdev_core_ops cpas_subdev_core_ops = {
	.ioctl = cam_cpas_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_cpas_subdev_compat_ioctl,
#endif
};

static const struct v4l2_subdev_ops cpas_subdev_ops = {
	.core = &cpas_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cpas_subdev_intern_ops = {
	.open = cam_cpas_subdev_open,
	.close = cam_cpas_subdev_close,
};

static int cam_cpas_subdev_register(struct platform_device *pdev)
{
	int rc;
	struct cam_subdev *subdev;

	if (!g_cpas_intf)
		return -EINVAL;

	subdev = &g_cpas_intf->subdev;

	subdev->name = CAM_CPAS_DEV_NAME;
	subdev->pdev = pdev;
	subdev->ops = &cpas_subdev_ops;
	subdev->internal_ops = &cpas_subdev_intern_ops;
	subdev->token = g_cpas_intf;
	subdev->sd_flags =
		V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	subdev->ent_function = CAM_CPAS_DEVICE_TYPE;
	subdev->close_seq_prior = CAM_SD_CLOSE_LOW_PRIORITY;

	rc = cam_register_subdev(subdev);
	if (rc) {
		CAM_ERR(CAM_CPAS, "failed register subdev: %s!",
			CAM_CPAS_DEV_NAME);
		return rc;
	}

	platform_set_drvdata(g_cpas_intf->pdev, g_cpas_intf);
	return rc;
}

static int cam_cpas_dev_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct cam_cpas_hw_caps *hw_caps;
	struct cam_hw_intf *hw_intf;
	int rc;
	struct platform_device *pdev = to_platform_device(dev);

	if (g_cpas_intf) {
		CAM_ERR(CAM_CPAS, "cpas component already binded");
		return -EALREADY;
	}

	g_cpas_intf = kzalloc(sizeof(*g_cpas_intf), GFP_KERNEL);
	if (!g_cpas_intf)
		return -ENOMEM;

	mutex_init(&g_cpas_intf->intf_lock);
	g_cpas_intf->pdev = pdev;

	rc = cam_cpas_hw_probe(pdev, &g_cpas_intf->hw_intf);
	if (rc || (g_cpas_intf->hw_intf == NULL)) {
		CAM_ERR(CAM_CPAS, "Failed in hw probe, rc=%d", rc);
		goto error_destroy_mem;
	}

	hw_intf = g_cpas_intf->hw_intf;
	hw_caps = &g_cpas_intf->hw_caps;

	if (hw_intf->hw_ops.get_hw_caps) {
		rc = hw_intf->hw_ops.get_hw_caps(hw_intf->hw_priv,
			hw_caps, sizeof(struct cam_cpas_hw_caps));
		if (rc) {
			CAM_ERR(CAM_CPAS, "Failed in get_hw_caps, rc=%d", rc);
			goto error_hw_remove;
		}
	} else {
		CAM_ERR(CAM_CPAS, "Invalid get_hw_caps ops");
		goto error_hw_remove;
	}

	rc = cam_cpas_subdev_register(pdev);
	if (rc)
		goto error_hw_remove;

	g_cpas_intf->probe_done = true;
	CAM_DBG(CAM_CPAS,
		"Component bound successfully %d, %d.%d.%d, %d.%d.%d, 0x%x",
		hw_caps->camera_family, hw_caps->camera_version.major,
		hw_caps->camera_version.minor, hw_caps->camera_version.incr,
		hw_caps->cpas_version.major, hw_caps->cpas_version.minor,
		hw_caps->cpas_version.incr, hw_caps->camera_capability);

	return rc;

error_hw_remove:
	cam_cpas_hw_remove(g_cpas_intf->hw_intf);
error_destroy_mem:
	mutex_destroy(&g_cpas_intf->intf_lock);
	kfree(g_cpas_intf);
	g_cpas_intf = NULL;
	CAM_ERR(CAM_CPAS, "CPAS component bind failed");
	return rc;
}

static void cam_cpas_dev_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	if (!CAM_CPAS_INTF_INITIALIZED()) {
		CAM_ERR(CAM_CPAS, "cpas intf not initialized");
		return;
	}

	mutex_lock(&g_cpas_intf->intf_lock);
	g_cpas_intf->probe_done = false;
	cam_unregister_subdev(&g_cpas_intf->subdev);
	cam_cpas_hw_remove(g_cpas_intf->hw_intf);
	mutex_unlock(&g_cpas_intf->intf_lock);
	mutex_destroy(&g_cpas_intf->intf_lock);
	kfree(g_cpas_intf);
	g_cpas_intf = NULL;
}

const static struct component_ops cam_cpas_dev_component_ops = {
	.bind = cam_cpas_dev_component_bind,
	.unbind = cam_cpas_dev_component_unbind,
};

static int cam_cpas_dev_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_CPAS, "Adding CPAS INTF component");
	rc = component_add(&pdev->dev, &cam_cpas_dev_component_ops);
	if (rc)
		CAM_ERR(CAM_CPAS, "failed to add component rc: %d", rc);

	return rc;
}

static int cam_cpas_dev_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_cpas_dev_component_ops);
	return 0;
}

static const struct of_device_id cam_cpas_dt_match[] = {
	{.compatible = "qcom,cam-cpas"},
	{}
};

struct platform_driver cam_cpas_driver = {
	.probe = cam_cpas_dev_probe,
	.remove = cam_cpas_dev_remove,
	.driver = {
		.name = CAM_CPAS_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cam_cpas_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_cpas_dev_init_module(void)
{
	return platform_driver_register(&cam_cpas_driver);
}

void cam_cpas_dev_exit_module(void)
{
	platform_driver_unregister(&cam_cpas_driver);
}

MODULE_DESCRIPTION("MSM CPAS driver");
MODULE_LICENSE("GPL v2");
