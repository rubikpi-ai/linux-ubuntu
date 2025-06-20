// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include "cam_sensor_core.h"
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "cam_req_mgr_dev.h"

#define CAM_SENSOR_PIPELINE_DELAY_MASK        0xFF
#define CAM_SENSOR_MODESWITCH_DELAY_SHIFT     8

extern struct completion *cam_sensor_get_i3c_completion(uint32_t index);

static int cam_sensor_notify_v4l2_error_event(
	struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t error_type, uint32_t error_code)
{
	int                        rc = 0;
	struct cam_req_mgr_message req_msg = {0};

	req_msg.session_hdl = s_ctrl->bridge_intf.session_hdl;
	req_msg.u.err_msg.device_hdl = s_ctrl->bridge_intf.device_hdl;
	req_msg.u.err_msg.link_hdl = s_ctrl->bridge_intf.link_hdl;
	req_msg.u.err_msg.error_type = error_type;
	req_msg.u.err_msg.request_id = s_ctrl->last_applied_req;
	req_msg.u.err_msg.resource_size = 0x0;
	req_msg.u.err_msg.error_code = error_code;

	CAM_DBG(CAM_SENSOR,
		"v4l2 error event [type: %u code: %u] for req: %llu on %s",
		error_type, error_code, s_ctrl->last_applied_req,
		s_ctrl->sensor_name);

	rc = cam_req_mgr_notify_message(&req_msg,
		V4L_EVENT_CAM_REQ_MGR_ERROR,
		V4L_EVENT_CAM_REQ_MGR_EVENT);
	if (rc)
		CAM_ERR(CAM_SENSOR,
			"Notifying v4l2 error [type: %u code: %u] failed for req id:%llu on %s",
			error_type, error_code, s_ctrl->last_applied_req,
			s_ctrl->sensor_name);

	return rc;
}

static int cam_sensor_update_req_mgr(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_packet *csl_packet)
{
	int rc = 0;
	struct cam_req_mgr_add_request add_req;

	memset(&add_req, 0, sizeof(add_req));
	add_req.link_hdl = s_ctrl->bridge_intf.link_hdl;
	add_req.req_id = csl_packet->header.request_id;
	CAM_DBG(CAM_SENSOR, " Rxed Req Id: %llu",
		csl_packet->header.request_id);
	add_req.dev_hdl = s_ctrl->bridge_intf.device_hdl;
	if (s_ctrl->bridge_intf.crm_cb &&
		s_ctrl->bridge_intf.crm_cb->add_req) {
		rc = s_ctrl->bridge_intf.crm_cb->add_req(&add_req);
		if (rc) {
			if (rc == -EBADR)
				CAM_INFO(CAM_SENSOR,
					"Adding request: %llu failed with request manager rc: %d, it has been flushed",
					csl_packet->header.request_id, rc);
			else
				CAM_ERR(CAM_SENSOR,
					"Adding request: %llu failed with request manager rc: %d",
					csl_packet->header.request_id, rc);
			return rc;
		}
	}

	CAM_DBG(CAM_SENSOR, "Successfully add req: %llu to req mgr",
			add_req.req_id);
	return rc;
}

static void cam_sensor_release_stream_rsc(
	struct cam_sensor_ctrl_t *s_ctrl)
{
	struct i2c_settings_array *i2c_set = NULL;
	int rc;

	i2c_set = &(s_ctrl->i2c_data.streamoff_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting Streamoff settings");
	}

	i2c_set = &(s_ctrl->i2c_data.streamon_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting Streamon settings");
	}
}

static void cam_sensor_release_per_frame_resource(
	struct cam_sensor_ctrl_t *s_ctrl)
{
	struct i2c_settings_array *i2c_set = NULL;
	int i, rc;

	if (s_ctrl->i2c_data.per_frame != NULL) {
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			i2c_set = &(s_ctrl->i2c_data.per_frame[i]);
			if (i2c_set->is_settings_valid == 1) {
				i2c_set->is_settings_valid = -1;
				rc = delete_request(i2c_set);
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"delete per frame setting for request: %lld rc: %d",
						i2c_set->request_id, rc);
			}
		}
	}

	if (s_ctrl->i2c_data.frame_skip != NULL) {
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			i2c_set = &(s_ctrl->i2c_data.frame_skip[i]);
			if (i2c_set->is_settings_valid == 1) {
				i2c_set->is_settings_valid = -1;
				rc = delete_request(i2c_set);
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"delete frame skip setting for request: %lld rc: %d",
						i2c_set->request_id, rc);
			}
		}
	}

	if (s_ctrl->i2c_data.bubble_update != NULL) {
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			i2c_set = &(s_ctrl->i2c_data.bubble_update[i]);
			if (i2c_set->is_settings_valid == 1) {
				i2c_set->is_settings_valid = -1;
				rc = delete_request(i2c_set);
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"delete bubble update setting for request: %lld rc: %d",
						i2c_set->request_id, rc);
			}
		}
	}
}

static int cam_sensor_handle_res_info(struct cam_sensor_res_info *res_info,
	struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint32_t idx = 0;

	if (!s_ctrl || !res_info) {
		CAM_ERR(CAM_SENSOR, "Invalid params: res_info: %s, s_ctrl: %s",
			CAM_IS_NULL_TO_STR(res_info),
			CAM_IS_NULL_TO_STR(s_ctrl));
		return -EINVAL;
	}

	idx = s_ctrl->last_updated_req % MAX_PER_FRAME_ARRAY;

	s_ctrl->sensor_res[idx].res_index = res_info->res_index;
	strscpy(s_ctrl->sensor_res[idx].caps, res_info->caps,
		sizeof(s_ctrl->sensor_res[idx].caps));
	s_ctrl->sensor_res[idx].width = res_info->width;
	s_ctrl->sensor_res[idx].height = res_info->height;
	s_ctrl->sensor_res[idx].fps = res_info->fps;

	if (res_info->num_valid_params > 0) {
		if (res_info->valid_param_mask & CAM_SENSOR_FEATURE_MASK)
			s_ctrl->sensor_res[idx].feature_mask =
				res_info->params[0];

		if (res_info->valid_param_mask & CAM_SENSOR_NUM_BATCHED_FRAMES)
			s_ctrl->num_batched_frames = res_info->params[1];
	}

	s_ctrl->is_res_info_updated = true;

	/* If request id is 0, it will be during an initial config/acquire */
	CAM_INFO(CAM_SENSOR,
		"Sensor[%s-%d] Feature: 0x%x updated for request id: %lu, res index: %u, width: 0x%x, height: 0x%x, capability: %s, fps: %u",
		s_ctrl->sensor_name, s_ctrl->soc_info.index,
		s_ctrl->sensor_res[idx].feature_mask,
		s_ctrl->sensor_res[idx].request_id, s_ctrl->sensor_res[idx].res_index,
		s_ctrl->sensor_res[idx].width, s_ctrl->sensor_res[idx].height,
		s_ctrl->sensor_res[idx].caps, s_ctrl->sensor_res[idx].fps);

	return rc;
}

static int32_t cam_sensor_generic_blob_handler(void *user_data,
	uint32_t blob_type, uint32_t blob_size, uint8_t *blob_data)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl =
		(struct cam_sensor_ctrl_t *) user_data;

	if (!blob_data || !blob_size) {
		CAM_ERR(CAM_SENSOR, "Invalid blob info %pK %u", blob_data,
			blob_size);
		return -EINVAL;
	}

	switch (blob_type) {
	case CAM_SENSOR_GENERIC_BLOB_RES_INFO: {
		struct cam_sensor_res_info *res_info =
			(struct cam_sensor_res_info *) blob_data;

		if (blob_size < sizeof(struct cam_sensor_res_info)) {
			CAM_ERR(CAM_SENSOR, "Invalid blob size expected: 0x%x actual: 0x%x",
				sizeof(struct cam_sensor_res_info), blob_size);
			return -EINVAL;
		}

		rc = cam_sensor_handle_res_info(res_info, s_ctrl);
		break;
	}
	default:
		CAM_WARN(CAM_SENSOR, "Invalid blob type %d", blob_type);
		break;
	}

	return rc;
}

static int32_t cam_sensor_pkt_parse(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg)
{
	int32_t rc = 0;
	uintptr_t generic_ptr;
	struct cam_control *ioctl_ctrl = NULL;
	struct cam_packet *csl_packet = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct cam_buf_io_cfg *io_cfg = NULL;
	struct i2c_settings_array *i2c_reg_settings = NULL;
	size_t len_of_buff = 0;
	size_t remain_len = 0;
	uint32_t *offset = NULL;
	int64_t prev_updated_req;
	uint32_t cmd_buf_type, idx;
	struct cam_config_dev_cmd config;
	struct i2c_data_settings *i2c_data = NULL;

	ioctl_ctrl = (struct cam_control *)arg;

	if (ioctl_ctrl->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_SENSOR, "Invalid Handle Type");
		return -EINVAL;
	}

	if (copy_from_user(&config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(config)))
		return -EFAULT;

	rc = cam_mem_get_cpu_buf(
		config.packet_handle,
		&generic_ptr,
		&len_of_buff);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed in getting the packet: %d", rc);
		return rc;
	}

	remain_len = len_of_buff;
	if ((sizeof(struct cam_packet) > len_of_buff) ||
		((size_t)config.offset >= len_of_buff -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_SENSOR,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len_of_buff);
		rc = -EINVAL;
		goto end;
	}

	remain_len -= (size_t)config.offset;
	csl_packet = (struct cam_packet *)(generic_ptr +
		(uint32_t)config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_SENSOR, "Invalid packet params");
		rc = -EINVAL;
		goto end;
	}

	if ((csl_packet->header.op_code & 0xFFFFFF) !=
		CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG &&
		(csl_packet->header.op_code & 0xFFFFFF) !=
		CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF &&
		csl_packet->header.request_id <= s_ctrl->last_flush_req
		&& s_ctrl->last_flush_req != 0) {
		CAM_ERR(CAM_SENSOR,
			"reject request %lld, last request to flush %u",
			csl_packet->header.request_id, s_ctrl->last_flush_req);
		rc = -EBADR;
		goto end;
	}

	if (csl_packet->header.request_id > s_ctrl->last_flush_req)
		s_ctrl->last_flush_req = 0;

	prev_updated_req = s_ctrl->last_updated_req;
	s_ctrl->is_res_info_updated = false;

	i2c_data = &(s_ctrl->i2c_data);
	CAM_DBG(CAM_SENSOR, "Header OpCode: %d", csl_packet->header.op_code);
	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG: {
		i2c_reg_settings = &i2c_data->init_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG: {
		i2c_reg_settings = &i2c_data->config_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON: {
		if (s_ctrl->streamon_count > 0) {
			delete_request(&i2c_data->streamon_settings);
			s_ctrl->streamon_count = 0;
		}

		s_ctrl->streamon_count = s_ctrl->streamon_count + 1;
		i2c_reg_settings = &i2c_data->streamon_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF: {
		if (s_ctrl->streamoff_count > 0) {
			delete_request(&i2c_data->streamoff_settings);
			s_ctrl->streamoff_count = 0;
		}

		s_ctrl->streamoff_count = s_ctrl->streamoff_count + 1;
		i2c_reg_settings = &i2c_data->streamoff_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_READ: {
		i2c_reg_settings = &(i2c_data->read_settings);
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;

		CAM_DBG(CAM_SENSOR, "number of IO configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs == 0) {
			CAM_ERR(CAM_SENSOR, "No I/O configs to process");
			goto end;
		}

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload_flex +
			csl_packet->io_configs_offset);

		if (io_cfg == NULL) {
			CAM_ERR(CAM_SENSOR, "I/O config is invalid(NULL)");
			goto end;
		}
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_ACQUIRE)) {
			CAM_WARN(CAM_SENSOR,
				"Rxed Update packets without linking");
			goto end;
		}

		i2c_reg_settings =
			&i2c_data->per_frame[csl_packet->header.request_id %
				MAX_PER_FRAME_ARRAY];
		CAM_DBG(CAM_SENSOR, "Received Packet: %lld req: %lld",
			csl_packet->header.request_id % MAX_PER_FRAME_ARRAY,
			csl_packet->header.request_id);
		if (i2c_reg_settings->is_settings_valid == 1) {
			CAM_ERR(CAM_SENSOR,
				"Already some pkt in offset req : %lld",
				csl_packet->header.request_id);
			/*
			 * Update req mgr even in case of failure.
			 * This will help not to wait indefinitely
			 * and freeze. If this log is triggered then
			 * fix it.
			 */
			rc = cam_sensor_update_req_mgr(s_ctrl, csl_packet);
			if (rc)
				CAM_ERR(CAM_SENSOR,
					"Failed in adding request to req_mgr");
			goto end;
		}
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_FRAME_SKIP_UPDATE: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_ACQUIRE)) {
			CAM_WARN(CAM_SENSOR,
				"Rxed Update packets without linking");
			goto end;
		}

		i2c_reg_settings =
			&i2c_data->frame_skip[csl_packet->header.request_id %
				MAX_PER_FRAME_ARRAY];
		CAM_DBG(CAM_SENSOR, "Received not ready packet: %lld req: %lld",
			csl_packet->header.request_id % MAX_PER_FRAME_ARRAY,
			csl_packet->header.request_id);
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_BUBBLE_UPDATE: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_ACQUIRE)) {
			CAM_WARN(CAM_SENSOR,
				"Rxed Update packets without linking");
			goto end;
		}

		i2c_reg_settings =
			&i2c_data->bubble_update[csl_packet->header.request_id %
				MAX_PER_FRAME_ARRAY];
		CAM_DBG(CAM_SENSOR, "Received bubble update packet: %lld req: %lld",
			csl_packet->header.request_id % MAX_PER_FRAME_ARRAY,
			csl_packet->header.request_id);
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_NOP: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_ACQUIRE)) {
			CAM_WARN(CAM_SENSOR,
				"Rxed NOP packets without linking");
			goto end;
		}

		i2c_reg_settings =
			&i2c_data->per_frame[csl_packet->header.request_id %
				MAX_PER_FRAME_ARRAY];
		i2c_reg_settings->request_id = csl_packet->header.request_id;
		i2c_reg_settings->is_settings_valid = 1;

		rc = cam_sensor_update_req_mgr(s_ctrl, csl_packet);
		if (rc)
			CAM_ERR(CAM_SENSOR,
				"Failed in adding request to req_mgr");
		goto end;
	}
	default:
		CAM_ERR(CAM_SENSOR, "Invalid Packet Header opcode: %d",
			csl_packet->header.op_code & 0xFFFFFF);
		rc = -EINVAL;
		goto end;
	}

	offset = (uint32_t *)&csl_packet->payload_flex;
	offset += csl_packet->cmd_buf_offset / 4;
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);
	cmd_buf_type = cmd_desc->meta_data;

	switch (cmd_buf_type) {
	case CAM_SENSOR_PACKET_I2C_COMMANDS:
		rc = cam_sensor_i2c_command_parser(&s_ctrl->io_master_info,
				i2c_reg_settings, cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Fail parsing I2C Pkt: %d", rc);
			goto end;
		}
		break;
	case CAM_SENSOR_PACKET_GENERIC_BLOB:
		if ((csl_packet->header.op_code & 0xFFFFFF) !=
			CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG) {
			rc = -EINVAL;
			CAM_ERR(CAM_SENSOR, "Wrong packet opcode sent with blob: %u",
				csl_packet->header.op_code & 0xFFFFFF);
			goto end;
		}

		s_ctrl->last_updated_req = csl_packet->header.request_id;
		idx = s_ctrl->last_updated_req % MAX_PER_FRAME_ARRAY;
		s_ctrl->sensor_res[idx].request_id = csl_packet->header.request_id;

		/**
		 * is_settings_valid is set to false for this case, as generic
		 * blobs are meant to be used to send debugging information
		 * alongside actual configuration settings. As these are sent
		 * as separate packets at present, while sharing the same CONFIG
		 * opcode, setting this to false prevents sensor driver from
		 * applying non-existent configuration and changing s_ctrl
		 * state to CAM_SENSOR_CONFIG
		 */
		i2c_reg_settings->is_settings_valid = 0;

		rc = cam_packet_util_process_generic_cmd_buffer(cmd_desc,
			cam_sensor_generic_blob_handler, s_ctrl);
		if (rc)
			s_ctrl->sensor_res[idx].request_id = 0;

		break;
	}

	/*
	 * If no res info in current request, then we pick previous
	 * resolution info as current resolution info.
	 * Don't copy the sensor resolution info when the request id
	 * is invalid.
	 */
	if ((!s_ctrl->is_res_info_updated) && (csl_packet->header.request_id != 0)) {
		/*
		 * Update the last updated req at two places.
		 * 1# Got generic blob: The req id can be zero for the initial res info updating
		 * 2# Copy previous res info: The req id can't be zero, in case some queue info
		 * are override by slot0.
		 */
		s_ctrl->last_updated_req = csl_packet->header.request_id;
		s_ctrl->sensor_res[s_ctrl->last_updated_req % MAX_PER_FRAME_ARRAY] =
			s_ctrl->sensor_res[prev_updated_req % MAX_PER_FRAME_ARRAY];
	}

	if ((csl_packet->header.op_code & 0xFFFFFF) ==
		CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE) {
		i2c_reg_settings->request_id =
			csl_packet->header.request_id;
		rc = cam_sensor_update_req_mgr(s_ctrl, csl_packet);
		if (rc) {
			CAM_ERR(CAM_SENSOR,
				"Failed in adding request to req_mgr");
			goto end;
		}
	}

	if ((csl_packet->header.op_code & 0xFFFFFF) ==
		CAM_SENSOR_PACKET_OPCODE_SENSOR_FRAME_SKIP_UPDATE) {
		i2c_reg_settings->request_id =
			csl_packet->header.request_id;
	}

	if ((csl_packet->header.op_code & 0xFFFFFF) ==
		CAM_SENSOR_PACKET_OPCODE_SENSOR_BUBBLE_UPDATE) {
		i2c_reg_settings->request_id =
			csl_packet->header.request_id;
	}

end:
	cam_mem_put_cpu_buf(config.packet_handle);
	return rc;
}

static int32_t cam_sensor_restore_slave_info(struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	switch (s_ctrl->io_master_info.master_type) {
	case CCI_MASTER:
		s_ctrl->io_master_info.cci_client->sid =
			(s_ctrl->sensordata->slave_info.sensor_slave_addr >> 1);
		s_ctrl->io_master_info.cci_client->i2c_freq_mode =
			s_ctrl->sensordata->slave_info.i2c_freq_mode;
		break;

	case I2C_MASTER:
		s_ctrl->io_master_info.client->addr =
			 s_ctrl->sensordata->slave_info.sensor_slave_addr;
		break;

	case SPI_MASTER:
		break;

	default:
		CAM_ERR(CAM_SENSOR, "Invalid master type: %d",
				s_ctrl->io_master_info.master_type);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int32_t cam_sensor_update_i2c_info(struct cam_cmd_i2c_info *i2c_info,
	struct cam_sensor_ctrl_t *s_ctrl,
	bool isInit)
{
	int32_t rc = 0;
	struct cam_sensor_cci_client   *cci_client = NULL;

	switch (s_ctrl->io_master_info.master_type) {
	case CCI_MASTER:
		cci_client = s_ctrl->io_master_info.cci_client;
		if (!cci_client) {
			CAM_ERR(CAM_SENSOR, "failed: cci_client %pK",
				cci_client);
			return -EINVAL;
		}
		cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;
		cci_client->sid = i2c_info->slave_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = i2c_info->i2c_freq_mode;
		CAM_DBG(CAM_SENSOR, " Master: %d sid: 0x%x freq_mode: %d",
			cci_client->cci_i2c_master, i2c_info->slave_addr,
			i2c_info->i2c_freq_mode);
		break;

	case I2C_MASTER:
		s_ctrl->io_master_info.client->addr = i2c_info->slave_addr;
		break;

	case SPI_MASTER:
		break;

	default:
		CAM_ERR(CAM_SENSOR, "Invalid master type: %d",
			s_ctrl->io_master_info.master_type);
		rc = -EINVAL;
		break;
	}

	if (isInit) {
		s_ctrl->sensordata->slave_info.sensor_slave_addr =
			i2c_info->slave_addr;
		s_ctrl->sensordata->slave_info.i2c_freq_mode =
			i2c_info->i2c_freq_mode;
	}

	return rc;
}

static int32_t cam_sensor_i2c_modes_util(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct i2c_settings_list *i2c_list)
{
	int32_t rc = 0;
	uint32_t i, size;
	struct camera_io_master *io_master_info;

	if (s_ctrl == NULL) {
		CAM_ERR(CAM_SENSOR, "Invalid args");
		return -EINVAL;
	}

	io_master_info = &s_ctrl->io_master_info;

	if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
		rc = camera_io_dev_write(io_master_info,
			&(i2c_list->i2c_settings));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to random write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			CAM_SENSOR_I2C_WRITE_SEQ);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to seq write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_BURST) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			CAM_SENSOR_I2C_WRITE_BURST);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to burst write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
		size = i2c_list->i2c_settings.size;
		for (i = 0; i < size; i++) {
			rc = camera_io_dev_poll(
			io_master_info,
			i2c_list->i2c_settings.reg_setting[i].reg_addr,
			i2c_list->i2c_settings.reg_setting[i].reg_data,
			i2c_list->i2c_settings.reg_setting[i].data_mask,
			i2c_list->i2c_settings.addr_type,
			i2c_list->i2c_settings.data_type,
			i2c_list->i2c_settings.reg_setting[i].delay);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"i2c poll apply setting Fail: %d", rc);
				return rc;
			}
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_SET_I2C_INFO) {
		rc = cam_sensor_update_i2c_info(&i2c_list->slave_info,
			s_ctrl,
			false);
	} else if ((i2c_list->op_code == CAM_SENSOR_I2C_READ_RANDOM) ||
		(i2c_list->op_code == CAM_SENSOR_I2C_READ_SEQ)) {
		rc = cam_sensor_i2c_read_data(
			&s_ctrl->i2c_data.read_settings,
			&s_ctrl->io_master_info);
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_READ_APPEND_WRITE) {
		CAM_DBG(CAM_SENSOR, "Captured READ_APPEND_WRITE OPCODE");
		rc = camera_io_dev_read_append_write(io_master_info,
			&(i2c_list->i2c_settings));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"i2c Read_append_write settings Failed: %d", rc);
			return rc;
		}
	} else if ((i2c_list->op_code == CAM_SENSOR_I2C_SEQUENTIAL_XFER_LOCK) ||
			(i2c_list->op_code == CAM_SENSOR_I2C_SEQUENTIAL_XFER_UNLOCK)) {
		rc = camera_io_dev_sequential_xfer(io_master_info,
			&(i2c_list->seq_xfer));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"i2c Sequential Xfer settings Failed: %d", rc);
			return rc;
		}
	}

	return rc;
}

int32_t cam_sensor_update_slave_info(void *probe_info,
	uint32_t cmd, struct cam_sensor_ctrl_t *s_ctrl, uint8_t probe_ver)
{
	int32_t rc = 0;
	struct cam_cmd_probe *sensor_probe_info;
	struct cam_cmd_probe_v2 *sensor_probe_info_v2;

	memset(s_ctrl->sensor_name, 0, CAM_SENSOR_NAME_MAX_SIZE);

	if (probe_ver == CAM_SENSOR_PACKET_OPCODE_SENSOR_PROBE) {
		sensor_probe_info = (struct cam_cmd_probe *)probe_info;
		s_ctrl->sensordata->slave_info.sensor_id_reg_addr =
			sensor_probe_info->reg_addr;
		s_ctrl->sensordata->slave_info.sensor_id =
			sensor_probe_info->expected_data;
		s_ctrl->sensordata->slave_info.sensor_id_mask =
			sensor_probe_info->data_mask;
		s_ctrl->pipeline_delay =
			sensor_probe_info->reserved;
		s_ctrl->modeswitch_delay = 0;

		s_ctrl->sensor_probe_addr_type = sensor_probe_info->addr_type;
		s_ctrl->sensor_probe_data_type = sensor_probe_info->data_type;
	} else if (probe_ver == CAM_SENSOR_PACKET_OPCODE_SENSOR_PROBE_V2) {
		sensor_probe_info_v2 = (struct cam_cmd_probe_v2 *)probe_info;
		s_ctrl->sensordata->slave_info.sensor_id_reg_addr =
			sensor_probe_info_v2->reg_addr;
		s_ctrl->sensordata->slave_info.sensor_id =
			sensor_probe_info_v2->expected_data;
		s_ctrl->sensordata->slave_info.sensor_id_mask =
			sensor_probe_info_v2->data_mask;
		s_ctrl->pipeline_delay =
			(sensor_probe_info_v2->pipeline_delay &
			CAM_SENSOR_PIPELINE_DELAY_MASK);
		s_ctrl->modeswitch_delay = (sensor_probe_info_v2->pipeline_delay >>
			CAM_SENSOR_MODESWITCH_DELAY_SHIFT);
		s_ctrl->sensor_probe_addr_type =
			sensor_probe_info_v2->addr_type;
		s_ctrl->sensor_probe_data_type =
			sensor_probe_info_v2->data_type;

		memcpy(s_ctrl->sensor_name, sensor_probe_info_v2->sensor_name,
			CAM_SENSOR_NAME_MAX_SIZE-1);
	}

	CAM_DBG(CAM_SENSOR,
		"%s Sensor Addr: 0x%x sensor_id: 0x%x sensor_mask: 0x%x sensor_pipeline_delay:0x%x",
		s_ctrl->sensor_name,
		s_ctrl->sensordata->slave_info.sensor_id_reg_addr,
		s_ctrl->sensordata->slave_info.sensor_id,
		s_ctrl->sensordata->slave_info.sensor_id_mask,
		s_ctrl->pipeline_delay);
	return rc;
}

int32_t cam_handle_cmd_buffers_for_probe(void *cmd_buf,
	struct cam_sensor_ctrl_t *s_ctrl,
	int32_t cmd_buf_num, uint32_t cmd,
	uint32_t cmd_buf_length, size_t remain_len,
	uint32_t probe_ver, struct cam_cmd_buf_desc *cmd_desc)
{
	int32_t rc = 0;
	size_t required_size = 0;

	switch (cmd_buf_num) {
	case 0: {
		struct cam_cmd_i2c_info *i2c_info = NULL;
		void *probe_info;

		if (probe_ver == CAM_SENSOR_PACKET_OPCODE_SENSOR_PROBE)
			required_size = sizeof(struct cam_cmd_i2c_info) +
				sizeof(struct cam_cmd_probe);
		else if(probe_ver == CAM_SENSOR_PACKET_OPCODE_SENSOR_PROBE_V2)
			required_size = sizeof(struct cam_cmd_i2c_info) +
				sizeof(struct cam_cmd_probe_v2);

		if (remain_len < required_size) {
			CAM_ERR(CAM_SENSOR,
				"not enough buffer for cam_cmd_i2c_info");
			return -EINVAL;
		}
		i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
		rc = cam_sensor_update_i2c_info(i2c_info, s_ctrl, true);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed in Updating the i2c Info");
			return rc;
		}
		probe_info = cmd_buf + sizeof(struct cam_cmd_i2c_info);
		rc = cam_sensor_update_slave_info(probe_info, cmd, s_ctrl,
							probe_ver);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Updating the slave Info");
			return rc;
		}
	}
		break;
	case 1: {
		rc = cam_sensor_update_power_settings(cmd_buf,
			cmd_buf_length, &s_ctrl->sensordata->power_info,
			remain_len);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed in updating power settings");
			return rc;
		}
	}
		break;

	case 2: {
		struct i2c_settings_array *i2c_reg_settings = NULL;
		struct i2c_data_settings *i2c_data = NULL;
		struct cam_buf_io_cfg *io_cfg = NULL;

		CAM_DBG(CAM_SENSOR, "reg_bank unlock settings");
		i2c_data = &(s_ctrl->i2c_data);
		i2c_reg_settings = &i2c_data->reg_bank_unlock_settings;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&s_ctrl->io_master_info,
				i2c_reg_settings, cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed in updating reg_bank unlock settings");
			return rc;
		}
	}
		break;
	case 3: {
		struct i2c_settings_array *i2c_reg_settings = NULL;
		struct i2c_data_settings *i2c_data = NULL;
		struct cam_buf_io_cfg *io_cfg = NULL;

		CAM_DBG(CAM_SENSOR, "reg_bank lock settings");
		i2c_data = &(s_ctrl->i2c_data);
		i2c_reg_settings = &i2c_data->reg_bank_lock_settings;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&s_ctrl->io_master_info,
				i2c_reg_settings, cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed in updating reg_bank lock settings");
			return rc;
		}
	}
		break;

	default:
		CAM_ERR(CAM_SENSOR, "Invalid command buffer");
		break;
	}
	return rc;
}

int32_t cam_handle_mem_ptr(uint64_t handle, uint32_t cmd,
	struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0, i;
	uint32_t *cmd_buf;
	void *ptr;
	size_t len;
	struct cam_packet *pkt = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	uintptr_t cmd_buf1 = 0;
	uintptr_t packet = 0;
	size_t    remain_len = 0;
	uint32_t probe_ver = 0;

	rc = cam_mem_get_cpu_buf(handle,
		&packet, &len);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed to get the command Buffer");
		return -EINVAL;
	}

	pkt = (struct cam_packet *)packet;
	if (pkt == NULL) {
		CAM_ERR(CAM_SENSOR, "packet pos is invalid");
		rc = -EINVAL;
		goto end;
	}

	if ((len < sizeof(struct cam_packet)) ||
		(pkt->cmd_buf_offset >= (len - sizeof(struct cam_packet)))) {
		CAM_ERR(CAM_SENSOR, "Not enough buf provided");
		rc = -EINVAL;
		goto end;
	}

	cmd_desc = (struct cam_cmd_buf_desc *)
		((uint32_t *)&pkt->payload_flex + pkt->cmd_buf_offset/4);
	if (cmd_desc == NULL) {
		CAM_ERR(CAM_SENSOR, "command descriptor pos is invalid");
		rc = -EINVAL;
		goto end;
	}

	probe_ver = pkt->header.op_code & 0xFFFFFF;
	CAM_DBG(CAM_SENSOR, "Received Header opcode: %u", probe_ver);

	for (i = 0; i < pkt->num_cmd_buf; i++) {
		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc)
			return rc;

		if (!(cmd_desc[i].length))
			continue;
		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
			&cmd_buf1, &len);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to parse the command Buffer Header");
			goto end;
		}
		if (cmd_desc[i].offset >= len) {
			CAM_ERR(CAM_SENSOR,
				"offset past length of buffer");
			cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
			rc = -EINVAL;
			goto end;
		}
		remain_len = len - cmd_desc[i].offset;
		if (cmd_desc[i].length > remain_len) {
			CAM_ERR(CAM_SENSOR,
				"Not enough buffer provided for cmd");
			cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
			rc = -EINVAL;
			goto end;
		}
		cmd_buf = (uint32_t *)cmd_buf1;
		cmd_buf += cmd_desc[i].offset/4;
		ptr = (void *) cmd_buf;

		rc = cam_handle_cmd_buffers_for_probe(ptr, s_ctrl,
			i, cmd, cmd_desc[i].length, remain_len, probe_ver, &cmd_desc[i]);

		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to parse the command Buffer Header");
			cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
			goto end;
		}
		cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
	}

end:
	cam_mem_put_cpu_buf(handle);
	return rc;
}

void cam_sensor_query_cap(struct cam_sensor_ctrl_t *s_ctrl,
	struct  cam_sensor_query_cap *query_cap)
{
	query_cap->pos_roll = s_ctrl->sensordata->pos_roll;
	query_cap->pos_pitch = s_ctrl->sensordata->pos_pitch;
	query_cap->pos_yaw = s_ctrl->sensordata->pos_yaw;
	query_cap->secure_camera = 0;
	query_cap->actuator_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_ACTUATOR];
	query_cap->csiphy_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_CSIPHY];
	query_cap->eeprom_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_EEPROM];
	query_cap->flash_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_LED_FLASH];
	query_cap->ois_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_OIS];
	query_cap->slot_info =
		s_ctrl->soc_info.index;
}

static uint16_t cam_sensor_id_by_mask(struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t chipid)
{
	uint16_t sensor_id = (uint16_t)(chipid & 0xFFFF);
	int16_t sensor_id_mask = s_ctrl->sensordata->slave_info.sensor_id_mask;

	if (!sensor_id_mask)
		sensor_id_mask = ~sensor_id_mask;

	sensor_id &= sensor_id_mask;
	sensor_id_mask &= -sensor_id_mask;
	sensor_id_mask -= 1;
	while (sensor_id_mask) {
		sensor_id_mask >>= 1;
		sensor_id >>= 1;
	}
	return sensor_id;
}

void cam_sensor_shutdown(struct cam_sensor_ctrl_t *s_ctrl)
{
	struct cam_sensor_power_ctrl_t *power_info =
		&s_ctrl->sensordata->power_info;
	int rc = 0;

	if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) &&
		(s_ctrl->is_probe_succeed == 0))
		return;

	cam_sensor_release_stream_rsc(s_ctrl);
	cam_sensor_release_per_frame_resource(s_ctrl);

	if (s_ctrl->sensor_state != CAM_SENSOR_INIT)
		cam_sensor_power_down(s_ctrl);

	if (s_ctrl->bridge_intf.device_hdl != -1) {
		rc = cam_destroy_device_hdl(s_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"dhdl already destroyed: rc = %d", rc);
	}

	s_ctrl->bridge_intf.device_hdl = -1;
	s_ctrl->bridge_intf.link_hdl = -1;
	s_ctrl->bridge_intf.session_hdl = -1;
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_setting_size = 0;
	power_info->power_down_setting_size = 0;
	s_ctrl->streamon_count = 0;
	s_ctrl->streamoff_count = 0;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->last_flush_req = 0;
	s_ctrl->sensor_state = CAM_SENSOR_INIT;
}

int cam_sensor_match_id(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint32_t chipid = 0;
	struct cam_camera_slave_info *slave_info;

	slave_info = &(s_ctrl->sensordata->slave_info);

	if (!slave_info) {
		CAM_ERR(CAM_SENSOR, " failed: %pK",
			 slave_info);
		return -EINVAL;
	}

	if (s_ctrl->hw_no_ops)
		return rc;

	rc = camera_io_dev_read(
		&(s_ctrl->io_master_info),
		slave_info->sensor_id_reg_addr,
		&chipid, s_ctrl->sensor_probe_addr_type,
		s_ctrl->sensor_probe_data_type, true);

	CAM_DBG(CAM_SENSOR, "%s read id: 0x%x expected id 0x%x:",
		s_ctrl->sensor_name, chipid, slave_info->sensor_id);

	if (cam_sensor_id_by_mask(s_ctrl, chipid) != slave_info->sensor_id) {
		CAM_WARN(CAM_SENSOR, "%s read id: 0x%x expected id 0x%x:",
				s_ctrl->sensor_name, chipid,
				slave_info->sensor_id);
		return -ENODEV;
	}
	return rc;
}

int cam_sensor_stream_off(struct cam_sensor_ctrl_t *s_ctrl)
{
	int               rc = 0;
	struct timespec64 ts;
	uint64_t          ms, sec, min, hrs;

	if (s_ctrl->sensor_state != CAM_SENSOR_START) {
		rc = -EINVAL;
		CAM_WARN(CAM_SENSOR,
			"Not in right state to stop %s state: %d",
			s_ctrl->sensor_name, s_ctrl->sensor_state);
		goto end;
	}

	if (s_ctrl->i2c_data.streamoff_settings.is_settings_valid &&
		(s_ctrl->i2c_data.streamoff_settings.request_id == 0)) {
		rc = cam_sensor_apply_settings(s_ctrl, 0,
			CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"cannot apply streamoff settings for %s",
				s_ctrl->sensor_name);
	}

	cam_sensor_release_per_frame_resource(s_ctrl);
	s_ctrl->last_flush_req = 0;
	s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
	memset(s_ctrl->sensor_res, 0, sizeof(s_ctrl->sensor_res));

	CAM_GET_TIMESTAMP(ts);
	CAM_CONVERT_TIMESTAMP_FORMAT(ts, hrs, min, sec, ms);

	CAM_INFO(CAM_SENSOR,
		"%llu:%llu:%llu.%llu CAM_STOP_DEV Success for %s sensor_id:0x%x,sensor_slave_addr:0x%x",
		hrs, min, sec, ms,
		s_ctrl->sensor_name,
		s_ctrl->sensordata->slave_info.sensor_id,
		s_ctrl->sensordata->slave_info.sensor_slave_addr);

end:
	return rc;
}

int32_t cam_sensor_driver_cmd(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg)
{
	int rc = 0, pkt_opcode = 0;
	struct cam_control *cmd = (struct cam_control *)arg;
	struct cam_sensor_power_ctrl_t *power_info = NULL;
	struct timespec64 ts;
	uint64_t ms, sec, min, hrs;

	if (!s_ctrl || !arg) {
		CAM_ERR(CAM_SENSOR, "s_ctrl is NULL");
		return -EINVAL;
	}

	power_info = &s_ctrl->sensordata->power_info;

	if (cmd->op_code != CAM_SENSOR_PROBE_CMD) {
		if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
			CAM_ERR(CAM_SENSOR, "Invalid handle type: %d",
				cmd->handle_type);
			return -EINVAL;
		}
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	switch (cmd->op_code) {
	case CAM_SENSOR_PROBE_CMD: {
		if (s_ctrl->is_probe_succeed == 1) {
			CAM_WARN(CAM_SENSOR,
				"Sensor %s already Probed in the slot",
				s_ctrl->sensor_name);
			break;
		}

		if (cmd->handle_type ==
			CAM_HANDLE_MEM_HANDLE) {
			rc = cam_handle_mem_ptr(cmd->handle, cmd->op_code,
				s_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "Get Buffer Handle Failed");
				goto release_mutex;
			}
		} else {
			CAM_ERR(CAM_SENSOR, "Invalid Command Type: %d",
				cmd->handle_type);
			rc = -EINVAL;
			goto release_mutex;
		}

		/* Parse and fill vreg params for powerup settings */
		rc = msm_camera_fill_vreg_params(
			&s_ctrl->soc_info,
			s_ctrl->sensordata->power_info.power_setting,
			s_ctrl->sensordata->power_info.power_setting_size);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Fail in filling vreg params for %s PUP rc %d",
				s_ctrl->sensor_name, rc);
			goto free_power_settings;
		}

		/* Parse and fill vreg params for powerdown settings*/
		rc = msm_camera_fill_vreg_params(
			&s_ctrl->soc_info,
			s_ctrl->sensordata->power_info.power_down_setting,
			s_ctrl->sensordata->power_info.power_down_setting_size);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Fail in filling vreg params for %s PDOWN rc %d",
				s_ctrl->sensor_name, rc);
			goto free_power_settings;
		}

		/* Power up and probe sensor */
		rc = cam_sensor_power_up(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Power up failed for %s sensor_id: 0x%x, slave_addr: 0x%x",
				s_ctrl->sensor_name,
				s_ctrl->sensordata->slave_info.sensor_id,
				s_ctrl->sensordata->slave_info.sensor_slave_addr
				);
			goto free_power_settings;
		}

		if (s_ctrl->i2c_data.reg_bank_unlock_settings.is_settings_valid) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_REG_BANK_UNLOCK);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "REG_bank unlock failed");
				cam_sensor_power_down(s_ctrl);
				goto free_power_settings;
			}
			rc = delete_request(&(s_ctrl->i2c_data.reg_bank_unlock_settings));
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"failed while deleting REG_bank unlock settings");
				cam_sensor_power_down(s_ctrl);
				goto free_power_settings;
			}
		}

		/* Match sensor ID */
		rc = cam_sensor_match_id(s_ctrl);
		if (rc < 0) {
			CAM_INFO(CAM_SENSOR,
				"Probe failed for %s slot:%d, slave_addr:0x%x, sensor_id:0x%x",
				s_ctrl->sensor_name,
				s_ctrl->soc_info.index,
				s_ctrl->sensordata->slave_info.sensor_slave_addr,
				s_ctrl->sensordata->slave_info.sensor_id);
			cam_sensor_power_down(s_ctrl);
			goto free_power_settings;
		}

		if (s_ctrl->i2c_data.reg_bank_lock_settings.is_settings_valid) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_REG_BANK_LOCK);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "REG_bank lock failed");
				cam_sensor_power_down(s_ctrl);
				goto free_power_settings;
			}
			rc = delete_request(&(s_ctrl->i2c_data.reg_bank_lock_settings));
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"failed while deleting REG_bank lock settings");
				cam_sensor_power_down(s_ctrl);
				goto free_power_settings;
			}
		}

		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Fail in %s sensor Power Down",
				s_ctrl->sensor_name);
			goto free_power_settings;
		}

		/*
		 * Set probe succeeded flag to 1 so that no other camera shall
		 * probed on this slot
		 */
		s_ctrl->is_probe_succeed = 1;
		s_ctrl->sensor_state = CAM_SENSOR_INIT;

		CAM_INFO(CAM_SENSOR,
				"Probe success for %s slot:%d,slave_addr:0x%x,sensor_id:0x%x",
				s_ctrl->sensor_name,
				s_ctrl->soc_info.index,
				s_ctrl->sensordata->slave_info.sensor_slave_addr,
				s_ctrl->sensordata->slave_info.sensor_id);

	}
		break;
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev sensor_acq_dev;
		struct cam_create_dev_hdl bridge_params;

		if ((s_ctrl->is_probe_succeed == 0) ||
			(s_ctrl->sensor_state != CAM_SENSOR_INIT)) {
			CAM_WARN(CAM_SENSOR,
				"Not in right state to aquire %s state: %d",
				s_ctrl->sensor_name, s_ctrl->sensor_state);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (s_ctrl->bridge_intf.device_hdl != -1) {
			CAM_ERR(CAM_SENSOR,
				"%s Device is already acquired",
				s_ctrl->sensor_name);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = copy_from_user(&sensor_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(sensor_acq_dev));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed Copying from user");
			goto release_mutex;
		}

		bridge_params.session_hdl = sensor_acq_dev.session_handle;
		bridge_params.ops = &s_ctrl->bridge_intf.ops;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = s_ctrl;
		bridge_params.dev_id = CAM_SENSOR;

		sensor_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		if (sensor_acq_dev.device_handle <= 0) {
			rc = -EFAULT;
			CAM_ERR(CAM_SENSOR, "Can not create device handle");
			goto release_mutex;
		}
		s_ctrl->bridge_intf.device_hdl = sensor_acq_dev.device_handle;
		s_ctrl->bridge_intf.session_hdl = sensor_acq_dev.session_handle;

		CAM_DBG(CAM_SENSOR, "%s Device Handle: %d",
			s_ctrl->sensor_name, sensor_acq_dev.device_handle);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&sensor_acq_dev,
			sizeof(struct cam_sensor_acquire_dev))) {
			CAM_ERR(CAM_SENSOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}

		rc = cam_sensor_power_up(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Sensor Power up failed for %s sensor_id:0x%x, slave_addr:0x%x",
				s_ctrl->sensor_name,
				s_ctrl->sensordata->slave_info.sensor_id,
				s_ctrl->sensordata->slave_info.sensor_slave_addr
				);
			goto release_mutex;
		}

		s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
		s_ctrl->last_flush_req = 0;
		s_ctrl->is_stopped_by_user = false;
		s_ctrl->last_updated_req = 0;
		s_ctrl->last_applied_req = 0;
		s_ctrl->num_batched_frames = 0;
		memset(s_ctrl->sensor_res, 0, sizeof(s_ctrl->sensor_res));
		CAM_INFO(CAM_SENSOR,
			"CAM_ACQUIRE_DEV Success for %s sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensor_name,
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	case CAM_RELEASE_DEV: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_START)) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
				"Not in right state to release %s state: %d",
				s_ctrl->sensor_name, s_ctrl->sensor_state);
			goto release_mutex;
		}

		if (s_ctrl->bridge_intf.link_hdl != -1) {
			CAM_ERR(CAM_SENSOR,
				"%s Device [%d] still active on link 0x%x",
				s_ctrl->sensor_name,
				s_ctrl->sensor_state,
				s_ctrl->bridge_intf.link_hdl);
			rc = -EAGAIN;
			goto release_mutex;
		}

		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Sensor Power Down failed for %s sensor_id: 0x%x, slave_addr:0x%x",
				s_ctrl->sensor_name,
				s_ctrl->sensordata->slave_info.sensor_id,
				s_ctrl->sensordata->slave_info.sensor_slave_addr
				);
			goto release_mutex;
		}

		cam_sensor_release_per_frame_resource(s_ctrl);
		cam_sensor_release_stream_rsc(s_ctrl);
		if (s_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_SENSOR,
				"Invalid Handles: %s link hdl: %d device hdl: %d",
				s_ctrl->sensor_name,
				s_ctrl->bridge_intf.device_hdl,
				s_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(s_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"Failed in destroying %s device hdl",
				s_ctrl->sensor_name);
		s_ctrl->bridge_intf.device_hdl = -1;
		s_ctrl->bridge_intf.link_hdl = -1;
		s_ctrl->bridge_intf.session_hdl = -1;

		s_ctrl->sensor_state = CAM_SENSOR_INIT;
		CAM_INFO(CAM_SENSOR,
			"CAM_RELEASE_DEV Success for %s sensor_id:0x%x, slave_addr:0x%x",
			s_ctrl->sensor_name,
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
		s_ctrl->streamon_count = 0;
		s_ctrl->streamoff_count = 0;
		s_ctrl->last_flush_req = 0;
	}
		break;
	case CAM_QUERY_CAP: {
		struct  cam_sensor_query_cap sensor_cap;

		CAM_DBG(CAM_SENSOR, "%s Sensor Queried", s_ctrl->sensor_name);
		cam_sensor_query_cap(s_ctrl, &sensor_cap);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&sensor_cap, sizeof(struct  cam_sensor_query_cap))) {
			CAM_ERR(CAM_SENSOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		break;
	}
	case CAM_START_DEV: {
		struct cam_req_mgr_timer_notify timer;
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_START)) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
			"Not in right state to start %s state: %d",
			s_ctrl->sensor_name,
			s_ctrl->sensor_state);
			goto release_mutex;
		}

		if (s_ctrl->i2c_data.streamon_settings.is_settings_valid &&
			(s_ctrl->i2c_data.streamon_settings.request_id == 0)) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply streamon settings for %s",
					s_ctrl->sensor_name);
				goto release_mutex;
			}
		}
		s_ctrl->sensor_state = CAM_SENSOR_START;

		if (s_ctrl->stream_off_after_eof)
			s_ctrl->is_stopped_by_user = false;

		if (s_ctrl->bridge_intf.crm_cb &&
			s_ctrl->bridge_intf.crm_cb->notify_timer) {
			timer.link_hdl = s_ctrl->bridge_intf.link_hdl;
			timer.dev_hdl = s_ctrl->bridge_intf.device_hdl;
			timer.state = true;
			rc = s_ctrl->bridge_intf.crm_cb->notify_timer(&timer);
			if (rc) {
				CAM_ERR(CAM_SENSOR,
					"%s Enable CRM SOF freeze timer failed rc: %d",
					s_ctrl->sensor_name, rc);
				return rc;
			}
		}

		CAM_GET_TIMESTAMP(ts);
		CAM_CONVERT_TIMESTAMP_FORMAT(ts, hrs, min, sec, ms);

		CAM_INFO(CAM_SENSOR,
			"%llu:%llu:%llu.%llu CAM_START_DEV Success for %s sensor_id:0x%x,sensor_slave_addr:0x%x num_batched_frames:%d",
			hrs, min, sec, ms,
			s_ctrl->sensor_name,
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr,
			s_ctrl->num_batched_frames);
	}
		break;
	case CAM_STOP_DEV: {
		if (s_ctrl->stream_off_after_eof) {
			s_ctrl->is_stopped_by_user = true;
			CAM_DBG(CAM_SENSOR, "Ignore stop dev cmd for VFPS feature");
			goto release_mutex;
		}

		rc = cam_sensor_stream_off(s_ctrl);
		if (rc)
			goto release_mutex;
	}
		break;
	case CAM_CONFIG_DEV: {
		rc = cam_sensor_pkt_parse(s_ctrl, arg);
		if (rc < 0) {
			if (rc == -EBADR)
				CAM_INFO(CAM_SENSOR,
					"%s:Failed pkt parse. rc: %d, it has been flushed",
					s_ctrl->sensor_name, rc);
			else
				CAM_ERR(CAM_SENSOR,
					"%s:Failed pkt parse. rc: %d",
					s_ctrl->sensor_name, rc);
			goto release_mutex;
		}
		if (s_ctrl->i2c_data.init_settings.is_settings_valid &&
			(s_ctrl->i2c_data.init_settings.request_id == 0)) {
			pkt_opcode =
				CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG;
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				pkt_opcode);

			if ((rc == -EAGAIN) &&
			(s_ctrl->io_master_info.master_type == CCI_MASTER)) {
				/* If CCI hardware is resetting we need to wait
				 * for sometime before reapply
				 */
				CAM_WARN(CAM_SENSOR,
					"%s: Reapplying the Init settings due to cci hw reset",
					s_ctrl->sensor_name);
				usleep_range(1000, 1010);
				rc = cam_sensor_apply_settings(s_ctrl, 0,
					pkt_opcode);
			}
			s_ctrl->i2c_data.init_settings.request_id = -1;

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"%s: cannot apply init settings rc= %d",
					s_ctrl->sensor_name, rc);
				delete_request(&s_ctrl->i2c_data.init_settings);
				goto release_mutex;
			}
			rc = delete_request(&s_ctrl->i2c_data.init_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"%s: Fail in deleting the Init settings",
					s_ctrl->sensor_name);
				goto release_mutex;
			}
		}

		if (s_ctrl->i2c_data.config_settings.is_settings_valid &&
			(s_ctrl->i2c_data.config_settings.request_id == 0)) {
			if (s_ctrl->sensor_state == CAM_SENSOR_START) {
				delete_request(&s_ctrl->i2c_data.config_settings);
				CAM_ERR(CAM_SENSOR,
					"%s: get config setting in start state",
					s_ctrl->sensor_name);
					goto release_mutex;
			}

			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG);

			s_ctrl->i2c_data.config_settings.request_id = -1;

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"%s: cannot apply config settings",
					s_ctrl->sensor_name);
				delete_request(
					&s_ctrl->i2c_data.config_settings);
				goto release_mutex;
			}
			rc = delete_request(&s_ctrl->i2c_data.config_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"%s: Fail in deleting the config settings",
					s_ctrl->sensor_name);
				goto release_mutex;
			}
			s_ctrl->sensor_state = CAM_SENSOR_CONFIG;
		}

		if (s_ctrl->i2c_data.read_settings.is_settings_valid) {
			if (!s_ctrl->hw_no_ops)
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_READ);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply read settings");
				delete_request(
					&s_ctrl->i2c_data.read_settings);
				goto release_mutex;
			}
			rc = delete_request(
				&s_ctrl->i2c_data.read_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"%s: Fail in deleting the read settings",
					s_ctrl->sensor_name);
				goto release_mutex;
			}
		}

		CAM_DBG(CAM_SENSOR,
			"CAM_CONFIG_DEV done sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	default:
		CAM_ERR(CAM_SENSOR, "%s: Invalid Opcode: %d",
			s_ctrl->sensor_name, cmd->op_code);
		rc = -EINVAL;
		goto release_mutex;
	}

release_mutex:
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int cam_sensor_publish_dev_info(struct cam_req_mgr_device_info *info)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	if (!info)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(info->dev_hdl);

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	info->dev_id = CAM_REQ_MGR_DEVICE_SENSOR;
	strscpy(info->name, CAM_SENSOR_NAME, sizeof(info->name));
	if (s_ctrl->num_batched_frames >= 2) {
		info->p_delay = 1;
		info->m_delay = s_ctrl->modeswitch_delay;
	} else if (s_ctrl->pipeline_delay >= 1 && s_ctrl->pipeline_delay <= 3) {
		info->p_delay = s_ctrl->pipeline_delay;
		info->m_delay = s_ctrl->modeswitch_delay;
	} else {
		info->p_delay = CAM_PIPELINE_DELAY_2;
		info->m_delay = CAM_MODESWITCH_DELAY_2;
	}
	info->trigger = CAM_TRIGGER_POINT_SOF;

	CAM_DBG(CAM_REQ, "num batched frames %d p_delay is %d",
		s_ctrl->num_batched_frames, info->p_delay);

	return rc;
}

int cam_sensor_establish_link(struct cam_req_mgr_core_dev_link_setup *link)
{
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	if (!link)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(link->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&s_ctrl->cam_sensor_mutex);
	if (link->link_enable) {
		s_ctrl->bridge_intf.link_hdl = link->link_hdl;
		s_ctrl->bridge_intf.crm_cb = link->crm_cb;
	} else {
		s_ctrl->bridge_intf.link_hdl = -1;
		s_ctrl->bridge_intf.crm_cb = NULL;
	}
	mutex_unlock(&s_ctrl->cam_sensor_mutex);

	return 0;
}

int cam_sensor_power(struct v4l2_subdev *sd, int on)
{
	struct cam_sensor_ctrl_t *s_ctrl = v4l2_get_subdevdata(sd);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "s_ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	if (!on && s_ctrl->sensor_state == CAM_SENSOR_START) {
		cam_sensor_power_down(s_ctrl);
		s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
	}
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));

	return 0;
}

int cam_sensor_power_up(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_camera_slave_info   *slave_info;
	struct cam_hw_soc_info         *soc_info = &s_ctrl->soc_info;
	struct completion              *i3c_probe_completion = NULL;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "failed: %pK", s_ctrl);
		return -EINVAL;
	}

	if (s_ctrl->hw_no_ops)
		return rc;

	power_info = &s_ctrl->sensordata->power_info;
	slave_info = &(s_ctrl->sensordata->slave_info);

	if (!power_info || !slave_info) {
		CAM_ERR(CAM_SENSOR, "failed: %pK %pK", power_info, slave_info);
		return -EINVAL;
	}

	soc_info = &s_ctrl->soc_info;

	if (s_ctrl->bob_pwm_switch) {
		if (cam_sensor_bob_pwm_mode_switch(soc_info,
			s_ctrl->bob_reg_index, true))
			CAM_WARN(CAM_SENSOR, "BoB PWM setup failed");
	}

	if (s_ctrl->aon_camera_id != NOT_AON_CAM) {
		CAM_INFO(CAM_SENSOR,
			"Setup for Main Camera with csiphy index: %d",
			s_ctrl->sensordata->subdev_id[SUB_MODULE_CSIPHY]);
		rc = cam_sensor_util_aon_ops(true,
			s_ctrl->sensordata->subdev_id[SUB_MODULE_CSIPHY]);
		if (rc) {
			CAM_ERR(CAM_SENSOR,
				"Main camera access operation is not successful rc: %d",
				rc);
			return rc;
		}
	}

	if (s_ctrl->io_master_info.master_type == I3C_MASTER)
		i3c_probe_completion = cam_sensor_get_i3c_completion(s_ctrl->soc_info.index);

	rc = cam_sensor_core_power_up(power_info, soc_info, i3c_probe_completion);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "core power up failed:%d", rc);
		return rc;
	}

	rc = camera_io_init(&(s_ctrl->io_master_info));
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "cci_init failed: rc: %d", rc);
		goto cci_failure;
	}

	return rc;

cci_failure:
	if (cam_sensor_util_power_down(power_info, soc_info))
		CAM_ERR(CAM_SENSOR, "power down failure");

	return rc;

}

int cam_sensor_power_down(struct cam_sensor_ctrl_t *s_ctrl)
{
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info *soc_info;
	int rc = 0;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "failed: s_ctrl %pK", s_ctrl);
		return -EINVAL;
	}

	if (s_ctrl->hw_no_ops)
		return rc;

	power_info = &s_ctrl->sensordata->power_info;
	soc_info = &s_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_SENSOR, "failed: %s power_info %pK",
			s_ctrl->sensor_name, power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "%s core power down failed:%d",
			s_ctrl->sensor_name, rc);
		return rc;
	}

	if (s_ctrl->aon_camera_id != NOT_AON_CAM) {
		CAM_INFO(CAM_SENSOR,
			"Setup for AON FW with csiphy index: %d",
			s_ctrl->sensordata->subdev_id[SUB_MODULE_CSIPHY]);
		rc = cam_sensor_util_aon_ops(false,
			s_ctrl->sensordata->subdev_id[SUB_MODULE_CSIPHY]);
		if (rc) {
			CAM_ERR(CAM_SENSOR,
				"AON FW access operation is not successful rc: %d",
				rc);
			return rc;
		}
	}

	soc_info = &s_ctrl->soc_info;

	if (s_ctrl->bob_pwm_switch) {
		rc = cam_sensor_bob_pwm_mode_switch(soc_info,
			s_ctrl->bob_reg_index, false);
		if (rc) {
			CAM_WARN(CAM_SENSOR,
				"%s BoB PWM setup failed rc: %d",
				s_ctrl->sensor_name, rc);
			rc = 0;
		}
	}

	camera_io_release(&(s_ctrl->io_master_info));

	return rc;
}

int cam_sensor_apply_settings(struct cam_sensor_ctrl_t *s_ctrl,
	int64_t req_id, enum cam_sensor_packet_opcodes opcode)
{
	int rc = 0, offset, i;
	uint64_t top = 0, del_req_id = 0;
	struct i2c_settings_array *i2c_set = NULL;
	struct i2c_settings_list *i2c_list;

	if (req_id == 0) {
		switch (opcode) {
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON: {
			i2c_set = &s_ctrl->i2c_data.streamon_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG: {
			i2c_set = &s_ctrl->i2c_data.init_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG: {
			i2c_set = &s_ctrl->i2c_data.config_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF: {
			i2c_set = &s_ctrl->i2c_data.streamoff_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_REG_BANK_UNLOCK: {
			i2c_set = &s_ctrl->i2c_data.reg_bank_unlock_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_REG_BANK_LOCK: {
			i2c_set = &s_ctrl->i2c_data.reg_bank_lock_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_READ: {
			i2c_set = &s_ctrl->i2c_data.read_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE:
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_FRAME_SKIP_UPDATE:
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_PROBE:
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_PROBE_V2:
		default:
			return 0;
		}
		if (i2c_set->is_settings_valid == 1) {
			list_for_each_entry(i2c_list,
				&(i2c_set->list_head), list) {
				if (!s_ctrl->hw_no_ops)
					rc = cam_sensor_i2c_modes_util(s_ctrl,
						i2c_list);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR,
						"Failed to apply settings: %d",
						rc);
					goto EXIT_RESTORE;
				}
			}
		}
	} else if (req_id > 0) {
		offset = req_id % MAX_PER_FRAME_ARRAY;

		if (opcode == CAM_SENSOR_PACKET_OPCODE_SENSOR_FRAME_SKIP_UPDATE)
			i2c_set = s_ctrl->i2c_data.frame_skip;
		else if (opcode == CAM_SENSOR_PACKET_OPCODE_SENSOR_BUBBLE_UPDATE) {
			i2c_set = s_ctrl->i2c_data.bubble_update;
			/*
			 * If bubble update isn't valid, then we just use
			 * per frame update.
			 */
			if (!(i2c_set[offset].is_settings_valid == 1) &&
				(i2c_set[offset].request_id == req_id))
				i2c_set = s_ctrl->i2c_data.per_frame;
		} else
			i2c_set = s_ctrl->i2c_data.per_frame;

		if (i2c_set[offset].is_settings_valid == 1 &&
			i2c_set[offset].request_id == req_id) {
			list_for_each_entry(i2c_list,
				&(i2c_set[offset].list_head), list) {
				if (!s_ctrl->hw_no_ops)
					rc = cam_sensor_i2c_modes_util(s_ctrl,
						i2c_list);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR,
						"Failed to apply settings: %d",
						rc);
					goto EXIT_RESTORE;
				}
			}
			CAM_DBG(CAM_SENSOR, "applied req_id: %llu", req_id);
		} else {
			CAM_DBG(CAM_SENSOR,
				"Invalid/NOP request to apply: %lld", req_id);
		}

		s_ctrl->last_applied_req = req_id;
		CAM_DBG(CAM_REQ,
			"Sensor[%d] updating last_applied [req id: %lld last_applied: %lld] with opcode:%d",
			s_ctrl->soc_info.index, req_id, s_ctrl->last_applied_req, opcode);

		/* Change the logic dynamically */
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			if ((req_id >=
				i2c_set[i].request_id) &&
				(top <
				i2c_set[i].request_id) &&
				(i2c_set[i].is_settings_valid
					== 1)) {
				del_req_id = top;
				top = i2c_set[i].request_id;
			}
		}

		if (top < req_id) {
			if ((((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) >= BATCH_SIZE_MAX) ||
				(((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) <= -BATCH_SIZE_MAX))
				del_req_id = req_id;
		}

		if (!del_req_id)
			goto EXIT_RESTORE;

		CAM_DBG(CAM_SENSOR, "top: %llu, del_req_id:%llu",
			top, del_req_id);

		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			if ((del_req_id >
				 i2c_set[i].request_id) && (
				 i2c_set[i].is_settings_valid
					== 1)) {
				i2c_set[i].request_id = 0;
				rc = delete_request(
					&(i2c_set[i]));
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"Delete request Fail:%lld rc:%d",
						del_req_id, rc);
			}
		}

		/*
		 * If the op code is bubble update, then we also need to delete
		 * req for per frame update, vice versa.
		 */
		if (opcode == CAM_SENSOR_PACKET_OPCODE_SENSOR_BUBBLE_UPDATE)
			i2c_set = s_ctrl->i2c_data.per_frame;
		else if (opcode == CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE)
			i2c_set = s_ctrl->i2c_data.bubble_update;
		else
			i2c_set = NULL;

		if (i2c_set) {
			for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
				if ((del_req_id >
					 i2c_set[i].request_id) && (
					 i2c_set[i].is_settings_valid
						== 1)) {
					i2c_set[i].request_id = 0;
					rc = delete_request(
						&(i2c_set[i]));
					if (rc < 0)
						CAM_ERR(CAM_SENSOR,
							"Delete request Fail:%lld rc:%d",
							del_req_id, rc);
				}
			}
		}
	}

EXIT_RESTORE:
	(void)cam_sensor_restore_slave_info(s_ctrl);

	return rc;
}

int32_t cam_sensor_apply_request(struct cam_req_mgr_apply_request *apply)
{
	int32_t rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	int32_t curr_idx, last_applied_idx;
	enum cam_sensor_packet_opcodes opcode =
		CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE;

	if (!apply)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(apply->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	if ((apply->recovery) && (apply->request_id > 0)) {
		if (apply->request_id <= s_ctrl->last_applied_req) {
			/*
			 * Use bubble update for request reapply
			 */
			curr_idx = apply->request_id % MAX_PER_FRAME_ARRAY;
			last_applied_idx = s_ctrl->last_applied_req % MAX_PER_FRAME_ARRAY;
			opcode = CAM_SENSOR_PACKET_OPCODE_SENSOR_BUBBLE_UPDATE;
			CAM_INFO(CAM_REQ,
				"Sensor[%d] update req id: %lld [last_applied: %lld] with opcode:%d recovery: %d last_applied_res_idx: %u current_res_idx: %u",
				s_ctrl->soc_info.index, apply->request_id,
				s_ctrl->last_applied_req, opcode, apply->recovery,
				s_ctrl->sensor_res[last_applied_idx].res_index,
				s_ctrl->sensor_res[curr_idx].res_index);
		}
	}

	CAM_DBG(CAM_REQ,
		"Sensor[%d] update req id: %lld [last_applied: %lld] with opcode:%d recovery: %d",
		s_ctrl->soc_info.index, apply->request_id,
		s_ctrl->last_applied_req, opcode, apply->recovery);
	trace_cam_apply_req("Sensor", s_ctrl->soc_info.index, apply->request_id, apply->link_hdl);

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	rc = cam_sensor_apply_settings(s_ctrl, apply->request_id,
		opcode);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int32_t cam_sensor_notify_frame_skip(struct cam_req_mgr_apply_request *apply)
{
	int32_t rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	enum cam_sensor_packet_opcodes opcode =
		CAM_SENSOR_PACKET_OPCODE_SENSOR_FRAME_SKIP_UPDATE;

	if (!apply)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(apply->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	CAM_DBG(CAM_REQ, " Sensor[%d] handle frame skip for req id: %lld",
		s_ctrl->soc_info.index, apply->request_id);
	trace_cam_notify_frame_skip("Sensor", apply->request_id);
	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	rc = cam_sensor_apply_settings(s_ctrl, apply->request_id,
		opcode);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int32_t cam_sensor_flush_request(struct cam_req_mgr_flush_request *flush_req)
{
	int32_t rc = 0, i;
	uint32_t cancel_req_id_found = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	struct i2c_settings_array *i2c_set = NULL;

	if (!flush_req)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(flush_req->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	if ((s_ctrl->sensor_state != CAM_SENSOR_START) &&
		(s_ctrl->sensor_state != CAM_SENSOR_CONFIG)) {
		mutex_unlock(&(s_ctrl->cam_sensor_mutex));
		return rc;
	}

	if (s_ctrl->i2c_data.per_frame == NULL) {
		CAM_ERR(CAM_SENSOR, "i2c frame data is NULL");
		mutex_unlock(&(s_ctrl->cam_sensor_mutex));
		return -EINVAL;
	}

	if (s_ctrl->i2c_data.frame_skip == NULL) {
		CAM_ERR(CAM_SENSOR, "i2c not ready data is NULL");
		mutex_unlock(&(s_ctrl->cam_sensor_mutex));
		return -EINVAL;
	}

	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_ALL) {
		s_ctrl->last_flush_req = flush_req->req_id;
		CAM_DBG(CAM_SENSOR, "last reqest to flush is %lld",
			flush_req->req_id);

		/*
		 * Sensor can't get EOF event during flush if we do the flush
		 * before EOF, so we need to stream off the sensor during flush
		 * for VFPS usecase.
		 */
		if (s_ctrl->stream_off_after_eof) {
			cam_sensor_stream_off(s_ctrl);
			s_ctrl->is_stopped_by_user = false;
		}
	}

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
		i2c_set = &(s_ctrl->i2c_data.per_frame[i]);

		if ((flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ)
				&& (i2c_set->request_id != flush_req->req_id))
			continue;

		if (i2c_set->is_settings_valid == 1) {
			rc = delete_request(i2c_set);
			if (rc < 0)
				CAM_ERR(CAM_SENSOR,
					"delete request: %lld rc: %d",
					i2c_set->request_id, rc);

			if (flush_req->type ==
				CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ) {
				cancel_req_id_found = 1;
				break;
			}
		}
	}

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
		i2c_set = &(s_ctrl->i2c_data.frame_skip[i]);

		if ((flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ)
				&& (i2c_set->request_id != flush_req->req_id))
			continue;

		if (i2c_set->is_settings_valid == 1) {
			rc = delete_request(i2c_set);
			if (rc < 0)
				CAM_ERR(CAM_SENSOR,
					"delete request for not ready packet: %lld rc: %d",
					i2c_set->request_id, rc);

			if (flush_req->type ==
				CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ) {
				cancel_req_id_found = 1;
				break;
			}
		}
	}

	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ &&
		!cancel_req_id_found)
		CAM_DBG(CAM_SENSOR,
			"Flush request id:%lld not found in the pending list",
			flush_req->req_id);

	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int cam_sensor_process_evt(struct cam_req_mgr_link_evt_data *evt_data)
{
	int                       rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	if (!evt_data)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(evt_data->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	CAM_DBG(CAM_SENSOR, "Received evt:%d", evt_data->evt_type);

	mutex_lock(&(s_ctrl->cam_sensor_mutex));

	switch (evt_data->evt_type) {
	case CAM_REQ_MGR_LINK_EVT_EOF:
		if (s_ctrl->stream_off_after_eof) {
			rc = cam_sensor_stream_off(s_ctrl);
			if (rc) {
				CAM_ERR(CAM_SENSOR, "Failed to stream off %s",
					s_ctrl->sensor_name);

				cam_sensor_notify_v4l2_error_event(s_ctrl,
					CAM_REQ_MGR_ERROR_TYPE_FULL_RECOVERY,
					CAM_REQ_MGR_SENSOR_STREAM_OFF_FAILED);
			}
		}
		break;
	case CAM_REQ_MGR_LINK_EVT_UPDATE_PROPERTIES:
		if (evt_data->u.properties_mask &
			CAM_LINK_PROPERTY_SENSOR_STANDBY_AFTER_EOF)
			s_ctrl->stream_off_after_eof = true;
		else
			s_ctrl->stream_off_after_eof = false;

		CAM_DBG(CAM_SENSOR, "sensor %s stream off after eof:%s",
			s_ctrl->sensor_name,
			CAM_BOOL_TO_YESNO(s_ctrl->stream_off_after_eof));
		break;
	default:
		/* No handling */
		break;
	}

	mutex_unlock(&(s_ctrl->cam_sensor_mutex));

	return rc;
}
