// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/videodev2.h>
#include <linux/uaccess.h>

#include "cam_node.h"
#include "cam_trace.h"
#include "cam_debug_util.h"

static void cam_node_print_ctx_state(
	struct cam_node *node)
{
	int i;
	struct cam_context *ctx;

	CAM_INFO(CAM_CORE, "[%s] state=%d, ctx_size %d",
		node->name, node->state, node->ctx_size);

	mutex_lock(&node->list_mutex);
	for (i = 0; i < node->ctx_size; i++) {
		ctx = &node->ctx_list[i];

		spin_lock_bh(&ctx->lock);
		CAM_INFO(CAM_CORE,
			"[%s][%d] : state=%d, refcount=%d, active_req_list=%d, pending_req_list=%d, wait_req_list=%d, free_req_list=%d",
			ctx->dev_name,
			i, ctx->state,
			atomic_read(&(ctx->refcount.refcount.refs)),
			list_empty(&ctx->active_req_list),
			list_empty(&ctx->pending_req_list),
			list_empty(&ctx->wait_req_list),
			list_empty(&ctx->free_req_list));
		spin_unlock_bh(&ctx->lock);
	}
	mutex_unlock(&node->list_mutex);
}

static struct cam_context *cam_node_get_ctxt_from_free_list(
		struct cam_node *node)
{
	struct cam_context *ctx = NULL;

	mutex_lock(&node->list_mutex);
	if (!list_empty(&node->free_ctx_list)) {
		ctx = list_first_entry(&node->free_ctx_list,
			struct cam_context, list);
		list_del_init(&ctx->list);
	}
	mutex_unlock(&node->list_mutex);
	if (ctx)
		kref_init(&ctx->refcount);
	return ctx;
}

void cam_node_put_ctxt_to_free_list(struct kref *ref)
{
	struct cam_context *ctx =
		container_of(ref, struct cam_context, refcount);
	struct cam_node *node = ctx->node;

	mutex_lock(&node->list_mutex);
	list_add_tail(&ctx->list, &node->free_ctx_list);
	mutex_unlock(&node->list_mutex);
}

static int __cam_node_handle_update_sensor_stream_cap(
	struct cam_node *node,
	struct cam_update_sensor_stream_cfg_cmd *update_sensor_stream)
{
	int rc = 0;

	if (!update_sensor_stream || !node) {
		CAM_ERR(CAM_CORE, "Invalid params");
		return -EINVAL;
	}

	if (node->hw_mgr_intf.hw_update_sensor_grp_stream_cfg) {
		rc = node->hw_mgr_intf.hw_update_sensor_grp_stream_cfg(
			node->hw_mgr_intf.hw_mgr_priv, update_sensor_stream);
	}

	if (rc)
		CAM_ERR(CAM_ISP, "update_sensor_static data failed: %d", rc);

	return rc;
}

static int __cam_node_handle_query_cap(uint32_t version,
	struct cam_node *node, struct cam_query_cap_cmd *query)
{
	struct cam_hw_mgr_intf *hw_mgr_intf = &node->hw_mgr_intf;
	int rc = 0;

	if (!query) {
		CAM_ERR(CAM_CORE, "Invalid params");
		return -EINVAL;
	}

	switch (version) {
	case CAM_QUERY_CAP:
		if (!hw_mgr_intf->hw_get_caps) {
			CAM_ERR(CAM_CORE, "Node %s query cap version: %u get hw cap intf is NULL",
				node->name, version);
			return -EINVAL;
		}
		rc = hw_mgr_intf->hw_get_caps(hw_mgr_intf->hw_mgr_priv, query);
		break;
	case CAM_QUERY_CAP_V2:
		if (!hw_mgr_intf->hw_get_caps_v2) {
			CAM_ERR(CAM_CORE, "Node %s query cap version: %u get hw cap intf is NULL",
				node->name, version);
			return -EINVAL;
		}
		rc = hw_mgr_intf->hw_get_caps_v2(hw_mgr_intf->hw_mgr_priv, query);
		break;
	default:
		CAM_ERR(CAM_CORE, "Invalid version number %u", version);
		return -EINVAL;
	}

	return rc;
}

static int __cam_node_handle_acquire_dev(struct cam_node *node,
	struct cam_acquire_dev_cmd *acquire)
{
	int rc = 0;
	struct cam_context *ctx = NULL;

	if (!acquire)
		return -EINVAL;

	ctx = cam_node_get_ctxt_from_free_list(node);
	if (!ctx) {
		CAM_ERR(CAM_CORE,
			"No free ctx in free list node %s with size:%d",
			node->name, node->ctx_size);
		cam_node_print_ctx_state(node);

		rc = -ENOMEM;
		goto err;
	}

	ctx->last_flush_req = 0;

	rc = cam_handle_validate(acquire->session_handle, acquire->session_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid session handle for acquire dev");
		goto free_ctx;
	}

	rc = cam_context_handle_acquire_dev(ctx, acquire);
	if (rc) {
		CAM_ERR(CAM_CORE, "Acquire device failed for node %s",
			node->name);
		goto free_ctx;
	}

	CAM_DBG(CAM_CORE, "[%s] Acquire ctx_id %d",
		node->name, ctx->ctx_id);

	return 0;
free_ctx:
	cam_context_putref(ctx);
err:
	return rc;
}

static void __cam_node_handle_acquired_hw_dump(
	struct cam_node *node)
{
	int i;

	for (i = 0; i < node->ctx_size; i++)
		cam_context_handle_info_dump(&(node->ctx_list[i]),
			CAM_CTX_DUMP_ACQ_INFO);
}

static int __cam_node_handle_acquire_hw_v1(struct cam_node *node,
	struct cam_acquire_hw_cmd_v1 *acquire)
{
	int rc = 0;
	struct cam_context *ctx = NULL;

	if (!acquire)
		return -EINVAL;

	rc = cam_handle_validate(acquire->session_handle, acquire->session_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid session handle for context");
		return rc;
	}

	rc = cam_handle_validate(acquire->session_handle, acquire->dev_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid device handle for context");
		return rc;
	}

	ctx = (struct cam_context *)cam_get_device_priv(acquire->dev_handle);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			acquire->dev_handle);
		return -EINVAL;
	}

	if (strcmp(node->name, ctx->dev_name)) {
		CAM_ERR(CAM_CORE, "node name %s dev name:%s not matching",
			node->name, ctx->dev_name);
		return -EINVAL;
	}

	rc = cam_context_handle_acquire_hw(ctx, acquire);
	if (rc) {
		CAM_ERR(CAM_CORE, "Acquire device failed for node %s",
			node->name);
		__cam_node_handle_acquired_hw_dump(node);
		return rc;
	}

	CAM_DBG(CAM_CORE, "[%s] Acquire ctx_id %d",
		node->name, ctx->ctx_id);

	return 0;
}

static int __cam_node_handle_acquire_hw_v2(struct cam_node *node,
	struct cam_acquire_hw_cmd_v2 *acquire)
{
	int rc = 0;
	struct cam_context *ctx = NULL;

	if (!acquire)
		return -EINVAL;

	rc = cam_handle_validate(acquire->session_handle, acquire->session_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid session handle for context");
		return rc;
	}

	rc = cam_handle_validate(acquire->session_handle, acquire->dev_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid device handle for context");
		return rc;
	}

	ctx = (struct cam_context *)cam_get_device_priv(acquire->dev_handle);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			acquire->dev_handle);
		return -EINVAL;
	}

	rc = cam_context_handle_acquire_hw(ctx, acquire);
	if (rc) {
		CAM_ERR(CAM_CORE, "Acquire device failed for node %s",
			node->name);
		__cam_node_handle_acquired_hw_dump(node);
		return rc;
	}

	CAM_DBG(CAM_CORE, "[%s] Acquire ctx_id %d",
		node->name, ctx->ctx_id);

	return 0;
}

static int __cam_node_handle_start_dev(struct cam_node *node,
	struct cam_start_stop_dev_cmd *start)
{
	struct cam_context *ctx = NULL;
	int rc;

	if (!start)
		return -EINVAL;

	rc = cam_handle_validate(start->session_handle, start->session_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid session handle for context");
		return rc;
	}

	rc = cam_handle_validate(start->session_handle, start->dev_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid device handle for context");
		return rc;
	}

	ctx = (struct cam_context *)cam_get_device_priv(start->dev_handle);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			start->dev_handle);
		return -EINVAL;
	}

	if (strcmp(node->name, ctx->dev_name)) {
		CAM_ERR(CAM_CORE, "node name %s dev name:%s not matching",
			node->name, ctx->dev_name);
		return -EINVAL;
	}

	rc = cam_context_handle_start_dev(ctx, start);
	if (rc)
		CAM_ERR(CAM_CORE, "Start failure for node %s", node->name);

	return rc;
}

static int __cam_node_handle_stop_dev(struct cam_node *node,
	struct cam_start_stop_dev_cmd *stop)
{
	struct cam_context *ctx = NULL;
	int rc;

	if (!stop)
		return -EINVAL;

	rc = cam_handle_validate(stop->session_handle, stop->session_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid session handle for context");
		return rc;
	}

	rc = cam_handle_validate(stop->session_handle, stop->dev_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid device handle for context");
		return rc;
	}

	ctx = (struct cam_context *)cam_get_device_priv(stop->dev_handle);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			stop->dev_handle);
		return -EINVAL;
	}

	if (strcmp(node->name, ctx->dev_name)) {
		CAM_ERR(CAM_CORE, "node name %s dev name:%s not matching",
			node->name, ctx->dev_name);
		return -EINVAL;
	}

	rc = cam_context_handle_stop_dev(ctx, stop);
	if (rc)
		CAM_ERR(CAM_CORE, "Stop failure for node %s", node->name);

	return rc;
}

static int __cam_node_handle_config_dev(struct cam_node *node,
	struct cam_config_dev_cmd *config)
{
	struct cam_context *ctx = NULL;
	int rc;

	if (!config)
		return -EINVAL;

	rc = cam_handle_validate(config->session_handle, config->session_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid session handle for context");
		return rc;
	}

	rc = cam_handle_validate(config->session_handle, config->dev_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid device handle for context");
		return rc;
	}

	ctx = (struct cam_context *)cam_get_device_priv(config->dev_handle);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			config->dev_handle);
		return -EINVAL;
	}

	if (strcmp(node->name, ctx->dev_name)) {
		CAM_ERR(CAM_CORE, "node name %s dev name:%s not matching",
			node->name, ctx->dev_name);
		return -EINVAL;
	}

	rc = cam_context_handle_config_dev(ctx, config);
	if (rc) {
		if (ctx->state == CAM_CTX_FLUSHED)
			CAM_INFO(CAM_CORE,
				"Config failure for node %s, it has been flushed",
				node->name);
		else
			CAM_ERR(CAM_CORE, "Config failure for node %s", node->name);
	}
	return rc;
}

static int __cam_node_handle_flush_dev(struct cam_node *node,
	struct cam_flush_dev_cmd *flush)
{
	struct cam_context *ctx = NULL;
	int rc;

	if (!flush)
		return -EINVAL;

	rc = cam_handle_validate(flush->session_handle, flush->session_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid session handle for context");
		return rc;
	}

	rc = cam_handle_validate(flush->session_handle, flush->dev_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid device handle for context");
		return rc;
	}

	ctx = (struct cam_context *)cam_get_device_priv(flush->dev_handle);
	if (!ctx) {
		CAM_ERR_RATE_LIMIT(CAM_CORE,
			"Can not get context for handle %d",
			flush->dev_handle);
		return -EINVAL;
	}

	if (strcmp(node->name, ctx->dev_name)) {
		CAM_ERR_RATE_LIMIT(CAM_CORE,
			"node name %s dev name:%s not matching",
			node->name, ctx->dev_name);
		return -EINVAL;
	}

	rc = cam_context_handle_flush_dev(ctx, flush);
	if (rc)
		CAM_ERR_RATE_LIMIT(CAM_CORE,
			"Flush failure for node %s", node->name);

	return rc;
}

static int __cam_node_handle_release_dev(struct cam_node *node,
	struct cam_release_dev_cmd *release)
{
	int rc = 0;
	struct cam_context *ctx = NULL;

	if (!release)
		return -EINVAL;

	rc = cam_handle_validate(release->session_handle, release->session_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid session handle for context");
		return rc;
	}

	rc = cam_handle_validate(release->session_handle, release->dev_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid device handle for context");
		return rc;
	}

	ctx = (struct cam_context *)cam_get_device_priv(release->dev_handle);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d node %s",
			release->dev_handle, node->name);
		return -EINVAL;
	}

	if (strcmp(node->name, ctx->dev_name)) {
		CAM_ERR(CAM_CORE, "node name %s dev name:%s not matching",
			node->name, ctx->dev_name);
		return -EINVAL;
	}

	if (ctx->state > CAM_CTX_UNINIT && ctx->state < CAM_CTX_STATE_MAX) {
		rc = cam_context_handle_release_dev(ctx, release);
		if (rc)
			CAM_ERR(CAM_CORE, "context release failed for node %s",
				node->name);
	} else {
		CAM_WARN(CAM_CORE,
			"node %s context id %u state %d invalid to release hdl",
			node->name, ctx->ctx_id, ctx->state);
		goto destroy_dev_hdl;
	}

	cam_context_putref(ctx);

destroy_dev_hdl:
	rc = cam_destroy_device_hdl(release->dev_handle);
	if (rc)
		CAM_ERR(CAM_CORE, "destroy device hdl failed for node %s",
			node->name);
	else
		ctx->dev_hdl = -1;

	CAM_DBG(CAM_CORE, "[%s] Release ctx_id=%d, refcount=%d",
		node->name, ctx->ctx_id,
		atomic_read(&(ctx->refcount.refcount.refs)));

	return rc;
}

static int __cam_node_handle_dump_dev(struct cam_node *node,
	struct cam_dump_req_cmd *dump)
{
	int                 rc;
	struct cam_context *ctx = NULL;

	if (!dump)
		return -EINVAL;

	rc = cam_handle_validate(dump->session_handle, dump->session_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid session handle for context");
		return rc;
	}

	rc = cam_handle_validate(dump->session_handle, dump->dev_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid device handle for context");
		return rc;
	}

	ctx = (struct cam_context *)cam_get_device_priv(dump->dev_handle);
	if (!ctx) {
		CAM_ERR_RATE_LIMIT(CAM_CORE,
			"Can not get context for handle %d",
			dump->dev_handle);
		return -EINVAL;
	}

	rc = cam_context_handle_dump_dev(ctx, dump);
	if (rc)
		CAM_ERR_RATE_LIMIT(CAM_CORE,
			"Dump failure for node %s", node->name);

	return rc;
}

static int __cam_node_handle_release_hw_v1(struct cam_node *node,
	struct cam_release_hw_cmd_v1 *release)
{
	int rc = 0;
	struct cam_context *ctx = NULL;

	if (!release)
		return -EINVAL;

	rc = cam_handle_validate(release->session_handle, release->session_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid session handle for context");
		return rc;
	}

	rc = cam_handle_validate(release->session_handle, release->dev_handle);
	if (rc) {
		CAM_ERR(CAM_CORE, "Invalid device handle for context");
		return rc;
	}

	ctx = (struct cam_context *)cam_get_device_priv(release->dev_handle);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d node %s",
			release->dev_handle, node->name);
		return -EINVAL;
	}

	if (strcmp(node->name, ctx->dev_name)) {
		CAM_ERR(CAM_CORE, "node name %s dev name:%s not matching",
			node->name, ctx->dev_name);
		return -EINVAL;
	}

	rc = cam_context_handle_release_hw(ctx, release);
	if (rc)
		CAM_ERR(CAM_CORE, "context release failed node %s", node->name);

	CAM_DBG(CAM_CORE, "[%s] Release ctx_id=%d, refcount=%d",
		node->name, ctx->ctx_id,
		atomic_read(&(ctx->refcount.refcount.refs)));

	return rc;
}

static int __cam_node_crm_get_dev_info(struct cam_req_mgr_device_info *info)
{
	struct cam_context *ctx = NULL;

	if (!info)
		return -EINVAL;

	ctx = (struct cam_context *) cam_get_device_priv(info->dev_hdl);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context  for handle %d",
			info->dev_hdl);
		return -EINVAL;
	}
	return cam_context_handle_crm_get_dev_info(ctx, info);
}

static int __cam_node_crm_link_setup(
	struct cam_req_mgr_core_dev_link_setup *setup)
{
	int rc;
	struct cam_context *ctx = NULL;

	if (!setup)
		return -EINVAL;

	ctx = (struct cam_context *) cam_get_device_priv(setup->dev_hdl);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			setup->dev_hdl);
		return -EINVAL;
	}

	if (setup->link_enable)
		rc = cam_context_handle_crm_link(ctx, setup);
	else
		rc = cam_context_handle_crm_unlink(ctx, setup);

	return rc;
}

static int __cam_node_crm_apply_req(struct cam_req_mgr_apply_request *apply)
{
	struct cam_context *ctx = NULL;

	if (!apply)
		return -EINVAL;

	ctx = (struct cam_context *) cam_get_device_priv(apply->dev_hdl);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			apply->dev_hdl);
		return -EINVAL;
	}

	trace_cam_apply_req("Node", ctx->ctx_id, apply->request_id, apply->link_hdl);

	return cam_context_handle_crm_apply_req(ctx, apply);
}

static int __cam_node_crm_notify_frame_skip(
	struct cam_req_mgr_apply_request *apply)
{
	struct cam_context *ctx = NULL;

	if (!apply)
		return -EINVAL;

	ctx = (struct cam_context *) cam_get_device_priv(apply->dev_hdl);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			apply->dev_hdl);
		return -EINVAL;
	}

	trace_cam_apply_req("Node", ctx->ctx_id, apply->request_id, apply->link_hdl);

	return cam_context_handle_crm_notify_frame_skip(ctx, apply);
}

static int __cam_node_crm_flush_req(struct cam_req_mgr_flush_request *flush)
{
	struct cam_context *ctx = NULL;

	if (!flush) {
		CAM_ERR(CAM_CORE, "Invalid flush request payload");
		return -EINVAL;
	}

	ctx = (struct cam_context *) cam_get_device_priv(flush->dev_hdl);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			flush->dev_hdl);
		return -EINVAL;
	}

	return cam_context_handle_crm_flush_req(ctx, flush);
}

static int __cam_node_crm_process_evt(
	struct cam_req_mgr_link_evt_data *evt_data)
{
	struct cam_context *ctx = NULL;

	if (!evt_data) {
		CAM_ERR(CAM_CORE, "Invalid process event request payload");
		return -EINVAL;
	}

	ctx = (struct cam_context *) cam_get_device_priv(evt_data->dev_hdl);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			evt_data->dev_hdl);
		return -EINVAL;
	}
	return cam_context_handle_crm_process_evt(ctx, evt_data);
}

static int __cam_node_crm_dump_req(struct cam_req_mgr_dump_info *dump)
{
	struct cam_context *ctx = NULL;

	if (!dump) {
		CAM_ERR(CAM_CORE, "Invalid dump request payload");
		return -EINVAL;
	}

	ctx = (struct cam_context *) cam_get_device_priv(dump->dev_hdl);
	if (!ctx) {
		CAM_ERR(CAM_CORE, "Can not get context for handle %d",
			dump->dev_hdl);
		return -EINVAL;
	}

	return cam_context_handle_crm_dump_req(ctx, dump);
}

int cam_node_deinit(struct cam_node *node)
{
	if (node)
		memset(node, 0, sizeof(*node));

	CAM_DBG(CAM_CORE, "deinit complete");

	return 0;
}

int cam_node_shutdown(struct cam_node *node)
{
	int i = 0;
	int rc = 0;

	if (!node)
		return -EINVAL;

	for (i = 0; i < node->ctx_size; i++) {
		if (node->ctx_list[i].dev_hdl > 0) {
			rc = cam_context_shutdown(&(node->ctx_list[i]));
			CAM_DBG(CAM_CORE,
				"Node [%s] invoking shutdown on context [%d], rc %d",
				node->name, i, rc);
		}
	}

	if (node->hw_mgr_intf.hw_close)
		node->hw_mgr_intf.hw_close(node->hw_mgr_intf.hw_mgr_priv,
			NULL);

	return 0;
}

static int __cam_node_handle_synx_test(
	struct cam_node *node, void *params)
{
	int i, rc = -EINVAL;

	for (i = 0; i < node->ctx_size; i++) {
		if (node->ctx_list[i].dev_hdl > 0) {
			CAM_ERR(CAM_CORE, "Node [%s] has active context [%d]",
				node->name, i);
			return -EAGAIN;
		}
	}

	if (node->hw_mgr_intf.synx_trigger)
		rc = node->hw_mgr_intf.synx_trigger(
			node->hw_mgr_intf.hw_mgr_priv, params);

	return rc;
}

int cam_node_init(struct cam_node *node, struct cam_hw_mgr_intf *hw_mgr_intf,
	struct cam_context *ctx_list, uint32_t ctx_size, char *name)
{
	int rc = 0;
	int i;

	if (!node || !hw_mgr_intf ||
		sizeof(node->hw_mgr_intf) != sizeof(*hw_mgr_intf)) {
		return -EINVAL;
	}

	strscpy(node->name, name, sizeof(node->name));

	memcpy(&node->hw_mgr_intf, hw_mgr_intf, sizeof(node->hw_mgr_intf));
	node->crm_node_intf.apply_req = __cam_node_crm_apply_req;
	node->crm_node_intf.get_dev_info = __cam_node_crm_get_dev_info;
	node->crm_node_intf.link_setup = __cam_node_crm_link_setup;
	node->crm_node_intf.flush_req = __cam_node_crm_flush_req;
	node->crm_node_intf.process_evt = __cam_node_crm_process_evt;
	node->crm_node_intf.dump_req = __cam_node_crm_dump_req;
	node->crm_node_intf.notify_frame_skip =
		__cam_node_crm_notify_frame_skip;

	mutex_init(&node->list_mutex);
	INIT_LIST_HEAD(&node->free_ctx_list);
	node->ctx_list = ctx_list;
	node->ctx_size = ctx_size;
	for (i = 0; i < ctx_size; i++) {
		if (!ctx_list[i].state_machine) {
			CAM_ERR(CAM_CORE,
				"camera context %d is not initialized", i);
			rc = -1;
			goto err;
		}
		INIT_LIST_HEAD(&ctx_list[i].list);
		list_add_tail(&ctx_list[i].list, &node->free_ctx_list);
		ctx_list[i].node = node;
	}

	node->state = CAM_NODE_STATE_INIT;
err:
	CAM_DBG(CAM_CORE, "Exit. (rc = %d)", rc);
	return rc;
}

int cam_node_handle_ioctl(struct cam_node *node, struct cam_control *cmd)
{
	int rc = 0;

	if (!cmd)
		return -EINVAL;

	CAM_DBG(CAM_CORE, "handle cmd %d", cmd->op_code);

	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		fallthrough;
	case CAM_QUERY_CAP_V2: {
		struct cam_query_cap_cmd query;

		if (copy_from_user(&query, u64_to_user_ptr(cmd->handle),
			sizeof(query))) {
			rc = -EFAULT;
			break;
		}

		rc = __cam_node_handle_query_cap(cmd->op_code, node, &query);
		if (rc) {
			CAM_ERR(CAM_CORE, "querycap is failed(rc = %d)",
				rc);
			break;
		}

		if (copy_to_user(u64_to_user_ptr(cmd->handle), &query, sizeof(query)))
			rc = -EFAULT;

		break;
	}
	case CAM_ACQUIRE_DEV: {
		struct cam_acquire_dev_cmd acquire;

		if (copy_from_user(&acquire, u64_to_user_ptr(cmd->handle),
			sizeof(acquire))) {
			rc = -EFAULT;
			break;
		}
		rc = __cam_node_handle_acquire_dev(node, &acquire);
		if (rc) {
			CAM_ERR(CAM_CORE, "acquire device failed(rc = %d)",
				rc);
			break;
		}
		if (copy_to_user(u64_to_user_ptr(cmd->handle), &acquire,
			sizeof(acquire)))
			rc = -EFAULT;
		break;
	}
	case CAM_ACQUIRE_HW: {
		uint32_t api_version;
		void *acquire_ptr = NULL;
		size_t acquire_size;

		if (copy_from_user(&api_version, (void __user *)cmd->handle,
			sizeof(api_version))) {
			rc = -EFAULT;
			break;
		}

		if (api_version == 1) {
			acquire_size = sizeof(struct cam_acquire_hw_cmd_v1);
		} else if (api_version == 2) {
			acquire_size = sizeof(struct cam_acquire_hw_cmd_v2);
		} else {
			CAM_ERR(CAM_CORE, "Unsupported api version %d",
				api_version);
			rc = -EINVAL;
			break;
		}

		acquire_ptr = kzalloc(acquire_size, GFP_KERNEL);
		if (!acquire_ptr) {
			CAM_ERR(CAM_CORE, "No memory for acquire HW");
			rc = -ENOMEM;
			break;
		}

		if (copy_from_user(acquire_ptr, (void __user *)cmd->handle,
			acquire_size)) {
			rc = -EFAULT;
			goto acquire_kfree;
		}

		if (api_version == 1) {
			rc = __cam_node_handle_acquire_hw_v1(node, acquire_ptr);
			if (rc) {
				CAM_ERR(CAM_CORE,
					"acquire hw failed(rc = %d)", rc);
				goto acquire_kfree;
			}
		} else if (api_version == 2) {
			rc = __cam_node_handle_acquire_hw_v2(node, acquire_ptr);
			if (rc) {
				CAM_ERR(CAM_CORE,
					"acquire hw failed(rc = %d)", rc);
				goto acquire_kfree;
			}
		}

		if (copy_to_user((void __user *)cmd->handle, acquire_ptr,
			acquire_size))
			rc = -EFAULT;

acquire_kfree:
		kfree(acquire_ptr);
		break;
	}
	case CAM_START_DEV: {
		struct cam_start_stop_dev_cmd start;

		if (copy_from_user(&start, u64_to_user_ptr(cmd->handle),
			sizeof(start)))
			rc = -EFAULT;
		else {
			rc = __cam_node_handle_start_dev(node, &start);
			if (rc)
				CAM_ERR(CAM_CORE,
					"start device failed(rc = %d)", rc);
		}
		break;
	}
	case CAM_STOP_DEV: {
		struct cam_start_stop_dev_cmd stop;

		if (copy_from_user(&stop, u64_to_user_ptr(cmd->handle),
			sizeof(stop)))
			rc = -EFAULT;
		else {
			rc = __cam_node_handle_stop_dev(node, &stop);
			if (rc)
				CAM_ERR(CAM_CORE,
					"stop device failed(rc = %d)", rc);
		}
		break;
	}
	case CAM_CONFIG_DEV: {
		struct cam_config_dev_cmd config;

		if (copy_from_user(&config, u64_to_user_ptr(cmd->handle),
			sizeof(config)))
			rc = -EFAULT;
		else {
			rc = __cam_node_handle_config_dev(node, &config);
			if (rc)
				CAM_ERR(CAM_CORE,
					"config device failed(rc = %d)", rc);
		}
		break;
	}
	case CAM_RELEASE_DEV: {
		struct cam_release_dev_cmd release;

		if (copy_from_user(&release, u64_to_user_ptr(cmd->handle),
			sizeof(release)))
			rc = -EFAULT;
		else {
			rc = __cam_node_handle_release_dev(node, &release);
			if (rc)
				CAM_ERR(CAM_CORE,
					"release device failed(rc = %d)", rc);
		}
		break;
	}
	case CAM_RELEASE_HW: {
		uint32_t api_version;
		size_t release_size;
		void *release_ptr = NULL;

		if (copy_from_user(&api_version, (void __user *)cmd->handle,
			sizeof(api_version))) {
			rc = -EFAULT;
			break;
		}

		if (api_version == 1) {
			release_size = sizeof(struct cam_release_hw_cmd_v1);
		} else {
			CAM_ERR(CAM_CORE, "Unsupported api version %d",
				api_version);
			rc = -EINVAL;
			break;
		}

		release_ptr = kzalloc(release_size, GFP_KERNEL);
		if (!release_ptr) {
			CAM_ERR(CAM_CORE, "No memory for release HW");
			rc = -ENOMEM;
			break;
		}

		if (copy_from_user(release_ptr, (void __user *)cmd->handle,
			release_size)) {
			rc = -EFAULT;
			goto release_kfree;
		}

		if (api_version == 1) {
			rc = __cam_node_handle_release_hw_v1(node, release_ptr);
			if (rc)
				CAM_ERR(CAM_CORE,
					"release device failed(rc = %d)", rc);
		}

release_kfree:
		kfree(release_ptr);
		break;
	}
	case CAM_FLUSH_REQ: {
		struct cam_flush_dev_cmd flush;

		if (copy_from_user(&flush, u64_to_user_ptr(cmd->handle),
			sizeof(flush)))
			rc = -EFAULT;
		else {
			rc = __cam_node_handle_flush_dev(node, &flush);
			if (rc)
				CAM_ERR(CAM_CORE,
					"flush device failed(rc = %d)", rc);
		}
		break;
	}
	case CAM_DUMP_REQ: {
		struct cam_dump_req_cmd dump;

		if (copy_from_user(&dump, u64_to_user_ptr(cmd->handle),
			sizeof(dump))) {
			rc = -EFAULT;
			break;
		}
		rc = __cam_node_handle_dump_dev(node, &dump);
		if (rc) {
			CAM_ERR(CAM_CORE,
			    "Dump device %s failed(rc = %d) ",
			    node->name, rc);
			break;
		}
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&dump, sizeof(dump))) {
			CAM_ERR(CAM_CORE,
			    "Dump device %s copy_to_user fail",
			    node->name);
			rc = -EFAULT;
		}
		break;
	}
	case CAM_SYNX_TEST_TRIGGER: {
		struct cam_synx_test_params synx_params;

		if (copy_from_user(&synx_params, u64_to_user_ptr(cmd->handle),
			sizeof(synx_params))) {
			rc = -EFAULT;
			break;
		}

		rc = __cam_node_handle_synx_test(node, &synx_params);
		if (rc)
			CAM_ERR(CAM_CORE, "Synx test on %s failed(rc = %d)",
			    node->name, rc);
		break;
	}
	case CAM_UPDATE_SENSOR_STREAM_CONFIG: {
		struct cam_update_sensor_stream_cfg_cmd update_sensor_stream;

		if (copy_from_user(&update_sensor_stream, u64_to_user_ptr(cmd->handle),
			sizeof(update_sensor_stream)))
			rc = -EFAULT;
		else {
			rc = __cam_node_handle_update_sensor_stream_cap(node,
				&update_sensor_stream);
			if (rc)
				CAM_ERR(CAM_CORE,
					"update sensor stream config failed(rc = %d)", rc);
		}
		break;
	}
	default:
		CAM_ERR(CAM_CORE, "Unknown op code %d", cmd->op_code);
		rc = -EINVAL;
	}

	return rc;
}
