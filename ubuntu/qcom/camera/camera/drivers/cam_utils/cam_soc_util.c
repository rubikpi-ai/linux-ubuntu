// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_opp.h>

#include "cam_soc_util.h"
#include "cam_debug_util.h"
#include "cam_cx_ipeak.h"
#include "cam_mem_mgr.h"
#include "cam_presil_hw_access.h"
#include "cam_compat.h"

#if IS_ENABLED(CONFIG_QCOM_CRM)
#include <soc/qcom/crm.h>
#include <linux/clk/qcom.h>
#endif

#define CAM_TO_MASK(bitn)          (1 << (int)(bitn))
#define CAM_IS_BIT_SET(mask, bit)  ((mask) & CAM_TO_MASK(bit))
#define CAM_SET_BIT(mask, bit)     ((mask) |= CAM_TO_MASK(bit))
#define CAM_CLEAR_BIT(mask, bit)   ((mask) &= ~CAM_TO_MASK(bit))

#define CAM_SS_START_PRESIL 0x08c00000
#define CAM_SS_START        0x0ac00000

#define CAM_CLK_DIRNAME "clk"

static uint skip_mmrm_set_rate;
module_param(skip_mmrm_set_rate, uint, 0644);

/**
 * struct cam_clk_wrapper_clk: This represents an entry corresponding to a
 *                             shared clock in Clk wrapper. Clients that share
 *                             the same clock are registered to this clk entry
 *                             and set rate from them is consolidated before
 *                             setting it to clk driver.
 *
 * @list:           List pointer to point to next shared clk entry
 * @clk_id:         Clk Id of this clock
 * @curr_clk_rate:  Current clock rate set for this clock
 * @client_list:    List of clients registered to this shared clock entry
 * @num_clients:    Number of registered clients
 * @active_clients: Number of active clients
 * @mmrm_client:    MMRM Client handle for src clock
 * @soc_info:       soc_info of client with which mmrm handle is created.
 *                  This is used as unique identifier for a client and mmrm
 *                  callback data. When client corresponds to this soc_info is
 *                  unregistered, need to unregister mmrm handle as well.
 * @is_nrt_dev:     Whether this clock corresponds to NRT device
 * @min_clk_rate:   Minimum clk rate that this clock supports
 **/
struct cam_clk_wrapper_clk {
	struct list_head list;
	uint32_t clk_id;
	int64_t curr_clk_rate;
	struct list_head client_list;
	uint32_t num_clients;
	uint32_t active_clients;
	void *mmrm_handle;
	struct cam_hw_soc_info *soc_info;
	bool is_nrt_dev;
	int64_t min_clk_rate;
};

/**
 * struct cam_clk_wrapper_client: This represents a client (device) that wants
 *                                to share the clock with some other client.
 *
 * @list:           List pointer to point to next client that share the
 *                  same clock
 * @soc_info:       soc_info of client. This is used as unique identifier
 *                  for a client
 * @clk:            Clk handle
 * @curr_clk_rate:  Current clock rate set for this client
 **/
struct cam_clk_wrapper_client {
	struct list_head list;
	struct cam_hw_soc_info *soc_info;
	struct clk *clk;
	int64_t curr_clk_rate;
};

static char supported_clk_info[256];

static DEFINE_MUTEX(wrapper_lock);
static LIST_HEAD(wrapper_clk_list);

#define CAM_IS_VALID_CESTA_IDX(idx) ((idx >= 0) && (idx < CAM_CESTA_MAX_CLIENTS))

#define CAM_CRM_DEV_IDENTIFIER "cam_crm"

const struct device *cam_cesta_crm_dev;

#if IS_ENABLED(CONFIG_QCOM_CRM) && IS_ENABLED(CONFIG_SPECTRA_USE_CLK_CRM_API)
static int cam_soc_util_set_hw_client_rate_through_mmrm(
	void *mmrm_handle, long low_val, long high_val,
	uint32_t num_hw_blocks, int cesta_client_idx);
#endif

#if IS_ENABLED(CONFIG_QCOM_CRM)
static inline const struct device *cam_wrapper_crm_get_device(
	const char *name)
{
	if (debug_bypass_drivers & CAM_BYPASS_CESTA) {
		CAM_WARN(CAM_UTIL, "Bypass crm get device");
		return (const struct device *)BYPASS_VALUE;
	}

	return crm_get_device(name);
}

static inline int cam_wrapper_crm_write_pwr_states(const struct device *dev,
	u32 drv_id)
{
	if (debug_bypass_drivers & CAM_BYPASS_CESTA) {
		CAM_WARN(CAM_UTIL, "Bypass crm write pwr states");
		return 0;
	}

	return crm_write_pwr_states(cam_cesta_crm_dev, drv_id);
}

#endif

#if IS_ENABLED(CONFIG_QCOM_CRM) && IS_ENABLED(CONFIG_SPECTRA_USE_CLK_CRM_API)
static inline int cam_wrapper_qcom_clk_crm_set_rate(struct clk *clk,
	enum crm_drv_type client_type, u32 client_idx,
	u32 pwr_st, unsigned long rate)
{
	if (debug_bypass_drivers & CAM_BYPASS_CESTA) {
		CAM_WARN(CAM_UTIL, "Bypass qcom clk crm set rate");
		return 0;
	}

	return qcom_clk_crm_set_rate(clk, client_type, client_idx, pwr_st, rate);
}
#endif

static inline int cam_wrapper_clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (debug_bypass_drivers & CAM_BYPASS_CLKS) {
		CAM_WARN(CAM_UTIL, "Bypass clk set rate");
		return 0;
	}

	return clk_set_rate(clk, rate);
}

static inline long cam_wrapper_clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (debug_bypass_drivers & CAM_BYPASS_CLKS) {
		CAM_WARN(CAM_UTIL, "Bypass clk round rate");
		return rate;
	}

	return clk_round_rate(clk, rate);
}

inline unsigned long cam_wrapper_clk_get_rate(struct clk *clk)
{
	if (debug_bypass_drivers & CAM_BYPASS_CLKS) {
		CAM_WARN(CAM_UTIL, "Bypass clk get rate");
		return DEFAULT_CLK_VALUE;
	}

	return clk_get_rate(clk);
}

static inline struct clk *cam_wrapper_clk_get(struct device *dev, const char *id)
{
	if (debug_bypass_drivers & CAM_BYPASS_CLKS) {
		CAM_WARN(CAM_UTIL, "Bypass clk get");
		return (struct clk *)BYPASS_VALUE;
	}

	return clk_get(dev, id);
}

static inline void cam_wrapper_clk_put(struct clk *clk)
{
	if (debug_bypass_drivers & CAM_BYPASS_CLKS) {
		CAM_WARN(CAM_UTIL, "Bypass clk put");
		return;
	}

	clk_put(clk);
}

static inline struct clk *cam_wrapper_of_clk_get_from_provider(
	struct of_phandle_args *clkspec)
{
	if (debug_bypass_drivers & CAM_BYPASS_CLKS) {
		CAM_WARN(CAM_UTIL, "Bypass of clk get from provider");
		return (struct clk *)BYPASS_VALUE;
	}

	return of_clk_get_from_provider(clkspec);
}

static inline int cam_wrapper_clk_prepare_enable(struct clk *clk)
{
	if (debug_bypass_drivers & CAM_BYPASS_CLKS) {
		CAM_WARN(CAM_UTIL, "Bypass clk prepare enable");
		return 0;
	}

	return clk_prepare_enable(clk);
}

static inline void cam_wrapper_clk_disable_unprepare(struct clk *clk)
{
	if (debug_bypass_drivers & CAM_BYPASS_CLKS) {
		CAM_WARN(CAM_UTIL, "Bypass clk disable unprepare");
		return;
	}

	clk_disable_unprepare(clk);
}

static inline struct regulator *cam_wrapper_regulator_get(struct device *dev,
	const char *id)
{
	if (debug_bypass_drivers & CAM_BYPASS_RGLTR) {
		CAM_WARN(CAM_UTIL, "Bypass regulator get");
		return (struct regulator *)BYPASS_VALUE;
	}

	return regulator_get(dev, id);
}

static inline void cam_wrapper_regulator_put(struct regulator *regulator)
{
	if (debug_bypass_drivers & CAM_BYPASS_RGLTR) {
		CAM_WARN(CAM_UTIL, "Bypass regulator put");
		return;
	}

	regulator_put(regulator);
}

static inline int cam_wrapper_regulator_disable(struct regulator *regulator)
{
	if (debug_bypass_drivers & CAM_BYPASS_RGLTR) {
		CAM_WARN(CAM_UTIL, "Bypass regulator disable");
		return 0;
	}

	return regulator_disable(regulator);
}

static inline int cam_wrapper_regulator_enable(struct regulator *regulator)
{
	if (debug_bypass_drivers & CAM_BYPASS_RGLTR) {
		CAM_WARN(CAM_UTIL, "Bypass regulator enable");
		return 0;
	}

	return regulator_enable(regulator);
}

static inline int cam_wrapper_regulator_set_voltage(
	struct regulator *regulator, int min_uV, int max_uV)
{
	if (debug_bypass_drivers & CAM_BYPASS_RGLTR) {
		CAM_WARN(CAM_UTIL, "Bypass regulator set voltage");
		return 0;
	}

	return regulator_set_voltage(regulator, min_uV, max_uV);
}

static inline int cam_wrapper_regulator_count_voltages(
	struct regulator *regulator)
{
	if (debug_bypass_drivers & CAM_BYPASS_RGLTR) {
		CAM_WARN(CAM_UTIL, "Bypass regulator count voltages");
		return 0;
	}

	return regulator_count_voltages(regulator);
}

inline int cam_wrapper_regulator_set_load(
	struct regulator *regulator, int uA_load)
{
	if (debug_bypass_drivers & CAM_BYPASS_RGLTR) {
		CAM_WARN(CAM_UTIL, "Bypass regulator set load");
		return 0;
	}

	return regulator_set_load(regulator, uA_load);
}

inline int cam_wrapper_regulator_set_mode(
	struct regulator *regulator, unsigned int mode)
{
	if (debug_bypass_drivers & CAM_BYPASS_RGLTR_MODE) {
		CAM_WARN(CAM_UTIL, "Bypass regulator set mode");
		return 0;
}

	return regulator_set_mode(regulator, mode);
}

static inline int cam_wrapper_regulator_is_enabled(
	struct regulator *regulator)
{
	if (debug_bypass_drivers & CAM_BYPASS_RGLTR) {
		CAM_WARN(CAM_UTIL, "Bypass regulator is enabled");
		return 0;
	}

	return regulator_is_enabled(regulator);
}

inline void cam_soc_util_set_bypass_drivers(
	uint32_t bypass_drivers)
{
	debug_bypass_drivers = bypass_drivers;
	CAM_INFO(CAM_UTIL, "bypass drivers %d", debug_bypass_drivers);
}

#if IS_ENABLED(CONFIG_QCOM_CRM)
inline int cam_soc_util_cesta_populate_crm_device(void)
{
	cam_cesta_crm_dev =  cam_wrapper_crm_get_device(CAM_CRM_DEV_IDENTIFIER);
	if (!cam_cesta_crm_dev) {
		CAM_ERR(CAM_UTIL, "Failed to get cesta crm dev for %s", CAM_CRM_DEV_IDENTIFIER);
		return -ENODEV;
	}

	return 0;
}

int cam_soc_util_cesta_channel_switch(uint32_t cesta_client_idx, const char *identifier)
{
	int rc = 0;

	if (!cam_cesta_crm_dev) {
		CAM_ERR(CAM_UTIL, "camera cesta crm device is null");
		return -EINVAL;
	}

	if (!CAM_IS_VALID_CESTA_IDX(cesta_client_idx)) {
		CAM_ERR(CAM_UTIL, "Invalid client index for camera cesta idx: %d max: %d",
			cesta_client_idx, CAM_CESTA_MAX_CLIENTS);
		return -EINVAL;
	}

	CAM_DBG(CAM_PERF, "CESTA Channel switch : hw client idx %d identifier=%s",
		cesta_client_idx, identifier);

	rc = cam_wrapper_crm_write_pwr_states(cam_cesta_crm_dev, cesta_client_idx);
	if (rc) {
		CAM_ERR(CAM_UTIL,
			"Failed to trigger cesta channel switch cesta_client_idx: %u rc: %d",
			cesta_client_idx, rc);
		return rc;
	}

	return rc;
}
#else
inline int cam_soc_util_cesta_populate_crm_device(void)
{
	CAM_ERR(CAM_UTIL, "Not supported");

	return -EOPNOTSUPP;
}

inline int cam_soc_util_cesta_channel_switch(uint32_t cesta_client_idx, const char *identifier)
{
	CAM_ERR(CAM_UTIL, "Not supported, cesta_client_idx=%d, identifier=%s",
		cesta_client_idx, identifier);

	return -EOPNOTSUPP;
}
#endif

#if IS_ENABLED(CONFIG_QCOM_CRM) && IS_ENABLED(CONFIG_SPECTRA_USE_CLK_CRM_API)
static int cam_soc_util_set_cesta_clk_rate(struct cam_hw_soc_info *soc_info,
	uint32_t cesta_client_idx, unsigned long high_val, unsigned long low_val,
	unsigned long *applied_high_val, unsigned long *applied_low_val)
{
	int32_t src_clk_idx;
	struct clk *clk = NULL;
	int rc = 0;

	if (!soc_info || (soc_info->src_clk_idx < 0) ||
		(soc_info->src_clk_idx >= CAM_SOC_MAX_CLK)) {
		CAM_ERR(CAM_UTIL, "Invalid src_clk_idx: %d",
			soc_info ? soc_info->src_clk_idx : -1);
		return -EINVAL;
	}

	if (!CAM_IS_VALID_CESTA_IDX(cesta_client_idx)) {
		CAM_ERR(CAM_UTIL, "Invalid client index for camera cesta idx: %d max: %d",
			cesta_client_idx, CAM_CESTA_MAX_CLIENTS);
		return -EINVAL;
	}

	/* Only source clocks are supported by this API to set HW client clock votes */
	src_clk_idx = soc_info->src_clk_idx;
	clk = soc_info->clk[src_clk_idx];

	if (!skip_mmrm_set_rate && soc_info->mmrm_handle) {
		CAM_DBG(CAM_UTIL, "cesta mmrm hw client: set %s, high-rate %lld low-rate %lld",
			soc_info->clk_name[src_clk_idx], high_val, low_val);

		rc = cam_soc_util_set_hw_client_rate_through_mmrm(
			soc_info->mmrm_handle, low_val, high_val, 1,
			cesta_client_idx);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"set_sw_client_rate through mmrm failed on %s clk_id %d low_val %llu high_val %llu client idx=%d",
				soc_info->clk_name[src_clk_idx], soc_info->clk_id[src_clk_idx],
				low_val, high_val, cesta_client_idx);
			return rc;
		}
		goto end;
	}

	CAM_DBG(CAM_UTIL, "%s Requested clk rate [high low]: [%llu %llu] cesta_client_idx: %d",
		soc_info->clk_name[src_clk_idx], high_val, low_val, cesta_client_idx);

	rc = cam_wrapper_qcom_clk_crm_set_rate(
		clk, CRM_HW_DRV, cesta_client_idx, CRM_PWR_STATE1, high_val);
	if (rc) {
		CAM_ERR(CAM_UTIL,
			"Failed in setting cesta high clk rate, client idx: %u pwr state: %u clk_val: %llu rc: %d",
			cesta_client_idx, CRM_PWR_STATE1, high_val, rc);
		return rc;
	}

	rc = cam_wrapper_qcom_clk_crm_set_rate(
		clk, CRM_HW_DRV, cesta_client_idx, CRM_PWR_STATE0, low_val);
	if (rc) {
		CAM_ERR(CAM_UTIL,
			"Failed in setting cesta low clk rate, client idx: %u pwr state: %u clk_val: %llu rc: %d",
			cesta_client_idx, CRM_PWR_STATE0, low_val, rc);
		return rc;
	}

end:
	if (applied_high_val)
		*applied_high_val = high_val;

	if (applied_low_val)
		*applied_low_val = low_val;

	return rc;
}

#if IS_REACHABLE(CONFIG_MSM_MMRM)
int cam_soc_util_set_hw_client_rate_through_mmrm(
	void *mmrm_handle, long low_val, long high_val,
	uint32_t num_hw_blocks, int cesta_client_idx)
{
	int rc = 0;
	struct mmrm_client_data client_data;

	client_data.num_hw_blocks = num_hw_blocks;
	client_data.crm_drv_idx = cesta_client_idx;
	client_data.drv_type = MMRM_CRM_HW_DRV;
	client_data.pwr_st = CRM_PWR_STATE1;
	client_data.flags = 0;

	CAM_DBG(CAM_UTIL,
		"hw client mmrm=%pK, high_val %ld, low_val %ld, num_blocks=%d, pwr_state: %u, client_idx: %d",
		mmrm_handle, high_val, low_val, num_hw_blocks, CRM_PWR_STATE1, cesta_client_idx);

	rc = mmrm_client_set_value((struct mmrm_client *)mmrm_handle,
		&client_data, high_val);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Set high rate failed rate %ld rc %d",
			high_val, rc);
		return rc;
	}

	/* We vote a second time for pwr_st = low */
	client_data.pwr_st = CRM_PWR_STATE0;

	rc = mmrm_client_set_value((struct mmrm_client *)mmrm_handle,
		&client_data, low_val);
	if (rc)
		CAM_ERR(CAM_UTIL, "Set low rate failed rate %ld rc %d", low_val, rc);

	return rc;
}

#else
int cam_soc_util_set_hw_client_rate_through_mmrm(
	void *mmrm_handle, long low_val, long high_val,
	uint32_t num_hw_blocks, int cesta_client_idx)
{
	return 0;
}
#endif

#else
static inline int cam_soc_util_set_cesta_clk_rate(struct cam_hw_soc_info *soc_info,
	uint32_t cesta_client_idx, unsigned long high_val, unsigned long low_val,
	unsigned long *applied_high_val, unsigned long *applied_low_val)
{
	CAM_ERR(CAM_UTIL, "Not supported, dev=%s, cesta_client_idx=%d, high_val=%ld, low_val=%ld",
		soc_info->dev_name, cesta_client_idx, high_val, low_val);

	return -EOPNOTSUPP;
}
#endif

#if IS_REACHABLE(CONFIG_MSM_MMRM)
bool cam_is_mmrm_supported_on_current_chip(void)
{
	bool is_supported;

	is_supported = mmrm_client_check_scaling_supported(MMRM_CLIENT_CLOCK,
			MMRM_CLIENT_DOMAIN_CAMERA);
	CAM_DBG(CAM_UTIL, "is mmrm supported: %s",
			CAM_BOOL_TO_YESNO(is_supported));;

	return is_supported;
}

int cam_mmrm_notifier_callback(
	struct mmrm_client_notifier_data *notifier_data)
{
	if (!notifier_data) {
		CAM_ERR(CAM_UTIL, "Invalid notifier data");
		return -EBADR;
	}

	if (notifier_data->cb_type == MMRM_CLIENT_RESOURCE_VALUE_CHANGE) {
		struct cam_hw_soc_info *soc_info = notifier_data->pvt_data;

		CAM_WARN(CAM_UTIL, "Dev %s Clk %s value change from %ld to %ld",
			soc_info->dev_name,
			(soc_info->src_clk_idx == -1) ? "No src clk" :
			soc_info->clk_name[soc_info->src_clk_idx],
			notifier_data->cb_data.val_chng.old_val,
			notifier_data->cb_data.val_chng.new_val);
	}

	return 0;
}

int cam_soc_util_register_mmrm_client(
	uint32_t clk_id, struct clk *clk, bool is_nrt_dev,
	struct cam_hw_soc_info *soc_info, const char *clk_name,
	void **mmrm_handle)
{
	struct mmrm_client *mmrm_client;
	struct mmrm_client_desc desc = { };

	if (!mmrm_handle) {
		CAM_ERR(CAM_UTIL, "Invalid mmrm input");
		return -EINVAL;
	}

	*mmrm_handle = (void *)NULL;

	if (debug_bypass_drivers & CAM_BYPASS_CLKS) {
		CAM_WARN(CAM_UTIL, "Bypass register mmrm client");
		return 0;
	}

	if (!cam_is_mmrm_supported_on_current_chip())
		return 0;

	desc.client_type = MMRM_CLIENT_CLOCK;
	desc.client_info.desc.client_domain = MMRM_CLIENT_DOMAIN_CAMERA;
	desc.client_info.desc.client_id = clk_id;
	desc.client_info.desc.clk = clk;

#if IS_ENABLED(CONFIG_QCOM_CRM) && IS_ENABLED(CONFIG_SPECTRA_USE_CLK_CRM_API)
	if (soc_info->is_clk_drv_en) {
		desc.client_info.desc.hw_drv_instances = CAM_CESTA_MAX_CLIENTS;
		desc.client_info.desc.num_pwr_states = CAM_NUM_PWR_STATES;
	} else {
		desc.client_info.desc.hw_drv_instances = 0;
		desc.client_info.desc.num_pwr_states = 0;
	}
#endif

	snprintf((char *)desc.client_info.desc.name,
		sizeof(desc.client_info.desc.name), "%s_%s",
		soc_info->dev_name, clk_name);

	desc.priority = is_nrt_dev ?
		MMRM_CLIENT_PRIOR_LOW : MMRM_CLIENT_PRIOR_HIGH;
	desc.pvt_data = soc_info;
	desc.notifier_callback_fn = cam_mmrm_notifier_callback;

	mmrm_client = mmrm_client_register(&desc);
	if (!mmrm_client) {
		CAM_ERR(CAM_UTIL, "MMRM Register failed Dev %s clk %s id %d",
			soc_info->dev_name, clk_name, clk_id);
		return -EINVAL;
	}

	CAM_DBG(CAM_UTIL,
		"MMRM Register success Dev %s is_nrt_dev %d clk %s id %d handle=%pK",
		soc_info->dev_name, is_nrt_dev, clk_name, clk_id, mmrm_client);

	*mmrm_handle = (void *)mmrm_client;

	return 0;
}

int cam_soc_util_unregister_mmrm_client(
	void *mmrm_handle)
{
	int rc = 0;

	CAM_DBG(CAM_UTIL, "MMRM UnRegister handle=%pK", mmrm_handle);

	if (mmrm_handle) {
		rc = mmrm_client_deregister((struct mmrm_client *)mmrm_handle);
		if (rc)
			CAM_ERR(CAM_UTIL,
				"Failed in deregister handle=%pK, rc %d",
				mmrm_handle, rc);
	}

	return rc;
}

static int cam_soc_util_set_sw_client_rate_through_mmrm(
	void *mmrm_handle, bool is_nrt_dev, long min_rate,
	long req_rate, uint32_t num_hw_blocks)
{
	int rc = 0;
	struct mmrm_client_data client_data;
	struct mmrm_client_res_value val;

	client_data.num_hw_blocks = num_hw_blocks;
	client_data.flags = 0;

#if IS_ENABLED(CONFIG_QCOM_CRM) && IS_ENABLED(CONFIG_SPECTRA_USE_CLK_CRM_API)
	client_data.drv_type = MMRM_CRM_SW_DRV;
#endif

	CAM_DBG(CAM_UTIL,
		"sw client mmrm=%pK, nrt=%d, min_rate=%ld req_rate %ld, num_blocks=%d",
		mmrm_handle, is_nrt_dev, min_rate, req_rate, num_hw_blocks);

	if (is_nrt_dev) {
		val.min = min_rate;
		val.cur = req_rate;

		rc = mmrm_client_set_value_in_range(
			(struct mmrm_client *)mmrm_handle, &client_data, &val);
	} else {
		rc = mmrm_client_set_value(
			(struct mmrm_client *)mmrm_handle,
			&client_data, req_rate);
	}

	if (rc)
		CAM_ERR(CAM_UTIL, "Set rate failed rate %ld rc %d",
			req_rate, rc);

	return rc;
}
#else
int cam_soc_util_register_mmrm_client(
	uint32_t clk_id, struct clk *clk, bool is_nrt_dev,
	struct cam_hw_soc_info *soc_info, const char *clk_name,
	void **mmrm_handle)
{
	if (!mmrm_handle) {
		CAM_ERR(CAM_UTIL, "Invalid mmrm input");
		return -EINVAL;
	}

	*mmrm_handle = NULL;

	return 0;
}

int cam_soc_util_unregister_mmrm_client(
	void *mmrm_handle)
{
	return 0;
}

static int cam_soc_util_set_sw_client_rate_through_mmrm(
	void *mmrm_handle, bool is_nrt_dev, long min_rate,
	long req_rate, uint32_t num_hw_blocks)
{
	return 0;
}
#endif

static int cam_soc_util_clk_wrapper_register_entry(
	uint32_t clk_id, struct clk *clk, bool is_src_clk,
	struct cam_hw_soc_info *soc_info, int64_t min_clk_rate,
	const char *clk_name)
{
	struct cam_clk_wrapper_clk *wrapper_clk;
	struct cam_clk_wrapper_client *wrapper_client;
	bool clock_found = false;
	int rc = 0;

	mutex_lock(&wrapper_lock);

	list_for_each_entry(wrapper_clk, &wrapper_clk_list, list) {
		CAM_DBG(CAM_UTIL, "Clk list id %d num clients %d",
			wrapper_clk->clk_id, wrapper_clk->num_clients);

		if (wrapper_clk->clk_id == clk_id) {
			clock_found = true;
			list_for_each_entry(wrapper_client,
				&wrapper_clk->client_list, list) {
				CAM_DBG(CAM_UTIL,
					"Clk id %d entry client %s",
					wrapper_clk->clk_id,
					wrapper_client->soc_info->dev_name);
				if (wrapper_client->soc_info == soc_info) {
					CAM_ERR(CAM_UTIL,
						"Register with same soc info, clk id %d, client %s",
						clk_id, soc_info->dev_name);
					rc = -EINVAL;
					goto end;
				}
			}
			break;
		}
	}

	if (!clock_found) {
		CAM_DBG(CAM_UTIL, "Adding new entry for clk id %d", clk_id);
		wrapper_clk = kzalloc(sizeof(struct cam_clk_wrapper_clk),
			GFP_KERNEL);
		if (!wrapper_clk) {
			CAM_ERR(CAM_UTIL,
				"Failed in allocating new clk entry %d",
				clk_id);
			rc = -ENOMEM;
			goto end;
		}

		wrapper_clk->clk_id = clk_id;
		INIT_LIST_HEAD(&wrapper_clk->list);
		INIT_LIST_HEAD(&wrapper_clk->client_list);
		list_add_tail(&wrapper_clk->list, &wrapper_clk_list);
	}
	wrapper_client = kzalloc(sizeof(struct cam_clk_wrapper_client),
		GFP_KERNEL);
	if (!wrapper_client) {
		CAM_ERR(CAM_UTIL, "Failed in allocating new client entry %d",
			clk_id);
		rc = -ENOMEM;
		goto end;
	}

	wrapper_client->soc_info = soc_info;
	wrapper_client->clk = clk;

	if (is_src_clk && !wrapper_clk->mmrm_handle) {
		wrapper_clk->is_nrt_dev = soc_info->is_nrt_dev;
		wrapper_clk->min_clk_rate = min_clk_rate;
		wrapper_clk->soc_info = soc_info;

		rc = cam_soc_util_register_mmrm_client(clk_id, clk,
			wrapper_clk->is_nrt_dev, soc_info, clk_name,
			&wrapper_clk->mmrm_handle);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"Failed in register mmrm client Dev %s clk id %d",
				soc_info->dev_name, clk_id);
			kfree(wrapper_client);
			goto end;
		}
	}

	INIT_LIST_HEAD(&wrapper_client->list);
	list_add_tail(&wrapper_client->list, &wrapper_clk->client_list);
	wrapper_clk->num_clients++;

	CAM_DBG(CAM_UTIL,
		"Adding new client %s for clk[%s] id %d, num clients %d",
		soc_info->dev_name, clk_name, clk_id, wrapper_clk->num_clients);

end:
	mutex_unlock(&wrapper_lock);
	return rc;
}

static int cam_soc_util_clk_wrapper_unregister_entry(
	uint32_t clk_id, struct cam_hw_soc_info *soc_info)
{
	struct cam_clk_wrapper_clk *wrapper_clk;
	struct cam_clk_wrapper_client *wrapper_client;
	bool clock_found = false;
	bool client_found = false;
	int rc = 0;

	mutex_lock(&wrapper_lock);

	list_for_each_entry(wrapper_clk, &wrapper_clk_list, list) {
		CAM_DBG(CAM_UTIL, "Clk list id %d num clients %d",
			wrapper_clk->clk_id, wrapper_clk->num_clients);

		if (wrapper_clk->clk_id == clk_id) {
			clock_found = true;
			list_for_each_entry(wrapper_client,
				&wrapper_clk->client_list, list) {
				CAM_DBG(CAM_UTIL, "Clk id %d entry client %s",
					wrapper_clk->clk_id,
					wrapper_client->soc_info->dev_name);
				if (wrapper_client->soc_info == soc_info) {
					client_found = true;
					break;
				}
			}
			break;
		}
	}

	if (!clock_found) {
		CAM_ERR(CAM_UTIL, "Shared clk id %d entry not found", clk_id);
		rc = -EINVAL;
		goto end;
	}

	if (!client_found) {
		CAM_ERR(CAM_UTIL,
			"Client %pK for Shared clk id %d entry not found",
			soc_info, clk_id);
		rc = -EINVAL;
		goto end;
	}

	wrapper_clk->num_clients--;
	if (wrapper_clk->mmrm_handle && (wrapper_clk->soc_info == soc_info)) {
		cam_soc_util_unregister_mmrm_client(wrapper_clk->mmrm_handle);
		wrapper_clk->mmrm_handle = NULL;
		wrapper_clk->soc_info = NULL;
	}

	list_del_init(&wrapper_client->list);
	kfree(wrapper_client);

	CAM_DBG(CAM_UTIL, "Unregister client %s for clk id %d, num clients %d",
		soc_info->dev_name, clk_id, wrapper_clk->num_clients);

	if (!wrapper_clk->num_clients) {
		list_del_init(&wrapper_clk->list);
		kfree(wrapper_clk);
	}
end:
	mutex_unlock(&wrapper_lock);
	return rc;
}

static int cam_soc_util_clk_wrapper_set_clk_rate(
	uint32_t clk_id, struct cam_hw_soc_info *soc_info,
	struct clk *clk, int64_t clk_rate)
{
	struct cam_clk_wrapper_clk *wrapper_clk;
	struct cam_clk_wrapper_client *wrapper_client;
	bool clk_found = false;
	bool client_found = false;
	int rc = 0;
	int64_t final_clk_rate = 0;
	uint32_t active_clients = 0;

	if (!soc_info || !clk) {
		CAM_ERR(CAM_UTIL, "Invalid param soc_info %pK clk %pK",
			soc_info, clk);
		return -EINVAL;
	}

	mutex_lock(&wrapper_lock);

	list_for_each_entry(wrapper_clk, &wrapper_clk_list, list) {
		CAM_DBG(CAM_UTIL, "Clk list id %d num clients %d",
			wrapper_clk->clk_id, wrapper_clk->num_clients);
		if (wrapper_clk->clk_id == clk_id) {
			clk_found = true;
			break;
		}
	}

	if (!clk_found) {
		CAM_ERR(CAM_UTIL, "Clk entry not found id %d client %s",
			clk_id, soc_info->dev_name);
		rc = -EINVAL;
		goto end;
	}

	list_for_each_entry(wrapper_client, &wrapper_clk->client_list, list) {
		CAM_DBG(CAM_UTIL, "Clk id %d client %s, clk rate %lld",
			wrapper_clk->clk_id, wrapper_client->soc_info->dev_name,
			wrapper_client->curr_clk_rate);
		if (wrapper_client->soc_info == soc_info) {
			client_found = true;
			CAM_DBG(CAM_UTIL,
				"Clk enable clk id %d, client %s curr %ld new %ld",
				clk_id, wrapper_client->soc_info->dev_name,
				wrapper_client->curr_clk_rate, clk_rate);

			wrapper_client->curr_clk_rate = clk_rate;
		}

		if (wrapper_client->curr_clk_rate > 0)
			active_clients++;

		if (final_clk_rate < wrapper_client->curr_clk_rate)
			final_clk_rate = wrapper_client->curr_clk_rate;
	}

	if (!client_found) {
		CAM_ERR(CAM_UTIL,
			"Wrapper clk enable without client entry clk id %d client %s",
			clk_id, soc_info->dev_name);
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_UTIL,
		"Clk id %d, client %s, clients rate %ld, curr %ld final %ld",
		wrapper_clk->clk_id, soc_info->dev_name, clk_rate,
		wrapper_clk->curr_clk_rate, final_clk_rate);

	if ((final_clk_rate != wrapper_clk->curr_clk_rate) ||
		(active_clients != wrapper_clk->active_clients)) {
		bool set_rate_finish = false;

		if (!skip_mmrm_set_rate && wrapper_clk->mmrm_handle) {
			rc = cam_soc_util_set_sw_client_rate_through_mmrm(
				wrapper_clk->mmrm_handle,
				wrapper_clk->is_nrt_dev,
				wrapper_clk->min_clk_rate,
				final_clk_rate, active_clients);
			if (rc) {
				CAM_ERR(CAM_UTIL,
					"set_sw_client_rate through mmrm failed clk_id %d, rate=%ld",
					wrapper_clk->clk_id, final_clk_rate);
				goto end;
			}

			set_rate_finish = true;
		}

		if (!set_rate_finish && final_clk_rate &&
			(final_clk_rate != wrapper_clk->curr_clk_rate)) {
			rc = cam_wrapper_clk_set_rate(clk, final_clk_rate);
			if (rc) {
				CAM_ERR(CAM_UTIL, "set_rate failed on clk %d",
					wrapper_clk->clk_id);
				goto end;
			}
		}

		wrapper_clk->curr_clk_rate = final_clk_rate;
		wrapper_clk->active_clients = active_clients;
	}

end:
	mutex_unlock(&wrapper_lock);
	return rc;
}

int cam_soc_util_get_clk_level(struct cam_hw_soc_info *soc_info,
	int64_t clk_rate, int clk_idx, int32_t *clk_lvl)
{
	int i;
	long clk_rate_round;

	if (!soc_info || (clk_idx < 0) || (clk_idx >= CAM_SOC_MAX_CLK)) {
		CAM_ERR(CAM_UTIL, "Invalid src_clk_idx: %d", clk_idx);
		*clk_lvl = -1;
		return -EINVAL;
	}

	clk_rate_round = cam_wrapper_clk_round_rate(
		soc_info->clk[clk_idx], clk_rate);
	if (clk_rate_round < 0) {
		CAM_ERR(CAM_UTIL, "round failed rc = %ld",
			clk_rate_round);
		*clk_lvl = -1;
		return -EINVAL;
	}

	if (debug_bypass_drivers & CAM_BYPASS_CLKS) {
		CAM_WARN(CAM_UTIL, "Bypass get clk level");
		*clk_lvl = CAM_NOMINAL_VOTE;
		return 0;
	}

	for (i = 0; i < CAM_MAX_VOTE; i++) {
		if ((soc_info->clk_level_valid[i]) &&
			(soc_info->clk_rate[i][clk_idx] >=
			clk_rate_round)) {
			CAM_DBG(CAM_UTIL,
				"soc = %d round rate = %ld actual = %lld",
				soc_info->clk_rate[i][clk_idx],
				clk_rate_round, clk_rate);
			*clk_lvl = i;
			return 0;
		}
	}

	CAM_WARN(CAM_UTIL, "Invalid clock rate %ld", clk_rate_round);
	*clk_lvl = -1;
	return -EINVAL;
}

const char *cam_soc_util_get_string_from_level(enum cam_vote_level level)
{
	switch (level) {
	case CAM_SUSPEND_VOTE:
		return "";
	case CAM_MINSVS_VOTE:
		return "MINSVS[1]";
	case CAM_LOWSVS_D1_VOTE:
		return "LOWSVSD1[2]";
	case CAM_LOWSVS_VOTE:
		return "LOWSVS[3]";
	case CAM_SVS_VOTE:
		return "SVS[4]";
	case CAM_SVSL1_VOTE:
		return "SVSL1[5]";
	case CAM_NOMINAL_VOTE:
		return "NOM[6]";
	case CAM_NOMINALL1_VOTE:
		return "NOML1[7]";
	case CAM_TURBO_VOTE:
		return "TURBO[8]";
	default:
		return "";
	}
}

/**
 * cam_soc_util_get_supported_clk_levels()
 *
 * @brief:      Returns the string of all the supported clk levels for
 *              the given device
 *
 * @soc_info:   Device soc information
 *
 * @return:     String containing all supported clk levels
 */
static const char *cam_soc_util_get_supported_clk_levels(
	struct cam_hw_soc_info *soc_info)
{
	int i = 0;

	scnprintf(supported_clk_info, sizeof(supported_clk_info), "Supported levels: ");

	for (i = 0; i < CAM_MAX_VOTE; i++) {
		if (soc_info->clk_level_valid[i] == true) {
			strlcat(supported_clk_info,
				cam_soc_util_get_string_from_level(i),
				sizeof(supported_clk_info));
			strlcat(supported_clk_info, " ",
				sizeof(supported_clk_info));
		}
	}

	strlcat(supported_clk_info, "\n", sizeof(supported_clk_info));
	return supported_clk_info;
}

static int cam_soc_util_clk_lvl_options_open(struct inode *inode,
	struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t cam_soc_util_clk_lvl_options_read(struct file *file,
	char __user *clk_info, size_t size_t, loff_t *loff_t)
{
	struct cam_hw_soc_info *soc_info =
		(struct cam_hw_soc_info *)file->private_data;
	const char *display_string =
		cam_soc_util_get_supported_clk_levels(soc_info);

	return simple_read_from_buffer(clk_info, size_t, loff_t, display_string,
		strlen(display_string));
}

static const struct file_operations cam_soc_util_clk_lvl_options = {
	.open = cam_soc_util_clk_lvl_options_open,
	.read = cam_soc_util_clk_lvl_options_read,
};

static int cam_soc_util_set_clk_lvl_override(void *data, u64 val)
{
	struct cam_hw_soc_info *soc_info = (struct cam_hw_soc_info *)data;

	if ((val <= CAM_SUSPEND_VOTE) || (val >= CAM_MAX_VOTE)) {
		CAM_WARN(CAM_UTIL, "Invalid clk lvl override %d", val);
		return 0;
	}

	if (soc_info->clk_level_valid[val])
		soc_info->clk_level_override_high = val;
	else
		soc_info->clk_level_override_high = 0;

	return 0;
}

static int cam_soc_util_get_clk_lvl_override(void *data, u64 *val)
{
	struct cam_hw_soc_info *soc_info = (struct cam_hw_soc_info *)data;

	*val = soc_info->clk_level_override_high;

	return 0;
}

static int cam_soc_util_set_clk_lvl_override_low(void *data, u64 val)
{
	struct cam_hw_soc_info *soc_info = (struct cam_hw_soc_info *)data;

	if ((val <= CAM_SUSPEND_VOTE) || (val >= CAM_MAX_VOTE)) {
		CAM_WARN(CAM_UTIL, "Invalid clk lvl override %d", val);
		return 0;
	}

	if (soc_info->clk_level_valid[val])
		soc_info->clk_level_override_low = val;
	else
		soc_info->clk_level_override_low = 0;

	return 0;
}

static int cam_soc_util_get_clk_lvl_override_low(void *data, u64 *val)
{
	struct cam_hw_soc_info *soc_info = (struct cam_hw_soc_info *)data;

	*val = soc_info->clk_level_override_low;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_soc_util_clk_lvl_control,
	cam_soc_util_get_clk_lvl_override, cam_soc_util_set_clk_lvl_override, "%08llu");

DEFINE_SIMPLE_ATTRIBUTE(cam_soc_util_clk_lvl_control_low,
	cam_soc_util_get_clk_lvl_override_low, cam_soc_util_set_clk_lvl_override_low, "%08llu");


/**
 * cam_soc_util_create_clk_lvl_debugfs()
 *
 * @brief:      Creates debugfs files to view/control device clk rates
 *
 * @soc_info:   Device soc information
 *
 * @return:     Success or failure
 */
static int cam_soc_util_create_clk_lvl_debugfs(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;
	struct dentry *clkdirptr = NULL;

	if (!cam_debugfs_available())
		return 0;

	if (soc_info->dentry) {
		CAM_DBG(CAM_UTIL, "Debugfs entry for %s already exists",
			soc_info->dev_name);
		goto end;
	}

	rc = cam_debugfs_lookup_subdir(CAM_CLK_DIRNAME, &clkdirptr);
	if (rc) {
		rc = cam_debugfs_create_subdir(CAM_CLK_DIRNAME, &clkdirptr);
		if (rc) {
			CAM_ERR(CAM_UTIL, "DebugFS could not create clk directory!");
			rc = -ENOENT;
			goto end;
		}
	}

	soc_info->dentry = debugfs_create_dir(soc_info->dev_name, clkdirptr);
	if (IS_ERR_OR_NULL(soc_info->dentry)) {
		CAM_ERR(CAM_UTIL, "DebugFS could not create directory for dev:%s!",
			soc_info->dev_name);
		rc = -ENOENT;
		goto end;
	}
	/* Store parent inode for cleanup in caller */
	debugfs_create_file("clk_lvl_options", 0444,
		soc_info->dentry, soc_info, &cam_soc_util_clk_lvl_options);
	debugfs_create_file("clk_lvl_control", 0644,
		soc_info->dentry, soc_info, &cam_soc_util_clk_lvl_control);
	debugfs_create_file("clk_lvl_control_low", 0644,
		soc_info->dentry, soc_info, &cam_soc_util_clk_lvl_control_low);
end:
	return rc;
}

int cam_soc_util_get_level_from_string(const char *string,
	enum cam_vote_level *level)
{
	if (!level)
		return -EINVAL;

	if (!strcmp(string, "suspend")) {
		*level = CAM_SUSPEND_VOTE;
	} else if (!strcmp(string, "minsvs")) {
		*level = CAM_MINSVS_VOTE;
	} else if (!strcmp(string, "lowsvsd1")) {
		*level = CAM_LOWSVS_D1_VOTE;
	} else if (!strcmp(string, "lowsvs")) {
		*level = CAM_LOWSVS_VOTE;
	} else if (!strcmp(string, "svs")) {
		*level = CAM_SVS_VOTE;
	} else if (!strcmp(string, "svs_l1")) {
		*level = CAM_SVSL1_VOTE;
	} else if (!strcmp(string, "nominal")) {
		*level = CAM_NOMINAL_VOTE;
	} else if (!strcmp(string, "nominal_l1")) {
		*level = CAM_NOMINALL1_VOTE;
	} else if (!strcmp(string, "turbo")) {
		*level = CAM_TURBO_VOTE;
	} else {
		CAM_ERR(CAM_UTIL, "Invalid string %s", string);
		return -EINVAL;
	}

	return 0;
}

/**
 * cam_soc_util_get_clk_level_to_apply()
 *
 * @brief:              Get the clock level to apply. If the requested level
 *                      is not valid, bump the level to next available valid
 *                      level. If no higher level found, return failure.
 *
 * @soc_info:           Device soc struct to be populated
 * @req_level:          Requested level
 * @apply_level         Level to apply
 *
 * @return:             success or failure
 */
static int cam_soc_util_get_clk_level_to_apply(
	struct cam_hw_soc_info *soc_info, enum cam_vote_level req_level,
	enum cam_vote_level *apply_level)
{
	if (req_level >= CAM_MAX_VOTE) {
		CAM_ERR(CAM_UTIL, "Invalid clock level parameter %d",
			req_level);
		return -EINVAL;
	}

	if (soc_info->clk_level_valid[req_level] == true) {
		*apply_level = req_level;
	} else {
		int i;

		for (i = (req_level + 1); i < CAM_MAX_VOTE; i++)
			if (soc_info->clk_level_valid[i] == true) {
				*apply_level = i;
				break;
			}

		if (i == CAM_MAX_VOTE) {
			CAM_ERR(CAM_UTIL,
				"No valid clock level found to apply, req=%d",
				req_level);
			return -EINVAL;
		}
	}

	CAM_DBG(CAM_UTIL, "Req level %s, Applying %s",
		cam_soc_util_get_string_from_level(req_level),
		cam_soc_util_get_string_from_level(*apply_level));

	return 0;
}

unsigned long cam_soc_util_get_clk_rate_applied(
	struct cam_hw_soc_info *soc_info, int32_t index, bool is_src,
	enum cam_vote_level clk_level)
{
	unsigned long clk_rate = 0;
	struct clk *clk = NULL;
	int rc = 0;
	enum cam_vote_level apply_level;

	if (is_src) {
		clk = soc_info->clk[index];
		clk_rate = cam_wrapper_clk_get_rate(clk);
	} else {
		rc = cam_soc_util_get_clk_level_to_apply(soc_info, clk_level,
			&apply_level);
		if (rc)
			return rc;
		if (soc_info->clk_rate[apply_level][index] > 0) {
			clk = soc_info->clk[index];
			clk_rate = cam_wrapper_clk_get_rate(clk);
		}
	}
	return clk_rate;
}

int cam_soc_util_irq_enable(struct cam_hw_soc_info *soc_info)
{
	int i, rc = 0;

	if (!soc_info) {
		CAM_ERR(CAM_UTIL, "Invalid arguments");
		return -EINVAL;
	}

	for (i = 0; i < soc_info->irq_count; i++) {
		if (soc_info->irq_num[i] < 0) {
			CAM_ERR(CAM_UTIL, "No IRQ line available for irq: %s dev: %s",
				soc_info->irq_name[i], soc_info->dev_name);
			rc = -ENODEV;
			goto disable_irq;
		}

		enable_irq(soc_info->irq_num[i]);
	}

	return rc;

disable_irq:
	for (i = i - 1; i >= 0; i--)
		disable_irq(soc_info->irq_num[i]);

	return rc;
}

int cam_soc_util_irq_disable(struct cam_hw_soc_info *soc_info)
{
	int i, rc = 0;

	if (!soc_info) {
		CAM_ERR(CAM_UTIL, "Invalid arguments");
		return -EINVAL;
	}

	for (i = 0; i < soc_info->irq_count; i++) {
		if (soc_info->irq_num[i] < 0) {
			CAM_ERR(CAM_UTIL, "No IRQ line available irq: %s dev:",
				soc_info->irq_name[i], soc_info->dev_name);
			rc = -ENODEV;
			continue;
		}

		disable_irq(soc_info->irq_num[i]);
	}

	return rc;
}

long cam_soc_util_get_clk_round_rate(struct cam_hw_soc_info *soc_info,
	uint32_t clk_index, unsigned long clk_rate)
{
	if (!soc_info || (clk_index >= soc_info->num_clk) || (clk_rate == 0)) {
		CAM_ERR(CAM_UTIL, "Invalid input params %pK, %d %lu",
			soc_info, clk_index, clk_rate);
		return clk_rate;
	}

	return cam_wrapper_clk_round_rate(soc_info->clk[clk_index], clk_rate);
}

/**
 * cam_soc_util_set_clk_rate()
 *
 * @brief:            Sets the given rate for the clk requested for
 *
 * @clk:              Clock structure information for which rate is to be set
 * @clk_name:         Name of the clock for which rate is being set
 * @clk_rate:         Clock rate to be set
 * @shared_clk:       Whether this is a shared clk
 * @is_src_clk:       Whether this is source clk
 * @clk_id:           Clock ID
 * @applied_clk_rate: Final clock rate set to the clk
 *
 * @return:         Success or failure
 */
static int cam_soc_util_set_clk_rate(struct cam_hw_soc_info *soc_info,
	struct clk *clk, const char *clk_name,
	int64_t clk_rate, bool shared_clk, bool is_src_clk, uint32_t clk_id,
	unsigned long *applied_clk_rate)
{
	int rc = 0;
	long clk_rate_round = -1;
	bool set_rate = false;

	if (!clk_name) {
		CAM_ERR(CAM_UTIL, "Invalid input clk %pK clk_name %pK",
			clk, clk_name);
		return -EINVAL;
	}

	CAM_DBG(CAM_UTIL, "set %s, rate %lld", clk_name, clk_rate);
	if (!clk)
		return 0;
	if (clk_rate > 0) {
		clk_rate_round = cam_wrapper_clk_round_rate(clk, clk_rate);
		CAM_DBG(CAM_UTIL, "new_rate %ld", clk_rate_round);
		if (clk_rate_round < 0) {
			CAM_ERR(CAM_UTIL, "round failed for clock %s rc = %ld",
				clk_name, clk_rate_round);
			return clk_rate_round;
		}
		set_rate = true;
	} else if (clk_rate == INIT_RATE) {
		clk_rate_round = cam_wrapper_clk_get_rate(clk);
		CAM_DBG(CAM_UTIL, "init new_rate %ld", clk_rate_round);
		if (clk_rate_round == 0) {
			clk_rate_round = cam_wrapper_clk_round_rate(clk, 0);
			if (clk_rate_round <= 0) {
				CAM_ERR(CAM_UTIL, "round rate failed on %s",
					clk_name);
				return clk_rate_round;
			}
		}
		set_rate = true;
	}

	if (set_rate) {
		if (shared_clk) {
			CAM_DBG(CAM_UTIL,
				"Dev %s clk %s id %d Set Shared clk %ld",
				soc_info->dev_name, clk_name, clk_id,
				clk_rate_round);
			cam_soc_util_clk_wrapper_set_clk_rate(
				clk_id, soc_info, clk, clk_rate_round);
		} else {
			bool set_rate_finish = false;

			CAM_DBG(CAM_UTIL,
				"Dev %s clk %s clk_id %d src_idx %d src_clk_id %d",
				soc_info->dev_name, clk_name, clk_id,
				soc_info->src_clk_idx,
				(soc_info->src_clk_idx == -1) ? -1 :
				soc_info->clk_id[soc_info->src_clk_idx]);

			if (is_src_clk && soc_info->mmrm_handle &&
				!skip_mmrm_set_rate) {
				uint32_t idx = soc_info->src_clk_idx;
				uint32_t min_level = soc_info->lowest_clk_level;

				rc = cam_soc_util_set_sw_client_rate_through_mmrm(
					soc_info->mmrm_handle,
					soc_info->is_nrt_dev,
					soc_info->clk_rate[min_level][idx],
					clk_rate_round, 1);

				if (rc) {
					CAM_ERR(CAM_UTIL,
						"set_sw_client_rate through mmrm failed on %s clk_id %d, rate=%ld",
						clk_name, clk_id, clk_rate_round);
					return rc;
				}
				set_rate_finish = true;
			}

			if (!set_rate_finish) {
				rc = cam_wrapper_clk_set_rate(clk, clk_rate_round);
				if (rc) {
					CAM_ERR(CAM_UTIL, "set_rate failed on %s", clk_name);
					return rc;
				}
			}
		}
	}

	if (applied_clk_rate)
		*applied_clk_rate = clk_rate_round;

	return rc;
}

int cam_soc_util_set_src_clk_rate(struct cam_hw_soc_info *soc_info, int cesta_client_idx,
	unsigned long clk_rate_high, unsigned long clk_rate_low)
{
	int rc = 0;
	int i = 0;
	int32_t src_clk_idx;
	int32_t scl_clk_idx;
	struct clk *clk = NULL;
	int32_t apply_level;
	uint32_t clk_level_override_high = 0, clk_level_override_low = 0;

	if (!soc_info || (soc_info->src_clk_idx < 0) ||
		(soc_info->src_clk_idx >= CAM_SOC_MAX_CLK)) {
		CAM_ERR(CAM_UTIL, "Invalid src_clk_idx: %d",
			soc_info ? soc_info->src_clk_idx : -1);
		return -EINVAL;
	}

	src_clk_idx = soc_info->src_clk_idx;
	clk_level_override_high = soc_info->clk_level_override_high;
	clk_level_override_low = soc_info->clk_level_override_low;

	if (clk_level_override_high && clk_rate_high)
		clk_rate_high = soc_info->clk_rate[clk_level_override_high][src_clk_idx];

	if (clk_level_override_low && clk_rate_low)
		clk_rate_low = soc_info->clk_rate[clk_level_override_low][src_clk_idx];

	clk = soc_info->clk[src_clk_idx];
	rc = cam_soc_util_get_clk_level(soc_info, clk_rate_high, src_clk_idx,
		&apply_level);
	if (rc || (apply_level < 0) || (apply_level >= CAM_MAX_VOTE)) {
		CAM_ERR(CAM_UTIL,
			"set %s, rate %lld dev_name = %s apply level = %d",
			soc_info->clk_name[src_clk_idx], clk_rate_high,
			soc_info->dev_name, apply_level);
			return -EINVAL;
	}

	CAM_DBG(CAM_UTIL,
		"set %s, cesta_client_idx: %d rate [%ld %ld] dev_name = %s apply level = %d",
		soc_info->clk_name[src_clk_idx], cesta_client_idx, clk_rate_high, clk_rate_low,
		soc_info->dev_name, apply_level);

	if ((soc_info->cam_cx_ipeak_enable) && (clk_rate_high > 0)) {
		cam_cx_ipeak_update_vote_cx_ipeak(soc_info,
			apply_level);
	}

	rc = dev_pm_opp_set_rate(soc_info->dev,
			  soc_info->clk_rate[apply_level][soc_info->src_clk_idx]);
	if (rc) {
		CAM_ERR(CAM_UTIL,
			"Unable to set operating point for dev %s clk_name %s rc %d",
			soc_info->dev_name, soc_info->clk_name[soc_info->src_clk_idx],
			rc);
		return rc;
	}

	if (soc_info->is_clk_drv_en && CAM_IS_VALID_CESTA_IDX(cesta_client_idx)) {
		rc = cam_soc_util_set_cesta_clk_rate(soc_info, cesta_client_idx, clk_rate_high,
			clk_rate_low,
			&soc_info->applied_src_clk_rates.hw_client[cesta_client_idx].high,
			&soc_info->applied_src_clk_rates.hw_client[cesta_client_idx].low);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"Failed in setting cesta clk rates[high low]:[%ld %ld] client_idx:%d rc:%d",
				clk_rate_high, clk_rate_low, cesta_client_idx, rc);
			return rc;
		}
		goto end;
	}

	rc = cam_soc_util_set_clk_rate(soc_info, clk,
		soc_info->clk_name[src_clk_idx], clk_rate_high,
		CAM_IS_BIT_SET(soc_info->shared_clk_mask, src_clk_idx),
		true, soc_info->clk_id[src_clk_idx],
		&soc_info->applied_src_clk_rates.sw_client);
	if (rc) {
		CAM_ERR(CAM_UTIL,
			"SET_RATE Failed: src clk: %s, rate %lld, dev_name = %s rc: %d",
			soc_info->clk_name[src_clk_idx], clk_rate_high,
			soc_info->dev_name, rc);
		return rc;
	}

	/* set clk rate for scalable clk if available */

	for (i = 0; i < soc_info->scl_clk_count; i++) {
		scl_clk_idx = soc_info->scl_clk_idx[i];
		if (scl_clk_idx < 0) {
			CAM_DBG(CAM_UTIL, "Scl clk index invalid");
			continue;
		}
		clk = soc_info->clk[scl_clk_idx];
		rc = cam_soc_util_set_clk_rate(soc_info, clk,
			soc_info->clk_name[scl_clk_idx],
			soc_info->clk_rate[apply_level][scl_clk_idx],
			CAM_IS_BIT_SET(soc_info->shared_clk_mask, scl_clk_idx),
			false, soc_info->clk_id[scl_clk_idx],
			NULL);
		if (rc) {
			CAM_WARN(CAM_UTIL,
			"SET_RATE Failed: scl clk: %s, rate %d dev_name = %s, rc: %d",
			soc_info->clk_name[scl_clk_idx],
			soc_info->clk_rate[apply_level][scl_clk_idx],
			soc_info->dev_name, rc);
		}
	}

end:
	return 0;
}

int cam_soc_util_put_optional_clk(struct cam_hw_soc_info *soc_info,
	int32_t clk_indx)
{
	if (clk_indx < 0) {
		CAM_ERR(CAM_UTIL, "Invalid params clk %d", clk_indx);
		return -EINVAL;
	}

	if (CAM_IS_BIT_SET(soc_info->optional_shared_clk_mask, clk_indx))
		cam_soc_util_clk_wrapper_unregister_entry(
			soc_info->optional_clk_id[clk_indx], soc_info);

	cam_wrapper_clk_put(soc_info->optional_clk[clk_indx]);
	soc_info->optional_clk[clk_indx] = NULL;

	return 0;
}

static struct clk *cam_soc_util_option_clk_get(struct device_node *np,
	int index, uint32_t *clk_id)
{
	struct of_phandle_args clkspec;
	struct clk *clk;
	int rc;

	if (index < 0)
		return ERR_PTR(-EINVAL);

	rc = of_parse_phandle_with_args(np, "clocks-option", "#clock-cells",
		index, &clkspec);
	if (rc)
		return ERR_PTR(rc);

	clk = cam_wrapper_of_clk_get_from_provider(&clkspec);

	*clk_id = clkspec.args[0];
	of_node_put(clkspec.np);

	return clk;
}

int cam_soc_util_get_option_clk_by_name(struct cam_hw_soc_info *soc_info,
	const char *clk_name, int32_t *clk_index)
{
	int index = 0;
	int rc = 0;
	struct device_node *of_node = NULL;
	uint32_t shared_clk_val;

	if (!soc_info || !clk_name || !clk_index) {
		CAM_ERR(CAM_UTIL,
			"Invalid params soc_info %pK clk_name %s clk_index %pK",
			soc_info, clk_name, clk_index);
		return -EINVAL;
	}

	of_node = soc_info->dev->of_node;

	index = of_property_match_string(of_node, "clock-names-option",
		clk_name);
	if (index < 0) {
		CAM_DBG(CAM_UTIL, "No clk data for %s", clk_name);
		*clk_index = -1;
		return -EINVAL;
	}

	if (index >= CAM_SOC_MAX_OPT_CLK) {
		CAM_ERR(CAM_UTIL, "Insufficient optional clk entries %d %d",
			index, CAM_SOC_MAX_OPT_CLK);
		return -EINVAL;
	}

	of_property_read_string_index(of_node, "clock-names-option",
		index, &(soc_info->optional_clk_name[index]));

	soc_info->optional_clk[index] = cam_soc_util_option_clk_get(of_node,
		index, &soc_info->optional_clk_id[index]);
	if (IS_ERR(soc_info->optional_clk[index])) {
		CAM_ERR(CAM_UTIL, "No clk named %s found. Dev %s", clk_name,
			soc_info->dev_name);
		*clk_index = -1;
		return -EFAULT;
	}
	*clk_index = index;

	rc = of_property_read_u32_index(of_node, "clock-rates-option",
		index, &soc_info->optional_clk_rate[index]);
	if (rc) {
		CAM_ERR(CAM_UTIL,
			"Error reading clock-rates clk_name %s index %d",
			clk_name, index);
		goto error;
	}

	/*
	 * Option clocks are assumed to be available to single Device here.
	 * Hence use INIT_RATE instead of NO_SET_RATE.
	 */
	soc_info->optional_clk_rate[index] =
		(soc_info->optional_clk_rate[index] == 0) ?
		(int32_t)INIT_RATE : soc_info->optional_clk_rate[index];

	CAM_DBG(CAM_UTIL, "clk_name %s index %d clk_rate %d",
		clk_name, *clk_index, soc_info->optional_clk_rate[index]);

	rc = of_property_read_u32_index(of_node, "shared-clks-option",
		index, &shared_clk_val);
	if (rc) {
		CAM_DBG(CAM_UTIL, "Not shared clk  %s index %d",
			clk_name, index);
	} else if (shared_clk_val > 1) {
		CAM_WARN(CAM_UTIL, "Invalid shared clk val %d", shared_clk_val);
	} else {
		CAM_DBG(CAM_UTIL,
			"Dev %s shared clk  %s index %d, clk id %d, shared_clk_val %d",
			soc_info->dev_name, clk_name, index,
			soc_info->optional_clk_id[index], shared_clk_val);

		if (shared_clk_val) {
			CAM_SET_BIT(soc_info->optional_shared_clk_mask, index);

			/* Create a wrapper entry if this is a shared clock */
			CAM_DBG(CAM_UTIL,
				"Dev %s, clk %s, id %d register wrapper entry for shared clk",
				soc_info->dev_name,
				soc_info->optional_clk_name[index],
				soc_info->optional_clk_id[index]);

			rc = cam_soc_util_clk_wrapper_register_entry(
				soc_info->optional_clk_id[index],
				soc_info->optional_clk[index], false,
				soc_info,
				soc_info->optional_clk_rate[index],
				soc_info->optional_clk_name[index]);
			if (rc) {
				CAM_ERR(CAM_UTIL,
					"Failed in registering shared clk Dev %s id %d",
					soc_info->dev_name,
					soc_info->optional_clk_id[index]);
				goto error;
			}
		}
	}

	return 0;
error:
	cam_wrapper_clk_put(soc_info->optional_clk[index]);
	soc_info->optional_clk_rate[index] = 0;
	soc_info->optional_clk[index] = NULL;
	*clk_index = -1;

	return rc;
}

int cam_soc_util_clk_enable(struct cam_hw_soc_info *soc_info, int cesta_client_idx,
	bool optional_clk, int32_t clk_idx, int32_t apply_level)
{
	int rc = 0;
	struct clk *clk;
	const char *clk_name;
	unsigned long clk_rate;
	uint32_t shared_clk_mask;
	uint32_t clk_id;
	bool is_src_clk = false;

	if (!soc_info || (clk_idx < 0) || (apply_level >= CAM_MAX_VOTE)) {
		CAM_ERR(CAM_UTIL, "Invalid param %d %d", clk_idx, apply_level);
		return -EINVAL;
	}

	if (optional_clk) {
		clk = soc_info->optional_clk[clk_idx];
		clk_name = soc_info->optional_clk_name[clk_idx];
		clk_rate = (apply_level == -1) ?
			0 : soc_info->optional_clk_rate[clk_idx];
		shared_clk_mask = soc_info->optional_shared_clk_mask;
		clk_id = soc_info->optional_clk_id[clk_idx];
	} else {
		clk = soc_info->clk[clk_idx];
		clk_name = soc_info->clk_name[clk_idx];
		clk_rate = (apply_level == -1) ?
			0 : soc_info->clk_rate[apply_level][clk_idx];
		shared_clk_mask = soc_info->shared_clk_mask;
		clk_id = soc_info->clk_id[clk_idx];
		if (clk_idx == soc_info->src_clk_idx)
			is_src_clk = true;
	}
	if (!clk)
		return 0;

	if (is_src_clk && soc_info->is_clk_drv_en && CAM_IS_VALID_CESTA_IDX(cesta_client_idx)) {
		rc = cam_soc_util_set_cesta_clk_rate(soc_info, cesta_client_idx, clk_rate, clk_rate,
			&soc_info->applied_src_clk_rates.hw_client[cesta_client_idx].high,
			&soc_info->applied_src_clk_rates.hw_client[cesta_client_idx].low);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"[%s] Failed in setting cesta clk rates[high low]:[%ld %ld] client_idx:%d rc:%d",
				soc_info->dev_name, clk_rate, clk_rate, cesta_client_idx, rc);
			return rc;
		}

		rc = cam_soc_util_cesta_channel_switch(cesta_client_idx, soc_info->dev_name);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"[%s] Failed to apply power states for cesta client:%d rc:%d",
				soc_info->dev_name, cesta_client_idx, rc);
			return rc;
		}
	} else {
		rc = cam_soc_util_set_clk_rate(soc_info, clk, clk_name, clk_rate,
			CAM_IS_BIT_SET(shared_clk_mask, clk_idx), is_src_clk, clk_id,
			&soc_info->applied_src_clk_rates.sw_client);
		if (rc) {
			CAM_ERR(CAM_UTIL, "[%s] Failed in setting clk rate %ld rc:%d",
				soc_info->dev_name, clk_rate, rc);
			return rc;
		}
	}

	CAM_DBG(CAM_UTIL, "[%s] : clk enable %s", soc_info->dev_name, clk_name);
	rc = cam_wrapper_clk_prepare_enable(clk);
	if (rc) {
		CAM_ERR(CAM_UTIL, "enable failed for %s: rc(%d)", clk_name, rc);
		return rc;
	}

	return rc;
}

int cam_soc_util_clk_disable(struct cam_hw_soc_info *soc_info, int cesta_client_idx,
	bool optional_clk, int32_t clk_idx)
{
	int rc = 0;
	struct clk *clk;
	const char *clk_name;
	uint32_t shared_clk_mask;
	uint32_t clk_id;

	if (!soc_info || (clk_idx < 0)) {
		CAM_ERR(CAM_UTIL, "Invalid param %d", clk_idx);
		return -EINVAL;
	}

	if (optional_clk) {
		clk = soc_info->optional_clk[clk_idx];
		clk_name = soc_info->optional_clk_name[clk_idx];
		shared_clk_mask = soc_info->optional_shared_clk_mask;
		clk_id = soc_info->optional_clk_id[clk_idx];
	} else {
		clk = soc_info->clk[clk_idx];
		clk_name = soc_info->clk_name[clk_idx];
		shared_clk_mask = soc_info->shared_clk_mask;
		clk_id = soc_info->clk_id[clk_idx];
	}

	CAM_DBG(CAM_UTIL, "disable %s", clk_name);
	if (!clk)
		return 0;

	cam_wrapper_clk_disable_unprepare(clk);

	if ((clk_idx == soc_info->src_clk_idx) && soc_info->is_clk_drv_en &&
		CAM_IS_VALID_CESTA_IDX(cesta_client_idx)) {
		rc = cam_soc_util_set_cesta_clk_rate(soc_info, cesta_client_idx, 0, 0,
			&soc_info->applied_src_clk_rates.hw_client[cesta_client_idx].high,
			&soc_info->applied_src_clk_rates.hw_client[cesta_client_idx].low);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"Failed in setting cesta clk rates[high low]:[0 0] client_idx:%d rc:%d",
				cesta_client_idx, rc);
			return rc;
		}

		rc = cam_soc_util_cesta_channel_switch(cesta_client_idx, soc_info->dev_name);
		if (rc) {
			CAM_ERR(CAM_CSIPHY,
				"Failed to apply power states for cesta_client_idx:%d rc:%d",
				cesta_client_idx, rc);
			return rc;
		}
	} else {
		if (CAM_IS_BIT_SET(shared_clk_mask, clk_idx)) {
			CAM_DBG(CAM_UTIL,
				"Dev %s clk %s Disabling Shared clk, set 0 rate",
				soc_info->dev_name, clk_name);
			cam_soc_util_clk_wrapper_set_clk_rate(clk_id, soc_info, clk, 0);
		} else if (soc_info->mmrm_handle && (!skip_mmrm_set_rate) &&
				(soc_info->src_clk_idx == clk_idx)) {
			CAM_DBG(CAM_UTIL, "Dev %s Disabling %s clk, set 0 rate",
				soc_info->dev_name, clk_name);
			cam_soc_util_set_sw_client_rate_through_mmrm(
				soc_info->mmrm_handle,
				soc_info->is_nrt_dev,
				0, 0, 1);
		}
	}

	return 0;
}

void cam_soc_util_power_domain_disable_default(
	struct cam_hw_soc_info *soc_info)
{
	int i, ret = 0, num_pds = soc_info->num_genpd;

	if (num_pds < 1) {
		CAM_DBG(CAM_UTIL,
			"power-domains not defined for dev %s num_pds = %d",
			num_pds, soc_info->dev_name);
		return;
	}

	if (num_pds == 1) {
		dev_pm_genpd_set_performance_state(soc_info->dev, 0);
		ret = pm_runtime_put_sync(soc_info->dev);
		CAM_DBG(CAM_UTIL,
			"power-domain disabled for dev %s ret %d",
			soc_info->dev_name, ret);
		return;
	}

	for (i = num_pds - 1; i >= 0; i--) {
		if (!soc_info->genpd)
			continue;

		if (!soc_info->genpd[i])
			continue;

		dev_pm_genpd_set_performance_state(soc_info->genpd[i], 0);
		pm_runtime_put(soc_info->genpd[i]);
	}
}

/**
 * cam_soc_util_clk_enable_default()
 *
 * @brief:              This function enables the default clocks present
 *                      in soc_info
 *
 * @soc_info:           Device soc struct to be populated
 * @cesta_client_idx:   CESTA Client idx for hw client based src clocks
 * @clk_level:          Clk level to apply while enabling
 *
 * @return:             success or failure
 */
int cam_soc_util_clk_enable_default(struct cam_hw_soc_info *soc_info,
	int cesta_client_idx, enum cam_vote_level clk_level)
{
	int                          i, rc = 0;
	enum cam_vote_level          apply_level;

	if ((soc_info->num_clk == 0) ||
		(soc_info->num_clk >= CAM_SOC_MAX_CLK)) {
		CAM_ERR(CAM_UTIL, "Invalid number of clock %d",
			soc_info->num_clk);
		return -EINVAL;
	}

	rc = cam_soc_util_get_clk_level_to_apply(soc_info, clk_level,
		&apply_level);
	if (rc) {
		CAM_ERR(CAM_UTIL, "[%s] : failed to get level clk_level=%d, rc=%d",
			soc_info->dev_name, clk_level, rc);
		return rc;
	}

	if (soc_info->cam_cx_ipeak_enable)
		cam_cx_ipeak_update_vote_cx_ipeak(soc_info, apply_level);

	rc = dev_pm_opp_set_rate(soc_info->dev,
			soc_info->clk_rate[apply_level][soc_info->src_clk_idx]);
	if (rc) {
		CAM_ERR(CAM_UTIL,
			"Unable to set operating point for dev %s clk_name %s rc %d",
			soc_info->dev_name, soc_info->clk_name[soc_info->src_clk_idx],
			rc);
		return rc;
	}

	CAM_DBG(CAM_UTIL, "Dev[%s] : cesta client %d, request level %s, apply level %s",
		soc_info->dev_name, cesta_client_idx,
		cam_soc_util_get_string_from_level(clk_level),
		cam_soc_util_get_string_from_level(apply_level));

	memset(&soc_info->applied_src_clk_rates, 0, sizeof(struct cam_soc_util_clk_rates));

	for (i = 0; i < soc_info->num_clk; i++) {
		rc = cam_soc_util_clk_enable(soc_info, cesta_client_idx, false, i, apply_level);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"[%s] : failed to enable clk apply_level=%d, rc=%d, cesta_client_idx=%d",
				soc_info->dev_name, apply_level, rc, cesta_client_idx);
			goto clk_disable;
		}

		if (soc_info->cam_cx_ipeak_enable)
			CAM_DBG(CAM_UTIL,
				"dev name = %s clk name = %s idx = %d apply_level = %d clc idx = %d",
				soc_info->dev_name, soc_info->clk_name[i], i, apply_level, i);
	}

	return rc;

clk_disable:
	if (soc_info->cam_cx_ipeak_enable)
		cam_cx_ipeak_update_vote_cx_ipeak(soc_info, 0);
	for (i--; i >= 0; i--) {
		cam_soc_util_clk_disable(soc_info, cesta_client_idx, false, i);
	}

	return rc;
}

/**
 * cam_soc_util_clk_disable_default()
 *
 * @brief:              This function disables the default clocks present
 *                      in soc_info
 *
 * @soc_info:           device soc struct to be populated
 * @cesta_client_idx:   CESTA Client idx for hw client based src clocks
 *
 * @return:             success or failure
 */
void cam_soc_util_clk_disable_default(struct cam_hw_soc_info *soc_info,
	int cesta_client_idx)
{
	int i;

	if (soc_info->num_clk == 0)
		return;

	if (soc_info->cam_cx_ipeak_enable)
		cam_cx_ipeak_unvote_cx_ipeak(soc_info);

	dev_pm_opp_set_rate(soc_info->dev, 0);

	for (i = soc_info->num_clk - 1; i >= 0; i--)
		cam_soc_util_clk_disable(soc_info, cesta_client_idx, false, i);
}

/**
 * cam_soc_util_get_dt_clk_info()
 *
 * @brief:              Parse the DT and populate the Clock properties
 *
 * @soc_info:           device soc struct to be populated
 * @src_clk_str         name of src clock that has rate control
 *
 * @return:             success or failure
 */
static int cam_soc_util_get_dt_clk_info(struct cam_hw_soc_info *soc_info)
{
	struct device_node *of_node = NULL;
	int count;
	int num_clk_rates, num_clk_levels;
	int i, j, rc;
	int32_t num_clk_level_strings;
	const char *src_clk_str = NULL;
	const char *scl_clk_str = NULL;
	const char *clk_control_debugfs = NULL;
	const char *clk_cntl_lvl_string = NULL;
	enum cam_vote_level level;
	int shared_clk_cnt;
	struct of_phandle_args clk_args = {0};

	if (!soc_info || !soc_info->dev)
		return -EINVAL;

	of_node = soc_info->dev->of_node;

	if (!of_property_read_bool(of_node, "use-shared-clk")) {
		CAM_DBG(CAM_UTIL, "No shared clk parameter defined");
		soc_info->use_shared_clk = false;
	} else {
		soc_info->use_shared_clk = true;
	}

	count = of_property_count_strings(of_node, "clock-names");

	CAM_DBG(CAM_UTIL, "E: dev_name = %s count = %d",
		soc_info->dev_name, count);
	if (count > CAM_SOC_MAX_CLK) {
		CAM_ERR(CAM_UTIL, "invalid count of clocks, count=%d", count);
		rc = -EINVAL;
		return rc;
	}
	if (count <= 0) {
		CAM_DBG(CAM_UTIL, "No clock-names found");
		count = 0;
		soc_info->num_clk = count;
		return 0;
	}
	soc_info->num_clk = count;

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node, "clock-names",
				i, &(soc_info->clk_name[i]));
		CAM_DBG(CAM_UTIL, "clock-names[%d] = %s",
			i, soc_info->clk_name[i]);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"i= %d count= %d reading clock-names failed",
				i, count);
			return rc;
		}
	}

	num_clk_rates = of_property_count_u32_elems(of_node, "clock-rates");
	if (num_clk_rates <= 0) {
		CAM_ERR(CAM_UTIL, "reading clock-rates count failed");
		return -EINVAL;
	}

	if ((num_clk_rates % soc_info->num_clk) != 0) {
		CAM_ERR(CAM_UTIL,
			"mismatch clk/rates, No of clocks=%d, No of rates=%d",
			soc_info->num_clk, num_clk_rates);
		return -EINVAL;
	}

	num_clk_levels = (num_clk_rates / soc_info->num_clk);

	num_clk_level_strings = of_property_count_strings(of_node,
		"clock-cntl-level");
	if (num_clk_level_strings != num_clk_levels) {
		CAM_ERR(CAM_UTIL,
			"Mismatch No of levels=%d, No of level string=%d",
			num_clk_levels, num_clk_level_strings);
		return -EINVAL;
	}

	soc_info->lowest_clk_level = CAM_TURBO_VOTE;

	for (i = 0; i < num_clk_levels; i++) {
		rc = of_property_read_string_index(of_node,
			"clock-cntl-level", i, &clk_cntl_lvl_string);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"Error reading clock-cntl-level, rc=%d", rc);
			return rc;
		}

		rc = cam_soc_util_get_level_from_string(clk_cntl_lvl_string,
			&level);
		if (rc)
			return rc;

		CAM_DBG(CAM_UTIL,
			"[%d] : %s %d", i, clk_cntl_lvl_string, level);
		soc_info->clk_level_valid[level] = true;
		for (j = 0; j < soc_info->num_clk; j++) {
			rc = of_property_read_u32_index(of_node, "clock-rates",
				((i * soc_info->num_clk) + j),
				&soc_info->clk_rate[level][j]);
			if (rc) {
				CAM_ERR(CAM_UTIL,
					"Error reading clock-rates, rc=%d",
					rc);
				return rc;
			}

			soc_info->clk_rate[level][j] =
				(soc_info->clk_rate[level][j] == 0) ?
				(int32_t)NO_SET_RATE :
				soc_info->clk_rate[level][j];

			CAM_DBG(CAM_UTIL, "soc_info->clk_rate[%d][%d] = %d",
				level, j,
				soc_info->clk_rate[level][j]);
		}

		if ((level > CAM_MINSVS_VOTE) &&
			(level < soc_info->lowest_clk_level))
			soc_info->lowest_clk_level = level;
	}

	soc_info->src_clk_idx = -1;
	rc = of_property_read_string_index(of_node, "src-clock-name", 0,
		&src_clk_str);
	if (rc || !src_clk_str) {
		CAM_DBG(CAM_UTIL, "No src_clk_str found");
		rc = 0;
		goto end;
	}

	for (i = 0; i < soc_info->num_clk; i++) {
		if (strcmp(soc_info->clk_name[i], src_clk_str) == 0) {
			soc_info->src_clk_idx = i;
			CAM_DBG(CAM_UTIL, "src clock = %s, index = %d",
				src_clk_str, i);
		}

		rc = of_parse_phandle_with_args(of_node, "clocks",
			"#clock-cells", i, &clk_args);
		if (rc) {
			CAM_ERR(CAM_CPAS,
				"failed to clock info rc=%d", rc);
			rc = -EINVAL;
			goto end;
		}

		soc_info->clk_id[i] = clk_args.args[0];
		of_node_put(clk_args.np);

		CAM_DBG(CAM_UTIL, "Dev %s clk %s id %d",
			soc_info->dev_name, soc_info->clk_name[i],
			soc_info->clk_id[i]);
	}

	CAM_DBG(CAM_UTIL, "Dev %s src_clk_idx %d, lowest_clk_level %d",
		soc_info->dev_name, soc_info->src_clk_idx,
		soc_info->lowest_clk_level);

	soc_info->shared_clk_mask = 0;
	shared_clk_cnt = of_property_count_u32_elems(of_node, "shared-clks");
	if (shared_clk_cnt <= 0) {
		CAM_DBG(CAM_UTIL, "Dev %s, no shared clks", soc_info->dev_name);
	} else if (shared_clk_cnt != count) {
		CAM_ERR(CAM_UTIL, "Dev %s, incorrect shared clock count %d %d",
			soc_info->dev_name, shared_clk_cnt, count);
		rc = -EINVAL;
		goto end;
	} else {
		uint32_t shared_clk_val;

		for (i = 0; i < shared_clk_cnt; i++) {
			rc = of_property_read_u32_index(of_node,
				"shared-clks", i, &shared_clk_val);
			if (rc || (shared_clk_val > 1)) {
				CAM_ERR(CAM_UTIL,
					"Incorrect shared clk info at %d, val=%d, count=%d",
					i, shared_clk_val, shared_clk_cnt);
				rc = -EINVAL;
				goto end;
			}

			if (shared_clk_val)
				CAM_SET_BIT(soc_info->shared_clk_mask, i);
		}

		CAM_DBG(CAM_UTIL, "Dev %s shared clk mask 0x%x",
			soc_info->dev_name, soc_info->shared_clk_mask);
	}

	/* scalable clk info parsing */
	soc_info->scl_clk_count = 0;
	soc_info->scl_clk_count = of_property_count_strings(of_node,
		"scl-clk-names");
	if ((soc_info->scl_clk_count <= 0) ||
		(soc_info->scl_clk_count > CAM_SOC_MAX_CLK)) {
		if (soc_info->scl_clk_count == -EINVAL) {
			CAM_DBG(CAM_UTIL, "scl_clk_name prop not avialable");
		} else if ((soc_info->scl_clk_count == -ENODATA) ||
			(soc_info->scl_clk_count > CAM_SOC_MAX_CLK)) {
			CAM_ERR(CAM_UTIL, "Invalid scl_clk_count: %d",
				soc_info->scl_clk_count);
			return -EINVAL;
		}
		CAM_DBG(CAM_UTIL, "Invalid scl_clk count: %d",
			soc_info->scl_clk_count);
		soc_info->scl_clk_count = -1;
	} else {
		CAM_DBG(CAM_UTIL, "No of scalable clocks: %d",
			soc_info->scl_clk_count);
		for (i = 0; i < soc_info->scl_clk_count; i++) {
			rc = of_property_read_string_index(of_node,
				"scl-clk-names", i,
				(const char **)&scl_clk_str);
			if (rc || !scl_clk_str) {
				CAM_WARN(CAM_UTIL, "scl_clk_str is NULL");
				soc_info->scl_clk_idx[i] = -1;
				continue;
			}
			for (j = 0; j < soc_info->num_clk; j++) {
				if (strnstr(scl_clk_str, soc_info->clk_name[j],
					strlen(scl_clk_str))) {
					soc_info->scl_clk_idx[i] = j;
					CAM_DBG(CAM_UTIL,
						"scl clock = %s, index = %d",
						scl_clk_str, j);
					break;
				}
			}
		}
	}

	rc = of_property_read_string_index(of_node,
		"clock-control-debugfs", 0, &clk_control_debugfs);
	if (rc || !clk_control_debugfs) {
		CAM_DBG(CAM_UTIL, "No clock_control_debugfs property found");
		rc = 0;
		goto end;
	}

	if (strcmp("true", clk_control_debugfs) == 0)
		soc_info->clk_control_enable = true;

	CAM_DBG(CAM_UTIL, "X: dev_name = %s count = %d",
		soc_info->dev_name, count);
end:
	return rc;
}

int cam_soc_util_set_clk_rate_level(struct cam_hw_soc_info *soc_info,
	int cesta_client_idx, enum cam_vote_level clk_level_high,
	enum cam_vote_level clk_level_low, bool do_not_set_src_clk)
{
	int i, rc = 0;
	enum cam_vote_level apply_level_high;
	enum cam_vote_level apply_level_low = soc_info->lowest_clk_level;
	unsigned long applied_clk_rate;

	if ((soc_info->num_clk == 0) ||
		(soc_info->num_clk >= CAM_SOC_MAX_CLK)) {
		CAM_ERR(CAM_UTIL, "Invalid number of clock %d", soc_info->num_clk);
		return -EINVAL;
	}

	rc = cam_soc_util_get_clk_level_to_apply(soc_info, clk_level_high,
		&apply_level_high);
	if (rc) {
		CAM_ERR(CAM_UTIL, "[%s] : failed to get level clk_level_high=%d, rc=%d",
			soc_info->dev_name, clk_level_high, rc);
		return rc;
	}

	if (soc_info->is_clk_drv_en && CAM_IS_VALID_CESTA_IDX(cesta_client_idx)) {
		rc = cam_soc_util_get_clk_level_to_apply(soc_info, clk_level_low,
			&apply_level_low);
		if (rc) {
			CAM_ERR(CAM_UTIL, "[%s] : failed to get level clk_level_low=%d, rc=%d",
				soc_info->dev_name, clk_level_low, rc);
			return rc;
		}
	}

	if (soc_info->cam_cx_ipeak_enable)
		cam_cx_ipeak_update_vote_cx_ipeak(soc_info, apply_level_high);

	rc = dev_pm_opp_set_rate(soc_info->dev,
			  soc_info->clk_rate[apply_level_high][soc_info->src_clk_idx]);
	if (rc) {
		CAM_ERR(CAM_UTIL,
			"Unable to set operating point for dev %s clk_name %s rc %d",
			soc_info->dev_name, soc_info->clk_name[soc_info->src_clk_idx],
			rc);
		return rc;
	}

	for (i = 0; i < soc_info->num_clk; i++) {
		if (do_not_set_src_clk && (i == soc_info->src_clk_idx)) {
			CAM_DBG(CAM_UTIL, "Skipping set rate for src clk %s",
				soc_info->clk_name[i]);
			continue;
		}

		if (soc_info->is_clk_drv_en && CAM_IS_VALID_CESTA_IDX(cesta_client_idx) &&
			(i == soc_info->src_clk_idx)) {
			rc = cam_soc_util_set_cesta_clk_rate(soc_info, cesta_client_idx,
				soc_info->clk_rate[apply_level_high][i],
				soc_info->clk_rate[apply_level_low][i],
				&soc_info->applied_src_clk_rates.hw_client[cesta_client_idx].high,
				&soc_info->applied_src_clk_rates.hw_client[cesta_client_idx].low);
			if (rc) {
				CAM_ERR(CAM_UTIL,
					"Failed to set the req clk level[high low]: [%s %s] cesta_client_idx: %d",
					cam_soc_util_get_string_from_level(apply_level_high),
					cam_soc_util_get_string_from_level(apply_level_low),
					cesta_client_idx);
				break;
			}

			continue;
		}

		CAM_DBG(CAM_UTIL, "Set rate for clk %s rate %d", soc_info->clk_name[i],
			soc_info->clk_rate[apply_level_high][i]);

		rc = cam_soc_util_set_clk_rate(soc_info, soc_info->clk[i],
			soc_info->clk_name[i],
			soc_info->clk_rate[apply_level_high][i],
			CAM_IS_BIT_SET(soc_info->shared_clk_mask, i),
			(i == soc_info->src_clk_idx) ? true : false,
			soc_info->clk_id[i],
			&applied_clk_rate);
		if (rc < 0) {
			CAM_DBG(CAM_UTIL,
				"dev name = %s clk_name = %s idx = %d apply_level = %s",
				soc_info->dev_name, soc_info->clk_name[i],
				i, cam_soc_util_get_string_from_level(apply_level_high));
			if (soc_info->cam_cx_ipeak_enable)
				cam_cx_ipeak_update_vote_cx_ipeak(soc_info, 0);
			break;
		}

		if (i == soc_info->src_clk_idx)
			soc_info->applied_src_clk_rates.sw_client = applied_clk_rate;
	}

	return rc;
};

int cam_soc_util_dump_clk(struct cam_hw_soc_info *soc_info)
{
	int i, rc = 0;

	if (!soc_info)
		return -EINVAL;

	for (i = 0; i < soc_info->num_clk; i++) {
		CAM_INFO(CAM_UTIL, "Dumping clock = %s", soc_info->clk_name[i]);
		qcom_clk_dump(soc_info->clk[i], NULL, false);
	}

	return rc;
}

static int cam_soc_util_get_dt_gpio_req_tbl(struct device_node *of_node,
	struct cam_soc_gpio_data *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0;
	uint32_t *val_array = NULL;

	if (!of_get_property(of_node, "gpio-req-tbl-num", &count))
		return 0;

	count /= sizeof(uint32_t);
	if (!count) {
		CAM_ERR(CAM_UTIL, "gpio-req-tbl-num 0");
		return 0;
	}

	val_array = kcalloc(count, sizeof(uint32_t), GFP_KERNEL);
	if (!val_array)
		return -ENOMEM;

	gconf->cam_gpio_req_tbl = kcalloc(count, sizeof(struct gpio),
		GFP_KERNEL);
	if (!gconf->cam_gpio_req_tbl) {
		rc = -ENOMEM;
		goto free_val_array;
	}
	gconf->cam_gpio_req_tbl_size = count;

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-num",
		val_array, count);
	if (rc) {
		CAM_ERR(CAM_UTIL, "failed in reading gpio-req-tbl-num, rc = %d",
			rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			CAM_ERR(CAM_UTIL, "gpio req tbl index %d invalid",
				val_array[i]);
			goto free_gpio_req_tbl;
		}
		gconf->cam_gpio_req_tbl[i].gpio = gpio_array[val_array[i]];
		CAM_DBG(CAM_UTIL, "cam_gpio_req_tbl[%d].gpio = %d", i,
			gconf->cam_gpio_req_tbl[i].gpio);
	}

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-flags",
		val_array, count);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Failed in gpio-req-tbl-flags, rc %d", rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++) {
		gconf->cam_gpio_req_tbl[i].flags = val_array[i];
		CAM_DBG(CAM_UTIL, "cam_gpio_req_tbl[%d].flags = %ld", i,
			gconf->cam_gpio_req_tbl[i].flags);
	}

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node,
			"gpio-req-tbl-label", i,
			&gconf->cam_gpio_req_tbl[i].label);
		if (rc) {
			CAM_ERR(CAM_UTIL, "Failed rc %d", rc);
			goto free_gpio_req_tbl;
		}
		CAM_DBG(CAM_UTIL, "cam_gpio_req_tbl[%d].label = %s", i,
			gconf->cam_gpio_req_tbl[i].label);
	}

	kfree(val_array);

	return rc;

free_gpio_req_tbl:
	kfree(gconf->cam_gpio_req_tbl);
free_val_array:
	kfree(val_array);
	gconf->cam_gpio_req_tbl_size = 0;

	return rc;
}

static int cam_soc_util_get_gpio_info(struct cam_hw_soc_info *soc_info)
{
	int32_t rc = 0, i = 0;
	uint16_t *gpio_array = NULL;
	int16_t gpio_array_size = 0;
	struct cam_soc_gpio_data *gconf = NULL;
	struct device_node *of_node = NULL;

	if (!soc_info || !soc_info->dev)
		return -EINVAL;

	if (soc_info->is_child_node) {
		of_node = soc_info->parent_node;
	} else {
		of_node = soc_info->dev->of_node;
	}

	if (!of_node) {
		CAM_ERR(CAM_UTIL, "of_node is NULL");
		return -EINVAL;
	}

	/* Validate input parameters */
	if (!of_node) {
		CAM_ERR(CAM_UTIL, "Invalid param of_node");
		return -EINVAL;
	}

	gpio_array_size = of_count_phandle_with_args(of_node, "gpios", "#gpio-cells");

	if (gpio_array_size <= 0)
		return 0;

	CAM_DBG(CAM_UTIL, "gpio count %d", gpio_array_size);

	gpio_array = kcalloc(gpio_array_size, sizeof(uint16_t), GFP_KERNEL);
	if (!gpio_array) {
		rc = -ENOMEM;
		goto err;
	}

	for (i = 0; i < gpio_array_size; i++) {
		gpio_array[i] = of_get_named_gpio(of_node, "gpios", i);
		CAM_DBG(CAM_UTIL, "gpio_array[%d] = %d", i, gpio_array[i]);
	}

	gconf = kzalloc(sizeof(*gconf), GFP_KERNEL);
	if (!gconf) {
		rc = -ENOMEM;
		goto free_gpio_array;
	}

	rc = cam_soc_util_get_dt_gpio_req_tbl(of_node, gconf, gpio_array,
		gpio_array_size);
	if (rc) {
		CAM_ERR(CAM_UTIL, "failed in msm_camera_get_dt_gpio_req_tbl");
		goto free_gpio_conf;
	}

	gconf->cam_gpio_common_tbl = kcalloc(gpio_array_size,
				sizeof(struct gpio), GFP_KERNEL);
	if (!gconf->cam_gpio_common_tbl) {
		rc = -ENOMEM;
		goto free_gpio_conf;
	}

	for (i = 0; i < gpio_array_size; i++)
		gconf->cam_gpio_common_tbl[i].gpio = gpio_array[i];

	gconf->cam_gpio_common_tbl_size = gpio_array_size;
	soc_info->gpio_data = gconf;
	kfree(gpio_array);

	return rc;

free_gpio_conf:
	kfree(gconf);
free_gpio_array:
	kfree(gpio_array);
err:
	soc_info->gpio_data = NULL;

	return rc;
}

static int cam_soc_util_request_gpio_table(
	struct cam_hw_soc_info *soc_info, bool gpio_en)
{
	int rc = 0, i = 0;
	uint8_t size = 0;
	struct cam_soc_gpio_data *gpio_conf =
			soc_info->gpio_data;
	struct gpio *gpio_tbl = NULL;


	if (!gpio_conf) {
		CAM_DBG(CAM_UTIL, "No GPIO entry");
		return 0;
	}
	if (gpio_conf->cam_gpio_common_tbl_size <= 0) {
		CAM_ERR(CAM_UTIL, "GPIO table size is invalid");
		return -EINVAL;
	}
	size = gpio_conf->cam_gpio_req_tbl_size;
	gpio_tbl = gpio_conf->cam_gpio_req_tbl;

	if (!gpio_tbl || !size) {
		CAM_ERR(CAM_UTIL, "Invalid gpio_tbl %pK / size %d",
			gpio_tbl, size);
		return -EINVAL;
	}
	for (i = 0; i < size; i++) {
		CAM_DBG(CAM_UTIL, "i=%d, gpio=%d dir=%ld", i,
			gpio_tbl[i].gpio, gpio_tbl[i].flags);
	}
	if (gpio_en) {
		for (i = 0; i < size; i++) {
			rc = gpio_request_one(gpio_tbl[i].gpio,
				gpio_tbl[i].flags, gpio_tbl[i].label);
			if (rc) {
				/*
				 * After GPIO request fails, contine to
				 * apply new gpios, outout a error message
				 * for driver bringup debug
				 */
				CAM_ERR(CAM_UTIL, "gpio %d:%s request fails",
					gpio_tbl[i].gpio, gpio_tbl[i].label);
			}
		}
	} else {
		gpio_free_array(gpio_tbl, size);
	}

	return rc;
}

static int cam_soc_util_get_dt_regulator_info
	(struct cam_hw_soc_info *soc_info)
{
	int rc = 0, count = 0, i = 0;
	struct device_node *of_node = NULL;

	if (!soc_info || !soc_info->dev) {
		CAM_ERR(CAM_UTIL, "Invalid parameters");
		return -EINVAL;
	}

	if (soc_info->is_child_node) {
		of_node = soc_info->parent_node;
	} else {
		of_node = soc_info->dev->of_node;
	}

	if (!of_node) {
		CAM_ERR(CAM_UTIL, "of_node is NULL");
		return -EINVAL;
	}

	soc_info->num_rgltr = 0;
	count = of_property_count_strings(of_node, "regulator-names");
	if (count != -EINVAL) {
		if (count <= 0) {
			CAM_ERR(CAM_UTIL, "no regulators found");
			return -EINVAL;
		}

		soc_info->num_rgltr = count;

	} else {
		CAM_DBG(CAM_UTIL, "No regulators node found");
		return 0;
	}

	if (soc_info->num_rgltr > CAM_SOC_MAX_REGULATOR) {
		CAM_ERR(CAM_UTIL, "Invalid regulator count:%d",
			soc_info->num_rgltr);
		return -EINVAL;
	}

	for (i = 0; i < soc_info->num_rgltr; i++) {
		rc = of_property_read_string_index(of_node,
			"regulator-names", i, &soc_info->rgltr_name[i]);
		CAM_DBG(CAM_UTIL, "rgltr_name[%d] = %s",
			i, soc_info->rgltr_name[i]);
		if (rc) {
			CAM_ERR(CAM_UTIL, "no regulator resource at cnt=%d", i);
			return -ENODEV;
		}
	}

	if (!of_property_read_bool(of_node, "rgltr-cntrl-support")) {
		CAM_DBG(CAM_UTIL, "No regulator control parameter defined");
		soc_info->rgltr_ctrl_support = false;
		return 0;
	}

	soc_info->rgltr_ctrl_support = true;

	rc = of_property_read_u32_array(of_node, "rgltr-min-voltage",
		soc_info->rgltr_min_volt, soc_info->num_rgltr);
	if (rc) {
		CAM_ERR(CAM_UTIL, "No minimum volatage value found, rc=%d", rc);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(of_node, "rgltr-max-voltage",
		soc_info->rgltr_max_volt, soc_info->num_rgltr);
	if (rc) {
		CAM_ERR(CAM_UTIL, "No maximum volatage value found, rc=%d", rc);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(of_node, "rgltr-load-current",
		soc_info->rgltr_op_mode, soc_info->num_rgltr);
	if (rc) {
		CAM_ERR(CAM_UTIL, "No Load curent found rc=%d", rc);
		return -EINVAL;
	}

	return rc;
}

#ifdef CONFIG_CAM_PRESIL
static uint32_t next_dummy_irq_line_num = 0x000f;
struct resource dummy_irq_line[512];
#endif

int cam_soc_util_get_dt_properties(struct cam_hw_soc_info *soc_info)
{
	struct device_node *of_node = NULL;
	int count = 0, i = 0, rc = 0;

	if (!soc_info || !soc_info->dev)
		return -EINVAL;

	of_node = soc_info->dev->of_node;

	rc = of_property_read_u32(of_node, "cell-index", &soc_info->index);
	if (rc) {
		CAM_ERR(CAM_UTIL, "device %s failed to read cell-index",
			soc_info->dev_name);
		return rc;
	}

	count = of_property_count_strings(of_node, "reg-names");
	if (count <= 0) {
		CAM_DBG(CAM_UTIL, "no reg-names found for: %s",
			soc_info->dev_name);
		count = 0;
	}
	soc_info->num_mem_block = count;

	for (i = 0; i < soc_info->num_mem_block; i++) {
		rc = of_property_read_string_index(of_node, "reg-names", i,
			&soc_info->mem_block_name[i]);
		if (rc) {
			CAM_ERR(CAM_UTIL, "failed to read reg-names at %d", i);
			return rc;
		}
		soc_info->mem_block[i] =
			platform_get_resource_byname(soc_info->pdev,
			IORESOURCE_MEM, soc_info->mem_block_name[i]);

		if (!soc_info->mem_block[i]) {
			CAM_ERR(CAM_UTIL, "no mem resource by name %s",
				soc_info->mem_block_name[i]);
			rc = -ENODEV;
			return rc;
		}
	}

	rc = of_property_read_string(of_node, "label", &soc_info->label_name);
	if (rc)
		CAM_DBG(CAM_UTIL, "Label is not available in the node: %d", rc);

	if (soc_info->num_mem_block > 0) {
		rc = of_property_read_u32_array(of_node, "reg-cam-base",
			soc_info->mem_block_cam_base, soc_info->num_mem_block);
		if (rc) {
			CAM_ERR(CAM_UTIL, "Error reading register offsets");
			return rc;
		}
	}

	count = of_property_count_strings(of_node, "interrupt-names");
	if (count <= 0) {
		CAM_DBG(CAM_UTIL, "No interrupt line present for: %s", soc_info->dev_name);
		soc_info->irq_count = 0;
	} else {
		if (count > CAM_SOC_MAX_IRQ_LINES_PER_DEV) {
			CAM_ERR(CAM_UTIL,
				"Number of interrupt: %d exceeds maximum allowable interrupts: %d",
				count, CAM_SOC_MAX_IRQ_LINES_PER_DEV);
			return -EINVAL;
		}

		soc_info->irq_count = count;
		for (i = 0; i < soc_info->irq_count; i++) {
			rc = of_property_read_string_index(of_node, "interrupt-names",
				i, &soc_info->irq_name[i]);
			if (rc) {
				CAM_ERR(CAM_UTIL, "failed to read interrupt name at %d", i);
				return rc;
			}
		}

		rc = cam_compat_util_get_irq(soc_info);
		if (rc < 0) {
			CAM_ERR(CAM_UTIL, "get irq resource failed: %d for: %s",
				rc, soc_info->dev_name);
#ifndef CONFIG_CAM_PRESIL
			return rc;
#else
			/* Pre-sil for new devices not present on old */
			for (i = 0; i < soc_info->irq_count; i++) {
				soc_info->irq_line[i] =
					&dummy_irq_line[next_dummy_irq_line_num++];
				CAM_DBG(CAM_PRESIL,
					"interrupt line for dev %s irq name %s number %d",
					soc_info->dev_name, soc_info->irq_name[i],
					soc_info->irq_line[i]->start);
			}
#endif
		}
	}

	rc = of_property_read_string_index(of_node, "compatible", 0,
		(const char **)&soc_info->compatible);
	if (rc)
		CAM_DBG(CAM_UTIL, "No compatible string present for: %s",
			soc_info->dev_name);

	soc_info->is_nrt_dev = false;
	if (of_property_read_bool(of_node, "nrt-device"))
		soc_info->is_nrt_dev = true;
	CAM_DBG(CAM_UTIL, "Dev %s, nrt_dev %d",
		soc_info->dev_name, soc_info->is_nrt_dev);

	rc = cam_soc_util_get_dt_regulator_info(soc_info);
	if (rc)
		return rc;

	rc = cam_soc_util_get_dt_clk_info(soc_info);
	if (rc)
		return rc;

	rc = cam_soc_util_get_gpio_info(soc_info);
	if (rc)
		return rc;

	if (of_find_property(of_node, "qcom,cam-cx-ipeak", NULL))
		rc = cam_cx_ipeak_register_cx_ipeak(soc_info);

	return rc;
}

/**
 * cam_soc_util_get_regulator()
 *
 * @brief:              Get regulator resource named vdd
 *
 * @dev:                Device associated with regulator
 * @reg:                Return pointer to be filled with regulator on success
 * @rgltr_name:         Name of regulator to get
 *
 * @return:             0 for Success, negative value for failure
 */
static int cam_soc_util_get_regulator(struct device *dev,
	struct regulator **reg, const char *rgltr_name)
{
	int rc = 0;
	*reg = cam_wrapper_regulator_get(dev, rgltr_name);
	if (IS_ERR_OR_NULL(*reg)) {
		rc = PTR_ERR(*reg);
		rc = rc ? rc : -EINVAL;
		CAM_ERR(CAM_UTIL, "Regulator %s get failed %d", rgltr_name, rc);
		*reg = NULL;
	}
	return rc;
}

int cam_soc_util_regulator_disable(struct regulator *rgltr,
	const char *rgltr_name, uint32_t rgltr_min_volt,
	uint32_t rgltr_max_volt, uint32_t rgltr_op_mode,
	uint32_t rgltr_delay_ms)
{
	int32_t rc = 0;

	if (!rgltr) {
		CAM_ERR(CAM_UTIL, "Invalid NULL parameter");
		return -EINVAL;
	}

	rc = cam_wrapper_regulator_disable(rgltr);
	if (rc) {
		CAM_ERR(CAM_UTIL, "%s regulator disable failed", rgltr_name);
		return rc;
	}

	if (rgltr_delay_ms > 20)
		msleep(rgltr_delay_ms);
	else if (rgltr_delay_ms)
		usleep_range(rgltr_delay_ms * 1000,
			(rgltr_delay_ms * 1000) + 1000);

	if (cam_wrapper_regulator_count_voltages(rgltr) > 0) {
		cam_wrapper_regulator_set_load(rgltr, 0);
		cam_wrapper_regulator_set_voltage(rgltr, 0, rgltr_max_volt);
	}

	return rc;
}


int cam_soc_util_regulator_enable(struct regulator *rgltr,
	const char *rgltr_name,
	uint32_t rgltr_min_volt, uint32_t rgltr_max_volt,
	uint32_t rgltr_op_mode, uint32_t rgltr_delay)
{
	int32_t rc = 0;

	if (!rgltr) {
		CAM_ERR(CAM_UTIL, "Invalid NULL parameter");
		return -EINVAL;
	}

	if (cam_wrapper_regulator_count_voltages(rgltr) > 0) {
		CAM_DBG(CAM_UTIL, "[%s] voltage min=%d, max=%d",
			rgltr_name, rgltr_min_volt, rgltr_max_volt);

		rc = cam_wrapper_regulator_set_voltage(
			rgltr, rgltr_min_volt, rgltr_max_volt);
		if (rc) {
			CAM_ERR(CAM_UTIL, "%s set voltage failed", rgltr_name);
			return rc;
		}

		rc = cam_wrapper_regulator_set_load(rgltr, rgltr_op_mode);
		if (rc) {
			CAM_ERR(CAM_UTIL, "%s set optimum mode failed",
				rgltr_name);
			return rc;
		}
	}

	rc = cam_wrapper_regulator_enable(rgltr);
	if (rc) {
		CAM_ERR(CAM_UTIL, "%s regulator_enable failed", rgltr_name);
		return rc;
	}

	if (rgltr_delay > 20)
		msleep(rgltr_delay);
	else if (rgltr_delay)
		usleep_range(rgltr_delay * 1000,
			(rgltr_delay * 1000) + 1000);

	return rc;
}

int cam_soc_util_select_pinctrl_state(struct cam_hw_soc_info *soc_info,
	int pctrl_idx, bool active)
{
	int rc = 0;

	struct cam_soc_pinctrl_info *pctrl_info = &soc_info->pinctrl_info;

	if (pctrl_idx >= CAM_SOC_MAX_PINCTRL_MAP) {
		CAM_ERR(CAM_UTIL, "Invalid Map idx: %d max supported: %d",
			pctrl_idx, CAM_SOC_MAX_PINCTRL_MAP);
		return -EINVAL;
	}

	if (pctrl_info->pctrl_state[pctrl_idx].gpio_state_active &&
		active &&
		!pctrl_info->pctrl_state[pctrl_idx].is_active) {
		rc = pinctrl_select_state(pctrl_info->pinctrl,
			pctrl_info->pctrl_state[pctrl_idx].gpio_state_active);
		if (rc)
			CAM_ERR(CAM_UTIL,
				"Pinctrl active state transition failed: rc: %d",
				rc);
		else {
			pctrl_info->pctrl_state[pctrl_idx].is_active = true;
			CAM_DBG(CAM_UTIL, "Pctrl_idx: %d is in active state",
				pctrl_idx);
		}
	}

	if (pctrl_info->pctrl_state[pctrl_idx].gpio_state_suspend &&
		!active &&
		pctrl_info->pctrl_state[pctrl_idx].is_active) {
		rc = pinctrl_select_state(pctrl_info->pinctrl,
			pctrl_info->pctrl_state[pctrl_idx].gpio_state_suspend);
		if (rc)
			CAM_ERR(CAM_UTIL,
				"Pinctrl suspend state transition failed: rc: %d",
				rc);
		else {
			pctrl_info->pctrl_state[pctrl_idx].is_active = false;
			CAM_DBG(CAM_UTIL, "Pctrl_idx: %d is in suspend state",
				pctrl_idx);
		}
	}

	return rc;
}

static int cam_soc_util_request_pinctrl(
	struct cam_hw_soc_info *soc_info)
{
	struct cam_soc_pinctrl_info *device_pctrl = &soc_info->pinctrl_info;
	struct device *dev = soc_info->dev;
	struct device_node *of_node = dev->of_node;
	uint32_t i = 0;
	int rc = 0;
	const char *name;
	uint32_t idx;
	char pctrl_active[50];
	char pctrl_suspend[50];
	int32_t num_of_map_idx = 0;
	int32_t num_of_string = 0;

	device_pctrl->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(device_pctrl->pinctrl)) {
		CAM_DBG(CAM_UTIL, "Pinctrl not available");
		device_pctrl->pinctrl = NULL;
		return 0;
	}

	num_of_map_idx = of_property_count_u32_elems(
		of_node, "pctrl-idx-mapping");
	if (num_of_map_idx <= 0) {
		CAM_ERR(CAM_UTIL,
			"Reading pctrl-idx-mapping failed");
		return -EINVAL;
	}

	num_of_string = of_property_count_strings(
		of_node, "pctrl-map-names");
	if (num_of_string <= 0) {
		CAM_ERR(CAM_UTIL, "no pinctrl-mapping found for: %s",
			soc_info->dev_name);
		device_pctrl->pinctrl = NULL;
		return -EINVAL;
	}

	if (num_of_map_idx != num_of_string) {
		CAM_ERR(CAM_UTIL,
			"Incorrect inputs mapping-idx count: %d mapping-names: %d",
			num_of_map_idx, num_of_string);
		device_pctrl->pinctrl = NULL;
		return -EINVAL;
	}

	if (num_of_map_idx > CAM_SOC_MAX_PINCTRL_MAP) {
		CAM_ERR(CAM_UTIL, "Invalid mapping %u max supported: %d",
			num_of_map_idx, CAM_SOC_MAX_PINCTRL_MAP);
		return -EINVAL;
	}

	for (i = 0; i < num_of_map_idx; i++) {
		of_property_read_u32_index(of_node,
			"pctrl-idx-mapping", i, &idx);

		if (idx >= CAM_SOC_MAX_PINCTRL_MAP) {
			CAM_ERR(CAM_UTIL, "Invalid Index: %d max supported: %d",
				idx, CAM_SOC_MAX_PINCTRL_MAP);
			return -EINVAL;
		}

		rc = of_property_read_string_index(
			of_node, "pctrl-map-names", i, &name);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"failed to read pinctrl-mapping at %d", i);
			return rc;
		}

		snprintf(pctrl_active, sizeof(pctrl_active),
			"%s%s",	name, "_active");
		CAM_DBG(CAM_UTIL, "pctrl_active at index: %d name: %s",
			i, pctrl_active);
		snprintf(pctrl_suspend, sizeof(pctrl_suspend),
			"%s%s", name, "_suspend");
		CAM_DBG(CAM_UTIL, "pctrl_suspend at index: %d name: %s",
			i, pctrl_suspend);

		device_pctrl->pctrl_state[idx].gpio_state_active =
			pinctrl_lookup_state(device_pctrl->pinctrl,
			pctrl_active);
		if (IS_ERR_OR_NULL(
			device_pctrl->pctrl_state[idx].gpio_state_active)) {
			CAM_ERR(CAM_UTIL,
				"Failed to get the active state pinctrl handle");
			device_pctrl->pctrl_state[idx].gpio_state_active =
				NULL;
			return -EINVAL;
		}
		device_pctrl->pctrl_state[idx].gpio_state_suspend =
			pinctrl_lookup_state(device_pctrl->pinctrl,
			pctrl_suspend);
		if (IS_ERR_OR_NULL(
			device_pctrl->pctrl_state[idx].gpio_state_suspend)) {
			CAM_ERR(CAM_UTIL,
				"Failed to get the active state pinctrl handle");
			device_pctrl->pctrl_state[idx].gpio_state_suspend = NULL;
			return -EINVAL;
		}
	}

	return 0;
}

static void cam_soc_util_release_pinctrl(struct cam_hw_soc_info *soc_info)
{
	if (soc_info->pinctrl_info.pinctrl)
		devm_pinctrl_put(soc_info->pinctrl_info.pinctrl);
}

static void cam_soc_util_regulator_disable_default(
	struct cam_hw_soc_info *soc_info)
{
	int j = 0;
	uint32_t num_rgltr = soc_info->num_rgltr;

	for (j = num_rgltr-1; j >= 0; j--) {
		if (soc_info->rgltr_ctrl_support == true) {
			cam_soc_util_regulator_disable(soc_info->rgltr[j],
				soc_info->rgltr_name[j],
				soc_info->rgltr_min_volt[j],
				soc_info->rgltr_max_volt[j],
				soc_info->rgltr_op_mode[j],
				soc_info->rgltr_delay[j]);
		} else {
			if (soc_info->rgltr[j])
				cam_wrapper_regulator_disable(soc_info->rgltr[j]);
		}
	}
}

static int cam_soc_util_regulator_enable_default(
	struct cam_hw_soc_info *soc_info)
{
	int j = 0, rc = 0;
	uint32_t num_rgltr = soc_info->num_rgltr;

	if (num_rgltr > CAM_SOC_MAX_REGULATOR) {
		CAM_ERR(CAM_UTIL,
			"%s has invalid regulator number %d",
			soc_info->dev_name, num_rgltr);
		return -EINVAL;
	}

	for (j = 0; j < num_rgltr; j++) {
		CAM_DBG(CAM_UTIL, "[%s] : start regulator %s enable, rgltr_ctrl_support %d",
			soc_info->dev_name, soc_info->rgltr_name[j], soc_info->rgltr_ctrl_support);
		if (soc_info->rgltr_ctrl_support == true) {
			rc = cam_soc_util_regulator_enable(soc_info->rgltr[j],
				soc_info->rgltr_name[j],
				soc_info->rgltr_min_volt[j],
				soc_info->rgltr_max_volt[j],
				soc_info->rgltr_op_mode[j],
				soc_info->rgltr_delay[j]);
		} else {
			if (soc_info->rgltr[j])
				rc = cam_wrapper_regulator_enable(soc_info->rgltr[j]);
		}

		if (rc) {
			CAM_ERR(CAM_UTIL, "%s enable failed",
				soc_info->rgltr_name[j]);
			goto disable_rgltr;
		}
	}

	return rc;
disable_rgltr:

	for (j--; j >= 0; j--) {
		if (soc_info->rgltr_ctrl_support == true) {
			cam_soc_util_regulator_disable(soc_info->rgltr[j],
				soc_info->rgltr_name[j],
				soc_info->rgltr_min_volt[j],
				soc_info->rgltr_max_volt[j],
				soc_info->rgltr_op_mode[j],
				soc_info->rgltr_delay[j]);
		} else {
			if (soc_info->rgltr[j])
				cam_wrapper_regulator_disable(soc_info->rgltr[j]);
		}
	}

	return rc;
}

static bool cam_soc_util_is_presil_address_space(unsigned long mem_block_start)
{
	if(mem_block_start >= CAM_SS_START_PRESIL && mem_block_start < CAM_SS_START)
		return true;

	return false;
}

#ifndef CONFIG_CAM_PRESIL
void __iomem * cam_soc_util_get_mem_base(
	unsigned long mem_block_start,
	unsigned long mem_block_size,
	const char *mem_block_name,
	uint32_t reserve_mem)
{
	void __iomem * mem_base;

	if (reserve_mem) {
		if (!request_mem_region(mem_block_start,
			mem_block_size,
			mem_block_name)) {
			CAM_ERR(CAM_UTIL,
				"Error Mem region request Failed:%s",
				mem_block_name);
			return NULL;
		}
	}

	mem_base = ioremap(mem_block_start, mem_block_size);

	if (!mem_base) {
		CAM_ERR(CAM_UTIL, "get mem base failed");
	}

	return mem_base;
}

int cam_soc_util_request_irq(struct device *dev,
	unsigned int irq_line_start,
	irq_handler_t handler,
	unsigned long irqflags,
	const char *irq_name,
	void *irq_data,
	unsigned long mem_block_start)
{
	int rc;

	rc = devm_request_irq(dev,
		irq_line_start,
		handler,
		IRQF_TRIGGER_RISING,
		irq_name,
		irq_data);
	if (rc) {
		CAM_ERR(CAM_UTIL, "irq request fail rc %d", rc);
		return -EBUSY;
	}

	disable_irq(irq_line_start);

	return rc;
}

#else
void __iomem * cam_soc_util_get_mem_base(
	unsigned long mem_block_start,
	unsigned long mem_block_size,
	const char *mem_block_name,
	uint32_t reserve_mem)
{
	void __iomem * mem_base;

	if(cam_soc_util_is_presil_address_space(mem_block_start))
		mem_base = (void __iomem *)mem_block_start;
	else {
		if (reserve_mem) {
			if (!request_mem_region(mem_block_start,
				mem_block_size,
				mem_block_name)) {
				CAM_ERR(CAM_UTIL,
					"Error Mem region request Failed:%s",
					mem_block_name);
				return NULL;
			}
		}

		mem_base = ioremap(mem_block_start, mem_block_size);
	}

	if (!mem_base) {
		CAM_ERR(CAM_UTIL, "get mem base failed");
	}

	return mem_base;
}

int cam_soc_util_request_irq(struct device *dev,
	unsigned int irq_line_start,
	irq_handler_t handler,
	unsigned long irqflags,
	const char *irq_name,
	void *irq_data,
	unsigned long mem_block_start)
{
	int rc;

	if(cam_soc_util_is_presil_address_space(mem_block_start)) {
		rc = devm_request_irq(dev,
			irq_line_start,
			handler,
			irqflags,
			irq_name,
			irq_data);
		if (rc) {
			CAM_ERR(CAM_UTIL, "presil irq request fail");
			return -EBUSY;
		}

		disable_irq(irq_line_start);

		rc = !(cam_presil_subscribe_device_irq(irq_line_start,
			handler, irq_data, irq_name));
		CAM_DBG(CAM_PRESIL, "Subscribe presil IRQ: rc=%d NUM=%d Name=%s handler=0x%x",
			rc, irq_line_start, irq_name, handler);
		if (rc) {
			CAM_ERR(CAM_UTIL, "presil irq request fail");
			return -EBUSY;
		}
	} else {
		rc = devm_request_irq(dev,
			irq_line_start,
			handler,
			irqflags,
			irq_name,
			irq_data);
		if (rc) {
			CAM_ERR(CAM_UTIL, "irq request fail");
			return -EBUSY;
		}
		disable_irq(irq_line_start);
		CAM_INFO(CAM_UTIL, "Subscribe for non-presil IRQ success");
	}

	CAM_INFO(CAM_UTIL, "returning IRQ for mem_block_start 0x%0x rc %d",
		mem_block_start, rc);

	return rc;
}
#endif

int cam_soc_util_power_domain_enable_default(
	struct cam_hw_soc_info *soc_info)
{
	int i = 0, ret = 0;
	int32_t num_pds = soc_info->num_genpd;

	if (num_pds < 1) {
		CAM_DBG(CAM_UTIL,
			"power-domains not defined for dev %s num_pds = %d",
			num_pds, soc_info->dev_name);
		goto end;
	}

	if (num_pds == 1) {
		dev_pm_genpd_set_performance_state(soc_info->dev, INT_MAX);
		ret = pm_runtime_get_sync(soc_info->dev);
		CAM_DBG(CAM_UTIL,
			"power-domain enabled for dev %s ret %d",
			soc_info->dev_name, ret);

		if (ret < 0) {
			CAM_ERR(CAM_UTIL,
				"power-domain enable failed for dev %s ret %d",
				soc_info->dev_name, ret);
			pm_runtime_put(soc_info->dev);
			dev_pm_genpd_set_performance_state(soc_info->dev, 0);
		}

		goto end;
	}

	for (i = 0; i < num_pds; i++) {
		if (!soc_info->genpd)
			continue;

		if (!soc_info->genpd[i])
			continue;

		dev_pm_genpd_set_performance_state(soc_info->genpd[i], INT_MAX);
		ret = pm_runtime_get_sync(soc_info->genpd[i]);
		if (ret < 0) {
			CAM_ERR(CAM_UTIL,
				"power-domain enable failed for dev %s ret %d i %d",
				soc_info->dev_name, ret, i);
			goto disable_pds;
		}
	}

	goto end;

disable_pds:
	for (i--; i >= 0; i--) {
		if (!soc_info->genpd[i])
			continue;

		dev_pm_genpd_set_performance_state(soc_info->genpd[i], 0);
		pm_runtime_put(soc_info->genpd[i]);
	}
end:
	return ret;
}

int cam_soc_util_configure_pd(struct cam_hw_soc_info *soc_info)
{
	int i, ret = 0;

	soc_info->num_genpd = of_count_phandle_with_args(soc_info->dev->of_node,
							"power-domains",
							"#power-domain-cells");
	if (soc_info->num_genpd < 1) {
		CAM_WARN(CAM_UTIL,
			"DBG: power-domains not defined for %s",
			soc_info->dev_name);
		soc_info->num_genpd = 0;
		goto end;
	}

	CAM_DBG(CAM_UTIL,
		"num_genpd defined for dev %s: %d",
		soc_info->dev_name, soc_info->num_genpd);
	/*
	 * With only one power domain defined there is no need to have
	 * references of genpd devices as the genpd attach will happen
	 * during probe with dev_pm_domain_attach().
	 */
	if (soc_info->num_genpd == 1) {
		goto end;
	}

	soc_info->genpd = devm_kmalloc_array(soc_info->dev, soc_info->num_genpd,
					     sizeof(struct device), GFP_KERNEL);
	if (!soc_info->genpd) {
		CAM_ERR(CAM_UTIL,
			"devm_kmalloc_array failed for dev %s, num_genpd %d",
			soc_info->dev_name, soc_info->num_genpd);
		ret = -ENOMEM;
		goto end;
	}

	for (i = 0; i < soc_info->num_genpd; i++) {
		soc_info->genpd[i] = dev_pm_domain_attach_by_id(soc_info->dev, i);
		if (IS_ERR(soc_info->genpd[i])) {
			CAM_ERR(CAM_UTIL,
				"Error in pm_domain_attach");
			ret = PTR_ERR(soc_info->genpd[i]);
			goto fail_pm;
		}
	}

	goto end;

fail_pm:
	for (--i ; i >= 0; i--)
		dev_pm_domain_detach(soc_info->genpd[i], true);
end:
	return ret;
}

int cam_soc_util_configure_opp(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = devm_pm_opp_set_clkname(soc_info->dev, soc_info->clk_name[soc_info->src_clk_idx]);
	if (rc) {
		CAM_ERR(CAM_UTIL,
			"OPP set_clkname failed for dev %s clk_name %s rc %d",
			soc_info->dev_name, soc_info->clk_name[soc_info->src_clk_idx], rc);
		return rc;
	}

	rc = devm_pm_opp_of_add_table(soc_info->dev);
	if (rc) {
		CAM_ERR(CAM_UTIL,
			"OPP add_table failed for dev %s rc %d",
			soc_info->dev_name, rc);
	}
	return rc;
}

int cam_soc_util_request_platform_resource(
	struct cam_hw_soc_info *soc_info,
	irq_handler_t handler, void **irq_data)
{
	int i = 0, rc = 0;

	if (!soc_info || !soc_info->dev) {
		CAM_ERR(CAM_UTIL, "Invalid parameters");
		return -EINVAL;
	}

	if (unlikely(soc_info->irq_count > CAM_SOC_MAX_IRQ_LINES_PER_DEV)) {
		CAM_ERR(CAM_UTIL, "Invalid irq count: %u Max IRQ per device: %d",
			soc_info->irq_count, CAM_SOC_MAX_IRQ_LINES_PER_DEV);
		return -EINVAL;
	}

	for (i = 0; i < soc_info->num_mem_block; i++) {

		soc_info->reg_map[i].mem_base = cam_soc_util_get_mem_base(
			soc_info->mem_block[i]->start,
			resource_size(soc_info->mem_block[i]),
			soc_info->mem_block_name[i],
			soc_info->reserve_mem);

		if (!soc_info->reg_map[i].mem_base) {
			CAM_ERR(CAM_UTIL, "i= %d base NULL", i);
			rc = -ENOMEM;
			goto unmap_base;
		}

		soc_info->reg_map[i].mem_cam_base =
			soc_info->mem_block_cam_base[i];
		soc_info->reg_map[i].size =
			resource_size(soc_info->mem_block[i]);

		soc_info->num_reg_map++;
	}

	for (i = 0; i < soc_info->num_rgltr; i++) {
		if (soc_info->rgltr_name[i] == NULL) {
			CAM_ERR(CAM_UTIL, "can't find regulator name");
			goto put_regulator;
		}

		rc = cam_soc_util_get_regulator(soc_info->dev,
			&soc_info->rgltr[i],
			soc_info->rgltr_name[i]);
		if (rc)
			goto put_regulator;
	}

	rc = cam_soc_util_configure_pd(soc_info);
	if (rc)
		goto put_regulator;

	pm_runtime_enable(soc_info->dev);

	/*
	 * Doing a pm_runtime_get_sync() and pm_runtime_put_sync() after pm_runtime_enable
	 * makes sure that the GDSC is off and only enabled later by the usecase when there
	 * is such a requirement.
	 */
	cam_soc_util_power_domain_enable_default(soc_info);
	cam_soc_util_power_domain_disable_default(soc_info);


	for (i = 0; i < soc_info->irq_count; i++) {
		rc = cam_soc_util_request_irq(soc_info->dev, soc_info->irq_num[i],
			handler, IRQF_TRIGGER_RISING, soc_info->irq_name[i],
				irq_data[i], soc_info->mem_block[0]->start);
		if (rc) {
			CAM_ERR(CAM_UTIL, "irq request fail for irq name: %s dev: %s",
				soc_info->irq_name[i], soc_info->dev_name);
			rc = -EBUSY;
			goto put_irq;
		}

		soc_info->irq_data[i] = irq_data[i];
	}

	/* Get Clock */
	for (i = 0; i < soc_info->num_clk; i++) {
		soc_info->clk[i] = cam_wrapper_clk_get(soc_info->dev,
			soc_info->clk_name[i]);
		if (IS_ERR(soc_info->clk[i])) {
			CAM_ERR(CAM_UTIL, "get failed for %s",
				soc_info->clk_name[i]);
			rc = -ENOENT;
			goto put_clk;
		} else if (!soc_info->clk[i]) {
			CAM_DBG(CAM_UTIL, "%s handle is NULL skip get",
				soc_info->clk_name[i]);
			continue;
		}

		/* Create a wrapper entry if this is a shared clock */
		if (CAM_IS_BIT_SET(soc_info->shared_clk_mask, i)) {
			uint32_t min_level = soc_info->lowest_clk_level;

			CAM_DBG(CAM_UTIL,
				"Dev %s, clk %s, id %d register wrapper entry for shared clk",
				soc_info->dev_name, soc_info->clk_name[i],
				soc_info->clk_id[i]);

			rc = cam_soc_util_clk_wrapper_register_entry(
				soc_info->clk_id[i], soc_info->clk[i],
				(i == soc_info->src_clk_idx) ? true : false,
				soc_info, soc_info->clk_rate[min_level][i],
				soc_info->clk_name[i]);
			if (rc) {
				CAM_ERR(CAM_UTIL,
					"Failed in registering shared clk Dev %s id %d",
					soc_info->dev_name,
					soc_info->clk_id[i]);
				cam_wrapper_clk_put(soc_info->clk[i]);
				soc_info->clk[i] = NULL;
				goto put_clk;
			}
		} else if (i == soc_info->src_clk_idx) {
			rc = cam_soc_util_register_mmrm_client(
				soc_info->clk_id[i], soc_info->clk[i],
				soc_info->is_nrt_dev,
				soc_info, soc_info->clk_name[i],
				&soc_info->mmrm_handle);
			if (rc) {
				CAM_ERR(CAM_UTIL,
					"Failed in register mmrm client Dev %s clk id %d",
					soc_info->dev_name,
					soc_info->clk_id[i]);
				cam_wrapper_clk_put(soc_info->clk[i]);
				soc_info->clk[i] = NULL;
				goto put_clk;
			}
		}
	}

	rc = cam_soc_util_configure_opp(soc_info);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Failed to configure OPP");
		goto put_clk;
	}

	rc = cam_soc_util_request_pinctrl(soc_info);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Failed in requesting Pinctrl, rc: %d", rc);
		goto put_clk;
	}

	rc = cam_soc_util_request_gpio_table(soc_info, true);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Failed in request gpio table, rc=%d", rc);
		goto put_clk;
	}

	if (soc_info->clk_control_enable)
		cam_soc_util_create_clk_lvl_debugfs(soc_info);

	return rc;

put_clk:

	if (soc_info->mmrm_handle) {
		cam_soc_util_unregister_mmrm_client(soc_info->mmrm_handle);
		soc_info->mmrm_handle = NULL;
	}

	for (i = i - 1; i >= 0; i--) {
		if (soc_info->clk[i]) {
			if (CAM_IS_BIT_SET(soc_info->shared_clk_mask, i))
				cam_soc_util_clk_wrapper_unregister_entry(
					soc_info->clk_id[i], soc_info);

			cam_wrapper_clk_put(soc_info->clk[i]);
			soc_info->clk[i] = NULL;
		}
	}

put_irq:
	if (i == -1)
		i = soc_info->irq_count;
	for (i = i - 1; i >= 0; i--) {
		if (soc_info->irq_num[i] > 0)
			disable_irq(soc_info->irq_num[i]);
	}

put_regulator:
	if (i == -1)
		i = soc_info->num_rgltr;
	for (i = i - 1; i >= 0; i--) {
		if (soc_info->rgltr[i]) {
			cam_wrapper_regulator_disable(soc_info->rgltr[i]);
			cam_wrapper_regulator_put(soc_info->rgltr[i]);
			soc_info->rgltr[i] = NULL;
		}
	}

unmap_base:
	if (i == -1)
		i = soc_info->num_reg_map;
	for (i = i - 1; i >= 0; i--) {
		if (soc_info->reserve_mem)
			release_mem_region(soc_info->mem_block[i]->start,
				resource_size(soc_info->mem_block[i]));
		iounmap(soc_info->reg_map[i].mem_base);
		soc_info->reg_map[i].mem_base = NULL;
		soc_info->reg_map[i].size = 0;
	}

	return rc;
}

int cam_soc_util_release_platform_resource(struct cam_hw_soc_info *soc_info)
{
	int i;
	bool b_ret = false;

	if (!soc_info || !soc_info->dev) {
		CAM_ERR(CAM_UTIL, "Invalid parameter");
		return -EINVAL;
	}

	if (soc_info->mmrm_handle) {
		cam_soc_util_unregister_mmrm_client(soc_info->mmrm_handle);
		soc_info->mmrm_handle = NULL;
	}

	for (i = soc_info->num_clk - 1; i >= 0; i--) {
		if (CAM_IS_BIT_SET(soc_info->shared_clk_mask, i))
			cam_soc_util_clk_wrapper_unregister_entry(
				soc_info->clk_id[i], soc_info);
		if (!soc_info->clk[i]) {
			CAM_DBG(CAM_UTIL, "%s handle is NULL skip put",
				soc_info->clk_name[i]);
			continue;
		}
		cam_wrapper_clk_put(soc_info->clk[i]);
		soc_info->clk[i] = NULL;
	}

	for (i = soc_info->num_rgltr - 1; i >= 0; i--) {
		if (soc_info->rgltr[i]) {
			cam_wrapper_regulator_put(soc_info->rgltr[i]);
			soc_info->rgltr[i] = NULL;
		}
	}

	pm_runtime_disable(soc_info->dev);
	for (i = 0; i < soc_info->num_genpd; i++) {
		if (!soc_info->genpd)
			continue;
		if (!soc_info->genpd[i])
			continue;
		dev_pm_domain_detach(soc_info->genpd[i], true);
	}

	for (i = soc_info->num_reg_map - 1; i >= 0; i--) {
		iounmap(soc_info->reg_map[i].mem_base);
		soc_info->reg_map[i].mem_base = NULL;
		soc_info->reg_map[i].size = 0;
	}

	for (i = soc_info->irq_count; i >= 0; i--) {
		if (soc_info->irq_num[i] > 0) {
			if (cam_presil_mode_enabled()) {
				if (cam_soc_util_is_presil_address_space(
					soc_info->mem_block[0]->start)) {
					b_ret = cam_presil_unsubscribe_device_irq(
						soc_info->irq_line[i]->start);
					CAM_DBG(CAM_PRESIL,
						"UnSubscribe IRQ: Ret=%d NUM=%d Name=%s",
						b_ret, soc_info->irq_line[i]->start,
						soc_info->irq_name[i]);
				}
			}

			disable_irq(soc_info->irq_num[i]);
		}
	}

	cam_soc_util_release_pinctrl(soc_info);

	/* release for gpio */
	cam_soc_util_request_gpio_table(soc_info, false);

	soc_info->dentry = NULL;

	return 0;
}

int cam_soc_util_enable_platform_resource(struct cam_hw_soc_info *soc_info,
	int cesta_client_idx, bool enable_clocks, enum cam_vote_level clk_level,
	bool irq_enable)
{
	int rc = 0, i;

	if (!soc_info)
		return -EINVAL;

	rc = cam_soc_util_regulator_enable_default(soc_info);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Regulators enable failed");
		return rc;
	}

	rc = cam_soc_util_power_domain_enable_default(soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_UTIL, "Power domains enable failed");
		goto disable_regulator;
	}

	if (enable_clocks) {
		rc = cam_soc_util_clk_enable_default(soc_info, cesta_client_idx, clk_level);
		if (rc)
			goto disable_power_domain;
	}

	if (irq_enable) {
		for (i = 0; i < soc_info->irq_count; i++) {
			if (soc_info->irq_num[i] < 0) {
				CAM_ERR(CAM_UTIL, "No IRQ line available for irq: %s dev: %s",
					soc_info->irq_name[i], soc_info->dev_name);
				rc = -ENODEV;
				goto disable_irq;
			}

			enable_irq(soc_info->irq_num[i]);
		}
	}

	return rc;

disable_irq:
	if (irq_enable) {
		for (i = i - 1; i >= 0; i--)
			disable_irq(soc_info->irq_num[i]);
	}

	if (enable_clocks)
		cam_soc_util_clk_disable_default(soc_info, cesta_client_idx);

disable_power_domain:
	cam_soc_util_power_domain_disable_default(soc_info);

disable_regulator:
	cam_soc_util_regulator_disable_default(soc_info);

	return rc;
}

int cam_soc_util_disable_platform_resource(struct cam_hw_soc_info *soc_info,
	int cesta_client_idx, bool disable_clocks, bool disable_irq)
{
	int rc = 0;

	if (!soc_info)
		return -EINVAL;

	if (disable_irq)
		rc |= cam_soc_util_irq_disable(soc_info);

	if (disable_clocks)
		cam_soc_util_clk_disable_default(soc_info, cesta_client_idx);

	cam_soc_util_power_domain_disable_default(soc_info);

	cam_soc_util_regulator_disable_default(soc_info);

	return rc;
}

int cam_soc_util_reg_dump(struct cam_hw_soc_info *soc_info,
	uint32_t base_index, uint32_t offset, int size)
{
	void __iomem     *base_addr = NULL;

	CAM_DBG(CAM_UTIL, "base_idx %u size=%d", base_index, size);

	if (!soc_info || base_index >= soc_info->num_reg_map ||
		size <= 0 || (offset + size) >=
		CAM_SOC_GET_REG_MAP_SIZE(soc_info, base_index))
		return -EINVAL;

	base_addr = CAM_SOC_GET_REG_MAP_START(soc_info, base_index);

	/*
	 * All error checking already done above,
	 * hence ignoring the return value below.
	 */
	cam_io_dump(base_addr, offset, size);

	return 0;
}

static inline int cam_soc_util_reg_addr_validation(
	uint32_t reg_map_size, uint32_t offset, char *reg_unit)
{
	if (!IS_ALIGNED(offset, 4)) {
		CAM_ERR(CAM_UTIL, "Offset: 0x%X of %s is not memory aligned",
			offset, reg_unit);
		return -EINVAL;
	} else if (offset > reg_map_size) {
		CAM_ERR(CAM_UTIL,
			"Reg offset: 0x%X of %s out of range, reg_map size: 0x%X",
			offset, reg_unit, reg_map_size);
		return -EINVAL;
	}

	return 0;
}

static int cam_soc_util_dump_cont_reg_range(
	struct cam_hw_soc_info *soc_info,
	struct cam_reg_range_read_desc *reg_read, uint32_t base_idx,
	struct cam_reg_dump_out_buffer *dump_out_buf, uintptr_t cmd_buf_end)
{
	int         i = 0, rc = 0;
	uint32_t    write_idx = 0, reg_map_size;

	if (!soc_info || !dump_out_buf || !reg_read || !cmd_buf_end) {
		CAM_ERR(CAM_UTIL,
			"Invalid input args soc_info: %pK, dump_out_buffer: %pK reg_read: %pK cmd_buf_end: %pK",
			soc_info, dump_out_buf, reg_read, cmd_buf_end);
		rc = -EINVAL;
		goto end;
	}

	if ((reg_read->num_values) && ((reg_read->num_values > U32_MAX / 2) ||
		(sizeof(uint32_t) > ((U32_MAX -
		sizeof(struct cam_reg_dump_out_buffer) -
		dump_out_buf->bytes_written) /
		(reg_read->num_values * 2))))) {
		CAM_ERR(CAM_UTIL,
			"Integer Overflow bytes_written: [%u] num_values: [%u]",
			dump_out_buf->bytes_written, reg_read->num_values);
		rc = -EOVERFLOW;
		goto end;
	}

	if ((cmd_buf_end - (uintptr_t)dump_out_buf) <=
		(uintptr_t)(sizeof(struct cam_reg_dump_out_buffer)
		- sizeof(uint32_t) + dump_out_buf->bytes_written +
		(reg_read->num_values * 2 * sizeof(uint32_t)))) {
		CAM_ERR(CAM_UTIL,
			"Insufficient space in out buffer num_values: [%d] cmd_buf_end: %pK dump_out_buf: %pK",
			reg_read->num_values, cmd_buf_end,
			(uintptr_t)dump_out_buf);
		rc = -EINVAL;
		goto end;
	}

	write_idx = dump_out_buf->bytes_written / sizeof(uint32_t);
	reg_map_size = (uint32_t)soc_info->reg_map[base_idx].size;
	for (i = 0; i < reg_read->num_values; i++) {
		rc = cam_soc_util_reg_addr_validation(reg_map_size,
			reg_read->offset + (i * sizeof(uint32_t)),
			"cont_reg_range");
		if (rc)
			continue;

		dump_out_buf->dump_data_flex[write_idx++] = reg_read->offset +
			(i * sizeof(uint32_t));
		dump_out_buf->dump_data_flex[write_idx++] =
			cam_soc_util_r(soc_info, base_idx,
			(reg_read->offset + (i * sizeof(uint32_t))));
		dump_out_buf->bytes_written += (2 * sizeof(uint32_t));
	}

end:
	return rc;
}

static int cam_soc_util_dump_dmi_reg_range(
	struct cam_hw_soc_info *soc_info,
	struct cam_dmi_read_desc *dmi_read, uint32_t base_idx,
	struct cam_reg_dump_out_buffer *dump_out_buf, uintptr_t cmd_buf_end)
{
	int        i = 0, rc = 0;
	uint32_t   write_idx = 0, reg_map_size;

	if (!soc_info || !dump_out_buf || !dmi_read || !cmd_buf_end) {
		CAM_ERR(CAM_UTIL,
			"Invalid input args soc_info: %pK, dump_out_buffer: %pK",
			soc_info, dump_out_buf);
		rc = -EINVAL;
		goto end;
	}

	if (dmi_read->num_pre_writes > CAM_REG_DUMP_DMI_CONFIG_MAX ||
		dmi_read->num_post_writes > CAM_REG_DUMP_DMI_CONFIG_MAX) {
		CAM_ERR(CAM_UTIL,
			"Invalid number of requested writes, pre: %d post: %d",
			dmi_read->num_pre_writes, dmi_read->num_post_writes);
		rc = -EINVAL;
		goto end;
	}

	if ((dmi_read->num_pre_writes + dmi_read->dmi_data_read.num_values)
		&& ((dmi_read->num_pre_writes > U32_MAX / 2) ||
		(dmi_read->dmi_data_read.num_values > U32_MAX / 2) ||
		((dmi_read->num_pre_writes * 2) > U32_MAX -
		(dmi_read->dmi_data_read.num_values * 2)) ||
		(sizeof(uint32_t) > ((U32_MAX -
		sizeof(struct cam_reg_dump_out_buffer) -
		dump_out_buf->bytes_written) / ((dmi_read->num_pre_writes +
		dmi_read->dmi_data_read.num_values) * 2))))) {
		CAM_ERR(CAM_UTIL,
			"Integer Overflow bytes_written: [%u] num_pre_writes: [%u] num_values: [%u]",
			dump_out_buf->bytes_written, dmi_read->num_pre_writes,
			dmi_read->dmi_data_read.num_values);
		rc = -EOVERFLOW;
		goto end;
	}

	if ((cmd_buf_end - (uintptr_t)dump_out_buf) <=
		(uintptr_t)(
		sizeof(struct cam_reg_dump_out_buffer) - sizeof(uint32_t) +
		(dump_out_buf->bytes_written +
		(dmi_read->num_pre_writes * 2 * sizeof(uint32_t)) +
		(dmi_read->dmi_data_read.num_values * 2 *
		sizeof(uint32_t))))) {
		CAM_ERR(CAM_UTIL,
			"Insufficient space in out buffer num_read_val: [%d] num_write_val: [%d] cmd_buf_end: %pK dump_out_buf: %pK",
			dmi_read->dmi_data_read.num_values,
			dmi_read->num_pre_writes, cmd_buf_end,
			(uintptr_t)dump_out_buf);
		rc = -EINVAL;
		goto end;
	}

	write_idx = dump_out_buf->bytes_written / sizeof(uint32_t);
	reg_map_size = (uint32_t)soc_info->reg_map[base_idx].size;
	for (i = 0; i < dmi_read->num_pre_writes; i++) {
		rc = cam_soc_util_reg_addr_validation(reg_map_size,
			dmi_read->pre_read_config[i].offset,
			"pre_read_config");
		if (rc)
			continue;

		cam_soc_util_w_mb(soc_info, base_idx,
			dmi_read->pre_read_config[i].offset,
			dmi_read->pre_read_config[i].value);
		dump_out_buf->dump_data_flex[write_idx++] =
			dmi_read->pre_read_config[i].offset;
		dump_out_buf->dump_data_flex[write_idx++] =
			dmi_read->pre_read_config[i].value;
		dump_out_buf->bytes_written += (2 * sizeof(uint32_t));
	}

	rc = cam_soc_util_reg_addr_validation(reg_map_size,
		dmi_read->dmi_data_read.offset,
		"dmi_data_read");
	if (!rc) {
		for (i = 0; i < dmi_read->dmi_data_read.num_values; i++) {
			dump_out_buf->dump_data_flex[write_idx++] =
				dmi_read->dmi_data_read.offset;
			dump_out_buf->dump_data_flex[write_idx++] =
				cam_soc_util_r_mb(soc_info, base_idx,
				dmi_read->dmi_data_read.offset);
			dump_out_buf->bytes_written += (2 * sizeof(uint32_t));
		}
	}

	for (i = 0; i < dmi_read->num_post_writes; i++) {
		rc = cam_soc_util_reg_addr_validation(reg_map_size,
			dmi_read->post_read_config[i].offset,
			"post_read_config");
		if (rc)
			continue;

		cam_soc_util_w_mb(soc_info, base_idx,
			dmi_read->post_read_config[i].offset,
			dmi_read->post_read_config[i].value);
	}

end:
	return rc;
}

static int cam_soc_util_dump_dmi_reg_range_user_buf(
	struct cam_hw_soc_info *soc_info,
	struct cam_dmi_read_desc *dmi_read, uint32_t base_idx,
	struct cam_hw_soc_dump_args *dump_args)
{
	int                            i;
	int                            rc;
	size_t                         buf_len = 0;
	uint8_t                       *dst;
	size_t                         remain_len;
	uint32_t                       min_len, reg_map_size;
	uint32_t                      *waddr, *start;
	uintptr_t                      cpu_addr;
	struct cam_hw_soc_dump_header *hdr;

	if (!soc_info || !dump_args || !dmi_read) {
		CAM_ERR(CAM_UTIL,
			"Invalid input args soc_info: %pK, dump_args: %pK",
			soc_info, dump_args);
		return -EINVAL;
	}

	if (dmi_read->num_pre_writes > CAM_REG_DUMP_DMI_CONFIG_MAX ||
		dmi_read->num_post_writes > CAM_REG_DUMP_DMI_CONFIG_MAX) {
		CAM_ERR(CAM_UTIL,
			"Invalid number of requested writes, pre: %d post: %d",
			dmi_read->num_pre_writes, dmi_read->num_post_writes);
		return -EINVAL;
	}

	rc = cam_mem_get_cpu_buf(dump_args->buf_handle, &cpu_addr, &buf_len);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Invalid handle %u rc %d",
			dump_args->buf_handle, rc);
		return rc;
	}

	if (buf_len <= dump_args->offset) {
		CAM_WARN(CAM_UTIL, "Dump offset overshoot offset %zu len %zu",
			dump_args->offset, buf_len);
		rc = -ENOSPC;
		goto end;
	}
	remain_len = buf_len - dump_args->offset;
	min_len = (dmi_read->num_pre_writes * 2 * sizeof(uint32_t)) +
		(dmi_read->dmi_data_read.num_values * 2 * sizeof(uint32_t)) +
		sizeof(uint32_t);
	if (remain_len < min_len) {
		CAM_WARN(CAM_UTIL,
			"Dump Buffer exhaust read %d write %d remain %zu min %u",
			dmi_read->dmi_data_read.num_values,
			dmi_read->num_pre_writes, remain_len,
			min_len);
		rc = -ENOSPC;
		goto end;
	}

	dst = (uint8_t *)cpu_addr + dump_args->offset;
	hdr = (struct cam_hw_soc_dump_header *)dst;
	memset(hdr, 0, sizeof(struct cam_hw_soc_dump_header));
	scnprintf(hdr->tag, CAM_SOC_HW_DUMP_TAG_MAX_LEN,
		"DMI_DUMP:");
	waddr = (uint32_t *)(dst + sizeof(struct cam_hw_soc_dump_header));
	start = waddr;
	hdr->word_size = sizeof(uint32_t);
	*waddr = soc_info->index;
	waddr++;

	reg_map_size = (uint32_t)soc_info->reg_map[base_idx].size;
	for (i = 0; i < dmi_read->num_pre_writes; i++) {
		rc = cam_soc_util_reg_addr_validation(reg_map_size,
			dmi_read->pre_read_config[i].offset,
			"pre_read_config");
		if (rc)
			continue;

		cam_soc_util_w_mb(soc_info, base_idx,
			dmi_read->pre_read_config[i].offset,
			dmi_read->pre_read_config[i].value);
		*waddr++ = dmi_read->pre_read_config[i].offset;
		*waddr++ = dmi_read->pre_read_config[i].value;
	}

	rc = cam_soc_util_reg_addr_validation(reg_map_size,
		dmi_read->dmi_data_read.offset,
		"dmi_data_read");
	if (!rc) {
		for (i = 0; i < dmi_read->dmi_data_read.num_values; i++) {
			*waddr++ = dmi_read->dmi_data_read.offset;
			*waddr++ = cam_soc_util_r_mb(soc_info, base_idx,
				dmi_read->dmi_data_read.offset);
		}
	}

	for (i = 0; i < dmi_read->num_post_writes; i++) {
		rc = cam_soc_util_reg_addr_validation(reg_map_size,
			dmi_read->post_read_config[i].offset,
			"post_read_config");
		if (rc)
			continue;

		cam_soc_util_w_mb(soc_info, base_idx,
			dmi_read->post_read_config[i].offset,
			dmi_read->post_read_config[i].value);
	}
	hdr->size = (waddr - start) * hdr->word_size;
	dump_args->offset +=  hdr->size +
		sizeof(struct cam_hw_soc_dump_header);

end:
	cam_mem_put_cpu_buf(dump_args->buf_handle);
	return rc;
}

static int cam_soc_util_dump_cont_reg_range_user_buf(
	struct cam_hw_soc_info *soc_info,
	struct cam_reg_range_read_desc *reg_read,
	uint32_t base_idx,
	struct cam_hw_soc_dump_args *dump_args)
{
	int                            i;
	int                            rc = 0;
	size_t                         buf_len;
	uint8_t                       *dst;
	size_t                         remain_len;
	uint32_t                       min_len, reg_map_size;
	uint32_t                      *waddr, *start;
	uintptr_t                      cpu_addr;
	struct cam_hw_soc_dump_header  *hdr;

	if (!soc_info || !dump_args || !reg_read) {
		CAM_ERR(CAM_UTIL,
			"Invalid input args soc_info: %pK, dump_out_buffer: %pK reg_read: %pK",
			soc_info, dump_args, reg_read);
		return -EINVAL;
	}

	rc = cam_mem_get_cpu_buf(dump_args->buf_handle, &cpu_addr, &buf_len);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Invalid handle %u rc %d",
			dump_args->buf_handle, rc);
		return rc;
	}
	if (buf_len <= dump_args->offset) {
		CAM_WARN(CAM_UTIL, "Dump offset overshoot %zu %zu",
			dump_args->offset, buf_len);
		rc = -ENOSPC;
		goto end;
	}
	remain_len = buf_len - dump_args->offset;
	min_len = (reg_read->num_values * 2 * sizeof(uint32_t)) +
		sizeof(struct cam_hw_soc_dump_header) + sizeof(uint32_t);
	if (remain_len < min_len) {
		CAM_WARN(CAM_UTIL,
			"Dump Buffer exhaust read_values %d remain %zu min %u",
			reg_read->num_values,
			remain_len,
			min_len);
		rc = -ENOSPC;
		goto end;
	}
	dst = (uint8_t *)cpu_addr + dump_args->offset;
	hdr = (struct cam_hw_soc_dump_header *)dst;
	memset(hdr, 0, sizeof(struct cam_hw_soc_dump_header));
	scnprintf(hdr->tag, CAM_SOC_HW_DUMP_TAG_MAX_LEN, "%s_REG:",
		soc_info->dev_name);
	waddr = (uint32_t *)(dst + sizeof(struct cam_hw_soc_dump_header));
	start = waddr;
	hdr->word_size = sizeof(uint32_t);
	*waddr = soc_info->index;
	waddr++;

	reg_map_size = (uint32_t)soc_info->reg_map[base_idx].size;
	for (i = 0; i < reg_read->num_values; i++) {
		rc = cam_soc_util_reg_addr_validation(reg_map_size,
			reg_read->offset + (i * sizeof(uint32_t)),
			"cont_reg_range_user_buf");
		if (rc)
			continue;

		waddr[0] = reg_read->offset + (i * sizeof(uint32_t));
		waddr[1] = cam_soc_util_r(soc_info, base_idx,
			(reg_read->offset + (i * sizeof(uint32_t))));
		waddr += 2;
	}
	hdr->size = (waddr - start) * hdr->word_size;
	dump_args->offset +=  hdr->size +
		sizeof(struct cam_hw_soc_dump_header);
end:
	cam_mem_put_cpu_buf(dump_args->buf_handle);
	return rc;
}

static int cam_soc_util_user_reg_dump(
	struct cam_reg_dump_desc *reg_dump_desc,
	struct cam_hw_soc_dump_args *dump_args,
	struct cam_hw_soc_info *soc_info,
	uint32_t reg_base_idx)
{
	int rc = 0;
	int i;
	struct cam_reg_read_info  *reg_read_info = NULL;

	if (!dump_args || !reg_dump_desc || !soc_info) {
		CAM_ERR(CAM_UTIL,
			"Invalid input parameters %pK %pK %pK",
			dump_args, reg_dump_desc, soc_info);
		return -EINVAL;
	}
	for (i = 0; i < reg_dump_desc->num_read_range; i++) {

		reg_read_info = &reg_dump_desc->read_range_flex[i];
		if (reg_read_info->type ==
				CAM_REG_DUMP_READ_TYPE_CONT_RANGE) {
			rc = cam_soc_util_dump_cont_reg_range_user_buf(
				soc_info,
				&reg_read_info->reg_read,
				reg_base_idx,
				dump_args);
		} else if (reg_read_info->type ==
				CAM_REG_DUMP_READ_TYPE_DMI) {
			rc = cam_soc_util_dump_dmi_reg_range_user_buf(
				soc_info,
				&reg_read_info->dmi_read,
				reg_base_idx,
				dump_args);
		} else {
			CAM_ERR(CAM_UTIL,
					"Invalid Reg dump read type: %d",
					reg_read_info->type);
			rc = -EINVAL;
			goto end;
		}

		if (rc) {
			CAM_ERR(CAM_UTIL,
				"Reg range read failed rc: %d reg_base_idx: %d",
				rc, reg_base_idx);
			goto end;
		}
	}
end:
	return rc;
}

int cam_soc_util_reg_dump_to_cmd_buf(void *ctx,
	struct cam_cmd_buf_desc *cmd_desc, uint64_t req_id,
	cam_soc_util_regspace_data_cb reg_data_cb,
	struct cam_hw_soc_dump_args *soc_dump_args,
	bool user_triggered_dump)
{
	int                               rc = 0, i, j;
	uintptr_t                         cpu_addr = 0;
	uintptr_t                         cmd_buf_start = 0;
	uintptr_t                         cmd_in_data_end = 0;
	uintptr_t                         cmd_buf_end = 0;
	uint32_t                          reg_base_type = 0;
	size_t                            buf_size = 0, remain_len = 0;
	struct cam_reg_dump_input_info   *reg_input_info = NULL;
	struct cam_reg_dump_desc         *reg_dump_desc = NULL;
	struct cam_reg_dump_out_buffer   *dump_out_buf = NULL;
	struct cam_reg_read_info         *reg_read_info = NULL;
	struct cam_hw_soc_info           *soc_info;
	uint32_t                          reg_base_idx = 0;

	if (!ctx || !cmd_desc || !reg_data_cb) {
		CAM_ERR(CAM_UTIL, "Invalid args to reg dump [%pK] [%pK]",
			cmd_desc, reg_data_cb);
		return -EINVAL;
	}

	if (!cmd_desc->length || !cmd_desc->size) {
		CAM_ERR(CAM_UTIL, "Invalid cmd buf size %d %d",
			cmd_desc->length, cmd_desc->size);
		return -EINVAL;
	}

	rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle, &cpu_addr, &buf_size);
	if (rc || !cpu_addr || (buf_size == 0)) {
		CAM_ERR(CAM_UTIL, "Failed in Get cpu addr, rc=%d, cpu_addr=%pK",
			rc, (void *)cpu_addr);
		if (rc)
			return rc;
		goto end;
	}

	CAM_DBG(CAM_UTIL, "Get cpu buf success req_id: %llu buf_size: %zu",
		req_id, buf_size);
	if ((buf_size < sizeof(uint32_t)) ||
		((size_t)cmd_desc->offset > (buf_size - sizeof(uint32_t)))) {
		CAM_ERR(CAM_UTIL, "Invalid offset for cmd buf: %zu",
			(size_t)cmd_desc->offset);
		rc = -EINVAL;
		goto end;
	}

	remain_len = buf_size - (size_t)cmd_desc->offset;
	if ((remain_len < (size_t)cmd_desc->size) || (cmd_desc->size <
		cmd_desc->length)) {
		CAM_ERR(CAM_UTIL,
			"Invalid params for cmd buf len: %zu size: %zu remain_len: %zu",
			(size_t)cmd_desc->length, (size_t)cmd_desc->length,
			remain_len);
		rc = -EINVAL;
		goto end;
	}

	cmd_buf_start = cpu_addr + (uintptr_t)cmd_desc->offset;
	cmd_in_data_end = cmd_buf_start + (uintptr_t)cmd_desc->length;
	cmd_buf_end = cmd_buf_start + (uintptr_t)cmd_desc->size;
	if ((cmd_buf_end <= cmd_buf_start) ||
		(cmd_in_data_end <= cmd_buf_start)) {
		CAM_ERR(CAM_UTIL,
			"Invalid length or size for cmd buf: [%zu] [%zu]",
			(size_t)cmd_desc->length, (size_t)cmd_desc->size);
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_UTIL,
		"Buffer params start [%pK] input_end [%pK] buf_end [%pK]",
		cmd_buf_start, cmd_in_data_end, cmd_buf_end);
	reg_input_info = (struct cam_reg_dump_input_info *) cmd_buf_start;
	if ((reg_input_info->num_dump_sets > 1) && (sizeof(uint32_t) >
		((U32_MAX - sizeof(struct cam_reg_dump_input_info)) /
		(reg_input_info->num_dump_sets - 1)))) {
		CAM_ERR(CAM_UTIL,
			"Integer Overflow req_id: [%llu] num_dump_sets: [%u]",
			req_id, reg_input_info->num_dump_sets);
		rc = -EOVERFLOW;
		goto end;
	}

	if ((!reg_input_info->num_dump_sets) ||
		((cmd_in_data_end - cmd_buf_start) <= (uintptr_t)
		(sizeof(struct cam_reg_dump_input_info) +
		((reg_input_info->num_dump_sets - 1) * sizeof(uint32_t))))) {
		CAM_ERR(CAM_UTIL,
			"Invalid number of dump sets, req_id: [%llu] num_dump_sets: [%u]",
			req_id, reg_input_info->num_dump_sets);
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_UTIL,
		"reg_input_info req_id: %llu ctx %pK num_dump_sets: %d",
		req_id, ctx, reg_input_info->num_dump_sets);
	for (i = 0; i < reg_input_info->num_dump_sets; i++) {
		if ((cmd_in_data_end - cmd_buf_start) <= (uintptr_t)
			reg_input_info->dump_set_offsets_flex[i]) {
			CAM_ERR(CAM_UTIL,
				"Invalid dump set offset: [%pK], cmd_buf_start: [%pK] cmd_in_data_end: [%pK]",
				(uintptr_t)reg_input_info->dump_set_offsets_flex[i],
				cmd_buf_start, cmd_in_data_end);
			rc = -EINVAL;
			goto end;
		}

		reg_dump_desc = (struct cam_reg_dump_desc *)
			(cmd_buf_start +
			(uintptr_t)reg_input_info->dump_set_offsets_flex[i]);
		if ((reg_dump_desc->num_read_range > 1) &&
			(sizeof(struct cam_reg_read_info) > ((U32_MAX -
			sizeof(struct cam_reg_dump_desc)) /
			(reg_dump_desc->num_read_range - 1)))) {
			CAM_ERR(CAM_UTIL,
				"Integer Overflow req_id: [%llu] num_read_range: [%u]",
				req_id, reg_dump_desc->num_read_range);
			rc = -EOVERFLOW;
			goto end;
		}

		if ((!reg_dump_desc->num_read_range) ||
			((cmd_in_data_end - (uintptr_t)reg_dump_desc) <=
			(uintptr_t)(sizeof(struct cam_reg_dump_desc) +
			((reg_dump_desc->num_read_range - 1) *
			sizeof(struct cam_reg_read_info))))) {
			CAM_ERR(CAM_UTIL,
				"Invalid number of read ranges, req_id: [%llu] num_read_range: [%d]",
				req_id, reg_dump_desc->num_read_range);
			rc = -EINVAL;
			goto end;
		}

		if ((cmd_buf_end - cmd_buf_start) <= (uintptr_t)
			(reg_dump_desc->dump_buffer_offset +
			sizeof(struct cam_reg_dump_out_buffer))) {
			CAM_ERR(CAM_UTIL,
				"Invalid out buffer offset: [%pK],  cmd_buf_start: [%pK] cmd_buf_end: [%pK]",
				(uintptr_t)reg_dump_desc->dump_buffer_offset,
				cmd_buf_start, cmd_buf_end);
			rc = -EINVAL;
			goto end;
		}

		reg_base_type = reg_dump_desc->reg_base_type;
		if (reg_base_type == 0 || reg_base_type >
			CAM_REG_DUMP_BASE_TYPE_SFE_RIGHT) {
			CAM_ERR(CAM_UTIL,
				"Invalid Reg dump base type: %d",
				reg_base_type);
			rc = -EINVAL;
			goto end;
		}

		rc = reg_data_cb(reg_base_type, ctx, &soc_info, &reg_base_idx);
		if (rc || !soc_info) {
			CAM_ERR(CAM_UTIL,
				"Reg space data callback failed rc: %d soc_info: [%pK]",
				rc, soc_info);
			rc = -EINVAL;
			goto end;
		}

		if (reg_base_idx > soc_info->num_reg_map) {
			CAM_ERR(CAM_UTIL,
				"Invalid reg base idx: %d num reg map: %d",
				reg_base_idx, soc_info->num_reg_map);
			rc = -EINVAL;
			goto end;
		}

		CAM_DBG(CAM_UTIL,
			"Reg data callback success req_id: %llu base_type: %d base_idx: %d num_read_range: %d",
			req_id, reg_base_type, reg_base_idx,
			reg_dump_desc->num_read_range);

		/* If the dump request is triggered by user space
		 * buffer will be different from the buffer which is received
		 * in init packet. In this case, dump the data to the
		 * user provided buffer and exit.
		 */
		if (user_triggered_dump) {
			rc = cam_soc_util_user_reg_dump(reg_dump_desc,
				soc_dump_args, soc_info, reg_base_idx);
			CAM_INFO(CAM_UTIL,
				"%s reg_base_idx %d dumped offset %u",
				soc_info->dev_name, reg_base_idx,
				soc_dump_args->offset);
			goto end;
		}

		/* Below code is executed when data is dumped to the
		 * out buffer received in init packet
		 */
		dump_out_buf = (struct cam_reg_dump_out_buffer *)
			(cmd_buf_start +
			(uintptr_t)reg_dump_desc->dump_buffer_offset);
		dump_out_buf->req_id = req_id;
		dump_out_buf->bytes_written = 0;

		for (j = 0; j < reg_dump_desc->num_read_range; j++) {
			CAM_DBG(CAM_UTIL,
				"Number of bytes written to cmd buffer: %u req_id: %llu",
				dump_out_buf->bytes_written, req_id);
			reg_read_info = &reg_dump_desc->read_range_flex[j];
			if (reg_read_info->type ==
				CAM_REG_DUMP_READ_TYPE_CONT_RANGE) {
				rc = cam_soc_util_dump_cont_reg_range(soc_info,
					&reg_read_info->reg_read, reg_base_idx,
					dump_out_buf, cmd_buf_end);
			} else if (reg_read_info->type ==
				CAM_REG_DUMP_READ_TYPE_DMI) {
				rc = cam_soc_util_dump_dmi_reg_range(soc_info,
					&reg_read_info->dmi_read, reg_base_idx,
					dump_out_buf, cmd_buf_end);
			} else {
				CAM_ERR(CAM_UTIL,
					"Invalid Reg dump read type: %d",
					reg_read_info->type);
				rc = -EINVAL;
				goto end;
			}

			if (rc) {
				CAM_ERR(CAM_UTIL,
					"Reg range read failed rc: %d reg_base_idx: %d dump_out_buf: %pK",
					rc, reg_base_idx, dump_out_buf);
				goto end;
			}
		}
	}

end:
	cam_mem_put_cpu_buf(cmd_desc->mem_handle);
	return rc;
}

/**
 * cam_soc_util_print_clk_freq()
 *
 * @brief:              This function gets the clk rates for each clk from clk
 *                      driver and prints in log
 *
 * @soc_info:           Device soc struct to be populated
 *
 * @return:             success or failure
 */
int cam_soc_util_print_clk_freq(struct cam_hw_soc_info *soc_info)
{
	int i;
	unsigned long clk_rate = 0;

	if (!soc_info) {
		CAM_ERR(CAM_UTIL, "Invalid soc info");
		return -EINVAL;
	}

	if ((soc_info->num_clk == 0) ||
		(soc_info->num_clk >= CAM_SOC_MAX_CLK)) {
		CAM_ERR(CAM_UTIL, "[%s] Invalid number of clock %d",
			soc_info->dev_name, soc_info->num_clk);
		return -EINVAL;
	}

	for (i = 0; i < soc_info->num_clk; i++) {
		clk_rate = cam_wrapper_clk_get_rate(soc_info->clk[i]);

		CAM_INFO(CAM_UTIL,
			"[%s] idx = %d clk name = %s clk_rate=%lld",
			soc_info->dev_name, i, soc_info->clk_name[i],
			clk_rate);
	}

	return 0;
}

inline unsigned long cam_soc_util_get_applied_src_clk(
	struct cam_hw_soc_info *soc_info, bool is_max)
{
	unsigned long clk_rate;

	/*
	 * For CRMC type, exa - ife, csid, cphy
	 *     final clk = max(hw_client_0, hw_client_1, hw_client_2, sw_client)
	 * For CRMB type, exa - camnoc axi
	 *     final clk = max(hw_client_0 + hw_client_1 + hw_client_2, sw_client)
	 */

	if (is_max) {
		clk_rate = max(soc_info->applied_src_clk_rates.hw_client[0].high,
			soc_info->applied_src_clk_rates.hw_client[1].high);
		clk_rate = max(clk_rate, soc_info->applied_src_clk_rates.hw_client[2].high);
		clk_rate = max(clk_rate, soc_info->applied_src_clk_rates.sw_client);
	} else {
		clk_rate = max((soc_info->applied_src_clk_rates.hw_client[0].high +
			soc_info->applied_src_clk_rates.hw_client[1].high +
			soc_info->applied_src_clk_rates.hw_client[2].high),
			soc_info->applied_src_clk_rates.sw_client);
	}

	return clk_rate;
}

int cam_soc_util_regulators_enabled(struct cam_hw_soc_info *soc_info)
{
	int j = 0, rc = 0;
	int enabled_cnt = 0;

	for (j = 0; j < soc_info->num_rgltr; j++) {
		if (soc_info->rgltr[j]) {
			rc = cam_wrapper_regulator_is_enabled(soc_info->rgltr[j]);
			if (rc < 0) {
				CAM_ERR(CAM_UTIL, "%s regulator_is_enabled failed",
					soc_info->rgltr_name[j]);
			} else if (rc > 0) {
				CAM_DBG(CAM_UTIL, "%s regulator enabled",
					soc_info->rgltr_name[j]);
				enabled_cnt++;
			} else {
				CAM_DBG(CAM_UTIL, "%s regulator is disabled",
					soc_info->rgltr_name[j]);
			}
		}
	}

	return enabled_cnt;
}
