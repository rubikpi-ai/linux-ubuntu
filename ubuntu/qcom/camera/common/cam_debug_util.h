/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_DEBUG_UTIL_H_
#define _CAM_DEBUG_UTIL_H_

#include <msm-camera.h>
#include <linux/platform_device.h>
#include "cam_trace.h"

#ifndef CONFIG_SPECTRA_KT
#include "cam_presil_hw_access.h"
#endif // !CONFIG_SPECTRA_KT

extern unsigned long long debug_mdl;
extern unsigned int debug_type;
extern unsigned int debug_priority;
extern unsigned int debug_drv;
extern unsigned int debug_bypass_drivers;

#define CAM_IS_NULL_TO_STR(ptr) ((ptr) ? "Non-NULL" : "NULL")

#define CAM_LOG_BUF_LEN                  512
#define BYPASS_VALUE       0xDEADBEEF
#define DEFAULT_CLK_VALUE  19200000

/* Module IDs used for debug logging */
enum cam_debug_module_id {
	CAM_CDM,                 /* bit 0 */
	CAM_CORE,                /* bit 1 */
	CAM_CPAS,                /* bit 2 */
	CAM_ISP,                 /* bit 3 */
	CAM_CRM,                 /* bit 4 */
	CAM_SENSOR,              /* bit 5 */
	CAM_SMMU,                /* bit 6 */
	CAM_SYNC,                /* bit 7 */
	CAM_ICP,                 /* bit 8 */
	CAM_JPEG,                /* bit 9 */
	CAM_FD,                  /* bit 10 */
	CAM_LRME,                /* bit 11 */
	CAM_FLASH,               /* bit 12 */
	CAM_ACTUATOR,            /* bit 13 */
	CAM_CCI,                 /* bit 14 */
	CAM_CSIPHY,              /* bit 15 */
	CAM_EEPROM,              /* bit 16 */
	CAM_UTIL,                /* bit 17 */
	CAM_HFI,                 /* bit 18 */
	CAM_CTXT,                /* bit 19 */
	CAM_OIS,                 /* bit 20 */
	CAM_RES,                 /* bit 21 */
	CAM_MEM,                 /* bit 22 */
	CAM_IRQ_CTRL,            /* bit 23 */
	CAM_REQ,                 /* bit 24 */
	CAM_PERF,                /* bit 25 */
	CAM_CUSTOM,              /* bit 26 */
	CAM_PRESIL,              /* bit 27 */
	CAM_OPE,                 /* bit 28 */
	CAM_IO_ACCESS,           /* bit 29 */
	CAM_SFE,                 /* bit 30 */
	CAM_IR_LED,              /* bit 31 */
	CAM_CRE,                 /* bit 32 */
	CAM_PRESIL_CORE,         /* bit 33 */
	CAM_TPG,                 /* bit 34 */
	CAM_DMA_FENCE,           /* bit 35 */
	CAM_SENSOR_UTIL,         /* bit 36 */
	CAM_SYNX,                /* bit 37 */
	CAM_DBG_MOD_MAX
};

/* Log level types */
enum cam_debug_log_level {
	CAM_TYPE_TRACE,
	CAM_TYPE_ERR,
	CAM_TYPE_WARN,
	CAM_TYPE_INFO,
	CAM_TYPE_DBG,
	CAM_TYPE_MAX,
};

/*
 * enum cam_debug_priority - Priority of debug log (0 = Lowest)
 */
enum cam_debug_priority {
	CAM_DBG_PRIORITY_0,
	CAM_DBG_PRIORITY_1,
	CAM_DBG_PRIORITY_2,
};

#define ___CAM_DBG_MOD_NAME(module_id)                                      \
__builtin_choose_expr(((module_id) == CAM_CDM), "CAM-CDM",                  \
__builtin_choose_expr(((module_id) == CAM_CORE), "CAM-CORE",                \
__builtin_choose_expr(((module_id) == CAM_CRM), "CAM-CRM",                  \
__builtin_choose_expr(((module_id) == CAM_CPAS), "CAM-CPAS",                \
__builtin_choose_expr(((module_id) == CAM_ISP), "CAM-ISP",                  \
__builtin_choose_expr(((module_id) == CAM_SENSOR), "CAM-SENSOR",            \
__builtin_choose_expr(((module_id) == CAM_SMMU), "CAM-SMMU",                \
__builtin_choose_expr(((module_id) == CAM_SYNC), "CAM-SYNC",                \
__builtin_choose_expr(((module_id) == CAM_ICP), "CAM-ICP",                  \
__builtin_choose_expr(((module_id) == CAM_JPEG), "CAM-JPEG",                \
__builtin_choose_expr(((module_id) == CAM_FD), "CAM-FD",                    \
__builtin_choose_expr(((module_id) == CAM_LRME), "CAM-LRME",                \
__builtin_choose_expr(((module_id) == CAM_FLASH), "CAM-FLASH",              \
__builtin_choose_expr(((module_id) == CAM_ACTUATOR), "CAM-ACTUATOR",        \
__builtin_choose_expr(((module_id) == CAM_CCI), "CAM-CCI",                  \
__builtin_choose_expr(((module_id) == CAM_CSIPHY), "CAM-CSIPHY",            \
__builtin_choose_expr(((module_id) == CAM_EEPROM), "CAM-EEPROM",            \
__builtin_choose_expr(((module_id) == CAM_UTIL), "CAM-UTIL",                \
__builtin_choose_expr(((module_id) == CAM_CTXT), "CAM-CTXT",                \
__builtin_choose_expr(((module_id) == CAM_HFI), "CAM-HFI",                  \
__builtin_choose_expr(((module_id) == CAM_OIS), "CAM-OIS",                  \
__builtin_choose_expr(((module_id) == CAM_IRQ_CTRL), "CAM-IRQ-CTRL",        \
__builtin_choose_expr(((module_id) == CAM_MEM), "CAM-MEM",                  \
__builtin_choose_expr(((module_id) == CAM_PERF), "CAM-PERF",                \
__builtin_choose_expr(((module_id) == CAM_REQ), "CAM-REQ",                  \
__builtin_choose_expr(((module_id) == CAM_CUSTOM), "CAM-CUSTOM",            \
__builtin_choose_expr(((module_id) == CAM_OPE), "CAM-OPE",                  \
__builtin_choose_expr(((module_id) == CAM_PRESIL), "CAM-PRESIL",            \
__builtin_choose_expr(((module_id) == CAM_RES), "CAM-RES",                  \
__builtin_choose_expr(((module_id) == CAM_IO_ACCESS), "CAM-IO-ACCESS",      \
__builtin_choose_expr(((module_id) == CAM_SFE), "CAM-SFE",                  \
__builtin_choose_expr(((module_id) == CAM_CRE), "CAM-CRE",                  \
__builtin_choose_expr(((module_id) == CAM_PRESIL_CORE), "CAM-CORE-PRESIL",  \
__builtin_choose_expr(((module_id) == CAM_TPG), "CAM-TPG",                  \
__builtin_choose_expr(((module_id) == CAM_DMA_FENCE), "CAM-DMA-FENCE",      \
__builtin_choose_expr(((module_id) == CAM_SENSOR_UTIL), "CAM-SENSOR-UTIL",      \
__builtin_choose_expr(((module_id) == CAM_SYNX), "CAM-SYNX",                \
"CAMERA")))))))))))))))))))))))))))))))))))))

enum cam_log_print_type {
	CAM_PRINT_LOG   = 0x1,
	CAM_PRINT_TRACE = 0x2,
	CAM_PRINT_BOTH  = 0x3,
};

#define __CAM_LOG_FMT KERN_INFO "%s: %s: %s: %d: %s "

/**
 * cam_print_log() - function to print logs (internal use only, use macros instead)
 *
 * @type:      Corresponds to enum cam_log_print_type, selects if logs are printed in log buffer,
 *        trace buffers or both
 * @module_id: Module calling the log macro
 * @tag:       Tag for log level
 * @func:      Function string
 * @line:      Line number
 * @fmt:       Formatting string
 */

void cam_print_log(int type, int module, int tag, const char *func,
	int line, const char *fmt, ...);

#define __CAM_LOG(type, tag, module_id, fmt, args...)                               \
({                                                                                  \
	cam_print_log(type,                                      \
		module_id, tag, __func__,   \
		__LINE__,  fmt, ##args);                                                  \
})

#define CAM_LOG(tag, module_id, fmt, args...) \
__CAM_LOG(CAM_PRINT_BOTH, tag, module_id, fmt, ##args)

#define CAM_LOG_RL_CUSTOM(type, module_id, interval, burst, fmt, args...)                \
({                                                                                       \
	static DEFINE_RATELIMIT_STATE(_rs, (interval * HZ), burst);                      \
	__CAM_LOG(__ratelimit(&_rs) ? CAM_PRINT_BOTH : CAM_PRINT_TRACE,                  \
		type, module_id, fmt, ##args);                                           \
})

#define CAM_LOG_RL(type, module_id, fmt, args...)                                        \
CAM_LOG_RL_CUSTOM(type, module_id, DEFAULT_RATELIMIT_INTERVAL, DEFAULT_RATELIMIT_BURST,  \
fmt, ##args)

#define __CAM_DBG(module_id, priority, fmt, args...)                                              \
({                                                                                                \
	if (unlikely((debug_mdl & BIT_ULL(module_id)) && (priority >= debug_priority))) {         \
		CAM_LOG(CAM_TYPE_DBG, module_id, fmt, ##args);                                    \
	}                                                                                         \
})

/**
 * CAM_ERR / CAM_WARN / CAM_INFO / CAM_TRACE
 *
 * @brief: Macros to print logs at respective level error/warn/info/trace. All
 * logs except CAM_TRACE are printed in both log and trace buffers.
 *
 * @__module: Respective enum cam_debug_module_id
 * @fmt:      Format string
 * @args:     Arguments to match with format
 */
#define CAM_ERR(__module, fmt, args...)  CAM_LOG(CAM_TYPE_ERR, __module, fmt, ##args)
#define CAM_WARN(__module, fmt, args...) CAM_LOG(CAM_TYPE_WARN, __module, fmt, ##args)
#define CAM_INFO(__module, fmt, args...) CAM_LOG(CAM_TYPE_INFO, __module, fmt, ##args)
#define CAM_TRACE(__module, fmt, args...) \
__CAM_LOG(CAM_PRINT_TRACE, CAM_TYPE_TRACE, __module, fmt, ##args)

/**
 * CAM_ERR_RATE_LIMIT / CAM_WARN_RATE_LIMIT / CAM_INFO_RATE_LIMIT
 *
 * @brief: Rate limited version of logs used to reduce log spew.
 *
 * @__module: Respective enum cam_debug_module_id
 * @fmt:      Format string
 * @args:     Arguments to match with format
 */
#define CAM_ERR_RATE_LIMIT(__module, fmt, args...)  CAM_LOG_RL(CAM_TYPE_ERR, __module, fmt, ##args)
#define CAM_WARN_RATE_LIMIT(__module, fmt, args...) CAM_LOG_RL(CAM_TYPE_WARN, __module, fmt, ##args)
#define CAM_INFO_RATE_LIMIT(__module, fmt, args...) CAM_LOG_RL(CAM_TYPE_INFO, __module, fmt, ##args)

/**
 * CAM_ERR_RATE_LIMIT_CUSTOM / CAM_WARN_RATE_LIMITT_CUSTOM/ CAM_INFO_RATE_LIMITT_CUSTOM
 *
 * @brief: Rate limited version of logs used to reduce log spew that can have
 * customized burst rate
 *
 * @__module: Respective enum cam_debug_module_id
 * @interval: Sliding window interval in which to count logs
 * @burst:    Maximum number of logs in the specified interval
 * @fmt:      Format string
 * @args:     Arguments to match with format
 */
#define CAM_ERR_RATE_LIMIT_CUSTOM(__module, interval, burst, fmt, args...)  \
	CAM_LOG_RL_CUSTOM(CAM_TYPE_ERR, __module, interval, burst, fmt, ##args)

#define CAM_WARN_RATE_LIMIT_CUSTOM(__module, interval, burst, fmt, args...) \
	CAM_LOG_RL_CUSTOM(CAM_TYPE_WARN, __module, interval, burst, fmt, ##args)

#define CAM_INFO_RATE_LIMIT_CUSTOM(__module, interval, burst, fmt, args...) \
	CAM_LOG_RL_CUSTOM(CAM_TYPE_INFO, __module, interval, burst, fmt, ##args)

/*
 * CAM_DBG
 * @brief    :  This Macro will print debug logs when enabled using GROUP and
 *              if its priority is greater than the priority parameter
 *
 * @__module :  Respective module id which is been calling this Macro
 * @fmt      :  Formatted string which needs to be print in log
 * @args     :  Arguments which needs to be print in log
 */
#define CAM_DBG(__module, fmt, args...)     __CAM_DBG(__module, CAM_DBG_PRIORITY_0, fmt, ##args)
#define CAM_DBG_PR1(__module, fmt, args...) __CAM_DBG(__module, CAM_DBG_PRIORITY_1, fmt, ##args)
#define CAM_DBG_PR2(__module, fmt, args...) __CAM_DBG(__module, CAM_DBG_PRIORITY_2, fmt, ##args)

/**
 * cam_print_to_buffer
 * @brief:         Function to print to camera logs to a buffer. Don't use directly. Use macros
 *                 provided below.
 *
 * @buf:           Buffer to print into
 * @buf_size:      Total size of the buffer
 * @len:           Pointer to variable used to keep track of the length
 * @tag:           Log level tag to be prefixed
 * @module_id:     Module id tag to be prefixed
 * @fmt:           Formatted string which needs to be print in log
 * @args:          Arguments which needs to be print in log
 */
void cam_print_to_buffer(char *buf, const size_t buf_size, size_t *len, unsigned int tag,
	unsigned long long module_id, const char *fmt, ...);

/**
 * CAM_[ERR/WARN/INFO]_BUF
 * @brief:         Macro to print a new line into log buffer.
 *
 * @module_id:     Module id tag to be prefixed
 * @buf:           Buffer to print into
 * @buf_size:      Total size of the buffer
 * @len:           Pointer to the variable used to keep track of the length
 * @fmt:           Formatted string which needs to be print in log
 * @args:          Arguments which needs to be print in log
 */
#define CAM_ERR_BUF(module_id, buf, buf_size, len, fmt, args...)                                   \
	cam_print_to_buffer(buf, buf_size, len, CAM_TYPE_ERR, module_id, fmt, ##args)
#define CAM_WARN_BUF(module_id, buf, buf_size, len, fmt, args...)                                  \
	cam_print_to_buffer(buf, buf_size, len, CAM_TYPE_WARN, module_id, fmt, ##args)
#define CAM_INFO_BUF(module_id, buf, buf_size, len, fmt, args...)                                  \
	cam_print_to_buffer(buf, buf_size, len, CAM_TYPE_INFO, module_id, fmt, ##args)

#define CAM_BOOL_TO_YESNO(val) ((val) ? "Y" : "N")

/**
 * struct cam_cpas_debug_settings - Sysfs debug settings for cpas driver
 */
struct cam_cpas_debug_settings {
	uint64_t mnoc_hf_0_ab_bw;
	uint64_t mnoc_hf_0_ib_bw;
	uint64_t mnoc_hf_1_ab_bw;
	uint64_t mnoc_hf_1_ib_bw;
	uint64_t mnoc_sf_0_ab_bw;
	uint64_t mnoc_sf_0_ib_bw;
	uint64_t mnoc_sf_1_ab_bw;
	uint64_t mnoc_sf_1_ib_bw;
	uint64_t mnoc_sf_icp_ab_bw;
	uint64_t mnoc_sf_icp_ib_bw;
	uint64_t camnoc_bw;
	uint64_t cam_ife_0_drv_ab_high_bw;
	uint64_t cam_ife_0_drv_ib_high_bw;
	uint64_t cam_ife_1_drv_ab_high_bw;
	uint64_t cam_ife_1_drv_ib_high_bw;
	uint64_t cam_ife_2_drv_ab_high_bw;
	uint64_t cam_ife_2_drv_ib_high_bw;
	uint64_t cam_ife_0_drv_ab_low_bw;
	uint64_t cam_ife_0_drv_ib_low_bw;
	uint64_t cam_ife_1_drv_ab_low_bw;
	uint64_t cam_ife_1_drv_ib_low_bw;
	uint64_t cam_ife_2_drv_ab_low_bw;
	uint64_t cam_ife_2_drv_ib_low_bw;
	uint64_t cam_ife_0_drv_low_set_zero;
	uint64_t cam_ife_1_drv_low_set_zero;
	uint64_t cam_ife_2_drv_low_set_zero;
	bool     is_updated;
};

/**
 * struct camera_debug_settings - Sysfs debug settings for camera
 *
 * @cpas_settings: Debug settings for cpas driver.
 */
struct camera_debug_settings {
	struct cam_cpas_debug_settings cpas_settings;
};

/**
 * @brief : API to get camera debug settings
 * @return const struct camera_debug_settings pointer.
 */
const struct camera_debug_settings *cam_debug_get_settings(void);

/**
 * @brief : API to parse and store input from sysfs debug node
 * @return Number of bytes read from buffer on success, or -EPERM on error.
 */
ssize_t cam_debug_sysfs_node_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

/**
 * cam_debugfs_init()
 *
 * @brief: create camera debugfs root folder
 */
void cam_debugfs_init(void);

/**
 * cam_debugfs_deinit()
 *
 * @brief: remove camera debugfs root folder
 */
void cam_debugfs_deinit(void);

/**
 * cam_debugfs_create_subdir()
 *
 * @brief:  create a directory within the camera debugfs root folder
 *
 * @name:   name of the directory
 * @subdir: pointer to the newly created directory entry
 *
 * @return: 0 on success, negative on failure
 */
int cam_debugfs_create_subdir(const char *name, struct dentry **subdir);

/**
 * cam_debugfs_lookup_subdir()
 *
 * @brief:  lookup a directory within the camera debugfs root folder
 *
 * @name:   name of the directory
 * @subdir: pointer to the successfully found directory entry
 *
 * @return: 0 on success, negative on failure
 */
int cam_debugfs_lookup_subdir(const char *name, struct dentry **subdir);

/**
 * cam_debugfs_available()
 *
 * @brief:  Check if debugfs is enabled for camera. Use this function before creating any
 *          debugfs entries.
 *
 * @return: true if enabled, false otherwise
 */
static inline bool cam_debugfs_available(void)
{
	#if defined(CONFIG_DEBUG_FS)
		return true;
	#else
		return false;
	#endif
}

#endif /* _CAM_DEBUG_UTIL_H_ */
