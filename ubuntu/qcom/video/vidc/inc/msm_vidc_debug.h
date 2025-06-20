/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __MSM_VIDC_DEBUG__
#define __MSM_VIDC_DEBUG__

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

struct msm_vidc_core;
struct msm_vidc_inst;

#ifndef VIDC_DBG_LABEL
#define VIDC_DBG_LABEL "msm_vidc"
#endif

/* Allow only 6 prints/sec */
#define VIDC_DBG_SESSION_RATELIMIT_INTERVAL (1 * HZ)
#define VIDC_DBG_SESSION_RATELIMIT_BURST 6

#define VIDC_DBG_TAG_INST VIDC_DBG_LABEL ": %4s: %s: "
#define VIDC_DBG_TAG_CORE VIDC_DBG_LABEL ": %4s: %08x: %s: "
#define FW_DBG_TAG VIDC_DBG_LABEL ": %6s: "
#define DEFAULT_SID ((u32)-1)

#ifndef MSM_VIDC_EMPTY_BRACE
#define MSM_VIDC_EMPTY_BRACE {},
#endif

extern unsigned int msm_vidc_debug;
extern unsigned int msm_fw_debug;
extern bool msm_vidc_lossless_encode;
extern bool msm_vidc_syscache_disable;
extern int msm_vidc_clock_voting;
extern int msm_vidc_ddr_bw;
extern int msm_vidc_llc_bw;
extern bool msm_vidc_fw_dump;
extern unsigned int msm_vidc_enable_bugon;
extern bool msm_vidc_synx_fence_enable;

/* do not modify the log message as it is used in test scripts */
#define FMT_STRING_SET_CTRL \
	"%s: state %s, name %s, id 0x%x value %d\n"
#define FMT_STRING_STATE_CHANGE \
	"%s: state changed to %s from %s\n"
#define FMT_STRING_MSG_SFR \
	"SFR Message from FW: %s\n"
#define FMT_STRING_FAULT_HANDLER \
	"%s: faulting address: %lx\n"
#define FMT_STRING_SET_CAP \
	"set cap: name: %24s, cap value: %#10x, hfi: %#10llx\n"

/* To enable messages OR these values and
 * echo the result to debugfs file.
 *
 * To enable all messages set msm_vidc_debug = 0x101F
 */

enum vidc_msg_prio_drv {
	VIDC_ERR        = 0x00000001,
	VIDC_HIGH       = 0x00000002,
	VIDC_LOW        = 0x00000004,
	VIDC_PERF       = 0x00000008,
	VIDC_PKT        = 0x00000010,
	VIDC_BUS        = 0x00000020,
	VIDC_STAT       = 0x00000040,
	VIDC_ENCODER    = 0x00000100,
	VIDC_DECODER    = 0x00000200,
	VIDC_PRINTK     = 0x10000000,
	VIDC_FTRACE     = 0x20000000,
};

enum vidc_msg_prio_fw {
	FW_LOW          = 0x00000001,
	FW_MED          = 0x00000002,
	FW_HIGH         = 0x00000004,
	FW_ERROR        = 0x00000008,
	FW_FATAL        = 0x00000010,
	FW_PERF         = 0x00000020,
	FW_CACHE_LOW    = 0x00000100,
	FW_CACHE_MED    = 0x00000200,
	FW_CACHE_HIGH   = 0x00000400,
	FW_CACHE_ERROR  = 0x00000800,
	FW_CACHE_FATAL  = 0x00001000,
	FW_CACHE_PERF   = 0x00002000,
	FW_PRINTK       = 0x10000000,
	FW_FTRACE       = 0x20000000,
};

#define DRV_LOG        (VIDC_ERR | VIDC_PRINTK)
#define DRV_LOGSHIFT   (0)
#define DRV_LOGMASK    (0x0FFFFFFF)

#define FW_LOG         (FW_ERROR | FW_FATAL | FW_PRINTK)
#define FW_LOGSHIFT    (0)
#define FW_LOGMASK     (0x0FFFFFFF)

#define dprintk_inst(__level, __level_str, inst, __fmt, ...) \
	do { \
		if (msm_vidc_debug & VIDC_FTRACE) { \
			if (inst && (msm_vidc_debug & (__level))) { \
				trace_printk(VIDC_DBG_TAG_INST __fmt, \
					__level_str, \
					inst->debug_str, \
					##__VA_ARGS__); \
			} \
		} else { \
			if (inst && (msm_vidc_debug & (__level))) { \
				pr_info(VIDC_DBG_TAG_INST __fmt, \
					__level_str, \
					inst->debug_str, \
					##__VA_ARGS__); \
			} \
		} \
	} while (0)

#define i_vpr_e(inst, __fmt, ...) dprintk_inst(VIDC_ERR,  "err ", inst, __fmt, ##__VA_ARGS__)
#define i_vpr_i(inst, __fmt, ...) dprintk_inst(VIDC_HIGH, "high", inst, __fmt, ##__VA_ARGS__)
#define i_vpr_h(inst, __fmt, ...) dprintk_inst(VIDC_HIGH, "high", inst, __fmt, ##__VA_ARGS__)
#define i_vpr_l(inst, __fmt, ...) dprintk_inst(VIDC_LOW,  "low ", inst, __fmt, ##__VA_ARGS__)
#define i_vpr_p(inst, __fmt, ...) dprintk_inst(VIDC_PERF, "perf", inst, __fmt, ##__VA_ARGS__)
#define i_vpr_t(inst, __fmt, ...) dprintk_inst(VIDC_PKT,  "pkt ", inst, __fmt, ##__VA_ARGS__)
#define i_vpr_b(inst, __fmt, ...) dprintk_inst(VIDC_BUS,  "bus ", inst, __fmt, ##__VA_ARGS__)
#define i_vpr_s(inst, __fmt, ...) dprintk_inst(VIDC_STAT, "stat", inst, __fmt, ##__VA_ARGS__)

#define i_vpr_hp(inst, __fmt, ...) \
	dprintk_inst(VIDC_HIGH | VIDC_PERF, "high", inst, __fmt, ##__VA_ARGS__)
#define i_vpr_hs(inst, __fmt, ...) \
	dprintk_inst(VIDC_HIGH | VIDC_STAT, "stat", inst, __fmt, ##__VA_ARGS__)

#define dprintk_core(__level, __level_str, __fmt, ...) \
	do { \
		if (msm_vidc_debug & VIDC_FTRACE) { \
			if (msm_vidc_debug & (__level)) { \
				trace_printk(VIDC_DBG_TAG_CORE __fmt, \
					__level_str, \
					DEFAULT_SID, \
					"codec", \
					##__VA_ARGS__); \
			} \
		} else { \
			if (msm_vidc_debug & (__level)) { \
				pr_info(VIDC_DBG_TAG_CORE __fmt, \
					__level_str, \
					DEFAULT_SID, \
					"codec", \
					##__VA_ARGS__); \
			} \
		} \
	} while (0)

#define d_vpr_e(__fmt, ...) dprintk_core(VIDC_ERR,  "err ", __fmt, ##__VA_ARGS__)
#define d_vpr_h(__fmt, ...) dprintk_core(VIDC_HIGH, "high", __fmt, ##__VA_ARGS__)
#define d_vpr_l(__fmt, ...) dprintk_core(VIDC_LOW,  "low ", __fmt, ##__VA_ARGS__)
#define d_vpr_p(__fmt, ...) dprintk_core(VIDC_PERF, "perf", __fmt, ##__VA_ARGS__)
#define d_vpr_t(__fmt, ...) dprintk_core(VIDC_PKT,  "pkt ", __fmt, ##__VA_ARGS__)
#define d_vpr_b(__fmt, ...) dprintk_core(VIDC_BUS,  "bus ", __fmt, ##__VA_ARGS__)
#define d_vpr_s(__fmt, ...) dprintk_core(VIDC_STAT, "stat", __fmt, ##__VA_ARGS__)
#define d_vpr_hs(__fmt, ...) \
	dprintk_core(VIDC_HIGH | VIDC_STAT, "high", __fmt, ##__VA_ARGS__)

#define dprintk_ratelimit(__level, __level_str, __fmt, ...) \
	do { \
		if (msm_vidc_check_ratelimit()) { \
			dprintk_core(__level, __level_str, __fmt, ##__VA_ARGS__); \
		} \
	} while (0)

#define dprintk_firmware(__level, __fmt, ...)	\
	do { \
		if ((msm_fw_debug & (__level)) & FW_PRINTK) { \
			pr_info(FW_DBG_TAG __fmt, \
				"fw", \
				##__VA_ARGS__); \
		} else if ((msm_fw_debug & (__level)) & FW_FTRACE) { \
			trace_printk(FW_DBG_TAG __fmt, \
				"fw", \
				##__VA_ARGS__); \
		} \
	} while (0)

#define MSM_VIDC_FATAL(value)	\
	do { \
		if (value) { \
			d_vpr_e("bug on\n"); \
			BUG_ON(value); \
		} \
	} while (0)

enum msm_vidc_debugfs_event {
	MSM_VIDC_DEBUGFS_EVENT_ETB,
	MSM_VIDC_DEBUGFS_EVENT_EBD,
	MSM_VIDC_DEBUGFS_EVENT_FTB,
	MSM_VIDC_DEBUGFS_EVENT_FBD,
};

enum msm_vidc_bug_on_error {
	MSM_VIDC_BUG_ON_FATAL             = BIT(0),
	MSM_VIDC_BUG_ON_NOC               = BIT(1),
	MSM_VIDC_BUG_ON_WD_TIMEOUT        = BIT(2),
};

struct dentry *msm_vidc_debugfs_init_drv(void);
struct dentry *msm_vidc_debugfs_init_core(struct msm_vidc_core *core);
struct dentry *msm_vidc_debugfs_init_inst(struct msm_vidc_inst *inst,
					  struct dentry *parent);
void msm_vidc_debugfs_deinit_inst(struct msm_vidc_inst *inst);
void msm_vidc_debugfs_update(struct msm_vidc_inst *inst,
			     enum msm_vidc_debugfs_event e);
int msm_vidc_check_ratelimit(void);
void msm_vidc_show_stats(struct msm_vidc_inst *inst);

static inline bool is_stats_enabled(void)
{
	return !!(msm_vidc_debug & VIDC_STAT);
}

#endif
