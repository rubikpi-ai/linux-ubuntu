/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_CSID_HW_INTF_H_
#define _CAM_CSID_HW_INTF_H_

#include "cam_isp_hw.h"
#include "cam_hw_intf.h"

/* MAX IFE CSID instance */
#define CAM_IFE_CSID_HW_NUM_MAX                        8
#define CAM_IFE_CSID_UDI_MAX                           3
#define RT_BASE_IDX                                    2
#define CAM_ISP_MAX_PATHS                              8

/**
 * enum cam_ife_csid_hw_irq_regs - Specify the top irq reg
 */
enum cam_ife_csid_hw_irq_regs {
	CAM_IFE_CSID_IRQ_TOP_REG_STATUS0,
	CAM_IFE_CSID_IRQ_REGISTERS_MAX,
};

/**
 * enum cam_ife_csid_top_irq_regs - Specify the top irq reg
 */
enum cam_ife_csid_top_irq_regs {
	CAM_IFE_CSID_TOP_IRQ_STATUS_REG0,
	CAM_IFE_CSID_TOP2_IRQ_STATUS_REG1,
	CAM_IFE_CSID_TOP_IRQ_STATUS_REG_MAX,
};

/**
 * enum cam_ife_csid_rx_irq_regs - Specify the rx irq reg
 */
enum cam_ife_csid_rx_irq_regs {
	CAM_IFE_CSID_RX_IRQ_STATUS_REG0,
	CAM_IFE_CSID_RX2_IRQ_STATUS_REG1,
	CAM_IFE_CSID_RX_IRQ_STATUS_REG_MAX,
};

/**
 * enum cam_ife_csid_input_core_type - Specify the csid input core
 */
enum cam_ife_csid_input_core_type {
	CAM_IFE_CSID_INPUT_CORE_NONE,
	CAM_IFE_CSID_INPUT_CORE_IFE,
	CAM_IFE_CSID_INPUT_CORE_SFE_IFE,
	CAM_IFE_CSID_INPUT_CORE_SFE,
	CAM_IFE_CSID_INPUT_CORE_CUST_IFE,
};

/**
 * enum cam_ife_pix_path_res_id - Specify the csid patch
 */
enum cam_ife_pix_path_res_id {
	CAM_IFE_PIX_PATH_RES_RDI_0,
	CAM_IFE_PIX_PATH_RES_RDI_1,
	CAM_IFE_PIX_PATH_RES_RDI_2,
	CAM_IFE_PIX_PATH_RES_RDI_3,
	CAM_IFE_PIX_PATH_RES_RDI_4,
	CAM_IFE_PIX_PATH_RES_RDI_5,
	CAM_IFE_PIX_PATH_RES_IPP,
	CAM_IFE_PIX_PATH_RES_PPP,
	CAM_IFE_PIX_PATH_RES_UDI_0,
	CAM_IFE_PIX_PATH_RES_UDI_1,
	CAM_IFE_PIX_PATH_RES_UDI_2,
	CAM_IFE_PIX_PATH_RES_IPP_1,
	CAM_IFE_PIX_PATH_RES_IPP_2,
	CAM_IFE_PIX_PATH_RES_MAX,
};

/**
 * enum cam_ife_cid_res_id - Specify the csid cid
 */
enum cam_ife_cid_res_id {
	CAM_IFE_CSID_CID_0,
	CAM_IFE_CSID_CID_1,
	CAM_IFE_CSID_CID_2,
	CAM_IFE_CSID_CID_3,
	CAM_IFE_CSID_CID_MAX,
};

/**
 * enum cam_ife_csid_secondary_evt_type - Specify the event type
 */
enum cam_ife_csid_secondary_evt_type {
	CAM_IFE_CSID_EVT_SOF = 1,
	CAM_IFE_CSID_EVT_EPOCH,
	CAM_IFE_CSID_EVT_EOF,
	CAM_IFE_CSID_EVT_SENSOR_SYNC_FRAME_DROP,
	CAM_IFE_CSID_EVT_MAX,
};

/**
 * struct cam_ife_csid_hw_caps- get the CSID hw capability
 * @num_rdis:              number of rdis supported by CSID HW device
 * @num_pix:               number of pxl paths supported by CSID HW device
 * @num_ppp:               number of ppp paths supported by CSID HW device
 * @major_version :        major version
 * @minor_version:         minor version
 * @version_incr:          version increment
 * @sfe_ipp_input_rdi_res: RDI Res as an input to SFE
 * @is_lite:               is the ife_csid lite
 * @global_reset_en:       flag to indicate if global reset is enabled
 * @rup_en:                flag to indicate if rup is on csid side
 * @only_master_rup:       flag to indicate if only master RUP
 * @camif_irq_support:     flag to indicate if CSID supports CAMIF irq
 * @is_ife_sfe_mapped:     flag to indicate if IFE & SFE are one-one mapped
 */
struct cam_ife_csid_hw_caps {
	uint32_t      num_rdis;
	uint32_t      num_pix;
	uint32_t      num_ppp;
	uint32_t      major_version;
	uint32_t      minor_version;
	uint32_t      version_incr;
	uint32_t      sfe_ipp_input_rdi_res;
	bool          is_lite;
	bool          global_reset_en;
	bool          rup_en;
	bool          only_master_rup;
	bool          camif_irq_support;
	bool          is_ife_sfe_mapped;
};

struct cam_isp_out_port_generic_info {
	uint32_t                res_type;
	uint32_t                format;
	uint32_t                width;
	uint32_t                height;
	uint32_t                comp_grp_id;
	uint32_t                split_point;
	uint32_t                secure_mode;
	uint32_t                reserved;
	uint32_t                wm_mode;
	uint32_t                hw_context_id;
	uint32_t                acquired_res_type;
};

struct cam_isp_in_port_generic_info {
	uint32_t                        major_ver;
	uint32_t                        minor_ver;
	uint32_t                        res_type;
	uint32_t                        lane_type;
	uint32_t                        lane_num;
	uint32_t                        lane_cfg;
	uint32_t                        vc[CAM_ISP_VC_DT_CFG];
	uint32_t                        dt[CAM_ISP_VC_DT_CFG];
	uint32_t                        num_valid_vc_dt;
	uint32_t                        format[CAM_ISP_VC_DT_CFG];
	uint32_t                        test_pattern;
	uint32_t                        usage_type;
	uint32_t                        left_start;
	uint32_t                        left_stop;
	uint32_t                        left_width;
	uint32_t                        right_start;
	uint32_t                        right_stop;
	uint32_t                        right_width;
	uint32_t                        line_start;
	uint32_t                        line_stop;
	uint32_t                        height;
	uint32_t                        pixel_clk;
	uint32_t                        batch_size;
	uint32_t                        dsp_mode;
	uint32_t                        hbi_cnt;
	uint32_t                        fe_unpacker_fmt;
	uint32_t                        cust_node;
	uint32_t                        num_out_res;
	uint32_t                        horizontal_bin;
	uint32_t                        vertical_bin;
	uint32_t                        qcfa_bin;
	uint32_t                        num_bytes_out;
	uint32_t                        ipp_count;
	uint32_t                        ppp_count;
	uint32_t                        rdi_count;
	uint32_t                        udi_count;
	uint32_t                        lcr_count;
	uint32_t                        ife_rd_count;
	uint32_t                        lite_path_count;
	uint32_t                        sfe_port_count;
	uint32_t                        sfe_in_path_type;
	uint32_t                        sfe_ife_enable;
	uint32_t                        epoch_factor;
	uint32_t                        path_id;
	uint32_t                        ipp_dst_hw_ctxt_mask;
	bool                            secure_mode;
	uint32_t                        sensor_id;
	bool                            dynamic_sensor_switch_en;
	bool                            can_use_lite;
	bool                            sfe_binned_epoch_cfg;
	bool                            epd_supported;
	bool                            aeb_mode;
	bool                            dynamic_hdr_switch_en;
	bool                            yuv_chroma_conversion_en;
	bool                            per_port_en;
	bool                            is_offline;
	struct cam_isp_out_port_generic_info    *data;
};

/**
 * struct cam_csid_secondary_evt_config - secondary event enablement
 * @evt_type:           Type of secondary event enabled [SOF/EPOCH/EOF...]
 * @en_secondary_evt:   Enable secondary event
 *
 */
struct cam_csid_secondary_evt_config {
	enum cam_ife_csid_secondary_evt_type evt_type;
	bool                                 en_secondary_evt;
};

/**
 * struct cam_csid_hw_reserve_resource- hw reserve
 * @res_type :           Reource type CID or PATH
 *                       if type is CID, then res_id is not required,
 *                       if type is path then res id need to be filled
 * @res_id  :            Resource id to be reserved
 * @in_port :            Input port resource info
 * @out_port:            Output port resource info, used for RDI path only
 * @sync_mode:           Sync mode
 *                       Sync mode could be master, slave or none
 * @master_idx:          Master device index to be configured in the
 *                       slave path
 *                       for master path, this value is not required.
 *                       only slave need to configure the master index value
 * @dual_core_id:        In case of dual csid, core id of another hw
 *                       reserve
 * @node_res :           Reserved resource structure pointer
 * @sec_evt_config:      Config to enable secondary events for the given resource
 *                       depending on the use-case
 * @crop_enable :        Flag to indicate CSID crop enable
 * @drop_enable :        Flag to indicate CSID drop enable
 * @sfe_inline_shdr:     Flag to indicate if sfe is inline shdr
 * @is_offline :         Flag to indicate offline
 * @need_top_cfg:        Flag to indicate if top cfg is needed
 * @tasklet:             Tasklet to schedule bottom halves
 * @buf_done_controller: IRQ controller for buf done for version 680 hw
 * @cdm_ops:             CDM Ops
 * @event_cb:            Callback function to hw mgr in case of hw events
 * @phy_sel:             Phy selection number if tpg is enabled from userspace
 * @cb_priv:             Private pointer to return to callback
 * @sfe_en:              Flag to indicate if SFE is enabled
 * @use_wm_pack:         [OUT]Flag to indicate if WM packing is to be used for packing
 * @handle_camif_irq:    Flag to indicate if CSID IRQ is enabled
 * @per_port_acquire:    Indicates if acquire as real acquire or per port
 *                       virtual acquire for current res path
 * @vc:                  input virtual channel number
 * @dt:                  input data type number
 * @decode_format        input data format
 * @per_port_group_index: Group index if per port is enabled
 *
 */
struct cam_csid_hw_reserve_resource_args {
	enum cam_isp_resource_type                res_type;
	uint32_t                                  res_id;
	struct cam_isp_in_port_generic_info      *in_port;
	struct cam_isp_out_port_generic_info     *out_port;
	enum cam_isp_hw_sync_mode                 sync_mode;
	uint32_t                                  master_idx;
	uint32_t                                  dual_core_id;
	struct cam_isp_resource_node             *node_res;
	struct cam_csid_secondary_evt_config      sec_evt_config;
	bool                                      crop_enable;
	bool                                      drop_enable;
	bool                                      sfe_inline_shdr;
	bool                                      is_offline;
	bool                                      need_top_cfg;
	void                                     *tasklet;
	void                                     *buf_done_controller;
	void                                     *cdm_ops;
	cam_hw_mgr_event_cb_func                  event_cb;
	uint32_t                                  phy_sel;
	void                                     *cb_priv;
	bool                                      sfe_en;
	bool                                      use_wm_pack;
	bool                                      handle_camif_irq;
	bool                                      per_port_acquire;
	uint32_t                                  vc;
	uint32_t                                  dt;
	uint32_t                                  decode_format;
	int                                       per_port_grp_index;
};

/**
 * struct cam_ife_mgr_csid_resource_update
 * @priv:                  Context data
 * @res:                   HW resource to get the update from
 * @csid_acquire:          Csid hw reserve
 *
 */
struct cam_csid_resource_update {
	void                                     *priv;
	struct cam_isp_hw_mgr_res                *res;
	struct cam_csid_hw_reserve_resource_args *csid_acquire;
};

/**
 * struct cam_csid_res_irq_info
 * @priv:                  Context data
 * @node_res:              reource pointer array( ie CSID/IFE_SRC/IFE_OUT)
 * @num_res:               number of resources
 * @enable_irq:            TRUE: enables irq for requested path
 *                         FALSE: disables irq for requested path
 * @is_internal_start:     Start triggered internally for reset & recovery
 *
 */
struct cam_csid_res_irq_info {
	void                                *priv;
	struct cam_isp_resource_node        **node_res;
	uint32_t                             num_res;
	bool                                 enable_irq;
	bool                                 is_internal_start;
};

/**
 *  enum cam_ife_csid_halt_cmd - Specify the halt command type
 */
enum cam_ife_csid_halt_cmd {
	CAM_CSID_HALT_AT_FRAME_BOUNDARY,
	CAM_CSID_RESUME_AT_FRAME_BOUNDARY,
	CAM_CSID_HALT_IMMEDIATELY,
	CAM_CSID_HALT_MAX,
};

/**
 *  enum cam_ife_csid_halt_mode - Specify the halt command type
 */
enum cam_ife_csid_halt_mode {
	CAM_CSID_HALT_MODE_INTERNAL,
	CAM_CSID_HALT_MODE_GLOBAL,
	CAM_CSID_HALT_MODE_MASTER,
	CAM_CSID_HALT_MODE_SLAVE,
	CAM_CSID_HALT_MODE_MAX,
};

/**
 * struct cam_ife_csid_hw_halt_args
 * @halt_mode : Applicable only for PATH resources
 *              0 Internal : The CSID responds to the HALT_CMD
 *              1 Global   : The CSID responds to the GLOBAL_HALT_CMD
 *              2 Master   : The CSID responds to the HALT_CMD
 *              3 Slave    : The CSID responds to the external halt command
 *                           and not the HALT_CMD register
 * @node_res : reource pointer array( ie cid or CSID)
 *
 */
struct cam_ife_csid_hw_halt_args {
	enum cam_ife_csid_halt_mode     halt_mode;
	struct cam_isp_resource_node   *node_res;
};

/**
 * struct cam_csid_hw_stop- stop all resources
 * @stop_cmd : Applicable only for PATH resources
 *             if stop command set to Halt immediately,driver will stop
 *             path immediately, manager need to reset the path after HI
 *             if stop command set to halt at frame boundary, driver will set
 *             halt at frame boundary and wait for frame boundary
 * @node_res :  reource pointer array( ie cid or CSID)
 * @num_res :   number of resources to be stopped
 *
 */
struct cam_csid_hw_stop_args {
	enum cam_ife_csid_halt_cmd                stop_cmd;
	struct cam_isp_resource_node            **node_res;
	uint32_t                                  num_res;
};

/**
 * struct cam_csid_hw_start_args - Relevant info to pass from ife_hw_mgr layer
 *                                 to start various resource nodes.
 *
 * @node_res:           Resource pointer array (cid or CSID)
 * @num_res:            Number of resources in node_res
 * @cdm_hw_idx:         Physical CDM in use together with these resources
 * @is_secure:          If these resources are run in secure session
 * @is_internal_start:  Start triggered internally for reset & recovery
 * @start_only:         start only, no init required
 * @is_drv_config_en:   If drv config is enabled
 * @is_per_port_start:  Indicates if start Hw is called on real start call or
 *                      on per port enabled start call.
 */
struct cam_csid_hw_start_args {
	struct cam_isp_resource_node            **node_res;
	uint32_t                                  num_res;
	uint32_t                                  cdm_hw_idx;
	bool                                      is_secure;
	bool                                      is_internal_start;
	bool                                      start_only;
	bool                                      is_drv_config_en;
	bool                                      is_per_port_start;
};


/**
 * enum cam_ife_csid_reset_type - Specify the reset type
 */
enum cam_ife_csid_reset_type {
	CAM_IFE_CSID_RESET_GLOBAL,
	CAM_IFE_CSID_RESET_PATH,
	CAM_IFE_CSID_RESET_MAX,
};

/**
 * struct cam_ife_csid_reset_cfg-  csid reset configuration
 * @ reset_type : Global reset or path reset
 * @res_node :   resource need to be reset
 *
 */
struct cam_csid_reset_cfg_args {
	enum cam_ife_csid_reset_type   reset_type;
	struct cam_isp_resource_node  *node_res;
};

/**
 * struct cam_csid_reset_out_of_sync_count_args
 * @res_node :   resource need to be reset
 *
 */
struct cam_csid_reset_out_of_sync_count_args {
	struct cam_isp_resource_node  *node_res;
};

/**
 * struct cam_csid_get_time_stamp_args-  time stamp capture arguments
 * @node_res            : resource to get the time stamp
 * @raw_boot_time       : Pointer to raw boot ts captured from top-half, if available
 * @time_stamp_val      : captured time stamp
 * @boot_timestamp      : boot time stamp
 * @prev_time_stamp_val : previous captured time stamp
 * @get_prev_timestamp  : flag to fetch previous captured time stamp from hardware
 * @get_curr_timestamp  : flag to skip CSID timestamp reg read if already read from top-half
 */
struct cam_csid_get_time_stamp_args {
	struct cam_isp_resource_node      *node_res;
	struct timespec64                 *raw_boot_time;
	uint64_t                           time_stamp_val;
	uint64_t                           boot_timestamp;
	uint64_t                           prev_time_stamp_val;
	bool                               get_prev_timestamp;
	bool                               get_curr_timestamp;
};

/**
 * cam_ife_csid_hw_init()
 *
 * @brief:               Initialize function for the CSID hardware
 *
 * @ife_csid_hw:         CSID hardware instance returned
 * @hw_idex:             CSID hardware instance id
 */
int cam_ife_csid_hw_init(struct cam_hw_intf **ife_csid_hw,
	uint32_t hw_idx);

/*
 * struct cam_ife_csid_clock_update_args:
 *
 * @clk_rate:                Clock rate requested
 */
struct cam_ife_csid_clock_update_args {
	uint64_t                           clk_rate;
};

/*
 * struct cam_ife_csid_qcfa_update_args:
 *
 * @res:                         Res node pointer
 * @qcfa_binning:                QCFA binning supported
 */
struct cam_ife_csid_qcfa_update_args {
	struct cam_isp_resource_node      *res;
	uint32_t                           qcfa_binning;
};

/*
 * struct cam_ife_sensor_dim_update_args:
 *
 * @res:                          Resource for which data is updated
 * @sensor_data:                  expected path configuration
 */
struct cam_ife_sensor_dimension_update_args {
	struct cam_isp_resource_node         *res;
	struct cam_isp_sensor_dimension       sensor_data;
};

/* struct cam_ife_csid_top_config_args:
 *
 * @input_core_type:              Input core type for CSID
 * @core_idx:                     Core idx for out core
 * @is_sfe_offline:               flag to indicate if sfe is offline
 * @is_sfe_fs:                    flag to indicate if use-case is Inline Bayer Fast Shutter (BRF)
 */
struct cam_ife_csid_top_config_args {
	uint32_t   input_core_type;
	uint32_t   core_idx;
	bool       is_sfe_offline;
	bool       is_sfe_fs;
};

/*
 * struct cam_ife_csid_dual_sync_args:
 *
 * @sync_mode:                Sync mode for dual csid master/slave
 * @dual_core_id:             Core idx for another core in case of dual isp
 *
 */
struct cam_ife_csid_dual_sync_args {
	enum cam_isp_hw_sync_mode   sync_mode;
	uint32_t                    dual_core_id;
};

/*
 * struct cam_isp_csid_reg_update_args:
 *
 * @cmd:              cmd buf update args
 * @node_res:         Node res pointer
 * @num_res:          Num of resources
 * @last_applied_mup: last applied MUP
 * @reg_write:        if set use AHB to config rup/aup
 * @mup_val:          MUP value if configured
 * @mup_en:           Flag if dynamic sensor switch is enabled
 */
struct cam_isp_csid_reg_update_args {
	struct cam_isp_hw_cmd_buf_update  cmd;
	struct cam_isp_resource_node     *res[CAM_IFE_PIX_PATH_RES_MAX];
	uint32_t                          num_res;
	uint32_t                          last_applied_mup;
	bool                              reg_write;
	uint32_t                          mup_val;
	uint32_t                          mup_en;
};

/*
 * struct cam_ife_csid_offline_cmd_update_args:
 *
 * @cmd:           cmd buf update args
 * @node_res:      Node res pointer for offline RDI
 */
struct cam_ife_csid_offline_cmd_update_args {
	struct cam_isp_hw_cmd_buf_update  cmd;
	struct cam_isp_resource_node     *res;
};

/*
 * struct cam_ife_csid_discard_frame_cfg_update:
 *
 * @reset_discard_cfg:  Set if discard config needs to be reset
 * @num_exposures:      Number of current exposures for sHDR
 */
struct cam_ife_csid_discard_frame_cfg_update {
	uint32_t num_exposures;
	bool     reset_discard_cfg;
};

/*
 * struct cam_ife_csid_ts_reg_addr:
 *
 * @curr0_ts_addr:      Reg addr of curr0_sof, input if set,
 *                      output otherwise
 * @curr1_ts_addr:      Reg addr of curr1_sof, input if set,
 *                      output otherwise
 * @res_id (input):     CSID path id, for get op
 * @get:                If call is to get addr
 */
struct cam_ife_csid_ts_reg_addr {
	void __iomem                     *curr0_ts_addr;
	void __iomem                     *curr1_ts_addr;
	uint32_t                          res_id;
	bool                              get_addr;
};

/*
 * struct cam_ife_csid_mup_update_args:
 *
 * @mup_val:  MUP for odd or even vc
 * @use_mup:  To indicate if CSID needs to consume this MUP
 */
struct cam_ife_csid_mup_update_args {
	uint32_t mup_val;
	bool use_mup;
};

/*
 * struct cam_ife_csid_mode_switch_update_args:
 *
 * @mup_args:         MUP related arguments
 * @exp_update_args:  Exposure update arguments
 */
struct cam_ife_csid_mode_switch_update_args {
	struct cam_ife_csid_mup_update_args mup_args;
	struct cam_ife_csid_discard_frame_cfg_update exp_update_args;
};

/*
 * struct cam_ife_csid_discard_init_frame_args:
 *
 * @num_frames: Num frames to discard
 * @res: Node res for this path
 */
struct cam_ife_csid_discard_init_frame_args {
	uint32_t                          num_frames;
	struct cam_isp_resource_node     *res;
};

/*
 * struct cam_ife_csid_debug_cfg_args:
 *
 * @csid_debug: CSID debug val
 * @csid_rx_capture_debug: CSID rx capture debug val
 * @csid_testbus_debug: CSID test bus val
 * @rx_capture_debug_set: CSID rx capture debug set;
 */
struct cam_ife_csid_debug_cfg_args {
	uint64_t                          csid_debug;
	uint32_t                          csid_rx_capture_debug;
	uint32_t                          csid_testbus_debug;
	bool                              rx_capture_debug_set;
};

/*
 * struct cam_ife_csid_drv_config_args:
 *
 * @is_init_config: Indicator for init config
 * @drv_en:         Indicator for camera DRV enable
 * @timeout_val:    Timeout value from SOF to trigger up vote, given in number of GC cycles
 * @path_idle_en:   Mask for paths to be considered for consolidated IDLE
 */
struct cam_ife_csid_drv_config_args {
	bool                               is_init_config;
	bool                               drv_en;
	uint32_t                           timeout_val;
	uint32_t                           path_idle_en;
};

#endif /* _CAM_CSID_HW_INTF_H_ */
