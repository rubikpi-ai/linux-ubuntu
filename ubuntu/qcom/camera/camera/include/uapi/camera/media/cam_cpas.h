/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __UAPI_CAM_CPAS_H__
#define __UAPI_CAM_CPAS_H__

#include <media/cam_defs.h>

#define CAM_FAMILY_CAMERA_SS     1
#define CAM_FAMILY_CPAS_SS       2

/* AXI BW Voting Version */
#define CAM_AXI_BW_VOTING_V2                2

/* AXI BW Voting Transaction Type */
#define CAM_AXI_TRANSACTION_READ            0
#define CAM_AXI_TRANSACTION_WRITE           1

/* AXI BW Voting Path Data Type */
#define CAM_AXI_PATH_DATA_IFE_START_OFFSET 0
#define CAM_AXI_PATH_DATA_IFE_LINEAR    (CAM_AXI_PATH_DATA_IFE_START_OFFSET + 0)
#define CAM_AXI_PATH_DATA_IFE_VID       (CAM_AXI_PATH_DATA_IFE_START_OFFSET + 1)
#define CAM_AXI_PATH_DATA_IFE_DISP      (CAM_AXI_PATH_DATA_IFE_START_OFFSET + 2)
#define CAM_AXI_PATH_DATA_IFE_STATS     (CAM_AXI_PATH_DATA_IFE_START_OFFSET + 3)
#define CAM_AXI_PATH_DATA_IFE_RDI0      (CAM_AXI_PATH_DATA_IFE_START_OFFSET + 4)
#define CAM_AXI_PATH_DATA_IFE_RDI1      (CAM_AXI_PATH_DATA_IFE_START_OFFSET + 5)
#define CAM_AXI_PATH_DATA_IFE_RDI2      (CAM_AXI_PATH_DATA_IFE_START_OFFSET + 6)
#define CAM_AXI_PATH_DATA_IFE_RDI3      (CAM_AXI_PATH_DATA_IFE_START_OFFSET + 7)
#define CAM_AXI_PATH_DATA_IFE_PDAF      (CAM_AXI_PATH_DATA_IFE_START_OFFSET + 8)
#define CAM_AXI_PATH_DATA_IFE_PIXEL_RAW \
	(CAM_AXI_PATH_DATA_IFE_START_OFFSET + 9)
#define CAM_AXI_PATH_DATA_IFE_MAX_OFFSET \
	(CAM_AXI_PATH_DATA_IFE_START_OFFSET + 31)

#define CAM_AXI_PATH_DATA_IPE_START_OFFSET 32
#define CAM_AXI_PATH_DATA_IPE_RD_IN     (CAM_AXI_PATH_DATA_IPE_START_OFFSET + 0)
#define CAM_AXI_PATH_DATA_IPE_RD_REF    (CAM_AXI_PATH_DATA_IPE_START_OFFSET + 1)
#define CAM_AXI_PATH_DATA_IPE_WR_VID    (CAM_AXI_PATH_DATA_IPE_START_OFFSET + 2)
#define CAM_AXI_PATH_DATA_IPE_WR_DISP   (CAM_AXI_PATH_DATA_IPE_START_OFFSET + 3)
#define CAM_AXI_PATH_DATA_IPE_WR_REF    (CAM_AXI_PATH_DATA_IPE_START_OFFSET + 4)
#define CAM_AXI_PATH_DATA_IPE_WR_APP    (CAM_AXI_PATH_DATA_IPE_START_OFFSET + 5)
#define CAM_AXI_PATH_DATA_IPE_MAX_OFFSET \
	(CAM_AXI_PATH_DATA_IPE_START_OFFSET + 31)

#define CAM_AXI_PATH_DATA_OPE_START_OFFSET 64
#define CAM_AXI_PATH_DATA_OPE_RD_IN     (CAM_AXI_PATH_DATA_OPE_START_OFFSET + 0)
#define CAM_AXI_PATH_DATA_OPE_RD_REF    (CAM_AXI_PATH_DATA_OPE_START_OFFSET + 1)
#define CAM_AXI_PATH_DATA_OPE_WR_VID    (CAM_AXI_PATH_DATA_OPE_START_OFFSET + 2)
#define CAM_AXI_PATH_DATA_OPE_WR_DISP   (CAM_AXI_PATH_DATA_OPE_START_OFFSET + 3)
#define CAM_AXI_PATH_DATA_OPE_WR_REF    (CAM_AXI_PATH_DATA_OPE_START_OFFSET + 4)
#define CAM_AXI_PATH_DATA_OPE_MAX_OFFSET \
	(CAM_AXI_PATH_DATA_OPE_START_OFFSET + 31)

#define CAM_AXI_PATH_DATA_SFE_START_OFFSET 96
#define CAM_AXI_PATH_DATA_SFE_NRDI      (CAM_AXI_PATH_DATA_SFE_START_OFFSET + 0)
#define CAM_AXI_PATH_DATA_SFE_RDI0      (CAM_AXI_PATH_DATA_SFE_START_OFFSET + 1)
#define CAM_AXI_PATH_DATA_SFE_RDI1      (CAM_AXI_PATH_DATA_SFE_START_OFFSET + 2)
#define CAM_AXI_PATH_DATA_SFE_RDI2      (CAM_AXI_PATH_DATA_SFE_START_OFFSET + 3)
#define CAM_AXI_PATH_DATA_SFE_RDI3      (CAM_AXI_PATH_DATA_SFE_START_OFFSET + 4)
#define CAM_AXI_PATH_DATA_SFE_RDI4      (CAM_AXI_PATH_DATA_SFE_START_OFFSET + 5)
#define CAM_AXI_PATH_DATA_SFE_STATS     (CAM_AXI_PATH_DATA_SFE_START_OFFSET + 6)
#define CAM_AXI_PATH_DATA_SFE_MAX_OFFSET \
	(CAM_AXI_PATH_DATA_SFE_START_OFFSET + 31)

#define CAM_AXI_PATH_DATA_CRE_START_OFFSET    (CAM_AXI_PATH_DATA_SFE_MAX_OFFSET + 1)
#define CAM_AXI_PATH_DATA_CRE_RD_IN           (CAM_AXI_PATH_DATA_CRE_START_OFFSET + 0)
#define CAM_AXI_PATH_DATA_CRE_WR_OUT          (CAM_AXI_PATH_DATA_CRE_START_OFFSET + 1)
#define CAM_AXI_PATH_DATA_CRE_MAX_OFFSET \
	(CAM_AXI_PATH_DATA_CRE_START_OFFSET + 31)

#define CAM_AXI_PATH_DATA_OFE_START_OFFSET    (CAM_AXI_PATH_DATA_CRE_MAX_OFFSET + 1)
#define CAM_AXI_PATH_DATA_OFE_RD_EXT          (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 0)
#define CAM_AXI_PATH_DATA_OFE_RD_INT_PDI      (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 1)
#define CAM_AXI_PATH_DATA_OFE_RD_INT_HDR      (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 2)
#define CAM_AXI_PATH_DATA_OFE_WR_VID          (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 3)
#define CAM_AXI_PATH_DATA_OFE_WR_DISP         (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 4)
#define CAM_AXI_PATH_DATA_OFE_WR_IR           (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 5)
#define CAM_AXI_PATH_DATA_OFE_WR_HDR_LTM      (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 6)
#define CAM_AXI_PATH_DATA_OFE_WR_DC4          (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 7)
#define CAM_AXI_PATH_DATA_OFE_WR_AI           (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 8)
#define CAM_AXI_PATH_DATA_OFE_WR_PDI          (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 9)
#define CAM_AXI_PATH_DATA_OFE_WR_IDEALRAW     (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 10)
#define CAM_AXI_PATH_DATA_OFE_WR_STATS        (CAM_AXI_PATH_DATA_OFE_START_OFFSET + 11)
#define CAM_AXI_PATH_DATA_OFE_MAX_OFFSET \
	(CAM_AXI_PATH_DATA_OFE_START_OFFSET + 31)

#define CAM_AXI_PATH_DATA_ALL  256
#define CAM_CPAS_FUSES_MAX     32
#define CAM_CPAS_DOMAIN_ID_MAX 5

/* DRV Vote level */
#define CAM_CPAS_VOTE_LEVEL_HIGH   1
#define CAM_CPAS_VOTE_LEVEL_LOW    2

/* Domain id types */
#define CAM_CPAS_NON_SECURE_DOMAIN  0
#define CAM_CPAS_SECURE_DOMAIN      1

/* sysfs entry of camera subparts info */
#define CAM_SYSFS_SUBPARTS_INFO_FILENAME "subparts_info"

/**
 * struct cam_cpas_fuse_value - CPAS fuse value
 *
 * @fuse_id     : Camera fuse identification
 * @fuse_val    : Camera Fuse Value
 */
struct cam_cpas_fuse_value {
	__u32 fuse_id;
	__u32 fuse_val;
};

/**
 * struct cam_cpas_fuse_info - CPAS fuse info
 *
 * @num_fuses     : Number of fuses
 * @fuse_val      : Array of different fuse info.
 */
struct cam_cpas_fuse_info {
	__u32  num_fuses;
	struct cam_cpas_fuse_value fuse_val[CAM_CPAS_FUSES_MAX];
};

/**
 * struct cam_cpas_domain_id_pairing - CPAS domain id mapping
 *
 * @domain_type    : Domain type
 * @mapping_id     : ID of domain type
 */
struct cam_cpas_domain_id_pairing {
	__u32 domain_type;
	__u32 mapping_id;
	__u32 num_valid_params;
	__u32 valid_param_mask;
	__u32 params[4];
};

/**
 * struct cam_cpas_domain_id_caps - CPAS domain id info
 *
 * @is_supported      : If domain id is supported on target
 * @num_mapping       : Number of domain id types supported, if any
 * @entries           : Stores mapping between domain type and its ID
 * @num_valid_params  : Number of valid params
 * @valid_param_mask  : Valid param mask
 * @params            : These fields are reserved for future extensions
 *                      to this structure.
 */
struct cam_cpas_domain_id_caps {
	__u32  is_supported;
	__u32  num_mapping;
	struct cam_cpas_domain_id_pairing entries[CAM_CPAS_DOMAIN_ID_MAX];
	__u32  num_valid_params;
	__u32  valid_param_mask;
	__u32  params[6];
};


/**
 * struct cam_cpas_query_cap - CPAS query device capability payload
 *
 * @camera_family     : Camera family type
 * @reserved          : Reserved field for alignment
 * @camera_version    : Camera platform version
 * @cpas_version      : Camera CPAS version within camera platform
 *
 */
struct cam_cpas_query_cap {
	__u32                 camera_family;
	__u32                 reserved;
	struct cam_hw_version camera_version;
	struct cam_hw_version cpas_version;
};

/**
 * struct cam_cpas_query_cap - CPAS query device capability payload
 *
 * @camera_family     : Camera family type
 * @reserved          : Reserved field for alignment
 * @camera_version    : Camera platform version
 * @cpas_version      : Camera CPAS version within camera platform
 * @fuse_info         : Camera fuse info
 *
 */
struct cam_cpas_query_cap_v2 {
	__u32                     camera_family;
	__u32                     reserved;
	struct cam_hw_version     camera_version;
	struct cam_hw_version     cpas_version;
	struct cam_cpas_fuse_info fuse_info;
};

/**
 * struct cam_cpas_query_cap - CPAS query device capability payload
 *
 * @version           : Struct version
 * @camera_family     : Camera family type
 * @camera_caps       : Camera capability
 * @camera_version    : Camera platform version
 * @cpas_version      : Camera CPAS version within camera platform
 * @fuse_info         : Camera fuse info
 * @domain_id_info    : Domain id info
 * @num_valid_params  : Number of valid params
 * @valid_param_mask  : Valid param mask
 * @params            : Reserved fields to make this query cap
 *                      extendable in the future
 */
struct cam_cpas_query_cap_v3 {
	__u32                             version;
	__u32                             camera_family;
	__u32                             camera_caps;
	struct cam_hw_version             camera_version;
	struct cam_hw_version             cpas_version;
	struct cam_cpas_fuse_info         fuse_info;
	struct cam_cpas_domain_id_caps    domain_id_info;
	__u32                             num_valid_params;
	__u32                             valid_param_mask;
	__u32                             params[10];
};

/**
 * struct cam_axi_per_path_bw_vote_v2 - Per path bandwidth vote information
 *
 * @usage_data:              client usage data (left/right/rdi)
 * @transac_type:            Transaction type on the path (read/write)
 * @path_data_type:          Path for which vote is given (video, display, rdi)
 * @vote_level:              Vote level for this path
 * @camnoc_bw:               CAMNOC bw for this path
 * @mnoc_ab_bw:              MNOC AB bw for this path
 * @mnoc_ib_bw:              MNOC IB bw for this path
 * @ddr_ab_bw:               DDR AB bw for this path
 * @ddr_ib_bw:               DDR IB bw for this path
 * @num_valid_params:        Number of valid params
 * @valid_param_mask:        Valid param mask
 * @params:                  params
 */
struct cam_axi_per_path_bw_vote_v2 {
	__u32                      usage_data;
	__u32                      transac_type;
	__u32                      path_data_type;
	__u32                      vote_level;
	__u64                      camnoc_bw;
	__u64                      mnoc_ab_bw;
	__u64                      mnoc_ib_bw;
	__u64                      ddr_ab_bw;
	__u64                      ddr_ib_bw;
	__u32                      num_valid_params;
	__u32                      valid_param_mask;
	__u32                      params[4];
};

/**
 * struct cam_axi_per_path_bw_vote - Per path bandwidth vote information
 *
 * @usage_data               client usage data (left/right/rdi)
 * @transac_type             Transaction type on the path (read/write)
 * @path_data_type           Path for which vote is given (video, display, rdi)
 * @reserved                 Reserved for alignment
 * @camnoc_bw                CAMNOC bw for this path
 * @mnoc_ab_bw               MNOC AB bw for this path
 * @mnoc_ib_bw               MNOC IB bw for this path
 * @ddr_ab_bw                DDR AB bw for this path
 * @ddr_ib_bw                DDR IB bw for this path
 */
struct cam_axi_per_path_bw_vote {
	__u32                      usage_data;
	__u32                      transac_type;
	__u32                      path_data_type;
	__u32                      reserved;
	__u64                      camnoc_bw;
	__u64                      mnoc_ab_bw;
	__u64                      mnoc_ib_bw;
	__u64                      ddr_ab_bw;
	__u64                      ddr_ib_bw;
};

#define CAM_CPAS_CUSTOM_CMD_FD_PORT_CFG 0

/**
 * struct cam_cpas_fd_port_config : CPAS custom cmd struct for updating FD
 *                                  port config
 *
 * @is_secure                Security mode of the FD port
 * @reserved                 Reserved for alignment
 */
struct cam_cpas_fd_port_config {
	__u32 is_secure;
	__u32 reserved;
};

#endif /* __UAPI_CAM_CPAS_H__ */
