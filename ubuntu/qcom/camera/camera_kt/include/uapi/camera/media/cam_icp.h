/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __UAPI_CAM_ICP_H__
#define __UAPI_CAM_ICP_H__

#include <media/cam_defs.h>
#include <media/cam_cpas.h>

/* icp, ipe, bps, cdm(ipe/bps) are used in querycap */
#define CAM_ICP_DEV_TYPE_A5      1
#define CAM_ICP_DEV_TYPE_IPE     2
#define CAM_ICP_DEV_TYPE_BPS     3
#define CAM_ICP_DEV_TYPE_IPE_CDM 4
#define CAM_ICP_DEV_TYPE_BPS_CDM 5
#define CAM_ICP_DEV_TYPE_MAX     5

/* definitions needed for icp aquire device */
#define CAM_ICP_RES_TYPE_BPS         1
#define CAM_ICP_RES_TYPE_IPE_RT      2
#define CAM_ICP_RES_TYPE_IPE         3
#define CAM_ICP_RES_TYPE_IPE_SEMI_RT 4
#define CAM_ICP_RES_TYPE_BPS_RT      5
#define CAM_ICP_RES_TYPE_BPS_SEMI_RT 6
#define CAM_ICP_RES_TYPE_MAX         7

/* packet opcode types */
#define CAM_ICP_OPCODE_IPE_UPDATE   0
#define CAM_ICP_OPCODE_BPS_UPDATE   1
#define CAM_ICP_OPCODE_IPE_SETTINGS 2
#define CAM_ICP_OPCODE_BPS_SETTINGS 3


/* IPE input port resource type */
#define CAM_ICP_IPE_INPUT_IMAGE_FULL            0x0
#define CAM_ICP_IPE_INPUT_IMAGE_DS4             0x1
#define CAM_ICP_IPE_INPUT_IMAGE_DS16            0x2
#define CAM_ICP_IPE_INPUT_IMAGE_DS64            0x3
#define CAM_ICP_IPE_INPUT_IMAGE_FULL_REF        0x4
#define CAM_ICP_IPE_INPUT_IMAGE_DS4_REF         0x5
#define CAM_ICP_IPE_INPUT_IMAGE_DS16_REF        0x6
#define CAM_ICP_IPE_INPUT_IMAGE_DS64_REF        0x7

/* IPE output port resource type */
#define CAM_ICP_IPE_OUTPUT_IMAGE_DISPLAY        0x8
#define CAM_ICP_IPE_OUTPUT_IMAGE_VIDEO          0x9
#define CAM_ICP_IPE_OUTPUT_IMAGE_FULL_REF       0xA
#define CAM_ICP_IPE_OUTPUT_IMAGE_DS4_REF        0xB
#define CAM_ICP_IPE_OUTPUT_IMAGE_DS16_REF       0xC
#define CAM_ICP_IPE_OUTPUT_IMAGE_DS64_REF       0xD

#define CAM_ICP_IPE_IMAGE_MAX                   0xE

/* BPS input port resource type */
#define CAM_ICP_BPS_INPUT_IMAGE                 0x0

/* BPS output port resource type */
#define CAM_ICP_BPS_OUTPUT_IMAGE_FULL           0x1
#define CAM_ICP_BPS_OUTPUT_IMAGE_DS4            0x2
#define CAM_ICP_BPS_OUTPUT_IMAGE_DS16           0x3
#define CAM_ICP_BPS_OUTPUT_IMAGE_DS64           0x4
#define CAM_ICP_BPS_OUTPUT_IMAGE_STATS_BG       0x5
#define CAM_ICP_BPS_OUTPUT_IMAGE_STATS_BHIST    0x6
#define CAM_ICP_BPS_OUTPUT_IMAGE_REG1           0x7
#define CAM_ICP_BPS_OUTPUT_IMAGE_REG2           0x8

#define CAM_ICP_BPS_IO_IMAGES_MAX               0x9

/* Command meta types */
#define CAM_ICP_CMD_META_GENERIC_BLOB           0x1

/* Generic blob types */
#define CAM_ICP_CMD_GENERIC_BLOB_CLK            0x1
#define CAM_ICP_CMD_GENERIC_BLOB_CFG_IO         0x2
#define CAM_ICP_CMD_GENERIC_BLOB_FW_MEM_MAP     0x3
#define CAM_ICP_CMD_GENERIC_BLOB_FW_MEM_UNMAP   0x4
#define CAM_ICP_CMD_GENERIC_BLOB_CLK_V2         0x5

/**
 * struct cam_icp_clk_bw_request_v2
 *
 * @budget_ns: Time required to process frame
 * @frame_cycles: Frame cycles needed to process the frame
 * @rt_flag: Flag to indicate real time stream
 * @reserved: For memory alignment
 * @num_paths: Number of axi paths in bw request
 * @axi_path: Per path vote info for IPE/BPS
 */
struct cam_icp_clk_bw_request_v2 {
	__u64                           budget_ns;
	__u32                           frame_cycles;
	__u32                           rt_flag;
	__u32                           reserved;
	__u32                           num_paths;
    union {
        struct cam_axi_per_path_bw_vote axi_path[1];
        __DECLARE_FLEX_ARRAY(struct cam_axi_per_path_bw_vote, axi_path_flex);
    };
};

/**
 * struct cam_icp_clk_bw_request
 *
 * @budget_ns: Time required to process frame
 * @frame_cycles: Frame cycles needed to process the frame
 * @rt_flag: Flag to indicate real time stream
 * @uncompressed_bw: Bandwidth required to process frame
 * @compressed_bw: Compressed bandwidth to process frame
 */
struct cam_icp_clk_bw_request {
	__u64 budget_ns;
	__u32 frame_cycles;
	__u32 rt_flag;
	__u64 uncompressed_bw;
	__u64 compressed_bw;
};

/**
 * struct cam_icp_dev_ver - Device information for particular hw type
 *
 * This is used to get device version info of
 * ICP, IPE, BPS and CDM related IPE and BPS from firmware
 * and use this info in CAM_QUERY_CAP IOCTL
 *
 * @dev_type: hardware type for the cap info(icp, ipe, bps, cdm(ipe/bps))
 * @reserved: reserved field
 * @hw_ver: major, minor and incr values of a device version
 */
struct cam_icp_dev_ver {
	__u32                 dev_type;
	__u32                 reserved;
	struct cam_hw_version hw_ver;
};

/**
 * struct cam_icp_ver - ICP version info
 *
 * This strcuture is used for fw and api version
 * this is used to get firmware version and api version from firmware
 * and use this info in CAM_QUERY_CAP IOCTL
 *
 * @major: FW version major
 * @minor: FW version minor
 * @revision: FW version increment
 */
struct cam_icp_ver {
	__u32 major;
	__u32 minor;
	__u32 revision;
	__u32 reserved;
};

/**
 * struct cam_icp_query_cap_cmd - ICP query device capability payload
 *
 * @dev_iommu_handle: icp iommu handles for secure/non secure modes
 * @cdm_iommu_handle: iommu handles for secure/non secure modes
 * @fw_version: firmware version info
 * @api_version: api version info
 * @num_ipe: number of ipes
 * @num_bps: number of bps
 * @dev_ver: returned device capability array
 */
struct cam_icp_query_cap_cmd {
	struct cam_iommu_handle dev_iommu_handle;
	struct cam_iommu_handle cdm_iommu_handle;
	struct cam_icp_ver      fw_version;
	struct cam_icp_ver      api_version;
	__u32                   num_ipe;
	__u32                   num_bps;
	struct cam_icp_dev_ver  dev_ver[CAM_ICP_DEV_TYPE_MAX];
};

/**
 * struct cam_icp_res_info - ICP output resource info
 *
 * @format: format of the resource
 * @width:  width in pixels
 * @height: height in lines
 * @fps:  fps
 */
struct cam_icp_res_info {
	__u32 format;
	__u32 width;
	__u32 height;
	__u32 fps;
};

/**
 * struct cam_icp_acquire_dev_info - An ICP device info
 *
 * @scratch_mem_size: Output param - size of scratch memory
 * @dev_type: device type (IPE_RT/IPE_NON_RT/BPS)
 * @io_config_cmd_size: size of IO config command
 * @io_config_cmd_handle: IO config command for each acquire
 * @secure_mode: camera mode (secure/non secure)
 * @chain_info: chaining info of FW device handles
 * @in_res: resource info used for clock and bandwidth calculation
 * @num_out_res: number of output resources
 * @out_res: output resource
 */
struct cam_icp_acquire_dev_info {
	__u32                   scratch_mem_size;
	__u32                   dev_type;
	__u32                   io_config_cmd_size;
	__s32                   io_config_cmd_handle;
	__u32                   secure_mode;
	__s32                   chain_info;
	struct cam_icp_res_info in_res;
	__u32                   num_out_res;
    union {
        struct cam_icp_res_info out_res[1];
        __DECLARE_FLEX_ARRAY(struct cam_icp_res_info, out_res_flex);
    };
} __attribute__((__packed__));

#endif /* __UAPI_CAM_ICP_H__ */
