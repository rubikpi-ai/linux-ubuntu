/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __MSM_CAMERA_H
#define __MSM_CAMERA_H

/* CPAS path data types */
#define CAM_CPAS_PATH_DATA_IFE_START_OFFSET 0
#define CAM_CPAS_PATH_DATA_IFE_LINEAR (CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 0)
#define CAM_CPAS_PATH_DATA_IFE_VID (CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 1)
#define CAM_CPAS_PATH_DATA_IFE_DISP (CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 2)
#define CAM_CPAS_PATH_DATA_IFE_STATS (CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 3)
#define CAM_CPAS_PATH_DATA_IFE_RDI0 (CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 4)
#define CAM_CPAS_PATH_DATA_IFE_RDI1 (CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 5)
#define CAM_CPAS_PATH_DATA_IFE_RDI2 (CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 6)
#define CAM_CPAS_PATH_DATA_IFE_RDI3 (CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 7)
#define CAM_CPAS_PATH_DATA_IFE_PDAF (CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 8)
#define CAM_CPAS_PATH_DATA_IFE_PIXEL_RAW \
	(CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 9)
#define CAM_CPAS_PATH_DATA_IFE_MAX_OFFSET \
	(CAM_CPAS_PATH_DATA_IFE_START_OFFSET + 31)

#define CAM_CPAS_PATH_DATA_IPE_START_OFFSET 32
#define CAM_CPAS_PATH_DATA_IPE_RD_IN (CAM_CPAS_PATH_DATA_IPE_START_OFFSET + 0)
#define CAM_CPAS_PATH_DATA_IPE_RD_REF (CAM_CPAS_PATH_DATA_IPE_START_OFFSET + 1)
#define CAM_CPAS_PATH_DATA_IPE_WR_VID (CAM_CPAS_PATH_DATA_IPE_START_OFFSET + 2)
#define CAM_CPAS_PATH_DATA_IPE_WR_DISP (CAM_CPAS_PATH_DATA_IPE_START_OFFSET + 3)
#define CAM_CPAS_PATH_DATA_IPE_WR_REF (CAM_CPAS_PATH_DATA_IPE_START_OFFSET + 4)
#define CAM_CPAS_PATH_DATA_IPE_WR_APP (CAM_CPAS_PATH_DATA_IPE_START_OFFSET + 5)
#define CAM_CPAS_PATH_DATA_IPE_MAX_OFFSET \
	(CAM_CPAS_PATH_DATA_IPE_START_OFFSET + 31)

#define CAM_CPAS_PATH_DATA_OPE_START_OFFSET 64
#define CAM_CPAS_PATH_DATA_OPE_RD_IN (CAM_CPAS_PATH_DATA_OPE_START_OFFSET + 0)
#define CAM_CPAS_PATH_DATA_OPE_RD_REF (CAM_CPAS_PATH_DATA_OPE_START_OFFSET + 1)
#define CAM_CPAS_PATH_DATA_OPE_WR_VID (CAM_CPAS_PATH_DATA_OPE_START_OFFSET + 2)
#define CAM_CPAS_PATH_DATA_OPE_WR_DISP (CAM_CPAS_PATH_DATA_OPE_START_OFFSET + 3)
#define CAM_CPAS_PATH_DATA_OPE_WR_REF (CAM_CPAS_PATH_DATA_OPE_START_OFFSET + 4)
#define CAM_CPAS_PATH_DATA_OPE_MAX_OFFSET \
	(CAM_CPAS_PATH_DATA_OPE_START_OFFSET + 31)

#define CAM_CPAS_PATH_DATA_SFE_START_OFFSET 96
#define CAM_CPAS_PATH_DATA_SFE_NRDI    (CAM_CPAS_PATH_DATA_SFE_START_OFFSET + 0)
#define CAM_CPAS_PATH_DATA_SFE_RDI0    (CAM_CPAS_PATH_DATA_SFE_START_OFFSET + 1)
#define CAM_CPAS_PATH_DATA_SFE_RDI1    (CAM_CPAS_PATH_DATA_SFE_START_OFFSET + 2)
#define CAM_CPAS_PATH_DATA_SFE_RDI2    (CAM_CPAS_PATH_DATA_SFE_START_OFFSET + 3)
#define CAM_CPAS_PATH_DATA_SFE_RDI3    (CAM_CPAS_PATH_DATA_SFE_START_OFFSET + 4)
#define CAM_CPAS_PATH_DATA_SFE_RDI4    (CAM_CPAS_PATH_DATA_SFE_START_OFFSET + 5)
#define CAM_CPAS_PATH_DATA_SFE_STATS   (CAM_CPAS_PATH_DATA_SFE_START_OFFSET + 6)
#define CAM_CPAS_PATH_DATA_SFE_MAX_OFFSET \
	(CAM_CPAS_PATH_DATA_SFE_START_OFFSET + 31)

#define CAM_CPAS_PATH_DATA_CRE_START_OFFSET    (CAM_CPAS_PATH_DATA_SFE_MAX_OFFSET + 1)
#define CAM_CPAS_PATH_DATA_CRE_RD_IN           (CAM_CPAS_PATH_DATA_CRE_START_OFFSET + 0)
#define CAM_CPAS_PATH_DATA_CRE_WR_OUT          (CAM_CPAS_PATH_DATA_CRE_START_OFFSET + 1)
#define CAM_CPAS_PATH_DATA_CRE_MAX_OFFSET \
	(CAM_CPAS_PATH_DATA_CRE_START_OFFSET + 31)

#define CAM_CPAS_PATH_DATA_OFE_START_OFFSET (CAM_CPAS_PATH_DATA_CRE_MAX_OFFSET + 1)
#define CAM_CPAS_PATH_DATA_OFE_RD_EXT       (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 0)
#define CAM_CPAS_PATH_DATA_OFE_RD_INT_PDI   (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 1)
#define CAM_CPAS_PATH_DATA_OFE_RD_INT_HDR   (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 2)
#define CAM_CPAS_PATH_DATA_OFE_WR_VID       (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 3)
#define CAM_CPAS_PATH_DATA_OFE_WR_DISP      (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 4)
#define CAM_CPAS_PATH_DATA_OFE_WR_IR        (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 5)
#define CAM_CPAS_PATH_DATA_OFE_WR_HDR_LTM   (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 6)
#define CAM_CPAS_PATH_DATA_OFE_WR_DC4       (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 7)
#define CAM_CPAS_PATH_DATA_OFE_WR_AI        (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 8)
#define CAM_CPAS_PATH_DATA_OFE_WR_PDI       (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 9)
#define CAM_CPAS_PATH_DATA_OFE_WR_IDEALRAW  (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 10)
#define CAM_CPAS_PATH_DATA_OFE_WR_STATS     (CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 11)
#define CAM_CPAS_PATH_DATA_OFE_MAX_OFFSET \
	(CAM_CPAS_PATH_DATA_OFE_START_OFFSET + 31)

#define CAM_CPAS_PATH_DATA_CONSO_OFFSET 256
#define CAM_CPAS_PATH_DATA_ALL (CAM_CPAS_PATH_DATA_CONSO_OFFSET + 0)

/* IFE consolidated paths */
#define CAM_CPAS_PATH_DATA_IFE_LINEAR_PDAF (CAM_CPAS_PATH_DATA_CONSO_OFFSET + 1)
#define CAM_CPAS_PATH_DATA_IFE_UBWC_STATS (CAM_CPAS_PATH_DATA_CONSO_OFFSET + 2)
#define CAM_CPAS_PATH_DATA_IFE_PIXEL_ALL (CAM_CPAS_PATH_DATA_CONSO_OFFSET + 3)
#define CAM_CPAS_PATH_DATA_IFE_RDI_PIXEL_RAW \
	(CAM_CPAS_PATH_DATA_CONSO_OFFSET + 4)
#define CAM_CPAS_PATH_DATA_IFE_RDI_ALL (CAM_CPAS_PATH_DATA_CONSO_OFFSET + 5)
#define CAM_CPAS_PATH_DATA_IFE_UBWC (CAM_CPAS_PATH_DATA_CONSO_OFFSET + 6)
#define CAM_CPAS_PATH_DATA_IFE_LINEAR_STATS (CAM_CPAS_PATH_DATA_CONSO_OFFSET + 7)
#define CAM_CPAS_PATH_DATA_IFE_UBWC_LINEAR (CAM_CPAS_PATH_DATA_CONSO_OFFSET + 8)
#define CAM_CPAS_PATH_DATA_IFE_PDAF_LINEAR (CAM_CPAS_PATH_DATA_CONSO_OFFSET + 9)

/* IPE Consolidated paths */
#define CAM_CPAS_PATH_DATA_IPE_WR_VID_DISP (CAM_CPAS_PATH_DATA_CONSO_OFFSET + 1)

/* CPAS transaction types */
#define CAM_CPAS_TRANSACTION_READ 0
#define CAM_CPAS_TRANSACTION_WRITE 1

/* CPAS traffic merge types */
#define CAM_CPAS_TRAFFIC_MERGE_SUM 0
#define CAM_CPAS_TRAFFIC_MERGE_SUM_INTERLEAVE 1

/* Feature bit type */
#define CAM_CPAS_FEATURE_TYPE_DISABLE 0
#define CAM_CPAS_FEATURE_TYPE_ENABLE 1
#define CAM_CPAS_FEATURE_TYPE_VALUE 2

/* Feature support bit positions in feature fuse register*/
#define CAM_CPAS_QCFA_BINNING_ENABLE 0
#define CAM_CPAS_SECURE_CAMERA_ENABLE 1
#define CAM_CPAS_MF_HDR_ENABLE 2
#define CAM_CPAS_MP_LIMIT_FUSE 3
#define CAM_CPAS_ISP_FUSE      4
#define CAM_CPAS_ISP_PIX_FUSE  5
#define CAM_CPAS_ISP_LITE_FUSE 6
#define CAM_CPAS_CSIPHY_FUSE   7
#define CAM_CPAS_IPE_VID_OUT_8BPP_LIMIT_ENABLE 8
#define CAM_CPAS_SFE_FUSE 9
#define CAM_CPAS_SHDR_FUSE     7
#define CAM_CPAS_CSIPHY_FUSE   8
#define CAM_CPAS_RT_OT_FUSE    9
#define CAM_CPAS_FUSE_FEATURE_MAX 10
#define CAM_CPAS_CUSTOM_FUSE 10
#define CAM_CPAS_CAM_FUSE 11
#define CAM_CPAS_SHDR_FUSE 12
#define CAM_CPAS_FUSE_FEATURE_MAX 13

/* Flash type*/
#define CAM_FLASH_TYPE_PMIC 0
#define CAM_FLASH_TYPE_I2C  1
#define CAM_FLASH_TYPE_GPIO 2

/* CCI master */
#define CCI_MASTER_0 0
#define CCI_MASTER_1 1
#define CCI_MASTER_MAX 2

/* AON Camera IDs*/
#define AON_CAM1             0
#define AON_CAM2             1
#define MAX_AON_CAM          2
#define NOT_AON_CAM          255

/* Camera DRV enable masks */
#define CAM_DDR_DRV    0x1
#define CAM_CLK_DRV    0x2

/* Port index for BW voting */
#define CAM_CPAS_PORT_HLOS_DRV    0
#define CAM_CPAS_PORT_DRV_0       1
#define CAM_CPAS_PORT_DRV_1       2
#define CAM_CPAS_PORT_DRV_2       3
#define CAM_CPAS_PORT_DRV_DYN     32

/* Domain ID types */
#define CAM_CPAS_NON_SECURE_DOMAIN  0
#define CAM_CPAS_SECURE_DOMAIN      1

/* Debug bypass driver */
#define CAM_BYPASS_RGLTR      0x1
#define CAM_BYPASS_RGLTR_MODE 0x2
#define CAM_BYPASS_CLKS       0x4
#define CAM_BYPASS_CESTA      0x8
#define CAM_BYPASS_ICC        0x10

#endif
