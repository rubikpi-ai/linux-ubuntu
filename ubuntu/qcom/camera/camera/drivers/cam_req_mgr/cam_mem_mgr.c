// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/version.h>
#include <linux/debugfs.h>
#if IS_REACHABLE(CONFIG_DMABUF_HEAPS)
#include <linux/mem-buf.h>
#include <soc/qcom/secure_buffer.h>
#endif

#include "cam_compat.h"
#include "cam_req_mgr_util.h"
#include "cam_mem_mgr.h"
#include "cam_smmu_api.h"
#include "cam_debug_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_presil_hw_access.h"

#define CAM_MEM_SHARED_BUFFER_PAD_4K (4 * 1024)

static struct cam_mem_table tbl;
static atomic_t cam_mem_mgr_state = ATOMIC_INIT(CAM_MEM_MGR_UNINITIALIZED);

/* Number of words for dumping req state info */
#define CAM_MEM_MGR_DUMP_BUF_NUM_WORDS  29

/* cam_mem_mgr_debug - global struct to keep track of debug settings for mem mgr
 *
 * @dentry                  : Directory entry to the mem mgr root folder
 * @alloc_profile_enable    : Whether to enable alloc profiling
 * @override_cpu_access_dir : Override cpu access direction to BIDIRECTIONAL
 */
static struct {
	struct dentry *dentry;
	bool alloc_profile_enable;
	bool override_cpu_access_dir;
} g_cam_mem_mgr_debug;

#if IS_REACHABLE(CONFIG_DMABUF_HEAPS)
static void cam_mem_mgr_put_dma_heaps(void);
static int cam_mem_mgr_get_dma_heaps(void);
#endif

#ifdef CONFIG_CAM_PRESIL
static inline void cam_mem_mgr_reset_presil_params(int idx)
{
	tbl.bufq[idx].presil_params.fd_for_umd_daemon = -1;
	tbl.bufq[idx].presil_params.refcount = 0;
}
#else
static inline void cam_mem_mgr_reset_presil_params(int idx)
{
	return;
}
#endif

static unsigned long cam_mem_mgr_mini_dump_cb(void *dst, unsigned long len,
	void *priv_data)
{
	struct cam_mem_table_mini_dump      *md;

	if (!dst) {
		CAM_ERR(CAM_MEM, "Invalid  params");
		return 0;
	}

	if (len < sizeof(*md)) {
		CAM_ERR(CAM_MEM, "Insufficient length %u", len);
		return 0;
	}

	md = (struct cam_mem_table_mini_dump *)dst;
	memcpy(md->bufq, tbl.bufq, CAM_MEM_BUFQ_MAX * sizeof(struct cam_mem_buf_queue));
	md->dbg_buf_idx = tbl.dbg_buf_idx;
	md->alloc_profile_enable = g_cam_mem_mgr_debug.alloc_profile_enable;
	md->force_cache_allocs = tbl.force_cache_allocs;
	md->need_shared_buffer_padding = tbl.need_shared_buffer_padding;
	return sizeof(*md);
}

static void cam_mem_mgr_print_tbl(void)
{
	int i;
	uint64_t ms, hrs, min, sec;
	struct timespec64 current_ts;

	CAM_GET_TIMESTAMP(current_ts);
	CAM_CONVERT_TIMESTAMP_FORMAT(current_ts, hrs, min, sec, ms);
	CAM_INFO(CAM_MEM, "***%llu:%llu:%llu:%llu Mem mgr table dump***",
		hrs, min, sec, ms);
	for (i = 1; i < CAM_MEM_BUFQ_MAX; i++) {
		CAM_CONVERT_TIMESTAMP_FORMAT((tbl.bufq[i].timestamp), hrs, min, sec, ms);
		CAM_INFO(CAM_MEM,
			"%llu:%llu:%llu:%llu idx %d fd %d i_ino %lu size %llu active %d buf_handle %d krefCount %d urefCount %d buf_name %s",
			hrs, min, sec, ms, i, tbl.bufq[i].fd, tbl.bufq[i].i_ino,
			tbl.bufq[i].len, tbl.bufq[i].active, tbl.bufq[i].buf_handle,
			kref_read(&tbl.bufq[i].krefcount), kref_read(&tbl.bufq[i].urefcount),
			tbl.bufq[i].buf_name);
	}

}
/**
 * For faster lookups, maintaining same indexing as SMMU
 * for saving iova for a given buffer for a given context
 * bank
 *
 * Buffer X : [iova_1, 0x0, iova_3, ...]
 * Here iova_1 is for device_1, no iova available for device_2,
 * iova_3 for device_3 and so on
 */
static inline bool cam_mem_mgr_get_hwva_entry_idx(
	int32_t mem_handle, int32_t *entry_idx)
{
	int entry;

	entry = GET_SMMU_TABLE_IDX(mem_handle);
	if (unlikely((entry < 0) || (entry >= tbl.max_hdls_supported))) {
		CAM_ERR(CAM_MEM,
			"Invalid mem_hdl: 0x%x, failed to lookup", mem_handle);
		return false;
	}

	*entry_idx = entry;
	return true;
}

static int cam_mem_util_get_dma_dir(uint32_t flags)
{
	int rc = -EINVAL;

	if (flags & CAM_MEM_FLAG_HW_READ_ONLY)
		rc = DMA_TO_DEVICE;
	else if (flags & CAM_MEM_FLAG_HW_WRITE_ONLY)
		rc = DMA_FROM_DEVICE;
	else if (flags & CAM_MEM_FLAG_HW_READ_WRITE)
		rc = DMA_BIDIRECTIONAL;
	else if (flags & CAM_MEM_FLAG_PROTECTED_MODE)
		rc = DMA_BIDIRECTIONAL;

	return rc;
}

static int cam_mem_util_map_cpu_va(struct dma_buf *dmabuf, uintptr_t *vaddr, size_t *len)
{
	int rc = 0;

	/*
	 * dma_buf_begin_cpu_access() and dma_buf_end_cpu_access()
	 * need to be called in pair to avoid stability issue.
	 */
	rc = dma_buf_begin_cpu_access(dmabuf, DMA_BIDIRECTIONAL);
	if (rc) {
		CAM_ERR(CAM_MEM, "dma begin access failed rc=%d", rc);
		return rc;
	}

	rc = cam_compat_util_get_dmabuf_va(dmabuf, vaddr);
	if (rc) {
		CAM_ERR(CAM_MEM, "kernel vmap failed: rc = %d", rc);
		*len = 0;
		dma_buf_end_cpu_access(dmabuf, DMA_BIDIRECTIONAL);
	}
	else {
		*len = dmabuf->size;
		CAM_DBG(CAM_MEM, "vaddr = %llu, len = %zu", *vaddr, *len);
	}

	return rc;
}

static int cam_mem_util_unmap_cpu_va(struct dma_buf *dmabuf,
	uint64_t vaddr)
{
	int rc = 0;

	if (!dmabuf || !vaddr) {
		CAM_ERR(CAM_MEM, "Invalid input args %pK %llX", dmabuf, vaddr);
		return -EINVAL;
	}

	cam_compat_util_put_dmabuf_va(dmabuf, (void *)vaddr);

	/*
	 * dma_buf_begin_cpu_access() and
	 * dma_buf_end_cpu_access() need to be called in pair
	 * to avoid stability issue.
	 */
	rc = dma_buf_end_cpu_access(dmabuf, DMA_BIDIRECTIONAL);
	if (rc) {
		CAM_ERR(CAM_MEM, "Failed in end cpu access, dmabuf=%pK",
			dmabuf);
		return rc;
	}

	return rc;
}

static int cam_mem_mgr_create_debug_fs(void)
{
	int rc = 0;
	struct dentry *dbgfileptr = NULL;

	if (!cam_debugfs_available() || g_cam_mem_mgr_debug.dentry)
		return 0;

	rc = cam_debugfs_create_subdir("memmgr", &dbgfileptr);
	if (rc) {
		CAM_ERR(CAM_MEM, "DebugFS could not create directory!");
		rc = -ENOENT;
		goto end;
	}

	g_cam_mem_mgr_debug.dentry = dbgfileptr;

	debugfs_create_bool("alloc_profile_enable", 0644, g_cam_mem_mgr_debug.dentry,
		&g_cam_mem_mgr_debug.alloc_profile_enable);

	debugfs_create_bool("override_cpu_access_dir", 0644, g_cam_mem_mgr_debug.dentry,
		&g_cam_mem_mgr_debug.override_cpu_access_dir);
end:
	return rc;
}

int cam_mem_mgr_init(void)
{
	int i;
	int bitmap_size;
	int rc = 0;

	if (atomic_read(&cam_mem_mgr_state))
		return 0;

	memset(tbl.bufq, 0, sizeof(tbl.bufq));

	if (cam_smmu_need_force_alloc_cached(&tbl.force_cache_allocs)) {
		CAM_ERR(CAM_MEM, "Error in getting force cache alloc flag");
		return -EINVAL;
	}

	tbl.need_shared_buffer_padding = cam_smmu_need_shared_buffer_padding();

#if IS_REACHABLE(CONFIG_DMABUF_HEAPS)
	rc = cam_mem_mgr_get_dma_heaps();
	if (rc) {
		CAM_ERR(CAM_MEM, "Failed in getting dma heaps rc=%d", rc);
		return rc;
	}
#endif
	bitmap_size = BITS_TO_LONGS(CAM_MEM_BUFQ_MAX) * sizeof(long);
	tbl.bitmap = kzalloc(bitmap_size, GFP_KERNEL);
	if (!tbl.bitmap) {
		rc = -ENOMEM;
		goto put_heaps;
	}

	tbl.bits = bitmap_size * BITS_PER_BYTE;
	bitmap_zero(tbl.bitmap, tbl.bits);
	/* We need to reserve slot 0 because 0 is invalid */
	set_bit(0, tbl.bitmap);

	for (i = 1; i < CAM_MEM_BUFQ_MAX; i++) {
		tbl.bufq[i].fd = -1;
		tbl.bufq[i].buf_handle = -1;
		cam_mem_mgr_reset_presil_params(i);
		mutex_init(&tbl.bufq[i].q_lock);
		spin_lock_init(&tbl.bufq[i].idx_lock);
	}
	mutex_init(&tbl.m_lock);

	atomic_set(&cam_mem_mgr_state, CAM_MEM_MGR_INITIALIZED);

	cam_mem_mgr_create_debug_fs();
	cam_common_register_mini_dump_cb(cam_mem_mgr_mini_dump_cb,
		"cam_mem", NULL);

	rc = cam_smmu_driver_init(&tbl.csf_version, &tbl.max_hdls_supported);
	if (rc)
		goto clean_bitmap_and_mutex;

	if (!tbl.max_hdls_supported) {
		CAM_ERR(CAM_MEM, "Invalid number of supported handles");
		rc = -EINVAL;
		goto clean_bitmap_and_mutex;
	}

	tbl.max_hdls_info_size = sizeof(struct cam_mem_buf_hw_hdl_info) *
		tbl.max_hdls_supported;

	/* Index 0 is reserved as invalid slot */
	for (i = 1; i < CAM_MEM_BUFQ_MAX; i++) {
		tbl.bufq[i].hdls_info = kzalloc(tbl.max_hdls_info_size, GFP_KERNEL);

		if (!tbl.bufq[i].hdls_info) {
			CAM_ERR(CAM_MEM, "Failed to allocate hdls array queue idx: %d", i);
			rc = -ENOMEM;
			goto free_hdls_info;
		}
	}

	return 0;

free_hdls_info:
	for (--i; i > 0; i--) {
		kfree(tbl.bufq[i].hdls_info);
		tbl.bufq[i].hdls_info = NULL;
	}

clean_bitmap_and_mutex:
	kfree(tbl.bitmap);
	tbl.bitmap = NULL;
	for (i = 1; i < CAM_MEM_BUFQ_MAX; i++)
		mutex_destroy(&tbl.bufq[i].q_lock);
	mutex_destroy(&tbl.m_lock);
	atomic_set(&cam_mem_mgr_state, CAM_MEM_MGR_UNINITIALIZED);
put_heaps:
#if IS_REACHABLE(CONFIG_DMABUF_HEAPS)
	cam_mem_mgr_put_dma_heaps();
#endif
	return rc;
}

static int32_t cam_mem_get_slot(void)
{
	int32_t idx;

	mutex_lock(&tbl.m_lock);
	idx = find_first_zero_bit(tbl.bitmap, tbl.bits);
	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0) {
		mutex_unlock(&tbl.m_lock);
		return -ENOMEM;
	}
	set_bit(idx, tbl.bitmap);
	mutex_unlock(&tbl.m_lock);

	mutex_lock(&tbl.bufq[idx].q_lock);
	spin_lock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].active = true;
	spin_unlock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].release_deferred = false;
	CAM_GET_TIMESTAMP((tbl.bufq[idx].timestamp));
	mutex_unlock(&tbl.bufq[idx].q_lock);

	return idx;
}

static void cam_mem_put_slot(int32_t idx)
{
	mutex_lock(&tbl.bufq[idx].q_lock);
	spin_lock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].active = false;
	spin_unlock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].release_deferred = false;
	tbl.bufq[idx].is_internal = false;
	memset(&tbl.bufq[idx].timestamp, 0, sizeof(struct timespec64));
	kref_init(&tbl.bufq[idx].krefcount);
	kref_init(&tbl.bufq[idx].urefcount);
	mutex_unlock(&tbl.bufq[idx].q_lock);

	mutex_lock(&tbl.m_lock);
	clear_bit(idx, tbl.bitmap);
	mutex_unlock(&tbl.m_lock);
}

static bool cam_mem_mgr_is_iova_info_updated_locked(
	struct cam_mem_buf_hw_hdl_info *hw_vaddr_info_arr,
	int32_t iommu_hdl)
{
	int entry;
	struct cam_mem_buf_hw_hdl_info *vaddr_entry;

	/* validate hdl for entry idx */
	if (!cam_mem_mgr_get_hwva_entry_idx(iommu_hdl, &entry))
		return false;

	vaddr_entry = &hw_vaddr_info_arr[entry];
	if (vaddr_entry->valid_mapping &&
		vaddr_entry->iommu_hdl == iommu_hdl)
		return true;

	return false;
}

static void cam_mem_mgr_update_iova_info_locked(
	struct cam_mem_buf_hw_hdl_info *hw_vaddr_info_arr,
	dma_addr_t vaddr, int32_t iommu_hdl, size_t len,
	bool valid_mapping, struct kref *ref_count)
{
	int entry;
	struct cam_mem_buf_hw_hdl_info *vaddr_entry;

	/* validate hdl for entry idx */
	if (!cam_mem_mgr_get_hwva_entry_idx(iommu_hdl, &entry))
		return;

	vaddr_entry = &hw_vaddr_info_arr[entry];

	vaddr_entry->vaddr = vaddr;
	vaddr_entry->iommu_hdl = iommu_hdl;
	vaddr_entry->addr_updated = true;
	vaddr_entry->valid_mapping = valid_mapping;
	vaddr_entry->len = len;
	vaddr_entry->ref_count = ref_count;
}

/* Utility to be invoked with bufq entry lock held */
static int cam_mem_mgr_try_retrieving_hwva_locked(
	int idx, int32_t mmu_handle, dma_addr_t *iova_ptr, size_t *len_ptr,
	struct list_head *buf_tracker)
{
	int rc = -EINVAL, entry;
	struct cam_mem_buf_hw_hdl_info *hdl_info = NULL;

	/* Check for valid entry */
	if (cam_mem_mgr_get_hwva_entry_idx(mmu_handle, &entry)) {
		hdl_info =  &tbl.bufq[idx].hdls_info[entry];

		/* Ensure we are picking a valid entry */
		if ((hdl_info->iommu_hdl == mmu_handle) && (hdl_info->addr_updated)) {
			*iova_ptr = hdl_info->vaddr;
			*len_ptr = hdl_info->len;
			if (buf_tracker)
				cam_smmu_add_buf_to_track_list(tbl.bufq[idx].fd,
					tbl.bufq[idx].i_ino, &hdl_info->ref_count, buf_tracker,
					GET_SMMU_TABLE_IDX(mmu_handle));
			rc = 0;
		}
	}

	return rc;
}

int cam_mem_get_io_buf(int32_t buf_handle, int32_t mmu_handle,
	dma_addr_t *iova_ptr, size_t *len_ptr, uint32_t *flags,
	struct list_head *buf_tracker)
{
	int rc = 0, idx;
	bool retrieved_iova = false;
	struct kref *ref_count;

	*len_ptr = 0;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	idx = CAM_MEM_MGR_GET_HDL_IDX(buf_handle);
	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0)
		return -ENOENT;

	mutex_lock(&tbl.bufq[idx].q_lock);
	if (!tbl.bufq[idx].active) {
		CAM_ERR(CAM_MEM, "Buffer at idx=%d is already unmapped,",
			idx);
		rc = -EAGAIN;
		goto err;
	}

	if (buf_handle != tbl.bufq[idx].buf_handle) {
		rc = -EINVAL;
		goto err;
	}

	if (flags)
		*flags = tbl.bufq[idx].flags;

	/* Try retrieving iova if saved previously */
	rc = cam_mem_mgr_try_retrieving_hwva_locked(idx, mmu_handle, iova_ptr, len_ptr,
		buf_tracker);
	if (!rc) {
		retrieved_iova = true;
		goto end;
	}

	if (CAM_MEM_MGR_IS_SECURE_HDL(buf_handle))
		rc = cam_smmu_get_stage2_iova(mmu_handle, tbl.bufq[idx].fd, tbl.bufq[idx].dma_buf,
			iova_ptr, len_ptr, buf_tracker, &ref_count);
	else
		rc = cam_smmu_get_iova(mmu_handle, tbl.bufq[idx].fd, tbl.bufq[idx].dma_buf,
			iova_ptr, len_ptr, buf_tracker, &ref_count);

	if (rc) {
		CAM_ERR(CAM_MEM,
			"failed to find buf_hdl:0x%x, mmu_hdl: 0x%x for fd:%d i_ino:%lu",
			buf_handle, mmu_handle, tbl.bufq[idx].fd, tbl.bufq[idx].i_ino);
		goto err;
	}

	/* Save iova in bufq for future use */
	cam_mem_mgr_update_iova_info_locked(tbl.bufq[idx].hdls_info,
		*iova_ptr, mmu_handle, *len_ptr, false, ref_count);

end:
	CAM_DBG(CAM_MEM,
		"handle:0x%x fd:%d i_ino:%lu iova_ptr:0x%lx len_ptr:%lu retrieved from bufq: %s",
		mmu_handle, tbl.bufq[idx].fd, tbl.bufq[idx].i_ino, *iova_ptr, *len_ptr,
		CAM_BOOL_TO_YESNO(retrieved_iova));
err:
	mutex_unlock(&tbl.bufq[idx].q_lock);
	return rc;
}
EXPORT_SYMBOL(cam_mem_get_io_buf);

int cam_mem_get_cpu_buf(int32_t buf_handle, uintptr_t *vaddr_ptr, size_t *len)
{
	int idx, rc = 0;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	if (!buf_handle || !vaddr_ptr || !len)
		return -EINVAL;

	idx = CAM_MEM_MGR_GET_HDL_IDX(buf_handle);

	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0)
		return -EINVAL;

	mutex_lock(&tbl.bufq[idx].q_lock);
	if (!tbl.bufq[idx].active) {
		CAM_ERR(CAM_MEM, "Buffer at idx=%d is already unmapped,",
			idx);
		rc = -EPERM;
		goto end;
	}

	if (buf_handle != tbl.bufq[idx].buf_handle) {
		CAM_ERR(CAM_MEM, "idx: %d Invalid buf handle %d",
				idx, buf_handle);
		rc = -EINVAL;
		goto end;
	}

	if (!(tbl.bufq[idx].flags & CAM_MEM_FLAG_KMD_ACCESS)) {
		CAM_ERR(CAM_MEM, "idx: %d Invalid flag 0x%x",
					idx, tbl.bufq[idx].flags);
		rc = -EINVAL;
		goto end;
	}

	spin_lock(&tbl.bufq[idx].idx_lock);
	if (tbl.bufq[idx].kmdvaddr && kref_get_unless_zero(&tbl.bufq[idx].krefcount)) {
		*vaddr_ptr = tbl.bufq[idx].kmdvaddr;
		*len = tbl.bufq[idx].len;
	} else {
		CAM_ERR(CAM_MEM, "No KMD access requested, kmdvddr= %p, idx= %d, buf_handle= %d",
			tbl.bufq[idx].kmdvaddr, idx, buf_handle);
		rc = -EINVAL;
	}
	spin_unlock(&tbl.bufq[idx].idx_lock);
end:
	mutex_unlock(&tbl.bufq[idx].q_lock);
	return rc;
}
EXPORT_SYMBOL(cam_mem_get_cpu_buf);

int cam_mem_mgr_cache_ops(struct cam_mem_cache_ops_cmd *cmd)
{
	int rc = 0, idx;
	uint32_t cache_dir;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	if (!cmd)
		return -EINVAL;

	idx = CAM_MEM_MGR_GET_HDL_IDX(cmd->buf_handle);
	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0)
		return -EINVAL;

	mutex_lock(&tbl.m_lock);
	if (!test_bit(idx, tbl.bitmap)) {
		CAM_ERR(CAM_MEM, "Buffer at idx=%d is already unmapped,",
			idx);
		mutex_unlock(&tbl.m_lock);
		return -EINVAL;
	}
	mutex_unlock(&tbl.m_lock);

	mutex_lock(&tbl.bufq[idx].q_lock);
	if (cmd->buf_handle != tbl.bufq[idx].buf_handle) {
		rc = -EINVAL;
		goto end;
	}

#if IS_REACHABLE(CONFIG_DMABUF_HEAPS)
	CAM_DBG(CAM_MEM, "Calling dmap buf APIs for cache operations");
	cache_dir = DMA_BIDIRECTIONAL;
#else
	unsigned long dmabuf_flag = 0;
	rc = dma_buf_get_flags(tbl.bufq[idx].dma_buf, &dmabuf_flag);
	if (rc) {
		CAM_ERR(CAM_MEM, "cache get flags failed %d", rc);
		goto end;
	}

	if (dmabuf_flag & ION_FLAG_CACHED) {
		switch (cmd->mem_cache_ops) {
		case CAM_MEM_CLEAN_CACHE:
			cache_dir = DMA_TO_DEVICE;
			break;
		case CAM_MEM_INV_CACHE:
			cache_dir = DMA_FROM_DEVICE;
			break;
		case CAM_MEM_CLEAN_INV_CACHE:
			cache_dir = DMA_BIDIRECTIONAL;
			break;
		default:
			CAM_ERR(CAM_MEM,
				"invalid cache ops :%d", cmd->mem_cache_ops);
			rc = -EINVAL;
			goto end;
		}
	} else {
		CAM_DBG(CAM_MEM, "BUF is not cached");
		goto end;
	}
#endif
	rc = dma_buf_begin_cpu_access(tbl.bufq[idx].dma_buf,
		(cmd->mem_cache_ops == CAM_MEM_CLEAN_INV_CACHE) ?
		DMA_BIDIRECTIONAL : DMA_TO_DEVICE);
	if (rc) {
		CAM_ERR(CAM_MEM, "dma begin access failed rc=%d", rc);
		goto end;
	}

	rc = dma_buf_end_cpu_access(tbl.bufq[idx].dma_buf,
		cache_dir);
	if (rc) {
		CAM_ERR(CAM_MEM, "dma end access failed rc=%d", rc);
		goto end;
	}

end:
	mutex_unlock(&tbl.bufq[idx].q_lock);
	return rc;
}
EXPORT_SYMBOL(cam_mem_mgr_cache_ops);

int cam_mem_mgr_cpu_access_op(struct cam_mem_cpu_access_op *cmd)
{
	int rc = 0, idx;
	uint32_t direction;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	if (!cmd) {
		CAM_ERR(CAM_MEM, "Invalid cmd");
		return -EINVAL;
	}

	idx = CAM_MEM_MGR_GET_HDL_IDX(cmd->buf_handle);
	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0) {
		CAM_ERR(CAM_MEM, "Invalid idx=%d, buf_handle 0x%x, access=0x%x",
			idx, cmd->buf_handle, cmd->access);
		return -EINVAL;
	}

	mutex_lock(&tbl.m_lock);
	if (!test_bit(idx, tbl.bitmap)) {
		CAM_ERR(CAM_MEM, "Buffer at idx=%d is already freed/unmapped", idx);
		mutex_unlock(&tbl.m_lock);
		return -EINVAL;
	}
	mutex_unlock(&tbl.m_lock);

	mutex_lock(&tbl.bufq[idx].q_lock);
	if (cmd->buf_handle != tbl.bufq[idx].buf_handle) {
		CAM_ERR(CAM_MEM,
			"Buffer at idx=%d is different incoming handle 0x%x, actual handle 0x%x",
			idx, cmd->buf_handle, tbl.bufq[idx].buf_handle);
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_MEM, "buf_handle=0x%x, access=0x%x, access_type=0x%x, override_access=%d",
		cmd->buf_handle, cmd->access, cmd->access_type,
		g_cam_mem_mgr_debug.override_cpu_access_dir);

	if (cmd->access_type & CAM_MEM_CPU_ACCESS_READ &&
		cmd->access_type & CAM_MEM_CPU_ACCESS_WRITE) {
		direction = DMA_BIDIRECTIONAL;
	} else if (cmd->access_type & CAM_MEM_CPU_ACCESS_READ) {
		direction = DMA_FROM_DEVICE;
	} else if (cmd->access_type & CAM_MEM_CPU_ACCESS_WRITE) {
		direction = DMA_TO_DEVICE;
	} else {
		direction = DMA_BIDIRECTIONAL;
		CAM_WARN(CAM_MEM,
			"Invalid access type buf_handle=0x%x, access=0x%x, access_type=0x%x",
			cmd->buf_handle, cmd->access, cmd->access_type);
	}

	if (g_cam_mem_mgr_debug.override_cpu_access_dir)
		direction = DMA_BIDIRECTIONAL;

	if (cmd->access & CAM_MEM_BEGIN_CPU_ACCESS) {
		rc = dma_buf_begin_cpu_access(tbl.bufq[idx].dma_buf, direction);
		if (rc) {
			CAM_ERR(CAM_MEM,
				"dma begin cpu access failed rc=%d, buf_handle=0x%x, access=0x%x, access_type=0x%x",
				rc, cmd->buf_handle, cmd->access, cmd->access_type);
			goto end;
		}
	}

	if (cmd->access & CAM_MEM_END_CPU_ACCESS) {
		rc = dma_buf_end_cpu_access(tbl.bufq[idx].dma_buf, direction);
		if (rc) {
			CAM_ERR(CAM_MEM,
				"dma end cpu access failed rc=%d, buf_handle=0x%x, access=0x%x, access_type=0x%x",
				rc, cmd->buf_handle, cmd->access, cmd->access_type);
			goto end;
		}
	}

end:
	mutex_unlock(&tbl.bufq[idx].q_lock);
	return rc;
}
EXPORT_SYMBOL(cam_mem_mgr_cpu_access_op);

#if IS_REACHABLE(CONFIG_DMABUF_HEAPS)

#define CAM_MAX_VMIDS 4

static void cam_mem_mgr_put_dma_heaps(void)
{
	CAM_DBG(CAM_MEM, "Releasing DMA Buf heaps usage");
}

static int cam_mem_mgr_get_dma_heaps(void)
{
	int rc = 0;

	tbl.system_heap = NULL;
	tbl.system_movable_heap = NULL;
	tbl.system_uncached_heap = NULL;
	tbl.camera_heap = NULL;
	tbl.camera_uncached_heap = NULL;
	tbl.secure_display_heap = NULL;
	tbl.ubwc_p_heap = NULL;
	tbl.ubwc_p_movable_heap = NULL;

	tbl.system_heap = dma_heap_find("system");
	if (IS_ERR_OR_NULL(tbl.system_heap)) {
		rc = PTR_ERR(tbl.system_heap);
		CAM_ERR(CAM_MEM, "System heap not found, rc=%d", rc);
		tbl.system_heap = NULL;
		goto put_heaps;
	}

	tbl.system_movable_heap = dma_heap_find("qcom,system-movable");
	if (IS_ERR_OR_NULL(tbl.system_movable_heap)) {
		rc = PTR_ERR(tbl.system_movable_heap);
		CAM_DBG(CAM_MEM, "qcom system heap not found, rc=%d", rc);
		tbl.system_movable_heap = NULL;
		/* not fatal error, we can fallback to system heap */
	}

	tbl.system_uncached_heap = dma_heap_find("qcom,system-uncached");
	if (IS_ERR_OR_NULL(tbl.system_uncached_heap)) {
		if (tbl.force_cache_allocs) {
			/* optional, we anyway do not use uncached */
			CAM_DBG(CAM_MEM,
				"qcom system-uncached heap not found, err=%d",
				PTR_ERR(tbl.system_uncached_heap));
			tbl.system_uncached_heap = NULL;
		} else {
			/* fatal, must need uncached heaps */
			rc = PTR_ERR(tbl.system_uncached_heap);
			CAM_ERR(CAM_MEM,
				"qcom system-uncached heap not found, rc=%d",
				rc);
			tbl.system_uncached_heap = NULL;
			goto put_heaps;
		}
	}

	tbl.ubwc_p_heap = dma_heap_find("qcom,ubwcp");
	if (IS_ERR_OR_NULL(tbl.ubwc_p_heap)) {
		CAM_DBG(CAM_MEM, "qcom ubwcp heap not found, err=%d", PTR_ERR(tbl.ubwc_p_heap));
		tbl.ubwc_p_heap = NULL;
	}

	tbl.ubwc_p_movable_heap = dma_heap_find("qcom,ubwcp-movable");
	if (IS_ERR_OR_NULL(tbl.ubwc_p_movable_heap)) {
		CAM_DBG(CAM_MEM, "qcom ubwcp movable heap not found, err=%d",
			PTR_ERR(tbl.ubwc_p_movable_heap));
		tbl.ubwc_p_movable_heap = NULL;
	}

	tbl.secure_display_heap = dma_heap_find("qcom,display");
	if (IS_ERR_OR_NULL(tbl.secure_display_heap)) {
		rc = PTR_ERR(tbl.secure_display_heap);
		CAM_ERR(CAM_MEM, "qcom,display heap not found, rc=%d",
			rc);
		tbl.secure_display_heap = NULL;
		goto put_heaps;
	}

	tbl.camera_heap = dma_heap_find("qcom,camera");
	if (IS_ERR_OR_NULL(tbl.camera_heap)) {
		/* optional heap, not a fatal error */
		CAM_DBG(CAM_MEM, "qcom camera heap not found, err=%d",
			PTR_ERR(tbl.camera_heap));
		tbl.camera_heap = NULL;
	}

	tbl.camera_uncached_heap = dma_heap_find("qcom,camera-uncached");
	if (IS_ERR_OR_NULL(tbl.camera_uncached_heap)) {
		/* optional heap, not a fatal error */
		CAM_DBG(CAM_MEM, "qcom camera heap not found, err=%d",
			PTR_ERR(tbl.camera_uncached_heap));
		tbl.camera_uncached_heap = NULL;
	}

	CAM_INFO(CAM_MEM,
		"Heaps : system=%pK %pK, system_uncached=%pK, camera=%pK, camera-uncached=%pK, secure_display=%pK, ubwc_p=%pK %pK",
		tbl.system_heap, tbl.system_movable_heap, tbl.system_uncached_heap,
		tbl.camera_heap, tbl.camera_uncached_heap,
		tbl.secure_display_heap, tbl.ubwc_p_heap,  tbl.ubwc_p_movable_heap);

	return 0;
put_heaps:
	cam_mem_mgr_put_dma_heaps();
	return rc;
}

int cam_mem_mgr_check_for_supported_heaps(uint64_t *heap_mask)
{
	uint64_t heap_caps = 0;

	if (!heap_mask)
		return -EINVAL;

	if (tbl.ubwc_p_heap)
		heap_caps |= CAM_REQ_MGR_MEM_UBWC_P_HEAP_SUPPORTED;

	if ((tbl.camera_heap) || (tbl.camera_uncached_heap))
		heap_caps |= CAM_REQ_MGR_MEM_CAMERA_HEAP_SUPPORTED;

	*heap_mask = heap_caps;
	return 0;
}

static int cam_mem_util_get_dma_buf(size_t len,
	unsigned int cam_flags,
	enum cam_mem_mgr_allocator alloc_type,
	struct dma_buf **buf,
	unsigned long *i_ino)
{
	int rc = 0;
	struct dma_heap *heap = NULL, *try_heap = NULL;
	struct timespec64 ts1, ts2;
	long microsec = 0;
	bool use_cached_heap = false;
	struct mem_buf_lend_kernel_arg arg;
	int vmids[CAM_MAX_VMIDS];
	int perms[CAM_MAX_VMIDS];
	int num_vmids = 0;

	if (!buf) {
		CAM_ERR(CAM_MEM, "Invalid params");
		return -EINVAL;
	}

	if (g_cam_mem_mgr_debug.alloc_profile_enable)
		CAM_GET_TIMESTAMP(ts1);

	if ((cam_flags & CAM_MEM_FLAG_CACHE) ||
		(tbl.force_cache_allocs &&
		(!(cam_flags & CAM_MEM_FLAG_PROTECTED_MODE)))) {
		CAM_DBG(CAM_MEM,
			"Using CACHED heap, cam_flags=0x%x, force_cache_allocs=%d",
			cam_flags, tbl.force_cache_allocs);
		use_cached_heap = true;
	} else if (cam_flags & CAM_MEM_FLAG_PROTECTED_MODE) {
		use_cached_heap = true;
		CAM_DBG(CAM_MEM,
			"Using CACHED heap for secure, cam_flags=0x%x, force_cache_allocs=%d",
			cam_flags, tbl.force_cache_allocs);
	} else {
		use_cached_heap = false;
		if (!tbl.system_uncached_heap && !tbl.camera_uncached_heap) {
			CAM_ERR(CAM_MEM,
				"Using UNCACHED heap not supported, cam_flags=0x%x, force_cache_allocs=%d",
				cam_flags, tbl.force_cache_allocs);
			return -EINVAL;
		}
	}

	if (cam_flags & CAM_MEM_FLAG_PROTECTED_MODE) {
		if (IS_CSF25(tbl.csf_version.arch_ver, tbl.csf_version.max_ver)) {
			heap = tbl.system_heap;
			len = cam_align_dma_buf_size(len);
		} else {
			heap = tbl.secure_display_heap;
			vmids[num_vmids] = VMID_CP_CAMERA;
			perms[num_vmids] = PERM_READ | PERM_WRITE;
			num_vmids++;
		}

		if (cam_flags & CAM_MEM_FLAG_CDSP_OUTPUT) {
			CAM_DBG(CAM_MEM, "Secure mode CDSP flags");

			vmids[num_vmids] = VMID_CP_CDSP;
			perms[num_vmids] = PERM_READ | PERM_WRITE;
			num_vmids++;
		}
	} else if (cam_flags & CAM_MEM_FLAG_EVA_NOPIXEL) {
		heap = tbl.secure_display_heap;
		vmids[num_vmids] = VMID_CP_NON_PIXEL;
		perms[num_vmids] = PERM_READ | PERM_WRITE;
		num_vmids++;
	} else if (cam_flags & CAM_MEM_FLAG_UBWC_P_HEAP) {
		if (!tbl.ubwc_p_heap) {
			CAM_ERR(CAM_MEM, "ubwc-p heap is not available, can't allocate");
			return -EINVAL;
		}

		if (tbl.ubwc_p_movable_heap && (alloc_type == CAM_MEMMGR_ALLOC_USER))
			heap = tbl.ubwc_p_movable_heap;
		else
			heap = tbl.ubwc_p_heap;
		CAM_DBG(CAM_MEM, "Allocating from ubwc-p heap %pK, size=%d, flags=0x%x",
			heap, len, cam_flags);
	} else if (use_cached_heap) {

		/*
		 * The default scheme is to try allocating from the camera heap
		 * if available; if not, try for the system heap. Userland can also select
		 * to pick a specific heap for allocation; this will deviate from the
		 * default selection scheme.
		 *
		 */
		if (!(cam_flags & CAM_MEM_FLAG_USE_SYS_HEAP_ONLY))
			try_heap = tbl.camera_heap;

		if (!(cam_flags & CAM_MEM_FLAG_USE_CAMERA_HEAP_ONLY)) {
			if (tbl.system_movable_heap && (alloc_type == CAM_MEMMGR_ALLOC_USER))
				heap = tbl.system_movable_heap;
			else
				heap = tbl.system_heap;
		}
	} else {
		if (!(cam_flags & CAM_MEM_FLAG_USE_SYS_HEAP_ONLY))
			try_heap = tbl.camera_uncached_heap;

		if (!(cam_flags & CAM_MEM_FLAG_USE_CAMERA_HEAP_ONLY))
			heap = tbl.system_uncached_heap;
	}

	CAM_DBG(CAM_MEM, "Using heaps : try=%pK, heap=%pK", try_heap, heap);

	*buf = NULL;

	if (!try_heap && !heap) {
		CAM_ERR(CAM_MEM,
			"No heap available for allocation, can't allocate flag: 0x%x",
			cam_flags);
		return -EINVAL;
	}

	if (try_heap) {
		*buf = dma_heap_buffer_alloc(try_heap, len, O_RDWR, 0);
		if (IS_ERR(*buf)) {
			CAM_WARN(CAM_MEM,
				"Failed in allocating from try heap, heap=%pK, len=%zu, err=%d",
				try_heap, len, PTR_ERR(*buf));
			*buf = NULL;
		}
	}

	if (*buf == NULL) {
		*buf = dma_heap_buffer_alloc(heap, len, O_RDWR, 0);
		if (IS_ERR(*buf)) {
			rc = PTR_ERR(*buf);
			CAM_ERR(CAM_MEM,
				"Failed in allocating from heap, heap=%pK, len=%zu, err=%d",
				heap, len, rc);
			*buf = NULL;
			return rc;
		}
	}

	*i_ino = file_inode((*buf)->file)->i_ino;

	if (((cam_flags & CAM_MEM_FLAG_PROTECTED_MODE) &&
		!IS_CSF25(tbl.csf_version.arch_ver, tbl.csf_version.max_ver)) ||
		(cam_flags & CAM_MEM_FLAG_EVA_NOPIXEL)) {
		if (num_vmids >= CAM_MAX_VMIDS) {
			CAM_ERR(CAM_MEM, "Insufficient array size for vmids %d", num_vmids);
			rc = -EINVAL;
			goto end;
		}

		arg.nr_acl_entries = num_vmids;
		arg.vmids = vmids;
		arg.perms = perms;

		rc = mem_buf_lend(*buf, &arg);
		if (rc) {
			CAM_ERR(CAM_MEM,
				"Failed in buf lend rc=%d, buf=%pK, vmids [0]=0x%x, [1]=0x%x, [2]=0x%x",
				rc, *buf, vmids[0], vmids[1], vmids[2]);
			goto end;
		}
	}

	CAM_DBG(CAM_MEM, "Allocate success, len=%zu, *buf=%pK, i_ino=%lu", len, *buf, *i_ino);

	if (g_cam_mem_mgr_debug.alloc_profile_enable) {
		CAM_GET_TIMESTAMP(ts2);
		CAM_GET_TIMESTAMP_DIFF_IN_MICRO(ts1, ts2, microsec);
		trace_cam_log_event("IONAllocProfile", "size and time in micro",
			len, microsec);
	}

	return rc;
end:
	dma_buf_put(*buf);
	return rc;
}
#else

bool cam_mem_mgr_ubwc_p_heap_supported(void)
{
	return false;
}

static int cam_mem_util_get_dma_buf(size_t len,
	unsigned int cam_flags,
	enum cam_mem_mgr_allocator alloc_type,
	struct dma_buf **buf,
	unsigned long *i_ino)
{
	int rc = 0;
	unsigned int heap_id;
	int32_t ion_flag = 0;
	struct timespec64 ts1, ts2;
	long microsec = 0;

	if (!buf) {
		CAM_ERR(CAM_MEM, "Invalid params");
		return -EINVAL;
	}

	if (cam_flags & CAM_MEM_FLAG_UBWC_P_HEAP) {
		CAM_ERR(CAM_MEM, "ubwcp heap not supported");
		return -EINVAL;
	}

	if (g_cam_mem_mgr_debug.alloc_profile_enable)
		CAM_GET_TIMESTAMP(ts1);

	if ((cam_flags & CAM_MEM_FLAG_PROTECTED_MODE) &&
		(cam_flags & CAM_MEM_FLAG_CDSP_OUTPUT)) {
		heap_id = ION_HEAP(ION_SECURE_DISPLAY_HEAP_ID);
		ion_flag |=
			ION_FLAG_SECURE | ION_FLAG_CP_CAMERA | ION_FLAG_CP_CDSP;
	} else if (cam_flags & CAM_MEM_FLAG_PROTECTED_MODE) {
		heap_id = ION_HEAP(ION_SECURE_DISPLAY_HEAP_ID);
		ion_flag |= ION_FLAG_SECURE | ION_FLAG_CP_CAMERA;
	} else {
		heap_id = ION_HEAP(ION_SYSTEM_HEAP_ID) |
			ION_HEAP(ION_CAMERA_HEAP_ID);
	}

	if (cam_flags & CAM_MEM_FLAG_CACHE)
		ion_flag |= ION_FLAG_CACHED;
	else
		ion_flag &= ~ION_FLAG_CACHED;

	if (tbl.force_cache_allocs && (!(ion_flag & ION_FLAG_SECURE)))
		ion_flag |= ION_FLAG_CACHED;

	*buf = ion_alloc(len, heap_id, ion_flag);
	if (IS_ERR_OR_NULL(*buf))
		return -ENOMEM;

	*i_ino = file_inode((*buf)->file)->i_ino;

	if (g_cam_mem_mgr_debug.alloc_profile_enable) {
		CAM_GET_TIMESTAMP(ts2);
		CAM_GET_TIMESTAMP_DIFF_IN_MICRO(ts1, ts2, microsec);
		trace_cam_log_event("IONAllocProfile", "size and time in micro",
			len, microsec);
	}

	return rc;
}
#endif

static int cam_mem_util_buffer_alloc(size_t len, uint32_t flags,
	struct dma_buf **dmabuf,
	int *fd,
	unsigned long *i_ino)
{
	int rc;

	rc = cam_mem_util_get_dma_buf(len, flags, CAM_MEMMGR_ALLOC_USER, dmabuf, i_ino);
	if (rc) {
		CAM_ERR(CAM_MEM,
			"Error allocating dma buf : len=%llu, flags=0x%x",
			len, flags);
		return rc;
	}

	/*
	 * increment the ref count so that ref count becomes 2 here
	 * when we close fd, refcount becomes 1 and when we do
	 * dmap_put_buf, ref count becomes 0 and memory will be freed.
	 */
	get_dma_buf(*dmabuf);

	*fd = dma_buf_fd(*dmabuf, O_CLOEXEC);
	if (*fd < 0) {
		CAM_ERR(CAM_MEM, "get fd fail, *fd=%d", *fd);
		rc = -EINVAL;
		goto put_buf;
	}

	CAM_DBG(CAM_MEM, "Alloc success : len=%zu, *dmabuf=%pK, fd=%d, i_ino=%lu",
		len, *dmabuf, *fd, *i_ino);

	return rc;

put_buf:
	dma_buf_put(*dmabuf);
	return rc;
}

static int cam_mem_util_check_alloc_flags(struct cam_mem_mgr_alloc_cmd_v2 *cmd)
{
	if (cmd->num_hdl > CAM_MEM_MMU_MAX_HANDLE) {
		CAM_ERR(CAM_MEM, "Num of mmu hdl exceeded maximum(%d)",
			CAM_MEM_MMU_MAX_HANDLE);
		return -EINVAL;
	}

	if (cmd->flags & CAM_MEM_FLAG_PROTECTED_MODE &&
		cmd->flags & CAM_MEM_FLAG_KMD_ACCESS) {
		CAM_ERR(CAM_MEM, "Kernel mapping in secure mode not allowed");
		return -EINVAL;
	}

	if ((cmd->flags & CAM_MEM_FLAG_EVA_NOPIXEL) &&
		(cmd->flags & CAM_MEM_FLAG_PROTECTED_MODE ||
		cmd->flags & CAM_MEM_FLAG_KMD_ACCESS)) {
		CAM_ERR(CAM_MEM,
			"Kernel mapping and secure mode not allowed in no pixel mode");
		return -EINVAL;
	}

	if (cmd->flags & CAM_MEM_FLAG_UBWC_P_HEAP &&
		(cmd->flags & CAM_MEM_FLAG_PROTECTED_MODE ||
		cmd->flags & CAM_MEM_FLAG_EVA_NOPIXEL ||
		cmd->flags & CAM_MEM_FLAG_KMD_ACCESS ||
		cmd->flags & CAM_MEM_FLAG_CMD_BUF_TYPE ||
		cmd->flags & CAM_MEM_FLAG_HW_SHARED_ACCESS ||
		cmd->flags & CAM_MEM_FLAG_HW_AND_CDM_OR_SHARED)) {
		CAM_ERR(CAM_MEM,
			"UBWC-P buffer not supported with this combinatation of flags 0x%x",
			cmd->flags);
		return -EINVAL;
	}

	return 0;
}

static int cam_mem_util_check_map_flags(struct cam_mem_mgr_map_cmd_v2 *cmd)
{
	if (!cmd->flags) {
		CAM_ERR(CAM_MEM, "Invalid flags");
		return -EINVAL;
	}

	if (cmd->num_hdl > CAM_MEM_MMU_MAX_HANDLE) {
		CAM_ERR(CAM_MEM, "Num of mmu hdl %d exceeded maximum(%d)",
			cmd->num_hdl, CAM_MEM_MMU_MAX_HANDLE);
		return -EINVAL;
	}

	if (cmd->flags & CAM_MEM_FLAG_PROTECTED_MODE &&
		cmd->flags & CAM_MEM_FLAG_KMD_ACCESS) {
		CAM_ERR(CAM_MEM,
			"Kernel mapping in secure mode not allowed, flags=0x%x",
			cmd->flags);
		return -EINVAL;
	}

	if (cmd->flags & CAM_MEM_FLAG_HW_SHARED_ACCESS) {
		CAM_ERR(CAM_MEM,
			"Shared memory buffers are not allowed to be mapped");
		return -EINVAL;
	}

	return 0;
}

static int cam_mem_util_map_hw_va(uint32_t flags,
	int32_t *mmu_hdls,
	int32_t num_hdls,
	int fd,
	struct dma_buf *dmabuf,
	struct cam_mem_buf_hw_hdl_info *hw_vaddr_info_arr,
	size_t *len,
	enum cam_smmu_region_id region,
	bool is_internal)
{
	int i;
	int rc = -1;
	int dir = cam_mem_util_get_dma_dir(flags);
	/* Delayed unmap should be disabled by default in QLI */
	bool dis_delayed_unmap = true;
	dma_addr_t hw_vaddr;
	struct kref *ref_count;
	struct cam_mem_buf_hw_hdl_info *hdl_info = NULL;

	if (dir < 0) {
		CAM_ERR(CAM_MEM, "fail to map DMA direction, dir=%d", dir);
		return dir;
	}

	if (flags & CAM_MEM_FLAG_DISABLE_DELAYED_UNMAP)
		dis_delayed_unmap = true;

	CAM_DBG(CAM_MEM,
		"map_hw_va : fd = %d, flags = 0x%x, dir=%d, num_hdls=%d",
		fd, flags, dir, num_hdls);

	for (i = 0; i < num_hdls; i++) {
		if (cam_mem_mgr_is_iova_info_updated_locked(hw_vaddr_info_arr, mmu_hdls[i]))
			continue;

		/* If 36-bit enabled, check for ICP cmd buffers and map them within the shared region */
		if (cam_smmu_is_expanded_memory() &&
			cam_smmu_supports_shared_region(mmu_hdls[i]) &&
			((flags & CAM_MEM_FLAG_CMD_BUF_TYPE) ||
			(flags & CAM_MEM_FLAG_HW_AND_CDM_OR_SHARED)))
			region = CAM_SMMU_REGION_SHARED;

		if (flags & CAM_MEM_FLAG_PROTECTED_MODE)
			rc = cam_smmu_map_stage2_iova(mmu_hdls[i], fd, dmabuf, dir, &hw_vaddr, len,
				&ref_count);
		else
			rc = cam_smmu_map_user_iova(mmu_hdls[i], fd, dmabuf, dis_delayed_unmap, dir,
				&hw_vaddr, len, region, is_internal, &ref_count);
		if (rc) {
			CAM_ERR(CAM_MEM,
					"Failed %s map to smmu, i=%d, fd=%d, dir=%d, mmu_hdl=%d, rc=%d",
					(flags & CAM_MEM_FLAG_PROTECTED_MODE) ? "" : "secured",
					i, fd, dir, mmu_hdls[i], rc);
			goto multi_map_fail;
		}

		/* cache hw va */
		cam_mem_mgr_update_iova_info_locked(hw_vaddr_info_arr,
			hw_vaddr, mmu_hdls[i], *len, true, ref_count);
	}

	return rc;
multi_map_fail:
	for (i = 0; i < tbl.max_hdls_supported; i++) {
		if (!hw_vaddr_info_arr[i].valid_mapping)
			continue;

		hdl_info = &hw_vaddr_info_arr[i];

		if (flags & CAM_MEM_FLAG_PROTECTED_MODE)
			cam_smmu_unmap_stage2_iova(hdl_info->iommu_hdl, fd, dmabuf,
				false);
		else
			cam_smmu_unmap_user_iova(hdl_info->iommu_hdl, fd, dmabuf,
				CAM_SMMU_REGION_IO, false);
	}
	/* reset any updated entries */
	memset(hw_vaddr_info_arr, 0x0, tbl.max_hdls_info_size);
	return rc;
}

int cam_mem_mgr_alloc_and_map(struct cam_mem_mgr_alloc_cmd_v2 *cmd)
{
	int rc, idx;
	struct dma_buf *dmabuf = NULL;
	int fd = -1;
	size_t len;
	uintptr_t kvaddr = 0;
	size_t klen;
	unsigned long i_ino = 0;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	if (!cmd) {
		CAM_ERR(CAM_MEM, " Invalid argument");
		return -EINVAL;
	}

	if (cmd->num_hdl > tbl.max_hdls_supported) {
		CAM_ERR(CAM_MEM, "Num of mmu hdl %d exceeded maximum(%d)",
			cmd->num_hdl, tbl.max_hdls_supported);
		return -EINVAL;
	}

	len = cmd->len;

	if (tbl.need_shared_buffer_padding &&
		(cmd->flags & CAM_MEM_FLAG_HW_SHARED_ACCESS)) {
		len += CAM_MEM_SHARED_BUFFER_PAD_4K;
		CAM_DBG(CAM_MEM, "Pad 4k size, actual %llu, allocating %zu",
			cmd->len, len);
	}

	rc = cam_mem_util_check_alloc_flags(cmd);
	if (rc) {
		CAM_ERR(CAM_MEM, "Invalid flags: flags = 0x%X, rc=%d",
			cmd->flags, rc);
		return rc;
	}

	rc = cam_mem_util_buffer_alloc(len, cmd->flags, &dmabuf, &fd, &i_ino);
	if (rc) {
		CAM_ERR(CAM_MEM,
			"Ion Alloc failed, len=%llu, align=%llu, flags=0x%x, num_hdl=%d",
			len, cmd->align, cmd->flags, cmd->num_hdl);
		cam_mem_mgr_print_tbl();
		return rc;
	}
	if (!dmabuf) {
		CAM_ERR(CAM_MEM,
			"Ion Alloc return NULL dmabuf! fd=%d, i_ino=%lu, len=%d", fd, i_ino, len);
		cam_mem_mgr_print_tbl();
		return rc;
	}

	idx = cam_mem_get_slot();
	if (idx < 0) {
		CAM_ERR(CAM_MEM, "Failed in getting mem slot, idx=%d", idx);
		rc = -ENOMEM;
		cam_mem_mgr_print_tbl();
		goto slot_fail;
	}

	if (cam_dma_buf_set_name(dmabuf, cmd->buf_name))
		CAM_ERR(CAM_MEM, "set dma buffer name(%s) failed", cmd->buf_name);

	if ((cmd->flags & CAM_MEM_FLAG_HW_READ_WRITE) ||
		(cmd->flags & CAM_MEM_FLAG_HW_SHARED_ACCESS) ||
		(cmd->flags & CAM_MEM_FLAG_PROTECTED_MODE)) {

		enum cam_smmu_region_id region;

		if (cmd->flags & CAM_MEM_FLAG_HW_READ_WRITE)
			region = CAM_SMMU_REGION_IO;


		if (cmd->flags & CAM_MEM_FLAG_HW_SHARED_ACCESS)
			region = CAM_SMMU_REGION_SHARED;

		if (cmd->flags & CAM_MEM_FLAG_PROTECTED_MODE)
			region = CAM_SMMU_REGION_IO;

		rc = cam_mem_util_map_hw_va(cmd->flags,
			cmd->mmu_hdls,
			cmd->num_hdl,
			fd,
			dmabuf,
			tbl.bufq[idx].hdls_info,
			&len,
			region,
			true);

		if (rc) {
			CAM_ERR(CAM_MEM,
				"Failed in map_hw_va len=%llu, flags=0x%x, fd=%d, region=%d, num_hdl=%d, rc=%d",
				len, cmd->flags,
				fd, region, cmd->num_hdl, rc);
			if (rc == -EALREADY) {
				if ((size_t)dmabuf->size != len)
					rc = -EBADR;
				cam_mem_mgr_print_tbl();
			}
			goto map_hw_fail;
		}
	}

	mutex_lock(&tbl.bufq[idx].q_lock);
	tbl.bufq[idx].fd = fd;
	tbl.bufq[idx].i_ino = i_ino;
	tbl.bufq[idx].dma_buf = NULL;
	tbl.bufq[idx].flags = cmd->flags;
	tbl.bufq[idx].buf_handle = GET_MEM_HANDLE(idx, fd);
	tbl.bufq[idx].is_internal = true;
	if (cmd->flags & CAM_MEM_FLAG_PROTECTED_MODE)
		CAM_MEM_MGR_SET_SECURE_HDL(tbl.bufq[idx].buf_handle, true);

	if (cmd->flags & CAM_MEM_FLAG_KMD_ACCESS) {
		rc = cam_mem_util_map_cpu_va(dmabuf, &kvaddr, &klen);
		if (rc) {
			CAM_ERR(CAM_MEM, "dmabuf: %pK mapping failed: %d",
				dmabuf, rc);
			goto map_kernel_fail;
		}
	}

	if (cmd->flags & CAM_MEM_FLAG_KMD_DEBUG_FLAG)
		tbl.dbg_buf_idx = idx;

	tbl.bufq[idx].kmdvaddr = kvaddr;
	tbl.bufq[idx].dma_buf = dmabuf;
	tbl.bufq[idx].len = len;
	tbl.bufq[idx].num_hdls = cmd->num_hdl;
	cam_mem_mgr_reset_presil_params(idx);
	tbl.bufq[idx].is_imported = false;

	if (cmd->flags & CAM_MEM_FLAG_KMD_ACCESS)
		kref_init(&tbl.bufq[idx].krefcount);

	kref_init(&tbl.bufq[idx].urefcount);

	tbl.bufq[idx].smmu_mapping_client = CAM_SMMU_MAPPING_USER;
	strscpy(tbl.bufq[idx].buf_name, cmd->buf_name, sizeof(tbl.bufq[idx].buf_name));
	mutex_unlock(&tbl.bufq[idx].q_lock);

	cmd->out.buf_handle = tbl.bufq[idx].buf_handle;
	cmd->out.fd = tbl.bufq[idx].fd;
	cmd->out.vaddr = 0;

	CAM_DBG(CAM_MEM,
		"fd=%d, flags=0x%x, num_hdl=%d, idx=%d, buf handle=%x, len=%zu, i_ino=%lu, name:%s",
		cmd->out.fd, cmd->flags, cmd->num_hdl, idx, cmd->out.buf_handle,
		tbl.bufq[idx].len, tbl.bufq[idx].i_ino, cmd->buf_name);

	return rc;

map_kernel_fail:
	mutex_unlock(&tbl.bufq[idx].q_lock);
map_hw_fail:
	cam_mem_put_slot(idx);
slot_fail:
	dma_buf_put(dmabuf);
	return rc;
}

static bool cam_mem_util_is_map_internal(int32_t fd, unsigned i_ino)
{
	uint32_t i;
	bool is_internal = false;

	mutex_lock(&tbl.m_lock);
	for_each_set_bit(i, tbl.bitmap, tbl.bits) {
		if ((tbl.bufq[i].fd == fd) && (tbl.bufq[i].i_ino == i_ino)) {
			is_internal = tbl.bufq[i].is_internal;
			break;
		}
	}
	mutex_unlock(&tbl.m_lock);

	return is_internal;
}

int cam_mem_mgr_map(struct cam_mem_mgr_map_cmd_v2 *cmd)
{
	int32_t idx;
	int rc;
	struct dma_buf *dmabuf;
	size_t len = 0;
	bool is_internal = false;
	unsigned long i_ino;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	if (!cmd || (cmd->fd < 0)) {
		CAM_ERR(CAM_MEM, "Invalid argument");
		return -EINVAL;
	}

	if (cmd->num_hdl > tbl.max_hdls_supported) {
		CAM_ERR(CAM_MEM, "Num of mmu hdl %d exceeded maximum(%d)",
			cmd->num_hdl, tbl.max_hdls_supported);
		return -EINVAL;
	}

	rc = cam_mem_util_check_map_flags(cmd);
	if (rc) {
		CAM_ERR(CAM_MEM, "Invalid flags: flags = %X", cmd->flags);
		return rc;
	}

	dmabuf = dma_buf_get(cmd->fd);
	if (IS_ERR_OR_NULL((void *)(dmabuf))) {
		CAM_ERR(CAM_MEM, "Failed to import dma_buf fd");
		return -EINVAL;
	}

	i_ino = file_inode(dmabuf->file)->i_ino;

	is_internal = cam_mem_util_is_map_internal(cmd->fd, i_ino);

	idx = cam_mem_get_slot();
	if (idx < 0) {
		CAM_ERR(CAM_MEM, "Failed in getting mem slot, idx=%d, fd=%d",
			idx, cmd->fd);
		rc = -ENOMEM;
		cam_mem_mgr_print_tbl();
		goto slot_fail;
	}

	if (cam_dma_buf_set_name(dmabuf, cmd->buf_name))
		CAM_DBG(CAM_MEM, "Dma buffer (%s) busy", cmd->buf_name);

	if ((cmd->flags & CAM_MEM_FLAG_HW_READ_WRITE) ||
		(cmd->flags & CAM_MEM_FLAG_PROTECTED_MODE)) {
		rc = cam_mem_util_map_hw_va(cmd->flags,
			cmd->mmu_hdls,
			cmd->num_hdl,
			cmd->fd,
			dmabuf,
			tbl.bufq[idx].hdls_info,
			&len,
			CAM_SMMU_REGION_IO,
			is_internal);
		if (rc) {
			CAM_ERR(CAM_MEM,
				"Failed in map_hw_va, flags=0x%x, fd=%d, len=%llu, region=%d, num_hdl=%d, rc=%d",
				cmd->flags, cmd->fd, len,
				CAM_SMMU_REGION_IO, cmd->num_hdl, rc);
			if (rc == -EALREADY) {
				if ((size_t)dmabuf->size != len) {
					rc = -EBADR;
					cam_mem_mgr_print_tbl();
				}
			}
			goto map_fail;
		}
	}

	mutex_lock(&tbl.bufq[idx].q_lock);
	tbl.bufq[idx].fd = cmd->fd;
	tbl.bufq[idx].i_ino = i_ino;
	tbl.bufq[idx].dma_buf = NULL;
	tbl.bufq[idx].flags = cmd->flags;
	spin_lock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].buf_handle = GET_MEM_HANDLE(idx, cmd->fd);
	spin_unlock(&tbl.bufq[idx].idx_lock);

	if (cmd->flags & CAM_MEM_FLAG_PROTECTED_MODE)
		CAM_MEM_MGR_SET_SECURE_HDL(tbl.bufq[idx].buf_handle, true);
	tbl.bufq[idx].kmdvaddr = 0;
	tbl.bufq[idx].dma_buf = dmabuf;
	tbl.bufq[idx].len = len;
	tbl.bufq[idx].num_hdls = cmd->num_hdl;
	tbl.bufq[idx].is_imported = true;
	tbl.bufq[idx].is_internal = is_internal;
	if (cmd->flags & CAM_MEM_FLAG_KMD_ACCESS)
		kref_init(&tbl.bufq[idx].krefcount);
	kref_init(&tbl.bufq[idx].urefcount);
	tbl.bufq[idx].smmu_mapping_client = CAM_SMMU_MAPPING_USER;
	strscpy(tbl.bufq[idx].buf_name, cmd->buf_name, sizeof(tbl.bufq[idx].buf_name));
	mutex_unlock(&tbl.bufq[idx].q_lock);

	cmd->out.buf_handle = tbl.bufq[idx].buf_handle;
	cmd->out.vaddr = 0;
	cmd->out.size = (uint32_t)len;
	CAM_DBG(CAM_MEM,
		"fd=%d, flags=0x%x, num_hdl=%d, idx=%d, buf handle=%x, len=%zu, i_ino=%lu, name:%s",
		cmd->fd, cmd->flags, cmd->num_hdl, idx, cmd->out.buf_handle,
		tbl.bufq[idx].len, tbl.bufq[idx].i_ino, cmd->buf_name);

	return rc;
map_fail:
	cam_mem_put_slot(idx);
slot_fail:
	dma_buf_put(dmabuf);
	return rc;
}

static int cam_mem_util_unmap_hw_va(int32_t idx,
	enum cam_smmu_region_id region,
	enum cam_smmu_mapping_client client, bool force_unmap)
{
	int i, fd, num_hdls;
	uint32_t flags;
	struct cam_mem_buf_hw_hdl_info *hdl_info = NULL;
	struct dma_buf *dma_buf;
	unsigned long i_ino;
	int rc = 0;

	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0) {
		CAM_ERR(CAM_MEM, "Incorrect index");
		return -EINVAL;
	}

	flags = tbl.bufq[idx].flags;
	num_hdls = tbl.bufq[idx].num_hdls;
	fd = tbl.bufq[idx].fd;
	dma_buf = tbl.bufq[idx].dma_buf;
	i_ino = tbl.bufq[idx].i_ino;

	if (unlikely(!num_hdls)) {
		CAM_DBG(CAM_MEM, "No valid handles to unmap");
		return 0;
	}

	CAM_DBG(CAM_MEM,
		"unmap_hw_va : idx=%d, fd=%x, i_ino=%lu flags=0x%x, num_hdls=%d, client=%d",
		idx, fd, i_ino, flags, tbl.bufq[idx].num_hdls, client);

	for (i = 0; i < tbl.max_hdls_supported; i++) {
		if (!tbl.bufq[idx].hdls_info[i].valid_mapping)
			continue;

		hdl_info = &tbl.bufq[idx].hdls_info[i];

		if (flags & CAM_MEM_FLAG_PROTECTED_MODE)
			rc = cam_smmu_unmap_stage2_iova(hdl_info->iommu_hdl, fd, dma_buf,
				force_unmap);
		else if (client == CAM_SMMU_MAPPING_USER)
			rc = cam_smmu_unmap_user_iova(hdl_info->iommu_hdl, fd, dma_buf, region,
				force_unmap);
		else if (client == CAM_SMMU_MAPPING_KERNEL)
			rc = cam_smmu_unmap_kernel_iova(hdl_info->iommu_hdl,
				tbl.bufq[idx].dma_buf, region);
		else {
			CAM_ERR(CAM_MEM, "invalid caller for unmapping : %d", client);
			rc = -EINVAL;
			goto end;
		}

		if (rc < 0) {
			CAM_ERR(CAM_MEM,
				"Failed in %s unmap, i=%d, fd=%d, i_ino=%lu, mmu_hdl=%d, rc=%d",
				((flags & CAM_MEM_FLAG_PROTECTED_MODE) ? "secure" : "non-secure"),
				i, fd, i_ino, hdl_info->iommu_hdl, rc);
			goto end;
		}

		CAM_DBG(CAM_MEM,
			"i: %d unmap_hw_va : idx=%d, fd=%x, i_ino=%lu flags=0x%x, num_hdls=%d, client=%d hdl: %d",
			i, idx, fd, i_ino, flags, tbl.bufq[idx].num_hdls,
			client, hdl_info->iommu_hdl);

		/* exit loop if all handles for this buffer have been unmapped */
		if (!(--num_hdls))
			break;
	}

end:
	return rc;
}

static void cam_mem_mgr_unmap_active_buf(int idx)
{
	enum cam_smmu_region_id region = CAM_SMMU_REGION_SHARED;

	if (tbl.bufq[idx].flags & CAM_MEM_FLAG_HW_SHARED_ACCESS)
		region = CAM_SMMU_REGION_SHARED;
	else if (tbl.bufq[idx].flags & CAM_MEM_FLAG_HW_READ_WRITE)
		region = CAM_SMMU_REGION_IO;

	cam_mem_util_unmap_hw_va(idx, region, CAM_SMMU_MAPPING_USER, true);

	if (tbl.bufq[idx].flags & CAM_MEM_FLAG_KMD_ACCESS)
		cam_mem_util_unmap_cpu_va(tbl.bufq[idx].dma_buf,
			tbl.bufq[idx].kmdvaddr);
}

static int cam_mem_mgr_cleanup_table(void)
{
	int i;

	for (i = 1; i < CAM_MEM_BUFQ_MAX; i++) {
		mutex_lock(&tbl.bufq[i].q_lock);
		spin_lock(&tbl.bufq[i].idx_lock);
		if (!tbl.bufq[i].active) {
			CAM_DBG(CAM_MEM,
				"Buffer inactive at idx=%d, continuing", i);
			spin_unlock(&tbl.bufq[i].idx_lock);
			mutex_unlock(&tbl.bufq[i].q_lock);
			mutex_destroy(&tbl.bufq[i].q_lock);
			continue;
		} else {
			CAM_DBG(CAM_MEM,
			"Active buffer at idx=%d, possible leak needs unmapping",
			i);
			spin_unlock(&tbl.bufq[i].idx_lock);
			cam_mem_mgr_unmap_active_buf(i);
		}

		if (tbl.bufq[i].dma_buf) {
			dma_buf_put(tbl.bufq[i].dma_buf);
			tbl.bufq[i].dma_buf = NULL;
		}
		tbl.bufq[i].fd = -1;
		tbl.bufq[i].i_ino = 0;
		tbl.bufq[i].flags = 0;
		tbl.bufq[i].buf_handle = -1;
		tbl.bufq[i].len = 0;
		tbl.bufq[i].num_hdls = 0;
		tbl.bufq[i].dma_buf = NULL;
		tbl.bufq[i].active = false;
		tbl.bufq[i].release_deferred = false;
		tbl.bufq[i].is_internal = false;
		kref_init(&tbl.bufq[i].krefcount);
		kref_init(&tbl.bufq[i].urefcount);
		memset(tbl.bufq[i].hdls_info, 0x0, tbl.max_hdls_info_size);
		cam_mem_mgr_reset_presil_params(i);
		mutex_unlock(&tbl.bufq[i].q_lock);
		mutex_destroy(&tbl.bufq[i].q_lock);
	}
	mutex_lock(&tbl.m_lock);
	bitmap_zero(tbl.bitmap, tbl.bits);
	/* We need to reserve slot 0 because 0 is invalid */
	set_bit(0, tbl.bitmap);
	mutex_unlock(&tbl.m_lock);

	return 0;
}

void cam_mem_mgr_deinit(void)
{
	int i;

	if (!atomic_read(&cam_mem_mgr_state))
		return;

	atomic_set(&cam_mem_mgr_state, CAM_MEM_MGR_UNINITIALIZED);
	cam_mem_mgr_cleanup_table();
	cam_smmu_driver_deinit();
	mutex_lock(&tbl.m_lock);
	bitmap_zero(tbl.bitmap, tbl.bits);
	kfree(tbl.bitmap);
	tbl.bitmap = NULL;
	tbl.dbg_buf_idx = -1;

	/* index 0 is reserved */
	for (i = 1; i < CAM_MEM_BUFQ_MAX; i++) {
		kfree(tbl.bufq[i].hdls_info);
		tbl.bufq[i].hdls_info = NULL;
	}

	mutex_unlock(&tbl.m_lock);
	mutex_destroy(&tbl.m_lock);
}

static void cam_mem_util_unmap_dummy(struct kref *kref)
{
	CAM_DBG(CAM_MEM, "Cam mem util unmap dummy");
}

static void cam_mem_util_unmap(int32_t idx)
{
	int rc = 0;
	enum cam_smmu_region_id region = CAM_SMMU_REGION_SHARED;
	enum cam_smmu_mapping_client client;

	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0) {
		CAM_ERR(CAM_MEM, "Incorrect index");
		return;
	}

	client = tbl.bufq[idx].smmu_mapping_client;

	CAM_DBG(CAM_MEM, "Flags = %X idx %d", tbl.bufq[idx].flags, idx);

	if (!tbl.bufq[idx].active) {
		CAM_WARN(CAM_MEM, "Buffer at idx=%d is already unmapped", idx);
		return;
	}

	/* Deactivate the buffer queue to prevent multiple unmap */
	spin_lock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].active = false;
	tbl.bufq[idx].buf_handle = -1;
	spin_unlock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].release_deferred = false;

	if (tbl.bufq[idx].flags & CAM_MEM_FLAG_KMD_ACCESS) {
		if (tbl.bufq[idx].dma_buf && tbl.bufq[idx].kmdvaddr) {
			rc = cam_mem_util_unmap_cpu_va(tbl.bufq[idx].dma_buf,
				tbl.bufq[idx].kmdvaddr);
			if (rc)
				CAM_ERR(CAM_MEM,
					"Failed, dmabuf=%pK, kmdvaddr=%pK",
					tbl.bufq[idx].dma_buf,
					(void *) tbl.bufq[idx].kmdvaddr);
		}
	}

	/* SHARED flag gets precedence, all other flags after it */
	if (tbl.bufq[idx].flags & CAM_MEM_FLAG_HW_SHARED_ACCESS) {
		region = CAM_SMMU_REGION_SHARED;
	} else {
		if (tbl.bufq[idx].flags & CAM_MEM_FLAG_HW_READ_WRITE)
			region = CAM_SMMU_REGION_IO;
	}

	if ((tbl.bufq[idx].flags & CAM_MEM_FLAG_HW_READ_WRITE) ||
		(tbl.bufq[idx].flags & CAM_MEM_FLAG_HW_SHARED_ACCESS) ||
		(tbl.bufq[idx].flags & CAM_MEM_FLAG_PROTECTED_MODE)) {
		rc = cam_mem_util_unmap_hw_va(idx, region, client, false);
		if (rc)
			CAM_ERR(CAM_MEM, "Failed, dmabuf=%pK",
				tbl.bufq[idx].dma_buf);
	}

	tbl.bufq[idx].flags = 0;

	CAM_DBG(CAM_MEM,
		"Ion buf at idx = %d freeing fd = %d, imported %d, dma_buf %pK, i_ino %lu",
		idx, tbl.bufq[idx].fd, tbl.bufq[idx].is_imported, tbl.bufq[idx].dma_buf,
		tbl.bufq[idx].i_ino);

	if (tbl.bufq[idx].dma_buf)
		dma_buf_put(tbl.bufq[idx].dma_buf);

	tbl.bufq[idx].fd = -1;
	tbl.bufq[idx].i_ino = 0;
	tbl.bufq[idx].dma_buf = NULL;
	tbl.bufq[idx].is_imported = false;
	tbl.bufq[idx].is_internal = false;
	tbl.bufq[idx].len = 0;
	tbl.bufq[idx].num_hdls = 0;
	memset(tbl.bufq[idx].hdls_info, 0x0, tbl.max_hdls_info_size);
	cam_mem_mgr_reset_presil_params(idx);
	memset(&tbl.bufq[idx].timestamp, 0, sizeof(struct timespec64));
	memset(&tbl.bufq[idx].krefcount, 0, sizeof(struct kref));
	memset(&tbl.bufq[idx].urefcount, 0, sizeof(struct kref));

	mutex_lock(&tbl.m_lock);
	clear_bit(idx, tbl.bitmap);
	mutex_unlock(&tbl.m_lock);

}

static void cam_mem_util_unmap_wrapper(struct kref *kref)
{
	int32_t idx;
	struct cam_mem_buf_queue *bufq = container_of(kref, typeof(*bufq), krefcount);

	idx = CAM_MEM_MGR_GET_HDL_IDX(bufq->buf_handle);
	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0) {
		CAM_ERR(CAM_MEM, "idx: %d not valid", idx);
		return;
	}

	cam_mem_util_unmap(idx);
}

void cam_mem_put_cpu_buf(int32_t buf_handle)
{
	int idx;
	uint64_t ms, hrs, min, sec;
	struct timespec64 current_ts;
	uint32_t krefcount = 0, urefcount = 0;
	bool unmap = false;

	if (!buf_handle) {
		CAM_ERR(CAM_MEM, "Invalid buf_handle");
		return;
	}

	idx = CAM_MEM_MGR_GET_HDL_IDX(buf_handle);
	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0) {
		CAM_ERR(CAM_MEM, "idx: %d not valid", idx);
		return;
	}

	mutex_lock(&tbl.bufq[idx].q_lock);
	spin_lock(&tbl.bufq[idx].idx_lock);
	if (!tbl.bufq[idx].active) {
		CAM_ERR(CAM_MEM, "idx: %d not active", idx);
		spin_unlock(&tbl.bufq[idx].idx_lock);
		goto end;
	}

	if (buf_handle != tbl.bufq[idx].buf_handle) {
		CAM_ERR(CAM_MEM, "idx: %d Invalid buf handle %d",
				idx, buf_handle);
		spin_unlock(&tbl.bufq[idx].idx_lock);
		goto end;
	}

	kref_put(&tbl.bufq[idx].krefcount, cam_mem_util_unmap_dummy);

	krefcount = kref_read(&tbl.bufq[idx].krefcount);
	urefcount = kref_read(&tbl.bufq[idx].urefcount);
	spin_unlock(&tbl.bufq[idx].idx_lock);

	if ((krefcount == 1) && (urefcount == 0))
		unmap = true;

	if (unmap) {
		cam_mem_util_unmap(idx);
		CAM_GET_TIMESTAMP(current_ts);
		CAM_CONVERT_TIMESTAMP_FORMAT(current_ts, hrs, min, sec, ms);
		CAM_DBG(CAM_MEM,
			"%llu:%llu:%llu:%llu  Called unmap from here, buf_handle: %u, idx: %d",
			hrs, min, sec, ms, buf_handle, idx);
	} else if (tbl.bufq[idx].release_deferred) {
		CAM_CONVERT_TIMESTAMP_FORMAT((tbl.bufq[idx].timestamp), hrs, min, sec, ms);
		CAM_ERR(CAM_MEM,
			"%llu:%llu:%llu:%llu idx %d fd %d i_ino %lu size %llu active %d buf_handle %d krefCount %d urefCount %d buf_name %s",
			hrs, min, sec, ms, idx, tbl.bufq[idx].fd, tbl.bufq[idx].i_ino,
			tbl.bufq[idx].len, tbl.bufq[idx].active, tbl.bufq[idx].buf_handle,
			krefcount, urefcount, tbl.bufq[idx].buf_name);
		CAM_GET_TIMESTAMP(current_ts);
		CAM_CONVERT_TIMESTAMP_FORMAT(current_ts, hrs, min, sec, ms);
		CAM_ERR(CAM_MEM,
			"%llu:%llu:%llu:%llu  Not unmapping even after defer, buf_handle: %u, idx: %d",
			hrs, min, sec, ms, buf_handle, idx);
	} else if (krefcount == 0) {
		CAM_ERR(CAM_MEM,
			"Unbalanced release Called buf_handle: %u, idx: %d kref: %d",
			tbl.bufq[idx].buf_handle, idx, krefcount);
	}

end:
	mutex_unlock(&tbl.bufq[idx].q_lock);
}
EXPORT_SYMBOL(cam_mem_put_cpu_buf);

void cam_mem_put_kref(int32_t buf_handle)
{
	int idx;
	uint32_t krefcount = 0, urefcount = 0;
	uint64_t ms, hrs, min, sec;

	if (!buf_handle) {
		CAM_ERR(CAM_MEM, "Invalid buf_handle");
		return;
	}

	idx = CAM_MEM_MGR_GET_HDL_IDX(buf_handle);
	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0) {
		CAM_ERR(CAM_MEM, "idx: %d not valid", idx);
		return;
	}

	spin_lock(&tbl.bufq[idx].idx_lock);
	if (tbl.bufq[idx].active && (buf_handle == tbl.bufq[idx].buf_handle)) {
		urefcount = kref_read(&tbl.bufq[idx].urefcount);
		krefcount = kref_read(&tbl.bufq[idx].krefcount);

		if (urefcount == 0) {
			spin_unlock(&tbl.bufq[idx].idx_lock);
			goto warn;
		} else
			kref_put(&tbl.bufq[idx].krefcount, cam_mem_util_unmap_dummy);
	}
	spin_unlock(&tbl.bufq[idx].idx_lock);
	return;
warn:
	CAM_CONVERT_TIMESTAMP_FORMAT((tbl.bufq[idx].timestamp), hrs, min, sec, ms);
	CAM_ERR(CAM_MEM,
		"%llu:%llu:%llu:%llu idx %d fd %d i_ino %lu size %llu active %d buf_handle %d krefCount %d urefCount %d buf_name %s",
		hrs, min, sec, ms, idx, tbl.bufq[idx].fd, tbl.bufq[idx].i_ino,
		tbl.bufq[idx].len, tbl.bufq[idx].active, tbl.bufq[idx].buf_handle,
		krefcount, urefcount, tbl.bufq[idx].buf_name);
	CAM_ERR(CAM_MEM, "Buffer unmap called from UMD before KMD , not unmapping!");

}
EXPORT_SYMBOL(cam_mem_put_kref);

int cam_mem_mgr_release(struct cam_mem_mgr_release_cmd *cmd)
{
	int idx;
	int rc = 0;
	uint64_t ms, hrs, min, sec;
	struct timespec64 current_ts;
	uint32_t krefcount = 0, urefcount = 0;
	bool unmap = false;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	if (!cmd) {
		CAM_ERR(CAM_MEM, "Invalid argument");
		return -EINVAL;
	}

	idx = CAM_MEM_MGR_GET_HDL_IDX(cmd->buf_handle);
	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0) {
		CAM_ERR(CAM_MEM, "Incorrect index %d extracted from mem handle",
			idx);
		return -EINVAL;
	}
	mutex_lock(&tbl.bufq[idx].q_lock);
	if (!tbl.bufq[idx].active) {
		CAM_ERR(CAM_MEM, "Released buffer state should be active");
		rc = -EINVAL;
		goto end;
	}

	if (tbl.bufq[idx].buf_handle != cmd->buf_handle) {
		CAM_ERR(CAM_MEM,
			"Released buf handle %d not matching within table %d, idx=%d",
			cmd->buf_handle, tbl.bufq[idx].buf_handle, idx);
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_MEM, "Releasing hdl = %x, idx = %d", cmd->buf_handle, idx);

	kref_put(&tbl.bufq[idx].urefcount, cam_mem_util_unmap_dummy);

	urefcount = kref_read(&tbl.bufq[idx].urefcount);

	if (tbl.bufq[idx].flags & CAM_MEM_FLAG_KMD_ACCESS) {
		krefcount = kref_read(&tbl.bufq[idx].krefcount);
		if ((krefcount == 1) && (urefcount == 0))
			unmap = true;
	} else {
		if (urefcount == 0)
			unmap = true;
	}

	if (unmap) {
		cam_mem_util_unmap(idx);
		CAM_DBG(CAM_MEM,
			"Called unmap from here, buf_handle: %u, idx: %d", cmd->buf_handle, idx);
	} else if (tbl.bufq[idx].flags & CAM_MEM_FLAG_KMD_ACCESS) {
		rc = -EINVAL;
		CAM_GET_TIMESTAMP(current_ts);
		CAM_CONVERT_TIMESTAMP_FORMAT(current_ts, hrs, min, sec, ms);
		CAM_CONVERT_TIMESTAMP_FORMAT((tbl.bufq[idx].timestamp), hrs, min, sec, ms);
		CAM_ERR(CAM_MEM,
			"%llu:%llu:%llu:%llu idx %d fd %d i_ino %lu size %llu active %d buf_handle %d krefCount %d urefCount %d buf_name %s",
			hrs, min, sec, ms, idx, tbl.bufq[idx].fd, tbl.bufq[idx].i_ino,
			tbl.bufq[idx].len, tbl.bufq[idx].active, tbl.bufq[idx].buf_handle,
			krefcount, urefcount, tbl.bufq[idx].buf_name);
		if (tbl.bufq[idx].release_deferred)
			CAM_ERR(CAM_MEM, "Unbalanced release Called buf_handle: %u, idx: %d",
				tbl.bufq[idx].buf_handle, idx);
		tbl.bufq[idx].release_deferred = true;
	}

end:
	mutex_unlock(&tbl.bufq[idx].q_lock);
	return rc;
}

int cam_mem_mgr_request_mem(struct cam_mem_mgr_request_desc *inp,
	struct cam_mem_mgr_memory_desc *out)
{
	struct dma_buf *buf = NULL;
	int ion_fd = -1, rc = 0;
	uintptr_t kvaddr;
	dma_addr_t iova = 0;
	size_t request_len = 0;
	uint32_t mem_handle;
	int32_t idx;
	int32_t smmu_hdl = 0;
	unsigned long i_ino = 0;

	enum cam_smmu_region_id region = CAM_SMMU_REGION_SHARED;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	if (!inp || !out) {
		CAM_ERR(CAM_MEM, "Invalid params");
		return -EINVAL;
	}

	if (!(inp->flags & CAM_MEM_FLAG_HW_READ_WRITE ||
		inp->flags & CAM_MEM_FLAG_HW_SHARED_ACCESS ||
		inp->flags & CAM_MEM_FLAG_CACHE)) {
		CAM_ERR(CAM_MEM, "Invalid flags for request mem");
		return -EINVAL;
	}

	rc = cam_mem_util_get_dma_buf(inp->size, inp->flags, CAM_MEMMGR_ALLOC_KERNEL, &buf, &i_ino);

	if (rc) {
		CAM_ERR(CAM_MEM, "ION alloc failed for shared buffer");
		goto ion_fail;
	} else if (!buf) {
		CAM_ERR(CAM_MEM, "ION alloc returned NULL buffer");
		goto ion_fail;
	} else {
		CAM_DBG(CAM_MEM, "Got dma_buf = %pK", buf);
	}

	/*
	 * we are mapping kva always here,
	 * update flags so that we do unmap properly
	 */
	inp->flags |= CAM_MEM_FLAG_KMD_ACCESS;
	rc = cam_mem_util_map_cpu_va(buf, &kvaddr, &request_len);
	if (rc) {
		CAM_ERR(CAM_MEM, "Failed to get kernel vaddr");
		goto map_fail;
	}

	if (!inp->smmu_hdl) {
		CAM_ERR(CAM_MEM, "Invalid SMMU handle");
		rc = -EINVAL;
		goto smmu_fail;
	}

	/* SHARED flag gets precedence, all other flags after it */
	if (inp->flags & CAM_MEM_FLAG_HW_SHARED_ACCESS) {
		region = CAM_SMMU_REGION_SHARED;
	} else {
		if (inp->flags & CAM_MEM_FLAG_HW_READ_WRITE)
			region = CAM_SMMU_REGION_IO;
	}

	rc = cam_smmu_map_kernel_iova(inp->smmu_hdl,
		buf,
		CAM_SMMU_MAP_RW,
		&iova,
		&request_len,
		region);

	if (rc < 0) {
		CAM_ERR(CAM_MEM, "SMMU mapping failed");
		goto smmu_fail;
	}

	smmu_hdl = inp->smmu_hdl;

	idx = cam_mem_get_slot();
	if (idx < 0) {
		CAM_ERR(CAM_MEM, "Failed in getting mem slot, idx=%d", idx);
		rc = -ENOMEM;
		cam_mem_mgr_print_tbl();
		goto slot_fail;
	}

	mutex_lock(&tbl.bufq[idx].q_lock);
	mem_handle = GET_MEM_HANDLE(idx, ion_fd);
	tbl.bufq[idx].dma_buf = buf;
	tbl.bufq[idx].fd = -1;
	tbl.bufq[idx].i_ino = i_ino;
	tbl.bufq[idx].flags = inp->flags;
	spin_lock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].buf_handle = mem_handle;
	spin_unlock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].kmdvaddr = kvaddr;

	cam_mem_mgr_update_iova_info_locked(tbl.bufq[idx].hdls_info,
		iova, inp->smmu_hdl, inp->size, true, NULL);

	tbl.bufq[idx].len = inp->size;
	tbl.bufq[idx].num_hdls = 1;
	tbl.bufq[idx].is_imported = false;
	kref_init(&tbl.bufq[idx].krefcount);
	tbl.bufq[idx].smmu_mapping_client = CAM_SMMU_MAPPING_KERNEL;
	mutex_unlock(&tbl.bufq[idx].q_lock);

	out->kva = kvaddr;
	out->iova = (uint32_t)iova;
	out->smmu_hdl = smmu_hdl;
	out->mem_handle = mem_handle;
	out->len = inp->size;
	out->region = region;

	CAM_DBG(CAM_MEM, "idx=%d, dmabuf=%pK, i_ino=%lu, flags=0x%x, mem_handle=0x%x",
		idx, buf, i_ino, inp->flags, mem_handle);

	return rc;
slot_fail:
	cam_smmu_unmap_kernel_iova(inp->smmu_hdl,
		buf, region);
smmu_fail:
	cam_mem_util_unmap_cpu_va(buf, kvaddr);
map_fail:
	dma_buf_put(buf);
ion_fail:
	return rc;
}
EXPORT_SYMBOL(cam_mem_mgr_request_mem);

int cam_mem_mgr_release_mem(struct cam_mem_mgr_memory_desc *inp)
{
	int32_t idx;
	int rc = 0;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	if (!inp) {
		CAM_ERR(CAM_MEM, "Invalid argument");
		return -EINVAL;
	}

	idx = CAM_MEM_MGR_GET_HDL_IDX(inp->mem_handle);
	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0) {
		CAM_ERR(CAM_MEM, "Incorrect index extracted from mem handle");
		return -EINVAL;
	}
	mutex_lock(&tbl.bufq[idx].q_lock);
	if (!tbl.bufq[idx].active) {
		CAM_ERR(CAM_MEM, "Released buffer state should be active");
		rc = -EINVAL;
		goto end;
	}

	if (tbl.bufq[idx].buf_handle != inp->mem_handle) {
		CAM_ERR(CAM_MEM,
			"Released buf handle not matching within table");
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_MEM, "Releasing hdl = %X", inp->mem_handle);
	if (kref_put(&tbl.bufq[idx].krefcount, cam_mem_util_unmap_wrapper))
		CAM_DBG(CAM_MEM,
			"Called unmap from here, buf_handle: %u, idx: %d",
			tbl.bufq[idx].buf_handle, idx);
	else {
		CAM_ERR(CAM_MEM,
			"Unbalanced release Called buf_handle: %u, idx: %d",
			tbl.bufq[idx].buf_handle, idx);
		rc = -EINVAL;
	}
end:
	mutex_unlock(&tbl.bufq[idx].q_lock);
	return rc;
}
EXPORT_SYMBOL(cam_mem_mgr_release_mem);

int cam_mem_mgr_reserve_memory_region(struct cam_mem_mgr_request_desc *inp,
	enum cam_smmu_region_id region,
	struct cam_mem_mgr_memory_desc *out)
{
	struct dma_buf *buf = NULL;
	int rc = 0, ion_fd = -1;
	dma_addr_t iova = 0;
	size_t request_len = 0;
	uint32_t mem_handle;
	int32_t idx;
	int32_t smmu_hdl = 0;
	uintptr_t kvaddr = 0;
	unsigned long i_ino = 0;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	if (!inp || !out) {
		CAM_ERR(CAM_MEM, "Invalid param(s)");
		return -EINVAL;
	}

	if (!inp->smmu_hdl) {
		CAM_ERR(CAM_MEM, "Invalid SMMU handle");
		return -EINVAL;
	}

	if ((region != CAM_SMMU_REGION_SECHEAP) &&
		(region != CAM_SMMU_REGION_FWUNCACHED)) {
		CAM_ERR(CAM_MEM, "Only secondary heap supported");
		return -EINVAL;
	}

	rc = cam_mem_util_get_dma_buf(inp->size, 0, CAM_MEMMGR_ALLOC_KERNEL, &buf, &i_ino);

	if (rc) {
		CAM_ERR(CAM_MEM, "ION alloc failed for sec heap buffer");
		goto ion_fail;
	} else if (!buf) {
		CAM_ERR(CAM_MEM, "ION alloc returned NULL buffer");
		goto ion_fail;
	} else {
		CAM_DBG(CAM_MEM, "Got dma_buf = %pK", buf);
	}

	if (inp->flags & CAM_MEM_FLAG_KMD_ACCESS) {
		rc = cam_mem_util_map_cpu_va(buf, &kvaddr, &request_len);
		if (rc) {
			CAM_ERR(CAM_MEM, "Failed to get kernel vaddr");
			goto kmap_fail;
		}
	}

	rc = cam_smmu_reserve_buf_region(region,
		inp->smmu_hdl, buf, &iova, &request_len);

	if (rc) {
		CAM_ERR(CAM_MEM, "Reserving secondary heap failed");
		goto smmu_fail;
	}

	smmu_hdl = inp->smmu_hdl;

	idx = cam_mem_get_slot();
	if (idx < 0) {
		CAM_ERR(CAM_MEM, "Failed in getting mem slot, idx=%d", idx);
		rc = -ENOMEM;
		cam_mem_mgr_print_tbl();
		goto slot_fail;
	}

	mutex_lock(&tbl.bufq[idx].q_lock);
	mem_handle = GET_MEM_HANDLE(idx, ion_fd);
	tbl.bufq[idx].fd = -1;
	tbl.bufq[idx].i_ino = i_ino;
	tbl.bufq[idx].dma_buf = buf;
	tbl.bufq[idx].flags = inp->flags;
	spin_lock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].buf_handle = mem_handle;
	spin_unlock(&tbl.bufq[idx].idx_lock);
	tbl.bufq[idx].kmdvaddr = kvaddr;

	cam_mem_mgr_update_iova_info_locked(tbl.bufq[idx].hdls_info,
		iova, inp->smmu_hdl, request_len, true, NULL);

	tbl.bufq[idx].len = request_len;
	tbl.bufq[idx].num_hdls = 1;
	tbl.bufq[idx].is_imported = false;
	kref_init(&tbl.bufq[idx].krefcount);
	tbl.bufq[idx].smmu_mapping_client = CAM_SMMU_MAPPING_KERNEL;
	mutex_unlock(&tbl.bufq[idx].q_lock);

	out->kva = kvaddr;
	out->iova = (uint32_t)iova;
	out->smmu_hdl = smmu_hdl;
	out->mem_handle = mem_handle;
	out->len = request_len;
	out->region = region;

	return rc;

slot_fail:
	cam_smmu_release_buf_region(region, smmu_hdl);
smmu_fail:
	if (region == CAM_SMMU_REGION_FWUNCACHED)
		cam_mem_util_unmap_cpu_va(buf, kvaddr);
kmap_fail:
	dma_buf_put(buf);
ion_fail:
	return rc;
}
EXPORT_SYMBOL(cam_mem_mgr_reserve_memory_region);

static void *cam_mem_mgr_user_dump_buf(
	void *dump_struct, uint8_t *addr_ptr)
{
	struct cam_mem_buf_queue          *buf = NULL;
	uint64_t                          *addr;
	int                                i = 0;

	buf = (struct cam_mem_buf_queue *)dump_struct;

	addr = (uint64_t *)addr_ptr;

	*addr++ = buf->timestamp.tv_sec;
	*addr++ = buf->timestamp.tv_nsec / NSEC_PER_USEC;
	*addr++ = buf->fd;
	*addr++ = buf->i_ino;
	*addr++ = buf->buf_handle;
	*addr++ = buf->len;
	*addr++ = buf->align;
	*addr++ = buf->flags;
	*addr++ = buf->kmdvaddr;
	*addr++ = buf->is_imported;
	*addr++ = buf->is_internal;
	*addr++ = buf->num_hdls;
	for (i = 0; i < tbl.max_hdls_supported; i++) {
		if (!buf->hdls_info[i].addr_updated)
			continue;

		*addr++ = buf->hdls_info[i].iommu_hdl;
		*addr++ = buf->hdls_info[i].vaddr;
	}

	return addr;
}

int cam_mem_mgr_dump_user(struct cam_dump_req_cmd *dump_req)
{
	int                             rc = 0;
	int                             i;
	struct cam_common_hw_dump_args  dump_args;
	size_t                          buf_len;
	size_t                          remain_len;
	uint32_t                        min_len;
	uintptr_t                       cpu_addr;

	rc = cam_mem_get_cpu_buf(dump_req->buf_handle,
		&cpu_addr, &buf_len);
	if (rc) {
		CAM_ERR(CAM_MEM, "Invalid handle %u rc %d",
			dump_req->buf_handle, rc);
		return rc;
	}
	if (buf_len <= dump_req->offset) {
		CAM_WARN(CAM_MEM, "Dump buffer overshoot len %zu offset %zu",
			buf_len, dump_req->offset);
		cam_mem_put_cpu_buf(dump_req->buf_handle);
		return -ENOSPC;
	}

	remain_len = buf_len - dump_req->offset;
	min_len =
		(CAM_MEM_BUFQ_MAX *
		(CAM_MEM_MGR_DUMP_BUF_NUM_WORDS * sizeof(uint64_t) +
		sizeof(struct cam_common_hw_dump_header)));

	if (remain_len < min_len) {
		CAM_WARN(CAM_MEM, "Dump buffer exhaust remain %zu min %u",
			remain_len, min_len);
		cam_mem_put_cpu_buf(dump_req->buf_handle);
		return -ENOSPC;
	}

	dump_args.req_id = dump_req->issue_req_id;
	dump_args.cpu_addr = cpu_addr;
	dump_args.buf_len = buf_len;
	dump_args.offset = dump_req->offset;
	dump_args.ctxt_to_hw_map = NULL;

	for (i = 1; i < CAM_MEM_BUFQ_MAX; i++) {
		mutex_lock(&tbl.bufq[i].q_lock);
		if (tbl.bufq[i].active) {
			rc = cam_common_user_dump_helper(&dump_args,
				cam_mem_mgr_user_dump_buf,
				&tbl.bufq[i],
				sizeof(uint64_t), "MEM_MGR_BUF.%d:", i);
			if (rc) {
				CAM_ERR(CAM_CRM,
					"Dump state info failed, rc: %d",
					rc);
				mutex_unlock(&tbl.bufq[i].q_lock);
				return rc;
			}
		}
		mutex_unlock(&tbl.bufq[i].q_lock);
	}

	dump_req->offset = dump_args.offset;
	cam_mem_put_cpu_buf(dump_req->buf_handle);

	return rc;
}


int cam_mem_mgr_free_memory_region(struct cam_mem_mgr_memory_desc *inp)
{
	int32_t rc = 0, idx, entry_idx;

	if (!atomic_read(&cam_mem_mgr_state)) {
		CAM_ERR(CAM_MEM, "failed. mem_mgr not initialized");
		return -EINVAL;
	}

	if (!inp) {
		CAM_ERR(CAM_MEM, "Invalid argument");
		return -EINVAL;
	}

	if ((inp->region != CAM_SMMU_REGION_SECHEAP) &&
		(inp->region != CAM_SMMU_REGION_FWUNCACHED)) {
		CAM_ERR(CAM_MEM, "Only secondary heap supported");
		return -EINVAL;
	}

	idx = CAM_MEM_MGR_GET_HDL_IDX(inp->mem_handle);
	if (idx >= CAM_MEM_BUFQ_MAX || idx <= 0) {
		CAM_ERR(CAM_MEM, "Incorrect index extracted from mem handle");
		return -EINVAL;
	}

	mutex_lock(&tbl.bufq[idx].q_lock);
	if (!tbl.bufq[idx].active) {
		CAM_ERR(CAM_MEM, "Released buffer state should be active");
		rc = -EINVAL;
		goto end;
	}

	if (tbl.bufq[idx].buf_handle != inp->mem_handle) {
		CAM_ERR(CAM_MEM,
			"Released buf handle not matching within table");
		rc = -EINVAL;
		goto end;
	}

	if (tbl.bufq[idx].num_hdls != 1) {
		CAM_ERR(CAM_MEM,
			"Sec heap region should have only one smmu hdl");
		rc = -ENODEV;
		goto end;
	}

	if (!cam_mem_mgr_get_hwva_entry_idx(inp->smmu_hdl, &entry_idx)) {
		CAM_ERR(CAM_MEM,
			"Passed SMMU handle not a valid handle");
		rc = -ENODEV;
		goto end;
	}

	if (inp->smmu_hdl != tbl.bufq[idx].hdls_info[entry_idx].iommu_hdl) {
		CAM_ERR(CAM_MEM,
			"Passed SMMU handle doesn't match with internal hdl");
		rc = -ENODEV;
		goto end;
	}

	rc = cam_smmu_release_buf_region(inp->region, inp->smmu_hdl);
	if (rc) {
		CAM_ERR(CAM_MEM,
			"Sec heap region release failed");
		rc = -ENODEV;
		goto end;
	}

	CAM_DBG(CAM_MEM, "Releasing hdl = %X", inp->mem_handle);
	if (kref_put(&tbl.bufq[idx].krefcount, cam_mem_util_unmap_wrapper))
		CAM_DBG(CAM_MEM,
			"Called unmap from here, buf_handle: %u, idx: %d",
			inp->mem_handle, idx);
	else {
		CAM_ERR(CAM_MEM,
			"Unbalanced release Called buf_handle: %u, idx: %d",
			inp->mem_handle, idx);
		rc = -EINVAL;
	}
end:
	mutex_unlock(&tbl.bufq[idx].q_lock);
	return rc;
}
EXPORT_SYMBOL(cam_mem_mgr_free_memory_region);

#ifdef CONFIG_CAM_PRESIL
struct dma_buf *cam_mem_mgr_get_dma_buf(int fd)
{
	struct dma_buf *dmabuf = NULL;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL((void *)(dmabuf))) {
		CAM_ERR(CAM_MEM, "Failed to import dma_buf for fd");
		return NULL;
	}

	CAM_INFO(CAM_PRESIL, "Received DMA Buf* %pK", dmabuf);

	return dmabuf;
}

int cam_mem_mgr_put_dmabuf_from_fd(uint64_t input_dmabuf)
{
	struct dma_buf *dmabuf = (struct dma_buf *)(uint64_t)input_dmabuf;
	int idx = 0;

	CAM_INFO(CAM_PRESIL, "Received dma_buf :%pK", dmabuf);

	if (!dmabuf) {
		CAM_ERR(CAM_PRESIL, "NULL to import dma_buf fd");
		return -EINVAL;
	}

	for (idx = 0; idx < CAM_MEM_BUFQ_MAX; idx++) {
		if ((tbl.bufq[idx].dma_buf != NULL) && (tbl.bufq[idx].dma_buf == dmabuf)) {
			if (tbl.bufq[idx].presil_params.refcount)
				tbl.bufq[idx].presil_params.refcount--;
			else
				CAM_ERR(CAM_PRESIL, "Unbalanced dmabuf put: %pK", dmabuf);

			if (!tbl.bufq[idx].presil_params.refcount) {
				dma_buf_put(dmabuf);
				cam_mem_mgr_reset_presil_params(idx);
				CAM_DBG(CAM_PRESIL, "Done dma_buf_put for %pK", dmabuf);
			}
		}
	}

	return 0;
}

int cam_mem_mgr_get_fd_from_dmabuf(uint64_t input_dmabuf)
{
	int fd_for_dmabuf = -1;
	struct dma_buf *dmabuf = (struct dma_buf *)(uint64_t)input_dmabuf;
	int idx = 0;

	CAM_DBG(CAM_PRESIL, "Received dma_buf :%pK", dmabuf);

	if (!dmabuf) {
		CAM_ERR(CAM_PRESIL, "NULL to import dma_buf fd");
		return -EINVAL;
	}

	for (idx = 0; idx < CAM_MEM_BUFQ_MAX; idx++) {
		if ((tbl.bufq[idx].dma_buf != NULL) && (tbl.bufq[idx].dma_buf == dmabuf)) {
			CAM_DBG(CAM_PRESIL,
				"Found entry for request from Presil UMD Daemon at %d, dmabuf %pK fd_for_umd_daemon %d refcount: %d",
				idx, tbl.bufq[idx].dma_buf,
				tbl.bufq[idx].presil_params.fd_for_umd_daemon,
				tbl.bufq[idx].presil_params.refcount);

			if (tbl.bufq[idx].presil_params.fd_for_umd_daemon < 0) {
				fd_for_dmabuf = dma_buf_fd(dmabuf, O_CLOEXEC);
				if (fd_for_dmabuf < 0) {
					CAM_ERR(CAM_PRESIL, "get fd fail, fd_for_dmabuf=%d",
						fd_for_dmabuf);
					return -EINVAL;
				}

				tbl.bufq[idx].presil_params.fd_for_umd_daemon = fd_for_dmabuf;
				CAM_INFO(CAM_PRESIL,
					"Received generated idx %d fd_for_dmabuf Buf* %lld", idx,
					fd_for_dmabuf);
			} else {
				fd_for_dmabuf = tbl.bufq[idx].presil_params.fd_for_umd_daemon;
				CAM_INFO(CAM_PRESIL,
					"Received existing at idx %d fd_for_dmabuf Buf* %lld", idx,
					fd_for_dmabuf);
			}

			tbl.bufq[idx].presil_params.refcount++;
		} else {
			CAM_DBG(CAM_MEM,
				"Not found dmabuf at idx=%d, dma_buf %pK handle 0x%0x active %d ",
				idx, tbl.bufq[idx].dma_buf, tbl.bufq[idx].buf_handle,
				tbl.bufq[idx].active);
		}
	}

	return (int)fd_for_dmabuf;
}

int cam_mem_mgr_send_buffer_to_presil(int32_t iommu_hdl, int32_t buf_handle)
{
	int rc = 0;

	/* Sending Presil IO Buf to PC side ( as iova start address indicates) */
	uint64_t io_buf_addr;
	size_t io_buf_size;
	int i, j, fd = -1, idx = 0;
	uint8_t *iova_ptr = NULL;
	uint64_t dmabuf = 0;
	bool is_mapped_in_cb = false;

	CAM_DBG(CAM_PRESIL, "buf handle 0x%0x", buf_handle);

	idx = CAM_MEM_MGR_GET_HDL_IDX(buf_handle);
	for (i = 0; i < tbl.bufq[idx].num_hdl; i++) {
		if (tbl.bufq[idx].hdls[i] == iommu_hdl)
			is_mapped_in_cb = true;
	}

	if (!is_mapped_in_cb) {
		for (j = 0; j < CAM_MEM_BUFQ_MAX; j++) {
			if (tbl.bufq[j].i_ino == tbl.bufq[idx].i_ino) {
				for (i = 0; i < tbl.bufq[j].num_hdl; i++) {
					if (tbl.bufq[j].hdls[i] == iommu_hdl)
						is_mapped_in_cb = true;
				}
			}
		}

		if (!is_mapped_in_cb) {
			CAM_DBG(CAM_PRESIL,
				"Still Could not find idx=%d, FD %d buf_handle 0x%0x",
				idx, GET_FD_FROM_HANDLE(buf_handle), buf_handle);

			/*
			 * Okay to return 0, since this function also gets called for buffers that
			 * are shared only between umd/kmd, these may not be mapped with smmu
			 */
			return 0;
		}
	}

	if ((tbl.bufq[idx].buf_handle != 0) && (tbl.bufq[idx].active) &&
		(tbl.bufq[idx].buf_handle == buf_handle)) {
		CAM_DBG(CAM_PRESIL,
			"Found dmabuf in bufq idx %d, FD %d handle 0x%0x dmabuf %pK",
			idx, tbl.bufq[idx].fd, tbl.bufq[idx].buf_handle, tbl.bufq[idx].dma_buf);
		dmabuf = (uint64_t)tbl.bufq[idx].dma_buf;
		fd = tbl.bufq[idx].fd;
	} else {
		CAM_ERR(CAM_PRESIL,
			"Could not find dmabuf Invalid Mem idx=%d, FD %d handle 0x%0x active %d",
			idx, tbl.bufq[idx].fd, tbl.bufq[idx].buf_handle, tbl.bufq[idx].active);
		return -EINVAL;
	}

	rc = cam_mem_get_io_buf(buf_handle, iommu_hdl, &io_buf_addr, &io_buf_size,
		NULL, NULL);
	if (rc || NULL == (void *)io_buf_addr) {
		CAM_DBG(CAM_PRESIL, "Invalid ioaddr : 0x%x, fd = %d,  dmabuf = %pK",
			io_buf_addr, fd, dmabuf);
		return -EINVAL;
	}

	iova_ptr = (uint8_t *)io_buf_addr;
	CAM_INFO(CAM_PRESIL, "Sending buffer with ioaddr : 0x%x, fd = %d, dmabuf = %pK",
		io_buf_addr, fd, dmabuf);

	rc = cam_presil_send_buffer(dmabuf, 0, 0, (uint32_t)io_buf_size, (uint64_t)iova_ptr);

	return rc;
}

int cam_mem_mgr_send_all_buffers_to_presil(int32_t iommu_hdl)
{
	int idx = 0;
	int rc = 0;
	int32_t fd_already_sent[128];
	int fd_already_sent_count = 0;
	int fd_already_index = 0;
	int fd_already_sent_found = 0;


	memset(&fd_already_sent, 0x0, sizeof(fd_already_sent));

	for (idx = 0; idx < CAM_MEM_BUFQ_MAX; idx++) {
		if ((tbl.bufq[idx].buf_handle != 0) && (tbl.bufq[idx].active)) {
			CAM_DBG(CAM_PRESIL, "Sending %d, FD %d handle 0x%0x", idx, tbl.bufq[idx].fd,
				tbl.bufq[idx].buf_handle);
			fd_already_sent_found = 0;

			for (fd_already_index = 0; fd_already_index < fd_already_sent_count;
				fd_already_index++) {

				if (fd_already_sent[fd_already_index] == tbl.bufq[idx].fd) {
					fd_already_sent_found = 1;
					CAM_DBG(CAM_PRESIL,
						"fd_already_sent %d, FD %d handle 0x%0x flags=0x%0x",
						idx, tbl.bufq[idx].fd, tbl.bufq[idx].buf_handle,
						tbl.bufq[idx].flags);
				}
			}

			if (fd_already_sent_found)
				continue;

			CAM_DBG(CAM_PRESIL, "Sending %d, FD %d handle 0x%0x flags=0x%0x", idx,
				tbl.bufq[idx].fd, tbl.bufq[idx].buf_handle, tbl.bufq[idx].flags);

			rc = cam_mem_mgr_send_buffer_to_presil(iommu_hdl, tbl.bufq[idx].buf_handle);
			fd_already_sent[fd_already_sent_count++] = tbl.bufq[idx].fd;

		} else {
			CAM_DBG(CAM_PRESIL, "Invalid Mem idx=%d, FD %d handle 0x%0x active %d",
				idx, tbl.bufq[idx].fd, tbl.bufq[idx].buf_handle,
				tbl.bufq[idx].active);
		}
	}

	return rc;
}
EXPORT_SYMBOL(cam_mem_mgr_send_all_buffers_to_presil);

int cam_mem_mgr_retrieve_buffer_from_presil(int32_t buf_handle, uint32_t buf_size,
	uint32_t offset, int32_t iommu_hdl)
{
	int rc = 0;

	/* Receive output buffer from Presil IO Buf to PC side (as iova start address indicates) */
	uint64_t io_buf_addr;
	size_t io_buf_size;
	uint64_t dmabuf = 0;
	int fd = 0;
	uint8_t *iova_ptr = NULL;
	int idx = 0;


	CAM_DBG(CAM_PRESIL, "buf handle 0x%0x ", buf_handle);
	rc = cam_mem_get_io_buf(buf_handle, iommu_hdl, &io_buf_addr, &io_buf_size,
		NULL, NULL);
	if (rc) {
		CAM_ERR(CAM_PRESIL, "Unable to get IOVA for buffer buf_hdl: 0x%0x iommu_hdl: 0x%0x",
			buf_handle, iommu_hdl);
		return -EINVAL;
	}

	iova_ptr = (uint8_t *)io_buf_addr;
	iova_ptr += offset;   // correct target address to start writing buffer to.

	if (!buf_size) {
		buf_size = io_buf_size;
		CAM_DBG(CAM_PRESIL, "Updated buf_size from Zero to 0x%0x", buf_size);
	}

	fd = GET_FD_FROM_HANDLE(buf_handle);

	idx = CAM_MEM_MGR_GET_HDL_IDX(buf_handle);
	if ((tbl.bufq[idx].buf_handle != 0) && (tbl.bufq[idx].active) &&
		(tbl.bufq[idx].buf_handle == buf_handle)) {
		CAM_DBG(CAM_PRESIL, "Found dmabuf in bufq idx %d, FD %d handle 0x%0x dmabuf %pK",
			idx, tbl.bufq[idx].fd, tbl.bufq[idx].buf_handle, tbl.bufq[idx].dma_buf);
		dmabuf = (uint64_t)tbl.bufq[idx].dma_buf;
	} else {
		CAM_ERR(CAM_PRESIL,
			"Could not find dmabuf Invalid Mem idx=%d, FD %d handle 0x%0x active %d ",
			idx, tbl.bufq[idx].fd, tbl.bufq[idx].buf_handle, tbl.bufq[idx].active);
	}

	CAM_DBG(CAM_PRESIL,
		"Retrieving buffer with ioaddr : 0x%x, offset = %d, size = %d, fd = %d, dmabuf = %pK",
		io_buf_addr, offset, buf_size, fd, dmabuf);

	rc = cam_presil_retrieve_buffer(dmabuf, 0, 0, (uint32_t)buf_size, (uint64_t)io_buf_addr);

	CAM_INFO(CAM_PRESIL,
		"Retrieved buffer with ioaddr : 0x%x, offset = %d, size = %d, fd = %d, dmabuf = %pK",
		io_buf_addr, 0, buf_size, fd, dmabuf);

	return rc;
}

#else /* ifdef CONFIG_CAM_PRESIL */
struct dma_buf * cam_mem_mgr_get_dma_buf(int fd)
{
	return NULL;
}

int cam_mem_mgr_send_all_buffers_to_presil(int32_t iommu_hdl)
{
       return 0;
}

int cam_mem_mgr_send_buffer_to_presil(int32_t iommu_hdl, int32_t buf_handle)
{
	return 0;
}

int cam_mem_mgr_retrieve_buffer_from_presil(int32_t buf_handle,
	uint32_t buf_size,
	uint32_t offset,
	int32_t iommu_hdl)
{
	return 0;
}
#endif /* ifdef CONFIG_CAM_PRESIL */
