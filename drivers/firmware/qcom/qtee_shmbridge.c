// SPDX-License-Identifier: GPL-2.0-only
/*
 * QTI TEE shared memory bridge driver
 *
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "shmbridge: [%d][%s]: " fmt, __LINE__,  __func__

#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/dma-mapping.h>
#include <linux/qtee_shmbridge.h>
#include <linux/of_platform.h>

#define DEFAULT_BRIDGE_SIZE	SZ_4M	/*4M*/
#define MIN_BRIDGE_SIZE		SZ_4K   /*4K*/

#define MAXSHMVMS 4
#define PERM_BITS 3
#define VM_BITS 16
#define VMID_NUM_HYP 0
#define VMID_NUM_HLOS 1
#define SELF_OWNER_BIT 1
#define SHM_NUM_VM_SHIFT 9
#define SHM_VM_MASK 0xFFFF
#define SHM_PERM_MASK 0x7

#define SHMBRIDGE_E_NOT_SUPPORTED 4	/* SHMbridge is not implemented */

#define AC_ERR_SHARED_MEMORY_SINGLE_SOURCE 15

/* ns_vmids */
#define UPDATE_NS_VMIDS(ns_vmids, id)	\
				(((uint64_t)(ns_vmids) << VM_BITS) \
				| ((uint64_t)(id) & SHM_VM_MASK))

/* ns_perms */
#define UPDATE_NS_PERMS(ns_perms, perm)	\
				(((uint64_t)(ns_perms) << PERM_BITS) \
				| ((uint64_t)(perm) & SHM_PERM_MASK))

/* pfn_and_ns_perm_flags = paddr | ns_perms */
#define UPDATE_PFN_AND_NS_PERM_FLAGS(paddr, ns_perms)	\
				((uint64_t)(paddr) | (ns_perms))


/* ipfn_and_s_perm_flags = ipaddr | tz_perm */
#define UPDATE_IPFN_AND_S_PERM_FLAGS(ipaddr, tz_perm)	\
				((uint64_t)(ipaddr) | (uint64_t)(tz_perm))

/* size_and_flags when dest_vm is not HYP */
#define UPDATE_SIZE_AND_FLAGS(size, destnum)	\
				((size) | (destnum) << SHM_NUM_VM_SHIFT)

struct bridge_info {
	phys_addr_t paddr;
	void *vaddr;
	size_t size;
	uint64_t handle;
	int min_alloc_order;
	struct gen_pool *genpool;
	struct device *dev;
};

struct bridge_list {
	struct list_head head;
	struct mutex lock;
};

struct bridge_list_entry {
	struct list_head list;
	phys_addr_t paddr;
	uint64_t handle;
	int32_t ref_count;
};

struct cma_heap_bridge_info {
	uint32_t heapid;
	uint64_t handle;
};

enum CMA_HEAP_TYPE {
	QSEECOM_HEAP = 0,
	QSEECOM_TA_HEAP,
	USER_CONTI_HEAP,
	HEAP_TYPE_MAX
};

static struct bridge_info default_bridge;
static struct bridge_info dma_bridge;
static struct bridge_list bridge_list_head;
static bool qtee_shmbridge_enabled;
static bool support_hyp;
static bool qtee_shmbridge_free_pages_alloc;

/* enable shared memory bridge mechanism in HYP */
static int32_t qtee_shmbridge_enable(bool enable)
{
	int32_t ret = 0;

	qtee_shmbridge_enabled = false;

	/*
	 * Check property to disable shmbridge mechanism and fallback to default
	 * hyp based mechanism.
	 */
	if (of_property_read_bool(default_bridge.dev->of_node,
	    "qcom,disable-shmbridge-support")) {
		return ret;
	}

	if (!enable) {
		pr_warn("shmbridge isn't enabled\n");
		return ret;
	}

	qtee_shmbridge_enabled = true;
	pr_info("shmbridge is enabled\n");
	return ret;
}

/* Check whether shmbridge mechanism is enabled in HYP or not */
bool qtee_shmbridge_is_enabled(void)
{
	return qtee_shmbridge_enabled;
}
EXPORT_SYMBOL_GPL(qtee_shmbridge_is_enabled);

static int32_t qtee_shmbridge_list_add_locked(phys_addr_t paddr,
						uint64_t handle)
{
	struct bridge_list_entry *entry;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;
	entry->handle = handle;
	entry->paddr = paddr;
	entry->ref_count = 0;

	list_add_tail(&entry->list, &bridge_list_head.head);
	return 0;
}

static void qtee_shmbridge_list_del_locked(uint64_t handle)
{
	struct bridge_list_entry *entry;

	list_for_each_entry(entry, &bridge_list_head.head, list) {
		if (entry->handle == handle) {
			list_del(&entry->list);
			kfree(entry);
			break;
		}
	}
}

/**
 * qtee_shmbridge_list_dec_refcount_locked: Decrement the reference count of
 *					registered shmbridge and if refcount reached to zero delete
 *					the shmbridge (i.e send a scm call to tz and remove that
 *					from out local list too.
 *					API suppose to be called in a locked enviorment.
 *
 * Return 0 in case of success else error code in case of failure
 */
static int32_t qtee_shmbridge_list_dec_refcount_locked(uint64_t handle)
{
	struct bridge_list_entry *entry;
	int32_t ret = -EINVAL;

	list_for_each_entry(entry, &bridge_list_head.head, list)
		if (entry->handle == handle) {

			if (entry->ref_count > 0) {
				/* decrement reference count */
				entry->ref_count--;
				pr_debug("bridge on %lld exists decrease refcount :%d\n",
						handle, entry->ref_count);

				if (entry->ref_count == 0) {
				/* All valid reference are freed, it's time to delete the bridge */
					ret = qcom_scm_delete_shm_bridge(handle);
					if (ret) {
						pr_err("Failed to del bridge %lld, ret = %d\n",
								handle, ret);
						/* restore reference count in case of failure */
						entry->ref_count++;
						goto exit;
					}
					qtee_shmbridge_list_del_locked(handle);
				}
				ret = 0;
			} else
				pr_err("ref_count should not be negative handle %lld , refcount: %d\n",
					 handle, entry->ref_count);
			break;
		}
exit:
	if (ret == -EINVAL)
		pr_err("Not able to find bridge handle %lld in map\n", handle);

	return ret;
}

/**
 * qtee_shmbridge_list_inc_refcount_locked: Increment the ref count in case if
 *					we try to register a pre-registered phyaddr with shmbridge
 *					and provide a valid handle to the caller API which was
 *					passed by caller as a pointer.
 *					API suppose to be called in a locked enviorment.
 *
 * Return 0 in case of success else error code in case of failure.
 */
static int32_t qtee_shmbridge_list_inc_refcount_locked(phys_addr_t paddr, uint64_t *handle)
{
	struct bridge_list_entry *entry;
	int32_t ret = -EINVAL;

	list_for_each_entry(entry, &bridge_list_head.head, list)
		if (entry->paddr == paddr) {

			entry->ref_count++;
			pr_debug("%s: bridge on %llx exists increase refcount :%d\n",
				__func__, (uint64_t)paddr, entry->ref_count);

			/* update handle in case we found paddr already exist */
			*handle = entry->handle;
			ret = 0;
			break;
		}
	if (ret)
		pr_err("Not able to find bridge paddr %llx in map\n", (uint64_t)paddr);
	return ret;
}

static int32_t qtee_shmbridge_query_locked(phys_addr_t paddr)
{
	struct bridge_list_entry *entry;

	list_for_each_entry(entry, &bridge_list_head.head, list)
		if (entry->paddr == paddr) {
			pr_debug("A bridge on %llx exists\n", (uint64_t)paddr);
			return -EEXIST;
		}
	return 0;
}

/* Check whether a bridge starting from paddr exists */
int32_t qtee_shmbridge_query(phys_addr_t paddr)
{
	int32_t ret = 0;

	mutex_lock(&bridge_list_head.lock);
	ret = qtee_shmbridge_query_locked(paddr);
	mutex_unlock(&bridge_list_head.lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qtee_shmbridge_query);

/* Register paddr & size as a bridge, return bridge handle */
int32_t qtee_shmbridge_register(
		phys_addr_t paddr,
		size_t size,
		uint32_t *ns_vmid_list,
		uint32_t *ns_vm_perm_list,
		uint32_t ns_vmid_num,
		uint32_t tz_perm,
		uint64_t *handle)

{
	int32_t ret = 0;
	uint64_t pfn_and_ns_perm_flags = 0;
	uint64_t ipfn_and_s_perm_flags = 0;
	uint64_t size_and_flags = 0;
	uint64_t ns_perms = 0;
	uint64_t ns_vmids = 0;
	int i = 0;

	if (!qtee_shmbridge_enabled)
		return 0;

	if (!handle || !ns_vmid_list || !ns_vm_perm_list ||
				ns_vmid_num > MAXSHMVMS) {
		pr_err("invalid input parameters\n");
		return -EINVAL;
	}

	mutex_lock(&bridge_list_head.lock);
	ret = qtee_shmbridge_query_locked(paddr);
	if (ret) {
		pr_debug("found 0%llu already exist with shmbridge\n", paddr);
		goto bridge_exist;
	}

	for (i = 0; i < ns_vmid_num; i++) {
		ns_perms = UPDATE_NS_PERMS(ns_perms, ns_vm_perm_list[i]);
		ns_vmids = UPDATE_NS_VMIDS(ns_vmids, ns_vmid_list[i]);
	}

	pfn_and_ns_perm_flags = UPDATE_PFN_AND_NS_PERM_FLAGS(paddr, ns_perms);
	ipfn_and_s_perm_flags = UPDATE_IPFN_AND_S_PERM_FLAGS(paddr, tz_perm);
	size_and_flags = UPDATE_SIZE_AND_FLAGS(size, ns_vmid_num);

	if (support_hyp) {
		size_and_flags |= SELF_OWNER_BIT << 1;
		size_and_flags |= QCOM_SCM_PERM_RW << 2;
	}

	pr_debug("%s: desc.args[0] %llx, args[1] %llx, args[2] %llx, args[3] %llx\n",
		__func__, pfn_and_ns_perm_flags, ipfn_and_s_perm_flags,
		size_and_flags, ns_vmids);

	ret = qcom_scm_create_shm_bridge(pfn_and_ns_perm_flags,
			ipfn_and_s_perm_flags, size_and_flags, ns_vmids,
			handle);

	if (ret) {
		pr_err("Shm creation failed, ret: %d, NS PA|Perm: 0x%llx, size|flags: 0x%llx, ns_vmids: 0x%llx\n",
			ret, pfn_and_ns_perm_flags, size_and_flags, ns_vmids);

		/*
		 * If bridge is already existing and we are not real owner also paddr
		 * not exist in our map we will add an entry in our map and go for
		 * deregister for this since QTEE also maintain ref_count. So for this
		 * we should deregister to decrease ref_count in QTEE.
		 */
		if (ret == AC_ERR_SHARED_MEMORY_SINGLE_SOURCE)
			pr_err("bridge %llu exist but not registered in our map\n", paddr);
		else {
			ret = -EINVAL;
			goto exit;
		}
	}

	ret = qtee_shmbridge_list_add_locked(paddr, *handle);
bridge_exist:
	ret = qtee_shmbridge_list_inc_refcount_locked(paddr, handle);
exit:
	mutex_unlock(&bridge_list_head.lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qtee_shmbridge_register);

/* Deregister bridge */
int32_t qtee_shmbridge_deregister(uint64_t handle)
{
	int32_t ret = 0;

	if (!qtee_shmbridge_enabled)
		return 0;

	mutex_lock(&bridge_list_head.lock);
	ret = qtee_shmbridge_list_dec_refcount_locked(handle);
	mutex_unlock(&bridge_list_head.lock);

	return ret;
}
EXPORT_SYMBOL_GPL(qtee_shmbridge_deregister);


/* Sub-allocate from default kernel bridge created by shmb driver */
int32_t qtee_shmbridge_allocate_shm(size_t size, struct qtee_shm *shm)
{
	int32_t ret = 0;
	unsigned long va;

	if (IS_ERR_OR_NULL(shm)) {
		pr_err("qtee_shm is NULL\n");
		ret = -EINVAL;
		goto exit;
	}

	if (size > default_bridge.size) {
		pr_err("requestd size %zu is larger than bridge size %zu\n",
			size, default_bridge.size);
		ret = -EINVAL;
		goto exit;
	}

	if (!default_bridge.genpool) {
		pr_err("Shmbridge pool not available!\n");
		ret = -ENOMEM;
		goto exit;
	}

	size = roundup(size, 1 << default_bridge.min_alloc_order);

	va = gen_pool_alloc(default_bridge.genpool, size);
	if (!va) {
		pr_err("failed to sub-allocate %zu bytes from bridge\n", size);
		ret = -ENOMEM;
		goto exit;
	}

	memset((void *)va, 0, size);
	shm->vaddr = (void *)va;
	shm->paddr = gen_pool_virt_to_phys(default_bridge.genpool, va);
	shm->size = size;

	pr_debug("%s: shm->paddr %llx, size %zu\n",
			__func__, (uint64_t)shm->paddr, shm->size);

exit:
	return ret;
}
EXPORT_SYMBOL_GPL(qtee_shmbridge_allocate_shm);


/* Free buffer that is sub-allocated from default kernel bridge */
void qtee_shmbridge_free_shm(struct qtee_shm *shm)
{
	if (IS_ERR_OR_NULL(shm) || !shm->vaddr)
		return;
	gen_pool_free(default_bridge.genpool, (unsigned long)shm->vaddr,
		      shm->size);
}
EXPORT_SYMBOL_GPL(qtee_shmbridge_free_shm);

/* cache clean operation for buffer sub-allocated from default bridge */
void qtee_shmbridge_flush_shm_buf(struct qtee_shm *shm)
{
	if (shm)
		return dma_sync_single_for_device(default_bridge.dev,
				shm->paddr, shm->size, DMA_TO_DEVICE);
}
EXPORT_SYMBOL_GPL(qtee_shmbridge_flush_shm_buf);

/* cache invalidation operation for buffer sub-allocated from default bridge */
void qtee_shmbridge_inv_shm_buf(struct qtee_shm *shm)
{
	if (shm)
		return dma_sync_single_for_cpu(default_bridge.dev,
				shm->paddr, shm->size, DMA_FROM_DEVICE);
}
EXPORT_SYMBOL_GPL(qtee_shmbridge_inv_shm_buf);

/*
 * shared memory bridge initialization
 *
 */
static int qtee_shmbridge_init(struct platform_device *pdev)
{
	int ret = 0;
	uint32_t custom_bridge_size, ns_vm_nums;
	uint32_t *ns_vm_ids;
	uint32_t ns_vm_ids_hlos[] = {QCOM_SCM_VMID_HLOS};
	uint32_t ns_vm_ids_hyp[] = {};
	uint32_t ns_vm_perms[] = {QCOM_SCM_PERM_RW};
	struct device_node *mem_node;
	struct reserved_mem *rmem;

	support_hyp = of_property_read_bool((&pdev->dev)->of_node,
			"qcom,support-hypervisor");
	if (support_hyp) {
		ns_vm_ids = ns_vm_ids_hyp;
		ns_vm_nums = VMID_NUM_HYP;
	} else {
		ns_vm_ids = ns_vm_ids_hlos;
		ns_vm_nums = VMID_NUM_HLOS;
	}

	if (default_bridge.vaddr) {
		pr_err("qtee shmbridge is already initialized\n");
		return 0;
	}

	ret = of_property_read_u32((&pdev->dev)->of_node,
		"qcom,custom-bridge-size", &custom_bridge_size);
	if (ret)
		default_bridge.size = DEFAULT_BRIDGE_SIZE;
	else
		default_bridge.size = custom_bridge_size * MIN_BRIDGE_SIZE;

	pr_info("qtee shmbridge registered default bridge with size %zu bytes\n",
		default_bridge.size);

	/*
	 * First try to allocate memory via free_pages speciying DMA capabilities
	 * as 32bit(GFP_DMA32) by default as _scm->dev is also based on 32bit.
	 *
	 * In case of failures, try to allocate with dma_alloc_coherent.
	 */
	default_bridge.vaddr = (void *)__get_free_pages(GFP_KERNEL|__GFP_COMP|GFP_DMA32,
				get_order(default_bridge.size));

	if (default_bridge.vaddr) {
		default_bridge.paddr = dma_map_single(&pdev->dev,
				default_bridge.vaddr, default_bridge.size,
				DMA_TO_DEVICE);
		if (dma_mapping_error(&pdev->dev, default_bridge.paddr)) {
			pr_err("dma_map_single() failed\n");
			ret = -ENOMEM;
			goto exit_freebuf;
		}
		qtee_shmbridge_free_pages_alloc = true;
	} else {
		pr_info("Allocation via free_pages failed, trying with dma_alloc now..\n");
		default_bridge.vaddr = dma_alloc_coherent(&pdev->dev, default_bridge.size,
								&default_bridge.paddr, GFP_KERNEL);
		if (!default_bridge.vaddr)
			return -ENOMEM;
	}
	default_bridge.dev = &pdev->dev;

	/* create a general mem pool */
	default_bridge.min_alloc_order = PAGE_SHIFT; /* 4K page size aligned */
	default_bridge.genpool = gen_pool_create(
					default_bridge.min_alloc_order, -1);
	if (!default_bridge.genpool) {
		pr_err("gen_pool_add_virt() failed\n");
		ret = -ENOMEM;
		goto exit_unmap;
	}

	gen_pool_set_algo(default_bridge.genpool, gen_pool_best_fit, NULL);
	ret = gen_pool_add_virt(default_bridge.genpool,
			(uintptr_t)default_bridge.vaddr,
				default_bridge.paddr, default_bridge.size, -1);
	if (ret) {
		pr_err("gen_pool_add_virt() failed, ret = %d\n", ret);
		goto exit_destroy_pool;
	}

	mutex_init(&bridge_list_head.lock);
	INIT_LIST_HEAD(&bridge_list_head.head);

	/* temporarily disable shm bridge mechanism */
	ret = qtee_shmbridge_enable(true);
	if (ret) {
		/* keep the mem pool and return if failed to enable bridge */
		ret = 0;
		goto exit;
	}

	/*register default bridge*/
	ret = qtee_shmbridge_register(default_bridge.paddr,
			default_bridge.size, ns_vm_ids,
			ns_vm_perms, ns_vm_nums, QCOM_SCM_PERM_RW,
			&default_bridge.handle);

	if (ret) {
		pr_err("Failed to register default bridge, size %zu\n",
			default_bridge.size);
		goto exit_deregister_default_bridge;
	}

	pr_debug("qtee shmbridge registered default bridge with size %zu bytes\n",
			default_bridge.size);

	mem_node = of_parse_phandle((&pdev->dev)->of_node, "memory-region", 0);
	if (!mem_node) {
		pr_info("%s: Could not parse memory region for DMA bridge, skipping\n", __func__);
		goto exit_success;
	}

	rmem = of_reserved_mem_lookup(mem_node);
	of_node_put(mem_node);
	if (!rmem) {
		pr_info("%s: Failed to get memory-region node, skipping\n", __func__);
		goto exit_success;
	}

	dma_bridge.paddr = rmem->base;
	dma_bridge.size = rmem->size;

	/* register dma_bridge */
	ret = qtee_shmbridge_register(dma_bridge.paddr,
			dma_bridge.size, ns_vm_ids,
			ns_vm_perms, ns_vm_nums, QCOM_SCM_PERM_RW,
			&dma_bridge.handle);

	if (ret) {
		pr_err("Failed to register dma bridge, size %zu\n",
			dma_bridge.size);
		goto exit_deregister_dma_bridge;
	}

exit_success:
	return 0;

exit_deregister_dma_bridge:
	qtee_shmbridge_deregister(dma_bridge.handle);
exit_deregister_default_bridge:
	qtee_shmbridge_deregister(default_bridge.handle);
	qtee_shmbridge_enable(false);
exit_destroy_pool:
	gen_pool_destroy(default_bridge.genpool);
exit_unmap:
	if (qtee_shmbridge_free_pages_alloc)
		dma_unmap_single(&pdev->dev, default_bridge.paddr, default_bridge.size,
			DMA_TO_DEVICE);
exit_freebuf:
	if (qtee_shmbridge_free_pages_alloc)
		free_pages((long)default_bridge.vaddr, get_order(default_bridge.size));
	else
		dma_free_coherent(&pdev->dev, default_bridge.size,
			default_bridge.vaddr, default_bridge.paddr);

	default_bridge.vaddr = NULL;
exit:
	return ret;
}

static int qtee_shmbridge_probe(struct platform_device *pdev)
{
	/* Defer if qcom_scm is not available */
	if (!qcom_scm_is_available())
		return dev_err_probe(&pdev->dev, -EPROBE_DEFER, "qcom_scm is not up!\n");

	return qtee_shmbridge_init(pdev);
}

static int qtee_shmbridge_remove(struct platform_device *pdev)
{
	qtee_shmbridge_deregister(default_bridge.handle);
	gen_pool_destroy(default_bridge.genpool);
	if (qtee_shmbridge_free_pages_alloc) {
		dma_unmap_single(&pdev->dev, default_bridge.paddr, default_bridge.size,
			DMA_TO_DEVICE);
		free_pages((long)default_bridge.vaddr, get_order(default_bridge.size));
	} else {
		dma_free_coherent(&pdev->dev, default_bridge.size,
			default_bridge.vaddr, default_bridge.paddr);
	}
	return 0;
}

static const struct of_device_id qtee_shmbridge_of_match[] = {
	{ .compatible = "qcom,tee-shared-memory-bridge"},
	{}
};
MODULE_DEVICE_TABLE(of, qtee_shmbridge_of_match);

static struct platform_driver qtee_shmbridge_driver = {
	.probe = qtee_shmbridge_probe,
	.remove = qtee_shmbridge_remove,
	.driver = {
		.name = "shared_memory_bridge",
		.of_match_table = qtee_shmbridge_of_match,
	},
};

static int __init qtee_shmbridge_driver_init(void)
{
	return platform_driver_register(&qtee_shmbridge_driver);
}
/*
 * Keep shmbridge init level same as qcom_scm so that it's available after
 * qcom_scm and well before loading other dependent modules.
 */
subsys_initcall(qtee_shmbridge_driver_init);

static void __exit qtee_shmbridge_driver_exit(void)
{
	platform_driver_unregister(&qtee_shmbridge_driver);
}
module_exit(qtee_shmbridge_driver_exit);

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. SHM Bridge driver");
MODULE_LICENSE("GPL");
