// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */


#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/ktime.h>
#include <linux/of_address.h>
#include <linux/pm_domain.h>
#include <linux/version.h>
#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
#include <linux/firmware/qcom/qcom_scm.h>
#else
#include <linux/qcom_scm.h>
#endif
#ifdef CONFIG_QCOM_KGSL_UPSTREAM
#include <linux/firmware/qcom/qcom_scm_addon.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/string.h>

#include "adreno.h"
#include "kgsl_util.h"

bool kgsl_genpd_is_enabled(struct device *dev)
{
	struct generic_pm_domain *genpd;

	if (IS_ERR_OR_NULL(dev) || IS_ERR_OR_NULL(dev->pm_domain))
		return false;

	genpd = pd_to_genpd(dev->pm_domain);
	return (genpd->status == GENPD_STATE_ON);
}

struct clk *kgsl_of_clk_by_name(struct clk_bulk_data *clks, int count,
		const char *id)
{
	int i;

	for (i = 0; clks && i < count; i++)
		if (!strcmp(clks[i].id, id))
			return clks[i].clk;

	return NULL;
}

int kgsl_regulator_set_voltage(struct device *dev,
		struct regulator *reg, u32 voltage)
{
	int ret;

	if (IS_ERR_OR_NULL(reg))
		return 0;

	ret = regulator_set_voltage(reg, voltage, INT_MAX);
	if (ret)
		dev_err(dev, "Regulator set voltage:%d failed:%d\n", voltage, ret);

	return ret;
}

int kgsl_clk_set_rate(struct clk_bulk_data *clks, int num_clks,
		const char *id, unsigned long rate)
{
	struct clk *clk;

	clk = kgsl_of_clk_by_name(clks, num_clks, id);
	if (!clk)
		return -ENODEV;

	return clk_set_rate(clk, rate);
}

#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
int kgsl_scm_gpu_init_regs(struct device *dev, u32 gpu_req)
{
	int ret;

	if (!gpu_req)
		return -EOPNOTSUPP;

	ret = qcom_scm_kgsl_init_regs(gpu_req);
	if (ret)
		dev_err(dev, "Scm call for requests:0x%x failed with ret:: %d\n",
									gpu_req, ret);

	return ret;
}
#endif

/*
 * The PASID has stayed consistent across all targets thus far so we are
 * cautiously optimistic that we can hard code it
 */
#define GPU_PASID 13

int kgsl_zap_shader_load(struct device *dev, const char *name)
{
	struct device_node *np, *mem_np;
	const struct firmware *fw;
	void *mem_region = NULL;
	phys_addr_t mem_phys;
	struct resource res;
	ssize_t mem_size;
	int ret;

	np = of_get_child_by_name(dev->of_node, "zap-shader");
	if (!np) {
		dev_err(dev, "zap-shader node not found. Please update the device tree\n");
		return -ENODEV;
	}

	mem_np = of_parse_phandle(np, "memory-region", 0);
	of_node_put(np);
	if (!mem_np) {
		dev_err(dev, "Couldn't parse the mem-region from the zap-shader node\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(mem_np, 0, &res);
	of_node_put(mem_np);
	if (ret)
		return ret;

	ret = request_firmware(&fw, name, dev);
	if (ret) {
		dev_err(dev, "Couldn't load the firmware %s\n", name);
		return ret;
	}

	mem_size = qcom_mdt_get_size(fw);
	if (mem_size < 0) {
		ret = mem_size;
		goto out;
	}

	if (mem_size > resource_size(&res)) {
		ret = -E2BIG;
		goto out;
	}

	mem_phys = res.start;

	mem_region = memremap(mem_phys, mem_size, MEMREMAP_WC);
	if (!mem_region) {
		ret = -ENOMEM;
		goto out;
	}

	ret = qcom_mdt_load(dev, fw, name, GPU_PASID, mem_region,
		mem_phys, mem_size, NULL);
	if (ret) {
		dev_err(dev, "Error %d while loading the MDT\n", ret);
		goto out;
	}

	ret = qcom_scm_pas_auth_and_reset(GPU_PASID);

out:
	if (mem_region)
		memunmap(mem_region);

	release_firmware(fw);
	return ret;
}

int kgsl_zap_shader_unload(struct device *dev)
{
	int ret;

	ret = qcom_scm_pas_shutdown_retry(GPU_PASID);
	if (ret)
		dev_err(dev, "Error %d while PAS shutdown\n", ret);

	return ret;
}

int kgsl_hwlock(struct cpu_gpu_lock *lock)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);

	/* Indicate that the CPU wants the lock */
	lock->cpu_req = 1;

	/* post the request */
	wmb();

	/* Wait for our turn */
	lock->turn = 0;

	/* Finish all memory transactions before moving on */
	mb();

	/*
	 * Spin here while GPU ucode holds the lock, lock->gpu_req will
	 * be set to 0 after GPU ucode releases the lock. Maximum wait time
	 * is 1 second and this should be enough for GPU to release the lock.
	 */
	while (lock->gpu_req && lock->turn == 0) {
		cpu_relax();
		/* Get the latest updates from GPU */
		rmb();

		if (time_after(jiffies, timeout))
			break;
	}

	if (lock->gpu_req && lock->turn == 0)
		return -EBUSY;

	return 0;
}

void kgsl_hwunlock(struct cpu_gpu_lock *lock)
{
	/* Make sure all writes are done before releasing the lock */
	wmb();
	lock->cpu_req = 0;
}

int get_ddrtype(void)
{
	int ret;
	u64 ddr_type;
	struct device_node *root_node;
	struct device_node *mem_node = NULL;

	root_node = of_find_node_by_path("/");

	if (root_node == NULL) {
		pr_err("kgsl: Unable to find device tree root node\n");
		return -ENOENT;
	}

	do {
		mem_node = of_get_next_child(root_node, mem_node);
		if (of_node_name_prefix(mem_node, "memory"))
			break;
	} while (mem_node != NULL);

	of_node_put(root_node);
	if (mem_node == NULL) {
		pr_err("kgsl: Unable to find device tree memory node\n");
		return -ENOENT;
	}

	ret = of_property_read_u64(mem_node, "ddr_device_type", &ddr_type);

	of_node_put(mem_node);
	if (ret < 0) {
		pr_err("kgsl: ddr_device_type read failed\n");
		return ret;
	}

	return ddr_type;
}

#if IS_ENABLED(CONFIG_QCOM_VA_MINIDUMP)
#include <soc/qcom/minidump.h>

void kgsl_add_to_minidump(char *name, u64 virt_addr, u64 phy_addr, size_t size)
{
	struct md_region md_entry = {0};
	int ret;

	if (!msm_minidump_enabled())
		return;

	scnprintf(md_entry.name, sizeof(md_entry.name), name);
	md_entry.virt_addr = virt_addr;
	md_entry.phys_addr = phy_addr;
	md_entry.size = size;
	ret = msm_minidump_add_region(&md_entry);
	if (ret < 0 && ret != -EEXIST)
		pr_err("kgsl: Failed to register %s with minidump:%d\n", name, ret);

}

void kgsl_remove_from_minidump(char *name, u64 virt_addr, u64 phy_addr, size_t size)
{
	struct md_region md_entry = {0};
	int ret;

	if (!msm_minidump_enabled())
		return;

	scnprintf(md_entry.name, sizeof(md_entry.name), name);
	md_entry.virt_addr = virt_addr;
	md_entry.phys_addr = phy_addr;
	md_entry.size = size;
	ret = msm_minidump_remove_region(&md_entry);
	if (ret < 0 && ret != -ENOENT)
		pr_err("kgsl: Failed to remove %s from minidump\n", name);
}

int kgsl_add_va_to_minidump(struct device *dev, const char *name, void *ptr,
		size_t size)
{
	struct va_md_entry entry = {0};
	int ret;

	scnprintf(entry.owner, sizeof(entry.owner), name);
	entry.vaddr = (u64)(ptr);
	entry.size = size;
	ret = qcom_va_md_add_region(&entry);
	if (ret < 0)
		dev_err(dev, "Failed to register %s with va_minidump: %d\n", name,
				ret);

	return ret;
}

static int kgsl_add_driver_data_to_va_minidump(struct kgsl_device *device)
{
	int ret;
	char name[MAX_VA_MINIDUMP_STR_LEN];
	struct kgsl_pagetable *pt;
	struct adreno_context *ctxt;
	struct kgsl_process_private *p;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	ret = kgsl_add_va_to_minidump(device->dev, KGSL_DRIVER,
			(void *)(&kgsl_driver), sizeof(struct kgsl_driver));
	if (ret)
		return ret;

	/* hwsched path may not have scratch entry */
	if (device->scratch) {
		ret = kgsl_add_va_to_minidump(device->dev, KGSL_SCRATCH_ENTRY,
				device->scratch->hostptr, device->scratch->size);
		if (ret)
			return ret;
	}

	ret = kgsl_add_va_to_minidump(device->dev, KGSL_MEMSTORE_ENTRY,
			device->memstore->hostptr, device->memstore->size);
	if (ret)
		return ret;

	spin_lock(&adreno_dev->active_list_lock);
	list_for_each_entry(ctxt, &adreno_dev->active_list, active_node) {
		snprintf(name, sizeof(name), KGSL_ADRENO_CTX_ENTRY"_%d", ctxt->base.id);
		ret = kgsl_add_va_to_minidump(device->dev, name,
				(void *)(ctxt), sizeof(struct adreno_context));
		if (ret)
			break;
	}
	spin_unlock(&adreno_dev->active_list_lock);

	read_lock(&kgsl_driver.proclist_lock);
	list_for_each_entry(p, &kgsl_driver.process_list, list) {
		snprintf(name, sizeof(name), KGSL_PROC_PRIV_ENTRY "_%d", pid_nr(p->pid));
		ret = kgsl_add_va_to_minidump(device->dev, name,
				(void *)(p), sizeof(struct kgsl_process_private));
		if (ret)
			break;
	}
	read_unlock(&kgsl_driver.proclist_lock);

	spin_lock(&kgsl_driver.ptlock);
	list_for_each_entry(pt, &kgsl_driver.pagetable_list, list) {
		snprintf(name, sizeof(name), KGSL_PGTABLE_ENTRY"_%d", pt->name);
		ret = kgsl_add_va_to_minidump(device->dev, name,
				(void *)(pt), sizeof(struct kgsl_pagetable));
		if (ret)
			break;
	}
	spin_unlock(&kgsl_driver.ptlock);

	return ret;
}

static int kgsl_va_minidump_callback(struct notifier_block *nb,
		unsigned long action, void *unused)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(kgsl_driver.devp[0]);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);

	if (kgsl_add_driver_data_to_va_minidump(kgsl_driver.devp[0]))
		return NOTIFY_BAD;

	if (gpudev->add_to_va_minidump(adreno_dev))
		return NOTIFY_BAD;

	return NOTIFY_OK;
}

static struct notifier_block kgsl_va_minidump_nb = {
	.priority = INT_MAX,
	.notifier_call = kgsl_va_minidump_callback,
};

void kgsl_qcom_va_md_register(struct kgsl_device *device)
{
	int ret;

	if (!qcom_va_md_enabled())
		return;

	ret = qcom_va_md_register("KGSL", &kgsl_va_minidump_nb);
	if (ret)
		dev_err(device->dev, "Failed to register notifier with va_minidump: %d\n", ret);
}

void kgsl_qcom_va_md_unregister(struct kgsl_device *device)
{
	int ret;

	if (!qcom_va_md_enabled())
		return;

	ret = qcom_va_md_unregister("KGSL", &kgsl_va_minidump_nb);
	if (ret)
		dev_err(device->dev, "Failed to unregister notifier with va_minidump: %d\n", ret);
}
#endif
