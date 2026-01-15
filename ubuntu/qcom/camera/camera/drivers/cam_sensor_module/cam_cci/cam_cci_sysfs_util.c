// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <media/cam_sensor.h>
#include "cam_debug_util.h"
#include "cam_sensor_cmn_header.h"
#include "cam_sensor_io.h"
#include "cam_sensor_i2c.h"
#include "cam_sensor_dev.h"
#include "cam_cci_dev.h"
#include "cam_sensor_core.h"

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/fs.h>

#define TOTAL_BUFFER_SIZE       2000
#define INFO_BUFFER_SIZE        200
#define NUM_OF_CCI_READ_PARAMS      4
#define NUM_OF_CCI_WRITE_PARAMS     6
#define USAGE_STRING   "Read format: r/R, reg_addr(hex, e.g., 0x300a), addr_type, "\
	"data_type, slave_addr(8-bit write address in hex, e.g., 0xc0)\n"\
	"Write format: w/W, reg_addr(hex, e.g., 0x300a), reg_value(hex), "\
	"delay, addr_type, data_type, slave_addr(8-bit write address in hex, e.g., 0xc0)\n"



struct cam_sensor_pwr_sysfs_attr {
	/** @attr: Attribute for the sysfs node */
	struct attribute attr;
	/** @show: Show function for the node */
	ssize_t (*show)(struct cam_sensor_ctrl_t *device, char *buf);
	/** @store: Store function for the node */
	ssize_t (*store)(struct cam_sensor_ctrl_t *device, const char *buf,
			size_t count);
};

#define CAM_SENSOR_PWR_SYSFS_ATTR(_name, _mode, _show, _store)		\
	const struct cam_sensor_pwr_sysfs_attr cam_sensor_pwr_sysfs_attr_##_name = {		\
			.attr = { .name = __stringify(_name), .mode = _mode }, 			\
			.show = _show,								\
			.store = _store,							\
	}


struct cci_sysfs_attr {
		/** @attr: Attribute for the sysfs node */
		struct attribute attr;
		/** @show: Show function for the node */
		ssize_t (*show)(struct cci_sysfs *device, char *buf);
		/** @store: Store function for the node */
		ssize_t (*store)(struct cci_sysfs *device, const char *buf,
						size_t count);
};

#define CCI_SYSFS_ATTR(_name, _mode, _show, _store)		\
	const struct cci_sysfs_attr cci_sysfs_attr_##_name = {		\
			.attr = { .name = __stringify(_name), .mode = _mode },	\
			.show = _show,						\
			.store = _store,					\
	}

static char *display_cci_buf[MAX_CCI*MASTER_MAX] = {0};

static struct kobject *cam_cci_base_kobject;
static struct kobject *cam_pwr_base_kobject;


int32_t sysfs_cam_sensor_power_up(void *ctrl)
{
	int rc;
	struct cam_sensor_ctrl_t *sensor = (struct cam_sensor_ctrl_t *) ctrl;
	if (sensor && sensor->sysfs_state == false) {

		CAM_DBG(CAM_CCI, "cam_sensor_power_up %d", sensor->soc_info.index);
		mutex_lock(&(sensor->cam_sensor_mutex));
		rc = cam_sensor_power_up(sensor);
		if (rc < 0) {
			CAM_ERR(CAM_CCI,
					"Power up failed for %s sensor_id: 0x%x, slave_addr: 0x%x",
					sensor->sensor_name,
					sensor->sensordata->slave_info.sensor_id,
					sensor->sensordata->slave_info.sensor_slave_addr);
		} else {
			sensor->sysfs_state = true;
		}
		mutex_unlock(&(sensor->cam_sensor_mutex));
	}
	return 0;
}

int32_t sysfs_cam_sensor_power_down(void *ctrl)
{
	int rc;

	struct cam_sensor_ctrl_t *sensor = (struct cam_sensor_ctrl_t *) ctrl;

	if (sensor && sensor->sysfs_state == true) {
		CAM_DBG(CAM_CCI, "cam_sensor_power_down %d", sensor->soc_info.index);
		mutex_lock(&(sensor->cam_sensor_mutex));
		rc = cam_sensor_power_down(sensor);
		if (rc < 0) {
			CAM_ERR(CAM_CCI,
					"Power down failed for %s sensor_id: 0x%x, slave_addr: 0x%x",
					sensor->sensor_name,
					sensor->sensordata->slave_info.sensor_id,
					sensor->sensordata->slave_info.sensor_slave_addr);
		} else {
			sensor->sysfs_state = false;
		}
		mutex_unlock(&(sensor->cam_sensor_mutex));
	}
	return 0;
}

static ssize_t sensor_pwr_show(struct cam_sensor_ctrl_t	*sensor, char *buf)
{
	int  count = 0;

	if (sensor) {
		mutex_lock(&(sensor->cam_sensor_mutex));
		count = scnprintf(buf, PAGE_SIZE,
				"SENSOR STATE=%d POWER STATE=%d\n",
				sensor->sensor_state, sensor->sysfs_state);

		mutex_unlock(&(sensor->cam_sensor_mutex));
	}

	return count;
}

static ssize_t sensor_pwr_store(struct cam_sensor_ctrl_t *ctrl,
		const char *buf, size_t count)
{
	int val = 0;

	if (ctrl == NULL) {
		CAM_ERR(CAM_CCI, "INVALID SESNOR ");
		return 0;
	}

	sscanf(buf, "%d", &val);

	if (val == 1)
		sysfs_cam_sensor_power_up(ctrl);
	else if (val == 0)
		sysfs_cam_sensor_power_down(ctrl);
	else
		CAM_ERR(CAM_CCI, "INVALID VALUE ");

	return count;
}

static ssize_t sensor_name_show(struct cam_sensor_ctrl_t *sensor, char *buf)
{
	int  count = 0;

	if (sensor) {
		mutex_lock(&(sensor->cam_sensor_mutex));

		CAM_DBG(CAM_CCI, "cci_id=%d cci_master=%d",
				sensor->io_master_info.cci_client->cci_device,
				sensor->io_master_info.cci_client->cci_i2c_master);

		count = scnprintf(buf, PAGE_SIZE, "%s_cam-cci%d_master%d\n",
				sensor->sensor_name,
				sensor->io_master_info.cci_client->cci_device,
				sensor->io_master_info.cci_client->cci_i2c_master);

		mutex_unlock(&(sensor->cam_sensor_mutex));
		return count;
	}

	return count;
}

static ssize_t cam_sensor_pwr_sysfs_attr_show(struct kobject *kobj,
		struct attribute *__attr, char *buf)
{
	struct cam_sensor_pwr_sysfs_attr *attr = container_of(__attr,
			struct cam_sensor_pwr_sysfs_attr, attr);
	struct cam_sensor_ctrl_t *device = container_of(kobj,
			struct cam_sensor_ctrl_t, sysfs_kobj);

	if (attr->show)
		return attr->show(device, buf);

	return -EIO;
}

static ssize_t cam_sensor_pwr_sysfs_attr_store(struct kobject *kobj,
		struct attribute *__attr, const char *buf, size_t count)
{
	struct cam_sensor_pwr_sysfs_attr *attr = container_of(__attr,
			struct cam_sensor_pwr_sysfs_attr, attr);
	struct cam_sensor_ctrl_t *device = container_of(kobj,
			struct cam_sensor_ctrl_t, sysfs_kobj);

	if (attr->store)
		return attr->store(device, buf, count);

	return -EIO;
}

/* Dummy release function - we have nothing to do here */
static void cam_sensor_pwr_sysfs_release(struct kobject *kobj)
{
}

static const struct sysfs_ops cam_sensor_pwr_sysfs_ops = {
	.show = cam_sensor_pwr_sysfs_attr_show,
	.store = cam_sensor_pwr_sysfs_attr_store,
};

static struct kobj_type cam_sensor_pwr_sysfs_ktype = {
	.sysfs_ops = &cam_sensor_pwr_sysfs_ops,
	.release = cam_sensor_pwr_sysfs_release,
};

static CAM_SENSOR_PWR_SYSFS_ATTR(pwr_enable, 0660, sensor_pwr_show, sensor_pwr_store);
static CAM_SENSOR_PWR_SYSFS_ATTR(name, 0440, sensor_name_show, NULL);

static const struct attribute *cam_sensor_pwr_sysfs_attr_list[] = {
	&cam_sensor_pwr_sysfs_attr_pwr_enable.attr,
	&cam_sensor_pwr_sysfs_attr_name.attr,
	NULL,
};

int32_t cam_sensor_add_device(void *ctrl_struct)
{
	char buff[7];
	int error;

	struct cam_sensor_ctrl_t *sensor =
		(struct cam_sensor_ctrl_t *) ctrl_struct;

	if (NULL == sensor) {
		CAM_ERR(CAM_CCI, "Invalid/NULL arguments");
		return -EINVAL;
	}

	CAM_DBG(CAM_CCI, "SLOT_id=%d Name=%s %s cci_id=%d cci_master=%d",
			sensor->soc_info.index,
			sensor->device_name,
			sensor->sensor_name,
			sensor->io_master_info.cci_client->cci_device,
			sensor->io_master_info.
				cci_client->cci_i2c_master);

	if (!cam_pwr_base_kobject) {
		cam_pwr_base_kobject = kobject_create_and_add("cam-sensor",
				kernel_kobj);
		if (!cam_pwr_base_kobject) {
			return -ENOMEM;
		}
	}

	scnprintf(buff, sizeof(buff), "slot%d",
			sensor->soc_info.index);

	error = kobject_init_and_add(&sensor->sysfs_kobj, &cam_sensor_pwr_sysfs_ktype,
					cam_pwr_base_kobject, buff);
	if (error) {
		CAM_ERR(CAM_CCI, "KOBJ_INIT AND ADD FAILED");
		return -ENOMEM;
	}

	error = sysfs_create_files(&sensor->sysfs_kobj,
			cam_sensor_pwr_sysfs_attr_list);
	if (error) {
		CAM_ERR(CAM_CCI, "failed to create the cam file in /sys/kernel/");
		return error;
	}

	return 0;
}

static uint32_t cam_cci_parse_master(
		uint32_t cci_dev_id, uint32_t master_id,
		struct cam_sensor_cci_client *cci_client,
		uint32_t slave_addr)
{
	int32_t rc = 0;
	struct cci_device *cci_dev;
	const struct cam_hw_soc_info *soc_info = NULL;
	struct v4l2_subdev *sd;
	struct device *dev;

	sd = cam_cci_get_subdev(cci_dev_id);
	cci_dev = v4l2_get_subdevdata(sd);
	if (!cci_dev) {
		CAM_ERR(CAM_CCI,
				"Invalid params cci_dev: %p",
				cci_dev);
		rc = -EINVAL;
		return rc;
	}

	if (cci_client == NULL) {
		CAM_ERR(CAM_CCI,
				"NULL argument cci_client");
		rc = -EINVAL;
		return rc;
	}

	soc_info = &cci_dev->soc_info;
	dev =  soc_info->dev;

	CAM_DBG(CAM_CCI, "cci_dev_id %d, master_id: %d",
			cci_dev_id, master_id);

	cci_client->cci_device = cci_dev_id;
	cci_client->cci_i2c_master = master_id;
	cci_client->cci_subdev = sd;
	cci_client->sid = slave_addr >> 1;
	cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
	return rc;
}

static int32_t cam_cci_parse_data(const char *p_line,
		struct cam_sensor_i2c_reg_setting *reg_list,
		bool *is_read, struct cam_sensor_cci_client *cci_client,
		int32_t master_id, int32_t cci_dev_id)
{
	int32_t rc;
	struct cam_sensor_i2c_reg_array *reg_array = reg_list->reg_setting;
	int32_t bus_id;
	uint32_t slave_addr;

	if (!strlen(p_line))
		return -EINVAL;

	if (p_line[0] == 'r' || p_line[0] == 'R') {
		*is_read = true;

		rc = sscanf(p_line+2, "%x,%d,%d,%x",
				&reg_array->reg_addr, &reg_list->addr_type, &reg_list->data_type,
				&slave_addr);
		if (rc == NUM_OF_CCI_READ_PARAMS) {
			rc = cam_cci_parse_master(
					cci_dev_id, master_id,
					cci_client, slave_addr);
			if (rc)
				return -EINVAL;
			return 0;
		}
	} else if (p_line[0] == 'w'  || p_line[0] == 'W') {
		*is_read = false;

		rc = sscanf(p_line+2, "%x,%x,%d,%d,%d,%x",
				&reg_array->reg_addr,  &reg_array->reg_data, &reg_array->delay,
				&reg_list->addr_type, &reg_list->data_type,
				&slave_addr);
		if (rc == NUM_OF_CCI_WRITE_PARAMS) {
			rc = cam_cci_parse_master(
					cci_dev_id, master_id,
					cci_client, slave_addr);
			if (rc)
				return -EINVAL;
			return 0;
		}
	}

	return -EINVAL;
}

static ssize_t cci_show(struct cci_sysfs *obj,
		char *buf)
{
	int count = 0;
	int idx = 0;

	mutex_lock(&(obj->cci_mutex));
	idx = (obj->cci_dev * MASTER_MAX) + obj->master;
	if (display_cci_buf[idx]) {
		count = scnprintf(buf, PAGE_SIZE, "%s\n", display_cci_buf[idx]);
		memset(display_cci_buf[idx], '\0', TOTAL_BUFFER_SIZE);
	}
	mutex_unlock(&(obj->cci_mutex));

	return count;
}

static size_t cci_process_cb(const char *buf, size_t count, int32_t master, int32_t bus)
{
	int rc = 0, master_lock = 0;
	char *token = NULL, *input_buf = NULL, *saveptr = NULL;
	struct cam_sensor_cci_client *cci_client = NULL;
	char *line_buffer = NULL;
	struct cam_sensor_i2c_reg_setting read_write;
	struct cam_sensor_i2c_reg_array reg_array;
	struct cam_cci_ctrl cci_ctrl;
	int32_t idx = (bus*MASTER_MAX) + master;

	if (!display_cci_buf[idx] || !buf || count == 0 || (count+2 > PAGE_SIZE)) {

		if (count+2 > PAGE_SIZE) {
			CAM_ERR(CAM_CCI, "Too many commands ..");
		}
		return -EINVAL;
	}

	input_buf = kzalloc(sizeof(char) * (count+2), GFP_KERNEL);
	if (!input_buf)
		return -ENOMEM;

	saveptr = input_buf;

	line_buffer = kzalloc(sizeof(char) * INFO_BUFFER_SIZE, GFP_KERNEL);
	if (!line_buffer) {
		CAM_ERR(CAM_CCI, "line_buffer allocation failed ..");
		kfree(input_buf);
		return -ENOMEM;
	}

	memcpy(input_buf, buf, count);
	input_buf[count + 1] = '\0';

	cci_client = kzalloc(sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!cci_client) {
		CAM_ERR(CAM_CCI, "cci client allocation failed ..");
		kfree(input_buf);
		kfree(line_buffer);
		return -ENOMEM;
	}

	token = strsep(&input_buf, ";");
	while (token) {
		bool is_read = false;

		memset(&read_write, 0, sizeof(read_write));
		memset(&reg_array, 0, sizeof(reg_array));
		memset(&cci_ctrl, 0, sizeof(cci_ctrl));
		memset(line_buffer, 0, INFO_BUFFER_SIZE);

		read_write.reg_setting = &reg_array;

		rc = cam_cci_parse_data(token, &read_write, &is_read, cci_client, master, bus);
		if (rc) {
			strlcat(display_cci_buf[idx], USAGE_STRING, TOTAL_BUFFER_SIZE);
			token = strsep(&input_buf, ";");
			CAM_ERR(CAM_CCI, "Incorrect syntax. Syntax: %s", USAGE_STRING);
			continue;
		}

		if (master_lock == 0) {
			memset(&cci_ctrl, 0, sizeof(cci_ctrl));
			cci_ctrl.cmd = MSM_CCI_I2C_SEQUENTIAL_XFER_LOCK;
			cci_ctrl.cci_info = cci_client;

			rc = v4l2_subdev_call(cci_client->cci_subdev,
					core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_CCI, "MASTER LOCK FAILED");
				count = rc;
				goto handle_error;
			}

			CAM_DBG(CAM_CCI, "MASTER LOCK ACQUIRED");
			master_lock++;
		}

		if (is_read) {
			rc = cam_cci_i2c_read(cci_client,
					reg_array.reg_addr,
					&reg_array.reg_data,
					read_write.addr_type,
					read_write.data_type,
					false);
			if (!rc) {
				scnprintf(line_buffer, INFO_BUFFER_SIZE,
						"Read: 0x%X, 0x%X, 0x%x, 0x%X rc: %u\n",
						read_write.reg_setting->reg_addr,
						read_write.reg_setting->reg_data,
						read_write.data_type,
						read_write.addr_type, rc);
				strlcat(display_cci_buf[idx], line_buffer, TOTAL_BUFFER_SIZE);
			} else if (rc < 0) {
				count = rc;
				CAM_ERR(CAM_CCI, "cci read fail for reg: 0x%X, error code :%d",
						read_write.reg_setting->reg_addr, rc);
				goto handle_error;
			}
		} else {
			if (read_write.addr_type <= CAMERA_SENSOR_I2C_TYPE_INVALID ||
					read_write.addr_type >= CAMERA_SENSOR_I2C_TYPE_MAX ||
					read_write.data_type <= CAMERA_SENSOR_I2C_TYPE_INVALID ||
					read_write.data_type >= CAMERA_SENSOR_I2C_TYPE_MAX) {
				count = -EINVAL;
				CAM_ERR(CAM_CCI, "cam cci write invalid input arguments");
				goto handle_error;
			}

			read_write.size = 1;
			cci_ctrl.cci_info = cci_client;
			cci_ctrl.cmd = MSM_CCI_I2C_WRITE;
			cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting = &reg_array;
			cci_ctrl.cfg.cci_i2c_write_cfg.data_type = read_write.data_type;
			cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = read_write.addr_type;
			cci_ctrl.cfg.cci_i2c_write_cfg.size = read_write.size;

			rc = v4l2_subdev_call(cci_client->cci_subdev,
					core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
			if (rc) {
				scnprintf(line_buffer, INFO_BUFFER_SIZE,
						"Write Error: 0x%X, 0x%X, 0x%x, 0x%X rc: %d\n",
						read_write.reg_setting->reg_addr,
						read_write.reg_setting->reg_data,
						read_write.data_type,
						read_write.addr_type, rc);
				strlcat(display_cci_buf[idx], line_buffer, TOTAL_BUFFER_SIZE);
			} else if (rc < 0) {
				count = rc;
				CAM_ERR(CAM_CCI, "cci write fail for reg: 0x%X value:%d, rc:%d",
						read_write.reg_setting->reg_addr,
						read_write.reg_setting->reg_data, rc);

				goto handle_error;
			}

			if (reg_array.delay > 20)
				msleep(reg_array.delay);
			else if (reg_array.delay)
				usleep_range(reg_array.delay * 1000, reg_array.delay * 1000 + 1000);
		}

		token = strsep(&input_buf, ";");
	}

handle_error:
	if (master_lock > 0) {
		cci_ctrl.cmd = MSM_CCI_I2C_SEQUENTIAL_XFER_UNLOCK;
		cci_ctrl.cci_info = cci_client;

		rc = v4l2_subdev_call(cci_client->cci_subdev,
				core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_CCI, "MASTER UNLOCK FAILED", rc);
		} else {
			CAM_DBG(CAM_CCI, "MASTER LOCK RELEASED", rc);

		}
		master_lock--;
	}
	if (saveptr) {
		kfree(saveptr);
	}
	if (line_buffer)
		kfree(line_buffer);
	if (cci_client)
		kfree(cci_client);

	return count;
}

static ssize_t cci_store(struct cci_sysfs *obj,
		const char *buf, size_t count)
{
	if (obj == NULL) {
		CAM_ERR(CAM_CCI, "INVALID CCI ENTRY ");
		return 0;
	}
	mutex_lock(&(obj->cci_mutex));
	count = cci_process_cb(buf, count, obj->master, obj->cci_dev);
	mutex_unlock(&(obj->cci_mutex));

	return count;
}

static CCI_SYSFS_ATTR(i2c_rw, 0660, cci_show,
				cci_store);

static ssize_t cci_sysfs_attr_show(struct kobject *kobj,
		struct attribute *__attr, char *buf)
{
	struct cci_sysfs_attr *attr = container_of(__attr,
			struct cci_sysfs_attr, attr);
	struct cci_sysfs *device = container_of(kobj,
			struct cci_sysfs, sysfs_kobj);

	if (attr->show)
		return attr->show(device, buf);

	return -EIO;
}

static ssize_t cci_sysfs_attr_store(struct kobject *kobj,
		struct attribute *__attr, const char *buf, size_t count)
{
	struct cci_sysfs_attr *attr = container_of(__attr,
			struct cci_sysfs_attr, attr);
	struct cci_sysfs *device = container_of(kobj,
			struct cci_sysfs, sysfs_kobj);

	if (attr->store)
		return attr->store(device, buf, count);

	return -EIO;
}

/* Dummy release function - we have nothing to do here */
static void cci_sysfs_release(struct kobject *kobj)
{
}

static const struct sysfs_ops cci_sysfs_ops = {
	.show = cci_sysfs_attr_show,
	.store = cci_sysfs_attr_store,
};

static struct kobj_type cci_sysfs_ktype = {
	.sysfs_ops = &cci_sysfs_ops,
	.release = cci_sysfs_release,
};

static const struct attribute *cci_sysfs_attr_list[] = {
	&cci_sysfs_attr_i2c_rw.attr,
	NULL,
};

int  cam_sysfs_add_cci(void *cci_device_ptr)
{

	int error = 0;
	char buff[5];
	struct device_node *of_node;
	struct device *dev;
	struct cam_hw_soc_info *soc_info;
	uint32_t i = 0;
	uint32_t idx;
	uint32_t cci_idx;
	int32_t num_of_map_idx = 0;
	struct cci_device *cci_dev = (struct cci_device *) cci_device_ptr;

	if (cci_dev != NULL) {
		soc_info = &cci_dev->soc_info;
	} else {
		CAM_DBG(CAM_CCI, "CCI DEV NULL");
		return -EINVAL;
	}


	if (soc_info->dev == NULL) {
		CAM_DBG(CAM_CCI, "SOCINFO DEV NULL");
		return -EINVAL;
	}

	dev = soc_info->dev;

	if (dev->of_node == NULL) {
		CAM_DBG(CAM_CCI, "CCI OF_NODE NULL");
		return -EINVAL;
	}

	of_node = dev->of_node;

	if (!cam_cci_base_kobject) {
		cam_cci_base_kobject = kobject_create_and_add("cam-cci",
				kernel_kobj);
		if (!cam_cci_base_kobject)
			return -ENOMEM;
	}

	num_of_map_idx = of_property_count_u32_elems(
			of_node, "pctrl-idx-mapping");
	if (num_of_map_idx <= 0 && num_of_map_idx > MASTER_MAX) {
		CAM_ERR(CAM_CCI, "Reading pctrl-idx-mapping failed");
		return -EINVAL;
	}

	for (i = 0; i < num_of_map_idx; i++) {
		of_property_read_u32_index(of_node,
				"pctrl-idx-mapping", i, &idx);
		cci_dev->cci_master_sysfs[i].master = idx;
		cci_dev->cci_master_sysfs[i].cci_dev = soc_info->index;
		cci_dev->num_masters = num_of_map_idx;
		cci_idx = (soc_info->index * num_of_map_idx) + idx;

		scnprintf(buff, sizeof(buff), "cci%d", cci_idx);

		error = kobject_init_and_add(&cci_dev->cci_master_sysfs[i].sysfs_kobj,
				&cci_sysfs_ktype, cam_cci_base_kobject, buff);
		if (error) {
			CAM_ERR(CAM_CCI, "KOBJ_INIT AND ADD FAILED");
			return -ENOMEM;
		}

		error = sysfs_create_files(&cci_dev->cci_master_sysfs[i].sysfs_kobj,
				cci_sysfs_attr_list);
		if (error) {
			CAM_ERR(CAM_CCI, "failed to create the cam file in /sys/kernel/");
			return error;
		}

		mutex_init(&(cci_dev->cci_master_sysfs[i].cci_mutex));

		if ((cci_dev->cci_master_sysfs[i].sysfs_kobj.state_initialized) &&
				!display_cci_buf[cci_idx]) {
			display_cci_buf[cci_idx] = kzalloc(sizeof(char) * TOTAL_BUFFER_SIZE,
							GFP_KERNEL);
		}
	}

	return error;
}

void cam_sysfs_exit(void)
{
	if (cam_cci_base_kobject) {
		kobject_put(cam_cci_base_kobject);
		cam_cci_base_kobject = NULL;
	}
	if (cam_pwr_base_kobject) {
		kobject_put(cam_pwr_base_kobject);
		cam_pwr_base_kobject = NULL;
	}
}

int32_t cam_sysfs_remove_cci(void *cci_device_ptr)
{
	uint32_t i = 0;
	int32_t idx;
	struct cci_device *cci_dev = (struct cci_device *) cci_device_ptr;
	struct cci_sysfs *obj = NULL;

	if (cci_dev == NULL) {
		CAM_DBG(CAM_CCI, "CCI DEV NULL. CCI SYSFS REMOVAL FAILED");
		return -EINVAL;
	}

	for (i = 0; i < cci_dev->num_masters; i++) {
		obj = &cci_dev->cci_master_sysfs[i];
		mutex_lock(&(obj->cci_mutex));

		if (obj->sysfs_kobj.state_initialized) {
			sysfs_remove_files(&obj->sysfs_kobj,
					cci_sysfs_attr_list);
			kobject_put(&obj->sysfs_kobj);

			idx = (obj->cci_dev * cci_dev->num_masters) + obj->master;
			if (display_cci_buf[idx]) {
				kfree(display_cci_buf[idx]);
				display_cci_buf[idx] = NULL;
			}
		}

		mutex_unlock(&(obj->cci_mutex));
	}

	return 1;
}

int32_t cam_sensor_remove_device(void *ctrl)
{
	struct cam_sensor_ctrl_t *sensor = (struct cam_sensor_ctrl_t *) ctrl;
	int rc = 0;

	if (sensor == NULL) {
		CAM_DBG(CAM_CCI, "CCI DEV NULL. CCI SYSFS REMOVAL FAILED");
		return -EINVAL;
	}
	mutex_lock(&(sensor->cam_sensor_mutex));

	if (sensor->sysfs_kobj.state_initialized) {
		if (sensor->sysfs_state == true) {
			CAM_ERR(CAM_CCI, "SENSOR %d POWERED ON. POWEROFF before teardown",
					sensor->soc_info.index);
			rc = cam_sensor_power_down(sensor);
			if (rc < 0) {
				CAM_ERR(CAM_CCI, "SENSOR %d POWEROFF DONE",
					sensor->soc_info.index);
				sensor->sysfs_state = false;
			}
		}
		sysfs_remove_files(&sensor->sysfs_kobj,
				cam_sensor_pwr_sysfs_attr_list);
		kobject_put(&sensor->sysfs_kobj);

	}
	mutex_unlock(&(sensor->cam_sensor_mutex));

	return 1;
}
