// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 ITE
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */
#include <crypto/hash.h>
#include <drm/display/drm_hdcp_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <sound/hdmi-codec.h>
#include <video/videomode.h>

#define SUPPORT_CREAT_CONNECTOR
#define EBABLE_HDCP
// #define FW_VIDEO_AUTO_MODE
/* Power and reset control is not ready for MCU*/
// #define ENABLE_POWER_RESET_CONTROL
// infoblock
#define DATA_BUFFER_DEPTH 32

#define OFFSET_CHIP_ID_L 0x00
#define OFFSET_CHIP_ID_M 0x01
#define OFFSET_CHIP_ID_H 0x02
#define OFFSET_VERSION_L 0x03
#define OFFSET_VERSION_M 0x04
#define OFFSET_VERSION_H 0x03
#define OFFSET_MIPI_CONFIG_L 0x06
#define OFFSET_MIPI_CONFIG_H 0x07
#define OFFSET_TX_CONFIG 0x08
#define OFFSET_TX_SETTING 0x09
#define OFFSET_MIPI_STATUS 0x0A

#define OFFSET_TX_STATUS 0x0C
#define B_TX_STATUS_HPD 7
#define B_TX_STATUS_VIDEO_STB 6
#define B_TX_STATUS_HDCP 4
#define M_TX_STATUS_HDCP 0x30

#define TX_VIDEO_STB BIT(B_TX_STATUS_VIDEO_STB)
#define TX_STATUS_HPD BIT(B_TX_STATUS_HPD)

#define GET_TX_HPD_STATUS(x) ((x & TX_STATUS_HPD) >> B_TX_STATUS_HPD)
#define GET_TX_VIDEO_STATUS(x) ((x & TX_VIDEO_STB) >> B_TX_STATUS_VIDEO_STB)
#define GET_TX_HDCP_STATUS(x) ((x & M_TX_STATUS_HDCP) >> B_TX_STATUS_HDCP)

#define OFFSET_SINK_CAP 0x0D
#define B_SINK_CAP_HDCP_VER 4
#define M_SINK_CAP_HDCP_VER 0x30

#define GET_SINK_CAP_HDCP_VER(x) \
	((x & M_SINK_CAP_HDCP_VER) >> B_SINK_CAP_HDCP_VER)

#define OFFSET_DRIVER_DATA 0x0E

#define OFFSET_DATA_TYPE_IDX 0x0F
#define OFFSET_DATA_BUFFER 0x20

#define OFFSET_HOST_SETTING 0xFE
#define B_CONFIG_CHG BIT(7)
#define B_SET_CHG BIT(6)
#define HOST_SETTING_VIDEO_INFO (1)
#define HOST_SETTING_AUDIO_INFO (2)
#define HOST_SETTING_VIDEO_AUDIO_INFO (3)
#define HOST_SETTING_EDID_R (0x04)
#define HOST_SETTING_HDCP_R (0x05)

#define OFFSET_EVENT_CHG 0xFF
#define B_EVENT_CHG_BUFFER 4
#define M_EVENT_CHG_BUFFER_STS (0x30)

#define B_EVENT_CHG 1
#define B_EVENT_IC_READY 0

#define EVENT_CHG BIT(B_EVENT_CHG)
#define EVENT_READY BIT(B_EVENT_IC_READY)

#define IS_EVENT_CHG

#define GET_BUFFER_STATUS(x) \
	((x & M_EVENT_CHG_BUFFER_STS) >> B_EVENT_CHG_BUFFER)

#define TIMEOUT_INFOBLOCK_MS 1800

enum it6162_audio_select {
	I2S = 0,
	SPDIF,
};

enum it6162_audio_word_length {
	WORD_LENGTH_16BIT = 0x0,
	WORD_LENGTH_18BIT = 0x1,
	WORD_LENGTH_20BIT = 0x2,
	WORD_LENGTH_24BIT = 0x3,
};

enum it6162_audio_sample_rate {
	SAMPLE_RATE_32K = 0x3,
	SAMPLE_RATE_48K = 0x2,
	SAMPLE_RATE_64K = 0xB,
	SAMPLE_RATE_96K = 0xA,
	SAMPLE_RATE_192K = 0xE,
	SAMPLE_RATE_44_1K = 0x0,
	SAMPLE_RATE_88_2K = 0x8,
	SAMPLE_RATE_176_4K = 0xC,
};

enum it6162_audio_type {
	LPCM = 0,
	NLPCM,
};

enum data_buf_sts {
	NO_STS = 0x00,
	BUF_READY = 0x01,
	BUF_FAIL = 0x02,
};

enum hdcp_state {
	NO_HDCP_STATE = 0x00,
	AUTH_DONE = 0x01,
	AUTH_FAIL = 0x02,
};

enum hdcp_ver {
	NO_HDCP = 0x0,
	HDCP_14 = 0x1,
	HDCP_23 = 0x2,
};

struct it6162_chip_info {
	u32 chip_id;
	u32 version;
};

struct it6162_audio {
	enum it6162_audio_select select;
	enum it6162_audio_type type;
	enum it6162_audio_sample_rate sample_rate;
	u8 word_length;
	unsigned int audio_enable;
	unsigned int sample_width;
	unsigned int channel_number;
	unsigned int user_cts;
	u8 infoframe[HDMI_INFOFRAME_SIZE(AUDIO)];
	unsigned char channel_status[AES_IEC958_STATUS_SIZE];
};

struct it6162_viedo {
	u8 vic;
	u16 clock;
	u16 htotal;
	u16 hfp;
	u16 hsw;
	u16 hbp;
	u16 hdew;
	u16 vtotal;
	u16 vfp;
	u16 vsw;
	u16 vbp;
	u16 vdew;
	u8 hpol;
	u8 vpol;
	u8 prog;
	u16 v_aspect;
	u16 h_aspect;
	u8 pix_rep;
	u8 colorspace;
};

enum sync_mode {
	SYNC_EVENT = 0x0,
	SYNC_PULSE = 0x1,
	SYNC_AUTO = 0x2,
};

struct it6162_mipirx_config {
	u8 lane_num;
	bool pn_swap;
	bool lane_swap;
	bool en_port0;
	bool en_port1;
	bool continuous_clk;
	enum sync_mode mode;
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
};

struct it6162_tx_out_set {
	enum hdcp_ver hdcp_version;
	bool hdcp_encyption;
	u8 stream_ID;
};

struct it6162_infoblock_msg {
	u8 action;
	int len;
	u8 msg[32];
};

struct it6162 {
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct device *dev;
	enum drm_connector_status connector_status;
	struct drm_device *drm;

	struct i2c_client *it6162_i2c;
	struct regmap *it6162_regmap;

	struct work_struct hdcp_work;

	struct regulator *pwr18;
	struct regulator *ovdd;
	struct regulator *ivdd;
	struct gpio_desc *gpiod_reset;

	bool is_hdmi;
	bool has_audio;
	bool en_hdcp;

	/* operations can only be served one at the time */
	struct mutex lock;

	/* it6162 DSI RX related params */
	struct mipi_dsi_device *dsi;
	struct it6162_mipirx_config mipirx_config;
	struct it6162_tx_out_set tx_out_set;

	bool support_audio;
	struct it6162_audio audio_config;
	struct platform_device *audio_pdev;
	hdmi_codec_plugged_cb plugged_cb;
	struct device *codec_dev;

	struct it6162_chip_info chip_info;

	enum data_buf_sts data_buf_sts;
	enum hdcp_state hdcp_sts;
	enum hdcp_ver hdcp_version;

	bool bridge_hpd_enable;
};

static void it6162_audio_update_connector_status(struct it6162 *it6162);

static struct it6162 *bridge_to_it6162(struct drm_bridge *bridge)
{
	return container_of(bridge, struct it6162, bridge);
}

static const struct regmap_config it6162_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
};

static int it6162_i2c_regmap_init(struct i2c_client *client,
		struct it6162 *it6162)
{
	it6162->it6162_i2c = client;
	i2c_set_clientdata(client, it6162);

	it6162->it6162_regmap =
		devm_regmap_init_i2c(it6162->it6162_i2c, &it6162_regmap_config);
	if (IS_ERR(it6162->it6162_regmap))
		return PTR_ERR(it6162->it6162_regmap);

	return 0;
}

static unsigned int it6162_infoblock_read(struct it6162 *it6162,
		unsigned int reg)
{
	unsigned int val;
	int err;
	struct device *dev = it6162->dev;

	err = regmap_read(it6162->it6162_regmap, reg, &val);
	if (err < 0) {
		dev_err(dev, "read failed rx reg[0x%x] err: %d", reg, err);
		return err;
	}

	return val;
}

static int it6162_infoblock_write(struct it6162 *it6162, unsigned int reg,
		unsigned int val)
{
	int err;
	struct device *dev = it6162->dev;

	err = regmap_write(it6162->it6162_regmap, reg, val);

	if (err < 0) {
		dev_err(dev,
				"write failed rx reg[0x%x] = 0x%x err = %d",
				reg,
				val,
				err);
		return err;
	}

	return 0;
}

static inline void it6162_infoblock_update_bits(struct it6162 *it6162,
		unsigned int reg,
		unsigned int mask,
		unsigned int val)
{
	regmap_update_bits(it6162->it6162_regmap, reg, mask, val);
}

static inline void it6162_infoblock_bulk_read(struct it6162 *it6162,
		unsigned int reg, u8 *buf,
		size_t len)
{
	regmap_bulk_read(it6162->it6162_regmap, reg, buf, len);
}

static inline void it6162_infoblock_read_bufer(struct it6162 *it6162, u8 *buf,
		size_t len)
{
	regmap_bulk_read(it6162->it6162_regmap, OFFSET_DATA_BUFFER, buf, len);
}

static inline void it6162_infoblock_bulk_write(struct it6162 *it6162,
		unsigned int reg, u8 *buf,
		size_t len)
{
	regmap_bulk_write(it6162->it6162_regmap, reg, buf, len);
}

static inline void it6162_infoblock_write_bufer(struct it6162 *it6162, u8 *buf,
		size_t len)
{
	regmap_bulk_write(it6162->it6162_regmap, OFFSET_DATA_BUFFER, buf, len);
}

static bool it6162_infoblock_complete(struct it6162 *it6162)
{
	int tmp;

	tmp = it6162_infoblock_read(it6162, OFFSET_HOST_SETTING);
	dev_info(it6162->dev, "%s %x", __func__, tmp);
	return tmp == 0 ? true : false;
}

static int it6162_infoblock_wait_complete(struct it6162 *it6162)
{
	struct device *dev = it6162->dev;
	int status;
	bool val;

	status = readx_poll_timeout(it6162_infoblock_complete,
			it6162,
			val,
			val == true,
			50 * 1000,
			TIMEOUT_INFOBLOCK_MS * 1000);

	dev_info(dev, "%s status = %d %d", __func__, status, (int)val);
	if (status < 0) {
		dev_err(dev, "%s err status = %d", __func__, status);
		return -ETIMEDOUT;
	}

	return 0;
}

static int it6162_infoblock_host_set(struct it6162 *it6162, u8 setting)
{
	dev_info(it6162->dev, "%s %x", __func__, setting);
	it6162_infoblock_write(it6162, OFFSET_HOST_SETTING, setting);
	/*wait command complete*/
	it6162_infoblock_wait_complete(it6162);

	return 0;
}

static int it6162_infoblock_request_data(struct it6162 *it6162, u8 setting,
		u8 index, u8 *buf)
{
	struct device *dev = it6162->dev;
	int status;
	bool val;

	it6162->data_buf_sts = NO_STS;
	it6162_infoblock_write(it6162, OFFSET_DATA_TYPE_IDX, index);
	it6162_infoblock_write(it6162, OFFSET_HOST_SETTING, setting);

	status =
		readx_poll_timeout(it6162_infoblock_complete,
				it6162,
				val,
				val == true && it6162->data_buf_sts == BUF_READY,
				50 * 1000,
				TIMEOUT_INFOBLOCK_MS * 1000);

	dev_info(dev,
			"%s status = %d %d, %d",
			__func__,
			status,
			it6162->data_buf_sts,
			(int)val);
	if (status < 0) {
		dev_err(dev,
				"%s err status = %d %d",
				__func__,
				status,
				it6162->data_buf_sts);
		return -ETIMEDOUT;
	}

	dev_info(
			dev, "%s [0x%x] 0x%x", __func__, setting, it6162->data_buf_sts);
	if (it6162->data_buf_sts != BUF_READY)
		return -EIO;

	it6162_infoblock_read_bufer(it6162, buf, DATA_BUFFER_DEPTH);
	return 0;
}

static void it6162_infoblock_mipi_config(struct it6162 *it6162,
		struct it6162_mipirx_config *cfg)
{
	u8 cfg_val = 0;

	cfg_val = (cfg->continuous_clk << 6) | (cfg->en_port1 << 5) |
		(cfg->en_port0 << 4) | (cfg->lane_swap << 3) |
		(cfg->pn_swap << 2) | (cfg->lane_num - 1);

	dev_dbg(it6162->dev, "%s 0x%02x 0x%02x", __func__, cfg_val, cfg->mode);

	it6162_infoblock_write(it6162, OFFSET_MIPI_CONFIG_L, cfg_val);
	it6162_infoblock_write(it6162, OFFSET_MIPI_CONFIG_H, cfg->mode);
}

static inline void it6162_infoblock_write_msg(struct it6162 *it6162,
		struct it6162_infoblock_msg *msg)
{
	it6162_infoblock_write_bufer(it6162, msg->msg, msg->len);
	it6162_infoblock_host_set(it6162, msg->action);
}

/**
 * Following APIs use infoblock access
 */

static void it6162_set_default_config(struct it6162 *it6162)
{
	guard(mutex)(&it6162->lock);
	it6162_infoblock_mipi_config(it6162, &it6162->mipirx_config);
	it6162_infoblock_update_bits(it6162, OFFSET_MIPI_CONFIG_L, 0x30, 0x00);
	it6162_infoblock_write(it6162, OFFSET_TX_CONFIG, 0x00);
	it6162_infoblock_write(it6162, OFFSET_TX_SETTING, 0x00);
	it6162_infoblock_host_set(it6162, B_CONFIG_CHG | B_SET_CHG);
}

static void it6162_mipi_disable(struct it6162 *it6162)
{
	guard(mutex)(&it6162->lock);
	it6162_infoblock_update_bits(it6162, OFFSET_MIPI_CONFIG_L, 0x30, 0x00);
	it6162_infoblock_host_set(it6162, B_CONFIG_CHG);
}

static void it6162_mipi_enable(struct it6162 *it6162)
{
	unsigned int cfg_val = 0;

	cfg_val = it6162->mipirx_config.en_port0 << 4 |
		it6162->mipirx_config.en_port1 << 5;

	guard(mutex)(&it6162->lock);
	it6162_infoblock_update_bits(
			it6162, OFFSET_MIPI_CONFIG_L, 0x30, cfg_val);
	it6162_infoblock_host_set(it6162, B_CONFIG_CHG);
}

static void it6162_tx_hdcp_enable(struct it6162 *it6162,
		struct it6162_tx_out_set *tx_out)
{
	u8 tmp;

	it6162->hdcp_sts = NO_HDCP_STATE;
	tmp = it6162_infoblock_read(it6162, OFFSET_TX_SETTING) & 0x0F;
	tmp |= (tx_out->hdcp_version << 6) | (tx_out->hdcp_encyption << 5) |
		(tx_out->stream_ID << 4);

	guard(mutex)(&it6162->lock);
	it6162_infoblock_write(it6162, OFFSET_TX_SETTING, tmp);
	it6162_infoblock_host_set(it6162, B_SET_CHG);

	dev_dbg(it6162->dev, "%s 0x%02x", __func__, tmp);
}

static void it6162_tx_hdcp_disable(struct it6162 *it6162)
{
	u8 tmp;

	it6162->hdcp_sts = NO_HDCP_STATE;
	it6162->hdcp_version = NO_HDCP;
	guard(mutex)(&it6162->lock);
	tmp = it6162_infoblock_read(it6162, OFFSET_TX_SETTING);
	tmp &= 0x0F;
	it6162_infoblock_write(it6162, OFFSET_TX_SETTING, tmp);
	it6162_infoblock_host_set(it6162, B_SET_CHG);

	dev_dbg(it6162->dev, "%s 0x%02x", __func__, tmp);
}

static void it6162_tx_hdcp_setup(struct it6162 *it6162, u8 ver, bool encription)
{
	struct it6162_tx_out_set tx_out;

	dev_dbg(it6162->dev, "%s ver %x enc %d", __func__, ver, encription);

	if (ver != NO_HDCP) {
		tx_out.hdcp_version = ver;
		tx_out.hdcp_encyption = encription;
		it6162_tx_hdcp_enable(it6162, &tx_out);
	} else {
		it6162_tx_hdcp_disable(it6162);
	}
}

static void it6162_tx_enable(struct it6162 *it6162)
{
	dev_dbg(it6162->dev, "%s", __func__);
	it6162->en_hdcp = true;
	it6162->hdcp_version = it6162->tx_out_set.hdcp_version;
}

static void it6162_tx_disable(struct it6162 *it6162)
{
	dev_dbg(it6162->dev, "%s", __func__);
	it6162->en_hdcp = false;
	it6162_tx_hdcp_setup(it6162, NO_HDCP, false);
	cancel_work_sync(&it6162->hdcp_work);
}

static void it6162_show_drm_video_mode(struct it6162 *it6162,
		const struct videomode *vm)
{
	struct device *dev = it6162->dev;

	dev_info(dev, "HActive = %u", vm->hactive);
	dev_info(dev, "VActive = %u", vm->vactive);
	dev_info(
			dev,
			"HTotal  = %u",
			vm->hactive + vm->hfront_porch + vm->hsync_len + vm->hback_porch);
	dev_info(
			dev,
			"VTotal  = %u",
			vm->vactive + vm->vfront_porch + vm->vsync_len + vm->vback_porch);
	dev_info(dev, "PCLK    = %lukhz", vm->pixelclock / 1000);
	dev_info(dev, "HFP     = %u", vm->hfront_porch);
	dev_info(dev, "HSW     = %u", vm->hsync_len);
	dev_info(dev, "HBP     = %u", vm->hback_porch);
	dev_info(dev, "VFP     = %u", vm->vfront_porch);
	dev_info(dev, "VSW     = %u", vm->vsync_len);
	dev_info(dev, "VBP     = %u", vm->vback_porch);
	if (vm->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		dev_info(dev, "HPOL +");
	else
		dev_info(dev, "HPOL -");

	if (vm->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		dev_info(dev, "VPOL +");
	else
		dev_info(dev, "VPOL -");

	if (vm->flags & DISPLAY_FLAGS_INTERLACED)
		dev_info(dev, "Intelaced");
	else
		dev_info(dev, "Progressive");
}

static void it6162_get_video_setting(struct it6162 *it6162,
		struct it6162_viedo *video,
		struct videomode *vm,
		struct hdmi_avi_infoframe *avi_info)
{
	it6162_show_drm_video_mode(it6162, vm);

	if (vm->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		video->hpol = 1;

	if (vm->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		video->vpol = 1;

	if (vm->flags & DISPLAY_FLAGS_INTERLACED)
		video->prog = false;

	video->clock = vm->pixelclock / 1000;
	video->hdew = vm->hactive;

	video->hfp = vm->hfront_porch;
	video->hsw = vm->hsync_len;
	video->hbp = vm->hback_porch;
	video->htotal =
		vm->hactive + vm->hfront_porch + vm->hsync_len + vm->hback_porch;

	video->vdew = vm->vactive;
	video->vfp = vm->vfront_porch;
	video->vsw = vm->vsync_len;
	video->vbp = vm->vback_porch;
	video->vtotal =
		vm->vactive + vm->vfront_porch + vm->vsync_len + vm->vback_porch;

	video->vic = avi_info->video_code;

	dev_dbg(it6162->dev, "vic %x", video->vic);

	switch (avi_info->picture_aspect) {
	case HDMI_PICTURE_ASPECT_4_3:
		video->h_aspect = 4;
		video->v_aspect = 3;
		break;
	case HDMI_PICTURE_ASPECT_16_9:
		video->h_aspect = 16;
		video->v_aspect = 9;
		break;
	case HDMI_PICTURE_ASPECT_64_27:
		video->h_aspect = 64;
		video->v_aspect = 27;
		break;
	case HDMI_PICTURE_ASPECT_256_135:
		video->h_aspect = 256;
		video->v_aspect = 135;
		break;
	default:
		video->h_aspect = 4;
		video->v_aspect = 3;
		break;
	}
	dev_dbg(it6162->dev, "aspect %d:%d", video->h_aspect, video->v_aspect);

	video->pix_rep = avi_info->pixel_repeat + 1;
	dev_dbg(it6162->dev, "pix_rep %d", video->pix_rep);

	video->colorspace = avi_info->colorspace;
	dev_dbg(it6162->dev, "colorspace %d", video->colorspace);
}

static void it6162_pack_video_setting(struct it6162 *it6162,
		struct it6162_viedo *video,
		struct it6162_infoblock_msg *msg)
{
	msg->action = HOST_SETTING_VIDEO_INFO;
	msg->len = 0x1C;

	msg->msg[0x00] = video->hdew & 0xFF;
	msg->msg[0x01] = (video->hdew >> 8) & 0x3F;
	msg->msg[0x02] = video->vdew & 0xFF;
	msg->msg[0x03] = (video->vdew >> 8) & 0x3F;
	msg->msg[0x04] = video->clock & 0xFF;
	msg->msg[0x05] = (video->clock >> 8) & 0xFF;
	msg->msg[0x06] = (video->clock >> 16) & 0xFF;
	msg->msg[0x07] = (video->clock >> 24) & 0xFF;
	msg->msg[0x08] = video->hfp & 0xFF;
	msg->msg[0x09] = (video->hfp >> 8) & 0x3F;
	msg->msg[0x0A] = video->hsw & 0xFF;
	msg->msg[0x0B] = (video->hsw >> 8) & 0x3F;
	msg->msg[0x0C] = video->hbp & 0xFF;
	msg->msg[0x0D] = (video->hbp >> 8) & 0x3F;
	msg->msg[0x0E] = video->vfp & 0xFF;
	msg->msg[0x0F] = (video->vfp >> 8) & 0x3F;
	msg->msg[0x10] = video->vsw & 0xFF;
	msg->msg[0x11] = (video->vsw >> 8) & 0x3F;
	msg->msg[0x12] = video->vbp & 0xFF;
	msg->msg[0x13] = (video->vbp >> 8) & 0x3F;
	msg->msg[0x14] = (video->prog << 2) | (video->vpol << 1) | video->hpol;
	msg->msg[0x15] = video->v_aspect;
	msg->msg[0x16] = video->h_aspect & 0xFF;
	msg->msg[0x17] = video->h_aspect >> 8;
	msg->msg[0x18] = video->pix_rep;
	msg->msg[0x19] = video->vic;
	msg->msg[0x1A] = video->colorspace;
	msg->msg[0x1B] = 1;
}

static void it6162_mipi_set_d2v_video_timing(
		struct it6162 *it6162, struct videomode *vm,
		struct hdmi_avi_infoframe *avi_info)
{
	struct it6162_viedo video;
	struct it6162_infoblock_msg msg;

	it6162_get_video_setting(it6162, &video, vm, avi_info);
	it6162_pack_video_setting(it6162, &video, &msg);
	guard(mutex)(&it6162->lock);
	it6162_infoblock_write_msg(it6162, &msg);
}

static void it6162_show_hdcp(struct it6162 *it6162)
{
	int i, j, dev_count;
	u8 buf[DATA_BUFFER_DEPTH];

	guard(mutex)(&it6162->lock);
	/*read Bstatus & BKSV*/
	if (it6162_infoblock_request_data(
				it6162, HOST_SETTING_HDCP_R, 0x00, buf) < 0) {
		dev_err(it6162->dev, "read Bstatus & BKSV fail!!!!");
		return;
	}

	dev_info(it6162->dev, "Bstatus 0x%02x%02x", buf[1], buf[0]);
	dev_info(it6162->dev,
			"BKsv %02X %02X %02X %02X %02X",
			buf[2],
			buf[3],
			buf[4],
			buf[5],
			buf[6]);

	if (it6162->hdcp_version == HDCP_14)
		dev_count = buf[0] & 0x7F;
	else
		dev_count = (buf[1] & 0x01 << 8) | (buf[0] & 0xF0);
	if (dev_count > 31) {
		dev_err(it6162->dev, "dev_count %x over 30", dev_count);
		dev_count = 30;
	}

	if (!dev_count)
		return;

	for (i = 0; i < (dev_count / 6 + 1); i++) {
		/* KsV lists */
		if (it6162_infoblock_request_data(
					it6162, HOST_SETTING_HDCP_R, i + 1, buf) < 0) {
			dev_info(it6162->dev, "ksvlist %x fail!!!!!", i);
			return;
		}

		for (j = 0; j < 30; j += 5) {
			if ((i * 6 + j / 5) >= dev_count)
				break;
			dev_info(it6162->dev,
					"[%x] %02X %02X %02X %02X %02X",
					(i * 6 + j / 5),
					buf[j],
					buf[j + 1],
					buf[j + 2],
					buf[j + 3],
					buf[j + 4]);
		}

		return;
	}
}

static void it6162_hdcp_handler(struct it6162 *it6162)
{
	unsigned int int_status, tx_status, sink_cap;
	enum hdcp_state hdcp_sts;
	u8 hdcp_ver;

	dev_info(it6162->dev, "%s %d", __func__, it6162->en_hdcp);
	if (!it6162->en_hdcp)
		return;

	tx_status = it6162_infoblock_read(it6162, OFFSET_TX_STATUS);
	sink_cap = it6162_infoblock_read(it6162, OFFSET_SINK_CAP);
	dev_info(it6162->dev, "Tx status %x", tx_status);
	dev_info(it6162->dev, "SINK capability %x", sink_cap);

	if (it6162->tx_out_set.hdcp_version != NO_HDCP &&
			it6162->hdcp_version != NO_HDCP) {
		hdcp_sts = GET_TX_HDCP_STATUS(tx_status);
		hdcp_ver = GET_SINK_CAP_HDCP_VER(sink_cap);
		dev_info(it6162->dev,
				"hdcp %x->%x, ver %x-%x",
				it6162->hdcp_sts,
				hdcp_sts,
				it6162->hdcp_version,
				hdcp_ver);

		if (it6162->hdcp_sts != hdcp_sts ||
				it6162->hdcp_sts == NO_HDCP_STATE) {
			it6162->hdcp_sts = hdcp_sts;
			it6162->hdcp_version = hdcp_ver;
			switch (hdcp_sts) {
			case AUTH_DONE:
				dev_info(it6162->dev, "HDCP AUTH DONE");
				it6162_show_hdcp(it6162);
				break;
			case AUTH_FAIL:
				dev_info(it6162->dev, "HDCP AUTH FAIL");
				if (it6162->hdcp_version == HDCP_23) {
					it6162->hdcp_version = HDCP_14;
					it6162_tx_hdcp_setup(
							it6162, it6162->hdcp_version, true);
				} else {
					it6162_tx_hdcp_disable(it6162);
				}

				break;
			default:
				dev_info(it6162->dev, "HDCP NO AUTH");
				it6162_tx_hdcp_setup(
						it6162, it6162->hdcp_version, true);
				break;
			}
		}
	}
}

static void it6162_interrupt_handler(struct it6162 *it6162)
{
	unsigned int int_status, tx_status, mipi_status, sink_cap;
	enum drm_connector_status connector_status;

	int_status = it6162_infoblock_read(it6162, OFFSET_EVENT_CHG);
	it6162_infoblock_write(it6162, OFFSET_EVENT_CHG, 0xFF);

	if (!!GET_BUFFER_STATUS(int_status)) {
		dev_info(it6162->dev, "IRQ int_status = %x", int_status);
		it6162_infoblock_write(it6162, OFFSET_DRIVER_DATA, int_status);
		it6162->data_buf_sts = GET_BUFFER_STATUS(int_status);
	}

	if (!(int_status & EVENT_CHG))
		return;

	dev_info(it6162->dev, "evnet change");
	tx_status = it6162_infoblock_read(it6162, OFFSET_TX_STATUS);
	dev_info(it6162->dev, "Tx status %x", tx_status);

	mipi_status = it6162_infoblock_read(it6162, OFFSET_MIPI_STATUS);
	dev_info(it6162->dev, "MIPI status %x", mipi_status);

	sink_cap = it6162_infoblock_read(it6162, OFFSET_SINK_CAP);
	dev_info(it6162->dev, "SINK capability %x", sink_cap);

	connector_status = GET_TX_HPD_STATUS(tx_status)
		? connector_status_connected
		: connector_status_disconnected;

	if (it6162->connector_status != connector_status) {
		it6162->connector_status = connector_status;

		if (it6162->bridge_hpd_enable)
			drm_bridge_hpd_notify(&it6162->bridge,
					it6162->connector_status);

		it6162_audio_update_connector_status(it6162);
	}

	if (it6162->en_hdcp && connector_status == connector_status_connected)
		schedule_work(&it6162->hdcp_work);
}

static bool it6162_wait_devices(struct it6162 *it6162)
{
	struct device *dev = &it6162->it6162_i2c->dev;
	unsigned int status, i;
	unsigned int regEF;

	for (i = 0; i < 10; i++) {
		msleep(200);
		regEF = it6162_infoblock_read(it6162, OFFSET_HOST_SETTING);
		status = it6162_infoblock_read(it6162, OFFSET_EVENT_CHG);
		dev_err(dev, "wait 6162 rdy %x %x %u", status, regEF, i);

		if (status & EVENT_READY) {
			dev_info(dev, "IC status %01x", status);
			return true;
		}
		it6162_infoblock_write(it6162, 0x00, 0x00);
	}

	dev_err(dev, "-ENODEV %s %x", __func__, status);
	return false;
}

static void it6162_reset_init(struct it6162 *it6162)
{
	it6162_set_default_config(it6162);
}

static int it6162_platform_set_power(struct it6162 *it6162)
{
#ifdef ENABLE_POWER_RESET_CONTROL
	struct device *dev = it6162->dev;
	int err;

	if (it6162->ivdd) {
		err = regulator_enable(it6162->ivdd);
		if (err) {
			dev_err(dev, "Failed to enable IVDD: %d", err);
			return err;
		}
	}

	if (it6162->pwr18) {
		err = regulator_enable(it6162->pwr18);
		if (err) {
			dev_err(dev, "Failed to enable VDD18: %d", err);
			return err;
		}
	}

	if (it6162->ovdd) {
		err = regulator_enable(it6162->ovdd);
		if (err)
			return err;
	}

	if (it6162->gpiod_reset) {
		usleep_range(10000, 20000);
		gpiod_set_value_cansleep(it6162->gpiod_reset, 1);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(it6162->gpiod_reset, 0);
		usleep_range(10000, 20000);
	}

	// if (it6162->ivdd || it6162->pwr18 || it6162->ovdd || it6162->gpiod_reset)
	//	msleep(1500);
#endif
	return 0;
}

static int it6162_platform_clear_power(struct it6162 *it6162)
{
#ifdef ENABLE_POWER_RESET_CONTROL
	struct device *dev = it6162->dev;
	int err;

	dev_dbg(dev, "Clear powered start");

	if (it6162->ivdd) {
		err = regulator_disable(it6162->ivdd);
		if (err) {
			dev_err(dev, "Failed to disable IVDD: %d", err);
			return err;
		}
		usleep_range(2000, 3000);
	}

	if (it6162->pwr18) {
		err = regulator_disable(it6162->pwr18);
		if (err) {
			dev_err(dev, "Failed to disable VDD18: %d", err);
			return err;
		}
	}

	if (it6162->ovdd) {
		err = regulator_disable(it6162->ovdd);
		if (err)
			return err;
	}
#endif
	return 0;
}

static int it6162_detect_devices(struct it6162 *it6162)
{
	struct device *dev = &it6162->it6162_i2c->dev;
	const struct it6162_chip_info *chip_info;
	u32 chip_id, version;
	u8 buf[6];

	it6162_platform_set_power(it6162);

	if (!it6162_wait_devices(it6162))
		return -ENODEV;

	chip_info = of_device_get_match_data(dev);

	it6162_infoblock_bulk_read(it6162, OFFSET_CHIP_ID_L, &buf[0], 6);
	dev_info(dev, "chip id %02x %02x %02X", buf[0], buf[1], buf[2]);
	dev_info(dev, "chip VER %02x %02x %02x", buf[3], buf[4], buf[5]);

	chip_id = (buf[0] << 16) | (buf[1] << 8) | (buf[2]);
	version = (buf[3] << 16) | (buf[4] << 8) | (buf[5]);
	dev_info(dev, "chip id 0x%06x, version 0x%06x", chip_id, version);

	if (chip_id != chip_info->chip_id || version < chip_info->version) {
		dev_err(dev,
				"chip_id 0x%06x != 0x%06x",
				chip_id,
				chip_info->chip_id);
		dev_err(dev,
				"version 0x%06x != 0x%06x",
				version,
				chip_info->version);

		return -ENODEV;
	}

	it6162->chip_info.chip_id = chip_info->chip_id;
	it6162->chip_info.version = chip_info->version;

	return 0;
}

static int __maybe_unused it6162_poweron(struct it6162 *it6162)
{
	struct device *dev = it6162->dev;
	int err;

	dev_dbg(dev, "powered on Start");

	err = it6162_platform_set_power(it6162);

	if (err < 0)
		return err;

	/*wait for info block ready after power-on-rest*/
	if (!it6162_wait_devices(it6162))
		return -ENODEV;

	it6162->connector_status = connector_status_disconnected;
	it6162_reset_init(it6162);

	enable_irq(it6162->it6162_i2c->irq);
	dev_dbg(dev, "enable irq %d", it6162->it6162_i2c->irq);

	dev_info(dev, "it6162 poweron end");
	return 0;
}

static int __maybe_unused it6162_poweroff(struct it6162 *it6162)
{
	struct device *dev = it6162->dev;
	int err;

	dev_dbg(dev, "powered off start");
	disable_irq(it6162->it6162_i2c->irq);
	dev_dbg(dev, "disable irq %d", it6162->it6162_i2c->irq);
	err = it6162_platform_clear_power(it6162);
	if (err < 0)
		return err;

	dev_dbg(dev, "it6162 poweroff");
	return 0;
}

static void it6162_config_default(struct it6162 *it6162)
{
	struct it6162_mipirx_config *mipirx_cfg = &it6162->mipirx_config;
	struct it6162_tx_out_set *tx_out = &it6162->tx_out_set;
	struct it6162_audio *audio_config = &it6162->audio_config;

	mipirx_cfg->lane_num = 4;
	mipirx_cfg->pn_swap = false;
	mipirx_cfg->lane_swap = false;
	mipirx_cfg->en_port0 = false;
	mipirx_cfg->en_port1 = false;
	mipirx_cfg->continuous_clk = true;
	mipirx_cfg->mode = SYNC_EVENT;
	mipirx_cfg->format = MIPI_DSI_FMT_RGB888;
	mipirx_cfg->mode_flags = MIPI_DSI_MODE_VIDEO;

#ifdef EBABLE_HDCP
	tx_out->hdcp_version = HDCP_23;
	tx_out->hdcp_encyption = true;
#endif

	tx_out->stream_ID = 0;

	audio_config->select = I2S;
	audio_config->sample_rate = SAMPLE_RATE_48K;
	audio_config->type = LPCM;
	audio_config->word_length = WORD_LENGTH_16BIT;
	audio_config->channel_number = 2;

	it6162->connector_status = connector_status_disconnected;
}

static enum drm_connector_status it6162_detect(struct it6162 *it6162)
{
	unsigned int tx_status;

	tx_status = it6162_infoblock_read(it6162, OFFSET_TX_STATUS);
	it6162->connector_status = GET_TX_HPD_STATUS(tx_status)
		? connector_status_connected
		: connector_status_disconnected;

	dev_info(it6162->dev,
			"%s connector_status %x",
			__func__,
			it6162->connector_status);
	return it6162->connector_status;
}

static int it6162_get_edid_block(void *data, u8 *buf, unsigned int block,
		size_t len)
{
	struct it6162 *it6162 = data;
	unsigned int cnt;
	unsigned int i;
	u8 config;

	if (len > EDID_LENGTH)
		return -EINVAL;

	guard(mutex)(&it6162->lock);
	// dev_info(it6162->dev, "it6162_get_edid_block %d, %d", block,
	// (int)len);

	cnt = 0;

	for (i = 0; i < EDID_LENGTH; i += DATA_BUFFER_DEPTH, cnt++) {
		config = (block << 2) | (cnt);
		if (it6162_infoblock_request_data(
					it6162, HOST_SETTING_EDID_R, config, buf + i) < 0)
			return -EIO;
	}

	return 0;
}

static void it6162_enable_audio(struct it6162 *it6162)
{
	struct it6162_audio *config = &it6162->audio_config;

	guard(mutex)(&it6162->lock);
	it6162_infoblock_write(it6162, 0x3D, config->sample_rate);
	it6162_infoblock_write(it6162,
			0x3C,
			(config->channel_number) |
			(config->select << 4) | (config->type << 6));
	it6162_infoblock_host_set(it6162, HOST_SETTING_AUDIO_INFO);
}

static void it6162_disable_audio(struct it6162 *it6162)
{
	guard(mutex)(&it6162->lock);
	it6162_infoblock_write(it6162, 0x3C, 0x00);
	it6162_infoblock_host_set(it6162, HOST_SETTING_AUDIO_INFO);
}

static irqreturn_t it6162_int_threaded_handler(int unused, void *data)
{
	struct it6162 *it6162 = data;

	it6162_interrupt_handler(it6162);

	return IRQ_HANDLED;
}

static void it6162_audio_update_connector_status(struct it6162 *it6162)
{
	if (it6162->plugged_cb && it6162->codec_dev) {
		it6162->plugged_cb(
				it6162->codec_dev,
				it6162->connector_status == connector_status_connected);
	}
}

static int it6162_audio_update_hw_params(struct it6162 *it6162,
		struct hdmi_codec_daifmt *fmt,
		struct hdmi_codec_params *hparms)
{
	struct it6162_audio *config = &it6162->audio_config;

	hdmi_audio_infoframe_pack(
			&hparms->cea, &config->infoframe, sizeof(config->infoframe));

	memcpy(config->channel_status,
			&hparms->iec.status[0],
			AES_IEC958_STATUS_SIZE);

	config->channel_number = hparms->channels;

	switch (hparms->sample_rate) {
	case 32000:
		config->sample_rate = SAMPLE_RATE_32K;
		break;
	case 44100:
		config->sample_rate = SAMPLE_RATE_44_1K;
		break;
	case 48000:
		config->sample_rate = SAMPLE_RATE_48K;
		break;
	case 88200:
		config->sample_rate = SAMPLE_RATE_88_2K;
		break;
	case 96000:
		config->sample_rate = SAMPLE_RATE_96K;
		break;
	case 176400:
		config->sample_rate = SAMPLE_RATE_176_4K;
		break;
	case 192000:
		config->sample_rate = SAMPLE_RATE_192K;
		break;
	default:
		return -EINVAL;
	}

	switch (hparms->sample_width) {
	case 16:
		config->sample_width = WORD_LENGTH_16BIT;
		break;
	case 24:
		config->sample_width = WORD_LENGTH_18BIT;
		break;
		// case 18:
		//	config->sample_width = WORD_LENGTH_20BIT;
		//	break;
	case 20:
		config->sample_width = WORD_LENGTH_24BIT;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt->fmt) {
	case HDMI_I2S:
		config->select = I2S;
		break;
	case HDMI_SPDIF:
		config->select = SPDIF;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int it6162_hdmi_hw_params(struct device *dev, void *data,
		struct hdmi_codec_daifmt *fmt,
		struct hdmi_codec_params *hparms)
{
	struct it6162 *it6162 = dev_get_drvdata(dev);

	return it6162_audio_update_hw_params(it6162, fmt, hparms);
}

static int it6162_hdmi_audio_startup(struct device *dev, void *data)
{
	struct it6162 *it6162 = dev_get_drvdata(dev);

	it6162_enable_audio(it6162);

	return 0;
}

static void it6162_hdmi_audio_shutdown(struct device *dev, void *data)
{
	struct it6162 *it6162 = dev_get_drvdata(dev);

	it6162_disable_audio(it6162);
}

static int it6162_hdmi_audio_hook_plugged_cb(struct device *dev, void *data,
		hdmi_codec_plugged_cb fn,
		struct device *codec_dev)
{
	struct it6162 *it6162 = data;

	it6162->plugged_cb = fn;
	it6162->codec_dev = codec_dev;
	it6162_audio_update_connector_status(it6162);

	return 0;
}

static const struct hdmi_codec_ops it6162_audio_codec_ops = {
	.hw_params = it6162_hdmi_hw_params,
	.audio_shutdown = it6162_hdmi_audio_shutdown,
	.audio_startup = it6162_hdmi_audio_startup,
	.hook_plugged_cb = it6162_hdmi_audio_hook_plugged_cb,
};

static int it6162_hdmi_register_audio_driver(struct device *dev)
{
	struct it6162 *it6162 = dev_get_drvdata(dev);
	struct hdmi_codec_pdata codec_data = {
		.ops = &it6162_audio_codec_ops,
		.max_i2s_channels = 8,
		.i2s = 1,
		.data = it6162,
	};

	it6162->audio_pdev = platform_device_register_data(dev,
			HDMI_CODEC_DRV_NAME,
			PLATFORM_DEVID_AUTO,
			&codec_data,
			sizeof(codec_data));

	return PTR_ERR_OR_ZERO(it6162->audio_pdev);
}

void it6162_hdmi_audio_dev_exit(struct it6162 *it6162)
{
	if (it6162->audio_pdev) {
		platform_device_unregister(it6162->audio_pdev);
		it6162->audio_pdev = NULL;
	}
}

static void it6162_hdcp_work(struct work_struct *work)
{
	struct it6162 *it6162 = container_of(work, struct it6162, hdcp_work);
	it6162_hdcp_handler(it6162);
}

static int it6162_of_get_dsi_host(struct it6162 *it6162,
		struct mipi_dsi_host **host)
{
	struct device *dev = it6162->dev;
	struct device_node *host_node;
	struct device_node *endpoint;
	int port, node, ready;

	node = 0;
	ready = 0;
	for (port = 0; port < 2; port++) {
		endpoint =
			of_graph_get_endpoint_by_regs(dev->of_node, port, -1);
		if (!endpoint)
			continue;
		dev_info(dev, "find endpoint[%d]", port);
		host_node = of_graph_get_remote_port_parent(endpoint);
		of_node_put(endpoint);

		if (!host_node)
			continue;
		dev_info(dev, "fine remote endpoint[%d]", port);
		node++;
		host[port] = of_find_mipi_dsi_host_by_node(host_node);
		of_node_put(host_node);
		if (host[port])
			ready++;
	}

	dev_info(dev, "%s %d --> %d", __func__, node, ready);
	if (node == 0)
		return -ENODEV;

	if (node != ready) {
		dev_info(dev, "DSI host not ready");
		return -EPROBE_DEFER;
	}

	return 0;
}

static int it6162_attach_dsi(struct it6162 *it6162, struct mipi_dsi_host *host)
{
	struct device *dev = it6162->dev;
	const struct mipi_dsi_device_info info = {"it6162", 0, dev->of_node};
	struct mipi_dsi_device *dsi;
	int ret = 0;

	dsi = devm_mipi_dsi_device_register_full(dev, host, &info);
	if (IS_ERR(dsi)) {
		dev_err(dev, "failed to create dsi device");
		ret = PTR_ERR(dsi);
		return -ENODEV;
	}

	it6162->dsi = dsi;
	dsi->lanes = 4;
	dsi->format = it6162->mipirx_config.format;
	dsi->mode_flags = it6162->mipirx_config.mode_flags;
	ret = devm_mipi_dsi_attach(dev, dsi);

	return ret;
}

void it6162_load_mipi_pars(struct it6162 *it6162, struct device_node *endpoint)
{
	struct device *dev = it6162->dev;
	struct it6162_mipirx_config *mipirx = &it6162->mipirx_config;
	int dsi_lanes;

	dsi_lanes = drm_of_get_data_lanes_count(endpoint, 1, 4);

	if (dsi_lanes < 0)
		mipirx->lane_num = 4;
	else
		mipirx->lane_num = dsi_lanes;

	mipirx->pn_swap =
		of_property_present(endpoint, "ite,mipi-dsi-phy-pn-swap");
	mipirx->lane_swap =
		of_property_present(endpoint, "ite,mipi-dsi-phy-link-swap");

	if (of_property_present(endpoint,
				"ite,mipi-dsi-mode-video-sync-pulse")) {
		mipirx->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
		mipirx->mode = SYNC_PULSE;
	}

	if (of_property_present(endpoint, "ite,mipi-dsi-clock-non-continous")) {
		mipirx->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;
		mipirx->continuous_clk = false;
	}

	dev_info(dev,
			"lanes: %d pn_swap: %d, lane_swap: %d, mode_flags: %lu",
			mipirx->lane_num,
			mipirx->pn_swap,
			mipirx->lane_swap,
			mipirx->mode_flags);
}
static unsigned int it6162_parse_dt(struct it6162 *it6162)
{
	struct device *dev = it6162->dev;
	struct device_node *np = dev->of_node;
	struct it6162_mipirx_config *mipirx = &it6162->mipirx_config;
	struct device_node *endpoint;

	if (!np)
		return -EINVAL;

	/* get audio support*/
	it6162->support_audio = of_property_present(np, "ite,i2s-audio");
	/* GEt mipi link properity*/
	endpoint = of_graph_get_endpoint_by_regs(np, 0, -1);
	if (endpoint) {
		mipirx->en_port0 = true;
		it6162_load_mipi_pars(it6162, endpoint);
		of_node_put(endpoint);
	}

	endpoint = of_graph_get_endpoint_by_regs(np, 1, -1);
	if (endpoint) {
		mipirx->en_port1 = true;

		if (!mipirx->en_port0)
			it6162_load_mipi_pars(it6162, endpoint);
		of_node_put(endpoint);
	}
	return 0;
}

static int it6162_init_pdata(struct it6162 *it6162)
{
	struct device *dev = it6162->dev;

	it6162->ivdd = devm_regulator_get(dev, "ivdd");
	if (IS_ERR(it6162->ivdd)) {
		dev_err(dev, "ivdd regulator not found");
		// return PTR_ERR(it6162->ivdd);
	}

	it6162->pwr18 = devm_regulator_get(dev, "pwr18");
	if (IS_ERR(it6162->pwr18)) {
		dev_err(dev, "pwr18 regulator not found");
		// return PTR_ERR(it6162->pwr18);
	}

	it6162->ovdd = devm_regulator_get(dev, "ovdd");
	if (IS_ERR(it6162->ovdd)) {
		dev_err(dev, "ovdd regulator not found");
		// return PTR_ERR(it6162->ovdd);
	}

	it6162->gpiod_reset = devm_gpiod_get(dev, "reset-gpios", GPIOD_OUT_LOW);
	if (IS_ERR(it6162->gpiod_reset)) {
		dev_err(dev, "reset-gpios gpio not found");
		// return PTR_ERR(it6162->gpiod_reset);
	}
	return 0;
}

static int it6162_bridge_attach(struct drm_bridge *bridge,
		enum drm_bridge_attach_flags flags)
{
	struct it6162 *it6162 = bridge_to_it6162(bridge);
	struct device *dev = it6162->dev;
	struct drm_device *drm = bridge->dev;

	it6162->drm = drm;

	if (!drm_core_check_feature(drm, DRIVER_ATOMIC)) {
		dev_err(dev, "it6162 driver only copes with atomic updates");
		return -EOPNOTSUPP;
	}

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)) {
#ifdef SUPPORT_CREAT_CONNECTOR
		struct drm_connector *connector;
		int ret;

		connector = drm_bridge_connector_init(drm, bridge->encoder);
		if (IS_ERR(connector)) {
			DRM_ERROR("Unable to create bridge connector\n");
			return PTR_ERR(connector);
		}

		ret = drm_connector_attach_encoder(connector, bridge->encoder);
		if (ret < 0)
			return ret;
#else
		dev_err(dev, "DRM_BRIDGE_ATTACH_NO_CONNECTOR must be supplied");
		return -EINVAL;
#endif
	}

	return 0;
}

static void it6162_bridge_detach(struct drm_bridge *bridge) {}

static enum drm_mode_status it6162_bridge_mode_valid(
		struct drm_bridge *bridge, const struct drm_display_info *info,
		const struct drm_display_mode *mode)
{
	if (mode->clock > 300000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static enum drm_connector_status it6162_bridge_detect(struct drm_bridge *bridge)
{
	struct it6162 *it6162 = bridge_to_it6162(bridge);

	return it6162_detect(it6162);
}

static void it6162_bridge_hpd_enable(struct drm_bridge *bridge)
{
	struct it6162 *it6162 = bridge_to_it6162(bridge);

	it6162->bridge_hpd_enable = true;
}

static void it6162_bridge_hpd_disable(struct drm_bridge *bridge)
{
	struct it6162 *it6162 = bridge_to_it6162(bridge);

	it6162->bridge_hpd_enable = false;
}

static void it6162_bridge_atomic_enable(struct drm_bridge *bridge,
		struct drm_bridge_state *old_state)
{
	struct it6162 *it6162 = bridge_to_it6162(bridge);
	struct device *dev = it6162->dev;
	struct drm_atomic_state *state = old_state->base.state;
	struct drm_crtc_state *crtc_state;
	struct drm_connector_state *conn_state;
	struct drm_display_mode *mode;
	struct drm_connector *connector;
	struct videomode vm;
	struct hdmi_avi_infoframe avi_info;
	int ret;

	dev_info(it6162->dev, "it6162_bridge_atomic_enable");
	connector =
		drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);

	if (!connector)
		return;
	it6162->connector = *connector;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	mode = &crtc_state->adjusted_mode;

	it6162->is_hdmi = connector->display_info.is_hdmi;
	it6162->has_audio = connector->display_info.has_audio;

	dev_info(dev,
			"%s mode, monitor %ssupport audio",
			it6162->is_hdmi ? "HDMI" : "DVI",
			it6162->has_audio ? "" : "not ");

	if (it6162->is_hdmi) {
		ret = drm_hdmi_avi_infoframe_from_display_mode(
				&avi_info, connector, mode);
		if (ret)
			dev_err(dev, "Failed to setup AVI infoframe: %d", ret);
	}

	drm_display_mode_to_videomode(mode, &vm);

#ifdef FW_VIDEO_AUTO_MODE
	dev_info(it6162->dev, "SKIP VIDEO Setting");
#else
	it6162_mipi_set_d2v_video_timing(it6162, &vm, &avi_info);
#endif

	if (it6162->support_audio && it6162->has_audio)
		it6162_enable_audio(it6162);
	else
		it6162_disable_audio(it6162);

	it6162_tx_enable(it6162);
	it6162_mipi_enable(it6162);
}

static void it6162_bridge_atomic_disable(struct drm_bridge *bridge,
		struct drm_bridge_state *old_state)
{
	struct it6162 *it6162 = bridge_to_it6162(bridge);

	dev_info(it6162->dev, "it6162_bridge_atomic_disable");
	it6162_tx_disable(it6162);
	it6162_mipi_disable(it6162);
}

static struct edid *it6162_bridge_get_edid(struct drm_bridge *bridge,
		struct drm_connector *connector)
{
	struct it6162 *it6162 = bridge_to_it6162(bridge);
	struct device *dev = it6162->dev;
	const struct drm_edid *edid;

	dev_info(it6162->dev, "it6162_bridge_get_edid");
	edid = drm_edid_read_custom(connector, it6162_get_edid_block, it6162);
	if (!edid) {
		dev_err(dev, "failed to read EDID");
		return 0;
	}

	return (struct edid *)drm_edid_raw(edid);
}

static const struct drm_edid *it6162_bridge_read_edid(
		struct drm_bridge *bridge, struct drm_connector *connector)
{
	struct it6162 *it6162 = bridge_to_it6162(bridge);
	struct device *dev = it6162->dev;
	const struct drm_edid *edid;

	dev_info(it6162->dev, "it6162_bridge_read_edid");
	edid = drm_edid_read_custom(connector, it6162_get_edid_block, it6162);
	if (!edid) {
		dev_err(dev, "failed to read EDID");
		return 0;
	}

	return edid;
}

static void it6162_bridge_mode_set(struct drm_bridge *bridge,
		const struct drm_display_mode *mode,
		const struct drm_display_mode *adj)
{
	struct it6162 *it6162 = bridge_to_it6162(bridge);

	dev_info(it6162->dev, "it6162_bridge_mode_set");
}

static const struct drm_bridge_funcs it6162_bridge_funcs = {
	.attach = it6162_bridge_attach,
	.detach = it6162_bridge_detach,
	.mode_valid = it6162_bridge_mode_valid,
	.detect = it6162_bridge_detect,
	.hpd_enable = it6162_bridge_hpd_enable,
	.hpd_disable = it6162_bridge_hpd_disable,

	.atomic_enable = it6162_bridge_atomic_enable,
	.atomic_disable = it6162_bridge_atomic_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,

	.edid_read = it6162_bridge_read_edid,
	.get_edid = it6162_bridge_get_edid,
	.mode_set = it6162_bridge_mode_set,
};

static int it6162_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct mipi_dsi_host *host[2];
	struct it6162 *it6162;
	int i, ret;

	it6162 = devm_kzalloc(dev, sizeof(*it6162), GFP_KERNEL);
	if (!it6162)
		return -ENOMEM;

	it6162->dev = dev;

	ret = it6162_of_get_dsi_host(it6162, host);
	if (ret < 0)
		return ret;

	ret = it6162_i2c_regmap_init(client, it6162);
	if (ret != 0)
		return ret;

	ret = it6162_init_pdata(it6162);
	if (ret) {
		dev_err(dev, "Failed to initialize pdata: %d", ret);
		return ret;
	}

	it6162_config_default(it6162);
	it6162_parse_dt(it6162);

	if (it6162_detect_devices(it6162) < 0)
		return -ENODEV;

	if (!client->irq) {
		dev_err(dev, "Failed to get INTP IRQ");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(
			&client->dev,
			client->irq,
			NULL,
			it6162_int_threaded_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_AUTOEN,
			"it6162-intp",
			it6162);
	if (ret) {
		dev_err(dev, "Failed to request INTP threaded IRQ: %d", ret);
		return ret;
	}

	INIT_WORK(&it6162->hdcp_work, it6162_hdcp_work);

	mutex_init(&it6162->lock);

	/*register audio*/
	if (it6162->support_audio) {
		ret = it6162_hdmi_register_audio_driver(dev);
		if (ret < 0) {
			dev_err(
					dev, "Failed to register audio driver: %d", ret);
			return ret;
		}
	}

	it6162->bridge.funcs = &it6162_bridge_funcs;
	it6162->bridge.of_node = np;
	it6162->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID |
		DRM_BRIDGE_OP_MODES | DRM_BRIDGE_OP_HPD;
	it6162->bridge.type = DRM_MODE_CONNECTOR_HDMIA;

	devm_drm_bridge_add(dev, &it6162->bridge);

	for (i = 0; i < 2; i++) {
		if (host[i]) {
			ret = it6162_attach_dsi(it6162, host[i]);
			if (ret < 0) {
				dev_err(
						dev, "Failed attach_dsi[%d] %d", i, ret);
				return ret;
			}
		}
	}

	dev_info(it6162->dev, "driver build 20251027-001");
	it6162_poweron(it6162);

	return 0;
}

static void it6162_remove(struct i2c_client *client)
{
	struct it6162 *it6162 = i2c_get_clientdata(client);

	disable_irq(client->irq);
	cancel_work_sync(&it6162->hdcp_work);
	it6162_hdmi_audio_dev_exit(it6162);
	mutex_destroy(&it6162->lock);
}

static const struct it6162_chip_info it6162_chip_info = {
	.chip_id = 0x616200,
	.version = 0x006500,
};

static const struct of_device_id it6162_dt_ids[] = {
	{.compatible = "ite,it6162", .data = &it6162_chip_info}, {}};
MODULE_DEVICE_TABLE(of, it6162_dt_ids);

static const struct i2c_device_id it6162_i2c_ids[] = {
	{"it6162", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, it6162_i2c_ids);

static struct i2c_driver it6162_driver = {
	.driver = {
		.name = "it6162",
		.of_match_table = it6162_dt_ids,
	},
	.probe = it6162_probe,
	.remove = it6162_remove,
	.id_table = it6162_i2c_ids,
};

module_i2c_driver(it6162_driver);

MODULE_AUTHOR("Pet Weng <pet.weng@ite.com.tw>");
MODULE_AUTHOR("Hermes Wu <Hermes.Wu@ite.com.tw>");
MODULE_DESCRIPTION("it6162 mipi to hdmi driver");
MODULE_LICENSE("GPL v2");
