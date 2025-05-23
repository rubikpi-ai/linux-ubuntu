// SPDX-License-Identifier: GPL-2.0-only
/*
 * es8316.c -- es8316 ALSA SoC audio driver
 * Copyright Everest Semiconductor Co.,Ltd
 *
 * Author: David Yang <yangxiaohua@everest-semi.com>
 *
 * Based on es8316.c
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include <sound/jack.h>
#include <net/netlink.h>
#include <net/net_namespace.h>
#include "es8316.h"

#define INVALID_GPIO -1
#define GPIO_LOW  0
#define GPIO_HIGH 1
#define es8316_DEF_VOL	0xbb
#define MCLK 0
static int es8316_init_reg;
static struct snd_soc_component *es8316_component;

#ifdef ENABLE_NETLINK
static struct sock *nlsk;
#define NETLINK_TEST_ES8316 29
#endif

static const struct reg_default es8316_reg_defaults[] = {
	{0x00, 0x03}, {0x01, 0x03}, {0x02, 0x00}, {0x03, 0x20},
	{0x04, 0x11}, {0x05, 0x00}, {0x06, 0x11}, {0x07, 0x00},
	{0x08, 0x00}, {0x09, 0x01}, {0x0a, 0x00}, {0x0b, 0x00},
	{0x0c, 0xf8}, {0x0d, 0x3f}, {0x0e, 0x00}, {0x0f, 0x00},
	{0x10, 0x01}, {0x11, 0xfc}, {0x12, 0x28}, {0x13, 0x00},
	{0x14, 0x00}, {0x15, 0x33}, {0x16, 0x00}, {0x17, 0x00},
	{0x18, 0x88}, {0x19, 0x06}, {0x1a, 0x22}, {0x1b, 0x03},
	{0x1c, 0x0f}, {0x1d, 0x00}, {0x1e, 0x80}, {0x1f, 0x80},
	{0x20, 0x00}, {0x21, 0x00}, {0x22, 0x30}, {0x23, 0x00},
	{0x24, 0x01}, {0x25, 0x08}, {0x26, 0x10}, {0x27, 0xc0},
	{0x28, 0x00}, {0x29, 0x1c}, {0x2a, 0x00}, {0x2b, 0xb0},
	{0x2c, 0x32}, {0x2d, 0x03}, {0x2e, 0x00}, {0x2f, 0x11},
	{0x30, 0x10}, {0x31, 0x00}, {0x32, 0x00}, {0x33, 0x00},
	{0x34, 0x00}, {0x35, 0x1f}, {0x36, 0xf7}, {0x37, 0xfd},
	{0x38, 0xff}, {0x39, 0x1f}, {0x3a, 0xf7}, {0x3b, 0xfd},
	{0x3c, 0xff}, {0x3d, 0x1f}, {0x3e, 0xf7}, {0x3f, 0xfd},
	{0x40, 0xff}, {0x41, 0x1f}, {0x42, 0xf7}, {0x43, 0xfd},
	{0x44, 0xff}, {0x45, 0x1f}, {0x46, 0xf7}, {0x47, 0xfd},
	{0x48, 0xff}, {0x49, 0x1f}, {0x4a, 0xf7}, {0x4b, 0xfd},
	{0x4c, 0xff}, {0x4d, 0x00}, {0x4e, 0x00},
	{0x50, 0x00}, {0x51, 0x00}, {0x52, 0x00}, {0x53, 0x00},
};

/* codec private data */
struct es8316_priv {
	struct regmap *regmap;
	unsigned int dmic_amic;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	struct clk *mclk;
	int debounce_time;
	int hp_det_invert;
	struct delayed_work work;
	struct delayed_work pcm_pop_work;
	int spk_ctl_gpio;
	int hp_det_gpio;
	bool muted;
	bool hp_inserted;
	bool spk_active_level;
	int power_en_gpio;
	int pwr_count;
	struct mutex lock;
	struct snd_soc_jack *jack;
	int irq;
	int irq_gpio;
	bool jd;
};

static int detect;

enum es8316_connection_status {
	ES8316_DISCONNECT = 0,
	ES8316_CONNECTION = 1,
};

const char *const es8316_status_string[] = {
	[ES8316_DISCONNECT] = "Headset Disconnect",
	[ES8316_CONNECTION] = "Headset Connection",
};

/*
 * es8316_reset
 * write value 0xff to reg0x00, the chip will be in reset mode
 * then, writer 0x00 to reg0x00, unreset the chip
 */
static int es8316_reset(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8316_RESET_REG00, 0x3F);
	usleep_range(5000, 5500);
	return snd_soc_component_write(component, ES8316_RESET_REG00, 0x03);
}

static void pcm_pop_work_events(struct work_struct *work)
{
	struct snd_soc_component *component = es8316_component;

	/*oepn dac output here*/
	snd_soc_component_write(component, ES8316_CLKMGR_CLKSW_REG01, 0x7F);
	snd_soc_component_write(component, ES8316_SYS_PDN_REG0D, 0x00);
	snd_soc_component_write(component, ES8316_SYS_LP1_REG0E, 0x00);
	snd_soc_component_write(component, ES8316_SYS_LP2_REG0F, 0x00);
	snd_soc_component_write(component, ES8316_DAC_PDN_REG2F, 0x00);
	snd_soc_component_write(component, ES8316_HPMIX_SWITCH_REG14, 0x88);
	snd_soc_component_write(component, ES8316_HPMIX_PDN_REG15, 0x00);
	snd_soc_component_write(component, ES8316_HPMIX_VOL_REG16, 0xBB);
	snd_soc_component_write(component, ES8316_CPHP_PDN2_REG1A, 0x10);
	snd_soc_component_write(component, ES8316_CPHP_LDOCTL_REG1B, 0x30);
	snd_soc_component_write(component, ES8316_CPHP_PDN1_REG19, 0x03);
	snd_soc_component_write(component, ES8316_CPHP_ICAL_VOL_REG18, 0x11);
	snd_soc_component_write(component, ES8316_ADC_PDN_LINSEL_REG22, 0x30);
	msleep(20);
	snd_soc_component_write(component, ES8316_RESET_REG00, 0xC0);
	snd_soc_component_write(component, ES8316_CPHP_OUTEN_REG17, 0x66);
	snd_soc_component_write(component, ES8316_DAC_VOLL_REG33, 0x00);
	snd_soc_component_write(component, ES8316_DAC_VOLR_REG34, 0x00);
	es8316_init_reg = 1;
}

static void es8316_enable_spk(struct es8316_priv *es8316, bool enable)
{
	bool level;

	level = enable ? es8316->spk_active_level : !es8316->spk_active_level;
	gpio_set_value(es8316->spk_ctl_gpio, level);
}

static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -9600, 50, 1);
static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -9600, 50, 1);
static const DECLARE_TLV_DB_SCALE(hpmixer_gain_tlv, -1200, 150, 0);
static const DECLARE_TLV_DB_SCALE(mic_bst_tlv, 0, 1200, 0);

static unsigned int linin_pga_tlv[] = {
	TLV_DB_RANGE_HEAD(9),
	0, 0, TLV_DB_SCALE_ITEM(0, 0, 0),
	1, 1, TLV_DB_SCALE_ITEM(300, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(600, 0, 0),
	3, 3, TLV_DB_SCALE_ITEM(900, 0, 0),
	4, 4, TLV_DB_SCALE_ITEM(1200, 0, 0),
	5, 5, TLV_DB_SCALE_ITEM(1500, 0, 0),
	6, 6, TLV_DB_SCALE_ITEM(1800, 0, 0),
	7, 7, TLV_DB_SCALE_ITEM(2100, 0, 0),
	8, 8, TLV_DB_SCALE_ITEM(2400, 0, 0),
};

static unsigned int hpout_vol_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 3, TLV_DB_SCALE_ITEM(-4800, 1200, 0),
};

static const char *const alc_func_txt[] = { "Off", "On" };

static const struct soc_enum alc_func =
	SOC_ENUM_SINGLE(ES8316_ADC_ALC1_REG29, 6, 2, alc_func_txt);

static const char *const ng_type_txt[] = {
	"Constant PGA Gain", "Mute ADC Output" };

static const struct soc_enum ng_type =
	SOC_ENUM_SINGLE(ES8316_ADC_ALC6_REG2E, 6, 2, ng_type_txt);

static const char *const adcpol_txt[] = { "Normal", "Invert" };

static const struct soc_enum adcpol =
	SOC_ENUM_SINGLE(ES8316_ADC_MUTE_REG26, 1, 2, adcpol_txt);

static const char *const dacpol_txt[] = {
	"Normal", "R Invert", "L Invert", "L + R Invert" };

static const struct soc_enum dacpol =
	SOC_ENUM_SINGLE(ES8316_DAC_SET1_REG30, 0, 4, dacpol_txt);

static const struct snd_kcontrol_new es8316_snd_controls[] = {
	/* HP OUT VOLUME */
	SOC_DOUBLE_TLV("HP Playback Volume", ES8316_CPHP_ICAL_VOL_REG18,
		       4, 0, 4, 1, hpout_vol_tlv),
	/* HPMIXER VOLUME Control */
	SOC_DOUBLE_TLV("HPMixer Gain", ES8316_HPMIX_VOL_REG16,
		       0, 4, 7, 0, hpmixer_gain_tlv),

	/* DAC Digital controls */
	SOC_DOUBLE_R_TLV("DAC Playback Volume", ES8316_DAC_VOLL_REG33,
			 ES8316_DAC_VOLR_REG34, 0, 0xC0, 1, dac_vol_tlv),

	SOC_SINGLE("Enable DAC Soft Ramp", ES8316_DAC_SET1_REG30, 4, 1, 1),
	SOC_SINGLE("DAC Soft Ramp Rate", ES8316_DAC_SET1_REG30, 2, 4, 0),

	SOC_ENUM("Playback Polarity", dacpol),
	SOC_SINGLE("DAC Notch Filter", ES8316_DAC_SET2_REG31, 6, 1, 0),
	SOC_SINGLE("DAC Double Fs Mode", ES8316_DAC_SET2_REG31, 7, 1, 0),
	SOC_SINGLE("DAC Volume Control-LeR", ES8316_DAC_SET2_REG31, 2, 1, 0),
	SOC_SINGLE("DAC Stereo Enhancement", ES8316_DAC_SET3_REG32, 0, 7, 0),

	/* +20dB D2SE PGA Control */
	SOC_SINGLE_TLV("MIC Boost", ES8316_ADC_D2SEPGA_REG24,
		       0, 1, 0, mic_bst_tlv),
	/* 0-+24dB Lineinput PGA Control */
	SOC_SINGLE_TLV("Input PGA", ES8316_ADC_PGAGAIN_REG23,
		       4, 8, 0, linin_pga_tlv),

	/* ADC Digital  Control */
	SOC_SINGLE_TLV("ADC Capture Volume", ES8316_ADC_VOLUME_REG27,
		       0, 0xC0, 1, adc_vol_tlv),
	SOC_SINGLE("ADC Soft Ramp", ES8316_ADC_MUTE_REG26, 4, 1, 0),
	SOC_ENUM("Capture Polarity", adcpol),
	SOC_SINGLE("ADC Double FS Mode", ES8316_ADC_DMIC_REG25, 4, 1, 0),
	/* ADC ALC  Control */
	SOC_SINGLE("ALC Capture Target Volume",
		   ES8316_ADC_ALC3_REG2B, 4, 10, 0),
	SOC_SINGLE("ALC Capture Max PGA", ES8316_ADC_ALC1_REG29, 0, 28, 0),
	SOC_SINGLE("ALC Capture Min PGA", ES8316_ADC_ALC2_REG2A, 0, 28, 0),
	SOC_ENUM("ALC Capture Function", alc_func),
	SOC_SINGLE("ALC Capture Hold Time", ES8316_ADC_ALC3_REG2B, 0, 10, 0),
	SOC_SINGLE("ALC Capture Decay Time", ES8316_ADC_ALC4_REG2C, 4, 10, 0),
	SOC_SINGLE("ALC Capture Attack Time", ES8316_ADC_ALC4_REG2C, 0, 10, 0),
	SOC_SINGLE("ALC Capture NG Threshold", ES8316_ADC_ALC6_REG2E, 0, 31, 0),
	SOC_ENUM("ALC Capture NG Type", ng_type),
	SOC_SINGLE("ALC Capture NG Switch", ES8316_ADC_ALC6_REG2E, 5, 1, 0),
};

/* Analog Input MUX */
static const char * const es8316_analog_in_txt[] = {
	"lin1-rin1",
	"lin2-rin2",
	"lin1-rin1 with 20db Boost",
	"lin2-rin2 with 20db Boost"
};

static const unsigned int es8316_analog_in_values[] = { 0, 1, 2, 3 };

static const struct soc_enum es8316_analog_input_enum =
	SOC_VALUE_ENUM_SINGLE(ES8316_ADC_PDN_LINSEL_REG22, 4, 3,
			      ARRAY_SIZE(es8316_analog_in_txt),
			      es8316_analog_in_txt,
			      es8316_analog_in_values);

static const struct snd_kcontrol_new es8316_analog_in_mux_controls =
	SOC_DAPM_ENUM("Route", es8316_analog_input_enum);

/* Dmic MUX */
static const char * const es8316_dmic_txt[] = {
	"dmic disable",
	"dmic data at high level",
	"dmic data at low level",
};

static const unsigned int es8316_dmic_values[] = { 0, 1, 2 };

static const struct soc_enum es8316_dmic_src_enum =
	SOC_VALUE_ENUM_SINGLE(ES8316_ADC_DMIC_REG25, 0, 3,
			      ARRAY_SIZE(es8316_dmic_txt),
			      es8316_dmic_txt,
			      es8316_dmic_values);

static const struct snd_kcontrol_new es8316_dmic_src_controls =
	SOC_DAPM_ENUM("Route", es8316_dmic_src_enum);

/* hp mixer mux */
static const char *const es8316_hpmux_texts[] = {
	"lin1-rin1",
	"lin2-rin2",
	"lin-rin with Boost",
	"lin-rin with Boost and PGA"
};

static const unsigned int es8316_hpmux_values[] = { 0, 1, 2, 3 };

static const struct soc_enum es8316_left_hpmux_enum =
	SOC_VALUE_ENUM_SINGLE(ES8316_HPMIX_SEL_REG13, 4, 7,
			      ARRAY_SIZE(es8316_hpmux_texts),
			      es8316_hpmux_texts,
			      es8316_hpmux_values);

static const struct snd_kcontrol_new es8316_left_hpmux_controls =
	SOC_DAPM_ENUM("Route", es8316_left_hpmux_enum);

static const struct soc_enum es8316_right_hpmux_enum =
	SOC_VALUE_ENUM_SINGLE(ES8316_HPMIX_SEL_REG13, 0, 7,
			      ARRAY_SIZE(es8316_hpmux_texts),
			      es8316_hpmux_texts,
			      es8316_hpmux_values);

static const struct snd_kcontrol_new es8316_right_hpmux_controls =
	SOC_DAPM_ENUM("Route", es8316_right_hpmux_enum);

/* headphone Output Mixer */
static const struct snd_kcontrol_new es8316_out_left_mix[] = {
	SOC_DAPM_SINGLE("LLIN Switch", ES8316_HPMIX_SWITCH_REG14,
			6, 1, 0),
	SOC_DAPM_SINGLE("Left DAC Switch", ES8316_HPMIX_SWITCH_REG14,
			7, 1, 0),
};

static const struct snd_kcontrol_new es8316_out_right_mix[] = {
	SOC_DAPM_SINGLE("RLIN Switch", ES8316_HPMIX_SWITCH_REG14,
			2, 1, 0),
	SOC_DAPM_SINGLE("Right DAC Switch", ES8316_HPMIX_SWITCH_REG14,
			3, 1, 0),
};

/* DAC data source mux */
static const char * const es8316_dacsrc_texts[] = {
	"LDATA TO LDAC, RDATA TO RDAC",
	"LDATA TO LDAC, LDATA TO RDAC",
	"RDATA TO LDAC, RDATA TO RDAC",
	"RDATA TO LDAC, LDATA TO RDAC",
};

static const unsigned int es8316_dacsrc_values[] = { 0, 1, 2, 3 };

static const struct soc_enum es8316_dacsrc_mux_enum =
	SOC_VALUE_ENUM_SINGLE(ES8316_DAC_SET1_REG30, 6, 4,
			      ARRAY_SIZE(es8316_dacsrc_texts),
			      es8316_dacsrc_texts,
			      es8316_dacsrc_values);
static const struct snd_kcontrol_new es8316_dacsrc_mux_controls =
	SOC_DAPM_ENUM("Route", es8316_dacsrc_mux_enum);

static const struct snd_soc_dapm_widget es8316_dapm_widgets[] = {
	/* Input Lines */
	SND_SOC_DAPM_INPUT("DMIC"),
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),

	SND_SOC_DAPM_MICBIAS("micbias", SND_SOC_NOPM, 0, 0),
	/* Input MUX */
	SND_SOC_DAPM_MUX("Differential Mux", SND_SOC_NOPM, 0, 0,
			 &es8316_analog_in_mux_controls),

	SND_SOC_DAPM_PGA("Line input PGA", ES8316_ADC_PDN_LINSEL_REG22,
			 7, 1, NULL, 0),

	/* ADCs */
	SND_SOC_DAPM_ADC("Mono ADC", NULL, ES8316_ADC_PDN_LINSEL_REG22, 6, 1),

	/* Dmic MUX */
	SND_SOC_DAPM_MUX("Digital Mic Mux", SND_SOC_NOPM, 0, 0,
			 &es8316_dmic_src_controls),

	/* Digital Interface */
	SND_SOC_DAPM_AIF_OUT("I2S OUT", "I2S1 Capture",  1,
			     ES8316_SDP_ADCFMT_REG0A, 6, 0),

	SND_SOC_DAPM_AIF_IN("I2S IN", "I2S1 Playback", 0,
			    SND_SOC_NOPM, 0, 0),

	/*  DACs DATA SRC MUX */
	SND_SOC_DAPM_MUX("DAC SRC Mux", SND_SOC_NOPM, 0, 0,
			 &es8316_dacsrc_mux_controls),
	/*  DACs  */
	SND_SOC_DAPM_DAC("Right DAC", NULL, ES8316_DAC_PDN_REG2F, 0, 1),
	SND_SOC_DAPM_DAC("Left DAC", NULL, ES8316_DAC_PDN_REG2F, 4, 1),

	/* Headphone Output Side */
	/* hpmux for hp mixer */
	SND_SOC_DAPM_MUX("Left Hp mux", SND_SOC_NOPM, 0, 0,
			 &es8316_left_hpmux_controls),
	SND_SOC_DAPM_MUX("Right Hp mux", SND_SOC_NOPM, 0, 0,
			 &es8316_right_hpmux_controls),
	/* Output mixer  */
	SND_SOC_DAPM_MIXER("Left Hp mixer", ES8316_HPMIX_PDN_REG15,
			   4, 1, &es8316_out_left_mix[0],
			   ARRAY_SIZE(es8316_out_left_mix)),
	SND_SOC_DAPM_MIXER("Right Hp mixer", ES8316_HPMIX_PDN_REG15,
			   0, 1, &es8316_out_right_mix[0],
			   ARRAY_SIZE(es8316_out_right_mix)),
	SND_SOC_DAPM_MIXER("Left Hp mixer", SND_SOC_NOPM,
			   4, 1, &es8316_out_left_mix[0],
			   ARRAY_SIZE(es8316_out_left_mix)),
	SND_SOC_DAPM_MIXER("Right Hp mixer", SND_SOC_NOPM,
			   0, 1, &es8316_out_right_mix[0],
			   ARRAY_SIZE(es8316_out_right_mix)),
	/* Output charge pump */
	SND_SOC_DAPM_PGA("HPCP L", SND_SOC_NOPM,
			 6, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HPCP R", SND_SOC_NOPM,
			 2, 0, NULL, 0),
	/* Output Driver */
	SND_SOC_DAPM_PGA("HPVOL L", SND_SOC_NOPM,
			 5, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HPVOL R", SND_SOC_NOPM,
			 1, 0, NULL, 0),
	/* Output Lines */
	SND_SOC_DAPM_OUTPUT("HPOL"),
	SND_SOC_DAPM_OUTPUT("HPOR"),
};

static const struct snd_soc_dapm_route es8316_dapm_routes[] = {
	/*
	 * record route map
	 */
	{"MIC1", NULL, "micbias"},
	{"MIC2", NULL, "micbias"},
	{"DMIC", NULL, "micbias"},

	{"Differential Mux", "lin1-rin1", "MIC1"},
	{"Differential Mux", "lin2-rin2", "MIC2"},
	{"Line input PGA", NULL, "Differential Mux"},

	{"Mono ADC", NULL, "Line input PGA"},

	{"Digital Mic Mux", "dmic disable", "Mono ADC"},
	{"Digital Mic Mux", "dmic data at high level", "DMIC"},
	{"Digital Mic Mux", "dmic data at low level", "DMIC"},

	{"I2S OUT", NULL, "Digital Mic Mux"},
	/*
	 * playback route map
	 */
	{"DAC SRC Mux", "LDATA TO LDAC, RDATA TO RDAC", "I2S IN"},
	{"DAC SRC Mux", "LDATA TO LDAC, LDATA TO RDAC", "I2S IN"},
	{"DAC SRC Mux", "RDATA TO LDAC, RDATA TO RDAC", "I2S IN"},
	{"DAC SRC Mux", "RDATA TO LDAC, LDATA TO RDAC", "I2S IN"},

	{"Left DAC", NULL, "DAC SRC Mux"},
	{"Right DAC", NULL, "DAC SRC Mux"},

	{"Left Hp mux", "lin1-rin1", "MIC1"},
	{"Left Hp mux", "lin2-rin2", "MIC2"},
	{"Left Hp mux", "lin-rin with Boost", "Differential Mux"},
	{"Left Hp mux", "lin-rin with Boost and PGA", "Line input PGA"},

	{"Right Hp mux", "lin1-rin1", "MIC1"},
	{"Right Hp mux", "lin2-rin2", "MIC2"},
	{"Right Hp mux", "lin-rin with Boost", "Differential Mux"},
	{"Right Hp mux", "lin-rin with Boost and PGA", "Line input PGA"},

	{"Left Hp mixer", "LLIN Switch", "Left Hp mux"},
	{"Left Hp mixer", "Left DAC Switch", "Left DAC"},

	{"Right Hp mixer", "RLIN Switch", "Right Hp mux"},
	{"Right Hp mixer", "Right DAC Switch", "Right DAC"},

	{"HPCP L", NULL, "Left Hp mixer"},
	{"HPCP R", NULL, "Right Hp mixer"},

	{"HPVOL L", NULL, "HPCP L"},
	{"HPVOL R", NULL, "HPCP R"},

	{"HPOL", NULL, "HPVOL L"},
	{"HPOR", NULL, "HPVOL R"},
};

struct _coeff_div {
	u32 mclk;       /*mclk frequency*/
	u32 rate;       /*sample rate*/
	u8 div;         /*adcclk and dacclk divider*/
	u8 lrck_h;      /*adclrck divider and daclrck divider*/
	u8 lrck_l;
	u8 sr;          /*sclk divider*/
	u8 osr;         /*adc osr*/
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{ 12288000, 8000, 6, 0x06, 0x00, 21, 32 },
	{ 11289600, 8000, 6, 0x05, 0x83, 20, 29 },
	{ 18432000, 8000, 9, 0x09, 0x00, 27, 32 },
	{ 16934400, 8000, 8, 0x08, 0x44, 25, 33 },
	{ 12000000, 8000, 7, 0x05, 0xdc, 21, 25 },
	{ 19200000, 8000, 12, 0x09, 0x60, 27, 25 },

	/* 11.025k */
	{ 11289600, 11025, 4, 0x04, 0x00, 16, 32 },
	{ 16934400, 11025, 6, 0x06, 0x00, 21, 32 },
	{ 12000000, 11025, 4, 0x04, 0x40, 17, 34 },

	/* 16k */
	{ 12288000, 16000, 3, 0x03, 0x00, 12, 32 },
	{ 18432000, 16000, 5, 0x04, 0x80, 18, 25 },
	{ 12000000, 16000, 3, 0x02, 0xee, 12, 31 },
	{ 19200000, 16000, 6, 0x04, 0xb0, 18, 25 },

	/* 22.05k */
	{ 11289600, 22050, 2, 0x02, 0x00, 8, 32 },
	{ 16934400, 22050, 3, 0x03, 0x00, 12, 32 },
	{ 12000000, 22050, 2, 0x02, 0x20, 8, 34 },

	/* 32k */
	{ 12288000, 32000, 1, 0x01, 0x80, 6, 48 },
	{ 18432000, 32000, 2, 0x02, 0x40, 9, 32 },
	{ 12000000, 32000, 1, 0x01, 0x77, 6, 31 },
	{ 19200000, 32000, 3, 0x02, 0x58, 10, 25 },

	/* 44.1k */
	{ 11289600, 44100, 1, 0x01, 0x00, 4, 32 },
	{ 16934400, 44100, 1, 0x01, 0x80, 6, 32 },
	{ 12000000, 44100, 1, 0x01, 0x10, 4, 34 },

	/* 48k */
	{ 12288000, 48000, 1, 0x01, 0x00, 4, 32 },
	{ 18432000, 48000, 1, 0x01, 0x80, 6, 32 },
	{ 12000000, 48000, 1, 0x00, 0xfa, 4, 31 },
	{ 19200000, 48000, 2, 0x01, 0x90, 6, 25 },

	/* 88.2k */
	{ 11289600, 88200, 1, 0x00, 0x80, 2, 32 },
	{ 16934400, 88200, 1, 0x00, 0xc0, 3, 48 },
	{ 12000000, 88200, 1, 0x00, 0x88, 2, 34 },

	/* 96k */
	{ 12288000, 96000, 1, 0x00, 0x80, 2, 32 },
	{ 18432000, 96000, 1, 0x00, 0xc0, 3, 48 },
	{ 12000000, 96000, 1, 0x00, 0x7d, 1, 31 },
	{ 19200000, 96000, 1, 0x00, 0xc8, 3, 25 },
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	return -EINVAL;
}

/* The set of rates we can generate from the above for each SYSCLK */

static unsigned int rates_12288[] = {
	8000, 12000, 16000, 24000, 24000, 32000, 48000, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12288 = {
	.count	= ARRAY_SIZE(rates_12288),
	.list	= rates_12288,
};

static unsigned int rates_112896[] = {
	8000, 11025, 22050, 44100,
};

static struct snd_pcm_hw_constraint_list constraints_112896 = {
	.count	= ARRAY_SIZE(rates_112896),
	.list	= rates_112896,
};

static unsigned int rates_12[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
	48000, 88235, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12 = {
	.count	= ARRAY_SIZE(rates_12),
	.list	= rates_12,
};

/*
 * Note that this should be called from init rather than from hw_params.
 */
static int es8316_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);

	switch (freq) {
	case 11289600:
	case 18432000:
	case 22579200:
	case 36864000:
		es8316->sysclk_constraints = &constraints_112896;
		es8316->sysclk = freq;
		return 0;
	case 12288000:
	case 19200000:
	case 16934400:
	case 24576000:
	case 33868800:
		es8316->sysclk_constraints = &constraints_12288;
		es8316->sysclk = freq;
		return 0;
	case 12000000:
	case 24000000:
		es8316->sysclk_constraints = &constraints_12;
		es8316->sysclk = freq;
		return 0;
	}


	return 0;
}

static int es8316_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 iface = 0;
	u8 adciface = 0;
	u8 daciface = 0;

	iface    = snd_soc_component_read(component, ES8316_IFACE);
	adciface = snd_soc_component_read(component, ES8316_ADC_IFACE);
	daciface = snd_soc_component_read(component, ES8316_DAC_IFACE);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x80;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		iface &= 0x7F;
		break;
	default:
		return -EINVAL;
	}

	/* interface format */

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		adciface &= 0xFC;
		daciface &= 0xFC;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		return -EINVAL;
	case SND_SOC_DAIFMT_LEFT_J:
		adciface &= 0xFC;
		daciface &= 0xFC;
		adciface |= 0x01;
		daciface |= 0x01;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		adciface &= 0xDC;
		daciface &= 0xDC;
		adciface |= 0x03;
		daciface |= 0x03;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		adciface &= 0xDC;
		daciface &= 0xDC;
		adciface |= 0x23;
		daciface |= 0x23;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		iface    &= 0xDF;
		adciface &= 0xDF;
		daciface &= 0xDF;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface    |= 0x20;
		adciface |= 0x20;
		daciface |= 0x20;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface    |= 0x20;
		adciface &= 0xDF;
		daciface &= 0xDF;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface    &= 0xDF;
		adciface |= 0x20;
		daciface |= 0x20;
		break;
	default:
		return -EINVAL;
	}
	snd_soc_component_write(component, ES8316_IFACE, iface);
	snd_soc_component_write(component, ES8316_ADC_IFACE, adciface);
	snd_soc_component_write(component, ES8316_DAC_IFACE, daciface);
	return 0;
}

static int es8316_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	bool playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	pr_info("%s: (%s) enter\n", __func__, playback ? "playback" : "capture");

	return 0;
}

static void es8316_pcm_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	bool playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	pr_info("%s: (%s) enter\n", __func__, playback ? "playback" : "capture");
}

static int es8316_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);
	int val = 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		val = ES8316_DACWL_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val = ES8316_DACWL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val = ES8316_DACWL_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		val = ES8316_DACWL_32;
		break;
	default:
		val = ES8316_DACWL_16;
		break;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_component_update_bits(component, ES8316_SDP_DACFMT_REG0B,
				    ES8316_DACWL_MASK, val);
	else
		snd_soc_component_update_bits(component, ES8316_SDP_ADCFMT_REG0A,
				    ES8316_ADCWL_MASK, val);

	if (es8316_init_reg == 0) {
		pr_info("%s: enable delayed work\n", __func__);
		queue_delayed_work(system_wq, &es8316->pcm_pop_work,
								msecs_to_jiffies(20));
	}

	return 0;
}

static int es8316_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);

	es8316->muted = mute;

	pr_info("%s: mute(%d) direction(%d)\n", __func__, mute, direction);

	if (direction == SNDRV_PCM_STREAM_PLAYBACK) {
		if (mute)
			snd_soc_component_write(component, ES8316_DAC_SET1_REG30, 0x20);
		else
			snd_soc_component_write(component, ES8316_DAC_SET1_REG30, 0x00);
	}

	return 0;
}

static int es8316_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	pr_info("%s: level(%d)\n", __func__, level);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}

	return 0;
}

#define es8316_RATES SNDRV_PCM_RATE_8000_96000

#define es8316_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_dai_ops es8316_ops = {
	.startup = es8316_pcm_startup,
	.hw_params = es8316_pcm_hw_params,
	.set_fmt = es8316_set_dai_fmt,
	.set_sysclk = es8316_set_dai_sysclk,
	.mute_stream = es8316_mute,
	.shutdown = es8316_pcm_shutdown,
};

static struct snd_soc_dai_driver es8316_dai = {
	.name = "ES8316 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es8316_RATES,
		.formats = es8316_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es8316_RATES,
		.formats = es8316_FORMATS,
	},
	.ops = &es8316_ops,
	.symmetric_rate = 1,
};

static int es8316_init_regs(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8316_RESET_REG00, 0x3f);
	usleep_range(5000, 5500);
	snd_soc_component_write(component, ES8316_RESET_REG00, 0x00);
	snd_soc_component_write(component, ES8316_SYS_VMIDSEL_REG0C, 0xFF);
	msleep(30);
	snd_soc_component_write(component, ES8316_CLKMGR_CLKSEL_REG02, 0x09);
	snd_soc_component_write(component, ES8316_CLKMGR_ADCOSR_REG03, 0x20);
	snd_soc_component_write(component, ES8316_CLKMGR_ADCDIV1_REG04, 0x11);
	snd_soc_component_write(component, ES8316_CLKMGR_ADCDIV2_REG05, 0x00);
	snd_soc_component_write(component, ES8316_CLKMGR_DACDIV1_REG06, 0x11);
	snd_soc_component_write(component, ES8316_CLKMGR_DACDIV2_REG07, 0x00);
	snd_soc_component_write(component, ES8316_CLKMGR_CPDIV_REG08, 0x00);
	snd_soc_component_write(component, ES8316_SDP_MS_BCKDIV_REG09, 0x04);
	snd_soc_component_write(component, ES8316_CLKMGR_CLKSW_REG01, 0x7F);

	snd_soc_component_write(component, ES8316_ADC_PDN_LINSEL_REG22, 0x30);
	snd_soc_component_write(component, ES8316_ADC_D2SEPGA_REG24, 0x00);
	snd_soc_component_write(component, ES8316_ADC_VOLUME_REG27, 0x00);
	snd_soc_component_write(component, ES8316_DAC_SET2_REG31, 0x00);
	snd_soc_component_write(component, ES8316_DAC_VOLL_REG33, 0x00);
	snd_soc_component_write(component, ES8316_DAC_VOLR_REG34, 0x00);
	snd_soc_component_write(component, ES8316_SDP_ADCFMT_REG0A, 0x0C);
	snd_soc_component_write(component, ES8316_SDP_DACFMT_REG0B, 0x0C);
	snd_soc_component_write(component, ES8316_SYS_VMIDLOW_REG10, 0x10);

	snd_soc_component_write(component, ES8316_SYS_LP1_REG0E, 0x00);
	snd_soc_component_write(component, ES8316_SYS_LP2_REG0F, 0x00);
	snd_soc_component_write(component, ES8316_DAC_PDN_REG2F, 0x00);
	snd_soc_component_write(component, ES8316_HPMIX_SEL_REG13, 0x00);
	snd_soc_component_write(component, ES8316_HPMIX_SWITCH_REG14, 0x88);
	snd_soc_component_write(component, ES8316_HPMIX_PDN_REG15, 0x00);
	snd_soc_component_write(component, ES8316_HPMIX_VOL_REG16, 0xBB);
	snd_soc_component_write(component, ES8316_CPHP_PDN2_REG1A, 0x10);
	snd_soc_component_write(component, ES8316_CPHP_LDOCTL_REG1B, 0x30);
	snd_soc_component_write(component, ES8316_CPHP_PDN1_REG19, 0x02);
	snd_soc_component_write(component, ES8316_CPHP_ICAL_VOL_REG18, 0x11);
	snd_soc_component_write(component, ES8316_GPIO_SEL_REG4D, 0x00);
	snd_soc_component_write(component, ES8316_GPIO_DEBUNCE_INT_REG4E, 0x02);
	snd_soc_component_write(component, ES8316_TESTMODE_REG50, 0xA0);
	snd_soc_component_write(component, ES8316_TEST1_REG51, 0x00);
	snd_soc_component_write(component, ES8316_TEST2_REG52, 0x00);

	snd_soc_component_write(component, ES8316_CAL_HPLIV_REG1E, 0x90);
	snd_soc_component_write(component, ES8316_CAL_HPRIV_REG1F, 0x90);
	snd_soc_component_write(component, ES8316_CAL_TYPE_REG1C, 0x0F);

	snd_soc_component_write(component, ES8316_SYS_PDN_REG0D, 0x00);
	snd_soc_component_write(component, ES8316_RESET_REG00, 0xC0);
	msleep(50);
	snd_soc_component_write(component, ES8316_ADC_PGAGAIN_REG23, 0x60);
	snd_soc_component_write(component, ES8316_ADC_D2SEPGA_REG24, 0x01);
	/* adc ds mode, HPF enable */
	snd_soc_component_write(component, ES8316_ADC_DMIC_REG25, 0x08);
	snd_soc_component_write(component, ES8316_ADC_ALC1_REG29, 0xcd);
	snd_soc_component_write(component, ES8316_ADC_ALC2_REG2A, 0x08);
	snd_soc_component_write(component, ES8316_ADC_ALC3_REG2B, 0xa0);
	snd_soc_component_write(component, ES8316_ADC_ALC4_REG2C, 0x05);
	snd_soc_component_write(component, ES8316_ADC_ALC5_REG2D, 0x06);
	snd_soc_component_write(component, ES8316_ADC_ALC6_REG2E, 0xab);

	snd_soc_component_write(component, ES8316_SYS_VMIDSEL, 0xff);

	snd_soc_component_update_bits(component, ES8316_SERDATA_DAC,
			    ES8316_SERDATA2_LEN_MASK, ES8316_SERDATA2_LEN_16);
	snd_soc_component_update_bits(component, ES8316_SERDATA_ADC,
			    ES8316_SERDATA2_LEN_MASK, ES8316_SERDATA2_LEN_16);

	snd_soc_component_write(component, 0x01, 0x7f);
	snd_soc_component_write(component, 0x02, 0x09);
	snd_soc_component_update_bits(component, ES8316_ADC_PDN_LINSEL, 0xcf, 0x00);
	snd_soc_component_write(component, 0x0a, 0x00);
	snd_soc_component_write(component, 0x0d, 0x00);

	return 0;
}

static int es8316_suspend(struct snd_soc_component *component)
{
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);

	snd_soc_component_write(component, ES8316_CPHP_OUTEN_REG17, 0x00);
	snd_soc_component_write(component, ES8316_DAC_PDN_REG2F, 0x11);
	snd_soc_component_write(component, ES8316_CPHP_LDOCTL_REG1B, 0x03);
	snd_soc_component_write(component, ES8316_CPHP_PDN2_REG1A, 0x22);
	snd_soc_component_write(component, ES8316_CPHP_PDN1_REG19, 0x06);
	snd_soc_component_write(component, ES8316_HPMIX_SWITCH_REG14, 0x00);
	snd_soc_component_write(component, ES8316_HPMIX_PDN_REG15, 0x33);
	snd_soc_component_write(component, ES8316_HPMIX_VOL_REG16, 0x00);
	snd_soc_component_write(component, ES8316_ADC_PDN_LINSEL_REG22, 0xC0);
	if (!es8316->hp_inserted)
		snd_soc_component_write(component, ES8316_SYS_PDN_REG0D, 0x3F);
	snd_soc_component_write(component, ES8316_SYS_LP1_REG0E, 0x3F);
	snd_soc_component_write(component, ES8316_SYS_LP2_REG0F, 0x1F);
	snd_soc_component_write(component, ES8316_RESET_REG00, 0x00);
	es8316_init_reg = 0;

	return 0;
}

static int es8316_resume(struct snd_soc_component *component)
{
	int ret;

	es8316_reset(component); /* UPDATED BY DAVID,15-3-5 */
	ret = snd_soc_component_read(component, ES8316_CLKMGR_ADCDIV2_REG05);
	if (!ret)
		es8316_init_regs(component);

	return 0;
}

/*
 * Call from rk_headset_irq_hook_adc.c
 *
 * Enable micbias for HOOK detection and disable external Amplifier
 * when jack insertion.
 */
int es8316_headset_detect(int jack_insert)
{
	struct es8316_priv *es8316;

	if (!es8316_component)
		return -1;
	es8316 = snd_soc_component_get_drvdata(es8316_component);

	es8316->hp_inserted = jack_insert;

	/*enable micbias and disable PA*/
	if (jack_insert) {
		snd_soc_component_update_bits(es8316_component,
				    ES8316_SYS_PDN_REG0D, 0x3f, 0);
		es8316_enable_spk(es8316, false);
	}

	return 0;
}
EXPORT_SYMBOL(es8316_headset_detect);

static struct snd_soc_jack es8316_jack;

/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin es8316_headset_pins[] = {
	{
		.pin = "Headset",
		.mask = SND_JACK_HEADSET,
	},
};

static int es8316_probe(struct snd_soc_component *component)
{
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	es8316_component = component;

#if MCLK
	es8316->mclk = devm_clk_get(component->dev, "mclk");
	if (PTR_ERR(es8316->mclk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	ret = clk_prepare_enable(es8316->mclk);
	if (ret)
		return ret;
#endif
	ret = snd_soc_component_read(component, ES8316_CLKMGR_ADCDIV2_REG05);
	if (!ret) {
		es8316_reset(component); /* UPDATED BY DAVID,15-3-5 */
		ret = snd_soc_component_read(component, ES8316_CLKMGR_ADCDIV2_REG05);
		if (!ret) {
			es8316_init_regs(component);
			snd_soc_component_write(component, ES8316_CPHP_OUTEN_REG17, 0x00);
			snd_soc_component_write(component, ES8316_DAC_PDN_REG2F, 0x11);
			snd_soc_component_write(component, ES8316_CPHP_LDOCTL_REG1B, 0x03);
			snd_soc_component_write(component, ES8316_CPHP_PDN2_REG1A, 0x22);
			snd_soc_component_write(component, ES8316_CPHP_PDN1_REG19, 0x06);
			snd_soc_component_write(component, ES8316_HPMIX_SWITCH_REG14, 0x00);
			snd_soc_component_write(component, ES8316_HPMIX_PDN_REG15, 0x33);
			snd_soc_component_write(component, ES8316_HPMIX_VOL_REG16, 0x00);
			snd_soc_component_write(component, ES8316_ADC_PDN_LINSEL_REG22, 0xC0);
			if (!es8316->hp_inserted)
				snd_soc_component_write(component, ES8316_SYS_PDN_REG0D, 0x3F);
			snd_soc_component_write(component, ES8316_SYS_LP1_REG0E, 0x3F);
			snd_soc_component_write(component, ES8316_SYS_LP2_REG0F, 0x1F);
		}
	}
	INIT_DELAYED_WORK(&es8316->pcm_pop_work,
				pcm_pop_work_events);

	ret = snd_soc_card_jack_new_pins(component->card, "Headset",
				    SND_JACK_HEADSET | SND_JACK_BTN_0,
				    &es8316_jack,
				    es8316_headset_pins,
				    ARRAY_SIZE(es8316_headset_pins));
	if (ret)
		pr_err("%s: new a jack failed\n", __func__);

	snd_soc_component_set_jack(component, &es8316_jack, NULL);

	return ret;
}

static void es8316_remove(struct snd_soc_component *component)
{
}

static const struct regmap_range es8316_volatile_ranges[] = {
	regmap_reg_range(ES8316_GPIO_FLAG, ES8316_GPIO_FLAG),
};

static const struct regmap_access_table es8316_volatile_table = {
	.yes_ranges	= es8316_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(es8316_volatile_ranges),
};

const struct regmap_config es8316_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= ES8316_TEST3_REG53,
	.cache_type	= REGCACHE_RBTREE,
	.volatile_table	= &es8316_volatile_table,
	.reg_defaults = es8316_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es8316_reg_defaults),
};

#ifdef ENABLE_NETLINK
static void es8316_device_connect_status_notify(enum es8316_connection_status status)
{
	const char *ptr = es8316_status_string[status];
	int len = strlen(ptr) + 1;
	struct sk_buff *nl_skb;
	struct nlmsghdr *nlh;
	int ret;

	if (!nlsk) {
		pr_err("%s: nlsk is NULL\n", __func__);
		return;
	}

	nl_skb = nlmsg_new(len, GFP_ATOMIC);
	if (!nl_skb) {
		pr_err("%s: netlink alloc failure\n", __func__);
		return;
	}

	nlh = nlmsg_put(nl_skb, 0, 0, NETLINK_TEST_ES8316, len, 0);
	if (!nlh) {
		pr_err("%s: nlmsg_put failaure\n", __func__);
		nlmsg_free(nl_skb);
		return;
	}

	memcpy(nlmsg_data(nlh), ptr, len);
	ret = netlink_unicast(nlsk, nl_skb, 200, MSG_DONTWAIT);
}
#endif

static void es8316_send_uevent(struct device *dev, int connect)
{
	char name[32], headset[32];
	char *envp[5];

	scnprintf(name, 32, "NAME=%s", "es8316");
	scnprintf(headset, 32, "HEADSET=%d", connect ? 1 : 0);
	envp[0] = name;
	envp[1] = headset;
	envp[2] = NULL;
	envp[3] = NULL;
	envp[4] = NULL;

	dev_info(dev, "name[%s] headset[%s]\n", name, headset);
	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
}

static irqreturn_t es8316_irq(int irq, void *data)
{
	struct es8316_priv *es8316 = data;
	struct snd_soc_component *comp = es8316_component;
	unsigned int flags;

	dev_err(comp->dev, "Enter into %s\n", __func__);

	mutex_lock(&es8316->lock);
	msleep(500);
	regmap_read(es8316->regmap, ES8316_GPIO_FLAG, &flags);

	if (flags == 0x00)
		goto out; /* Powered-down / reset */

	/* Catch spurious IRQ before set_jack is called */
	if (!es8316->jack)
		goto out;

	if (flags & 0x04) {
		pr_err("%s: Detected 3.5mm headphone connection\n", __func__);
		detect = 1;
#ifdef ENABLE_NETLINK
		es8316_device_connect_status_notify(ES8316_CONNECTION);
#endif
		es8316_send_uevent(comp->dev, ES8316_CONNECTION);
		snd_soc_jack_report(es8316->jack,
					SND_JACK_HEADSET,
					SND_JACK_HEADSET);
	} else {
		pr_err("%s: Detected 3.5mm headphone disconnection\n", __func__);
		detect = 0;
#ifdef ENABLE_NETLINK
		es8316_device_connect_status_notify(ES8316_DISCONNECT);
#endif
		es8316_send_uevent(comp->dev, ES8316_DISCONNECT);
		snd_soc_jack_report(es8316->jack, 0,
					    SND_JACK_HEADSET);
	}

out:
	mdelay(20);

	mutex_unlock(&es8316->lock);
	return IRQ_HANDLED;
}

static void es8316_enable_micbias_for_mic_gnd_short_detect(
	struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	snd_soc_dapm_mutex_lock(dapm);
	snd_soc_dapm_force_enable_pin_unlocked(dapm, "Analog power");
	snd_soc_dapm_sync_unlocked(dapm);
	snd_soc_dapm_mutex_unlock(dapm);
	msleep(20);
}

static void es8316_disable_micbias_for_mic_gnd_short_detect(
	struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	snd_soc_dapm_mutex_lock(dapm);
	snd_soc_dapm_disable_pin_unlocked(dapm, "Analog power");
	snd_soc_dapm_sync_unlocked(dapm);
	snd_soc_dapm_mutex_unlock(dapm);
}

static void es8316_enable_jack_detect(struct snd_soc_component *component,
				      struct snd_soc_jack *jack)
{
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);
	static int initial;

	if (!initial) {
		es8316->jd = false;
		mutex_lock(&es8316->lock);

		es8316->jack = jack;
		if (es8316->jack->status & SND_JACK_MICROPHONE)
			es8316_enable_micbias_for_mic_gnd_short_detect(component);

		snd_soc_component_update_bits(component, ES8316_GPIO_DEBOUNCE,
				      ES8316_GPIO_ENABLE_INTERRUPT,
				      ES8316_GPIO_ENABLE_INTERRUPT);

		mutex_unlock(&es8316->lock);
		/* Enable irq and sync initial jack state */
		enable_irq(es8316->irq);
		es8316_irq(es8316->irq, es8316);
		initial = 1;
	}
}

static void es8316_disable_jack_detect(struct snd_soc_component *component)
{
	struct es8316_priv *es8316 = snd_soc_component_get_drvdata(component);

	if (!es8316->jack)
		return; /* Already disabled (or never enabled) */

	disable_irq(es8316->irq);

	mutex_lock(&es8316->lock);

	snd_soc_component_update_bits(component, ES8316_GPIO_DEBOUNCE,
				      ES8316_GPIO_ENABLE_INTERRUPT, 0);

	if (es8316->jack->status & SND_JACK_MICROPHONE) {
		es8316_disable_micbias_for_mic_gnd_short_detect(component);
		snd_soc_jack_report(es8316->jack, 0, SND_JACK_BTN_0);
	}

	es8316->jack = NULL;

	mutex_unlock(&es8316->lock);
}

static int es8316_set_jack(struct snd_soc_component *component,
			   struct snd_soc_jack *jack, void *data)
{
	if (jack)
		es8316_enable_jack_detect(component, jack);
	else
		es8316_disable_jack_detect(component);
	return 0;
}

static const struct snd_soc_component_driver soc_component_dev_es8316 = {
	.probe =	es8316_probe,
	.remove =	es8316_remove,
	.suspend =	es8316_suspend,
	.resume =	es8316_resume,
	.set_bias_level = es8316_set_bias_level,
	.set_jack		= es8316_set_jack,

	.controls = es8316_snd_controls,
	.num_controls = ARRAY_SIZE(es8316_snd_controls),
	.dapm_widgets = es8316_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(es8316_dapm_widgets),
	.dapm_routes = es8316_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(es8316_dapm_routes),
};

static ssize_t detect_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%d\n", detect);
}

static DEVICE_ATTR_RO(detect_info);

static struct attribute *es8316_sysfs_attrs[] = {
	&dev_attr_detect_info.attr,
	NULL,
};

static struct attribute_group es8316_sysfs_attr_grp = {
	.attrs = es8316_sysfs_attrs,
};

static int es8316_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct es8316_priv *es8316;
	int ret = -1;
	struct device_node *np = i2c->dev.of_node;

	es8316 = devm_kzalloc(&i2c->dev, sizeof(*es8316), GFP_KERNEL);
	if (!es8316)
		return -ENOMEM;

	es8316->debounce_time = 200;
	es8316->hp_det_invert = 0;
	es8316->pwr_count = 0;
	es8316->hp_inserted = false;
	es8316->muted = true;

	es8316->regmap = devm_regmap_init_i2c(i2c, &es8316_regmap_config);
	if (IS_ERR(es8316->regmap)) {
		ret = PTR_ERR(es8316->regmap);
		dev_err(&i2c->dev, "Failed to init regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, es8316);
	es8316->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (es8316->irq_gpio < 0) {
		dev_err(dev, "no irq-gpio provided, will not power on device");
	} else {
		dev_info(dev, "irq-gpio provided ok");
		ret = devm_gpio_request_one(dev, es8316->irq_gpio,
									GPIOF_IN, "irq-gpio");
		es8316->irq = gpio_to_irq(es8316->irq_gpio);
	}

	mutex_init(&es8316->lock);

	ret = devm_request_threaded_irq(dev, es8316->irq, NULL, es8316_irq,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"es8316", es8316);
	if (ret == 0) {
		/* Gets re-enabled by es8316_set_jack() */
		disable_irq(es8316->irq);
		dev_warn(dev, "success to get IRQ %d: %d\n", es8316->irq, ret);
	} else {
		dev_warn(dev, "Failed to get IRQ %d: %d\n", es8316->irq, ret);
		es8316->irq = -ENXIO;
	}

	es8316->power_en_gpio = of_get_named_gpio(np, "power_en-gpio", 0);
	if (es8316->power_en_gpio < 0) {
		pr_err("no power_en-gpio provided, will not power on device");
	} else {
		pr_err("power_en-gpio provided ok");
		ret = devm_gpio_request_one(&i2c->dev, es8316->power_en_gpio,
									GPIOF_OUT_INIT_HIGH, "es8316_power_en");
		if (ret) {
			pr_err("power_en-gpio request failed");
		} else {
			pr_err("power_en-gpio request successfull");
			if (gpio_is_valid(es8316->power_en_gpio))
				gpio_set_value(es8316->power_en_gpio, 1);
		}
	}

	ret = sysfs_create_group(&dev->kobj, &es8316_sysfs_attr_grp);
	if (ret)
		pr_err("%s: sysfs group creation failed %d\n", __func__, ret);

#ifdef ENABLE_NETLINK
	nlsk = netlink_kernel_create(&init_net, NETLINK_TEST_ES8316, NULL);
	if (!nlsk)
		pr_err("%s: netlink_kernel_create error !\n", __func__);
#endif

	ret = snd_soc_register_component(&i2c->dev,
				     &soc_component_dev_es8316,
				     &es8316_dai, 1);
	return ret;
}

static void es8316_i2c_remove(struct i2c_client *client)
{
	kfree(i2c_get_clientdata(client));
}

static void es8316_i2c_shutdown(struct i2c_client *client)
{
	struct es8316_priv *es8316 = i2c_get_clientdata(client);

	if (es8316_component != NULL)
		es8316_enable_spk(es8316, false);
}

static int es8316_pm_resume(struct device *dev)
{
	struct es8316_priv *es8316;

	if (!dev)
		return -ENODEV;

	es8316 = dev_get_drvdata(dev);

#ifdef ENABLE_NETLINK
	const char *ptr = "Restart Adsp";
	int len = strlen(ptr) + 1;
	struct sk_buff *nl_skb;
	struct nlmsghdr *nlh;
	int ret;

	if (!nlsk) {
		pr_err("%s: nlsk is NULL\n", __func__);
		return -EINVAL;
	}

	nl_skb = nlmsg_new(len, GFP_ATOMIC);
	if (!nl_skb) {
		pr_err("%s: netlink alloc failure\n", __func__);
		return -EINVAL;
	}

	nlh = nlmsg_put(nl_skb, 0, 0, NETLINK_TEST_ES8316, len, 0);
	if (!nlh) {
		pr_err("%s: nlmsg_put failaure\n", __func__);
		nlmsg_free(nl_skb);
		return -EINVAL;
	}

	memcpy(nlmsg_data(nlh), ptr, len);
	ret = netlink_unicast(nlsk, nl_skb, 200, MSG_DONTWAIT);
	pr_info("%s: send msg[%s] ret[%d]\n", __func__, ptr, ret);
#endif

	return 0;
}

static int es8316_pm_suspend(struct device *dev)
{
	struct es8316_priv *es8316;

	if (!dev)
		return -ENODEV;

	es8316 = dev_get_drvdata(dev);

	return 0;
}

static const struct dev_pm_ops es8316_pm_ops = {
	.suspend = es8316_pm_suspend,
	.resume = es8316_pm_resume,
};

static const struct i2c_device_id es8316_i2c_id[] = {
	{"es8316", 0},
	{"10ES8316:00", 0},
	{"10ES8316", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, es8316_i2c_id);

static const struct of_device_id es8316_of_match[] = {
	{ .compatible = "everest,es8316", },
	{},
};
MODULE_DEVICE_TABLE(of, es8316_of_match);

static struct i2c_driver es8316_i2c_driver = {
	.driver = {
		.name		= "es8316",
		.of_match_table = es8316_of_match,
		.pm = &es8316_pm_ops,
	},
	.probe    = es8316_i2c_probe,
	.remove	= es8316_i2c_remove,
	.shutdown = es8316_i2c_shutdown,
	.id_table = es8316_i2c_id,
};


module_i2c_driver(es8316_i2c_driver);
MODULE_DESCRIPTION("ASoC es8316 driver");
MODULE_AUTHOR("Will <will@everset-semi.com>");
MODULE_LICENSE("GPL");
