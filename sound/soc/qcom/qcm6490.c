// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
// Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.

#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <linux/soundwire/sdw.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include "lpass.h"
#include "qdsp6/q6afe.h"
#include "qdsp6/q6prm.h"
#include "common.h"
#include "sdw.h"

#define DRIVER_NAME		"qcm6490"
#define WCN_CDC_SLIM_RX_CH_MAX	2
#define WCN_CDC_SLIM_TX_CH_MAX	2
#define NAME_SIZE	32

#define DEFAULT_MCLK_RATE		24576000
#define TDM_BCLK_RATE			6144000
#define MI2S_BCLK_RATE			1536000

struct qcm6490_snd_data {
	struct qcom_snd_common_data common_priv;
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	uint32_t pri_mi2s_clk_count;
	uint32_t pri_mi2s_mclk_count;
	uint32_t quat_mi2s_clk_count;
	uint32_t quat_tdm_clk_count;
	struct sdw_stream_runtime *sruntime[AFE_PORT_MAX];
	struct snd_soc_jack jack;
	bool jack_setup;
	struct snd_soc_jack hdmi_jack[8];
};

static int qcm6490_slim_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	unsigned int rx_ch[WCN_CDC_SLIM_RX_CH_MAX] = {157, 158};
	unsigned int tx_ch[WCN_CDC_SLIM_TX_CH_MAX]  = {159, 162};
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);

	ret = snd_soc_dai_set_channel_map(codec_dai, ARRAY_SIZE(tx_ch),
		tx_ch, ARRAY_SIZE(rx_ch), rx_ch);

	return ret;
}

/*
 * The ES8316 IC requires MCLK to be constantly on. If MCLK switches on
 * and offas playback starts and stops, it can easily cause POP sound
 */
static int qcm6490_mi2s_mclk_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	struct snd_soc_card *card = rtd->card;
	struct qcm6490_snd_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
	case PRIMARY_MI2S_TX:
		if (++(data->pri_mi2s_mclk_count) == 1) {
			snd_soc_dai_set_sysclk(cpu_dai,
				Q6AFE_LPASS_CLK_ID_MCLK_1,
				DEFAULT_MCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		}
		break;
	default:
		dev_err(rtd->dev, "%s: invalid dai id 0x%x\n", __func__, cpu_dai->id);
	}

	return ret;
}

static int qcm6490_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct qcm6490_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret = 0;
	char jack_name[NAME_SIZE];
	struct snd_soc_jack *hdmi_jack  = NULL;
	int hdmi_pcm_id = 0;
	struct snd_soc_dai *codec_dai;
	int rval, i;


	switch (cpu_dai->id) {
	case TX_CODEC_DMA_TX_3:
	case RX_CODEC_DMA_RX_0:
		ret = qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
		break;
	case VA_CODEC_DMA_TX_0:
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_TX_0:
	case PRIMARY_TDM_RX_0:
	case PRIMARY_TDM_TX_0:
	case QUATERNARY_MI2S_RX:
	case QUATERNARY_MI2S_TX:
		break;
	case PRIMARY_MI2S_RX:
	case PRIMARY_MI2S_TX:
		return qcm6490_mi2s_mclk_init(rtd);
	case SLIMBUS_0_RX:
	case SLIMBUS_0_TX:
		ret = qcm6490_slim_dai_init(rtd);
		break;
	case DISPLAY_PORT_RX_0:
		hdmi_pcm_id = 0;
		hdmi_jack = &data->hdmi_jack[0];
		break;
	case DISPLAY_PORT_RX_1:
		hdmi_pcm_id = 1;
		hdmi_jack = &data->hdmi_jack[1];
		break;

	default:
		break;
	}
	if (hdmi_jack) {
		snprintf(jack_name, sizeof(jack_name), "HDMI/DP%d Jack", hdmi_pcm_id);
		rval = snd_soc_card_jack_new(rtd->card, jack_name, SND_JACK_AVOUT, hdmi_jack);

		if (rval)
			return rval;

		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			rval = snd_soc_component_set_jack(codec_dai->component, hdmi_jack, NULL);
			if (rval != 0 && rval != -EOPNOTSUPP) {
				dev_warn(rtd->card->dev, "Failed to set HDMI jack: %d\n", rval);
				return rval;
			}
		}
		return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
	}

	return ret;
}

static int qcm6490_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	channels->min = 2;
	channels->max = 2;
	switch (cpu_dai->id) {
	case TX_CODEC_DMA_TX_0:
	case TX_CODEC_DMA_TX_1:
	case TX_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_3:
		channels->min = 1;
		break;
	default:
		break;
	}

	return 0;
}

static int qcm6490_snd_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct qcm6490_snd_data *pdata = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *codec_dai = NULL;
	struct qcom_snd_dailink_data *link_priv = &pdata->common_priv.link_data[rtd->num];
	unsigned int daifmt = 0;
	u32 mclk_fs, mclk_rate, clk_dir;
	int ret = 0;
	int i;

	daifmt = rtd->dai_link->dai_fmt;
	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		if (daifmt)
			snd_soc_dai_set_fmt(codec_dai, daifmt);

		mclk_fs = link_priv->mclk_fs;
		clk_dir = link_priv->clk_direction;
		if (mclk_fs) {
			mclk_rate = params_rate(params) * mclk_fs;
			ret = snd_soc_dai_set_sysclk(codec_dai, link_priv->mclk_id,
						     mclk_rate, clk_dir);
			if (ret < 0) {
				dev_err(rtd->dev, "snd_soc_dai_set_sysclk err = %d\n",
					ret);
				return ret;
			}
		}
	}

	return qcom_snd_sdw_hw_params(substream, params, &pdata->sruntime[cpu_dai->id]);
}

static int qcm6490_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct qcm6490_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct sdw_stream_runtime *sruntime = data->sruntime[cpu_dai->id];

	return qcom_snd_sdw_prepare(substream, sruntime,
				    &data->stream_prepared[cpu_dai->id]);
}

static int qcm6490_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct qcm6490_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sdw_stream_runtime *sruntime = data->sruntime[cpu_dai->id];

	return qcom_snd_sdw_hw_free(substream, sruntime,
				    &data->stream_prepared[cpu_dai->id]);
}

static int qcm6490_snd_startup(struct snd_pcm_substream *substream)
{
	unsigned int fmt = SND_SOC_DAIFMT_BP_FP;
	unsigned int codec_dai_fmt = SND_SOC_DAIFMT_BC_FC;
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct qcm6490_snd_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	int j;
	int ret;

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
	case PRIMARY_MI2S_TX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF;
		if (++(data->pri_mi2s_clk_count) == 1) {
			snd_soc_dai_set_sysclk(cpu_dai,
				Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
				MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		}
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	case QUATERNARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		if (++(data->quat_mi2s_clk_count) == 1) {
			snd_soc_dai_set_sysclk(cpu_dai,
				Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT,
				MI2S_BCLK_RATE * 2, SNDRV_PCM_STREAM_PLAYBACK);
		}
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;

	default:
		pr_err("%s: invalid dai id 0x%x\n", __func__, cpu_dai->id);
		break;
	}
	return 0;
}

static void  qcm6490_snd_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_card *card = rtd->card;
	struct qcm6490_snd_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
	case PRIMARY_MI2S_TX:
		if (--(data->pri_mi2s_clk_count) == 0) {
			snd_soc_dai_set_sysclk(cpu_dai,
				Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
				0, SNDRV_PCM_STREAM_PLAYBACK);
		}
		break;
	case QUATERNARY_MI2S_RX:
		if (--(data->quat_mi2s_clk_count) == 0) {
			snd_soc_dai_set_sysclk(cpu_dai,
				Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT,
				0, SNDRV_PCM_STREAM_PLAYBACK);
		}
		break;

	default:
		pr_err("%s: invalid dai id 0x%x\n", __func__, cpu_dai->id);
		break;
	}
}

static const struct snd_soc_dapm_widget qcm6490_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_PINCTRL("STUB_AIF0_PINCTRL", "stub_aif0_active", "stub_aif0_sleep"),
	SND_SOC_DAPM_PINCTRL("STUB_AIF1_PINCTRL", "stub_aif1_active", "stub_aif1_sleep"),
	SND_SOC_DAPM_PINCTRL("STUB_AIF2_PINCTRL", "stub_aif2_active", "stub_aif2_sleep"),
	SND_SOC_DAPM_PINCTRL("STUB_AIF3_PINCTRL", "stub_aif3_active", "stub_aif3_sleep"),
};

static const struct snd_soc_dapm_route qcm6490_dapm_routes[] = {
	{"STUB_AIF0_RX", NULL, "STUB_AIF0_PINCTRL"},
	{"STUB_AIF0_TX", NULL, "STUB_AIF0_PINCTRL"},
	{"STUB_AIF1_RX", NULL, "STUB_AIF1_PINCTRL"},
	{"STUB_AIF1_TX", NULL, "STUB_AIF1_PINCTRL"},
	{"STUB_AIF2_RX", NULL, "STUB_AIF2_PINCTRL"},
	{"STUB_AIF2_TX", NULL, "STUB_AIF2_PINCTRL"},
	{"STUB_AIF3_RX", NULL, "STUB_AIF3_PINCTRL"},
	{"STUB_AIF3_TX", NULL, "STUB_AIF3_PINCTRL"},
};

static const struct snd_soc_dapm_widget qcs6490_rb3gen2_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_PINCTRL("STUB_AIF1_PINCTRL", "stub_aif1_active", "stub_aif1_sleep"),
};

static const struct snd_soc_dapm_route qcs6490_rb3gen2_dapm_routes[] = {
	{"STUB_AIF1_RX", NULL, "STUB_AIF1_PINCTRL"},
	{"STUB_AIF1_TX", NULL, "STUB_AIF1_PINCTRL"},
};

static const struct snd_soc_dapm_widget qcs6490_rb3gen2_ia_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_soc_dapm_widget qcs6490_rb3gen2_ptz_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_PINCTRL("STUB_AIF1_PINCTRL", "stub_aif1_active", "stub_aif1_sleep"),
};

static const struct snd_soc_dapm_route qcs6490_rb3gen2_ptz_dapm_routes[] = {
	{"STUB_AIF1_RX", NULL, "STUB_AIF1_PINCTRL"},
	{"STUB_AIF1_TX", NULL, "STUB_AIF1_PINCTRL"},
};

static const struct snd_soc_dapm_widget qcs6490_rb3gen2_video_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_PINCTRL("STUB_AIF1_PINCTRL", "stub_aif1_active", "stub_aif1_sleep"),
};

static const struct snd_soc_dapm_route qcs6490_rb3gen2_video_dapm_routes[] = {
	{"STUB_AIF1_RX", NULL, "STUB_AIF1_PINCTRL"},
	{"STUB_AIF1_TX", NULL, "STUB_AIF1_PINCTRL"},
};

static const struct snd_soc_dapm_widget qcs6490_rb3gen2_vision_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_PINCTRL("STUB_AIF1_PINCTRL", "stub_aif1_active", "stub_aif1_sleep"),
};

static const struct snd_soc_dapm_route qcs6490_rb3gen2_vision_dapm_routes[] = {
	{"STUB_AIF1_RX", NULL, "STUB_AIF1_PINCTRL"},
	{"STUB_AIF1_TX", NULL, "STUB_AIF1_PINCTRL"},
};

static const struct snd_soc_ops qcm6490_be_ops = {
	.hw_params = qcm6490_snd_hw_params,
	.hw_free = qcm6490_snd_hw_free,
	.prepare = qcm6490_snd_prepare,
	.startup = qcm6490_snd_startup,
	.shutdown = qcm6490_snd_shutdown,
};

static struct snd_soc_card qcm6490_data = {
	.name = "qcm6490",
	.dapm_widgets = qcm6490_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(qcm6490_dapm_widgets),
	.dapm_routes = qcm6490_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(qcm6490_dapm_routes),
};

static struct snd_soc_card qcs6490_rb3gen2_data = {
	.name = "qcs6490-rb3gen2",
	.dapm_widgets = qcs6490_rb3gen2_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(qcs6490_rb3gen2_dapm_widgets),
	.dapm_routes = qcs6490_rb3gen2_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(qcs6490_rb3gen2_dapm_routes),
};

static struct snd_soc_card qcs6490_rb3gen2_ia_data = {
	.name = "qcs6490-rb3gen2-ia-mezz",
	.dapm_widgets = qcs6490_rb3gen2_ia_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(qcs6490_rb3gen2_ia_dapm_widgets),
};

static struct snd_soc_card qcs6490_rb3gen2_ptz_data = {
	.name = "qcs6490-rb3gen2-ptz-mezz",
	.dapm_widgets = qcs6490_rb3gen2_ptz_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(qcs6490_rb3gen2_ptz_dapm_widgets),
	.dapm_routes = qcs6490_rb3gen2_ptz_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(qcs6490_rb3gen2_ptz_dapm_routes),
};

static struct snd_soc_card qcs6490_rb3gen2_video_data = {
	.name = "qcs6490-rb3gen2-video-mezz",
	.dapm_widgets = qcs6490_rb3gen2_video_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(qcs6490_rb3gen2_video_dapm_widgets),
	.dapm_routes = qcs6490_rb3gen2_video_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(qcs6490_rb3gen2_video_dapm_routes),
};

static struct snd_soc_card qcs6490_rb3gen2_vision_data = {
	.name = "qcs6490-rb3gen2-vision-mezz",
	.dapm_widgets = qcs6490_rb3gen2_vision_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(qcs6490_rb3gen2_vision_dapm_widgets),
	.dapm_routes = qcs6490_rb3gen2_vision_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(qcs6490_rb3gen2_vision_dapm_routes),
};

static void qcm6490_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if ((link->num_codecs != 1) || (link->codecs->dai_name
					&& strcmp(link->codecs->dai_name, "snd-soc-dummy-dai"))) {
			link->init = qcm6490_snd_init;
			link->be_hw_params_fixup = qcm6490_be_hw_params_fixup;
			link->ops = &qcm6490_be_ops;
		}
	}
}

static int qcm6490_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct qcm6490_snd_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	card = (struct snd_soc_card *)of_device_get_match_data(&pdev->dev);
	if (!card)
		return -EINVAL;

	card->owner = THIS_MODULE;
	/* Allocate the private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->dev = dev;

	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);
	ret = qcom_snd_parse_of_v2(card, &data->common_priv);
	if (ret)
		return ret;

	card->driver_name = DRIVER_NAME;
	qcm6490_add_be_ops(card);

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id snd_qcm6490_dt_match[] = {
	{.compatible = "qcom,qcm6490-sndcard", .data = &qcm6490_data},
	{.compatible = "qcom,qcs6490-rb3gen2-sndcard", .data = &qcs6490_rb3gen2_data},
	{.compatible = "qcom,qcs6490-rb3gen2-ia-sndcard", .data = &qcs6490_rb3gen2_ia_data},
	{.compatible = "qcom,qcs6490-rb3gen2-ptz-sndcard", .data = &qcs6490_rb3gen2_ptz_data},
	{.compatible = "qcom,qcs6490-rb3gen2-video-sndcard", .data = &qcs6490_rb3gen2_video_data},
	{.compatible = "qcom,qcs6490-rb3gen2-vision-sndcard", .data = &qcs6490_rb3gen2_vision_data},
	{}
};

MODULE_DEVICE_TABLE(of, snd_qcm6490_dt_match);

static struct platform_driver snd_qcm6490_driver = {
	.probe  = qcm6490_platform_probe,
	.driver = {
		.name = "snd-qcm6490",
		.of_match_table = snd_qcm6490_dt_match,
	},
};
module_platform_driver(snd_qcm6490_driver);
MODULE_DESCRIPTION("qcm6490 ASoC Machine Driver");
MODULE_LICENSE("GPL");
