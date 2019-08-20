/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <linux/io.h>
#include <linux/pinctrl/consumer.h>

#include "ipq40xx-adss.h"

#include "../codecs/wcd934x/wcd934x.h"
#include "../codecs/wcd934x/wcd934x-mbhc.h"
#include "../codecs/wsa881x.h"
#include "../codecs/wcd9xxx-common-v2.h"
#include "../codecs/wcd934x/wcd934x.h"

#define CODEC_MCLK_12P288MHZ        12288000
#define DEV_NAME_STR_LEN            32

static struct platform_device *spdev;
static struct snd_soc_aux_dev *msm_aux_dev;
static struct snd_soc_codec_conf *msm_codec_conf;

static int ipq_wsa881x_init(struct snd_soc_dapm_context *dapm);

static int apq_snd_enable_codec_ext_clk(struct snd_soc_codec *codec,
					int enable);

enum pinctrl_pin_state {
	STATE_DISABLE = 0, /* All pins are in sleep state */
	STATE_MI2S_ACTIVE,  /* IS2 = active, TDM = sleep */
	STATE_TDM_ACTIVE,  /* IS2 = sleep, TDM = active */
};

static struct snd_soc_aux_dev ipq4019_aux_dev[] = {
	{
		.name = "wsa881x.0",
		.codec_name = "wsa881x.21170213",
		.init = ipq_wsa881x_init,
	},
	{
		.name = "wsa881x.1",
		.codec_name = "wsa881x.21170214",
		.init = ipq_wsa881x_init,
	},
};

static struct snd_soc_codec_conf ipq4019_codec_conf[] = {
	{
		.dev_name = "wsa881x.21170213",
		.name_prefix = "SpkrLeft",
	},
	{
		.dev_name = "wsa881x.21170214",
		.name_prefix = "SpkrRight",
	},
};

struct msm_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *mi2s_disable;
	struct pinctrl_state *tdm_disable;
	struct pinctrl_state *mi2s_active;
	struct pinctrl_state *tdm_active;
	enum pinctrl_pin_state curr_state;
};

struct msm_asoc_mach_data {
	u32 mclk_freq;
	int us_euro_gpio; /* used by gpio driver API */
	struct device_node *us_euro_gpio_p; /* used by pinctrl API */
	struct device_node *hph_en1_gpio_p; /* used by pinctrl API */
	struct device_node *hph_en0_gpio_p; /* used by pinctrl API */
	struct snd_info_entry *codec_root;
	struct msm_pinctrl_info pinctrl_info;
};

struct msm_wsa881x_dev_info {
	struct device_node *of_node;
	u32 index;
};

struct apq8096_i2c_asoc_mach_data {
	u32 mclk_freq;
	int us_euro_gpio;
	int hph_en1_gpio;
	int hph_en0_gpio;
	struct snd_info_entry *codec_root;
};

static int apq_snd_enable_codec_ext_clk(struct snd_soc_codec *codec,
					int enable)
{
	return tavil_cdc_mclk_enable(codec, enable);
}

static int apq8096_i2c_mclk_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	pr_debug("%s: event = %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return apq_snd_enable_codec_ext_clk(w->codec, 1);
	case SND_SOC_DAPM_POST_PMD:
		return apq_snd_enable_codec_ext_clk(w->codec, 0);
	}
	return 0;
}

static const struct snd_soc_dapm_widget apq8096_i2c_dapm_widgets[] = {

	SND_SOC_DAPM_SUPPLY("MCLK",  SND_SOC_NOPM, 0, 0,
	apq8096_i2c_mclk_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SPK("Lineout_1 amp", NULL),
	SND_SOC_DAPM_SPK("Lineout_3 amp", NULL),
	SND_SOC_DAPM_SPK("Lineout_2 amp", NULL),
	SND_SOC_DAPM_SPK("Lineout_4 amp", NULL),
	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("ANCRight Headset Mic", NULL),
	SND_SOC_DAPM_MIC("ANCLeft Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Analog Mic4", NULL),
	SND_SOC_DAPM_MIC("Analog Mic6", NULL),
	SND_SOC_DAPM_MIC("Analog Mic7", NULL),
	SND_SOC_DAPM_MIC("Analog Mic8", NULL),

	SND_SOC_DAPM_MIC("Digital Mic0", NULL),
	SND_SOC_DAPM_MIC("Digital Mic1", NULL),
	SND_SOC_DAPM_MIC("Digital Mic2", NULL),
	SND_SOC_DAPM_MIC("Digital Mic3", NULL),
	SND_SOC_DAPM_MIC("Digital Mic4", NULL),
	SND_SOC_DAPM_MIC("Digital Mic5", NULL),
	SND_SOC_DAPM_MIC("Digital Mic6", NULL),
};

static struct snd_soc_dapm_route wcd9335_audio_paths[] = {
	{"MIC BIAS1", NULL, "MCLK"},
	{"MIC BIAS2", NULL, "MCLK"},
	{"MIC BIAS3", NULL, "MCLK"},
	{"MIC BIAS4", NULL, "MCLK"},
};

static int apq_mi2s_audrx_init(struct snd_soc_pcm_runtime *rtd)
{
	int err;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_card *card;
	struct snd_info_entry *entry;
	struct apq8096_i2c_asoc_mach_data *pdata =
				snd_soc_card_get_drvdata(rtd->card);

	pr_info("%s: dev_name%s\n", __func__, dev_name(cpu_dai->dev));

	rtd->pmdown_time = 0;

	snd_soc_dapm_new_controls(dapm, apq8096_i2c_dapm_widgets,
				ARRAY_SIZE(apq8096_i2c_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, wcd9335_audio_paths,
				ARRAY_SIZE(wcd9335_audio_paths));
	snd_soc_dapm_enable_pin(dapm, "Lineout_1 amp");
	snd_soc_dapm_enable_pin(dapm, "Lineout_3 amp");
	snd_soc_dapm_enable_pin(dapm, "Lineout_2 amp");
	snd_soc_dapm_enable_pin(dapm, "Lineout_4 amp");

	snd_soc_dapm_ignore_suspend(dapm, "Lineout_1 amp");
	snd_soc_dapm_ignore_suspend(dapm, "Lineout_3 amp");
	snd_soc_dapm_ignore_suspend(dapm, "Lineout_2 amp");
	snd_soc_dapm_ignore_suspend(dapm, "Lineout_4 amp");
	snd_soc_dapm_ignore_suspend(dapm, "ultrasound amp");
	snd_soc_dapm_ignore_suspend(dapm, "Handset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "Headset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "ANCRight Headset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "ANCLeft Headset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic1");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic2");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic3");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic4");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic5");
	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic4");
	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic6");
	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic7");
	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic8");
	snd_soc_dapm_ignore_suspend(dapm, "MADINPUT");
	snd_soc_dapm_ignore_suspend(dapm, "MAD_CPE_INPUT");
	snd_soc_dapm_ignore_suspend(dapm, "EAR");
	snd_soc_dapm_ignore_suspend(dapm, "LINEOUT1");
	snd_soc_dapm_ignore_suspend(dapm, "LINEOUT2");
	snd_soc_dapm_ignore_suspend(dapm, "LINEOUT3");
	snd_soc_dapm_ignore_suspend(dapm, "LINEOUT4");
	snd_soc_dapm_ignore_suspend(dapm, "ANC EAR");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC2");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC3");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC4");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC5");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC6");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC2");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC3");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC4");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC5");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic0");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC0");
	snd_soc_dapm_ignore_suspend(dapm, "SPK1 OUT");
	snd_soc_dapm_ignore_suspend(dapm, "SPK2 OUT");
	snd_soc_dapm_ignore_suspend(dapm, "HPHL");
	snd_soc_dapm_ignore_suspend(dapm, "HPHR");
	snd_soc_dapm_ignore_suspend(dapm, "ANC HPHL");
	snd_soc_dapm_ignore_suspend(dapm, "ANC HPHR");
	snd_soc_dapm_ignore_suspend(dapm, "ANC LINEOUT1");
	snd_soc_dapm_ignore_suspend(dapm, "ANC LINEOUT2");
	snd_soc_dapm_ignore_suspend(dapm, "AIF4 VI");
	snd_soc_dapm_ignore_suspend(dapm, "VIINPUT");

	snd_soc_dapm_sync(dapm);

	card = rtd->card->snd_card;

	entry = snd_info_card_create(card);
	if (!entry) {
		pr_debug("%s: Cannot create codecs module entry\n",
			 __func__);
		err = 0;
		goto out;
	}
	pdata->codec_root = entry;

	tavil_codec_info_create_codec_entry(pdata->codec_root, codec);

	return 0;
out:
	return err;
}

static struct snd_soc_dai_link ipq40xx_snd_dai[] = {
	{
		.name		= "IPQ40xx Media2",
		.stream_name	= "I2S",
		.cpu_dai_name	= "qca-i2s-dai",
		.platform_name	= "7709000.qca-pcm-i2s",
		.codec_dai_name	= "tavil_i2s_rx1",
		.codec_name	= "tavil_codec",
		.init  = &apq_mi2s_audrx_init,
		.ignore_suspend = 1,
	},
};

static struct snd_soc_card snd_soc_card_qca = {
	.name = "ipq40xx-tavil-snd-card",
	.dai_link	= ipq40xx_snd_dai,
	.num_links	= ARRAY_SIZE(ipq40xx_snd_dai),
};

static int apq8096_i2c_populate_dai_link_component_of_node(
					struct snd_soc_card *card)
{
	int i, index, ret = 0;
	struct device *cdev = card->dev;
	struct snd_soc_dai_link *dai_link = card->dai_link;
	struct device_node *np;

	if (!cdev) {
		pr_err("%s: Sound card device memory NULL\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < card->num_links; i++) {
		if (dai_link[i].platform_of_node && dai_link[i].cpu_of_node)
			continue;

		/* populate platform_of_node for snd card dai links */
		if (dai_link[i].platform_name &&
		    !dai_link[i].platform_of_node) {
			index = of_property_match_string(cdev->of_node,
						"asoc-platform-names",
						dai_link[i].platform_name);
			if (index < 0) {
				pr_err("%s: No match found for platform name: %s\n",
					__func__, dai_link[i].platform_name);
				ret = index;
				goto err;
			}
			np = of_parse_phandle(cdev->of_node, "asoc-platform",
					      index);
			if (!np) {
				pr_err("%s: retrieving phandle for platform %s, index %d failed\n",
					__func__, dai_link[i].platform_name,
					index);
				ret = -ENODEV;
				goto err;
			}
			dai_link[i].platform_of_node = np;
			dai_link[i].platform_name = NULL;
		}

		/* populate cpu_of_node for snd card dai links */
		if (dai_link[i].cpu_dai_name && !dai_link[i].cpu_of_node) {
			index = of_property_match_string(cdev->of_node,
						 "asoc-cpu-names",
						 dai_link[i].cpu_dai_name);
			if (index >= 0) {
				np = of_parse_phandle(cdev->of_node, "asoc-cpu",
						index);
				if (!np) {
					pr_err("%s: retrieving phandle for cpu dai %s failed\n",
						__func__,
						dai_link[i].cpu_dai_name);
					ret = -ENODEV;
					goto err;
				}
				dai_link[i].cpu_of_node = np;
				dai_link[i].cpu_dai_name = NULL;
			}
		}

		/* populate codec_of_node for snd card dai links */
		if (dai_link[i].codec_name && !dai_link[i].codec_of_node) {
			index = of_property_match_string(cdev->of_node,
						 "asoc-codec-names",
						 dai_link[i].codec_name);
			if (index < 0)
				continue;
			np = of_parse_phandle(cdev->of_node, "asoc-codec",
					      index);
			if (!np) {
				pr_err("%s: retrieving phandle for codec %s failed\n",
					__func__, dai_link[i].codec_name);
				ret = -ENODEV;
				goto err;
			}
			dai_link[i].codec_of_node = np;
			dai_link[i].codec_name = NULL;
		}
	}

err:
	return ret;
}

static int ipq_wsa881x_init(struct snd_soc_dapm_context *dapm)
{
	u8 spkleft_ports[WSA881X_MAX_SWR_PORTS] = {100, 101, 102, 106};
	u8 spkright_ports[WSA881X_MAX_SWR_PORTS] = {103, 104, 105, 107};
	unsigned int ch_rate[WSA881X_MAX_SWR_PORTS] = {2400, 600, 300, 1200};
	unsigned int ch_mask[WSA881X_MAX_SWR_PORTS] = {0x1, 0xF, 0x3, 0x3};
	struct msm_asoc_mach_data *pdata;
	int ret = 0;
	struct snd_soc_codec *codec = dapm->codec;

	if (!codec) {
		pr_err("%s codec is NULL\n", __func__);
		return -EINVAL;
	}

	/*
	 * Name prefix is not available at this time.  SOC deletes it.
	 * So device name must be used.
	 */
	if (!strcmp(codec->name, "wsa881x.21170213")) {
		dev_dbg(codec->dev, "%s: setting left ch map to codec %s\n",
			__func__, codec->name);
		wsa881x_set_channel_map(codec, &spkleft_ports[0],
				WSA881X_MAX_SWR_PORTS, &ch_mask[0],
				&ch_rate[0]);
		snd_soc_dapm_ignore_suspend(dapm, "SpkrLeft IN");
		snd_soc_dapm_ignore_suspend(dapm, "SpkrLeft SPKR");
	} else if (!strcmp(codec->name, "wsa881x.21170214")) {
		dev_dbg(codec->dev, "%s: setting right ch map to codec %s\n",
			__func__, codec->name);
		wsa881x_set_channel_map(codec, &spkright_ports[0],
				WSA881X_MAX_SWR_PORTS, &ch_mask[0],
				&ch_rate[0]);
		snd_soc_dapm_ignore_suspend(dapm, "SpkrRight IN");
		snd_soc_dapm_ignore_suspend(dapm, "SpkrRight SPKR");
	} else {
		dev_err(codec->dev, "%s: wrong codec name %s\n", __func__,
			codec->name);
		ret = -EINVAL;
		goto err_codec;
	}
	pdata = snd_soc_card_get_drvdata(codec->card);
	if (pdata && pdata->codec_root)
		wsa881x_codec_info_create_codec_entry(pdata->codec_root,
						      codec);

err_codec:
	return ret;
}

static int ipq_init_wsa_dev(struct platform_device *pdev,
			struct snd_soc_card *card)
{
	card->aux_dev = ipq4019_aux_dev;
	card->codec_conf = ipq4019_codec_conf;
	card->num_aux_devs = 2;
	card->num_configs = 2;
	return 0;
}


static const struct of_device_id ipq40xx_wcd_audio_id_table[] = {
	{ .compatible = "qcom,ipq40xx-asoc-snd-tavil", },
	{},
};
MODULE_DEVICE_TABLE(of, ipq40xx_wcd_audio_id_table);

static int ipq40xx_wcd_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &snd_soc_card_qca;
	struct dev_pin_info *pins;
	struct pinctrl_state *pin_state;
	struct apq8096_i2c_asoc_mach_data *pdata;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No platform supplied from device tree\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct apq8096_i2c_asoc_mach_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, pdata);

	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret) {
		dev_err(&pdev->dev, "parse card name failed, err:%d\n", ret);
		goto err;
	}

	ret = snd_soc_of_parse_audio_routing(card, "qcom,audio-routing");
	if (ret) {
		dev_err(&pdev->dev, "parse audio routing failed, err:%d\n",
			ret);
		goto err;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
			"qcom,tavil-mclk-clk-freq", &pdata->mclk_freq);
	if (ret) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed, err%d\n",
			"qcom,tavil-mclk-clk-freq",
			pdev->dev.of_node->full_name, ret);
		goto err;
	}

	spdev = pdev;

	card->dev = &pdev->dev;
	pins = card->dev->pins;

/*
 * If the sound card registration fails, then the audio TLMM change
 * is also reverted. Due to this, the pins are seen to toggle causing
 * pop noise. To avoid this, the pins are set to GPIO state and moved
 * to audio functionality only when the sound card registration is
 * successful.
 */
	pin_state = pinctrl_lookup_state(pins->p, "audio");
	if (IS_ERR(pin_state)) {
		pr_err("audio pinctrl state not available\n");
		return PTR_ERR(pin_state);
	}

	ret = ipq_init_wsa_dev(pdev, card);
	if (ret)
		goto err;

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret == -EPROBE_DEFER) {
		goto err;
	} else if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err;
	}

	ipq40xx_audio_adss_init();

	pinctrl_select_state(pins->p, pin_state);

	dev_info(&pdev->dev, "Sound card %s registered\n", card->name);

	return ret;

err:
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static int ipq40xx_wcd_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	card->dev = NULL;

	return 0;
}

static struct platform_driver ipq40xx_wcd_audio_driver = {
	.driver = {
		.name = "ipq40xx_wcd_audio",
		.owner = THIS_MODULE,
		.of_match_table = ipq40xx_wcd_audio_id_table,
	},
	.probe = ipq40xx_wcd_audio_probe,
	.remove = ipq40xx_wcd_audio_remove,
};

module_platform_driver(ipq40xx_wcd_audio_driver);

MODULE_ALIAS("platform:ipq40xx_wcd_audio");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("ALSA SoC IPQ40xx Machine Driver");
