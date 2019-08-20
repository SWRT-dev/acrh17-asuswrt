/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#include <asm/io.h>
#include <linux/pinctrl/consumer.h>

#include "ipq40xx-adss.h"

static struct snd_soc_dai_link ipq40xx_snd_dai[] = {
	/* Front end DAI Links */
	{
		.name		= "IPQ40xx Media1",
		.stream_name	= "I2S",
		/* Front End DAI Name */
		.cpu_dai_name	= "qca-i2s-dai",
		/* Platform Driver Name */
		.platform_name	= "7709000.qca-pcm-i2s",
		/* Codec DAI Name */
		.codec_dai_name	= "qca-i2s-codec-dai",
		/*Codec Driver Name */
		.codec_name	= "qca_codec.0-0012",
	},
	{
		.name		= "IPQ40xx Media2",
		.stream_name	= "TDM",
		.cpu_dai_name	= "qca-tdm-dai",
		.platform_name	= "7709000.qca-pcm-tdm",
		.codec_dai_name	= "qca-tdm-codec-dai",
		.codec_name	= "qca_codec.0-0012",
	},
	{
		.name		= "IPQ40xx Media3",
		.stream_name	= "I2S1",
		.cpu_dai_name	= "qca-i2s1-dai",
		.platform_name	= "770b000.qca-pcm-i2s1",
		.codec_dai_name	= "qca-i2s1-codec-dai",
		.codec_name	= "qca_codec.0-0012",
	},
	{
		.name		= "IPQ40xx Media4",
		.stream_name	= "I2S2",
		.cpu_dai_name	= "qca-i2s2-dai",
		.platform_name	= "770d000.qca-pcm-i2s2",
		.codec_dai_name	= "qca-i2s2-codec-dai",
		.codec_name	= "qca_codec.0-0012",
	},
	{
		.name           = "IPQ40xx Media5",
		.stream_name    = "SPDIF",
		.cpu_dai_name   = "qca-spdif-dai",
		.platform_name  = "7707000.qca-pcm-spdif",
		.codec_dai_name = "qca-spdif-codec-dai",
		.codec_name	= "qca_codec.0-0012",
	},
};

static struct snd_soc_card snd_soc_card_qca = {
	.name		= "ipq40xx_snd_card",
	.dai_link	= ipq40xx_snd_dai,
	.num_links	= ARRAY_SIZE(ipq40xx_snd_dai),
};

static const struct of_device_id ipq40xx_audio_id_table[] = {
	{ .compatible = "qca,ipq40xx-audio" },
	{},
};
MODULE_DEVICE_TABLE(of, ipq40xx_audio_id_table);

static int ipq40xx_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &snd_soc_card_qca;
	struct dev_pin_info *pins;
	struct pinctrl_state *pin_state;

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

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		pr_err("\nsnd_soc_register_card() failed:%d\n", ret);
		return ret;
	}

	ipq40xx_audio_adss_init();

	pinctrl_select_state(pins->p, pin_state);

	return ret;
}

static int ipq40xx_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	card->dev = NULL;

	return 0;
}

static struct platform_driver ipq40xx_audio_driver = {
	.driver = {
		.name = "ipq40xx_audio",
		.owner = THIS_MODULE,
		.of_match_table = ipq40xx_audio_id_table,
	},
	.probe = ipq40xx_audio_probe,
	.remove = ipq40xx_audio_remove,
};

module_platform_driver(ipq40xx_audio_driver);

MODULE_ALIAS("platform:ipq40xx_audio");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("ALSA SoC IPQ40xx Machine Driver");
