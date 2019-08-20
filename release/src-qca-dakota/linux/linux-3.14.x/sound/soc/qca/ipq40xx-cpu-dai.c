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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/clk-provider.h>

#include "ipq40xx-mbox.h"
#include "ipq40xx-adss.h"

struct dai_priv_st {
	int stereo_tx;
	int stereo_rx;
	int mbox_tx;
	int mbox_rx;
	int tx_enabled;
	int rx_enabled;
	int is_txmclk_fixed;
	struct platform_device *pdev;
};
struct dai_priv_st dai_priv[MAX_INTF];

struct clk *audio_tx_bclk;
struct clk *audio_tx_mclk;
struct clk *audio_rx_bclk;
struct clk *audio_rx_mclk;
struct clk *audio_spdif_src;
struct clk *audio_spdif_div2;
struct clk *audio_spdifinfast_src;

/* Get Stereo channel ID based on I2S/TDM/SPDIF intf and direction */
uint32_t get_stereo_id(struct snd_pcm_substream *substream, int intf)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return dai_priv[intf].stereo_tx;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return dai_priv[intf].stereo_rx;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(get_stereo_id);

/* Get MBOX channel ID based on I2S/TDM/SPDIF intf and direction */
uint32_t get_mbox_id(struct snd_pcm_substream *substream, int intf)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return dai_priv[intf].mbox_tx;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return dai_priv[intf].mbox_rx;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(get_mbox_id);

uint32_t ipq40xx_get_act_bit_width(uint32_t bit_width)
{
	switch (bit_width) {
	case SNDRV_PCM_FORMAT_S8:
	case SNDRV_PCM_FORMAT_U8:
		return __BIT_8;
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
	case SNDRV_PCM_FORMAT_U16_LE:
	case SNDRV_PCM_FORMAT_U16_BE:
		return __BIT_16;
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
	case SNDRV_PCM_FORMAT_U24_3LE:
	case SNDRV_PCM_FORMAT_U24_3BE:
		return __BIT_32;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_BE:
	case SNDRV_PCM_FORMAT_U24_LE:
	case SNDRV_PCM_FORMAT_U24_BE:
		return __BIT_24;
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_S32_BE:
	case SNDRV_PCM_FORMAT_U32_LE:
	case SNDRV_PCM_FORMAT_U32_BE:
		return __BIT_32;
	default:
		return __BIT_INVAL;
	}
}
EXPORT_SYMBOL(ipq40xx_get_act_bit_width);

static int ipq40xx_audio_clk_get(struct clk **clk, struct device *dev,
					const char *id)
{
	*clk = devm_clk_get(dev, id);
	if (IS_ERR(*clk)) {
		dev_err(dev, "%s: Error in %s\n", __func__, id);
		return PTR_ERR(*clk);
	}

	return 0;
}

static int ipq40xx_audio_clk_set(struct clk *clk, struct device *dev,
					uint32_t val)
{
	int ret;

	ret = clk_set_rate(clk, val);
	if (ret != 0) {
		dev_err(dev, "%s: Error in setting %s\n", __func__,
					__clk_get_name(clk));
		return ret;
	}

	ret = clk_prepare_enable(clk);
	if (ret != 0) {
		dev_err(dev, "%s: Error in enable %s\n", __func__,
					__clk_get_name(clk));
		return ret;
	}

	return 0;
}

static void ipq40xx_audio_clk_disable(struct clk **clk, struct device *dev)
{
	if (*clk) {
		if (__clk_is_enabled(*clk))
			clk_disable_unprepare(*clk);
		devm_clk_put(dev, *clk);
	}
}

static int ipq40xx_audio_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	uint32_t intf = dai->driver->id;
	int ret = 0;
	struct device *dev = &(dai_priv[intf].pdev->dev);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Check if the direction is enabled */
		if (dai_priv[intf].tx_enabled != ENABLE)
			goto error;

		ipq40xx_glb_tx_data_port_en(ENABLE);
		ipq40xx_glb_tx_framesync_port_en(ENABLE);

		ret = ipq40xx_audio_clk_get(&audio_tx_bclk, dev,
						"audio_tx_bclk");

		if (!ret && !(dai_priv[intf].is_txmclk_fixed))
			ret = ipq40xx_audio_clk_get(&audio_tx_mclk, dev,
						"audio_tx_mclk");

	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* Check if the direction is enabled */
		if (dai_priv[intf].rx_enabled != ENABLE)
			goto error;

		ipq40xx_glb_rx_data_port_en(ENABLE);
		ipq40xx_glb_rx_framesync_port_en(ENABLE);

		ret = ipq40xx_audio_clk_get(&audio_rx_bclk, dev,
						"audio_rx_bclk");
		if (!ret)
			ret = ipq40xx_audio_clk_get(&audio_rx_mclk, dev,
						"audio_rx_mclk");
	}
	if (ret)
		return ret;

	if (intf == I2S || intf == I2S1 || intf == I2S2) {
		/* Select I2S mode */
		ipq40xx_glb_audio_mode(I2S, substream->stream);
	} else if (intf == TDM) {
		/* Select TDM mode */
		ipq40xx_glb_audio_mode(TDM, substream->stream);

		/* Set TDM Ctrl register */
		ipq40xx_glb_tdm_ctrl_sync_num(TDM_SYNC_NUM, substream->stream);
		ipq40xx_glb_tdm_ctrl_delay(TDM_DELAY, substream->stream);
	}

	return 0;
error:
	pr_err("%s: Direction not enabled\n", __func__);
	return -EFAULT;
}

static int ipq40xx_audio_prepare(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s:%d\n", __func__, __LINE__);
	return 0;
}

static int ipq40xx_audio_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params,
					struct snd_soc_dai *dai)
{
	uint32_t bit_width, channels, rate;
	uint32_t intf = dai->driver->id;
	uint32_t stereo_id = get_stereo_id(substream, intf);
	uint32_t mbox_id = get_mbox_id(substream, intf);
	uint32_t bit_act;
	int ret;
	uint32_t mclk, bclk;
	struct device *dev = &(dai_priv[intf].pdev->dev);

	bit_width = params_format(params);
	channels = params_channels(params);
	rate = params_rate(params);

	bit_act = ipq40xx_get_act_bit_width(bit_width);

	if (intf == TDM) {
		/* Set TDM number of channels */
		ipq40xx_glb_tdm_ctrl_ch_num((channels-1), substream->stream);
		mclk = bclk = rate * bit_act * channels;
	} else {
		bclk = rate * bit_act * channels;
		mclk = bclk * MCLK_MULTI;
	}

	ipq40xx_glb_clk_enable_oe(substream->stream);

	ipq40xx_config_master(ENABLE, stereo_id);

	ret = ipq40xx_cfg_bit_width(bit_width, stereo_id);
	if (ret) {
		pr_err("%s: BitWidth %d not supported\n",
			__FUNCTION__, bit_width);
		return ret;
	}

	ipq40xx_stereo_config_enable(DISABLE, stereo_id);

	ipq40xx_stereo_config_reset(ENABLE, stereo_id);
	ipq40xx_stereo_config_mic_reset(ENABLE, stereo_id);

	mdelay(5);

	ret = ipq40xx_mbox_fifo_reset(mbox_id);
	if (ret) {
		pr_err("%s: %d: Error in dma fifo reset\n",
					__func__, __LINE__);
		return ret;
	}

	ipq40xx_stereo_config_reset(DISABLE, stereo_id);
	ipq40xx_stereo_config_mic_reset(DISABLE, stereo_id);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (!(dai_priv[intf].is_txmclk_fixed)) {
			ret = ipq40xx_audio_clk_set(audio_tx_mclk, dev, mclk);
			if (ret)
				return ret;
		}

		ret = ipq40xx_audio_clk_set(audio_tx_bclk, dev, bclk);
		if (ret)
			return ret;

	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ret = ipq40xx_audio_clk_set(audio_rx_mclk, dev, mclk);
		if (ret)
			return ret;

		ret = ipq40xx_audio_clk_set(audio_rx_bclk, dev, bclk);
		if (ret)
			return ret;
	}

	return 0;
}

static void ipq40xx_audio_shutdown(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	uint32_t intf = dai->driver->id;
	struct device *dev = &(dai_priv[intf].pdev->dev);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ipq40xx_glb_tx_data_port_en(DISABLE);
		ipq40xx_glb_tx_framesync_port_en(DISABLE);

		/* Disable the clocks */
		ipq40xx_audio_clk_disable(&audio_tx_bclk, dev);
		if (!(dai_priv[intf].is_txmclk_fixed))
			ipq40xx_audio_clk_disable(&audio_tx_mclk, dev);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ipq40xx_glb_rx_data_port_en(DISABLE);
		ipq40xx_glb_rx_framesync_port_en(DISABLE);

		/* Disable the clocks */
		ipq40xx_audio_clk_disable(&audio_rx_bclk, dev);
		ipq40xx_audio_clk_disable(&audio_rx_mclk, dev);
	}
	/* Disable the I2S Stereo block */
}

static int ipq40xx_audio_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	dev_dbg(dai->dev, "%s:%d\n", __func__, __LINE__);
	return 0;
}

static int ipq40xx_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params,
					struct snd_soc_dai *dai)
{
	uint32_t bit_width, channels, rate, bit_act;
	int ret;
	uint32_t stereo_id = get_stereo_id(substream, SPDIF);
	uint32_t mclk, bclk;
	struct device *dev = &(dai_priv[SPDIF].pdev->dev);
	uint32_t spdif_bclk;
	uint32_t spdif_mclk;

	bit_width = params_format(params);
	channels = params_channels(params);
	rate = params_rate(params);
	bit_act = ipq40xx_get_act_bit_width(bit_width);

	bclk = rate * bit_act * channels;
	mclk = bclk * MCLK_MULTI;

	/* SPDIF subframe is always 32 bit and 2 channels */
	spdif_bclk = rate * 32 * 2;
	spdif_mclk = spdif_bclk * 2;

	if (substream->stream == PLAYBACK) {
		/* Set the clocks */
		ret = ipq40xx_audio_clk_set(audio_tx_mclk, dev, mclk);
		if (ret)
			return ret;

		ret = ipq40xx_audio_clk_set(audio_tx_bclk, dev, bclk);
		if (ret)
			return ret;

		ret = ipq40xx_audio_clk_set(audio_spdif_src, dev,
						spdif_mclk);
		if (ret)
			return ret;

		ret = ipq40xx_audio_clk_set(audio_spdif_div2, dev,
						spdif_bclk);
		if (ret)
			return ret;

		ipq40xx_glb_clk_enable_oe(substream->stream);

		/* Set MASTER mode */
		ipq40xx_config_master(ENABLE, stereo_id);

		/* Configure bit width */
		ret = ipq40xx_cfg_bit_width(bit_width, stereo_id);
		if (ret) {
			pr_err("%s: BitWidth %d not supported\n",
				__func__, bit_width);
			return ret;
		}

	} else if (substream->stream == CAPTURE) {
		/* Set the clocks */
		ret = ipq40xx_audio_clk_set(audio_spdifinfast_src, dev,
						AUDIO_SPDIFINFAST);
		if (ret)
			return ret;
	}

	return 0;
}

static int ipq40xx_spdif_prepare(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s:%d\n", __func__, __LINE__);
	return 0;
}

static int ipq40xx_spdif_startup(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	int ret = 0;
	struct device *dev = &(dai_priv[SPDIF].pdev->dev);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Check if the direction is enabled */
		if (dai_priv[SPDIF].tx_enabled != ENABLE)
			goto error;

		ipq40xx_glb_tx_data_port_en(ENABLE);
		ipq40xx_glb_tx_framesync_port_en(ENABLE);
		ipq40xx_glb_spdif_out_en(ENABLE);
		/* Select I2S/TDM */
		ipq40xx_glb_audio_mode(I2S, substream->stream);

		ret = ipq40xx_audio_clk_get(&audio_tx_bclk, dev,
						"audio_tx_bclk");
		if (ret)
			return ret;
		ret = ipq40xx_audio_clk_get(&audio_tx_mclk, dev,
						"audio_tx_mclk");
		if (ret)
			return ret;
		ret = ipq40xx_audio_clk_get(&audio_spdif_src, dev,
						"audio_spdif_src");
		if (ret)
			return ret;
		ret = ipq40xx_audio_clk_get(&audio_spdif_div2, dev,
						"audio_spdif_div2");
		if (ret)
			return ret;

	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* Check if the direction is enabled */
		if (dai_priv[SPDIF].rx_enabled != ENABLE)
			goto error;
		ipq40xx_spdifin_ctrl_spdif_en(DISABLE);

		ipq40xx_glb_rx_data_port_en(ENABLE);
		ipq40xx_glb_rx_framesync_port_en(ENABLE);
		ipq40xx_glb_audio_mode(I2S, substream->stream);
		ipq40xx_spdifin_cfg();

		ret = ipq40xx_audio_clk_get(&audio_spdifinfast_src, dev,
						"audio_spdifinfast_src");
	}

	return ret;
error:
	pr_err("%s: Direction not enabled\n", __func__);
	return -EFAULT;
}

static void ipq40xx_spdif_shutdown(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	struct device *dev = &(dai_priv[SPDIF].pdev->dev);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ipq40xx_glb_tx_data_port_en(DISABLE);
		ipq40xx_glb_tx_framesync_port_en(DISABLE);

		/* Disable the clocks */
		ipq40xx_audio_clk_disable(&audio_tx_bclk, dev);
		ipq40xx_audio_clk_disable(&audio_tx_mclk, dev);
		ipq40xx_audio_clk_disable(&audio_spdif_src, dev);
		ipq40xx_audio_clk_disable(&audio_spdif_div2, dev);

	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ipq40xx_glb_rx_data_port_en(DISABLE);
		ipq40xx_glb_rx_framesync_port_en(DISABLE);

		/* Disable the clocks */
		ipq40xx_audio_clk_disable(&audio_spdifinfast_src, dev);
	}
}

static int ipq40xx_spdif_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	dev_dbg(dai->dev, "%s:%d\n", __func__, __LINE__);
	return 0;
}

static struct snd_soc_dai_ops ipq40xx_audio_ops = {
	.startup	= ipq40xx_audio_startup,
	.prepare	= ipq40xx_audio_prepare,
	.hw_params	= ipq40xx_audio_hw_params,
	.shutdown	= ipq40xx_audio_shutdown,
	.set_fmt	= ipq40xx_audio_set_fmt,
};

static struct snd_soc_dai_ops ipq40xx_spdif_ops = {
	.startup	= ipq40xx_spdif_startup,
	.prepare	= ipq40xx_spdif_prepare,
	.hw_params	= ipq40xx_spdif_hw_params,
	.shutdown	= ipq40xx_spdif_shutdown,
	.set_fmt	= ipq40xx_spdif_set_fmt,
};

static struct snd_soc_dai_driver ipq40xx_cpu_dais[] = {
	{
		.playback = {
			.rates		= RATE_16000_96000,
			.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S32,
			.channels_min	= CH_STEREO,
			.channels_max	= CH_STEREO,
			.rate_min	= FREQ_16000,
			.rate_max	= FREQ_96000,
		},
		.capture = {
			.rates		= RATE_16000_96000,
			.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S32,
			.channels_min	= CH_STEREO,
			.channels_max	= CH_STEREO,
			.rate_min	= FREQ_16000,
			.rate_max	= FREQ_96000,
		},
		.ops = &ipq40xx_audio_ops,
		.id = I2S,
		.name = "qca-i2s-dai"
	},
	{
		.playback = {
			.rates		= RATE_16000_96000,
			.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S32,
			.channels_min	= CH_STEREO,
			.channels_max	= CH_7_1,
			.rate_min	= FREQ_16000,
			.rate_max	= FREQ_96000,
		},
		.capture = {
			.rates		= RATE_16000_96000,
			.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S32,
			.channels_min	= CH_STEREO,
			.channels_max	= CH_7_1,
			.rate_min	= FREQ_16000,
			.rate_max	= FREQ_96000,
		},
		.ops = &ipq40xx_audio_ops,
		.id = TDM,
		.name = "qca-tdm-dai"
	},
	{
		.playback = {
			.rates		= RATE_16000_96000,
			.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S32,
			.channels_min	= 2,
			.channels_max	= 2,
			.rate_min	= FREQ_16000,
			.rate_max	= FREQ_96000,
		},
		.ops = &ipq40xx_audio_ops,
		.id = I2S1,
		.name = "qca-i2s1-dai"
	},
	{
		.playback = {
			.rates		= RATE_16000_96000,
			.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S32,
			.channels_min	= 2,
			.channels_max	= 2,
			.rate_min	= FREQ_16000,
			.rate_max	= FREQ_96000,
		},
		.ops = &ipq40xx_audio_ops,
		.id = I2S2,
		.name = "qca-i2s2-dai"
	},
	{
		.playback = {
			.rates		= RATE_16000_96000,
			.formats        = SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S24_3,
			.channels_min   = CH_STEREO,
			.channels_max   = CH_STEREO,
			.rate_min       = FREQ_16000,
			.rate_max       = FREQ_96000,
		},
		.capture = {
			.rates		= RATE_16000_96000,
			.formats        = SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S24_3,
			.channels_min   = CH_STEREO,
			.channels_max   = CH_STEREO,
			.rate_min       = FREQ_16000,
			.rate_max       = FREQ_96000,
		},
		.ops = &ipq40xx_spdif_ops,
		.name = "qca-spdif-dai"
	},
};

static const struct snd_soc_component_driver ipq40xx_i2s_component = {
	.name           = "qca-cpu-dai",
};

static const struct of_device_id ipq40xx_cpu_dai_id_table[] = {
	{ .compatible = "qca,ipq40xx-i2s", .data = (void *)I2S},
	{ .compatible = "qca,ipq40xx-tdm", .data = (void *)TDM},
	{ .compatible = "qca,ipq40xx-spdif", .data = (void *)SPDIF},
	{ .compatible = "qca,ipq40xx-i2s1", .data = (void *)I2S1},
	{ .compatible = "qca,ipq40xx-i2s2", .data = (void *)I2S2},
	{},
};
MODULE_DEVICE_TABLE(of, ipq40xx_cpu_dai_id_table);

static int ipq40xx_dai_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device_node *np = NULL;
	int ret;
	int intf;

	ret = snd_soc_register_component(&pdev->dev, &ipq40xx_i2s_component,
			 ipq40xx_cpu_dais, ARRAY_SIZE(ipq40xx_cpu_dais));
	if (ret) {
		dev_err(&pdev->dev,
			"%s: error registering soc dais\n", __func__);
		return ret;
	}

	match = of_match_device(ipq40xx_cpu_dai_id_table, &pdev->dev);
	if (!match) {
		ret = -ENODEV;
		goto error;
	}

	intf = (uint32_t)match->data;

	np = of_node_get(pdev->dev.of_node);

	/* TX is enabled only when both DMA and Stereo TX channel
	 * is specified in the DTSi
	 */
	if (!(of_property_read_u32(np, "dma-tx-channel",
					&dai_priv[intf].mbox_tx)
		|| of_property_read_u32(np, "stereo-tx-port",
					&dai_priv[intf].stereo_tx))) {
		dai_priv[intf].tx_enabled = ENABLE;
	}

	/* RX is enabled only when both DMA and Stereo RX channel
	 * is specified in the DTSi, except in case of SPDIF RX
	 */
	if (!(of_property_read_u32(np, "dma-rx-channel",
					&dai_priv[intf].mbox_rx))) {
		if (intf == SPDIF) {
			dai_priv[intf].rx_enabled = ENABLE;
			dai_priv[intf].stereo_rx = MAX_STEREO_ENTRIES;
		} else if (!(of_property_read_u32(np, "stereo-rx-port",
					&dai_priv[intf].stereo_rx))) {
			dai_priv[intf].rx_enabled = ENABLE;
		}
	}

	/* Either TX or Rx should have been enabled for a DMA/Stereo Channel */
	if (!(dai_priv[intf].tx_enabled || dai_priv[intf].rx_enabled)) {
		pr_err("%s: error reading critical device"
				" node properties\n", np->name);
		ret = -EFAULT;
		goto error_node;
	}

	if (of_property_read_u32(np, "ipq,txmclk-fixed",
					&dai_priv[intf].is_txmclk_fixed))
		pr_debug("%s: ipq,txmclk-fixed not enabled\n", __func__);

	dai_priv[intf].pdev = pdev;

	of_node_put(pdev->dev.of_node);

	return 0;
error_node:
	of_node_put(pdev->dev.of_node);
error:
	snd_soc_unregister_component(&pdev->dev);
	return ret;
}

static int ipq40xx_dai_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static struct platform_driver ipq40xx_dai_driver = {
	.probe = ipq40xx_dai_probe,
	.remove = ipq40xx_dai_remove,
	.driver = {
		.name = "qca-cpu-dai",
		.owner = THIS_MODULE,
		.of_match_table = ipq40xx_cpu_dai_id_table,
	},
};

module_platform_driver(ipq40xx_dai_driver);

MODULE_ALIAS("platform:qca-cpu-dai");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("IPQ40xx CPU DAI DRIVER");
