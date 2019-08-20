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
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#include "ipq40xx-adss.h"

struct stereo_priv_data {
	void __iomem *stereo_base;
	spinlock_t stereo_lock;
};

struct stereo_priv_data stereo_priv[MAX_STEREO_ENTRIES];

/* Stereo buffers and I2S state reset */
void ipq40xx_stereo_config_reset(uint32_t reset, uint32_t stereo_id)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&stereo_priv[stereo_id].stereo_lock, flags);
	cfg = readl(stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	cfg &= ~(STEREOn_CONFIG_RESET);
	if (reset)
		cfg |= STEREOn_CONFIG_RESET;
	writel(cfg, stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	spin_unlock_irqrestore(&stereo_priv[stereo_id].stereo_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_stereo_config_reset);

/* MIC buffers reset */
void ipq40xx_stereo_config_mic_reset(uint32_t reset, uint32_t stereo_id)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&stereo_priv[stereo_id].stereo_lock, flags);
	cfg = readl(stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	cfg &= ~(STEREOn_CONFIG_MIC_RESET);
	if (reset)
		cfg |= STEREOn_CONFIG_MIC_RESET;
	writel(cfg, stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	spin_unlock_irqrestore(&stereo_priv[stereo_id].stereo_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_stereo_config_mic_reset);

/* Enable the I2S Stereo block for operation */
void ipq40xx_stereo_config_enable(uint32_t enable, uint32_t stereo_id)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&stereo_priv[stereo_id].stereo_lock, flags);
	cfg = readl(stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	cfg &= ~(STEREOn_CONFIG_ENABLE);
	if (enable)
		cfg |= STEREOn_CONFIG_ENABLE;
	writel(cfg, stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	spin_unlock_irqrestore(&stereo_priv[stereo_id].stereo_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_stereo_config_enable);

/* Enable the SPDIF Stereo block for operation */
void ipq40xx_stereo_spdif_enable(uint32_t enable, uint32_t stereo_id)
{
	uint32_t cfg;

	cfg = readl(stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	cfg &= ~(STEREOn_CONFIG_SPDIF_ENABLE);
	if (enable)
		cfg |= STEREOn_CONFIG_SPDIF_ENABLE;
	writel(cfg, stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
}
EXPORT_SYMBOL(ipq40xx_stereo_spdif_enable);

/* Enable/disable the swap within PCM sample */
void ipq40xx_stereo_spdif_pcmswap(uint32_t enable, uint32_t stereo_id)
{
	uint32_t cfg;

	cfg = readl(stereo_priv[stereo_id].stereo_base
		+ ADSS_STEREOn_STEREO0_CONFIG_REG);

	cfg &= ~(STEREOn_CONFIG_PCM_SWAP);
	if (enable)
		cfg |= STEREOn_CONFIG_PCM_SWAP;

	writel(cfg, stereo_priv[stereo_id].stereo_base
		+ ADSS_STEREOn_STEREO0_CONFIG_REG);
}
EXPORT_SYMBOL(ipq40xx_stereo_spdif_pcmswap);

/* Configure
 * Data word size : Word size loaded into the PCM
 *			register from the MBOX FIFO.
 * I2S word size : Word size sent to the external I2S DAC.
 *			When set to 32 bit words the PCM data
 *			will be left justified in the I2S word.
 */
int ipq40xx_cfg_bit_width(uint32_t bit_width, uint32_t stereo_id)
{
	uint32_t cfg, mask = 0;
	unsigned long flags;

	spin_lock_irqsave(&stereo_priv[stereo_id].stereo_lock, flags);
	switch(bit_width) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		mask |= (STEREOn_CONFIG_DATA_WORD_SIZE(1) |
			STEREOn_CONFIG_I2S_WORD_SIZE_16 |
			STEREOn_CONFIG_MIC_WORD_SIZE_16);
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
		mask |= (STEREOn_CONFIG_DATA_WORD_SIZE(2) |
			STEREOn_CONFIG_I2S_WORD_SIZE_32 |
			STEREOn_CONFIG_MIC_WORD_SIZE_16);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_S32_BE:
		mask |= (STEREOn_CONFIG_DATA_WORD_SIZE(3) |
			STEREOn_CONFIG_I2S_WORD_SIZE_32 |
			STEREOn_CONFIG_MIC_WORD_SIZE_32);
		break;
	default:
		spin_unlock_irqrestore(&stereo_priv[stereo_id].stereo_lock,
					flags);
		return -ENOTSUPP;
	}

	cfg = readl(stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	cfg &= ~(STEREOn_CONFIG_DATA_WORD_SIZE_MASK);
	cfg &= ~(STEREOn_CONFIG_I2S_WORD_SIZE_32);
	cfg &= ~(STEREOn_CONFIG_MIC_WORD_SIZE_32);
	cfg |= mask;
	writel(cfg, stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	spin_unlock_irqrestore(&stereo_priv[stereo_id].stereo_lock, flags);

	return 0;
}
EXPORT_SYMBOL(ipq40xx_cfg_bit_width);

/* Configure stereo/mono mode */
void ipq40xx_config_stereo_mode(uint32_t mode, uint32_t stereo_id)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&stereo_priv[stereo_id].stereo_lock, flags);
	cfg = readl(stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	cfg &= ~(STEREOn_CONFIG_STEREO_MONO_MASK);
	if (mode == CH_STEREO)
		cfg |= STEREOn_CONFIG_STEREO_MODE;
	writel(cfg, stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	spin_unlock_irqrestore(&stereo_priv[stereo_id].stereo_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_config_stereo_mode);

/* Configure master mode */
void ipq40xx_config_master(uint32_t enable, uint32_t stereo_id)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&stereo_priv[stereo_id].stereo_lock, flags);
	cfg = readl(stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	cfg &= ~(STEREOn_CONFIG_MASTER);
	if (enable)
		cfg |= STEREOn_CONFIG_MASTER;
	writel(cfg, stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	spin_unlock_irqrestore(&stereo_priv[stereo_id].stereo_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_config_master);

/* Selects the raw clock source between
 * divided audio clock and input master clock
 * Val 0: Raw master clock is divided audio PLL clock
 * Val 1: Raw master clock is MCLK IN
 */
void ipq40xx_config_mclk_sel(uint32_t stereo_id, uint32_t val)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&stereo_priv[stereo_id].stereo_lock, flags);
	cfg = readl(stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	cfg &= ~(STEREOn_CONFIG_MCK_SEL);
	cfg |= val;
	writel(cfg, stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	spin_unlock_irqrestore(&stereo_priv[stereo_id].stereo_lock, flags);

}
EXPORT_SYMBOL(ipq40xx_config_mclk_sel);

/* Strategy to clear the sample counter TX and RX registers */
void ipq40xx_config_sample_cnt_clear_type(uint32_t stereo_id)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&stereo_priv[stereo_id].stereo_lock, flags);
	cfg = readl(stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	/* 0 - write an explicit zero data through software
	 *	to the TX and RX sample counter registers
	 * 1 - software read of the TX and RX sample counter
	 *	registers clears the counter registers
	 */
	cfg |= STEREOn_CONFIG_SAMPLE_CNT_CLEAR_TYPE; /* Write 1 */
	writel(cfg, stereo_priv[stereo_id].stereo_base
			+ ADSS_STEREOn_STEREO0_CONFIG_REG);
	spin_unlock_irqrestore(&stereo_priv[stereo_id].stereo_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_config_sample_cnt_clear_type);

static const struct of_device_id ipq40xx_audio_stereo_id_table[] = {
	{ .compatible = "qca,ipq40xx-stereo" },
	{},
};
MODULE_DEVICE_TABLE(of, ipq40xx_audio_stereo_id_table);

static int ipq40xx_audio_stereo_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device_node *np = NULL;
	uint32_t stereo_port_id = 0;

	np = of_node_get(pdev->dev.of_node);
	if (!(of_property_read_u32(np, "stereo-index", &stereo_port_id))) {
		if (stereo_port_id >= MAX_STEREO_ENTRIES) {
			of_node_put(pdev->dev.of_node);
			return -EFAULT;
		}

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		stereo_priv[stereo_port_id].stereo_base =
			devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(stereo_priv[stereo_port_id].stereo_base)) {
			of_node_put(pdev->dev.of_node);
			return PTR_ERR(stereo_priv[stereo_port_id].stereo_base);
		}
	} else {
		pr_err("%s: error reading critical device"
			"node properties\n", np->name);
		of_node_put(pdev->dev.of_node);
		return -EFAULT;
	}

	spin_lock_init(&stereo_priv[stereo_port_id].stereo_lock);

	of_node_put(pdev->dev.of_node);
	return 0;
}

static int ipq40xx_audio_stereo_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ipq40xx_audio_stereo_driver = {
	.probe = ipq40xx_audio_stereo_probe,
	.remove = ipq40xx_audio_stereo_remove,
	.driver = {
		.name = "ipq40xx-stereo",
		.owner = THIS_MODULE,
		.of_match_table = ipq40xx_audio_stereo_id_table,
	},
};

module_platform_driver(ipq40xx_audio_stereo_driver);

MODULE_ALIAS("platform:ipq40xx-stereo");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("IPQ40xx AUDIO Stereo driver");
