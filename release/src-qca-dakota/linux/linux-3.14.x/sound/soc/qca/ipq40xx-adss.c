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
#include <linux/reset.h>
#include <linux/spinlock.h>

#include "ipq40xx-adss.h"

void __iomem *adss_audio_local_base;
void __iomem *adss_audio_spdifin_base;
struct reset_control *audio_blk_rst;
static spinlock_t i2s_ctrl_lock;
static spinlock_t tdm_ctrl_lock;
static spinlock_t glb_mode_lock;

/* API to write ADSS registers */
void ipq40xx_audio_adss_writel(uint32_t val, uint32_t offset)
{
	if (!adss_audio_local_base) {
		pr_err("adss_audio_local_base not mapped\n");
		return;
	}
	writel(val, adss_audio_local_base + offset);
}
EXPORT_SYMBOL(ipq40xx_audio_adss_writel);

/* API to read ADSS regitsers */
uint32_t ipq40xx_audio_adss_readl(uint32_t offset)
{
	if (adss_audio_local_base)
		return readl(adss_audio_local_base + offset);
	pr_err("adss_audio_local_base not mapped\n");
	return 0;
}
EXPORT_SYMBOL(ipq40xx_audio_adss_readl);

/* I2S Interface Enable */
void ipq40xx_glb_i2s_interface_en(int enable)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&i2s_ctrl_lock, flags);
	cfg = readl(adss_audio_local_base + ADSS_GLB_CHIP_CTRL_I2S_REG);
	cfg &= ~(GLB_CHIP_CTRL_I2S_INTERFACE_EN);
	if (enable)
		cfg |= GLB_CHIP_CTRL_I2S_INTERFACE_EN;
	writel(cfg, adss_audio_local_base + ADSS_GLB_CHIP_CTRL_I2S_REG);
	spin_unlock_irqrestore(&i2s_ctrl_lock, flags);
	mdelay(5);
}
EXPORT_SYMBOL(ipq40xx_glb_i2s_interface_en);

/* Enable Stereo0/Stereo1/Stereo2 channel */
void ipq40xx_glb_stereo_ch_en(int enable, int stereo_ch)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&i2s_ctrl_lock, flags);
	cfg = readl(adss_audio_local_base + ADSS_GLB_CHIP_CTRL_I2S_REG);
	if (stereo_ch == STEREO0) {
		cfg &= ~(GLB_CHIP_CTRL_I2S_STEREO0_GLB_EN);
		cfg |= GLB_CHIP_CTRL_I2S_STEREO0_GLB_EN;
	} else if (stereo_ch == STEREO1) {
		cfg &= ~(GLB_CHIP_CTRL_I2S_STEREO1_GLB_EN);
		cfg |= GLB_CHIP_CTRL_I2S_STEREO1_GLB_EN;
	} else if (stereo_ch == STEREO2) {
		cfg &= ~(GLB_CHIP_CTRL_I2S_STEREO2_GLB_EN);
		cfg |= GLB_CHIP_CTRL_I2S_STEREO2_GLB_EN;
	}
	writel(cfg, adss_audio_local_base + ADSS_GLB_CHIP_CTRL_I2S_REG);
	spin_unlock_irqrestore(&i2s_ctrl_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_glb_stereo_ch_en);

/* I2S Module Reset */
void ipq40xx_glb_i2s_reset(uint32_t reset)
{
	writel(GLB_I2S_RESET_VAL, adss_audio_local_base + ADSS_GLB_I2S_RST_REG);
	mdelay(5);
	writel(0x0, adss_audio_local_base + ADSS_GLB_I2S_RST_REG);
}
EXPORT_SYMBOL(ipq40xx_glb_i2s_reset);

/* Enable I2S/TDM and Playback/Capture Audio Mode */
void ipq40xx_glb_audio_mode(int mode, int dir)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&glb_mode_lock, flags);
	cfg = readl(adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	if (mode == I2S && dir == PLAYBACK) {
		cfg &= ~(1);
		cfg |= GLB_AUDIO_MODE_XMIT_I2S;
	} else if (mode == I2S && dir == CAPTURE) {
		cfg &= ~(4);
		cfg |= GLB_AUDIO_MODE_RECV_I2S;
	} else if (mode == TDM && dir == PLAYBACK) {
		cfg &= ~(1);
		cfg |= GLB_AUDIO_MODE_XMIT_TDM;
	} else if (mode == TDM && dir == CAPTURE) {
		cfg &= ~(4);
		cfg |= GLB_AUDIO_MODE_RECV_TDM;
	}
	writel(cfg, adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	spin_unlock_irqrestore(&glb_mode_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_glb_audio_mode);

/* I2S0 TX Data Port Enable */
/* Todo : Check if bits 6:4 configures only
	  I2S0 or other channels as well */
void ipq40xx_glb_tx_data_port_en(uint32_t enable)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&glb_mode_lock, flags);
	cfg = readl(adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	cfg &= ~(GLB_AUDIO_MODE_I2S0_TXD_OE);
	if (enable)
		cfg |= GLB_AUDIO_MODE_I2S0_TXD_OE;
	writel(cfg, adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	spin_unlock_irqrestore(&glb_mode_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_glb_tx_data_port_en);

/* I2S3 RX Data Port Enable */
void ipq40xx_glb_rx_data_port_en(uint32_t enable)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&glb_mode_lock, flags);
	cfg = readl(adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	cfg &= ~(GLB_AUDIO_MODE_I2S3_RXD_OE);
	if (enable)
		cfg |= GLB_AUDIO_MODE_I2S3_RXD_OE;
	writel(cfg, adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	spin_unlock_irqrestore(&glb_mode_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_glb_rx_data_port_en);

/* Cross 1K Boundary */
void ipq40xx_glb_audio_mode_B1K(void)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&glb_mode_lock, flags);
	cfg =  readl(adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	cfg &= ~(GLB_AUDIO_MODE_B1K);
	cfg |= GLB_AUDIO_MODE_B1K;
	writel(cfg, adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	spin_unlock_irqrestore(&glb_mode_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_glb_audio_mode_B1K);

/* Frame Sync Port Enable for I2S0 TX */
void ipq40xx_glb_tx_framesync_port_en(uint32_t enable)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&glb_mode_lock, flags);
	cfg =  readl(adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	cfg &= ~(GLB_AUDIO_MODE_I2S0_FS_OE);
	if (enable)
		cfg |= GLB_AUDIO_MODE_I2S0_FS_OE;
	writel(cfg, adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	spin_unlock_irqrestore(&glb_mode_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_glb_tx_framesync_port_en);

/* Frame Sync Port Enable for I2S3 RX */
void ipq40xx_glb_rx_framesync_port_en(uint32_t enable)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&glb_mode_lock, flags);
	cfg =  readl(adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	cfg &= ~(GLB_AUDIO_MODE_I2S3_FS_OE);
	if (enable)
		cfg |= GLB_AUDIO_MODE_I2S3_FS_OE;
	writel(cfg, adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	spin_unlock_irqrestore(&glb_mode_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_glb_rx_framesync_port_en);

void ipq40xx_glb_clk_enable_oe(uint32_t dir)
{
	uint32_t cfg;

	cfg = readl(adss_audio_local_base + ADSS_GLB_CLK_I2S_CTRL_REG);

	if (dir == PLAYBACK) {
		cfg |= (GLB_CLK_I2S_CTRL_TX_BCLK_OE |
			GLB_CLK_I2S_CTRL_TX_MCLK_OE);
	} else {
		cfg |= (GLB_CLK_I2S_CTRL_RX_BCLK_OE |
			GLB_CLK_I2S_CTRL_RX_MCLK_OE);
	}
	writel(cfg, adss_audio_local_base + ADSS_GLB_CLK_I2S_CTRL_REG);
}
EXPORT_SYMBOL(ipq40xx_glb_clk_enable_oe);

/* Channel Number Per Frame for Transmitter/Receiver
 * Real value = val + 1
 */
void ipq40xx_glb_tdm_ctrl_ch_num(uint32_t val, uint32_t dir)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&tdm_ctrl_lock, flags);
	cfg = readl(adss_audio_local_base + ADSS_GLB_TDM_CTRL_REG);

	if (dir == PLAYBACK) {
		cfg &= ~(GLB_TDM_CTRL_TX_CHAN_NUM_MASK);
		cfg |= GLB_TDM_CTRL_TX_CHAN_NUM(val);
	} else if (dir == CAPTURE) {
		cfg &= ~(GLB_TDM_CTRL_RX_CHAN_NUM_MASK);
		cfg |= GLB_TDM_CTRL_RX_CHAN_NUM(val);
	}
	writel(cfg, adss_audio_local_base + ADSS_GLB_TDM_CTRL_REG);
	spin_unlock_irqrestore(&tdm_ctrl_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_glb_tdm_ctrl_ch_num);

/* FSYNC Hi Duration for Transmitter/Receiver */
void ipq40xx_glb_tdm_ctrl_sync_num(uint32_t val, uint32_t dir)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&tdm_ctrl_lock, flags);
	cfg = readl(adss_audio_local_base + ADSS_GLB_TDM_CTRL_REG);

	if (dir == PLAYBACK) {
		cfg &= ~(GLB_TDM_CTRL_TX_SYNC_NUM_MASK);
		cfg |= GLB_TDM_CTRL_TX_SYNC_NUM(val);
	} else if (dir == CAPTURE) {
		cfg &= ~(GLB_TDM_CTRL_RX_SYNC_NUM_MASK);
		cfg |= GLB_TDM_CTRL_RX_SYNC_NUM(val);
	}
	writel(cfg, adss_audio_local_base + ADSS_GLB_TDM_CTRL_REG);
	spin_unlock_irqrestore(&tdm_ctrl_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_glb_tdm_ctrl_sync_num);

/* Serial Data Delay for transmitter/receiver */
void ipq40xx_glb_tdm_ctrl_delay(uint32_t delay, uint32_t dir)
{
	uint32_t cfg;
	unsigned long flags;

	spin_lock_irqsave(&tdm_ctrl_lock, flags);
	cfg = readl(adss_audio_local_base + ADSS_GLB_TDM_CTRL_REG);

	if (dir == PLAYBACK) {
		cfg &= ~(GLB_TDM_CTRL_TX_DELAY);
		if (delay)
			cfg |= GLB_TDM_CTRL_TX_DELAY;
	} else if (dir == CAPTURE) {
		cfg &= ~(GLB_TDM_CTRL_RX_DELAY);
		if (delay)
			cfg |= GLB_TDM_CTRL_RX_DELAY;
	}
	writel(cfg, adss_audio_local_base + ADSS_GLB_TDM_CTRL_REG);
	spin_unlock_irqrestore(&tdm_ctrl_lock, flags);
}
EXPORT_SYMBOL(ipq40xx_glb_tdm_ctrl_delay);

/* PCM RAW clock configuration */
void ipq40xx_pcm_clk_cfg(void)
{
	uint32_t reg_val;

	/* set ADSS_AUDIO_PLL_CONFIG1_REG as required */
	reg_val = readl(adss_audio_local_base + ADSS_AUDIO_PLL_CONFIG1_REG);
	reg_val |= AUDIO_PLL_CONFIG1_SRESET_L(1);
	writel(reg_val, adss_audio_local_base + ADSS_AUDIO_PLL_CONFIG1_REG);

	/* set ADSS_AUDIO_PLL_CONFIG_REG as required */
	reg_val = readl(adss_audio_local_base + ADSS_AUDIO_PLL_CONFIG_REG);
	reg_val &= ~AUDIO_PLL_CONFIG_REFDIV_MASK;
	reg_val |= AUDIO_PLL_CONFIG_REFDIV(5);
	writel(reg_val, adss_audio_local_base + ADSS_AUDIO_PLL_CONFIG_REG);

	/* set ADSS_AUDIO_PLL_CONFIG_REG as required */
	reg_val = readl(adss_audio_local_base + ADSS_AUDIO_PLL_CONFIG_REG);
	reg_val &= ~AUDIO_PLL_CONFIG_POSTPLLDIV_MASK;
	reg_val |= AUDIO_PLL_CONFIG_POSTPLLDIV(1);
	writel(reg_val, adss_audio_local_base + ADSS_AUDIO_PLL_CONFIG_REG);

	reg_val = readl(adss_audio_local_base + ADSS_AUDIO_PLL_CONFIG_REG);
	reg_val &= ~AUDIO_PLL_CONFIG_PLLPWD;
	writel(reg_val, adss_audio_local_base + ADSS_AUDIO_PLL_CONFIG_REG);

	/*set ADSS_AUDIO_PLL_MODULATION_REG as required */
	reg_val = readl(adss_audio_local_base + ADSS_AUDIO_PLL_MODULATION_REG);
	reg_val &= ~AUDIO_PLL_MODULATION_TGT_DIV_MASK;
	reg_val |= AUDIO_PLL_MODULATION_TGT_DIV_FRAC(0x9BA5);
	reg_val |= AUDIO_PLL_MODULATION_TGT_DIV_INT(49);
	writel(reg_val, adss_audio_local_base + ADSS_AUDIO_PLL_MODULATION_REG);

	/* set ADSS_AUDIO_PCM_CFG_RCGR_REG as required */
	reg_val = readl(adss_audio_local_base + ADSS_AUDIO_PCM_CFG_RCGR_REG);
	reg_val |= AUDIO_PCM_CFG_RCGR_SRC_SEL(1)
			| AUDIO_PCM_CFG_RGCR_SRC_DIV(3);
	writel(reg_val, adss_audio_local_base + ADSS_AUDIO_PCM_CFG_RCGR_REG);

	/* set ADSS_AUDIO_PCM_MISC_REG  as required */
	reg_val = AUDIO_PCM_MISC_AUTO_SCALE_DIV(11);
	writel(reg_val, adss_audio_local_base + ADSS_AUDIO_PCM_MISC_REG);

	/* set ADSS_AUDIO_PCM_CMD_RCGR_REG as required */
	reg_val = 3;
	writel(reg_val, adss_audio_local_base + ADSS_AUDIO_PCM_CMD_RCGR_REG);

	/* write ADSS_PCM_OFFSET_REG */
	reg_val = readl(adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	reg_val |= GLB_AUDIO_MODE_B1K;
	writel(reg_val, adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);

}
EXPORT_SYMBOL(ipq40xx_pcm_clk_cfg);

/* PCM RAW ADSS_GLB_PCM_RST_REG register */
void ipq40xx_glb_pcm_rst(uint32_t enable)
{
	uint32_t reg_val;

	if (enable)
		reg_val = GLB_PCM_RST_CTRL(1);
	else
		reg_val = GLB_PCM_RST_CTRL(0);

	writel(reg_val, adss_audio_local_base + ADSS_GLB_PCM_RST_REG);
}
EXPORT_SYMBOL(ipq40xx_glb_pcm_rst);

void ipq40xx_spdifin_ctrl_spdif_en(uint32_t enable)
{
	uint32_t reg_val;

	reg_val = readl(adss_audio_spdifin_base + ADSS_SPDIFIN_SPDIF_CTRL_REG);

	if (enable)
		reg_val |= SPDIF_CTRL_SPDIF_ENABLE;
	else
		reg_val &= ~SPDIF_CTRL_SPDIF_ENABLE;

	writel(reg_val, adss_audio_spdifin_base + ADSS_SPDIFIN_SPDIF_CTRL_REG);

}
EXPORT_SYMBOL(ipq40xx_spdifin_ctrl_spdif_en);

void ipq40xx_spdifin_cfg(void)
{
	uint32_t reg_val;

	reg_val = readl(adss_audio_spdifin_base + ADSS_SPDIFIN_SPDIF_CTRL_REG);
	reg_val &= ~(SPDIF_CTRL_CHANNEL_MODE
			| SPDIF_CTRL_VALIDITYCHECK
			| SPDIF_CTRL_PARITYCHECK);
	reg_val |= (SPDIF_CTRL_USE_FIFO_IF
			| SPDIF_CTRL_SFR_ENABLE
			| SPDIF_CTRL_FIFO_ENABLE);
	writel(reg_val, adss_audio_spdifin_base + ADSS_SPDIFIN_SPDIF_CTRL_REG);
}
EXPORT_SYMBOL(ipq40xx_spdifin_cfg);

void ipq40xx_glb_spdif_out_en(uint32_t enable)
{
	int32_t cfg;

	cfg = readl(adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
	cfg &= ~(GLB_AUDIO_MODE_SPDIF_OUT_OE);
	if (enable)
		cfg |= GLB_AUDIO_MODE_SPDIF_OUT_OE;
	writel(cfg, adss_audio_local_base + ADSS_GLB_AUDIO_MODE_REG);
}
EXPORT_SYMBOL(ipq40xx_glb_spdif_out_en);

void ipq40xx_audio_adss_init(void)
{
	spin_lock_init(&i2s_ctrl_lock);
	spin_lock_init(&tdm_ctrl_lock);
	spin_lock_init(&glb_mode_lock);

	/* I2S in reset */
	ipq40xx_glb_i2s_reset(1);

	/* Enable I2S interface */
	ipq40xx_glb_i2s_interface_en(ENABLE);

	ipq40xx_glb_audio_mode_B1K();
}
EXPORT_SYMBOL(ipq40xx_audio_adss_init);

static const struct of_device_id ipq40xx_audio_adss_id_table[] = {
	{ .compatible = "qca,ipq40xx-audio-adss" },
	{},
};
MODULE_DEVICE_TABLE(of, ipq40xx_audio_adss_id_table);

static int ipq40xx_audio_adss_probe(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	adss_audio_local_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(adss_audio_local_base))
		return PTR_ERR(adss_audio_local_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	adss_audio_spdifin_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(adss_audio_spdifin_base))
		return PTR_ERR(adss_audio_spdifin_base);

	audio_blk_rst = devm_reset_control_get(&pdev->dev, "blk_rst");
	if (IS_ERR(audio_blk_rst))
		return PTR_ERR(audio_blk_rst);

	return 0;
}

static int ipq40xx_audio_adss_remove(struct platform_device *pdev)
{
	ipq40xx_glb_i2s_interface_en(DISABLE);
	return 0;
}

static struct platform_driver ipq40xx_audio_adss_driver = {
	.probe = ipq40xx_audio_adss_probe,
	.remove = ipq40xx_audio_adss_remove,
	.driver = {
		.name = "ipq40xx-adss",
		.owner = THIS_MODULE,
		.of_match_table = ipq40xx_audio_adss_id_table,
	},
};

module_platform_driver(ipq40xx_audio_adss_driver);

MODULE_ALIAS("platform:ipq40xx-adss");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("IPQ40xx AUDIO ADSS driver");
