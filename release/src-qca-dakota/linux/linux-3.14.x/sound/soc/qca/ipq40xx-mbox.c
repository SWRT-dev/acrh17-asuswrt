/*
 * Copyright (c) 2015 The Linux Foundation. All rights reserved.
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
 */

#include <linux/dma-mapping.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/dmapool.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <asm/bitops.h>

#include "ipq40xx-mbox.h"

enum {
	CHN_DISABLED = 0x00,
	CHN_ENABLED = 0x01, /* from dtsi */
	CHN_STARTED = 0x02, /* dma inited */
	CHN_STATUS_DISABLE = 0xFF,
};

struct ipq40xx_mbox_rt_priv *mbox_rtime[ADSS_MBOX_NR_CHANNELS];

static struct ipq40xx_mbox_desc *get_next(
				struct ipq40xx_mbox_rt_dir_priv *rtdir,
				struct ipq40xx_mbox_desc *desc)
{
	struct ipq40xx_mbox_desc *end;

	end = rtdir->dma_virt_head + rtdir->ndescs;

	desc++;

	if (desc >= end)
		desc = rtdir->dma_virt_head;

	return desc;
}

void ipq40xx_mbox_desc_own(u32 channel_id, int desc_no, int own)
{
	struct ipq40xx_mbox_desc *desc;
	struct ipq40xx_mbox_rt_dir_priv *rtdir;
	u32 chan;
	u32 dir;

	chan = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	rtdir = &mbox_rtime[chan]->dir_priv[dir];

	desc = rtdir->dma_virt_head;
	desc += desc_no;

	rtdir->write = desc_no;

	desc->OWN = own;
	desc->ei = 1;
}
EXPORT_SYMBOL(ipq40xx_mbox_desc_own);

uint32_t ipq40xx_mbox_get_played_offset(u32 channel_id)
{
	struct ipq40xx_mbox_desc *desc, *write;
	struct ipq40xx_mbox_rt_dir_priv *rtdir;
	unsigned int i, size_played = 0;
	u32 chan, dir;

	chan = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	rtdir = &mbox_rtime[chan]->dir_priv[dir];

	desc = rtdir->dma_virt_head;
	write = &rtdir->dma_virt_head[rtdir->write];
	desc += rtdir->read;

	for (i = 0; i < rtdir->ndescs; i++) {
		if (desc->OWN == 0) {
			size_played = desc->size;
			rtdir->read = (rtdir->read + 1) % rtdir->ndescs;
		} else {
			break;
		}

		if (desc == write)
			break;

		desc = get_next(rtdir, desc);
	}

	return size_played * rtdir->read;
}

uint32_t ipq40xx_mbox_get_played_offset_set_own(u32 channel_id)
{
	struct ipq40xx_mbox_desc *desc, *last_played, *prev;
	struct ipq40xx_mbox_rt_dir_priv *rtdir;
	unsigned int i, desc_own, size_played = 0;
	u32 chan, dir;

	chan = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	rtdir = &mbox_rtime[chan]->dir_priv[dir];
	last_played = NULL;

	/* Point to the last desc */
	prev = &rtdir->dma_virt_head[rtdir->ndescs - 1];
	desc_own = prev->OWN;

	/* point to first desc */
	desc = &rtdir->dma_virt_head[0];

	for (i = 0; i < rtdir->ndescs; i++) {
		if (prev->OWN == 0) {
			if (i == (rtdir->ndescs - 1)) {
				if (desc_own == 1)
					last_played = desc;
			} else if (desc->OWN == 1) {
				last_played = desc;
			}
			prev->OWN = 1;
			prev->ei = 1;
		}
		prev = desc;
		desc += 1;
	}
	if (last_played) {
		desc = &rtdir->dma_virt_head[0];
		size_played = last_played->BufPtr - desc->BufPtr;
	} else {
		pr_debug("%s last played buf not found\n", __func__);
		rtdir->last_played_is_null++;
	}

	return size_played;
}

int ipq40xx_mbox_fifo_reset(int channel_id)
{
	volatile void __iomem *mbox_reg;
	uint32_t index, dir;

	index = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	if (!mbox_rtime[index])
		return -ENOMEM;

	mbox_reg = mbox_rtime[index]->mbox_reg_base;

	switch (dir) {
	case PLAYBACK:
		writel(MBOX_FIFO_RESET_TX_INIT,
			 mbox_reg + ADSS_MBOXn_MBOX_FIFO_RESET_REG);
		break;
	case CAPTURE:
		writel(MBOX_FIFO_RESET_RX_INIT,
			 mbox_reg + ADSS_MBOXn_MBOX_FIFO_RESET_REG);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(ipq40xx_mbox_fifo_reset);

int ipq40xx_mbox_dma_start(int channel_id)
{
	volatile void __iomem *mbox_reg;
	uint32_t index, dir;

	index = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	if (!mbox_rtime[index])
		return -ENOMEM;

	mbox_reg = mbox_rtime[index]->mbox_reg_base;
	mbox_rtime[index]->mbox_started = 1;

	switch (dir) {
	case PLAYBACK:
		writel(ADSS_MBOXn_DMA_RX_CONTROL_START,
			mbox_reg + ADSS_MBOXn_MBOXn_DMA_RX_CONTROL_REG);
		break;

	case CAPTURE:
		writel(ADSS_MBOXn_DMA_TX_CONTROL_START,
			mbox_reg + ADSS_MBOXn_MBOXn_DMA_TX_CONTROL_REG);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(ipq40xx_mbox_dma_start);

int ipq40xx_mbox_dma_resume(int channel_id)
{
	volatile void __iomem *mbox_reg;
	uint32_t index, dir;

	index = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	if (!mbox_rtime[index])
		return -ENOMEM;

	/* resume is meaningful only when dma is started. */
	if (!mbox_rtime[index]->mbox_started)
		return 0;

	mbox_reg = mbox_rtime[index]->mbox_reg_base;

	switch (dir) {
	case PLAYBACK:
		writel(ADSS_MBOXn_DMA_RX_CONTROL_RESUME,
			mbox_reg + ADSS_MBOXn_MBOXn_DMA_RX_CONTROL_REG);
		break;

	case CAPTURE:
		writel(ADSS_MBOXn_DMA_TX_CONTROL_RESUME,
			mbox_reg + ADSS_MBOXn_MBOXn_DMA_TX_CONTROL_REG);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(ipq40xx_mbox_dma_resume);

int ipq40xx_mbox_dma_stop(int channel_id)
{
	volatile void __iomem *mbox_reg;
	struct ipq40xx_mbox_rt_dir_priv *mbox_cb;
	uint32_t index, dir;

	index = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	if (!mbox_rtime[index])
		return -ENOMEM;

	mbox_reg = mbox_rtime[index]->mbox_reg_base;
	mbox_rtime[index]->mbox_started = 0;

	switch (dir) {
	case PLAYBACK:
		writel(ADSS_MBOXn_DMA_RX_CONTROL_STOP,
			mbox_reg + ADSS_MBOXn_MBOXn_DMA_RX_CONTROL_REG);
		break;

	case CAPTURE:
		writel(ADSS_MBOXn_DMA_TX_CONTROL_STOP,
			mbox_reg + ADSS_MBOXn_MBOXn_DMA_TX_CONTROL_REG);
		break;
	}

	/* Delay for the dynamically calculated max time based on
	sample size, channel, sample rate + margin to ensure that the
	DMA engine will be truly idle. */

	mdelay(10);

	mbox_cb = &mbox_rtime[index]->dir_priv[dir];
	mbox_cb->read = 0;
	mbox_cb->write = 0;

	return 0;

}
EXPORT_SYMBOL(ipq40xx_mbox_dma_stop);

static inline bool ipq40xx_is_chn_already_inited(uint32_t index, uint32_t dir)
{
	if (dir == PLAYBACK)
		return (test_bit(CHN_STARTED,
			&mbox_rtime[index]->dir_priv[CAPTURE].status));
	else
		return (test_bit(CHN_STARTED,
			&mbox_rtime[index]->dir_priv[PLAYBACK].status));
}

int ipq40xx_mbox_dma_reset_swap(int channel_id)
{
	unsigned int val;
	volatile void __iomem *mbox_reg;
	uint32_t index;

	index = ipq40xx_convert_id_to_channel(channel_id);

	if (!mbox_rtime[index])
		return -EINVAL;

	mbox_reg = mbox_rtime[index]->mbox_reg_base;

	val = readl(mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);
	val &= ~(MBOX_DMA_POLICY_RXD_END_SWAP | MBOX_DMA_POLICY_RXD_16BIT_SWAP);

	writel(val, mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);

	return 0;
}
EXPORT_SYMBOL(ipq40xx_mbox_dma_reset_swap);

int ipq40xx_mbox_dma_swap(int channel_id, snd_pcm_format_t format)
{
	unsigned int val;
	volatile void __iomem *mbox_reg;
	uint32_t index;

	index = ipq40xx_convert_id_to_channel(channel_id);

	if (!mbox_rtime[index])
		return -ENOMEM;

	mbox_reg = mbox_rtime[index]->mbox_reg_base;

	val = readl(mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);
	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		val |= MBOX_DMA_POLICY_RXD_16BIT_SWAP;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
		val |= MBOX_DMA_POLICY_RXD_END_SWAP;
		break;
	default:
		/* Nothing to do */
		break;
	}
	writel(val, mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);

	return 0;
}
EXPORT_SYMBOL(ipq40xx_mbox_dma_swap);

int ipq40xx_mbox_dma_prepare(int channel_id)
{
	struct ipq40xx_mbox_desc *desc;
	unsigned int val;
	int err;
	volatile void __iomem *mbox_reg;
	dma_addr_t phys_addr;
	uint32_t index, dir;

	index = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	if (!mbox_rtime[index])
		return -ENOMEM;

	mbox_reg = mbox_rtime[index]->mbox_reg_base;

	/* do not reset DMA registers if the other direction is active */
	if (!ipq40xx_is_chn_already_inited(index, dir)) {

		val = readl(mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);
		val |= MBOX_DMA_POLICY_SW_RESET;
		writel(val, mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);

		mdelay(10);

		val &= ~(MBOX_DMA_POLICY_SW_RESET);
		writel(val, mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);
	}

	desc = mbox_rtime[index]->dir_priv[dir].dma_virt_head;
	phys_addr = mbox_rtime[index]->dir_priv[dir].dma_phys_head;
	val = readl(mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);

	if (dir == PLAYBACK) {
		/* Request the DMA channel to the controller */

		val |= MBOX_DMA_POLICY_RX_INT_TYPE;
		writel(val, mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);

		/* The direction is indicated from the DMA engine perspective
		 * i.e. we'll be using the RX registers for Playback and
		 * the TX registers for capture */
		writel((readl(mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG) |
			ADSS_MBOX_DMA_POLICY_SRAM_AC(phys_addr)),
				mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);
		writel(phys_addr & 0xfffffff,
			mbox_reg + ADSS_MBOXn_MBOXn_DMA_RX_DESCRIPTOR_BASE_REG);

		err = ipq40xx_mbox_interrupt_enable(channel_id,
				MBOX_INT_ENABLE_RX_DMA_COMPLETE);
	} else {

		val |= MBOX_DMA_POLICY_TX_INT_TYPE |
			ADSS_MBOX_DMA_POLICY_TX_FIFO_THRESHOLD(6);

		writel(val, mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);

		writel((readl(mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG) |
			ADSS_MBOX_DMA_POLICY_SRAM_AC(phys_addr)),
				mbox_reg + ADSS_MBOXn_MBOX_DMA_POLICY_REG);

		writel(phys_addr & 0xfffffff,
			mbox_reg + ADSS_MBOXn_MBOXn_DMA_TX_DESCRIPTOR_BASE_REG);

		err = ipq40xx_mbox_interrupt_enable(channel_id,
				MBOX_INT_ENABLE_TX_DMA_COMPLETE);
	}

	return err;
}
EXPORT_SYMBOL(ipq40xx_mbox_dma_prepare);

void ipq40xx_mbox_vuc_setup(int channel_id)
{
	uint32_t index, dir;
	struct ipq40xx_mbox_desc *desc;
	int ndescs;
	int i;

	index = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);
	ndescs = mbox_rtime[index]->dir_priv[dir].ndescs;
	desc = mbox_rtime[index]->dir_priv[dir].dma_virt_head;

	/* Copy VUC from previous descriptors */
	for (i = 0; i < ndescs; i++) {
		/* Setup V bits as 1, Acc to IEC 60958-3 Standard
		 *    for non PCM data, we need to set invalid for
		 *     both channels
		 * There are 6 DWORDS (192 bits) for Channel A
		 * and 6 DWORDS (192 bits) for channel B
		 */
		desc[i].vuc_dword[CHANNEL_A_VDWORD_1] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_A_VDWORD_2] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_A_VDWORD_3] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_A_VDWORD_4] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_A_VDWORD_5] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_A_VDWORD_6] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_B_VDWORD_1] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_B_VDWORD_2] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_B_VDWORD_3] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_B_VDWORD_4] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_B_VDWORD_5] = ADSS_MBOX_INVALID_PCM;
		desc[i].vuc_dword[CHANNEL_B_VDWORD_6] = ADSS_MBOX_INVALID_PCM;

		/* Now setup C bits, acc to IEC-60958-3 */
		desc[i].vuc_dword[CHANNEL_A_CDWORD_1] = SPDIF_CONSUMER_COMPRESD;
		desc[i].vuc_dword[CHANNEL_B_CDWORD_2] = SPDIF_CONSUMER_COMPRESD;
	}
}
EXPORT_SYMBOL(ipq40xx_mbox_vuc_setup);

int ipq40xx_mbox_form_ring(int channel_id, dma_addr_t baseaddr, u8 *area,
				int period_bytes, int bufsize, int own_bit)
{
	struct ipq40xx_mbox_desc *desc, *_desc_p;
	dma_addr_t desc_p, baseaddr_const;
	unsigned int i, ndescs;
	uint32_t index, dir;
	struct ipq40xx_mbox_rt_dir_priv *mbox_cb;

	index = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	if (!mbox_rtime[index])
		return -ENOMEM;

	mbox_cb = &mbox_rtime[index]->dir_priv[dir];
	ndescs = ((bufsize + (period_bytes - 1)) / period_bytes);

	desc = (struct ipq40xx_mbox_desc *)(area + (ndescs * period_bytes));
	desc_p = baseaddr + (ndescs * period_bytes);

	/* Finding whether duplicate desc entries are required or not should
	 * be done after desc is initialized else if ndescs are less than
	 * MBOX_MIN_DESC_NUM and this if condition is before desc initialization
	 * then desc will point to a region beyond allocated coherent memory */
	ndescs = ipq40xx_get_mbox_descs_duplicate(ndescs);

	memset(desc, 0, ndescs * sizeof(struct ipq40xx_mbox_desc));
	mbox_cb->read = 0;
	mbox_cb->write = 0;
	mbox_rtime[index]->dir_priv[dir].ndescs = ndescs;
	mbox_rtime[index]->dir_priv[dir].dma_virt_head = desc;
	mbox_rtime[index]->dir_priv[dir].dma_phys_head = desc_p;
	_desc_p = (struct ipq40xx_mbox_desc *)desc_p;

	baseaddr_const = baseaddr;

	for (i = 0; i < ndescs; i++) {

		desc->OWN = own_bit;
		desc->ei = 1;
		desc->rsvd1 = desc->rsvd2 = desc->rsvd3 = desc->EOM = 0;
		desc->BufPtr = (baseaddr & 0xfffffff);
		desc->NextPtr =
			(dma_addr_t)(&_desc_p[(i + 1) % ndescs]) & 0xfffffff;
		desc->size = period_bytes;
		desc->length = desc->size;
		baseaddr += ALIGN(period_bytes, L1_CACHE_BYTES);
		if (baseaddr >= (baseaddr_const + bufsize)) {
			if (bufsize % period_bytes)
				desc->size = bufsize % period_bytes;
			else
				desc->size = period_bytes;

			baseaddr = baseaddr_const;
		}
		desc += 1;
	}

	return 0;
}
EXPORT_SYMBOL(ipq40xx_mbox_form_ring);

int ipq40xx_mbox_dma_release(int channel_id)
{
	uint32_t index, dir;

	index = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	if (!mbox_rtime[index]) {
		/* Handling an error condition where same channel
		 * is been forced to release more than once */
		return -EINVAL;
	}

	if (test_bit(CHN_STARTED, &mbox_rtime[index]->dir_priv[dir].status)) {
		ipq40xx_mbox_interrupt_disable(channel_id,
				(MBOX_INT_ENABLE_TX_DMA_COMPLETE |
					MBOX_INT_ENABLE_RX_DMA_COMPLETE));

		mbox_rtime[index]->dir_priv[dir].dma_virt_head = NULL;
		clear_bit(CHN_STARTED,
				&mbox_rtime[index]->dir_priv[dir].status);
		return 0;
	}

	return -ENXIO;
}
EXPORT_SYMBOL(ipq40xx_mbox_dma_release);

static irqreturn_t ipq40xx_mbox_dma_irq(int irq, void *dev_id)
{
	uint32_t ret = IRQ_NONE;
	unsigned int status;
	struct ipq40xx_mbox_rt_priv *curr_rtime =
				(struct ipq40xx_mbox_rt_priv*)dev_id;

	status = readl(curr_rtime->mbox_reg_base +
			ADSS_MBOXn_MBOX_INT_STATUS_REG);

	if (status & MBOX_INT_STATUS_RX_DMA_COMPLETE) {
		ipq40xx_mbox_interrupt_ack(
			curr_rtime->dir_priv[PLAYBACK].channel_id,
			MBOX_INT_STATUS_RX_DMA_COMPLETE);

		if (curr_rtime->dir_priv[PLAYBACK].callback)
			curr_rtime->dir_priv[PLAYBACK].callback(irq,
				curr_rtime->dir_priv[PLAYBACK].dai_priv);
		ret = IRQ_HANDLED;
	}

	if (status & MBOX_INT_STATUS_TX_DMA_COMPLETE) {
		ipq40xx_mbox_interrupt_ack(
			curr_rtime->dir_priv[CAPTURE].channel_id,
			MBOX_INT_STATUS_TX_DMA_COMPLETE);

		if (curr_rtime->dir_priv[CAPTURE].callback)
			curr_rtime->dir_priv[CAPTURE].callback(irq,
				curr_rtime->dir_priv[CAPTURE].dai_priv);
		ret = IRQ_HANDLED;
	}

	if ((status & MBOX_INT_STATUS_RX_UNDERFLOW) ||
			(status & MBOX_INT_STATUS_RX_FIFO_UNDERFLOW)) {
		curr_rtime->dir_priv[PLAYBACK].err_stats++;
		ret = IRQ_HANDLED;
	}

	if ((status & MBOX_INT_STATUS_TX_OVERFLOW) ||
			(status & MBOX_INT_STATUS_TX_FIFO_OVERFLOW)) {
		curr_rtime->dir_priv[CAPTURE].err_stats++;
		ret = IRQ_HANDLED;
	}

	return ret;
}

int ipq40xx_mbox_dma_init(struct device *dev, int channel_id,
			irq_handler_t callback, void *private_data)
{
	uint32_t index;
	uint32_t dir;

	index = ipq40xx_convert_id_to_channel(channel_id);
	dir = ipq40xx_convert_id_to_dir(channel_id);

	if (index  >= ADSS_MBOX_NR_CHANNELS)
		return -EINVAL;

	if (!mbox_rtime[index])
		return -ENOMEM;

	if (!(mbox_rtime[index]->dir_priv[dir].status & CHN_ENABLED))
		return -EINVAL;

	if (test_and_set_bit(CHN_STARTED, &mbox_rtime[index]->dir_priv[dir].status)) {
		return -EBUSY;
	}

	mbox_rtime[index]->dir_priv[dir].dai_priv = private_data;
	mbox_rtime[index]->dir_priv[dir].callback = callback;
	mbox_rtime[index]->dir_priv[dir].dev = dev;

	return 0;
}
EXPORT_SYMBOL(ipq40xx_mbox_dma_init);

static int ipq40xx_mbox_probe(struct platform_device *pdev)
{
	struct device_node *np = NULL;
	int irq;
	uint32_t tx_channel;
	uint32_t rx_channel;
	uint32_t id;
	void __iomem *reg_base;
	struct resource *res;
	int rc;

	if (!pdev)
		return -ENODEV;

	np = of_node_get(pdev->dev.of_node);
	if ((of_property_read_u32(np, "dma-index", &id))) {
		pr_err("%s: error reading critical device "
				"node properties\n", np->name);
		rc = -EINVAL;
		goto init_err;
	}

	if (id >= ADSS_MBOX_NR_CHANNELS) {
		rc = -EINVAL;
		goto init_err;
	}

	if ((of_property_read_u32(np, "tx-channel", &tx_channel)))
		tx_channel = CHN_STATUS_DISABLE;

	if ((of_property_read_u32(np, "rx-channel", &rx_channel)))
		rx_channel = CHN_STATUS_DISABLE;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "%s: %d: Error getting mbox resource\n",
						__func__, __LINE__);
		return -ENODEV;
	}

	/* Read interrupt and store */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "%s: MBOX %d IRQ is not provided\n",
						__func__, id);
		rc = -EINVAL;
		goto init_err;
	}

	reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(reg_base)) {
		of_node_put(pdev->dev.of_node);
		return PTR_ERR(reg_base);
	}

	if (!mbox_rtime[id])
		mbox_rtime[id] = kzalloc(sizeof(struct ipq40xx_mbox_rt_priv),
								GFP_KERNEL);

	rc = request_irq(irq, ipq40xx_mbox_dma_irq, 0, "ipq40xx-mbox",
					mbox_rtime[id]);
	if (rc) {
		of_node_put(pdev->dev.of_node);
		kfree(mbox_rtime[id]);
		return rc;
	}

	mbox_rtime[id]->mbox_reg_base = reg_base;
	mbox_rtime[id]->dir_priv[PLAYBACK].channel_id = tx_channel;
	mbox_rtime[id]->dir_priv[CAPTURE].channel_id = rx_channel;
	mbox_rtime[id]->dir_priv[PLAYBACK].status =
		(tx_channel == CHN_STATUS_DISABLE) ? CHN_DISABLED : CHN_ENABLED;
	mbox_rtime[id]->dir_priv[CAPTURE].status =
		(rx_channel == CHN_STATUS_DISABLE) ? CHN_DISABLED : CHN_ENABLED;
	mbox_rtime[id]->irq_no = irq;
init_err:
	of_node_put(pdev->dev.of_node);
	return rc;
}

static int ipq40xx_mbox_remove(struct platform_device *pdev)
{
	uint32_t i;

	for (i = 0; i < ADSS_MBOX_NR_CHANNELS; i++) {
		if (mbox_rtime[i]) {
			free_irq(mbox_rtime[i]->irq_no, mbox_rtime[i]);
			ipq40xx_mbox_dma_release(i * 2 + PLAYBACK);
			ipq40xx_mbox_dma_release(i * 2 + CAPTURE);
			kfree(mbox_rtime[i]);
			mbox_rtime[i] = NULL;
		}
	}
	return 0;
}

static const struct of_device_id ipq40xx_mbox_table[] = {
	{ .compatible = "qca,ipq40xx-mbox" },
	{},
};

static struct platform_driver ipq40xx_mbox_driver = {
	.probe = ipq40xx_mbox_probe,
	.remove = ipq40xx_mbox_remove,
	.driver = {
		.name = "ipq40xx-mbox",
		.owner = THIS_MODULE,
		.of_match_table = ipq40xx_mbox_table,
	},
};

module_platform_driver(ipq40xx_mbox_driver);

MODULE_ALIAS("platform:ipq40xx-mbox");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("IPQ40xx MBOX DRIVER");
