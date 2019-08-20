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

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/io.h>

#include "ipq40xx-pcm-raw.h"
#include "ipq40xx-adss.h"
#include "ipq40xx-mbox.h"

/*
 * Constant / Define Declarations
 */
#define PCM_OFFSET_VAL		0
#define PCM_DIVIDER_VAL		1
#define PCM_START_VAL		1
#define PCM_STOP_VAL		0

#define PCM_MULT_FACTOR		4
#define SET_DESC_OWN		1

/*
 * Global Constant Definitions
 */
void __iomem *adss_pcm_base;

/*
 * Static Variable Definitions
 */
static atomic_t data_avail;
static atomic_t data_at;
static struct voice_dma_buffer *rx_dma_buffer;
static struct voice_dma_buffer *tx_dma_buffer;
static struct platform_device *pcm_pdev;
static spinlock_t pcm_lock;
static struct pcm_context context;
static struct clk *pcm_clk;

static uint32_t rx_size_count;

static DECLARE_WAIT_QUEUE_HEAD(pcm_q);

/*
 * Functions Forward Declarations
 */
static int voice_allocate_dma_buffer(struct device *dev,
		struct voice_dma_buffer *dma_buffer);
static int voice_free_dma_buffer(struct device *dev,
		struct voice_dma_buffer *dma_buff);

static void pcm_init_tx_data(struct ipq_pcm_params *params);

/*
 * FUNCTION: pcm_rx_irq_handler
 *
 * DESCRIPTION: pcm rx dma irq handler
 *
 * RETURN VALUE: none
 */
static irqreturn_t pcm_rx_irq_handler(int intrsrc, void *data)
{
	uint32_t dma_at;
	uint32_t rx_size;

	rx_size = ipq40xx_mbox_get_played_offset_set_own(
			rx_dma_buffer->channel_id);

	/* the buffer number calculated would actually point to the next
	 * buffer to be played. We need to go to the previous buffer, keeping
	 * ring buffer in picture.
	 */
	dma_at = rx_size / (rx_dma_buffer->size / NUM_BUFFERS);
	dma_at = (dma_at + NUM_BUFFERS - 1) % NUM_BUFFERS;

	atomic_set(&data_at, dma_at);
	atomic_set(&data_avail, 1);

	wake_up_interruptible(&pcm_q);

	return IRQ_HANDLED;
}

/*
 * FUNCTION: pcm_tx_irq_handler
 *
 * DESCRIPTION: pcm dma tx irq handler
 *
 * RETURN VALUE: none
 */
static irqreturn_t pcm_tx_irq_handler(int intrsrc, void *data)
{
	ipq40xx_mbox_get_elapsed_size(tx_dma_buffer->channel_id);
	/* do nothing for now, done in rx irq handler */
	return IRQ_HANDLED;
}

/*
 * FUNCTION: ipq40xx_pcm_validate_params
 *
 * DESCRIPTION: validates the input parameters
 *
 * RETURN VALUE: error if any
 */
uint32_t ipq40xx_pcm_validate_params(struct ipq_pcm_params *params)
{
	uint32_t count;

	if (!params) {
		pr_err("%s: Invalid Params.\n", __func__);
		return -EINVAL;
	}

	/* Bit width supported is 8 or 16 */
	if (!((params->bit_width == IPQ40xx_PCM_BIT_WIDTH_8) ||
			(params->bit_width == IPQ40xx_PCM_BIT_WIDTH_16))) {
		pr_err("%s: Invalid Bitwidth %d.\n",
				__func__, params->bit_width);
		return -EINVAL;
	}

	if ((params->bit_width == IPQ40xx_PCM_BIT_WIDTH_8) &&
			(params->slot_count != IPQ40xx_PCM_SLOTS_32)) {
		pr_err("%s: Invalid slot count for bit width 8 %d.\n",
				__func__, params->slot_count);
		return -EINVAL;
	}

	if ((params->bit_width == IPQ40xx_PCM_BIT_WIDTH_16) &&
			(params->slot_count != IPQ40xx_PCM_SLOTS_16)) {
		pr_err("%s: Invalid slot count for bit width 16 %d.\n",
				__func__, params->slot_count);
		return -EINVAL;
	}

	if ((params->rate <  IPQ40xx_PCM_SAMPLING_RATE_MIN) ||
		(params->rate > IPQ40xx_PCM_SAMPLING_RATE_MAX)) {
		pr_err("%s: Invalid sampling rate %d.\n",
				__func__, params->rate);
		return -EINVAL;
	}

	/* Number of active slots should be less than or same as max slots */
	if ((!params->active_slot_count) ||
			(params->active_slot_count > IPQ40xx_PCM_MAX_SLOTS)) {
		pr_err("%s: Active slots should be less than or same as %d\n",
				__func__, IPQ40xx_PCM_MAX_SLOTS);
		return -EINVAL;
	}

	for (count = 0; count < params->active_slot_count; count++) {
		if (params->tx_slots[count] >= params->slot_count) {
			pr_err("%s: slot number in tx_slots should be less than %d.\n",
					__func__, params->slot_count);
			return -EINVAL;
		}
		if (params->rx_slots[count] >= params->slot_count) {
			pr_err("%s: slot number in rx_slots should be less than %d.\n",
					__func__, params->slot_count);
			return -EINVAL;
		}
		if (params->rx_slots[count] != params->tx_slots[count]) {
			pr_err("%s: rx_slots should be equal to tx slots.\n",
					__func__);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * FUNCTION: ipq40xx_pcm_init
 *
 * DESCRIPTION: initializes PCM interface and MBOX interface
 *
 * RETURN VALUE: error if any
 */
int ipq_pcm_init(struct ipq_pcm_params *params)
{
	int ret;
	uint32_t single_buf_size;
	uint32_t clk_rate;
	uint32_t i;
	uint32_t reg_val;

	ret = ipq40xx_pcm_validate_params(params);
	if (ret)
		return ret;

	atomic_set(&data_avail, 0);
	atomic_set(&data_at, 0);

	clk_rate = params->slot_count * params->bit_width
					* params->rate * PCM_MULT_FACTOR;

	ret = clk_set_rate(pcm_clk, clk_rate);
	if (ret) {
		pr_err("%s : clk_set_rate failed for pcm clock\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(pcm_clk);
	if (ret) {
		pr_err("%s : clk_prepare_enable failed for pcm clock\n",
					__func__);
		return ret;
	}

	ipq40xx_glb_audio_mode_B1K();

	/* set ADSS_PCM_CTRL_REG */
	reg_val = PCM_CTRL_CPU_MODE(0) | PCM_CTRL_FRAME_SYNC_LEN(0);

	if (params->bit_width == IPQ40xx_PCM_BIT_WIDTH_16)
		reg_val |= PCM_CTRL_PCM_SLOT_MODE(1);

	writel(reg_val, adss_pcm_base + AADSS_PCM_CTRL_REG);

	/* write ADSS_PCM_OFFSET_REG */
	writel(PCM_OFFSET_VAL, adss_pcm_base + AADSS_PCM_OFFSET_REG);

	/* write ADSS_PCM_DIVIDER_REG */
	writel(PCM_DIVIDER_VAL, adss_pcm_base + AADSS_PCM_DIVIDER_REG);

	/* write ADSS_PCM_BITMAP_REG */
	reg_val = 0;
	for (i = 0; i < params->active_slot_count; i++)
		reg_val |= (1 << params->tx_slots[i]);

	writel(reg_val, adss_pcm_base + AADSS_PCM_BITMAP_REG);

	/* allocate DMA buffers */
	single_buf_size = IPQ40xx_PCM_SAMPLES_PER_10MS(params->rate) *
			IPQ40xx_PCM_BYTES_PER_SAMPLE(params->bit_width) *
			params->active_slot_count;

	rx_dma_buffer->size = single_buf_size * NUM_BUFFERS;
	ret = voice_allocate_dma_buffer(&pcm_pdev->dev, rx_dma_buffer);
	if (ret) {
		dev_err(&pcm_pdev->dev, "\n%s: %d:Error in allocating rx dma buffer, error %d\n",
				__func__, __LINE__, ret);
		return -ENOMEM;
	}

	tx_dma_buffer->size = single_buf_size * NUM_BUFFERS;
	ret = voice_allocate_dma_buffer(&pcm_pdev->dev, tx_dma_buffer);
	if (ret) {
		dev_err(&pcm_pdev->dev, "\n%s: %d:Error in allocating tx dma buffer, error: %d\n",
				__func__, __LINE__, ret);
		ret = -ENOMEM;
		goto err_mem_tx;
	}

	pcm_init_tx_data(params);

	/* initalize mbox here*/
	/* first for QCM_PCM_STREAM_PLAYBACK */
	ret = ipq40xx_mbox_dma_init(&pcm_pdev->dev,
			tx_dma_buffer->channel_id,
			pcm_tx_irq_handler, NULL);

	if (ret) {
		pr_err("\n%s: %d: Error initializing dma for playback, error %d\n",
				__func__, __LINE__, ret);
		goto err_mbox_tx_init;
	}

	ret = ipq40xx_mbox_form_ring(tx_dma_buffer->channel_id,
			tx_dma_buffer->addr,
			tx_dma_buffer->area,
			single_buf_size,
			tx_dma_buffer->size,
			SET_DESC_OWN);
	if (ret) {
		pr_err("\n%s: %d: Error dma form ring for playback, error %d\n",
				__func__, __LINE__, ret);
		goto err_mbox_tx_dma_init;
	}

	ret = ipq40xx_mbox_dma_prepare(tx_dma_buffer->channel_id);
	if (ret) {
		pr_err("\n%s: %d: Error dma prepare for playback, error %d\n",
					__func__, __LINE__, ret);
		goto err_mbox_tx_dma_init;
	}

	/* next for IPQ40xx_PCM_STREAM_CAPTURE */
	ret = ipq40xx_mbox_dma_init(&pcm_pdev->dev,
			rx_dma_buffer->channel_id,
			pcm_rx_irq_handler, NULL);

	if (ret) {
		pr_err("\n %s: %d: Error initializing dma for capture, error: %d\n",
				__func__, __LINE__, ret);
		goto err_mbox_tx_dma_init;
	}

	ret = ipq40xx_mbox_form_ring(rx_dma_buffer->channel_id,
			rx_dma_buffer->addr,
			rx_dma_buffer->area,
			single_buf_size,
			rx_dma_buffer->size,
			SET_DESC_OWN);
	if (ret) {
		pr_err("\n %s: %d: Error dma form ring for capture, error: %d\n",
				__func__, __LINE__, ret);
		goto err_mbox_rx_dma_init;
	}

	ret = ipq40xx_mbox_dma_prepare(rx_dma_buffer->channel_id);
	if (ret) {
		pr_err("\n%s: %d: Error dma prepare for capture, error %d\n",
					__func__, __LINE__, ret);
		goto err_mbox_rx_dma_init;
	}

	/*take pcm out of reset */
	ipq40xx_glb_pcm_rst(DISABLE);

	writel(PCM_START_VAL, adss_pcm_base + AADSS_PCM_START_REG);

	context.needs_deinit = 1;
	context.pcm_started = 1;
	rx_size_count = 0;

	ipq40xx_mbox_fifo_reset(rx_dma_buffer->channel_id);
	ipq40xx_mbox_fifo_reset(tx_dma_buffer->channel_id);

	/* Give delay for the fifo reset to reflect */
	mdelay(20);

	ipq40xx_mbox_dma_start(rx_dma_buffer->channel_id);
	ipq40xx_mbox_dma_start(tx_dma_buffer->channel_id);


	return 0;

err_mbox_rx_dma_init:
	ipq40xx_mbox_dma_release(rx_dma_buffer->channel_id);
err_mbox_tx_dma_init:
	ipq40xx_mbox_dma_release(tx_dma_buffer->channel_id);
err_mbox_tx_init:
	voice_free_dma_buffer(&pcm_pdev->dev, tx_dma_buffer);
err_mem_tx:
	voice_free_dma_buffer(&pcm_pdev->dev, rx_dma_buffer);
	return ret;
}
EXPORT_SYMBOL(ipq_pcm_init);


static void pcm_init_tx_data(struct ipq_pcm_params *params)
{
	uint32_t i, slot;
	uint32_t *buffer;

	buffer = (uint32_t *)tx_dma_buffer->area;

	for (i = 0; i < tx_dma_buffer->size/4; ) {
		for (slot = 0; slot < params->active_slot_count; slot++) {
			if (params->bit_width == IPQ40xx_PCM_BIT_WIDTH_16) {
				buffer[i] |= params->tx_slots[slot] << 16;
			} else {
				buffer[i] |= params->tx_slots[slot] << 8;
				buffer[i] |= 1 << 15; /* valid bit */
			}
			i++;
		}
	}
}
/*
 * FUNCTION: ipq_pcm_data
 *
 * DESCRIPTION: calculate the free tx buffer and full rx buffers for use by the
 *		upper layer
 *
 * RETURN VALUE: returns the rx and tx buffer pointers and the size to fill or
 *		read
 */
uint32_t ipq_pcm_data(char **rx_buf, char **tx_buf)
{
	unsigned long flag;
	uint32_t offset;
	uint32_t size;

	wait_event_interruptible(pcm_q, atomic_read(&data_avail) != 0);

	atomic_set(&data_avail, 0);
	offset = (rx_dma_buffer->size / NUM_BUFFERS) * atomic_read(&data_at);

	spin_lock_irqsave(&pcm_lock, flag);
	*rx_buf = rx_dma_buffer->area + offset;
	*tx_buf = tx_dma_buffer->area + offset;
	spin_unlock_irqrestore(&pcm_lock, flag);

	size = rx_dma_buffer->size / NUM_BUFFERS;

	return size;
}
EXPORT_SYMBOL(ipq_pcm_data);

/*
 * FUNCTION: ipq_pcm_done
 *
 * DESCRIPTION: this api tells the PCM that the upper layer has finished
 *		updating the Tx buffer
 *
 * RETURN VALUE: none
 */
void ipq_pcm_done(void)
{
	atomic_set(&data_avail, 0);
}
EXPORT_SYMBOL(ipq_pcm_done);


void ipq_pcm_send_event(void)
{
	atomic_set(&data_avail, 1);
	wake_up_interruptible(&pcm_q);
}
EXPORT_SYMBOL(ipq_pcm_send_event);

/*
 * FUNCTION: ipq_pcm_deinit
 *
 * DESCRIPTION: deinitialization api, clean up everything
 *
 * RETURN VALUE: none
 */
void ipq_pcm_deinit(void)
{
	if (context.needs_deinit) {

		ipq40xx_mbox_dma_stop(rx_dma_buffer->channel_id);
		ipq40xx_mbox_dma_stop(tx_dma_buffer->channel_id);

		ipq40xx_mbox_dma_release(rx_dma_buffer->channel_id);
		ipq40xx_mbox_dma_release(tx_dma_buffer->channel_id);

		memset(&context, 0, sizeof(struct pcm_context));

		writel(PCM_STOP_VAL, adss_pcm_base + AADSS_PCM_START_REG);

		clk_disable_unprepare(pcm_clk);
	}
	if (tx_dma_buffer)
		voice_free_dma_buffer(&pcm_pdev->dev, tx_dma_buffer);
	if (rx_dma_buffer)
		voice_free_dma_buffer(&pcm_pdev->dev, rx_dma_buffer);
}
EXPORT_SYMBOL(ipq_pcm_deinit);

/*
 * FUNCTION: voice_allocate_dma_buffer
 *
 * DESCRIPTION: dma buffer allocation
 *
 * RETURN VALUE: error if any
 */
static int voice_allocate_dma_buffer(struct device *dev,
					struct voice_dma_buffer *dma_buffer)
{
	int size;
	int ndescs;

	ndescs = ipq40xx_get_mbox_descs_duplicate(NUM_BUFFERS);
	size = (dma_buffer->size * NUM_BUFFERS) + (ndescs *
				sizeof(struct ipq40xx_mbox_desc));

	dma_buffer->area = dma_zalloc_coherent(dev, size,
					&dma_buffer->addr, GFP_KERNEL);

	if (!dma_buffer->area)
		return -ENOMEM;

	return 0;
}

/*
 * FUNCTION: voice_free_dma_buffer
 *
 * DESCRIPTION: dma buffer deallocation
 *
 * RETURN VALUE: error if any
 */
static int voice_free_dma_buffer(struct device *dev,
					struct voice_dma_buffer *dma_buff)
{
	if (!dma_buff->area)
		return 0;

	dma_free_coherent(dev, dma_buff->size + 4,
			dma_buff->area, dma_buff->addr);
	dma_buff->area = NULL;
	return 0;
}

/*
 * FUNCTION: ipq40xx_pcm_driver_probe
 *
 * DESCRIPTION: very basic one time activities
 *
 * RETURN VALUE: error if any
 */
static int ipq40xx_pcm_driver_probe(struct platform_device *pdev)
{
	uint32_t tx_channel;
	uint32_t rx_channel;
	struct resource *res;
	struct device_node *np = NULL;

	if (!pdev)
		return -EINVAL;

	pcm_pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	adss_pcm_base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(adss_pcm_base))
		return PTR_ERR(adss_pcm_base);

	np = of_node_get(pdev->dev.of_node);

	if (of_property_read_u32(np, "dma-tx-channel", &tx_channel)) {
		pr_err("%s: error reading critical device node properties\n",
				np->name);
		return -EINVAL;
	}

	if (of_property_read_u32(np, "dma-rx-channel", &rx_channel)) {
		pr_err("%s: error reading critical device node properties\n",
				np->name);
		return -EINVAL;
	}

	pcm_clk = devm_clk_get(&pdev->dev, "audio_pcm_clk");
	if (IS_ERR(pcm_clk)) {
		pr_err("%s: Error in getting PCM Clk\n", __func__);
		return PTR_ERR(pcm_clk);
	}

	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	/* Allocate memory for rx  and tx instance */
	rx_dma_buffer = kzalloc(sizeof(struct voice_dma_buffer), GFP_KERNEL);
	if (rx_dma_buffer == NULL) {
		pr_err("%s: Error in allocating mem for rx_dma_buffer\n",
				__func__);
		return -ENOMEM;
	}

	tx_dma_buffer = kzalloc(sizeof(struct voice_dma_buffer), GFP_KERNEL);
	if (tx_dma_buffer == NULL) {
		pr_err("%s: Error in allocating mem for tx_dma_buffer\n",
				__func__);
		kfree(rx_dma_buffer);
		return -ENOMEM;
	}

	rx_dma_buffer->channel_id = rx_channel;
	tx_dma_buffer->channel_id = tx_channel;

	memset(&context, 0, sizeof(struct pcm_context));
	spin_lock_init(&pcm_lock);

	return 0;
}

/*
 * FUNCTION: ipq40xx_pcm_driver_remove
 *
 * DESCRIPTION: clean up
 *
 * RETURN VALUE: error if any
 */
static int ipq40xx_pcm_driver_remove(struct platform_device *pdev)
{
	ipq_pcm_deinit();
	devm_clk_put(&pdev->dev, pcm_clk);
	kfree(rx_dma_buffer);
	kfree(tx_dma_buffer);
	return 0;
}

/*
 * DESCRIPTION OF PCM RAW MODULE
 */

#define DRIVER_NAME "ipq40xx_pcm_raw"

static const struct of_device_id qca_raw_match_table[] = {
	{ .compatible = "qca,ipq40xx-pcm", },
	{},
};

static struct platform_driver ipq40xx_pcm_raw_driver = {
	.probe		= ipq40xx_pcm_driver_probe,
	.remove		= ipq40xx_pcm_driver_remove,
	.driver		= {
		.name		= DRIVER_NAME,
		.of_match_table = qca_raw_match_table,
	},
};

module_platform_driver(ipq40xx_pcm_raw_driver);

MODULE_ALIAS(DRIVER_NAME);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("QCA RAW PCM VoIP Platform Driver");
