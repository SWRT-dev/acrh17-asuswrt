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
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#define USB30_HS_PHY_CTRL	0x00000010
#define SW_SESSVLD			(0x01 << 0x1C)
#define UTMI_OTG_VBUS_VALID	(0x01 << 0x14)

#define USB30_SS_PHY_CTRL	0x00000030
#define LANE0_PWR_PRESENT	(0x01 << 0x18)

struct dwc3_ipq40xx {
	struct device *dev;

	void __iomem *qscratch_base;

	struct clk *master_clk;
	struct clk *mock_utmi_clk;
	struct clk *sleep_clk;

	unsigned int host;
};

/**
 * Write register.
 *
 * @base - PHY base virtual address.
 * @offset - register offset.
 * @val - value to write.
 */
static inline void qscratch_write(void __iomem *base, u32 offset, u32 val)
{
	writel(val, base + offset);
}

/**
 * Write register and read back masked value to confirm it is written
 *
 * @base - base virtual address.
 * @offset - register offset.
 * @mask - register bitmask specifying what should be updated
 * @val - value to write.
 */
static inline void qscratch_write_readback(void __iomem *base, u32 offset,
		const u32 mask, u32 val)
{
	u32 write_val, tmp = readl(base + offset);

	tmp &= ~mask;       /* retain other bits */
	write_val = tmp | val;

	writel(write_val, base + offset);

	/* Read back to see if val was written */
	tmp = readl(base + offset);
	tmp &= mask;        /* clear other bits */

	if (tmp != val)
		pr_err("write: %x to QSCRATCH: %x FAILED\n", val, offset);
}

static void dwc3_ipq40xx_enable_vbus_valid(struct dwc3_ipq40xx *mdwc)
{
	if (!mdwc->host) {
		/* Enable VBUS valid for HS PHY*/
		qscratch_write_readback(mdwc->qscratch_base, USB30_HS_PHY_CTRL,
				SW_SESSVLD, SW_SESSVLD);
		qscratch_write_readback(mdwc->qscratch_base, USB30_HS_PHY_CTRL,
				UTMI_OTG_VBUS_VALID, UTMI_OTG_VBUS_VALID);

		/* Enable VBUS valid for SS PHY*/
		qscratch_write_readback(mdwc->qscratch_base, USB30_SS_PHY_CTRL,
				LANE0_PWR_PRESENT, LANE0_PWR_PRESENT);
	}
}

static int dwc3_ipq40xx_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct dwc3_ipq40xx *mdwc;
	int ret = 0;
	struct resource *res;
	struct device_node *np = NULL;

	mdwc = devm_kzalloc(&pdev->dev, sizeof(*mdwc), GFP_KERNEL);
	if (!mdwc) {
		pr_err("%s: failed to get memory\n", __func__);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, mdwc);

	mdwc->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mdwc->qscratch_base = devm_ioremap_resource(mdwc->dev, res);
	if (IS_ERR(mdwc->qscratch_base)) {
		dev_err(mdwc->dev, "failed to get qscratch res\n");
		return PTR_ERR(mdwc->qscratch_base);
	}

	mdwc->master_clk = devm_clk_get(mdwc->dev, "master");
	if (IS_ERR(mdwc->master_clk)) {
		dev_err(mdwc->dev, "failed to get master clock\n");
		return PTR_ERR(mdwc->master_clk);
	}

	mdwc->mock_utmi_clk = devm_clk_get(mdwc->dev, "mock_utmi");
	if (IS_ERR(mdwc->mock_utmi_clk)) {
		dev_err(mdwc->dev, "failed to get mock_utmi clock\n");
		return PTR_ERR(mdwc->mock_utmi_clk);
	}

	mdwc->sleep_clk = devm_clk_get(mdwc->dev, "sleep");
	if (IS_ERR(mdwc->sleep_clk)) {
		dev_dbg(mdwc->dev, "failed to get sleep clock\n");
		return PTR_ERR(mdwc->sleep_clk);
	}

	np = of_node_get(pdev->dev.of_node);
	if (of_property_read_u32(np, "qca,host", &mdwc->host)) {
		dev_err(mdwc->dev,
			"%s: error reading critical device node properties\n",
			np->name);
		return -EFAULT;
	}

	dwc3_ipq40xx_enable_vbus_valid(mdwc);

	clk_prepare_enable(mdwc->master_clk);
	clk_prepare_enable(mdwc->mock_utmi_clk);
	clk_prepare_enable(mdwc->sleep_clk);

	ret = of_platform_populate(node, NULL, NULL, mdwc->dev);
	if (ret) {
		dev_err(mdwc->dev, "failed to add create dwc3 core\n");
		goto dis_clks;
	}

	return 0;

dis_clks:
	dev_err(mdwc->dev, "disabling clocks\n");
	clk_disable_unprepare(mdwc->sleep_clk);
	clk_disable_unprepare(mdwc->mock_utmi_clk);
	clk_disable_unprepare(mdwc->master_clk);

	return ret;
}

static int dwc3_ipq40xx_remove_core(struct device *dev, void *c)
{
	struct platform_device *pdev = to_platform_device(dev);

	of_device_unregister(pdev);
	return 0;
}

static int dwc3_ipq40xx_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct dwc3_ipq40xx *mdwc = platform_get_drvdata(pdev);

	clk_disable_unprepare(mdwc->sleep_clk);
	clk_disable_unprepare(mdwc->mock_utmi_clk);
	clk_disable_unprepare(mdwc->master_clk);

	device_for_each_child(&pdev->dev, NULL, dwc3_ipq40xx_remove_core);

	return ret;
}

static const struct of_device_id of_dwc3_match[] = {
	{ .compatible = "qca,dwc3" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_dwc3_match);

static struct platform_driver dwc3_ipq40xx_driver = {
	.probe		= dwc3_ipq40xx_probe,
	.remove		= dwc3_ipq40xx_remove,
	.driver		= {
		.name	= "qca-dwc3",
		.owner	= THIS_MODULE,
		.of_match_table	= of_dwc3_match,
	},
};

module_platform_driver(dwc3_ipq40xx_driver);

MODULE_ALIAS("platform:qca-dwc3");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("DesignWare USB3 QCA Glue Layer");
