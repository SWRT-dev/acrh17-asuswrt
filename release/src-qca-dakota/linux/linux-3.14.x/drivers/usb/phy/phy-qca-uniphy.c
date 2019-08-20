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
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/phy.h>
#include <linux/reset.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>
#include <soc/qcom/scm.h>
#include <linux/slab.h>

#define USB_CALIBRATION_CMD	0x10
#define USB3PHY_SPARE_1		0x7FC
#define RX_LOS_1		0x7C8
#define MISC_SOURCE_REG		0x21c
#define CDR_CONTROL_REG_1	0x80
#define PCS_INTERNAL_CONTROL14	0x364
#define MMD1_REG_REG_MASK	(0x7F << 8)
#define OTP_MASK		(0x7F << 5)
#define MMD1_REG_AUTOLOAD_MASK	(0x1 << 7)
#define SPARE_1_BIT14_MASK	(0x1 << 14)
/**
 *  USB Hardware registers
 */
#define MDIO_CTRL_0_REG		0x40
#define MDIO_CTRL_1_REG		0x44
#define MDIO_CTRL_2_REG		0x48
#define MDIO_CTRL_3_REG		0x4C
#define MDIO_CTRL_4_REG		0x50

#define MDIO_USB_PHY_ID		(0x0 << 13)
#define MDC_MODE			(0x1 << 12)
#define MDIO_CLAUSE_22		(0x0 << 8)
#define MDIO_CLAUSE_45		(0x1 << 8)
#define MDIO_USB_CLK_DIV	(0xF)
#define MDIO_MMD_ID			(0x1)

#define MDIO_ACCESS_BUSY	(0x1 << 16)
#define MDIO_ACCESS_START	(0x1 << 8)
#define MDIO_TIMEOUT_STATIC	1000

#define MDIO_ACCESS_22_WRITE		(0x1)
#define MDIO_ACCESS_22_READ			(0x0)
#define MDIO_ACCESS_45_WRITE    	(0x2)
#define MDIO_ACCESS_45_READ     	(0x1)
#define MDIO_ACCESS_45_READ_ADDR	(0x0)

struct qca_uni_ss_phy {
	struct usb_phy phy;
	struct device *dev;

	void __iomem *base;

	struct reset_control *por_rst;

	unsigned int host;
	unsigned int emulation;
};

struct qf_read {
	uint32_t value;
};

#define	phy_to_dw_phy(x)	container_of((x), struct qca_uni_ss_phy, phy)

/**
 * Write register
 *
 * @base - PHY base virtual address.
 * @offset - register offset.
 */
static u32 qca_uni_ss_read(void __iomem *base, u32 offset)
{
	u32 value;
	value = readl_relaxed(base + offset);
	return value;
}

/**
 * Write register
 *
 * @base - PHY base virtual address.
 * @offset - register offset.
 * @val - value to write.
 */
static void qca_uni_ss_write(void __iomem *base, u32 offset, u32 val)
{
	writel(val, base + offset);
	udelay(100);
}

int mdio_wait(void __iomem *base)
{
	unsigned int mdio_access;
	unsigned int timeout = MDIO_TIMEOUT_STATIC;

	do {
		mdio_access = qca_uni_ss_read(base, MDIO_CTRL_4_REG);
		if (!timeout--)
			return -EFAULT;
	} while (mdio_access & MDIO_ACCESS_BUSY);

	return 0;
}

int mdio_mii_read(void __iomem *base, unsigned char regAddr, unsigned short *data)
{
	unsigned short mdio_ctl_0 = (MDIO_USB_PHY_ID | MDC_MODE | MDIO_CLAUSE_22
								| MDIO_USB_CLK_DIV);
	unsigned int regVal;

	qca_uni_ss_write(base, MDIO_CTRL_0_REG, mdio_ctl_0);
	qca_uni_ss_write(base, MDIO_CTRL_1_REG, regAddr);

	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_22_READ);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_22_READ| MDIO_ACCESS_START);

	/* wait for access busy to be cleared */
	if (mdio_wait(base)) {
		pr_err("%s MDIO Access Busy Timeout %x\n", __func__, regAddr);
		return -EFAULT;
	}

	regVal =  qca_uni_ss_read(base, MDIO_CTRL_3_REG);
	*data = (unsigned short)regVal;
	return 0;
}

int mdio_mii_write(void __iomem *base, unsigned char regAddr, unsigned short data)
{
	unsigned short mdio_ctl_0 = (MDIO_USB_PHY_ID | MDC_MODE | MDIO_CLAUSE_22
								| MDIO_USB_CLK_DIV);

	qca_uni_ss_write(base, MDIO_CTRL_0_REG, mdio_ctl_0);
	qca_uni_ss_write(base, MDIO_CTRL_1_REG, regAddr);
	qca_uni_ss_write(base, MDIO_CTRL_2_REG, data);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_22_WRITE);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_22_WRITE| MDIO_ACCESS_START);

	/* wait for access busy to be cleared */
	if (mdio_wait(base)) {
		pr_err("%s MDIO Access Busy Timeout %x\n", __func__, regAddr);
		return -EFAULT;
	}

	return 0;
}

int mdio_mmd_read(void __iomem *base, unsigned short regAddr, unsigned short *data)
{
	unsigned short mdio_ctl_0 = (MDIO_USB_PHY_ID | MDC_MODE | MDIO_CLAUSE_45
								| MDIO_USB_CLK_DIV);
	unsigned int regVal;

	qca_uni_ss_write(base, MDIO_CTRL_0_REG, mdio_ctl_0);
	qca_uni_ss_write(base, MDIO_CTRL_1_REG, MDIO_MMD_ID);
	qca_uni_ss_write(base, MDIO_CTRL_2_REG, regAddr);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_45_READ_ADDR);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_45_READ_ADDR| MDIO_ACCESS_START);

	/* wait for access busy to be cleared */
	if (mdio_wait(base)) {
		pr_err("%s MDIO Access Busy Timeout %x\n", __func__, regAddr);
		return -EFAULT;
	}

	qca_uni_ss_write(base, MDIO_CTRL_1_REG, MDIO_MMD_ID);
	qca_uni_ss_write(base, MDIO_CTRL_2_REG, regAddr);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_45_WRITE);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_45_WRITE| MDIO_ACCESS_START);

	/* wait for access busy to be cleared */
	if (mdio_wait(base)) {
		pr_err("%s MDIO Access Busy Timeout %x\n", __func__, regAddr);
		return -EFAULT;
	}

	regVal =  qca_uni_ss_read(base, MDIO_CTRL_3_REG);
	*data = (unsigned short)regVal;

	return 0;
}

int mdio_mmd_write(void __iomem *base, unsigned short regAddr, unsigned short data)
{

	unsigned short mdio_ctl_0 = (MDIO_USB_PHY_ID | MDC_MODE | MDIO_CLAUSE_45
								| MDIO_USB_CLK_DIV);

	qca_uni_ss_write(base, MDIO_CTRL_0_REG, mdio_ctl_0);
	qca_uni_ss_write(base, MDIO_CTRL_1_REG, MDIO_MMD_ID);
	qca_uni_ss_write(base, MDIO_CTRL_2_REG, regAddr);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_45_READ_ADDR);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_45_READ_ADDR| MDIO_ACCESS_START);

	/* wait for access busy to be cleared */
	if (mdio_wait(base)) {
		pr_err("%s MDIO Access Busy Timeout %x\n", __func__, regAddr);
		return -EFAULT;
	}

	qca_uni_ss_write(base, MDIO_CTRL_2_REG, data);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_45_READ);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_45_READ| MDIO_ACCESS_START);

	qca_uni_ss_write(base, MDIO_CTRL_2_REG, regAddr);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_45_WRITE);
	qca_uni_ss_write(base, MDIO_CTRL_4_REG, MDIO_ACCESS_45_WRITE| MDIO_ACCESS_START);

	/* wait for access busy to be cleared */
	if (mdio_wait(base)) {
		pr_err("%s MDIO Access Busy Timeout %x\n", __func__, regAddr);
		return -EFAULT;
	}

	return 0;
}

static void qca_uni_ss_phy_shutdown(struct usb_phy *x)
{
	struct qca_uni_ss_phy *phy = phy_to_dw_phy(x);

	/* assert SS PHY POR reset */
	reset_control_assert(phy->por_rst);
}

extern uint32_t *gl_tmp_otp_value;
/* Function to read the value from OTP through scm call */
int qca_uni_ss_read_otp(uint32_t *otp_read_data)
{
	int ret, alloc=0;
	uint32_t *otp_value = gl_tmp_otp_value;
	struct qf_read rdip;

	if (!otp_value) {
		pr_err("tmp_otp is NULL!!!\n");
		otp_value = kzalloc(sizeof(uint32_t), GFP_KERNEL);
		if (!otp_value)
			return -ENOMEM;
		alloc=1;
	}

	rdip.value = dma_map_single(NULL, otp_value,
			sizeof(uint32_t), DMA_FROM_DEVICE);

	ret = dma_mapping_error(NULL, rdip.value);
	if (ret != 0) {
		pr_err("DMA Mapping Error\n");
		goto err_write;
	}

	ret = scm_call(SCM_SVC_FUSE, USB_CALIBRATION_CMD,
		&rdip, sizeof(struct qf_read), NULL, 0);

	dma_unmap_single(NULL, rdip.value,
		sizeof(uint32_t), DMA_FROM_DEVICE);

	if (ret != 0) {
		pr_err("Error in reading value: %d\n", ret);
		goto err_write;
	}

	*otp_read_data = *otp_value;
err_write:
	if (alloc)
		kfree(otp_value);
	return ret;
}

int qca_uni_ss_phy_usb_los_calibration(void __iomem *base)
{
	uint32_t data, otp_val = 0;

	/* Get OTP value */
	if ((qca_uni_ss_read_otp(&otp_val) < 0) || (!(otp_val & OTP_MASK))) {
		pr_err("USB Calibration Falied with error %d\n", otp_val);
		return 0;
	}

	/*
	 * Read the USB3PHY_SPARE_1 register and
	 * set bit 14 to 0
	 */
	data = qca_uni_ss_read(base, USB3PHY_SPARE_1);
	data = data & (~SPARE_1_BIT14_MASK);
	qca_uni_ss_write(base, USB3PHY_SPARE_1, data);

	/*
	 * Get bit 11:5 value, add with 0x14 and set to the
	 * register USB3PHY_RX_LOS_1 bit MMD1_REG_REG
	 */
	data = qca_uni_ss_read(base, RX_LOS_1);
	otp_val = ((otp_val & OTP_MASK) >> 5) + 0x14;
	otp_val = otp_val << 8;
	data = data & (~MMD1_REG_REG_MASK);
	data = data | otp_val;
	qca_uni_ss_write(base, RX_LOS_1, data);

	/*
	 * Set bit MMD1_REG_AUTOLOAD_SEL_RX_LOS_THRES in
	 * USB3PHY_RX_LOS_1 to 1
	 */
	data = qca_uni_ss_read(base, RX_LOS_1);
	data = data | MMD1_REG_AUTOLOAD_MASK;
	qca_uni_ss_write(base, RX_LOS_1, data);

	qca_uni_ss_write(base, PCS_INTERNAL_CONTROL14, 0x4000);
	qca_uni_ss_write(base, MISC_SOURCE_REG, 0xaa0a);
	qca_uni_ss_write(base, CDR_CONTROL_REG_1, 0x0202);

	return 0;
}

static int qca_uni_ss_phy_init(struct usb_phy *x)
{
	int ret;
	struct qca_uni_ss_phy *phy = phy_to_dw_phy(x);

	/* assert SS PHY POR reset */
	reset_control_assert(phy->por_rst);

	msleep(100);

	if (phy->emulation) {
		mdio_mii_write(phy->base, 0x1, 0x8017);
		mdio_mii_write(phy->base, 0xb, 0x300d);
		mdio_mmd_write(phy->base, 0x2d, 0x681a);
		mdio_mmd_write(phy->base, 0x7d, 0x8);
		mdio_mmd_write(phy->base, 0x7f, 0x5ed5);
		mdio_mmd_write(phy->base, 0x87, 0xaa0a);
		mdio_mmd_write(phy->base, 0x4, 0x0802);
		mdio_mmd_write(phy->base, 0x8, 0x0280);
		mdio_mmd_write(phy->base, 0x9, 0x8854);
		mdio_mmd_write(phy->base, 0xa, 0x2815);
		mdio_mmd_write(phy->base, 0xb, 0x2120);
		mdio_mmd_write(phy->base, 0xb, 0x0120);
		mdio_mmd_write(phy->base, 0xc, 0x0480);
		mdio_mmd_write(phy->base, 0x13, 0x8000);
		mdio_mmd_write(phy->base, 0x7c, 0x82);
	} else {
		/* Add ASIC PHY register write here */
	}

	msleep(10);

	/* deassert SS PHY POR reset */
	reset_control_deassert(phy->por_rst);

	/* USB LOS Calibration */
	ret = qca_uni_ss_phy_usb_los_calibration(phy->base);

	return ret;
}

static int qca_uni_ss_get_resources(struct platform_device *pdev,
		struct qca_uni_ss_phy *phy)
{
	struct resource *res;
	struct device_node *np = NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	phy->base = devm_ioremap_resource(phy->dev, res);
	if (IS_ERR(phy->base))
		return PTR_ERR(phy->base);

	phy->por_rst = devm_reset_control_get(phy->dev, "por_rst");
	if (IS_ERR(phy->por_rst))
		return PTR_ERR(phy->por_rst);

	np = of_node_get(pdev->dev.of_node);
	if (of_property_read_u32(np, "qca,host", &phy->host)
			|| of_property_read_u32(np, "qca,emulation", &phy->emulation)) {
		pr_err("%s: error reading critical device node properties\n", np->name);
		return -EFAULT;
	}

	return 0;
}

static int qca_dummy_ss_phy_init(struct usb_phy *x)
{
	return 0;
}

static void qca_dummy_ss_phy_shutdown(struct usb_phy *x)
{
}

static int qca_uni_ss_remove(struct platform_device *pdev)
{
	struct qca_uni_ss_phy *phy = platform_get_drvdata(pdev);

	usb_remove_phy(&phy->phy);
	return 0;
}

static const struct of_device_id qca_uni_ss_id_table[] = {
	{ .compatible = "qca,uni-ssphy" },
	{ .compatible = "qca,dummy-ssphy"},
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, qca_uni_ss_id_table);

static int qca_uni_ss_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device_node *np = pdev->dev.of_node;
	struct qca_uni_ss_phy  *phy;
	int ret;

	match = of_match_device(qca_uni_ss_id_table, &pdev->dev);
	if (!match)
		return -ENODEV;

	phy = devm_kzalloc(&pdev->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	platform_set_drvdata(pdev, phy);
	phy->dev = &pdev->dev;

	if (of_device_is_compatible(np, "qca,dummy-ssphy")) {
		phy->phy.dev        = phy->dev;
		phy->phy.label      = "qca-dummy-ssphy";
		phy->phy.init       = qca_dummy_ss_phy_init;
		phy->phy.shutdown   = qca_dummy_ss_phy_shutdown;
		phy->phy.type       = USB_PHY_TYPE_USB3;

		ret = usb_add_phy_dev(&phy->phy);
		return ret;
	}

	ret = qca_uni_ss_get_resources(pdev, phy);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request resources: %d\n", ret);
		return ret;
	}

	phy->phy.dev        = phy->dev;
	phy->phy.label      = "qca-uni-ssphy";
	phy->phy.init       = qca_uni_ss_phy_init;
	phy->phy.shutdown   = qca_uni_ss_phy_shutdown;
	phy->phy.type       = USB_PHY_TYPE_USB3;

	ret = usb_add_phy_dev(&phy->phy);
	return ret;
}

static struct platform_driver qca_uni_ss_driver = {
	.probe		= qca_uni_ss_probe,
	.remove		= qca_uni_ss_remove,
	.driver		= {
		.name	= "qca-uni-ssphy",
		.owner	= THIS_MODULE,
		.of_match_table = qca_uni_ss_id_table,
	},
};

module_platform_driver(qca_uni_ss_driver);

MODULE_ALIAS("platform:qca-uni-ssphy");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("USB3 QCA UNI SSPHY driver");
