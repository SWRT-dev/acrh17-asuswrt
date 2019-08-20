/* Copyright (c) 2011-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/mfd/core.h>
#include <linux/mfd/wcd9xxx/core.h>
#include <linux/mfd/wcd9xxx/wcd9xxx-irq.h>
#include <linux/mfd/wcd9xxx/pdata.h>
#include <linux/mfd/wcd9xxx/wcd9xxx_registers.h>
#include <linux/mfd/msm-cdc-pinctrl.h>
#include <linux/mfd/wcd9xxx/wcd9xxx-utils.h>
#include <linux/mfd/msm-cdc-supply.h>
#include <linux/mfd/wcd934x/registers.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include "wcd9xxx-regmap.h"

#define WCD9XXX_REGISTER_START_OFFSET 0x800
#define WCD9XXX_SLIM_RW_MAX_TRIES 3
#define SLIMBUS_PRESENT_TIMEOUT 100

#define MAX_WCD9XXX_DEVICE	4
#define WCD9XXX_I2C_GSBI_SLAVE_ID "3-000d"
#define WCD9XXX_I2C_TOP_SLAVE_ADDR	0x0d
#define WCD9XXX_ANALOG_I2C_SLAVE_ADDR	0x77
#define WCD9XXX_DIGITAL1_I2C_SLAVE_ADDR	0x66
#define WCD9XXX_DIGITAL2_I2C_SLAVE_ADDR	0x55
#define WCD9XXX_I2C_TOP_LEVEL	0
#define WCD9XXX_I2C_ANALOG	1
#define WCD9XXX_I2C_DIGITAL_1	2
#define WCD9XXX_I2C_DIGITAL_2	3

/*
 * Number of return values needs to be checked for each
 * registration of Slimbus of I2C bus for each codec
 */
#define NUM_WCD9XXX_REG_RET	5

#define SLIM_USR_MC_REPEAT_CHANGE_VALUE 0x0
#define SLIM_REPEAT_WRITE_MAX_SLICE 16
#define REG_BYTES 2
#define VAL_BYTES 1
#define WCD9XXX_PAGE_NUM(reg)    (((reg) >> 8) & 0xff)
#define WCD9XXX_PAGE_SIZE 256

struct wcd9xxx_i2c {
	struct i2c_client *client;
	struct i2c_msg xfer_msg[2];
	struct mutex xfer_lock;
	int mod_id;
};

static struct regmap_config wcd9xxx_base_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.can_multi_write = true,
};

static struct regmap_config wcd9xxx_i2c_base_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.can_multi_write = false,
	.use_single_rw = true,
};

static u8 wcd9xxx_pgd_la;
static u8 wcd9xxx_inf_la;

static const int wcd9xxx_cdc_types[] = {
	[WCD9XXX] = WCD9XXX,
	[WCD9330] = WCD9330,
	[WCD9335] = WCD9335,
	[WCD934X] = WCD934X,
};

static const struct of_device_id wcd9xxx_of_match[] = {
	{ .compatible = "qcom,tavil-i2c-pgd",
	  .data = (void *)&wcd9xxx_cdc_types[WCD934X]},
	{ .compatible = "qcom,tasha-i2c-pgd",
	  .data = (void *)&wcd9xxx_cdc_types[WCD9335]},
	{ .compatible = "qcom,wcd9xxx-i2c",
	  .data = (void *)&wcd9xxx_cdc_types[WCD9330]},
	{ }
};
MODULE_DEVICE_TABLE(of, wcd9xxx_of_match);

static void wcd9xxx_set_codec_specific_param(struct wcd9xxx *wcd9xxx);

struct wcd9xxx_i2c wcd9xxx_modules[MAX_WCD9XXX_DEVICE];

static int wcd9xxx_read(struct wcd9xxx *wcd9xxx, unsigned short reg,
		       int bytes, void *dest, bool interface_reg)
{
	int i, ret;

	if (bytes <= 0) {
		dev_err(wcd9xxx->dev, "Invalid byte read length %d\n", bytes);
		return -EINVAL;
	}

	ret = wcd9xxx->read_dev(wcd9xxx, reg, bytes, dest, interface_reg);
	if (ret < 0) {
		dev_err(wcd9xxx->dev, "Codec read failed\n");
		return ret;
	} else {
		for (i = 0; i < bytes; i++)
			dev_dbg(wcd9xxx->dev, "Read 0x%02x from 0x%x\n",
				((u8 *)dest)[i], reg + i);
	}

	return 0;
}

static int wcd9xxx_write(struct wcd9xxx *wcd9xxx, unsigned short reg,
			int bytes, void *src, bool interface_reg)
{
	int i;

	if (bytes <= 0) {
		pr_err("%s: Error, invalid write length\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < bytes; i++)
		dev_dbg(wcd9xxx->dev, "Write %02x to 0x%x\n", ((u8 *)src)[i],
			reg + i);

	return wcd9xxx->write_dev(wcd9xxx, reg, bytes, src, interface_reg);
}

static int __wcd9xxx_reg_update_bits(struct wcd9xxx *wcd9xxx,
				     unsigned short reg, u8 mask, u8 val)
{
	int ret;
	u8 orig, tmp;

	if (wcd9xxx->using_regmap)
		ret = regmap_update_bits(wcd9xxx->regmap, reg, mask, val);
	else {
		mutex_lock(&wcd9xxx->io_lock);
		ret = wcd9xxx_read(wcd9xxx, reg, 1, &orig, false);
		if (ret < 0) {
			dev_err(wcd9xxx->dev, "%s: Codec read 0x%x failed\n",
				__func__, reg);
			goto err;
		}
		tmp = orig & ~mask;
		tmp |= val & mask;
		if (tmp != orig)
			ret = wcd9xxx_write(wcd9xxx, reg, 1, &tmp, false);
err:
		mutex_unlock(&wcd9xxx->io_lock);
	}
	return ret;
}

int wcd9xxx_reg_update_bits(
	struct wcd9xxx_core_resource *core_res,
	unsigned short reg, u8 mask, u8 val)
{
	struct wcd9xxx *wcd9xxx = (struct wcd9xxx *) core_res->parent;
	return __wcd9xxx_reg_update_bits(wcd9xxx, reg, mask, val);
}
EXPORT_SYMBOL(wcd9xxx_reg_update_bits);

static int __wcd9xxx_bulk_read(
	struct wcd9xxx *wcd9xxx,
	unsigned short reg,
	int count, u8 *buf)
{
	int ret;

	if (wcd9xxx->using_regmap) {
		ret = regmap_bulk_read(wcd9xxx->regmap, reg, buf, count);
	} else {
		mutex_lock(&wcd9xxx->io_lock);
		ret = wcd9xxx_read(wcd9xxx, reg, count, buf, false);
		mutex_unlock(&wcd9xxx->io_lock);
	}

	return ret;
}

int wcd9xxx_bulk_read(
	struct wcd9xxx_core_resource *core_res,
	unsigned short reg,
	int count, u8 *buf)
{
	struct wcd9xxx *wcd9xxx =
			(struct wcd9xxx *) core_res->parent;
	return __wcd9xxx_bulk_read(wcd9xxx, reg, count, buf);
}
EXPORT_SYMBOL(wcd9xxx_bulk_read);

static int __wcd9xxx_bulk_write(struct wcd9xxx *wcd9xxx, unsigned short reg,
		     int count, u8 *buf)
{
	int ret;

	if (wcd9xxx->using_regmap) {
		ret = regmap_bulk_write(wcd9xxx->regmap, reg, buf, count);
	} else {
		mutex_lock(&wcd9xxx->io_lock);
		ret = wcd9xxx_write(wcd9xxx, reg, count, buf, false);
		mutex_unlock(&wcd9xxx->io_lock);
	}

	return ret;
}

int wcd9xxx_bulk_write(
	struct wcd9xxx_core_resource *core_res,
	unsigned short reg, int count, u8 *buf)
{
	struct wcd9xxx *wcd9xxx =
			(struct wcd9xxx *) core_res->parent;
	return __wcd9xxx_bulk_write(wcd9xxx, reg, count, buf);
}
EXPORT_SYMBOL(wcd9xxx_bulk_write);

/*
 * wcd9xxx_interface_reg_read: Read slim interface registers
 *
 * @wcd9xxx: Pointer to wcd9xxx structure
 * @reg: register adderss
 *
 * Returns register value in success and negative error code in case of failure
 */
int wcd9xxx_interface_reg_read(struct wcd9xxx *wcd9xxx, unsigned short reg)
{
	u8 val;
	int ret;

	mutex_lock(&wcd9xxx->io_lock);
	ret = wcd9xxx->read_dev(wcd9xxx, reg, 1, (void *)&val,
				true);
	if (ret < 0)
		dev_err(wcd9xxx->dev, "%s: Codec read 0x%x failed\n",
			__func__, reg);
	else
		dev_dbg(wcd9xxx->dev, "%s: Read 0x%02x from 0x%x\n",
			__func__, val, reg);

	mutex_unlock(&wcd9xxx->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL(wcd9xxx_interface_reg_read);

/*
 * wcd9xxx_interface_reg_write: Write slim interface registers
 *
 * @wcd9xxx: Pointer to wcd9xxx structure
 * @reg: register adderss
 * @val: value of the register to be written
 *
 * Returns 0 for success and negative error code in case of failure
 */
int wcd9xxx_interface_reg_write(struct wcd9xxx *wcd9xxx, unsigned short reg,
		     u8 val)
{
	int ret;

	mutex_lock(&wcd9xxx->io_lock);
	ret = wcd9xxx->write_dev(wcd9xxx, reg, 1, (void *)&val, true);
	dev_dbg(wcd9xxx->dev, "%s: Write %02x to 0x%x ret(%d)\n",
		__func__, val, reg, ret);
	mutex_unlock(&wcd9xxx->io_lock);

	return ret;
}
EXPORT_SYMBOL(wcd9xxx_interface_reg_write);

int wcd9xxx_swrm_i2s_bulk_write(struct wcd9xxx *wcd9xxx,
				struct wcd9xxx_reg_val *bulk_reg,
				size_t len)
{
	int i, ret = 0;
	unsigned short swr_wr_addr_base;
	unsigned short swr_wr_data_base;

	swr_wr_addr_base = WCD934X_SWR_AHB_BRIDGE_WR_ADDR_0;
	swr_wr_data_base = WCD934X_SWR_AHB_BRIDGE_WR_DATA_0;

	for (i = 0; i < (len * 2); i += 2) {
		/* First Write the Data to register */
		ret = wcd9xxx_bulk_write(&wcd9xxx->core_res,
			swr_wr_data_base, 4, bulk_reg[i].buf);
		if (ret < 0) {
			dev_err(wcd9xxx->dev, "%s: WR Data Failure\n",
				__func__);
			break;
		}
		/* Next Write Address */
		ret = wcd9xxx_bulk_write(&wcd9xxx->core_res,
			swr_wr_addr_base, 4, bulk_reg[i+1].buf);
		if (ret < 0) {
			dev_err(wcd9xxx->dev, "%s: WR Addr Failure\n",
				__func__);
			break;
		}
	}
	return ret;
}
EXPORT_SYMBOL(wcd9xxx_swrm_i2s_bulk_write);

static int wcd9xxx_num_irq_regs(const struct wcd9xxx *wcd9xxx)
{
	return (wcd9xxx->codec_type->num_irqs / 8) +
		((wcd9xxx->codec_type->num_irqs % 8) ? 1 : 0);
}

static int wcd9xxx_regmap_init_cache(struct wcd9xxx *wcd9xxx)
{
	struct regmap_config *regmap_config;
	int rc;

	regmap_config = wcd9xxx_get_regmap_config(wcd9xxx->type);
	if (!regmap_config) {
		dev_err(wcd9xxx->dev, "regmap config is not defined\n");
		return -EINVAL;
	}

	rc = regmap_reinit_cache(wcd9xxx->regmap, regmap_config);
	if (rc != 0) {
		dev_err(wcd9xxx->dev, "%s:Failed to reinit register cache: %d\n",
			__func__, rc);
	}

	return rc;
}

static int wcd9xxx_device_init(struct wcd9xxx *wcd9xxx)
{
	int ret = 0, i;
	struct wcd9xxx_core_resource *core_res = &wcd9xxx->core_res;
	regmap_patch_fptr regmap_apply_patch = NULL;

	mutex_init(&wcd9xxx->io_lock);
	mutex_init(&wcd9xxx->xfer_lock);
	mutex_init(&wcd9xxx->reset_lock);

	ret = wcd9xxx_bringup(wcd9xxx->dev);
	if (ret) {
		ret = -EPROBE_DEFER;
		goto err_bring_up;
	}

	wcd9xxx->codec_type = devm_kzalloc(wcd9xxx->dev,
			sizeof(struct wcd9xxx_codec_type), GFP_KERNEL);
	if (!wcd9xxx->codec_type) {
		ret = -ENOMEM;
		goto err_bring_up;
	}
	ret = wcd9xxx_get_codec_info(wcd9xxx->dev);
	if (ret) {
		ret = -EPROBE_DEFER;
		goto fail_cdc_fill;
	}
	wcd9xxx->version = wcd9xxx->codec_type->version;
	if (!wcd9xxx->codec_type->dev || !wcd9xxx->codec_type->size)
		goto fail_cdc_fill;

	core_res->parent = wcd9xxx;
	core_res->dev = wcd9xxx->dev;
	core_res->intr_table = wcd9xxx->codec_type->intr_tbl;
	core_res->intr_table_size = wcd9xxx->codec_type->intr_tbl_size;

	for (i = 0; i < WCD9XXX_INTR_REG_MAX; i++)
		wcd9xxx->core_res.intr_reg[i] =
			wcd9xxx->codec_type->intr_reg[i];

	wcd9xxx_core_res_init(&wcd9xxx->core_res,
			      wcd9xxx->codec_type->num_irqs,
			      wcd9xxx_num_irq_regs(wcd9xxx),
			      wcd9xxx->regmap);

	if (wcd9xxx_core_irq_init(&wcd9xxx->core_res))
		goto err;

	ret = wcd9xxx_regmap_init_cache(wcd9xxx);
	if (ret)
		goto err_irq;

	regmap_apply_patch = wcd9xxx_get_regmap_reg_patch(
			wcd9xxx->type);
	if (regmap_apply_patch) {
		ret = regmap_apply_patch(wcd9xxx->regmap,
				wcd9xxx->version);
		if (ret)
			dev_err(wcd9xxx->dev,
					"Failed to register patch: %d\n", ret);
	}

	ret = mfd_add_devices(wcd9xxx->dev, -1, wcd9xxx->codec_type->dev,
			      wcd9xxx->codec_type->size, NULL, 0, NULL);
	if (ret != 0) {
		dev_err(wcd9xxx->dev, "Failed to add children: %d\n", ret);
		goto err_irq;
	}

	ret = device_init_wakeup(wcd9xxx->dev, true);
	if (ret) {
		dev_err(wcd9xxx->dev, "Device wakeup init failed: %d\n", ret);
		goto err_irq;
	}

	return ret;
err_irq:
	wcd9xxx_irq_exit(&wcd9xxx->core_res);
fail_cdc_fill:
	devm_kfree(wcd9xxx->dev, wcd9xxx->codec_type);
	wcd9xxx->codec_type = NULL;
err:
	wcd9xxx_bringdown(wcd9xxx->dev);
	wcd9xxx_core_res_deinit(&wcd9xxx->core_res);
err_bring_up:
	mutex_destroy(&wcd9xxx->io_lock);
	mutex_destroy(&wcd9xxx->xfer_lock);
	mutex_destroy(&wcd9xxx->reset_lock);
	return ret;
}

static void wcd9xxx_device_exit(struct wcd9xxx *wcd9xxx)
{
	device_init_wakeup(wcd9xxx->dev, false);
	wcd9xxx_irq_exit(&wcd9xxx->core_res);
	wcd9xxx_bringdown(wcd9xxx->dev);
	wcd9xxx_reset_low(wcd9xxx->dev);
	wcd9xxx_core_res_deinit(&wcd9xxx->core_res);
	mutex_destroy(&wcd9xxx->io_lock);
	mutex_destroy(&wcd9xxx->xfer_lock);
	mutex_destroy(&wcd9xxx->reset_lock);
}


#ifdef CONFIG_DEBUG_FS
struct wcd9xxx *debugCodec;

static struct dentry *debugfs_wcd9xxx_dent;
static struct dentry *debugfs_peek;
static struct dentry *debugfs_poke;
static struct dentry *debugfs_power_state;
static struct dentry *debugfs_reg_dump;

static unsigned char read_data;

static int codec_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int get_parameters(char *buf, long int *param1, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");

	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token != NULL) {
			if ((token[1] == 'x') || (token[1] == 'X'))
				base = 16;
			else
				base = 10;

			if (kstrtoul(token, base, &param1[cnt]) != 0)
				return -EINVAL;

			token = strsep(&buf, " ");
		} else
			return -EINVAL;
	}
	return 0;
}

static ssize_t codec_debug_read(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char lbuf[8];
	char *access_str = file->private_data;
	ssize_t ret_cnt = 0;

	if (*ppos < 0 || !count)
		return -EINVAL;

	return ret_cnt;
}

static void wcd9xxx_set_reset_pin_state(struct wcd9xxx *wcd9xxx,
					struct wcd9xxx_pdata *pdata,
					bool active)
{
	if (wcd9xxx->wcd_rst_np) {
		if (active)
			msm_cdc_pinctrl_select_active_state(
						wcd9xxx->wcd_rst_np);
		else
			msm_cdc_pinctrl_select_sleep_state(
						wcd9xxx->wcd_rst_np);

		return;
	} else if (gpio_is_valid(wcd9xxx->reset_gpio)) {
		gpio_direction_output(wcd9xxx->reset_gpio,
				      (active == true ? 1 : 0));
	}
}

static ssize_t codec_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *access_str = filp->private_data;
	char lbuf[32];
	int rc = 0;
	long int param[5];

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';

	if (rc == 0)
		rc = cnt;
	else
		pr_err("%s: rc = %d\n", __func__, rc);

	return rc;
}

static const struct file_operations codec_debug_ops = {
	.open = codec_debug_open,
	.write = codec_debug_write,
	.read = codec_debug_read
};
#endif

static struct wcd9xxx_i2c *wcd9xxx_i2c_get_device_info(struct wcd9xxx *wcd9xxx,
						u16 reg)
{
	u16 mask = 0x0f00;
	int value = 0;
	struct wcd9xxx_i2c *wcd9xxx_i2c = NULL;

	if (wcd9xxx->type == WCD9335) {
		wcd9xxx_i2c = &wcd9xxx_modules[0];
	} else {
		value = ((reg & mask) >> 8) & 0x000f;
		switch (value) {
		case 0:
			wcd9xxx_i2c = &wcd9xxx_modules[0];
			break;
		case 1:
			wcd9xxx_i2c = &wcd9xxx_modules[1];
			break;
		case 2:
			wcd9xxx_i2c = &wcd9xxx_modules[2];
			break;
		case 3:
			wcd9xxx_i2c = &wcd9xxx_modules[3];
			break;

		default:
			break;
		}
	}
	return wcd9xxx_i2c;
}

static int wcd9xxx_i2c_write_device(struct wcd9xxx *wcd9xxx, u16 reg, u8 *value,
				u32 bytes)
{

	struct i2c_msg *msg;
	int ret = 0;
	u8 reg_addr = 0;
	u8 data[bytes + 1];
	struct wcd9xxx_i2c *wcd9xxx_i2c;

	wcd9xxx_i2c = wcd9xxx_i2c_get_device_info(wcd9xxx, reg);
	if (wcd9xxx_i2c == NULL || wcd9xxx_i2c->client == NULL) {
		pr_err("failed to get device info\n");
		return -ENODEV;
	}
	reg_addr = (u8)reg;
	msg = &wcd9xxx_i2c->xfer_msg[0];
	msg->addr = wcd9xxx_i2c->client->addr;
	msg->len = bytes + 1;
	msg->flags = 0;
	data[0] = reg;
	data[1] = *value;
	msg->buf = data;
	ret = i2c_transfer(wcd9xxx_i2c->client->adapter,
			   wcd9xxx_i2c->xfer_msg, 1);
	/* Try again if the write fails */
	if (ret != 1) {
		ret = i2c_transfer(wcd9xxx_i2c->client->adapter,
						wcd9xxx_i2c->xfer_msg, 1);
		if (ret != 1) {
			pr_err("failed to write the device\n");
			return ret;
		}
	}
	pr_debug("write sucess register = %x val = %x\n", reg, data[1]);
	return 0;
}


static int wcd9xxx_i2c_read_device(struct wcd9xxx *wcd9xxx, unsigned short reg,
				  int bytes, unsigned char *dest)
{
	struct i2c_msg *msg;
	int ret = 0;
	u8 reg_addr = 0;
	struct wcd9xxx_i2c *wcd9xxx_i2c;
	u8 i = 0;

	wcd9xxx_i2c = wcd9xxx_i2c_get_device_info(wcd9xxx, reg);
	if (wcd9xxx_i2c == NULL || wcd9xxx_i2c->client == NULL) {
		pr_err("failed to get device info\n");
		return -ENODEV;
	}
	for (i = 0; i < bytes; i++) {
		reg_addr = (u8)reg++;
		msg = &wcd9xxx_i2c->xfer_msg[0];
		msg->addr = wcd9xxx_i2c->client->addr;
		msg->len = 1;
		msg->flags = 0;
		msg->buf = &reg_addr;

		msg = &wcd9xxx_i2c->xfer_msg[1];
		msg->addr = wcd9xxx_i2c->client->addr;
		msg->len = 1;
		msg->flags = I2C_M_RD;
		msg->buf = dest++;
		ret = i2c_transfer(wcd9xxx_i2c->client->adapter,
				wcd9xxx_i2c->xfer_msg, 2);

		/* Try again if read fails first time */
		if (ret != 2) {
			ret = i2c_transfer(wcd9xxx_i2c->client->adapter,
					   wcd9xxx_i2c->xfer_msg, 2);
			if (ret != 2) {
				pr_err("failed to read wcd9xxx register\n");
				return ret;
			}
		}
	}
	return 0;
}

int wcd9xxx_i2c_read(struct wcd9xxx *wcd9xxx, unsigned short reg,
			int bytes, void *dest, bool interface_reg)
{
	return wcd9xxx_i2c_read_device(wcd9xxx, reg, bytes, dest);
}

int wcd9xxx_i2c_write(struct wcd9xxx *wcd9xxx, unsigned short reg,
			 int bytes, void *src, bool interface_reg)
{
	return wcd9xxx_i2c_write_device(wcd9xxx, reg, src, bytes);
}

static int wcd9xxx_i2c_get_client_index(struct i2c_client *client,
					int *wcd9xx_index)
{
	int ret = 0;

	switch (client->addr) {
	case WCD9XXX_I2C_TOP_SLAVE_ADDR:
		*wcd9xx_index = WCD9XXX_I2C_TOP_LEVEL;
	break;
	case WCD9XXX_ANALOG_I2C_SLAVE_ADDR:
		*wcd9xx_index = WCD9XXX_I2C_ANALOG;
	break;
	case WCD9XXX_DIGITAL1_I2C_SLAVE_ADDR:
		*wcd9xx_index = WCD9XXX_I2C_DIGITAL_1;
	break;
	case WCD9XXX_DIGITAL2_I2C_SLAVE_ADDR:
		*wcd9xx_index = WCD9XXX_I2C_DIGITAL_2;
	break;
	default:
		ret = -EINVAL;
	break;
	}
	return ret;
}

static int wcd9xxx_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct wcd9xxx *wcd9xxx = NULL;
	struct wcd9xxx_pdata *pdata = NULL;
	int val = 0;
	int ret = 0;
	int wcd9xx_index = 0;
	struct device *dev;
	int intf_type;
	const struct of_device_id *of_id;

	intf_type = wcd9xxx_get_intf_type();

	pr_debug("%s: interface status %d\n", __func__, intf_type);
	if (intf_type == WCD9XXX_INTERFACE_TYPE_SLIMBUS) {
		dev_dbg(&client->dev, "%s:Codec is detected in slimbus mode\n",
			__func__);
		return -ENODEV;
	} else if (intf_type == WCD9XXX_INTERFACE_TYPE_I2C) {
		ret = wcd9xxx_i2c_get_client_index(client, &wcd9xx_index);
		if (ret != 0)
			dev_err(&client->dev, "%s: I2C set codec I2C\n"
				"client failed\n", __func__);
		else {
			dev_err(&client->dev, "%s:probe for other slaves\n"
				"devices of codec I2C slave Addr = %x\n",
				__func__, client->addr);
			wcd9xxx_modules[wcd9xx_index].client = client;
		}
		return ret;
	} else if (intf_type == WCD9XXX_INTERFACE_TYPE_PROBING) {
		dev = &client->dev;
		if (client->dev.of_node) {
			dev_dbg(&client->dev, "%s:Platform data\n"
				"from device tree\n", __func__);
			pdata = wcd9xxx_populate_dt_data(&client->dev);
			if (!pdata) {
				dev_err(&client->dev,
					"%s: Fail to obtain pdata from device tree\n",
					 __func__);
				ret = -EINVAL;
				goto fail;
			}
			client->dev.platform_data = pdata;
		} else {
			dev_dbg(&client->dev, "%s:Platform data from\n"
				"board file\n", __func__);
			pdata = client->dev.platform_data;
		}
		wcd9xxx = devm_kzalloc(&client->dev, sizeof(struct wcd9xxx),
				       GFP_KERNEL);
		if (!wcd9xxx) {
			ret = -ENOMEM;
			goto fail;
		}

		if (!pdata) {
			dev_dbg(&client->dev, "no platform data?\n");
			ret = -EINVAL;
			goto fail;
		}
		wcd9xxx->type = WCD9XXX;
		if (client->dev.of_node) {
			of_id = of_match_device(wcd9xxx_of_match, &client->dev);
			if (of_id) {
				wcd9xxx->type = *((int *)of_id->data);
				dev_info(&client->dev, "%s: codec type is %d\n",
					 __func__, wcd9xxx->type);
			}
		} else {
			dev_info(&client->dev, "%s: dev.of_node is NULL, default to WCD9XXX\n",
				 __func__);
			wcd9xxx->type = WCD9XXX;
		}
		wcd9xxx_set_codec_specific_param(wcd9xxx);
		wcd9xxx->regmap = wcd9xxx_regmap_init(&client->dev,
				&wcd9xxx_i2c_base_regmap_config);
		if (IS_ERR(wcd9xxx->regmap)) {
			ret = PTR_ERR(wcd9xxx->regmap);
			dev_err(&client->dev, "%s: Failed to allocate register map: %d\n",
					__func__, ret);
			goto err_codec;
		}
		wcd9xxx->reset_gpio = pdata->reset_gpio;
		wcd9xxx->wcd_rst_np = pdata->wcd_rst_np;

		if (!wcd9xxx->wcd_rst_np) {
			pdata->use_pinctrl = false;
			dev_err(&client->dev, "%s: pinctrl not used for rst_n\n",
				 __func__);
			goto err_codec;
		}

		if (i2c_check_functionality(client->adapter,
					    I2C_FUNC_I2C) == 0) {
			dev_dbg(&client->dev, "can't talk I2C?\n");
			ret = -EIO;
			goto fail;
		}
		dev_set_drvdata(&client->dev, wcd9xxx);
		wcd9xxx->dev = &client->dev;
		wcd9xxx->dev_up = true;
		if (client->dev.of_node)
			wcd9xxx->mclk_rate = pdata->mclk_rate;

		wcd9xxx->num_of_supplies = pdata->num_supplies;
		pdata->regulator = NULL;
		pdata->num_supplies = 0;
		if (wcd9xxx->num_of_supplies) {
			ret = msm_cdc_init_supplies(wcd9xxx->dev,
						&wcd9xxx->supplies,
						pdata->regulator,
						pdata->num_supplies);

			if (!wcd9xxx->supplies) {
				dev_err(wcd9xxx->dev,
					"%s: Cannot init wcd supplies\n",
					__func__);
				goto err_codec;
			}
			ret = msm_cdc_enable_static_supplies(wcd9xxx->dev,
							 wcd9xxx->supplies,
							 pdata->regulator,
							 pdata->num_supplies);
			if (ret) {
				dev_err(wcd9xxx->dev,
					"%s: wcd static supply enable failed\n",
					__func__);
				goto err_codec;
			}
			/* For WCD9335, it takes about 600us for the Vout_A and
			 * Vout_D to be ready after BUCK_SIDO is powered up \
			 * SYS_RST_N shouldn't be pulled high during this time
			 */
			if (wcd9xxx->type == WCD9335)
				usleep_range(600, 650);
			else
				usleep_range(5, 10);
		}

		ret = wcd9xxx_reset(wcd9xxx->dev);
		if (ret) {
			pr_err("%s: Resetting Codec failed\n", __func__);
			goto err_supplies;
		}

		ret = wcd9xxx_i2c_get_client_index(client, &wcd9xx_index);
		if (ret != 0) {
			pr_err("%s:Set codec I2C client failed\n", __func__);
			goto err_supplies;
		}

		wcd9xxx_modules[wcd9xx_index].client = client;
		wcd9xxx->read_dev = wcd9xxx_i2c_read;
		wcd9xxx->write_dev = wcd9xxx_i2c_write;
		if (!wcd9xxx->dev->of_node)
			wcd9xxx_assign_irq(&wcd9xxx->core_res,
					pdata->irq, pdata->irq_base);

		ret = wcd9xxx_device_init(wcd9xxx);
		if (ret) {
			pr_err("%s: error, initializing device failed (%d)\n",
			       __func__, ret);
			goto err_device_init;
		}

		ret = wcd9xxx_i2c_read(wcd9xxx, WCD9XXX_A_CHIP_STATUS, 1,
				       &val, 0);
		if (ret < 0)
			pr_err("%s: failed to read the wcd9xxx status (%d)\n",
			       __func__, ret);
		if (val != wcd9xxx->codec_type->i2c_chip_status)
			pr_err("%s: unknown chip status 0x%x\n", __func__, val);

		wcd9xxx_set_intf_type(WCD9XXX_INTERFACE_TYPE_I2C);

		return ret;
	} else
		pr_err("%s: I2C probe in wrong state\n", __func__);

err_device_init:
	wcd9xxx_reset_low(wcd9xxx->dev);
err_supplies:
	if (wcd9xxx->num_of_supplies) {
		msm_cdc_release_supplies(wcd9xxx->dev, wcd9xxx->supplies,
				pdata->regulator, pdata->num_supplies);
	}
	pdata->regulator = NULL;
	pdata->num_supplies = 0;
err_codec:
	devm_kfree(&client->dev, wcd9xxx);
	dev_set_drvdata(&client->dev, NULL);
fail:
	return ret;
}

static int wcd9xxx_i2c_remove(struct i2c_client *client)
{
	struct wcd9xxx *wcd9xxx;
	struct wcd9xxx_pdata *pdata = client->dev.platform_data;

	wcd9xxx = dev_get_drvdata(&client->dev);
	msm_cdc_release_supplies(wcd9xxx->dev, wcd9xxx->supplies,
				 pdata->regulator,
				 pdata->num_supplies);
	wcd9xxx_device_exit(wcd9xxx);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static void wcd9xxx_set_codec_specific_param(struct wcd9xxx *wcd9xxx)
{
	if (!wcd9xxx) {
		pr_err("%s: wcd9xxx is NULL\n", __func__);
		return;
	}

	switch (wcd9xxx->type) {
	case WCD934X:
	case WCD9335:
	case WCD9330:
		wcd9xxx->using_regmap = true;
		wcd9xxx->prev_pg_valid = false;
		break;
	default:
		wcd9xxx->using_regmap = false;
		break;
	}
	pr_debug("%s: Codec %s regmap\n",
		__func__, (wcd9xxx->using_regmap ? "using" : "not using"));

	return;
}

static int wcd9xxx_device_up(struct wcd9xxx *wcd9xxx)
{
	int ret = 0;
	struct wcd9xxx_core_resource *wcd9xxx_res = &wcd9xxx->core_res;

	dev_info(wcd9xxx->dev, "%s: codec bring up\n", __func__);
	wcd9xxx_bringup(wcd9xxx->dev);
	ret = wcd9xxx_irq_init(wcd9xxx_res);
	if (ret) {
		pr_err("%s: wcd9xx_irq_init failed : %d\n", __func__, ret);
	} else {
		if (wcd9xxx->post_reset)
			ret = wcd9xxx->post_reset(wcd9xxx);
	}
	return ret;
}

static int wcd9xxx_i2c_resume(struct device *dev)
{
	struct wcd9xxx *wcd9xxx = dev_get_drvdata(dev);

	if (wcd9xxx)
		return wcd9xxx_core_res_resume(&wcd9xxx->core_res);
	else
		return 0;
}

static int __wcd9xxx_reg_read(
	struct wcd9xxx *wcd9xxx,
	unsigned short reg)
{
	unsigned int val = 0;
	int ret;

	if (wcd9xxx->using_regmap) {
		ret = regmap_read(wcd9xxx->regmap, reg, &val);
	} else {
		mutex_lock(&wcd9xxx->io_lock);
		ret = wcd9xxx_read(wcd9xxx, reg, 1, &val, false);
		mutex_unlock(&wcd9xxx->io_lock);
	}

	if (ret < 0)
		return ret;
	else
		return val;
}

int wcd9xxx_reg_read(
	struct snd_soc_codec *codec,
	unsigned short reg)
{
	struct wcd9xxx *wcd9xxx = dev_get_drvdata(codec->dev->parent);
	return __wcd9xxx_reg_read(wcd9xxx, reg);
}
EXPORT_SYMBOL(wcd9xxx_reg_read);

static int __wcd9xxx_reg_write(
	struct wcd9xxx *wcd9xxx,
	unsigned short reg, u8 val)
{
	int ret;

	if (wcd9xxx->using_regmap)
		ret = regmap_write(wcd9xxx->regmap, reg, val);
	else {
		mutex_lock(&wcd9xxx->io_lock);
		ret = wcd9xxx_write(wcd9xxx, reg, 1, &val, false);
		mutex_unlock(&wcd9xxx->io_lock);
	}

	return ret;
}

int wcd9xxx_reg_write(
	struct snd_soc_codec *codec,
	unsigned short reg, u8 val)
{
	struct wcd9xxx *wcd9xxx = dev_get_drvdata(codec->dev->parent);
	return __wcd9xxx_reg_write(wcd9xxx, reg, val);
}
EXPORT_SYMBOL(wcd9xxx_reg_write);

static int wcd9xxx_i2c_suspend(struct device *dev)
{
	struct wcd9xxx *wcd9xxx = dev_get_drvdata(dev);
	pm_message_t pmesg = {0};

	if (wcd9xxx)
		return wcd9xxx_core_res_suspend(&wcd9xxx->core_res, pmesg);
	else
		return 0;
}

static struct i2c_device_id wcd9xxx_id_table[] = {
	{"wcd9xxx-i2c", WCD9XXX_I2C_TOP_LEVEL},
	{"wcd9xxx-i2c", WCD9XXX_I2C_ANALOG},
	{"wcd9xxx-i2c", WCD9XXX_I2C_DIGITAL_1},
	{"wcd9xxx-i2c", WCD9XXX_I2C_DIGITAL_2},
	{}
};

static struct i2c_device_id tasha_id_table[] = {
	{"tasha-i2c-pgd", WCD9XXX_I2C_TOP_LEVEL},
	{}
};

static struct i2c_device_id tavil_id_table[] = {
	{"tavil-i2c-pgd", WCD9XXX_I2C_TOP_LEVEL},
	{}
};

static struct i2c_device_id tabla_id_table[] = {
	{"tabla top level", WCD9XXX_I2C_TOP_LEVEL},
	{"tabla analog", WCD9XXX_I2C_ANALOG},
	{"tabla digital1", WCD9XXX_I2C_DIGITAL_1},
	{"tabla digital2", WCD9XXX_I2C_DIGITAL_2},
	{}
};
MODULE_DEVICE_TABLE(i2c, tabla_id_table);

static const struct dev_pm_ops wcd9xxx_i2c_pm_ops = {
	.suspend = wcd9xxx_i2c_suspend,
	.resume	= wcd9xxx_i2c_resume,
};

static struct i2c_driver tabla_i2c_driver = {
	.driver                 = {
		.owner          =       THIS_MODULE,
		.name           =       "tabla-i2c-core",
		.pm		=	&wcd9xxx_i2c_pm_ops,
	},
	.id_table               =       tabla_id_table,
	.probe                  =       wcd9xxx_i2c_probe,
	.remove                 =       wcd9xxx_i2c_remove,
};

static struct i2c_driver wcd9xxx_i2c_driver = {
	.driver                 = {
		.owner          =       THIS_MODULE,
		.name           =       "wcd9xxx-i2c-core",
		.pm		=	&wcd9xxx_i2c_pm_ops,
	},
	.id_table               =       wcd9xxx_id_table,
	.probe                  =       wcd9xxx_i2c_probe,
	.remove                 =       wcd9xxx_i2c_remove,
};

static struct i2c_driver wcd9335_i2c_driver = {
	.driver	                = {
		.owner	        =       THIS_MODULE,
		.name           =       "tasha-i2c-core",
		.pm		=	&wcd9xxx_i2c_pm_ops,
	},
	.id_table               =       tasha_id_table,
	.probe                  =       wcd9xxx_i2c_probe,
	.remove                 =       wcd9xxx_i2c_remove,
};

static struct i2c_driver wcd934x_i2c_driver = {
	.driver	                = {
		.owner	        =       THIS_MODULE,
		.name           =       "tavil-i2c-core",
		.pm		=	&wcd9xxx_i2c_pm_ops,
	},
	.id_table               =       tavil_id_table,
	.probe                  =       wcd9xxx_i2c_probe,
	.remove                 =       wcd9xxx_i2c_remove,
};

static int __init wcd9xxx_init(void)
{
	int ret[NUM_WCD9XXX_REG_RET] = {0};
	int i = 0;

	wcd9xxx_set_intf_type(WCD9XXX_INTERFACE_TYPE_PROBING);

	ret[0] = i2c_add_driver(&tabla_i2c_driver);
	if (ret[0])
		pr_err("%s: Failed to add the tabla2x I2C driver: %d\n",
			__func__, ret[0]);

	ret[1] = i2c_add_driver(&wcd9xxx_i2c_driver);
	if (ret[1])
		pr_err("%s: Failed to add the wcd9xxx I2C driver: %d\n",
			__func__, ret[1]);

	ret[2] = i2c_add_driver(&wcd9335_i2c_driver);
	if (ret[2])
		pr_err("%s: Failed to add the wcd9335 I2C driver: %d\n",
			__func__, ret[2]);

	ret[3] = i2c_add_driver(&wcd934x_i2c_driver);
	if (ret[3])
		pr_err("%s: Failed to add the wcd934x I2C driver: %d\n",
			__func__, ret[3]);

	for (i = 0; i < NUM_WCD9XXX_REG_RET; i++) {
		if (ret[i])
			return ret[i];
	}

	return 0;
}
module_init(wcd9xxx_init);

static void __exit wcd9xxx_exit(void)
{
	wcd9xxx_set_intf_type(WCD9XXX_INTERFACE_TYPE_PROBING);

	i2c_del_driver(&tabla_i2c_driver);
	i2c_del_driver(&wcd9xxx_i2c_driver);
	i2c_del_driver(&wcd9335_i2c_driver);
}
module_exit(wcd9xxx_exit);

MODULE_DESCRIPTION("Codec core driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
