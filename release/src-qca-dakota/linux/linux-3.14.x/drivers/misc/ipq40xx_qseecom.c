/* Qualcomm Technologies, Inc Secure Execution Environment Communicator (QSEECOM) driver
 *
 * Copyright (c) 2012, 2015-2016 The Linux Foundation. All rights reserved.
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

/* Usage:
 *
 *(1) Step 1: To provide the sampleapp files to the kernel module
 *
 * cat /lib/firmware/sampleapp.mdt > /sys/firmware/mdt_file
 * cat /lib/firmware/sampleapp.b00 > /sys/firmware/seg0_file
 * cat /lib/firmware/sampleapp.b01 > /sys/firmware/seg1_file
 * cat /lib/firmware/sampleapp.b02 > /sys/firmware/seg2_file
 * cat /lib/firmware/sampleapp.b03 > /sys/firmware/seg3_file
 *
 *(2) Step 2: To start loading the sampleapp
 *
 * echo 1 > /sys/firmware/tzapp/load_start
 *
 *(3) Step 3:
 *
 * To test Crypto functionality:
 * echo 1 > /sys/firmware/tzapp/crypto
 *
 * To give input to Multiplication op:
 * echo 100 > /sys/firmware/tzapp/basic_data
 *
 * To view Secure Multiplication output:
 * cat /sys/firmware/tzapp/basic_data
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/highuid.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include "soc/qcom/scm.h"
#include <linux/gfp.h>
#include <linux/sysfs.h>
#include <linux/dma-mapping.h>

#define CLIENT_CMD1_BASIC_DATA	1
#define CLIENT_CMD8_RUN_CRYPTO_TEST	3
#define CLIENT_CMD_AUTH	26
#define MAX_APP_NAME_SIZE	32
#define MAX_INPUT_SIZE	4096
#define SCM_SVC_TZSCHEDULER	0xFC
#define QSEE_APP_NOTIFY_COMMAND	13
#define TZAPP_START_ADDRESS	0x87b80000
#define TZAPP_SIZE	0x27FFFF

struct qsee_notify_app {
	uint32_t cmd_id;
	uint32_t applications_region_addr;
	uint32_t applications_region_size;
};


struct qsc_send_cmd {
	uint32_t cmd_id;
	uint32_t data;
	uint32_t data2;
	uint32_t len;
	uint32_t start_pkt;
	uint32_t end_pkt;
	uint32_t test_buf_size;
};

struct qsc_send_cmd_rsp {
	uint32_t data;
	int32_t status;
};

__packed struct qseecom_unload_app_ireq {
	uint32_t qsee_cmd_id;
	uint32_t  app_id;
};

enum qseecom_command_scm_resp_type {
	QSEOS_APP_ID = 0xEE01,
	QSEOS_LISTENER_ID
};

__packed struct qseecom_command_scm_resp {
	uint32_t result;
	enum qseecom_command_scm_resp_type resp_type;
	unsigned int data;
};

__packed struct qseecom_client_send_data_ireq {
	uint32_t qsee_cmd_id;
	uint32_t app_id;
	dma_addr_t *req_ptr;
	uint32_t req_len;
	dma_addr_t rsp_ptr;	/* First 4 bytes should be the return status */
	uint32_t rsp_len;
};

__packed struct qseecom_load_app_ireq {
	uint32_t qsee_cmd_id;
	uint32_t mdt_len;		/* Length of the mdt file */
	uint32_t img_len;		/* Length of .bxx and .mdt files */
	uint32_t phy_addr;		/* phy addr of the start of image */
	char	 app_name[MAX_APP_NAME_SIZE];	/* application name*/
};

enum qseecom_qceos_cmd_status {
	QSEOS_RESULT_SUCCESS = 0,
	QSEOS_RESULT_INCOMPLETE,
	QSEOS_RESULT_FAILURE  = 0xFFFFFFFF
};

enum qseecom_qceos_cmd_id {
	QSEOS_APP_START_COMMAND      = 0x01,
	QSEOS_APP_SHUTDOWN_COMMAND,
	QSEOS_APP_LOOKUP_COMMAND,
	QSEOS_REGISTER_LISTENER,
	QSEOS_DEREGISTER_LISTENER,
	QSEOS_CLIENT_SEND_DATA_COMMAND,
	QSEOS_LISTENER_DATA_RSP_COMMAND,
	QSEOS_LOAD_EXTERNAL_ELF_COMMAND,
	QSEOS_UNLOAD_EXTERNAL_ELF_COMMAND,
	QSEOS_CMD_MAX	  = 0xEFFFFFFF
};

static uint32_t qsee_app_id;
static void *qsee_sbuffer;
static int32_t basic_output;
static int basic_data_len;
static int mdt_size;
static int seg0_size;
static int seg1_size;
static int seg2_size;
static int seg3_size;
static int auth_size;
static uint8_t *mdt_file;
static uint8_t *seg0_file;
static uint8_t *seg1_file;
static uint8_t *seg2_file;
static uint8_t *seg3_file;
static uint8_t *auth_file;

static ssize_t mdt_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t pos, size_t count)
{
	uint8_t *tmp;
	/*
	 * Position '0' means new file being written,
	 * Hence allocate new memory after freeing already allocated mem if any
	 */
	if (pos == 0) {
		kfree(mdt_file);
		mdt_file = kzalloc((count) * sizeof(uint8_t), GFP_KERNEL);
	} else {
		tmp = mdt_file;
		mdt_file = krealloc(tmp,
			(pos + count) * sizeof(uint8_t), GFP_KERNEL);
	}

	if (!mdt_file)
		return -ENOMEM;

	memcpy((mdt_file + pos), buf, count);
	mdt_size = pos + count;
	return count;
}

static ssize_t seg0_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t pos, size_t count)
{
	uint8_t *tmp;
	if (pos == 0) {
		kfree(seg0_file);
		seg0_file = kzalloc((count) * sizeof(uint8_t), GFP_KERNEL);
	} else {
		tmp = seg0_file;
		seg0_file = krealloc(tmp, (pos + count) * sizeof(uint8_t),
					GFP_KERNEL);
	}

	if (!seg0_file)
		return -ENOMEM;

	memcpy((seg0_file + pos), buf, count);
	seg0_size = pos + count;
	return count;
}

static ssize_t seg1_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t pos, size_t count)
{
	uint8_t *tmp;
	if (pos == 0) {
		kfree(seg1_file);
		seg1_file = kzalloc((count) * sizeof(uint8_t), GFP_KERNEL);
	} else {
		tmp = seg1_file;
		seg1_file = krealloc(tmp, (pos + count) * sizeof(uint8_t),
					GFP_KERNEL);
	}

	if (!seg1_file)
		return -ENOMEM;

	memcpy((seg1_file + pos), buf, count);
	seg1_size = pos + count;
	return count;
}

static ssize_t seg2_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t pos, size_t count)
{
	uint8_t *tmp;
	if (pos == 0) {
		kfree(seg2_file);
		seg2_file = kzalloc((count) * sizeof(uint8_t), GFP_KERNEL);
	} else {
		tmp = seg2_file;
		seg2_file = krealloc(tmp, (pos + count) * sizeof(uint8_t),
					GFP_KERNEL);
	}

	if (!seg2_file)
		return -ENOMEM;

	memcpy((seg2_file + pos), buf, count);
	seg2_size = pos + count;
	return count;
}

static ssize_t seg3_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t pos, size_t count)
{
	uint8_t *tmp;
	if (pos == 0) {
		kfree(seg3_file);
		seg3_file = kzalloc((count) * sizeof(uint8_t), GFP_KERNEL);
	} else {
		tmp = seg3_file;
		seg3_file = krealloc(tmp, (pos + count) * sizeof(uint8_t),
					GFP_KERNEL);
	}

	if (!seg3_file)
		return -ENOMEM;

	memcpy((seg3_file + pos), buf, count);
	seg3_size = pos + count;
	return count;
}

static ssize_t auth_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t pos, size_t count)
{
	uint8_t *tmp = NULL;

	if (pos == 0) {
		kfree(auth_file);
		auth_file = kzalloc((count) * sizeof(uint8_t), GFP_KERNEL);
	} else {
		tmp = auth_file;
		auth_file = krealloc(tmp, (pos + count) * sizeof(uint8_t),
					GFP_KERNEL);
	}

	if (!auth_file) {
		if (tmp)
			kfree(tmp);
		return -ENOMEM;
	}

	memcpy((auth_file + pos), buf, count);
	auth_size = pos + count;

	return count;
}

struct bin_attribute mdt_attr = {
	.attr = {.name = "mdt_file", .mode = 0666},
	.write = mdt_write,
};

struct bin_attribute seg0_attr = {
	.attr = {.name = "seg0_file", .mode = 0666},
	.write = seg0_write,
};

struct bin_attribute seg1_attr = {
	.attr = {.name = "seg1_file", .mode = 0666},
	.write = seg1_write,
};

struct bin_attribute seg2_attr = {
	.attr = {.name = "seg2_file", .mode = 0666},
	.write = seg2_write,
};

struct bin_attribute seg3_attr = {
	.attr = {.name = "seg3_file", .mode = 0666},
	.write = seg3_write,
};

struct bin_attribute auth_attr = {
	.attr = {.name = "auth_file", .mode = 0666},
	.write = auth_write,
};

static int qseecom_unload_app(void)
{
	struct qseecom_unload_app_ireq req;
	struct qseecom_command_scm_resp resp;
	int ret;

	req.qsee_cmd_id = QSEOS_APP_SHUTDOWN_COMMAND;
	req.app_id = qsee_app_id;

	/* SCM_CALL to unload the app */
	ret = scm_call(SCM_SVC_TZSCHEDULER, 1,	&req,
			sizeof(struct qseecom_unload_app_ireq),
			&resp, sizeof(resp));
	if (ret)
		pr_err("scm_call to unload app (id = %d) failed\n", req.app_id);

	pr_info("App id %d now unloaded\n", req.app_id);
	return 0;
}

static int tzapp_test(void *input, void *output, int input_len, int option)
{
	int ret = 0;
	int ret1, ret2;

	struct qseecom_client_send_data_ireq send_data_req;
	struct qseecom_command_scm_resp resp;
	struct qsc_send_cmd *msgreq;	 /* request data sent to QSEE */
	struct qsc_send_cmd_rsp *msgrsp; /* response data sent from QSEE */
	struct page *pg_tmp;
	unsigned long pg_addr;

	/*
	 * Using alloc_pages to avoid colliding with input pointer's
	 * allocated page, since qsee_register_shared_buffer() in sampleapp
	 * checks if input ptr is in secure area. Page where msgreq/msgrsp
	 * is allocated is added to blacklisted area by sampleapp and added
	 * as secure memory region, hence input data (shared buffer)
	 * cannot be in that secure memory region
	 */
	pg_tmp = alloc_page(GFP_KERNEL);
	if (!pg_tmp) {
		pr_err("\nFailed to allocate page");
		return -ENOMEM;
	}
	/*
	 * Getting virtual page address. pg_tmp will be pointing to
	 * first page structure
	 */
	msgreq = page_address(pg_tmp);
	if (!msgreq) {
		pr_err("Unable to allocate memory\n");
		return -ENOMEM;
	}
	/* pg_addr for passing to free_page */
	pg_addr = (unsigned long) msgreq;

	msgrsp = (struct qsc_send_cmd_rsp *)((uint8_t *) msgreq +
						sizeof(struct qsc_send_cmd));
	if (!msgrsp) {
		kfree(msgreq);
		pr_err("Unable to allocate memory\n");
		return -ENOMEM;
	}

	/*
	 * option = 1 -> Basic Multiplication, option = 2 -> Crypto Function
	 * option = 3 -> Authorized OTP fusing
	 */
	switch (option) {
	case 1:
		msgreq->cmd_id = CLIENT_CMD1_BASIC_DATA;
		msgreq->data = *((uint32_t *)input);
		break;
	case 2:
		msgreq->cmd_id = CLIENT_CMD8_RUN_CRYPTO_TEST;
		break;
	case 3:
		if (!auth_file) {
			pr_err("No OTP file provided\n");
			return -ENOMEM;
		}

		msgreq->cmd_id = CLIENT_CMD_AUTH;
		msgreq->data = dma_map_single(NULL, auth_file,
				auth_size, DMA_TO_DEVICE);
		ret = dma_mapping_error(NULL, msgreq->data);
		if (ret) {
			pr_err("DMA Mapping Error Return values: otp_buffer %d",
									ret);
			return ret;
		}

		break;
	default:
		pr_err("\n Invalid Option");
		goto fn_exit;
	}

	send_data_req.qsee_cmd_id = QSEOS_CLIENT_SEND_DATA_COMMAND;
	send_data_req.app_id = qsee_app_id;

	send_data_req.req_ptr = dma_map_single(NULL, msgreq,
				sizeof(*msgreq), DMA_TO_DEVICE);
	send_data_req.rsp_ptr = dma_map_single(NULL, msgrsp,
				sizeof(*msgrsp), DMA_FROM_DEVICE);

	ret1 = dma_mapping_error(NULL, send_data_req.req_ptr);
	ret2 = dma_mapping_error(NULL, send_data_req.rsp_ptr);

	if (!ret1 && !ret2) {
		send_data_req.req_len = sizeof(struct qsc_send_cmd);
		send_data_req.rsp_len = sizeof(struct qsc_send_cmd_rsp);
		ret = scm_call(SCM_SVC_TZSCHEDULER, 1,
					(const void *) &send_data_req,
					sizeof(send_data_req),
					&resp, sizeof(resp));
	}

	if (!ret1) {
		dma_unmap_single(NULL, send_data_req.req_ptr,
			sizeof(*msgreq), DMA_TO_DEVICE);
	}

	if (!ret2) {
		dma_unmap_single(NULL, send_data_req.rsp_ptr,
			sizeof(*msgrsp), DMA_FROM_DEVICE);
	}

	if (ret1 || ret2) {
		pr_err("\nDMA Mapping Error Return values:"
			"req_ptr %d rsp_ptr %d", ret1, ret2);
		return ret1 ? ret1 : ret2;
	}

	if (ret) {
		pr_err("qseecom_scm_call failed with err: %d\n", ret);
		goto fn_exit;
	}

	if (resp.result == QSEOS_RESULT_INCOMPLETE) {
		pr_err("Result incomplete\n");
		ret = -EINVAL;
		goto fn_exit;
	} else {
		if (resp.result != QSEOS_RESULT_SUCCESS) {
			pr_err("Response result %d not supported\n",
							resp.result);
			ret = -EINVAL;
			goto fn_exit;
		} else {
			if (option == 2)
				pr_info("Crypto functionality test successful\n");
		}
	}

	if (option == 1) {
		if (msgrsp->status) {
			pr_err("Input size exceeded supported range\n");
			ret = -EINVAL;
		}
		basic_output = msgrsp->data;
	} else if (option == 3) {
		if (msgrsp->status) {
			pr_err("Auth and blow failed with response %d\n",
							msgrsp->status);
			ret = -EIO;
		}
		else
			pr_info("Auth and blow Success");

	}
fn_exit:
	free_page(pg_addr);
	if (option == 3) {
		dma_unmap_single(NULL, msgreq->data,
			auth_size, DMA_TO_DEVICE);
	}

	return ret;
}

static int32_t copy_files(int *img_size)
{
	uint8_t *buf;

	if (mdt_file && seg0_file && seg1_file && seg2_file && seg3_file) {
		*img_size = mdt_size + seg0_size + seg1_size
				+ seg2_size + seg3_size;

		qsee_sbuffer = kzalloc(*img_size, GFP_KERNEL);
		if (!qsee_sbuffer) {
			pr_err("Error: qsee_sbuffer alloc failed\n");
			return -ENOMEM;
		}
		buf = qsee_sbuffer;

		memcpy(buf, mdt_file, mdt_size);
		buf += mdt_size;
		memcpy(buf, seg0_file, seg0_size);
		buf += seg0_size;
		memcpy(buf, seg1_file, seg1_size);
		buf += seg1_size;
		memcpy(buf, seg2_file, seg2_size);
		buf += seg2_size;
		memcpy(buf, seg3_file, seg3_size);
	} else {
		pr_err("\nSampleapp file Inputs not provided\n");
		return -EINVAL;
	}
	return 0;
}

static int load_app(void)
{
	struct qseecom_load_app_ireq load_req;
	struct qseecom_command_scm_resp resp;
	int ret, ret1;
	int img_size;
	struct qsee_notify_app notify_app;

	notify_app.cmd_id = QSEE_APP_NOTIFY_COMMAND;
	notify_app.applications_region_addr = TZAPP_START_ADDRESS;
	notify_app.applications_region_size = TZAPP_SIZE;
	ret = scm_call(SCM_SVC_TZSCHEDULER, 1, &notify_app,
		sizeof(struct qsee_notify_app), &resp, sizeof(resp));
	if (ret) {
		pr_err("Notify App failed\n");
		return -1;
	}

	ret = copy_files(&img_size);
	if (ret) {
		pr_err("Copying Files failed\n");
		return ret;
	}

	/* Populate the structure for sending scm call to load image */
	strlcpy(load_req.app_name, "sampleapp", sizeof("sampleapp"));
	load_req.qsee_cmd_id = QSEOS_APP_START_COMMAND;
	load_req.mdt_len = mdt_size;
	load_req.img_len = img_size;
	load_req.phy_addr = dma_map_single(NULL, qsee_sbuffer,
				img_size, DMA_TO_DEVICE);
	ret1 = dma_mapping_error(NULL, load_req.phy_addr);

	if (ret1 == 0) {
		/* SCM_CALL to load the app and get the app_id back */
		ret = scm_call(SCM_SVC_TZSCHEDULER, 1,	&load_req,
			sizeof(struct qseecom_load_app_ireq),
			&resp, sizeof(resp));
		dma_unmap_single(NULL, load_req.phy_addr,
				img_size, DMA_TO_DEVICE);
	}
	if (ret1) {
		pr_err("\nDMA Mapping error (qsee_sbuffer)");
		return ret1;
	}

	if (ret) {
		pr_err("SCM_CALL to load app failed\n");
		return ret;
	}

	if (resp.result == QSEOS_RESULT_FAILURE) {
		pr_err("SCM_CALL rsp.result is QSEOS_RESULT_FAILURE\n");
		return -EFAULT;
	}

	if (resp.result == QSEOS_RESULT_INCOMPLETE)
		pr_err("Process_incomplete_cmd ocurred\n");

	if (resp.result != QSEOS_RESULT_SUCCESS) {
		pr_err("scm_call failed resp.result unknown, %d\n",
				resp.result);
		return -EFAULT;
	}

	pr_info("\n Loaded Sampleapp Succesfully!\n");

	qsee_app_id = resp.data;
	return 0;
}

/* To show basic multiplication output */
static ssize_t
show_basic_output(struct device *dev, struct device *attr,
					char *buf)
{
	return snprintf(buf, (basic_data_len + 1), "%d", basic_output);
}

/* Basic multiplication App*/
static ssize_t
store_basic_input(struct device *dev, struct device *attr,
					const char *buf, size_t count)
{
	uint32_t basic_input __aligned(32);

	basic_data_len = count;
	if ((count - 1) == 0) {
		pr_err("\n Input cannot be NULL!");
		return -EINVAL;
	}
	if (kstrtouint(buf, 10, &basic_input))
		pr_info("\n Please enter a valid unsigned integer");

	else
		tzapp_test(&basic_input, NULL, 0, 1);

	return count;
}

/* Crypto Test*/
static ssize_t
store_crypto_input(struct device *dev, struct device *attr,
		const char *buf, size_t count)
{
	int32_t ret = -EINVAL;

	ret = tzapp_test(NULL, NULL, 0, 2);
	return count;
}

static ssize_t
store_fuse_otp_input(struct device *dev, struct device *attr,
		const char *buf, size_t count)
{
	tzapp_test(buf, NULL, 0, 3);
	return count;
}

static ssize_t
store_load_start(struct device *dev, struct device *attr,
		const char *buf, size_t count)
{
	int load_cmd;

	if (kstrtouint(buf, 10, &load_cmd)) {
		pr_err("\n Provide valid integer input!");
		pr_err("Echo 1 to start loading app");
		return -EINVAL;
	}
	if (load_cmd == 1)
		load_app();
	else
		pr_info("\nEcho 1 to start loading app");

	return count;
}

static  DEVICE_ATTR(fuse_otp, 0666, NULL, store_fuse_otp_input);
static	DEVICE_ATTR(crypto, 0666, NULL,
					store_crypto_input);
static	DEVICE_ATTR(basic_data, 0666, show_basic_output,
					store_basic_input);
static	DEVICE_ATTR(load_start, 0222, NULL,
					store_load_start);

static struct attribute *tzapp_attrs[] = {
	&dev_attr_fuse_otp.attr,
	&dev_attr_crypto.attr,
	&dev_attr_basic_data.attr,
	&dev_attr_load_start.attr,
	NULL,
};

static struct attribute_group tzapp_attr_grp = {
	.attrs = tzapp_attrs,
};

struct kobject *tzapp_kobj;

static int __init tzapp_init(void)
{
	int err;

	tzapp_kobj = kobject_create_and_add("tzapp", firmware_kobj);

	err = sysfs_create_group(tzapp_kobj, &tzapp_attr_grp);

	if (err) {
		kobject_put(tzapp_kobj);
		return err;
	}
	return 0;
}

static int __init qseecom_init(void)
{
	sysfs_create_bin_file(firmware_kobj, &mdt_attr);
	sysfs_create_bin_file(firmware_kobj, &seg0_attr);
	sysfs_create_bin_file(firmware_kobj, &seg1_attr);
	sysfs_create_bin_file(firmware_kobj, &seg2_attr);
	sysfs_create_bin_file(firmware_kobj, &seg3_attr);
	sysfs_create_bin_file(firmware_kobj, &auth_attr);

	if (!tzapp_init())
		pr_info("\nLoaded Module Successfully!\n");
	else
		pr_info("\nFailed to load module\n");
	return 0;
}

static void __exit qseecom_exit(void)
{
	qseecom_unload_app();

	sysfs_remove_bin_file(firmware_kobj, &mdt_attr);
	sysfs_remove_bin_file(firmware_kobj, &seg0_attr);
	sysfs_remove_bin_file(firmware_kobj, &seg1_attr);
	sysfs_remove_bin_file(firmware_kobj, &seg2_attr);
	sysfs_remove_bin_file(firmware_kobj, &seg3_attr);
	sysfs_remove_bin_file(firmware_kobj, &auth_attr);

	sysfs_remove_group(tzapp_kobj, &tzapp_attr_grp);
	kobject_put(tzapp_kobj);

	kfree(mdt_file);
	kfree(seg0_file);
	kfree(seg1_file);
	kfree(seg2_file);
	kfree(seg3_file);
	kfree(auth_file);
	kfree(qsee_sbuffer);
}
MODULE_LICENSE("GPL v2");
module_init(qseecom_init);
module_exit(qseecom_exit);
