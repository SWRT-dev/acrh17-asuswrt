/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/debugfs.h> /* this is for DebugFS libraries */
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <soc/qcom/scm.h>
#include <linux/slab.h>

#define BUF_LEN 0x1000
#define TZ_INFO_GET_DIAG_ID 0x2

struct dentry *dirret, *fileret;

static char ker_buf[BUF_LEN] __aligned(4096), tmp_buf[BUF_LEN];

struct tzbsp_log_pos_t {
	uint16_t wrap;		/* Ring buffer wrap-around ctr */
	uint16_t offset;	/* Ring buffer current position */
};

struct tzbsp_diag_log_t {
	struct tzbsp_log_pos_t log_pos;	/* Ring buffer position mgmt */
	uint8_t log_buf[1];		/* Open ended array to the end
					 * of the 4K IMEM buffer
					 */
};

struct tzbsp_diag_t {
	uint32_t unused[7];	/* Unused variable is to support the
				 * corresponding structure in trustzone
				 */
	uint32_t ring_off;
	uint32_t unused1[514];
	struct tzbsp_diag_log_t log;
};

struct log_read {
	uint32_t log_buf;
	uint32_t buf_size;
};

/* Read file operation */
static ssize_t tz_log_read(struct file *fp, char __user *user_buffer,
				size_t count, loff_t *position)
{
	int ret;
	struct log_read rdip;
	struct tzbsp_diag_t *tz_diag;
	uint16_t offset;
	uint16_t ring;

	rdip.buf_size = BUF_LEN;
	rdip.log_buf = dma_map_single(NULL, ker_buf, BUF_LEN, DMA_FROM_DEVICE);
	ret = dma_mapping_error(NULL, rdip.log_buf);
	if (ret != 0) {
		pr_err("DMA Mapping Error : %d\n", ret);
		return -EINVAL;
	}

	/* SCM call to TZ to get the tz log */
	ret = scm_call(SCM_SVC_INFO, TZ_INFO_GET_DIAG_ID,
		&rdip, sizeof(struct log_read),  NULL, 0);
	dma_unmap_single(NULL, rdip.log_buf, BUF_LEN, DMA_FROM_DEVICE);
	if (ret != 0) {
		pr_err("Error in getting tz log : %d\n", ret);
		return -EINVAL;
	}

	tz_diag = (struct tzbsp_diag_t *)ker_buf;
	offset = tz_diag->log.log_pos.offset;
	ring = tz_diag->ring_off;

	if (tz_diag->log.log_pos.wrap != 0) {
		memcpy(tmp_buf, (ker_buf + offset + ring),
					(BUF_LEN - offset - ring));
		memcpy(tmp_buf + (BUF_LEN - offset - ring), (ker_buf + ring),
					offset);
	} else {
		memcpy(tmp_buf, (ker_buf + ring), offset);
	}

	return simple_read_from_buffer(user_buffer, count,
					position, tmp_buf, BUF_LEN);
}

static const struct file_operations fops_tz_log = {
	.read = tz_log_read,
};

static int __init init_tz_log(void)
{
	int filevalue;
	dirret = debugfs_create_dir("qcom_debug_logs", NULL);
	fileret = debugfs_create_file("tz_log", 0444, dirret,
					&filevalue, &fops_tz_log);
	return 0;
}

static void __exit exit_tz_log(void)
{
	/* removing the directory recursively which
	in turn cleans all the file */
	debugfs_remove_recursive(dirret);
}

module_init(init_tz_log);
module_exit(exit_tz_log);
