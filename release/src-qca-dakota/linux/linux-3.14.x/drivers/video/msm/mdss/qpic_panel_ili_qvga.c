/* Copyright (c) 2013 - 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/memory.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/io.h>

#include "mdss.h"
#include "mdss_qpic.h"
#include "mdss_qpic_panel.h"



void ili9341_off(struct qpic_panel_io_desc *qpic_panel_io)
{
	/* TBD */

}

int ili9341_on(struct qpic_panel_io_desc *qpic_panel_io)
{
	u8 param[4];

	qpic_send_pkt(OP_SOFT_RESET, NULL, 0);
	/* wait for 120 ms after reset as panel spec suggests */
	msleep(120);
	qpic_send_pkt(OP_SET_DISPLAY_OFF, NULL, 0);
	/* wait for 20 ms after disply off */
	msleep(20);

	/* set memory access control */
	param[0] = 0x48;
	qpic_send_pkt(OP_SET_ADDRESS_MODE, param, 1);
	/* wait for 20 ms after command sent as panel spec suggests */
	msleep(20);

	param[0] = 0x66;
	qpic_send_pkt(OP_SET_PIXEL_FORMAT, param, 1);
	/* wait for 20 ms after command sent as panel spec suggests */
	msleep(20);

	/* set interface */
	param[0] = 1;
	param[1] = 0;
	param[2] = 0;
	qpic_send_pkt(OP_ILI9341_INTERFACE_CONTROL, param, 3);
	/* wait for 20 ms after command sent */
	msleep(20);

	/* exit sleep mode */
	qpic_send_pkt(OP_EXIT_SLEEP_MODE, NULL, 0);
	/* wait for 20 ms after command sent as panel spec suggests */
	msleep(20);

	/* normal mode */
	qpic_send_pkt(OP_ENTER_NORMAL_MODE, NULL, 0);
	/* wait for 20 ms after command sent as panel spec suggests */
	msleep(20);

	/* display on */
	qpic_send_pkt(OP_SET_DISPLAY_ON, NULL, 0);
	/* wait for 20 ms after command sent as panel spec suggests */
	msleep(20);

	param[0] = 0;
	qpic_send_pkt(OP_ILI9341_TEARING_EFFECT_LINE_ON, param, 1);

	/* test */
	param[0] = qpic_read_data(OP_GET_PIXEL_FORMAT, 1);
	pr_debug("Pixel format =%x", param[0]);

	return 0;
}

static int mdss_qpic_ili9341_panel_init(void)
{
	qpic_panel_on = ili9341_on;
	qpic_panel_off = ili9341_off;
	return 0;
}
static void mdss_qpic_ili9341_panel_exit(void)
{
	qpic_panel_on = NULL;
	qpic_panel_off = NULL;
	return;
}
module_init(mdss_qpic_ili9341_panel_init);
module_exit(mdss_qpic_ili9341_panel_exit);
