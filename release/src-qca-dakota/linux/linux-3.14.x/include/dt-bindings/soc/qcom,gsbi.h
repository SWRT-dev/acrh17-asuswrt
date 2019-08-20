/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#ifndef __DT_BINDINGS_QCOM_GSBI_H
#define __DT_BINDINGS_QCOM_GSBI_H

#define GSBI_PROT_IDLE		0
#define GSBI_PROT_I2C_UIM	1
#define GSBI_PROT_I2C		2
#define GSBI_PROT_SPI		3
#define GSBI_PROT_UART_W_FC	4
#define GSBI_PROT_UIM		5
#define GSBI_PROT_I2C_UART	6

#define GSBI_CRCI_QUP		0
#define GSBI_CRCI_UART		1

/* Masks for GSBI ADM MUX SEL */
#define IPQ806X_GSBI1_ADM_CRCI_MUX_SEL_MASK     0x1800
#define IPQ806X_GSBI2_ADM_CRCI_MUX_SEL_MASK     0x6000
#define IPQ806X_GSBI3_ADM_CRCI_MUX_SEL_MASK     0x0030
#define IPQ806X_GSBI4_ADM_CRCI_MUX_SEL_MASK     0x00C0
#define IPQ806X_GSBI5_ADM_CRCI_MUX_SEL_MASK     0x0300
#define IPQ806X_GSBI6_ADM_CRCI_MUX_RX_SEL_MASK  0x0400
#define IPQ806X_GSBI6_ADM_CRCI_MUX_TX_SEL_MASK  0x0020
#define IPQ806X_GSBI7_ADM_CRCI_MUX_SEL_MASK     0x00C0

#endif
