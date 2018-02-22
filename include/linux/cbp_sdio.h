/*
 * include/linux/cbp_sdio.h
 *
 * VIA CBP SDIO driver for Linux
 *
 * Copyright (C) 2009 VIA TELECOM Corporation, Inc.
 * Author: VIA TELECOM Corporation, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef CBP_SDIO_H
#define CBP_SDIO_H

#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/mmc/host.h>
#include <linux/ap_sync_cbp.h>


#define CBP_DRIVER_NAME "cbp"

struct cbp_wait_event {
	wait_queue_head_t wait_q;
	atomic_t state;
	int wait_gpio;
	int wait_polar;
};

struct cbp_platform_data {
	char *bus;
	char *host_id;

	bool ipc_enable;
	bool data_ack_enable;
	bool rst_ind_enable;
	bool flow_ctrl_enable;
	bool tx_disable_irq;
	struct asc_config *tx_handle;

	int gpio_ap_wkup_cp;
	int gpio_cp_ready;
	int gpio_cp_wkup_ap;
	int gpio_ap_ready;
	int gpio_sync_polar;

	int gpio_data_ack;
	int gpio_data_ack_polar;

	int gpio_rst_ind;
	int gpio_rst_ind_polar;

	int gpio_flow_ctrl;
	int gpio_flow_ctrl_polar;

	int gpio_pwr_on;
	int gpio_pmic;
	int gpio_rst;
	//for the level transfor chip fssd06
	int gpio_lv_shift;

	int gpio_ets_sel;
	int gpio_ets_sel1;
	int gpio_usb_switch;
	int gpio_sim_swap;
	int gpio_ant_ctrl;
#ifdef CONFIG_CBP_SIM_HOTPLUG
	int gpio_sim_detect;
#endif
	struct cbp_wait_event *cbp_data_ack;
	void (*data_ack_wait_event)(struct cbp_wait_event *pdata_ack);
	struct cbp_wait_event *cbp_flow_ctrl;
	void (*flow_ctrl_wait_event)(struct cbp_wait_event *pflow_ctrl);
};

typedef enum {
	MODEM_ST_READY = 0, /*modem ready*/
	MODEM_ST_TX_RX,
	MODEM_ST_UNKNOW
} data_ack_state;

#endif
