/*
 * drivers/mmc/card/modem_sdio.h
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
#ifndef MODEM_SDIO_H
#define MODEM_SDIO_H

#include <linux/cbp_sdio.h>

#define SDIO_TTY_NR 6			/* Number of UARTs this driver can handle */

struct sdio_msg_head
{
	unsigned char start_flag;	/*start flag(1), little endian*/
	unsigned char chanInfo;		/*channel id*/
	unsigned char tranHi;		/*len High byte, little endian*/
	unsigned char tranLow;		/*len low byte, little endian*/
};

struct sdio_msg
{
	struct sdio_msg_head head;
	unsigned char buffer[4096];
};

typedef enum{
	SFLOW_CTRL_DISABLE = 0,
	SFLOW_CTRL_ENABLE,
} Sflow_ctrl_state;

struct sdio_modem_port {
	struct sdio_modem *modem;
	struct sdio_func	*func;
	struct tty_port	port;
	struct kref		kref;
	struct mutex		mutex;
	spinlock_t		write_lock;
	struct tty_struct	*tty;
	unsigned int		index;
	unsigned char		do_wake;

	struct kfifo		transmit_fifo;
	unsigned int		transmit_fifo_size;
	const char  *name;
	char work_name[64];
	struct workqueue_struct *write_q;
	struct work_struct	write_work;
	struct device dev;

	wait_queue_head_t sflow_ctrl_wait_q;
	atomic_t sflow_ctrl_state;

	/* Settings for the port */
	int rts_state;	/* Handshaking pins (outputs) */
	int dtr_state;
	int cts_state;	/* Handshaking pins (inputs) */
	int dsr_state;
	spinlock_t inception_lock;
	int inception;
	wait_queue_head_t wait_q;
};

struct ctrl_port_msg {
	struct sdio_msg_head  head;
	unsigned char id_hi;
	unsigned char id_low;
	unsigned char chan_num;
	unsigned char option;
};

struct sdio_modem_ctrl_port {
	struct ctrl_port_msg chan_ctrl_msg;
	unsigned int chan_state;
	unsigned int msg_id;
	wait_queue_head_t sflow_ctrl_wait_q;
	atomic_t sflow_ctrl_state;
};

struct sdio_modem {
	struct sdio_modem_port *port[SDIO_TTY_NR];
	struct sdio_modem_ctrl_port *ctrl_port;
	struct sdio_func *func;
	struct sdio_msg *msg;
	unsigned char *trans_buffer;
	struct cbp_platform_data *cbp_data;
	struct semaphore sem;
	unsigned int data_length;
#ifdef CONFIG_PM_RUNTIME
	bool runtime;
#endif
};

extern int sdio_log_level;

#define LOG_ERR		0
#define LOG_INFO	1
#define LOG_NOTICE	2
#define LOG_DEBUG	3

#define LOGPRT(lvl,x...) do{ \
		if(lvl < (sdio_log_level + 1)) \
		printk(KERN_INFO "[SDIO MODEM] " x); \
	} while (0)

extern int modem_sdio_init(struct cbp_platform_data *pdata);
extern void modem_sdio_exit(void);
#endif
