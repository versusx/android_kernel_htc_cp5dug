/*
 * drivers/mmc/card/modem_sdio.c
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

#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/circ_buf.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/kfifo.h>
#include <linux/slab.h>

#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/mmc/host.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include "modem_sdio.h"
#include <linux/pm_runtime.h>

static int sdio_tx_cnt = 0;
static int sdio_rx_cnt = 0;

#define FIFO_SIZE	8 * PAGE_SIZE

#define TRANSMIT_SHIFT	(10)
#define TRANSMIT_BUFFER_SIZE	(1UL << TRANSMIT_SHIFT)
#define TRANSMIT_MAX_SIZE	 ((1UL << TRANSMIT_SHIFT)  - 4)

#define TTY_PORT_KEEP_EXIST 1
#ifdef TTY_PORT_KEEP_EXIST
static int tty_port_initial = 0;
#define TTY_PORT_GPS_BLOCK 1
#define GPS_PORT_INDEX 5
#endif

#define CONFIG_DO_WAKE	1

static struct tty_driver *modem_sdio_tty_driver;
static struct cbp_platform_data *cbp_pdata;
static struct sdio_modem_port *sdio_modem_table[SDIO_TTY_NR];
#ifdef CONFIG_PM_RUNTIME
static struct platform_device *cbp_rt_device;
static struct sdio_modem *modem_rt;
bool is_rt_suspend = false;
bool rt_unbal = false;
wait_queue_head_t rt_wait_q;
#define CBP_RT_DRIVER_NAME "cbp_rt"
#endif
static DEFINE_SPINLOCK(sdio_modem_table_lock);
static DEFINE_MUTEX(tty_remove_mutex);
int sdio_log_level = LOG_INFO;

extern bool is_host_suspend;
extern bool host_js;
bool sdio_in_use = false;
EXPORT_SYMBOL(sdio_in_use);
wait_queue_head_t wr_wait_q;
EXPORT_SYMBOL(wr_wait_q);

static void sdio_rx_dump(const unsigned char *buf)
{
	unsigned int count;
	const unsigned char *print_buf = buf;
	int i;

	count =(((*(print_buf + 2) & 0x0F) << 8) | (*(print_buf + 3) & 0xFF));

	printk("MODEM SDIO: read %d from port%d\n",
			count, *(print_buf + 1) -1);

	if (count > 64)
		count = 64;

	for(i = 0; i < count + 4; i++) {
		printk("%x-", *(print_buf + i));
		if (i % 31 == 0 && i != 0)
			printk("\n");
		else if (i == count + 3)
			printk("\n");
	}

	printk("MODEM SDIO: dump data finished!\n");
}
static void sdio_tx_rx_printk(const unsigned char *buf, unsigned char tx)
{
	unsigned int count;
	const unsigned char *print_buf = buf;
	int i;

	count =(((*(print_buf + 2) & 0x0F) << 8) | (*(print_buf + 3) & 0xFF));
	if(tx == 1){
		printk("MODEM SDIO: write %d to port%d/[%d]>>",
			count, *(print_buf+1) -1, sdio_tx_cnt);
	}
	else{
		printk("MODEM SDIO: read %d from port%d/[%d]<<",
			count, *(print_buf + 1) -1, sdio_rx_cnt);
	}
	if(*(print_buf + 1) == 4){
		if(count > 20)
			count = 20;
		for(i = 0; i < count; i++)
		{
			printk("%c", *(print_buf + i));
		}
		printk("\n");
	} else {
		if (count > 20)
			count = 20;
		for(i = 0; i < count + 4; i++)
		{
			printk("%x-", *(print_buf + i));
		}
		printk("\n");
	}
}

static int dump_asc_tx_rx_state(void)
{
#if 1
	unsigned char ap_ready, cp_ready, cp_wake_ap, ap_wake_cp;

	ap_ready = gpio_get_value(131);

	cp_ready = gpio_get_value(123);

	cp_wake_ap = gpio_get_value(124);

	ap_wake_cp = gpio_get_value(147);

	LOGPRT(LOG_INFO, "ap_ready = %d, cp_ready = %d, cp_wake_ap = %d, ap_wake_cp = %d\n",
			ap_ready, cp_ready, cp_wake_ap, ap_wake_cp);
#endif

	return 0;
}

static const char *port_name(int id) {
	char * name;
	switch (id) {
		case 0:
			name = "data";
			break;
		case 1:
			name =  "ets";
			break;
		case 2:
			name =  "rfs";
			break;
		case 3:
			name =  "atc";
			break;
		case 4:
			name =  "pcv";
			break;
		case 5:
			name =  "gps";
			break;
		default:
			name =  "null";
			break;
	}
	return name;
}

static struct sdio_modem_port *sdio_modem_port_get(unsigned index)
{
	struct sdio_modem_port *port;

	if (index >= SDIO_TTY_NR)
		return NULL;

	spin_lock(&sdio_modem_table_lock);
	port = sdio_modem_table[index];
	if (port) {
		kref_get(&port->kref);
	}
	spin_unlock(&sdio_modem_table_lock);

	return port;
}

static int check_port(struct sdio_modem_port *port) {
	if (!port || !port->func)
		return -ENODEV;
	
	return 0;
}

static void modem_sdio_write(struct sdio_modem_port *port, int addr,
		void *buf, size_t len);

enum cbp_contrl_message_type {
	CHAN_ONOFF_MSG_ID = 0,
	MDM_STATUS_IND_MSG_ID,
	MDM_STATUS_QUERY_MSG_ID,
	CHAN_SWITCH_REQ_MSG_ID,
	CHAN_STATUS_QUERY_MSG_ID,
	FLOW_CONTROL_MSG_ID,
	CHAN_LOOPBACK_TST_MSG_ID,
	MESSAGE_COUNT,
};

typedef enum {
	OPT_LOOPBACK_NON  = 0,
	OPT_LOOPBACK_OPEN  = 1,
	OPT_LOOPBACK_CLOSE  = 2,
	OPT_LOOPBACK_QUERY = 3,
	OPT_LOOPBACK_NUM
}IOP_OPT_LOOPBACK;

typedef enum {
         RSLT_LOOPBACK_SUCCESS  = 0,
         RSLT_LOOPBACK_WORK = 1,
         RSLT_LOOPBACK_CLOSED = 2,
         RSLT_LOOPBACK_INVALID = 3,
         RSLT_LOOPBACK_FAIL = 4,
         RSLT_LOOPBACK_NUM
}IOP_RSLT_LOOPBACK;

static int contruct_ctrl_chan_msg(struct sdio_modem_ctrl_port *ctrl_port , int msg,
						unsigned char chan_num, unsigned char opt)
{
	if (unlikely(ctrl_port == NULL)){
		LOGPRT(LOG_ERR, "%s %d: control channel is null.\n", __func__, __LINE__);
		return -EINVAL;
	}

	ctrl_port->chan_ctrl_msg.head.start_flag = 0xFE;
	ctrl_port->chan_ctrl_msg.head.chanInfo = 0;
	ctrl_port->chan_ctrl_msg.head.tranHi = 0;		
	ctrl_port->chan_ctrl_msg.head.tranLow = 4;		
	ctrl_port->chan_ctrl_msg.id_hi = msg >> 8;		
	ctrl_port->chan_ctrl_msg.id_low = msg;			
	ctrl_port->chan_ctrl_msg.chan_num = chan_num;		
	ctrl_port->chan_ctrl_msg.option = opt;

	return 0;
}
static unsigned char loop_back[12];

int modem_on_off_ctrl_chan(unsigned char on)
{
	struct sdio_modem *modem;
	struct sdio_modem_port *port;
	struct sdio_modem_ctrl_port *ctrl_port;
	unsigned long timeout = 0;
	unsigned char msg_len = 0;
	int ret = 0;

	LOGPRT(LOG_NOTICE,  "%s: enter\n", __func__);
	port = sdio_modem_port_get(0);
	ret = check_port(port);
	if (ret < 0) {
		LOGPRT(LOG_ERR,  "%s %d check_port failed\n", __func__,__LINE__);
		goto down_out;
	}
	modem = port->modem;
	if (down_interruptible(&modem->sem)) {
		LOGPRT(LOG_ERR,  "%s %d down_interruptible failed.\n", __func__,__LINE__);
		ret =  -ERESTARTSYS;
		goto down_out;
	}

	ctrl_port = modem->ctrl_port;

	timeout = wait_event_timeout(ctrl_port->sflow_ctrl_wait_q,
				SFLOW_CTRL_DISABLE == atomic_read(&ctrl_port->sflow_ctrl_state),
				msecs_to_jiffies(5000));
	if (!timeout) {
		LOGPRT(LOG_INFO, "%s %d: wait sflow ctrl off timeout!!\n",
					__func__, __LINE__);
	}

	ret = contruct_ctrl_chan_msg(ctrl_port, CHAN_ONOFF_MSG_ID, 0, on);
	if (ret < 0) {
		LOGPRT(LOG_ERR,  "%s contruct_ctrl_chan_msg failed\n", __func__);
		goto up_sem;
	}
	msg_len = sizeof(struct ctrl_port_msg);
	msg_len = (msg_len + 3) & ~0x03;  
	modem_sdio_write(port, 0x00, &(ctrl_port->chan_ctrl_msg), msg_len);

up_sem:
	up(&modem->sem);
down_out:
	return ret;
}
EXPORT_SYMBOL(modem_on_off_ctrl_chan);

int modem_dtr_send(bool on)
{
	struct sdio_modem *modem;
	struct sdio_modem_port *port;
	struct sdio_modem_ctrl_port *ctrl_port;
	unsigned char control_signal=0;
	unsigned long timeout = 0;
	unsigned char msg_len = 0;
	int ret = 0;

	LOGPRT(LOG_NOTICE,  "%s: enter\n", __func__);
	port = sdio_modem_port_get(0);
	ret = check_port(port);
	if (ret < 0) {
		LOGPRT(LOG_ERR,  "%s %d check_port failed\n", __func__,__LINE__);
		goto down_out;
	}
	modem = port->modem;
	if (down_interruptible(&modem->sem)) {
		LOGPRT(LOG_ERR,  "%s %d down_interruptible failed.\n", __func__,__LINE__);
		ret =  -ERESTARTSYS;
		goto down_out;
	}

	ctrl_port = modem->ctrl_port;

	timeout = wait_event_timeout(ctrl_port->sflow_ctrl_wait_q,
				SFLOW_CTRL_DISABLE == atomic_read(&ctrl_port->sflow_ctrl_state),
				msecs_to_jiffies(5000));
	if (!timeout) {
		LOGPRT(LOG_INFO, "%s %d: wait sflow ctrl off timeout!!\n",
					__func__, __LINE__);
	}

	if (ctrl_port->chan_state == 1) {
		if (on) {
			control_signal |= 0x04;
		} else {
			control_signal &= 0xFB;
		}

		ret = contruct_ctrl_chan_msg(ctrl_port, MDM_STATUS_IND_MSG_ID, 2, control_signal);
		if (ret < 0) {
			LOGPRT(LOG_ERR,  "%s contruct_ctrl_chan_msg failed\n", __func__);
			goto up_sem;
		}
		msg_len = sizeof(struct ctrl_port_msg);
		msg_len = (msg_len + 3) & ~0x03;  
		modem_sdio_write(port, 0x00, &(ctrl_port->chan_ctrl_msg), msg_len);
	} else {
		ret = -1;
		LOGPRT(LOG_ERR,  "%s: ctrl channel is off, please turn on first\n", __func__);
	}

up_sem:
	up(&modem->sem);
down_out:
	return ret;
}
EXPORT_SYMBOL(modem_dtr_send);

int modem_dtr_query(int *status,const char query_mode)
{
	struct sdio_modem *modem;
	struct sdio_modem_port *port;
	struct sdio_modem_ctrl_port *ctrl_port;
	unsigned char msg_len = 0;
	unsigned long timeout = 0;
	int ret = 0;

	LOGPRT(LOG_NOTICE,  "%s: enter\n", __func__);
	port = sdio_modem_port_get(0);
	ret = check_port(port);
	if (ret < 0) {
		LOGPRT(LOG_ERR,  "%s %d check_port failed\n", __func__,__LINE__);
		goto down_out;
	}
	modem = port->modem;
	ctrl_port = modem->ctrl_port;

	if (ctrl_port->chan_state == 1) {
		if (query_mode)
		{
			timeout = wait_event_timeout(ctrl_port->sflow_ctrl_wait_q,
						SFLOW_CTRL_DISABLE == atomic_read(&ctrl_port->sflow_ctrl_state),
						msecs_to_jiffies(5000));
			if (!timeout) {
				LOGPRT(LOG_INFO, "%s %d: wait sflow ctrl off timeout!!\n",
							__func__, __LINE__);
			}

			if (down_interruptible(&modem->sem)) {
				LOGPRT(LOG_ERR,  "%s %d down_interruptible failed.\n", __func__,__LINE__);
				ret =  -ERESTARTSYS;
				goto down_out;
			}

			ret = contruct_ctrl_chan_msg(ctrl_port, MDM_STATUS_QUERY_MSG_ID, 2, 0);
			if (ret < 0) {
				LOGPRT(LOG_ERR,  "%s contruct_ctrl_chan_msg failed\n", __func__);
				goto up_sem;
			}
			msg_len = sizeof(struct ctrl_port_msg);
			msg_len = (msg_len + 3) & ~0x03;  
			modem_sdio_write(port, 0x00, &(ctrl_port->chan_ctrl_msg), msg_len);
up_sem:
			up(&modem->sem);
		}
		*status = port->dtr_state;
	} else {
		ret = -1;
		LOGPRT(LOG_ERR,  "%s: ctrl channel is off, please turn on first\n", __func__);
	}
down_out:
	return ret;
}
EXPORT_SYMBOL(modem_dtr_query);

int modem_loop_back_chan(unsigned char chan_num, unsigned char opt)
{
	struct sdio_modem *modem;
	struct sdio_modem_port *port;
	struct sdio_modem_ctrl_port *ctrl_port;
	unsigned long timeout = 0;
	unsigned char msg_len = 0;
	int ret = 0;

	LOGPRT(LOG_NOTICE,  "%s: enter\n", __func__);
	port = sdio_modem_port_get(0);
	ret = check_port(port);
	if (ret < 0){
		LOGPRT(LOG_ERR,  "%s %d check_port failed\n", __func__,__LINE__);
		goto down_out;
	}
	modem = port->modem;
	if(down_interruptible(&modem->sem)){
		LOGPRT(LOG_ERR,  "%s %d down_interruptible failed.\n", __func__,__LINE__);
		ret =  -ERESTARTSYS;
		goto down_out;
	}

	ctrl_port = modem->ctrl_port;

	timeout = wait_event_timeout(ctrl_port->sflow_ctrl_wait_q,
				SFLOW_CTRL_DISABLE == atomic_read(&ctrl_port->sflow_ctrl_state),
				msecs_to_jiffies(5000));
	if (!timeout) {
		LOGPRT(LOG_INFO, "%s %d: wait sflow ctrl off timeout!!\n",
					__func__, __LINE__);
	}

	if(ctrl_port->chan_state == 1){
		loop_back[0]		= 0xFE;
		loop_back[1]		= 0;
		loop_back[2]		= 0;		
		loop_back[3]		= 6;		
		loop_back[4]		= 0x00;		
		loop_back[5]		= 0x06;		
		loop_back[6]		= 3;		
		loop_back[7]		= opt;
		loop_back[8]		= chan_num;	
		loop_back[9]		= 0;

		msg_len = 12;
		msg_len = (msg_len + 3) & ~0x03;	
		modem_sdio_write(port, 0x00, &(loop_back[0]), msg_len);
	}
	else{
		ret = -1;
		LOGPRT(LOG_ERR,  "%s: ctrl channel is off, please turn on first\n", __func__);
	}
	up(&modem->sem);
down_out:
	return ret;
}
EXPORT_SYMBOL(modem_loop_back_chan);

static int ctrl_msg_analyze(struct sdio_modem *modem)
{
	struct sdio_modem_ctrl_port *ctrl_port;
	const unsigned char msg_id_high = *modem->msg->buffer;
	const unsigned char msg_id_low = *(modem->msg->buffer + 1);
	const unsigned int msg_id = (msg_id_high << 8) + msg_id_low;
	unsigned char option = *(modem->msg->buffer + 3);
	struct sdio_modem_port *port = sdio_modem_port_get(0);
	unsigned char chan_num;

	ctrl_port = modem->ctrl_port;

	switch(msg_id)
	{
		case CHAN_ONOFF_MSG_ID:
			if (option == 1) {
				ctrl_port->chan_state = 1;
				LOGPRT(LOG_INFO,  "%s: ctrl channel is open\n", __func__);
			} else if (option == 0) {
				ctrl_port->chan_state = 0;
				LOGPRT(LOG_INFO,  "%s: ctrl channel is close\n", __func__);
			} else {
				LOGPRT(LOG_ERR,  "%s: err option value = %d\n",
				__func__, option);
			}
		case MDM_STATUS_IND_MSG_ID:
			if (option & 0x80) {
				port->dtr_state = 1;
			} else {
				port->dtr_state = 0;
			}
			break;
		case MDM_STATUS_QUERY_MSG_ID:
			if (option & 0x80) {
				port->dtr_state = 1;
			} else {
				port->dtr_state = 0;
			}
			
			break;
		case FLOW_CONTROL_MSG_ID:
			chan_num = *(modem->msg->buffer + 2);
			if (chan_num > 0 && chan_num < (SDIO_TTY_NR + 1)) {
				chan_num = chan_num - 1;
				port = modem->port[chan_num];
				if (option == 1) {
					LOGPRT(LOG_INFO,  "%s %d: channel%d soft flow ctrl enable!\n", __func__, __LINE__, (port->index + 1));
					atomic_set(&port->sflow_ctrl_state, SFLOW_CTRL_ENABLE);
				} else if (option == 0) {
					LOGPRT(LOG_INFO,  "%s %d: channel%d soft flow ctrl disable!\n", __func__, __LINE__, (port->index + 1));
					atomic_set(&port->sflow_ctrl_state, SFLOW_CTRL_DISABLE);
					wake_up(&port->sflow_ctrl_wait_q);
				}
			} else if (chan_num == 0) {
				if (option == 1) {
					LOGPRT(LOG_INFO,  "%s %d: ctrl channel soft flow ctrl enable!\n", __func__, __LINE__);
					atomic_set(&ctrl_port->sflow_ctrl_state, SFLOW_CTRL_ENABLE);
				} else if(option == 0) {
					LOGPRT(LOG_INFO,  "%s %d: ctrl channel soft flow ctrl disable!\n", __func__, __LINE__);
					atomic_set(&ctrl_port->sflow_ctrl_state, SFLOW_CTRL_DISABLE);
					wake_up(&ctrl_port->sflow_ctrl_wait_q);
				}
			} else {
				LOGPRT(LOG_ERR,  "%s %d: unkown channel num %d\n", __func__, __LINE__, chan_num);
			}
			break;
		case CHAN_LOOPBACK_TST_MSG_ID:
			{
				unsigned char chan_num = *(modem->msg->buffer + 4);
				unsigned char res = *(modem->msg->buffer + 5);
				if(option == OPT_LOOPBACK_OPEN)
				{
					LOGPRT(LOG_ERR,  "%s %d: open chan %d, result = %d\n",
									__func__, __LINE__,chan_num, res);
				}
				else if(option == OPT_LOOPBACK_CLOSE)
				{
					LOGPRT(LOG_ERR,  "%s %d: close chan %d, result = %d\n",
									__func__, __LINE__,chan_num, res);
				}
				else if(option == OPT_LOOPBACK_QUERY)
				{
					LOGPRT(LOG_ERR,  "%s %d: query chan %d, result = %d\n",
									__func__, __LINE__,chan_num, res);
				}
				else
				{
					LOGPRT(LOG_ERR,  "%s %d: unknow option %d\n", __func__, __LINE__, option);
				}
			}
			break;
		case CHAN_SWITCH_REQ_MSG_ID:
			
			break;
		case CHAN_STATUS_QUERY_MSG_ID:
			
			break;
		default:
			LOGPRT(LOG_ERR,  "%s %d: unknow control message received\n", __func__, __LINE__);
			goto err_wrong_format;
	}
	return 0;

err_wrong_format:
	return -1;
}

static int modem_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct sdio_modem_port *port;
	int idx = tty->index;

	LOGPRT(LOG_INFO, "%s %d: tty index %d\n", __func__, __LINE__, idx);

	port = sdio_modem_port_get(idx);

	if (!port) {
		tty->driver_data = NULL;
		LOGPRT(LOG_ERR,  "%s %d can't find sdio modem port.\n", __func__, __LINE__);
		return -ENODEV;
	}

	if (mutex_lock_interruptible(&port->mutex)) {
		LOGPRT(LOG_ERR,  "%s %d mutex lock return.\n", __func__, __LINE__);
		return -ERESTARTSYS;
	}

	++port->port.count;
	tty->driver_data = port;
	tty_port_tty_set(&port->port, tty);
	if (port->port.count == 1) {
	}

	mutex_unlock(&port->mutex);

	LOGPRT(LOG_INFO,  "%s %d: port %d.\n", __func__, __LINE__, port->index);

	return 0;
}

static void modem_tty_close(struct tty_struct *tty, struct file * filp)
{
	struct sdio_modem_port *port = tty->driver_data;

	pid_t pid = get_current()->pid;

	mutex_lock(&tty_remove_mutex);

	LOGPRT(LOG_INFO, "%s %d: tty index %d, pid = %d\n", __func__, __LINE__, tty->index, pid);

	if (!port) {
		LOGPRT(LOG_ERR,  "%s %d no port find.\n", __func__, __LINE__);
		mutex_unlock(&tty_remove_mutex);
		return;
	}

	LOGPRT(LOG_INFO,  "%s %d: port %d.\n", __func__, __LINE__, port->index);

	mutex_lock(&port->mutex);
	if (port->port.count == 0) {
		mutex_unlock(&port->mutex);
		mutex_unlock(&tty_remove_mutex);
		return;
	}

	if (port->port.count == 1) {
		struct tty_struct *tty = tty_port_tty_get(&port->port);
		if (tty) {
			if (tty->driver_data)
				tty->driver_data = NULL;
			tty_port_tty_set(&port->port, NULL);
			tty_kref_put(tty);
		}
	}

	--port->port.count;
	mutex_unlock(&port->mutex);

	LOGPRT(LOG_INFO,  "%s %d: port %d.\n", __func__, __LINE__, port->index);

	mutex_unlock(&tty_remove_mutex);
}

extern int ppp_flag;
static int modem_tty_write(struct tty_struct *tty, const unsigned char *buf,
		int count)
{
	struct sdio_modem_port *port = tty->driver_data;
	unsigned int copy = 0;
	unsigned long timeout = 0;
	int ret = 0;

	ret = check_port(port);
	if (ret < 0) {
		LOGPRT(LOG_ERR,  "%s %d check_port failed\n", __func__,__LINE__);
		return ret;
	}

	if (port->inception) {
		LOGPRT(LOG_INFO, "%s %d: port%d inception!\n",
					__func__,__LINE__, port->index);
		return -EBUSY;
	}

	if (count > port->transmit_fifo_size) {
		LOGPRT(LOG_ERR,  "%s %d FIFO size is not enough!\n", __func__,__LINE__);
		return -1;
	}

	copy = kfifo_in_locked(&port->transmit_fifo, buf, count, &port->write_lock);

#if CONFIG_DO_WAKE
	if (copy == 0) {
		if (!ppp_flag) {
			LOGPRT(LOG_DEBUG, "%s %d: port%d write buffer is full!\n",
						__func__,__LINE__, port->index);
			port->do_wake = true;
			timeout = wait_event_interruptible_timeout(port->wait_q,
						port->do_wake == false,
						msecs_to_jiffies(5000));
			if (!timeout) {
				LOGPRT(LOG_INFO, "%s %d: wait event timeout!!\n",
							__func__, __LINE__);
			} else
				copy = kfifo_in_locked(&port->transmit_fifo, buf, count, &port->write_lock);
			port->do_wake = false;
		} else {
			LOGPRT(LOG_INFO, "%s %d: port%d write buffer is full!\n",
						__func__,__LINE__, port->index);
			return -ENOMEM;
		}
	}
#endif
	queue_work(port->write_q, &port->write_work);

	LOGPRT(LOG_NOTICE, "%s %d: port%d count = %d copy = %d\n",
				__func__,__LINE__, port->index, count, copy);

	return copy;
}

static int modem_tty_write_room(struct tty_struct *tty)
{
	struct sdio_modem_port *port = tty->driver_data;
	unsigned long flags = 0;
	unsigned int data_len = 0;
	int ret;

	ret = check_port(port);
	if (ret < 0) {
		LOGPRT(LOG_ERR,  "%s %d check_port failed\n", __func__,__LINE__);
		return ret;
	}

	spin_lock_irqsave(&port->write_lock, flags);
	data_len = port->transmit_fifo_size - kfifo_len(&port->transmit_fifo);
	spin_unlock_irqrestore(&port->write_lock, flags);

	LOGPRT(LOG_NOTICE, "%s %d: port %d free size %d.\n",
				__func__, __LINE__, port->index, data_len);

	return data_len;
}

static int modem_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct sdio_modem_port *port = tty->driver_data;
	unsigned long flags = 0;
	unsigned int data_len = 0;
	int ret;

	ret = check_port(port);
	if (ret < 0) {
		LOGPRT(LOG_ERR,  "%s %d ret=%d\n", __func__,__LINE__,ret);
		return ret;
	}

	spin_lock_irqsave(&port->write_lock, flags);
	data_len = kfifo_len(&port->transmit_fifo);
	spin_unlock_irqrestore(&port->write_lock, flags);

	LOGPRT(LOG_NOTICE, "%s %d: port %d chars in buffer %d.\n",
				__func__, __LINE__, port->index, data_len);

	return data_len;
}

static void modem_tty_set_termios(struct tty_struct *tty,
		struct ktermios *old_termios)
{
	struct sdio_modem_port *port = tty->driver_data;
	int ret;

	ret = check_port(port);
	if (ret < 0) {
		LOGPRT(LOG_ERR,  "%s %d ret=%d\n", __func__,__LINE__,ret);
		return ;
	}

	tty_termios_copy_hw(tty->termios, old_termios);
}

static int modem_tty_tiocmget(struct tty_struct *tty)
{
	struct sdio_modem_port *port = tty->driver_data;
	int ret;

	ret = check_port(port);
	if (ret < 0) {
		LOGPRT(LOG_ERR,  "%s %d ret=%d\n", __func__,__LINE__,ret);
		return ret;
	}
	return 0;
}

static int modem_tty_tiocmset(struct tty_struct *tty,
		unsigned int set, unsigned int clear)
{
	struct sdio_modem_port *port = tty->driver_data;
	int ret;

	ret = check_port(port);
	if (ret < 0) {
		LOGPRT(LOG_ERR,  "%s %d ret=%d\n", __func__,__LINE__,ret);
		return ret;
	}
	return 0;
}

static const struct tty_operations modem_tty_ops = {
	.open			= modem_tty_open,
	.close			= modem_tty_close,
	.write			= modem_tty_write,
	.write_room		= modem_tty_write_room,
	.chars_in_buffer	= modem_tty_chars_in_buffer,
	.set_termios		= modem_tty_set_termios,
	.tiocmget		= modem_tty_tiocmget,
	.tiocmset		= modem_tty_tiocmset,
};

static void sdio_port_work_name(const char	*name, int index, char *p)
{
	sprintf(p, "%s%d", name, index);
}

static void sdio_write_port_work(struct work_struct *work)
{
	struct sdio_modem_port *port;
	struct sdio_modem *modem;
	unsigned int count;
	unsigned int left, todo;
	unsigned int write_len;
	unsigned int fifo_size;
	unsigned long flags = 0;
	unsigned long timeout = 0;
	int ret = 0;

	mutex_lock(&tty_remove_mutex);
	port = container_of(work, struct sdio_modem_port, write_work);
	modem = port->modem;

	if (!modem) {
		mutex_unlock(&tty_remove_mutex);
		goto down_out;
	}

	if (down_interruptible(&modem->sem)) {
		LOGPRT(LOG_ERR,  "%s %d down_interruptible failed.\n", __func__,__LINE__);
		ret =  -ERESTARTSYS;
		mutex_unlock(&tty_remove_mutex);
		goto down_out;
	}

	spin_lock_irqsave(&port->write_lock, flags);
	count = kfifo_len(&port->transmit_fifo);
	spin_unlock_irqrestore(&port->write_lock, flags);


	
	if (count == 0) {
		LOGPRT(LOG_INFO, "%s %d: port%d kfifo len is zero!\n", __func__, __LINE__, port->index);
		up(&modem->sem);
		mutex_unlock(&tty_remove_mutex);
		goto down_out;
	}

	left = count;
	do {
		timeout = wait_event_timeout(port->sflow_ctrl_wait_q,
					SFLOW_CTRL_DISABLE == atomic_read(&port->sflow_ctrl_state),
					msecs_to_jiffies(5000));
		if (!timeout) {
			LOGPRT(LOG_INFO, "%s %d: wait sflow ctrl off timeout!!\n",
						__func__, __LINE__);
		}

		todo = left;
		if (todo > TRANSMIT_MAX_SIZE - 1) {
			todo = TRANSMIT_MAX_SIZE;
		} else if (todo > 508) {
			todo = 508;
		}

		*modem->trans_buffer = 0xFE;
		*(modem->trans_buffer + 1) = 0x0F & (port->index + 1);
		*(modem->trans_buffer + 2) = 0x0F & (todo >> 8);
		*(modem->trans_buffer + 3) = 0xFF & todo;

		fifo_size = kfifo_out_locked(&port->transmit_fifo, modem->trans_buffer + 4, todo, &port->write_lock);
		if (fifo_size == 0) {
			LOGPRT(LOG_INFO, "%s %d: port%d fifo empty.\n", __func__, __LINE__, port->index);
			break;
		}

		if (todo != fifo_size) {
			LOGPRT(LOG_ERR,  "%s %d: port%d todo !=  kfifo lock out size.\n", __func__, __LINE__,port->index);
			todo = fifo_size;
		}

#if CONFIG_DO_WAKE
		if (port->do_wake) {
			LOGPRT(LOG_DEBUG, "%s %d: port = %d do wake up!\n", __func__, __LINE__, port->index);
			port->do_wake = false;
			wake_up_interruptible(&port->wait_q);
		}
#endif

		write_len = (todo + 4 + 3) & ~0x03;  
		modem_sdio_write(port, 0x00, modem->trans_buffer, write_len);
		left -= todo;
	} while (left);

	up(&modem->sem);
	mutex_unlock(&tty_remove_mutex);

down_out:
	
	ret = ret;
}

static int modem_irq_query(struct sdio_func *func,unsigned char *pendingirq)
{
	int func_num = 0;
	int ret = 0;

	func_num = func->num;
	func->num = 0;

	*pendingirq = sdio_readb(func,SDIO_CCCR_INTx, &ret);

	func->num = func_num;

	return ret;
}

static void modem_sdio_irq(struct sdio_func *func)
{
	struct sdio_modem *modem;
	struct sdio_modem_port *port;
	unsigned char reg = 0;
	int  bytecnt = 0;
	int ret = 0;
	int iir =0;
	int readcnt = 0;
	struct tty_struct *tty;
	unsigned char index = 0;
	unsigned char payload_offset = 0;
	int todo = 0;
	int left = 0;
	int read_times = 0;
	int req_size = 0;
	int todo_size, left_size, req_offset;
	int req_buf_times = 0;
	unsigned char pending = 0;


#if 0
	if (gpio_get_value(GPIO67_GPIO) == 0) {
		LOGPRT(LOG_INFO, "%s %d: error sdio irq!!\n", __func__, __LINE__);
		dump_asc_tx_rx_state();
	}
#endif

	if (is_host_suspend) {
		LOGPRT(LOG_INFO, "%s %d: host is suspend!!\n",
				__func__, __LINE__);
		dump_asc_tx_rx_state();
	}

	ret = modem_irq_query(func,&pending);
	if(ret) {
		LOGPRT(LOG_ERR, "read SDIO_CCCR_INTx err ret = %d\n",ret);
	}
	if ((pending & BIT(1)) == 0) {
		LOGPRT(LOG_ERR, "pending = %d ret = %d\n",pending,ret);
		goto out;
	}
#if 0
retry_rd_intx:
	pending = sdio_read_intx(func, SDIO_CCCR_INTx, &ret);
	if (ret) {
		LOGPRT(LOG_ERR, "%s %d: error reading SDIO_CCCR_INTx\n", __func__, __LINE__);
		goto out;
	} else {
		if (pending != (1 << 1)) {
			if (retry_cnt++ < 2)
				goto retry_rd_intx;
			else
				goto out;
		}
	}
#endif

	sdio_rx_cnt++;
	modem = sdio_get_drvdata(func);
	do
	{
		iir =  sdio_readb(func, 0x04, &ret);
	} while ((iir != 1) && (readcnt++ <= 20));

	if ((iir != 1))
	{
		LOGPRT(LOG_ERR,  "%s %d error iir value = %d!!!\n", __func__,__LINE__,iir);
		goto out;
	}

	
	reg =  sdio_readb(func, 0x08, &ret);
	bytecnt = reg;
	reg =  sdio_readb(func, 0x09, &ret);
	bytecnt |= (reg << 8);

	if (bytecnt == 0) {
		LOGPRT(LOG_ERR,  "%s %d error read size %d.\n", __func__,__LINE__, bytecnt);
		goto out;
	}
	modem->msg->head.start_flag = 0;
	modem->msg->head.chanInfo = 0;
	modem->msg->head.tranHi = 0;
	modem->msg->head.tranLow = 0;
	memset(modem->msg->buffer, 0, sizeof(modem->msg->buffer));

	if (modem->cbp_data->data_ack_enable) {
		atomic_set(&modem->cbp_data->cbp_data_ack->state, MODEM_ST_TX_RX);
	}

	if (bytecnt > 1024) {
		todo = 1024;
		bytecnt -= 1024;
	} else {
		todo = bytecnt;
		bytecnt = 0;
	}
	ret = sdio_readsb(func, modem->msg, 0x00, todo);
	if (ret) {
		LOGPRT(LOG_ERR,  "%s %d: port%d sdio read with error code = %d, read bytecount = %d!!!\n",
			__func__, __LINE__, modem->msg->head.chanInfo, ret, todo);

		if (modem->cbp_data->data_ack_enable) {
			modem->cbp_data->data_ack_wait_event(modem->cbp_data->cbp_data_ack);
		}

		goto out;
	}

	left = bytecnt;
	while (left) {
		todo = left;
		if (todo > 1024) {
			todo = 1024;
		}
		udelay(100);
		ret = sdio_readsb(func, modem->msg->buffer + 1020 + 1024 * read_times, 0x00, todo);
		if (ret) {
			LOGPRT(LOG_ERR,  "%s %d: port%d sdio read with error code = %d, read bytecount = %d!!!\n",
				__func__, __LINE__, modem->msg->head.chanInfo, ret, todo);
			goto out;
		}
		read_times++;
		left -= todo;
	}

	if (sdio_log_level > LOG_NOTICE) {
		sdio_tx_rx_printk((const unsigned char *) modem->msg, 0);
	}

	if (modem->msg->head.start_flag != 0xFE) {
		LOGPRT(LOG_ERR,  "%s %d: start_flag != 0xFE and value is 0x%x, go out.\n",
			__func__, __LINE__, modem->msg->head.start_flag);
		sdio_rx_dump((const unsigned char *) modem->msg);
		goto out;
	}

	if (host_js == true) {
		host_js = false;
		LOGPRT(LOG_INFO, "Cbp Rx chanId %d\n",
				modem->msg->head.chanInfo);
	}

	if (modem->msg->head.chanInfo > 0 && modem->msg->head.chanInfo < (SDIO_TTY_NR + 1))
	{
		index = modem->msg->head.chanInfo - 1;
		payload_offset = ((modem->msg->head.tranHi & 0xC0) >> 6);
		if (payload_offset) {
			LOGPRT(LOG_NOTICE,  "%s %d: payload_offset = %d.\n",__func__, __LINE__, payload_offset);
		}
		modem->data_length =(((modem->msg->head.tranHi & 0x0F) << 8) |
							(modem->msg->head.tranLow & 0xFF));
		if (modem->data_length == 0) {
			LOGPRT(LOG_ERR,  "%s %d: data_length is 0\n",__func__,__LINE__);
			goto out;
		}

		if (payload_offset >= modem->data_length) {
			LOGPRT(LOG_ERR,  "%s %d: payload_offset %d >= data length %d!\n",
						__func__,__LINE__, payload_offset, modem->data_length);
			goto out;
		}

		port = modem->port[index];
		ret = check_port(port);
		if (ret < 0) {
			LOGPRT(LOG_ERR,  "%s %d: check port error\n",__func__,__LINE__);
			goto out;
		}
		tty = tty_port_tty_get(&port->port);
		if (!tty) {
			LOGPRT(LOG_NOTICE,  "%s %d: tty is NULL, channel info = %d \n", __func__,__LINE__, index);
		}

		if (tty && modem->data_length) {
			todo_size = left_size = req_offset = 0;

			modem->data_length -= payload_offset;

			req_size = tty_buffer_request_room(tty, modem->data_length);
			if (req_size != 0) {
				if (req_size != modem->data_length)
					LOGPRT(LOG_INFO, "%s %d: request tty buffer failed,"
							"need %d, actually request %d!\n",
							__func__, __LINE__, modem->data_length, req_size);
				tty_insert_flip_string(tty, modem->msg->buffer + payload_offset, req_size);
				tty_flip_buffer_push(tty);
			}

			req_offset += req_size;
			left_size = modem->data_length - req_size;
			while (left_size && req_buf_times++ < 2) {
				LOGPRT(LOG_INFO, "try to request tty buffer left, channel info %d !!\n", index);
				todo_size = left_size;
				if (todo_size > 1536) {
					todo_size = 1536;
				}
				req_size = tty_buffer_request_room(tty, todo_size);
				if (req_size != 0) {
					if (req_size != todo_size)
						LOGPRT(LOG_INFO, "%s %d request 1.5K tty buffer failed,"
								"need %d, actually request %d!\n",
								__func__, __LINE__, todo_size, req_size);
					tty_insert_flip_string(tty, modem->msg->buffer + payload_offset +
								req_offset, req_size);
					tty_flip_buffer_push(tty);
					left_size -= req_size;
					req_offset += req_size;
				} else
					msleep(200);
			}
			if (left_size)
				LOGPRT(LOG_ERR, "%s %d request tty buffer finally failed, channel info %d !!\n",
						__func__, __LINE__, index);
		} else {
			LOGPRT(LOG_NOTICE,  "%s %d: empty read sdio received\n", __func__,__LINE__);
		}
		tty_kref_put(tty);
	} else if (modem->msg->head.chanInfo ==0) {
		ctrl_msg_analyze(modem);
	} else {
		LOGPRT(LOG_ERR,  "%s %d: error chanInfo is %d, go out.\n",
			__func__, __LINE__, modem->msg->head.chanInfo);
		goto out;
	}

	if (modem->cbp_data->data_ack_enable) {
		modem->cbp_data->data_ack_wait_event(modem->cbp_data->cbp_data_ack);
	}
out:
	return;
}

static int func_enable_irq(struct sdio_func *func, int enable)
{
	int func_num = 0;
	u8 cccr = 0;
	int ret = 0;

	
	func_num = func->num;
	func->num = 0;

	cccr = sdio_readb(func, SDIO_CCCR_IENx, &ret);
	if (WARN_ON(ret))
		goto set_func;

	if (enable) {
		
		cccr |= BIT(0);
		
		cccr |= BIT(func_num);
	} else {
		
		cccr &= ~(BIT(0));
		
		cccr &= ~(BIT(func_num));
	}

	sdio_writeb(func, cccr, SDIO_CCCR_IENx, &ret);
	if (WARN_ON(ret))
		goto set_func;

	
	func->num = func_num;
	return 0;

set_func:
	func->num = func_num;
	return ret;
}

static void modem_sdio_write(struct sdio_modem_port *port, int addr,
		void *buf, size_t len)
{
	struct sdio_func *func = port->func;
	struct sdio_modem *modem = port->modem;
	struct mmc_host *host = func->card->host;
	int ret;
	unsigned long timeout = -1;

	sdio_tx_cnt++;
	if (modem->cbp_data->flow_ctrl_enable) {
		modem->cbp_data->flow_ctrl_wait_event(modem->cbp_data->cbp_flow_ctrl);
	}

	if (modem->cbp_data->tx_disable_irq) {
		if (host->caps & MMC_CAP_SDIO_IRQ)
			host->ops->enable_sdio_irq(host, 0);
	}

	sdio_in_use = true;

	if (modem->cbp_data->ipc_enable) {
		ret = asc_tx_auto_ready(modem->cbp_data->tx_handle->name, 1);
		if (ret) {
			dump_asc_tx_rx_state();
		}
	}

	sdio_claim_host(func);

	if (is_host_suspend) {
		LOGPRT(LOG_INFO, "%s %d host is suspend!!\n",
				__func__, __LINE__);
		timeout = wait_event_interruptible_timeout(wr_wait_q,
					is_host_suspend == false,
					msecs_to_jiffies(1000));
		if (!timeout) {
			LOGPRT(LOG_INFO, "%s %d: wait event timeout!!\n",
						__func__, __LINE__);
		}
	}

	if (modem->cbp_data->tx_disable_irq) {
		ret = func_enable_irq(func, 0);
		if (ret) {
			LOGPRT(LOG_ERR,  "%s %d: port%d func_disable_irq failed ret=%d\n", __func__, __LINE__, port->index,ret);
		}
	}

	if (modem->cbp_data->data_ack_enable) {
		atomic_set(&modem->cbp_data->cbp_data_ack->state, MODEM_ST_TX_RX);
	}

	if (sdio_log_level > LOG_NOTICE) {
		sdio_tx_rx_printk(buf, 1);
	}

	ret = sdio_writesb(func, addr, buf, len);
	if (ret) {
		LOGPRT(LOG_ERR,  "%s %d: port%d failed ret=%d\n", __func__, __LINE__, port->index,ret);
		goto out;
	}

	if (modem->cbp_data->data_ack_enable) {
		modem->cbp_data->data_ack_wait_event(modem->cbp_data->cbp_data_ack);
	}

out:
	if (modem->cbp_data->tx_disable_irq) {
		func_enable_irq(func, 1);
		if (ret) {
			LOGPRT(LOG_ERR,  "%s %d: port%d func_enable_irq failed ret=%d\n", __func__, __LINE__, port->index,ret);
		}
	}

	sdio_release_host(func);
	if (modem->cbp_data->tx_disable_irq) {
		if (host->caps & MMC_CAP_SDIO_IRQ)
			host->ops->enable_sdio_irq(host, true);
	}

	sdio_in_use = false;

}

static void sdio_modem_remove(struct sdio_modem *modem)
{
	struct sdio_modem_port *port;
	struct sdio_func *func;
	struct tty_struct *tty;
	int index;

	func = modem->func;
	modem->func = NULL;

	for (index = 0; index < SDIO_TTY_NR; index++) {
		port = modem->port[index];
		tty = tty_port_tty_get(&port->port);
		
		if (tty) {
			tty_hangup(tty);
			if (tty->driver_data)
				tty->driver_data = NULL;
			tty_port_tty_set(&port->port, NULL);
			tty_kref_put(tty);
		}
	}
}

static int sdio_modem_port_init(struct sdio_modem_port *port, int index)
{
	int ret = -EBUSY;

	kref_init(&port->kref);
	mutex_init(&port->mutex);
	spin_lock_init(&port->write_lock);

	if (index == 0)
		port->transmit_fifo_size = FIFO_SIZE * 4;
	else
		port->transmit_fifo_size = FIFO_SIZE;

	if (kfifo_alloc(&port->transmit_fifo, port->transmit_fifo_size, GFP_KERNEL)) {
		LOGPRT(LOG_ERR,  "%s %d : Couldn't allocate transmit_fifo\n",__func__,__LINE__);
		return -ENOMEM;
	}

	
	port->name = "modem_sdio_write_wq";
	sdio_port_work_name(port->name, index, port->work_name);
	port->write_q = create_singlethread_workqueue(port->work_name);
	if (port->write_q == NULL) {
		LOGPRT(LOG_ERR,  "%s %d error creat write workqueue \n",__func__, __LINE__);
		return -ENOMEM;
	}
	INIT_WORK(&port->write_work, sdio_write_port_work);
	init_waitqueue_head(&port->wait_q);
	port->do_wake = false;

	init_waitqueue_head(&port->sflow_ctrl_wait_q);
	atomic_set(&port->sflow_ctrl_state, SFLOW_CTRL_DISABLE);

	spin_lock(&sdio_modem_table_lock);
	if (!sdio_modem_table[index]) {
		port->index = index;
		sdio_modem_table[index] = port;
		ret = 0;
	}
	spin_unlock(&sdio_modem_table_lock);

	return ret;
}

static void sdio_modem_port_exit(struct sdio_modem *modem)
{
	struct sdio_modem_port *port;
	int index;

	for (index = 0; index < SDIO_TTY_NR; index++) {
		port = modem->port[index];
		if (port->write_q) {
			destroy_workqueue(port->write_q);
			kfifo_free(&port->transmit_fifo);
		}
		spin_lock(&sdio_modem_table_lock);
		sdio_modem_table[index] = NULL;
		spin_unlock(&sdio_modem_table_lock);
	}
}

static ssize_t modem_log_level_show(struct kobject *kobj, struct kobj_attribute *attr,
					char *buf)
{
	char *s = buf;
	s += sprintf(s, "%d\n", sdio_log_level);

	return (s - buf);
}

static ssize_t modem_log_level_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;

	sdio_log_level = val;

	return n;
}

static ssize_t modem_refer_show(struct kobject *kobj, struct kobj_attribute *attr,
				 char *buf)
{
	char *s = buf;
	s += sprintf(s, "Tx:  times %d\n", sdio_tx_cnt);
	s += sprintf(s, "\n");
	s += sprintf(s, "Rx:  times %d\n", sdio_rx_cnt);

	return (s - buf);
}

static ssize_t modem_refer_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	return n;
}

static ssize_t modem_ctrl_on_show(struct kobject *kobj, struct kobj_attribute *attr,
					char *buf)
{
	return 0;
}

static ssize_t modem_ctrl_on_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;

	if (val) {
		modem_on_off_ctrl_chan(1);
	} else {
		modem_on_off_ctrl_chan(0);
	}
	return n;
}

static ssize_t modem_dtr_send_show(struct kobject *kobj, struct kobj_attribute *attr,
				 char *buf)
{
	return 0;
}

static ssize_t modem_dtr_send_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;

	if (val) {
		modem_dtr_send(true);
	} else {
		modem_dtr_send(false);
	}
	return n;
}

static ssize_t modem_dtr_query_show(struct kobject *kobj, struct kobj_attribute *attr,
				 char *buf)
{
	return 0;
}

static ssize_t modem_dtr_query_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;
	int data;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;

	if (val) {
		modem_dtr_query(&data, 1);
	} else {
		modem_dtr_query(&data, 0);
	}
	return n;
}

static unsigned char loop_back_chan = 0;
static ssize_t modem_loop_back_chan_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return 0;
}

static ssize_t modem_loop_back_chan_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;
	if (val <= 6) {
		loop_back_chan = val;
	} else {
		LOGPRT(LOG_ERR, "%s %d val = %ld\n", __func__, __LINE__, val);
	}

	return n;
}

static ssize_t modem_loop_back_mod_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return 0;
}

static ssize_t modem_loop_back_mod_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;

	if (val < 4) {
		modem_loop_back_chan(loop_back_chan, val);
	} else {
		LOGPRT(LOG_ERR, "%s %d val = %ld\n", __func__, __LINE__, val);
	}
	return n;
}

#define modem_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0664,			\
	},					\
	.show	= modem_##_name##_show,			\
	.store	= modem_##_name##_store,		\
}
modem_attr(log_level);
modem_attr(refer);
modem_attr(ctrl_on);
modem_attr(dtr_send);
modem_attr(dtr_query);
modem_attr(loop_back_chan);
modem_attr(loop_back_mod);


static struct attribute * modem_sdio_attr[] = {
	&log_level_attr.attr,
	&refer_attr.attr,
	&ctrl_on_attr.attr,
	&dtr_send_attr.attr,
	&dtr_query_attr.attr,
	&loop_back_chan_attr.attr,
	&loop_back_mod_attr.attr,
	NULL,
};

static struct kobject *modem_sdio_kobj;
static struct attribute_group g_modem_attr_group = {
	.attrs =modem_sdio_attr,
};

static ssize_t show_inception(struct device *dev, struct device_attribute
		*attr, char *buf) {

	struct sdio_modem_port *port = container_of(dev,struct sdio_modem_port,dev);
	int inception;

	spin_lock(&port->inception_lock);
	inception = port->inception;
	spin_unlock(&port->inception_lock);

	return sprintf(buf, "%d", inception);
}

static ssize_t store_inception(struct device *dev, struct device_attribute
		*attr, const char *buf, size_t count) {

	struct sdio_modem_port *port = container_of(dev,struct sdio_modem_port,dev);

	int inception;

	inception = simple_strtoul(buf, NULL, 10);
	LOGPRT(LOG_INFO, "modem inception = %d\n", inception);
	spin_lock(&port->inception_lock);
	if ((!!inception) == port->inception) {
		spin_unlock(&port->inception_lock);
		return count;
	}
	spin_unlock(&port->inception_lock);

	spin_lock(&port->inception_lock);
	if (inception != port->inception)
		  port->inception = !!inception;
	spin_unlock(&port->inception_lock);

	return count;
}

DEVICE_ATTR(inception, 0664, show_inception, store_inception);
#ifdef CONFIG_PM_RUNTIME
void cbp_runtime(int enable)
{
	int ret = 0;
	struct sdio_func *func_rt;

	if (enable)
		wait_event(rt_wait_q, is_rt_suspend == false);

	mutex_lock(&tty_remove_mutex);

	if (!modem_rt) {
		LOGPRT(LOG_INFO,  "Not Initialized\n");
		mutex_unlock(&tty_remove_mutex);
		return;
	}
	func_rt = modem_rt->func;

	if (enable && !modem_rt->runtime) {
		modem_rt->runtime = true;
		ret = pm_runtime_get_sync(&func_rt->dev);
		LOGPRT(LOG_INFO, "Runtime get %d,%d,%d\n",pm_runtime_enabled(&func_rt->dev),pm_runtime_suspended(&func_rt->dev),ret);
	} else if (!enable) {
		if (!modem_rt->runtime) {
			if (!rt_unbal) {
				LOGPRT(LOG_INFO, "Runtime unbalanced\n");
				rt_unbal = true;
			} else {
				LOGPRT(LOG_INFO, "Runtime unbalanced!\n");
				mutex_unlock(&tty_remove_mutex);
				return;
			}
		}

		modem_rt->runtime = false;
		pm_runtime_mark_last_busy(&func_rt->dev);
		ret = pm_runtime_put_autosuspend(&func_rt->dev);
		LOGPRT(LOG_INFO, "Runtime put %d,%d,%d\n",pm_runtime_enabled(&func_rt->dev),pm_runtime_suspended(&func_rt->dev),ret);
	}
	mutex_unlock(&tty_remove_mutex);
}
EXPORT_SYMBOL(cbp_runtime);
#endif

static int __devinit cbp_rt_probe(struct platform_device *pdev)
{
	LOGPRT(LOG_INFO,  "%s\n",__func__);
	is_rt_suspend = false;
	rt_unbal = false;
	return 0;
}
static int __devexit cbp_rt_remove(struct platform_device *pdev)
{
	LOGPRT(LOG_INFO,  "%s\n",__func__);
	is_rt_suspend = false;
	rt_unbal = false;
	wake_up(&rt_wait_q);
	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int cbp_rt_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	is_rt_suspend = true;
	LOGPRT(LOG_INFO,  "%s\n",__func__);
	return ret;
}

static int cbp_rt_resume(struct platform_device *pdev)
{
	is_rt_suspend = false;
	wake_up(&rt_wait_q);
	LOGPRT(LOG_INFO,  "%s\n",__func__);
	return 0;
}
#endif
#else
#define cbp_rt_suspend	NULL
#define cbp_rt_resume	NULL
#endif

static struct platform_driver cbp_rt_driver = {
	.driver = {
		.name = CBP_RT_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= cbp_rt_probe,
	.suspend	= cbp_rt_suspend,
	.resume		= cbp_rt_resume,
	.remove		= __devexit_p(cbp_rt_remove),
};

static struct device_driver modem_port_driver = {
	.name		= "modem_port",
	.owner		= THIS_MODULE,
};

#if 0
int sdio_buffer_push(int port_num, void *buf, int count)
{
	int ret, data_len;
	unsigned long flags;
	
	struct sdio_modem_port *port = sdio_modem_port_get(port_num);

	ret = check_port(port);
	if (ret < 0){
		LOGPRT(LOG_ERR,  "%s %d failed\n", __FUNCTION__,__LINE__);
		return ret;
	}
	spin_lock_irqsave(&port->write_lock, flags);
	data_len = FIFO_SIZE - kfifo_len(&port->transmit_fifo);
	spin_unlock_irqrestore(&port->write_lock, flags);
	if(data_len <= 512)
		return -ENOMEM;
	ret = kfifo_in_locked(&port->transmit_fifo, buf, count, &port->write_lock);
	queue_work(port->write_q, &port->write_work);

	return 0;
}
EXPORT_SYMBOL_GPL(sdio_buffer_push);
#endif

static void sdio_port_release(struct device *dev)
{

}

#ifdef TTY_PORT_KEEP_EXIST
static int sdio_tty_init_bypass(void)
{
	int index = 0;
	spin_lock(&sdio_modem_table_lock);
	for (index = 0; index < SDIO_TTY_NR; index++) {
		if(sdio_modem_table[index] !=NULL ){
			sdio_modem_table[index]->inception = true;
#ifdef TTY_PORT_GPS_BLOCK
			if (index == GPS_PORT_INDEX)
				LOGPRT(LOG_INFO,  "GPS port inception\n");
#endif
		} else {
			LOGPRT(LOG_ERR,  "check port why is release\n");
		}
	}
	spin_unlock(&sdio_modem_table_lock);
	return 0;
}

static int sdio_tty_init(struct sdio_func *func, struct sdio_modem *modem)
{
	static struct sdio_modem_port *port = NULL;
	int ret = 0;
	int index = 0;
	unsigned long flags = 0;

	if (tty_port_initial == 1){
		spin_lock(&sdio_modem_table_lock);
		for (index = 0; index < SDIO_TTY_NR; index++){
			sdio_modem_table[index]->func = func;
			sdio_modem_table[index]->modem = modem;
#ifdef TTY_PORT_GPS_BLOCK
			if (index != GPS_PORT_INDEX)
#endif
				sdio_modem_table[index]->inception = false;
			modem->port[index] = sdio_modem_table[index];
		}
		spin_unlock(&sdio_modem_table_lock);
		for (index = 0; index < SDIO_TTY_NR; index++){
			port = modem->port[index];
			spin_lock_irqsave(&port->write_lock, flags);
			kfifo_reset_out(&port->transmit_fifo);
			spin_unlock_irqrestore(&port->write_lock, flags);
		}
		goto done;
	}

	tty_port_initial = 1;
	for (index = 0; index < SDIO_TTY_NR; index++) {
		port = kzalloc(sizeof(struct sdio_modem_port), GFP_KERNEL);
		if (!port)
		{
			LOGPRT(LOG_ERR,  "%s %d kzalloc sdio_modem_port %d failed.\n",
				__func__, __LINE__, index);
			ret = -ENOMEM;
			goto err_kazlloc_sdio_modem_port;
		}
		tty_port_init(&port->port);
		port->func = func;
		port->modem = modem;
		modem->port[index] = port;
		spin_lock_init(&port->inception_lock);
		port->inception = false;
	}

	for (index = 0; index < SDIO_TTY_NR; index++)
	{
		port = modem->port[index];
		ret = sdio_modem_port_init(port, index);
		if (ret) {
			LOGPRT(LOG_ERR,  "%s %d sdio add port failed.\n",__func__, __LINE__);
			goto err_sdio_modem_port_init;
		} else {
			struct device *dev;
			int rc;
			port->dev.parent = NULL;
			port->dev.driver = &modem_port_driver;
			port->dev.release = sdio_port_release;
			dev_set_drvdata(&port->dev, port);
			dev_set_name(&port->dev, "%s", port_name(index));
			rc = device_register(&port->dev);
			if (rc < 0)
				goto err_tty_register_device;
			else {
				ret = device_create_file(&port->dev, &dev_attr_inception);
				if (ret < 0)
				goto err_tty_register_device;
			}
			dev = tty_register_device(modem_sdio_tty_driver,
					port->index, &port->dev);
			if (IS_ERR(dev)) {
				ret = PTR_ERR(dev);
				LOGPRT(LOG_ERR,  "%s %d tty register failed \n",__func__,__LINE__);
				goto err_tty_register_device;
			}
		}
	}

done:
	return ret;

err_tty_register_device:
	sdio_modem_remove(modem);
err_sdio_modem_port_init:
	sdio_modem_port_exit(modem);
err_kazlloc_sdio_modem_port:
	for (index = 0; index < SDIO_TTY_NR; index++) {
		port = modem->port[index];
		if (port) {
			kfree(port);
		}
	}
	return ret;
}
#endif

static int modem_sdio_probe(struct sdio_func *func,
		const struct sdio_device_id *id)
{
	struct sdio_modem *modem = NULL;
#ifndef TTY_PORT_KEEP_EXIST
	struct sdio_modem_port *port = NULL;
	int index = 0;
#endif
	int ret = 0;

	LOGPRT(LOG_INFO,  "%s %d: enter.\n", __func__, __LINE__);
	modem = kzalloc(sizeof(struct sdio_modem), GFP_KERNEL);
	if (!modem) {
		LOGPRT(LOG_ERR,  "%s %d kzalloc sdio_modem failed.\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto err_kzalloc_sdio_modem;
	}

	modem->ctrl_port = kzalloc(sizeof(struct sdio_modem_ctrl_port), GFP_KERNEL);
	if (!modem->ctrl_port) {
		LOGPRT(LOG_ERR,  "%s %d kzalloc ctrl_port failed \n",__func__, __LINE__);
		ret =  -ENOMEM;
		goto err_kzalloc_ctrl_port;
	}

	modem->msg = kzalloc(sizeof(struct sdio_msg), GFP_KERNEL);
	if (!modem->msg) {
		LOGPRT(LOG_ERR,  "%s %d kzalloc sdio_msg failed \n",__func__, __LINE__);
		ret = -ENOMEM;
		goto err_kzalloc_sdio_msg;
	}

	modem->trans_buffer = kzalloc(TRANSMIT_BUFFER_SIZE, GFP_KERNEL);
	if (!modem->trans_buffer) {
		LOGPRT(LOG_ERR,  "%s %d kzalloc trans_buffer failed \n",__func__, __LINE__);
		ret =  -ENOMEM;
		goto err_kzalloc_trans_buffer;
	}

	modem->func = func;
	sema_init(&modem->sem, 1);
	sdio_set_drvdata(func, modem);
	modem->cbp_data = cbp_pdata;
	modem->ctrl_port->chan_state = 0;

	init_waitqueue_head(&modem->ctrl_port->sflow_ctrl_wait_q);
	atomic_set(&modem->ctrl_port->sflow_ctrl_state, SFLOW_CTRL_DISABLE);

	sdio_claim_host(func);
	ret = sdio_enable_func(func);
	if (ret) {
		LOGPRT(LOG_ERR,  "%s %d sdio enable func failed with ret = %d\n",__func__, __LINE__, ret);
		goto err_enable_func;
	}

	ret = sdio_set_block_size(func, 1024);
	if (ret) {
		LOGPRT(LOG_ERR,  "%s %d: set block size failed with ret = %d\n",__func__, __LINE__, ret);
		goto error_set_block_size;
	}

	sdio_writeb(func, 0x01, 0x28, &ret);
	if (ret) {
		LOGPRT(LOG_ERR,  "%s %d: sdio_writeb 0x28 failed with ret = %d\n",__func__, __LINE__, ret);
		goto error_set_block_size;
	}

#ifdef TTY_PORT_KEEP_EXIST
	sdio_tty_init(func, modem);
#else
	for (index = 0; index < SDIO_TTY_NR; index++) {
		port = kzalloc(sizeof(struct sdio_modem_port), GFP_KERNEL);
		if (!port)
		{
			LOGPRT(LOG_ERR,  "%s %d kzalloc sdio_modem_port %d failed.\n",
				__func__, __LINE__, index);
			ret = -ENOMEM;
			goto err_kazlloc_sdio_modem_port;
		}

		tty_port_init(&port->port);
		port->func = func;
		port->modem = modem;
		modem->port[index] = port;
		spin_lock_init(&port->inception_lock);
		port->inception = false;
	}

	for (index = 0; index < SDIO_TTY_NR; index++)
	{
		port = modem->port[index];
		ret = sdio_modem_port_init(port, index);
		if (ret) {
			LOGPRT(LOG_ERR,  "%s %d sdio add port failed.\n",__func__, __LINE__);
			goto err_sdio_modem_port_init;
		} else {
			struct device *dev;
			int rc;
			port->dev.parent = &func->dev;
			port->dev.driver = &modem_port_driver;
			port->dev.release = sdio_port_release;
			dev_set_drvdata(&port->dev, port);
			dev_set_name(&port->dev, "%s", port_name(index));
			rc = device_register(&port->dev);
			if (rc < 0)
				goto err_tty_register_device;
			else {
				ret = device_create_file(&port->dev, &dev_attr_inception);
				if (ret < 0)
					goto err_tty_register_device;
			}
			dev = tty_register_device(modem_sdio_tty_driver,
					port->index, &func->dev);
			if (IS_ERR(dev)) {
				ret = PTR_ERR(dev);
				LOGPRT(LOG_ERR,  "%s %d tty register failed \n",__func__,__LINE__);
				goto err_tty_register_device;
			}
		}
	}
#endif

	ret = sdio_claim_irq(func, modem_sdio_irq);
	if (ret) {
		LOGPRT(LOG_ERR,  "%s %d sdio claim irq failed.\n",__func__,__LINE__);
		goto err_tty_register_device;
	}

	modem_sdio_kobj = kobject_create_and_add("modem_sdio", NULL);
	if (!modem_sdio_kobj) {
		ret = -ENOMEM;
		goto err_create_kobj;
	}

	LOGPRT(LOG_NOTICE,  "%s %d: exit.\n", __func__, __LINE__);

#ifdef CONFIG_PM_RUNTIME
	if (platform_driver_register(&cbp_rt_driver)) {
		LOGPRT(LOG_ERR, "CBP rt driver register fail\n");
		ret = -ENODEV;
		goto err_reg_rt_drv;
	}

	cbp_rt_device = platform_device_alloc(CBP_RT_DRIVER_NAME, -1);
	if (!cbp_rt_device) {
		LOGPRT(LOG_ERR, "CBP rt dev alloc fail\n");
		ret = -ENOMEM;
		goto err_cbp_rt_driver;
	}

	ret = platform_device_add(cbp_rt_device);
	if (ret) {
		LOGPRT(LOG_ERR, "CBP rt dev add fail\n");
		ret = -ENODEV;
		goto err_cbp_rt_device;
	}
	modem->runtime = false;
	modem_rt = modem;
	pm_suspend_ignore_children(&func->dev, true);
	pm_runtime_set_autosuspend_delay(&func->dev, 200);
	pm_runtime_use_autosuspend(&func->dev);
	pm_runtime_resume(&func->dev);
#endif
	sdio_release_host(func);

	return sysfs_create_group(modem_sdio_kobj, &g_modem_attr_group);;
#ifdef CONFIG_PM_RUNTIME
err_cbp_rt_device:
	platform_device_put(cbp_rt_device);
err_cbp_rt_driver:
	platform_driver_unregister(&cbp_rt_driver);
err_reg_rt_drv:
	sysfs_remove_group(modem_sdio_kobj, &g_modem_attr_group);
	kobject_put(modem_sdio_kobj);
#endif
err_create_kobj:
	sdio_release_irq(func);
err_tty_register_device:
	sdio_modem_remove(modem);
#ifndef TTY_PORT_KEEP_EXIST
err_sdio_modem_port_init:
	sdio_modem_port_exit(modem);
err_kazlloc_sdio_modem_port:
	for (index = 0; index < SDIO_TTY_NR; index++) {
		port = modem->port[index];
		if (port) {
			kfree(port);
		}
	}
#endif
error_set_block_size:
	sdio_disable_func(func);
err_enable_func:
	sdio_release_host(func);
	kfree(modem->trans_buffer);
err_kzalloc_trans_buffer:
	kfree(modem->msg);
err_kzalloc_sdio_msg:
	kfree(modem->ctrl_port);
err_kzalloc_ctrl_port:
	kfree(modem);
err_kzalloc_sdio_modem:
	return ret;
}

#ifdef TTY_PORT_KEEP_EXIST
static void sdio_modem_port_exit_ext(void)
{
	struct sdio_modem_port *port;
	int index;

	for (index = 0; index < SDIO_TTY_NR; index++) {
		port = sdio_modem_table[index];
		if (port->write_q) {
			destroy_workqueue(port->write_q);
			kfifo_free(&port->transmit_fifo);
		}
		spin_lock(&sdio_modem_table_lock);
		if(port)
			kfree(port);
		sdio_modem_table[index] = NULL;
		spin_unlock(&sdio_modem_table_lock);
	}
}

static void sdio_modem_remove_ext(void)
{
	struct sdio_modem_port *port;
	struct tty_struct *tty;
	int index;

	LOGPRT(LOG_INFO,  "%s %d : sdio tty is remove\n",__func__,__LINE__);

	for (index = 0; index < SDIO_TTY_NR; index++) {
		port = sdio_modem_table[index];
		tty = tty_port_tty_get(&port->port);
		
		if (tty) {
			tty_hangup(tty);
			tty_kref_put(tty);
		}
	}
}

static void sdio_tty_destroy( void )
{
	int index;
	struct sdio_modem_port *port;
	for (index= 0; index< SDIO_TTY_NR; index++) {
		port = sdio_modem_table[index];
		tty_unregister_device(modem_sdio_tty_driver, port->index);
		device_remove_file(&port->dev, &dev_attr_inception);
		device_unregister(&port->dev);
	}
	sdio_modem_remove_ext();
	sdio_modem_port_exit_ext();
}
#endif

static void modem_sdio_remove(struct sdio_func *func)
{
	struct sdio_modem *modem = sdio_get_drvdata(func);
#ifdef TTY_PORT_KEEP_EXIST
	struct sdio_modem_port *port[SDIO_TTY_NR];
	struct tty_struct *tty;
	unsigned long flags = 0;
#else
	struct sdio_modem_port *port;
#endif
	int index;

	mutex_lock(&tty_remove_mutex);

	LOGPRT(LOG_INFO, "%s %d\n", __func__, __LINE__);
#ifdef CONFIG_PM_RUNTIME
	platform_device_unregister(cbp_rt_device);
	platform_driver_unregister(&cbp_rt_driver);
#endif
	sysfs_remove_group(modem_sdio_kobj, &g_modem_attr_group);
	kobject_put(modem_sdio_kobj);
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_get_sync(&func->dev);
#endif
#ifdef TTY_PORT_KEEP_EXIST
	sdio_tty_init_bypass();
	for (index= 0; index< SDIO_TTY_NR; index++) {
		port[index] = modem->port[index];
		
		spin_lock_irqsave(&port[index]->write_lock, flags);
		kfifo_reset_out(&port[index]->transmit_fifo);
		spin_unlock_irqrestore(&port[index]->write_lock, flags);
		tty = port[index]->port.tty;
		
		if (tty)
			tty_wakeup(tty);
	}
#else
	for (index= 0; index< SDIO_TTY_NR; index++) {
		port = modem->port[index];
		tty_unregister_device(modem_sdio_tty_driver, port->index);
		device_remove_file(&port->dev, &dev_attr_inception);
		device_unregister(&port->dev);
	}

	sdio_modem_remove(modem);
	sdio_modem_port_exit(modem);

	for (index= 0; index < SDIO_TTY_NR; index++) {
		port = modem->port[index];
		if (port) {
			kfree(port);
			port = NULL;
		}
	}
#endif
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_irq(func);
	sdio_release_host(func);

	kfree(modem->trans_buffer);
	modem->trans_buffer = NULL;
	kfree(modem->msg);
	modem->msg = NULL;
	kfree(modem);
#ifdef CONFIG_PM_RUNTIME
	modem_rt = NULL;
	pm_runtime_put_sync(&func->dev);
	pm_runtime_disable(&func->dev);
#endif
#ifdef TTY_PORT_KEEP_EXIST
	for (index= 0; index< SDIO_TTY_NR; index++) {
		port[index]->modem = NULL;
	}
#endif
	LOGPRT(LOG_INFO, "%s %d\n", __func__, __LINE__);

	mutex_unlock(&tty_remove_mutex);
}

#define SDIO_VENDOR_ID_CBP		0x0296
#define SDIO_DEVICE_ID_CBP		0x5347

static const struct sdio_device_id modem_sdio_ids[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID_CBP, SDIO_DEVICE_ID_CBP) }, 
	{} 
};

MODULE_DEVICE_TABLE(sdio, modem_sdio_ids);

#ifdef CONFIG_CBP_PM
static int modem_sdio_suspend(struct device *pdev)
{
	return 0;
}

static int modem_sdio_resume(struct device *pdev)
{
	return 0;
}
#else
#define modem_sdio_suspend	NULL
#define modem_sdio_resume	NULL
#endif

static const struct dev_pm_ops modem_sdio_pm_ops = {
         .suspend	= modem_sdio_suspend,
         .resume	= modem_sdio_resume,
};

static struct sdio_driver modem_sdio_driver = {
	.probe		= modem_sdio_probe,
	.remove		= __devexit_p(modem_sdio_remove),
	.name		= "modem_sdio",
	.id_table	= modem_sdio_ids,
#ifdef CONFIG_CBP_PM
	.drv = {
		.pm		= &modem_sdio_pm_ops,
	},
#endif
};

int modem_sdio_init(struct cbp_platform_data *pdata)
{
	int ret;
	struct tty_driver *tty_drv;
#ifdef TTY_PORT_KEEP_EXIST
	tty_port_initial = 0;
#endif

	mutex_init(&tty_remove_mutex);

	modem_sdio_tty_driver = tty_drv = alloc_tty_driver(SDIO_TTY_NR);
	cbp_pdata = pdata;
	if (!tty_drv)
		return -ENOMEM;

	tty_drv->owner = THIS_MODULE;
	tty_drv->driver_name = "modem_sdio";
	tty_drv->name = "ttySDIO";
	tty_drv->major = 0;  
	tty_drv->minor_start = 0;
	tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	tty_drv->subtype = SERIAL_TYPE_NORMAL;
	tty_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_drv->init_termios = tty_std_termios;
	tty_drv->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_drv->init_termios.c_ispeed = 9600;
	tty_drv->init_termios.c_ospeed = 9600;
	tty_set_operations(tty_drv, &modem_tty_ops);

	ret = tty_register_driver(tty_drv);
	if (ret) {
		LOGPRT(LOG_ERR, "%s: tty_register_driver failed.\n", __func__);
		goto exit_reg_driver;
	}

	ret = sdio_register_driver(&modem_sdio_driver);
	if (ret) {
		LOGPRT(LOG_ERR, "%s: sdio_register_driver failed.\n",__func__);
		goto exit_tty;
	}

	init_waitqueue_head(&wr_wait_q);
#ifdef CONFIG_PM_RUNTIME
	init_waitqueue_head(&rt_wait_q);
#endif
	LOGPRT(LOG_INFO, "%s: sdio driver is initialized!\n",__func__);
	return ret;

exit_tty:
	tty_unregister_driver(tty_drv);
exit_reg_driver:
	LOGPRT(LOG_ERR, "%s: returning with error %d\n",__func__, ret);
	put_tty_driver(tty_drv);
	mutex_destroy(&tty_remove_mutex);
	return ret;
}

void modem_sdio_exit(void)
{
#ifdef TTY_PORT_KEEP_EXIST
	sdio_tty_destroy();
	tty_port_initial = 0;
#endif
	LOGPRT(LOG_INFO, "%s %d\n", __func__, __LINE__);
	sdio_unregister_driver(&modem_sdio_driver);
	tty_unregister_driver(modem_sdio_tty_driver);
	put_tty_driver(modem_sdio_tty_driver);
	mutex_destroy(&tty_remove_mutex);
}
