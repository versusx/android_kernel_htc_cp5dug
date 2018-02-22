/*
 * include/linux/ap_sync_cp.h
 *
 * VIA CBP driver for Linux
 *
 * Copyright (C) 2011 VIA TELECOM Corporation, Inc.
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
#ifndef _AP_SYNC_CBP_H
#define _AP_SYNC_CBP_H

/* notifer events */
#define ASC_NTF_TX_READY        0x0001 /*notifie CBP is ready to work*/
#define ASC_NTF_TX_UNREADY    0x0002 /*notifie CBP is not ready to work*/
#define ASC_NTF_RX_PREPARE    0x1001 /* notifier the device active to receive data from CBP*/
#define ASC_NTF_RX_POST          0x1002 /* notifer the device CBP stop tx data*/

#define ASC_NAME_LEN   (64)

/*used to register handle*/
struct asc_config{
    char name[ASC_NAME_LEN];
    int gpio_ready;
    int gpio_wake;
    /*the level which indicate ap is ready*/
    int polar;
};

/*Used to registe user accoring to handle*/
struct asc_infor {
    char name[ASC_NAME_LEN];
    void *data;
    int (*notifier)(int, void *);
};

struct asc_tx_handle {
	struct asc_config cfg;
	atomic_t state;
	atomic_t count;
	struct list_head user_list;
	struct asc_state_dsp *table;
	/*process the event to switch different states*/
	struct task_struct  *thread;
	int ntf;
	int wait_try;
	int auto_delay;
	spinlock_t slock;
	wait_queue_head_t wait;
	//struct completion wait_tx_state;
	wait_queue_head_t wait_tx_state;
	struct mutex mlock;
	struct wake_lock wlock;
	struct timer_list timer_wait_ready;
	struct timer_list timer_wait_idle;
	struct timer_list timer_wait_sleep;
	struct work_struct ntf_work;
	struct list_head event_q;
	struct list_head node;
	struct kobject *kobj;

	struct workqueue_struct *wq;
	struct work_struct	wq_work;
};

struct asc_rx_handle {
	struct asc_config cfg;
	atomic_t state;
	struct list_head user_list;
	struct asc_state_dsp *table;
	int ntf;
	/*process the event to switch different states*/
	struct task_struct  *thread;
	spinlock_t slock;
	wait_queue_head_t wait;
	struct mutex mlock;
	struct wake_lock wlock;
	struct timer_list timer;
	struct list_head event_q;
	struct list_head node;
	struct work_struct ntf_work;
};

int asc_tx_register_handle(struct asc_config *cfg);
int asc_tx_add_user(const char *name, struct asc_infor *infor);
int asc_tx_get_ready(const char *path, int sync);
int asc_tx_put_ready(const char *path, int sync);
int asc_tx_auto_ready(const char *name, int sync);
int asc_tx_check_ready(const char *name);
int asc_tx_set_auto_delay(const char *name, int delay);
int asc_tx_user_count(const char *path);
void asc_tx_reset(const char *name);

int asc_rx_register_handle(struct asc_config *cfg);
int asc_rx_add_user(const char *name, struct asc_infor *infor);
int asc_rx_confirm_ready(const char *name, int ready);
void asc_rx_reset(const char *name);
#endif /*_AP_SYNC_CBP_H*/
