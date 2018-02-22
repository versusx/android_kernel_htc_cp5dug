/*
 * drivers/mmc/card/cbp_sdio.c
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
#include "modem_sdio.h"
#include <linux/mmc/host.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/ap_sync_cbp.h>

static int GPIO_MDM_PWR_EN;
static int GPIO_MDM_PMIC;
static int GPIO_MDM_RST;
static int GPIO_ETS_SEL;
static int GPIO_ETS_SEL1;
static int GPIO_MDM_USB_SWITCH;
static int GPIO_MDM_SIM_SWAP;
#ifdef CONFIG_CBP_SIM_HOTPLUG
static int GPIO_SIM_DETECT;
#endif
typedef enum {
	MDM_POWER_OFF,
	MDM_POWER_OFF_INPROGRESS,
	MDM_POWER_ON_INPROGRESS,
	MDM_POWER_ON,
} cbp_mdm_status;

typedef enum {
	FLOW_CTRL_DISABLE = 0,
	FLOW_CTRL_ENABLE
} flow_ctrl_state;

typedef enum {
	AP_ALIVE_MDM_SLEEP,
	AP_ALIVE_MDM_STBY,
	AP_SLEEP_MDM_SLEEP,
	AP_SLEEP_MDM_STBY,
} ap_mdm_status_t;

struct cbp_notify_info {
	const char *name;
	struct kobject *cbp_notify_kobj;
	struct workqueue_struct *cbp_notify_wq;
};

static struct cbp_notify_info cbp_notify;

extern unsigned char *get_htc_smem(void);

extern struct asc_rx_handle *cbp_asc_rx_handle_lookup(const char *name);

static struct kobject *cbp_power_kobj;

static unsigned char cbp_power_state = MDM_POWER_OFF;

static unsigned char swp_state = 0;

bool is_host_suspend = false;
EXPORT_SYMBOL(is_host_suspend);

bool host_js = false;
EXPORT_SYMBOL(host_js);

unsigned char assert_status = 0;
EXPORT_SYMBOL(assert_status);

extern int board_mfg_mode(void);
#include <mach/board_htc.h>
static int modem_usb_switch(void)
{
	int radio_flag = get_radio_flag_via();
	int boot_mode = board_mfg_mode();

	if (boot_mode == 10) {
		
		printk(KERN_INFO "Radio Router Mode Enabled!\n");
		gpio_direction_output(GPIO_MDM_USB_SWITCH, 1);
		gpio_direction_output(GPIO_ETS_SEL, 0);
	} else if (boot_mode == 9) {
		
		printk(KERN_INFO "Krouter Mode Enabled!\n");
		gpio_direction_output(GPIO_MDM_USB_SWITCH, 0);
		gpio_direction_output(GPIO_ETS_SEL, 1);
	} else {
		if (radio_flag & 0x200 ) {
			
			gpio_direction_output(GPIO_MDM_USB_SWITCH, 0);
			gpio_direction_output(GPIO_ETS_SEL, 1);
		} else if (radio_flag & 0x400) {
			
			gpio_direction_output(GPIO_MDM_USB_SWITCH, 1);
			gpio_direction_output(GPIO_ETS_SEL, 0);
		} else {
			gpio_direction_output(GPIO_MDM_USB_SWITCH, 1);
			gpio_direction_output(GPIO_ETS_SEL, 1);
		}
	}

	return 0;
}

#ifdef CONFIG_CBP_SIM_HOTPLUG

static struct cbp_sim_detect_info *cbp_sim_detect;
static int sim_status;

static void sim_detect_uevent(struct work_struct *work)
{
	char *envp_cdma_sim_in[] = {"CBP_SIM_IN", NULL};
	char *envp_cdma_sim_out[] = {"CBP_SIM_OUT", NULL};
	char *envp_cp0_sim_in[] = {"SIM_PLUG_IN", NULL};
	char *envp_cp0_sim_out[] = {"SIM_PLUG_OUT", NULL};

	int value = gpio_get_value(GPIO_SIM_DETECT);

	if (sim_status != value) {
		sim_status = value;
	} else {
		printk("%s: find same status %d\n, and filter it",
					__func__, value);
		return;
	}

	if (value == 0) {
		printk("%s: %s\n", __func__, "SIM_PLUG_OUT");
		kobject_uevent_env(cbp_notify.cbp_notify_kobj,
					KOBJ_CHANGE, (swp_state == 0)?envp_cdma_sim_out:envp_cp0_sim_out);
       } else {
		printk("%s: %s\n", __func__, "SIM_PLUG_IN");
		kobject_uevent_env(cbp_notify.cbp_notify_kobj,
						KOBJ_CHANGE, (swp_state == 0)?envp_cdma_sim_in:envp_cp0_sim_in);
	}
}
static DECLARE_DELAYED_WORK(cbp_sim_uevent_work, sim_detect_uevent);


static irqreturn_t cbp_sim_detect_irq_handler(int irq, void *dev)
{
	printk("%s @ %d\n", __func__, __LINE__);

	queue_delayed_work(cbp_notify.cbp_notify_wq,
				&cbp_sim_uevent_work, msecs_to_jiffies(500));

	return IRQ_HANDLED;
}
#endif


static void cbp_assert_uevent(struct work_struct *work)
{
	char *envp[] = {"CBP_ASSERT", NULL};

	printk(KERN_INFO " CBP assert!!\n");

	kobject_uevent_env(cbp_notify.cbp_notify_kobj, KOBJ_CHANGE, envp);
}
static DECLARE_DELAYED_WORK(cbp_assert_uevent_work, cbp_assert_uevent);

static void cbp_power_off_uevent(struct work_struct *work)
{
	char *envp[] = {"CBP_POWER_OFF", NULL};

	printk(KERN_INFO " CBP POWER OFF!!\n");

	kobject_uevent_env(cbp_notify.cbp_notify_kobj, KOBJ_CHANGE, envp);
}
static DECLARE_DELAYED_WORK(cbp_power_off_uevent_work, cbp_power_off_uevent);
#if 0
static int modem_detect_host(const char *host_id)
{
	struct mmc_host *mmc = NULL;
	struct class_dev_iter iter;
	struct device *dev;
	int ret = -1;

	printk("%s @ %d\n", __func__, __LINE__);

	mmc = mmc_alloc_host(0, NULL);
	if (!mmc) {
		ret =  -ENOMEM;
		goto out;
	}

	BUG_ON(!mmc->class_dev.class);
	class_dev_iter_init(&iter, mmc->class_dev.class, NULL, NULL);
	for (;;) {
		dev = class_dev_iter_next(&iter);
		if (!dev) {
			LOGPRT(LOG_ERR, "%s is not found.\n",host_id);
			ret = -1;
			break;
		} else {
			struct mmc_host *host = container_of(dev,
				struct mmc_host, class_dev);
			if (dev_name(&host->class_dev) &&
				strcmp(dev_name(mmc_dev(host)),
					host_id))
				continue;
			ret = 0;
			break;
		}
	}
	mmc_free_host(mmc);
out:
	return ret;
}
#endif

static struct cbp_wait_event *cbp_data_ack = NULL;

static irqreturn_t gpio_irq_data_ack(int irq, void *data)
{
	struct cbp_wait_event *cbp_data_ack = (struct cbp_wait_event *)data;
	int level;

	level = !!gpio_get_value(cbp_data_ack->wait_gpio);

	if (level == cbp_data_ack->wait_polar) {
		atomic_set(&cbp_data_ack->state, MODEM_ST_READY);
		wake_up(&cbp_data_ack->wait_q);
	}
	return IRQ_HANDLED;
}


static void data_ack_wait_event(struct cbp_wait_event *pdata_ack)
{
	struct cbp_wait_event *cbp_data_ack = (struct cbp_wait_event *)pdata_ack;
	unsigned long timeout;

	LOGPRT(LOG_NOTICE, "%s @ %d\n", __func__, __LINE__);
	timeout = wait_event_timeout(cbp_data_ack->wait_q,
				MODEM_ST_READY == atomic_read(&cbp_data_ack->state),
				msecs_to_jiffies(5000));
	if (!timeout)
		LOGPRT(LOG_ERR, "wait data ack event timeout!!\n");
	LOGPRT(LOG_NOTICE, "%s @ %d\n", __func__, __LINE__);
}


static struct cbp_wait_event *cbp_flow_ctrl = NULL;

static irqreturn_t gpio_irq_flow_ctrl(int irq, void *data)
{
	struct cbp_wait_event *cbp_flow_ctrl = (struct cbp_wait_event *)data;
	int level;

	level = !!gpio_get_value(cbp_flow_ctrl->wait_gpio);

	if (level == cbp_flow_ctrl->wait_polar) {
		atomic_set(&cbp_flow_ctrl->state, FLOW_CTRL_ENABLE);
		LOGPRT(LOG_NOTICE, "%s: flow control is enable, please write later!\n", __func__);
	} else {
		atomic_set(&cbp_flow_ctrl->state, FLOW_CTRL_DISABLE);
		LOGPRT(LOG_NOTICE, "%s: flow control is disable, can write now!\n", __func__);
		wake_up(&cbp_flow_ctrl->wait_q);
	}
	return IRQ_HANDLED;
}


static void flow_ctrl_wait_event(struct cbp_wait_event *pflow_ctrl)
{
	struct cbp_wait_event *cbp_flow_ctrl = (struct cbp_wait_event *)pflow_ctrl;
	unsigned long timeout = 0;

	LOGPRT(LOG_NOTICE, "%s @ %d\n", __func__, __LINE__);
	timeout = wait_event_timeout(cbp_flow_ctrl->wait_q,
				FLOW_CTRL_DISABLE == atomic_read(&cbp_flow_ctrl->state),
				msecs_to_jiffies(5000));
	if (!timeout)
		LOGPRT(LOG_ERR, "wait flow ctrl event timeout!!\n");
	LOGPRT(LOG_NOTICE, "%s @ %d\n", __func__, __LINE__);
}


static int modem_sdio_tx_notifier(int event, void *data);
static int modem_sdio_rx_notifier(int event, void *data);

static struct asc_config sdio_tx_handle ={
	.name = "sdio",
};

static struct asc_infor sdio_tx_user ={
	.name = "cbp",
	.data = &sdio_tx_handle,
	.notifier = modem_sdio_tx_notifier,
};

static struct asc_config sdio_rx_handle = {
	.name = "sdio",
};

static struct asc_infor sdio_rx_user = {
	.name = "cbp",
	.data = &sdio_rx_handle,
	.notifier = modem_sdio_rx_notifier,
};


static int modem_sdio_tx_notifier(int event, void *data)
{
	return 0;
}


static int modem_sdio_rx_notifier(int event, void *data)
{
	struct asc_config *rx_config  = (struct asc_config *)data;
	int ret = 0;

	switch(event) {
		case ASC_NTF_RX_PREPARE:
			asc_rx_confirm_ready(rx_config->name, 1);
			break;
		case ASC_NTF_RX_POST:
			asc_rx_confirm_ready(rx_config->name, 0);
			break;
		default:
			LOGPRT(LOG_ERR, "%s: ignor unknow evernt!!\n", __func__);
			break;
	}
	return ret;
}


static char *modem_mmc_id;
static int modem_detect_card(void)
{
	struct mmc_host *mmc = NULL;
	struct class_dev_iter iter;
	struct device *dev;
	int ret = 0;

	mmc = mmc_alloc_host(0, NULL);
	if (!mmc) {
		ret =  -ENOMEM;
		goto out;
	}

	BUG_ON(!mmc->class_dev.class);
	class_dev_iter_init(&iter, mmc->class_dev.class, NULL, NULL);
	for (;;) {
		dev = class_dev_iter_next(&iter);
		if (!dev) {
			LOGPRT(LOG_ERR, "%s is not found.\n",
				modem_mmc_id);
			ret = -1;
			break;
		} else {
			struct mmc_host *host = container_of(dev,
				struct mmc_host, class_dev);
			if (dev_name(&host->class_dev) &&
					strcmp(dev_name(mmc_dev(host)),
						modem_mmc_id))
				continue;

#if 1
			mmc_detect_change(host, msecs_to_jiffies(200));
#else
			mmc_detect_change(host, 0);
#endif
			ret = 0;
			break;
		}
	}
	mmc_free_host(mmc);
out:
	return ret;
}


static void modem_detect(struct work_struct *work)
{
	int ret;

	ret = modem_detect_card();
	if (ret) {
		LOGPRT(LOG_ERR, "%s: modem detect failed.\n", __func__);
	}
}

struct cbp_reset {
	struct mmc_host *host;
	const char  *name;
	struct workqueue_struct *reset_wq;
	struct work_struct	reset_work;
	struct timer_list timer_gpio;
	int rst_ind_gpio;
	int rst_ind_polar;
};
static struct cbp_reset *cbp_rst_ind = NULL;

static void cbp_reset_ind(struct work_struct *work)
{
	int level = 0;

	level = !!gpio_get_value(cbp_rst_ind->rst_ind_gpio);
	printk(KERN_INFO "%s: %d enter!! level = %d\n", __func__, __LINE__, level);
	if (level == cbp_rst_ind->rst_ind_polar) {
		if (cbp_power_state == MDM_POWER_ON_INPROGRESS) {
			cbp_power_state = MDM_POWER_ON;

			if (cbp_rst_ind->host) {
				mmc_detect_change(cbp_rst_ind->host, msecs_to_jiffies(150));
			} else {
				queue_work(cbp_rst_ind->reset_wq, &cbp_rst_ind->reset_work);
			}
		}

		if (assert_status == 1) {
			printk("%s @ assert_status %d\n", __func__, assert_status);
			if (cbp_rst_ind->host) {
				mmc_detect_change(cbp_rst_ind->host, msecs_to_jiffies(150));
			} else {
				queue_work(cbp_rst_ind->reset_wq, &cbp_rst_ind->reset_work);
			}
		}
	} else {
		if (cbp_power_state == MDM_POWER_ON) {
			assert_status = 1;
			queue_delayed_work(cbp_notify.cbp_notify_wq,
						&cbp_assert_uevent_work, 0);
		}

		if (cbp_rst_ind->host) {
			mmc_detect_change(cbp_rst_ind->host, msecs_to_jiffies(150));
		} else {
			queue_work(cbp_rst_ind->reset_wq, &cbp_rst_ind->reset_work);
		}
	}
}
static DECLARE_DELAYED_WORK(cbp_reset_ind_work, cbp_reset_ind);

static irqreturn_t gpio_irq_cbp_rst_ind(int irq, void *data)
{
	if (cbp_notify.cbp_notify_wq) {
		queue_delayed_work(cbp_notify.cbp_notify_wq,
					&cbp_reset_ind_work, msecs_to_jiffies(100));
	} else {
		LOGPRT(LOG_ERR, "%s: cbp_notify_wq NULL.\n", __func__);
	}
	return IRQ_HANDLED;
}

static void set_cbp_power(int on) {
	if (on) {
		gpio_direction_output(GPIO_MDM_PMIC, 0);
		gpio_direction_output(GPIO_MDM_RST, 1);
		gpio_direction_output(GPIO_MDM_PWR_EN, 1);
		msleep(200);
		gpio_direction_output(GPIO_MDM_PMIC, 1);
		msleep(500);
		gpio_direction_output(GPIO_MDM_PWR_EN, 0);
		msleep(200);
		gpio_direction_output(GPIO_MDM_RST, 0);
	} else {
		gpio_direction_output(GPIO_MDM_RST, 0);
		gpio_direction_output(GPIO_MDM_PMIC, 0);
		gpio_direction_output(GPIO_MDM_RST, 1);
		msleep(500);
		msleep(600);
		gpio_direction_output(GPIO_MDM_RST, 0);
		gpio_direction_output(GPIO_MDM_PWR_EN, 0);
		gpio_direction_output(GPIO_ETS_SEL1, 0);
	}
}

static void reset_cbp(void)
{
	gpio_direction_output(GPIO_MDM_PMIC, 1);
	msleep(10);
	gpio_direction_output(GPIO_MDM_RST, 1);
	msleep(100);
	msleep(300);
	gpio_direction_output(GPIO_MDM_RST, 0); 
}


static ssize_t cbp_power_on_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	char *s = buf;
	s += sprintf(s, "%d\n", cbp_power_state);

	return (s - buf);
}

extern void cbp_power_init(int en);

static ssize_t cbp_power_on_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;

	if (val) {
		if (cbp_power_state == MDM_POWER_OFF) {
			assert_status = 0;
			cbp_power_state = MDM_POWER_ON_INPROGRESS;
			printk(KERN_INFO "%s: start power on cbp.\n", __func__);
			cbp_power_init(1);
			msleep(50);
			set_cbp_power(1);

#if 0
			LOGPRT(LOG_DEBUG, "%s: config uart gpio.\n", __func__);
			nmk_config_pin(GPIO29_U2_RXD, false);
			nmk_config_pin(GPIO30_U2_TXD, false);

			LOGPRT(LOG_DEBUG, "%s: config sdio gpio.\n", __func__);
			nmk_config_pin(GPIO219_MC3_CLK, false);
			nmk_config_pin(GPIO220_MC3_FBCLK, false);
			nmk_config_pin(GPIO221_MC3_CMD | PIN_INPUT_PULLUP, false);
			nmk_config_pin(GPIO222_MC3_DAT0 | PIN_INPUT_PULLUP, false);
			nmk_config_pin(GPIO223_MC3_DAT1 | PIN_INPUT_PULLUP, false);
			nmk_config_pin(GPIO224_MC3_DAT2 | PIN_INPUT_PULLUP, false);
			nmk_config_pin(GPIO225_MC3_DAT3 | PIN_INPUT_PULLUP, false);
#endif

		} else if (cbp_power_state == MDM_POWER_ON_INPROGRESS){
			LOGPRT(LOG_ERR, "%s: CBP power on in progress.\n", __func__);
		} else if (cbp_power_state == MDM_POWER_ON) {
			LOGPRT(LOG_ERR, "%s: CBP is already power on.\n", __func__);
		} else {
			LOGPRT(LOG_ERR, "%s: CBP power off in progress.\n", __func__);
		}
	} else {
		if (cbp_power_state != MDM_POWER_OFF) {
			cbp_power_state = MDM_POWER_OFF_INPROGRESS;
			queue_delayed_work(cbp_notify.cbp_notify_wq,
						&cbp_power_off_uevent_work, 0);
			assert_status = 0;
			set_cbp_power(0);
			cbp_power_init(0);
			printk(KERN_INFO "%s: power off cbp.\n", __func__);
			cbp_power_state = MDM_POWER_OFF;
		} else {
			LOGPRT(LOG_ERR, "%s: CBP is already power off.\n", __func__);
		}
	}

	return n;
}

static ssize_t cbp_reset_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return 0;
}

static ssize_t cbp_reset_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;

	if (val) {
		reset_cbp();
		printk(KERN_INFO "%s: reset cbp.\n", __func__);
	} else {
		printk(KERN_ERR "%s: reset cbp use value 1.\n", __func__);
	}

	return n;
}


static ssize_t cbp_htc_config_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	char *s;

	printk("%s @ %d\n", __func__, __LINE__);

	s = get_htc_smem();

	if (s) {
		memcpy(buf, s, 128);
		return 128;
	} else {
		return 0;
	}
}


static ssize_t cbp_htc_config_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	printk("%s @ %d\n", __func__, __LINE__);

	return n;
}

#if 0
static ssize_t cbp_sim_swap_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	char *s = buf;
	s += sprintf(s, "%d\n", swp_state);

	if (swp_state)
		printk(KERN_INFO "%s: Swap to GSM Modem.\n", __func__);
	else
		printk(KERN_INFO "%s: Swap to CDMA Modem.\n", __func__);

	return (s - buf);
}


static ssize_t cbp_sim_swap_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;

	if (val) {
		gpio_direction_output(GPIO_MDM_SIM_SWAP, 1);
		swp_state = 1;
		printk(KERN_WARNING "%s: Swap to GSM Modem.\n", __func__);
	} else {
		gpio_direction_output(GPIO_MDM_SIM_SWAP, 0);
		swp_state = 0;
		printk(KERN_WARNING "%s: Swap to CDMA Modem.\n", __func__);
	}

	return n;
}

static ssize_t cbp_sim_detect_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	char *s = buf;
	
	s += sprintf(s, "%d\n", swp_state?0:sim_status);

	if (sim_status && !swp_state)
		LOGPRT(LOG_INFO, "%s: sim card exist.\n", __func__);
	else
		LOGPRT(LOG_INFO, "%s: No sim card.\n", __func__);

	return (s - buf);
}

static ssize_t cbp_sim_detect_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	return n;
}
#endif

static ssize_t cbp_board_version_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	char *s = buf;
	char board_ver;

	board_ver = 0;
	s += sprintf(s, "%d\n", board_ver);

	return (s - buf);
}


static ssize_t cbp_board_version_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	return n;
}


static ssize_t cbp_usb_switch_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	char *s = buf;

	return (s - buf);
}


static ssize_t cbp_usb_switch_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;

	LOGPRT(LOG_INFO, "%s: enable %ld \n", __func__, val);
	if (val) {
		gpio_direction_output(GPIO_MDM_USB_SWITCH, 0);
	} else {
		gpio_direction_output(GPIO_MDM_USB_SWITCH, 1);
	}

	return n;
}

static ssize_t cbp_ets_sel_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	char *s = buf;

	return (s - buf);
}


static ssize_t cbp_ets_sel_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;


	LOGPRT(LOG_INFO, "%s %ld\n", __func__, val);
	if (val) {
		gpio_direction_output(GPIO_ETS_SEL, 1);
	} else {
		gpio_direction_output(GPIO_ETS_SEL, 0);
	}

	return n;
}

static ssize_t cbp_ets_sel1_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	char *s = buf;

	return (s - buf);
}


static ssize_t cbp_ets_sel1_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	unsigned long val;
	int i = 0;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val < 0)
		return -EINVAL;


	LOGPRT(LOG_ERR, "cbp_assert trigger %ld\n",val);
	if (val == 1) {
		gpio_direction_output(GPIO_ETS_SEL1, 1);
		gpio_direction_output(GPIO_ETS_SEL, 0);
		msleep(80);
		for (i = 0; i < 3; i++) {
			gpio_direction_output(GPIO_ETS_SEL, 1);
			msleep(80);
			gpio_direction_output(GPIO_ETS_SEL, 0);
			msleep(80);
		}
	}
	return n;
}

#define cbp_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= cbp_##_name##_show,			\
	.store	= cbp_##_name##_store,		\
}
cbp_attr(power_on);
cbp_attr(reset);
cbp_attr(htc_config);
#if 0
cbp_attr(sim_swap);
cbp_attr(sim_detect);
#endif
cbp_attr(board_version);
cbp_attr(usb_switch);
cbp_attr(ets_sel);
cbp_attr(ets_sel1);

static struct attribute * cbp_power_attr[] = {
	&power_on_attr.attr,
	&reset_attr.attr,
	&htc_config_attr.attr,
#if 0
	&sim_swap_attr.attr,
	&sim_detect_attr.attr,
#endif
	&board_version_attr.attr,
	&usb_switch_attr.attr,
	&ets_sel_attr.attr,
	&ets_sel1_attr.attr,
	NULL,
};

static struct attribute_group g_power_attr_group = {
	.attrs = cbp_power_attr,
};


static ssize_t cbp_notify_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return 0;
}

static ssize_t cbp_notify_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(cbp_notify, S_IRUSR | S_IRGRP, cbp_notify_show, cbp_notify_store);


static int __devinit cbp_probe(struct platform_device *pdev)
{
	struct cbp_platform_data *plat = pdev->dev.platform_data;
	int ret = -1;

	printk("%s @ %d\n", __func__, __LINE__);

	
	if (!plat) {
		ret = -EINVAL;
		goto out;
	}

#if 0
	if ((gpio_is_valid(plat->gpio_lv_shift))) {
		ret = gpio_request(plat->gpio_lv_shift, CBP_DRIVER_NAME"(MC3_LEVEL_SHIFT)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to requset MC3_LEVEL_SHIFT gpio %d ret =%d!!\n",
				__func__, __LINE__, plat->gpio_lv_shift, ret);
			goto out;
		}
	}
#endif
	
	GPIO_MDM_PWR_EN = plat->gpio_pwr_on;
	GPIO_MDM_PMIC = plat->gpio_pmic;
	GPIO_MDM_RST = plat->gpio_rst;
	GPIO_MDM_SIM_SWAP = plat->gpio_sim_swap;
	modem_mmc_id = plat->host_id;

	if (gpio_is_valid(plat->gpio_data_ack)) {
		cbp_data_ack = kzalloc(sizeof(struct cbp_wait_event), GFP_KERNEL);
		if (!cbp_data_ack) {
			ret = -ENOMEM;
			LOGPRT(LOG_ERR, "%s %d kzalloc cbp_data_ack failed \n",__func__, __LINE__);
			goto err_kzalloc_cbp_data_ack;
		}

		init_waitqueue_head(&cbp_data_ack->wait_q);
		atomic_set(&cbp_data_ack->state, MODEM_ST_UNKNOW);
		cbp_data_ack->wait_gpio = plat->gpio_data_ack;
		cbp_data_ack->wait_polar = plat->gpio_data_ack_polar;

		ret = gpio_request(plat->gpio_data_ack, CBP_DRIVER_NAME "(data_ack)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to requset data_ack gpio %d ret =%d!!\n",
				__func__, __LINE__, plat->gpio_data_ack, ret);
			goto err_req_data_ack;
		}
		gpio_direction_input(plat->gpio_data_ack);
		ret = request_irq(gpio_to_irq(plat->gpio_data_ack), gpio_irq_data_ack,
				IRQF_SHARED | IRQF_TRIGGER_RISING, CBP_DRIVER_NAME "(data_ack)", cbp_data_ack);
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to request irq for data_ack!!\n", __func__, __LINE__);
			goto err_req_irq_data_ack;
		}
		plat->cbp_data_ack = cbp_data_ack;
		plat->data_ack_wait_event = data_ack_wait_event;
		plat->data_ack_enable =true;
	}

	if (gpio_is_valid(plat->gpio_flow_ctrl)) {
		cbp_flow_ctrl = kzalloc(sizeof(struct cbp_wait_event), GFP_KERNEL);
		if (!cbp_flow_ctrl) {
			ret = -ENOMEM;
			LOGPRT(LOG_ERR, "%s %d kzalloc cbp_flow_ctrl failed \n",__func__, __LINE__);
			goto err_kzalloc_cbp_flow_ctrl;
		}

		init_waitqueue_head(&cbp_flow_ctrl->wait_q);
		atomic_set(&cbp_flow_ctrl->state, FLOW_CTRL_DISABLE);
		cbp_flow_ctrl->wait_gpio = plat->gpio_flow_ctrl;
		cbp_flow_ctrl->wait_polar = plat->gpio_flow_ctrl_polar;

		ret = gpio_request(plat->gpio_flow_ctrl, CBP_DRIVER_NAME "(flow_ctrl)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to requset flow_ctrl gpio %d ret =%d!!\n",
				__func__, __LINE__, plat->gpio_flow_ctrl, ret);
			goto err_req_flow_ctrl;
		}
		gpio_direction_input(plat->gpio_flow_ctrl);
		ret = request_irq(gpio_to_irq(plat->gpio_flow_ctrl), gpio_irq_flow_ctrl,
				IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				CBP_DRIVER_NAME "(flow_ctrl)", cbp_flow_ctrl);
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to request irq for flow_ctrl!!\n", __func__, __LINE__);
			goto err_req_irq_flow_ctrl;
		}

		plat->cbp_flow_ctrl= cbp_flow_ctrl;
		plat->flow_ctrl_wait_event = flow_ctrl_wait_event;
		plat->flow_ctrl_enable = true;
	}


	if (gpio_is_valid(plat->gpio_rst_ind)) {
		cbp_rst_ind = kzalloc(sizeof(struct cbp_reset), GFP_KERNEL);
		if (!cbp_rst_ind) {
			ret = -ENOMEM;
			LOGPRT(LOG_ERR, "%s %d kzalloc cbp_rst_ind failed \n",__func__, __LINE__);
			goto err_kzalloc_cbp_rst_ind;
		}

		cbp_rst_ind->name = "cbp_rst_ind_wq";
		cbp_rst_ind->reset_wq = create_singlethread_workqueue(cbp_rst_ind->name);
		if (cbp_rst_ind->reset_wq == NULL) {
			ret = -ENOMEM;
			LOGPRT(LOG_ERR, "%s %d error creat rst_ind_workqueue \n",__func__, __LINE__);
			goto err_create_work_queue;
		}
		INIT_WORK(&cbp_rst_ind->reset_work, modem_detect);
		cbp_rst_ind->rst_ind_gpio = plat->gpio_rst_ind;
		cbp_rst_ind->rst_ind_polar = plat->gpio_rst_ind_polar;

		ret = gpio_request(plat->gpio_rst_ind, CBP_DRIVER_NAME "(rst_ind)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to requset rst_ind gpio %d ret =%d!!\n",
				__func__, __LINE__, plat->gpio_rst_ind, ret);
			goto err_req_rst_ind;
		}

		gpio_direction_input(plat->gpio_rst_ind);
		irq_set_irq_wake(gpio_to_irq(plat->gpio_rst_ind), IRQF_TRIGGER_FALLING);
		ret = request_irq(gpio_to_irq(plat->gpio_rst_ind), gpio_irq_cbp_rst_ind,
			IRQF_SHARED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND, CBP_DRIVER_NAME "(rst_ind)", cbp_rst_ind);
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to request irq for rst_ind!!\n", __func__, __LINE__);
			goto err_req_irq_rst_ind;
		}
		plat->rst_ind_enable = true;
	}

	if ((gpio_is_valid(plat->gpio_pwr_on)) && (gpio_is_valid(plat->gpio_rst)) ) {
		ret = gpio_request(plat->gpio_pwr_on, CBP_DRIVER_NAME "(pwr_on)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to requset power on gpio %d ret =%d!!\n",
				__func__, __LINE__, plat->gpio_pwr_on,ret);
			goto err_req_pwr_on;
		}

		ret = gpio_request(plat->gpio_rst,CBP_DRIVER_NAME "(rst)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to requset reset gpio %d ret =%d!!\n",
				__func__, __LINE__, plat->gpio_rst,ret);
			goto err_req_rst;
		}
	}

	if (gpio_is_valid(plat->gpio_pmic)) {
		ret = gpio_request(plat->gpio_pmic, CBP_DRIVER_NAME "(pmic on)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to requset power on gpio %d ret =%d!!\n",
				__func__, __LINE__, plat->gpio_pwr_on, ret);
			goto err_req_pwr_pmic;
		}
	}

	if (gpio_is_valid(plat->gpio_ets_sel)) {
		ret = gpio_request(plat->gpio_ets_sel, CBP_DRIVER_NAME "(ets_sel)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to request ets select on gpio %d ret = %d!!\n",
				__func__, __LINE__, plat->gpio_ets_sel, ret);
			goto err_req_ets_sel;
		}
		GPIO_ETS_SEL = plat->gpio_ets_sel;
		gpio_direction_output(GPIO_ETS_SEL, 0); 
	}

	if (gpio_is_valid(plat->gpio_ets_sel1)) {
		ret = gpio_request(plat->gpio_ets_sel1, CBP_DRIVER_NAME "(ets_sel1)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to request ets select 1 on gpio %d ret = %d!!\n",
				__func__, __LINE__, plat->gpio_ets_sel1, ret);
			goto err_req_ets_sel1;
		}
		GPIO_ETS_SEL1 = plat->gpio_ets_sel1;
		gpio_direction_output(GPIO_ETS_SEL1, 0); 
	}

	if (gpio_is_valid(plat->gpio_usb_switch)) {
		ret = gpio_request(plat->gpio_usb_switch, CBP_DRIVER_NAME "(usb_switch)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to request usb switch on gpio %d ret = %d!!\n",
				__func__, __LINE__, plat->gpio_usb_switch, ret);
			goto err_req_usb_switch;
		}
		GPIO_MDM_USB_SWITCH = plat->gpio_usb_switch;
		modem_usb_switch();
	}

	if (gpio_is_valid(plat->gpio_sim_swap)) {
		ret = gpio_request(plat->gpio_sim_swap, CBP_DRIVER_NAME "(sim_swap)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to request sim switch on gpio %d ret = %d!!\n",
				__func__, __LINE__, plat->gpio_sim_swap, ret);
			goto err_req_sim_swap;
		}
		if (board_mfg_mode() == 11) {
			gpio_direction_output(plat->gpio_sim_swap, 1);
			swp_state = 1;
		} else {
			gpio_direction_output(plat->gpio_sim_swap, 0);
			swp_state = 0;
		}
	}

	if (gpio_is_valid(plat->gpio_ant_ctrl)) {
		ret = gpio_request(plat->gpio_ant_ctrl, CBP_DRIVER_NAME "(ant_ctrl)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to request ant ctrl on gpio %d ret = %d!!\n",
				__func__, __LINE__, plat->gpio_ant_ctrl, ret);
			goto err_req_ant_ctrl;
		}
		if (board_mfg_mode() == 11)
			gpio_direction_output(plat->gpio_ant_ctrl, 0);
		else
			gpio_direction_output(plat->gpio_ant_ctrl, 1);
	}

	if ((gpio_is_valid(plat->gpio_ap_wkup_cp)) && (gpio_is_valid(plat->gpio_cp_ready)) &&
		(gpio_is_valid(plat->gpio_cp_wkup_ap))  && (gpio_is_valid(plat->gpio_ap_ready))) {
		sdio_tx_handle.gpio_wake = plat->gpio_ap_wkup_cp;
		sdio_tx_handle.gpio_ready = plat->gpio_cp_ready;
		sdio_tx_handle.polar = plat->gpio_sync_polar;
		ret = asc_tx_register_handle(&sdio_tx_handle);
		if (ret) {
			LOGPRT(LOG_ERR, "%s %d asc_tx_register_handle failed.\n",__FUNCTION__,__LINE__);
			goto err_create_sysfs_notify;
		}
		ret = asc_tx_add_user(sdio_tx_handle.name, &sdio_tx_user);
		if (ret) {
			LOGPRT(LOG_ERR, "%s %d asc_tx_add_user failed.\n",__FUNCTION__,__LINE__);
			goto err_create_sysfs_notify;
		}

		sdio_rx_handle.gpio_wake = plat->gpio_cp_wkup_ap;
		sdio_rx_handle.gpio_ready = plat->gpio_ap_ready;
		sdio_rx_handle.polar = plat->gpio_sync_polar;
		ret = asc_rx_register_handle(&sdio_rx_handle);
		if (ret) {
			LOGPRT(LOG_ERR, "%s %d asc_rx_register_handle failed.\n", __FUNCTION__, __LINE__);
			goto err_create_sysfs_notify;
		}
		ret = asc_rx_add_user(sdio_rx_handle.name, &sdio_rx_user);
		if (ret) {
			LOGPRT(LOG_ERR, "%s %d asc_rx_add_user failed.\n", __FUNCTION__, __LINE__);
			goto err_create_sysfs_notify;
		}
		plat->ipc_enable = true;
		plat->tx_handle = &sdio_tx_handle;
	}

	memset(&cbp_notify, 0, sizeof(struct cbp_notify_info));

	cbp_notify.cbp_notify_kobj = &pdev->dev.kobj;

	ret = sysfs_create_file(cbp_notify.cbp_notify_kobj, &dev_attr_cbp_notify.attr);
	if (ret) {
		goto err_create_sysfs_notify;
	}

	cbp_notify.cbp_notify_wq = create_workqueue("cbp_notify");
	if (!cbp_notify.cbp_notify_wq) {
		ret = -ENOMEM;
		LOGPRT(LOG_ERR, "%s %d error creat cbp_notify_workqueue \n", __func__, __LINE__);
		goto err_create_notify_wq;
	}

#ifdef CONFIG_CBP_SIM_HOTPLUG
	GPIO_SIM_DETECT = plat->gpio_sim_detect;
	if (gpio_is_valid(plat->gpio_sim_detect)) {
		ret = gpio_request(plat->gpio_sim_detect, CBP_DRIVER_NAME "(sim_detect)");
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to requset sim detect gpio %d ret =%d!!\n",
				__func__, __LINE__, plat->gpio_sim_detect, ret);
			goto err_req_sim_detect;
		}

		gpio_direction_input(plat->gpio_sim_detect);
		irq_set_irq_wake(gpio_to_irq(plat->gpio_sim_detect), IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
		ret = request_irq(gpio_to_irq(plat->gpio_sim_detect), cbp_sim_detect_irq_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, CBP_DRIVER_NAME "(sim_detect)", (void *)&cbp_notify);
		if (ret < 0) {
			LOGPRT(LOG_ERR, "%s: %d fail to request irq for rst_ind!!\n", __func__, __LINE__);
			goto err_req_irq_sim_detect;
		}

		sim_status = gpio_get_value(GPIO_SIM_DETECT);
	}
#endif

	ret = modem_sdio_init(plat);
	if (ret) {
		LOGPRT(LOG_ERR, "%s: host %s setup failed!\n", __func__, plat->host_id);
		goto err_ipc;
	}

	cbp_power_kobj = kobject_create_and_add("cbp_power", NULL);
	if (!cbp_power_kobj) {
		ret = -ENOMEM;
		goto err_create_kobj;
	}

	ret = sysfs_create_group(cbp_power_kobj, &g_power_attr_group);
	if (ret) {
		goto err_create_sysfs_power;
	}

	LOGPRT(LOG_INFO, "cbp initialized on host %s successfully, bus is %s !\n", plat->host_id, plat->bus);
	return ret;

err_create_sysfs_power:
	kobject_put(cbp_power_kobj);
err_create_kobj:
	modem_sdio_exit();
err_ipc:
#ifdef CONFIG_CBP_SIM_HOTPLUG
	if (gpio_is_valid(plat->gpio_sim_detect)) {
		free_irq(gpio_to_irq(plat->gpio_sim_detect), NULL);
	}
err_req_irq_sim_detect:
	if (gpio_is_valid(plat->gpio_sim_detect)) {
		gpio_free(plat->gpio_sim_detect);
	}
err_req_sim_detect:
#endif
	destroy_workqueue(cbp_notify.cbp_notify_wq);
err_create_notify_wq:
	sysfs_remove_file(cbp_notify.cbp_notify_kobj,
				&dev_attr_cbp_notify.attr);
err_create_sysfs_notify:
	if (gpio_is_valid(plat->gpio_ant_ctrl))
		gpio_free(plat->gpio_ant_ctrl);
err_req_ant_ctrl:
	if (gpio_is_valid(plat->gpio_sim_swap))
		gpio_free(plat->gpio_sim_swap);
err_req_sim_swap:
	if (gpio_is_valid(plat->gpio_usb_switch))
		gpio_free(plat->gpio_usb_switch);
err_req_usb_switch:
	if (gpio_is_valid(plat->gpio_ets_sel1))
		gpio_free(plat->gpio_ets_sel1);
err_req_ets_sel1:
	if (gpio_is_valid(plat->gpio_ets_sel))
		gpio_free(plat->gpio_ets_sel);
err_req_ets_sel:
	if (gpio_is_valid(plat->gpio_pmic))
		gpio_free(plat->gpio_pmic);
err_req_pwr_pmic:
	if (gpio_is_valid(plat->gpio_rst))
		gpio_free(plat->gpio_rst);
err_req_rst:
	if (gpio_is_valid(plat->gpio_pwr_on)) {
		gpio_free(plat->gpio_pwr_on);
	}
err_req_pwr_on:
	if (plat->rst_ind_enable && (gpio_is_valid(plat->gpio_rst_ind))) {
		free_irq(gpio_to_irq(plat->gpio_rst_ind), cbp_rst_ind);
	}
err_req_irq_rst_ind:
	if (plat->rst_ind_enable && (gpio_is_valid(plat->gpio_rst_ind))) {
		gpio_free(plat->gpio_rst_ind);
	}
err_req_rst_ind:
	if (plat->rst_ind_enable && (gpio_is_valid(plat->gpio_rst_ind))) {
		destroy_workqueue(cbp_rst_ind->reset_wq);
	}
err_create_work_queue:
	if (plat->rst_ind_enable && (gpio_is_valid(plat->gpio_rst_ind))) {
		kfree(cbp_rst_ind);
	}
err_kzalloc_cbp_rst_ind:
	if (plat->flow_ctrl_enable && (gpio_is_valid(plat->gpio_flow_ctrl))) {
		free_irq(gpio_to_irq(plat->gpio_flow_ctrl), cbp_flow_ctrl);
	}
err_req_irq_flow_ctrl:
	if (plat->flow_ctrl_enable && (gpio_is_valid(plat->gpio_flow_ctrl))) {
		gpio_free(plat->gpio_flow_ctrl);
	}
err_req_flow_ctrl:
	if (plat->flow_ctrl_enable && (gpio_is_valid(plat->gpio_flow_ctrl))) {
		kfree(cbp_flow_ctrl);
	}

err_kzalloc_cbp_flow_ctrl:
	if (plat->data_ack_enable && (gpio_is_valid(plat->gpio_data_ack))) {
		free_irq(gpio_to_irq(plat->gpio_data_ack), cbp_data_ack);
	}
err_req_irq_data_ack:
	if (plat->data_ack_enable && (gpio_is_valid(plat->gpio_data_ack))) {
		gpio_free(plat->gpio_data_ack);
	}
err_req_data_ack:
	if (plat->data_ack_enable && (gpio_is_valid(plat->gpio_data_ack))) {
		kfree(cbp_data_ack);
	}
err_kzalloc_cbp_data_ack:
	if ((gpio_is_valid(plat->gpio_lv_shift))) {
		gpio_free(plat->gpio_lv_shift);
	}
out:
	return ret;
}

static int __devexit cbp_remove(struct platform_device *pdev)
{
	struct cbp_platform_data *plat = pdev->dev.platform_data;

	if ((gpio_is_valid(plat->gpio_lv_shift))) {
		gpio_free(plat->gpio_lv_shift);
	}

	if (plat->data_ack_enable && (gpio_is_valid(plat->gpio_data_ack))) {
		free_irq(gpio_to_irq(plat->gpio_data_ack), cbp_data_ack);
		gpio_free(plat->gpio_data_ack);
		kfree(cbp_data_ack);
	}

	if (plat->flow_ctrl_enable && (gpio_is_valid(plat->gpio_flow_ctrl))) {
		free_irq(gpio_to_irq(plat->gpio_flow_ctrl), cbp_flow_ctrl);
		gpio_free(plat->gpio_flow_ctrl);
		kfree(cbp_flow_ctrl);
	}

	if (plat->rst_ind_enable && (gpio_is_valid(plat->gpio_rst_ind))) {
		free_irq(gpio_to_irq(plat->gpio_rst_ind), cbp_rst_ind);
		gpio_free(plat->gpio_rst_ind);
		destroy_workqueue(cbp_rst_ind->reset_wq);
		kfree(cbp_rst_ind);
	}

	if (gpio_is_valid(plat->gpio_pmic)) {
		gpio_free(plat->gpio_pmic);
	}

	if (gpio_is_valid(plat->gpio_ets_sel)) {
		gpio_free(plat->gpio_ets_sel);
	}

	if (gpio_is_valid(plat->gpio_ets_sel1)) {
		gpio_free(plat->gpio_ets_sel1);
	}

	if (gpio_is_valid(plat->gpio_usb_switch)) {
		gpio_free(plat->gpio_usb_switch);
	}

	if (gpio_is_valid(plat->gpio_sim_swap)) {
		gpio_free(plat->gpio_sim_swap);
	}

	if (gpio_is_valid(plat->gpio_ant_ctrl)) {
		gpio_free(plat->gpio_ant_ctrl);
	}

#ifdef CONFIG_CBP_SIM_HOTPLUG
	if (gpio_is_valid(plat->gpio_sim_detect)) {
		destroy_workqueue(cbp_notify.cbp_notify_wq);
		sysfs_remove_file(cbp_notify.cbp_notify_kobj,
					&dev_attr_cbp_notify.attr);
		kobject_put(cbp_notify.cbp_notify_kobj);
		free_irq(gpio_to_irq(plat->gpio_sim_detect), cbp_sim_detect);
		gpio_free(plat->gpio_sim_detect);
		kobject_put(cbp_notify.cbp_notify_kobj);
	}
#endif
	modem_sdio_exit();
	sysfs_remove_group(cbp_power_kobj, &g_power_attr_group);
	kobject_put(cbp_power_kobj);

	return 0;
}


#ifdef CONFIG_PM
extern wait_queue_head_t wr_wait_q;

static int cbp_suspend(struct platform_device *pdev, pm_message_t state) {

	struct asc_rx_handle *rx;
	int ret;

	ret = 0;
	rx = cbp_asc_rx_handle_lookup(sdio_rx_handle.name);

	printk("%s ++\n", __func__);

	is_host_suspend = true;
	host_js = true;
	if (wake_lock_active(&rx->wlock)) {
		is_host_suspend = false;
		wake_up(&rx->wait);
		ret = -EBUSY;
	} else {
	}

	printk("%s -- ret %d.\n", __func__, ret);

	return ret;
}


static int cbp_resume(struct platform_device *pdev) {

	struct asc_rx_handle *rx;

	printk("%s ++\n", __func__);

	rx = cbp_asc_rx_handle_lookup(sdio_rx_handle.name);


	is_host_suspend = false;

	wake_up(&rx->wait);
	wake_up_interruptible(&wr_wait_q);

	printk("%s --\n", __func__);

	return 0;
}
#else
#define cbp_suspend	NULL
#define cbp_resume	NULL
#endif

static struct platform_driver cbp_driver = {
	.driver = {
		.name = CBP_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= cbp_probe,
	.suspend	= cbp_suspend,
	.resume		= cbp_resume,
	.remove		= __devexit_p(cbp_remove),
};

static int __init cp2dcg_cbp_init(void)
{
	int ret;

	ret = platform_driver_register(&cbp_driver);
	if (ret)
		LOGPRT(LOG_ERR, "platform_driver_register failed\n");
	return ret;
}

static void __exit cp2dcg_cbp_exit(void)
{
	platform_driver_unregister(&cbp_driver);
}

module_init(cp2dcg_cbp_init);
module_exit(cp2dcg_cbp_exit);

MODULE_DESCRIPTION("CP2DCG CBP SDIO driver");
