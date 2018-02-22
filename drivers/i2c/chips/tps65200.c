/* drivers/i2c/chips/tps65200.c
 *
 * Copyright (C) 2009 HTC Corporation
 * Author: Josh Hsiao <Josh_Hsiao@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/tps65200.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include "../../staging/android/android_alarm.h"
#include <linux/alarmtimer.h>
#include <linux/time.h>
#include <mach/board_htc.h>
#include <mach/board.h>
#include <linux/max17050_battery.h>

#if defined(CONFIG_MACH_HOLIDAY)
#define AC_CURRENT_SWTICH_DELAY_200MS		200
#define AC_CURRENT_SWTICH_DELAY_100MS		100
#endif	
#define pr_tps_fmt(fmt) "[BATT][tps65200] " fmt
#define pr_tps_err_fmt(fmt) "[BATT][tps65200] err:" fmt
#define pr_tps_war_fmt(fmt) "[BATT][tps65200] WARNING:" fmt
#define pr_tps_info(fmt, ...) \
	printk(KERN_INFO pr_tps_fmt(fmt), ##__VA_ARGS__)
#define pr_tps_err(fmt, ...) \
	printk(KERN_ERR pr_tps_err_fmt(fmt), ##__VA_ARGS__)
#define pr_tps_war(fmt, ...) \
	printk(KERN_WARNING pr_tps_war_fmt(fmt), ##__VA_ARGS__)
#define TPS65200_CHECK_INTERVAL (15)
#define DELAY_MHL_INIT	msecs_to_jiffies(200)
#define DELAY_KICK_TPS	msecs_to_jiffies(10)
#define CHECK_OVP_PERIOD	msecs_to_jiffies(30000)
#define OVP_OCCUR_PERIOD	msecs_to_jiffies(5000)
#define SET_VDPM_AS_444	1
#define CHSTAT 0x1C

static struct workqueue_struct *tps65200_wq;
static struct work_struct chg_stat_work;
static struct delayed_work set_vdpm_work;
static struct delayed_work kick_dog;
static struct delayed_work auto_check_ovp;

static struct alarm tps65200_check_alarm;
static struct work_struct check_alarm_work;

static int chg_stat_int;
static unsigned int chg_stat_enabled;
static spinlock_t chg_stat_lock;
static int chg_status_incorrect = 0;

static struct tps65200_chg_int_data *chg_int_data;
static LIST_HEAD(tps65200_chg_int_list);
static DEFINE_MUTEX(notify_lock);

static const unsigned short normal_i2c[] = { I2C_CLIENT_END };

static int tps65200_initial = -1;
static int tps65200_low_chg;
static int tps65200_vdpm_chg = 0;

static struct wake_lock charger_wake_lock;

extern struct timespec timespec_set(const long secs, const unsigned long nsecs);
extern BOOL is_high_current_charging_avaialable(void);
extern int check_boost_mode_function(void);
#ifdef CONFIG_SUPPORT_DQ_BATTERY
static int htc_is_dq_pass;
#endif
static bool tps65200_suspended = false;
static bool chg_stat_triggered = false;
static bool tps65200_int_triggered = false;
static bool tps65200_alarm = false;
u8 batt_charging_state;

static void tps65200_set_check_alarm(void)
{
	struct timespec interval;
	struct timespec next_alarm;
	pr_tps_info("%s interval:%d\n",__func__,TPS65200_CHECK_INTERVAL);
	interval = timespec_set(TPS65200_CHECK_INTERVAL, 0);
	get_monotonic_boottime(&next_alarm);
	next_alarm = timespec_add(next_alarm, interval);
	alarm_start(&tps65200_check_alarm, timespec_to_ktime(next_alarm));
}


static int tps65200_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
#if 0
static int tps65200_detect(struct i2c_client *client, int kind,
			 struct i2c_board_info *info);
#endif
static int tps65200_remove(struct i2c_client *client);

struct tps65200_i2c_client {
	struct i2c_client *client;
	u8 address;
	
	struct i2c_msg xfer_msg[2];
	
	struct mutex xfer_lock;
};
static struct tps65200_i2c_client tps65200_i2c_module;

static int tps65200_i2c_write(u8 *value, u8 reg, u8 num_bytes)
{
	int ret;
	struct tps65200_i2c_client *tps;
	struct i2c_msg *msg;

	tps = &tps65200_i2c_module;

	mutex_lock(&tps->xfer_lock);
	msg = &tps->xfer_msg[0];
	msg->addr = tps->address;
	msg->len = num_bytes + 1;
	msg->flags = 0;
	msg->buf = value;
	
	*value = reg;
	ret = i2c_transfer(tps->client->adapter, tps->xfer_msg, 1);
	mutex_unlock(&tps->xfer_lock);

	
	if (ret >= 0)
		ret = 0;
	return ret;
}


static int tps65200_i2c_read(u8 *value, u8 reg, u8 num_bytes)
{
	int ret;
	u8 val;
	struct tps65200_i2c_client *tps;
	struct i2c_msg *msg;

	tps = &tps65200_i2c_module;

	mutex_lock(&tps->xfer_lock);
	
	msg = &tps->xfer_msg[0];
	msg->addr = tps->address;
	msg->len = 1;
	msg->flags = 0; 
	val = reg;
	msg->buf = &val;
	
	msg = &tps->xfer_msg[1];
	msg->addr = tps->address;
	msg->flags = I2C_M_RD;  
	msg->len = num_bytes;   
	msg->buf = value;
	ret = i2c_transfer(tps->client->adapter, tps->xfer_msg, 2);
	mutex_unlock(&tps->xfer_lock);

	
	if (ret >= 0)
		ret = 0;
	return ret;
}


static int tps65200_i2c_write_byte(u8 value, u8 reg)
{
	
	int result;
	int i;
	u8 temp_buffer[2] = { 0 };
	
	temp_buffer[1] = value;
	for (i = 0; i < 10; i++) {
		result = tps65200_i2c_write(temp_buffer, reg, 1);
		if (result == 0)
			break;
		pr_tps_err("TPS65200 I2C write retry count = %d, result = %d\n", i+1, result);
		msleep(10);
	}
	if (result != 0)
		pr_tps_err("TPS65200 I2C write fail = %d\n", result);

	return result;
}

static int tps65200_i2c_read_byte(u8 *value, u8 reg)
{
	int result = 0;
	int i;
	for (i = 0; i < 10; i++) {
		result = tps65200_i2c_read(value, reg, 1);
		if (result == 0)
			break;
		pr_tps_err("TPS65200 I2C read retry count = %d, result = %d\n", i+1, result);
		msleep(10);
	}
	if (result != 0)
		pr_tps_err("TPS65200 I2C read fail = %d\n", result);

	return result;
}

void tps65200_kick_charger_ic(int charge_enable)
{
	u8 regh = 0;
	tps65200_i2c_read_byte(&regh, 0x06);
	if (!(regh & CHSTAT)) {
		chg_status_incorrect++;
		pr_tps_info("[BATT] chg_status is incorrect-> 0x%x, %dtimes", regh, chg_status_incorrect);
	}
	pr_tps_info("[BATT] re-enable charger-> %d", charge_enable);
	tps_set_charger_ctrl(charge_enable);
}

int tps_register_notifier(struct tps65200_chg_int_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_lock);
	list_add(&notifier->notifier_link,
		&tps65200_chg_int_list);
	mutex_unlock(&notify_lock);
	return 0;
}
EXPORT_SYMBOL(tps_register_notifier);

static void send_tps_chg_int_notify(int int_reg, int value)
{
	static struct tps65200_chg_int_notifier *notifier;

	mutex_lock(&notify_lock);
	list_for_each_entry(notifier,
		&tps65200_chg_int_list,
		notifier_link) {
		if (notifier->func != NULL)
			notifier->func(int_reg, value);
	}
	mutex_unlock(&notify_lock);
}

static int tps65200_set_chg_stat(unsigned int ctrl)
{
	unsigned long flags;
	if (!chg_stat_int)
		return -1;
	spin_lock_irqsave(&chg_stat_lock, flags);
	chg_stat_enabled = ctrl;
	spin_unlock_irqrestore(&chg_stat_lock, flags);

	return 0;
}

int tps65200_dump_register(void)
{
	u8 regh0 = 0, regh1 = 0, regh2 = 0, regh3 = 0;
	int result = 0;
	tps65200_i2c_read_byte(&regh1, 0x01);
	tps65200_i2c_read_byte(&regh0, 0x00);
	tps65200_i2c_read_byte(&regh3, 0x03);
	tps65200_i2c_read_byte(&regh2, 0x02);
	pr_tps_info("regh 0x00=%x, regh 0x01=%x, regh 0x02=%x, regh 0x03=%x\n",
			regh0, regh1, regh2, regh3);
	tps65200_i2c_read_byte(&regh0, 0x04);
	tps65200_i2c_read_byte(&regh1, 0x05);
	tps65200_i2c_read_byte(&regh2, 0x06);
	tps65200_i2c_read_byte(&regh3, 0x07);
	pr_tps_info("regh 0x04=%x, 0x05=%x, regh 0x06=%x, 0x07 = %x\n",
			regh0, regh1, regh2, regh3);
	tps65200_i2c_read_byte(&regh0, 0x0B);
	tps65200_i2c_read_byte(&regh1, 0x0C);
	tps65200_i2c_read_byte(&regh2, 0x0D);
	result = tps65200_i2c_read_byte(&regh3, 0x0E);
	pr_tps_info("regh 0x0B=%x, 0x0C=%x, regh 0x0D=%x, regh 0x0E=%x\n",
			regh0, regh1, regh2, regh3);


	return result;
}
EXPORT_SYMBOL(tps65200_dump_register);

u32 htc_fake_charger_for_testing(u32 ctl)
{
	u32 new_ctl = POWER_SUPPLY_ENABLE_FAST_CHARGE;

	if((ctl > POWER_SUPPLY_ENABLE_INTERNAL) || (ctl == POWER_SUPPLY_DISABLE_CHARGE))
		return ctl;

#if defined(CONFIG_MACH_VERDI_LTE)
	new_ctl = POWER_SUPPLY_ENABLE_9VAC_CHARGE;
#else
	
#endif

	pr_tps_info("[BATT] %s(%d -> %d)\n", __func__, ctl , new_ctl);
	batt_charging_state = new_ctl;
	return new_ctl;
}

static void set_vdpm(struct work_struct *work)
{
	if (tps65200_vdpm_chg)
		tps_set_charger_ctrl(VDPM_ORIGIN_V);
}

int tps65200_mask_interrupt_register(int status)
{
	if (status == CHARGER_USB) {	
		tps65200_i2c_write_byte(0x7F, 0x0C);
	} else if (status == CHARGER_BATTERY) {
		tps65200_i2c_write_byte(0xFF, 0x0C);
		
		reverse_protection_handler(REVERSE_PROTECTION_CONTER_CLEAR);
	}
	return 0;
}
EXPORT_SYMBOL(tps65200_mask_interrupt_register);

int tps_set_charger_ctrl(u32 ctl)
{
	int result = 0;
	u8 status = 0;
	u8 regh = 0;
	u8 regh1 = 0, regh2 = 0, regh3 = 0;
	int i2c_status = 0;


	if (tps65200_initial < 0)
		return 0;

	switch (ctl) {
	case POWER_SUPPLY_DISABLE_CHARGE:  
		if(check_boost_mode_function()){
			printk("[BATT]OTG function is enabled, do not switch charger off.\n");
			break;
		}
		printk("[BATT]Switch charger OFF\n");
		tps65200_set_chg_stat(0);
		tps65200_i2c_write_byte(0x29, 0x01);
		tps65200_i2c_write_byte(0x28, 0x00);

#if SET_VDPM_AS_444
		cancel_delayed_work_sync(&set_vdpm_work);
		tps65200_i2c_read_byte(&regh, 0x03);
		regh |= 0x83;
		tps65200_vdpm_chg = 0;
		tps65200_i2c_write_byte(regh, 0x03); 
#endif 

		
		alarm_cancel(&tps65200_check_alarm);
		break;
	case POWER_SUPPLY_ENABLE_SLOW_CHARGE:  
	case POWER_SUPPLY_ENABLE_WIRELESS_CHARGE:  
		tps65200_i2c_write_byte(0x29, 0x01);
		tps65200_i2c_write_byte(0x2A, 0x00);
#if SET_VDPM_AS_444
		regh = 0x83; 
#else  

#ifdef CONFIG_SUPPORT_DQ_BATTERY
		if (htc_is_dq_pass)*/
			
			regh = 0x83;
#endif
#endif
		if (tps65200_low_chg)
			regh |= 0x08;	
		tps65200_i2c_write_byte(regh, 0x03);

#ifdef CONFIG_SUPPORT_DQ_BATTERY
		if (htc_is_dq_pass)
			regh = 0x6A;
		else
			regh = 0x63;
#endif
		if(is_high_current_charging_avaialable())
			regh = 0x60;
		tps65200_i2c_write_byte(regh, 0x02);
		tps65200_i2c_read_byte(&regh1, 0x03);
		tps65200_dump_register();
		tps65200_set_chg_stat(1);
#if SET_VDPM_AS_444
		tps65200_vdpm_chg = 1;
		result = schedule_delayed_work(&set_vdpm_work, DELAY_MHL_INIT);
		if (!result) {
			cancel_delayed_work(&set_vdpm_work);
			schedule_delayed_work(&set_vdpm_work, DELAY_MHL_INIT);
		}
#endif 
		break;
	case POWER_SUPPLY_ENABLE_FAST_CHARGE:  

		tps65200_i2c_write_byte(0x29, 0x01);
		tps65200_i2c_write_byte(0x2A, 0x00);

#if SET_VDPM_AS_444
		regh = 0x83;
#else 
#ifdef CONFIG_SUPPORT_DQ_BATTERY
		if (htc_is_dq_pass)
			
			regh = 0x85;
#endif
#endif

		if (tps65200_low_chg)
			regh |= 0x08;	
		tps65200_i2c_write_byte(regh, 0x03);

#ifdef CONFIG_SUPPORT_DQ_BATTERY
		if (htc_is_dq_pass)
			regh = 0xAA;
		else
			regh = 0xA3;
#endif
		if(is_high_current_charging_avaialable())
			regh = 0xA0;  
		tps65200_i2c_write_byte(regh, 0x02);

		tps65200_dump_register();
		tps65200_set_chg_stat(1);
#if SET_VDPM_AS_444
		tps65200_vdpm_chg = 1;
		result = schedule_delayed_work(&set_vdpm_work, DELAY_MHL_INIT);
		if (!result) {
			cancel_delayed_work(&set_vdpm_work);
			schedule_delayed_work(&set_vdpm_work, DELAY_MHL_INIT);
		}
#endif 
		break;
	case POWER_SUPPLY_ENABLE_SLOW_HV_CHARGE:
		tps65200_i2c_write_byte(0x29, 0x01);
		tps65200_i2c_write_byte(0x2A, 0x00);
		regh = 0x85;
		if (tps65200_low_chg)
			regh |= 0x08;
		tps65200_i2c_write_byte(regh, 0x03);
		tps65200_i2c_write_byte(0x6A, 0x02);
		tps65200_i2c_read_byte(&regh, 0x03);
		tps65200_i2c_read_byte(&regh1, 0x02);
		pr_tps_info("Switch charger ON (SLOW_HV): regh 0x03=%x, "
				"regh 0x02=%x\n", regh, regh1);
		break;
	case POWER_SUPPLY_ENABLE_FAST_HV_CHARGE:
		tps65200_i2c_write_byte(0x29, 0x01);
		tps65200_i2c_write_byte(0x2A, 0x00);
		regh = 0x85;
		if (tps65200_low_chg)
			regh |= 0x08;
		tps65200_i2c_write_byte(regh, 0x03);
		tps65200_i2c_write_byte(0xAA, 0x02);
		tps65200_i2c_read_byte(&regh, 0x01);
		tps65200_i2c_read_byte(&regh1, 0x00);
		tps65200_i2c_read_byte(&regh2, 0x03);
		tps65200_i2c_read_byte(&regh3, 0x02);
		pr_tps_info("Switch charger ON (FAST_HV): regh 0x01=%x, "
				"regh 0x01=%x, regh 0x03=%x, regh 0x02=%x\n",
				regh, regh1, regh2, regh3);
		break;
	case ENABLE_LIMITED_CHG:
		tps65200_i2c_read_byte(&regh, 0x03);
		regh |= 0x08;
		tps65200_i2c_write_byte(regh, 0x03);
		tps65200_low_chg = 1;
		tps65200_i2c_read_byte(&regh, 0x03);
		pr_tps_info("Switch charger ON (LIMITED): regh 0x03=%x\n", regh);
		break;
	case CLEAR_LIMITED_CHG:
		tps65200_i2c_read_byte(&regh, 0x03);
		regh &= 0xF7;
		tps65200_i2c_write_byte(regh, 0x03);

		tps65200_low_chg = 0;
		tps65200_i2c_read_byte(&regh, 0x03);
		pr_tps_info("Switch charger OFF (LIMITED): regh 0x03=%x\n", regh);
		break;
	case CHECK_CHG:
		i2c_status = tps65200_i2c_read_byte(&regh3, 0x0E);
		if (i2c_status == -5) {
			pr_tps_info("Delay 200ms to kick tps watchdog!\n");
			schedule_delayed_work(&kick_dog, DELAY_KICK_TPS);
		}
		break;
	case SET_ICL500:
		pr_tps_info("Switch charger SET_ICL500 \n");
		tps65200_i2c_write_byte(0xA3, 0x02);
		break;
	case SET_ICL100:
		pr_tps_info("Switch charger SET_ICL100 \n");
		tps65200_i2c_write_byte(0x23, 0x02);
		break;
	case CHECK_INT1:
		tps65200_i2c_read_byte(&status, 0x08);
		pr_tps_info("Switch charger CHECK_INT1: regh 0x08h=%x\n", status);
		result = (int)status;
		break;
	case CHECK_INT2:
		tps65200_i2c_read_byte(&status, 0x09);
		pr_tps_info("TPS65200 INT2 0x09h=%x\n", status);
		result = (int)status;
		break;
	case CHECK_CONTROL:
		tps65200_i2c_read_byte(&status, 0x00);
		pr_tps_info("TPS65200 check control, regh 0x00=%x\n", status);
		break;
	case OVERTEMP_VREG:
		tps65200_i2c_read_byte(&regh, 0x02);
		regh = (regh & 0xC0) | 0x1C;
		tps65200_i2c_write_byte(regh, 0x02);
		tps65200_i2c_read_byte(&regh, 0x02);
		pr_tps_info("Switch charger OVERTEMP_VREG_4060: regh 0x02=%x\n", regh);
		break;
	case NORMALTEMP_VREG:
#ifdef CONFIG_SUPPORT_DQ_BATTERY
		pr_tps_info("Switch charger CONFIG_D: regh 0x04=%x\n", regh);
		if (htc_is_dq_pass) {
			tps65200_i2c_read_byte(&regh, 0x02);
			regh = (regh & 0xC0) | 0X2A;
			tps65200_i2c_write_byte(regh, 0x02);
			tps65200_i2c_read_byte(&regh, 0x02);
			pr_tps_info("Switch charger NORMALTEMP_VREG_4340: regh 0x02=%x\n", regh);
			break;
		}
#endif
		tps65200_i2c_read_byte(&regh, 0x02);
		regh = (regh & 0xC0) | 0X23;
		tps65200_i2c_write_byte(regh, 0x02);
		tps65200_i2c_read_byte(&regh, 0x02);
		pr_tps_info("Switch charger NORMALTEMP_VREG_4200: regh 0x02=%x\n", regh);
		break;
	case NORMALTEMP_VREG_HV:
		tps65200_i2c_read_byte(&regh, 0x02);
		regh = (regh & 0xC0) | 0x2A;
		tps65200_i2c_write_byte(regh, 0x02);
		tps65200_i2c_read_byte(&regh, 0x02);
		pr_tps_info("Switch charger NORMALTEMP_VREG_4200: regh 0x02=%x\n", regh);
		break;
	case POWER_SUPPLY_ENABLE_INTERNAL:
		
		tps65200_i2c_read_byte(&regh, 0x00);
		pr_tps_info("regh 0x00=%x\n", regh);
		regh |= 0x01;
		regh &= 0xfd;
		tps65200_i2c_write_byte(regh, 0x00);
		tps65200_i2c_write_byte(0x19, 0x02);  
		
		tps65200_set_check_alarm();
		tps65200_i2c_read_byte(&regh, 0x00);
		pr_tps_info("Switch charger to Boost mode: regh 0x00=%x\n", regh);
		break;
	case VDPM_ORIGIN_V:
		tps65200_i2c_read_byte(&regh, 0x03);
#ifdef CONFIG_SUPPORT_DQ_BATTERY
		if (htc_is_dq_pass)
			regh = (regh & 0xF8) | 0x03; 
#else
		regh = (regh & 0xF8) | 0x03; 
#endif
		tps65200_i2c_write_byte(regh, 0x03);
		tps65200_i2c_read_byte(&regh, 0x03);
		pr_tps_info("Set VDPM back to original V: regh 0x03=%x\n", regh);
		tps65200_vdpm_chg = 0;
		break;
	case VDPM_444V:
		tps65200_i2c_read_byte(&regh, 0x03);
		regh = (regh & 0xF8) | 0x03;
		tps65200_i2c_write_byte(regh, 0x03);
		tps65200_i2c_read_byte(&regh, 0x03);
		pr_tps_info("Set VDPM as 4.44V: regh 0x03=%x\n", regh);
		break;
	default:
		pr_tps_info("%s: Not supported battery ctr called.!", __func__);
		result = -EINVAL;
		break;
	}

	return result;
}
EXPORT_SYMBOL(tps_set_charger_ctrl);

#if 0
static int tps65200_detect(struct i2c_client *client, int kind,
			 struct i2c_board_info *info)
{
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
				     I2C_FUNC_SMBUS_BYTE))
		return -ENODEV;

	strlcpy(info->type, "tps65200", I2C_NAME_SIZE);

	return 0;
}
#endif

static irqreturn_t chg_stat_handler(int irq, void *data)
{
	pr_tps_info("interrupt chg_stat is triggered. "
			"chg_stat_enabled:%u\n", chg_stat_enabled);
	if ( tps65200_suspended == false){
		printk("lock charger_wake_lock charger-tps65200 in %s\n",__func__);
		wake_lock_timeout(&charger_wake_lock, 5*HZ);
		queue_work(tps65200_wq, &chg_stat_work);
		chg_stat_triggered = false;
	}else if(chg_stat_enabled){
		printk("not resumed now, mark it\n");
		chg_stat_triggered = true;
	}

	return IRQ_HANDLED;
}

static irqreturn_t chg_int_handler(int irq, void *data)
{
	pr_tps_info("interrupt chg_int is triggered.\n");

	disable_irq_nosync(chg_int_data->gpio_chg_int);
	chg_int_data->tps65200_reg = 0;
	schedule_delayed_work(&chg_int_data->int_work, msecs_to_jiffies(200));
	return IRQ_HANDLED;
}
static void tps65200_int_handler(void)
{
	int fault_bit;
	switch (chg_int_data->tps65200_reg) {
		case CHECK_INT1:
			fault_bit = tps_set_charger_ctrl(CHECK_INT1);
			fault_bit = tps_set_charger_ctrl(CHECK_INT1);

			if (fault_bit & 0x40) {
				send_tps_chg_int_notify(CHECK_INT1, 1);
				schedule_delayed_work(&chg_int_data->int_work,
						msecs_to_jiffies(5000));
				pr_tps_info("over voltage fault bit "
						"on TPS65200 is raised: %x\n", fault_bit);

			} else {
				send_tps_chg_int_notify(CHECK_INT1, 0);
				cancel_delayed_work(&chg_int_data->int_work);
				enable_irq(chg_int_data->gpio_chg_int);
				pr_tps_info("clean over voltage fault bit "
						"on TPS65200 is raised: %x\n", fault_bit);
			}
			break;
		default:
			fault_bit = tps_set_charger_ctrl(CHECK_INT2);
			pr_tps_info("Read register INT2 value: %x\n", fault_bit);
			if (fault_bit & 0x80) {
				fault_bit = tps_set_charger_ctrl(CHECK_INT2);
				fault_bit = tps_set_charger_ctrl(CHECK_INT2);
				fault_bit = tps_set_charger_ctrl(CHECK_CONTROL);
				pr_tps_info("Reverse current protection happened.\n");
				tps65200_set_chg_stat(0);
				tps65200_i2c_write_byte(0x29, 0x01);
				tps65200_i2c_write_byte(0x28, 0x00);
				reverse_protection_handler(REVERSE_PROTECTION_HAPPEND);

				send_tps_chg_int_notify(CHECK_INT2, 1);
				cancel_delayed_work(&chg_int_data->int_work);
				enable_irq(chg_int_data->gpio_chg_int);
			} else if (fault_bit & 0x01) {
				pr_tps_info("Charger warning. Input voltage DPM loop active\n");
			} else {
				fault_bit = tps_set_charger_ctrl(CHECK_INT1);
				if (fault_bit & 0x40) {
					chg_int_data->tps65200_reg = CHECK_INT1;
					schedule_delayed_work(&chg_int_data->int_work,
							msecs_to_jiffies(200));
				} else {
					pr_tps_war("CHG_INT should not be triggered "
							"without fault bit!\n");
					enable_irq(chg_int_data->gpio_chg_int);
				}
			}
	}
	printk("unlock charger_wake_lock charger-tps65200 in %s\n",__func__);
	wake_unlock(&charger_wake_lock);
}
static void tps65200_int_func(struct work_struct *work)
{
	printk("lock charger_wake_lock charger-tps65200 in %s\n",__func__);
	wake_lock_timeout(&charger_wake_lock, 5*HZ);
	if(tps65200_suspended == true){
		tps65200_int_triggered = true;
	}else{
		tps65200_int_handler();
		tps65200_int_triggered = false;
	}
}

static void chg_stat_work_func(struct work_struct *work)
{
	tps65200_set_chg_stat(0);
	tps65200_i2c_write_byte(0x29, 0x01);
	tps65200_i2c_write_byte(0x28, 0x00);
	printk("release wake lock charger-tps65200\n");
	wake_unlock(&charger_wake_lock);
	return;
}
static void kick_tps_watchdog(struct work_struct *work)
{
	pr_tps_info("To kick tps watchgod through delay workqueue !\n");
	tps65200_dump_register();
	return;
}
static void auto_check_ovp_func(struct work_struct *work)
{
	static int pre_ovp_notify = 0;
	int fault_bit;
	u8 status = 0;

	tps65200_i2c_read_byte(&status, 0x08);
	fault_bit = (int)status;
	if (fault_bit & 0x40) {
		send_tps_chg_int_notify(CHECK_INT1, 1);
		pre_ovp_notify = 1;
		pr_tps_info("over voltage fault bit "
				"on TPS65200 is raised: %x\n", fault_bit);
		schedule_delayed_work(&auto_check_ovp, OVP_OCCUR_PERIOD);
	} else if(pre_ovp_notify) {
		send_tps_chg_int_notify(CHECK_INT1, 0);
		pre_ovp_notify = 0;
		schedule_delayed_work(&auto_check_ovp, CHECK_OVP_PERIOD);
	} else{
		schedule_delayed_work(&auto_check_ovp, CHECK_OVP_PERIOD);
	}
	return;
}

static enum alarmtimer_restart tps65200_check_alarm_handler(struct alarm *alarm, ktime_t now)
{
	printk("lock charger_wake_lock charger-tps65200 in %s\n",__func__);
	wake_lock(&charger_wake_lock);
	if(tps65200_suspended == false){
		queue_work(tps65200_wq, &check_alarm_work);
		tps65200_alarm = false;
	}else{
		tps65200_alarm = true;
	}
	return ALARMTIMER_NORESTART;
}

static void check_alarm_work_func(struct work_struct *work)
{
	tps_set_charger_ctrl(CHECK_CHG);
	
	tps65200_set_check_alarm();
	printk("unlock charger_wake_lock charger-tps65200 in %s\n",__func__);
	wake_unlock(&charger_wake_lock);
}

static int tps65200_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc, gpio_irq;
	struct tps65200_i2c_client   *data = &tps65200_i2c_module;
	struct tps65200_platform_data *pdata =
					client->dev.platform_data;

	pr_tps_info("%s\n",__func__);

	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		pr_tps_err("I2C fail\n");
		return -EIO;
	}

	tps65200_wq = create_singlethread_workqueue("tps65200");
	if (!tps65200_wq) {
		pr_tps_err("Failed to create tps65200 workqueue.");
		return -ENOMEM;
	}

	wake_lock_init(&charger_wake_lock, WAKE_LOCK_SUSPEND, "charger-tps65200");
	
	INIT_WORK(&check_alarm_work, check_alarm_work_func);
	alarm_init(&tps65200_check_alarm,
			ALARM_BOOTTIME,
			tps65200_check_alarm_handler);

#ifdef CONFIG_SUPPORT_DQ_BATTERY
	htc_is_dq_pass = pdata->dq_result;
	if (htc_is_dq_pass)
		pr_tps_info("HV battery is detected.\n");
	else
		pr_tps_info("not HV battery.\n");

#endif
	
	chg_stat_int = 0;
	if (pdata->gpio_chg_stat > 0) {
		rc = gpio_request(pdata->gpio_chg_stat, NULL);
		if(rc) {
			goto chg_stat_irq_failed;
		}
		gpio_irq = gpio_to_irq(pdata->gpio_chg_stat);
		rc = request_irq( gpio_irq,
				chg_stat_handler,
				IRQF_TRIGGER_RISING,
				"chg_stat", NULL);

		if (rc < 0)
			goto chg_stat_irq_failed;
		else {
			INIT_WORK(&chg_stat_work, chg_stat_work_func);
			chg_stat_int = pdata->gpio_chg_stat;
		}
	} 

	
	if (pdata->gpio_chg_int > 0) {
		chg_int_data = (struct tps65200_chg_int_data *)
				kmalloc(sizeof(struct tps65200_chg_int_data),
					GFP_KERNEL);
		if (!chg_int_data) {
			pr_tps_err("No memory for chg_int_data!\n");
			return -1;
		}

		rc = gpio_request(pdata->gpio_chg_int, NULL);
		if(rc) {
			goto chg_int_irq_failed;
		}
		gpio_irq = gpio_to_irq(pdata->gpio_chg_int);
		chg_int_data->gpio_chg_int = 0;
		INIT_DELAYED_WORK(&chg_int_data->int_work,
				tps65200_int_func);

		rc = request_irq(
				gpio_irq,
				chg_int_handler,
				IRQF_TRIGGER_FALLING,
				"chg_int", NULL);
		if (rc < 0)
			pr_tps_err("request chg_int irq failed!\n");
		else {
			pr_tps_info("init chg_int interrupt.\n");
			chg_int_data->gpio_chg_int =
				pdata->gpio_chg_int;
		}
	} 
	INIT_DELAYED_WORK(&set_vdpm_work, set_vdpm);

	pr_tps_info("To init delay workqueue to kick tps watchdog!\n");
	INIT_DELAYED_WORK(&kick_dog, kick_tps_watchdog);

	data->address = client->addr;
	data->client = client;
	mutex_init(&data->xfer_lock);
	tps65200_initial = 1;

#if SET_VDPM_AS_444
	tps_set_charger_ctrl(VDPM_444V);
#endif 

	tps_set_charger_ctrl(POWER_SUPPLY_ENABLE_SLOW_CHARGE);

	if (pdata->check_ovp) {
		INIT_DELAYED_WORK(&auto_check_ovp, auto_check_ovp_func);
		schedule_delayed_work(&auto_check_ovp, CHECK_OVP_PERIOD);
	}
	tps65200_suspended = false;

	pr_tps_info("[TPS65200]: Driver registration done\n");
	return 0;
chg_stat_irq_failed:
   if(pdata->gpio_chg_stat) {
	   pr_tps_info("[TPS65200]: chg_stat irq failed.\n");
	   gpio_free(pdata->gpio_chg_stat);
   }
   return 0;
chg_int_irq_failed:
   if(pdata->gpio_chg_int) {
	   pr_tps_info("[TPS65200]: chg_int irq failed.\n");
	   gpio_free(pdata->gpio_chg_int);
   }
   return 0;
}

static int tps65200_remove(struct i2c_client *client)
{
	struct tps65200_i2c_client   *data = i2c_get_clientdata(client);
	pr_tps_info("TPS65200 remove\n");
	if (data->client && data->client != client)
		i2c_unregister_device(data->client);
	tps65200_i2c_module.client = NULL;
	if (tps65200_wq)
		destroy_workqueue(tps65200_wq);
	return 0;
}

static void tps65200_shutdown(struct i2c_client *client)
{
	u8 regh = 0;

	pr_tps_info("TPS65200 shutdown\n");
	tps65200_i2c_read_byte(&regh, 0x00);
	
	regh &= 0xDF;
	tps65200_i2c_write_byte(regh, 0x00);
}
static int tps65200_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("tps65200_suspend\n");
	tps65200_suspended = true;
	chg_stat_triggered = false;
	return 0;
}
static int tps65200_resume(struct i2c_client *client)
{
	printk("tps65200_resume\n");
	tps65200_suspended = false;
	if(chg_stat_triggered == true){
		printk("lock charger_wake_lock charger-tps65200 in %s\n",__func__);
		wake_lock_timeout(&charger_wake_lock, 5*HZ);
		chg_stat_triggered = false;
		printk("chg_stat triggered during suspend\n");
		queue_work(tps65200_wq, &chg_stat_work);
	}

	if(tps65200_int_triggered == true){
		tps65200_int_handler();
		tps65200_int_triggered = false;
	}

	if(tps65200_alarm == true){
		tps65200_alarm = false;
		queue_work(tps65200_wq, &check_alarm_work);
	}
	return 0;
}


static const struct i2c_device_id tps65200_id[] = {
	{ "tps65200", 0 },
	{  },
};
static struct i2c_driver tps65200_driver = {
	.driver.name    = "tps65200",
	.id_table   = tps65200_id,
	.probe      = tps65200_probe,
	.remove     = tps65200_remove,
	.shutdown   = tps65200_shutdown,
	.suspend    = tps65200_suspend,
	.resume     = tps65200_resume,
};

static int __init sensors_tps65200_init(void)
{
	int res;

	tps65200_low_chg = 0;
	chg_stat_enabled = 0;
	spin_lock_init(&chg_stat_lock);
	res = i2c_add_driver(&tps65200_driver);
	if (res)
		pr_tps_err("[TPS65200]: Driver registration failed \n");

	return res;
}

static void __exit sensors_tps65200_exit(void)
{
	kfree(chg_int_data);
	i2c_del_driver(&tps65200_driver);
}

MODULE_AUTHOR("Josh Hsiao <Josh_Hsiao@htc.com>");
MODULE_DESCRIPTION("tps65200 driver");
MODULE_LICENSE("GPL");

fs_initcall(sensors_tps65200_init);
module_exit(sensors_tps65200_exit);
