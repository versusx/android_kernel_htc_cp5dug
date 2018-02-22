/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Copyright (c) 2012 High Tech Computer Corporation

Module Name:

		max17050_battery.c

Abstract:

		This module implements the power algorithm, including below concepts:
		1. Charging function control.
		2. Charging full condition.
		3. Recharge control.
		4. Battery capacity maintainance.
		5. Battery full capacity calibration.
---------------------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include "../staging/android/android_alarm.h"
#include <linux/alarmtimer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/max17050_battery.h>
#include <mach/htc_battery_types.h>
#include <linux/tps65200.h>
#include <mach/htc_battery.h>
#include <linux/i2c.h>  					
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <mach/board.h>
#include <linux/irq.h>
#include <linux/max17050_gauge.h>
#include <linux/reboot.h>
#include <linux/fs.h>
#include <mach/adc.h>

#define MSPERIOD(end, start)	ktime_to_ms(timespec_to_ktime(timespec_sub(end, start)))
#define BAT_ALRT_PIN 148
#define BAT_ALRT_LEVEL_1 0xAA 
#define BAT_ALRT_LEVEL_2 0xA5 
#define BAT_ALRT_LEVEL_3 0x9B 
#define BAT_ALRT_LEVEL_4 0x96 
#define ONE_HOUR	60

static struct work_struct bat_alrt_work;
static struct workqueue_struct *bat_alrt_wqueue;
static int high_temp_flag = 0;
static int high_temp_changed = 0;

struct max17050_device_info {

		struct device *dev;
		struct device *w1_dev;
		struct workqueue_struct *monitor_wqueue;
		
		struct delayed_work monitor_work;
		
		struct mutex lock;
		
		unsigned long update_time;	
		struct alarm alarm;
		struct wake_lock work_wake_lock;
		spinlock_t spin_lock;
		bool suspended;
		bool cable_changed;
		bool alarm_triggered;
		struct timespec last_poll;
};
static struct wake_lock alrt_wake_lock;
bool SERIOUS_LEVEL_ALRT = FALSE;

static struct poweralg_type poweralg = {0};
static struct poweralg_config_type config = {0};
static struct poweralg_config_type debug_config = {0};
BOOL is_need_battery_id_detection = TRUE;
static struct max17050_device_info *g_di_ptr = NULL;

static int g_first_update_charger_ctl = 1;

static int charger_control;
static int force_update_batt_info = 0;
static int force_set_chg = 0;
static int reverse_protecion_counter;
static int set_phone_call_in_poll;

#define FAST_POLL	(1 * 60)
#define SLOW_POLL	(60 * 60)
#define PHONE_CALL_POLL	(5 * 60)
#define PREDIC_POLL	20

#define HTC_BATTERY_I2C_DEBUG_ENABLE		0
#define HTC_BATTERY_MAX17050_DEBUG_ENABLE 	1

static UINT32 delta_time_sec = 0;
static UINT32 chg_en_time_sec = 0;
static UINT32 chg_kick_time_sec = 0;
static UINT32 super_chg_on_time_sec = 0;
static struct timespec cable_remove_timespec;
static struct timespec last_poll_timespec;
static int batt_full_eoc_stop;

#define HTC_EXT_UNKNOWN_USB_CHARGER		(1<<0)
#define HTC_EXT_CHG_UNDER_RATING		(1<<1)
#define HTC_EXT_CHG_SAFTY_TIMEOUT		(1<<2)
#define HTC_EXT_CHG_FULL_EOC_STOP		(1<<3)



#define MAX17050_STATUS_REG	0x01
#define MAX17050_AUX0_MSB		0x08
#define MAX17050_AUX0_LSB 	0x09
#define MAX17050_AUX1_MSB 	0x0A
#define MAX17050_AUX1_LSB 	0x0B
#define MAX17050_VOLT_MSB 	0x0C
#define MAX17050_VOLT_LSB 	0x0D
#define MAX17050_CURRENT_MSB	0x0E
#define MAX17050_CURRENT_LSB	0x0F
#define MAX17050_ACR_MSB  	0x10
#define MAX17050_ACR_LSB  	0x11


struct timespec timespec_set(const long secs, const unsigned long nsecs)
{
	return (struct timespec) { .tv_sec = secs, .tv_nsec = nsecs};
}
EXPORT_SYMBOL(timespec_set);

static BOOL b_is_charge_off_by_bounding = FALSE;
static void bounding_fullly_charged_level(int upperbd)
{
	static int pingpong = 1;
	int lowerbd;
	int current_level;
	b_is_charge_off_by_bounding = FALSE;
	if (upperbd <= 0)
		return; 
	lowerbd = upperbd - 5; 

	if (lowerbd < 0)
		lowerbd = 0;

	current_level = CEILING(poweralg.capacity_01p, 10);

	if (pingpong == 1 && upperbd <= current_level) {
		printk(DRIVER_ZONE "MFG: lowerbd=%d, upperbd=%d, current=%d, pingpong:1->0 turn off\n", lowerbd, upperbd, current_level);
		b_is_charge_off_by_bounding = TRUE;
		pingpong = 0;
	} else if (pingpong == 0 && lowerbd < current_level) {
		printk(DRIVER_ZONE "MFG: lowerbd=%d, upperbd=%d, current=%d, toward 0, turn off\n", lowerbd, upperbd, current_level);
		b_is_charge_off_by_bounding = TRUE;
	} else if (pingpong == 0 && current_level <= lowerbd) {
		printk(DRIVER_ZONE "MFG: lowerbd=%d, upperbd=%d, current=%d, pingpong:0->1 turn on\n", lowerbd, upperbd, current_level);
		pingpong = 1;
	} else {
		printk(DRIVER_ZONE "MFG: lowerbd=%d, upperbd=%d, current=%d, toward %d, turn on\n", lowerbd, upperbd, current_level, pingpong);
	}

}

static BOOL is_charge_off_by_bounding_condition(void)
{
	return b_is_charge_off_by_bounding;
}

void calibrate_id_ohm(struct battery_type *battery)
{
	if (!poweralg.charging_source || !poweralg.charging_enable){
		battery->id_ohm += 500; 		
	}
	else if (poweralg.charging_source == 2 && battery->current_mA >= 400 && battery->id_ohm >= 1500){
		battery->id_ohm -= 1500;		
	}
	else if (battery->id_ohm >= 700){
		battery->id_ohm -= 700; 		
	}
}

static BOOL is_charging_avaiable(void)
{
	BOOL chg_avalible = TRUE;
	if (poweralg.is_superchg_software_charger_timeout) chg_avalible = FALSE;
	if (poweralg.is_software_charger_timeout) chg_avalible = FALSE;
	if (!poweralg.protect_flags.is_charging_enable_available &&
		!poweralg.protect_flags.is_fake_room_temp)chg_avalible = FALSE;
	if (poweralg.protect_flags.is_charging_reverse_protect) {
		printk(DRIVER_ZONE "Disable charger due to reverse protection\n");
		chg_avalible = FALSE;
	}
	if (!poweralg.is_cable_in) chg_avalible = FALSE;
	if (poweralg.charge_state == CHARGE_STATE_PENDING) chg_avalible = FALSE;
	if (poweralg.charge_state == CHARGE_STATE_FULL_PENDING)	chg_avalible = FALSE;
	
	if (is_charge_off_by_bounding_condition()) chg_avalible = FALSE;
	if (poweralg.battery.id_index == BATTERY_ID_UNKNOWN) chg_avalible = FALSE;
	if (charger_control)
		chg_avalible = FALSE;
	printk("[BATT] chg_avalible = %d.\n", (chg_avalible == TRUE)?1:0);
	return chg_avalible; 
}

BOOL is_high_current_charging_avaialable(void)
{
	if (poweralg.protect_flags.is_charging_high_current_avaialble &&
		!poweralg.protect_flags.is_fake_room_temp)	return TRUE;
	return FALSE;
}
EXPORT_SYMBOL(is_high_current_charging_avaialable);

static BOOL is_super_current_charging_avaialable(void)
{
	if (!poweralg.is_super_ac) return FALSE;
	return TRUE;
}

static BOOL is_set_min_taper_current(void)
{
	if ((config.min_taper_current_ma > 0) &&
		(config.min_taper_current_mv > 0) &&
		(poweralg.battery.current_mA < config.min_taper_current_ma) &&
		(config.min_taper_current_mv < poweralg.battery.voltage_mV))
		return TRUE;

	return FALSE;
}
static int wait_to_full = 0;
static void update_next_charge_state(BOOL bFirstEntry)
{
	static UINT32 count_charging_full_condition;
	static UINT32 count_charge_over_load;
	int next_charge_state;
	int i;
	struct timespec end_timespec;

	getnstimeofday(&end_timespec );

	if (MSPERIOD(poweralg.start_timespec, end_timespec) > 0) {
		poweralg.start_timespec = end_timespec;
		printk(DRIVER_ZONE "Time changed, reassigned start time [%lld]\n",timespec_to_ns(&(poweralg.start_timespec)));
	}

	for (i = 0; i < 25; i++) 
	{
		next_charge_state = poweralg.charge_state;

		if (next_charge_state == poweralg.charge_state){
			
			
			if (poweralg.charge_state == CHARGE_STATE_UNKNOWN ||
				poweralg.charge_state == CHARGE_STATE_CHARGING ||
				poweralg.charge_state == CHARGE_STATE_PENDING ||
				poweralg.charge_state == CHARGE_STATE_FULL_WAIT_STABLE ||
				poweralg.charge_state == CHARGE_STATE_FULL_CHARGING ||
				poweralg.charge_state == CHARGE_STATE_FULL_RECHARGING ||
				poweralg.charge_state == CHARGE_STATE_FULL_PENDING){
				if (!poweralg.is_cable_in){
					next_charge_state = CHARGE_STATE_DISCHARGE;
				}
				else if (!poweralg.protect_flags.is_charging_enable_available){
					next_charge_state = CHARGE_STATE_PENDING;
				}
			}

			
			
			if (poweralg.charge_state == CHARGE_STATE_UNKNOWN ||
				poweralg.charge_state == CHARGE_STATE_DISCHARGE){
				if (poweralg.is_cable_in){
					next_charge_state = CHARGE_STATE_CHARGING;
				}
			}
		}

		

		
		if (next_charge_state == poweralg.charge_state){
			switch (poweralg.charge_state){
				case CHARGE_STATE_PREDICTION:
						getnstimeofday(&end_timespec);
						if (MSPERIOD(end_timespec, poweralg.start_timespec) >= 50 * 1000) {
							poweralg.start_timespec = end_timespec;
							printk(DRIVER_ZONE "reassign prediction start timestamp as [%lld]\n", timespec_to_ns(&end_timespec));
						} else if (MSPERIOD(end_timespec, poweralg.start_timespec) >= config.predict_timeout_sec * 1000) {
							printk(DRIVER_ZONE "predict done [%lld->%lld]\n", timespec_to_ns(&(poweralg.start_timespec)), timespec_to_ns(&end_timespec));
							next_charge_state = CHARGE_STATE_UNKNOWN;
						}
					break;
				case CHARGE_STATE_CHARGING:
					if (poweralg.capacity_01p >= 990){
						wait_to_full++;
					}else
						wait_to_full = 0;

					if (poweralg.capacity_01p > 990)
						poweralg.capacity_01p = 990;

					if (poweralg.battery.voltage_mV >= config.full_charging_mv &&
						poweralg.battery.current_mA >= 0 &&
						poweralg.battery.current_mA <= config.full_charging_ma){
						
						next_charge_state = CHARGE_STATE_FULL_WAIT_STABLE;
					}
					if (wait_to_full > ONE_HOUR) {
						next_charge_state = CHARGE_STATE_FULL_CHARGING;
						printk(DRIVER_ZONE "Capacity keep 99 percent over 1hr during charging,"
								"force update percent full -> %d\n", wait_to_full);
					}

					if (poweralg.battery.current_mA <= 0){
						
						if (count_charge_over_load < 5)
							count_charge_over_load++;
						else
							poweralg.is_charge_over_load = TRUE;
					}
					else{
						count_charge_over_load = 0;
						poweralg.is_charge_over_load = FALSE;
					}

					
					
					
					if (!poweralg.protect_flags.is_fake_room_temp &&
						get_cable_status() > CONNECT_TYPE_USB &&
						config.software_charger_timeout_sec &&
						config.software_charger_timeout_sec <=
						chg_en_time_sec) {
								printk(DRIVER_ZONE "Disable charger due"
								" to charging time lasts %u s > 16hr\n",
								chg_en_time_sec);
								poweralg.is_software_charger_timeout = TRUE;
					}
					break;
				case CHARGE_STATE_FULL_WAIT_STABLE:
					{
						
						if (poweralg.battery.voltage_mV >= config.full_charging_mv &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_charging_ma){
							if(poweralg.capacity_01p >= 990)
								count_charging_full_condition++;
							else poweralg.capacity_01p++;
						}
						else{
							count_charging_full_condition = 0;
							next_charge_state = CHARGE_STATE_CHARGING;
						}

						if (count_charging_full_condition >= 3){

							poweralg.capacity_01p = 1000;

							next_charge_state = CHARGE_STATE_FULL_CHARGING;
						}
					}
					break;
				case CHARGE_STATE_FULL_CHARGING:
					{
						
						getnstimeofday(&end_timespec);
						wait_to_full = 0;

						if (poweralg.battery.voltage_mV < config.voltage_exit_full_mv){
							if (poweralg.capacity_01p > 990)
								poweralg.capacity_01p = 990;
							next_charge_state = CHARGE_STATE_CHARGING;
						}
						else if (config.full_pending_ma != 0 &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_pending_ma){ 

							printk(DRIVER_ZONE " charge-full pending(%dmA)(%lld:%lld)\n",
								poweralg.battery.current_mA,
								timespec_to_ns(&(poweralg.start_timespec)),
								timespec_to_ns(&end_timespec));
							if (!poweralg.protect_flags.is_fake_room_temp)
								next_charge_state = CHARGE_STATE_FULL_PENDING;
						}
						else if (MSPERIOD(end_timespec, poweralg.start_timespec) >=
							config.full_charging_timeout_sec * 1000){

							printk(DRIVER_ZONE " charge-full (expect:%dsec)(%lld:%lld)\n",
								config.full_charging_timeout_sec,
								timespec_to_ns(&poweralg.start_timespec),
								timespec_to_ns(&end_timespec));
							if (!poweralg.protect_flags.is_fake_room_temp)
								next_charge_state = CHARGE_STATE_FULL_PENDING;
						}
					}
					break;
				case CHARGE_STATE_FULL_PENDING:
					if ((poweralg.battery.voltage_mV >= 0 &&
						poweralg.battery.voltage_mV < config.voltage_recharge_mv) ||
						(poweralg.capacity_01p>= 0 &&
						poweralg.capacity_01p <= config.capacity_recharge_p * 10)){
						printk(DRIVER_ZONE " Battery precentage = %d, re-charger!\n", poweralg.capacity_01p);
						
						next_charge_state = CHARGE_STATE_FULL_RECHARGING;
						batt_full_eoc_stop = false;
					} else {
						if (poweralg.protect_flags.is_temperature_fault == TRUE)
							batt_full_eoc_stop = false;
						else
							batt_full_eoc_stop = true;
					}
					break;
				case CHARGE_STATE_FULL_RECHARGING:
					{
						if (poweralg.battery.voltage_mV < config.voltage_exit_full_mv){
							if (poweralg.capacity_01p > 990)
								poweralg.capacity_01p = 990;
							next_charge_state = CHARGE_STATE_CHARGING;
						}
						else if (poweralg.battery.voltage_mV >= config.full_charging_mv &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_charging_ma){
							
							next_charge_state = CHARGE_STATE_FULL_CHARGING;
						}
					}
					break;
				case CHARGE_STATE_PENDING:
				case CHARGE_STATE_DISCHARGE:
					{
						getnstimeofday(&end_timespec);

						if (!poweralg.is_voltage_stable){
							if (MSPERIOD(end_timespec, poweralg.start_timespec) >=
								config.wait_votlage_statble_sec * 1000){

								printk(DRIVER_ZONE " voltage stable\n");
								poweralg.is_voltage_stable = TRUE;
							}
						}
					}

					if (poweralg.is_cable_in &&
						poweralg.protect_flags.is_charging_enable_available){
						
						next_charge_state = CHARGE_STATE_CHARGING;
					}
					break;
			}
		}

		
		
		if (next_charge_state != poweralg.charge_state){
			
			switch (poweralg.charge_state){
				case CHARGE_STATE_UNKNOWN:
					if (poweralg.capacity_01p > 990)
						poweralg.capacity_01p = 990;
					if (poweralg.capacity_01p < 0)
						poweralg.capacity_01p = 0;
					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;
					break;
				case CHARGE_STATE_PREDICTION:
					battery_param_update(&poweralg.battery,
						&poweralg.protect_flags, 1);
					if (poweralg.capacity_01p > 1000)
						poweralg.capacity_01p = 1000;
					if (poweralg.capacity_01p < 0)
						poweralg.capacity_01p = 0;

					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					break;
			}

			
			getnstimeofday(&poweralg.start_timespec);

			switch (next_charge_state){
				case CHARGE_STATE_DISCHARGE:
				case CHARGE_STATE_PENDING:
					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					poweralg.is_voltage_stable = FALSE;
					if(poweralg.charge_state > CHARGE_STATE_FULL_WAIT_STABLE) {
						u16 value;
						max17050_i2c_read(MAX17050_FG_RepCap, (u8 *)&value , 2);
						max17050_i2c_write(MAX17050_FG_FullCAP , (u8 *)&value ,2);
					}

					break;
				case CHARGE_STATE_CHARGING:
					poweralg.is_need_toggle_charger = FALSE;
					poweralg.last_charger_enable_toggled_time_ms = BAHW_MyGetMSecs();
					poweralg.is_software_charger_timeout = FALSE;   
					poweralg.is_charge_over_load = FALSE;
					count_charge_over_load = 0;
					poweralg.battery.charge_full_real_mAh = poweralg.battery.charge_full_design_mAh;
					break;
				case CHARGE_STATE_FULL_WAIT_STABLE:
					
					count_charging_full_condition = 0;
					break;
			}

			printk(DRIVER_ZONE " state change(%d->%d), full count=%d, over load count=%d [%lld]\n",
				poweralg.charge_state,
				next_charge_state,
				count_charging_full_condition,
				count_charge_over_load,
				timespec_to_ns(&(poweralg.start_timespec)));

			poweralg.charge_state = next_charge_state;
			continue;
		}

		break;
	}
}

static void __update_capacity(BOOL bFirstEntry)
{
	if (poweralg.charge_state == CHARGE_STATE_PREDICTION ||
		poweralg.charge_state == CHARGE_STATE_UNKNOWN){
		if (bFirstEntry) {
			
			poweralg.capacity_01p = 550;
			printk(DRIVER_ZONE "fake percentage (%d) during prediction.\n",
				poweralg.capacity_01p);
		}
	}
	else if (poweralg.charge_state == CHARGE_STATE_FULL_CHARGING ||
		poweralg.charge_state == CHARGE_STATE_FULL_RECHARGING ||
		poweralg.charge_state == CHARGE_STATE_FULL_PENDING){
		poweralg.capacity_01p = 1000;
	}

	if (poweralg.capacity_01p > 1000)
			poweralg.capacity_01p = 1000;
	if (poweralg.capacity_01p < 0)
			poweralg.capacity_01p = 0;
}

int check_charging_function(void)
{
	if (is_charging_avaiable()) {
		chg_en_time_sec += delta_time_sec;
		chg_kick_time_sec += delta_time_sec;
		
		if (poweralg.pdata->func_kick_charger_ic &&
			600 <= chg_kick_time_sec) {
			chg_kick_time_sec = 0;
			poweralg.pdata->func_kick_charger_ic(poweralg.charging_enable);
		}
		if (config.charger_hw_safety_timer_watchdog_sec) {
			if (config.charger_hw_safety_timer_watchdog_sec
				<= chg_en_time_sec) {
				printk(DRIVER_ZONE "need software toggle "
					"charger: lasts %d sec\n",
					chg_en_time_sec);
				chg_en_time_sec = 0;
				chg_kick_time_sec = 0;
				poweralg.is_need_toggle_charger = FALSE;
				poweralg.protect_flags.is_charging_reverse_protect = FALSE;
				max17050_charger_control(DISABLE);
				udelay(200);
			}
		}

		max17050_charger_control(poweralg.charging_source); 
		if (is_super_current_charging_avaialable())
				max17050_charger_control(ENABLE_SUPER_CHG);

		
		
		if ((config.min_taper_current_ma > 0)) {
			if (is_set_min_taper_current())
				max17050_charger_control(ENABLE_MIN_TAPER);
			else
				max17050_charger_control(DISABLE_MIN_TAPER);
		}
	} else {
		max17050_charger_control(DISABLE);
		chg_en_time_sec = 0;
		chg_kick_time_sec = 0;
		super_chg_on_time_sec = 0;
		poweralg.is_need_toggle_charger = FALSE;
		poweralg.protect_flags.is_charging_reverse_protect = FALSE;
	}

	if (config.debug_disable_hw_timer && poweralg.is_charge_over_load) {
		max17050_charger_control(DISABLE);
		printk(DRIVER_ZONE "Toggle charger due to HW disable charger.\n");
	}

	return 0;
}

void update_htc_extension_state(void)
{
	if (batt_full_eoc_stop != 0)
		poweralg.htc_extension |= HTC_EXT_CHG_FULL_EOC_STOP;
	else
		poweralg.htc_extension &= ~HTC_EXT_CHG_FULL_EOC_STOP;

	if (poweralg.is_software_charger_timeout)
		poweralg.htc_extension |= HTC_EXT_CHG_SAFTY_TIMEOUT;
	else
		poweralg.htc_extension &= ~HTC_EXT_CHG_SAFTY_TIMEOUT;
}

static struct timespec s_pre_time_timespec, pre_param_update_timespec;
#define MIN(X, Y) ((X) <= (Y) ? (X) : (Y))
BOOL do_power_alg(BOOL is_event_triggered)
{
	
	static BOOL s_bFirstEntry = TRUE;
	static INT32 s_level;
	struct timespec now_time_timespec;

	getnstimeofday(&now_time_timespec);

	

	if (MSPERIOD(pre_param_update_timespec, now_time_timespec) > 0
			|| MSPERIOD(s_pre_time_timespec, now_time_timespec) > 0
			|| MSPERIOD(cable_remove_timespec, now_time_timespec) > 0) {
		printk(DRIVER_ZONE "Time changed, update to the current time [%lld]\n",timespec_to_ns(&now_time_timespec));
		pre_param_update_timespec = now_time_timespec;
		cable_remove_timespec = now_time_timespec;
		s_pre_time_timespec = now_time_timespec;
	}

	if (s_bFirstEntry ||
			((MSPERIOD(now_time_timespec, pre_param_update_timespec) >= 3 * 1000) &&MSPERIOD(now_time_timespec ,cable_remove_timespec)>=10*1000)
			||is_event_triggered){
		pre_param_update_timespec = now_time_timespec;
		if (!battery_param_update(&poweralg.battery, &poweralg.protect_flags, 0)){
			printk(DRIVER_ZONE "battery_param_update fail, please retry next time.\n");
			return FALSE;
		}else{
			if(false == poweralg.battery.shutdown){
				int real_level = max17050_get_batt_level(&poweralg.battery);
				if(SERIOUS_LEVEL_ALRT && (!poweralg.charging_enable)){
					printk(KERN_ALERT"[BATT] Start -6 alg\n");
					if(poweralg.battery.voltage_mV <= 3300){
						poweralg.capacity_01p = (poweralg.capacity_01p > 60)? (poweralg.capacity_01p - 60) : 0;
						poweralg.capacity_01p = MIN(poweralg.capacity_01p, real_level);
					}else{
						SERIOUS_LEVEL_ALRT =  false;
						poweralg.capacity_01p = real_level;
					}
					if(poweralg.capacity_01p == 0 ) max17050_batt_softPOR();
				}else
					poweralg.capacity_01p = real_level;
			}else{
				printk(DRIVER_ZONE " Report capacity 0, device will shutdown\n");
				poweralg.capacity_01p = 0;
			}
		}
	}

	update_next_charge_state(s_bFirstEntry);

	
	update_htc_extension_state();

	if (poweralg.charge_state != CHARGE_STATE_UNKNOWN)
		poweralg.is_gauge_driver_ready = TRUE;

	if (s_bFirstEntry || MSPERIOD(now_time_timespec, cable_remove_timespec) >= 10 * 1000 || !is_event_triggered){
		
		__update_capacity(s_bFirstEntry);

		if (!is_event_triggered)
			s_bFirstEntry = FALSE;
		s_pre_time_timespec = now_time_timespec;
	}

	if (config.debug_disable_shutdown){
		if (poweralg.capacity_01p <= 0){
			poweralg.capacity_01p = 1;
		}
	}

	s_level = CEILING(poweralg.capacity_01p, 10);
	if (CEILING(poweralg.last_capacity_01p, 10) != s_level ||
		poweralg.battery.last_temp_01c != poweralg.battery.temp_01c) {

		poweralg.battery.last_temp_01c = poweralg.battery.temp_01c;
		poweralg.last_capacity_01p = poweralg.capacity_01p;
		max17050_blocking_notify(MAX17050_LEVEL_UPDATE, &s_level);
	}

	bounding_fullly_charged_level(config.full_level);

	
	if (config.superchg_software_charger_timeout_sec && poweralg.is_super_ac
		&& FALSE==poweralg.is_superchg_software_charger_timeout){
		super_chg_on_time_sec += delta_time_sec;
		if (config.superchg_software_charger_timeout_sec <= super_chg_on_time_sec){
			printk(DRIVER_ZONE "superchg charger on timer timeout: %u sec\n",
				super_chg_on_time_sec);
			poweralg.is_superchg_software_charger_timeout = TRUE;
		}
	}

	check_charging_function();


	htc_battery_update_change(force_update_batt_info);
	if (force_update_batt_info)
		force_update_batt_info = 0;

	printk(DRIVER_ZONE "S=%d P=%d(%x) chg=%d cable=%d%d%d flg=%d%d%d%d dbg=%d%d%d%d fst_dischg=%d/%d [%u], wait_to_full=%d, htc_extension=0x%x\n",
		poweralg.charge_state,
		poweralg.capacity_01p,
		poweralg.battery.capacity_raw,
		poweralg.charging_enable,
		poweralg.is_cable_in,
		poweralg.is_china_ac_in,
		poweralg.is_super_ac,
		poweralg.protect_flags.is_charging_enable_available,
		poweralg.protect_flags.is_charging_high_current_avaialble,
		poweralg.protect_flags.is_battery_dead,
		poweralg.protect_flags.is_charging_reverse_protect,
		config.debug_disable_shutdown,
		config.debug_fake_room_temp,
		config.debug_disable_hw_timer,
		config.debug_always_predict,
		poweralg.fst_discharge_capacity_01p,
		poweralg.fst_discharge_acr_mAh,
		BAHW_MyGetMSecs(),
		wait_to_full,
		poweralg.htc_extension);

	
	return TRUE;
}

static void poweralg_config_init(struct poweralg_config_type *config)
{ 
  
	config->full_charging_mv = 4110;
	config->full_charging_ma = 50;
	config->full_pending_ma = 0;	
	config->full_charging_timeout_sec = 60 * 60;
	config->voltage_recharge_mv = 4150;
	config->capacity_recharge_p = 0;
	config->voltage_exit_full_mv = 4100;
	config->min_taper_current_mv = 0; 
	config->min_taper_current_ma = 0; 
	config->wait_votlage_statble_sec = 1 * 60;
	config->predict_timeout_sec = 10;
	
	config->polling_time_in_charging_sec = 30;
	config->polling_time_in_discharging_sec = 30;

	config->enable_full_calibration = TRUE;
	config->enable_weight_percentage = TRUE;
	config->software_charger_timeout_sec = 0;	
	config->superchg_software_charger_timeout_sec = 0;	
	config->charger_hw_safety_timer_watchdog_sec =  0;	

	config->debug_disable_shutdown = FALSE;
	config->debug_fake_room_temp = FALSE;
	config->debug_disable_hw_timer = FALSE;
	config->debug_always_predict = FALSE;
	config->debug_fake_percentage = FALSE;
	config->full_level = 0;
}

void power_alg_init(struct poweralg_config_type *debug_config)
{
	poweralg.charge_state = CHARGE_STATE_UNKNOWN;
	poweralg.capacity_01p = 990;
	poweralg.last_capacity_01p = poweralg.capacity_01p;
	poweralg.fst_discharge_capacity_01p = 0;
	poweralg.fst_discharge_acr_mAh = 0;
	poweralg.is_need_calibrate_at_49p = TRUE;
	poweralg.is_need_calibrate_at_14p = TRUE;
	poweralg.is_charge_over_load = FALSE;
	poweralg.is_cable_in = FALSE;
	poweralg.is_china_ac_in = FALSE;
	poweralg.is_super_ac = FALSE;
	poweralg.is_voltage_stable = FALSE;
	poweralg.is_software_charger_timeout = FALSE;
	poweralg.is_superchg_software_charger_timeout = FALSE;
	poweralg.is_need_toggle_charger = FALSE;
	poweralg.last_charger_enable_toggled_time_ms = 0;
	getnstimeofday(&poweralg.start_timespec);
	cable_remove_timespec = timespec_set(0, 0);

	if(get_cable_status() == CONNECT_TYPE_USB) {
		poweralg.is_cable_in = TRUE;
		poweralg.charging_source = CONNECT_TYPE_USB;
		max17050_charger_control(ENABLE_SLOW_CHG);
	}
	else if (get_cable_status() == CONNECT_TYPE_AC) {
		poweralg.is_cable_in = TRUE;
		poweralg.is_china_ac_in = TRUE;
		poweralg.charging_source = CONNECT_TYPE_AC;
		max17050_charger_control(ENABLE_FAST_CHG);
	}
	else if (get_cable_status() == CONNECT_TYPE_9V_AC) {
		poweralg.is_cable_in = TRUE;
		poweralg.is_china_ac_in = TRUE;
		poweralg.is_super_ac = TRUE;
		poweralg.charging_source = CONNECT_TYPE_9V_AC;
		max17050_charger_control(ENABLE_SUPER_CHG);
	} else {
		poweralg.charging_source = CONNECT_TYPE_NONE;
		max17050_charger_control(DISABLE);
	}
	printk("[BATT] [%s]charging source = %d.\n", __FUNCTION__, poweralg.charging_source);
	if (poweralg.pdata && poweralg.pdata->func_poweralg_config_init)
		poweralg.pdata->func_poweralg_config_init(&config);
	else
		poweralg_config_init(&config);

#if (defined(CONFIG_MACH_PRIMODS) || defined(CONFIG_MACH_PROTOU) || defined(CONFIG_MACH_PROTODUG) || defined(CONFIG_MACH_MAGNIDS))
	 
	if (poweralg.battery.id_index!=BATTERY_ID_TWS_SDI_1650MAH &&
		poweralg.battery.id_index!=BATTERY_ID_FORMOSA_SANYO) {
			config.full_charging_mv = 4110;
			config.voltage_recharge_mv = 4150;
			config.voltage_exit_full_mv = 4000;
		}
#endif

	if (debug_config->debug_disable_shutdown)
		config.debug_disable_shutdown = debug_config->debug_disable_shutdown;
	if (debug_config->debug_fake_room_temp)
		config.debug_fake_room_temp = debug_config->debug_fake_room_temp;
	if (debug_config->debug_disable_hw_timer)
		config.debug_disable_hw_timer = debug_config->debug_disable_hw_timer;
	if (debug_config->debug_always_predict)
		config.debug_always_predict = debug_config->debug_always_predict;
	if (debug_config->debug_fake_percentage)
		config.debug_fake_percentage = debug_config->debug_fake_percentage;


	poweralg.protect_flags.is_charging_enable_available = TRUE;
	poweralg.protect_flags.is_battery_dead = FALSE;
	poweralg.protect_flags.is_charging_high_current_avaialble = FALSE;
	poweralg.protect_flags.is_fake_room_temp = config.debug_fake_room_temp;
	poweralg.protect_flags.is_charging_reverse_protect = FALSE;
	poweralg.protect_flags.func_update_charging_protect_flag = NULL;

	battery_param_init(&poweralg.battery);
	
}

void power_alg_preinit(void)
{
	
}

static BLOCKING_NOTIFIER_HEAD(max17050_notifier_list);
int max17050_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&max17050_notifier_list, nb);
}

int max17050_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&max17050_notifier_list, nb);
}


int max17050_blocking_notify(unsigned long val, void *v)
{
	int chg_ctl;

	if (val == MAX17050_CHARGING_CONTROL){
		chg_ctl = *(int *) v;
		if(false == debug_config.debug_force_fast_charging){
			if (poweralg.battery.id_index != BATTERY_ID_UNKNOWN && (TOGGLE_CHARGER == chg_ctl || ENABLE_MIN_TAPER == chg_ctl || DISABLE_MIN_TAPER == chg_ctl)) {
				if (0 == poweralg.charging_enable)
					return 0;
			} else if (poweralg.battery.id_index != BATTERY_ID_UNKNOWN && poweralg.charge_state != CHARGE_STATE_PREDICTION) {
				
				if ((high_temp_flag == 0 && is_high_current_charging_avaialable())|| (1 == high_temp_flag && !is_high_current_charging_avaialable())){
					high_temp_changed = 1;
					high_temp_flag = 1- high_temp_flag;
				}
				if (g_first_update_charger_ctl == 1) {
					printk(DRIVER_ZONE "first update charger control forcely.\n");
					g_first_update_charger_ctl = 0;
					poweralg.charging_enable = chg_ctl;
				} else if (poweralg.charging_enable == chg_ctl && force_set_chg == 0 && high_temp_changed != 1) {
					return 0;
				} else if (force_set_chg == 1) {
					force_set_chg = 0;
					poweralg.charging_enable = chg_ctl;
				} else
					poweralg.charging_enable = chg_ctl;
				high_temp_changed = 0;
			} else {
				if (poweralg.charging_enable == DISABLE) {
					
				} else if(get_battery_id() == BATTERY_ID_UNKNOWN){ 
					poweralg.charging_enable = DISABLE;
					*(int*)v = DISABLE;
					printk(DRIVER_ZONE "Charging disable due to Unknown battery\n");
				}
			}
		}else{
			printk(KERN_INFO"Force device fast charging\n");
			poweralg.charging_enable = ENABLE_FAST_CHG;
			*(int *) v = ENABLE_FAST_CHG;
		}

	}
	
	return blocking_notifier_call_chain(&max17050_notifier_list, val, v);
}


int max17050_get_battery_info(struct battery_info_reply *batt_info)
{

	battery_param_update(&poweralg.battery, &poweralg.protect_flags, 1);
	batt_info->batt_id = get_battery_id();
#if 0
	if(NULL != poweralg.get_battery_id)
		batt_info->batt_id = poweralg.get_battery_id(); 
	else{
		printk(KERN_ALERT"WARN: get_battery_id has not been registered yet\n");
		batt_info->batt_id = 0;
	}
#endif
	batt_info->batt_vol = poweralg.battery.voltage_mV; 
	batt_info->batt_temp = poweralg.battery.temp_01c; 
	batt_info->batt_current = poweralg.battery.current_mA; 
	batt_info->level = CEILING(poweralg.capacity_01p, 10); 
	batt_info->charging_source = poweralg.charging_source;
	batt_info->charging_enabled = poweralg.charging_enable;
	batt_info->full_bat = MAX17050_FULL_CAPACITY_DEFAULT; 
	batt_info->temp_fault = poweralg.protect_flags.is_temperature_fault;
	batt_info->batt_state = poweralg.is_gauge_driver_ready;
	
	if (config.debug_fake_percentage) {
		printk(DRIVER_ZONE "fake battery percentage for config 6 8000 level = %d\n", batt_info->level);
		batt_info->level = 70;
	}
	if (config.debug_fake_room_temp && (680 < poweralg.battery.temp_01c))
		batt_info->batt_temp = 680; 

	return 0;
}
struct gauge_reg_type {
	int reg;
	char * name;
	int value;
};
static struct gauge_reg_type dump_reg[] = {
	{MAX17050_FG_STATUS, "Status-Reg", 0}, 	
	{MAX17050_FG_RepCap, "Report-Cap", 20}, 	
	{MAX17050_FG_RepSOC, "Repoet-SOC", 2560}, 	
	{MAX17050_FG_TEMP, "Temp", 2560}, 		
	{MAX17050_FG_VCELL, "Battery-Vol", 128},		

	{MAX17050_FG_Current, "Current", 64}, 	
	{MAX17050_FG_AvgCurrent, "Avg-current", 64}, 	
	{MAX17050_FG_Qresidual, "QR", 20}, 	
	{MAX17050_FG_SOC, "Mixing-SOC", 2560},		
	{MAX17050_FG_AvSOC, "Ava-mixing-SOC", 2560},		

	{MAX17050_FG_RemCap, "Mixing-Ram-Cap", 20}, 	
	{MAX17050_FG_FullCAP, "Full-cap", 20}, 	
	{MAX17050_FG_FullSOCthr, "Full-SOCthr",0},
	{MAX17050_FG_Cycles, "Cycles", 10}, 	
	{MAX17050_FG_DesignCap, "Design-Cap", 20}, 	
	{MAX17050_FG_CONFIG, "Config", 0}, 	

	{MAX17050_FG_ICHGTerm, "ICHGterm", 0}, 	
	{MAX17050_FG_AvCap, "Remcap-Av", 20},		
	{MAX17050_FG_FullCAPNom, "Fullcap-Nam", 20},	
	{MAX17050_FG_AIN, "AIN", 0},		
	{MAX17050_FG_LearnCFG, "LearnCFG", 0}, 	

	{MAX17050_FG_SHFTCFG, "FilterCFG", 0}, 	
	{MAX17050_FG_RelaxCFG, "RelaxCFG", 0}, 	
	{MAX17050_FG_MiscCFG, "MiscCFG", 0}, 	
	{MAX17050_FG_TGAIN, "TGAIN", 0},		
	{MAX17050_FG_TOFF, "TOFF", 0},		

	{MAX17050_FG_FSTAT, "FSTAT", 0},		
	
	{MAX17050_FG_VFRemCap, "VFRamCap", 20}, 	
	{MAX17050_FG_QH, "QH", 20},		
	{MAX17050_FG_dQacc, "dQacc", 0},	
	{MAX17050_FG_dp_acc, "dPacc", 0},

	{MAX17050_FG_QRtable00, "QR00", 0},
	{MAX17050_FG_QRtable10, "QR10", 0},
	{MAX17050_FG_QRtable20, "QR20", 0},
	{MAX17050_FG_QRtable30, "QR30", 0},
	{0xFB, "VFOCV", 0},				
	{0xFF, "VFSOC", 0},				
};
ssize_t htc_battery_show_attr(struct device_attribute *attr, char *buf)
{
	int len = 0, i;
	int num_reg = 37;
	short int val;

	int vbus_v = htc_adc_to_vol(ADC_CHANNEL_VCHGSEN, sci_adc_get_value(ADC_CHANNEL_VCHGSEN, 1));
	if (!strcmp(attr->attr.name, "batt_attr_text")){
		len += scnprintf(buf +
				len,
				PAGE_SIZE -
				len,
				"Percentage(%%): %d;\n"
				"KADC(%%): %d;\n"
				"RARC(%%): %d;\n"
				"V_MBAT(mV): %d;\n"
				"Battery_ID: %d;\n"
				"pd_M: %d;\n"
				"Current(mA): %d;\n"
				"Temp: %d;\n"
				"Charging_source: %d;\n"
				"vBus_v(mv): %d;\n"
				"ACR(mAh): %d;\n"
				"FULL(mAh): %d;\n"
				"1st_dis_percentage(%%): %d;\n"
				"1st_dis_ACR: %d;\n"
				"config_dbg: %d%d%d%d;\n",
				poweralg.capacity_01p,
				CEILING(poweralg.battery.KADC_01p, 10),
				CEILING(poweralg.battery.RARC_01p, 10),
				poweralg.battery.voltage_mV,
				poweralg.battery.id_index,
				poweralg.battery.pd_m,
				poweralg.battery.current_mA,
				CEILING(poweralg.battery.temp_01c, 10),
				poweralg.charging_source,
				vbus_v,
				poweralg.battery.charge_counter_mAh,
				poweralg.battery.charge_full_real_mAh,
				CEILING(poweralg.fst_discharge_capacity_01p, 10),
				poweralg.fst_discharge_acr_mAh,
				config.debug_disable_shutdown,
				config.debug_fake_room_temp,
				config.debug_disable_hw_timer,
				config.debug_always_predict
		);
	}

	for( i = 0; i < num_reg; i++) {
		max17050_i2c_read(dump_reg[i].reg, (u8 *)&val, 2);
		if(dump_reg[i].value){
			int sig = val;
			if(dump_reg[i].reg == MAX17050_FG_TEMP||dump_reg[i].reg == MAX17050_FG_Current||dump_reg[i].reg == MAX17050_FG_AvgCurrent){			
				sig = sig*10/ (dump_reg[i].value);
			}else{
				unsigned short usig = (0xFFFF&val);
				sig = usig * 10 / dump_reg[i].value;
			}
			len += scnprintf(buf +
				len,
				PAGE_SIZE -
				len,
				"%s(0x%x): %d;\n",
				dump_reg[i].name,
				dump_reg[i].reg,
				sig);
		}
		else
			len += scnprintf(buf +
				len,
				PAGE_SIZE -
				len,
				"%s(0x%x): 0x%x;\n",
				dump_reg[i].name,
				dump_reg[i].reg,
				0xffff&val);
	}

	return len;
}
ssize_t htc_battery_show_htc_extension_attr(struct device_attribute *attr,
					char *buf)
{
	int len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len,"%d\n",
		poweralg.htc_extension);

	return len;
}

static void max17050_program_alarm(struct max17050_device_info *di, int seconds)
{
	struct timespec low_interval = timespec_set(seconds, 0);
	struct timespec next;

	get_monotonic_boottime(&(di->last_poll));
	next = timespec_add(di->last_poll, low_interval);

	delta_time_sec = seconds;
	printk(DRIVER_ZONE "%s:last_poll = %lld + %d s = %lld\n",
		__func__,timespec_to_ns(&(di->last_poll)),seconds,timespec_to_ns(&next));
	alarm_start(&di->alarm, timespec_to_ktime(next));
}

static int cable_status_handler_func(struct notifier_block *nfb,
	unsigned long action, void *param)
{
	u32 cable_type = (u32) action;

	printk("[BATT] %s(%d)\n",__FUNCTION__, cable_type);


	
	batt_full_eoc_stop = false;
	poweralg.is_super_ac = 0;
	if (cable_type == CONNECT_TYPE_NONE) {
		poweralg.is_cable_in = 0;
		poweralg.is_china_ac_in = 0;
		wait_to_full = 0;
		poweralg.charging_source = cable_type;
		getnstimeofday(&cable_remove_timespec);
		chg_en_time_sec = super_chg_on_time_sec = delta_time_sec = chg_kick_time_sec = 0;
		force_update_batt_info = 1;
		if (TRUE == poweralg.is_superchg_software_charger_timeout) {
			poweralg.is_superchg_software_charger_timeout = FALSE;	
			printk(DRIVER_ZONE "reset superchg software timer\n");
		}
		if (!is_charging_avaiable()) {
			poweralg.protect_flags.is_charging_reverse_protect = FALSE;
		}
		if(g_di_ptr)
			g_di_ptr->cable_changed = true;
	} else if (cable_type == CONNECT_TYPE_USB
			||cable_type == CONNECT_TYPE_AC
			||cable_type == CONNECT_TYPE_9V_AC) {
		poweralg.is_cable_in = 1;
		poweralg.is_china_ac_in = 1;
		if(cable_type == CONNECT_TYPE_9V_AC)
			poweralg.is_super_ac = 1;
		poweralg.charging_source = cable_type;
		getnstimeofday(&cable_remove_timespec);
		chg_en_time_sec = super_chg_on_time_sec = delta_time_sec = chg_kick_time_sec = 0;
		force_update_batt_info = 1;
		force_set_chg = 1;
		SERIOUS_LEVEL_ALRT =  false;
		if(g_di_ptr)
			g_di_ptr->cable_changed = true;
	} else if (cable_type == 0xff) {
		if (param)
			config.full_level = *(INT32 *)param;
		printk(DRIVER_ZONE "Set the full level to %d\n", config.full_level);
		return NOTIFY_OK;
	} else if (cable_type == 0x10) {
		poweralg.protect_flags.is_fake_room_temp = TRUE;
		printk(DRIVER_ZONE "enable fake temp mode\n");
		return NOTIFY_OK;
	}

	if(g_di_ptr &&
		g_di_ptr->cable_changed == true){
		alarm_try_to_cancel(&g_di_ptr->alarm);
		max17050_program_alarm(g_di_ptr, 0);
	}
	return NOTIFY_OK;
}

void reverse_protection_handler(int status)
{
	if (status == REVERSE_PROTECTION_HAPPEND) {
		if (poweralg.charging_source != CONNECT_TYPE_NONE) {
			poweralg.protect_flags.is_charging_reverse_protect = TRUE;
			reverse_protecion_counter++;
			printk(DRIVER_ZONE "%s: reverse protection is happened: %d\n",__func__,reverse_protecion_counter);
		}
	}
	else if (status == REVERSE_PROTECTION_CONTER_CLEAR) {
		reverse_protecion_counter = 0;
	}
}
EXPORT_SYMBOL(reverse_protection_handler);

static struct notifier_block cable_status_handler =
{
  .notifier_call = cable_status_handler_func,
};

void max17050_charger_control(int type)
{
	int charge_type = type;
	printk(DRIVER_ZONE "%s(%d)\n",__func__, type);

	switch (charge_type){
		case DISABLE:
			
			max17050_blocking_notify(MAX17050_CHARGING_CONTROL, &charge_type);
			break;
		case ENABLE_SLOW_CHG:
			max17050_blocking_notify(MAX17050_CHARGING_CONTROL, &charge_type);
			break;
		case ENABLE_FAST_CHG:
			max17050_blocking_notify(MAX17050_CHARGING_CONTROL, &charge_type);
			break;
		case ENABLE_SUPER_CHG:
			max17050_blocking_notify(MAX17050_CHARGING_CONTROL, &charge_type);
			break;
		case TOGGLE_CHARGER:
			max17050_blocking_notify(MAX17050_CHARGING_CONTROL, &charge_type);
			break;
		case ENABLE_MIN_TAPER:
			max17050_blocking_notify(MAX17050_CHARGING_CONTROL, &charge_type);
			break;
		case DISABLE_MIN_TAPER:
			max17050_blocking_notify(MAX17050_CHARGING_CONTROL, &charge_type);
			break;
	}
}

static int alarm_delta_is_ready(void)
{
	struct timespec now, boottime, zero_time;
	getnstimeofday(&now);
	getboottime(&boottime);
	zero_time = timespec_set(0, 0);
	if (timespec_equal(&zero_time, &now) ||
		timespec_equal(&zero_time, &boottime))
		return 0;
	else
		return 1;
}

static void max17050_battery_work(struct work_struct *work)
{
	struct max17050_device_info *di = container_of(work,
				struct max17050_device_info, monitor_work.work); 
	static int alarm_delta_ready = 0;
	unsigned long flags;

	if (!alarm_delta_ready && !alarm_delta_is_ready()) {   
		printk(DRIVER_ZONE "alarm delta isn't ready so delay 500ms\n");
		cancel_delayed_work(&di->monitor_work);
		queue_delayed_work(di->monitor_wqueue, &di->monitor_work, msecs_to_jiffies(500));
		alarm_delta_ready = 1;
		return;
	} else
		alarm_delta_ready = 1;

	if(di->cable_changed == true){
		di->cable_changed = false;
		printk("charging source has changed to be:%d\n",poweralg.charging_source);
		max17050_blocking_notify(MAX17050_CHARGING_CONTROL, &poweralg.charging_source);
	}

	do_power_alg(0);
	getnstimeofday(&last_poll_timespec);
	get_monotonic_boottime(&di->last_poll);

	
	spin_lock_irqsave(&di->spin_lock, flags);
	printk("release wakelock:max17050-battery\n");
	wake_unlock(&di->work_wake_lock);

	max17050_program_alarm(di, FAST_POLL);

	spin_unlock_irqrestore(&di->spin_lock, flags);
}

static enum alarmtimer_restart max17050_battery_alarm(struct alarm *alarm, ktime_t now)
{
	struct max17050_device_info *di = container_of(alarm, struct max17050_device_info, alarm);

	di->alarm_triggered = false;
	if(di->suspended != true){
		printk("max17050_battery_alarm:acquire wake lock:max17050-battery\n");
		wake_lock(&di->work_wake_lock);
		queue_delayed_work(di->monitor_wqueue, &di->monitor_work, 0);
	}else{
		di->alarm_triggered = true;
		printk("in suspend now, not queue work but waiting for resume\n");
	}
	
	return ALARMTIMER_NORESTART;
}

static void enable_bat_alrt(int en)
{
	int v,ret;

	max17050_i2c_read(MAX17050_FG_CONFIG, (u8 *)&v, 2);

	if ( en ) {
		v |=  (1 << 2);
		v &= ~(1 << 5);
	} else {
		v &= ~(1 << 2);
		v |=  (1 << 5);
	}
	ret = max17050_i2c_write(MAX17050_FG_CONFIG, (u8 *)&v, 2);

	v = 0;
	max17050_i2c_read(MAX17050_FG_CONFIG, (u8 *)&v, 2);
	printk(DRIVER_ZONE "%s: write 0x%x to  CONFIG\n", __func__, v);

	if (unlikely(ret < 0))
		printk(DRIVER_ZONE "%s: Failed to write reg  CONFIG, ret=%d\n", __func__, ret);
}

static void set_bat_alrt_vol_to(int vol)
{
	int v, ret;

	max17050_i2c_read(MAX17050_FG_VALRT_Th, (u8 *)&v, 2);
	v &= ~0xFF;
	v |= vol;
	ret = max17050_i2c_write(MAX17050_FG_VALRT_Th, (u8 *)&v, 2);
	v = 0;
	max17050_i2c_read(MAX17050_FG_VALRT_Th, (u8 *)&v, 2);
	printk(DRIVER_ZONE " write result of valrt register 0x%x\n", v);

	if (unlikely(ret < 0))
		printk(DRIVER_ZONE " %s: Failed to write reg  VALRT_Th, ret=%d\n", __func__, ret);

}
static void max17050_set_next_alrt_level(void)
{
	int real_vol = max17050_get_average_vol();
	int next_level = BAT_ALRT_LEVEL_1;
	char* trig_level;

	if(real_vol >= BAT_ALRT_LEVEL_1 * 20){
		next_level = BAT_ALRT_LEVEL_1;
		trig_level = "3400mV";
#if defined(CONFIG_MACH_DUMMY)||defined(CONFIG_MACH_CP5DUG)||defined(CONFIG_MACH_DUMMY)||defined(CONFIG_MACH_DUMMY)||defined(CONFIG_MACH_DUMMY)
	}else if(real_vol >= BAT_ALRT_LEVEL_2 * 20){
		next_level = BAT_ALRT_LEVEL_2;
		trig_level = "3300mV";
#endif
	}else if(real_vol >= BAT_ALRT_LEVEL_3 * 20){
		next_level = BAT_ALRT_LEVEL_3;
		trig_level = "3100mV";
	}else{
		if(poweralg.charging_source == 0)
			SERIOUS_LEVEL_ALRT = TRUE;
		next_level = BAT_ALRT_LEVEL_4;
		trig_level = "3000mV";
	}
	printk(DRIVER_ZONE " Set next trigger level:%s\n",trig_level);
	set_bat_alrt_vol_to(next_level);
}

static void bat_alrt_work_func(struct work_struct *work)
{

	u16 v = 0;
	char* trig_level;
	max17050_i2c_read(MAX17050_FG_VALRT_Th, (u8 *)&v, 2);
	switch(v&0xFF){
		case BAT_ALRT_LEVEL_1:
			trig_level = "3400mV";
			break;
#if defined(CONFIG_MACH_DUMMY)||defined(CONFIG_MACH_CP5DUG)||defined(CONFIG_MACH_DUMMY)||defined(CONFIG_MACH_DUMMY)||defined(CONFIG_MACH_DUMMY)
		case BAT_ALRT_LEVEL_2:
			trig_level = "3300mV";
			break;
#endif
		case BAT_ALRT_LEVEL_3:
			trig_level = "3100mV";
			break;
		case BAT_ALRT_LEVEL_4:
			trig_level = "3000mV";
			break;
		default:
			trig_level = "None of the pre-set, it's abnormal";
			break;
	}
	printk(DRIVER_ZONE "Current trigger level:%s\n",trig_level);

	max17050_set_next_alrt_level();
	if(g_di_ptr)
		do_power_alg(1);
	wake_unlock(&alrt_wake_lock);
	return;
}
static bool batt_alrt = false;
static irqreturn_t bat_alrt_handler(int irq, void *data)
{
	if(NULL == g_di_ptr){
		queue_work(bat_alrt_wqueue, &bat_alrt_work);
		return IRQ_HANDLED;
	}

	printk(DRIVER_ZONE "battery: get interrupt of battery voltage alert. suspended:%d\n", g_di_ptr->suspended);
	wake_lock(&alrt_wake_lock);
	if(g_di_ptr->suspended == false){
		queue_work(bat_alrt_wqueue, &bat_alrt_work);
	}else{
		batt_alrt = true;
	}
	return IRQ_HANDLED;
}

static int max17050_battery_probe(struct platform_device *pdev)
{
	int rc;
	struct max17050_device_info *di;
	max17050_platform_data *pdata = pdev->dev.platform_data; 
	poweralg.pdata = pdev->dev.platform_data;
	poweralg.battery.thermal_id = pdata->func_get_thermal_id();
	poweralg.get_battery_id = pdata->func_get_battery_id;
	poweralg.battery.id_index = poweralg.get_battery_id();
	poweralg.battery.charge_full_real_mAh = MAX17050_FULL_CAPACITY_DEFAULT;
	is_need_battery_id_detection = FALSE;


	power_alg_preinit();
	power_alg_init(&debug_config);
	
	poweralg.protect_flags.func_update_charging_protect_flag = pdata->func_update_charging_protect_flag;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di){
		rc = -ENOMEM;
		goto fail_register;
	}

	g_di_ptr = di; 
	di->cable_changed = false;
	di->suspended = false;
	di->alarm_triggered = false;

	di->update_time = jiffies;
	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;

	
	INIT_DELAYED_WORK(&di->monitor_work, max17050_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));

	
	get_monotonic_boottime(&di->last_poll);
	spin_lock_init(&di->spin_lock);

	if (!di->monitor_wqueue){
		rc = -ESRCH;
		goto fail_workqueue;
	}
	wake_lock_init(&di->work_wake_lock, WAKE_LOCK_SUSPEND, "max17050-battery");
	alarm_init(&(di->alarm),
		ALARM_BOOTTIME,
		max17050_battery_alarm);
	wake_lock(&di->work_wake_lock);
	if (alarm_delta_is_ready()) {  
		printk(DRIVER_ZONE "[probe]alarm delta is ready\n");
		queue_delayed_work(di->monitor_wqueue, &di->monitor_work, 0);
		
	} else {
		printk(DRIVER_ZONE "[probe] alarm delta isn't ready so delay 500ms\n");
		queue_delayed_work(di->monitor_wqueue, &di->monitor_work, msecs_to_jiffies(500));
	}

	return 0;

fail_workqueue : fail_register : kfree(di);
	return rc;
}

int max17050_charger_switch(int charger_switch)
{
	printk("%s: charger_switch=%d\n",
		__func__, charger_switch);

	if (charger_switch == 0) {
		chg_en_time_sec = 0;
		chg_kick_time_sec = 0;
		super_chg_on_time_sec = 0;
		poweralg.is_need_toggle_charger = FALSE;
		poweralg.protect_flags.is_charging_reverse_protect = FALSE;
		charger_control = 1;
	} else {
		charger_control = 0;
	}
	if (g_di_ptr) {
		alarm_try_to_cancel(&g_di_ptr->alarm);
		max17050_program_alarm(g_di_ptr, 0);
	}

	return 0;

}
EXPORT_SYMBOL(max17050_charger_switch);

static int max17050_battery_remove(struct platform_device *pdev)
{
	struct max17050_device_info *di = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&di->monitor_work);
	
	destroy_workqueue(di->monitor_wqueue);

	return 0;
}

void max17050_phone_call_in(int phone_call_in)
{
	set_phone_call_in_poll = phone_call_in;
	if(set_phone_call_in_poll){
		enable_bat_alrt(0);
	}else{
		max17050_set_next_alrt_level();
		enable_bat_alrt(1);
	}
}

static int max17050_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max17050_device_info *di = platform_get_drvdata(pdev);
	unsigned long flags;
	int poll = 0;
	di->alarm_triggered = false;
	printk("max17050_suspend charger type:%d\n",poweralg.charging_source);
	spin_lock_irqsave(&di->spin_lock, flags);
	poll = SLOW_POLL;
	if(poweralg.charging_source != CONNECT_TYPE_NONE)
		poll >>= 1;
	if(set_phone_call_in_poll)
		poll = PHONE_CALL_POLL;
	printk("Poll new alarm:%d\n",poll);
	max17050_program_alarm(di, poll);
	spin_unlock_irqrestore(&di->spin_lock, flags);
	di->suspended = true;
	
	return 0;
}
static void max17050_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max17050_device_info *di = platform_get_drvdata(pdev);
	unsigned long flags;
	struct timespec now;
	
	
	spin_lock_irqsave(&di->spin_lock, flags);
	if(di->cable_changed == true){
		printk("charging source has been changed:%d, queue alarm immediately\n",poweralg.charging_source);
	}
	getnstimeofday(&now);
	if (MSPERIOD(pre_param_update_timespec, now) > 0
			|| MSPERIOD(s_pre_time_timespec, now) > 0
			|| MSPERIOD(cable_remove_timespec, now) > 0) {
		printk(DRIVER_ZONE "Time changed, update to the current time [%lld]\n",timespec_to_ns(&now));
		pre_param_update_timespec = now;
		cable_remove_timespec = now;
		s_pre_time_timespec = now;
	}
	if(batt_alrt == true){
		printk("got battery alert during suspend\n");
		queue_work(bat_alrt_wqueue, &bat_alrt_work);
		batt_alrt = false;
	}

	if(di->alarm_triggered == true){
		printk("acquire wake lock:max17050-battery in %s\n",__func__);
		wake_lock(&di->work_wake_lock);
		queue_delayed_work(di->monitor_wqueue, &di->monitor_work, 0);
		di->alarm_triggered = false;
	}else{
		int no_im = (MSPERIOD(now,pre_param_update_timespec)>= FAST_POLL * 1000)? 0:1;
		max17050_program_alarm(di, FAST_POLL*no_im);
	}
	di->suspended = false;
	spin_unlock_irqrestore(&di->spin_lock, flags);
	printk("battery resume acomplished\n");
}

static struct dev_pm_ops max17050_pm_ops = {
       .prepare = max17050_suspend,
       .complete  = max17050_resume,
};

MODULE_ALIAS("platform:max17050-battery");
static struct platform_driver max17050_battery_driver =
{
	.driver = {
	.name = "max17050-battery",
	.pm = &max17050_pm_ops,
	},
	.probe = max17050_battery_probe,
	.remove = max17050_battery_remove,
};

static int __init max17050_fake_temp_setup(char *str)
{
	if(!strcmp(str,"true"))
		debug_config.debug_fake_room_temp = TRUE;
	else
		debug_config.debug_fake_room_temp = FALSE;
	return 1;
}
__setup("battery_fake_temp=", max17050_fake_temp_setup);

static int __init max17050_fake_percentage_setup(char *str)
{
	if(!strcmp(str,"true"))
		debug_config.debug_fake_percentage = TRUE;
	else
		debug_config.debug_fake_percentage = FALSE;
	return 1;
}
__setup("battery_fake_percentage=", max17050_fake_percentage_setup);
static int __init max17050_force_fast_charging(char *str)
{
	debug_config.debug_force_fast_charging = (!strcmp(str,"true"));
	printk(KERN_INFO"battery %s fast charging\n",debug_config.debug_force_fast_charging?"force":"not force");
	return 1;
}
__setup("battery_force_fast_charging=",max17050_force_fast_charging);
static int max17050_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc , gpio_irq;

	wake_lock_init(&alrt_wake_lock, WAKE_LOCK_SUSPEND, "alrt_present");
	INIT_WORK(&bat_alrt_work, bat_alrt_work_func);
	bat_alrt_wqueue = create_singlethread_workqueue("bat_alrt");
	if (!bat_alrt_wqueue){
			return -ESRCH;
	}

	rc = gpio_request(BAT_ALRT_PIN, NULL);
	if(rc)
		goto batt_alrt_irq_failed;
	max17050_set_next_alrt_level();
	gpio_irq = gpio_to_irq(BAT_ALRT_PIN);
	rc = request_any_context_irq( gpio_irq,
		bat_alrt_handler,
		IRQF_TRIGGER_FALLING,
		"bat_alrt", NULL);

	if (rc < 0){
		goto batt_alrt_irq_failed;
	}else{
		printk(DRIVER_ZONE "[probe]request bat_alrt irq success!\n");
	}
	batt_alrt = false;
	enable_bat_alrt(1);
	return 0;

batt_alrt_irq_failed:
	if(BAT_ALRT_PIN)
		gpio_free(BAT_ALRT_PIN);
	printk(DRIVER_ZONE "[probe]request bat_alrt irq failed!\n");
	return rc;
}

static int max17050_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id max17050_id[] = {
	{"max17050", 0 },
	{ },
};
static struct i2c_driver max17050_driver = {
	.driver.name    = "max17050",
	.id_table   = max17050_id,
	.probe      = max17050_probe,
	.remove     = max17050_remove,
};

static int __init max17050_battery_init(void)
{
	int ret;

	charger_control = 0;

	register_notifier_cable_status(&cable_status_handler);

	ret = max17050_gauge_init();
	if (ret < 0){
		return ret;
	}
	ret = i2c_add_driver(&max17050_driver);
	if (ret) {
		printk(DRIVER_ZONE "add max17050_driver failed!\n");
		return ret;
	}else{
		printk(DRIVER_ZONE "add max17050_driver ok!\n");
	}

	
	return platform_driver_register(&max17050_battery_driver);
}

static void __exit max17050_battery_exit(void)
{
	max17050_gauge_exit();
	platform_driver_unregister(&max17050_battery_driver);
}

module_init(max17050_battery_init);
module_exit(max17050_battery_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("max17050 battery driver");

