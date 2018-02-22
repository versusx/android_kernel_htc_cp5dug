/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>

#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <mach/globalregs.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include "usb_hw.h"

#if defined(CONFIG_ARCH_SC8825)||defined(CONFIG_ARCH_SCX35)
#define  USB_LDO_NAME	"vddusb"
#define  USB_CLK_NAME    	"clk_usb_ref"
#else
#define	 USB_LDO_NAME    "V_USB"
#define  USB_CLK_NAME    "clk_usb_ref"
#endif

static LIST_HEAD(g_lh_usb_notifier_list);
static LIST_HEAD(g_lh_mhl_detect_notifier_list);
extern int in_calibration(void);


static void usb_ldo_switch(int is_on)
{
	struct regulator *usb_regulator = NULL;

	if(usb_regulator == NULL){
		usb_regulator = regulator_get(NULL,USB_LDO_NAME);
	}
	if(usb_regulator){
		if(is_on){
			regulator_enable(usb_regulator);
		}else{
			regulator_disable(usb_regulator);
		}
		regulator_put(usb_regulator);
	}
}
#if 0
static int usb_clock_enable(int is_on)
{
	struct clk *usb_clock = NULL;

	usb_clock = clk_get(NULL,USB_CLK_NAME);
	if (usb_clock) {
		if (is_on) {
			if(usb_clk_status == 0){
				clk_enable(usb_clock);
				usb_clk_status = 1;
			}
		} else {
			if(usb_clk_status == 1){
				clk_disable(usb_clock);
				usb_clk_status = 0;
			}
		}
	}
	return 0;
}
#endif
#if defined(CONFIG_ARCH_SCX35)
static void usb_enable_module(int en)
{
	if (en){
		sci_glb_clr(REG_AON_APB_PWR_CTRL,BIT(0));
		sci_glb_set(REG_AP_AHB_AHB_EB,BIT_USB_EB);
	}else{
		sci_glb_set(REG_AON_APB_PWR_CTRL,BIT(0));
		sci_glb_clr(REG_AP_AHB_AHB_EB,BIT_USB_EB);
	}
}
#else
static void usb_enable_module(int en)
{
	if (en){
		usb_clock_enable(1);
		sprd_greg_clear_bits(REG_TYPE_GLOBAL,BIT(9),GR_CLK_GEN5);
	}else {
		usb_clock_enable(0);
		sprd_greg_set_bits(REG_TYPE_GLOBAL,BIT(9),GR_CLK_GEN5);
		sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,AHB_CTL0_USBD_EN,AHB_CTL0);
	}
}
#endif
void usb_phy_init(void)
{
#ifdef CONFIG_USB_CORE_IP_293A
#if defined(CONFIG_ARCH_SCX35)
	#if defined(CONFIG_MACH_DUMMY)
		sci_glb_write(REG_AP_APB_USB_PHY_TUNE, 0x44079f33, 0xffffffff);
	#endif
	#if defined(CONFIG_MACH_DUMMY)
		if(cp5dtu_get_board_revision() >= BOARD_PVT_A) {
			printk("[USB] pvt phy setting\n");
			sci_glb_write(REG_AP_APB_USB_PHY_TUNE, 0x440b3f33, 0xffffffff);
		} else {
			printk("[USB] not pvt phy setting\n");
			sci_glb_write(REG_AP_APB_USB_PHY_TUNE, 0x440b8f33, 0xffffffff);
		}
	#endif
	#if defined(CONFIG_MACH_CP5DUG)
		if(cp5dug_get_board_revision() >= BOARD_PVT_A) {
			printk("[USB] pvt phy setting\n");
			sci_glb_write(REG_AP_APB_USB_PHY_TUNE, 0x440b3f33, 0xffffffff);
		} else {
			printk("[USB] not pvt phy setting\n");
			sci_glb_write(REG_AP_APB_USB_PHY_TUNE, 0x440b8f33, 0xffffffff);
		}
	#endif

#else
        sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(11), USB_PHY_CTRL);
        sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(10), USB_PHY_CTRL);
        sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(20), USB_PHY_CTRL);
        sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(9), USB_PHY_CTRL);
        sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(8), USB_PHY_CTRL);
        sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(13), USB_PHY_CTRL);
		sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(12), USB_PHY_CTRL);
        sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(15)|BIT(14), USB_PHY_CTRL);
#endif
#else
    if (sprd_greg_read(REG_TYPE_AHB_GLOBAL,CHIP_ID) == CHIP_ID_8810S){
                
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(3)|BIT(2), USB_PHY_CTRL);
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(1) | BIT(0), USB_PHY_CTRL);
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(9), USB_PHY_CTRL);
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(16), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(17), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(13), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, BIT(12), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(15)|BIT(14), USB_PHY_CTRL);
                sprd_greg_write(REG_TYPE_AHB_GLOBAL,0x28,USB_SPR_REG);
        }else{
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(8), USB_PHY_CTRL);
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(17), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(16), USB_PHY_CTRL);
                sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(13)|BIT(12), USB_PHY_CTRL);
                sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(15)|BIT(14), USB_PHY_CTRL);
        }
#endif
}
static void usb_startup(void)
{
	usb_ldo_switch(1);
	mdelay(15);
	usb_enable_module(1);
	mdelay(12);
#if defined(CONFIG_ARCH_SCX35)
	sci_glb_set(REG_AP_AHB_AHB_RST,BIT(5)|BIT(6)|BIT(7));
	mdelay(5);
	sci_glb_clr(REG_AP_AHB_AHB_RST,BIT(5)|BIT(6)|BIT(7));
	sci_glb_set(REG_AP_AHB_AHB_EB,BIT_USB_EB);
#else
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(1)|BIT(2),AHB_CTL3);
	sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(6),AHB_CTL3);

	sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,BIT(6)|BIT(7),AHB_SOFT_RST);
	mdelay(5);
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL,BIT(6)|BIT(7),AHB_SOFT_RST);
	sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL,AHB_CTL0_USBD_EN,AHB_CTL0);
#endif
}

void udc_enable(void)
{
	pr_info("%s \n", __func__);
	usb_startup();
}
void udc_disable(void)
{
        pr_info("%s \n", __func__);
        usb_enable_module(0);
        usb_ldo_switch(0);
}


int usb_alloc_vbus_irq(void)
{
	int irq;

	gpio_request(EIC_CHARGER_DETECT,"sprd_ogt");
	gpio_direction_input(EIC_CHARGER_DETECT);
	irq = gpio_to_irq(EIC_CHARGER_DETECT);
	

	return irq;
}

void usb_free_vbus_irq(int irq)
{
	gpio_free(EIC_CHARGER_DETECT);
}

int usb_get_vbus_irq(void)
{
	int value;

	value = gpio_to_irq(EIC_CHARGER_DETECT);

	return value;
}
int usb_get_vbus_state(void)
{
	int value;
	if(in_calibration())
		return 1;
	value = gpio_get_value(EIC_CHARGER_DETECT);
	return !!value;
}

void usb_set_vbus_irq_type(int irq, int irq_type)
{
	if (irq_type == VBUS_PLUG_IN)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	else if (irq_type == VBUS_PLUG_OUT)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
	else {
		pr_warning("error type for usb vbus\n");
	}

	return;
}

static DEFINE_MUTEX(notify_sem);
static void send_usb_connect_notify(enum usb_connect_type connect_type)
{
	static struct t_usb_status_notifier *notifier;

	pr_info("send connect type %d\n", connect_type);
	mutex_lock(&notify_sem);
	list_for_each_entry(notifier, &g_lh_usb_notifier_list, notifier_link) {
		if (notifier->func != NULL) {
			notifier->func(connect_type);
		}
	}
	mutex_unlock(&notify_sem);
}

int usb_register_notifier(struct t_usb_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_sem);
	list_add(&notifier->notifier_link,
			&g_lh_usb_notifier_list);
	mutex_unlock(&notify_sem);
	return 0;
}
#ifndef DWC_DEVICE_ONLY
void charge_pump_set(int state)
{
#if 0
	struct regulator *usb_regulator = NULL;
#define  USB_CHG_PUMP_NAME	"chg_pump"

	if(usb_regulator == NULL){
		usb_regulator = regulator_get(NULL,USB_CHG_PUMP_NAME);
	}
	if(usb_regulator){
		if(state){
			regulator_enable(usb_regulator);
		}else{
			regulator_disable(usb_regulator);
		}
		regulator_put(usb_regulator);
	}
#else
	if(state) {
		send_usb_connect_notify(CONNECT_TYPE_INTERNAL);
	} else {
		send_usb_connect_notify(CONNECT_TYPE_CLEAR);
	}
#endif
}

int usb_alloc_id_irq(void)
{
	int irq;

	gpio_request(USB_OTG_CABLE_DETECT,"USB OTG CABLE");
	gpio_direction_input(USB_OTG_CABLE_DETECT);
	irq = gpio_to_irq(USB_OTG_CABLE_DETECT);
	set_irq_flags(irq, IRQF_VALID | IRQF_NOAUTOEN);

	return irq;
}

void usb_free_id_irq(int irq)
{
	gpio_free(USB_OTG_CABLE_DETECT);
}

int usb_get_id_irq(void)
{
	int value;

	value = gpio_to_irq(USB_OTG_CABLE_DETECT);

	return value;
}
int usb_get_id_state(void)
{
	int value;
	value = gpio_get_value(USB_OTG_CABLE_DETECT);
	return !!value;
}

void usb_set_id_irq_type(int irq, int irq_type)
{
	if (irq_type == OTG_CABLE_PLUG_IN)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
	else if (irq_type == OTG_CABLE_PLUG_OUT)
		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	else {
		pr_warning("error type for usb vbus\n");
	}

	return;
}
#endif


EXPORT_SYMBOL(udc_disable);
EXPORT_SYMBOL(udc_enable);
