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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/pn544.h>

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/time.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/localtimer.h>

#include <mach/hardware.h>
#include <linux/i2c.h>
#ifdef CONFIG_TOUCHSCREEN_FT5306
#include <linux/i2c/ft5306_ts.h>
#endif
#include <linux/i2c/lis3dh.h>
#include <linux/i2c/ltr_558als.h>
#include <linux/akm8975.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <mach/serial_sprd.h>
#include <mach/adi.h>
#include <mach/adc.h>
#include <mach/pinmap.h>
#include <linux/mpu.h>
#include <linux/akm8975.h>
#include <linux/irq.h>
#include <linux/persistent_ram.h>

#include <mach/sci.h>
#include <mach/hardware.h>
#include <mach/sci_glb_regs.h>

#include "devices.h"

/*  HTC_PROC_EMMC_START */
#include <mach/board.h>
#include <mach/board_htc.h>
#include <linux/proc_fs.h>
/*  HTC_PROC_EMMC_END */

/* IRQ's for the multi sensor board */
#define MPUIRQ_GPIO 212
#define IDENTIFY_TOUCH_PANEL_GPIO 65

#include <linux/regulator/consumer.h>
#include <mach/regulator.h>
#include <linux/spi/mxd_cmmb_026x.h>
#include <linux/usb/android_board.h>
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_3K
#include <linux/synaptics_i2c_rmi.h>
#endif

extern void __init sci_reserve(void);
extern void __init sci_map_io(void);
extern void __init sci_init_irq(void);
extern void __init sci_timer_init(void);
extern int __init sci_clock_init(void);


static struct platform_device rfkill_device;
static struct platform_device brcm_bluesleep_device;
static struct platform_device kb_backlight_device;
static struct platform_device android_usb_device;

static struct platform_device *devices[] __initdata = {
	&sprd_serial_device0,
	&sprd_serial_device1,
	&sprd_serial_device2,
	&sprd_serial_device3,
	&sprd_device_rtc,
	&sprd_eic_gpio_device,
	&sprd_nand_device,
	&sprd_lcd_device0,
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&sprd_ram_console,
#endif
//	&sprd_backlight_device,
	&led_pwm_device,
	&sprd_i2c_device0,
	&sprd_i2c_device1,
	&sprd_i2c_device2,
	&sprd_i2c_device3,
	&sprd_spi0_device,
	&sprd_spi1_device,
	&sprd_spi2_device,
	&sprd_keypad_device,
	&sprd_audio_platform_vbc_pcm_device,
	&sprd_audio_cpu_dai_vaudio_device,
	&sprd_audio_cpu_dai_vbc_device,
	&sprd_audio_codec_sprd_codec_device,
	&sprd_battery_device,
#ifdef CONFIG_ION
	&sprd_ion_dev,
#endif
	&sprd_emmc_device,
	&sprd_sdio0_device,
	&sprd_sdio1_device,
	&sprd_sdio2_device, //wifi
	&sprd_vsp_device,
	&sprd_dcam_device,
	&sprd_scale_device,
	&sprd_rotation_device,
	&sprd_sensor_device,
	&sprd_isp_device,
	&sprd_ahb_bm0_device,
	&sprd_ahb_bm1_device,
	&sprd_ahb_bm2_device,
	&sprd_ahb_bm3_device,
	&sprd_ahb_bm4_device,
	&sprd_axi_bm0_device,
	&sprd_axi_bm1_device,
	&sprd_axi_bm2_device,
#if 0
	&rfkill_device,
	&brcm_bluesleep_device,
#endif
#ifdef CONFIG_SIPC
	&sprd_cproc_td_device,
        &sprd_spipe_td_device,
        &sprd_slog_td_device,
        &sprd_stty_td_device,
	&sprd_seth0_td_device,
	&sprd_seth1_td_device,
	&sprd_seth2_td_device,
#endif
	&kb_backlight_device,
#ifdef CONFIG_USB_G_ANDROID
	&android_usb_device,
#endif
};

#ifdef CONFIG_FLASHLIGHT_TPS61310
#include <linux/htc_flashlight.h>
#ifdef CONFIG_MSM_CAMERA_FLASH
static int flashlight_control(int mode)
{
	return tps61310_flashlight_control(mode);
}
#endif
static void config_flashlight_gpios(void);
static struct TPS61310_flashlight_platform_data tiger_flashlight_data = {
	.gpio_init = config_flashlight_gpios,
	.tps61310_strb0 = GPIO_FLASH_EN,
	.tps61310_strb1 = GPIO_TORCH_EN,
	.flash_duration_ms = 600,
	.led_count = 1,
	.mode_pin_suspend_state_low = 1,
	//.Tx_mask = 131,
};

//HTC_CAM_START chuck config the IP_GPIO for accessied GPIO for ISP_FW
static void config_flashlight_gpios(void){
        gpio_request(GPIO_FLASH_EN,"flash_en");
        gpio_direction_output(GPIO_FLASH_EN, 0);
        gpio_request(GPIO_TORCH_EN,"torch_en");
        gpio_direction_output(GPIO_TORCH_EN, 0);
}
//HTC_CAM_END
#endif

/* BT suspend/resume */
static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= GPIO_BT2AP_WAKE,
		.end	= GPIO_BT2AP_WAKE,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= GPIO_AP2BT_WAKE,
		.end	= GPIO_AP2BT_WAKE,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device brcm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

/* RFKILL */
static struct resource rfkill_resources[] = {
	{
		.name   = "bt_power",
		.start  = GPIO_BT_POWER,
		.end    = GPIO_BT_POWER,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bt_reset",
		.start  = GPIO_BT_RESET,
		.end    = GPIO_BT_RESET,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device rfkill_device = {
	.name = "rfkill",
	.id = -1,
	.num_resources	= ARRAY_SIZE(rfkill_resources),
	.resource	= rfkill_resources,
};

/* keypad backlight */
static struct platform_device kb_backlight_device = {
	.name           = "keyboard-backlight",
	.id             =  -1,
};

static struct sys_timer __timer = {
	.init = sci_timer_init,
};

static int calibration_mode = false;
static int __init calibration_start(char *str)
{
	int calibration_device =0;
	int mode=0,freq=0,device=0;

	if(str) {
		pr_info("modem calibartion:%s\n", str);
		sscanf(str, "%d,%d,%d", &mode,&freq,&device);
	}

	if(device & 0x80){
		calibration_device = device & 0xf0;
		calibration_mode = true;
		pr_info("calibration device = 0x%x\n",calibration_device);
	}
	calibration_mode = true;
	return 1;
}
__setup("calibration=", calibration_start);

int in_calibration(void)
{
	return (calibration_mode == true);
}

EXPORT_SYMBOL(in_calibration);

static void __init sprd_add_otg_device(void)
{
	/*
	 * if in calibrtaion mode, we do nothing, modem will handle everything
	 */
	platform_device_register(&sprd_otg_device);
}

static struct serial_data plat_data0 = {
	.wakeup_type = BT_RTS_HIGH_WHEN_SLEEP,
	.clk = 48000000,
};
static struct serial_data plat_data1 = {
	.wakeup_type = BT_RTS_HIGH_WHEN_SLEEP,
	.clk = 26000000,
};
static struct serial_data plat_data2 = {
	.wakeup_type = BT_RTS_HIGH_WHEN_SLEEP,
	.clk = 26000000,
};
static struct serial_data plat_data3 = {
	.wakeup_type = 0,
	.clk = 26000000,
};

#ifdef CONFIG_TOUCHSCREEN_FT5306
static struct ft5x0x_ts_platform_data ft5x0x_ts_info = {
	.irq_gpio_number	= GPIO_TOUCH_IRQ,
	.reset_gpio_number	= GPIO_TOUCH_RESET,
	.vdd_name 			= "vdd28",
};
#endif

static struct ltr558_pls_platform_data ltr558_pls_info = {
	.irq_gpio_number	= GPIO_PLSENSOR_IRQ,
};

static struct lis3dh_acc_platform_data lis3dh_plat_data = {
	.poll_interval = 10,
	.min_interval = 10,
	.g_range = LIS3DH_ACC_G_2G,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 1
};

struct akm8975_platform_data akm8975_platform_d = {
	.mag_low_x = -20480,
	.mag_high_x = 20479,
	.mag_low_y = -20480,
	.mag_high_y = 20479,
	.mag_low_z = -20480,
	.mag_high_z = 20479,
};

static struct mpu_platform_data mpu9150_platform_data = {
	.int_config = 0x00,
	.level_shifter = 0,
	.orientation = { -1, 0, 0,
					  0, -1, 0,
					  0, 0, +1 },
	.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id = COMPASS_ID_AK8963,
	.secondary_i2c_addr = 0x0C,
	.secondary_orientation = { 0, -1, 0,
					1, 0, 0,
					0, 0, 1 },
	.key = {0xec, 0x06, 0x17, 0xdf, 0x77, 0xfc, 0xe6, 0xac,
			0x7b, 0x6f, 0x12, 0x8a, 0x1d, 0x63, 0x67, 0x37},
};

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_3K

static ssize_t syn_vkeys_show_t4_5(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 200,
		__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":100:1025:100:90"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":440:1025:100:90"
		"\n");}

static ssize_t syn_vkeys_show_t4_3(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 200,
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":80:850:90:90"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":240:850:90:90"
	":" __stringify(EV_KEY) ":" __stringify(KEY_APP_SWITCH) ":400:850:90:90"
	"\n");
}

static struct kobj_attribute syn_vkeys_attr_t4_5 = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &syn_vkeys_show_t4_5,
};

static struct kobj_attribute syn_vkeys_attr_t4_3 = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &syn_vkeys_show_t4_3,
};

static struct attribute *syn_properties_attrs_t4_5[] = {
	&syn_vkeys_attr_t4_5.attr,
	NULL
};

static struct attribute *syn_properties_attrs_t4_3[] = {
	&syn_vkeys_attr_t4_3.attr,
	NULL
};

static struct attribute_group syn_properties_attr_group_t4_5 = {
	.attrs = syn_properties_attrs_t4_5,
};

static struct attribute_group syn_properties_attr_group_t4_3 = {
	.attrs = syn_properties_attrs_t4_3,
};

static void syn_init_vkeys(void)
{
	int rc = 0;
	static struct kobject *syn_properties_kobj;

	syn_properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (syn_properties_kobj)
	{
		gpio_request(IDENTIFY_TOUCH_PANEL_GPIO, "touch_panel_irq_key");
		gpio_direction_input(IDENTIFY_TOUCH_PANEL_GPIO);
		if(gpio_get_value(IDENTIFY_TOUCH_PANEL_GPIO)){// 4.5" touch screen
			printk("in %s, 4.5 inch touch screen.\n", __func__);
			rc = sysfs_create_group(syn_properties_kobj, &syn_properties_attr_group_t4_5);
		}
		else{// 4.3" touch screen.
			printk("in %s, 4.3 inch touch screen.\n", __func__);
			rc = sysfs_create_group(syn_properties_kobj, &syn_properties_attr_group_t4_3);
		}
		gpio_free(IDENTIFY_TOUCH_PANEL_GPIO);
	}
	if (!syn_properties_kobj || rc)
		pr_err("%s: failed to create board_properties\n", __func__);

	return;
}


static struct synaptics_i2c_rmi_platform_data syn_ts_3k_data_t4_5[] = {// 4.5" 4.5QHD LCD
	{
		.packrat_number  = 1293984, //1423922,
		.abs_x_min       = 0,
		.abs_x_max       = 800,
		.abs_y_min       = 0,
		.abs_y_max       = 1320,
		.display_width   = 540,
		.display_height  = 960,
		.gpio_irq        = GPIO_TP_ATTz,
		.gpio_reset      = GPIO_TP_RSTz,
		.default_config  = 1,
		.tw_pin_mask     = 0x0080,
		.report_type     = SYN_AND_REPORT_TYPE_A,
		.psensor_detection   = 1,
		.support_htc_event   = 1,
		.reduce_report_level = {65, 65, 50, 0, 0},
		.config = {
			0x43, 0x50, 0x30, 0x34, 0x00, 0x7F, 0x03, 0x1E,
			0x05, 0x09, 0x00, 0x01, 0x01, 0x00, 0x10, 0x2A,
			0x03, 0xA0, 0x05, 0x02, 0x14, 0x1E, 0x05, 0x3C,
			0x98, 0x13, 0x2F, 0x02, 0x01, 0x3C, 0x1D, 0x01,
			0x16, 0x02, 0x0A, 0x57, 0x00, 0x54, 0x7A, 0xAA,
			0x40, 0xB2, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00,
			0x09, 0x04, 0xB8, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x19, 0x01, 0x00, 0x0A, 0x0A, 0x14, 0x0A,
			0x00, 0x14, 0x0A, 0x40, 0x78, 0x07, 0xF6, 0xC8,
			0xC0, 0x43, 0x2A, 0x05, 0x00, 0x00, 0x00, 0x00,
			0x2A, 0xA0, 0x53, 0x3C, 0x32, 0x00, 0x00, 0x00,
			0x2A, 0xA0, 0x53, 0x1E, 0x05, 0x00, 0x02, 0x4A,
			0x01, 0x80, 0x03, 0x0E, 0x1F, 0x12, 0x3A, 0x00,
			0x13, 0x04, 0x1B, 0x00, 0x10, 0x0A, 0x60, 0x60,
			0x60, 0x60, 0x68, 0x60, 0x68, 0x60, 0x30, 0x2F,
			0x2E, 0x2D, 0x2C, 0x2B, 0x2A, 0x29, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x88,
			0x13, 0x00, 0x64, 0x00, 0xC8, 0x00, 0x80, 0x0A,
			0x66, 0xB8, 0x0B, 0x00, 0xC0, 0x80, 0x02, 0x02,
			0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x20, 0x20,
			0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x5A, 0x5D,
			0x5F, 0x61, 0x63, 0x66, 0x69, 0x6C, 0x00, 0xB4,
			0x00, 0x64, 0xC8, 0x00, 0x00, 0x00, 0x02, 0x04,
			0x06, 0x08, 0x0B, 0x0E, 0x0F, 0x00, 0x31, 0x04,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x51, 0x51, 0x51,
			0x51, 0x51, 0x51, 0x51, 0x51, 0xCD, 0x0D, 0x04,
			0x01, 0x14, 0x12, 0x13, 0x16, 0x15, 0x17, 0x18,
			0x19, 0x1B, 0x1A, 0xFF, 0xFF, 0xFF, 0xFF, 0x02,
			0x07, 0x0B, 0x05, 0x00, 0x0F, 0x0A, 0x11, 0x10,
			0x0C, 0x03, 0x06, 0x0E, 0x12, 0x04, 0x09, 0x08,
			0x0D, 0x13, 0x01, 0x00, 0x10, 0x00, 0x10, 0x00,
			0x10, 0x00, 0x10, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x0F, 0x01,
			0x20, 0x01
		},
	},
};

static struct synaptics_i2c_rmi_platform_data syn_ts_3k_data_t4_3[] = { //4//  4.3" WVGA LCD
	{
		.packrat_number  = 1293984,
		.abs_x_min       = 0,
		.abs_x_max       = 1100,
		.abs_y_min       = 0,
		.abs_y_max       = 1740,
		.display_width   = 480,
		.display_height  = 800,
		.gpio_irq        = GPIO_TP_ATTz,
		.gpio_reset      = GPIO_TP_RSTz,
		.default_config  = 1,
		.tw_pin_mask     = 0x0080,
		.report_type     = SYN_AND_REPORT_TYPE_A,
		.psensor_detection   = 1,
		.support_htc_event   = 1,
		.reduce_report_level = {65, 65, 50, 0, 0},
		.config = {
			0x33, 0x30, 0x00, 0x07, 0x00, 0x7F, 0x03, 0x1E,
			0x05, 0x09, 0x00, 0x01, 0x01, 0x00, 0x10, 0x4C,
			0x04, 0x6C, 0x07, 0x02, 0x14, 0x1E, 0x05, 0x50,
			0xDC, 0x1C, 0x6D, 0x03, 0x01, 0x3C, 0x1C, 0x01,
			0x1D, 0x01, 0xE1, 0x56, 0x00, 0x54, 0xB5, 0xAD,
			0x12, 0xB4, 0x01, 0xE0, 0x00, 0x00, 0x00, 0x00,
			0x09, 0x04, 0xAE, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x19, 0x01, 0x00, 0x0A, 0x0B, 0x13, 0x0A,
			0x00, 0x14, 0x0A, 0x40, 0x64, 0x07, 0xF4, 0x96,
			0xD2, 0x43, 0x2A, 0x05, 0x00, 0x00, 0x00, 0x00,
			0x4C, 0x6C, 0x74, 0x3C, 0x32, 0x00, 0x00, 0x00,
			0x4C, 0x6C, 0x74, 0x1E, 0x05, 0x00, 0x02, 0x40,
			0x01, 0x80, 0x03, 0x0E, 0x1F, 0x12, 0x36, 0x00,
			0x13, 0x04, 0x1B, 0x00, 0x64, 0xC8, 0x60, 0x60,
			0x60, 0x68, 0x60, 0x68, 0x60, 0x68, 0x32, 0x31,
			0x30, 0x2F, 0x2E, 0x2D, 0x2C, 0x2B, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x88,
			0x13, 0x00, 0x64, 0x00, 0xC8, 0x00, 0x80, 0x0A,
			0x80, 0xB8, 0x0B, 0x00, 0xC0, 0x19, 0x02, 0x02,
			0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x20, 0x20,
			0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x56, 0x59,
			0x5B, 0x5D, 0x5F, 0x61, 0x63, 0x66, 0x00, 0x8C,
			0x00, 0x64, 0xC8, 0x00, 0x00, 0x00, 0x02, 0x04,
			0x06, 0x08, 0x0A, 0x0C, 0x0D, 0x00, 0x31, 0x04,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x51, 0x51, 0x51,
			0x51, 0x51, 0x51, 0x51, 0x51, 0xCD, 0x0D, 0x04,
			0x01, 0x17, 0x15, 0x18, 0x16, 0x19, 0x13, 0x1B,
			0x12, 0x1A, 0x14, 0x11, 0xFF, 0xFF, 0xFF, 0x09,
			0x0F, 0x08, 0x0A, 0x0D, 0x11, 0x13, 0x10, 0x01,
			0x0C, 0x04, 0x05, 0x12, 0x0B, 0x0E, 0x07, 0x06,
			0x02, 0x03, 0xFF, 0x00, 0x10, 0x00, 0x10, 0x00,
			0x10, 0x00, 0x10, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x0A, 0x00,
			0x4F, 0x53,
		},
	},
	{
		.packrat_number  = 1248555,
		.abs_x_min       = 0,
		.abs_x_max       = 1100,
		.abs_y_min       = 0,
		.abs_y_max       = 1760,
		.display_width   = 480,
		.display_height  = 800,
		.gpio_irq        = GPIO_TP_ATTz,
		.gpio_reset      = GPIO_TP_RSTz,
		.default_config  = 1,
		.large_obj_check = 1,
		.tw_pin_mask       = 0x0080,
		.report_type       = SYN_AND_REPORT_TYPE_A,
		.support_htc_event = 1,
		.multitouch_calibration = 1,
		.support_htc_event      = 1,
		.reduce_report_level    = {65, 65, 50, 0, 0},
		.psensor_detection      = 1,
		.config = {
			0x33, 0x32, 0x00, 0x05, 0x00, 0x7F, 0x03, 0x1E,
			0x05, 0x09, 0x00, 0x01, 0x01, 0x00, 0x10, 0x4C,
			0x04, 0x6C, 0x07, 0x02, 0x14, 0x1E, 0x05, 0x3F,
			0x10, 0x1E, 0x64, 0x01, 0x01, 0x3C, 0x25, 0x02,
			0x21, 0x03, 0xE1, 0x56, 0x00, 0x54, 0x80, 0xBB,
			0x80, 0xBB, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00,
			0x09, 0x04, 0xAE, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x19, 0x01, 0x00, 0x0A, 0x0B, 0x13, 0x0A,
			0x00, 0x14, 0x0A, 0x40, 0x64, 0x07, 0xF4, 0x96,
			0xD2, 0x43, 0x2A, 0x05, 0x00, 0x00, 0x00, 0x00,
			0x4C, 0x6C, 0x74, 0x3C, 0x32, 0x00, 0x00, 0x00,
			0x4C, 0x6C, 0x74, 0x1E, 0x05, 0x00, 0x02, 0x7C,
			0x01, 0x80, 0x03, 0x0E, 0x1F, 0x12, 0x36, 0x00,
			0x13, 0x04, 0x1B, 0x00, 0x10, 0x00, 0xC0, 0xA0,
			0xA0, 0xA8, 0xA0, 0xA8, 0xA0, 0xA8, 0x47, 0x45,
			0x44, 0x42, 0x41, 0x40, 0x3E, 0x3D, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x88,
			0x13, 0x00, 0x64, 0x00, 0xC8, 0x00, 0x80, 0x0A,
			0x80, 0xB8, 0x0B, 0x00, 0xC0, 0x80, 0x02, 0x02,
			0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x20, 0x20,
			0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x56, 0x59,
			0x5B, 0x5D, 0x5F, 0x61, 0x63, 0x66, 0x00, 0x8C,
			0x00, 0x10, 0x28, 0x00, 0x00, 0x00, 0x02, 0x04,
			0x06, 0x08, 0x0A, 0x0C, 0x0D, 0x04, 0x31, 0x04,
			0x1A, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x51, 0x51, 0x51,
			0x51, 0x51, 0x51, 0x51, 0x51, 0xCD, 0x0D, 0x04,
			0x01, 0x17, 0x15, 0x18, 0x16, 0x19, 0x13, 0x1B,
			0x12, 0x1A, 0x14, 0x11, 0xFF, 0xFF, 0xFF, 0x09,
			0x0F, 0x08, 0x0A, 0x0D, 0x11, 0x13, 0x10, 0x01,
			0x0C, 0x04, 0x05, 0x12, 0x0B, 0x0E, 0x07, 0x06,
			0x02, 0x03, 0xFF, 0x00, 0x10, 0x00, 0x10, 0x00,
			0x10, 0x00, 0x10, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x0A, 0x00,
			0x00,
		},
	},
	{
		.packrat_number  = 974353,
		.abs_x_min       = 0,
		.abs_x_max       = 1100,
		.abs_y_min       = 0,
		.abs_y_max       = 1760,
		.display_width   = 480,
		.display_height  = 800,
		.flags           = SYNAPTICS_FLIP_X,
		.gpio_irq        = GPIO_TP_ATTz,
		.gpio_reset      = GPIO_TP_RSTz,
		.default_config  = 2,
		.large_obj_check = 1,
		.tw_pin_mask       = 0x0080,
		.report_type       = SYN_AND_REPORT_TYPE_A,
		.support_htc_event = 1,
		.segmentation_bef_unlock = 0x50,
		.multitouch_calibration  = 1,
		.psensor_detection       = 1,
		.support_htc_event 	 = 1,
		.config = {
			0x32, 0x30, 0x30, 0x31, 0x04, 0x0F, 0x03, 0x1E,
			0x05, 0x20, 0xB1, 0x00, 0x0B, 0x19, 0x19, 0x00,
			0x00, 0x4C, 0x04, 0x6C, 0x07, 0x1E, 0x05, 0x28,
			0xF5, 0x28, 0x1E, 0x05, 0x01, 0x30, 0x00, 0x30,
			0x00, 0x00, 0x48, 0x00, 0x48, 0x44, 0xA0, 0xD3,
			0xA1, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x0A,
			0x04, 0xC0, 0x00, 0x02, 0xA1, 0x01, 0x80, 0x02,
			0x0D, 0x1E, 0x00, 0x8C, 0x00, 0x19, 0x04, 0x1E,
			0x00, 0x10, 0x0A, 0x01, 0x11, 0x14, 0x1A, 0x12,
			0x1B, 0x13, 0x19, 0x16, 0x18, 0x15, 0x17, 0xFF,
			0xFF, 0xFF, 0x09, 0x0F, 0x08, 0x0A, 0x0D, 0x11,
			0x13, 0x10, 0x01, 0x0C, 0x04, 0x05, 0x12, 0x0B,
			0x0E, 0x07, 0x06, 0x02, 0x03, 0xFF, 0x40, 0x40,
			0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x24, 0x23,
			0x21, 0x20, 0x1F, 0x1D, 0x1C, 0x1A, 0x00, 0x07,
			0x0F, 0x18, 0x21, 0x2B, 0x37, 0x43, 0x00, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0x00, 0xFF, 0xFF, 0x00, 0xC0, 0x80, 0x00, 0x10,
			0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x02, 0x02, 0x02, 0x07, 0x02, 0x03,
			0x09, 0x03, 0x10, 0x10, 0x10, 0x40, 0x10, 0x10,
			0x40, 0x10, 0x59, 0x5D, 0x61, 0x74, 0x6A, 0x4A,
			0x68, 0x52, 0x30, 0x30, 0x00, 0x1E, 0x19, 0x05,
			0x00, 0x00, 0x3D, 0x08,
		}
	},
	{
		.packrat_number = 7788,
		.abs_x_min      = 0,
		.abs_x_max      = 1100,
		.abs_y_min      = 0,
		.abs_y_max      = 1760,
		.display_width  = 480,
		.display_height = 800,
		.flags          = SYNAPTICS_FLIP_X,
		.gpio_irq       = GPIO_TP_ATTz,
		.gpio_reset     = GPIO_TP_RSTz,
		.default_config = 2,
		.large_obj_check = 1,
		.tw_pin_mask       = 0x0080,
		.report_type       = SYN_AND_REPORT_TYPE_A,
		.support_htc_event = 1,
		.segmentation_bef_unlock = 0x50,
	},
};

#endif

static struct i2c_board_info i2c2_boardinfo[] = {
	{ I2C_BOARD_INFO(LIS3DH_ACC_I2C_NAME, LIS3DH_ACC_I2C_ADDR),
	  .platform_data = &lis3dh_plat_data,
	},
	{ I2C_BOARD_INFO("mpu9150", 0x68),
	  .irq = MPUIRQ_GPIO,
	  .platform_data = &mpu9150_platform_data,
	},
	{ I2C_BOARD_INFO(LTR558_I2C_NAME,  LTR558_I2C_ADDR),
	  .platform_data = &ltr558_pls_info,
	},
/*	{ I2C_BOARD_INFO(AKM8975_I2C_NAME,    AKM8975_I2C_ADDR),
	  .platform_data = &akm8975_platform_d,
	},*/
};

static struct i2c_board_info i2c1_boardinfo[] = {
	{I2C_BOARD_INFO("sensor_main",0x3C),},
	{I2C_BOARD_INFO("sensor_sub",0x21),},
};

static struct i2c_board_info i2c0_boardinfo_t4_3[] = {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_3K
	{
		I2C_BOARD_INFO(SYNAPTICS_3200_NAME, 0x40 >> 1),
		.platform_data = &syn_ts_3k_data_t4_3,
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_FT5306
	{
		I2C_BOARD_INFO(FT5206_TS_DEVICE, FT5206_TS_ADDR),
		.platform_data = &ft5x0x_ts_info,
	},
#endif
};

static struct i2c_board_info i2c0_boardinfo_t4_5[] = {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_3K
	{
		I2C_BOARD_INFO(SYNAPTICS_3200_NAME, 0x40 >> 1),
		.platform_data = &syn_ts_3k_data_t4_5,
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_FT5306
	{
		I2C_BOARD_INFO(FT5206_TS_DEVICE, FT5206_TS_ADDR),
		.platform_data = &ft5x0x_ts_info,
	},
#endif
};

#define TIGER_EVM_GPIO_NFC_IRQ                38
#define TIGER_EVM_GPIO_NFC_VEN                64
#define TIGER_EVM_GPIO_NFC_DL_MODE       135

static struct pn544_i2c_platform_data nfc_platform_data = {
	.irq_gpio = TIGER_EVM_GPIO_NFC_IRQ,
	.ven_gpio = TIGER_EVM_GPIO_NFC_VEN,
	.firm_gpio = TIGER_EVM_GPIO_NFC_DL_MODE,
	.ven_isinvert = 1,
};

static struct i2c_board_info i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO(PN544_I2C_NAME,0x50 >> 1),
		.platform_data = &nfc_platform_data,
	},
#ifdef CONFIG_FLASHLIGHT_TPS61310
	{
		I2C_BOARD_INFO("TPS61310_FLASHLIGHT", 0x66 >> 1),
		.platform_data = &tiger_flashlight_data,
	},
#endif
};
static int sc8810_add_i2c_devices(void)
{
	int i = 0;

	i2c_register_board_info(2, i2c2_boardinfo, ARRAY_SIZE(i2c2_boardinfo));
	i2c_register_board_info(1, i2c1_boardinfo, ARRAY_SIZE(i2c1_boardinfo));

	gpio_request(IDENTIFY_TOUCH_PANEL_GPIO, "touch_panel_irq_key");
	gpio_direction_input(IDENTIFY_TOUCH_PANEL_GPIO);
	i = gpio_get_value(IDENTIFY_TOUCH_PANEL_GPIO);
		printk("20120328-1: in %s, GPIO 65 value is %d.\n", __func__, i);
	if(i){// 4.5" touch screen
		printk("in %s, 4.5  inch touch screen.\n", __func__);
		i2c_register_board_info(0, i2c0_boardinfo_t4_5, ARRAY_SIZE(i2c0_boardinfo_t4_5));
	}
	else{// 4.3" touch screen.
		printk("in %s, 4.3 inch touch screen.\n", __func__);
		i2c_register_board_info(0, i2c0_boardinfo_t4_3, ARRAY_SIZE(i2c0_boardinfo_t4_3));
	}
	gpio_free(IDENTIFY_TOUCH_PANEL_GPIO);

	i2c_register_board_info(3, i2c3_boardinfo, ARRAY_SIZE(i2c3_boardinfo));

	return 0;
}

struct platform_device audio_pa_amplifier_device = {
	.name = "speaker-pa",
	.id = -1,
};

static int audio_pa_amplifier_l(u32 cmd, void *data)
{
	int ret = 0;
	if (cmd < 0) {
		/* get speaker amplifier status : enabled or disabled */
		ret = 0;
	} else {
		/* set speaker amplifier */
	}
	return ret;
}

/* Control ldo for maxscend cmmb chip according to HW design */
static struct regulator *cmmb_regulator_1v8 = NULL;

#define SPI_PIN_FUNC_MASK  (0x3<<4)
#define SPI_PIN_FUNC_DEF   (0x0<<4)
#define SPI_PIN_FUNC_GPIO  (0x3<<4)

struct spi_pin_desc {
	const char   *name;
	unsigned int pin_func;
	unsigned int reg;
	unsigned int gpio;
};

static struct spi_pin_desc spi_pin_group[] = {
	{"SPI_DI",  SPI_PIN_FUNC_DEF,  REG_PIN_SPI0_DI   + CTL_PIN_BASE,  158},
	{"SPI_CLK", SPI_PIN_FUNC_DEF,  REG_PIN_SPI0_CLK  + CTL_PIN_BASE,  159},
	{"SPI_DO",  SPI_PIN_FUNC_DEF,  REG_PIN_SPI0_DO   + CTL_PIN_BASE,  157},
	{"SPI_CS0", SPI_PIN_FUNC_GPIO, REG_PIN_SPI0_CSN  + CTL_PIN_BASE,  156}
};


static void sprd_restore_spi_pin_cfg(void)
{
	unsigned int reg;
	unsigned int  gpio;
	unsigned int  pin_func;
	unsigned int value;
	unsigned long flags;
	int i = 0;
	int regs_count = sizeof(spi_pin_group)/sizeof(struct spi_pin_desc);

	for (; i < regs_count; i++) {
	    pin_func = spi_pin_group[i].pin_func;
	    gpio = spi_pin_group[i].gpio;
	    if (pin_func == SPI_PIN_FUNC_DEF) {
		 reg = spi_pin_group[i].reg;
		 /* free the gpios that have request */
		 gpio_free(gpio);
		 local_irq_save(flags);
		 /* config pin default spi function */
		 value = ((__raw_readl(reg) & ~SPI_PIN_FUNC_MASK) | SPI_PIN_FUNC_DEF);
		 __raw_writel(value, reg);
		 local_irq_restore(flags);
	    }
	    else {
		 /* CS should config output */
		 gpio_direction_output(gpio, 1);
	    }
	}

}


static void sprd_set_spi_pin_input(void)
{
	unsigned int reg;
	unsigned int value;
	unsigned int  gpio;
	unsigned int  pin_func;
	const char    *name;
	unsigned long flags;
	int i = 0;

	int regs_count = sizeof(spi_pin_group)/sizeof(struct spi_pin_desc);

	for (; i < regs_count; i++) {
	    pin_func = spi_pin_group[i].pin_func;
	    gpio = spi_pin_group[i].gpio;
	    name = spi_pin_group[i].name;

	    /* config pin GPIO function */
	    if (pin_func == SPI_PIN_FUNC_DEF) {
		 reg = spi_pin_group[i].reg;

		 local_irq_save(flags);
		 value = ((__raw_readl(reg) & ~SPI_PIN_FUNC_MASK) | SPI_PIN_FUNC_GPIO);
		 __raw_writel(value, reg);
		 local_irq_restore(flags);
		 if (gpio_request(gpio, name)) {
		     printk("smsspi: request gpio %d failed, pin %s\n", gpio, name);
		 }

	    }

	    gpio_direction_input(gpio);
	}

}

static void mxd_cmmb_poweron(void)
{
        regulator_set_voltage(cmmb_regulator_1v8, 1700000, 1800000);
        regulator_disable(cmmb_regulator_1v8);
        msleep(3);
        regulator_enable(cmmb_regulator_1v8);
        msleep(5);

        /* enable 26M external clock */
        gpio_direction_output(GPIO_CMMB_26M_CLK_EN, 1);
}

static void mxd_cmmb_poweroff(void)
{
        regulator_disable(cmmb_regulator_1v8);
        gpio_direction_output(GPIO_CMMB_26M_CLK_EN, 0);
}

static int mxd_cmmb_init(void)
{
         int ret=0;
         ret = gpio_request(GPIO_CMMB_26M_CLK_EN,   "MXD_CMMB_CLKEN");
         if (ret)
         {
                   pr_debug("mxd spi req gpio clk en err!\n");
                   goto err_gpio_init;
         }
         gpio_direction_output(GPIO_CMMB_26M_CLK_EN, 0);
         cmmb_regulator_1v8 = regulator_get(NULL, "vddcmmb1p8");
         return 0;

err_gpio_init:
	 gpio_free(GPIO_CMMB_26M_CLK_EN);
         return ret;
}

static struct mxd_cmmb_026x_platform_data mxd_plat_data = {
	.poweron  = mxd_cmmb_poweron,
	.poweroff = mxd_cmmb_poweroff,
	.init     = mxd_cmmb_init,
	.set_spi_pin_input   = sprd_set_spi_pin_input,
	.restore_spi_pin_cfg = sprd_restore_spi_pin_cfg,
};

static int spi_cs_gpio_map[][2] = {
    {SPI0_CMMB_CS_GPIO,  0},
    {SPI0_CMMB_CS_GPIO,  0},
    {SPI0_CMMB_CS_GPIO,  0},
} ;

static struct spi_board_info spi_boardinfo[] = {
	{
	.modalias = "cmmb-dev",
	.bus_num = 0,
	.chip_select = 0,
	.max_speed_hz = 8 * 1000 * 1000,
	.mode = SPI_CPOL | SPI_CPHA,
        .platform_data = &mxd_plat_data,

	},
	{
	.modalias = "spidev",
	.bus_num = 1,
	.chip_select = 0,
	.max_speed_hz = 1000 * 1000,
	.mode = SPI_CPOL | SPI_CPHA,
	},
	{
	.modalias = "spidev",
	.bus_num = 2,
	.chip_select = 0,
	.max_speed_hz = 1000 * 1000,
	.mode = SPI_CPOL | SPI_CPHA,
	}
};

#ifdef CONFIG_USB_G_ANDROID
static struct android_usb_platform_data android_usb_data = {
	.vendor_id	= 0xbb4,
	.product_id	= 0xde3,
	.usb_id_pin_gpio	= 163,

	.product_name   = "Android Phone",
	.manufacturer_name = "HTC",
	.serial_number  = "0123456789ABCDEF0123456789ABCDE",

	.rndisVendorDescr = "HTC",
	.rndisVendorID = 0xbb4,
	.rndisEthaddr = {0x01, 0x11, 0x22, 0x33, 0x44, 0x55},

	.ecmVendorDescr = "HTC",
	.ecmVendorID = 0xbb4,
	.ecmEthaddr = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55},
	.fserial_init_string = "tty:modem,tty:serial,tty:serial,tty:serial",
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data	= &android_usb_data,
	},
};
static void fetch_usb_serial_no()
{
	//android_usb_data.serial_number = board_serialno();
}
static void cp4_usb_init()
{
	/* add cdrom support in normal mode */
	//if (board_mfg_mode() == 0) {
		android_usb_data.nluns = 1;
		android_usb_data.cdrom_lun = 0x1;
	//}
}
#endif
static void sprd_spi_init(void)
{
	int busnum, cs, gpio;
	int i;

	struct spi_board_info *info = spi_boardinfo;

	for (i = 0; i < ARRAY_SIZE(spi_boardinfo); i++) {
		busnum = info[i].bus_num;
		cs = info[i].chip_select;
		gpio   = spi_cs_gpio_map[busnum][cs];

		info[i].controller_data = (void *)gpio;
	}

        spi_register_board_info(info, ARRAY_SIZE(spi_boardinfo));
}

static int sc8810_add_misc_devices(void)
{
	if (0) {
		platform_set_drvdata(&audio_pa_amplifier_device, audio_pa_amplifier_l);
		if (platform_device_register(&audio_pa_amplifier_device))
			pr_err("faile to install audio_pa_amplifier_device\n");
	}
	return 0;
}

int __init sc8825_regulator_init(void)
{
	static struct platform_device sc8825_regulator_device = {
		.name 	= "sprd-regulator",
		.id	= -1,
	};
	return platform_device_register(&sc8825_regulator_device);
}

int __init __clock_init_early(void)
{
	pr_info("ahb ctl0 %08x, ctl2 %08x glb gen0 %08x gen1 %08x clk_en %08x\n",
		sci_glb_raw_read(REG_AHB_AHB_CTL0),
		sci_glb_raw_read(REG_AHB_AHB_CTL2),
		sci_glb_raw_read(REG_GLB_GEN0),
		sci_glb_raw_read(REG_GLB_GEN1),
		sci_glb_raw_read(REG_GLB_CLK_EN));
	/* FIXME: Force disable all unused clocks */
	sci_glb_clr(REG_AHB_AHB_CTL0,
		BIT_AXIBUSMON2_EB	|
		BIT_AXIBUSMON1_EB	|
		BIT_AXIBUSMON0_EB	|
//		BIT_EMC_EB       	|
//		BIT_AHB_ARCH_EB  	|
//		BIT_SPINLOCK_EB  	|
		BIT_SDIO2_EB     	|
		BIT_EMMC_EB      	|
//		BIT_DISPC_EB     	|
		BIT_G3D_EB       	|
		BIT_SDIO1_EB     	|
		BIT_DRM_EB       	|
		BIT_BUSMON4_EB   	|
		BIT_BUSMON3_EB   	|
		BIT_BUSMON2_EB   	|
		BIT_ROT_EB       	|
		BIT_VSP_EB       	|
		BIT_ISP_EB       	|
		BIT_BUSMON1_EB   	|
		BIT_DCAM_MIPI_EB 	|
		BIT_CCIR_EB      	|
		BIT_NFC_EB       	|
		BIT_BUSMON0_EB   	|
//		BIT_DMA_EB       	|
//		BIT_USBD_EB      	|
		BIT_SDIO0_EB     	|
//		BIT_LCDC_EB      	|
		BIT_CCIR_IN_EB   	|
		BIT_DCAM_EB      	|
		0);
	sci_glb_clr(REG_AHB_AHB_CTL2,
//		BIT_DISPMTX_CLK_EN	|
		BIT_MMMTX_CLK_EN    |
//		BIT_DISPC_CORE_CLK_EN|
//		BIT_LCDC_CORE_CLK_EN|
		BIT_ISP_CORE_CLK_EN |
		BIT_VSP_CORE_CLK_EN |
		BIT_DCAM_CORE_CLK_EN|
		0);
	sci_glb_clr(REG_AHB_AHB_CTL3,
//		BIT_CLK_ULPI_EN		|
//		BIT_CLK_USB_REF_EN	|
		0);
	sci_glb_clr(REG_GLB_GEN0,
		BIT_IC3_EB          |
		BIT_IC2_EB          |
		BIT_IC1_EB          |
//		BIT_RTC_TMR_EB      |
//		BIT_RTC_SYST0_EB    |
		BIT_RTC_KPD_EB      |
		BIT_IIS1_EB         |
//		BIT_RTC_EIC_EB      |
		BIT_UART2_EB        |
//		BIT_UART1_EB        |
		BIT_UART0_EB        |
//		BIT_SYST0_EB        |
		BIT_SPI1_EB         |
		BIT_SPI0_EB         |
//		BIT_SIM1_EB         |
//		BIT_EPT_EB          |
		BIT_CCIR_MCLK_EN    |
//		BIT_PINREG_EB       |
		BIT_IIS0_EB         |
//		BIT_MCU_DSP_RST		|
//		BIT_EIC_EB     		|
		BIT_KPD_EB     		|
		BIT_EFUSE_EB   		|
//		BIT_ADI_EB     		|
//		BIT_GPIO_EB    		|
		BIT_I2C0_EB    		|
//		BIT_SIM0_EB    		|
//		BIT_TMR_EB     		|
		BIT_SPI2_EB    		|
		BIT_UART3_EB   		|
		0);
	sci_glb_clr(REG_AHB_CA5_CFG,
//		BIT_CA5_CLK_DBG_EN	|
		0);
	sci_glb_clr(REG_GLB_GEN1,
		BIT_AUDIF_AUTO_EN	|
		BIT_VBC_EN			|
		BIT_AUD_TOP_EB		|
		BIT_AUD_IF_EB		|
		BIT_CLK_AUX1_EN		|
//		BIT_CLK_AUX0_EN		|
		0);
	sci_glb_clr(REG_GLB_CLK_EN,
		BIT_PWM3_EB			|
//		BIT_PWM2_EB			|
		BIT_PWM1_EB			|
//		BIT_PWM0_EB			|
		0);

	sci_glb_clr(REG_GLB_PCTRL,
	//		BIT_MCU_MPLL_EN 	|
	//		BIT_MCU_TDPLL_EN	|
	//		BIT_MCU_DPLL_EN 	|
			BIT_MCU_GPLL_EN);	/* clk_gpu */

	sci_glb_set(REG_GLB_TD_PLL_CTL,
	//		BIT_TDPLL_DIV2OUT_FORCE_PD	|	/* clk_384m */
	//		BIT_TDPLL_DIV3OUT_FORCE_PD	|	/* clk_256m */
	//		BIT_TDPLL_DIV4OUT_FORCE_PD	|	/* clk_192m */
	//		BIT_TDPLL_DIV5OUT_FORCE_PD	|	/* clk_153p6m */
			0);

	printk("sc clock module early init ok\n");
	return 0;
}

static void __init sc8825_init_machine(void)
{
	struct proc_dir_entry *entry = NULL; /*  HTC_KER_ADD robin_peng Sync from QCT- Create /proc/dying_processes to get latest 10 records of killed processes. */
	sci_adc_init((void __iomem *)ADC_BASE);
	sc8825_regulator_init();
	sprd_add_otg_device();
	platform_device_add_data(&sprd_serial_device0,(const void*)&plat_data0,sizeof(plat_data0));
	platform_device_add_data(&sprd_serial_device1,(const void*)&plat_data1,sizeof(plat_data1));
	platform_device_add_data(&sprd_serial_device2,(const void*)&plat_data2,sizeof(plat_data2));
	platform_device_add_data(&sprd_serial_device3,(const void*)&plat_data3,sizeof(plat_data3));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	sc8810_add_i2c_devices();
	sc8810_add_misc_devices();
	sprd_spi_init();
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_3K) && !defined(CONFIG_TOUCHSCREEN_FT5306)
	syn_init_vkeys();
#endif
#ifdef CONFIG_USB_G_ANDROID
	fetch_usb_serial_no();
	cp4_usb_init();
#endif
	/* HTC_PROC_EMMC_START */
	if (board_emmc_boot()) {
		/* rmt_storage_add_ramfs(); */
		create_proc_read_entry("emmc", 0, NULL, emmc_partition_read_proc, NULL);
	} else {
		printk(KERN_INFO "!board_emmc_boot(), do not read proc/emmc\n");
	}
	/* HTC_PROC_EMMC_END */

	/* HTC_KER_START robin_peng Sync from QCT- Create /proc/dying_processes to get latest 10 records of killed processes. */
	entry = create_proc_read_entry("dying_processes", 0, NULL, dying_processors_read_proc, NULL);
	if (!entry)
		printk(KERN_ERR"Create /proc/dying_processes FAILED!\n");
	/* HTC_KER_END robin_peng Sync from QCT- Create /proc/dying_processes to get latest 10 records of killed processes. */
}

extern void __init  sci_enable_timer_early(void);
static void __init sc8825_init_early(void)
{
	/* earlier init request than irq and timer */
	__clock_init_early();
	sci_enable_timer_early();
	sci_adi_init();
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	persistent_ram_early_init(&sprd_console_ram);
#endif
}

/*
 * Setup the memory banks.
 */
 
static void __init sc8825_fixup(struct machine_desc *desc,
	struct tag *tags, char **cmdline, struct meminfo *mi)
{
}

MACHINE_START(SCPHONE, "sc8825")	
	.reserve	= sci_reserve,
	.map_io		= sci_map_io,
	.fixup		= sc8825_fixup,
	.init_early	= sc8825_init_early,
	.handle_irq	= gic_handle_irq,
	.init_irq	= sci_init_irq,
	.timer		= &__timer,
	.init_machine	= sc8825_init_machine,
MACHINE_END

