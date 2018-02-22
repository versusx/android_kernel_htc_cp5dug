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

#ifndef __ARCH_ARM_MACH_SPRD_DEVICES_H
#define __ARCH_ARM_MACH_SPRD_DEVICES_H

extern struct platform_device sprd_hwspinlock_device0;
extern struct platform_device sprd_hwspinlock_device1;
extern struct platform_device sprd_serial_device0;
extern struct platform_device sprd_serial_device1;
extern struct platform_device sprd_serial_device2;
extern struct platform_device sprd_serial_device3;
extern struct platform_device sprd_serial_device4;
extern struct platform_device sprd_device_rtc;
extern struct platform_device sprd_eic_gpio_device;
extern struct platform_device sprd_nand_device;
extern struct platform_device sprd_lcd_device0;
extern struct platform_device sprd_lcd_device1;
#ifdef CONFIG_ANDROID_RAM_CONSOLE
extern struct platform_device sprd_ram_console;
extern struct persistent_ram sprd_console_ram;
#endif
extern struct platform_device sprd_otg_device;
extern struct platform_device sprd_backlight_device;
extern struct platform_device sprd_i2c_device0;
extern struct platform_device sprd_i2c_device1;
extern struct platform_device sprd_i2c_device2;
extern struct platform_device sprd_i2c_device3;
extern struct platform_device sprd_spi0_device;
extern struct platform_device sprd_spi1_device;
extern struct platform_device sprd_spi2_device;
extern struct platform_device sprd_keypad_device;
extern struct platform_device sprd_audio_platform_pcm_device;
extern struct platform_device sprd_thm_device;
extern struct platform_device sprd_thm_a_device;
extern struct platform_device sprd_audio_cpu_dai_vaudio_device;
extern struct platform_device sprd_audio_cpu_dai_vbc_device;
extern struct platform_device sprd_audio_codec_sprd_codec_device;
extern struct platform_device sprd_audio_cpu_dai_i2s_device;
extern struct platform_device sprd_audio_cpu_dai_i2s_device1;
#ifdef CONFIG_ARCH_SCX35
extern struct platform_device sprd_audio_cpu_dai_i2s_device2;
extern struct platform_device sprd_audio_cpu_dai_i2s_device3;
#endif
extern struct platform_device sprd_audio_codec_null_codec_device;

#if (defined(CONFIG_SND_SOC_TFA9887_CODEC))
extern struct platform_device sprd_audio_codec_tfa9887_codec_device;
#endif

extern struct platform_device sprd_battery_device;
extern struct platform_device sprd_vsp_device;
extern struct platform_device sprd_jpg_device;
#ifdef CONFIG_ION
extern struct platform_device sprd_ion_dev;
#endif
extern int compass_type;
extern struct platform_device sprd_sdio0_device;
extern struct platform_device sprd_sdio1_device;
extern struct platform_device sprd_sdio2_device;
extern struct platform_device sprd_emmc_device;
extern struct platform_device sprd_dcam_device;
extern struct platform_device sprd_scale_device;
extern struct platform_device sprd_gsp_device;
extern struct platform_device sprd_rotation_device;
extern struct platform_device sprd_sensor_device;
extern struct platform_device sprd_isp_device;
extern struct platform_device sprd_dma_copy_device;
#ifdef CONFIG_ARCH_SC8825
extern struct platform_device sprd_ahb_bm0_device;
extern struct platform_device sprd_ahb_bm1_device;
extern struct platform_device sprd_ahb_bm2_device;
extern struct platform_device sprd_ahb_bm3_device;
extern struct platform_device sprd_ahb_bm4_device;
extern struct platform_device sprd_axi_bm0_device;
extern struct platform_device sprd_axi_bm1_device;
extern struct platform_device sprd_axi_bm2_device;
#elif defined(CONFIG_ARCH_SCX35)
extern struct platform_device sprd_ahb_bm_device;
extern struct platform_device sprd_axi_bm_device;
#endif
extern struct platform_device led_pwm_device;
extern struct platform_device sprd_peer_state_device;
extern struct platform_device htc_battery_pdev;
extern struct platform_device max17050_battery_pdev;

extern struct platform_device sprd_a7_pmu_device;

#endif
