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

#ifndef __GPIO_SCX35EA_H__
#define __GPIO_SCX35EA_H__

#ifndef __ASM_ARCH_BOARD_H
#error  "Don't include this file directly, include <mach/board.h>"
#endif

#define GPIO_TOUCH_RESET         142
#define GPIO_TOUCH_IRQ           141

#define GPIO_SENSOR_RESET        41
#define GPIO_MAIN_SENSOR_PWN     42
#define GPIO_SUB_SENSOR_PWN      43

#define EIC_CHARGER_DETECT		(A_EIC_START + 0)
#define EIC_POWER_PBINT2        (A_EIC_START + 1)
#define EIC_POWER_PBINT         (A_EIC_START + 2)
#define EIC_AUD_HEAD_BUTTON     (A_EIC_START + 3)
#define EIC_AUD_HEAD_INST       (A_EIC_START + 4)
#define EIC_AUD_HEAD_INST2      (A_EIC_START + 5)
#define EIC_VCHG_OVI            (A_EIC_START + 6)
#define EIC_VBAT_OVI            (A_EIC_START + 7)

#define EIC_KEY_POWER           (EIC_POWER_PBINT)
#define HEADSET_BUTTON_GPIO		(EIC_AUD_HEAD_BUTTON)
#define HEADSET_DETECT_GPIO		139

#define SPI0_CMMB_CS_GPIO        156
#define SPI1_WIFI_CS_GPIO        44

#define GPIO_BK                  136

#define GPIO_CMMB_RESET         144
#define GPIO_CMMB_INT           143
#define GPIO_CMMB_26M_CLK_EN    197

#define GPIO_BT_RESET       194
#define GPIO_BT_POWER       190
#define GPIO_BT2AP_WAKE     58
#define GPIO_AP2BT_WAKE     51

#define GPIO_WIFI_SHUTDOWN	189
#define GPIO_WIFI_IRQ		52

#define GPIO_PLSENSOR_IRQ	213

// #define HEADSET_PA_CTL_GPIO   1000
#endif
