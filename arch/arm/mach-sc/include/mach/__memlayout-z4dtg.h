/* linux/arch/arm/mach-sc/include/mach/__memlayout-z4dtg.h
 * Copyright (C) 2013 HTC Corporation.
 * Author: Yili Xie <yili_xie@htc.com>
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


#ifndef __MEMLAYOUT_Z4DTG_H__
#define __MEMLAYOUT_Z4DTG_H__


/*
 * SPL remap the PA, so PA is continous.
 * Don't care the CS connection
 *
 *                        0xC0000000
 *   --------------------
 *   | LINUX   |  583MB |
 *   --------------------
 *   | ION     |  126MB |
 *   --------------------
 *   | FB      |  4MB   |
 *   --------------------
 *   | RC      |  1MB   |
 *   --------------------
 *   | W-M     |  64MB  |
 *   --------------------
 *   | LINUX   |  84MB  |
 *   --------------------
 *   | TD      |  44MB  |
 *   --------------------
 *   | LINUX   |  128MB |
 *   -------------------- 0x80000000
 *
 * */

#include <asm/sizes.h>

#define PROJECT_SPECIAL_LAYOUT

#define LINUX_BANK_NUM            3

#define LINUX_BASE_ADDR1		0x80000000
#define LINUX_SIZE_ADDR1		0x08000000  /* (128 * 1024 * 1024) */
#define LINUX_VIRT_ADDR1		(PAGE_OFFSET)

#define TD_BASE_ADDR			0x88000000
#define TD_SIZE_ADDR			0x02C00000  /* (44 * 1024 * 1024) */
#define TD_VIRT_ADDR			(LINUX_VIRT_ADDR1 + LINUX_SIZE_ADDR1)

#define LINUX_BASE_ADDR2		0x8AC00000
#define LINUX_SIZE_ADDR2		0x05400000  /* (84 * 1024 * 1024) */
#define LINUX_VIRT_ADDR2		(TD_VIRT_ADDR + TD_SIZE_ADDR)

#define GSM_BASE_ADDR			0x90000000
#define GSM_SIZE_ADDR			0x04000000  /* (64 * 1024 * 1024) */
#define GSM_VIRT_ADDR			(LINUX_VIRT_ADDR2 + LINUX_SIZE_ADDR2)

#define RC_BASE_ADDR			0x94000000
#define RC_SIZE_ADDR			0x00100000  /* (1 * 1024 * 1024) */
#define RC_VIRT_ADDR			(GSM_VIRT_ADDR + GSM_SIZE_ADDR)

#define FB_BASE_ADDR			0x94100000
#define FB_SIZE_ADDR			0x00400000  /* (4 * 1024 * 1024) */
#define FB_VIRT_ADDR			(RC_VIRT_ADDR + RC_SIZE_ADDR)

#define ION_BASE_ADDR			0x94500000
#define ION_SIZE_ADDR			0x07E00000  /* (126 * 1024 * 1024) */
#define ION_VIRT_ADDR			(FB_VIRT_ADDR + FB_SIZE_ADDR)

#define LINUX_BASE_ADDR3		0x9C300000
#define LINUX_SIZE_ADDR3		0x23D00000  /* (583 * 1024 * 1024) */
#define LINUX_VIRT_ADDR3		(ION_VIRT_ADDR + ION_SIZE_ADDR)

#define ELF_HEADER_ADDR			0x94900000
#define ELF_HEADER_SIZE			0x00100000  /* (1 * 1024 * 1024) */

#ifdef CONFIG_ION
#define SPRD_ION_OVERLAY_SIZE		(CONFIG_SPRD_ION_OVERLAY_SIZE * SZ_1M)
#define SPRD_ION_SIZE				(ION_SIZE_ADDR - SPRD_ION_OVERLAY_SIZE)
#else
#define SPRD_ION_OVERLAY_SIZE		(0 * SZ_1M)
#define SPRD_ION_SIZE			(0 * SZ_1M)
#endif

#define SPRD_IO_MEM_SIZE		(ION_SIZE_ADDR)
#define SPRD_IO_MEM_BASE		(ION_BASE_ADDR)

#define SPRD_ION_BASE		(ION_BASE_ADDR)
#define SPRD_ION_OVERLAY_BASE		(ION_BASE_ADDR + SPRD_ION_SIZE)

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define SPRD_RAM_CONSOLE_SIZE		(RC_SIZE_ADDR)
#define SPRD_RAM_CONSOLE_START		(RC_BASE_ADDR)
#endif

#ifdef CONFIG_HTC_DBG_UNCACHE_FTRACE
#define FT_BASE_PHY (248 * SZ_1M + 0x80000000)
#define FT_SIZE_PHY (8 * SZ_1M)
#endif

#endif
