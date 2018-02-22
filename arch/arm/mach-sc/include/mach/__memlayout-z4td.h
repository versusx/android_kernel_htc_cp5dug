/* linux/arch/arm/mach-sc/include/mach/__memlayout-z4td.h
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


#ifndef __MEMLAYOUT_Z4TD_H__
#define __MEMLAYOUT_Z4TD_H__


/*
 * SPL remap the PA, so PA is continous.
 * Don't care the CS connection
 *
 * No Smart log
 *   -------------------- 0xA0000000
 *   | RC      |  1MB   |
 *   --------------------
 *   | FB      |  5MB   |
 *   --------------------
 *   | ION     |  41MB  |
 *   --------------------
 *   | LINUX   |  315MB |
 *   --------------------
 *   | TD      |  26MB  |
 *   --------------------
 *   | LINUX   |  124MB |
 *   -------------------- 0x80000000
 *
 * Contains smart log
 *   -------------------- 0xA0000000
 *   | RC      |  1MB   |
 *   --------------------
 *   | FB      |  5MB   |
 *   --------------------
 *   | ION     |  41MB  |
 *   --------------------
 *   | LINUX   |  300MB |
 *   --------------------
 *   | TD      |  41MB  |
 *   --------------------
 *   | LINUX   |  124MB |
 *   -------------------- 0x80000000
 * */

#include <asm/sizes.h>

#define PROJECT_SPECIAL_LAYOUT

#define LINUX_BANK_NUM            2

#define LINUX_BASE_ADDR1		0x80000000
#define LINUX_SIZE_ADDR1		0x07C00000  /* (124 * 1024 * 1024) */
#define LINUX_VIRT_ADDR1		(PAGE_OFFSET)

/* without smart log */
#define TD_BASE_ADDR			0x87C00000
#define TD_SIZE_ADDR			0x01A00000  /* (26 * 1024 * 1024) */
#define TD_VIRT_ADDR			(LINUX_VIRT_ADDR1 + LINUX_SIZE_ADDR1)

#define LINUX_BASE_ADDR2		0x89600000
#define LINUX_SIZE_ADDR2		0x13B00000  /* (315 * 1024 * 1024) */
#define LINUX_VIRT_ADDR2		(TD_VIRT_ADDR + TD_SIZE_ADDR)

/* with smart log */
#define TDS_BASE_ADDR			0x87C00000
#define TDS_SIZE_ADDR			0x02900000  /* (41 * 1024 * 1024) */
#define TDS_VIRT_ADDR			(LINUX_VIRT_ADDR1 + LINUX_SIZE_ADDR1)

#define LINUXS_BASE_ADDR2		0x8A500000
#define LINUXS_SIZE_ADDR2		0x12C00000  /* (300 * 1024 * 1024) */
#define LINUXS_VIRT_ADDR2		(TDS_VIRT_ADDR + TDS_SIZE_ADDR)

/* The virtual addr of following sections are same no matter SM log */
#define ION_BASE_ADDR			0x9D100000
#define ION_SIZE_ADDR			0x02900000  /* (41 * 1024 * 1024) */
#define ION_VIRT_ADDR			(LINUX_VIRT_ADDR2 + LINUX_SIZE_ADDR2)

#define FB_BASE_ADDR			0x9FA00000
#define FB_SIZE_ADDR			0x00500000  /* (5 * 1024 * 1024) */
#define FB_VIRT_ADDR			(ION_VIRT_ADDR + ION_SIZE_ADDR)

#define RC_BASE_ADDR			0x9FF00000
#define RC_SIZE_ADDR			0x00100000  /* (1 * 1024 * 1024) */
#define RC_VIRT_ADDR			(FB_VIRT_ADDR + FB_SIZE_ADDR)

#define ELF_HEADER_ADDR			0x9F500000
#define ELF_HEADER_SIZE			0x00100000  /* (1 * 1024 * 1024) */

#ifdef CONFIG_ION
#define SPRD_ION_OVERLAY_SIZE		(CONFIG_SPRD_ION_OVERLAY_SIZE * SZ_1M)
#define SPRD_ION_SIZE			(ION_SIZE_ADDR - SPRD_ION_OVERLAY_SIZE)
#else
#define SPRD_ION_OVERLAY_SIZE		(0 * SZ_1M)
#define SPRD_ION_SIZE			(0 * SZ_1M)
#endif

#define SPRD_IO_MEM_SIZE		(ION_SIZE_ADDR)
#define SPRD_IO_MEM_BASE		(ION_BASE_ADDR)

#define SPRD_ION_BASE			(ION_BASE_ADDR)
#define SPRD_ION_OVERLAY_BASE		(ION_BASE_ADDR + SPRD_ION_SIZE)

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define SPRD_RAM_CONSOLE_SIZE		(RC_SIZE_ADDR)
#define SPRD_RAM_CONSOLE_START		(RC_BASE_ADDR)
#endif

#ifdef CONFIG_HTC_DBG_UNCACHE_FTRACE
#define FT_BASE_PHY (457 * SZ_1M + 0x80000000)
#define FT_SIZE_PHY (8 * SZ_1M)
#endif

#endif
