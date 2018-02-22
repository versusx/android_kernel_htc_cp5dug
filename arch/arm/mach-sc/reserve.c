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

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include <asm/memory.h>
#include <linux/memblock.h>

static int __init __iomem_reserve_memblock(void)
{
	if (memblock_is_region_reserved(SPRD_IO_MEM_BASE, SPRD_IO_MEM_SIZE))
		return -EBUSY;
	if (memblock_reserve(SPRD_IO_MEM_BASE, SPRD_IO_MEM_SIZE))
		return -ENOMEM;
	return 0;
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
int __init __ramconsole_reserve_memblock(void)
{
	if (memblock_is_region_reserved(SPRD_RAM_CONSOLE_START, SPRD_RAM_CONSOLE_SIZE))
		return -EBUSY;
	if (memblock_reserve(SPRD_RAM_CONSOLE_START, SPRD_RAM_CONSOLE_SIZE))
		return -ENOMEM;
	return 0;
}
#endif

#ifdef CONFIG_HTC_DBG_UNCACHE_FTRACE
int __init __ftrace_reserve_memblock(void)
{
	if (memblock_is_region_reserved(FT_BASE_PHY, FT_SIZE_PHY))
		return -EBUSY;
	if (memblock_reserve(FT_BASE_PHY, FT_SIZE_PHY))
		return -ENOMEM;

	return 0;
}
#endif

void __init sci_reserve(void)
{
#ifndef PROJECT_SPECIAL_LAYOUT
	int ret = __iomem_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for iomem. errno=%d\n", ret);

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	ret = __ramconsole_reserve_memblock();
	if (ret != 0)
		pr_err("Fail to reserve mem for ram_console. errno=%d\n", ret);
#endif
#else
#ifdef CONFIG_HTC_DBG_UNCACHE_FTRACE
	if (htc_get_config(HTC_DBG_FLAG_SUPERMAN) & SUPERMAN_FLAG_FTRACE) {
		int ret = __ftrace_reserve_memblock();
		if (ret)
			pr_err("Fail to reserve mem for FT. errno=%d\n", ret);
	}
#endif
	printk(KERN_INFO "[k] Using project special memory layout \n");
#endif
}
