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

#ifndef __ASM_ARCH_HARDWARE_SC8825_H
#define __ASM_ARCH_HARDWARE_SC8825_H

#ifndef __ASM_ARCH_SCI_HARDWARE_H
#error  "Don't include this file directly, include <mach/hardware.h>"
#endif

/*
 * sc8825 internal I/O mappings
 *
 * We have the following mapping according to asic spec.
 * We have set some trap gaps in the vaddr.
 */
 #define SCI_IOMAP_BASE  0xEB000000

#define SCI_IOMAP(x)	(SCI_IOMAP_BASE + (x))

#ifndef SCI_ADDR
#define SCI_ADDR(_b_, _o_)                              ( (u32)(_b_) + (_o_) )
#endif

#define LL_DEBUG_UART_PHYS		SPRD_UART1_PHYS
#define LL_DEBUG_UART_BASE		SPRD_UART1_BASE

//sc8825 mapping begin. From [0xe0000000 -- 0xe032ffff]
#define SPRD_CORESIGHT_BASE		SCI_IOMAP(0x0)
#define SPRD_CORESIGHT_PHYS		0x10000000
#define SPRD_CORESIGHT_SIZE		SZ_64K

#define SPRD_CORE_BASE		SCI_IOMAP(0x20000)
#define SPRD_CORE_PHYS		0x10400000
#define SPRD_CORE_SIZE		SZ_8K

#define SPRD_MALI_BASE		SCI_IOMAP(0x40000)
#define SPRD_MALI_PHYS		0x10500000
#define SPRD_MALI_SIZE		SZ_64K

#define SPRD_NIC301_PHYS	0x10600000	//TODO

#define SPRD_L2_BASE		SCI_IOMAP(0x100000)
#define SPRD_L2_PHYS		0x10800000
#define SPRD_L2_SIZE		SZ_4K

#define SPRD_DMA0_BASE		SCI_IOMAP(0x102000)
#define SPRD_DMA0_PHYS		0X20100000
#define SPRD_DMA0_SIZE		SZ_4K

#define SPRD_DCAM_BASE		SCI_IOMAP(0x104000)
#define SPRD_DCAM_PHYS		0X20200000
#define SPRD_DCAM_SIZE		SZ_16K

#define SPRD_USB_BASE		SCI_IOMAP(0x110000)
#define SPRD_USB_PHYS		0X20300000
#define SPRD_USB_SIZE		SZ_4K

#define SPRD_BM0_BASE		SCI_IOMAP(0x200000)
#define SPRD_BM0_PHYS		0X20400000
#define SPRD_BM0_SIZE		(SZ_16K + SZ_4K)

#define SPRD_BM1_BASE		(SPRD_BM0_BASE + SZ_4K)
#define SPRD_BM1_SIZE		(SZ_4K)

#define SPRD_BM2_BASE		(SPRD_BM0_BASE + SZ_8K)
#define SPRD_BM2_SIZE		(SZ_4K)
#define SPRD_BM3_BASE		(SPRD_BM0_BASE + SZ_4K + SZ_8K)
#define SPRD_BM3_SIZE		(SZ_4K)
#define SPRD_BM4_BASE		(SPRD_BM0_BASE + SZ_16K)
#define SPRD_BM4_SIZE		(SZ_4K)

#define SPRD_SDIO0_BASE		SCI_IOMAP(0x210000)
#define SPRD_SDIO0_PHYS		0X20500000
#define SPRD_SDIO0_SIZE		SZ_4K

#define SPRD_SDIO1_BASE		SCI_IOMAP(0x212000)
#define SPRD_SDIO1_PHYS		0X20600000
#define SPRD_SDIO1_SIZE		SZ_4K

#define SPRD_LCDC_BASE		SCI_IOMAP(0x220000)
#define SPRD_LCDC_PHYS		0X20700000
#define SPRD_LCDC_SIZE		SZ_4K

#define SPRD_ROTO_BASE		SCI_IOMAP(0x222000)
#define SPRD_ROTO_PHYS		0X20800000
#define SPRD_ROTO_SIZE		SZ_4K

#define SPRD_AHB_BASE		SCI_IOMAP(0x400000)//NOTE
#define SPRD_AHB_PHYS		0X20900000
#define SPRD_AHB_SIZE		SZ_64K

#define SPRD_AXIBM0_BASE	SCI_IOMAP(0x230000)
#define SPRD_AXIBM0_PHYS	0X20A00000
#define SPRD_AXIBM0_SIZE	(SZ_4K + SZ_8K)

#define SPRD_AXIBM1_BASE	(SPRD_AXIBM0_BASE + SZ_4K)
#define SPRD_AXIBM2_BASE	(SPRD_AXIBM0_BASE + SZ_8K)

#define SPRD_HWLOCK0_BASE	SCI_IOMAP(0x240000)
#define SPRD_HWLOCK0_PHYS	0X20A03000
#define SPRD_HWLOCK0_SIZE	SZ_4K

#define SPRD_DRM_BASE		SCI_IOMAP(0x242000)
#define SPRD_DRM_PHYS		0X20B00000
#define SPRD_DRM_SIZE		SZ_4K

#define SPRD_MEA_BASE   	SCI_IOMAP(0x300000)
#define SPRD_MEA_PHYS  		0x20C00000
#define SPRD_MEA_SIZE  		SZ_64K

#define SPRD_SDIO2_BASE		SCI_IOMAP(0x244000)
#define SPRD_SDIO2_PHYS		0X20E00000
#define SPRD_SDIO2_SIZE		SZ_4K

#define SPRD_EMMC_BASE		SCI_IOMAP(0x246000)
#define SPRD_EMMC_PHYS		0X20F00000
#define SPRD_EMMC_SIZE		SZ_4K

#define SPRD_DISPLAY_BASE	SCI_IOMAP(0x248000)
#define SPRD_DISPLAY_PHYS	0X21000000
#define SPRD_DISPLAY_SIZE	SZ_4K

#define SPRD_NFC_BASE		SCI_IOMAP(0x24C000)
#define SPRD_NFC_PHYS		0X21100000
#define SPRD_NFC_SIZE		SZ_4K

#define SPRD_INTC0_BASE		SCI_IOMAP(0x254000)
#define SPRD_INTC0_PHYS		0X40003000
#define SPRD_INTC0_SIZE		SZ_8K

#define SPRD_INTC1_BASE		(SPRD_INTC0_BASE + 0x1000)

#define SPRD_GPTIMER_BASE		SCI_IOMAP(0x258000)
#define SPRD_GPTIMER_PHYS		0X41000000
#define SPRD_GPTIMER_SIZE		SZ_4K

#define SPRD_ADI_BASE		SCI_IOMAP(0x260000)
#define SPRD_ADI_PHYS		0X42000000
#define SPRD_ADI_SIZE		(SZ_32K - SZ_8K)

#define SPRD_VB_BASE		(SPRD_ADI_BASE + 0X3000)
#define SPRD_VB_PHYS		(0X42003000)

#define SPRD_UART0_BASE		SCI_IOMAP(0x270000)
#define SPRD_UART0_PHYS		0X43000000
#define SPRD_UART0_SIZE		SZ_4K

#define SPRD_UART1_BASE		SCI_IOMAP(0x272000)
#define SPRD_UART1_PHYS		0X44000000
#define SPRD_UART1_SIZE		SZ_4K

#define SPRD_SIM0_BASE		SCI_IOMAP(0x274000)
#define SPRD_SIM0_PHYS		0X45000000
#define SPRD_SIM0_SIZE		SZ_4K

#define SPRD_SIM1_BASE		SCI_IOMAP(0x278000)
#define SPRD_SIM1_PHYS		0X45003000
#define SPRD_SIM1_SIZE		SZ_4K

#define SPRD_I2C0_BASE		SCI_IOMAP(0x280000)
#define SPRD_I2C0_PHYS		0X46000000
#define SPRD_I2C0_SIZE		SZ_16K

#define SPRD_I2C1_BASE		(SPRD_I2C0_BASE + SZ_4K)
#define SPRD_I2C2_BASE		(SPRD_I2C0_BASE + SZ_8K)
#define SPRD_I2C3_BASE		(SPRD_I2C0_BASE + SZ_4K + SZ_8K)

#define SPRD_KPD_BASE		SCI_IOMAP(0x290000)
#define SPRD_KPD_PHYS		0X47000000
#define SPRD_KPD_SIZE		SZ_4K

#define SPRD_SYSCNT_BASE	SCI_IOMAP(0x292000)
#define SPRD_SYSCNT_PHYS	0X47003000
#define SPRD_SYSCNT_SIZE	SZ_4K

#define SPRD_PWM_BASE		SCI_IOMAP(0x294000)
#define SPRD_PWM_PHYS		0X48000000
#define SPRD_PWM_SIZE		SZ_4K

#define SPRD_EFUSE_BASE		SCI_IOMAP(0x296000)
#define SPRD_EFUSE_PHYS		0X49000000
#define SPRD_EFUSE_SIZE		SZ_4K

#define SPRD_GPIO_BASE		SCI_IOMAP(0x298000)
#define SPRD_GPIO_PHYS		0X4A000000
#define SPRD_GPIO_SIZE		SZ_4K

#define SPRD_EIC_BASE		SCI_IOMAP(0x29C000)
#define SPRD_EIC_PHYS		0X4A001000
#define SPRD_EIC_SIZE		SZ_4K

#define SPRD_IPI_BASE		SCI_IOMAP(0x2A0000)
#define SPRD_IPI_PHYS		0X4A002000
#define SPRD_IPI_SIZE		SZ_4K

#define SPRD_GREG_BASE		SCI_IOMAP(0x320000)	//
#define SPRD_GREG_PHYS		0X4B000000
#define SPRD_GREG_SIZE		SZ_64K

#define SPRD_PIN_BASE		SCI_IOMAP(0x2A2000)
#define SPRD_PIN_PHYS		0X4C000000
#define SPRD_PIN_SIZE		SZ_4K

#define SPRD_EPT_BASE		SCI_IOMAP(0x2B0000)
#define SPRD_EPT_PHYS		0X4D000000
#define SPRD_EPT_SIZE		SZ_4K

#define SPRD_UART2_BASE		SCI_IOMAP(0x2B2000)
#define SPRD_UART2_PHYS		0X4E000000
#define SPRD_UART2_SIZE		SZ_4K

#define SPRD_IIS0_BASE		SCI_IOMAP(0x2B4000)
#define SPRD_IIS0_PHYS		0X4E001000
#define SPRD_IIS0_SIZE		SZ_4K

#define SPRD_SPI0_BASE		SCI_IOMAP(0x2B8000)
#define SPRD_SPI0_PHYS		0X4E002000
#define SPRD_SPI0_SIZE		SZ_4K

#define SPRD_SPI1_BASE		SCI_IOMAP(0x2BC000)
#define SPRD_SPI1_PHYS		0X4E003000
#define SPRD_SPI1_SIZE		SZ_4K

#define SPRD_IIS1_BASE		SCI_IOMAP(0x2C0000)
#define SPRD_IIS1_PHYS		0X4E004000
#define SPRD_IIS1_SIZE		SZ_4K

#define SPRD_UART3_BASE		SCI_IOMAP(0x2C2000)
#define SPRD_UART3_PHYS		0X4E005000
#define SPRD_UART3_SIZE		SZ_4K

#define SPRD_SPI2_BASE		SCI_IOMAP(0x2C4000)
#define SPRD_SPI2_PHYS		0X4E006000
#define SPRD_SPI2_SIZE		SZ_4K

#define SPRD_MIPI_DSIC_BASE		SCI_IOMAP(0x2D0000)
#define SPRD_MIPI_DSIC_PHYS		0X60100000
#define SPRD_MIPI_DSIC_SIZE	SZ_4K

#define SPRD_CSI_BASE		SCI_IOMAP(0x2D2000)
#define SPRD_CSI_PHYS		0X60101000
#define SPRD_CSI_SIZE		SZ_4K

#define SPRD_LPDDR2C_BASE		SCI_IOMAP(0x2D4000)
#define SPRD_LPDDR2C_PHYS		0X60200000
#define SPRD_LPDDR2C_SIZE		SZ_8K
#define SPRD_LPDDR2_PHY_BASE	(SPRD_LPDDR2C_BASE + SZ_4K)

#define SPRD_IRAM_BASE		SCI_IOMAP(0X2DC000)
#define SPRD_IRAM_PHYS		0X00004000
#define SPRD_IRAM_SIZE		SZ_16K

#define SPRD_ISP_BASE		SCI_IOMAP(0x500000)
#define SPRD_ISP_PHYS		0X22000000
#define SPRD_ISP_SIZE		SZ_64K

#define SPRD_TDPROC_BASE		SCI_IOMAP(0x520000)
#define SPRD_TDPROC_PHYS		0X30000
#define SPRD_TDPROC_SIZE		SZ_4K

#define CORE_GIC_CPU_VA	   (SPRD_CORE_BASE + 0x100)
#define SC8825_VA_GLOBAL_TIMER  (SPRD_CORE_BASE + 0x200)
#define SC8825_VA_PRIVATE_TIMER (SPRD_CORE_BASE + 0x600)
#define SC_LOCAL_TIMER_PA (SPRD_CORE_PHYS + 0x600)
#define CORE_GIC_DIS_VA		(SPRD_CORE_BASE + 0x1000)

#define HOLDING_PEN_VADDR	(SPRD_AHB_BASE + 0x240)
#define CPU0_JUMP_VADDR		(HOLDING_PEN_VADDR + 0x4)
#define CPU_JUMP_VADDR		(HOLDING_PEN_VADDR + 0X8)

/* registers for watchdog ,RTC, touch panel, aux adc, analog die... */
#define SPRD_MISC_BASE	((unsigned int)SPRD_ADI_BASE)
#define SPRD_MISC_PHYS	((unsigned int)0X42000000)

#define ANA_CTL_GLB_BASE		( SPRD_MISC_BASE + 0x0600 )

#define ADC_BASE	((unsigned int)SPRD_ADI_BASE + 0x300)
#define ANA_CTL_INT_BASE		( SPRD_MISC_BASE + 0x380 )

#ifndef REGS_AHB_BASE
#define REGS_AHB_BASE                                   ( SPRD_AHB_BASE  + 0x200)
#endif

#ifndef REGS_GLB_BASE
#define REGS_GLB_BASE                                   ( SPRD_GREG_BASE )
#define ANA_REGS_GLB_BASE                               ( SPRD_MISC_BASE + 0x600 )
#define ANA_REGS_GLB2_BASE                              ( SPRD_MISC_BASE + 0x580 )
#endif

#define CHIP_ID_LOW_REG		(ANA_CTL_GLB_BASE + 0xf8)
#define CHIP_ID_HIGH_REG	(ANA_CTL_GLB_BASE + 0xfc)

#endif
