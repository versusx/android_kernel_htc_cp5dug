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
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/printk.h>

#include <mach/hardware.h>

#include <linux/sipc.h>
#include <linux/sipc_priv.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>

#ifdef CONFIG_MACH_Z4TD
#define SMSG_TXBUF_ADDR		(0)
#define SMSG_TXBUF_SIZE		(SZ_1K)
#define SMSG_RXBUF_ADDR		(SMSG_TXBUF_SIZE)
#define SMSG_RXBUF_SIZE		(SZ_1K)
#else
#define SMSG_TXBUF_ADDR		(0)
#define SMSG_TXBUF_SIZE		(SZ_2K)
#define SMSG_RXBUF_ADDR		(SMSG_TXBUF_SIZE)
#define SMSG_RXBUF_SIZE		(SZ_2K)
#endif

#define SMSG_RINGHDR		(SMSG_TXBUF_SIZE + SMSG_RXBUF_SIZE)
#define SMSG_TXBUF_RDPTR	(SMSG_RINGHDR + 0)
#define SMSG_TXBUF_WRPTR	(SMSG_RINGHDR + 4)
#define SMSG_RXBUF_RDPTR	(SMSG_RINGHDR + 8)
#define SMSG_RXBUF_WRPTR	(SMSG_RINGHDR + 12)

#ifdef CONFIG_SIPC_TD
extern uint32_t cpt_rxirq_status(void);
extern void cpt_rxirq_clear(void);
extern void cpt_txirq_trigger(void);

static struct smsg_ipc smsg_ipc_cpt = {
	.name = "sipc-td",
	.dst = SIPC_ID_CPT,
	.irq = IRQ_SIPC_CPT,
	.rxirq_status = cpt_rxirq_status,
	.rxirq_clear = cpt_rxirq_clear,
	.txirq_trigger = cpt_txirq_trigger,
	.use_wakelock = 1,
};

static int __init sipc_td_init(void)
{
	uint32_t base = (uint32_t)ioremap(CPT_RING_ADDR, CPT_RING_SIZE);

	memset((void *)base, 0x00, CPT_RING_SIZE);

	smsg_ipc_cpt.txbuf_size = SMSG_TXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_cpt.txbuf_addr = base + SMSG_TXBUF_ADDR;
	smsg_ipc_cpt.txbuf_rdptr = base + SMSG_TXBUF_RDPTR;
	smsg_ipc_cpt.txbuf_wrptr = base + SMSG_TXBUF_WRPTR;

	smsg_ipc_cpt.rxbuf_size = SMSG_RXBUF_SIZE / sizeof(struct smsg);;
	smsg_ipc_cpt.rxbuf_addr = base + SMSG_RXBUF_ADDR;
	smsg_ipc_cpt.rxbuf_rdptr = base + SMSG_RXBUF_RDPTR;
	smsg_ipc_cpt.rxbuf_wrptr = base + SMSG_RXBUF_WRPTR;

	return smsg_ipc_create(SIPC_ID_CPT, &smsg_ipc_cpt);
}
#endif

#ifdef CONFIG_SIPC_WCDMA
extern uint32_t cpw_rxirq_status(void);
extern void cpw_rxirq_clear(void);
extern void cpw_txirq_trigger(void);

static struct smsg_ipc smsg_ipc_cpw = {
	.name = "sipc-wcdma",
	.dst = SIPC_ID_CPW,
	.irq = IRQ_SIPC_CPW,
	.rxirq_status = cpw_rxirq_status,
	.rxirq_clear = cpw_rxirq_clear,
	.txirq_trigger = cpw_txirq_trigger,
	.use_wakelock = 1,
};

static int __init sipc_wcdma_init(void)
{
	uint32_t base = (uint32_t)ioremap(CPW_RING_ADDR, CPW_RING_SIZE);

	memset((void *)base, 0x00, CPW_RING_SIZE);

	smsg_ipc_cpw.txbuf_size = SMSG_TXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_cpw.txbuf_addr = base + SMSG_TXBUF_ADDR;
	smsg_ipc_cpw.txbuf_rdptr = base + SMSG_TXBUF_RDPTR;
	smsg_ipc_cpw.txbuf_wrptr = base + SMSG_TXBUF_WRPTR;

	smsg_ipc_cpw.rxbuf_size = SMSG_RXBUF_SIZE / sizeof(struct smsg);;
	smsg_ipc_cpw.rxbuf_addr = base + SMSG_RXBUF_ADDR;
	smsg_ipc_cpw.rxbuf_rdptr = base + SMSG_RXBUF_RDPTR;
	smsg_ipc_cpw.rxbuf_wrptr = base + SMSG_RXBUF_WRPTR;

	return smsg_ipc_create(SIPC_ID_CPW, &smsg_ipc_cpw);
}
#endif


static int __init sipc_init(void)
{
	uint32_t smem_size = 0;

#ifdef CONFIG_ARCH_SCX35
	//sci_glb_clr(REG_AON_APB_APB_RST0,BIT_IPI_SOFT_RST);
	//sci_glb_set(REG_AON_APB_APB_RST0,BIT_IPI_SOFT_RST);
	sci_glb_set(REG_AON_APB_APB_EB0, BIT_IPI_EB);
#endif


#ifdef CONFIG_SIPC_TD
	smem_size += CPT_SMEM_SIZE;
	sipc_td_init();
#endif

#ifdef CONFIG_SIPC_WCDMA
	smem_size += CPW_SMEM_SIZE;
	sipc_wcdma_init();
#endif
	smem_init(SIPC_SMEM_ADDR, smem_size);

	return 0;
}

arch_initcall(sipc_init);

MODULE_AUTHOR("Chen Gaopeng");
MODULE_DESCRIPTION("SIPC module driver");
MODULE_LICENSE("GPL");
