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

#ifndef __SIPC_PRIV_H
#define __SIPC_PRIV_H

#include <linux/wakelock.h>

#ifdef CONFIG_MACH_DUMMY
#define SMSG_CACHE_NR		128
#else
#define SMSG_CACHE_NR		256
#endif

struct smsg_channel {
	
	wait_queue_head_t	rxwait;
	struct mutex		rxlock;

	
	uint32_t		wrptr[1];
	uint32_t		rdptr[1];
	struct smsg		caches[SMSG_CACHE_NR];
};

struct smsg_ipc {
	char			*name;
	uint8_t			dst;
	uint8_t			padding[3];

	
	uint32_t		txbuf_addr;
	uint32_t		txbuf_size;	
	uint32_t		txbuf_rdptr;
	uint32_t		txbuf_wrptr;

	
	uint32_t		rxbuf_addr;
	uint32_t		rxbuf_size;	
	uint32_t		rxbuf_rdptr;
	uint32_t		rxbuf_wrptr;

	
	int			irq;
	irq_handler_t		irq_handler;
	irq_handler_t		irq_threadfn;

	uint32_t 		(*rxirq_status)(void);
	void			(*rxirq_clear)(void);
	void			(*txirq_trigger)(void);

	
	struct task_struct	*thread;

	
	spinlock_t		txspinlock;

	struct mutex		rxlock;

	
	struct mutex		chlock;

	
	struct smsg_channel	*channels[SMSG_CH_NR];

	
	uint8_t			states[SMSG_CH_NR];
	wait_queue_head_t	smsg_wait;
	volatile uint32_t    smsg_state[SMSG_CH_NR];
	struct workqueue_struct*	smsg_wq;
	struct work_struct	smsg_work;

	uint32_t use_wakelock;
	struct wake_lock rx_wakelock;
	uint32_t rx_count;
	spinlock_t rx_count_plock;
};

#define SMSG_STATE_IDLE     0
#define SMSG_STATE_RESET    1


#define CHAN_STATE_UNUSED	0
#define CHAN_STATE_WAITING	1
#define CHAN_STATE_OPENED	2

int smsg_ipc_create(uint8_t dst, struct smsg_ipc *ipc);
int smsg_ipc_destroy(uint8_t dst);

int smem_init(uint32_t addr, uint32_t size);

#endif
