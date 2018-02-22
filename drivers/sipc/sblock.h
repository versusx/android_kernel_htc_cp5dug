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

#ifndef __SBLOCK_H
#define __SBLOCK_H

#define SMSG_CMD_SBLOCK_INIT		0x0001
#define SMSG_DONE_SBLOCK_INIT		0x0002

#define SMSG_EVENT_SBLOCK_SEND		0x0001
#define SMSG_EVENT_SBLOCK_RELEASE	0x0002

#define SBLOCK_STATE_IDLE		0
#define SBLOCK_STATE_READY		1
#define SBLOCK_STATE_RESET	       2

struct sblock_blks {
	uint32_t		addr; 
	uint32_t		length;
};

struct sblock_txunit {
	void*			addr; 
	struct list_head	list;
};

struct sblock_ring_header {
	
	uint32_t		txblk_addr;
	uint32_t		txblk_count;
	uint32_t		txblk_size;
	uint32_t		txblk_blks;
	uint32_t		txblk_rdptr;
	uint32_t		txblk_wrptr;

	
	uint32_t		rxblk_addr;
	uint32_t		rxblk_count;
	uint32_t		rxblk_size;
	uint32_t		rxblk_blks;
	uint32_t		rxblk_rdptr;
	uint32_t		rxblk_wrptr;
};

struct sblock_ring {
	struct sblock_ring_header	*header;
	void			*txblk_virt; 
	void			*rxblk_virt; 
	struct sblock_blks	*txblks;     
	struct sblock_blks	*rxblks;     

	struct sblock_txunit	*txunits;    
	struct list_head	txpool;
	spinlock_t		plock;

	uint32_t		txblk_count;

	struct mutex            txlock;
	struct mutex            rxlock;
	spinlock_t		txspinlock;
	spinlock_t		rxspinlock;

	wait_queue_head_t	getwait;
	wait_queue_head_t	recvwait;
};

struct sblock_mgr {
	uint8_t			dst;
	uint8_t			channel;
	volatile uint32_t	state;

	void			*smem_virt;
	uint32_t		smem_addr;
	uint32_t		smem_size;

	uint32_t		txblksz;
	uint32_t		rxblksz;

	struct sblock_ring	*ring;
	struct task_struct	*thread;
	wait_queue_head_t	thread_wait;
	struct mutex		statelock;

	void			(*handler)(int event, void *data);
	void			*data;
};

#endif
