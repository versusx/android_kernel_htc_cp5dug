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
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>

#include <linux/sipc.h>
#include "sblock.h"

#ifdef CONFIG_MODEM_SILENT_RESET
#define TEST_SBLOCK_RESET (sblock->state == SBLOCK_STATE_RESET)
#define SBLOCK_INIT_STATE SBLOCK_STATE_RESET
#else
#define TEST_SBLOCK_RESET 0
#define SBLOCK_INIT_STATE SBLOCK_STATE_IDLE
#endif

static struct sblock_mgr *sblocks[SIPC_ID_NR][SMSG_CH_NR];

static void __sblock_put(struct sblock_mgr *sblock, uint32_t addr)
{
	void* virt_addr;
	uint32_t index;
	struct sblock_ring_header *ringhd;
	struct sblock_txunit *st;
	struct sblock_ring *ring;
	struct list_head *head;

#if 0
	ringhd = sblock->ring->header;
#else
	ring = sblock->ring;
	ringhd = ring->header;
	head = &sblock->ring->txpool;
#endif

	spin_lock(&sblock->ring->plock);
	virt_addr = addr - sblock->smem_addr + sblock->smem_virt;
	index = (virt_addr - sblock->smem_virt) / sblock->ring->header->txblk_size;

	if (index >= ringhd->txblk_count){
		LOGE("dst=%d channel=%d wrong sblock address = 0x%x, need to check\n", sblock->dst, sblock->channel, addr);
		spin_unlock(&sblock->ring->plock);
		return;
	}

	if (sblock->channel >= SMSG_CH_VBC && sblock->channel <= SMSG_CH_CAPTURE_VOIP) {
		list_for_each_entry(st, head, list) {
			
			if (st->addr == virt_addr) {
				LOGW("list st address is phys = 0x%x virt = 0x%x\n", addr, (unsigned)virt_addr);
				spin_unlock(&sblock->ring->plock);
				return;
			}
		}
	}

	list_add_tail(&sblock->ring->txunits[index].list, &sblock->ring->txpool);
	sblock->ring->txblk_count++;
	spin_unlock(&sblock->ring->plock);
}

static int sblock_thread(void *data)
{
	struct sblock_mgr *sblock = data;
	struct smsg mcmd, mrecv;
	int rval;

restart:
	rval = wait_event_interruptible(sblock->thread_wait,!TEST_SBLOCK_RESET);
	LOGI("dst=%d channel=%d sblock state is %d\n",
			sblock->dst, sblock->channel, sblock->state);

	
	rval = smsg_ch_open(sblock->dst, sblock->channel, -1);
	if (rval != 0) {
		LOGE("dst=%d channel=%d failed to open\n", sblock->dst, sblock->channel);
		if(TEST_SBLOCK_RESET || rval == -ERESTARTSYS)
			goto restart;
		else
			return rval;
	}

	
	while (!kthread_should_stop()) {
		if(TEST_SBLOCK_RESET){
			LOGI("dst=%d channel=%d is under reset status\n",
					sblock->dst, sblock->channel);
			smsg_ch_close(sblock->dst, sblock->channel, 0);
			goto restart;
		}

		
		smsg_set(&mrecv, sblock->channel, 0, 0, 0);
		smsg_recv(sblock->dst, &mrecv, -1);

		LOGD("dst=%d channel=%d recv msg type=%d, flag=0x%04x, value=0x%08x\n",
				sblock->dst, sblock->channel,
				mrecv.type, mrecv.flag, mrecv.value);

		switch (mrecv.type) {
		case SMSG_TYPE_OPEN:
			
			LOGI("dst=%d channel=%d recv open msg\n", sblock->dst, sblock->channel);
			smsg_open_ack(sblock->dst, sblock->channel);
			break;
		case SMSG_TYPE_CLOSE:
			
			LOGI("dst=%d channel=%d recv close msg\n", sblock->dst, sblock->channel);
			smsg_close_ack(sblock->dst, sblock->channel);
			if (sblock->handler) {
				sblock->handler(SBLOCK_NOTIFY_STATUS, sblock->data);
			}
			
			mutex_lock(&sblock->statelock);
			if(!TEST_SBLOCK_RESET){
				sblock->state = SBLOCK_STATE_IDLE;
			}
			mutex_unlock(&sblock->statelock);
			break;
		case SMSG_TYPE_CMD:
			
			LOGI("dst=%d channel=%d recv command msg\n",sblock->dst, sblock->channel);
			WARN_ON(mrecv.flag != SMSG_CMD_SBLOCK_INIT);
			smsg_set(&mcmd, sblock->channel, SMSG_TYPE_DONE,
					SMSG_DONE_SBLOCK_INIT, sblock->smem_addr);
			smsg_send(sblock->dst, &mcmd, -1);
			if (sblock->handler) {
				sblock->handler(SBLOCK_NOTIFY_STATUS, sblock->data);
			}
			
			LOGI("dst=%d channel=%d set sblock state ready\n", sblock->dst, sblock->channel);
			mutex_lock(&sblock->statelock);
			if(!TEST_SBLOCK_RESET){
				sblock->state = SBLOCK_STATE_READY;
			}
			mutex_unlock(&sblock->statelock);
			break;
		case SMSG_TYPE_EVENT:
			
			switch (mrecv.flag) {
			case SMSG_EVENT_SBLOCK_SEND:
				wake_up_interruptible_all(&sblock->ring->recvwait);
				if (sblock->handler) {
					sblock->handler(SBLOCK_NOTIFY_RECV, sblock->data);
				}
				break;
			case SMSG_EVENT_SBLOCK_RELEASE:
				__sblock_put(sblock, mrecv.value);
				wake_up_interruptible_all(&(sblock->ring->getwait));
				if (sblock->handler) {
					sblock->handler(SBLOCK_NOTIFY_GET, sblock->data);
				}
				break;
			default:
				rval = 1;
				break;
			}
			break;
		default:
			rval = 1;
			break;
		};
		sipc_rx_try_wake_unlock(sblock->dst);
		if (rval) {
			LOGW("non-handled sblock msg: %d-%d, %d, %d, %d\n",
					sblock->dst, sblock->channel,
					mrecv.type, mrecv.flag, mrecv.value);
			rval = 0;
		}
	}

	return rval;
}

int sblock_create(uint8_t dst, uint8_t channel,
		uint32_t txblocknum, uint32_t txblocksize,
		uint32_t rxblocknum, uint32_t rxblocksize)
{
	struct sblock_mgr *sblock;
	volatile struct sblock_ring_header *ringhd;
	uint32_t hsize;
	int i;

	sblock = kzalloc(sizeof(struct sblock_mgr) , GFP_KERNEL);
	if (!sblock) {
		return -ENOMEM;
	}

	sblock->state = SBLOCK_INIT_STATE;
	sblock->dst = dst;
	sblock->channel = channel;
	sblock->txblksz = txblocksize;
	sblock->rxblksz = rxblocksize;

	
	hsize = sizeof(struct sblock_ring_header);
	sblock->smem_size = hsize +
		txblocknum * txblocksize + rxblocknum * rxblocksize +
		(txblocknum + rxblocknum) * sizeof(struct sblock_blks);
	sblock->smem_addr = smem_alloc(sblock->smem_size);
	if (!sblock->smem_addr) {
		LOGE("Failed to allocate smem for sblock\n");
		kfree(sblock);
		return -ENOMEM;
	}
	sblock->smem_virt = ioremap(sblock->smem_addr, sblock->smem_size);
	if (!sblock->smem_virt) {
		LOGE("Failed to map smem for sblock\n");
		smem_free(sblock->smem_addr, sblock->smem_size);
		kfree(sblock);
		return -EFAULT;
	}

	
	sblock->ring = kzalloc(sizeof(struct sblock_ring), GFP_KERNEL);
	if (!sblock->ring) {
		LOGE("Failed to allocate ring for sblock\n");
		iounmap(sblock->smem_virt);
		smem_free(sblock->smem_addr, sblock->smem_size);
		kfree(sblock);
		return -ENOMEM;
	}
	ringhd = (volatile struct sblock_ring_header *)(sblock->smem_virt);
	ringhd->txblk_addr = sblock->smem_addr + hsize;
	ringhd->txblk_count = txblocknum;
	ringhd->txblk_size = txblocksize;
	ringhd->txblk_rdptr = 0;
	ringhd->txblk_wrptr = 0;
	ringhd->txblk_blks = sblock->smem_addr + hsize +
		txblocknum * txblocksize + rxblocknum * rxblocksize;
	ringhd->rxblk_addr = ringhd->txblk_addr + txblocknum * txblocksize;
	ringhd->rxblk_count = rxblocknum;
	ringhd->rxblk_size = rxblocksize;
	ringhd->rxblk_rdptr = 0;
	ringhd->rxblk_wrptr = 0;
	ringhd->rxblk_blks = ringhd->txblk_blks + txblocknum * sizeof(struct sblock_blks);

	sblock->ring->header = sblock->smem_virt;
	sblock->ring->txblk_virt = sblock->smem_virt +
		(ringhd->txblk_addr - sblock->smem_addr);
	sblock->ring->txblks = sblock->smem_virt +
		(ringhd->txblk_blks - sblock->smem_addr);
	sblock->ring->rxblk_virt = sblock->smem_virt +
		(ringhd->rxblk_addr - sblock->smem_addr);
	sblock->ring->rxblks = sblock->smem_virt +
		(ringhd->rxblk_blks - sblock->smem_addr);

	sblock->ring->txunits = kzalloc(sizeof(struct sblock_txunit) * txblocknum, GFP_KERNEL);
	if (!sblock->ring->txunits) {
		LOGE("Failed to allocate txunits for sblock\n");
		kfree(sblock->ring);
		iounmap(sblock->smem_virt);
		smem_free(sblock->smem_addr, sblock->smem_size);
		kfree(sblock);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&sblock->ring->txpool);
	for (i = 0; i < txblocknum; i++) {
		sblock->ring->txunits[i].addr = sblock->ring->txblk_virt + i * txblocksize;
		list_add_tail(&sblock->ring->txunits[i].list, &sblock->ring->txpool);
		sblock->ring->txblk_count++;
	}

	init_waitqueue_head(&sblock->ring->getwait);
	init_waitqueue_head(&sblock->ring->recvwait);
	mutex_init(&sblock->ring->txlock);
	mutex_init(&sblock->ring->rxlock);
	spin_lock_init(&sblock->ring->plock);
	spin_lock_init(&sblock->ring->txspinlock);
	spin_lock_init(&sblock->ring->rxspinlock);

	init_waitqueue_head(&(sblock->thread_wait));
	mutex_init(&sblock->statelock);

	sblock->thread = kthread_create(sblock_thread, sblock,
			"sblock-%d-%d", dst, channel);
	if (IS_ERR(sblock->thread)) {
		LOGE("Failed to create kthread: sblock-%d-%d\n", dst, channel);
		kfree(sblock->ring->txunits);
		kfree(sblock->ring);
		iounmap(sblock->smem_virt);
		smem_free(sblock->smem_addr, sblock->smem_size);
		kfree(sblock);
		return PTR_ERR(sblock->thread);
	}

	sblocks[dst][channel]=sblock;
	wake_up_process(sblock->thread);

	return 0;
}

void sblock_destroy(uint8_t dst, uint8_t channel)
{
	struct sblock_mgr *sblock = sblocks[dst][channel];

	sblock->state = SBLOCK_STATE_IDLE;
	kthread_stop(sblock->thread);

	kfree(sblock->ring->txunits);
	kfree(sblock->ring);
	iounmap(sblock->smem_virt);
	smem_free(sblock->smem_addr, sblock->smem_size);
	kfree(sblock);

	sblocks[dst][channel]=NULL;
}

int sblock_register_notifier(uint8_t dst, uint8_t channel,
		void (*handler)(int event, void *data), void *data)
{
	struct sblock_mgr *sblock = sblocks[dst][channel];

	if (!sblock) {
		LOGE("dst=%d channel=%d not ready!\n", dst, channel);
		return -ENODEV;
	}

	if (sblock->handler) {
		LOGE("dst=%d channel=%d handler already registered\n",
				dst, channel);
		return -EBUSY;
	}

	sblock->handler = handler;
	sblock->data = data;

	return 0;
}

int sblock_get(uint8_t dst, uint8_t channel, struct sblock *blk, int timeout)
{
	struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	struct list_head *head;
	struct sblock_txunit *txunit;
	int rval = 0;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		LOGI("dst=%d channel=%d not ready!\n", dst, channel);
		return -ENODEV;
	}

	ring = sblock->ring;
	ringhd = ring->header;
	head = &sblock->ring->txpool;

	if (list_empty(head)) {
		LOGI("dst=%d channel=%d wait free sblock ++\n", dst, channel);
		if (timeout == 0) {
			
			LOGW("dst=%d channel=%d is empty!\n",
					dst, channel);
			rval = -ENODATA;
		} else if (timeout < 0) {
			
			rval = wait_event_interruptible(ring->getwait, !list_empty(head) || TEST_SBLOCK_RESET );
			if (rval < 0) {
				LOGW("dst=%d channel=%d wait interrupted!\n",
						dst, channel);
			}
		} else {
			
			rval = wait_event_interruptible_timeout(ring->getwait,
					!list_empty(head) || TEST_SBLOCK_RESET, timeout);
			if (rval < 0) {
				LOGW("dst=%d channel=%d wait interrupted!\n",
						dst, channel);
			} else if (rval == 0) {
				LOGW("dst=%d channel=%d wait timeout!\n",
						dst, channel);
				rval = -ETIME;
			}
		}
		LOGI("dst=%d channel=%d wait free sblock --\n", dst, channel);
	}

	if (TEST_SBLOCK_RESET) {
		LOGI("dst=%d channel=%d is under reset state\n", dst, channel);
		rval = -EFAULT;
	}

	if (rval < 0) {
		return rval;
	}

	
	spin_lock(&ring->plock);
	if (!list_empty(head)) {
		txunit = list_entry(head->next, struct sblock_txunit, list);
		blk->addr = txunit->addr;
		blk->length = sblock->txblksz;
		list_del(head->next);
		ring->txblk_count--;
	} else {
		rval = -EAGAIN;
	}
	spin_unlock(&ring->plock);

	return rval;
}

int sblock_send(uint8_t dst, uint8_t channel, struct sblock *blk)
{
	struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	struct smsg mevt;
	int txpos, ret;
	unsigned long flags;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		LOGI("dst=%d channel=%d not ready!\n", dst, channel);
		return -ENODEV;
	}

	LOGD("dst=%d, channel=%d, addr=%p, len=%d\n",
			dst, channel, blk->addr, blk->length);

	ring = sblock->ring;
	ringhd = ring->header;

	spin_lock_irqsave(&ring->txspinlock, flags);

	if (TEST_SBLOCK_RESET){
		LOGD("under reset state\n");
		spin_unlock_irqrestore(&ring->txspinlock, flags);
		return 0;
	}

	txpos = ringhd->txblk_wrptr % ringhd->txblk_count;
	ring->txblks[txpos].addr = blk->addr - sblock->smem_virt + sblock->smem_addr;
	ring->txblks[txpos].length = blk->length;
	LOGD("wrptr=%d, txpos=%d, addr=%x\n",
			ringhd->txblk_wrptr, txpos, ring->txblks[txpos].addr);
	ringhd->txblk_wrptr = ringhd->txblk_wrptr + 1;
	smsg_set(&mevt, channel, SMSG_TYPE_EVENT, SMSG_EVENT_SBLOCK_SEND, ring->txblks[txpos].addr);

	ret = smsg_send(dst, &mevt, 0);

	spin_unlock_irqrestore(&ring->txspinlock, flags);

	return ret;
}

int seth_sblock_send(uint8_t dst, uint8_t channel, struct sblock *blk)
{
	struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	struct smsg mevt;
	int txpos, ret;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		LOGI("dst=%d channel=%d not ready!\n", dst, channel);
		return -ENODEV;
	}

	LOGD("dst=%d, channel=%d, addr=%p, len=%d\n",
			dst, channel, blk->addr, blk->length);

	ring = sblock->ring;
	ringhd = ring->header;

	mutex_lock(&ring->txlock);

	if (TEST_SBLOCK_RESET){
		LOGD("under reset state\n");
		mutex_unlock(&ring->txlock);
		return 0;
	}

	txpos = ringhd->txblk_wrptr % ringhd->txblk_count;
	ring->txblks[txpos].addr = blk->addr - sblock->smem_virt + sblock->smem_addr;
	ring->txblks[txpos].length = blk->length;
	LOGD("wrptr=%d, txpos=%d, addr=%x\n",
			ringhd->txblk_wrptr, txpos, ring->txblks[txpos].addr);
	ringhd->txblk_wrptr = ringhd->txblk_wrptr + 1;
	smsg_set(&mevt, channel, SMSG_TYPE_EVENT, SMSG_EVENT_SBLOCK_SEND, ring->txblks[txpos].addr);

	ret = smsg_send(dst, &mevt, -1);

	mutex_unlock(&ring->txlock);

	return ret;
}

int sblock_put(uint8_t dst, uint8_t channel, struct sblock *blk)
{
	uint32_t index;
	struct sblock_mgr *sblock;

	sblock = (struct sblock_mgr *)sblocks[dst][channel];

	if (!sblock)
		return -ENODEV;

	spin_lock(&sblock->ring->plock);
	index = (blk->addr - sblock->smem_virt) / sblock->ring->header->txblk_size;
	list_add_tail(&sblock->ring->txunits[index].list, &sblock->ring->txpool);
	sblock->ring->txblk_count++;
	spin_unlock(&sblock->ring->plock);

	return 0;
}

int sblock_receive(uint8_t dst, uint8_t channel, struct sblock *blk, int timeout)
{
	struct sblock_mgr *sblock = sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	int rxpos, rval = 0;
	unsigned long flags;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		LOGI("dst=%d channel=%d not ready!\n", dst, channel);
		return -ENODEV;
	}

	ring = sblock->ring;
	ringhd = ring->header;

	LOGD("dst=%d, channel=%d, timeout=%d\n", dst, channel, timeout);
	LOGD("wrptr=%d, rdptr=%d", ringhd->rxblk_wrptr, ringhd->rxblk_rdptr);

	if (ringhd->rxblk_wrptr == ringhd->rxblk_rdptr) {
		if (timeout == 0) {
			
			LOGW("dst=%d channel=%d is empty!\n",
					dst, channel);
			rval = -ENODATA;
		} else if (timeout < 0) {
			
			rval = wait_event_interruptible(ring->recvwait,
				(ringhd->rxblk_wrptr != ringhd->rxblk_rdptr) || TEST_SBLOCK_RESET);
			if (rval < 0) {
				LOGW("dst=%d channel=%d wait interrupted!\n",
						dst, channel);
			}
		} else {
			
			rval = wait_event_interruptible_timeout(ring->recvwait,
				(ringhd->rxblk_wrptr != ringhd->rxblk_rdptr) || TEST_SBLOCK_RESET, timeout);
			if (rval < 0) {
				LOGW("dst=%d channel=%d wait interrupted!\n",
						dst, channel);
			} else if (rval == 0) {
				LOGW("dst=%d channel=%d wait timeout!\n",
						dst, channel);
				rval = -ETIME;
			}
		}
	}

	if (TEST_SBLOCK_RESET){
		LOGW("sblock release!\n");
		rval = -ENODATA;
	}
	if (rval < 0) {
		return rval;
	}

	
	spin_lock_irqsave(&ring->rxspinlock, flags);
	if (ringhd->rxblk_wrptr != ringhd->rxblk_rdptr) {
		rxpos = ringhd->rxblk_rdptr % ringhd->rxblk_count;
		blk->addr = ring->rxblks[rxpos].addr - sblock->smem_addr + sblock->smem_virt;
		blk->length = ring->rxblks[rxpos].length;
		ringhd->rxblk_rdptr = ringhd->rxblk_rdptr + 1;
		LOGD("rxpos=%d, addr=%p, len=%d\n",
				rxpos, blk->addr, blk->length);
	} else {
		rval = -EAGAIN;
	}
	spin_unlock_irqrestore(&ring->rxspinlock, flags);

	return rval;
}

int seth_sblock_receive(uint8_t dst, uint8_t channel, struct sblock *blk, int timeout)
{
	struct sblock_mgr *sblock = sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	int rxpos, rval = 0;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		LOGI("dst=%d channel=%d not ready!\n", dst, channel);
		return -ENODEV;
	}

	ring = sblock->ring;
	ringhd = ring->header;

	LOGD("dst=%d, channel=%d, timeout=%d\n", dst, channel, timeout);
	LOGD("wrptr=%d, rdptr=%d", ringhd->rxblk_wrptr, ringhd->rxblk_rdptr);

	if (ringhd->rxblk_wrptr == ringhd->rxblk_rdptr) {
		if (timeout == 0) {
			
			LOGW("dst=%d channel=%d is empty!\n",
					dst, channel);
			rval = -ENODATA;
		} else if (timeout < 0) {
			
			rval = wait_event_interruptible(ring->recvwait,
				(ringhd->rxblk_wrptr != ringhd->rxblk_rdptr) || TEST_SBLOCK_RESET);
			if (rval < 0) {
				LOGW("dst=%d channel=%d wait interrupted!\n",
						dst, channel);
			}
		} else {
			
			rval = wait_event_interruptible_timeout(ring->recvwait,
				(ringhd->rxblk_wrptr != ringhd->rxblk_rdptr) || TEST_SBLOCK_RESET, timeout);
			if (rval < 0) {
				LOGW("dst=%d channel=%d wait interrupted!\n",
						dst, channel);
			} else if (rval == 0) {
				LOGW("dst=%d channel=%d wait timeout!\n",
						dst, channel);
				rval = -ETIME;
			}
		}
	}

	if (TEST_SBLOCK_RESET){
		LOGW("sblock release!\n");
		rval = -ENODATA;
	}
	if (rval < 0) {
		return rval;
	}

	
	mutex_lock(&ring->rxlock);
	if (ringhd->rxblk_wrptr != ringhd->rxblk_rdptr){
		rxpos = ringhd->rxblk_rdptr % ringhd->rxblk_count;
		blk->addr = ring->rxblks[rxpos].addr - sblock->smem_addr + sblock->smem_virt;
		blk->length = ring->rxblks[rxpos].length;
		ringhd->rxblk_rdptr = ringhd->rxblk_rdptr + 1;
		LOGD("rxpos=%d, addr=%p, len=%d\n",
				rxpos, blk->addr, blk->length);
	} else {
		rval = -EAGAIN;
	}
	mutex_unlock(&ring->rxlock);

	return rval;
}

int sblock_get_free_count(uint8_t dst, uint8_t channel)
{
        struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	struct sblock_ring *ring;
	int blk_count = 0;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		LOGI("dst=%d channel=%d not ready!\n", dst, channel);
		return -ENODEV;
	}

	ring = sblock->ring;
        spin_lock(&ring->plock);
	blk_count= ring->txblk_count;
	spin_unlock(&ring->plock);

	return blk_count;
}

int sblock_release(uint8_t dst, uint8_t channel, struct sblock *blk)
{
	struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	struct smsg mevt;
	uint32_t addr;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		LOGI("dst=%d channel=%d not ready!\n", dst, channel);
		return -ENODEV;
	}

	LOGD("dst=%d, channel=%d, addr=%p, len=%d\n",
			dst, channel, blk->addr, blk->length);

	ring = sblock->ring;
	ringhd = ring->header;

	addr = blk->addr - sblock->smem_virt + sblock->smem_addr;
	LOGD("addr=%x\n", addr);

	
	smsg_set(&mevt, channel, SMSG_TYPE_EVENT, SMSG_EVENT_SBLOCK_RELEASE, addr);
	smsg_send(dst, &mevt, -1);

	return 0;
}

int sblock_reset(uint8_t dst, uint8_t channel)
{
	struct sblock_mgr *sblock = sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	unsigned long flags;
	int rval = 0;

	if (!sblock ) {
		LOGI("dst=%d channel=%d fail to find device!\n", dst, channel);
		return -ENODEV;
	}

	LOGD("dst=%d channel=%d +++\n", dst, channel);

	ring = sblock->ring;
	ringhd = ring->header;

	
	mutex_lock(&sblock->statelock);
	sblock->state = SBLOCK_STATE_RESET;
	mutex_unlock(&sblock->statelock);

	smsg_reset(dst, channel);

	wake_up_interruptible_all(&sblock->ring->recvwait);

	if (channel >= SMSG_CH_VBC && channel <= SMSG_CH_CAPTURE_VOIP)
		spin_lock_irqsave(&ring->rxspinlock, flags);
	else
		mutex_lock(&ring->rxlock);

	ringhd->rxblk_wrptr = ringhd->rxblk_rdptr = 0;

	if (channel >= SMSG_CH_VBC && channel <= SMSG_CH_CAPTURE_VOIP)
		spin_unlock_irqrestore(&ring->rxspinlock, flags);
	else
		mutex_unlock(&ring->rxlock);

	wake_up_interruptible_all(&(sblock->ring->getwait));

	if (channel >= SMSG_CH_VBC && channel <= SMSG_CH_CAPTURE_VOIP)
		spin_lock_irqsave(&ring->txspinlock, flags);
	else
		mutex_lock(&ring->txlock);

	ringhd->txblk_wrptr = ringhd->txblk_rdptr = 0;

	if (channel >= SMSG_CH_VBC && channel <= SMSG_CH_CAPTURE_VOIP)
		spin_unlock_irqrestore(&ring->txspinlock, flags);
	else
		mutex_unlock(&ring->txlock);

	LOGD("dst=%d channel=%d ---\n", dst, channel);

	return rval;
}

int sblock_restart(uint8_t dst, uint8_t channel)
{
	struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	struct list_head *head;
	int i = 0,rval = 0;

	if (!sblock ){
		LOGI("dst=%d channel=%d fail to find device!\n", dst, channel);
		return -ENODEV;
	}

	LOGD("dst=%d channel=%d +++\n", dst, channel);

	ring = sblock->ring;
	ringhd = ring->header;
	head = &sblock->ring->txpool;

	
	while (!list_empty(head)) {
		list_del(head->next);
		ring->txblk_count--;
	}
	ring->txblk_count = 0;
	for (i = 0; i < ringhd->txblk_count; i++) {
		ring->txunits[i].addr = ring->txblk_virt + i * ringhd->txblk_size;
		list_add_tail(&ring->txunits[i].list, &ring->txpool);
		ring->txblk_count++;
	}

	sblock->state = SBLOCK_STATE_IDLE;
	smsg_restart(dst, channel);
	wake_up_interruptible(&sblock->thread_wait);

	LOGD("dst=%d channel=%d ---\n", dst, channel);

	return rval;
}

EXPORT_SYMBOL(sblock_create);
EXPORT_SYMBOL(sblock_destroy);
EXPORT_SYMBOL(sblock_register_notifier);
EXPORT_SYMBOL(sblock_get);
EXPORT_SYMBOL(sblock_send);
EXPORT_SYMBOL(seth_sblock_send);
EXPORT_SYMBOL(sblock_put);
EXPORT_SYMBOL(sblock_receive);
EXPORT_SYMBOL(seth_sblock_receive);
EXPORT_SYMBOL(sblock_get_free_count);
EXPORT_SYMBOL(sblock_release);
EXPORT_SYMBOL(sblock_reset);
EXPORT_SYMBOL(sblock_restart);

MODULE_AUTHOR("Chen Gaopeng");
MODULE_DESCRIPTION("SIPC/SBLOCK driver");
MODULE_LICENSE("GPL");
