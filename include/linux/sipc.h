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

#ifndef __SIPC_H
#define __SIPC_H

#include <linux/poll.h>

#ifndef SIPC_DEBUG
#define DRV_SIPC "[SIPC]"

#define LOGD(fmt, ...) pr_debug(DRV_SIPC " %s: " fmt, __func__, ##__VA_ARGS__)
#define LOGI(fmt, ...) pr_info(DRV_SIPC  " %s: " fmt, __func__, ##__VA_ARGS__)
#define LOGW(fmt, ...) pr_warn(DRV_SIPC " WARN %s: " fmt, __func__, ##__VA_ARGS__)
#define LOGE(fmt, ...) pr_err(DRV_SIPC " ERROR %s: " fmt, __func__, ##__VA_ARGS__)
#endif



enum {
	SIPC_ID_AP = 0,		
	SIPC_ID_CPT,		
	SIPC_ID_CPW,		
	SIPC_ID_WCN,		
	SIPC_ID_NR,		
};

struct smsg {
	uint8_t			channel;	
	uint8_t			type;		
	uint16_t		flag;		
	uint32_t		value;		
};

enum {
	SMSG_CH_CTRL = 0,	
	SMSG_CH_COMM,		
	SMSG_CH_RPC_AP,		
	SMSG_CH_RPC_CP,		
	SMSG_CH_PIPE,		
	SMSG_CH_PLOG,		
	SMSG_CH_TTY,		
	SMSG_CH_DATA0,		
	SMSG_CH_DATA1,		
	SMSG_CH_DATA2,		
	SMSG_CH_VBC,		
	SMSG_CH_PLAYBACK, 	
	SMSG_CH_CAPTURE,	
	
	SMSG_CH_CTRL_VOIP,      
	SMSG_CH_PLAYBACK_VOIP,  
	SMSG_CH_CAPTURE_VOIP,   
	
	SMSG_CH_NR,		
};

enum {
	SMSG_TYPE_NONE = 0,
	SMSG_TYPE_OPEN,		
	SMSG_TYPE_CLOSE,	
	SMSG_TYPE_DATA,		
	SMSG_TYPE_EVENT,	
	SMSG_TYPE_CMD,		
	SMSG_TYPE_DONE,		
	SMSG_TYPE_SMEM_ALLOC,	
	SMSG_TYPE_SMEM_FREE,	
	SMSG_TYPE_SMEM_DONE,	
	SMSG_TYPE_FUNC_CALL,	
	SMSG_TYPE_FUNC_RETURN,	
	SMSG_TYPE_NR,		
};

#define	SMSG_OPEN_MAGIC		0xBEEE
#define	SMSG_CLOSE_MAGIC	0xEDDD

int smsg_ch_open(uint8_t dst, uint8_t channel, int timeout);

int smsg_ch_close(uint8_t dst, uint8_t channel, int timeout);

int smsg_send(uint8_t dst, struct smsg *msg, int timeout);

int smsg_recv(uint8_t dst, struct smsg *msg, int timeout);

static inline void smsg_set(struct smsg *msg, uint8_t channel,
		uint8_t type, uint16_t flag, uint32_t value)
{
	memset(msg, 0x00, sizeof(struct smsg));
	msg->channel = channel;
	msg->type = type;
	msg->flag = flag;
	msg->value = value;
}

static inline void smsg_open_ack(uint8_t dst, uint16_t channel)
{
	struct smsg mopen;
	smsg_set(&mopen, channel, SMSG_TYPE_OPEN, SMSG_OPEN_MAGIC, 0);
	smsg_send(dst, &mopen, -1);
}

static inline void smsg_close_ack(uint8_t dst, uint16_t channel)
{
	struct smsg mclose;
	smsg_set(&mclose, channel, SMSG_TYPE_CLOSE, SMSG_CLOSE_MAGIC, 0);
	smsg_send(dst, &mclose, -1);
}

int smsg_reset(uint8_t dst, uint8_t channel);
int smsg_restart(uint8_t dst, uint8_t channel);

int sipc_rx_try_wake_lock(uint8_t dst);
int sipc_rx_try_wake_unlock(uint8_t dst);
int sipc_rx_wake_lock_active(uint8_t dst);


uint32_t smem_alloc(uint32_t size);

void smem_free(uint32_t addr, uint32_t size);


int sbuf_create(uint8_t dst, uint8_t channel, uint32_t bufnum,
		uint32_t txbufsize, uint32_t rxbufsize);

void sbuf_destroy(uint8_t dst, uint8_t channel);

/**
 * sbuf_write -- write data to a sbuf
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @bufid: buffer ID
 * @buf: data to be written
 * @len: data length
 * @timeout: milliseconds, 0 means no wait, -1 means unlimited
 * @return: written bytes on success, <0 on failue
 */
int sbuf_write(uint8_t dst, uint8_t channel, uint32_t bufid,
		void *buf, uint32_t len, int timeout);

/**
 * sbuf_read -- write data to a sbuf
 *
 * @dst: dest processor ID
 * @channel: channel ID
 * @bufid: buffer ID
 * @buf: data to be written
 * @len: data length
 * @timeout: milliseconds, 0 means no wait, -1 means unlimited
 * @return: read bytes on success, <0 on failue
 */
int sbuf_read(uint8_t dst, uint8_t channel, uint32_t bufid,
		void *buf, uint32_t len, int timeout);

int sbuf_poll_wait(uint8_t dst, uint8_t channel, uint32_t bufid,
		struct file *file, poll_table *wait);

int sbuf_status(uint8_t dst, uint8_t channel);

int sbuf_has_data(uint8_t dst, uint8_t channel, uint32_t bufid);

int sbuf_reset(uint8_t dst, uint8_t channel);

int sbuf_restart(uint8_t dst, uint8_t channel);


struct sblock {
	void		*addr;
	uint32_t	length;
};

int sblock_create(uint8_t dst, uint8_t channel,
		uint32_t txblocknum, uint32_t txblocksize,
		uint32_t rxblocknum, uint32_t rxblocksize);

void sblock_destroy(uint8_t dst, uint8_t channel);

#define	SBLOCK_NOTIFY_GET	0x01
#define	SBLOCK_NOTIFY_RECV	0x02
#define	SBLOCK_NOTIFY_STATUS	0x03
int sblock_register_notifier(uint8_t dst, uint8_t channel,
		void (*handler)(int event, void *data), void *data);

int sblock_get(uint8_t dst, uint8_t channel, struct sblock *blk, int timeout);

int sblock_send(uint8_t dst, uint8_t channel, struct sblock *blk);
int seth_sblock_send(uint8_t dst, uint8_t channel, struct sblock *blk);

int sblock_receive(uint8_t dst, uint8_t channel, struct sblock *blk, int timeout);
int seth_sblock_receive(uint8_t dst, uint8_t channel, struct sblock *blk, int timeout);

int sblock_release(uint8_t dst, uint8_t channel, struct sblock *blk);


int sblock_get_free_count(uint8_t dst, uint8_t channel);

int sblock_reset(uint8_t dst, uint8_t channel);

int sblock_restart(uint8_t dst, uint8_t channel);

int sblock_put(uint8_t dst, uint8_t channel, struct sblock *blk);


enum {
	SPRC_ID_NONE = 0,
	SPRC_ID_XXX,
	SPRC_ID_NR,
};

struct srpc_server {
	uint32_t		id;
	
};

struct srpc_client {
	uint32_t		id;
	
};

int srpc_init(uint8_t dst);
int srpc_server_register(uint8_t dst, struct srpc_server *server);
int srpc_client_call(uint8_t dst, struct srpc_client *client);

#define GPIO27_GPIO		27

#define PIN_PULL_UP		1
#define PIN_PULL_DOWN		0

#endif
