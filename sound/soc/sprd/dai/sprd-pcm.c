/*
 * sound/soc/sprd/dai/sprd-pcm.c
 *
 * SpreadTrum DMA for the pcm stream.
 *
 * Copyright (C) 2012 SpreadTrum Ltd.
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
#define pr_fmt(fmt) "[audio: pcm ] " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <mach/dma.h>
#include <mach/sprd-audio.h>

#include "sprd-pcm.h"
#include "vaudio/vaudio.h"
#include "i2s/i2s.h"

#ifdef CONFIG_SPRD_AUDIO_DEBUG
#define sprd_pcm_dbg pr_debug
#else
#define sprd_pcm_dbg(...)
#endif

#ifndef  DMA_LINKLIST_CFG_NODE_SIZE
#define DMA_LINKLIST_CFG_NODE_SIZE  (sizeof(sprd_dma_desc))
#endif

struct sprd_runtime_data {
	int dma_addr_offset;
	struct sprd_pcm_dma_params *params;
	int uid_cid_map[2];
	int int_pos_update[2];
#ifdef DMA_VER_R1P0
	sprd_dma_desc *dma_desc_array;
#else
	sprd_dma_desc *dma_cfg_array;
	dma_addr_t *dma_desc_array;
#endif
	dma_addr_t dma_desc_array_phys;
	int burst_len;
	int hw_chan;
	int dma_pos_pre[2];
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
	int interleaved;
#endif
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	int buffer_in_iram;
#endif
};

#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
#define SPRD_AUDIO_DMA_NODE_SIZE (1024)
#endif

static const struct snd_pcm_hardware sprd_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
	    SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_NONINTERLEAVED |
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
	    SNDRV_PCM_INFO_INTERLEAVED |
#endif
	    SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	/* 16bits, stereo-2-channels */
	.period_bytes_min = VBC_FIFO_FRAME_NUM * 4,
	/* non limit */
	.period_bytes_max = VBC_FIFO_FRAME_NUM * 4 * 100,
	.periods_min = 1,
	/* non limit */
	.periods_max = PAGE_SIZE / DMA_LINKLIST_CFG_NODE_SIZE,
	.buffer_bytes_max = VBC_BUFFER_BYTES_MAX,
};

static const struct snd_pcm_hardware sprd_i2s_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
	    SNDRV_PCM_INFO_MMAP_VALID |
	    SNDRV_PCM_INFO_INTERLEAVED |
	    SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	/* 16bits, stereo-2-channels */
	.period_bytes_min = VBC_FIFO_FRAME_NUM * 4,  //HTC_AUD modify by eleven
	/* non limit */
	.period_bytes_max = VBC_FIFO_FRAME_NUM * 4 * 100,  //HTC_modify add by eleven
	.periods_min = 1,
	/* non limit */
	.periods_max = PAGE_SIZE / DMA_LINKLIST_CFG_NODE_SIZE,
	.buffer_bytes_max = I2S_BUFFER_BYTES_MAX,
};

static inline int sprd_is_vaudio(struct snd_soc_dai *cpu_dai)
{
	return ((cpu_dai->driver->id == VAUDIO_MAGIC_ID) ||(cpu_dai->driver->id == VAUDIO_MAGIC_ID+1));
}

static inline int sprd_is_i2s(struct snd_soc_dai *cpu_dai)
{
	return (cpu_dai->driver->id == I2S_MAGIC_ID);
}

static inline const char *sprd_dai_pcm_name(struct snd_soc_dai *cpu_dai)
{
	if (sprd_is_i2s(cpu_dai)) {
		return "I2S";
	} else if (sprd_is_vaudio(cpu_dai)) {
		return "VAUDIO";
	}
	return "VBC";
}

#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
static char *s_mem_for_iram = 0;
static char *s_iram_remap_base = 0;

static int sprd_buffer_iram_backup(void)
{
	void __iomem *iram_start;
	sprd_pcm_dbg("Entering %s 0x%x\n", __func__, (int)s_mem_for_iram);
	if (!s_iram_remap_base) {
		s_iram_remap_base =
		    ioremap_nocache(SPRD_IRAM_ALL_PHYS, SPRD_IRAM_ALL_SIZE);
	}
	if (!s_mem_for_iram) {
		s_mem_for_iram = kzalloc(SPRD_IRAM_ALL_SIZE, GFP_KERNEL);
	} else {
		sprd_pcm_dbg("iram is backup, be careful use iram!\n");
		return 0;
	}
	if (!s_mem_for_iram) {
		pr_err("iram backup error\n");
		return -ENOMEM;
	}
	iram_start = (void __iomem *)(s_iram_remap_base);
	memcpy_fromio(s_mem_for_iram, iram_start, SPRD_IRAM_ALL_SIZE);
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return 0;
}

static int sprd_buffer_iram_restore(void)
{
	void __iomem *iram_start;
	sprd_pcm_dbg("Entering %s 0x%x\n", __func__, (int)s_mem_for_iram);
	if (!s_mem_for_iram) {
		pr_err("iram not backup\n");
		return 0;
	}
	iram_start = (void __iomem *)(s_iram_remap_base);
	memcpy_toio(iram_start, s_mem_for_iram, SPRD_IRAM_ALL_SIZE);
	kfree(s_mem_for_iram);
	s_mem_for_iram = 0;
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return 0;
}
#endif

#ifdef CONFIG_SPRD_VBC_INTERLEAVED
static inline int sprd_pcm_is_interleaved(struct snd_pcm_runtime *runtime)
{
	return (runtime->access == SNDRV_PCM_ACCESS_RW_INTERLEAVED ||
		runtime->access == SNDRV_PCM_ACCESS_MMAP_INTERLEAVED);
}
#endif

#define PCM_DIR_NAME(stream) (stream == SNDRV_PCM_STREAM_PLAYBACK ? "Playback" : "Captrue")

static int sprd_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct sprd_runtime_data *rtd;
	struct i2s_private *i2s_private;
	struct i2s_config *config;
	int burst_len;
	int hw_chan;
	int ret;

	pr_info("%s open %s\n", sprd_dai_pcm_name(srtd->cpu_dai),
		PCM_DIR_NAME(substream->stream));

	if (sprd_is_i2s(srtd->cpu_dai)) {
		snd_soc_set_runtime_hwparams(substream, &sprd_i2s_pcm_hardware);
		i2s_private = srtd->cpu_dai->ac97_pdata;
		config = i2s_private->config;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			burst_len = I2S_FIFO_DEPTH - config->tx_watermark;
		} else {
			burst_len = config->rx_watermark;
		}
		burst_len <<= config->byte_per_chan;
		hw_chan = 1;
	} else {
		snd_soc_set_runtime_hwparams(substream, &sprd_pcm_hardware);
		burst_len = (VBC_FIFO_FRAME_NUM * 4);
		hw_chan = 2;
	}

	/*
	 * For mysterious reasons (and despite what the manual says)
	 * playback samples are lost if the DMA count is not a multiple
	 * of the DMA burst size.  Let's add a rule to enforce that.
	 */
	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
					 burst_len);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
					 burst_len);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	ret = -ENOMEM;
	rtd = kzalloc(sizeof(*rtd), GFP_KERNEL);
	if (!rtd)
		goto out;
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	if (sprd_is_i2s(srtd->cpu_dai)
	    || !((substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		 && 0 == sprd_buffer_iram_backup())) {
#endif
		rtd->dma_desc_array =
		    dma_alloc_writecombine(substream->pcm->card->dev,
				       hw_chan * PAGE_SIZE,
				       &rtd->dma_desc_array_phys, GFP_KERNEL);
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	} else {
		runtime->hw.periods_max =
			SPRD_AUDIO_DMA_NODE_SIZE / DMA_LINKLIST_CFG_NODE_SIZE,
		runtime->hw.buffer_bytes_max =
		    SPRD_IRAM_ALL_SIZE - (2 * SPRD_AUDIO_DMA_NODE_SIZE),
		    rtd->dma_desc_array =
		    (void *)(s_iram_remap_base + runtime->hw.buffer_bytes_max);
		rtd->dma_desc_array_phys =
		    SPRD_IRAM_ALL_PHYS + runtime->hw.buffer_bytes_max;
		rtd->buffer_in_iram = 1;
		/*must clear the dma_desc_array first here*/
		memset(rtd->dma_desc_array, 0, (2 * SPRD_AUDIO_DMA_NODE_SIZE));
	}
#endif
	if (!rtd->dma_desc_array)
		goto err1;

#ifdef DMA_VER_R4P0
	rtd->dma_cfg_array =
		kzalloc(hw_chan * runtime->hw.periods_max * sizeof(sprd_dma_desc), GFP_KERNEL);
	if (!rtd->dma_cfg_array)
		goto err2;
#endif

	rtd->uid_cid_map[0] = rtd->uid_cid_map[1] = -1;

	rtd->burst_len = burst_len;
	rtd->hw_chan = hw_chan;

	runtime->private_data = rtd;
	ret = 0;
	goto out;

#ifdef DMA_VER_R4P0
err2:
	pr_err("dma_cfg_array alloc fail!\n");

#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
		if (rtd->buffer_in_iram)
			sprd_buffer_iram_restore();
		else
#endif
		dma_free_writecombine(substream->pcm->card->dev,
				  hw_chan * PAGE_SIZE,
				  rtd->dma_desc_array,
				  rtd->dma_desc_array_phys);
#endif
err1:
	pr_err("dma_desc_array alloc fail!\n");
	kfree(rtd);
out:
	sprd_pcm_dbg("return %i\n", ret);
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return ret;
}

static int sprd_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sprd_runtime_data *rtd = runtime->private_data;

	pr_info("close %s\n", PCM_DIR_NAME(substream->stream));

#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	if (rtd->buffer_in_iram)
		sprd_buffer_iram_restore();
	else
#endif
	dma_free_writecombine(substream->pcm->card->dev,
			  rtd->hw_chan * PAGE_SIZE,
			  rtd->dma_desc_array,
			  rtd->dma_desc_array_phys);
#ifdef DMA_VER_R4P0
	kfree(rtd->dma_cfg_array);
#endif
	kfree(rtd);

	sprd_pcm_dbg("Leaving %s\n", __func__);

	return 0;
}

#ifdef DMA_VER_R1P0
static irqreturn_t sprd_pcm_dma_irq_ch(int dma_ch, void *dev_id)
#else
static void sprd_pcm_dma_irq_ch(int dma_ch, void *dev_id)
#endif
{
	struct snd_pcm_substream *substream = dev_id;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sprd_runtime_data *rtd = runtime->private_data;
	int i = 0;

	if (rtd->hw_chan == 1)
		goto irq_fast;

	for (i = 0; i < 2; i++) {
		if (dma_ch == rtd->uid_cid_map[i]) {
			rtd->int_pos_update[i] = 1;

			if (rtd->uid_cid_map[1 - i] >= 0) {
				if (rtd->int_pos_update[1 - i])
					goto irq_ready;
			} else {
				goto irq_ready;
			}
		}
	}
	goto irq_ret;
irq_ready:
	rtd->int_pos_update[0] = 0;
	rtd->int_pos_update[1] = 0;
irq_fast:
	snd_pcm_period_elapsed(dev_id);
irq_ret:
#ifdef  DMA_VER_R1P0
	return IRQ_HANDLED;
#else
	return;
#endif
}

#ifdef DMA_VER_R1P0
static int sprd_pcm_dma_config(struct snd_pcm_substream *substream)
{
	struct sprd_runtime_data *rtd = substream->runtime->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sprd_pcm_dma_params *dma;
	struct sprd_dma_channel_desc dma_cfg = { 0 };
	sprd_dma_desc *dma_desc[2];
	dma_addr_t next_desc_phys[2];
	int i;

	sprd_pcm_dbg("Entering %s\n", __func__);

	if (!rtd || !rtd->params)
		return 0;

	dma = rtd->params;
	dma_cfg = dma->desc;

	dma_desc[0] = rtd->dma_desc_array;
	dma_desc[1] = rtd->dma_desc_array + runtime->hw.periods_max;
	next_desc_phys[0] = rtd->dma_desc_array_phys;
	next_desc_phys[1] = rtd->dma_desc_array_phys +
	    runtime->hw.periods_max * sizeof(sprd_dma_desc);
	for (i = 0; i < rtd->hw_chan; i++) {
		if (rtd->uid_cid_map[i] >= 0) {
			dma_cfg.llist_ptr = next_desc_phys[i];
			dma_cfg.src_addr = dma_desc[i]->dsrc;
			dma_cfg.dst_addr = dma_desc[i]->ddst;
			sprd_dma_channel_config(rtd->uid_cid_map[i],
						dma->workmode, &dma_cfg);
		}
	}

	sprd_pcm_dbg("Leaving %s\n", __func__);

	return 0;
}

static int sprd_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sprd_runtime_data *rtd = runtime->private_data;
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct sprd_pcm_dma_params *dma;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	struct i2s_private *i2s_private;
	struct i2s_config *config;
	sprd_dma_desc *dma_desc[2];
	dma_addr_t dma_buff_phys[2], next_desc_phys[2];
	int ret = 0;
	int i;
	int used_chan_count;

	sprd_pcm_dbg("Entering %s\n", __func__);

	dma = snd_soc_dai_get_dma_data(srtd->cpu_dai, substream);

	if (!dma)
		goto no_dma;

	used_chan_count = params_channels(params);
	pr_info("chan=%d totsize=%d period=%d\n", used_chan_count, totsize,
		period);

	if (sprd_is_i2s(srtd->cpu_dai)) {
		i2s_private = srtd->cpu_dai->ac97_pdata;
		config = i2s_private->config;
		used_chan_count = rtd->hw_chan;
	} else {
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
		rtd->interleaved = (used_chan_count == 2)
		    && sprd_pcm_is_interleaved(runtime);
		if (rtd->interleaved) {
			sprd_pcm_dbg("interleaved access\n");
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				dma->desc.src_burst_mode =
				    SRC_BURST_MODE_SINGLE;
			} else {
				dma->desc.dst_burst_mode =
				    SRC_BURST_MODE_SINGLE;
			}
		}
#endif
	}

	/* this may get called several times by oss emulation
	 * with different params */
	if (rtd->params == NULL) {
		rtd->params = dma;
		for (i = 0; i < used_chan_count; i++) {
			ret = sprd_dma_request(dma->channels[i],
					       sprd_pcm_dma_irq_ch, substream);
			if (ret < 0) {
				pr_err("sprd-pcm request dma error %d\n",
				       dma->channels[i]);
				for (i--; i >= 0; i--) {
					sprd_dma_free(rtd->uid_cid_map[i]);
					rtd->uid_cid_map[i] = -1;
					rtd->params = NULL;
				}
				goto hw_param_err;
			}
			sprd_pcm_dbg("chan%d dma id=%d\n", i, ret);
			rtd->uid_cid_map[i] = ret;
			sprd_dma_set_irq_type(rtd->uid_cid_map[i],
					      rtd->params->irq_type, ON);
		}
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	runtime->dma_bytes = totsize;

	dma_desc[0] = rtd->dma_desc_array;
	dma_desc[1] = rtd->dma_desc_array + runtime->hw.periods_max;
	next_desc_phys[0] = rtd->dma_desc_array_phys;
	next_desc_phys[1] = rtd->dma_desc_array_phys +
	    runtime->hw.periods_max * sizeof(sprd_dma_desc);
	dma_buff_phys[0] = runtime->dma_addr;
	rtd->dma_addr_offset = (totsize / used_chan_count);
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
	if (sprd_pcm_is_interleaved(runtime))
		rtd->dma_addr_offset = 2;
#endif
	dma_buff_phys[1] = runtime->dma_addr + rtd->dma_addr_offset;
	if (!sprd_is_i2s(srtd->cpu_dai)) {
		rtd->burst_len = (VBC_FIFO_FRAME_NUM * 2);
	}
	sprd_pcm_dbg("burst lenght=%d\n", rtd->burst_len);

	do {
		for (i = 0; i < used_chan_count; i++) {
			next_desc_phys[i] += sizeof(sprd_dma_desc);
			if (rtd->params->workmode == DMA_LINKLIST) {
				dma_desc[i]->llptr = next_desc_phys[i];
			}

			dma_desc[i]->cfg = dma->desc.cfg_req_mode_sel |
			    dma->desc.cfg_src_data_width |
			    dma->desc.cfg_dst_data_width | rtd->burst_len;
			dma_desc[i]->sbm = dma->desc.src_burst_mode |
			    dma->desc.src_blk_postm;
			dma_desc[i]->dbm = dma->desc.dst_burst_mode |
			    dma->desc.dst_blk_postm;
			dma_desc[i]->tlen = period / used_chan_count;

			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				dma_desc[i]->dsrc = dma_buff_phys[i];
				dma_desc[i]->ddst = dma->dev_paddr[i];
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
				if (rtd->interleaved)
					dma_desc[i]->pmod =
					    (4 << SRC_ELEM_POSTM_SHIFT);
#endif
			} else {
				dma_desc[i]->dsrc = dma->dev_paddr[i];
				dma_desc[i]->ddst = dma_buff_phys[i];
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
				if (rtd->interleaved)
					dma_desc[i]->pmod = 4;
#endif
			}
			dma_buff_phys[i] += dma_desc[i]->tlen;
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
			if (rtd->interleaved)
				dma_buff_phys[i] += dma_desc[i]->tlen;
#endif
			dma_desc[i]++;
		}

		if (period > totsize)
			period = totsize;

	} while (totsize -= period);

	if (rtd->params->workmode == DMA_LINKLIST) {
		dma_desc[0][-1].llptr = rtd->dma_desc_array_phys;
		if (used_chan_count > 1) {
			dma_desc[1][-1].llptr = rtd->dma_desc_array_phys
			    + runtime->hw.periods_max * sizeof(sprd_dma_desc);
		}
	}

	sprd_pcm_dma_config(substream);

	goto ok_go_out;

no_dma:
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;
hw_param_err:
ok_go_out:
	sprd_pcm_dbg("return %i\n", ret);
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return ret;
}
#endif

#ifdef DMA_VER_R4P0
static int sprd_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sprd_runtime_data *rtd = runtime->private_data;
	struct snd_soc_pcm_runtime *srtd = substream->private_data;
	struct sprd_pcm_dma_params *dma;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	sprd_dma_desc *dma_desc[2];
	dma_addr_t dma_buff_phys[2];	/*, next_desc_phys[2]; */
	struct i2s_private *i2s_private;
	struct i2s_config *config = NULL;
	int ret = 0;
	int i, j = 0;
	int used_chan_count;
	int chan_id;
	struct reg_cfg_addr dma_reg_addr[2];

	sprd_pcm_dbg("Entering %s\n", __func__);

	dma = snd_soc_dai_get_dma_data(srtd->cpu_dai, substream);
	if (!dma)
		goto no_dma;

	used_chan_count = params_channels(params);
	pr_info("chan=%d totsize=%d period=%d\n", used_chan_count, totsize,
		period);
	if (sprd_is_i2s(srtd->cpu_dai)) {
		i2s_private = srtd->cpu_dai->ac97_pdata;
		config = i2s_private->config;
		used_chan_count = rtd->hw_chan;
	} else {
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
		rtd->interleaved = (used_chan_count == 2)
		    && sprd_pcm_is_interleaved(runtime);
		if (rtd->interleaved) {
			sprd_pcm_dbg("interleaved access\n");
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				dma->desc.src_step = 4;
			} else {
				dma->desc.des_step = 4;
			}
		} else {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				dma->desc.src_step = 2;
			} else {
				dma->desc.des_step = 2;
			}
		}
#endif
	}

	/* this may get called several times by oss emulation
	 * with different params */
	if (rtd->params == NULL) {
		rtd->params = dma;
		for (i = 0; i < used_chan_count; i++) {
			chan_id = sci_dma_request(dma->name, FULL_DMA_CHN);
			if (chan_id < 0) {
				pr_err("sprd-vbc-pcm request dma error %d\n",
				       dma->channels[i]);
				for (i--; i >= 0; i--) {
					sci_dma_free(rtd->uid_cid_map[i]);
					rtd->uid_cid_map[i] = -1;
					rtd->params = NULL;
				}
				goto hw_param_err;
			}
			rtd->uid_cid_map[i] = chan_id;
			sprd_pcm_dbg("chan%d dma id=%d\n",
				     rtd->params->channels[i],
				     rtd->uid_cid_map[i]);
		}
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	runtime->dma_bytes = totsize;

	dma_desc[0] = rtd->dma_cfg_array;
	dma_desc[1] = rtd->dma_cfg_array + runtime->hw.periods_max;

	dma_buff_phys[0] = runtime->dma_addr;
	rtd->dma_addr_offset = (totsize / used_chan_count);
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
	if (sprd_pcm_is_interleaved(runtime))
		rtd->dma_addr_offset = 2;
#endif
	dma_buff_phys[1] = runtime->dma_addr + rtd->dma_addr_offset;

	do {
		for (i = 0; i < used_chan_count; i++) {
			dma_desc[i]->datawidth = dma->desc.datawidth;
			if (sprd_is_i2s(srtd->cpu_dai)) {
				/*dma_desc[i]->fragmens_len = rtd->burst_len;*/
				if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
					dma_desc[i]->fragmens_len = (I2S_FIFO_DEPTH - config->tx_watermark) * dma_desc[i]->datawidth;
				} else {
					dma_desc[i]->fragmens_len = config->rx_watermark * dma_desc[i]->datawidth;
				}
			} else {
				dma_desc[i]->fragmens_len =
				    dma->desc.fragmens_len;
			}
			dma_desc[i]->block_len = period / used_chan_count;
			dma_desc[i]->transcation_len = 0;
			dma_desc[i]->req_mode = FRAG_REQ_MODE;
			dma_desc[i]->src_step = dma->desc.src_step;
			dma_desc[i]->des_step = dma->desc.des_step;
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				dma_desc[i]->src_addr = dma_buff_phys[i];
				dma_desc[i]->des_addr = dma->dev_paddr[i];
			} else {
				dma_desc[i]->src_addr = dma->dev_paddr[i];
				dma_desc[i]->des_addr = dma_buff_phys[i];
			}

			dma_buff_phys[i] += dma_desc[i]->block_len;
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
			if (rtd->interleaved)
				dma_buff_phys[i] += dma_desc[i]->block_len;
#endif
			dma_desc[i]++;
		}

		if (period > totsize)
			period = totsize;
		j++;
	} while (totsize -= period);

	sprd_pcm_dbg("nodesize:%d\n", j);

	dma_reg_addr[0].phys_addr =
	    (u32) (rtd->dma_desc_array_phys);
	dma_reg_addr[0].virt_addr =
	    (u32) (rtd->dma_desc_array);
	sprd_pcm_dbg("dma_reg_addr[0].virt_addr:0x%x\n",
		     dma_reg_addr[0].virt_addr);
	sprd_pcm_dbg("dma_reg_addr[0].phys_addr:0x%x\n",
		     dma_reg_addr[0].phys_addr);

	sci_dma_config((u32)(rtd->uid_cid_map[0]), (struct sci_dma_cfg *)(rtd->dma_cfg_array),
		       params_periods(params), &dma_reg_addr[0]);
	ret = sci_dma_register_irqhandle(rtd->uid_cid_map[0], rtd->params->irq_type,	/*dma->irq_type */
					 sprd_pcm_dma_irq_ch, substream);

	if (used_chan_count > 1) {
		dma_reg_addr[1].phys_addr =
		    (u32) (dma_reg_addr[0].phys_addr) + runtime->hw.periods_max * DMA_LINKLIST_CFG_NODE_SIZE;
		dma_reg_addr[1].virt_addr =
		    (u32) (dma_reg_addr[0].virt_addr) + runtime->hw.periods_max * DMA_LINKLIST_CFG_NODE_SIZE;
		sprd_pcm_dbg("dma_reg_addr[1].virt_addr:0x%x\n",
			     dma_reg_addr[1].virt_addr);
		sprd_pcm_dbg("dma_reg_addr[1].phys_addr:0x%x\n",
			     dma_reg_addr[1].phys_addr);

		sci_dma_config((u32)(rtd->uid_cid_map[1]),
			       (struct sci_dma_cfg *)(rtd->dma_cfg_array + runtime->hw.periods_max),
			       params_periods(params), &dma_reg_addr[1]);
		ret = sci_dma_register_irqhandle(rtd->uid_cid_map[1], rtd->params->irq_type,	/*dma->irq_type */
						 sprd_pcm_dma_irq_ch,
						 substream);

	}

	sprd_pcm_dbg("Leaving %s\n", __func__);

	goto ok_go_out;

no_dma:
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;
hw_param_err:
	sprd_pcm_dbg("hw_param_err\n");
ok_go_out:
	sprd_pcm_dbg("return %i\n", ret);
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return ret;
}
#endif
static int sprd_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct sprd_runtime_data *rtd = substream->runtime->private_data;
	struct sprd_pcm_dma_params *dma = rtd->params;
	int i;

	sprd_pcm_dbg("Entering %s\n", __func__);

	snd_pcm_set_runtime_buffer(substream, NULL);

	if (dma) {
		for (i = 0; i < rtd->hw_chan; i++) {
			if (rtd->uid_cid_map[i] >= 0) {
#ifdef DMA_VER_R1P0
				sprd_dma_free(rtd->uid_cid_map[i]);
#endif
#ifdef DMA_VER_R4P0
				sci_dma_free(rtd->uid_cid_map[i]);
#endif
				rtd->uid_cid_map[i] = -1;
			}
		}
		rtd->params = NULL;
	}

	sprd_pcm_dbg("Leaving %s\n", __func__);

	return 0;
}

static int sprd_pcm_prepare(struct snd_pcm_substream *substream)
{
	sprd_pcm_dbg("Entering %s\n", __func__);

	sprd_pcm_dbg("Leaving %s\n", __func__);

	return 0;
}

static int sprd_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct sprd_runtime_data *rtd = substream->runtime->private_data;
	struct sprd_pcm_dma_params *dma = rtd->params;
	int ret = 0;
	int i;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		for (i = 0; i < rtd->hw_chan; i++) {
			if (rtd->uid_cid_map[i] >= 0) {
#ifdef DMA_VER_R1P0
				sprd_dma_start(rtd->uid_cid_map[i]);
#endif
#ifdef DMA_VER_R4P0
				sci_dma_start(rtd->uid_cid_map[i],
					      dma->channels[i]);
#endif
			}
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		for (i = 0; i < rtd->hw_chan; i++) {
			if (rtd->uid_cid_map[i] >= 0) {
#ifdef DMA_VER_R1P0
				sprd_dma_stop(rtd->uid_cid_map[i]);
#endif
#ifdef DMA_VER_R4P0
				sci_dma_stop(rtd->uid_cid_map[i],
					     dma->channels[i]);
#endif
			}
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t sprd_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sprd_runtime_data *rtd = runtime->private_data;
	snd_pcm_uframes_t x;
	int now_pointer;
	int bytes_of_pointer = 0;
	int shift = 1;
	int sel_max = 0;
#ifdef CONFIG_SPRD_VBC_INTERLEAVED
	if (rtd->interleaved)
		shift = 0;
#endif

	if (rtd->uid_cid_map[0] >= 0) {
		now_pointer = sprd_pcm_dma_get_addr(rtd->uid_cid_map[0],
						    substream) -
		    runtime->dma_addr;
		bytes_of_pointer = now_pointer;
	}
	if (rtd->uid_cid_map[1] >= 0) {
		now_pointer = sprd_pcm_dma_get_addr(rtd->uid_cid_map[1],
						    substream) -
		    runtime->dma_addr - rtd->dma_addr_offset;
		if (!bytes_of_pointer) {
			bytes_of_pointer = now_pointer;
		} else {
			sel_max = (bytes_of_pointer < rtd->dma_pos_pre[0]);
			sel_max ^= (now_pointer < rtd->dma_pos_pre[1]);
			rtd->dma_pos_pre[0] = bytes_of_pointer;
			rtd->dma_pos_pre[1] = now_pointer;
			if (sel_max) {
				bytes_of_pointer =
				    max(bytes_of_pointer, now_pointer) << shift;
			} else {
				bytes_of_pointer =
				    min(bytes_of_pointer, now_pointer) << shift;
			}
		}
	}

	x = bytes_to_frames(runtime, bytes_of_pointer);

	if (x == runtime->buffer_size)
		x = 0;

#if 0
	sprd_pcm_dbg("p=%d f=%d\n", bytes_of_pointer, x);
#endif

	return x;
}

static int sprd_pcm_mmap(struct snd_pcm_substream *substream,
			 struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

#ifndef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr, runtime->dma_bytes);
#else
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start,
			       runtime->dma_addr >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start, vma->vm_page_prot);
#endif
}

static struct snd_pcm_ops sprd_pcm_ops = {
	.open = sprd_pcm_open,
	.close = sprd_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = sprd_pcm_hw_params,
	.hw_free = sprd_pcm_hw_free,
	.prepare = sprd_pcm_prepare,
	.trigger = sprd_pcm_trigger,
	.pointer = sprd_pcm_pointer,
	.mmap = sprd_pcm_mmap,
};

static int sprd_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = AUDIO_BUFFER_BYTES_MAX;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	if (sprd_is_i2s(rtd->cpu_dai)) {
		size = I2S_BUFFER_BYTES_MAX;
	} else {
		size = VBC_BUFFER_BYTES_MAX;
	}
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	if (sprd_is_i2s(rtd->cpu_dai)
	    || !((substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		 && 0 == sprd_buffer_iram_backup())) {
#endif
		buf->private_data = NULL;
		buf->area = dma_alloc_writecombine(pcm->card->dev, size,
						   &buf->addr, GFP_KERNEL);
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
	} else {
		buf->private_data = buf;
		buf->area = (void *)(s_iram_remap_base);
		buf->addr = SPRD_IRAM_ALL_PHYS;
		size = SPRD_IRAM_ALL_SIZE - (2 * SPRD_AUDIO_DMA_NODE_SIZE);
	}
#endif
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static u64 sprd_pcm_dmamask = DMA_BIT_MASK(32);
static struct snd_dma_buffer *save_p_buf = 0;
static struct snd_dma_buffer *save_c_buf = 0;
static struct snd_dma_buffer *save_c23_buf = 0;

static int sprd_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_pcm_substream *substream;
	int ret = 0;

	sprd_pcm_dbg("Entering %s %s\n", __func__, sprd_dai_pcm_name(cpu_dai));

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &sprd_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	if (substream) {
		struct snd_dma_buffer *buf = &substream->dma_buffer;
		if (sprd_is_i2s(cpu_dai) || !save_p_buf) {
			ret = sprd_pcm_preallocate_dma_buffer(pcm,
							      SNDRV_PCM_STREAM_PLAYBACK);
			if (ret)
				goto out;
			if (!sprd_is_i2s(cpu_dai)) {
				save_p_buf = buf;
			}
			sprd_pcm_dbg("playback alloc memery\n");
		} else {
			memcpy(buf, save_p_buf, sizeof(*buf));
			sprd_pcm_dbg("playback share memery\n");
		}
	}

	substream = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	if (substream) {
		struct snd_dma_buffer *buf = &substream->dma_buffer;
		if (sprd_is_i2s(cpu_dai)) {
			ret = sprd_pcm_preallocate_dma_buffer(pcm,
								  SNDRV_PCM_STREAM_CAPTURE);
			if (ret)
				goto out;
		} else if (!strcmp(rtd->cpu_dai->name, "vbc")  || !strcmp(rtd->cpu_dai->name, "vaudio")) {
			 if (!save_c_buf) {
				ret = sprd_pcm_preallocate_dma_buffer(pcm,
								      SNDRV_PCM_STREAM_CAPTURE);
				if (ret)
					goto out;
				save_c_buf = buf;
				sprd_pcm_dbg("capture ad01 alloc memery\n");
			} else {
				memcpy(buf, save_c_buf, sizeof(*buf));
				sprd_pcm_dbg("capture ad01 share memery\n");
			}
		} else if (!strcmp(rtd->cpu_dai->name, "vbc-ad23")  || !strcmp(rtd->cpu_dai->name, "vaudio-ad23")) {
			if ( !save_c23_buf) {
				ret = sprd_pcm_preallocate_dma_buffer(pcm,
								      SNDRV_PCM_STREAM_CAPTURE);
				if (ret)
					goto out;
				save_c23_buf = buf;
				sprd_pcm_dbg("capture ad23 alloc memery\n");
			} else {
				memcpy(buf, save_c23_buf, sizeof(*buf));
				sprd_pcm_dbg("capture ad23 share memery\n");
			}
		}
	}
out:
	sprd_pcm_dbg("return %i\n", ret);
	sprd_pcm_dbg("Leaving %s\n", __func__);
	return ret;
}

static void sprd_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	sprd_pcm_dbg("Entering %s\n", __func__);

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
		if (buf->private_data)
			sprd_buffer_iram_restore();
		else
#endif
			dma_free_writecombine(pcm->card->dev, buf->bytes,
					      buf->area, buf->addr);
		buf->area = NULL;
		if (buf == save_p_buf) {
			save_p_buf = 0;
		}
		if (buf == save_c_buf) {
			save_c_buf = 0;
		}
		if (buf == save_c23_buf) {
			save_c23_buf = 0;
		}
	}
	sprd_pcm_dbg("Leaving %s\n", __func__);
}

static struct snd_soc_platform_driver sprd_soc_platform = {
	.ops = &sprd_pcm_ops,
	.pcm_new = sprd_pcm_new,
	.pcm_free = sprd_pcm_free_dma_buffers,
};

static int __devinit sprd_soc_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &sprd_soc_platform);
}

static int __devexit sprd_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver sprd_pcm_driver = {
	.driver = {
		   .name = "sprd-pcm-audio",
		   .owner = THIS_MODULE,
		   },

	.probe = sprd_soc_platform_probe,
	.remove = __devexit_p(sprd_soc_platform_remove),
};

static int __init snd_sprd_pcm_init(void)
{
	return platform_driver_register(&sprd_pcm_driver);
}

static void __exit snd_sprd_pcm_exit(void)
{
	platform_driver_unregister(&sprd_pcm_driver);
}

module_init(snd_sprd_pcm_init);
module_exit(snd_sprd_pcm_exit);

MODULE_DESCRIPTION("SPRD ASoC PCM DMA");
MODULE_AUTHOR("Ken Kuang <ken.kuang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sprd-audio");
