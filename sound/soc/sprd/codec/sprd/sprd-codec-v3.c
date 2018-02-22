/*
 * sound/soc/sprd/codec/sprd/sprd-codec-v3.c
 *
 * SPRD-CODEC -- SpreadTrum Tiger intergrated codec.
 *
 * Copyright (C) 2013 SpreadTrum Ltd.
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
#define pr_fmt(fmt) "[audio:codec] " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/atomic.h>
#include <linux/regulator/consumer.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/power_supply.h>

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include <mach/sprd-audio.h>
#include <mach/globalregs.h>
#include "sprd-codec-v3.h"

#ifdef CONFIG_SPRD_AUDIO_DEBUG
#define sprd_codec_dbg pr_debug
#define sprd_bug_on BUG_ON
#else
#define sprd_codec_dbg(...)
#define sprd_bug_on(...)
#endif

#if  CONFIG_CODEC_SRC_SAMPLE_RATE != 32000 && CONFIG_CODEC_SRC_SAMPLE_RATE != 48000
#error "CODEC_SRC_SAMPLE_RATE value not support, you must set 32000 or 48000"
#endif

#define SOC_REG(r) ((unsigned short)(r))
#define FUN_REG(f) ((unsigned short)(-((f) + 1)))
#define ID_FUN(id, lr) ((unsigned short)(((id) << 1) | (lr)))

#define SPRD_CODEC_AP_BASE_HI (SPRD_CODEC_AP_BASE & 0xFFFF0000)
#define SPRD_CODEC_DP_BASE_HI (SPRD_CODEC_DP_BASE & 0xFFFF0000)

#define SPRD_CODEC_NUM_SUPPLIES 8
static const char *sprd_codec_supply_names[SPRD_CODEC_NUM_SUPPLIES] = {
	"audio_auxmicbias",	
	"audio_micbias",	
	"audio_vbo",		
	"audio_vcmbuf",		
	"audio_vcm",		
	"audio_bg_ibias",	
	"audio_bg",		
	"audio_vb",		
};

enum {
	SPRD_CODEC_PGA_SPKL = 0,
	SPRD_CODEC_PGA_SPKR,
	SPRD_CODEC_PGA_HPL,
	SPRD_CODEC_PGA_HPR,
	SPRD_CODEC_PGA_EAR,
	SPRD_CODEC_PGA_ADCL,
	SPRD_CODEC_PGA_ADCR,
	SPRD_CODEC_PGA_DACL,
	SPRD_CODEC_PGA_DACR,
	SPRD_CODEC_PGA_MIC,
	SPRD_CODEC_PGA_AUXMIC,
	SPRD_CODEC_PGA_HEADMIC,
	SPRD_CODEC_PGA_AIL,
	SPRD_CODEC_PGA_AIR,

	SPRD_CODEC_PGA_NUM
};

const char *sprd_codec_pga_debug_str[SPRD_CODEC_PGA_NUM] = {
	"SPKL",
	"SPKR",
	"HPL",
	"HPR",
	"EAR",
	"ADCL",
	"ADCR",
	"DACL",
	"DACR",
	"MIC",
	"AUXMIC",
	"HEADMIC",
	"AIL",
	"AIR",
};

typedef int (*sprd_codec_pga_set) (struct snd_soc_codec * codec, int pgaval);

struct sprd_codec_pga {
	sprd_codec_pga_set set;
	int min;
};

struct sprd_codec_pga_op {
	int pgaval;
	sprd_codec_pga_set set;
};

enum {
	SPRD_CODEC_LEFT = 0,
	SPRD_CODEC_RIGHT = 1,
};

enum {
	SPRD_CODEC_MIXER_START = 0,
	SPRD_CODEC_AIL = SPRD_CODEC_MIXER_START,
	SPRD_CODEC_AIR,
	SPRD_CODEC_MAIN_MIC,
	SPRD_CODEC_AUX_MIC,
	SPRD_CODEC_HP_MIC,
	SPRD_CODEC_ADC_MIXER_MAX,
	SPRD_CODEC_HP_DACL = SPRD_CODEC_ADC_MIXER_MAX,
	SPRD_CODEC_HP_DACR,
	SPRD_CODEC_HP_ADCL,
	SPRD_CODEC_HP_ADCR,
	SPRD_CODEC_HP_MIXER_MAX,
	SPRD_CODEC_SPK_DACL = SPRD_CODEC_HP_MIXER_MAX,
	SPRD_CODEC_SPK_DACR,
	SPRD_CODEC_SPK_ADCL,
	SPRD_CODEC_SPK_ADCR,
	SPRD_CODEC_SPK_MIXER_MAX,
	SPRD_CODEC_EAR_DACL = SPRD_CODEC_SPK_MIXER_MAX,
	SPRD_CODEC_EAR_MIXER_MAX,

	SPRD_CODEC_MIXER_MAX = SPRD_CODEC_EAR_MIXER_MAX << SPRD_CODEC_RIGHT
};

const char *sprd_codec_mixer_debug_str[SPRD_CODEC_MIXER_MAX] = {
	"AIL->ADCL",
	"AIL->ADCR",
	"AIR->ADCL",
	"AIR->ADCR",
	"MAIN MIC->ADCL",
	"MAIN MIC->ADCR",
	"AUX MIC->ADCL",
	"AUX MIC->ADCR",
	"HP MIC->ADCL",
	"HP MIC->ADCR",
	"DACL->HPL",
	"DACL->HPR",
	"DACR->HPL",
	"DACR->HPR",
	"ADCL->HPL",
	"ADCL->HPR",
	"ADCR->HPL",
	"ADCR->HPR",
	"DACL->SPKL",
	"DACL->SPKR",
	"DACR->SPKL",
	"DACR->SPKR",
	"ADCL->SPKL",
	"ADCL->SPKR",
	"ADCR->SPKL",
	"ADCR->SPKR",
	"DACL->EAR",
	"DACR->EAR(bug)"
};

#define IS_SPRD_CODEC_MIXER_RANG(reg) (((reg) >= SPRD_CODEC_MIXER_START) && ((reg) <= SPRD_CODEC_MIXER_MAX))

typedef int (*sprd_codec_mixer_set) (struct snd_soc_codec * codec, int on);
struct sprd_codec_mixer {
	int on;
	sprd_codec_mixer_set set;
};


struct sprd_codec_ldo_v_map {
	int ldo_v_level;
	int volt;
};

const static struct sprd_codec_ldo_v_map ldo_v_map[] = {
	
	{LDO_V_29, 3600},
	{LDO_V_31, 3700},
	{LDO_V_32, 3800},
	{LDO_V_33, 3900},
	{LDO_V_34, 4000},
	{LDO_V_35, 4100},
	{LDO_V_36, 4200},
	{LDO_V_38, 4300},
};

struct sprd_codec_inter_pa {
	
	int LDO_V_sel:4;
	int DTRI_F_sel:4;
	int is_DEMI_mode:1;
	int is_classD_mode:1;
	int is_LDO_mode:1;
	int is_auto_LDO_mode:1;
	int RESV:20;
};

struct sprd_codec_pa_setting {
	union {
		struct sprd_codec_inter_pa setting;
		u32 value;
	};
	int set;
};

static DEFINE_MUTEX(inter_pa_mutex);
static struct sprd_codec_pa_setting inter_pa;

struct sprd_codec_inter_hp_pa {
	
	int class_g_osc:2;
	int class_g_mode:2;
	int class_g_low_power:2;
	int RESV:26;
};

struct sprd_codec_hp_pa_setting {
	union {
		struct sprd_codec_inter_hp_pa setting;
		u32 value;
	};
	int set;
};

static DEFINE_MUTEX(inter_hp_pa_mutex);
static struct sprd_codec_hp_pa_setting inter_hp_pa;

enum {
	SPRD_CODEC_MIC_BIAS,
	SPRD_CODEC_AUXMIC_BIAS,
	SPRD_CODEC_HEADMIC_BIAS,
	SPRD_CODEC_MIC_BIAS_MAX
};

static const char *mic_bias_name[SPRD_CODEC_MIC_BIAS_MAX] = {
	"Mic Bias",
	"AuxMic Bias",
	"HeadMic Bias",
};

struct sprd_codec_priv {
	struct snd_soc_codec *codec;
	atomic_t power_refcount;
	int da_sample_val;
	int ad_sample_val;
	int ad1_sample_val;
	struct sprd_codec_mixer mixer[SPRD_CODEC_MIXER_MAX];
	struct sprd_codec_pga_op pga[SPRD_CODEC_PGA_NUM];
	int mic_bias[SPRD_CODEC_MIC_BIAS_MAX];
#ifdef CONFIG_SPRD_CODEC_USE_INT
	int ap_irq;
	struct completion completion_hp_pop;

	int dp_irq;
	struct completion completion_dac_mute;
#endif
	struct power_supply audio_ldo;
};

static struct sprd_codec_power_suppliy {
	struct regulator_bulk_data supplies[SPRD_CODEC_NUM_SUPPLIES];
	struct delayed_work mic_delayed_work;
	struct delayed_work auxmic_delayed_work;
	struct delayed_work headmic_delayed_work;
	atomic_t mic_on;
	atomic_t auxmic_on;
	atomic_t headmic_on;
	atomic_t ldo_refcount;
	int audio_ldo_open_ok;
} sprd_codec_power;

#define SPRD_CODEC_PA_SW_AOL (BIT(0))
#define SPRD_CODEC_PA_SW_EAR (BIT(1))
#define SPRD_CODEC_PA_SW_FUN (SPRD_CODEC_PA_SW_AOL | SPRD_CODEC_PA_SW_EAR)
static int sprd_codec_fun = 0;
static DEFINE_SPINLOCK(sprd_codec_fun_lock);

static void sprd_codec_set_fun(int fun)
{
	spin_lock(&sprd_codec_fun_lock);
	sprd_codec_fun |= fun;
	spin_unlock(&sprd_codec_fun_lock);
}

static void sprd_codec_clr_fun(int fun)
{
	spin_lock(&sprd_codec_fun_lock);
	sprd_codec_fun &= ~fun;
	spin_unlock(&sprd_codec_fun_lock);
}

static int sprd_codec_test_fun(int fun)
{
	int ret;
	spin_lock(&sprd_codec_fun_lock);
	ret = sprd_codec_fun & fun;
	spin_unlock(&sprd_codec_fun_lock);
	return ret;
}

static void sprd_codec_wait(u32 wait_time)
{
	if (wait_time)
		schedule_timeout_uninterruptible(msecs_to_jiffies(wait_time));
}

#if 0				
static unsigned int sprd_codec_read(struct snd_soc_codec *codec,
				    unsigned int reg);
static void sprd_codec_print_regs(struct snd_soc_codec *codec)
{
	int reg;
	pr_warn("sprd_codec register digital part\n");
	for (reg = SPRD_CODEC_DP_BASE; reg < SPRD_CODEC_DP_END; reg += 0x10) {
		pr_warn("0x%04x | 0x%04x 0x%04x 0x%04x 0x%04x\n",
			(reg - SPRD_CODEC_DP_BASE)
			, sprd_codec_read(codec, reg + 0x00)
			, sprd_codec_read(codec, reg + 0x04)
			, sprd_codec_read(codec, reg + 0x08)
			, sprd_codec_read(codec, reg + 0x0C)
		    );
	}
	pr_warn("sprd_codec register analog part\n");
	for (reg = SPRD_CODEC_AP_BASE; reg < SPRD_CODEC_AP_END; reg += 0x10) {
		pr_warn("0x%04x | 0x%04x 0x%04x 0x%04x 0x%04x\n",
			(reg - SPRD_CODEC_AP_BASE)
			, sprd_codec_read(codec, reg + 0x00)
			, sprd_codec_read(codec, reg + 0x04)
			, sprd_codec_read(codec, reg + 0x08)
			, sprd_codec_read(codec, reg + 0x0C)
		    );
	}
}
#endif

static inline int _sprd_codec_hold(int index, int volt)
{
	int scope = 40;		
	int ret = ((ldo_v_map[index].volt - volt) <= scope);
	if (index >= 1) {
		return (ret || ((volt - ldo_v_map[index - 1].volt) <= scope));
	}
	return ret;
}

static int sprd_codec_auto_ldo_volt(void (*set_level) (int), int init)
{
	int i;
	
	
	int volt = 3800;
	
	sprd_codec_dbg("Entering %s get %d\n", __func__, volt);
	for (i = 0; i < ARRAY_SIZE(ldo_v_map); i++) {
		if (volt <= ldo_v_map[i].volt) {
			sprd_codec_dbg("hold %d\n", _sprd_codec_hold(i, volt));
			if (init || !_sprd_codec_hold(i, volt)) {
				set_level(ldo_v_map[i].ldo_v_level);
			}
			return 0;
		}
	}
	return -EFAULT;
}

static inline void sprd_codec_vcm_v_sel(int v_sel)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, v_sel);
	mask = VCM_V_MASK << VCM_V;
	val = (v_sel << VCM_V) & mask;
	arch_audio_codec_write_mask(PMUR4_PMUR3, val, mask);
}

static inline void sprd_codec_auxadc_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(AUXADC_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(PMUR4_PMUR3, val, mask);
}

static int sprd_codec_pga_spk_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = DCGR3;
	val = (pgaval & 0xF) << 4;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0xF0, val);
}

static int sprd_codec_pga_spkr_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = DCGR3;
	val = pgaval & 0xF;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0x0F, val);
}

static int sprd_codec_pga_hpl_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = DCGR2_DCGR1;
	val = (pgaval & 0xF) << 4;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0xF0, val);
}

static int sprd_codec_pga_hpr_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = DCGR2_DCGR1;
	val = pgaval & 0xF;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0x0F, val);
}

static int sprd_codec_pga_ear_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = DCGR2_DCGR1;
	val = ((pgaval & 0xF) << 12);
	return snd_soc_update_bits(codec, SOC_REG(reg), 0xF000, val);
}

static int sprd_codec_pga_adcl_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = ACGR3_ACGR2;
	val = pgaval & 0x3F;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0x3F, val);
}

static int sprd_codec_pga_adcr_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = ACGR3_ACGR2;
	val = (pgaval & 0x3F) << 8;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0x3F00, val);
}

static int sprd_codec_pga_dacl_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = DACGR_DACR;
	val = (pgaval & 0x07) << 12;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0x7000, val);
}

static int sprd_codec_pga_dacr_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = DACGR_DACR;
	val = (pgaval & 0x07) << 8;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0x0700, val);
}

static int sprd_codec_pga_mic_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = ACGR1;
	val = (pgaval & 0x03) << 6;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0xC0, val);
}

static int sprd_codec_pga_auxmic_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = ACGR1;
	val = (pgaval & 0x03) << 4;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0x30, val);
}

static int sprd_codec_pga_headmic_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = ACGR1;
	val = (pgaval & 0x03) << 2;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0x0C, val);
}

static int sprd_codec_pga_ailr_set(struct snd_soc_codec *codec, int pgaval)
{
	int reg, val;
	reg = ACGR1;
	val = pgaval & 0x03;
	return snd_soc_update_bits(codec, SOC_REG(reg), 0x03, val);
}

static struct sprd_codec_pga sprd_codec_pga_cfg[SPRD_CODEC_PGA_NUM] = {
	{sprd_codec_pga_spk_set, 0},
	{sprd_codec_pga_spkr_set, 0},
	{sprd_codec_pga_hpl_set, 0},
	{sprd_codec_pga_hpr_set, 0},
	{sprd_codec_pga_ear_set, 0},

	{sprd_codec_pga_adcl_set, 0},
	{sprd_codec_pga_adcr_set, 0},

	{sprd_codec_pga_dacl_set, 0},
	{sprd_codec_pga_dacr_set, 0},
	{sprd_codec_pga_mic_set, 0},
	{sprd_codec_pga_auxmic_set, 0},
	{sprd_codec_pga_headmic_set, 0},
	{sprd_codec_pga_ailr_set, 0},
	{sprd_codec_pga_ailr_set, 0},
};


static int ailadcl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(AAICR2_AAICR1), BIT(AIL_ADCL),
				   on << AIL_ADCL);
}

static int ailadcr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(AAICR2_AAICR1), BIT(AIL_ADCR),
				   on << AIL_ADCR);
}

static int airadcl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(AAICR2_AAICR1), BIT(AIR_ADCL),
				   on << AIR_ADCL);
}

static int airadcr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(AAICR2_AAICR1), BIT(AIR_ADCR),
				   on << AIR_ADCR);
}

static int mainmicadcl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(AAICR2_AAICR1), BIT(MIC_ADCL),
				   on << MIC_ADCL);
}

static int mainmicadcr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(AAICR2_AAICR1), BIT(MIC_ADCR),
				   on << MIC_ADCR);
}

static int auxmicadcl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(AAICR2_AAICR1),
				   BIT(AUXMIC_ADCL), on << AUXMIC_ADCL);
}

static int auxmicadcr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(AAICR2_AAICR1),
				   BIT(AUXMIC_ADCR), on << AUXMIC_ADCR);
}

static int hpmicadcl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(AAICR2_AAICR1),
				   BIT(HEADMIC_ADCL), on << HEADMIC_ADCL);
}

static int hpmicadcr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(AAICR2_AAICR1),
				   BIT(HEADMIC_ADCR), on << HEADMIC_ADCR);
}


static int daclhpl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR3_DAOCR1),
				   BIT(DACL_P_HPL), on << DACL_P_HPL);
}

static int daclhpr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR3_DAOCR1),
				   BIT(DACL_N_HPR), on << DACL_N_HPR);
}

static int dacrhpl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR3_DAOCR1),
				   BIT(DACR_P_HPL), on << DACR_P_HPL);
}

static int dacrhpr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR3_DAOCR1),
				   BIT(DACR_P_HPR), on << DACR_P_HPR);
}

static int adclhpl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR3_DAOCR1),
				   BIT(ADCL_P_HPL), on << ADCL_P_HPL);
}

static int adclhpr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR3_DAOCR1),
				   BIT(ADCL_N_HPR), on << ADCL_N_HPR);
}

static int adcrhpl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR3_DAOCR1),
				   BIT(ADCR_P_HPL), on << ADCR_P_HPL);
}

static int adcrhpr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR3_DAOCR1),
				   BIT(ADCR_P_HPR), on << ADCR_P_HPR);
}


static int daclspkl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR2), BIT(DACL_AOL),
				   on << DACL_AOL);
}

static int dacrspkl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR2), BIT(DACR_AOL),
				   on << DACR_AOL);
}

static int adclspkl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR2), BIT(ADCL_AOL),
				   on << ADCL_AOL);
}

static int adcrspkl_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR2), BIT(ADCR_AOL),
				   on << ADCR_AOL);
}


static int daclspkr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR2), BIT(DACL_AOR),
				   on << DACL_AOR);
}

static int dacrspkr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR2), BIT(DACR_AOR),
				   on << DACR_AOR);
}

static int adclspkr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR2), BIT(ADCL_AOR),
				   on << ADCL_AOR);
}

static int adcrspkr_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR2), BIT(ADCR_AOR),
				   on << ADCR_AOR);
}


static int daclear_set(struct snd_soc_codec *codec, int on)
{
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	return snd_soc_update_bits(codec, SOC_REG(DAOCR3_DAOCR1), BIT(DACL_EAR),
				   on << DACL_EAR);
}

static sprd_codec_mixer_set mixer_setting[SPRD_CODEC_MIXER_MAX] = {
	
	ailadcl_set, ailadcr_set,
	airadcl_set, airadcr_set,
	mainmicadcl_set, mainmicadcr_set,
	auxmicadcl_set, auxmicadcr_set,
	hpmicadcl_set, hpmicadcr_set,
	
	daclhpl_set, daclhpr_set,
	dacrhpl_set, dacrhpr_set,
	adclhpl_set, adclhpr_set,
	adcrhpl_set, adcrhpr_set,
	
	daclspkl_set, daclspkr_set,
	dacrspkl_set, dacrspkr_set,
	adclspkl_set, adclspkr_set,
	adcrspkl_set, adcrspkr_set,
	
	daclear_set, 0,
};

static inline void __sprd_codec_pa_sw_en(int on)
{
	int mask;
	int val;
	mask = BIT(PA_SW_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(PMUR2_PMUR1, val, mask);
}

static DEFINE_SPINLOCK(sprd_codec_pa_sw_lock);
static inline void sprd_codec_pa_sw_set(int fun)
{
	sprd_codec_dbg("Entering %s fun 0x%08x\n", __func__, fun);
	spin_lock(&sprd_codec_pa_sw_lock);
	sprd_codec_set_fun(fun);
	__sprd_codec_pa_sw_en(1);
	spin_unlock(&sprd_codec_pa_sw_lock);
}

static inline void sprd_codec_pa_sw_clr(int fun)
{
	sprd_codec_dbg("Entering %s fun 0x%08x\n", __func__, fun);
	spin_lock(&sprd_codec_pa_sw_lock);
	sprd_codec_clr_fun(fun);
	if (!sprd_codec_test_fun(SPRD_CODEC_PA_SW_FUN))
		__sprd_codec_pa_sw_en(0);
	spin_unlock(&sprd_codec_pa_sw_lock);
}


static inline void sprd_codec_pa_d_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(PA_D_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(DCR2_DCR1, val, mask);
}

static inline void sprd_codec_pa_demi_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(PA_DEMI_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(DCR2_DCR1, val, mask);

	mask = BIT(DRV_OCP_AOL_PD) | BIT(DRV_OCP_AOR_PD);
	val = mask;
	arch_audio_codec_write_mask(DCR4_DCR3, val, mask);
}

static inline void sprd_codec_pa_ldo_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(PA_LDO_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(PMUR2_PMUR1, val, mask);
	if (on) {
		sprd_codec_pa_sw_clr(SPRD_CODEC_PA_SW_AOL);
	}
}

static inline void sprd_codec_pa_ldo_v_sel(int v_sel)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, v_sel);
	mask = PA_LDO_V_MASK << PA_LDO_V;
	val = (v_sel << PA_LDO_V) & mask;
	arch_audio_codec_write_mask(PMUR6_PMUR5, val, mask);
}

static inline void sprd_codec_pa_dtri_f_sel(int f_sel)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, f_sel);
	mask = PA_DTRI_F_MASK << PA_DTRI_F;
	val = (f_sel << PA_DTRI_F) & mask;
	arch_audio_codec_write_mask(DCR2_DCR1, val, mask);
}

static inline void sprd_codec_pa_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	spin_lock(&sprd_codec_pa_sw_lock);
	if (on) {
		mask = BIT(PA_EN);
		val = mask;
	} else {
		if (!sprd_codec_test_fun(SPRD_CODEC_PA_SW_FUN))
			mask = BIT(PA_EN) | BIT(PA_SW_EN) | BIT(PA_LDO_EN);
		else
			mask = BIT(PA_EN) | BIT(PA_LDO_EN);
		val = 0;
	}
	arch_audio_codec_write_mask(PMUR2_PMUR1, val, mask);
	spin_unlock(&sprd_codec_pa_sw_lock);
}

static inline void sprd_codec_inter_pa_init(void)
{
	inter_pa.setting.LDO_V_sel = 0x03;
	inter_pa.setting.DTRI_F_sel = 0x01;
}

int sprd_inter_speaker_pa(int on)
{
	pr_info("inter PA switch %s\n", on ? "ON" : "OFF");
	mutex_lock(&inter_pa_mutex);
	if (on) {
		sprd_codec_pa_d_en(inter_pa.setting.is_classD_mode);
		sprd_codec_pa_demi_en(inter_pa.setting.is_DEMI_mode);
		sprd_codec_pa_ldo_en(inter_pa.setting.is_LDO_mode);
		if (inter_pa.setting.is_LDO_mode) {
			if (inter_pa.setting.is_auto_LDO_mode) {
				sprd_codec_auto_ldo_volt
				    (sprd_codec_pa_ldo_v_sel, 1);
			} else {
				sprd_codec_pa_ldo_v_sel(inter_pa.setting.
							LDO_V_sel);
			}
		}
		sprd_codec_pa_dtri_f_sel(inter_pa.setting.DTRI_F_sel);
		sprd_codec_pa_en(1);
		inter_pa.set = 1;
	} else {
		inter_pa.set = 0;
		sprd_codec_pa_en(0);
		sprd_codec_pa_ldo_en(0);
	}
	mutex_unlock(&inter_pa_mutex);
	return 0;
}

EXPORT_SYMBOL(sprd_inter_speaker_pa);

static inline void sprd_codec_hp_pa_lpw(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(AUDIO_CHP_LPW);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(DCR8_DCR7, val, mask);
}

static inline void sprd_codec_hp_pa_mode(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(AUDIO_CHP_MODE);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(DCR8_DCR7, val, mask);
}

static inline void sprd_codec_hp_pa_osc(int osc)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, osc);
	mask = AUDIO_CHP_OSC_MASK << AUDIO_CHP_OSC;
	val = (osc << AUDIO_CHP_OSC) & mask;
	arch_audio_codec_write_mask(DCR8_DCR7, val, mask);
}

static inline void sprd_codec_hp_pa_ref_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(AUDIO_CHP_REF_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(DCR8_DCR7, val, mask);
}

static inline void sprd_codec_hp_pa_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(AUDIO_CHP_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(DCR8_DCR7, val, mask);
}

static inline void sprd_codec_hp_pa_hpl_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(AUDIO_CHP_HPL_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(DCR8_DCR7, val, mask);
}

static inline void sprd_codec_hp_pa_hpr_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(AUDIO_CHP_HPR_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(DCR8_DCR7, val, mask);
}

static inline void sprd_codec_hp_pa_hpl_mute(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(AUDIO_CHP_LMUTE);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(DCR8_DCR7, val, mask);
}

static inline void sprd_codec_hp_pa_hpr_mute(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(AUDIO_CHP_RMUTE);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(DCR8_DCR7, val, mask);
}

static inline void sprd_codec_inter_hp_pa_init(void)
{
	inter_hp_pa.setting.class_g_osc = 0x01;
}

int sprd_inter_headphone_pa(int on)
{
	static struct regulator *regulator = 0;
	pr_info("inter HP PA switch %s\n", on ? "ON" : "OFF");
	mutex_lock(&inter_hp_pa_mutex);
	if (on) {
		if (!regulator) {
			regulator = regulator_get(0, CLASS_G_LDO_ID);
			if (IS_ERR(regulator)) {
				pr_err("Failed to request %ld: %s\n",
				       PTR_ERR(regulator), CLASS_G_LDO_ID);
				BUG_ON(1);
			}
			regulator_set_mode(regulator, REGULATOR_MODE_STANDBY);
			regulator_enable(regulator);
		}
		sprd_codec_auxadc_en(1);
		sprd_codec_hp_pa_lpw(inter_hp_pa.setting.class_g_low_power);
		sprd_codec_hp_pa_mode(inter_hp_pa.setting.class_g_mode);
		sprd_codec_hp_pa_osc(inter_hp_pa.setting.class_g_osc);
		sprd_codec_hp_pa_hpl_en(1);
		sprd_codec_hp_pa_hpr_en(1);
		sprd_codec_hp_pa_ref_en(1);
		sprd_codec_hp_pa_en(1);
		inter_hp_pa.set = 1;
	} else {
		inter_hp_pa.set = 0;
		sprd_codec_hp_pa_en(0);
		sprd_codec_hp_pa_ref_en(0);
		sprd_codec_hp_pa_hpl_en(0);
		sprd_codec_hp_pa_hpr_en(0);
		sprd_codec_auxadc_en(0);
		if (regulator) {
			regulator_set_mode(regulator, REGULATOR_MODE_NORMAL);
			regulator_disable(regulator);
			regulator_put(regulator);
			regulator = 0;
		}
	}
	mutex_unlock(&inter_hp_pa_mutex);
	return 0;
}

EXPORT_SYMBOL(sprd_inter_headphone_pa);


static inline void sprd_codec_mic_bias_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(MICBIAS_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(PMUR4_PMUR3, val, mask);
}

static inline void sprd_codec_auxmic_bias_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(AUXMICBIAS_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(PMUR4_PMUR3, val, mask);
}

static inline void sprd_codec_headmic_bias_en(int on)
{
	int mask;
	int val;
	sprd_codec_dbg("Entering %s set %d\n", __func__, on);
	mask = BIT(HEADMICBIAS_EN);
	val = on ? mask : 0;
	arch_audio_codec_write_mask(PMUR2_PMUR1, val, mask);
}

static int sprd_codec_set_sample_rate(struct snd_soc_codec *codec, int rate,
				      int mask, int shift)
{
	switch (rate) {
	case 8000:
		snd_soc_update_bits(codec, SOC_REG(AUD_DAC_CTL), mask,
				    SPRD_CODEC_RATE_8000 << shift);
		break;
	case 11025:
		snd_soc_update_bits(codec, SOC_REG(AUD_DAC_CTL), mask,
				    SPRD_CODEC_RATE_11025 << shift);
		break;
	case 16000:
		snd_soc_update_bits(codec, SOC_REG(AUD_DAC_CTL), mask,
				    SPRD_CODEC_RATE_16000 << shift);
		break;
	case 22050:
		snd_soc_update_bits(codec, SOC_REG(AUD_DAC_CTL), mask,
				    SPRD_CODEC_RATE_22050 << shift);
		break;
	case 32000:
		snd_soc_update_bits(codec, SOC_REG(AUD_DAC_CTL), mask,
				    SPRD_CODEC_RATE_32000 << shift);
		break;
	case 44100:
		snd_soc_update_bits(codec, SOC_REG(AUD_DAC_CTL), mask,
				    SPRD_CODEC_RATE_44100 << shift);
		break;
	case 48000:
		snd_soc_update_bits(codec, SOC_REG(AUD_DAC_CTL), mask,
				    SPRD_CODEC_RATE_48000 << shift);
		break;
	case 96000:
		snd_soc_update_bits(codec, SOC_REG(AUD_DAC_CTL), mask,
				    SPRD_CODEC_RATE_96000 << shift);
		break;
	default:
		pr_err("sprd_codec not supports rate %d\n", rate);
		break;
	}
	sprd_codec_dbg("set playback rate 0x%x\n", snd_soc_read(codec, AUD_DAC_CTL));
	return 0;
}

void sprd_codec_set_da_sample_rate(struct snd_soc_codec *codec, int rate)
{
	sprd_codec_set_sample_rate(codec, rate,  0x0F, 0);
}
EXPORT_SYMBOL(sprd_codec_set_da_sample_rate);

static int sprd_codec_set_ad_sample_rate(struct snd_soc_codec *codec, int rate,
					 int mask, int shift)
{
	int set;
	if (rate == 44100)
		rate = CONFIG_CODEC_SRC_SAMPLE_RATE;
	set = rate / 4000;
	if (set > 13) {
		pr_err("sprd_codec not supports ad rate %d\n", rate);
	}
	snd_soc_update_bits(codec, SOC_REG(AUD_ADC_CTL), mask, set << shift);
	return 0;
}
static int sprd_codec_sample_rate_setting(struct sprd_codec_priv *sprd_codec)
{
	sprd_codec_dbg("%s ad %d da %d ad1 %d\n", __func__,
			sprd_codec->ad_sample_val, sprd_codec->da_sample_val, sprd_codec->ad1_sample_val);
	if (sprd_codec->ad_sample_val) {
		sprd_codec_set_ad_sample_rate(sprd_codec->codec,
					      sprd_codec->ad_sample_val, 0x0F,
					      0);
	}
	if (sprd_codec->ad1_sample_val) {
		sprd_codec_set_ad_sample_rate(sprd_codec->codec,
						  sprd_codec->ad1_sample_val, 0xF0,
						  4);
	}
	if (sprd_codec->da_sample_val) {
		sprd_codec_set_sample_rate(sprd_codec->codec,
					   sprd_codec->da_sample_val, 0x0F, 0);
	}
	return 0;
}

static int sprd_codec_update_bits(struct snd_soc_codec *codec,
				  unsigned short reg, unsigned int mask,
				  unsigned int value)
{
	if (!codec) {
		int rreg = reg | SPRD_CODEC_AP_BASE_HI;
		return arch_audio_codec_write_mask(rreg, value, mask);
	} else {
		return snd_soc_update_bits(codec, reg, mask, value);
	}
}

static int sprd_codec_ldo_on(struct sprd_codec_priv *sprd_codec)
{
	int i;
	int ret;
	struct snd_soc_codec *codec = 0;
	sprd_codec_dbg("Entering %s\n", __func__);

	atomic_inc(&sprd_codec_power.ldo_refcount);
	if (atomic_read(&sprd_codec_power.ldo_refcount) == 1) {
		sprd_codec_dbg("ldo on!\n");
		if (sprd_codec) {
			codec = sprd_codec->codec;
		}
		arch_audio_codec_switch(AUDIO_TO_AP_ARM_CTRL);
		arch_audio_codec_analog_reg_enable();
		arch_audio_codec_analog_enable();
		arch_audio_codec_analog_reset();
		sprd_codec_auto_ldo_volt(sprd_codec_vcm_v_sel, 1);

		for (i = 0; i < ARRAY_SIZE(sprd_codec_power.supplies); i++)
			sprd_codec_power.supplies[i].supply =
			    sprd_codec_supply_names[i];

		ret =
		    regulator_bulk_get(NULL,
				       ARRAY_SIZE(sprd_codec_power.supplies),
				       sprd_codec_power.supplies);
		if (ret != 0) {
			sprd_codec_power.audio_ldo_open_ok = 0;
			pr_err("Failed to request supplies: %d\n", ret);
		} else {
			sprd_codec_power.audio_ldo_open_ok = 1;
			for (i = 0; i < ARRAY_SIZE(sprd_codec_power.supplies);
			     i++)
				regulator_set_mode(sprd_codec_power.supplies[i].
						   consumer,
						   REGULATOR_MODE_STANDBY);
		}

		sprd_codec_update_bits(codec, SOC_REG(PMUR4_PMUR3),
				       BIT(BG_IBIAS_EN), BIT(BG_IBIAS_EN));
		sprd_codec_update_bits(codec, SOC_REG(PMUR4_PMUR3), BIT(BG_EN),
				       BIT(BG_EN));
		sprd_codec_update_bits(codec, SOC_REG(PMUR4_PMUR3), BIT(VCM_EN),
				       BIT(VCM_EN));
		sprd_codec_update_bits(codec, SOC_REG(PMUR4_PMUR3),
				       BIT(VCM_BUF_EN), BIT(VCM_BUF_EN));
		sprd_codec_update_bits(codec, SOC_REG(PMUR2_PMUR1), BIT(VB_EN),
				       BIT(VB_EN));
		sprd_codec_update_bits(codec, SOC_REG(PMUR2_PMUR1), BIT(VBO_EN),
				       BIT(VBO_EN));

		if (sprd_codec) {
			sprd_codec_wait(SPRD_CODEC_LDO_WAIT_TIME);
		}
	}

	sprd_codec_dbg("Leaving %s\n", __func__);
	return 0;
}

static int sprd_codec_ldo_off(struct sprd_codec_priv *sprd_codec)
{
	int i;
	struct snd_soc_codec *codec = 0;
	sprd_codec_dbg("Entering %s\n", __func__);

	if (atomic_dec_and_test(&sprd_codec_power.ldo_refcount)) {
		if (sprd_codec) {
			codec = sprd_codec->codec;
		}
		sprd_codec_update_bits(codec, SOC_REG(PMUR4_PMUR3), BIT(VCM_EN),
				       0);
		sprd_codec_wait(SPRD_CODEC_LDO_VCM_TIME);
		sprd_codec_update_bits(codec, SOC_REG(PMUR4_PMUR3),
				       BIT(VCM_BUF_EN), 0);
		sprd_codec_update_bits(codec, SOC_REG(PMUR2_PMUR1), BIT(VB_EN),
				       0);
		sprd_codec_update_bits(codec, SOC_REG(PMUR2_PMUR1), BIT(VBO_EN),
				       0);
		sprd_codec_update_bits(codec, SOC_REG(PMUR4_PMUR3),
				       BIT(BG_IBIAS_EN), 0);
		sprd_codec_update_bits(codec, SOC_REG(PMUR4_PMUR3), BIT(BG_EN),
				       0);

		if (sprd_codec_power.audio_ldo_open_ok) {
			for (i = 0; i < ARRAY_SIZE(sprd_codec_power.supplies);
			     i++)
				regulator_set_mode(sprd_codec_power.supplies[i].
						   consumer,
						   REGULATOR_MODE_NORMAL);

			regulator_bulk_free(ARRAY_SIZE
					    (sprd_codec_power.supplies),
					    sprd_codec_power.supplies);
		}

		arch_audio_codec_reset();
		arch_audio_codec_analog_disable();
		arch_audio_codec_analog_reg_disable();
		sprd_codec_dbg("ldo off!\n");
	}

	sprd_codec_dbg("Leaving %s\n", __func__);
	return 0;
}

static int sprd_codec_ldo_control(int on)
{
	if (on) {
		return sprd_codec_ldo_on(0);
	} else {
		return sprd_codec_ldo_off(0);
	}
}

static void sprd_codec_mic_delay_worker(struct work_struct *work)
{
	int on = atomic_read(&sprd_codec_power.mic_on) > 0;
	sprd_codec_ldo_control(on);
	sprd_codec_mic_bias_en(on);
}

static void sprd_codec_auxmic_delay_worker(struct work_struct *work)
{
	int on = atomic_read(&sprd_codec_power.auxmic_on) > 0;
	sprd_codec_ldo_control(on);
	sprd_codec_auxmic_bias_en(on);
}

static void sprd_codec_headmic_delay_worker(struct work_struct *work)
{
	int on = atomic_read(&sprd_codec_power.headmic_on) > 0;
	sprd_codec_ldo_control(on);
	sprd_codec_headmic_bias_en(on);
}

static int sprd_codec_mic_bias_inter(int on, atomic_t * v)
{
	if (on) {
		atomic_inc(v);
		return 1;
	} else {
		if (atomic_read(v) > 0) {
			atomic_dec(v);
			return 1;
		}
	}
	return 0;
}

static void sprd_codec_init_delayed_work(struct delayed_work *delayed_work,
					 work_func_t func)
{
	if (!delayed_work->work.func) {
		INIT_DELAYED_WORK(delayed_work, func);
	}
}

int sprd_codec_mic_bias_control(int on)
{
	if (sprd_codec_mic_bias_inter(on, &sprd_codec_power.mic_on)) {
		sprd_codec_init_delayed_work(&sprd_codec_power.mic_delayed_work,
					     sprd_codec_mic_delay_worker);
		schedule_delayed_work(&sprd_codec_power.mic_delayed_work,
				      msecs_to_jiffies(1));
	}
	return 0;
}

EXPORT_SYMBOL(sprd_codec_mic_bias_control);

int sprd_codec_auxmic_bias_control(int on)
{
	if (sprd_codec_mic_bias_inter(on, &sprd_codec_power.auxmic_on)) {
		sprd_codec_init_delayed_work
		    (&sprd_codec_power.auxmic_delayed_work,
		     sprd_codec_auxmic_delay_worker);
		schedule_delayed_work(&sprd_codec_power.auxmic_delayed_work,
				      msecs_to_jiffies(1));
	}
	return 0;
}

EXPORT_SYMBOL(sprd_codec_auxmic_bias_control);

int sprd_codec_headmic_bias_control(int on)
{
	if (sprd_codec_mic_bias_inter(on, &sprd_codec_power.headmic_on)) {
		sprd_codec_init_delayed_work
		    (&sprd_codec_power.headmic_delayed_work,
		     sprd_codec_headmic_delay_worker);
		schedule_delayed_work(&sprd_codec_power.headmic_delayed_work,
				      msecs_to_jiffies(1));
	}
	return 0;
}

EXPORT_SYMBOL(sprd_codec_headmic_bias_control);

static int sprd_codec_analog_open(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);

	sprd_codec_dbg("Entering %s\n", __func__);
	sprd_codec_sample_rate_setting(sprd_codec);

	
	snd_soc_update_bits(codec, SOC_REG(PMUR4_PMUR3), BIT(SEL_VCMI), BIT(SEL_VCMI));
	snd_soc_update_bits(codec, SOC_REG(PMUR4_PMUR3), BIT(VCMI_FAST_EN), BIT(VCMI_FAST_EN));

	sprd_codec_dbg("Leaving %s\n", __func__);
	return ret;
}

static int sprd_codec_digital_open(struct snd_soc_codec *codec)
{
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	sprd_codec_dbg("Entering %s\n", __func__);
	snd_soc_update_bits(codec, SOC_REG(AUD_DAC_CTL), BIT(15), 0); 
	sprd_codec_sample_rate_setting(sprd_codec);

	sprd_codec_dbg("Leaving %s\n", __func__);
	return ret;
}

static void sprd_codec_power_enable(struct snd_soc_codec *codec)
{
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	int ret;

	atomic_inc(&sprd_codec->power_refcount);
	if (atomic_read(&sprd_codec->power_refcount) == 1) {
		sprd_codec_dbg("Entering %s\n", __func__);
		ret = sprd_codec_ldo_on(sprd_codec);
		if (ret != 0)
			pr_err("sprd_codec open ldo error %d\n", ret);
		sprd_codec_analog_open(codec);
		sprd_codec_dbg("Leaving %s\n", __func__);
	}
}

static void sprd_codec_power_disable(struct snd_soc_codec *codec)
{
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);

	if (atomic_dec_and_test(&sprd_codec->power_refcount)) {
		sprd_codec_dbg("Entering %s\n", __func__);
		sprd_codec_ldo_off(sprd_codec);
		sprd_codec_dbg("Leaving %s\n", __func__);
	}
}

static const char *get_event_name(int event)
{
	const char *ev_name;
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ev_name = "PRE_PMU";
		break;
	case SND_SOC_DAPM_POST_PMU:
		ev_name = "POST_PMU";
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ev_name = "PRE_PMD";
		break;
	case SND_SOC_DAPM_POST_PMD:
		ev_name = "POST_PMD";
		break;
	default:
		BUG();
		return 0;
	}
	return ev_name;
}

static int digital_power_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	sprd_codec_dbg("Entering %s event is %s\n", __func__,
		       get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		arch_audio_codec_digital_reg_enable();
		arch_audio_codec_digital_enable();
		arch_audio_codec_digital_reset();
		sprd_codec_digital_open(w->codec);
		break;
	case SND_SOC_DAPM_POST_PMD:
		
		
		arch_audio_codec_digital_reg_disable();
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	return ret;
}

static int analog_power_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int ret = 0;

	sprd_codec_dbg("Entering %s event is %s\n", __func__,
		       get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		sprd_codec_power_enable(codec);
		break;
	case SND_SOC_DAPM_POST_PMD:
		sprd_codec_power_disable(codec);
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	return ret;
}

static int dac_event(struct snd_soc_dapm_widget *w,
		     struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;
	sprd_codec_dbg("Entering %s event is %s\n", __func__,
		       get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		pr_info("DAC ON\n");
		break;
	case SND_SOC_DAPM_POST_PMD:
		pr_info("DAC OFF\n");
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}

static int adc_event(struct snd_soc_dapm_widget *w,
		     struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;
	sprd_codec_dbg("Entering %s event is %s\n", __func__,
		       get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		pr_info("ADC ON\n");
		break;
	case SND_SOC_DAPM_POST_PMD:
		pr_info("ADC OFF\n");
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}

static int adc1_event(struct snd_soc_dapm_widget *w,
		      struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;
	sprd_codec_dbg("Entering %s event is %s\n", __func__,
		       get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		pr_info("ADC1 ON\n");
		break;
	case SND_SOC_DAPM_POST_PMD:
		pr_info("ADC1 OFF\n");
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}

static int _mixer_set_mixer(struct snd_soc_codec *codec, int id, int lr,
			    int try_on)
{
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	int reg = ID_FUN(id, lr);
	struct sprd_codec_mixer *mixer = &(sprd_codec->mixer[reg]);
	if (try_on) {
		mixer->set = mixer_setting[reg];
		return mixer->set(codec, mixer->on);
	} else {
		mixer_setting[reg] (codec, 0);
		mixer->set = 0;
	}
	return 0;
}

static inline int _mixer_setting(struct snd_soc_codec *codec, int start,
				 int end, int lr, int try_on)
{
	int id;
	for (id = start; id < end; id++) {
		_mixer_set_mixer(codec, id, lr, try_on);
	}
	return 0;
}

static inline int _mixer_setting_one(struct snd_soc_codec *codec, int id,
				     int try_on)
{
	int lr = id & 0x1;
	id >>= 1;
	return _mixer_setting(codec, id, id + 1, lr, try_on);
}

#ifdef CONFIG_SPRD_CODEC_USE_INT
#ifndef CONFIG_CODEC_NO_HP_POP
static void sprd_codec_hp_pop_irq_enable(struct snd_soc_codec *codec)
{
	int mask = BIT(AUDIO_POP_IRQ);
	snd_soc_update_bits(codec, SOC_REG(AUDIF_INT_CLR), mask, mask);
	snd_soc_update_bits(codec, SOC_REG(AUDIF_INT_EN), mask, mask);
}
#endif

static irqreturn_t sprd_codec_ap_irq(int irq, void *dev_id)
{
	int mask;
	struct sprd_codec_priv *sprd_codec = dev_id;
	struct snd_soc_codec *codec = sprd_codec->codec;
	mask = snd_soc_read(codec, AUDIF_INT_MASK);
	sprd_codec_dbg("hp pop irq mask = 0x%x\n", mask);
	if (BIT(AUDIO_POP_IRQ) & mask) {
		mask = BIT(AUDIO_POP_IRQ);
		snd_soc_update_bits(codec, SOC_REG(AUDIF_INT_EN), mask, 0);
		complete(&sprd_codec->completion_hp_pop);
	}
	return IRQ_HANDLED;
}
#endif

#ifndef CONFIG_CODEC_NO_HP_POP
static inline int is_hp_pop_compelet(struct snd_soc_codec *codec)
{
	int val;
	val = snd_soc_read(codec, IFR2_IFR1);
	val = (val >> HP_POP_FLG) & HP_POP_FLG_MASK;
	sprd_codec_dbg("HP POP= 0x%x\n", val);
	return HP_POP_FLG_NEAR_CMP == val;
}

static inline int hp_pop_wait_for_compelet(struct snd_soc_codec *codec)
{
#ifdef CONFIG_SPRD_CODEC_USE_INT
	int i;
	int hp_pop_complete;
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	hp_pop_complete = msecs_to_jiffies(SPRD_CODEC_HP_POP_TIMEOUT);
	for (i = 0; i < 2; i++) {
		sprd_codec_dbg("hp pop %d irq enable\n", i);
		sprd_codec_hp_pop_irq_enable(codec);
		init_completion(&sprd_codec->completion_hp_pop);
		hp_pop_complete =
		    wait_for_completion_timeout(&sprd_codec->completion_hp_pop,
						hp_pop_complete);
		sprd_codec_dbg("hp pop %d completion %d\n", i, hp_pop_complete);
		if (!hp_pop_complete) {
			if (!is_hp_pop_compelet(codec)) {
				pr_err("hp pop %d timeout not complete\n", i);
			} else {
				pr_err("hp pop %d timeout but complete\n", i);
			}
		} else {
			sprd_codec_wait(2);
			if (is_hp_pop_compelet(codec)) {
				return 0;
			}
		}
	}
#else
	int times;
	for (times = 0; times < SPRD_CODEC_HP_POP_TIME_COUNT; times++) {
		if (is_hp_pop_compelet(codec)) {
			sprd_codec_wait(2);
			if (is_hp_pop_compelet(codec)) {
				return 0;
			}
		}
		sprd_codec_wait(SPRD_CODEC_HP_POP_TIME_STEP);
	}
	pr_err("hp pop wait timeout: times = %d \n", times);
#endif
	return 0;
}

static int hp_pop_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int mask;
	int ret = 0;

	sprd_codec_dbg("Entering %s event is %s\n", __func__,
		       get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mask = HP_POP_STEP_MASK << HP_POP_STEP;
		snd_soc_update_bits(codec, SOC_REG(PNRCR2_PNRCR1), mask,
				    HP_POP_STEP_2 << HP_POP_STEP);
		mask = HP_POP_CTL_MASK << HP_POP_CTL;
		snd_soc_update_bits(codec, SOC_REG(PNRCR2_PNRCR1), mask,
				    HP_POP_CTL_UP << HP_POP_CTL);
		sprd_codec_dbg("U PNRCR1 = 0x%x\n",
			       snd_soc_read(codec, PNRCR2_PNRCR1));
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mask = HP_POP_STEP_MASK << HP_POP_STEP;
		snd_soc_update_bits(codec, SOC_REG(PNRCR2_PNRCR1), mask,
				    HP_POP_STEP_1 << HP_POP_STEP);
		mask = HP_POP_CTL_MASK << HP_POP_CTL;
		snd_soc_update_bits(codec, SOC_REG(PNRCR2_PNRCR1), mask,
				    HP_POP_CTL_DOWN << HP_POP_CTL);
		sprd_codec_dbg("D PNRCR1 = 0x%x\n",
			       snd_soc_read(codec, PNRCR2_PNRCR1));
		break;
	case SND_SOC_DAPM_POST_PMU:
		ret = hp_pop_wait_for_compelet(codec);
		mask = HP_POP_CTL_MASK << HP_POP_CTL;
		snd_soc_update_bits(codec, SOC_REG(PNRCR2_PNRCR1), mask,
				    HP_POP_CTL_HOLD << HP_POP_CTL);
		sprd_codec_dbg("HOLD PNRCR1 = 0x%x\n",
			       snd_soc_read(codec, PNRCR2_PNRCR1));
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = hp_pop_wait_for_compelet(codec);
		mask = HP_POP_CTL_MASK << HP_POP_CTL;
		snd_soc_update_bits(codec, SOC_REG(PNRCR2_PNRCR1), mask,
				    HP_POP_CTL_DIS << HP_POP_CTL);
		sprd_codec_dbg("DIS PNRCR1 = 0x%x\n",
			       snd_soc_read(codec, PNRCR2_PNRCR1));
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}
#else
#ifndef CONFIG_HP_POP_DELAY_TIME
#define CONFIG_HP_POP_DELAY_TIME (0)
#endif
static int hp_pop_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;
	sprd_codec_dbg("Entering %s event is %s wait %dms\n", __func__,
		       get_event_name(event), CONFIG_HP_POP_DELAY_TIME);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		sprd_codec_wait(CONFIG_HP_POP_DELAY_TIME);
		break;
	default:
		BUG();
		ret = -EINVAL;
	}
	sprd_codec_dbg("Leaving %s\n", __func__);
	return ret;
}
#endif

static int hp_switch_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
#ifndef CONFIG_CODEC_NO_HP_POP
	int mask;
#endif
	int ret = 0;

	sprd_codec_dbg("Entering %s event is %s\n", __func__,
		       get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
#if 1				
		snd_soc_update_bits(codec, SOC_REG(DCR2_DCR1), BIT(DIFF_EN),
				    BIT(DIFF_EN));
#endif
		
		snd_soc_update_bits(codec, SOC_REG(AUD_SDM_CTL0), DAC_SDM_DODVl_MASK << DAC_SDM_DODVl,
					0x7 << DAC_SDM_DODVl);

#ifndef CONFIG_CODEC_NO_HP_POP
		mask = HP_POP_CTL_MASK << HP_POP_CTL;
		snd_soc_update_bits(codec, SOC_REG(PNRCR2_PNRCR1), mask,
				    HP_POP_CTL_DIS << HP_POP_CTL);
		sprd_codec_dbg("DIS(en) PNRCR1 = 0x%x\n",
			       snd_soc_read(codec, PNRCR2_PNRCR1));
#endif
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, SOC_REG(DCR2_DCR1), BIT(DIFF_EN), 0);
		snd_soc_update_bits(codec, SOC_REG(AUD_SDM_CTL0), DAC_SDM_DODVl_MASK << DAC_SDM_DODVl,
					0x1 << DAC_SDM_DODVl);

#ifndef CONFIG_CODEC_NO_HP_POP
		mask = HP_POP_CTL_MASK << HP_POP_CTL;
		snd_soc_update_bits(codec, SOC_REG(PNRCR2_PNRCR1), mask,
				    HP_POP_CTL_HOLD << HP_POP_CTL);
		sprd_codec_dbg("HOLD(en) PNRCR1 = 0x%x\n",
			       snd_soc_read(codec, PNRCR2_PNRCR1));
#endif
		goto _pre_pmd;
		break;
	case SND_SOC_DAPM_POST_PMD:
		
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	_mixer_setting(codec, SPRD_CODEC_HP_DACL,
		       SPRD_CODEC_HP_MIXER_MAX, SPRD_CODEC_LEFT,
		       snd_soc_read(codec, DCR2_DCR1) & BIT(HPL_EN));

	_mixer_setting(codec, SPRD_CODEC_HP_DACL,
		       SPRD_CODEC_HP_MIXER_MAX, SPRD_CODEC_RIGHT,
		       snd_soc_read(codec, DCR2_DCR1) & BIT(HPR_EN));

_pre_pmd:
	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}

static int spk_switch_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	sprd_codec_dbg("Entering %s event is %s\n", __func__,
		       get_event_name(event));

	if (snd_soc_read(codec, DCR2_DCR1) & BIT(AOL_EN)) {
		switch (event) {
		case SND_SOC_DAPM_POST_PMU:
			sprd_codec_pa_sw_set(SPRD_CODEC_PA_SW_AOL);
			break;
		case SND_SOC_DAPM_PRE_PMD:
			sprd_codec_pa_sw_clr(SPRD_CODEC_PA_SW_AOL);
			return 0;
		default:
			break;
		}
	}

	_mixer_setting(codec, SPRD_CODEC_SPK_DACL,
		       SPRD_CODEC_SPK_MIXER_MAX, SPRD_CODEC_LEFT,
		       (snd_soc_read(codec, DCR2_DCR1) & BIT(AOL_EN)));

	_mixer_setting(codec, SPRD_CODEC_SPK_DACL,
		       SPRD_CODEC_SPK_MIXER_MAX, SPRD_CODEC_RIGHT,
		       (snd_soc_read(codec, DCR2_DCR1) & BIT(AOR_EN)));

	sprd_codec_dbg("Leaving %s\n", __func__);

	return 0;
}

static int ear_switch_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;
	struct snd_soc_codec *codec;
	
	sprd_codec_dbg("Entering %s event is %s\n", __func__,
		       get_event_name(event));
	codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
#ifdef CONFIG_SPRD_CODEC_EAR_WITH_IN_SPK
		sprd_codec_pa_sw_set(SPRD_CODEC_PA_SW_EAR);
#endif
		snd_soc_update_bits(codec, SOC_REG(AUD_SDM_CTL0), 0xfff, 0x400);
		snd_soc_update_bits(codec, SOC_REG(AUD_SDM_CTL1), 0x3ff,  0x0);
		break;
	case SND_SOC_DAPM_POST_PMD:
#ifdef CONFIG_SPRD_CODEC_EAR_WITH_IN_SPK
		sprd_codec_pa_sw_clr(SPRD_CODEC_PA_SW_EAR);
#endif
		printk("XXXXXXXX ear_switch_event PMD");
		snd_soc_update_bits(codec, SOC_REG(AUD_SDM_CTL0), 0xfff, 0x100);
		snd_soc_update_bits(codec, SOC_REG(AUD_SDM_CTL1), 0x3ff,  0x8);
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}

static int adcpgal_set(struct snd_soc_codec *codec, int on)
{
	int mask = ADCPGAL_EN_MASK << ADCPGAL_EN;
	return snd_soc_update_bits(codec, SOC_REG(AACR2_AACR1), mask,
				   on ? mask : 0);
}

static int adcpgar_set(struct snd_soc_codec *codec, int on)
{
	int mask = ADCPGAR_EN_MASK << ADCPGAR_EN;
	return snd_soc_update_bits(codec, SOC_REG(AACR2_AACR1), mask,
				   on ? mask : 0);
}

static int adc_switch_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int is_right = (w->shift == SPRD_CODEC_RIGHT);
	int on = (! !SND_SOC_DAPM_EVENT_ON(event));
	sprd_codec_dbg("Entering %s event is %s\n", __func__,
		       get_event_name(event));

	if (is_right) {
		adcpgar_set(codec, on);
		_mixer_setting(codec, SPRD_CODEC_AIL, SPRD_CODEC_ADC_MIXER_MAX,
			       SPRD_CODEC_RIGHT, on);
	} else {
		adcpgal_set(codec, on);
		_mixer_setting(codec, SPRD_CODEC_AIL, SPRD_CODEC_ADC_MIXER_MAX,
			       SPRD_CODEC_LEFT, on);
	}

	sprd_codec_dbg("Leaving %s\n", __func__);

	return 0;
}

static int pga_event(struct snd_soc_dapm_widget *w,
		     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	unsigned int id = FUN_REG(w->reg);
	struct sprd_codec_pga_op *pga = &(sprd_codec->pga[id]);
	int ret = 0;
	int min = sprd_codec_pga_cfg[id].min;
	static int s_need_wait = 1;

	if (id >= SPRD_CODEC_PGA_NUM) {
		printk(KERN_WARNING "reg(0x%x) overflow\n", w->reg);
		return -EINVAL;
	}

	sprd_codec_dbg("Entering %s set %s(%d) event is %s\n", __func__,
		       sprd_codec_pga_debug_str[id], pga->pgaval,
		       get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if ((id == SPRD_CODEC_PGA_ADCL) || (id == SPRD_CODEC_PGA_ADCR)) {
			if (s_need_wait == 1) {
				sprd_codec_wait(250);
				s_need_wait++;
				sprd_codec_dbg("ADC Switch ON delay\n");
			}
		}
		pga->set = sprd_codec_pga_cfg[id].set;
		ret = pga->set(codec, pga->pgaval);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if ((id == SPRD_CODEC_PGA_ADCL) || (id == SPRD_CODEC_PGA_ADCR)) {
			s_need_wait = 1;
		}
		pga->set = 0;
		ret = sprd_codec_pga_cfg[id].set(codec, min);
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}

static int mic_bias_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	int id = FUN_REG(w->reg);
	int ret = 0;

	sprd_codec_dbg("Entering %s %s event is %s\n", __func__,
		       mic_bias_name[id], get_event_name(event));

	switch (id) {
	case SPRD_CODEC_MIC_BIAS:
		if (!(atomic_read(&sprd_codec_power.mic_on) > 0)) {
			sprd_codec_mic_bias_en(SND_SOC_DAPM_EVENT_ON(event));
		}
		break;
	case SPRD_CODEC_AUXMIC_BIAS:
		if (!(atomic_read(&sprd_codec_power.auxmic_on) > 0)) {
			sprd_codec_auxmic_bias_en(SND_SOC_DAPM_EVENT_ON(event));
		}
		break;
	case SPRD_CODEC_HEADMIC_BIAS:
		if (!(atomic_read(&sprd_codec_power.headmic_on) > 0)) {
			sprd_codec_headmic_bias_en(SND_SOC_DAPM_EVENT_ON
						   (event));
		}
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}

static int mixer_event(struct snd_soc_dapm_widget *w,
		       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	int id = FUN_REG(w->reg);
	struct sprd_codec_mixer *mixer = &(sprd_codec->mixer[id]);
	int ret = 0;

	pr_info("%s event is %s\n", sprd_codec_mixer_debug_str[id],
		get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mixer->on = 1;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mixer->on = 0;
		break;
	default:
		BUG();
		ret = -EINVAL;
	}
	if (ret >= 0)
		_mixer_setting_one(codec, id, mixer->on);

	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}

static int mixer_get(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = wlist->widgets[0]->codec;
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	int id = FUN_REG(mc->reg);
	ucontrol->value.integer.value[0] = sprd_codec->mixer[id].on;
	return 0;
}

static int mixer_set(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = wlist->widgets[0]->codec;
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	int id = FUN_REG(mc->reg);
	struct sprd_codec_mixer *mixer = &(sprd_codec->mixer[id]);
	int ret = 0;

	pr_info("set %s switch %s\n", sprd_codec_mixer_debug_str[id],
		ucontrol->value.integer.value[0] ? "ON" : "OFF");

	if (mixer->on == ucontrol->value.integer.value[0])
		return 0;
	
	snd_soc_dapm_put_volsw(kcontrol, ucontrol);

	
	mixer->on = ucontrol->value.integer.value[0];

	if (mixer->set)
		ret = mixer->set(codec, mixer->on);

	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}

#define SPRD_CODEC_MIXER(xname, xreg)\
	SOC_SINGLE_EXT(xname, FUN_REG(xreg), 0, 1, 0, mixer_get, mixer_set)

static const struct snd_kcontrol_new adcl_mixer_controls[] = {
	SPRD_CODEC_MIXER("AILADCL Switch",
			 ID_FUN(SPRD_CODEC_AIL, SPRD_CODEC_LEFT)),
	SPRD_CODEC_MIXER("AIRADCL Switch",
			 ID_FUN(SPRD_CODEC_AIR, SPRD_CODEC_LEFT)),
	SPRD_CODEC_MIXER("MainMICADCL Switch",
			 ID_FUN(SPRD_CODEC_MAIN_MIC, SPRD_CODEC_LEFT)),
	SPRD_CODEC_MIXER("AuxMICADCL Switch",
			 ID_FUN(SPRD_CODEC_AUX_MIC, SPRD_CODEC_LEFT)),
	SPRD_CODEC_MIXER("HPMICADCL Switch",
			 ID_FUN(SPRD_CODEC_HP_MIC, SPRD_CODEC_LEFT)),
};

static const struct snd_kcontrol_new adcr_mixer_controls[] = {
	SPRD_CODEC_MIXER("AILADCR Switch",
			 ID_FUN(SPRD_CODEC_AIL, SPRD_CODEC_RIGHT)),
	SPRD_CODEC_MIXER("AIRADCR Switch",
			 ID_FUN(SPRD_CODEC_AIR, SPRD_CODEC_RIGHT)),
	SPRD_CODEC_MIXER("MainMICADCR Switch",
			 ID_FUN(SPRD_CODEC_MAIN_MIC, SPRD_CODEC_RIGHT)),
	SPRD_CODEC_MIXER("AuxMICADCR Switch",
			 ID_FUN(SPRD_CODEC_AUX_MIC, SPRD_CODEC_RIGHT)),
	SPRD_CODEC_MIXER("HPMICADCR Switch",
			 ID_FUN(SPRD_CODEC_HP_MIC, SPRD_CODEC_RIGHT)),
};

static const struct snd_kcontrol_new hpl_mixer_controls[] = {
	SPRD_CODEC_MIXER("DACLHPL Switch",
			 ID_FUN(SPRD_CODEC_HP_DACL, SPRD_CODEC_LEFT)),
	SPRD_CODEC_MIXER("DACRHPL Switch",
			 ID_FUN(SPRD_CODEC_HP_DACR, SPRD_CODEC_LEFT)),
	SPRD_CODEC_MIXER("ADCLHPL Switch",
			 ID_FUN(SPRD_CODEC_HP_ADCL, SPRD_CODEC_LEFT)),
	SPRD_CODEC_MIXER("ADCRHPL Switch",
			 ID_FUN(SPRD_CODEC_HP_ADCR, SPRD_CODEC_LEFT)),
};

static const struct snd_kcontrol_new hpr_mixer_controls[] = {
	SPRD_CODEC_MIXER("DACLHPR Switch",
			 ID_FUN(SPRD_CODEC_HP_DACL, SPRD_CODEC_RIGHT)),
	SPRD_CODEC_MIXER("DACRHPR Switch",
			 ID_FUN(SPRD_CODEC_HP_DACR, SPRD_CODEC_RIGHT)),
	SPRD_CODEC_MIXER("ADCLHPR Switch",
			 ID_FUN(SPRD_CODEC_HP_ADCL, SPRD_CODEC_RIGHT)),
	SPRD_CODEC_MIXER("ADCRHPR Switch",
			 ID_FUN(SPRD_CODEC_HP_ADCR, SPRD_CODEC_RIGHT)),
};

static const struct snd_kcontrol_new spkl_mixer_controls[] = {
	SPRD_CODEC_MIXER("DACLSPKL Switch",
			 ID_FUN(SPRD_CODEC_SPK_DACL, SPRD_CODEC_LEFT)),
	SPRD_CODEC_MIXER("DACRSPKL Switch",
			 ID_FUN(SPRD_CODEC_SPK_DACR, SPRD_CODEC_LEFT)),
	SPRD_CODEC_MIXER("ADCLSPKL Switch",
			 ID_FUN(SPRD_CODEC_SPK_ADCL, SPRD_CODEC_LEFT)),
	SPRD_CODEC_MIXER("ADCRSPKL Switch",
			 ID_FUN(SPRD_CODEC_SPK_ADCR, SPRD_CODEC_LEFT)),
};

static const struct snd_kcontrol_new spkr_mixer_controls[] = {
	SPRD_CODEC_MIXER("DACLSPKR Switch",
			 ID_FUN(SPRD_CODEC_SPK_DACL, SPRD_CODEC_RIGHT)),
	SPRD_CODEC_MIXER("DACRSPKR Switch",
			 ID_FUN(SPRD_CODEC_SPK_DACR, SPRD_CODEC_RIGHT)),
	SPRD_CODEC_MIXER("ADCLSPKR Switch",
			 ID_FUN(SPRD_CODEC_SPK_ADCL, SPRD_CODEC_RIGHT)),
	SPRD_CODEC_MIXER("ADCRSPKR Switch",
			 ID_FUN(SPRD_CODEC_SPK_ADCR, SPRD_CODEC_RIGHT)),
};

static const struct snd_soc_dapm_widget sprd_codec_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("Digital Power", SND_SOC_NOPM, 0, 0,
			    digital_power_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("Analog Power", 1, SND_SOC_NOPM, 0, 0,
			      analog_power_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("DA Clk", 2, SOC_REG(CCR), DAC_CLK_EN, 0, NULL,
			      0),
	SND_SOC_DAPM_SUPPLY_S("DRV Clk", 3, SOC_REG(CCR), DRV_CLK_EN, 0, NULL,
			      0),
	SND_SOC_DAPM_SUPPLY_S("AD IBUF", 2, SOC_REG(AACR2_AACR1), ADC_IBUF_PD,
			      1,
			      NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("AD Clk", 3, SOC_REG(CCR), ADC_CLK_EN, 0, NULL,
			      0),

	SND_SOC_DAPM_PGA_S("Digital DACL Switch", 4, SOC_REG(AUD_TOP_CTL),
			   DAC_EN_L, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("Digital DACR Switch", 4, SOC_REG(AUD_TOP_CTL),
			   DAC_EN_R, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("ADie Digital DACL Switch", 5, SOC_REG(AUDIF_ENB),
			   AUDIFA_DACL_EN, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("ADie Digital DACR Switch", 5, SOC_REG(AUDIF_ENB),
			   AUDIFA_DACR_EN, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("DACL Switch", 6, SOC_REG(DACGR_DACR), DACL_EN, 0,
			   NULL,
			   0),
	SND_SOC_DAPM_PGA_S("DACR Switch", 6, SOC_REG(DACGR_DACR), DACR_EN, 0,
			   NULL,
			   0),
	SND_SOC_DAPM_PGA_S("DACL Mute", 6, FUN_REG(SPRD_CODEC_PGA_DACL), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("DACR Mute", 6, FUN_REG(SPRD_CODEC_PGA_DACR), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("DAC", "Playback", SND_SOC_NOPM, 0, 0,
			   dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
#ifdef CONFIG_CODEC_NO_HP_POP
	SND_SOC_DAPM_PGA_S("HP POP", 6, SND_SOC_NOPM, 0, 0, hp_pop_event,
			   SND_SOC_DAPM_POST_PMU),
#else
	SND_SOC_DAPM_SUPPLY_S("HP POP", 4, SND_SOC_NOPM, 0, 0, hp_pop_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD |
			      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
#endif
	SND_SOC_DAPM_PGA_S("HPL Switch", 5, SOC_REG(DCR2_DCR1), HPL_EN, 0,
			   hp_switch_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD |
			   SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("HPR Switch", 5, SOC_REG(DCR2_DCR1), HPR_EN, 0,
			   hp_switch_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD |
			   SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("HPL Mute", 6, FUN_REG(SPRD_CODEC_PGA_HPL), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("HPR Mute", 6, FUN_REG(SPRD_CODEC_PGA_HPR), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER("HPL Mixer", SND_SOC_NOPM, 0, 0,
			   &hpl_mixer_controls[0],
			   ARRAY_SIZE(hpl_mixer_controls)),
	SND_SOC_DAPM_MIXER("HPR Mixer", SND_SOC_NOPM, 0, 0,
			   &hpr_mixer_controls[0],
			   ARRAY_SIZE(hpr_mixer_controls)),
	SND_SOC_DAPM_MIXER("SPKL Mixer", SND_SOC_NOPM, 0, 0,
			   &spkl_mixer_controls[0],
			   ARRAY_SIZE(spkl_mixer_controls)),
	SND_SOC_DAPM_MIXER("SPKR Mixer", SND_SOC_NOPM, 0, 0,
			   &spkr_mixer_controls[0],
			   ARRAY_SIZE(spkr_mixer_controls)),
	SND_SOC_DAPM_PGA_S("SPKL Switch", 5, SOC_REG(DCR2_DCR1), AOL_EN, 0,
			   spk_switch_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("SPKR Switch", 5, SOC_REG(DCR2_DCR1), AOR_EN, 0,
			   spk_switch_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_S("SPKL Mute", 6, FUN_REG(SPRD_CODEC_PGA_SPKL), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("SPKR Mute", 6, FUN_REG(SPRD_CODEC_PGA_SPKR), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("EAR Mixer", 5,
			   FUN_REG(ID_FUN
				   (SPRD_CODEC_EAR_DACL, SPRD_CODEC_LEFT)), 0,
			   0, mixer_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
#if defined(CONFIG_MACH_DUMMY)
	SND_SOC_DAPM_PGA_S("EAR Switch", 6, SOC_REG(DCR2_DCR1), EAR_EN, 0, NULL,
			   0),
#else  
	SND_SOC_DAPM_PGA_S("EAR Switch", 6, SOC_REG(DCR2_DCR1), EAR_EN, 0,
			   ear_switch_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
#endif
	SND_SOC_DAPM_PGA_S("EAR Mute", 7, FUN_REG(SPRD_CODEC_PGA_EAR), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MIXER("ADCL Mixer", SND_SOC_NOPM, 0, 0,
			   &adcl_mixer_controls[0],
			   ARRAY_SIZE(adcl_mixer_controls)),
	SND_SOC_DAPM_MIXER("ADCR Mixer", SND_SOC_NOPM, 0, 0,
			   &adcr_mixer_controls[0],
			   ARRAY_SIZE(adcr_mixer_controls)),
	SND_SOC_DAPM_PGA_S("Digital ADCL Switch", 3, SOC_REG(AUD_TOP_CTL),
			   ADC_EN_L, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("Digital ADCR Switch", 3, SOC_REG(AUD_TOP_CTL),
			   ADC_EN_R, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("ADie Digital ADCL Switch", 2, SOC_REG(AUDIF_ENB),
			   AUDIFA_ADCL_EN, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("ADie Digital ADCR Switch", 2, SOC_REG(AUDIF_ENB),
			   AUDIFA_ADCR_EN, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("ADCL Switch", 1, SOC_REG(AACR2_AACR1), ADCL_PD, 1,
			   NULL,
			   0),
	SND_SOC_DAPM_PGA_S("ADCR Switch", 1, SOC_REG(AACR2_AACR1), ADCR_PD, 1,
			   NULL,
			   0),
	SND_SOC_DAPM_PGA_E("ADCL PGA", SND_SOC_NOPM, SPRD_CODEC_LEFT, 0, NULL,
			   0,
			   adc_switch_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("ADCR PGA", SND_SOC_NOPM, SPRD_CODEC_RIGHT, 0, NULL,
			   0,
			   adc_switch_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_S("ADCL Mute", 3, FUN_REG(SPRD_CODEC_PGA_ADCL), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("ADCR Mute", 3, FUN_REG(SPRD_CODEC_PGA_ADCR), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_ADC_E("ADC", "Main-Capture", SND_SOC_NOPM, 0, 0,
			   adc_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_S("MIC Boost", 3, FUN_REG(SPRD_CODEC_PGA_MIC), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("AUXMIC Boost", 3, FUN_REG(SPRD_CODEC_PGA_AUXMIC), 0,
			   0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("HEADMIC Boost", 3, FUN_REG(SPRD_CODEC_PGA_HEADMIC),
			   0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("AIL Boost", 3, FUN_REG(SPRD_CODEC_PGA_AIL), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("AIR Boost", 3, FUN_REG(SPRD_CODEC_PGA_AIR), 0, 0,
			   pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_MICBIAS_E("Mic Bias", FUN_REG(SPRD_CODEC_MIC_BIAS), 0, 0,
			       mic_bias_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MICBIAS_E("AuxMic Bias", FUN_REG(SPRD_CODEC_AUXMIC_BIAS),
			       0, 0,
			       mic_bias_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MICBIAS_E("HeadMic Bias", FUN_REG(SPRD_CODEC_HEADMIC_BIAS),
			       0, 0,
			       mic_bias_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_S("Digital ADC1L Switch", 5, SOC_REG(AUD_TOP_CTL),
			   ADC1_EN_L, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("Digital ADC1R Switch", 5, SOC_REG(AUD_TOP_CTL),
			   ADC1_EN_R, 0, NULL, 0),
	SND_SOC_DAPM_ADC_E("ADC1", "Ext-Capture", SND_SOC_NOPM, 0, 0,
			   adc1_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	
#ifdef  CONFIG_SPRD_CODEC_DMIC
	SND_SOC_DAPM_PGA_S("Digital ADC DMIC In", 4, SOC_REG(AUD_TOP_CTL),
			   ADC_DMIC_SEL, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("Digital ADC1 DMIC In", 4, SOC_REG(AUD_TOP_CTL),
			   ADC1_DMIC1_SEL, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("DMIC Switch", 3, SOC_REG(AUD_DMIC_CTL),
			   ADC_DMIC_EN, 0,
			   NULL, 0),
	SND_SOC_DAPM_PGA_S("DMIC1 Switch", 3, SOC_REG(AUD_DMIC_CTL),
			   ADC1_DMIC1_EN, 0,
			   NULL, 0),
	SND_SOC_DAPM_INPUT("DMIC"),
	SND_SOC_DAPM_INPUT("DMIC1"),
#endif
	SND_SOC_DAPM_OUTPUT("HEAD_P_L"),
	SND_SOC_DAPM_OUTPUT("HEAD_P_R"),
	SND_SOC_DAPM_OUTPUT("AOL"),
	SND_SOC_DAPM_OUTPUT("SPKL"),
	SND_SOC_DAPM_OUTPUT("AOR"),
	SND_SOC_DAPM_OUTPUT("EAR"),

	SND_SOC_DAPM_INPUT("MIC"),
	SND_SOC_DAPM_INPUT("AUXMIC"),
	SND_SOC_DAPM_INPUT("HPMIC"),
	SND_SOC_DAPM_INPUT("AIL"),
	SND_SOC_DAPM_INPUT("AIR"),
};

static const struct snd_soc_dapm_route sprd_codec_intercon[] = {
	
	{"DA Clk", NULL, "Analog Power"},
	{"DA Clk", NULL, "Digital Power"},
	{"DAC", NULL, "DA Clk"},

	{"AD IBUF", NULL, "Analog Power"},
	{"AD Clk", NULL, "Digital Power"},
	{"AD Clk", NULL, "AD IBUF"},
	{"ADC", NULL, "AD Clk"},

	{"ADC1", NULL, "AD Clk"},

	{"ADCL PGA", NULL, "AD IBUF"},
	{"ADCR PGA", NULL, "AD IBUF"},

	{"HP POP", NULL, "DRV Clk"},
	{"HPL Switch", NULL, "DRV Clk"},
	{"HPR Switch", NULL, "DRV Clk"},
	{"SPKL Switch", NULL, "DRV Clk"},
	{"SPKR Switch", NULL, "DRV Clk"},
	{"EAR Switch", NULL, "DRV Clk"},

	
	{"Digital DACL Switch", NULL, "DAC"},
	{"Digital DACR Switch", NULL, "DAC"},
	{"ADie Digital DACL Switch", NULL, "Digital DACL Switch"},
	{"ADie Digital DACR Switch", NULL, "Digital DACR Switch"},
	{"DACL Mute", NULL, "ADie Digital DACL Switch"},
	{"DACR Mute", NULL, "ADie Digital DACR Switch"},
	{"DACL Switch", NULL, "DACL Mute"},
	{"DACR Switch", NULL, "DACR Mute"},

	
	{"HPL Mixer", "DACLHPL Switch", "DACL Switch"},
	{"HPL Mixer", "DACRHPL Switch", "DACR Switch"},
	{"HPR Mixer", "DACLHPR Switch", "DACL Switch"},
	{"HPR Mixer", "DACRHPR Switch", "DACR Switch"},

	{"HPL Mixer", "ADCLHPL Switch", "ADCL PGA"},
	{"HPL Mixer", "ADCRHPL Switch", "ADCR PGA"},
	{"HPR Mixer", "ADCLHPR Switch", "ADCL PGA"},
	{"HPR Mixer", "ADCRHPR Switch", "ADCR PGA"},

#ifdef CONFIG_CODEC_NO_HP_POP
	{"HEAD_P_L", NULL, "HP POP"},
	{"HEAD_P_R", NULL, "HP POP"},
	{"HP POP", NULL, "HPL Switch"},
	{"HP POP", NULL, "HPR Switch"},
#else
	{"HPL Switch", NULL, "HP POP"},
	{"HPR Switch", NULL, "HP POP"},
#endif
	{"HPL Switch", NULL, "HPL Mixer"},
	{"HPR Switch", NULL, "HPR Mixer"},
	{"HPL Mute", NULL, "HPL Switch"},
	{"HPR Mute", NULL, "HPR Switch"},
	{"HEAD_P_L", NULL, "HPL Mute"},
	{"HEAD_P_R", NULL, "HPR Mute"},

	{"SPKL Mixer", "DACLSPKL Switch", "DACL Switch"},
	{"SPKL Mixer", "DACRSPKL Switch", "DACR Switch"},
	{"SPKR Mixer", "DACLSPKR Switch", "DACL Switch"},
	{"SPKR Mixer", "DACRSPKR Switch", "DACR Switch"},

	{"SPKL Mixer", "ADCLSPKL Switch", "ADCL PGA"},
	{"SPKL Mixer", "ADCRSPKL Switch", "ADCR PGA"},
	{"SPKR Mixer", "ADCLSPKR Switch", "ADCL PGA"},
	{"SPKR Mixer", "ADCRSPKR Switch", "ADCR PGA"},

	{"SPKL Switch", NULL, "SPKL Mixer"},
	{"SPKR Switch", NULL, "SPKR Mixer"},
	{"SPKL Mute", NULL, "SPKL Switch"},
	{"SPKR Mute", NULL, "SPKR Switch"},
	{"AOL", NULL, "SPKL Mute"},
	{"AOR", NULL, "SPKR Mute"},

	{"EAR Mixer", NULL, "DACL Switch"},
	{"EAR Switch", NULL, "EAR Mixer"},
	{"EAR Mute", NULL, "EAR Switch"},
	{"EAR", NULL, "EAR Mute"},

	{"ADCL Mute", NULL, "ADCL Mixer"},
	{"ADCR Mute", NULL, "ADCR Mixer"},
	{"ADCL PGA", NULL, "ADCL Mute"},
	{"ADCR PGA", NULL, "ADCR Mute"},
	
	{"ADCL Mixer", "AILADCL Switch", "AIL Boost"},
	{"ADCR Mixer", "AILADCR Switch", "AIL Boost"},
	{"ADCL Mixer", "AIRADCL Switch", "AIR Boost"},
	{"ADCR Mixer", "AIRADCR Switch", "AIR Boost"},
	{"AIL Boost", NULL, "AIL"},
	{"AIR Boost", NULL, "AIR"},
	
	{"ADCL Mixer", "MainMICADCL Switch", "MIC Boost"},
	{"ADCR Mixer", "MainMICADCR Switch", "MIC Boost"},
	{"MIC Boost", NULL, "Mic Bias"},
	{"ADCL Mixer", "AuxMICADCL Switch", "AUXMIC Boost"},
	{"ADCR Mixer", "AuxMICADCR Switch", "AUXMIC Boost"},
	{"AUXMIC Boost", NULL, "AuxMic Bias"},
	{"ADCL Mixer", "HPMICADCL Switch", "HEADMIC Boost"},
	{"ADCR Mixer", "HPMICADCR Switch", "HEADMIC Boost"},
	{"HEADMIC Boost", NULL, "HeadMic Bias"},
	
	{"ADCL Switch", NULL, "ADCL PGA"},
	{"ADCR Switch", NULL, "ADCR PGA"},
	{"ADie Digital ADCL Switch", NULL, "ADCL Switch"},
	{"ADie Digital ADCR Switch", NULL, "ADCR Switch"},
	{"Digital ADCL Switch", NULL, "ADie Digital ADCL Switch"},
	{"Digital ADCR Switch", NULL, "ADie Digital ADCR Switch"},
	{"ADC", NULL, "Digital ADCL Switch"},
	{"ADC", NULL, "Digital ADCR Switch"},

	{"Digital ADC1L Switch", NULL, "ADie Digital ADCL Switch"},
	{"Digital ADC1R Switch", NULL, "ADie Digital ADCR Switch"},
	{"ADC1", NULL, "Digital ADC1L Switch"},
	{"ADC1", NULL, "Digital ADC1R Switch"},


	{"Mic Bias", NULL, "MIC"},
	{"AuxMic Bias", NULL, "AUXMIC"},
	{"HeadMic Bias", NULL, "HPMIC"},

#ifdef  CONFIG_SPRD_CODEC_DMIC
	
	{"DMIC Switch", NULL, "DMIC"},
	{"Digital ADC DMIC In", NULL, "DMIC Switch"},
	{"Digital ADCL Switch", NULL, "Digital ADC DMIC In"},
	{"Digital ADCR Switch", NULL, "Digital ADC DMIC In"},
	
	{"DMIC1 Switch", NULL, "DMIC1"},
	{"Digital ADC1 DMIC In", NULL, "DMIC1 Switch"},
	{"Digital ADC1L Switch", NULL, "Digital ADC1 DMIC In"},
	{"Digital ADC1R Switch", NULL, "Digital ADC1 DMIC In"},
#endif

	
	{"Mic Bias", NULL, "Analog Power"},
	{"AuxMic Bias", NULL, "Analog Power"},
	{"HeadMic Bias", NULL, "Analog Power"},
};

static int sprd_codec_vol_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	unsigned int reg = FUN_REG(mc->reg);
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val;
	struct sprd_codec_pga_op *pga = &(sprd_codec->pga[reg]);
	int ret = 0;

	if (reg >= SPRD_CODEC_PGA_NUM) {
		printk(KERN_WARNING "reg(0x%x) overflow\n", mc->reg);
		return -EINVAL;
	}

	pr_info("set PGA[%s] to %ld\n", sprd_codec_pga_debug_str[reg],
		ucontrol->value.integer.value[0]);

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = max - val;
	pga->pgaval = val;
	if (pga->set) {
		ret = pga->set(codec, pga->pgaval);
	}
	sprd_codec_dbg("Leaving %s\n", __func__);
	return ret;
}

static int sprd_codec_vol_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	unsigned int reg = FUN_REG(mc->reg);
	int max = mc->max;
	unsigned int invert = mc->invert;
	struct sprd_codec_pga_op *pga = &(sprd_codec->pga[reg]);

	ucontrol->value.integer.value[0] = pga->pgaval;
	if (invert) {
		ucontrol->value.integer.value[0] =
		    max - ucontrol->value.integer.value[0];
	}

	return 0;
}

static int sprd_codec_inter_pa_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val;
	int ret = 0;

	pr_info("config inter PA 0x%08x\n",
		(int)ucontrol->value.integer.value[0]);

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = max - val;
	mutex_lock(&inter_pa_mutex);
	inter_pa.value = (u32) val;
	if (inter_pa.set) {
		mutex_unlock(&inter_pa_mutex);
		sprd_inter_speaker_pa(1);
	} else {
		mutex_unlock(&inter_pa_mutex);
	}
	sprd_codec_dbg("Leaving %s\n", __func__);
	return ret;
}

static int sprd_codec_inter_pa_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;
	unsigned int invert = mc->invert;

	mutex_lock(&inter_pa_mutex);
	ucontrol->value.integer.value[0] = inter_pa.value;
	mutex_unlock(&inter_pa_mutex);
	if (invert) {
		ucontrol->value.integer.value[0] =
		    max - ucontrol->value.integer.value[0];
	}

	return 0;
}

static int sprd_codec_inter_hp_pa_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val;
	int ret = 0;

	pr_info("config inter HP PA 0x%08x\n",
		(int)ucontrol->value.integer.value[0]);

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = max - val;
	mutex_lock(&inter_hp_pa_mutex);
	inter_hp_pa.value = (u32) val;
	if (inter_hp_pa.set) {
		mutex_unlock(&inter_hp_pa_mutex);
		sprd_inter_headphone_pa(1);
	} else {
		mutex_unlock(&inter_hp_pa_mutex);
	}
	sprd_codec_dbg("Leaving %s\n", __func__);
	return ret;
}

static int sprd_codec_inter_hp_pa_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;
	unsigned int invert = mc->invert;

	mutex_lock(&inter_hp_pa_mutex);
	ucontrol->value.integer.value[0] = inter_hp_pa.value;
	mutex_unlock(&inter_hp_pa_mutex);
	if (invert) {
		ucontrol->value.integer.value[0] =
		    max - ucontrol->value.integer.value[0];
	}

	return 0;
}

static int sprd_codec_mic_bias_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	unsigned int reg = FUN_REG(mc->reg);
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val;
	int ret = 0;
	int id = reg;

	val = (ucontrol->value.integer.value[0] & mask);

	pr_info("%s switch %s\n", mic_bias_name[id], val ? "ON" : "OFF");

	if (invert)
		val = max - val;
	if (sprd_codec->mic_bias[id] == val) {
		return 0;
	}

	sprd_codec->mic_bias[id] = val;
	if (val) {
		ret =
		    snd_soc_dapm_force_enable_pin(&codec->card->dapm,
						  mic_bias_name[id]);
	} else {
		ret =
		    snd_soc_dapm_disable_pin(&codec->card->dapm,
					     mic_bias_name[id]);
	}

	
	snd_soc_dapm_sync(&codec->card->dapm);

	sprd_codec_dbg("Leaving %s\n", __func__);
	return ret;
}

static int sprd_codec_mic_bias_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	unsigned int reg = FUN_REG(mc->reg);
	int max = mc->max;
	unsigned int invert = mc->invert;
	int id = reg;

	ucontrol->value.integer.value[0] = sprd_codec->mic_bias[id];
	if (invert) {
		ucontrol->value.integer.value[0] =
		    max - ucontrol->value.integer.value[0];
	}

	return 0;
}

static const DECLARE_TLV_DB_SCALE(adc_tlv, -600, 75, 0);
static const DECLARE_TLV_DB_SCALE(hp_tlv, -3600, 300, 1);
static const DECLARE_TLV_DB_SCALE(ear_tlv, -3600, 300, 1);
static const DECLARE_TLV_DB_SCALE(spk_tlv, -2400, 300, 1);
static const DECLARE_TLV_DB_SCALE(dac_tlv, -350, 50, 1);
static const DECLARE_TLV_DB_SCALE(mic_tlv, 0, 600, 0);
static const DECLARE_TLV_DB_SCALE(auxmic_tlv, 0, 600, 0);
static const DECLARE_TLV_DB_SCALE(headmic_tlv, 0, 600, 0);
static const DECLARE_TLV_DB_SCALE(ailr_tlv, 0, 600, 0);

#define SPRD_CODEC_PGA(xname, xreg, tlv_array) \
	SOC_SINGLE_EXT_TLV(xname, FUN_REG(xreg), 0, 15, 0, \
			sprd_codec_vol_get, sprd_codec_vol_put, tlv_array)

#define SPRD_CODEC_PGA_MAX(xname, xreg, max, tlv_array) \
	SOC_SINGLE_EXT_TLV(xname, FUN_REG(xreg), 0, max, 0, \
			sprd_codec_vol_get, sprd_codec_vol_put, tlv_array)

#define SPRD_CODEC_MIC_BIAS(xname, xreg) \
	SOC_SINGLE_EXT(xname, FUN_REG(xreg), 0, 1, 0, \
			sprd_codec_mic_bias_get, sprd_codec_mic_bias_put)

static const struct snd_kcontrol_new sprd_codec_snd_controls[] = {
	SPRD_CODEC_PGA("SPKL Playback Volume", SPRD_CODEC_PGA_SPKL, spk_tlv),
	SPRD_CODEC_PGA("SPKR Playback Volume", SPRD_CODEC_PGA_SPKR, spk_tlv),
	SPRD_CODEC_PGA("HPL Playback Volume", SPRD_CODEC_PGA_HPL, hp_tlv),
	SPRD_CODEC_PGA("HPR Playback Volume", SPRD_CODEC_PGA_HPR, hp_tlv),
	SPRD_CODEC_PGA("EAR Playback Volume", SPRD_CODEC_PGA_EAR, ear_tlv),

	SPRD_CODEC_PGA_MAX("ADCL Capture Volume", SPRD_CODEC_PGA_ADCL, 63,
			   adc_tlv),
	SPRD_CODEC_PGA_MAX("ADCR Capture Volume", SPRD_CODEC_PGA_ADCR, 63,
			   adc_tlv),

	SPRD_CODEC_PGA_MAX("DACL Playback Volume", SPRD_CODEC_PGA_DACL, 7,
			   dac_tlv),
	SPRD_CODEC_PGA_MAX("DACR Playback Volume", SPRD_CODEC_PGA_DACR, 7,
			   dac_tlv),
	SPRD_CODEC_PGA_MAX("MIC Boost", SPRD_CODEC_PGA_MIC, 3, mic_tlv),
	SPRD_CODEC_PGA_MAX("AUXMIC Boost", SPRD_CODEC_PGA_AUXMIC, 3,
			   auxmic_tlv),
	SPRD_CODEC_PGA_MAX("HEADMIC Boost", SPRD_CODEC_PGA_HEADMIC, 3,
			   headmic_tlv),
	SPRD_CODEC_PGA_MAX("Linein Boost", SPRD_CODEC_PGA_AIL, 3, ailr_tlv),

	SOC_SINGLE_EXT("Inter PA Config", 0, 0, LONG_MAX, 0,
		       sprd_codec_inter_pa_get, sprd_codec_inter_pa_put),

	SOC_SINGLE_EXT("Inter HP PA Config", 0, 0, LONG_MAX, 0,
		       sprd_codec_inter_hp_pa_get, sprd_codec_inter_hp_pa_put),

	SPRD_CODEC_MIC_BIAS("MIC Bias Switch", SPRD_CODEC_MIC_BIAS),

	SPRD_CODEC_MIC_BIAS("AUXMIC Bias Switch", SPRD_CODEC_AUXMIC_BIAS),

	SPRD_CODEC_MIC_BIAS("HEADMIC Bias Switch", SPRD_CODEC_HEADMIC_BIAS),
};
 int vbc_reg_read(int reg);
int vbc_reg_write2(int reg, int val);
int vbc_mux_reg_read(int reg);

static unsigned int sprd_codec_read(struct snd_soc_codec *codec,
				    unsigned int reg)
{
	if (IS_SPRD_CODEC_AP_RANG(reg | SPRD_CODEC_AP_BASE_HI)) {
		reg |= SPRD_CODEC_AP_BASE_HI;
		return arch_audio_codec_read(reg);
	} else if (IS_SPRD_CODEC_DP_RANG(reg | SPRD_CODEC_DP_BASE_HI)) {
		reg |= SPRD_CODEC_DP_BASE_HI;
		return __raw_readl(reg);
	} else if (IS_SPRD_CODEC_MIXER_RANG(FUN_REG(reg))) {
		struct sprd_codec_priv *sprd_codec =
		    snd_soc_codec_get_drvdata(codec);
		int id = FUN_REG(reg);
		struct sprd_codec_mixer *mixer = &(sprd_codec->mixer[id]);
		return mixer->on;
	} else if (IS_SPRD_VBC_RANG(reg |SPRD_VBC_BASE_HI)) {
		sprd_codec_dbg("read the register is vbc's reg = 0x%x\n", (reg - VBC_REG_OFFSET));
		reg |= SPRD_VBC_BASE_HI;
		return vbc_reg_read(reg);
	} else if (IS_SPRD_VBC_MUX_RANG(FUN_REG(reg))) {
		sprd_codec_dbg("read the register is vbc  MUX  reg(%d) = 0x%x\n", FUN_REG(reg), vbc_mux_reg_read(reg));
		return vbc_mux_reg_read(reg);
	}
	sprd_codec_dbg("read the register is not codec's reg = 0x%x\n", reg);
	return 0;
}
static int sprd_codec_write(struct snd_soc_codec *codec, unsigned int reg,
			    unsigned int val)
{
	if (IS_SPRD_CODEC_AP_RANG(reg | SPRD_CODEC_AP_BASE_HI)) {
		reg |= SPRD_CODEC_AP_BASE_HI;
		return arch_audio_codec_write(reg, val);
	} else if (IS_SPRD_CODEC_DP_RANG(reg | SPRD_CODEC_DP_BASE_HI)) {
		reg |= SPRD_CODEC_DP_BASE_HI;
		return __raw_writel(val, reg);
	} else if (IS_SPRD_VBC_RANG(reg |SPRD_VBC_BASE_HI)) {
		sprd_codec_dbg("write the register is vbc's reg = 0x%x, val = %d\n", (reg - VBC_REG_OFFSET), val);
		reg |= SPRD_VBC_BASE_HI;
		return vbc_reg_write2(reg, val);
	}
	sprd_codec_dbg("write the register is not codec's reg = 0x%x\n", reg);
	return 0;
}

#if 0 
static int sprd_codec_pcm_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	#if 0 
	struct snd_soc_codec *codec = dai->codec;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		snd_soc_dapm_force_enable_pin(&codec->card->dapm, "DAC");
	} else {
		if (dai->id != SPRD_CODEC_IIS1_ID) {
			sprd_codec_dbg("open ADC\n");
			snd_soc_dapm_force_enable_pin(&codec->card->dapm,
						      "ADC");
		} else {
			sprd_codec_dbg("open ADC1\n");
			snd_soc_dapm_force_enable_pin(&codec->card->dapm,
						      "ADC1");

		}
	}
	#endif
	return 0;
}

static void sprd_codec_pcm_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		snd_soc_dapm_disable_pin(&codec->card->dapm, "DAC");
	} else {
		if (dai->id != SPRD_CODEC_IIS1_ID) {
			sprd_codec_dbg("shutdown ADC\n");
			snd_soc_dapm_disable_pin(&codec->card->dapm, "ADC");
		}
		else {
			sprd_codec_dbg("shutdown ADC1\n");
			snd_soc_dapm_disable_pin(&codec->card->dapm, "ADC1");
		}
	}
}
#endif

static int sprd_codec_pcm_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	int mask = 0x0F;
	int shift = 0;
	int rate = params_rate(params);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sprd_codec->da_sample_val = rate;
		pr_info("playback rate is [%d]\n", rate);
		sprd_codec_set_sample_rate(codec, rate, mask, shift);
	} else {
		pr_info("capture rate is [%d]\n", rate);
		if (dai->id != SPRD_CODEC_IIS1_ID) {
			sprd_codec->ad_sample_val = rate;
			sprd_codec_set_ad_sample_rate(codec, rate, mask, shift);
		} else {
			sprd_codec->ad1_sample_val = rate;
			sprd_codec_set_ad_sample_rate(codec, rate,
						      ADC1_SRC_N_MASK,
						      ADC1_SRC_N);
		}
	}

	return 0;
}

static int sprd_codec_pcm_hw_free(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	sprd_codec_dbg("Entering %s\n", __func__);
	sprd_codec_dbg("Leaving %s\n", __func__);
	return 0;
}

#ifdef CONFIG_SPRD_CODEC_USE_INT
#ifdef CONFIG_CODEC_DAC_MUTE_WAIT
static void sprd_codec_dac_mute_irq_enable(struct snd_soc_codec *codec)
{
	int mask = BIT(DAC_MUTE_D);
	snd_soc_update_bits(codec, SOC_REG(AUD_INT_CLR), mask, mask);
	snd_soc_update_bits(codec, SOC_REG(AUD_INT_EN), mask, mask);
}
#endif

static irqreturn_t sprd_codec_dp_irq(int irq, void *dev_id)
{
	int mask;
	struct sprd_codec_priv *sprd_codec = dev_id;
	struct snd_soc_codec *codec = sprd_codec->codec;
	mask = snd_soc_read(codec, AUD_AUD_STS0);
	sprd_codec_dbg("dac mute irq mask = 0x%x\n", mask);
	if (BIT(DAC_MUTE_D_MASK) & mask) {
		mask = BIT(DAC_MUTE_D);
		complete(&sprd_codec->completion_dac_mute);
	}
	if (BIT(DAC_MUTE_U_MASK) & mask) {
		mask = BIT(DAC_MUTE_U);
	}
	snd_soc_update_bits(codec, SOC_REG(AUD_INT_EN), mask, 0);
	return IRQ_HANDLED;
}
#endif

#if 0
static int sprd_codec_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	sprd_codec_dbg("Entering %s\n", __func__);
	if (dai->id == SPRD_CODEC_IIS1_ID)   
		return 0;

	if (atomic_read(&sprd_codec->power_refcount) >= 1) {
		sprd_codec_dbg("mute %i\n", mute);

		ret =
		    snd_soc_update_bits(codec, SOC_REG(AUD_DAC_CTL),
					BIT(DAC_MUTE_START),
					mute ? BIT(DAC_MUTE_START) : 0);
#ifdef CONFIG_CODEC_DAC_MUTE_WAIT
#ifdef CONFIG_SPRD_CODEC_USE_INT
		if (mute && ret) {
			int dac_mute_complete;
			struct sprd_codec_priv *sprd_codec =
			    snd_soc_codec_get_drvdata(codec);
			sprd_codec_dbg("dac mute irq enable\n");
			sprd_codec_dac_mute_irq_enable(codec);
			init_completion(&sprd_codec->completion_dac_mute);
			dac_mute_complete =
			    wait_for_completion_timeout
			    (&sprd_codec->completion_dac_mute,
			     msecs_to_jiffies(SPRD_CODEC_DAC_MUTE_TIMEOUT));
			sprd_codec_dbg("dac mute completion %d\n",
				       dac_mute_complete);
			if (!dac_mute_complete) {
				pr_err("dac mute timeout\n");
			}
		}
#else
		if (mute && ret) {
			sprd_codec_wait(SPRD_CODEC_DAC_MUTE_WAIT_TIME);
		}
#endif
#endif

		sprd_codec_dbg("return %i\n", ret);
	}

	sprd_codec_dbg("Leaving %s\n", __func__);

	return ret;
}
#endif
static struct snd_soc_dai_ops sprd_codec_dai_ops = {
	
	
	.hw_params = sprd_codec_pcm_hw_params,
	.hw_free = sprd_codec_pcm_hw_free,
	
};

#ifdef CONFIG_PM
int sprd_codec_soc_suspend(struct snd_soc_codec *codec)
{
	sprd_codec_dbg("Entering %s\n", __func__);
	arch_audio_codec_pwrdown(1);
	sprd_codec_dbg("Leaving %s\n", __func__);

	return 0;
}

int sprd_codec_soc_resume(struct snd_soc_codec *codec)
{
	sprd_codec_dbg("Entering %s\n", __func__);
	arch_audio_codec_pwrdown(0);
	sprd_codec_dbg("Leaving %s\n", __func__);

	return 0;
}
#else
#define sprd_codec_soc_suspend NULL
#define sprd_codec_soc_resume  NULL
#endif


#ifdef CONFIG_PROC_FS
static void sprd_codec_proc_read(struct snd_info_entry *entry,
				 struct snd_info_buffer *buffer)
{
	struct sprd_codec_priv *sprd_codec = entry->private_data;
	struct snd_soc_codec *codec = sprd_codec->codec;
	int reg;

	snd_iprintf(buffer, "%s digital part\n", codec->name);
	for (reg = SPRD_CODEC_DP_BASE; reg < SPRD_CODEC_DP_END; reg += 0x10) {
		snd_iprintf(buffer, "0x%04x | 0x%04x 0x%04x 0x%04x 0x%04x\n",
			    (unsigned int)(reg - SPRD_CODEC_DP_BASE)
			    , snd_soc_read(codec, reg + 0x00)
			    , snd_soc_read(codec, reg + 0x04)
			    , snd_soc_read(codec, reg + 0x08)
			    , snd_soc_read(codec, reg + 0x0C)
		    );
	}
	snd_iprintf(buffer, "%s analog part\n", codec->name);
	for (reg = SPRD_CODEC_AP_BASE; reg < SPRD_CODEC_AP_END; reg += 0x10) {
		snd_iprintf(buffer, "0x%04x | 0x%04x 0x%04x 0x%04x 0x%04x\n",
			    (unsigned int)(reg - SPRD_CODEC_AP_BASE)
			    , snd_soc_read(codec, reg + 0x00)
			    , snd_soc_read(codec, reg + 0x04)
			    , snd_soc_read(codec, reg + 0x08)
			    , snd_soc_read(codec, reg + 0x0C)
		    );
	}
}

static void sprd_codec_proc_init(struct sprd_codec_priv *sprd_codec)
{
	struct snd_info_entry *entry;
	struct snd_soc_codec *codec = sprd_codec->codec;

	if (!snd_card_proc_new(codec->card->snd_card, "sprd-codec", &entry))
		snd_info_set_text_ops(entry, sprd_codec, sprd_codec_proc_read);
}
#else 
static inline void sprd_codec_proc_init(struct sprd_codec_priv *sprd_codec)
{
}
#endif

static void sprd_codec_power_changed(struct power_supply *psy)
{
	sprd_codec_dbg("Entering %s\n", __func__);
#if 1
	mutex_lock(&inter_pa_mutex);
	if (inter_pa.set && inter_pa.setting.is_LDO_mode
	    && inter_pa.setting.is_auto_LDO_mode) {
		sprd_codec_auto_ldo_volt(sprd_codec_pa_ldo_v_sel, 0);
	}
	mutex_unlock(&inter_pa_mutex);
	if (atomic_read(&sprd_codec_power.ldo_refcount) >= 1) {
		sprd_codec_auto_ldo_volt(sprd_codec_vcm_v_sel, 0);
	}
#endif
	sprd_codec_dbg("Leaving %s\n", __func__);
}

static int sprd_codec_audio_ldo(struct sprd_codec_priv *sprd_codec)
{
	int ret = 0;
	struct snd_soc_codec *codec = sprd_codec->codec;
	sprd_codec->audio_ldo.name = "audio-ldo";
	sprd_codec->audio_ldo.external_power_changed = sprd_codec_power_changed;
	ret = power_supply_register(codec->dev, &sprd_codec->audio_ldo);
	if (ret) {
		pr_err("register power supply error!\n");
		return -EFAULT;
	}
	return ret;
}

#define SPRD_CODEC_PCM_RATES 	\
	(SNDRV_PCM_RATE_8000 |  \
	 SNDRV_PCM_RATE_11025 | \
	 SNDRV_PCM_RATE_16000 | \
	 SNDRV_PCM_RATE_22050 | \
	 SNDRV_PCM_RATE_32000 | \
	 SNDRV_PCM_RATE_44100 | \
	 SNDRV_PCM_RATE_48000 | \
	 SNDRV_PCM_RATE_96000)

#define SPRD_CODEC_PCM_AD_RATES	\
	(SNDRV_PCM_RATE_8000 |  \
	 SNDRV_PCM_RATE_16000 | \
	 SNDRV_PCM_RATE_32000 | \
	 SNDRV_PCM_RATE_44100 | \
	 SNDRV_PCM_RATE_48000)

struct snd_soc_dai_driver sprd_codec_dai[] = {
	{
		.name = "sprd-codec-i2s",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SPRD_CODEC_PCM_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "Main-Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SPRD_CODEC_PCM_AD_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &sprd_codec_dai_ops,
	},
	
	{
		.id = SPRD_CODEC_IIS1_ID,
		.name = "codec-i2s-ext",
		.capture = {
			.stream_name = "Ext-Capture",
			.channels_min = 1,
			.channels_max =2,
			.rates = SPRD_CODEC_PCM_AD_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &sprd_codec_dai_ops,
	},
};

static int sprd_codec_soc_probe(struct snd_soc_codec *codec)
{
	struct sprd_codec_priv *sprd_codec = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	sprd_codec_dbg("Entering %s\n", __func__);

	codec->dapm.idle_bias_off = 1;

	sprd_codec->codec = codec;

	sprd_codec_proc_init(sprd_codec);

	sprd_codec_audio_ldo(sprd_codec);

	arch_audio_codec_pwrdown(0);

	sprd_codec_dbg("return %i\n", ret);
	sprd_codec_dbg("Leaving %s\n", __func__);
	return ret;
}

static int sprd_codec_soc_remove(struct snd_soc_codec *codec)
{
	sprd_codec_dbg("Entering %s\n", __func__);

	sprd_codec_dbg("Leaving %s\n", __func__);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_sprd_codec = {
	.probe = sprd_codec_soc_probe,
	.remove = sprd_codec_soc_remove,
	.suspend = sprd_codec_soc_suspend,
	.resume = sprd_codec_soc_resume,
	.read = sprd_codec_read,
	.write = sprd_codec_write,
	.reg_word_size = sizeof(u16),
	.reg_cache_step = 2,
	.dapm_widgets = sprd_codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sprd_codec_dapm_widgets),
	.dapm_routes = sprd_codec_intercon,
	.num_dapm_routes = ARRAY_SIZE(sprd_codec_intercon),
	.controls = sprd_codec_snd_controls,
	.num_controls = ARRAY_SIZE(sprd_codec_snd_controls),
};

static __devinit int sprd_codec_probe(struct platform_device *pdev)
{
	struct sprd_codec_priv *sprd_codec;
	int ret;

	sprd_codec_dbg("Entering %s\n", __func__);

	sprd_codec = devm_kzalloc(&pdev->dev, sizeof(struct sprd_codec_priv),
				  GFP_KERNEL);
	if (sprd_codec == NULL)
		return -ENOMEM;
	sprd_codec->da_sample_val = 44100; 

	platform_set_drvdata(pdev, sprd_codec);

	atomic_set(&sprd_codec->power_refcount, 0);

#ifdef CONFIG_SPRD_CODEC_USE_INT
	sprd_codec->ap_irq = CODEC_AP_IRQ;

	ret =
	    request_irq(sprd_codec->ap_irq, sprd_codec_ap_irq, 0,
			"sprd_codec_ap", sprd_codec);
	if (ret) {
		pr_err("request_irq ap failed!\n");
		goto err_irq;
	}

	sprd_codec->dp_irq = CODEC_DP_IRQ;

	ret =
	    request_irq(sprd_codec->dp_irq, sprd_codec_dp_irq, 0,
			"sprd_codec_dp", sprd_codec);
	if (ret) {
		pr_err("request_irq dp failed!\n");
		goto dp_err_irq;
	}
#endif

	ret = snd_soc_register_codec(&pdev->dev,
				     &soc_codec_dev_sprd_codec, sprd_codec_dai,
				     ARRAY_SIZE(sprd_codec_dai));
	if (ret != 0) {
		pr_err("Failed to register CODEC: %d\n", ret);
		return ret;
	}

	sprd_codec_dbg("Leaving %s\n", __func__);

	return 0;

#ifdef CONFIG_SPRD_CODEC_USE_INT
dp_err_irq:
	free_irq(sprd_codec->ap_irq, sprd_codec);
err_irq:
	return -EINVAL;
#endif
}

static int __devexit sprd_codec_remove(struct platform_device *pdev)
{
#ifdef CONFIG_SPRD_CODEC_USE_INT
	struct sprd_codec_priv *sprd_codec = platform_get_drvdata(pdev);
	free_irq(sprd_codec->ap_irq, sprd_codec);
	free_irq(sprd_codec->dp_irq, sprd_codec);
#endif
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver sprd_codec_codec_driver = {
	.driver = {
		   .name = "sprd-codec",
		   .owner = THIS_MODULE,
		   },
	.probe = sprd_codec_probe,
	.remove = __devexit_p(sprd_codec_remove),
};

static int sprd_codec_init(void)
{
	sprd_codec_inter_pa_init();
	sprd_codec_inter_hp_pa_init();
	arch_audio_codec_switch(AUDIO_TO_AP_ARM_CTRL);
	return platform_driver_register(&sprd_codec_codec_driver);
}

static void sprd_codec_exit(void)
{
	platform_driver_unregister(&sprd_codec_codec_driver);
}

module_init(sprd_codec_init);
module_exit(sprd_codec_exit);

MODULE_DESCRIPTION("SPRD-CODEC ALSA SoC codec driver");
MODULE_AUTHOR("Zhenfang Wang <zhenfang.wang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("codec:sprd-codec");
