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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/math64.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/pid.h>

#include <mach/hardware.h>

#include "gsp_drv.h"
#include "scaler_coef_cal.h"

#ifdef GSP_WORK_AROUND1
#include <linux/dma-mapping.h>
#endif
static volatile uint32_t gsp_coef_force_calc = 0;

static ulong gsp_gap = 0;
module_param(gsp_gap, ulong, 0644);
MODULE_PARM_DESC(gsp_gap, "gsp ddr gap(0~255)");

static uint gsp_clock = 3;
module_param(gsp_clock, uint, 0644);
MODULE_PARM_DESC(gsp_clock, "gsp clock(0:96M 1:153.6M 2:192M 3:256M)");

static int ahb_clock = 2;
module_param(ahb_clock, int, 0644);
MODULE_PARM_DESC(ahb_clock, "ahb clock(0:26M 1:76M 2:128M 3:192M)");


#define PERF_MAGIC 0x0000dead
#define PERF_STATIC_CNT_PWR 8
#define PERF_STATIC_CNT_MAX (1 << PERF_STATIC_CNT_PWR)

static unsigned long gsp_perf_cnt = 0;
static unsigned long long gsp_perf_s = 0;
static unsigned long long gsp_perf_ns = 0;
static unsigned long gsp_perf = 0;
module_param(gsp_perf, ulong, 0644);
MODULE_PARM_DESC(gsp_perf, "write 0xdead to gsp_perf will trigger gsp static time cost,then write back to gsp_perf.");

static unsigned long gsp_workaround_perf_cnt = 0;
static unsigned long long gsp_workaround_perf_s = 0;
static unsigned long long gsp_workaround_perf_ns = 0;
static unsigned long gsp_workaround_perf = 0;
module_param(gsp_workaround_perf, ulong, 0644);
MODULE_PARM_DESC(gsp_workaround_perf, "write 0xdead to gsp_workaround_perf will trigger gsp static workaround time cost,then write back to gsp_workaround_perf.");

static ktime_t s_ktime_start;
static ktime_t s_ktime_end;
static volatile uint32_t		gsp_cur_client_phase = 0;
#define PHASE_IN_CONFIG			(0x1 << 0)
#define PHASE_IN_TRIGGER		(0x1 << 1)
#define PHASE_IN_WORKAROUND		(0x1 << 2)
#define PHASE_OUT_WORKAROUND	(0x1 << 3)
#define PHASE_IN_WAITDONE		(0x1 << 4)
#define PHASE_OUT_WAITDONE		(0x1 << 5)
#define PHASE_SET_LINE()		(gsp_cur_client_phase = ((gsp_cur_client_phase & 0xffff) | (__LINE__ << 16)))

static volatile pid_t           gsp_cur_client_pid = INVALID_USER_ID;
static struct semaphore         gsp_hw_resource_sem;
static struct semaphore         gsp_wait_interrupt_sem;
GSP_CONFIG_INFO_T               s_gsp_cfg;
static struct proc_dir_entry    *gsp_drv_proc_file;
struct clk						*g_gsp_emc_clk;


#define GSP_ERR_RECORD_CNT  8
static GSP_REG_T g_gsp_reg_err_record[GSP_ERR_RECORD_CNT];
static uint32_t g_gsp_reg_err_record_rp=0;
static uint32_t g_gsp_reg_err_record_wp=0;

#define ERR_RECORD_INDEX_ADD(v) (v = ((v+1)&(GSP_ERR_RECORD_CNT-1)))
#define ERR_RECORD_INDEX_ADD_RP()   ERR_RECORD_INDEX_ADD(g_gsp_reg_err_record_rp)
#define ERR_RECORD_INDEX_ADD_WP()   ERR_RECORD_INDEX_ADD(g_gsp_reg_err_record_wp)
#define ERR_RECORD_INDEX_GET_RP()   (g_gsp_reg_err_record_rp)
#define ERR_RECORD_INDEX_GET_WP()   (g_gsp_reg_err_record_wp)
#define ERR_RECORD_ADD(v)   (g_gsp_reg_err_record[g_gsp_reg_err_record_wp] = (v))
#define ERR_RECORD_GET(v)   (&g_gsp_reg_err_record[g_gsp_reg_err_record_rp])
#define ERR_RECORD_EMPTY()  (g_gsp_reg_err_record_rp == g_gsp_reg_err_record_wp)
#define ERR_RECORD_FULL()   ((g_gsp_reg_err_record_wp+1)&(GSP_ERR_RECORD_CNT-1) == g_gsp_reg_err_record_rp)


static gsp_user gsp_user_array[GSP_MAX_USER];
static gsp_user* gsp_get_user(pid_t user_pid)
{
    gsp_user* ret_user = NULL;
    int i;

    for (i = 0; i < GSP_MAX_USER; i ++)
    {
        if ((gsp_user_array + i)->pid == user_pid)
        {
            ret_user = gsp_user_array + i;
            break;
        }
    }

    if (ret_user == NULL)
    {
        for (i = 0; i < GSP_MAX_USER; i ++)
        {
            if ((gsp_user_array + i)->pid == INVALID_USER_ID)
            {
                ret_user = gsp_user_array + i;
                ret_user->pid = user_pid;
                break;
            }
        }
    }

    return ret_user;
}


static void printCfgInfo(void)
{
    printk("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& \n");
    printk( "misc: ahb_clock %d|gsp_clock %d|dithering_en %d|gsp_gap %d\n",
            s_gsp_cfg.misc_info.ahb_clock,
            s_gsp_cfg.misc_info.gsp_clock,
            s_gsp_cfg.misc_info.dithering_en,
            s_gsp_cfg.misc_info.gsp_gap);

    printk( "L0: format:%d,pitch:%d,clip(x:%d,y:%d,w:%d,h:%d) =rot:%d=> des(x:%d,y:%d,w:%d,h:%d)\n",
            s_gsp_cfg.layer0_info.img_format,
            s_gsp_cfg.layer0_info.pitch,
            s_gsp_cfg.layer0_info.clip_rect.st_x,
            s_gsp_cfg.layer0_info.clip_rect.st_y,
            s_gsp_cfg.layer0_info.clip_rect.rect_w,
            s_gsp_cfg.layer0_info.clip_rect.rect_h,
            s_gsp_cfg.layer0_info.rot_angle,
            s_gsp_cfg.layer0_info.des_rect.st_x,
            s_gsp_cfg.layer0_info.des_rect.st_y,
            s_gsp_cfg.layer0_info.des_rect.rect_w,
            s_gsp_cfg.layer0_info.des_rect.rect_h);
    printk( "L0: alpha:%d,colorkey_en:%d,pallet_en:%d,scaling_en:%d,layer_en:%d,pmargb_en:%d,pmargb_mod:%d\n",
            s_gsp_cfg.layer0_info.alpha,
            s_gsp_cfg.layer0_info.colorkey_en,
            s_gsp_cfg.layer0_info.pallet_en,
            s_gsp_cfg.layer0_info.scaling_en,
            s_gsp_cfg.layer0_info.layer_en,
            s_gsp_cfg.layer0_info.pmargb_en,
            s_gsp_cfg.layer0_info.pmargb_mod);
    printk( "L0: col_tap_mode:%d,row_tap_mode:%d,a_swap_mode:%d,rgb_swap_mode:%d,va_word_endn:%d,va_lng_wrd_endn:%d,uv_word_endn:%d,uv_lng_wrd_endn:%d,y_word_endn:%d,y_lng_wrd_endn:%d\n",
            s_gsp_cfg.layer0_info.col_tap_mode,
            s_gsp_cfg.layer0_info.row_tap_mode,
            s_gsp_cfg.layer0_info.endian_mode.a_swap_mode,
            s_gsp_cfg.layer0_info.endian_mode.rgb_swap_mode,
            s_gsp_cfg.layer0_info.endian_mode.va_word_endn,
            s_gsp_cfg.layer0_info.endian_mode.va_lng_wrd_endn,
            s_gsp_cfg.layer0_info.endian_mode.uv_word_endn,
            s_gsp_cfg.layer0_info.endian_mode.uv_lng_wrd_endn,
            s_gsp_cfg.layer0_info.endian_mode.y_word_endn,
            s_gsp_cfg.layer0_info.endian_mode.y_lng_wrd_endn);
    printk( "L0: addr_y:0x%08x,addr_uv:0x%08x,addr_v:0x%08x,(grey r:%d,g:%d,b:%d,a:%d),(colorkey r:%d,g:%d,b:%d,a:%d)\n",
           s_gsp_cfg.layer0_info.src_addr.addr_y,
            s_gsp_cfg.layer0_info.src_addr.addr_uv,
            s_gsp_cfg.layer0_info.src_addr.addr_v,
            s_gsp_cfg.layer0_info.grey.r_val,
            s_gsp_cfg.layer0_info.grey.g_val,
            s_gsp_cfg.layer0_info.grey.b_val,
            s_gsp_cfg.layer0_info.grey.a_val,
            s_gsp_cfg.layer0_info.colorkey.r_val,
            s_gsp_cfg.layer0_info.colorkey.g_val,
            s_gsp_cfg.layer0_info.colorkey.b_val,
            s_gsp_cfg.layer0_info.colorkey.a_val);


    printk( "L1: format:%d,pitch:%d,clip(x:%d,y:%d,w:%d,h:%d) =rot:%d=> des(x:%d,y:%d)\n",
            s_gsp_cfg.layer1_info.img_format,
            s_gsp_cfg.layer1_info.pitch,
            s_gsp_cfg.layer1_info.clip_rect.st_x,
            s_gsp_cfg.layer1_info.clip_rect.st_y,
            s_gsp_cfg.layer1_info.clip_rect.rect_w,
            s_gsp_cfg.layer1_info.clip_rect.rect_h,
            s_gsp_cfg.layer1_info.rot_angle,
            s_gsp_cfg.layer1_info.des_pos.pos_pt_x,
            s_gsp_cfg.layer1_info.des_pos.pos_pt_y);
    printk( "L1: alpha:%d,colorkey_en:%d,pallet_en:%d,layer_en:%d,pmargb_en:%d,pmargb_mod:%d\n",
            s_gsp_cfg.layer1_info.alpha,
            s_gsp_cfg.layer1_info.colorkey_en,
            s_gsp_cfg.layer1_info.pallet_en,
            s_gsp_cfg.layer1_info.layer_en,
            s_gsp_cfg.layer1_info.pmargb_en,
            s_gsp_cfg.layer1_info.pmargb_mod);

    printk( "L1: col_tap_mode:%d,row_tap_mode:%d,a_swap_mode:%d,rgb_swap_mode:%d,va_word_endn:%d,va_lng_wrd_endn:%d,uv_word_endn:%d,uv_lng_wrd_endn:%d,y_word_endn:%d,y_lng_wrd_endn:%d\n",
            s_gsp_cfg.layer1_info.col_tap_mode,
            s_gsp_cfg.layer1_info.row_tap_mode,
            s_gsp_cfg.layer1_info.endian_mode.a_swap_mode,
            s_gsp_cfg.layer1_info.endian_mode.rgb_swap_mode,
            s_gsp_cfg.layer1_info.endian_mode.va_word_endn,
            s_gsp_cfg.layer1_info.endian_mode.va_lng_wrd_endn,
            s_gsp_cfg.layer1_info.endian_mode.uv_word_endn,
            s_gsp_cfg.layer1_info.endian_mode.uv_lng_wrd_endn,
            s_gsp_cfg.layer1_info.endian_mode.y_word_endn,
            s_gsp_cfg.layer1_info.endian_mode.y_lng_wrd_endn);
    printk( "L1: addr_y:0x%08x,addr_uv:0x%08x,addr_v:0x%08x,(grey r:%d,g:%d,b:%d,a:%d),(colorkey r:%d,g:%d,b:%d,a:%d)\n",
            s_gsp_cfg.layer1_info.src_addr.addr_y,
            s_gsp_cfg.layer1_info.src_addr.addr_uv,
            s_gsp_cfg.layer1_info.src_addr.addr_v,
            s_gsp_cfg.layer1_info.grey.r_val,
            s_gsp_cfg.layer1_info.grey.g_val,
           s_gsp_cfg.layer1_info.grey.b_val,
            s_gsp_cfg.layer1_info.grey.a_val,
           s_gsp_cfg.layer1_info.colorkey.r_val,
            s_gsp_cfg.layer1_info.colorkey.g_val,
            s_gsp_cfg.layer1_info.colorkey.b_val,
            s_gsp_cfg.layer1_info.colorkey.a_val);


    printk( "Ld cfg:fmt:%d|pitch %04d|cmpr8 %d\n",
            s_gsp_cfg.layer_des_info.img_format,
            s_gsp_cfg.layer_des_info.pitch,
            s_gsp_cfg.layer_des_info.compress_r8_en);
   printk( "Ld Yaddr 0x%08x|Uaddr 0x%08x|Vaddr 0x%08x\n",
           s_gsp_cfg.layer_des_info.src_addr.addr_y,
            s_gsp_cfg.layer_des_info.src_addr.addr_uv,
            s_gsp_cfg.layer_des_info.src_addr.addr_v);
    printk( "Ld:a_swap_mode:%d,rgb_swap_mode:%d,va_word_endn:%d,va_lng_wrd_endn:%d,uv_word_endn:%d,uv_lng_wrd_endn:%d,y_word_endn:%d,y_lng_wrd_endn:%d\n",
           s_gsp_cfg.layer_des_info.endian_mode.a_swap_mode,
            s_gsp_cfg.layer_des_info.endian_mode.rgb_swap_mode,
            s_gsp_cfg.layer_des_info.endian_mode.va_word_endn,
            s_gsp_cfg.layer_des_info.endian_mode.va_lng_wrd_endn,
            s_gsp_cfg.layer_des_info.endian_mode.uv_word_endn,
           s_gsp_cfg.layer_des_info.endian_mode.uv_lng_wrd_endn,
           s_gsp_cfg.layer_des_info.endian_mode.y_word_endn,
            s_gsp_cfg.layer_des_info.endian_mode.y_lng_wrd_endn);
    printk("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& \n");
}


static void printGPSReg(void)
{
    GSP_REG_T *g_gsp_reg = (GSP_REG_T *)GSP_REG_BASE;

    printk("********************************************* \n");


    printk( "misc: run %d|busy %d|errflag %d|errcode %02d|dither %d|pmmod0 %d|pmmod1 %d|pmen %d|scale %d|reserv2 %d|scl_stat_clr %d|l0en %d|l1en %d|rb %d\n",
            g_gsp_reg->gsp_cfg_u.mBits.gsp_run,
            g_gsp_reg->gsp_cfg_u.mBits.gsp_busy,
            g_gsp_reg->gsp_cfg_u.mBits.error_flag,
            g_gsp_reg->gsp_cfg_u.mBits.error_code,
            g_gsp_reg->gsp_cfg_u.mBits.dither_en,
            g_gsp_reg->gsp_cfg_u.mBits.pmargb_mod0,
            g_gsp_reg->gsp_cfg_u.mBits.pmargb_mod1,
            g_gsp_reg->gsp_cfg_u.mBits.pmargb_en,
            g_gsp_reg->gsp_cfg_u.mBits.scale_en,
            g_gsp_reg->gsp_cfg_u.mBits.reserved2,
            g_gsp_reg->gsp_cfg_u.mBits.scale_status_clr,
           g_gsp_reg->gsp_cfg_u.mBits.l0_en,
            g_gsp_reg->gsp_cfg_u.mBits.l1_en,
            g_gsp_reg->gsp_cfg_u.mBits.dist_rb);
    printk( "misc: inten %d|intmod %d|intclr %d\n",
            g_gsp_reg->gsp_int_cfg_u.mBits.int_en,
            g_gsp_reg->gsp_int_cfg_u.mBits.int_mod,
            g_gsp_reg->gsp_int_cfg_u.mBits.int_clr);


    printk( "L0 cfg:fmt %d|rot %d|ck %d|pallet %d|rowtap %d|coltap %d\n",
            g_gsp_reg->gsp_layer0_cfg_u.mBits.img_format_l0,
            g_gsp_reg->gsp_layer0_cfg_u.mBits.rot_mod_l0,
            g_gsp_reg->gsp_layer0_cfg_u.mBits.ck_en_l0,
            g_gsp_reg->gsp_layer0_cfg_u.mBits.pallet_en_l0,
            g_gsp_reg->gsp_layer0_cfg_u.mBits.row_tap_mod,
            g_gsp_reg->gsp_layer0_cfg_u.mBits.col_tap_mod);
    printk( "L0 blockalpha %03d, pitch %04d,(%04d,%04d)%04dx%04d => (%04d,%04d)%04dx%04d\n",
            g_gsp_reg->gsp_layer0_alpha_u.mBits.alpha_l0,
            g_gsp_reg->gsp_layer0_pitch_u.mBits.pitch0,
            g_gsp_reg->gsp_layer0_clip_start_u.mBits.clip_start_x_l0,
            g_gsp_reg->gsp_layer0_clip_start_u.mBits.clip_start_y_l0,
            g_gsp_reg->gsp_layer0_clip_size_u.mBits.clip_size_x_l0,
            g_gsp_reg->gsp_layer0_clip_size_u.mBits.clip_size_y_l0,
            g_gsp_reg->gsp_layer0_des_start_u.mBits.des_start_x_l0,
            g_gsp_reg->gsp_layer0_des_start_u.mBits.des_start_y_l0,
            g_gsp_reg->gsp_layer0_des_size_u.mBits.des_size_x_l0,
           g_gsp_reg->gsp_layer0_des_size_u.mBits.des_size_y_l0);
    printk( "L0 Yaddr 0x%08x|Uaddr 0x%08x|Vaddr 0x%08x\n",
           g_gsp_reg->gsp_layer0_y_addr_u.dwValue,
            g_gsp_reg->gsp_layer0_uv_addr_u.dwValue,
            g_gsp_reg->gsp_layer0_va_addr_u.dwValue);
    printk( "L0 grey(%03d,%03d,%03d) colorkey(%03d,%03d,%03d)\n",
            g_gsp_reg->gsp_layer0_grey_rgb_u.mBits.layer0_grey_r,
            g_gsp_reg->gsp_layer0_grey_rgb_u.mBits.layer0_grey_g,
            g_gsp_reg->gsp_layer0_grey_rgb_u.mBits.layer0_grey_b,
            g_gsp_reg->gsp_layer0_ck_u.mBits.ck_r_l0,
           g_gsp_reg->gsp_layer0_ck_u.mBits.ck_g_l0,
            g_gsp_reg->gsp_layer0_ck_u.mBits.ck_b_l0);
    printk( "L0 endian: y %d|u %d|v %d|rgb %d|alpha %d\n",
            g_gsp_reg->gsp_layer0_endian_u.mBits.y_endian_mod_l0,
            g_gsp_reg->gsp_layer0_endian_u.mBits.uv_endian_mod_l0,
           g_gsp_reg->gsp_layer0_endian_u.mBits.va_endian_mod_l0,
           g_gsp_reg->gsp_layer0_endian_u.mBits.rgb_swap_mod_l0,
           g_gsp_reg->gsp_layer0_endian_u.mBits.a_swap_mod_l0);


    printk( "L1 cfg:fmt %d|rot %d|ck %d|pallet %d\n",
            g_gsp_reg->gsp_layer1_cfg_u.mBits.img_format_l1,
           g_gsp_reg->gsp_layer1_cfg_u.mBits.rot_mod_l1,
            g_gsp_reg->gsp_layer1_cfg_u.mBits.ck_en_l1,
          g_gsp_reg->gsp_layer1_cfg_u.mBits.pallet_en_l1);
    printk( "L1 blockalpha %03d, pitch %04d,(%04d,%04d)%04dx%04d => (%04d,%04d)\n",
            g_gsp_reg->gsp_layer1_alpha_u.mBits.alpha_l1,
            g_gsp_reg->gsp_layer1_pitch_u.mBits.pitch1,
           g_gsp_reg->gsp_layer1_clip_start_u.mBits.clip_start_x_l1,
            g_gsp_reg->gsp_layer1_clip_start_u.mBits.clip_start_y_l1,
           g_gsp_reg->gsp_layer1_clip_size_u.mBits.clip_size_x_l1,
           g_gsp_reg->gsp_layer1_clip_size_u.mBits.clip_size_y_l1,
           g_gsp_reg->gsp_layer1_des_start_u.mBits.des_start_x_l1,
            g_gsp_reg->gsp_layer1_des_start_u.mBits.des_start_y_l1);
   printk( "L1 Yaddr 0x%08x|Uaddr 0x%08x|Vaddr 0x%08x\n",
            g_gsp_reg->gsp_layer1_y_addr_u.dwValue,
           g_gsp_reg->gsp_layer1_uv_addr_u.dwValue,
           g_gsp_reg->gsp_layer1_va_addr_u.dwValue);
   printk( "L1 grey(%03d,%03d,%03d) colorkey(%03d,%03d,%03d)\n",
           g_gsp_reg->gsp_layer1_grey_rgb_u.mBits.grey_r_l1,
            g_gsp_reg->gsp_layer1_grey_rgb_u.mBits.grey_g_l1,
           g_gsp_reg->gsp_layer1_grey_rgb_u.mBits.grey_b_l1,
          g_gsp_reg->gsp_layer1_ck_u.mBits.ck_r_l1,
           g_gsp_reg->gsp_layer1_ck_u.mBits.ck_g_l1,
            g_gsp_reg->gsp_layer1_ck_u.mBits.ck_b_l1);
    printk( "L1 endian: y %d|u %d|v %d|rgb %d|alpha %d\n",
            g_gsp_reg->gsp_layer1_endian_u.mBits.y_endian_mod_l1,
           g_gsp_reg->gsp_layer1_endian_u.mBits.uv_endian_mod_l1,
            g_gsp_reg->gsp_layer1_endian_u.mBits.va_endian_mod_l1,
            g_gsp_reg->gsp_layer1_endian_u.mBits.rgb_swap_mod_l1,
            g_gsp_reg->gsp_layer1_endian_u.mBits.a_swap_mod_l1);


   printk( "Ld cfg:fmt %d|cmpr8 %d|pitch %04d\n",
            g_gsp_reg->gsp_des_data_cfg_u.mBits.des_img_format,
            g_gsp_reg->gsp_des_data_cfg_u.mBits.compress_r8,
            g_gsp_reg->gsp_des_pitch_u.mBits.des_pitch);
   printk( "Ld Yaddr 0x%08x|Uaddr 0x%08x|Vaddr 0x%08x\n",
           g_gsp_reg->gsp_des_y_addr_u.dwValue,
           g_gsp_reg->gsp_des_uv_addr_u.dwValue,
            g_gsp_reg->gsp_des_v_addr_u.dwValue);
    printk( "Ld endian: y %d|u %d|v %d|rgb %d|alpha %d\n",
            g_gsp_reg->gsp_des_data_endian_u.mBits.y_endian_mod,
            g_gsp_reg->gsp_des_data_endian_u.mBits.uv_endian_mod,
           g_gsp_reg->gsp_des_data_endian_u.mBits.v_endian_mod,
           g_gsp_reg->gsp_des_data_endian_u.mBits.rgb_swap_mod,
           g_gsp_reg->gsp_des_data_endian_u.mBits.a_swap_mod);
   printk( "********************************************* \n");
}
#ifdef GSP_WORK_AROUND1
const int32_t g_half_pi_cos[8]= {0,0,-1,0,1,0,-1,0};
#define half_pi_cos(r)  (g_half_pi_cos[(r)+4])
const int32_t g_half_pi_sin[8]= {0,1,0,-1,0,1,0,-1};
#define half_pi_sin(r)  (g_half_pi_sin[(r)+4])

static uint32_t GSP_point_in_layer0(uint32_t x,uint32_t y)
{
    GSP_RECT_T des_rect0;
    des_rect0 = s_gsp_cfg.layer0_info.des_rect;

    if(des_rect0.st_x <= x
       && des_rect0.st_y <= y
       && (des_rect0.st_x+des_rect0.rect_w-1) >= x
       && (des_rect0.st_y+des_rect0.rect_h-1) >= y)
    {
        return 1;
    }
    return 0;
}

static uint32_t GSP_point_in_layer1(uint32_t x,uint32_t y)
{
    GSP_RECT_T des_rect1;

    des_rect1.st_x = s_gsp_cfg.layer1_info.des_pos.pos_pt_x;
    des_rect1.st_y = s_gsp_cfg.layer1_info.des_pos.pos_pt_y;
    if(s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90_M
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270_M)
    {
        des_rect1.rect_w = s_gsp_cfg.layer1_info.clip_rect.rect_h;
        des_rect1.rect_h = s_gsp_cfg.layer1_info.clip_rect.rect_w;
    }
    else
    {
        des_rect1.rect_w = s_gsp_cfg.layer1_info.clip_rect.rect_w;
        des_rect1.rect_h = s_gsp_cfg.layer1_info.clip_rect.rect_h;
    }

    if(des_rect1.st_x <= x
       && des_rect1.st_y <= y
       && (des_rect1.st_x+des_rect1.rect_w-1) >= x
       && (des_rect1.st_y+des_rect1.rect_h-1) >= y)
    {
        return 1;
    }
    return 0;
}


static uint32_t GSP_get_points_in_layer0(void)
{
    uint32_t cnt = 0;
    GSP_RECT_T des_rect1;


    des_rect1.st_x = s_gsp_cfg.layer1_info.des_pos.pos_pt_x;
    des_rect1.st_y = s_gsp_cfg.layer1_info.des_pos.pos_pt_y;
	    if(s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90
            ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270
            ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90_M
            ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270_M)
    {
        des_rect1.rect_w = s_gsp_cfg.layer1_info.clip_rect.rect_h;
        des_rect1.rect_h = s_gsp_cfg.layer1_info.clip_rect.rect_w;
    }
    else
    {
        des_rect1.rect_w = s_gsp_cfg.layer1_info.clip_rect.rect_w;
        des_rect1.rect_h = s_gsp_cfg.layer1_info.clip_rect.rect_h;
    }

    cnt += GSP_point_in_layer0(des_rect1.st_x,des_rect1.st_y);
    cnt += GSP_point_in_layer0(des_rect1.st_x+des_rect1.rect_w-1,des_rect1.st_y);
    cnt += GSP_point_in_layer0(des_rect1.st_x,des_rect1.st_y+des_rect1.rect_h-1);
    cnt += GSP_point_in_layer0(des_rect1.st_x+des_rect1.rect_w-1,des_rect1.st_y+des_rect1.rect_h-1);
    return cnt;
}
static uint32_t GSP_get_points_in_layer1(void)
{
    uint32_t cnt = 0;
    GSP_RECT_T des_rect0;
    des_rect0 = s_gsp_cfg.layer0_info.des_rect;

    cnt += GSP_point_in_layer1(des_rect0.st_x,des_rect0.st_y);
    cnt += GSP_point_in_layer1(des_rect0.st_x+des_rect0.rect_w-1,des_rect0.st_y);
    cnt += GSP_point_in_layer1(des_rect0.st_x,des_rect0.st_y+des_rect0.rect_h-1);
    cnt += GSP_point_in_layer1(des_rect0.st_x+des_rect0.rect_w-1,des_rect0.st_y+des_rect0.rect_h-1);
    return cnt;
}

static uint32_t GSP_layer0_layer1_overlap(void)
{
    GSP_RECT_T des_rect0;
    GSP_RECT_T des_rect1;

    des_rect0 = s_gsp_cfg.layer0_info.des_rect;
    des_rect1.st_x = s_gsp_cfg.layer1_info.des_pos.pos_pt_x;
    des_rect1.st_y = s_gsp_cfg.layer1_info.des_pos.pos_pt_y;
    des_rect1.rect_w = s_gsp_cfg.layer1_info.clip_rect.rect_w;
    des_rect1.rect_h = s_gsp_cfg.layer1_info.clip_rect.rect_h;
    if(s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90_M
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270_M)
    {
        des_rect1.rect_w = s_gsp_cfg.layer1_info.clip_rect.rect_h;
        des_rect1.rect_h = s_gsp_cfg.layer1_info.clip_rect.rect_w;
    }
    else
    {
        des_rect1.rect_w = s_gsp_cfg.layer1_info.clip_rect.rect_w;
        des_rect1.rect_h = s_gsp_cfg.layer1_info.clip_rect.rect_h;
    }

    
    if(((des_rect1.st_x+des_rect1.rect_w-1) < des_rect0.st_x)
       ||((des_rect1.st_y+des_rect1.rect_h-1) < des_rect0.st_y))
    {
        return 0;
    }

    
    if(des_rect1.st_x > (des_rect0.st_x + des_rect0.rect_w - 1)
       ||des_rect1.st_y > (des_rect0.st_y + des_rect0.rect_h - 1))
    {
        return 0;
    }


    return 1;
}


#ifndef TRANSLATION_CALC_OPT
static void Matrix3x3SetIdentity(Matrix33 m)
{
    uint32_t r = 0;
    uint32_t c = 0;

    for(r=0; r<3; r++)
    {
        for(c=0; c<3; c++)
        {
            m[r][c]= (r==c);
        }
    }
}

static void MatrixMul33(Matrix33 m1,Matrix33 m2)
{
    uint32_t r = 0;
    uint32_t c = 0;
    Matrix33 tm;

    for(r=0; r<3; r++)
    {
        for(c=0; c<3; c++)
        {
            tm[r][c]=m1[r][0]*m2[0][c]+m1[r][1]*m2[1][c]+m1[r][2]*m2[2][c];
        }
    }

    for(r=0; r<3; r++)
    {
        for(c=0; c<3; c++)
        {
            m2[r][c]=tm[r][c];
        }
    }
}


static void MatrixMul31(Matrix33 m1,Matrix31 m2)
{
    uint32_t r = 0;
    Matrix31 tm;

    for(r=0; r<3; r++)
    {
        tm[r]=m1[r][0]*m2[0]+m1[r][1]*m2[1]+m1[r][2]*m2[2];
    }

    for(r=0; r<3; r++)
    {
        m2[r]=tm[r];
    }
}

#endif
static uint32_t GSP_Gen_CMDQ(GSP_LAYER1_REG_T *pDescriptors_walk,GSP_L1_L0_RELATIONSHIP_E relation)
{
    uint32_t part_cnt = 0;
    uint32_t i = 0;
    int32_t am_flag = 1;
    uint32_t L1_des_w = 0;
    uint32_t L1_des_h = 0;
    int32_t L1_anti_rotate = 0;
    int32_t L1_top_left_x = 4095;
    int32_t L1_top_left_y = 4095;
    GSP_RECT_T rect;
    GSP_POS_PT_T pos;

    PART_POINTS L0_des_Points = {0};
    PART_POINTS L1_des_Points = {0};
    PART_POINTS parts_Points_des[PARTS_CNT_MAX] = {{0}};
    PART_POINTS parts_Points_src[PARTS_CNT_MAX] = {{0}};

#ifdef TRANSLATION_CALC_OPT
    PART_POINTS parts_Points_des_co[PARTS_CNT_MAX] = {{0}};
    PART_POINTS parts_Points_des_am[PARTS_CNT_MAX] = {{0}};
    PART_POINTS parts_Points_des_ar[PARTS_CNT_MAX] = {{0}};
#else

    PART_POINTS parts_Points_des_ar_matrix[PARTS_CNT_MAX] = {{0}};
    PART_POINTS parts_Points_src_matrix[PARTS_CNT_MAX] = {{0}};

    Matrix33 matrix_t;
    Matrix33 matrix_am;
    Matrix33 matrix_ar;
    Matrix33 matrix_cmp;
    Matrix33 matrix_map_t;
    Matrix31 m_point0 = {0,0,1};
    Matrix31 m_point1 = {0,0,1};

    Matrix3x3SetIdentity(matrix_t);
    Matrix3x3SetIdentity(matrix_am);
    Matrix3x3SetIdentity(matrix_ar);
    Matrix3x3SetIdentity(matrix_cmp);
    Matrix3x3SetIdentity(matrix_map_t);
#endif



    if(s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90_M
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270_M)
    {
        L1_des_w = s_gsp_cfg.layer1_info.clip_rect.rect_h;
        L1_des_h = s_gsp_cfg.layer1_info.clip_rect.rect_w;

    }
    else
    {
        L1_des_w = s_gsp_cfg.layer1_info.clip_rect.rect_w;
        L1_des_h = s_gsp_cfg.layer1_info.clip_rect.rect_h;
    }


    L0_des_Points.st_x = s_gsp_cfg.layer0_info.des_rect.st_x;
    L0_des_Points.st_y = s_gsp_cfg.layer0_info.des_rect.st_y;
    L0_des_Points.end_x = L0_des_Points.st_x + s_gsp_cfg.layer0_info.des_rect.rect_w - 1;
    L0_des_Points.end_y = L0_des_Points.st_y + s_gsp_cfg.layer0_info.des_rect.rect_h - 1;

    L1_des_Points.st_x = s_gsp_cfg.layer1_info.des_pos.pos_pt_x;
    L1_des_Points.st_y = s_gsp_cfg.layer1_info.des_pos.pos_pt_y;
    L1_des_Points.end_x = L1_des_Points.st_x + L1_des_w - 1;
    L1_des_Points.end_y = L1_des_Points.st_y + L1_des_h - 1;

    
    GSP_TRACE("%s%d:relation==%d\n",__func__,__LINE__,relation);
    switch(relation)
    {
        case L1_L0_RELATIONSHIP_AQ:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.st_y - 1;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.end_y + 1;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.st_x - 1;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.end_x + 1;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            i++;
            parts_Points_des[i] = L0_des_Points;
            part_cnt = i;
        }
        break;
        case L1_L0_RELATIONSHIP_AD:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.st_y - 1;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.st_x - 1;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            part_cnt = i;
        }
        break;
        case L1_L0_RELATIONSHIP_AC:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.st_y - 1;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.st_x - 1;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.end_x + 1;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            part_cnt = i;
        }
        break;
        case L1_L0_RELATIONSHIP_AO:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.st_y - 1;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.st_x - 1;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.end_y + 1;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_BP:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.st_y - 1;

        i++;
        parts_Points_des[i].st_x = L1_des_Points.st_x;
        parts_Points_des[i].st_y = L0_des_Points.end_y + 1;
        parts_Points_des[i].end_x = L1_des_Points.end_x;
        parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_CQ:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.st_y - 1;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.end_x + 1;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.end_y + 1;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_CF:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.st_y - 1;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.end_x + 1;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_BE:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.st_y - 1;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_GK:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.st_x - 1;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.end_x + 1;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_OQ:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.st_x - 1;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.end_x + 1;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.end_y + 1;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_LO:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.st_x - 1;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.end_y + 1;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_GH:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.st_x - 1;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L0_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_NQ:
        {
            i = 0;
            parts_Points_des[i].st_x = L0_des_Points.end_x + 1;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.end_y + 1;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_JK:
        {
            i = 0;
            parts_Points_des[i].st_x = L0_des_Points.end_x + 1;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L0_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            part_cnt = i;

        }
        break;
        case L1_L0_RELATIONSHIP_MP:
        {
            i = 0;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L0_des_Points.end_y + 1;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L1_des_Points.end_y;

            i++;
            parts_Points_des[i].st_x = L1_des_Points.st_x;
            parts_Points_des[i].st_y = L1_des_Points.st_y;
            parts_Points_des[i].end_x = L1_des_Points.end_x;
            parts_Points_des[i].end_y = L0_des_Points.end_y;

            part_cnt = i;

        }
        break;
        default:
        {

        }
        break;
    }
    part_cnt = i+1;

#ifdef TRANSLATION_CALC_OPT
    
    i = 0;
    while(i < part_cnt)
    {
        parts_Points_des_co[i].st_x = parts_Points_des[i].st_x - L1_des_Points.st_x;
        parts_Points_des_co[i].st_y = parts_Points_des[i].st_y - L1_des_Points.st_y;
        parts_Points_des_co[i].end_x = parts_Points_des[i].end_x - L1_des_Points.st_x;
        parts_Points_des_co[i].end_y = parts_Points_des[i].end_y - L1_des_Points.st_y;
        i++;
    }

    
    if(s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_0_M
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90_M
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_180_M
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270_M)
    {
        am_flag = -1;
    }
    i = 0;
    while(i < part_cnt)
    {
        parts_Points_des_am[i].st_x = parts_Points_des_co[i].st_x;
        parts_Points_des_am[i].st_y = am_flag * parts_Points_des_co[i].st_y;
        parts_Points_des_am[i].end_x = parts_Points_des_co[i].end_x;
        parts_Points_des_am[i].end_y = am_flag * parts_Points_des_co[i].end_y;
        i++;
    }

    
    i = 0;
    L1_anti_rotate = s_gsp_cfg.layer1_info.rot_angle;
    L1_anti_rotate &= 0x3;
    while(i < part_cnt)
    {
        parts_Points_des_ar[i].st_x = parts_Points_des_am[i].st_x * half_pi_cos(L1_anti_rotate) - parts_Points_des_am[i].st_y * half_pi_sin(L1_anti_rotate);
        parts_Points_des_ar[i].st_y = parts_Points_des_am[i].st_x * half_pi_sin(L1_anti_rotate) + parts_Points_des_am[i].st_y * half_pi_cos(L1_anti_rotate);
        parts_Points_des_ar[i].end_x = parts_Points_des_am[i].end_x * half_pi_cos(L1_anti_rotate) - parts_Points_des_am[i].end_y * half_pi_sin(L1_anti_rotate);
        parts_Points_des_ar[i].end_y = parts_Points_des_am[i].end_x * half_pi_sin(L1_anti_rotate) + parts_Points_des_am[i].end_y * half_pi_cos(L1_anti_rotate);

        
        L1_top_left_x = (L1_top_left_x > parts_Points_des_ar[i].st_x)?parts_Points_des_ar[i].st_x:L1_top_left_x;
        L1_top_left_x = (L1_top_left_x > parts_Points_des_ar[i].end_x)?parts_Points_des_ar[i].end_x:L1_top_left_x;
        L1_top_left_y = (L1_top_left_y > parts_Points_des_ar[i].st_y)?parts_Points_des_ar[i].st_y:L1_top_left_y;
        L1_top_left_y = (L1_top_left_y > parts_Points_des_ar[i].end_y)?parts_Points_des_ar[i].end_y:L1_top_left_y;

        i++;
    }

    
    i = 0;
    while(i < part_cnt)
    {
        parts_Points_src[i].st_x = parts_Points_des_ar[i].st_x - L1_top_left_x + s_gsp_cfg.layer1_info.clip_rect.st_x;
        parts_Points_src[i].st_y = parts_Points_des_ar[i].st_y - L1_top_left_y + s_gsp_cfg.layer1_info.clip_rect.st_y;
        parts_Points_src[i].end_x = parts_Points_des_ar[i].end_x - L1_top_left_x + s_gsp_cfg.layer1_info.clip_rect.st_x;
        parts_Points_src[i].end_y = parts_Points_des_ar[i].end_y - L1_top_left_y + s_gsp_cfg.layer1_info.clip_rect.st_y;
        i++;
    }

    i = 0;
    while(i < part_cnt)
    {
        GSP_GET_RECT_FROM_PART_POINTS(parts_Points_src[i],rect);
        GSP_L1_CLIPRECT_SET(rect);
        pos.pos_pt_x = (parts_Points_des[i].st_x&(~0x1));
        pos.pos_pt_y = (parts_Points_des[i].st_y&(~0x1));
        GSP_L1_DESPOS_SET(pos);

        pDescriptors_walk[i] = *(volatile GSP_LAYER1_REG_T *)GSP_L1_BASE;
        i++;
    }
#else
    matrix_t[0][2] = -s_gsp_cfg.layer1_info.des_pos.pos_pt_x;
    matrix_t[1][2] = -s_gsp_cfg.layer1_info.des_pos.pos_pt_y;

    if(s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_0_M
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90_M
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_180_M
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270_M)
    {
        matrix_am[1][1] = -1;
    }

    L1_anti_rotate = s_gsp_cfg.layer1_info.rot_angle;
    L1_anti_rotate &= 0x3;
    matrix_ar[0][0] = half_pi_cos(L1_anti_rotate);
    matrix_ar[0][1] = -half_pi_sin(L1_anti_rotate);
    matrix_ar[1][1] = half_pi_cos(L1_anti_rotate);
    matrix_ar[1][0] = matrix_ar[0][1];

    MatrixMul33(matrix_t, matrix_cmp);
    MatrixMul33(matrix_am, matrix_cmp);
    MatrixMul33(matrix_ar, matrix_cmp);

    i = 0;
    while(i < part_cnt)
    {
        m_point0[0] = parts_Points_des[i].st_x;
        m_point0[1] = parts_Points_des[i].st_y;
        m_point1[0] = parts_Points_des[i].end_x;
        m_point1[1] = parts_Points_des[i].end_y;


        MatrixMul31(matrix_cmp, m_point0);
        parts_Points_des_ar_matrix[i].st_x = m_point0[0];
        parts_Points_des_ar_matrix[i].st_y = m_point0[1];
        MatrixMul31(matrix_cmp, m_point1);
        parts_Points_des_ar_matrix[i].end_x = m_point1[0];
        parts_Points_des_ar_matrix[i].end_y = m_point1[1];

        L1_top_left_x = MIN(L1_top_left_x, MIN(parts_Points_des_ar_matrix[i].st_x, parts_Points_des_ar_matrix[i].end_x));
        L1_top_left_y = MIN(L1_top_left_y, MIN(parts_Points_des_ar_matrix[i].st_y, parts_Points_des_ar_matrix[i].end_y));
        i++;
    }

    
    matrix_map_t[0][2] = - L1_top_left_x + s_gsp_cfg.layer1_info.clip_rect.st_x;
    matrix_map_t[1][2] = - L1_top_left_y + s_gsp_cfg.layer1_info.clip_rect.st_y;

    i = 0;
    while(i < part_cnt)
    {
        m_point0[0] = parts_Points_des_ar_matrix[i].st_x;
        m_point0[1] = parts_Points_des_ar_matrix[i].st_y;
        m_point1[0] = parts_Points_des_ar_matrix[i].end_x;
        m_point1[1] = parts_Points_des_ar_matrix[i].end_y;

        MatrixMul31(matrix_map_t,m_point0);
        parts_Points_src_matrix[i].st_x = m_point0[0];
        parts_Points_src_matrix[i].st_y = m_point0[1];
        MatrixMul31(matrix_map_t, m_point1);
        parts_Points_src_matrix[i].end_x = m_point1[0];
        parts_Points_src_matrix[i].end_y = m_point1[1];
        i++;
    }
    i = 0;
    while(i < part_cnt)
    {
        GSP_GET_RECT_FROM_PART_POINTS(parts_Points_src_matrix[i],rect);
        GSP_L1_CLIPRECT_SET(rect);
        pos.pos_pt_x = (parts_Points_des[i].st_x&(~0x1));
        pos.pos_pt_y = (parts_Points_des[i].st_y&(~0x1));
        GSP_L1_DESPOS_SET(pos);

        pDescriptors_walk[i] = *(volatile GSP_LAYER1_REG_T *)GSP_L1_BASE;
        i++;
    }
#endif
    return part_cnt-1;
}

void printCMDQ(GSP_LAYER1_REG_T *pDescriptors, uint32_t c)
{
    uint32_t i = 0;
    while(i < c)
    {
       printk("CMDQ[%d]cfg:%08x\n",i,pDescriptors->gsp_layer1_cfg_u.dwValue);
       printk("CMDQ[%d]YADDR:%08x\n",i,pDescriptors->gsp_layer1_y_addr_u.dwValue);
        printk("CMDQ[%d]UADDR:%08x\n",i,pDescriptors->gsp_layer1_uv_addr_u.dwValue);
        printk("CMDQ[%d]VADDR:%08x\n",i,pDescriptors->gsp_layer1_va_addr_u.dwValue);
        printk("CMDQ[%d]Pitch:%08x\n",i,pDescriptors->gsp_layer1_pitch_u.dwValue);
        printk("CMDQ[%d]clip x:%04d y:%04d\n",i,pDescriptors->gsp_layer1_clip_start_u.mBits.clip_start_x_l1,pDescriptors->gsp_layer1_clip_start_u.mBits.clip_start_y_l1);
        printk("CMDQ[%d]clip w:%04d h:%04d\n",i,pDescriptors->gsp_layer1_clip_size_u.mBits.clip_size_x_l1,pDescriptors->gsp_layer1_clip_size_u.mBits.clip_size_y_l1);
       printk("CMDQ[%d]des x:%04d y:%04d\n",i,pDescriptors->gsp_layer1_des_start_u.mBits.des_start_x_l1,pDescriptors->gsp_layer1_des_start_u.mBits.des_start_y_l1);
        printk("CMDQ[%d]GREY:%08x\n",i,pDescriptors->gsp_layer1_grey_rgb_u.dwValue);
        printk("CMDQ[%d]Endian:%08x\n",i,pDescriptors->gsp_layer1_endian_u.dwValue);
        printk("CMDQ[%d]Alpha:%08x\n",i,pDescriptors->gsp_layer1_alpha_u.dwValue);
       printk("CMDQ[%d]CK:%08x\n",i,pDescriptors->gsp_layer1_ck_u.dwValue);
        i++;
        pDescriptors++;
    }
}

#define L0_CFG_PREPARE_FOR_CMDQ()\
{\
    GSP_RECT_T clip_rect = {0,0,0,4095};\
    clip_rect.rect_w = s_gsp_cfg.layer_des_info.pitch;\
    GSP_L0_CLIPRECT_SET(clip_rect);\
    GSP_L0_DESRECT_SET(clip_rect);\
    GSP_L0_ADDR_SET(s_gsp_cfg.layer_des_info.src_addr);\
    GSP_L0_PITCH_SET(s_gsp_cfg.layer_des_info.pitch);\
    GSP_L0_IMGFORMAT_SET(s_gsp_cfg.layer_des_info.img_format);\
    GSP_L0_ENABLE_SET(0)


#define L0_CFG_RESTORE_FROM_CMDQ()\
    GSP_L0_ADDR_SET(s_gsp_cfg.layer0_info.src_addr);\
    GSP_L0_PITCH_SET(s_gsp_cfg.layer0_info.pitch);\
    GSP_L0_CLIPRECT_SET(s_gsp_cfg.layer0_info.clip_rect);\
    GSP_L0_DESRECT_SET(s_gsp_cfg.layer0_info.des_rect);\
    GSP_L0_IMGFORMAT_SET(s_gsp_cfg.layer0_info.img_format);\
    GSP_L0_ENABLE_SET(s_gsp_cfg.layer0_info.layer_en);\
}

static int32_t GSP_work_around1(gsp_user* pUserdata)
{
    uint32_t i=0;
    uint32_t blockcnt=0;

    uint32_t L1inL0cnt=0;
    uint32_t L0inL1cnt=0;
    int32_t ret = GSP_NO_ERR;
    GSP_L1_L0_RELATIONSHIP_E relation = L1_L0_RELATIONSHIP_I;
    uint32_t L1_des_w = 0;
    uint32_t L1_des_h = 0;
    PART_POINTS L0_des_Points = {0};
    PART_POINTS L1_des_Points = {0};
    uint32_t descriptors_byte_len = 0;
    static GSP_LAYER1_REG_T *pDescriptors = NULL;
    dma_addr_t      Descriptors_pa = 0;
    GSP_CMDQ_REG_T CmdQCfg;

    
    if(s_gsp_cfg.layer1_info.layer_en == 0
       ||s_gsp_cfg.layer0_info.layer_en == 0)
    {
        return GSP_NO_ERR;
    }
    memset(&CmdQCfg,0,sizeof(CmdQCfg));

    if(s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_90_M
       ||s_gsp_cfg.layer1_info.rot_angle == GSP_ROT_ANGLE_270_M)
    {
        
        if(!(s_gsp_cfg.layer0_info.clip_rect.rect_w >= s_gsp_cfg.layer0_info.des_rect.rect_h*2
                ||s_gsp_cfg.layer0_info.clip_rect.rect_h >= s_gsp_cfg.layer0_info.des_rect.rect_w*2))
        {
            return GSP_NO_ERR;
        }
        L1_des_w = s_gsp_cfg.layer1_info.clip_rect.rect_h;
        L1_des_h = s_gsp_cfg.layer1_info.clip_rect.rect_w;

    }
    else
    {
        
        if(!(s_gsp_cfg.layer0_info.clip_rect.rect_w >= s_gsp_cfg.layer0_info.des_rect.rect_w*2
                ||s_gsp_cfg.layer0_info.clip_rect.rect_h >= s_gsp_cfg.layer0_info.des_rect.rect_h*2))
        {
            return GSP_NO_ERR;
        }
        L1_des_w = s_gsp_cfg.layer1_info.clip_rect.rect_w;
        L1_des_h = s_gsp_cfg.layer1_info.clip_rect.rect_h;
    }


    L0_des_Points.st_x = s_gsp_cfg.layer0_info.des_rect.st_x;
    L0_des_Points.st_y = s_gsp_cfg.layer0_info.des_rect.st_y;
    L0_des_Points.end_x = L0_des_Points.st_x + s_gsp_cfg.layer0_info.des_rect.rect_w - 1;
    L0_des_Points.end_y = L0_des_Points.st_y + s_gsp_cfg.layer0_info.des_rect.rect_h - 1;

    L1_des_Points.st_x = s_gsp_cfg.layer1_info.des_pos.pos_pt_x;
    L1_des_Points.st_y = s_gsp_cfg.layer1_info.des_pos.pos_pt_y;
    L1_des_Points.end_x = L1_des_Points.st_x + L1_des_w - 1;
    L1_des_Points.end_y = L1_des_Points.st_y + L1_des_h - 1;



    
    if(GSP_layer0_layer1_overlap() == 0)
    {
        return GSP_NO_ERR;
    }

    
    L1inL0cnt = GSP_get_points_in_layer0();
    if(L1inL0cnt == 4)
    {
        return GSP_NO_ERR;
    }

    
    
    L0inL1cnt = GSP_get_points_in_layer1();
    if(L1_des_Points.st_x < L0_des_Points.st_x && L1_des_Points.st_y < L0_des_Points.st_y)
    {
        if(L1inL0cnt == 1)
        {
            relation = L1_L0_RELATIONSHIP_AD;
        }
        else if(L1_des_Points.end_x > L0_des_Points.end_x
                && L1_des_Points.end_y <= L0_des_Points.end_y)
        {
            relation = L1_L0_RELATIONSHIP_AC;
        }
        else if(L1_des_Points.end_x <= L0_des_Points.end_x
                && L1_des_Points.end_y > L0_des_Points.end_y)
        {
            relation = L1_L0_RELATIONSHIP_AO;
        }
        else
        {
            relation = L1_L0_RELATIONSHIP_AQ;
        }

    }
    else if(L0_des_Points.st_x <= L1_des_Points.st_x && L1_des_Points.st_x <= L0_des_Points.end_x
            && L1_des_Points.st_y < L0_des_Points.st_y)
    {
        if(L1inL0cnt == 0)
        {
            
            if(L1_des_Points.end_x <= L0_des_Points.end_x)
            {
                relation = L1_L0_RELATIONSHIP_BP;
            }
            else
            {
                relation = L1_L0_RELATIONSHIP_CQ;
            }
        }
        else if(L1inL0cnt == 1)
        {
            relation = L1_L0_RELATIONSHIP_CF;
        }
        else if(L1inL0cnt == 2)
        {
            relation = L1_L0_RELATIONSHIP_BE;
        }
    }
    else if(L0_des_Points.st_y <= L1_des_Points.st_y && L1_des_Points.st_y <= L0_des_Points.end_y
            && L1_des_Points.st_x < L0_des_Points.st_x)
    {
        if(L1inL0cnt == 0)
        {
            
            if(L1_des_Points.end_y <= L0_des_Points.end_y)
            {
                relation = L1_L0_RELATIONSHIP_GK;
            }
            else
            {
                relation = L1_L0_RELATIONSHIP_OQ;
            }
        }
        else if(L1inL0cnt == 1)
        {
            relation = L1_L0_RELATIONSHIP_LO;
        }
        else if(L1inL0cnt == 2)
        {
            relation = L1_L0_RELATIONSHIP_GH;
        }
    }
    else
    {
        if(L1inL0cnt == 1)
        {
            relation = L1_L0_RELATIONSHIP_NQ;
        }
        else if(L1_des_Points.end_x > L0_des_Points.end_x)
        {
            relation = L1_L0_RELATIONSHIP_JK;
        }
        else
        {
            relation = L1_L0_RELATIONSHIP_MP;
        }
    }


    
    descriptors_byte_len = sizeof(GSP_LAYER1_REG_T) * CMDQ_DPT_DEPTH;
	if(pDescriptors == NULL)
    {
    pDescriptors = (GSP_LAYER1_REG_T *)dma_alloc_coherent(NULL,
                   descriptors_byte_len,
                   &Descriptors_pa,
                   GFP_KERNEL|GFP_DMA);
	 if(pDescriptors)
        {
           printk("GSP_work_around1%d:pid:0x%08x, alloced CMDQ descriptor memory,va:0x%08x pa:0x%08x size:%d B \n",__LINE__,pUserdata->pid,
                   (unsigned int)pDescriptors,Descriptors_pa,descriptors_byte_len);
        }
   }
    if (!pDescriptors)
    {
        printk("GSP_work_around1:pid:0x%08x, can't alloc CMDQ descriptor memory!! just disable L1 !! L%d \n",pUserdata->pid,__LINE__);
        GSP_L1_ENABLE_SET(0);
        return GSP_KERNEL_WORKAROUND_ALLOC_ERR;
    }
	memset((void*)pDescriptors,0,descriptors_byte_len);
    blockcnt = GSP_Gen_CMDQ(pDescriptors,relation);

    
    if(0)
    {
        
        (*(uint32_t*)&CmdQCfg.gsp_cmd_addr_u) = (uint32_t)Descriptors_pa;
        CmdQCfg.gsp_cmd_cfg_u.mBits.cmd_num = blockcnt;
    CmdQCfg.gsp_cmd_cfg_u.mBits.cmd_en = 1;

    
    L0_CFG_PREPARE_FOR_CMDQ();
    GSP_L1_CMDQ_SET(CmdQCfg);

    ret = GSP_Trigger();
    GSP_TRACE("%s:pid:0x%08x, trigger %s!, L%d \n",__func__,pUserdata->pid,(ret)?"failed":"success",__LINE__);
    if(ret)
        {
            printk("%sL%d:triger failed, err_record[%d],pid:0x%08x, disable Layer1 !!!!!!\n",__func__,__LINE__,ERR_RECORD_INDEX_GET_WP(),pUserdata->pid);

            ERR_RECORD_ADD(*(GSP_REG_T *)GSP_REG_BASE);
            ERR_RECORD_INDEX_ADD_WP();
            GSP_L1_ENABLE_SET(0);
            goto cmdQ_exit;
        }

    
        ret = down_timeout(&gsp_wait_interrupt_sem,30);
    if (ret == 0)
    {
        GSP_TRACE("%s:pid:0x%08x, wait done sema success, L%d \n",__func__,pUserdata->pid,__LINE__);
    }
    else if (ret == -ETIME)
    {
         printk("%s:pid:0x%08x, wait done sema 30-jiffies-timeout,it's abnormal!!!!!!!! L%d \n",__func__,pUserdata->pid,__LINE__);
            ret = GSP_KERNEL_WORKAROUND_WAITDONE_TIMEOUT;
    }
    else if (ret)
    {
        printk("%s:pid:0x%08x, wait done sema interrupted by a signal, L%d \n",__func__,pUserdata->pid,__LINE__);
            ret = GSP_KERNEL_WORKAROUND_WAITDONE_INTR;
    }

    GSP_Wait_Finish();
    GSP_IRQSTATUS_CLEAR();
    GSP_IRQENABLE_SET(GSP_IRQ_TYPE_DISABLE);
    sema_init(&gsp_wait_interrupt_sem,0);

    cmdQ_exit:
        CmdQCfg.gsp_cmd_cfg_u.mBits.cmd_en = 0;
        GSP_L1_CMDQ_SET(CmdQCfg);
        L0_CFG_RESTORE_FROM_CMDQ();
    }
    else 
    {
        GSP_L0_ENABLE_SET(0);
        while(i < blockcnt)
        {
            GSP_CFG_L1_PARAM(pDescriptors[i]);

            ret = GSP_Trigger();
            GSP_TRACE("%s%d:pid:0x%08x, trigger CMDQ[%d] %s!\n",__func__,__LINE__,pUserdata->pid,i,(ret)?"failed":"success");
            if(ret)
            {
                printk("%s%d:pid:0x%08x, CMDQ[%d] trigger err. ignor not overlaped area of Layer1 !! er_code:%d \n",__func__,__LINE__,pUserdata->pid,i,ret);
                printCMDQ(pDescriptors,blockcnt+1);
                printGPSReg();
                ERR_RECORD_ADD(*(GSP_REG_T *)GSP_REG_BASE);
                ERR_RECORD_INDEX_ADD_WP();
                break;
            }

            
            ret = down_timeout(&gsp_wait_interrupt_sem,30);
            if (ret == 0)
            {
                GSP_TRACE("%s%d:pid:0x%08x, wait done sema success \n",__func__,__LINE__,pUserdata->pid);
            }
            else if (ret == -ETIME)
            {
                printk("%s%d:pid:0x%08x, wait done sema 30-jiffies-timeout,it's abnormal!!\n",__func__,__LINE__,pUserdata->pid);
				printk("%s%d:EMC_MATRIX:0x%08x,GSP_GAP:0x%08x,GSP_CLOCK:0x%08x,GSP_AUTO_GATE:0x%08x\n",__func__,__LINE__,
                       (uint32_t)(GSP_REG_READ(GSP_EMC_MATRIX_BASE)&GSP_EMC_MATRIX_BIT),
                       ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.dist_rb,
                       GSP_REG_READ(GSP_CLOCK_BASE)&0x3,
                       (uint32_t)(GSP_REG_READ(GSP_AUTO_GATE_ENABLE_BASE)&GSP_AUTO_GATE_ENABLE_BIT));
                printCfgInfo();
                printGPSReg();
                printk("%s%d:pid:0x%08x, ignor not overlaped area of Layer1 !! \n",__func__,__LINE__,pUserdata->pid);
                ret = GSP_KERNEL_WORKAROUND_WAITDONE_TIMEOUT;
                break;
            }
            else if (ret)
            {
                printk("%s%d:pid:0x%08x, wait done sema interrupted by a signal\n",__func__,__LINE__,pUserdata->pid);
                printk("%s%d:pid:0x%08x, ignor not overlaped area of Layer1 !!\n",__func__,__LINE__,pUserdata->pid);
                ret = GSP_KERNEL_WORKAROUND_WAITDONE_INTR;
                break;
            }

            GSP_Wait_Finish();
            GSP_IRQSTATUS_CLEAR();
            GSP_IRQENABLE_SET(GSP_IRQ_TYPE_DISABLE);
            sema_init(&gsp_wait_interrupt_sem,0);
            i++;
        }
        GSP_L0_ENABLE_SET(s_gsp_cfg.layer0_info.layer_en);
    }
    
    
    GSP_CFG_L1_PARAM(pDescriptors[blockcnt]);

    
    if(0)   
    {
    dma_free_coherent(NULL,
                      descriptors_byte_len,
                      pDescriptors,
                      Descriptors_pa);
    }
    return ret;

}
#endif


static int32_t gsp_drv_open(struct inode *node, struct file *file)
{
    int32_t ret = GSP_NO_ERR;
    gsp_user *pUserdata = NULL;

    GSP_TRACE("gsp_drv_open:pid:0x%08x enter.\n",current->pid);

    pUserdata = (gsp_user *)gsp_get_user(current->pid);

    if (NULL == pUserdata)
    {
        printk("gsp_drv_open:pid:0x%08x user cnt full.\n",current->pid);
        ret = GSP_KERNEL_FULL;
        goto exit;
    }
    GSP_TRACE("gsp_drv_open:pid:0x%08x bf wait open sema.\n",current->pid);
    ret = down_interruptible(&pUserdata->sem_open);
    if(!ret)
    {
        GSP_TRACE("gsp_drv_open:pid:0x%08x  wait open sema success.\n",current->pid);
        file->private_data = pUserdata;
    }
    else
    {
        ret = GSP_KERNEL_OPEN_INTR;
        printk("gsp_drv_open:pid:0x%08x  wait open sema failed.\n",current->pid);
    }

exit:
    return ret;
}



static int32_t gsp_drv_release(struct inode *node, struct file *file)
{
    gsp_user *pUserdata = file->private_data;

    
    GSP_TRACE("gsp_drv_release:pid:0x%08x.\n\n",current->pid);
    if(pUserdata == NULL)
    {
        printk("gsp_drv_release:error--pUserdata is null!, pid-0x%08x \n\n",current->pid);
        return -ENODEV;
    }
    pUserdata->pid = INVALID_USER_ID;
    sema_init(&pUserdata->sem_open, 1);
    file->private_data = NULL;

    return GSP_NO_ERR;
}


ssize_t gsp_drv_write(struct file *file, const char __user * u_data, size_t cnt, loff_t *cnt_ret)
{
    gsp_user* pUserdata = file->private_data;
    if(pUserdata == NULL)
    {
        printk("%s:error--pUserdata is null!, pid-0x%08x \n\n",__func__,current->pid);
        return -ENODEV;
    }

    
    GSP_TRACE("gsp_drv_write:pid:0x%08x.\n",current->pid);

    pUserdata->is_exit_force = 1;

    
    
    
    send_sig(SIGABRT, (struct task_struct *)pUserdata->pid, 0);

    return 1;
}

ssize_t gsp_drv_read(struct file *file, char __user *u_data, size_t cnt, loff_t *cnt_ret)
{
    char rt_word[32]= {0};
    gsp_user* pUserdata = file->private_data;
    if(pUserdata == NULL)
    {
        printk("%s:error--pUserdata is null!, pid-0x%08x \n\n",__func__,current->pid);
        return -ENODEV;
    }

    *cnt_ret = 0;
    *cnt_ret += sprintf(rt_word + *cnt_ret, "gsp read %d\n",cnt);
    return copy_to_user(u_data, (void*)rt_word, (uint32_t)*cnt_ret);
}


static void GSP_Coef_Tap_Convert(uint8_t h_tap,uint8_t v_tap)
{
    switch(h_tap)
    {
        case 8:
            s_gsp_cfg.layer0_info.row_tap_mode = 0;
            break;

        case 6:
            s_gsp_cfg.layer0_info.row_tap_mode = 1;
            break;

        case 4:
            s_gsp_cfg.layer0_info.row_tap_mode = 2;
            break;

        case 2:
            s_gsp_cfg.layer0_info.row_tap_mode = 3;
            break;

        default:
            s_gsp_cfg.layer0_info.row_tap_mode = 0;
            break;
    }

    switch(v_tap)
    {
        case 8:
            s_gsp_cfg.layer0_info.col_tap_mode = 0;
            break;

        case 6:
            s_gsp_cfg.layer0_info.col_tap_mode = 1;
            break;

        case 4:
            s_gsp_cfg.layer0_info.col_tap_mode = 2;
            break;

        case 2:
            s_gsp_cfg.layer0_info.col_tap_mode = 3;
            break;

        default:
            s_gsp_cfg.layer0_info.col_tap_mode = 0;
            break;
    }
    s_gsp_cfg.layer0_info.row_tap_mode &= 0x3;
    s_gsp_cfg.layer0_info.col_tap_mode &= 0x3;
}


static int32_t GSP_Scaling_Coef_Gen_And_Config(uint32_t* force_calc)
{
    uint8_t     h_tap = 8;
    uint8_t     v_tap = 8;
    uint32_t    *tmp_buf = NULL;
    uint32_t    *h_coeff = NULL;
    uint32_t    *v_coeff = NULL;
    uint32_t    coef_factor_w = 0;
    uint32_t    coef_factor_h = 0;
    uint32_t    after_rotate_w = 0;
    uint32_t    after_rotate_h = 0;
    uint32_t    coef_in_w = 0;
    uint32_t    coef_in_h = 0;
    uint32_t    coef_out_w = 0;
    uint32_t    coef_out_h = 0;
    static volatile uint32_t coef_in_w_last = 0;
    static volatile uint32_t coef_in_h_last = 0;
    static volatile uint32_t coef_out_w_last = 0;
    static volatile uint32_t coef_out_h_last = 0;

    if((s_gsp_cfg.layer0_info.clip_rect.rect_w != s_gsp_cfg.layer0_info.des_rect.rect_w) ||
       (s_gsp_cfg.layer0_info.clip_rect.rect_h != s_gsp_cfg.layer0_info.des_rect.rect_h))
    {
        s_gsp_cfg.layer0_info.scaling_en = 1;
    }

    if(s_gsp_cfg.layer0_info.scaling_en == 1)
    {
        if(s_gsp_cfg.layer0_info.des_rect.rect_w < 4
           ||s_gsp_cfg.layer0_info.des_rect.rect_h < 4)
        {
            return GSP_KERNEL_GEN_OUT_RANG;
        }

        if(s_gsp_cfg.layer0_info.rot_angle == GSP_ROT_ANGLE_0
           ||s_gsp_cfg.layer0_info.rot_angle == GSP_ROT_ANGLE_180
           ||s_gsp_cfg.layer0_info.rot_angle == GSP_ROT_ANGLE_0_M
           ||s_gsp_cfg.layer0_info.rot_angle == GSP_ROT_ANGLE_180_M)
        {
            after_rotate_w = s_gsp_cfg.layer0_info.clip_rect.rect_w;
            after_rotate_h = s_gsp_cfg.layer0_info.clip_rect.rect_h;
        }
        else if(s_gsp_cfg.layer0_info.rot_angle == GSP_ROT_ANGLE_90
                ||s_gsp_cfg.layer0_info.rot_angle == GSP_ROT_ANGLE_270
                ||s_gsp_cfg.layer0_info.rot_angle == GSP_ROT_ANGLE_90_M
                ||s_gsp_cfg.layer0_info.rot_angle == GSP_ROT_ANGLE_270_M)
        {
            after_rotate_w = s_gsp_cfg.layer0_info.clip_rect.rect_h;
            after_rotate_h = s_gsp_cfg.layer0_info.clip_rect.rect_w;
        }

        coef_factor_w = CEIL(after_rotate_w,s_gsp_cfg.layer0_info.des_rect.rect_w);
        coef_factor_h = CEIL(after_rotate_h,s_gsp_cfg.layer0_info.des_rect.rect_h);

        if(coef_factor_w > 16 || coef_factor_h > 16)
        {
            return GSP_KERNEL_GEN_OUT_RANG;
        }

        if(coef_factor_w > 8)
        {
            coef_factor_w = 4;
        }
        else if(coef_factor_w > 4)
        {
            coef_factor_w = 2;
        }
        else
        {
            coef_factor_w = 1;
        }

        if(coef_factor_h > 8)
        {
            coef_factor_h = 4;
        }
        else if(coef_factor_h > 4)
        {
            coef_factor_h = 2;
        }
        else
        {
            coef_factor_h= 1;
        }

        coef_in_w = CEIL(after_rotate_w,coef_factor_w);
        coef_in_h = CEIL(after_rotate_h,coef_factor_h);
        coef_out_w = s_gsp_cfg.layer0_info.des_rect.rect_w;
        coef_out_h = s_gsp_cfg.layer0_info.des_rect.rect_h;
        if(*force_calc == 1
		   ||coef_in_w_last != coef_in_w
           || coef_in_h_last != coef_in_h
           || coef_out_w_last != coef_out_w
           || coef_out_h_last != coef_out_h)
        {
            tmp_buf = (uint32_t *)kmalloc(GSP_COEFF_BUF_SIZE, GFP_KERNEL);
            if (NULL == tmp_buf)
            {
                printk("SCALE DRV: No mem to alloc coeff buffer! \n");
                return GSP_KERNEL_GEN_ALLOC_ERR;
            }
            h_coeff = tmp_buf;
            v_coeff = tmp_buf + (GSP_COEFF_COEF_SIZE/4);

            if (!(GSP_Gen_Block_Ccaler_Coef(coef_in_w,
                                            coef_in_h,
                                            coef_out_w,
                                            coef_out_h,
                                            h_tap,
                                            v_tap,
                                            h_coeff,
                                            v_coeff,
                                            tmp_buf + (GSP_COEFF_COEF_SIZE/2),
                                            GSP_COEFF_POOL_SIZE)))
            {
                kfree(tmp_buf);
                printk("GSP DRV: GSP_Gen_Block_Ccaler_Coef error! \n");
                return GSP_KERNEL_GEN_COMMON_ERR;
            }
            GSP_Scale_Coef_Tab_Config(h_coeff,v_coeff);
            coef_in_w_last = coef_in_w;
            coef_in_h_last = coef_in_h;
            coef_out_w_last = coef_out_w;
            coef_out_h_last = coef_out_h;
			*force_calc = 0;
        }

		GSP_Coef_Tap_Convert(h_tap,v_tap);
        GSP_L0_SCALETAPMODE_SET(s_gsp_cfg.layer0_info.row_tap_mode,s_gsp_cfg.layer0_info.col_tap_mode);
        GSP_TRACE("GSP DRV: GSP_Gen_Block_Ccaler_Coef, register: r_tap%d,c_tap%d \n",
               ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_cfg_u.mBits.row_tap_mod,
               ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_cfg_u.mBits.col_tap_mod);
        kfree(tmp_buf);
    }
    return GSP_NO_ERR;
}

static uint32_t GSP_Info_Config(void)
{
    
    if(GSP_WORKSTATUS_GET())
    {
        printk("GSP_Info_Config(): GSP busy when config!!!\n");
        GSP_ASSERT();
    }

    GSP_ConfigLayer(GSP_MODULE_LAYER0);
    GSP_ConfigLayer(GSP_MODULE_LAYER1);
    GSP_ConfigLayer(GSP_MODULE_ID_MAX);
    GSP_ConfigLayer(GSP_MODULE_DST);
    return GSP_ERRCODE_GET();
}

static void GSP_Cache_Flush(void)
{

}

static void GSP_Cache_Invalidate(void)
{

}

static void GSP_Release_HWSema(void)
{
    gsp_user *pTempUserdata = NULL;
    GSP_TRACE("%s:pid:0x%08x, was killed without release GSP hw semaphore, L%d \n",__func__,gsp_cur_client_pid,__LINE__);

    pTempUserdata = (gsp_user *)gsp_get_user(gsp_cur_client_pid);
    pTempUserdata->pid = INVALID_USER_ID;
    sema_init(&pTempUserdata->sem_open, 1);

    GSP_Wait_Finish();
    GSP_Deinit();
    gsp_cur_client_pid = INVALID_USER_ID;
    sema_init(&gsp_wait_interrupt_sem,0);
    up(&gsp_hw_resource_sem);
}


static long gsp_drv_ioctl(struct file *file,
                          uint32_t cmd,
                          unsigned long arg)
{
    int32_t ret = -GSP_NO_ERR;
    uint32_t param_size = _IOC_SIZE(cmd);
    gsp_user* pUserdata = file->private_data;
    struct timespec start_time;
    struct timespec end_time;
    

    GSP_TRACE("%s:pid:0x%08x, io number 0x%x, param_size %d \n",
              __func__,
              pUserdata->pid,
              _IOC_NR(cmd),
              param_size);

    switch (cmd)
    {
        case GSP_IO_SET_PARAM:
        {
            if (param_size)
            {
                GSP_TRACE("%s:pid:0x%08x, bf wait gsp-hw sema, L%d \n",__func__,pUserdata->pid,__LINE__);

                
                if(gsp_cur_client_pid != INVALID_USER_ID)
                {
                    struct pid * __pid = NULL;
                    struct task_struct *__task = NULL;
					pid_t temp_pid = INVALID_USER_ID;

                    GSP_TRACE("%sL%d current:%08x store_pid:0x%08x, \n",__func__,__LINE__,current->pid,gsp_cur_client_pid);
                    
                    temp_pid = gsp_cur_client_pid;
                    __pid = find_get_pid(temp_pid);
                    if(__pid != NULL)
                    {
                        __task = get_pid_task(__pid,PIDTYPE_PID);
                        

                        if(__task != NULL)
                        {
                            if(__task->pid != gsp_cur_client_pid)
                            {
                                GSP_Release_HWSema();
                            }
                        }
                        else
                        {
                            GSP_Release_HWSema();
                        }
                    }
                    else
                    {
                        GSP_Release_HWSema();
                    }
                    
                }


                ret = down_interruptible(&gsp_hw_resource_sem);
                if(ret)
                {
                s_ktime_end = ktime_get();
                printk("%s[%d]:pid:0x%08x, wait gsp-hw sema interrupted by signal,return\n",__func__,__LINE__,pUserdata->pid);
				printk("%s[%d]:pid:0x%08x, GSP_EMC:0x%08x \n",__func__,__LINE__,pUserdata->pid,(GSP_REG_READ(GSP_EMC_MATRIX_BASE) & GSP_EMC_MATRIX_BIT));
                printk("%s[%d]:pid:0x%08x, the gsp-hw sema hold by 0x%08x start:%lld, end:%lld, cost:%lld ns. phase:0x%08x\n",
                       __func__,__LINE__,pUserdata->pid,
                       gsp_cur_client_pid,
                       s_ktime_start.tv64,
                       s_ktime_end.tv64,
                       ktime_to_ns(ktime_sub(s_ktime_end, s_ktime_start)),
                       gsp_cur_client_phase);
                printGPSReg();
                    
                    ret = GSP_KERNEL_CFG_INTR;
                    goto exit;
                }
                GSP_TRACE("%s:pid:0x%08x, wait gsp-hw sema success, L%d \n",__func__,pUserdata->pid,__LINE__);
                gsp_cur_client_pid = pUserdata->pid;
            gsp_cur_client_phase = PHASE_IN_CONFIG;
            s_ktime_start = ktime_get();
                ret=copy_from_user((void*)&s_gsp_cfg, (void*)arg, param_size);
                if(ret)
                {
                    printk("%s:pid:0x%08x, copy_params_from_user failed! \n",__func__,pUserdata->pid);
                    ret = GSP_KERNEL_COPY_ERR;
                    gsp_cur_client_pid = INVALID_USER_ID;
                PHASE_SET_LINE();
                    up(&gsp_hw_resource_sem);
                    goto exit;
                }
                else
                {
                    GSP_TRACE("%s:pid:0x%08x, copy_params_from_user success!, L%d \n",__func__,pUserdata->pid,__LINE__);
                    GSP_Init();

                    
                    if((s_gsp_cfg.layer0_info.layer_en == 1)
                       &&((s_gsp_cfg.layer0_info.clip_rect.rect_w != s_gsp_cfg.layer0_info.des_rect.rect_w) ||
                          (s_gsp_cfg.layer0_info.clip_rect.rect_h != s_gsp_cfg.layer0_info.des_rect.rect_h)))
                    {
                        s_gsp_cfg.layer0_info.scaling_en = 1;
                    }
                    
                    s_gsp_cfg.misc_info.ahb_clock = ahb_clock;
                    s_gsp_cfg.misc_info.gsp_clock = gsp_clock;
                    if(gsp_gap & 0x100)
                    {
                    	s_gsp_cfg.misc_info.gsp_gap = (gsp_gap & 0xff);
                    }
                    else
                    {
                    	gsp_gap = s_gsp_cfg.misc_info.gsp_gap;
                    }

                    ret = GSP_Info_Config();
                    GSP_TRACE("%s:pid:0x%08x, config hw %s!, L%d \n",__func__,pUserdata->pid,(ret>0)?"failed":"success",__LINE__);
                    if(ret)
                    {
                        
                        printCfgInfo();
                        printGPSReg();
                        GSP_Deinit();
                        gsp_cur_client_pid = INVALID_USER_ID;
                    PHASE_SET_LINE();
                        up(&gsp_hw_resource_sem);
                        printk("%s%d:pid:0x%08x, gsp config err:%d, release hw sema.\n",__func__,__LINE__,pUserdata->pid,ret);
                    }
                }
            }
        }
        break;

        case GSP_IO_TRIGGER_RUN:
        {
            GSP_TRACE("%s:pid:0x%08x, in trigger to run , L%d \n",__func__,pUserdata->pid,__LINE__);
            if(gsp_cur_client_pid == pUserdata->pid)
            {
            gsp_cur_client_phase |= PHASE_IN_TRIGGER;
                GSP_TRACE("%s:pid:0x%08x, calc coef and trigger to run , L%d \n",__func__,pUserdata->pid,__LINE__);
                GSP_Cache_Flush();
#ifdef GSP_WORK_AROUND1
                if(gsp_workaround_perf == PERF_MAGIC)
                {
                    get_monotonic_boottime(&start_time);
                }

            gsp_cur_client_phase |= PHASE_IN_WORKAROUND;
            ret = GSP_work_around1(pUserdata);
            gsp_cur_client_phase |= PHASE_OUT_WORKAROUND;
            if(ret)
            {
                goto exit;
            }

                if(gsp_workaround_perf == PERF_MAGIC)
                {
                    get_monotonic_boottime(&end_time);

                    gsp_workaround_perf_cnt++;
                    gsp_workaround_perf_ns += (end_time.tv_nsec < start_time.tv_nsec)? (end_time.tv_nsec + 1000000000 - start_time.tv_nsec):(end_time.tv_nsec - start_time.tv_nsec);
                    gsp_workaround_perf_s += (end_time.tv_nsec < start_time.tv_nsec)? (end_time.tv_sec - 1 - start_time.tv_sec):(end_time.tv_sec - start_time.tv_sec);
                    gsp_workaround_perf_s += (gsp_workaround_perf_ns >= 1000000000)? 1:0;
                    gsp_workaround_perf_ns -= (gsp_workaround_perf_ns >= 1000000000)? 1000000000:0;

                    if(gsp_workaround_perf_cnt >= PERF_STATIC_CNT_MAX)
                    {
                        gsp_workaround_perf = ((gsp_workaround_perf_s*1000000+(gsp_workaround_perf_ns>>10)) >> PERF_STATIC_CNT_PWR);
                        gsp_workaround_perf_cnt = 0;
                        gsp_workaround_perf_s = 0;
                        gsp_workaround_perf_ns = 0;
                    }
                }
#endif
                ret = GSP_Scaling_Coef_Gen_And_Config(&gsp_coef_force_calc);
                if(ret)
                {
                    goto exit;
                }

                if(gsp_perf == PERF_MAGIC)
                {
                    get_monotonic_boottime(&start_time);
                }
                ret = GSP_Trigger();
                GSP_TRACE("%sL%d:pid:0x%08x, trigger %s!\n",__func__,__LINE__,pUserdata->pid,(ret)?"failed":"success");
                if(ret)
                {
                    printk("%s%d:pid:0x%08x, trigger failed!! err_code:%d \n",__func__,__LINE__,pUserdata->pid,ret);
                    printCfgInfo();
                    printGPSReg();
					ERR_RECORD_ADD(*(GSP_REG_T *)GSP_REG_BASE);
                    ERR_RECORD_INDEX_ADD_WP();
                    GSP_Deinit();

                    gsp_cur_client_pid = INVALID_USER_ID;
                PHASE_SET_LINE();
                    up(&gsp_hw_resource_sem);
                    GSP_TRACE("%s:pid:0x%08x, release hw sema, L%d \n",__func__,pUserdata->pid,__LINE__);
                }
            }
            else
            {
                GSP_TRACE("%s:pid:0x%08x,exit L%d \n",__func__,pUserdata->pid,__LINE__);
                ret = GSP_KERNEL_CALLER_NOT_OWN_HW;
                goto exit;
            }
        }
        break;


        case GSP_IO_WAIT_FINISH:
        {
            if(gsp_cur_client_pid == pUserdata->pid)
            {
            gsp_cur_client_phase |= PHASE_IN_WAITDONE;
                GSP_TRACE("%s:pid:0x%08x, bf wait done sema, L%d \n",__func__,pUserdata->pid,__LINE__);
                
                ret = down_timeout(&gsp_wait_interrupt_sem,60);
                if (ret == 0)
                {
                    if(gsp_perf == PERF_MAGIC)
                    {
                        get_monotonic_boottime(&end_time);
                        gsp_perf_cnt++;
                        gsp_perf_ns += (end_time.tv_nsec < start_time.tv_nsec)? (end_time.tv_nsec + 1000000000 - start_time.tv_nsec):(end_time.tv_nsec - start_time.tv_nsec);
                        gsp_perf_s += (end_time.tv_nsec < start_time.tv_nsec)? (end_time.tv_sec - 1 - start_time.tv_sec):(end_time.tv_sec - start_time.tv_sec);
                        gsp_perf_s += (gsp_perf_ns >= 1000000000)? 1:0;
                        gsp_perf_ns -= (gsp_perf_ns >= 1000000000)? 1000000000:0;

                        if(gsp_perf_cnt >= PERF_STATIC_CNT_MAX)
                        {
                            gsp_perf = ((gsp_perf_s*1000000+(gsp_perf_ns>>10)) >> PERF_STATIC_CNT_PWR);
                            gsp_perf_cnt = 0;
                            gsp_perf_s = 0;
                            gsp_perf_ns = 0;
                        }
                    }
                    GSP_TRACE("%s:pid:0x%08x, wait done sema success, L%d \n",__func__,pUserdata->pid,__LINE__);
                }
                else if (ret == -ETIME)
                {
                printk("%s%d:pid:0x%08x, wait done sema 60-jiffies-timeout,it's abnormal!!!!!!!! \n",__func__,__LINE__,pUserdata->pid);
                printk("%s%d:EMC_MATRIX:0x%08x,GSP_GAP:0x%08x,GSP_CLOCK:0x%08x,GSP_AUTO_GATE:0x%08x\n",__func__,__LINE__,
                       (uint32_t)(GSP_REG_READ(GSP_EMC_MATRIX_BASE)&GSP_EMC_MATRIX_BIT),
                       ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.dist_rb,
                       GSP_REG_READ(GSP_CLOCK_BASE)&0x3,
                       (uint32_t)(GSP_REG_READ(GSP_AUTO_GATE_ENABLE_BASE)&GSP_AUTO_GATE_ENABLE_BIT));
                printCfgInfo();
                printGPSReg();
                ret = GSP_KERNEL_WAITDONE_TIMEOUT;
            }
            else if (ret)
            {
                printk("%s:pid:0x%08x, wait done sema interrupted by a signal, L%d \n",__func__,pUserdata->pid,__LINE__);
                ret = GSP_KERNEL_WAITDONE_INTR;
            }

                if (pUserdata->is_exit_force)
                {
                    pUserdata->is_exit_force = 0;
                    ret = GSP_KERNEL_FORCE_EXIT;
                }

                GSP_Wait_Finish();
                GSP_Cache_Invalidate();
                GSP_Deinit();
                gsp_cur_client_pid = INVALID_USER_ID;
            gsp_cur_client_phase |= PHASE_OUT_WAITDONE;
                sema_init(&gsp_wait_interrupt_sem,0);
                GSP_TRACE("%s:pid:0x%08x, release gsp-hw sema, L%d \n",__func__,pUserdata->pid,__LINE__);
                up(&gsp_hw_resource_sem);

            }
        }
        break;

        default:
            ret = GSP_KERNEL_CTL_CMD_ERR;
            break;
    }

exit:
    if (ret)
    {
        printk("%s:pid:0x%08x, error code 0x%x \n", __func__,pUserdata->pid,ret);
    }
    return ret;

}



static int32_t  gsp_drv_proc_read(char *page,
                                  char **start,
                                  off_t off,
                                  int32_t count,
                                  int32_t *eof,
                                  void *data)
{
    int32_t len = 0;
    GSP_REG_T *g_gsp_reg =  NULL;

    len += sprintf(page + len, "********************************************* \n");


    if(!ERR_RECORD_EMPTY())
    {
        len += sprintf(page + len, "%s , u_data_size %d ,record register[%d] :\n",__func__,count,ERR_RECORD_INDEX_GET_RP());
        g_gsp_reg = (GSP_REG_T *)ERR_RECORD_GET();
        ERR_RECORD_INDEX_ADD_RP();
    }
    else
    {
        len += sprintf(page + len, "%s , u_data_size %d , register :\n",__func__,count);

        g_gsp_reg = (GSP_REG_T *)GSP_REG_BASE;
    }





    len += sprintf(page + len, "misc: run %d|busy %d|errflag %d|errcode %02d|dither %d|pmmod0 %d|pmmod1 %d|pmen %d|scale %d|reserv2 %d|scl_stat_clr %d|l0en %d|l1en %d|rb %d\n",
                   g_gsp_reg->gsp_cfg_u.mBits.gsp_run,
                   g_gsp_reg->gsp_cfg_u.mBits.gsp_busy,
                   g_gsp_reg->gsp_cfg_u.mBits.error_flag,
                   g_gsp_reg->gsp_cfg_u.mBits.error_code,
                   g_gsp_reg->gsp_cfg_u.mBits.dither_en,
                   g_gsp_reg->gsp_cfg_u.mBits.pmargb_mod0,
                   g_gsp_reg->gsp_cfg_u.mBits.pmargb_mod1,
                   g_gsp_reg->gsp_cfg_u.mBits.pmargb_en,
                   g_gsp_reg->gsp_cfg_u.mBits.scale_en,
                   g_gsp_reg->gsp_cfg_u.mBits.reserved2,
                   g_gsp_reg->gsp_cfg_u.mBits.scale_status_clr,
                   g_gsp_reg->gsp_cfg_u.mBits.l0_en,
                   g_gsp_reg->gsp_cfg_u.mBits.l1_en,
                   g_gsp_reg->gsp_cfg_u.mBits.dist_rb);
    len += sprintf(page + len, "misc: inten %d|intmod %d|intclr %d\n",
                   g_gsp_reg->gsp_int_cfg_u.mBits.int_en,
                   g_gsp_reg->gsp_int_cfg_u.mBits.int_mod,
                   g_gsp_reg->gsp_int_cfg_u.mBits.int_clr);


    len += sprintf(page + len, "L0 cfg:fmt %d|rot %d|ck %d|pallet %d|rowtap %d|coltap %d\n",
                   g_gsp_reg->gsp_layer0_cfg_u.mBits.img_format_l0,
                   g_gsp_reg->gsp_layer0_cfg_u.mBits.rot_mod_l0,
                   g_gsp_reg->gsp_layer0_cfg_u.mBits.ck_en_l0,
                   g_gsp_reg->gsp_layer0_cfg_u.mBits.pallet_en_l0,
                   g_gsp_reg->gsp_layer0_cfg_u.mBits.row_tap_mod,
                   g_gsp_reg->gsp_layer0_cfg_u.mBits.col_tap_mod);
    len += sprintf(page + len, "L0 blockalpha %03d, pitch %04d,(%04d,%04d)%04dx%04d => (%04d,%04d)%04dx%04d\n",
                   g_gsp_reg->gsp_layer0_alpha_u.mBits.alpha_l0,
                   g_gsp_reg->gsp_layer0_pitch_u.mBits.pitch0,
                   g_gsp_reg->gsp_layer0_clip_start_u.mBits.clip_start_x_l0,
                   g_gsp_reg->gsp_layer0_clip_start_u.mBits.clip_start_y_l0,
                   g_gsp_reg->gsp_layer0_clip_size_u.mBits.clip_size_x_l0,
                   g_gsp_reg->gsp_layer0_clip_size_u.mBits.clip_size_y_l0,
                   g_gsp_reg->gsp_layer0_des_start_u.mBits.des_start_x_l0,
                   g_gsp_reg->gsp_layer0_des_start_u.mBits.des_start_y_l0,
                   g_gsp_reg->gsp_layer0_des_size_u.mBits.des_size_x_l0,
                   g_gsp_reg->gsp_layer0_des_size_u.mBits.des_size_y_l0);
    len += sprintf(page + len, "L0 Yaddr 0x%08x|Uaddr 0x%08x|Vaddr 0x%08x\n",
                   g_gsp_reg->gsp_layer0_y_addr_u.dwValue,
                   g_gsp_reg->gsp_layer0_uv_addr_u.dwValue,
                   g_gsp_reg->gsp_layer0_va_addr_u.dwValue);
    len += sprintf(page + len, "L0 grey(%03d,%03d,%03d) colorkey(%03d,%03d,%03d)\n",
                   g_gsp_reg->gsp_layer0_grey_rgb_u.mBits.layer0_grey_r,
                   g_gsp_reg->gsp_layer0_grey_rgb_u.mBits.layer0_grey_g,
                   g_gsp_reg->gsp_layer0_grey_rgb_u.mBits.layer0_grey_b,
                   g_gsp_reg->gsp_layer0_ck_u.mBits.ck_r_l0,
                   g_gsp_reg->gsp_layer0_ck_u.mBits.ck_g_l0,
                   g_gsp_reg->gsp_layer0_ck_u.mBits.ck_b_l0);
    len += sprintf(page + len, "L0 endian: y %d|u %d|v %d|rgb %d|alpha %d\n",
                   g_gsp_reg->gsp_layer0_endian_u.mBits.y_endian_mod_l0,
                   g_gsp_reg->gsp_layer0_endian_u.mBits.uv_endian_mod_l0,
                   g_gsp_reg->gsp_layer0_endian_u.mBits.va_endian_mod_l0,
                   g_gsp_reg->gsp_layer0_endian_u.mBits.rgb_swap_mod_l0,
                   g_gsp_reg->gsp_layer0_endian_u.mBits.a_swap_mod_l0);


    len += sprintf(page + len, "L1 cfg:fmt %d|rot %d|ck %d|pallet %d\n",
                   g_gsp_reg->gsp_layer1_cfg_u.mBits.img_format_l1,
                   g_gsp_reg->gsp_layer1_cfg_u.mBits.rot_mod_l1,
                   g_gsp_reg->gsp_layer1_cfg_u.mBits.ck_en_l1,
                   g_gsp_reg->gsp_layer1_cfg_u.mBits.pallet_en_l1);
    len += sprintf(page + len, "L1 blockalpha %03d, pitch %04d,(%04d,%04d)%04dx%04d => (%04d,%04d)\n",
                   g_gsp_reg->gsp_layer1_alpha_u.mBits.alpha_l1,
                   g_gsp_reg->gsp_layer1_pitch_u.mBits.pitch1,
                   g_gsp_reg->gsp_layer1_clip_start_u.mBits.clip_start_x_l1,
                   g_gsp_reg->gsp_layer1_clip_start_u.mBits.clip_start_y_l1,
                   g_gsp_reg->gsp_layer1_clip_size_u.mBits.clip_size_x_l1,
                   g_gsp_reg->gsp_layer1_clip_size_u.mBits.clip_size_y_l1,
                   g_gsp_reg->gsp_layer1_des_start_u.mBits.des_start_x_l1,
                   g_gsp_reg->gsp_layer1_des_start_u.mBits.des_start_y_l1);
    len += sprintf(page + len, "L1 Yaddr 0x%08x|Uaddr 0x%08x|Vaddr 0x%08x\n",
                   g_gsp_reg->gsp_layer1_y_addr_u.dwValue,
                   g_gsp_reg->gsp_layer1_uv_addr_u.dwValue,
                   g_gsp_reg->gsp_layer1_va_addr_u.dwValue);
    len += sprintf(page + len, "L1 grey(%03d,%03d,%03d) colorkey(%03d,%03d,%03d)\n",
                   g_gsp_reg->gsp_layer1_grey_rgb_u.mBits.grey_r_l1,
                   g_gsp_reg->gsp_layer1_grey_rgb_u.mBits.grey_g_l1,
                   g_gsp_reg->gsp_layer1_grey_rgb_u.mBits.grey_b_l1,
                   g_gsp_reg->gsp_layer1_ck_u.mBits.ck_r_l1,
                   g_gsp_reg->gsp_layer1_ck_u.mBits.ck_g_l1,
                   g_gsp_reg->gsp_layer1_ck_u.mBits.ck_b_l1);
    len += sprintf(page + len, "L1 endian: y %d|u %d|v %d|rgb %d|alpha %d\n",
                   g_gsp_reg->gsp_layer1_endian_u.mBits.y_endian_mod_l1,
                   g_gsp_reg->gsp_layer1_endian_u.mBits.uv_endian_mod_l1,
                   g_gsp_reg->gsp_layer1_endian_u.mBits.va_endian_mod_l1,
                   g_gsp_reg->gsp_layer1_endian_u.mBits.rgb_swap_mod_l1,
                   g_gsp_reg->gsp_layer1_endian_u.mBits.a_swap_mod_l1);


    len += sprintf(page + len, "Ld cfg:fmt %d|cmpr8 %d|pitch %04d\n",
                   g_gsp_reg->gsp_des_data_cfg_u.mBits.des_img_format,
                   g_gsp_reg->gsp_des_data_cfg_u.mBits.compress_r8,
                   g_gsp_reg->gsp_des_pitch_u.mBits.des_pitch);
    len += sprintf(page + len, "Ld Yaddr 0x%08x|Uaddr 0x%08x|Vaddr 0x%08x\n",
                   g_gsp_reg->gsp_des_y_addr_u.dwValue,
                   g_gsp_reg->gsp_des_uv_addr_u.dwValue,
                   g_gsp_reg->gsp_des_v_addr_u.dwValue);
    len += sprintf(page + len, "Ld endian: y %d|u %d|v %d|rgb %d|alpha %d\n",
                   g_gsp_reg->gsp_des_data_endian_u.mBits.y_endian_mod,
                   g_gsp_reg->gsp_des_data_endian_u.mBits.uv_endian_mod,
                   g_gsp_reg->gsp_des_data_endian_u.mBits.v_endian_mod,
                   g_gsp_reg->gsp_des_data_endian_u.mBits.rgb_swap_mod,
                   g_gsp_reg->gsp_des_data_endian_u.mBits.a_swap_mod);
    len += sprintf(page + len, "********************************************* \n");

    msleep(10);

    return len;
}


static struct file_operations gsp_drv_fops =
{
    .owner          = THIS_MODULE,
    .open           = gsp_drv_open,
    .write          = gsp_drv_write,
    .read           = gsp_drv_read,
    .unlocked_ioctl = gsp_drv_ioctl,
    .release        = gsp_drv_release
};

static struct miscdevice gsp_drv_dev =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "sprd_gsp",
    .fops = &gsp_drv_fops
};

#if 0
static irqreturn_t gsp_irq_handler(int32_t irq, void *dev_id)
{
    ulong cost_ns=0;
    ulong cost_s=0;
    static uint32_t test_flag=1;
    static uint32_t cnt=0;

    static uint32_t change_first_cfg=0;
    static uint32_t test_type=0;
    static struct timespec start_time= {0};
    static struct timespec end_time= {0};


    GSP_TRACE("%s enter!\n",__func__);

    if(change_first_cfg==0)
    {
        change_first_cfg=1;
        GSP_EMC_GAP_SET(0);
        GSP_CLOCK_SET(0);
    }
    if((GSP_L0_PITCH_GET() == 640) && test_flag && cnt == 0)
    {
        get_monotonic_boottime(&start_time);
        printk("gsp-irq get start time(%04ld.%09ld)\n",start_time.tv_sec,start_time.tv_nsec);
    }

    GSP_IRQSTATUS_CLEAR();
    GSP_IRQENABLE_SET(GSP_IRQ_TYPE_DISABLE);

    cnt++;
    if((GSP_L0_PITCH_GET() == 640) && test_flag && cnt == 200)
    {
        cnt=0;
        switch(test_type)
        {
            case 0:
                GSP_EMC_GAP_SET(0);
                GSP_CLOCK_SET(1);
                break;
            case 1:
                GSP_EMC_GAP_SET(0);
                GSP_CLOCK_SET(2);
                break;
            case 2:
                GSP_EMC_GAP_SET(0);
                GSP_CLOCK_SET(3);
                break;
            case 3:
                GSP_EMC_GAP_SET(0xff);
                GSP_CLOCK_SET(0);
                break;
            case 4:
                GSP_EMC_GAP_SET(0xff);
                GSP_CLOCK_SET(1);
                break;
            case 5:
                GSP_EMC_GAP_SET(0xff);
                GSP_CLOCK_SET(2);
                break;
            case 6:
                GSP_EMC_GAP_SET(0xff);
                GSP_CLOCK_SET(3);
                break;
            case 7:
            default:
                GSP_EMC_GAP_SET(0);
                GSP_CLOCK_SET(3);
                test_flag = 0;
                break;
        }

        get_monotonic_boottime(&end_time);
        cost_s = end_time.tv_sec - start_time.tv_sec;
        cost_ns = end_time.tv_nsec-start_time.tv_nsec;
        printk("gsp-irq type%d (%04ld.%09ld--%04ld.%09ld),cost:%02u%09u ns\n",
               test_type,
               start_time.tv_sec,
               start_time.tv_nsec,
               end_time.tv_sec,
               end_time.tv_nsec,
               cost_s,
               cost_ns);
        test_type = (test_type + 1)%8;
    }
    if((GSP_L0_PITCH_GET() == 640) && test_flag)
    {
        printk("gsp-irq trigger %03d\n",cnt);
        GSP_Trigger();
    }
    else
    {
        printk("gsp-irq up\n");
        up(&gsp_wait_interrupt_sem);
    }
    return IRQ_HANDLED;
}

#else
static irqreturn_t gsp_irq_handler(int32_t irq, void *dev_id)
{
    GSP_TRACE("%s enter!\n",__func__);

    GSP_IRQSTATUS_CLEAR();
    GSP_IRQENABLE_SET(GSP_IRQ_TYPE_DISABLE);
    up(&gsp_wait_interrupt_sem);

    
    

    return IRQ_HANDLED;
}

#endif


static int gsp_suspend(struct platform_device *pdev,pm_message_t state)
{
	printk("%s%d\n",__func__,__LINE__);
	return 0;
}

static int gsp_resume(struct platform_device *pdev)
{
	printk("%s%d\n",__func__,__LINE__);
	gsp_coef_force_calc = 1;

    
    
    GSP_CLOCK_SET(GSP_CLOCK_256M_BIT);
    GSP_AUTO_GATE_ENABLE();
    
    
	return 0;
}

static int32_t gsp_emc_clock_init(void)
{
    struct clk *emc_clk_parent = NULL;
    int ret = 0;

    emc_clk_parent = clk_get(NULL, GSP_EMC_CLOCK_PARENT_NAME);
    if (IS_ERR(emc_clk_parent)) {
        printk(KERN_ERR "gsp: get emc clk_parent failed!\n");
        return -1;
    } else {
        printk(KERN_INFO "gsp: get emc clk_parent ok!\n");
    }

    g_gsp_emc_clk = clk_get(NULL, GSP_EMC_CLOCK_NAME);
    if (IS_ERR(g_gsp_emc_clk)) {
        printk(KERN_ERR "gsp: get emc clk failed!\n");
        return -1;
    } else {
        printk(KERN_INFO "gsp: get emc clk ok!\n");
    }

    ret = clk_set_parent(g_gsp_emc_clk, emc_clk_parent);
    if(ret) {
        printk(KERN_ERR "gsp: gsp set emc clk parent failed!\n");
    } else {
        printk(KERN_INFO "gsp: gsp set emc clk parent ok!\n");
    }

    return ret;
}
int32_t gsp_drv_probe(struct platform_device *pdev)
{
    int32_t ret = 0;
    int32_t i = 0;

    GSP_TRACE("gsp_probe enter .\n");
    printk("%s,AHB clock :%d\n", __func__,GSP_AHB_CLOCK_GET());
    
    
    GSP_CLOCK_SET(GSP_CLOCK_256M_BIT);
    GSP_AUTO_GATE_ENABLE();
    
    GSP_ENABLE_MM();

    ret = gsp_emc_clock_init();
    if (ret) {
        printk(KERN_ERR "gsp emc clock init failed. \n");
        goto exit;
    }
    ret = misc_register(&gsp_drv_dev);
    if (ret)
    {
        printk(KERN_ERR "gsp cannot register miscdev (%d)\n", ret);
        goto exit;
    }

    ret = request_irq(TB_GSP_INT,
                      gsp_irq_handler,
                      0,
                      "GSP",
                      gsp_irq_handler);

    if (ret)
    {
        printk("could not request irq %d\n", IRQ_GSP_INT);
        goto exit1;
    }

    gsp_drv_proc_file = create_proc_read_entry("driver/sprd_gsp",
                        0444,
                        NULL,
                        gsp_drv_proc_read,
                        NULL);
    if (unlikely(NULL == gsp_drv_proc_file))
    {
        printk("Can't create an entry for gsp in /proc \n");
        ret = ENOMEM;
        goto exit2;
    }


    for (i=0; i<GSP_MAX_USER; i++)
    {
        gsp_user_array[i].pid = INVALID_USER_ID;
        gsp_user_array[i].is_exit_force = 0;
        sema_init(&gsp_user_array[i].sem_open, 1);
    }

    
    memset(&s_gsp_cfg,0,sizeof(s_gsp_cfg));
    sema_init(&gsp_hw_resource_sem, 1);
    sema_init(&gsp_wait_interrupt_sem, 0);

    return ret;

exit2:
    free_irq(IRQ_GSP_INT, gsp_irq_handler);
exit1:
    misc_deregister(&gsp_drv_dev);
exit:
    return ret;
}

static int32_t gsp_drv_remove(struct platform_device *dev)
{
    GSP_TRACE( "gsp_remove called !\n");

    if (gsp_drv_proc_file)
    {
        remove_proc_entry("driver/sprd_gsp", NULL);
    }
    free_irq(IRQ_GSP_INT, gsp_irq_handler);
    misc_deregister(&gsp_drv_dev);
    return 0;
}

static struct platform_driver gsp_drv_driver =
{
    .probe = gsp_drv_probe,
    .remove = gsp_drv_remove,
    .suspend = gsp_suspend,
	.resume = gsp_resume,
    .driver =
    {
        .owner = THIS_MODULE,
        .name = "sprd_gsp"
    }
};

int32_t __init gsp_drv_init(void)
{
    printk("gsp_drv_init enter! \n");

    if (platform_driver_register(&gsp_drv_driver) != 0)
    {
        printk("gsp platform driver register Failed! \n");
        return -1;
    }
    else
    {
        GSP_TRACE("gsp platform driver registered successful! \n");
    }
    return 0;
}

void gsp_drv_exit(void)
{
    platform_driver_unregister(&gsp_drv_driver);
    GSP_TRACE("gsp platform driver unregistered! \n");
}

module_init(gsp_drv_init);
module_exit(gsp_drv_exit);

MODULE_DESCRIPTION("GSP Driver");
MODULE_LICENSE("GPL");

