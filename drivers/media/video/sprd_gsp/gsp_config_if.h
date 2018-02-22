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

#ifndef _SHARK_GSP_DRV_H_
#define _SHARK_GSP_DRV_H_

#ifdef __cplusplus
extern   "C"
{
#endif

#include <video/gsp_types_shark.h>
#include <mach/sci_glb_regs.h>
#include <mach/globalregs.h> 
#include <linux/delay.h>
#include <linux/clk.h>




    typedef struct _GSP_LAYER1_REG_T_TAG_
    {
        union _GSP_LAYER1_CFG_TAG gsp_layer1_cfg_u;
        union _GSP_LAYER1_Y_ADDR_TAG gsp_layer1_y_addr_u;
        union _GSP_LAYER1_UV_ADDR_TAG gsp_layer1_uv_addr_u;
        union _GSP_LAYER1_VA_ADDR_TAG gsp_layer1_va_addr_u;
        union _GSP_LAYER1_PITCH_TAG gsp_layer1_pitch_u;
        union _GSP_LAYER1_CLIP_START_TAG gsp_layer1_clip_start_u;
        union _GSP_LAYER1_CLIP_SIZE_TAG gsp_layer1_clip_size_u;
        union _GSP_LAYER1_DES_START_TAG gsp_layer1_des_start_u;
        union _GSP_LAYER1_GREY_RGB_TAG gsp_layer1_grey_rgb_u;
        union _GSP_LAYER1_ENDIAN_TAG gsp_layer1_endian_u;
        union _GSP_LAYER1_ALPHA_TAG gsp_layer1_alpha_u;
        union _GSP_LAYER1_CK_TAG gsp_layer1_ck_u;
    }
    GSP_LAYER1_REG_T;

    typedef struct _GSP_CMDQ_REG_T_TAG_
    {
        union _GSP_CMD_ADDR_TAG gsp_cmd_addr_u;
        union _GSP_CMD_CFG_TAG gsp_cmd_cfg_u;
    }
    GSP_CMDQ_REG_T;

#define GSP_L1_CMDQ_SET(cfg)\
		*(GSP_CMDQ_REG_T*)(&((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cmd_addr_u) = (cfg)




#if 0
#define GSP_MOD_EN          (SPRD_AHB_BASE)
#define GSP_SOFT_RESET      (SPRD_AHB_BASE + 0x04)

#define GSP_EMC_MATRIX_BASE		(SPRD_AONAPB_BASE + 0x04) 
#define GSP_EMC_MATRIX_BIT		(1<<11) 

#define GSP_CLOCK_BASE		(SPRD_APBCKG_BASE + 0x28)
#define GSP_CLOCK_256M_BIT  (3)

#define GSP_AUTO_GATE_ENABLE_BASE		(SPRD_AHB_BASE + 0x40)
#define GSP_AUTO_GATE_ENABLE_BIT		(1<<8)

#define GSP_AHB_CLOCK_BASE      (SPRD_APBCKG_BASE + 0x20)
#define GSP_AHB_CLOCK_26M_BIT   (0)
#define GSP_AHB_CLOCK_192M_BIT  (3)


#define TB_GSP_INT 			(0x33)  
#define GSP_IRQ_BIT			(1<<19) 
#define GSP_SOFT_RST_BIT    (1<<3) 
#define GSP_MOD_EN_BIT      (1<<3) 

#else
#define GSP_MOD_EN          (REG_AP_AHB_AHB_EB)
#define GSP_SOFT_RESET      (REG_AP_AHB_AHB_RST)

#define GSP_EMC_MATRIX_BASE		(REG_AON_APB_APB_EB1) 
#define GSP_EMC_MATRIX_BIT		(BIT_DISP_EMC_EB) 

#define GSP_CLOCK_BASE		(REG_AP_CLK_GSP_CFG)
#define GSP_CLOCK_256M_BIT  (3)


#define GSP_AUTO_GATE_ENABLE_BASE		(REG_AP_AHB_AP_SYS_AUTO_SLEEP_CFG)
#define GSP_AUTO_GATE_ENABLE_BIT		(BIT_GSP_AUTO_GATE_EN)
#define GSP_CKG_FORCE_ENABLE_BIT		(BIT_GSP_CKG_FORCE_EN)

#define GSP_AHB_CLOCK_BASE      (REG_AP_CLK_AP_AHB_CFG)
#define GSP_AHB_CLOCK_26M_BIT   (0)
#define GSP_AHB_CLOCK_192M_BIT  (3)

#define TB_GSP_INT 			(IRQ_GSP_INT)  
#define GSP_IRQ_BIT			SCI_INTC_IRQ_BIT(TB_GSP_INT) 
#define GSP_SOFT_RST_BIT    (BIT_GSP_SOFT_RST) 
#define GSP_MOD_EN_BIT      (BIT_GSP_EB) 

#endif


#define GSP_REG_BASE        (SPRD_GSP_BASE)
#define GSP_HOR_COEF_BASE   (GSP_REG_BASE + 0x90)
#define GSP_VER_COEF_BASE   (GSP_REG_BASE + 0x110)
#define GSP_L1_BASE			(GSP_REG_BASE + 0x60)

#ifndef GSP_ASSERT
#define GSP_ASSERT()        do{}while(1)
#endif

#define GSP_EMC_CLOCK_PARENT_NAME		("clk_aon_apb")
#define GSP_EMC_CLOCK_NAME				("clk_disp_emc")


#if 0
#define GSP_EMC_MATRIX_ENABLE() (*(volatile uint32_t*)(GSP_EMC_MATRIX_BASE) |= GSP_EMC_MATRIX_BIT)
#define GSP_CLOCK_SET(sel)\
{\
    *(volatile uint32_t*)(GSP_CLOCK_BASE) &= ~3;\
    *(volatile uint32_t*)(GSP_CLOCK_BASE) |= (sel);\
}
#define GSP_AUTO_GATE_ENABLE()  (*(volatile uint32_t*)(GSP_AUTO_GATE_ENABLE_BASE) |= GSP_AUTO_GATE_ENABLE_BIT)
#define GSP_AHB_CLOCK_SET(sel)\
{\
    *(volatile uint32_t*)(GSP_AHB_CLOCK_BASE) &= ~3;\
    *(volatile uint32_t*)(GSP_AHB_CLOCK_BASE) |= (sel);\
}
#define GSP_AHB_CLOCK_GET(sel)  (*(volatile uint32_t*)(GSP_AHB_CLOCK_BASE)&0x3)


#define GSP_ENABLE_MM(addr)\
{\
    *(volatile unsigned long *)(SPRD_PMU_BASE+0x1c) &= ~(1<<25);\
    *(volatile unsigned long *)(SPRD_AONAPB_BASE) |= (1<<25);\
}

#define GSP_HWMODULE_SOFTRESET()\
    *(volatile uint32_t *)(GSP_SOFT_RESET) |= GSP_SOFT_RST_BIT;\
    udelay(10);\
    *(volatile uint32_t *)(GSP_SOFT_RESET) &= (~GSP_SOFT_RST_BIT)

#define GSP_HWMODULE_ENABLE()\
    *(volatile uint32_t *)(GSP_MOD_EN) |= (GSP_MOD_EN_BIT)

#define GSP_HWMODULE_DISABLE(bit)\
        *(volatile uint32_t *)(GSP_MOD_EN) &= (~(GSP_MOD_EN_BIT))

#else
#include <mach/sci.h>
#define GSP_REG_READ(reg)  (*(volatile uint32_t*)reg)
#define GSP_EMC_MATRIX_ENABLE()     sci_glb_set(GSP_EMC_MATRIX_BASE, GSP_EMC_MATRIX_BIT)
#define GSP_CLOCK_SET(sel)          sci_glb_write(GSP_CLOCK_BASE, (sel), 0x3)
#define GSP_AUTO_GATE_ENABLE()      sci_glb_set(GSP_AUTO_GATE_ENABLE_BASE, GSP_AUTO_GATE_ENABLE_BIT)
#define GSP_AHB_CLOCK_SET(sel)      sci_glb_write(GSP_AHB_CLOCK_BASE, (sel), 0x3)
#define GSP_AHB_CLOCK_GET()      	sci_glb_read(GSP_AHB_CLOCK_BASE,0x3)



#if 0
#define GSP_ENABLE_MM(addr)\
{\
    sci_glb_clr((SPRD_PMU_BASE+0x1c),(1<<25));\
    sci_glb_set(SPRD_AONAPB_BASE,(1<<25));\
}
#else
#define GSP_ENABLE_MM(addr)\
{\
    sci_glb_clr((REG_PMU_APB_PD_MM_TOP_CFG),(BIT_PD_MM_TOP_FORCE_SHUTDOWN));\
    sci_glb_set(REG_AON_APB_APB_EB0,(BIT_PD_MM_TOP_FORCE_SHUTDOWN));\
}

#endif


#define GSP_HWMODULE_SOFTRESET()\
    sci_glb_set(GSP_SOFT_RESET,GSP_SOFT_RST_BIT);\
    udelay(10);\
    sci_glb_clr(GSP_SOFT_RESET,GSP_SOFT_RST_BIT)

#define GSP_HWMODULE_ENABLE()       sci_glb_set(GSP_MOD_EN,GSP_MOD_EN_BIT)
#define GSP_HWMODULE_DISABLE()		sci_glb_clr(GSP_MOD_EN,GSP_MOD_EN_BIT)

#endif
#define GSP_EMC_GAP_SET(interval)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.dist_rb = (interval)
#define GSP_L0_ADDR_SET(addr)\
    *(volatile GSP_DATA_ADDR_T*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_y_addr_u) = (addr)
#define GSP_L1_ADDR_SET(addr)\
    *(volatile GSP_DATA_ADDR_T*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_y_addr_u) = (addr)
#define GSP_Ld_ADDR_SET(addr)\
    *(volatile GSP_DATA_ADDR_T*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_des_y_addr_u) = (addr)

#define GSP_L0_PITCH_GET()  (*(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_pitch_u))
#define GSP_L0_PITCH_SET(pitch)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_pitch_u) = (pitch)
#define GSP_L1_PITCH_SET(pitch)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_pitch_u) = (pitch)
#define GSP_Ld_PITCH_SET(pitch)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_des_pitch_u) = (pitch)


#define GSP_L0_CLIPRECT_SET(rect)\
    *(volatile GSP_RECT_T*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_clip_start_u) = (rect)
#define GSP_L1_CLIPRECT_SET(rect)\
    *(volatile GSP_RECT_T*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_clip_start_u) = (rect)


#define GSP_L0_DESRECT_SET(rect)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_des_start_u) = *((uint32_t*)&(rect).st_x);\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_des_size_u) = *((uint32_t*)&(rect).rect_w)
#define GSP_L1_DESPOS_SET(pos)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_des_start_u) = *(uint32_t*)(&(pos))


#define GSP_L0_GREY_SET(grey)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_grey_rgb_u) = *(uint32_t*)(&(grey))
#define GSP_L1_GREY_SET(grey)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_grey_rgb_u) = *(uint32_t*)(&(grey))


#define GSP_L0_ENDIAN_SET(endian_mode)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_endian_u) = \
      (((endian_mode).y_word_endn & 0x03) << 0)\
     |(((endian_mode).y_lng_wrd_endn & 0x01) << 2)\
     |(((endian_mode).uv_word_endn & 0x03) << 3)\
     |(((endian_mode).uv_lng_wrd_endn & 0x01) << 5)\
     |(((endian_mode).va_word_endn & 0x03) << 6)\
     |(((endian_mode).va_lng_wrd_endn & 0x01) << 8)\
     |(((endian_mode).rgb_swap_mode & 0x07) << 9)\
     |(((endian_mode).a_swap_mode & 0x01) << 12)
#define GSP_L1_ENDIAN_SET(endian_mode)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_endian_u) = \
      (((endian_mode).y_word_endn & 0x03) << 0)\
     |(((endian_mode).y_lng_wrd_endn & 0x01) << 2)\
     |(((endian_mode).uv_word_endn & 0x03) << 3)\
     |(((endian_mode).uv_lng_wrd_endn & 0x01) << 5)\
     |(((endian_mode).va_word_endn & 0x03) << 6)\
     |(((endian_mode).va_lng_wrd_endn & 0x01) << 8)\
     |(((endian_mode).rgb_swap_mode & 0x07) << 9)\
     |(((endian_mode).a_swap_mode & 0x01) << 12)
#define GSP_Ld_ENDIAN_SET(endian_mode)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_des_data_endian_u) = \
      (((endian_mode).y_word_endn & 0x03) << 0)\
     |(((endian_mode).y_lng_wrd_endn & 0x01) << 2)\
     |(((endian_mode).uv_word_endn & 0x03) << 3)\
     |(((endian_mode).uv_lng_wrd_endn & 0x01) << 5)\
     |(((endian_mode).va_word_endn & 0x03) << 6)\
     |(((endian_mode).va_lng_wrd_endn & 0x01) << 8)\
     |(((endian_mode).rgb_swap_mode & 0x07) << 9)\
     |(((endian_mode).a_swap_mode & 0x01) << 12)

#define GSP_L0_ALPHA_SET(alpha)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_alpha_u) = (alpha)
#define GSP_L1_ALPHA_SET(alpha)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_alpha_u) = (alpha)

#define GSP_L0_COLORKEY_SET(colorkey)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_ck_u) = *(uint32_t*)(&(colorkey))
#define GSP_L1_COLORKEY_SET(colorkey)\
    *(volatile uint32_t*)(&((GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_ck_u) = *(uint32_t*)(&(colorkey))

#define GSP_L0_IMGFORMAT_SET(format)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_cfg_u.mBits.img_format_l0 = (format)
#define GSP_L1_IMGFORMAT_SET(format)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_cfg_u.mBits.img_format_l1 = (format)
#define GSP_Ld_IMGFORMAT_SET(format)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_des_data_cfg_u.mBits.des_img_format = (format)
#define GSP_Ld_COMPRESSRGB888_SET(enable)\
        ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_des_data_cfg_u.mBits.compress_r8 = (enable)

#define GSP_L0_ROTMODE_SET(mode)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_cfg_u.mBits.rot_mod_l0 = (mode)
#define GSP_L1_ROTMODE_SET(mode)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_cfg_u.mBits.rot_mod_l1 = (mode)

#define GSP_L0_COLORKEYENABLE_SET(colorkey_en)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_cfg_u.mBits.ck_en_l0 = (colorkey_en)
#define GSP_L1_COLORKEYENABLE_SET(colorkey_en)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_cfg_u.mBits.ck_en_l1 = (colorkey_en)

#define GSP_L0_PALLETENABLE_SET(pallet_en)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_cfg_u.mBits.pallet_en_l0 = (pallet_en)
#define GSP_L1_PALLETENABLE_SET(pallet_en)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer1_cfg_u.mBits.pallet_en_l1 = (pallet_en)

#define GSP_L0_SCALETAPMODE_SET(row_tap,col_tap)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_cfg_u.mBits.row_tap_mod = (row_tap);\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_layer0_cfg_u.mBits.col_tap_mod = (col_tap)

#define GSP_IRQMODE_SET(mode)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_int_cfg_u.mBits.int_mod = (mode)
#define GSP_IRQENABLE_SET(int_enable)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_int_cfg_u.mBits.int_en = (int_enable)

    

#define GSP_IRQSTATUS_CLEAR()\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_int_cfg_u.mBits.int_clr = 1;\
    udelay(10);\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_int_cfg_u.mBits.int_clr = 0

#define GSP_WORKSTATUS_GET()   (((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.gsp_busy)
#define GSP_ERRFLAG_GET()   (((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.error_flag)
#define GSP_ERRCODE_GET()   (((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.error_code)

#define GSP_DITHER_ENABLE_SET(dith_en)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.dither_en = (dith_en)
#define GSP_PMARGB_ENABLE_SET(pm_en)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.pmargb_en = (pm_en)
#define GSP_L0_PMARGBMODE_SET(mode)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.pmargb_mod0 = (mode)
#define GSP_L1_PMARGBMODE_SET(mode)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.pmargb_mod1 = (mode)
#define GSP_SCALE_ENABLE_SET(scal_en)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.scale_en = (scal_en)
#define GSP_SCALE_ENABLE_GET()  (((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.scale_en)


#define GSP_SCALESTATUS_RESET()\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.scale_status_clr = 1;\
    udelay(10);\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.scale_status_clr = 0

#define GSP_L0_ENABLE_SET(enable)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.l0_en = (enable)
#define GSP_L1_ENABLE_SET(enable)\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.l1_en = (enable)
#define GSP_ENGINE_TRIGGER()\
    ((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.gsp_run = 1

#define GSP_L0_ENABLE_GET()     (((volatile GSP_REG_T*)GSP_REG_BASE)->gsp_cfg_u.mBits.l0_en)

#define GSP_CFG_L1_PARAM(param)\
    *(volatile GSP_LAYER1_REG_T *)GSP_L1_BASE = (param)


    PUBLIC void GSP_Init(void);
    PUBLIC void GSP_Deinit(void);
    PUBLIC void GSP_ConfigLayer(GSP_MODULE_ID_E layer_id);
    PUBLIC uint32_t GSP_Trigger(void);
    PUBLIC void GSP_Wait_Finish(void);



#ifdef __cplusplus
}
#endif

#endif
