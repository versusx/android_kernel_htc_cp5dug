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

#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include "sprdfb_dispc_reg.h"
#include "sprdfb_panel.h"
#include "sprdfb.h"
#include "sprdfb_chip_common.h"


#define DISPC_CLOCK_PARENT ("clk_256m")
#define DISPC_CLOCK (256*1000000)
#define DISPC_DBI_CLOCK_PARENT ("clk_256m")
#define DISPC_DBI_CLOCK (256*1000000)
#define DISPC_DPI_CLOCK_PARENT ("clk_384m")
#define DISPC_EMC_EN_PARENT ("clk_aon_apb")

#define SPRDFB_DPI_CLOCK_SRC (384000000)

#define SPRDFB_CONTRAST (74)
#define SPRDFB_SATURATION (73)
#define SPRDFB_BRIGHTNESS (2)


typedef enum
{
   SPRDFB_DYNAMIC_CLK_FORCE,		
   SPRDFB_DYNAMIC_CLK_REFRESH,		
   SPRDFB_DYNAMIC_CLK_COUNT,		
   SPRDFB_DYNAMIC_CLK_MAX,
} SPRDFB_DYNAMIC_CLK_SWITCH_E;

struct sprdfb_dispc_context {
	struct clk		*clk_dispc;
	struct clk 		*clk_dispc_dpi;
	struct clk 		*clk_dispc_dbi;
	struct clk 		*clk_dispc_emc;
	uint32_t		firstswitch;
	uint32_t		lastcolor;
	bool			is_inited;
	bool			is_first_frame;
	bool			is_resume;
	bool			clk_is_open;
	bool			clk_is_refreshing;
	int				clk_open_count;
	spinlock_t clk_spinlock;

	struct sprdfb_device	*dev;

	uint32_t	 	vsync_waiter;
	wait_queue_head_t		vsync_queue;
	uint32_t	        vsync_done;

#ifdef  CONFIG_FB_LCD_OVERLAY_SUPPORT
	
	uint32_t  overlay_state;  
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	uint32_t	 	waitfor_vsync_waiter;
	wait_queue_head_t		waitfor_vsync_queue;
	uint32_t	        waitfor_vsync_done;
#endif
};

static struct sprdfb_dispc_context dispc_ctx = {0};
uint16_t sprdfb_enable = 0;

extern void sprdfb_panel_suspend(struct sprdfb_device *dev);
extern void sprdfb_panel_resume(struct sprdfb_device *dev, bool from_deep_sleep);
extern void sprdfb_panel_before_refresh(struct sprdfb_device *dev);
extern void sprdfb_panel_after_refresh(struct sprdfb_device *dev);
extern void sprdfb_panel_invalidate(struct panel_spec *self);
extern void sprdfb_panel_invalidate_rect(struct panel_spec *self,
				uint16_t left, uint16_t top,
				uint16_t right, uint16_t bottom);

#ifdef CONFIG_FB_DYNAMIC_FPS_SUPPORT
extern int32_t dsi_dpi_init(struct sprdfb_device *dev);
static int32_t sprdfb_dispc_change_fps(struct sprdfb_device *dev, int fps_level);
#endif

#ifdef CONFIG_FB_ESD_SUPPORT
uint16_t request_display_on = 0;
extern void set_backlight(bool value);
extern uint32_t sprdfb_panel_ESD_check(struct sprdfb_device *dev);
#endif

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
static int overlay_start(struct sprdfb_device *dev, uint32_t layer_index);
static int overlay_close(struct sprdfb_device *dev);
#endif

static int sprdfb_dispc_clk_disable(struct sprdfb_dispc_context *dispc_ctx_ptr, SPRDFB_DYNAMIC_CLK_SWITCH_E clock_switch_type);
static int sprdfb_dispc_clk_enable(struct sprdfb_dispc_context *dispc_ctx_ptr, SPRDFB_DYNAMIC_CLK_SWITCH_E clock_switch_type);
static int32_t sprdfb_dispc_init(struct sprdfb_device *dev);
static void dispc_reset(void);
static void dispc_module_enable(void);
static void dispc_stop_for_feature(struct sprdfb_device *dev);
static void dispc_stop_for_switch_color(struct sprdfb_device *dev);
static void dispc_run_for_feature(struct sprdfb_device *dev);
static unsigned int sprdfb_dispc_change_threshold(struct devfreq_dbs *h, unsigned int state);

static irqreturn_t dispc_isr(int irq, void *data)
{
	struct sprdfb_dispc_context *dispc_ctx = (struct sprdfb_dispc_context *)data;
	uint32_t reg_val;
	struct sprdfb_device *dev = dispc_ctx->dev;
	bool done = false;
#ifdef CONFIG_FB_VSYNC_SUPPORT
	bool vsync =  false;
#endif

	reg_val = dispc_read(DISPC_INT_STATUS);

	
	

	if(reg_val & 0x04){
		printk("[DISP] Warning: dispc underflow (0x%x)!\n",reg_val);
		{
			int32_t i;
			for(i=0;i<256;i+=16){
				printk("[DISP] sprdfb: %x: 0x%x, 0x%x, 0x%x, 0x%x\n", i, dispc_read(i), dispc_read(i+4), dispc_read(i+8), dispc_read(i+12));
			}
			printk("[DISP] **************************************\n");
		}

		dispc_write(0x04, DISPC_INT_CLR);
		dispc_clear_bits(BIT(2), DISPC_INT_EN);
	}

	if(NULL == dev){
		return IRQ_HANDLED;
	}

	if((reg_val & 0x10) && (SPRDFB_PANEL_IF_DPI ==  dev->panel_if_type)){
#if 0
		if(dispc_ctx->is_first_frame){
			
			dispc_clear_bits(BIT(4), DISPC_DPI_CTRL);

			
			dispc_set_bits((1 << 4), DISPC_CTRL);

			dispc_ctx->is_first_frame = false;
		}
#endif
		dispc_write(0x10, DISPC_INT_CLR);
		done = true;
	}else if ((reg_val & 0x1) && (SPRDFB_PANEL_IF_DPI !=  dev->panel_if_type)){ 
			dispc_write(1, DISPC_INT_CLR);
			dispc_ctx->is_first_frame = false;
			done = true;
	}
#ifdef CONFIG_FB_ESD_SUPPORT
#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
	if((reg_val & 0x2) && (SPRDFB_PANEL_IF_DPI ==  dev->panel_if_type)){ 
		dispc_write(0x2, DISPC_INT_CLR);
		if(0 != dev->esd_te_waiter){
			
			dev->esd_te_done =1;
			wake_up_interruptible_all(&(dev->esd_te_queue));
			dev->esd_te_waiter = 0;
		}
	}
#endif
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	if((reg_val & 0x1) && (SPRDFB_PANEL_IF_DPI ==  dev->panel_if_type)){
		dispc_write(1, DISPC_INT_CLR);
		vsync = true;
	}else if((reg_val & 0x2) && (SPRDFB_PANEL_IF_EDPI ==  dev->panel_if_type)){ 
		dispc_write(2, DISPC_INT_CLR);
		vsync = true;
	}
	if(vsync){
		
		dispc_ctx->waitfor_vsync_done = 1;
		if(dispc_ctx->waitfor_vsync_waiter){
			
			wake_up_interruptible_all(&(dispc_ctx->waitfor_vsync_queue));
		}
	}
#endif

	if(done){
		dispc_ctx->vsync_done = 1;

#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if(SPRDFB_PANEL_IF_DPI !=  dev->panel_if_type){
			sprdfb_dispc_clk_disable(dispc_ctx,SPRDFB_DYNAMIC_CLK_REFRESH);
		}
#endif

		if (dispc_ctx->vsync_waiter) {
			wake_up_interruptible_all(&(dispc_ctx->vsync_queue));
			dispc_ctx->vsync_waiter = 0;
		}
		sprdfb_panel_after_refresh(dev);
		pr_debug(KERN_INFO "[DISP] sprdfb: [%s]: Done INT, reg_val = %d !\n", __FUNCTION__, reg_val);
	}

	return IRQ_HANDLED;
}


static void dispc_reset(void)
{
	pr_debug("[DISP] sprdfb: REG_AHB_SOFT_RST:%x ,BIT_DISPC_SOFT_RST:%lx \n",REG_AHB_SOFT_RST,BIT_DISPC_SOFT_RST);
	pr_debug("[DISP] sprdfb: REG_AHB_SOFT_RST:%x \n",dispc_glb_read(REG_AHB_SOFT_RST));
	sci_glb_set(REG_AHB_SOFT_RST, (BIT_DISPC_SOFT_RST) );
	pr_debug("[DISP] sprdfb: REG_AHB_SOFT_RST:%x \n",dispc_glb_read(REG_AHB_SOFT_RST));
 	udelay(10);
	sci_glb_clr(REG_AHB_SOFT_RST, (BIT_DISPC_SOFT_RST) );
	pr_debug("[DISP] sprdfb: REG_AHB_SOFT_RST:%x \n",dispc_glb_read(REG_AHB_SOFT_RST));
}

static inline void dispc_set_bg_color(uint32_t bg_color)
{
	dispc_write(bg_color, DISPC_BG_COLOR);
}

static inline void dispc_set_osd_ck(uint32_t ck_color)
{
	dispc_write(ck_color, DISPC_OSD_CK);
}

static inline void dispc_osd_enable(bool is_enable)
{
	uint32_t reg_val;

	reg_val = dispc_read(DISPC_OSD_CTRL);
	if(is_enable){
		reg_val = reg_val | (BIT(0));
	}
	else{
		reg_val = reg_val & (~(BIT(0)));
	}
	dispc_write(reg_val, DISPC_OSD_CTRL);
}


static void dispc_dithering_enable(bool enable)
{
	if(enable){
		dispc_set_bits(BIT(6), DISPC_CTRL);
	}else{
		dispc_clear_bits(BIT(6), DISPC_CTRL);
	}
}

static void dispc_pwr_enable(bool enable)
{
	if(enable){
		dispc_set_bits(BIT(7), DISPC_CTRL);
	}else{
		dispc_clear_bits(BIT(7), DISPC_CTRL);
	}
}

static void dispc_set_exp_mode(uint16_t exp_mode)
{
	uint32_t reg_val = dispc_read(DISPC_CTRL);

	reg_val &= ~(0x3 << 16);
	reg_val |= (exp_mode << 16);
	dispc_write(reg_val, DISPC_CTRL);
}

static void dispc_module_enable(void)
{
	
	dispc_write((1<<0), DISPC_CTRL);

	
	dispc_write(0x0, DISPC_INT_EN);

	
	dispc_write(0x1F, DISPC_INT_CLR);
}

static inline int32_t  dispc_set_disp_size(struct fb_var_screeninfo *var)
{
	uint32_t reg_val;

	reg_val = (var->xres & 0xfff) | ((var->yres & 0xfff ) << 16);
	dispc_write(reg_val, DISPC_SIZE_XY);

	return 0;
}

static void dispc_layer_init(struct fb_var_screeninfo *var)
{
	uint32_t reg_val = 0;

	dispc_write(0x0, DISPC_IMG_CTRL);
	dispc_clear_bits((1<<0),DISPC_OSD_CTRL);

	

	
	reg_val |= (1 << 0);

	

	
	reg_val |= (1 << 2);

	
	if (var->bits_per_pixel == 32) {
		
		reg_val |= (3 << 4);
		
		reg_val |= (1 << 15);
	} else {
		
		reg_val |= (5 << 4);
		
		reg_val |= (2 << 8);
	}

	dispc_write(reg_val, DISPC_OSD_CTRL);

	
	dispc_write(0xff, DISPC_OSD_ALPHA);

	
	reg_val = ( var->xres & 0xfff) | (( var->yres & 0xfff ) << 16);
	dispc_write(reg_val, DISPC_OSD_SIZE_XY);

	
	dispc_write(0, DISPC_OSD_DISP_XY);

	
	reg_val = ( var->xres & 0xfff) ;
	dispc_write(reg_val, DISPC_OSD_PITCH);

	
	dispc_set_osd_ck(0x0);

	
	dispc_set_disp_size(var);
}

static void dispc_layer_update(struct fb_var_screeninfo *var)
{
	uint32_t reg_val = 0;

	

	
	reg_val |= (1 << 0);

	

	
	reg_val |= (1 << 2);

	
	if (var->bits_per_pixel == 32) {
		
		reg_val |= (3 << 4);
		
		reg_val |= (1 << 15);
	} else {
		
		reg_val |= (5 << 4);
		
		reg_val |= (2 << 8);
	}

	dispc_write(reg_val, DISPC_OSD_CTRL);
}

static int32_t dispc_sync(struct sprdfb_device *dev)
{
	int ret;

	if (dev->enable == 0) {
		printk("[DISP] sprdfb: dispc_sync fb suspeneded already!!\n");
		return -1;
	}

	ret = wait_event_interruptible_timeout(dispc_ctx.vsync_queue,
			          dispc_ctx.vsync_done, msecs_to_jiffies(100));

	if (!ret) { 
		dispc_ctx.vsync_done = 1; 
		printk(KERN_ERR "[DISP] sprdfb: dispc_sync time out!!!!!\n");
		{
			int32_t i;
			for(i=0;i<256;i+=16){
				printk("[DISP] sprdfb: %x: 0x%x, 0x%x, 0x%x, 0x%x\n", i, dispc_read(i), dispc_read(i+4), dispc_read(i+8), dispc_read(i+12));
			}
			printk("[DISP] **************************************\n");
		}

		return -1;
	}
	return 0;
}


static void dispc_run(struct sprdfb_device *dev)
{
	if(0 == dev->enable){
		return;
	}

	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		if(!dispc_ctx.is_first_frame){
			dispc_ctx.vsync_done = 0;
			dispc_ctx.vsync_waiter ++;
		}

		
		dispc_set_bits(BIT(5), DISPC_DPI_CTRL);

		udelay(30);

		if(dispc_ctx.is_first_frame){
			
			dispc_clear_bits(BIT(4), DISPC_DPI_CTRL);

			
			dispc_set_bits((1 << 4), DISPC_CTRL);

			dispc_ctx.is_first_frame = false;
		}else{
			dispc_sync(dev);
		}
	}else{
		dispc_ctx.vsync_done = 0;
		
		dispc_set_bits((1 << 4), DISPC_CTRL);
	}
}

static void dispc_stop(struct sprdfb_device *dev)
{
	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		
		dispc_set_bits(BIT(4), DISPC_DPI_CTRL);

		
		dispc_clear_bits((1 << 4), DISPC_CTRL);

		dispc_ctx.is_first_frame = true;
	}
}

static void dispc_update_clock(struct sprdfb_device *dev)
{
	uint32_t hpixels, vlines, need_clock,  dividor;
	int ret = 0;

	struct panel_spec* panel = dev->panel;
	struct info_mipi * mipi = panel->info.mipi;
	struct info_rgb* rgb = panel->info.rgb;

	pr_debug("[DISP] sprdfb:[%s]\n", __FUNCTION__);

	if(0 == panel->fps){
		printk("[DISP] sprdfb: No panel->fps specified!\n");
		return;
	}


	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		if(LCD_MODE_DSI == dev->panel->type ){
			hpixels = panel->width + mipi->timing->hsync + mipi->timing->hbp + mipi->timing->hfp;
			vlines = panel->height + mipi->timing->vsync + mipi->timing->vbp + mipi->timing->vfp;
		}else if(LCD_MODE_RGB == dev->panel->type ){
			hpixels = panel->width + rgb->timing->hsync + rgb->timing->hbp + rgb->timing->hfp;
			vlines = panel->height + rgb->timing->vsync + rgb->timing->vbp + rgb->timing->vfp;
		}else{
			printk("[DISP] sprdfb:[%s] unexpected panel type!(%d)\n", __FUNCTION__, dev->panel->type);
			return;
		}

		need_clock = hpixels * vlines * panel->fps;
		dividor  = SPRDFB_DPI_CLOCK_SRC/need_clock;
		if(SPRDFB_DPI_CLOCK_SRC - dividor*need_clock > (need_clock/2) ) {
			dividor += 1;
		}

		if((dividor < 1) || (dividor > 0x100)){
			printk("[DISP] sprdfb:[%s]: Invliad dividor(%d)!Not update dpi clock!\n", __FUNCTION__, dividor);
			return;
		}

		dev->dpi_clock = SPRDFB_DPI_CLOCK_SRC/dividor;

		ret = clk_set_rate(dispc_ctx.clk_dispc_dpi, dev->dpi_clock);
                dispc_print_clk();

		if(ret){
			printk(KERN_ERR "[DISP] sprdfb: dispc set dpi clk parent fail\n");
		}

		printk("[DISP] sprdfb:[%s] need_clock = %d, dividor = %d, dpi_clock = %d\n", __FUNCTION__, need_clock, dividor, dev->dpi_clock);
	}

}

static int32_t sprdfb_dispc_uninit(struct sprdfb_device *dev)
{
	pr_debug(KERN_INFO "[DISP] sprdfb:[%s]\n",__FUNCTION__);

	sprdfb_enable = 0;
	dev->enable = 0;
	sprdfb_dispc_clk_disable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_FORCE);

	return 0;
}

static int32_t dispc_clk_init(struct sprdfb_device *dev)
{
	int ret = 0;
	struct clk *clk_parent1, *clk_parent2, *clk_parent3, *clk_parent4;

	pr_debug(KERN_INFO "[DISP] sprdfb:[%s]\n", __FUNCTION__);

	dispc_print_clk();
	pr_debug("[DISP] sprdfb:BIT_DISPC_CORE_EN:%lx,DISPC_CORE_EN:%x\n",BIT_DISPC_CORE_EN,DISPC_CORE_EN);
	pr_debug("[DISP] sprdfb:BIT_DISPC_EMC_EN:%lx,DISPC_EMC_EN:%x\n",BIT_DISPC_EMC_EN,DISPC_EMC_EN);
	pr_debug("[DISP] sprdfb:DISPC_CORE_EN:%x\n",dispc_glb_read(DISPC_CORE_EN));
	pr_debug("[DISP] sprdfb:DISPC_EMC_EN:%x\n",dispc_glb_read(DISPC_EMC_EN));
	sci_glb_set(DISPC_CORE_EN, BIT_DISPC_CORE_EN);

	pr_debug("[DISP] sprdfb:DISPC_CORE_EN:%x\n",dispc_glb_read(DISPC_CORE_EN));
	pr_debug("[DISP] sprdfb:DISPC_EMC_EN:%x\n",dispc_glb_read(DISPC_EMC_EN));

	clk_parent1 = clk_get(NULL, DISPC_CLOCK_PARENT);
	if (IS_ERR(clk_parent1)) {
		printk(KERN_WARNING "[DISP] sprdfb: get clk_parent1 fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "[DISP] sprdfb: get clk_parent1 ok!\n");
	}

	clk_parent2 = clk_get(NULL, DISPC_DBI_CLOCK_PARENT);
	if (IS_ERR(clk_parent2)) {
		printk(KERN_WARNING "[DISP] sprdfb: get clk_parent2 fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "[DISP] sprdfb: get clk_parent2 ok!\n");
	}

	clk_parent3 = clk_get(NULL, DISPC_DPI_CLOCK_PARENT);
	if (IS_ERR(clk_parent3)) {
		printk(KERN_WARNING "[DISP] sprdfb: get clk_parent3 fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "[DISP] sprdfb: get clk_parent3 ok!\n");
	}

	clk_parent4 = clk_get(NULL, DISPC_EMC_EN_PARENT);
	if (IS_ERR(clk_parent4)) {
		printk(KERN_WARNING "[DISP] sprdfb: get clk_parent4 fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "[DISP] sprdfb: get clk_parent4 ok!\n");
	}

	dispc_ctx.clk_dispc = clk_get(NULL, DISPC_PLL_CLK);
	if (IS_ERR(dispc_ctx.clk_dispc)) {
		printk(KERN_WARNING "[DISP] sprdfb: get clk_dispc fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "[DISP] sprdfb: get clk_dispc ok!\n");
	}

	dispc_ctx.clk_dispc_dbi = clk_get(NULL, DISPC_DBI_CLK);
	if (IS_ERR(dispc_ctx.clk_dispc_dbi)) {
		printk(KERN_WARNING "[DISP] sprdfb: get clk_dispc_dbi fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "[DISP] sprdfb: get clk_dispc_dbi ok!\n");
	}

	dispc_ctx.clk_dispc_dpi = clk_get(NULL, DISPC_DPI_CLK);
	if (IS_ERR(dispc_ctx.clk_dispc_dpi)) {
		printk(KERN_WARNING "[DISP] sprdfb: get clk_dispc_dpi fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "[DISP] sprdfb: get clk_dispc_dpi ok!\n");
	}

	dispc_ctx.clk_dispc_emc = clk_get(NULL, DISPC_EMC_CLK);
	if (IS_ERR(dispc_ctx.clk_dispc_emc)) {
		printk(KERN_WARNING "[DISP] sprdfb: get clk_dispc_dpi fail!\n");
		return -1;
	} else {
		pr_debug(KERN_INFO "[DISP] sprdfb: get clk_dispc_emc ok!\n");
	}

	ret = clk_set_parent(dispc_ctx.clk_dispc, clk_parent1);
	if(ret){
		printk(KERN_ERR "[DISP] sprdfb: dispc set clk parent fail\n");
	}
	ret = clk_set_rate(dispc_ctx.clk_dispc, DISPC_CLOCK);
	if(ret){
		printk(KERN_ERR "[DISP] sprdfb: dispc set clk parent fail\n");
	}

	ret = clk_set_parent(dispc_ctx.clk_dispc_dbi, clk_parent2);
	if(ret){
		printk(KERN_ERR "[DISP] sprdfb: dispc set dbi clk parent fail\n");
	}
	ret = clk_set_rate(dispc_ctx.clk_dispc_dbi, DISPC_DBI_CLOCK);
	if(ret){
		printk(KERN_ERR "[DISP] sprdfb: dispc set dbi clk parent fail\n");
	}

	ret = clk_set_parent(dispc_ctx.clk_dispc_dpi, clk_parent3);
	if(ret){
		printk(KERN_ERR "[DISP] sprdfb: dispc set dpi clk parent fail\n");
	}

	ret = clk_set_parent(dispc_ctx.clk_dispc_emc, clk_parent4);
	if(ret){
		printk(KERN_ERR "[DISP] sprdfb: dispc set emc clk parent fail\n");
	}

	if(0 != dev->panel->fps){
		dispc_update_clock(dev);
	}else{
		dev->dpi_clock = DISPC_DPI_CLOCK;
		ret = clk_set_rate(dispc_ctx.clk_dispc_dpi, DISPC_DPI_CLOCK);
		if(ret){
			printk(KERN_ERR "[DISP] sprdfb: dispc set dpi clk parent fail\n");
		}
	}

	ret = clk_enable(dispc_ctx.clk_dispc_emc);
	if(ret){
		printk("[DISP] sprdfb:enable clk_dispc_emc error!!!\n");
		ret=-1;
	}

	ret = sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_FORCE);
	if (ret) {
		printk(KERN_WARNING "[DISP] sprdfb:[%s] enable dispc_clk fail!\n",__FUNCTION__);
		return -1;
	} else {
		pr_debug(KERN_INFO "[DISP] sprdfb:[%s] enable dispc_clk ok!\n",__FUNCTION__);
	}

	dispc_print_clk();

	return 0;
}

struct devfreq_dbs sprd_fb_notify = {
	.level = 0,
	.data = &dispc_ctx,
	.devfreq_notifier = sprdfb_dispc_change_threshold,
};

static int32_t sprdfb_dispc_module_init(struct sprdfb_device *dev)
{
	int ret = 0;

	if(dispc_ctx.is_inited){
		printk(KERN_WARNING "[DISP] sprdfb: dispc_module has already initialized! warning!!");
		return 0;
	}
	else{
		printk(KERN_INFO "[DISP] sprdfb: dispc_module_init. call only once!");
	}

	dispc_ctx.is_resume=false;

	dispc_ctx.vsync_done = 1;
	dispc_ctx.vsync_waiter = 0;
	init_waitqueue_head(&(dispc_ctx.vsync_queue));

#ifdef CONFIG_FB_ESD_SUPPORT
#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
	init_waitqueue_head(&(dev->esd_te_queue));
	dev->esd_te_waiter = 0;
	dev->esd_te_done = 0;
#endif
#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
	dispc_ctx.waitfor_vsync_done = 0;
	dispc_ctx.waitfor_vsync_waiter = 0;
	init_waitqueue_head(&(dispc_ctx.waitfor_vsync_queue));
#endif
	sema_init(&dev->refresh_lock, 1);

	ret = request_irq(IRQ_DISPC_INT, dispc_isr, IRQF_DISABLED, "DISPC", &dispc_ctx);
	if (ret) {
		printk(KERN_ERR "[DISP] sprdfb: dispcfailed to request irq!\n");
		sprdfb_dispc_uninit(dev);
		return -1;
	}

	dispc_ctx.is_inited = true;


	return 0;

}

static int32_t sprdfb_dispc_early_init(struct sprdfb_device *dev)
{
	int ret = 0;

	if(!dispc_ctx.is_inited){
		spin_lock_init(&dispc_ctx.clk_spinlock);
    }

	ret = dispc_clk_init(dev);
	if(ret){
		printk(KERN_WARNING "[DISP] sprdfb: dispc_clk_init fail!\n");
		return -1;
	}

	if(!dispc_ctx.is_inited){
		
		if(dev->panel_ready){
			
			printk(KERN_INFO "[DISP] sprdfb:[%s]: dispc has alread initialized\n", __FUNCTION__);
			dispc_ctx.is_first_frame = false;
		}else{
			
			printk(KERN_INFO "[DISP] sprdfb:[%s]: dispc is not initialized\n", __FUNCTION__);
			dispc_reset();
			dispc_module_enable();
			dispc_ctx.is_first_frame = true;
		}
		ret = sprdfb_dispc_module_init(dev);
	}else{
		
		printk(KERN_INFO "[DISP] sprdfb:[%s]: sprdfb_dispc_early_init resume\n", __FUNCTION__);
		dispc_reset();
		dispc_module_enable();
		dispc_ctx.is_first_frame = true;
	}

	return ret;
}


static int sprdfb_dispc_clk_disable(struct sprdfb_dispc_context *dispc_ctx_ptr, SPRDFB_DYNAMIC_CLK_SWITCH_E clock_switch_type)
{
	bool is_need_disable=false;
	unsigned long irqflags;

	pr_debug(KERN_INFO "sprdfb:[%s]\n",__FUNCTION__);
	if(!dispc_ctx_ptr){
		return 0;
	}

	spin_lock_irqsave(&dispc_ctx.clk_spinlock, irqflags);
	switch(clock_switch_type){
		case SPRDFB_DYNAMIC_CLK_FORCE:
			is_need_disable=true;
			break;
		case SPRDFB_DYNAMIC_CLK_REFRESH:
			dispc_ctx_ptr->clk_is_refreshing=false;
			if(dispc_ctx_ptr->clk_open_count<=0){
				is_need_disable=true;
			}
			break;
		case SPRDFB_DYNAMIC_CLK_COUNT:
			if(dispc_ctx_ptr->clk_open_count>0){
				dispc_ctx_ptr->clk_open_count--;
				if(dispc_ctx_ptr->clk_open_count==0){
					if(!dispc_ctx_ptr->clk_is_refreshing){
						is_need_disable=true;
					}
				}
			}
			break;
		default:
			break;
	}

	if(dispc_ctx_ptr->clk_is_open && is_need_disable){
		pr_debug(KERN_INFO "sprdfb:sprdfb_dispc_clk_disable real\n");
		clk_disable(dispc_ctx_ptr->clk_dispc);
		clk_disable(dispc_ctx_ptr->clk_dispc_dpi);
		clk_disable(dispc_ctx_ptr->clk_dispc_dbi);
		dispc_ctx_ptr->clk_is_open=false;
		dispc_ctx_ptr->clk_is_refreshing=false;
		dispc_ctx_ptr->clk_open_count=0;
	}

	spin_unlock_irqrestore(&dispc_ctx.clk_spinlock,irqflags);

	pr_debug(KERN_INFO "sprdfb:sprdfb_dispc_clk_disable type=%d refresh=%d,count=%d\n",clock_switch_type,dispc_ctx_ptr->clk_is_refreshing,dispc_ctx_ptr->clk_open_count);
	return 0;
}

static int sprdfb_dispc_clk_enable(struct sprdfb_dispc_context *dispc_ctx_ptr, SPRDFB_DYNAMIC_CLK_SWITCH_E clock_switch_type)
{
	int ret = 0;
	bool is_dispc_enable=false;
	bool is_dispc_dpi_enable=false;
	unsigned long irqflags;

	pr_debug(KERN_INFO "sprdfb:[%s]\n",__FUNCTION__);
	if(!dispc_ctx_ptr){
		return -1;
	}

	spin_lock_irqsave(&dispc_ctx.clk_spinlock, irqflags);

	if(!dispc_ctx_ptr->clk_is_open){
		pr_debug(KERN_INFO "sprdfb:sprdfb_dispc_clk_enable real\n");
		ret = clk_enable(dispc_ctx_ptr->clk_dispc);
		if(ret){
			printk("sprdfb:enable clk_dispc error!!!\n");
			ret=-1;
			goto ERROR_CLK_ENABLE;
		}
		is_dispc_enable=true;
		ret = clk_enable(dispc_ctx_ptr->clk_dispc_dpi);
		if(ret){
			printk("sprdfb:enable clk_dispc_dpi error!!!\n");
			ret=-1;
			goto ERROR_CLK_ENABLE;
		}
		is_dispc_dpi_enable=true;
		ret = clk_enable(dispc_ctx_ptr->clk_dispc_dbi);
		if(ret){
			printk("sprdfb:enable clk_dispc_dbi error!!!\n");
			ret=-1;
			goto ERROR_CLK_ENABLE;
		}
		dispc_ctx_ptr->clk_is_open=true;
	}

	switch(clock_switch_type){
		case SPRDFB_DYNAMIC_CLK_FORCE:
			break;
		case SPRDFB_DYNAMIC_CLK_REFRESH:
			dispc_ctx_ptr->clk_is_refreshing=true;
			break;
		case SPRDFB_DYNAMIC_CLK_COUNT:
			dispc_ctx_ptr->clk_open_count++;
			break;
		default:
			break;
	}

	spin_unlock_irqrestore(&dispc_ctx.clk_spinlock,irqflags);

	pr_debug(KERN_INFO "sprdfb:sprdfb_dispc_clk_enable type=%d refresh=%d,count=%d,ret=%d\n",clock_switch_type,dispc_ctx_ptr->clk_is_refreshing,dispc_ctx_ptr->clk_open_count,ret);
	return ret;

ERROR_CLK_ENABLE:
	if(is_dispc_enable){
		clk_disable(dispc_ctx_ptr->clk_dispc);
	}
	if(is_dispc_dpi_enable){
		clk_disable(dispc_ctx_ptr->clk_dispc_dpi);
	}

	spin_unlock_irqrestore(&dispc_ctx.clk_spinlock,irqflags);

	printk("sprdfb:sprdfb_dispc_clk_enable error!!!!!!\n");
	return ret;
}

static int32_t sprdfb_dispc_init(struct sprdfb_device *dev)
{
	pr_debug(KERN_INFO "[DISP] sprdfb:[%s]\n",__FUNCTION__);
#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	dispc_ctx.firstswitch = 0;
	dispc_ctx.lastcolor = SPRD_DATA_TYPE_LIMIT;
#endif
	
	dispc_write(0x9600960, 0x0c);
	
	dispc_set_bg_color(0xFFFFFFFF);
	
	dispc_dithering_enable(true);
	
	dispc_set_exp_mode(0x0);
	
	dispc_pwr_enable(true);

	if(dispc_ctx.is_first_frame){
		dispc_layer_init(&(dev->fb->var));
	}else{
		dispc_layer_update(&(dev->fb->var));
	}


	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		if(dispc_ctx.is_first_frame){
			
			dispc_set_bits(BIT(4), DISPC_DPI_CTRL);
		}else{
			
			dispc_clear_bits(BIT(4), DISPC_DPI_CTRL);
		}
		
		dispc_write((1<<4), DISPC_INT_EN);
	}else{
		
		dispc_write((1<<0), DISPC_INT_EN);
	}
	dispc_set_bits(BIT(2), DISPC_INT_EN);
	dev->enable = 1;
	sprdfb_enable = 1;
	return 0;
}


#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
static int refresh_count = 0;
static bool dim_enable = false;
#define REFRESH_COUNT    15
static LCM_Init_Code set_led_ctl =  {LCM_SEND(2), {0x53, 0x2C}};

extern struct ops_mipi sprdfb_mipi_ops;
#endif
static int32_t sprdfb_dispc_refresh (struct sprdfb_device *dev)
{
#ifdef  BIT_PER_PIXEL_SURPPORT
	uint32_t reg_val = 0;
#endif
	struct fb_info *fb = dev->fb;
#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	unsigned long flags;
#endif

#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	mipi_gen_write_t mipi_gen_write = sprdfb_mipi_ops.mipi_gen_write;
#endif

	uint32_t base = fb->fix.smem_start + fb->fix.line_length * fb->var.yoffset;
	ktime_t start_time, end_time;
	s64 delta_time;
	start_time = ktime_get();
	pr_debug(KERN_INFO "[DISP] sprdfb:[%s]\n",__FUNCTION__);

	down(&dev->refresh_lock);
	if(0 == dev->enable){
		printk("[DISP] sprdfb: [%s]: do not refresh in suspend!!!\n", __FUNCTION__);
		goto ERROR_REFRESH;
	}

	if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
		dispc_ctx.vsync_waiter ++;
		dispc_sync(dev);
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_REFRESH)){
			printk(KERN_WARNING "sprdfb: enable dispc_clk fail in refresh!\n");
			goto ERROR_REFRESH;
		}
#endif
	}

	pr_debug(KERN_INFO "[DISP] srpdfb: [%s] got sync\n", __FUNCTION__);

	dispc_ctx.dev = dev;
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	if(SPRD_OVERLAY_STATUS_STARTED == dispc_ctx.overlay_state){
		overlay_close(dev);
	}
#endif
#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	if(dispc_ctx.firstswitch == 0
		&& ((dispc_ctx.lastcolor != SPRD_DATA_TYPE_RGB888) && (dispc_ctx.lastcolor != SPRD_DATA_TYPE_LIMIT))){
		printk("[DISP] %s() switch color\n", __func__);
		dispc_ctx.firstswitch = 1;
	}
		dispc_ctx.lastcolor = SPRD_DATA_TYPE_RGB888;
#endif
#ifdef LCD_UPDATE_PARTLY
	if ((fb->var.reserved[0] == 0x6f766572) &&(SPRDFB_PANEL_IF_DPI != dev->panel_if_type)) {
		uint32_t x,y, width, height;

		x = fb->var.reserved[1] & 0xffff;
		y = fb->var.reserved[1] >> 16;
		width  = fb->var.reserved[2] &  0xffff;
		height = fb->var.reserved[2] >> 16;

		base += ((x + y * fb->var.xres) * fb->var.bits_per_pixel / 8);
		dispc_write(base, DISPC_OSD_BASE_ADDR);
		dispc_write(0, DISPC_OSD_DISP_XY);
		dispc_write(fb->var.reserved[2], DISPC_OSD_SIZE_XY);
		dispc_write(fb->var.xres, DISPC_OSD_PITCH);

		dispc_write(fb->var.reserved[2], DISPC_SIZE_XY);

		sprdfb_panel_invalidate_rect(dev->panel,
					x, y, x+width-1, y+height-1);
	} else
#endif
	{
		uint32_t size = (fb->var.xres & 0xffff) | ((fb->var.yres) << 16);

		dispc_write(base, DISPC_OSD_BASE_ADDR);
		dispc_write(0, DISPC_OSD_DISP_XY);
		dispc_write(size, DISPC_OSD_SIZE_XY);
		dispc_write(fb->var.xres, DISPC_OSD_PITCH);

		dispc_write(size, DISPC_SIZE_XY);

#ifdef  BIT_PER_PIXEL_SURPPORT
	        
	        if (fb->var.bits_per_pixel == 32) {
		        
		        reg_val |= (3 << 4);
		        
		        reg_val |= (1 << 15);
		        dispc_clear_bits(0x30000,DISPC_CTRL);
	        } else {
		        
		        reg_val |= (5 << 4);
		        
		        reg_val |= (2 << 8);

		        dispc_clear_bits(0x30000,DISPC_CTRL);
		        dispc_set_bits(0x10000,DISPC_CTRL);
	        }
	        reg_val |= (1 << 0);

	        
	        reg_val |= (1 << 2);

	        dispc_write(reg_val, DISPC_OSD_CTRL);
#endif

		if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
			sprdfb_panel_invalidate(dev->panel);
		}
	}

	sprdfb_panel_before_refresh(dev);

#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	dispc_set_bits(BIT(0), DISPC_OSD_CTRL);
	if(SPRD_OVERLAY_STATUS_ON == dispc_ctx.overlay_state){
		overlay_start(dev, (SPRD_LAYER_IMG));
	}
#endif
#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	if( dispc_ctx.firstswitch != 0){
		dispc_ctx.firstswitch = 0;
		local_irq_save(flags);
		dispc_stop_for_switch_color(dev);
		dispc_run_for_feature(dev);
		local_irq_restore(flags);
		printk("[DISP] %s() stop dispc for color switch\n", __func__);
	}
	else{
		dispc_run(dev);
	}
#else
	dispc_run(dev);
#endif

#ifdef CONFIG_FB_ESD_SUPPORT
	if(!dev->ESD_work_start){
		pr_debug("[DISP] sprdfb: schedule ESD work queue!\n");
		schedule_delayed_work(&dev->ESD_work, msecs_to_jiffies(dev->ESD_timeout_val));
		dev->ESD_work_start = true;
	}

	if (request_display_on) {
		hr_msleep(100);
		set_backlight(true);
		request_display_on = 0;
	}
#endif

ERROR_REFRESH:
	up(&dev->refresh_lock);
    if(dev->panel->is_clean_lcd){
		if(dispc_ctx.is_resume){
			dispc_osd_enable(true);
			dispc_ctx.is_resume =false;
		}
    }

#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	if (!dim_enable) {
		refresh_count++;
		if (refresh_count > REFRESH_COUNT) {
			mipi_gen_write(set_led_ctl.data, (set_led_ctl.tag & LCM_TAG_MASK));
			dim_enable = true;
		}
	}
#endif

	pr_debug("[DISP] DISPC_CTRL: 0x%x\n", dispc_read(DISPC_CTRL));
	pr_debug("[DISP] DISPC_SIZE_XY: 0x%x\n", dispc_read(DISPC_SIZE_XY));

	pr_debug("[DISP] DISPC_BG_COLOR: 0x%x\n", dispc_read(DISPC_BG_COLOR));

	pr_debug("[DISP] DISPC_INT_EN: 0x%x\n", dispc_read(DISPC_INT_EN));

	pr_debug("[DISP] DISPC_OSD_CTRL: 0x%x\n", dispc_read(DISPC_OSD_CTRL));
	pr_debug("[DISP] DISPC_OSD_BASE_ADDR: 0x%x\n", dispc_read(DISPC_OSD_BASE_ADDR));
	pr_debug("[DISP] DISPC_OSD_SIZE_XY: 0x%x\n", dispc_read(DISPC_OSD_SIZE_XY));
	pr_debug("[DISP] DISPC_OSD_PITCH: 0x%x\n", dispc_read(DISPC_OSD_PITCH));
	pr_debug("[DISP] DISPC_OSD_DISP_XY: 0x%x\n", dispc_read(DISPC_OSD_DISP_XY));
	pr_debug("[DISP] DISPC_OSD_ALPHA	: 0x%x\n", dispc_read(DISPC_OSD_ALPHA));

	end_time = ktime_get();
	delta_time = ktime_to_ms(ktime_sub(end_time, start_time));
	if (delta_time > 100)
		printk(KERN_DEBUG "[DISP] %s() start time = %lld, end time = %lld, delta time = %lld\n",
			__func__, ktime_to_us(start_time), ktime_to_us(end_time),delta_time);
	return 0;
}

static int32_t sprdfb_dispc_suspend(struct sprdfb_device *dev)
{
	printk(KERN_INFO "[DISP] sprdfb:[%s], dev->enable = %d\n",__FUNCTION__, dev->enable);

	if (0 != dev->enable){
		down(&dev->refresh_lock);
		if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
			
			dispc_ctx.vsync_waiter ++;
			dispc_sync(dev);
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
			printk("[DISP] sprdfb: open clk in suspend\n");
			if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_COUNT)){
				printk(KERN_WARNING "sprdfb:[%s] clk enable fail!!!\n",__FUNCTION__);
			}
#endif
			printk(KERN_INFO "[DISP] sprdfb:[%s] got sync\n",__FUNCTION__);
		}

		sprdfb_enable = 0;
		dev->enable = 0;
		up(&dev->refresh_lock);

#ifdef CONFIG_FB_ESD_SUPPORT
		if(dev->ESD_work_start == true){
			pr_debug("[DISP] sprdfb: cancel ESD work queue\n");
			cancel_delayed_work_sync(&dev->ESD_work);
			dev->ESD_work_start = false;
		}

#endif

		sprdfb_panel_suspend(dev);

		dispc_stop(dev);

		hr_msleep(50); 

		clk_disable(dispc_ctx.clk_dispc_emc);
		sprdfb_dispc_clk_disable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_FORCE);
		
		

#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
		refresh_count = 0;
		dim_enable = false;
#endif
	}else{
		printk(KERN_ERR "[DISP] sprdfb: [%s]: Invalid device status %d\n", __FUNCTION__, dev->enable);
	}
	return 0;
}

static int32_t sprdfb_dispc_resume(struct sprdfb_device *dev)
{
	printk(KERN_INFO "[DISP] sprdfb:[%s], dev->enable= %d\n",__FUNCTION__, dev->enable);

	if (dev->enable == 0) {
		if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_FORCE)){
			printk(KERN_WARNING "sprdfb:[%s] clk enable fail!!\n",__FUNCTION__);
			
		}
		

		dispc_ctx.vsync_done = 1;
		if (1){
			printk(KERN_INFO "sprdfb:[%s] from deep sleep\n",__FUNCTION__);
			sprdfb_dispc_early_init(dev);
			sprdfb_dispc_init(dev);
			sprdfb_panel_resume(dev, true);
		} else {
			printk(KERN_INFO "[DISP] sprdfb:[%s]  not from deep sleep\n",__FUNCTION__);
			sprdfb_panel_resume(dev, true);
		}

		dev->enable = 1;
		sprdfb_enable = 1;
		if(dev->panel->is_clean_lcd){
			dispc_osd_enable(false);
			dispc_set_bg_color(0x00);
			sprdfb_dispc_refresh(dev);
			hr_msleep(30);
			dispc_ctx.is_resume=true;
		}
#ifdef CONFIG_MACH_DUMMY
		sprdfb_dispc_refresh(dev);
#endif
	}
	printk(KERN_INFO "[DISP] sprdfb:[%s], leave dev->enable= %d\n",__FUNCTION__, dev->enable);

	return 0;
}


#ifdef CONFIG_FB_ESD_SUPPORT
static int32_t sprdfb_dispc_check_esd_dpi(struct sprdfb_device *dev)
{
	uint32_t ret = 0;

#ifdef FB_CHECK_ESD_BY_TE_SUPPORT
	ret = sprdfb_panel_ESD_check(dev);
	if(0 !=ret){
		dispc_run_for_feature(dev);
	}
#else
	unsigned long flags;

	local_irq_save(flags);
	dispc_stop_for_feature(dev);

	ret = sprdfb_panel_ESD_check(dev);	

	dispc_run_for_feature(dev);
	local_irq_restore(flags);
#endif

	return ret;
}

static int32_t sprdfb_dispc_check_esd_edpi(struct sprdfb_device *dev)
{
	uint32_t ret = 0;

	dispc_ctx.vsync_waiter ++;
	dispc_sync(dev);
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
	if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_COUNT)){
		printk(KERN_WARNING "sprdfb:[%s] clk enable fail!!!\n",__FUNCTION__);
		return -1;
	}
#endif

	ret = sprdfb_panel_ESD_check(dev);

	if(0 !=ret){
		dispc_run_for_feature(dev);
	}
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
	sprdfb_dispc_clk_disable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_COUNT);
#endif

	return ret;
}

static int32_t sprdfb_dispc_check_esd(struct sprdfb_device *dev)
{
	uint32_t ret = 0;
	bool	is_refresh_lock_down=false;

	pr_debug("[DISP] sprdfb: [%s] \n", __FUNCTION__);

	if(SPRDFB_PANEL_IF_DBI == dev->panel_if_type){
		pr_debug("[DISP] sprdfb: [%s] leave (not support dbi mode now)!\n", __FUNCTION__);
		ret = -1;
		goto ERROR_CHECK_ESD;
	}
	down(&dev->refresh_lock);
	is_refresh_lock_down=true;
	if(0 == dev->enable){
		pr_debug("[DISP] sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
		ret=-1;
		goto ERROR_CHECK_ESD;
	}

	pr_debug("[DISP] sprdfb: [%s] (%d, %d, %d)\n",__FUNCTION__, dev->check_esd_time, dev->panel_reset_time, dev->reset_dsi_time);
	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		ret=sprdfb_dispc_check_esd_dpi(dev);
	}
	else{
		ret=sprdfb_dispc_check_esd_edpi(dev);
	}

ERROR_CHECK_ESD:
	if(is_refresh_lock_down){
		up(&dev->refresh_lock);
 	}

	if (0 != ret) {
		sprdfb_dispc_refresh(dev);
	}

	printk("[DISP] sprdfb: [%s] done!\n", __FUNCTION__);

	return ret;
}

int esd_suspend_resume(struct sprdfb_device *dev) {
	pr_debug("[DISP] sprdfb: [%s] \n", __FUNCTION__);

	if (0 != dev->enable) {
		sprdfb_enable = 0;
		dev->enable = 0;
		sprdfb_panel_suspend(dev);
#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY)
		dispc_stop(dev);
		hr_msleep(50);
#endif
		sprdfb_dispc_clk_disable(&dispc_ctx, SPRDFB_DYNAMIC_CLK_FORCE);
	} else {
		printk(KERN_ERR "[DISP] sprdfb: [%s]: Invalid device status(Suspend)%d\n", __FUNCTION__, dev->enable);
		return 0;
	}

	hr_msleep(100);

	if (dev->enable == 0) {
		if(sprdfb_dispc_clk_enable(&dispc_ctx, SPRDFB_DYNAMIC_CLK_FORCE)){
			printk(KERN_WARNING "sprdfb:[%s] clk enable fail!!\n", __FUNCTION__);
		}

		dispc_ctx.vsync_done = 1;

		sprdfb_dispc_early_init(dev);
		sprdfb_dispc_init(dev);
		sprdfb_panel_resume(dev, true);

		dev->enable = 1;
		sprdfb_enable = 1;
		request_display_on = 1;
	} else {
		printk(KERN_ERR "[DISP] sprdfb: [%s]: Invalid device status(Resume) %d\n", __FUNCTION__, dev->enable);
		return 0;
	}

	return 1;
}
#endif


#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
static int overlay_open(void)
{
	pr_debug("[DISP] sprdfb: [%s] : %d\n", __FUNCTION__,dispc_ctx.overlay_state);


	dispc_ctx.overlay_state = SPRD_OVERLAY_STATUS_ON;
	return 0;
}

static int overlay_start(struct sprdfb_device *dev, uint32_t layer_index)
{
	pr_debug("[DISP] sprdfb: [%s] : %d, %d\n", __FUNCTION__,dispc_ctx.overlay_state, layer_index);


	if(SPRD_OVERLAY_STATUS_ON  != dispc_ctx.overlay_state){
		printk(KERN_ERR "[DISP] sprdfb: overlay start fail. (not opened)");
		return -1;
	}

	if((0 == dispc_read(DISPC_IMG_Y_BASE_ADDR)) && (0 == dispc_read(DISPC_OSD_BASE_ADDR))){
		printk(KERN_ERR "[DISP] sprdfb: overlay start fail. (not configged)");
		return -1;
	}

	dispc_set_bg_color(0x0);
	dispc_clear_bits(BIT(2), DISPC_OSD_CTRL); 
	dispc_write(0x80, DISPC_OSD_ALPHA);

	if((layer_index & SPRD_LAYER_IMG) && (0 != dispc_read(DISPC_IMG_Y_BASE_ADDR))){
		dispc_set_bits(BIT(0), DISPC_IMG_CTRL);
	}
	if((layer_index & SPRD_LAYER_OSD) && (0 != dispc_read(DISPC_OSD_BASE_ADDR))){
		dispc_set_bits(BIT(0), DISPC_OSD_CTRL);
	}
	dispc_ctx.overlay_state = SPRD_OVERLAY_STATUS_STARTED;
	return 0;
}

static int overlay_img_configure(struct sprdfb_device *dev, int type, overlay_rect *rect, unsigned char *buffer, int y_endian, int uv_endian, bool rb_switch)
{
	uint32_t reg_value;

	pr_debug("[DISP] sprdfb: [%s] : %d, (%d, %d,%d,%d), 0x%x\n", __FUNCTION__, type, rect->x, rect->y, rect->h, rect->w, (unsigned int)buffer);


	if(SPRD_OVERLAY_STATUS_ON  != dispc_ctx.overlay_state){
		printk(KERN_ERR "[DISP] sprdfb: Overlay config fail (not opened)");
		return -1;
	}

	if (type >= SPRD_DATA_TYPE_LIMIT) {
		printk(KERN_ERR "[DISP] sprdfb: Overlay config fail (type error)");
		return -1;
	}

	if((y_endian >= SPRD_IMG_DATA_ENDIAN_LIMIT) || (uv_endian >= SPRD_IMG_DATA_ENDIAN_LIMIT)){
		printk(KERN_ERR "[DISP] sprdfb: Overlay config fail (y, uv endian error)");
		return -1;
	}
#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	if(dispc_ctx.firstswitch == 0
		&& ((dispc_ctx.lastcolor != type) && (dispc_ctx.lastcolor != SPRD_DATA_TYPE_LIMIT))){
		printk("[DISP] %s() switch color\n", __func__);
		dispc_ctx.firstswitch = 1;
	}
#endif

	
	reg_value = (y_endian << 8)|(uv_endian<< 10)|(type << 4);
	if(rb_switch){
		reg_value |= (1 << 15);
	}
	dispc_write(reg_value, DISPC_IMG_CTRL);
#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	dispc_ctx.lastcolor = type;
#endif
	dispc_write((uint32_t)buffer, DISPC_IMG_Y_BASE_ADDR);
	if (type < SPRD_DATA_TYPE_RGB888) {
		uint32_t size = rect->w * rect->h;
		dispc_write((uint32_t)(buffer + size), DISPC_IMG_UV_BASE_ADDR);
	}

	reg_value = (rect->h << 16) | (rect->w);
	dispc_write(reg_value, DISPC_IMG_SIZE_XY);

	dispc_write(rect->w, DISPC_IMG_PITCH);

	reg_value = (rect->y << 16) | (rect->x);
	dispc_write(reg_value, DISPC_IMG_DISP_XY);

	if(type < SPRD_DATA_TYPE_RGB888) {
		dispc_write(1, DISPC_Y2R_CTRL);
		dispc_write(SPRDFB_CONTRAST, DISPC_Y2R_CONTRAST);
		dispc_write(SPRDFB_SATURATION, DISPC_Y2R_SATURATION);
		dispc_write(SPRDFB_BRIGHTNESS, DISPC_Y2R_BRIGHTNESS);
	}

	pr_debug("[DISP] DISPC_IMG_CTRL: 0x%x\n", dispc_read(DISPC_IMG_CTRL));
	pr_debug("[DISP] DISPC_IMG_Y_BASE_ADDR: 0x%x\n", dispc_read(DISPC_IMG_Y_BASE_ADDR));
	pr_debug("[DISP] DISPC_IMG_UV_BASE_ADDR: 0x%x\n", dispc_read(DISPC_IMG_UV_BASE_ADDR));
	pr_debug("[DISP] DISPC_IMG_SIZE_XY: 0x%x\n", dispc_read(DISPC_IMG_SIZE_XY));
	pr_debug("[DISP] DISPC_IMG_PITCH: 0x%x\n", dispc_read(DISPC_IMG_PITCH));
	pr_debug("[DISP] DISPC_IMG_DISP_XY: 0x%x\n", dispc_read(DISPC_IMG_DISP_XY));
	pr_debug("[DISP] DISPC_Y2R_CTRL: 0x%x\n", dispc_read(DISPC_Y2R_CTRL));
	pr_debug("[DISP] DISPC_Y2R_CONTRAST: 0x%x\n", dispc_read(DISPC_Y2R_CONTRAST));
	pr_debug("[DISP] DISPC_Y2R_SATURATION: 0x%x\n", dispc_read(DISPC_Y2R_SATURATION));
	pr_debug("[DISP] DISPC_Y2R_BRIGHTNESS: 0x%x\n", dispc_read(DISPC_Y2R_BRIGHTNESS));

	return 0;
}

static int overlay_osd_configure(struct sprdfb_device *dev, int type, overlay_rect *rect, unsigned char *buffer, int y_endian, int uv_endian, bool rb_switch)
{
	uint32_t reg_value;

	pr_debug("[DISP] sprdfb: [%s] : %d, (%d, %d,%d,%d), 0x%x\n", __FUNCTION__, type, rect->x, rect->y, rect->h, rect->w, (unsigned int)buffer);


	if(SPRD_OVERLAY_STATUS_ON  != dispc_ctx.overlay_state){
		printk(KERN_ERR "[DISP] sprdfb: Overlay config fail (not opened)");
		return -1;
	}

	if ((type >= SPRD_DATA_TYPE_LIMIT) || (type <= SPRD_DATA_TYPE_YUV400)) {
		printk(KERN_ERR "[DISP] sprdfb: Overlay config fail (type error)");
		return -1;
	}

	if(y_endian >= SPRD_IMG_DATA_ENDIAN_LIMIT ){
		printk(KERN_ERR "[DISP] sprdfb: Overlay config fail (rgb endian error)");
		return -1;
	}

#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
if(dispc_ctx.firstswitch == 0
	&& ((dispc_ctx.lastcolor != type) && (dispc_ctx.lastcolor != SPRD_DATA_TYPE_LIMIT))){
	printk("[DISP] %s() switch color\n", __func__);
	dispc_ctx.firstswitch = 1;
}
#endif
	

	
	reg_value = (y_endian<<8)|(type << 4|(1<<2))|(2<<16);
	if(rb_switch){
		reg_value |= (1 << 15);
	}
	dispc_write(reg_value, DISPC_OSD_CTRL);
#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	dispc_ctx.lastcolor = type;
#endif
	dispc_write((uint32_t)buffer, DISPC_OSD_BASE_ADDR);

	reg_value = (rect->h << 16) | (rect->w);
	dispc_write(reg_value, DISPC_OSD_SIZE_XY);

	dispc_write(rect->w, DISPC_OSD_PITCH);

	reg_value = (rect->y << 16) | (rect->x);
	dispc_write(reg_value, DISPC_OSD_DISP_XY);


	pr_debug("[DISP] DISPC_OSD_CTRL: 0x%x\n", dispc_read(DISPC_OSD_CTRL));
	pr_debug("[DISP] DISPC_OSD_BASE_ADDR: 0x%x\n", dispc_read(DISPC_OSD_BASE_ADDR));
	pr_debug("[DISP] DISPC_OSD_SIZE_XY: 0x%x\n", dispc_read(DISPC_OSD_SIZE_XY));
	pr_debug("[DISP] DISPC_OSD_PITCH: 0x%x\n", dispc_read(DISPC_OSD_PITCH));
	pr_debug("[DISP] DISPC_OSD_DISP_XY: 0x%x\n", dispc_read(DISPC_OSD_DISP_XY));

	return 0;
}

static int overlay_close(struct sprdfb_device *dev)
{
	if(SPRD_OVERLAY_STATUS_OFF  == dispc_ctx.overlay_state){
		printk(KERN_ERR "[DISP] sprdfb: overlay close fail. (has been closed)");
		return 0;
	}

	dispc_set_bg_color(0xFFFFFFFF);
	dispc_set_bits(BIT(2), DISPC_OSD_CTRL);
	dispc_write(0xff, DISPC_OSD_ALPHA);
	dispc_clear_bits(BIT(0), DISPC_IMG_CTRL);	
	dispc_write(0, DISPC_IMG_Y_BASE_ADDR);
	dispc_write(0, DISPC_OSD_BASE_ADDR);
	dispc_layer_init(&(dev->fb->var));
	dispc_ctx.overlay_state = SPRD_OVERLAY_STATUS_OFF;

	return 0;
}

static int32_t sprdfb_dispc_enable_overlay(struct sprdfb_device *dev, struct overlay_info* info, int enable)
{
	int result = -1;
	bool	is_refresh_lock_down=false;
	bool	is_clk_enable=false;

	pr_debug("[DISP] sprdfb: [%s]: %d, %d\n", __FUNCTION__, enable,  dev->enable);

	if(enable){  
		if(NULL == info){
			printk(KERN_ERR "[DISP] sprdfb: sprdfb_dispc_enable_overlay fail (Invalid parameter)\n");
			goto ERROR_ENABLE_OVERLAY;
		}

		down(&dev->refresh_lock);
		is_refresh_lock_down=true;

		if(0 == dev->enable){
			printk(KERN_ERR "[DISP] sprdfb: sprdfb_dispc_enable_overlay fail. (dev not enable)\n");
			goto ERROR_ENABLE_OVERLAY;
		}

		if(0 != dispc_sync(dev)){
			printk(KERN_ERR "[DISP] sprdfb: sprdfb_dispc_enable_overlay fail. (wait done fail)\n");
			goto ERROR_ENABLE_OVERLAY;
		}

#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
			if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_COUNT)){
				printk(KERN_WARNING "[DISP] sprdfb:[%s] clk enable fail!!!\n",__FUNCTION__);
				goto ERROR_ENABLE_OVERLAY;
			}
			is_clk_enable=true;
		}
#endif
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
		if(SPRD_OVERLAY_STATUS_STARTED == dispc_ctx.overlay_state){
			overlay_close(dev);
		}
#endif
		result = overlay_open();
		if(0 != result){
			result=-1;
			goto ERROR_ENABLE_OVERLAY;
		}

		if(SPRD_LAYER_IMG == info->layer_index){
			result = overlay_img_configure(dev, info->data_type, &(info->rect), info->buffer, info->y_endian, info->uv_endian, info->rb_switch);
		}else if(SPRD_LAYER_OSD == info->layer_index){
			result = overlay_osd_configure(dev, info->data_type, &(info->rect), info->buffer, info->y_endian, info->uv_endian, info->rb_switch);
		}else{
			printk(KERN_ERR "[DISP] sprdfb: sprdfb_dispc_enable_overlay fail. (invalid layer index)\n");
		}
		if(0 != result){
			result=-1;
			goto ERROR_ENABLE_OVERLAY;
		}
		
	}else{   
		
	}
ERROR_ENABLE_OVERLAY:
	if(is_clk_enable){
		sprdfb_dispc_clk_disable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_COUNT);
	}
	if(is_refresh_lock_down){
		up(&dev->refresh_lock);
	}

	pr_debug("[DISP] sprdfb: [%s] return %d\n", __FUNCTION__, result);
	return result;
}


static int32_t sprdfb_dispc_display_overlay(struct sprdfb_device *dev, struct overlay_display* setting)
{
	struct overlay_rect* rect = &(setting->rect);
	uint32_t size =( (rect->h << 16) | (rect->w & 0xffff));
#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	unsigned long flags;
#endif
	dispc_ctx.dev = dev;

	pr_debug("[DISP] sprdfb: sprdfb_dispc_display_overlay: layer:%d, (%d, %d,%d,%d)\n",
		setting->layer_index, setting->rect.x, setting->rect.y, setting->rect.h, setting->rect.w);

	down(&dev->refresh_lock);
	if(0 == dev->enable){
		printk("[DISP] sprdfb: [%s] leave (Invalid device status)!\n", __FUNCTION__);
		goto ERROR_DISPLAY_OVERLAY;
	}
	if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
		dispc_ctx.vsync_waiter ++;
		dispc_sync(dev);
		
#ifdef CONFIG_FB_DYNAMIC_CLK_SUPPORT
		if(sprdfb_dispc_clk_enable(&dispc_ctx,SPRDFB_DYNAMIC_CLK_REFRESH)){
			printk(KERN_WARNING "sprdfb:[%s] clk enable fail!!!\n",__FUNCTION__);
			goto ERROR_DISPLAY_OVERLAY;
		}
#endif

	}

	pr_debug(KERN_INFO "[DISP] srpdfb: [%s] got sync\n", __FUNCTION__);

	dispc_ctx.dev = dev;

#ifdef LCD_UPDATE_PARTLY
	if ((setting->rect->h < dev->panel->height) ||
		(setting->rect->w < dev->panel->width)){
		dispc_write(size, DISPC_SIZE_XY);

		sprdfb_panel_invalidate_rect(dev->panel,
					rect->x, rect->y, rect->x + rect->w-1, rect->y + rect->h-1);
	} else
#endif
	{
		dispc_write(size, DISPC_SIZE_XY);

		if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
			sprdfb_panel_invalidate(dev->panel);
		}
	}

	sprdfb_panel_before_refresh(dev);

	dispc_clear_bits(BIT(0), DISPC_OSD_CTRL);
	if(SPRD_OVERLAY_STATUS_ON == dispc_ctx.overlay_state){
		overlay_start(dev, setting->layer_index);
	}
#if defined (CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
	if( dispc_ctx.firstswitch != 0){
		dispc_ctx.firstswitch = 0;
		local_irq_save(flags);
		dispc_stop_for_switch_color(dev);
		dispc_run_for_feature(dev);
		local_irq_restore(flags);
		printk("[DISP] %s() switch color\n", __func__);
	}
	else{
		dispc_run(dev);
	}
#else
	dispc_run(dev);
#endif

	if((SPRD_OVERLAY_DISPLAY_SYNC == setting->display_mode) && (SPRDFB_PANEL_IF_DPI != dev->panel_if_type)){
		dispc_ctx.vsync_waiter ++;
		if (dispc_sync(dev) != 0) {
			printk("[DISP] sprdfb  do sprd_lcdc_display_overlay  time out!\n");
		}
		
	}

ERROR_DISPLAY_OVERLAY:
	up(&dev->refresh_lock);

	pr_debug("[DISP] DISPC_CTRL: 0x%x\n", dispc_read(DISPC_CTRL));
	pr_debug("[DISP] DISPC_SIZE_XY: 0x%x\n", dispc_read(DISPC_SIZE_XY));

	pr_debug("[DISP] DISPC_BG_COLOR: 0x%x\n", dispc_read(DISPC_BG_COLOR));

	pr_debug("[DISP] DISPC_INT_EN: 0x%x\n", dispc_read(DISPC_INT_EN));

	pr_debug("[DISP] DISPC_OSD_CTRL: 0x%x\n", dispc_read(DISPC_OSD_CTRL));
	pr_debug("[DISP] DISPC_OSD_BASE_ADDR: 0x%x\n", dispc_read(DISPC_OSD_BASE_ADDR));
	pr_debug("[DISP] DISPC_OSD_SIZE_XY: 0x%x\n", dispc_read(DISPC_OSD_SIZE_XY));
	pr_debug("[DISP] DISPC_OSD_PITCH: 0x%x\n", dispc_read(DISPC_OSD_PITCH));
	pr_debug("[DISP] DISPC_OSD_DISP_XY: 0x%x\n", dispc_read(DISPC_OSD_DISP_XY));
	pr_debug("[DISP] DISPC_OSD_ALPHA	: 0x%x\n", dispc_read(DISPC_OSD_ALPHA));
	return 0;
}

#endif

#ifdef CONFIG_FB_VSYNC_SUPPORT
static int32_t spdfb_dispc_wait_for_vsync(struct sprdfb_device *dev)
{
	int ret = 0;
	pr_debug("[DISP] sprdfb: [%s]\n", __FUNCTION__);

	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		if(!dispc_ctx.is_first_frame){
			dispc_ctx.waitfor_vsync_done = 0;
			dispc_set_bits(BIT(0), DISPC_INT_EN);
			dispc_ctx.waitfor_vsync_waiter++;
			ret  = wait_event_interruptible_timeout(dispc_ctx.waitfor_vsync_queue,
					dispc_ctx.waitfor_vsync_done, msecs_to_jiffies(100));
			dispc_ctx.waitfor_vsync_waiter = 0;
		}
	}else{
		dispc_ctx.waitfor_vsync_done = 0;
		dispc_set_bits(BIT(1), DISPC_INT_EN);
		dispc_ctx.waitfor_vsync_waiter++;
		ret  = wait_event_interruptible_timeout(dispc_ctx.waitfor_vsync_queue,
				dispc_ctx.waitfor_vsync_done, msecs_to_jiffies(100));
		dispc_ctx.waitfor_vsync_waiter = 0;
	}
	pr_debug("[DISP] sprdfb:[%s] (%d)\n", __FUNCTION__, ret);
	return 0;
}
#endif

static void dispc_stop_for_feature(struct sprdfb_device *dev)
{
	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		dispc_stop(dev);
		while(dispc_read(DISPC_DPI_STS1) & BIT(16));
		udelay(25);
	}
}
static void dispc_stop_for_switch_color(struct sprdfb_device *dev)
{
	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		dispc_stop(dev);
		while(dispc_read(DISPC_DPI_STS1) & BIT(16));
	}
}


static void dispc_run_for_feature(struct sprdfb_device *dev)
{
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	if(SPRD_OVERLAY_STATUS_ON != dispc_ctx.overlay_state)
#endif
	{
		dispc_run(dev);
	}
}

#ifdef CONFIG_FB_DYNAMIC_FPS_SUPPORT
static int32_t sprdfb_update_fps_clock(struct sprdfb_device *dev, int fps_level)
{
    uint32_t fps,dpi_clock;
    struct panel_spec* panel = dev->panel;
    fps = panel->fps;
    dpi_clock = dev->dpi_clock;

    printk("[DISP] sprdfb:sprdfb_update_fps_clock--fps_level:%d \n",fps_level);
    if((fps_level < 40) || (fps_level > 64)) {
	printk("sprdfb: invalid fps set!\n");
	return -1;
    }

    panel->fps = fps_level;

    dispc_update_clock(dev);
    dsi_dpi_init(dev);

    panel->fps = fps;
    dev->dpi_clock = dpi_clock;
    return 0;
}


static int32_t sprdfb_dispc_change_fps(struct sprdfb_device *dev, int fps_level)
{
    int32_t ret = 0;
    if(NULL == dev || 0 == dev->enable){
		printk(KERN_ERR "[DISP] sprdfb: sprdfb_dispc_change_fps fail. (dev not enable)\n");
		return -1;
	}
    if(SPRDFB_PANEL_IF_DPI != dev->panel_if_type){
		dispc_ctx.vsync_waiter ++;
		dispc_sync(dev);
                sprdfb_panel_change_fps(dev,fps_level);
	}else{
	        down(&dev->refresh_lock);
		dispc_stop_for_feature(dev);

                ret = sprdfb_update_fps_clock(dev,fps_level);

                dispc_run_for_feature(dev);
	        up(&dev->refresh_lock);
	}
    return ret;
}
#endif

static unsigned int sprdfb_dispc_change_threshold(struct devfreq_dbs *h, unsigned int state)
{
	struct sprdfb_dispc_context *dispc_ctx = (struct sprdfb_dispc_context *)h->data;
	struct sprdfb_device *dev = dispc_ctx->dev;
	unsigned int ret = 0;
	if(NULL == dev || 0 == dev->enable){
		printk(KERN_ERR "sprdfb: sprdfb_dispc_change_threshold fail.(dev not enable)\n");
		return 1;
	}
	printk(KERN_ERR "sprdfb: sprdfb_dispc_change_threshold state=%u\n", state);
	if(SPRDFB_PANEL_IF_DPI == dev->panel_if_type){
		down(&dev->refresh_lock);
		dispc_stop_for_feature(dev);
		if(state == DEVFREQ_PRE_CHANGE){
			dispc_write(0x9600960, 0x0c);
		}else{
			dispc_write(0x5000500, 0x0c);
		}
		dispc_run_for_feature(dev);
		up(&dev->refresh_lock);
	}
	return ret;
}


struct display_ctrl sprdfb_dispc_ctrl = {
	.name		= "dispc",
	.early_init		= sprdfb_dispc_early_init,
	.init		 	= sprdfb_dispc_init,
	.uninit		= sprdfb_dispc_uninit,
	.refresh		= sprdfb_dispc_refresh,
	.suspend		= sprdfb_dispc_suspend,
	.resume		= sprdfb_dispc_resume,
	.update_clk	= dispc_update_clock,
#ifdef CONFIG_FB_ESD_SUPPORT
	.ESD_check	= sprdfb_dispc_check_esd,
#endif
#ifdef CONFIG_FB_LCD_OVERLAY_SUPPORT
	.enable_overlay = sprdfb_dispc_enable_overlay,
	.display_overlay = sprdfb_dispc_display_overlay,
#endif
#ifdef CONFIG_FB_VSYNC_SUPPORT
	.wait_for_vsync = spdfb_dispc_wait_for_vsync,
#endif
#ifdef CONFIG_FB_DYNAMIC_FPS_SUPPORT
    .change_fps = sprdfb_dispc_change_fps,
#endif
};


