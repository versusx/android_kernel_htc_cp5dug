/*slh: copy from drivers/video/sc8825/lcd_nt35512_mipi.c */
/* drivers/video/sc8825/lcd_nt35517_mipi.c
 *
 * Support for nt35517 mipi LCD device
 *
 * Copyright (C) 2010 Spreadtrum
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
#include <linux/delay.h>
#include "sprdfb_panel.h"

//#define LCD_Delay(ms)  uDelay(ms*1000)


static LCM_Init_Code init_data[] = {
    {LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x08,0x00}},
    {LCM_SEND(6), {4, 0, 0xB1,0x68,0x00,0x01}},
    {LCM_SEND(4), {2, 0, 0xB4,0x78}},
    {LCM_SEND(7), {5, 0, 0xB8,0x01,0x02,0x02,0x02}},
    {LCM_SEND(6), {4, 0, 0xBC,0x00,0x00,0x00}},
    {LCM_SEND(9), {7, 0, 0xC9,0x63,0x06,0x0D,0x1A,0x17,0x00}},
    {LCM_SEND(2), {0x35,0x00}}, // Tearing Effect On 
    {LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x08,0x01}},
    {LCM_SEND(2), {0xB6,0x44}},
    {LCM_SEND(2), {0xB6,0x44}},
    {LCM_SEND(2), {0xB6,0x44}},
    {LCM_SEND(2), {0xB7,0x44}},
    {LCM_SEND(2), {0xB8,0x13}},
    {LCM_SEND(2), {0xB9,0x24}},
    {LCM_SEND(2), {0xBA,0x23}},
    {LCM_SEND(6), {4, 0, 0xBC,0x00,0xB8,0x00}},
    {LCM_SEND(6), {4, 0, 0xBD,0x00,0xB4,0x00}},
    {LCM_SEND(2), {0xBE,0x6A}},
    {LCM_SEND(5), {3, 0, 0xC0,0x04,0x00}},
    {LCM_SEND(2), {0xCF,0x44}},
    {LCM_SEND(19), {17, 0, 0xD1, 0x00, 0x75, 0x00, 0xB4, 0x00, 0xD9, 0x00, 0xF8, 0x01, 0x06, 0x01,0x1F, 0x01, 0x31,0x01,0x52}},
    {LCM_SEND(19), {17, 0, 0xD2, 0x01, 0x76, 0x01, 0xAC, 0x01, 0xD1, 0x02,0x0C, 0x02, 0x3D, 0x02,0x41, 0x02, 0x6C, 0x02,0x9E}},
    {LCM_SEND(19), {17, 0, 0xD3, 0x02, 0xBA, 0x02, 0xEC, 0x03, 0x12, 0x03,0x34, 0x03, 0x4C, 0x03,0x66, 0x03, 0x74, 0x03,0x86}},
	{LCM_SEND(7), {5, 0, 0xD4,0x03,0x9B,0x03,0xC7}}, 
	{LCM_SEND(2), {0x53,0x2C}},
	{LCM_SEND(2), {0x11,0x00}},
	{LCM_SLEEP(120),},
	{LCM_SEND(2), {0x13, 0x00}}, 
	{LCM_SEND(2), {0x29, 0x00}}, 
	{LCM_SEND(2), {0x51,0xFF}},  //backlight command
};

/*
static LCM_Init_Code disp_on =  {LCM_SEND(1), {0x29}};

static LCM_Init_Code sleep_in =  {LCM_SEND(1), {0x10}};

static LCM_Init_Code sleep_out =  {LCM_SEND(1), {0x11}};
*/

static LCM_Init_Code sleep_in[] =  {
	{LCM_SEND(2), {0x51,0}}, 
	{LCM_SEND(1), {0x28}},
	{LCM_SLEEP(10)},
	{LCM_SEND(1), {0x10}},
	{LCM_SLEEP(120)},
//	{LCM_SEND(2), {0x4f, 0x01}},
};

static LCM_Init_Code sleep_out[] =  {
	{LCM_SEND(1), {0x11}}, // sleep out 
	{LCM_SLEEP(120),},
	{LCM_SEND(1), {0x29}}, // display on 
	{LCM_SLEEP(100),},
	{LCM_SEND(1), {0x2c}}, // normal on
	{LCM_SEND(2), {0x53,0x24}}, 
	{LCM_SEND(2), {0x51,0xFF}}, 
};

static int32_t nt35517_mipi_init(struct panel_spec *self)
{
	int32_t i;
	LCM_Init_Code *init = init_data;
	unsigned int tag;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	printk("nt35517_init\n");

	mipi_set_cmd_mode();

	for(i = 0; i < ARRAY_SIZE(init_data); i++){
		tag = (init->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(init->data, (init->tag & LCM_TAG_MASK));
			printk("0x%x\n", init->data[2]);
		}else if(tag & LCM_TAG_SLEEP){
			udelay((init->tag & LCM_TAG_MASK) * 1000);
		}
		init++;
	}
	return 0;
}

static uint32_t nt35517_readid(struct panel_spec *self)
{
	/*Jessica TODO: need read id*/
	return 0x17;
}

static int32_t nt35517_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	int32_t i;
	LCM_Init_Code *sleep_in_out = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_dcs_write_t mipi_dcs_write = self->info.mipi->ops->mipi_dcs_write;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	printk(KERN_DEBUG "nt35517_enter_sleep, is_sleep = %d\n", is_sleep);

	if(is_sleep){
		sleep_in_out = sleep_in;
		size = ARRAY_SIZE(sleep_in);
	}else{
		sleep_in_out = sleep_out;
		size = ARRAY_SIZE(sleep_out);
	}

	for(i = 0; i <size ; i++){
		tag = (sleep_in_out->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(sleep_in_out->data, (sleep_in_out->tag & LCM_TAG_MASK));
		}else if(tag & LCM_TAG_SLEEP){
			udelay((sleep_in_out->tag & LCM_TAG_MASK) * 1000);
		}
		sleep_in_out++;
	}
	return 0;
}


static struct panel_operations lcd_nt35517_mipi_operations = {
	.panel_init = nt35517_mipi_init,
	.panel_readid = nt35517_readid,
	.panel_enter_sleep = nt35517_enter_sleep,
};

static struct timing_rgb lcd_nt35517_mipi_timing = {
	.hfp = 200,  /* unit: pixel */
	.hbp = 200,
	.hsync = 8,
	.vfp = 10, /*unit: line*/
	.vbp = 10,
	.vsync = 6,
};

static struct info_mipi lcd_nt35517_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width = 24, /*18,16*/
	.lan_number = 2,
	.phy_feq = 500*1000,
	.h_sync_pol = SPRDFB_POLARITY_POS,
	.v_sync_pol = SPRDFB_POLARITY_POS,
	.de_pol = SPRDFB_POLARITY_POS,
	.te_pol = SPRDFB_POLARITY_POS,
	.color_mode_pol = SPRDFB_POLARITY_NEG,
	.shut_down_pol = SPRDFB_POLARITY_NEG,
	.timing = &lcd_nt35517_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_nt35517_mipi_spec = {
	.width = 540,
	.height = 960,
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.info = {
		.mipi = &lcd_nt35517_mipi_info
	},
	.ops = &lcd_nt35517_mipi_operations,
};

struct panel_cfg lcd_nt35517_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x17,
	.lcd_name = "lcd_nt35517_mipi",
	.panel = &lcd_nt35517_mipi_spec,
};

static int __init lcd_nt35517_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_nt35517_mipi);
}

subsys_initcall(lcd_nt35517_mipi_init);
