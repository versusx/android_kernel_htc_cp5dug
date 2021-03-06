/* drivers/video/sc8825/lcd_nt35516_mipi.c
 *
 * Support for nt35516 mipi LCD device
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
	/* Vivid color & Skin tone start */
	/* Modify length from 16 to 18 for DSI long command */
	{LCM_SEND(19), {17, 0, 0xB4,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{LCM_SEND(19), {17, 0, 0xD6,0x00, 0x05, 0x10, 0x17, 0x22, 0x26, 0x29, 0x29, 0x26, 0x23, 0x17, 0x12, 0x06, 0x02, 0x01, 0x00}},

	{LCM_SEND(15), {13, 0, 0xD7,0x30, 0x30, 0x30, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
	{LCM_SEND(16), {14, 0, 0xD8,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x30, 0x00}},
	/* Vivid color & Skin tone end */
	{LCM_SEND(4), {2, 0, 0xB6,0x07}},
	{LCM_SEND(5), {3, 0, 0xB7,0x33, 0x03}},
	{LCM_SEND(7), {5, 0, 0xB8,0x00, 0x00, 0x02, 0x00}},
	{LCM_SEND(4), {2, 0, 0xBA,0x01}},
	{LCM_SEND(5), {3, 0, 0xBB,0x44, 0x40}},
	{LCM_SEND(4), {2, 0, 0xC1,0x01}},
	{LCM_SEND(11), {9, 0, 0xC2,0x00, 0x00, 0x55, 0x55, 0x55, 0x00, 0x55, 0x55}},
	{LCM_SEND(4), {2, 0, 0xC7,0x00}},
	{LCM_SEND(16), {14, 0, 0xCA,0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00}},
	{LCM_SEND(5), {3, 0,  0xE0,0x00, 0x01}},
	{LCM_SEND(5), {3, 0, 0xE1,0x00, 0xFF}},

	/* CABC start */
	{LCM_SEND(13), {11, 0, 0xE3,0xFF, 0xF7, 0xEF, 0xE7, 0xDF, 0xD7, 0xCF, 0xC7, 0xBF, 0xB7}},
	{LCM_SEND(4), {2, 0, 0x5E,0x06}},
	{LCM_SEND(4), {2, 0, 0x55,0x02}}, /* CABC on (set 0x00 to off) */
	{LCM_SEND(7), {5, 0, 0xFF,0xAA, 0x55, 0xA5, 0x80}},
	{LCM_SEND(11), {9, 0, 0xF5,0x44, 0x44, 0x44, 0x44, 0x44, 0x00, 0xD9, 0x17}},
	{LCM_SEND(7), {5, 0,  0xFF,0xAA, 0x55, 0xA5, 0x00}},
	/* CABC end */
	{LCM_SEND(8), {6, 0, 0xF0,0x55, 0xAA, 0x52, 0x08, 0x01}},
	{LCM_SEND(4), {2, 0, 0xB0,0x0A}},
	{LCM_SEND(4), {2, 0, 0xB1,0x0A}},
	{LCM_SEND(4), {2, 0, 0xB2,0x00}},
	{LCM_SEND(4), {2, 0, 0xB3,0x08}},
	{LCM_SEND(4), {2, 0, 0xB4,0x28}},
	{LCM_SEND(4), {2, 0, 0xB5,0x05}},
	{LCM_SEND(4), {2, 0, 0xB6,0x35}},
	{LCM_SEND(4), {2, 0, 0xB7,0x35}},
	{LCM_SEND(4), {2, 0, 0xB8,0x25}},
	{LCM_SEND(4), {2, 0, 0xB9,0x37}},
	{LCM_SEND(4), {2, 0, 0xBA,0x15}},
	{LCM_SEND(4), {2, 0, 0xCC,0x64}},

	/* Gamma table start */
	/* Modify length from 52 to 54 for DSI long command */
	/* Gamut R P */
	{LCM_SEND(55), {53, 0, 0xD1,0x00, 0xC7, 0x00, 0xCF, 0x00, 0xE2, 0x00, 0xE9, 0x00, 0xF8, 0x01, 0x0F, 0x01, 0x23, 0x01, 0x45, 0x01, 0x62, 0x01, 0x93, 0x01, 0xBB, 0x01, 0xFB, 0x02, 0x2D, 0x02, 0x2E, 0x02, 0x62, 0x02, 0x98,0x02, 0xBA, 0x02, 0xEB, 0x03, 0x0D, 0x03, 0x38, 0x03, 0x53, 0x03, 0x7A, 0x03, 0x97, 0x03, 0xA6, 0x03, 0xCA, 0x03, 0xD0}},
	/* Gamut G P */
	{LCM_SEND(55), {53, 0, 0xD2,0x00, 0x98, 0x00, 0xA1, 0x00, 0xBA, 0x00, 0xC8, 0x00, 0xD7, 0x00, 0xF3, 0x01, 0x0B, 0x01, 0x32,0x01,0x52,0x01, 0x87, 0x01, 0xB2, 0x01, 0xF4, 0x02, 0x29, 0x02, 0x2A, 0x02, 0x5F, 0x02, 0x96,0x02, 0xB8, 0x02, 0xE9, 0x03, 0x0B, 0x03, 0x37,0x03, 0x53, 0x03, 0x7A, 0x03, 0x96, 0x03, 0xAA,0x03, 0xCA, 0x03, 0xD0}},
	/* Gamut B P */
	{LCM_SEND(55), {53, 0,  0xD3,0x00, 0x3F, 0x00, 0x4C, 0x00, 0x71, 0x00, 0x7E, 0x00, 0x94, 0x00, 0xBB, 0x00, 0xD8, 0x01, 0x08,0x01, 0x2D, 0x01, 0x6A, 0x01, 0x9B, 0x01, 0xE6, 0x02, 0x1F, 0x02, 0x20, 0x02, 0x57, 0x02, 0x91,0x02, 0xB4, 0x02, 0xE7, 0x03, 0x09, 0x03, 0x37, 0x03, 0x54, 0x03, 0x7B, 0x03, 0x93, 0x03, 0xB3,0x03, 0xCA, 0x03, 0xD0}},
	/* Gamut R N */
	{LCM_SEND(55), {53, 0, 0xD4,0x00, 0xC7, 0x00, 0xCF, 0x00, 0xE2, 0x00, 0xE9, 0x00, 0xF8, 0x01, 0x0F, 0x01, 0x23, 0x01, 0x45,0x01, 0x62, 0x01, 0x93, 0x01, 0xBB, 0x01, 0xFB, 0x02, 0x2D, 0x02, 0x2E, 0x02, 0x62, 0x02, 0x98,0x02, 0xBA, 0x02, 0xEB, 0x03, 0x0D, 0x03, 0x38, 0x03, 0x53, 0x03, 0x7A, 0x03, 0x97, 0x03, 0xA6,0x03, 0xCA, 0x03, 0xD0}},
	/* Gamut G N */
	{LCM_SEND(55), {53, 0, 0xD5,0x00, 0x98, 0x00, 0xA1, 0x00, 0xBA, 0x00, 0xC8, 0x00, 0xD7, 0x00, 0xF3, 0x01, 0x0B, 0x01, 0x32,0x01, 0x52, 0x01, 0x87, 0x01, 0xB2, 0x01, 0xF4, 0x02, 0x29, 0x02, 0x2A, 0x02, 0x5F, 0x02, 0x96,0x02, 0xB8, 0x02, 0xE9, 0x03, 0x0B, 0x03, 0x37, 0x03, 0x53, 0x03, 0x7A, 0x03, 0x96, 0x03, 0xAA,0x03, 0xCA, 0x03, 0xD0}},
	/* Gamut B N */
	{LCM_SEND(55), {53, 0, 0xD6,0x00, 0x3F, 0x00, 0x4C, 0x00, 0x71, 0x00, 0x7E, 0x00, 0x94, 0x00, 0xBB, 0x00, 0xD8, 0x01, 0x08,0x01, 0x2D, 0x01, 0x6A, 0x01, 0x9B, 0x01, 0xE6, 0x02, 0x1F, 0x02, 0x20, 0x02, 0x57, 0x02, 0x91,0x02, 0xB4, 0x02, 0xE7, 0x03, 0x09, 0x03, 0x37, 0x03, 0x54, 0x03, 0x7B, 0x03, 0x93, 0x03, 0xB3,0x03, 0xCA, 0x03, 0xD0}},

	/* Gamma table end */
	{LCM_SEND(8), {6, 0, 0xF0,0x55, 0xAA, 0x52, 0x00, 0x00}},
	{LCM_SEND(7), {5, 0, 0xFF,0xAA, 0x55, 0xA5, 0x00}},

	{LCM_SEND(1), {0x11}}, // sleep out
	{LCM_SLEEP(120),},
	{LCM_SEND(1), {0x29}}, // display on
	{LCM_SLEEP(100),},
	{LCM_SEND(1), {0x2c}}, // normal on

	{LCM_SEND(2), {0x53,0x24}},
	{LCM_SEND(2), {0x51,0xFF}},
};


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

static int32_t nt35512_mipi_init(struct panel_spec *self)
{
	int32_t i;
	LCM_Init_Code *init = init_data;
	unsigned int tag;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	mipi_set_cmd_mode();

	printk("nt35512_init\n");

	for(i = 0; i < ARRAY_SIZE(init_data); i++){
		tag = (init->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(init->data, (init->tag & LCM_TAG_MASK));
			printk("0x%x\n", init->data[2]);
		} else if(tag & LCM_TAG_SLEEP){
			udelay((init->tag & LCM_TAG_MASK) * 1000);
		}
		init++;
	}

	return 0;
}

static uint32_t nt35512_readid(struct panel_spec *self)
{
	/*Jessica TODO: need read id*/
	return 0x12;
}

static int32_t nt35512_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	int32_t i;
	LCM_Init_Code *sleep_in_out = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_dcs_write_t mipi_dcs_write = self->info.mipi->ops->mipi_dcs_write;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	printk(KERN_DEBUG "nt35512_enter_sleep, is_sleep = %d\n", is_sleep);

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


static struct panel_operations lcd_nt35512_mipi_operations = {
	.panel_init = nt35512_mipi_init,
	.panel_readid = nt35512_readid,
	.panel_enter_sleep = nt35512_enter_sleep,
};

static struct timing_rgb lcd_nt35512_mipi_timing = {
	.hfp = 200,  /* unit: pixel */
	.hbp = 200,
	.hsync = 8,
	.vfp = 10, /*unit: line*/
	.vbp = 10,
	.vsync = 6,
};

static struct info_mipi lcd_nt35512_mipi_info = {
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
	.timing = &lcd_nt35512_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_nt35512_mipi_spec = {
	.width = 480,
	.height = 800,
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.info = {
		.mipi = &lcd_nt35512_mipi_info
	},
	.ops = &lcd_nt35512_mipi_operations,
};

struct panel_cfg lcd_nt35512_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x12,
	.lcd_name = "lcd_nt35512_mipi",
	.panel = &lcd_nt35512_mipi_spec,
};

static int __init lcd_nt35512_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_nt35512_mipi);
}

subsys_initcall(lcd_nt35512_mipi_init);
