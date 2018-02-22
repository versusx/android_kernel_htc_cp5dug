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


static LCM_Init_Code init_data_jdi[] = {
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

	{LCM_SLEEP(50),},
	{LCM_SEND(1), {0x11}}, // sleep out
	{LCM_SEND(1), {0x29}}, // display on
};

static LCM_Init_Code init_data_auo[] = {
	{LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x08,0x01}},
	{LCM_SEND(4), {2, 0, 0xB0,0x0F}},
	{LCM_SEND(4), {2, 0, 0xB1,0x0F}},
	{LCM_SEND(4), {2, 0, 0xB2,0x00}},
	{LCM_SEND(4), {2, 0, 0xB3,0x07}},
	{LCM_SEND(4), {2, 0, 0xB6,0x14}},
	{LCM_SEND(4), {2, 0, 0xB7,0x15}},
	{LCM_SEND(4), {2, 0, 0xB8,0x24}},
	{LCM_SEND(4), {2, 0, 0xB9,0x36}},
	{LCM_SEND(4), {2, 0, 0xBA,0x24}},
	{LCM_SEND(6), {4, 0, 0xBF,0x01,0xC3,0x11}},
	{LCM_SEND(4), {2, 0, 0xC2,0x00}},
	{LCM_SEND(5), {3, 0, 0xC0,0x00,0x00}},
	{LCM_SEND(6), {4, 0, 0xBC,0x00,0x88,0x00}},
	{LCM_SEND(6), {4, 0, 0xBD,0x00,0x88,0x00}},
	{LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{LCM_SEND(4), {2, 0, 0xB6,0x03}},
	{LCM_SEND(5), {3, 0, 0xB7,0x70,0x70}},
	{LCM_SEND(7), {5, 0, 0xB8,0x00,0x02,0x02,0x02}},
	{LCM_SEND(4), {2, 0, 0xBC,0x00}},
	{LCM_SEND(8), {6, 0, 0xB0,0x00,0x0A,0x0E,0x09,0x04}},
	{LCM_SEND(6), {4, 0, 0xB1,0x60,0x00,0x01}},
	{LCM_SEND(4), {2, 0, 0xB4,0x00}},
	{LCM_SEND(7), {5, 0, 0xFF,0xAA,0x55,0xA5,0x00}},
	{LCM_SEND(1), {0x32}},

	{LCM_SLEEP(50),},
	{LCM_SEND(1), {0x11}}, // sleep out
	{LCM_SEND(1), {0x29}}, // display on
};


static LCM_Init_Code sleep_in[] =  {
	{LCM_SEND(1), {0x28}},
	{LCM_SLEEP(55)},
	{LCM_SEND(1), {0x10}},
	{LCM_SLEEP(125)},
};

static LCM_Init_Code sleep_out[] =  {
	{LCM_SEND(1), {0x11}}, // sleep out 
	{LCM_SLEEP(120),},
	{LCM_SEND(1), {0x29}}, // display on 
	{LCM_SLEEP(20),},
};

static LCM_Force_Cmd_Code rd_prep_code_1[]={
	{0x37, {LCM_SEND(2), {0x1, 0}}},
};

static LCM_Init_Code change_fps_40[] =  {
	{LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{LCM_SEND(8), {6, 0, 0xBD,0x02,0x46,0x1C,0x1C,0x00}},
	{LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x00,0x00}},
};

static LCM_Init_Code change_fps_60[] =  {
	{LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{LCM_SEND(8), {6, 0, 0xBD,0x01,0x84,0x1C,0x1C,0x00}},
	{LCM_SEND(8), {6, 0, 0xF0,0x55,0xAA,0x52,0x00,0x00}},
};

static int32_t nt35512_mipi_init(struct panel_spec *self)
{
	int32_t i;
	int32_t array_size = 0;
	LCM_Init_Code *init = NULL;
	unsigned int tag;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	extern enum {
		NOVATEK_NT35512_JDI,
		NOVATEK_NT35512_AUO,
		NOVATEK_NT35517,
		NOVATEK_NT35517_JDI,
	} lcm_vendor;

	printk(KERN_INFO "[DISP] nt35512_mipi_init: ");

	if (lcm_vendor == NOVATEK_NT35512_JDI) {
		init = init_data_jdi;
		array_size = ARRAY_SIZE(init_data_jdi);
		printk(KERN_INFO "[DISP] NOVATEK_NT35512_JDI\n");
	} else if (lcm_vendor == NOVATEK_NT35512_AUO) {
		init = init_data_auo;
		array_size = ARRAY_SIZE(init_data_auo);
		printk(KERN_INFO "[DISP] NOVATEK_NT35512_AUO\n");
	}

	mipi_set_cmd_mode();

	for(i = 0; i < array_size; i++) {
		tag = (init->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(init->data, (init->tag & LCM_TAG_MASK));
			udelay(20);
			//printk("0x%x\n", init->data[2]);
		} else if(tag & LCM_TAG_SLEEP){
			hr_msleep((init->tag & LCM_TAG_MASK));
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

	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	printk(KERN_INFO "[DISP] nt35512_enter_sleep, is_sleep = %d\n", is_sleep);

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
			udelay(20);
		}else if(tag & LCM_TAG_SLEEP){
			hr_msleep((sleep_in_out->tag & LCM_TAG_MASK));
		}
		sleep_in_out++;
	}

	printk(KERN_INFO "[DISP] nt35512_enter_sleep function end");

	return 0;
}

static uint32_t nt35512_readpowermode(struct panel_spec *self)
{
	int32_t i = 0;
	uint32_t j =0;
	LCM_Force_Cmd_Code * rd_prepare = rd_prep_code_1;
	uint8_t read_data[1] = {0};
	int32_t read_rtn = 0;
	unsigned int tag = 0;

	mipi_force_write_t mipi_force_write = self->info.mipi->ops->mipi_force_write;
	mipi_force_read_t mipi_force_read = self->info.mipi->ops->mipi_force_read;
	mipi_eotp_set_t mipi_eotp_set = self->info.mipi->ops->mipi_eotp_set;

	pr_debug("[DISP] lcd_nt35512_mipi read power mode!\n");

	for(j = 0; j < 4; j++){
		rd_prepare = rd_prep_code_1;
		for(i = 0; i < ARRAY_SIZE(rd_prep_code_1); i++){
			tag = (rd_prepare->real_cmd_code.tag >> 24);
			if(tag & LCM_TAG_SEND){
				mipi_force_write(rd_prepare->datatype, rd_prepare->real_cmd_code.data, (rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
				udelay(20);
			}else if(tag & LCM_TAG_SLEEP){
				hr_msleep((rd_prepare->real_cmd_code.tag & LCM_TAG_MASK));
			}
			rd_prepare++;
		}
		mipi_eotp_set(0,0);
		read_rtn = mipi_force_read(0x0A, 1,(uint8_t *)read_data);
		udelay(20);
		mipi_eotp_set(1,1);
		pr_debug("[DISP] lcd_nt35512 mipi read power mode 0x0A value is 0x%x! , read result(%d)\n", read_data[0], read_rtn);
		if((0x9c == read_data[0])  && (0 == read_rtn)){
			pr_debug("[DISP] lcd_nt35512_mipi read power mode success!\n");
			return 0x9c;
		}
	}

	return 0x0;
}

static int32_t nt35512_check_esd(struct panel_spec *self)
{
	uint32_t power_mode;

	mipi_set_lp_mode_t mipi_set_data_lp_mode = self->info.mipi->ops->mipi_set_data_lp_mode;
	mipi_set_hs_mode_t mipi_set_data_hs_mode = self->info.mipi->ops->mipi_set_data_hs_mode;
	mipi_set_lp_mode_t mipi_set_lp_mode = self->info.mipi->ops->mipi_set_lp_mode;
	mipi_set_hs_mode_t mipi_set_hs_mode = self->info.mipi->ops->mipi_set_hs_mode;
	uint16_t work_mode = self->info.mipi->work_mode;

	pr_debug("[DISP] nt35512_check_esd!\n");
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_lp_mode();
	}else{
		mipi_set_data_lp_mode();
	}
	power_mode = nt35512_readpowermode(self);
//	power_mode = 0x0;
	if(SPRDFB_MIPI_MODE_CMD==work_mode){
		mipi_set_hs_mode();
	}else{
		mipi_set_data_hs_mode();
	}
	if(power_mode == 0x9c){
		pr_debug("[DISP] nt35512_check_esd OK!\n");
		return 1;
	}else{
		printk("[DISP] nt35512_check_esd fail!(0x%x)\n", power_mode);
		return 0;
	}
}

static int32_t nt35512_change_fps(struct panel_spec *self, int fps_level)
{
	int32_t i;
	LCM_Init_Code *change_fps = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	printk(KERN_DEBUG "nt35512_change_fps, fps_level = %d\n", fps_level);

	if(fps_level < 50){
		change_fps = change_fps_40;
		size = ARRAY_SIZE(change_fps_40);
	}else{
	    change_fps = change_fps_60;
		size = ARRAY_SIZE(change_fps_60);
    }

	for(i = 0; i <size ; i++){
		tag = (change_fps->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(change_fps->data, (change_fps->tag & LCM_TAG_MASK));
			udelay(20);
		}else if(tag & LCM_TAG_SLEEP){
			hr_msleep((change_fps->tag & LCM_TAG_MASK));
		}
		change_fps++;
	}
	return 0;

}

#ifdef CONFIG_FB_CABC_LEVEL_CONTROL
static LCM_Init_Code cabc_still_cmds[] =  {
	{LCM_SEND(8),  {6,  0, 0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{LCM_SEND(13), {11, 0, 0xE3,0xFF,0xFB,0xF3,0xEC,0xE2,0xCA,0xC3,0xBC,0xB5,0xB3}},
};

static LCM_Init_Code cabc_movie_cmds[] =  {
	{LCM_SEND(8),  {6,  0, 0xF0,0x55,0xAA,0x52,0x08,0x00}},
	{LCM_SEND(13), {11, 0, 0xE3,0xFF,0xF6,0xF0,0xEA,0xD8,0xC5,0xB4,0x9D,0x8E,0x76}},
};

static void nt35512_set_cabc_lvl(struct panel_spec *self, int mode)
{
	int32_t i;
	LCM_Init_Code *cabc_cmds = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	printk("[DISP] nt35512_set_cabc_lvl, mode = %d\n", mode);

	if(mode == 2) {
		cabc_cmds = cabc_still_cmds;
		size = ARRAY_SIZE(cabc_still_cmds);
	}else if(mode == 3) {
		cabc_cmds = cabc_movie_cmds;
		size = ARRAY_SIZE(cabc_movie_cmds);
	}else {
		printk("[DISP] nt35512_set_cabc_lvl, error mode!!!\n");
		return;
	}

	for(i = 0; i <size ; i++){
		tag = (cabc_cmds->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(cabc_cmds->data, (cabc_cmds->tag & LCM_TAG_MASK));
			udelay(20);
			//printk("0x%x\n", cabc_cmds->data[2]);
		}else if(tag & LCM_TAG_SLEEP){
			hr_msleep((cabc_cmds->tag & LCM_TAG_MASK));
		}
		cabc_cmds++;
	}
}
#endif

static struct panel_operations lcd_nt35512_mipi_operations = {
	.panel_init = nt35512_mipi_init,
	.panel_readid = nt35512_readid,
	.panel_enter_sleep = nt35512_enter_sleep,
	.panel_change_fps = nt35512_change_fps,
	.panel_esd_check = nt35512_check_esd,
#ifdef CONFIG_FB_CABC_LEVEL_CONTROL
	.set_cabc = nt35512_set_cabc_lvl,
#endif
};

static struct timing_rgb lcd_nt35512_mipi_timing = {
#if 0
	.hfp = 200,  /* unit: pixel */
	.hbp = 200,
	.hsync = 8,
	.vfp = 10, /*unit: line*/
	.vbp = 10,
	.vsync = 6,
#else
	.hfp = 270,  /* unit: pixel */
	.hbp = 270,
	.hsync = 4,
	.vfp = 16, /*unit: line*/
	.vbp = 16,
	.vsync = 3,
#endif
};

static struct info_mipi lcd_nt35512_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width = 24, /*18,16*/
	.lan_number = 2,
	.phy_feq = 404*1000,
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
	.fps = 60,
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
//	.is_clean_lcd = true,
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
