/******************************************************************************
 ** File Name:    sprdfb_chip_8830.c                                     *
 ** Author:       congfu.zhao                                           *
 ** DATE:         30/04/2013                                        *
 ** Copyright:    2013 Spreatrum, Incoporated. All Rights Reserved. *
 ** Description:                                                    *
 ******************************************************************************/
/******************************************************************************
 **                   Edit    History                               *
 **---------------------------------------------------------------------------*
 ** DATE          NAME            DESCRIPTION                       *

 ******************************************************************************/


#include "sprdfb_chip_8830.h"
#include "sprdfb_chip_common.h"


void dsi_enable(void)
{
	sci_glb_set(REG_AP_AHB_MISC_CKG_EN, BIT_DPHY_REF_CKG_EN);
	sci_glb_set(REG_AP_AHB_MISC_CKG_EN, BIT_DPHY_CFG_CKG_EN);
	sci_glb_set(DSI_REG_EB, DSI_BIT_EB);

	pr_debug("REG_AP_AHB_MISC_CKG_EN:%x \n",dispc_glb_read(REG_AP_AHB_MISC_CKG_EN));
	pr_debug("REG_AP_AHB_MISC_CKG_EN:%x \n",REG_AP_AHB_MISC_CKG_EN);
	pr_debug("DSI_REG_EB:%x \n",dispc_glb_read(DSI_REG_EB));
	pr_debug("DSI_REG_EB:%x \n",DSI_REG_EB);
	pr_debug("BIT_DPHY_REF_CKG_EN:%lx,BIT_DPHY_CFG_CKG_EN:%lx \n",BIT_DPHY_REF_CKG_EN,BIT_DPHY_CFG_CKG_EN);
}

void dsi_disable(void)
{
	sci_glb_clr(REG_AP_AHB_MISC_CKG_EN, BIT_DPHY_REF_CKG_EN);
	sci_glb_clr(REG_AP_AHB_MISC_CKG_EN, BIT_DPHY_CFG_CKG_EN);
	sci_glb_clr(DSI_REG_EB, DSI_BIT_EB);
}

void dispc_print_clk(void)
{
	pr_debug("sprdfb:0x402e0004 = 0x%x\n", dispc_glb_read((SPRD_AONAPB_BASE+0x4)));
	pr_debug("sprdfb:0x20d00000 = 0x%x\n", dispc_glb_read(SPRD_AHB_BASE));
	pr_debug("sprdfb:0x71300000 = 0x%x\n", dispc_glb_read(SPRD_APBREG_BASE));
	pr_debug("sprdfb:0x71200034 = 0x%x\n", dispc_glb_read((SPRD_APBCKG_BASE+0x34)));
	pr_debug("sprdfb:0x71200030 = 0x%x\n", dispc_glb_read((SPRD_APBCKG_BASE+0x30)));
	pr_debug("sprdfb:0x7120002c = 0x%x\n", dispc_glb_read((SPRD_APBCKG_BASE+0x2c)));
}




