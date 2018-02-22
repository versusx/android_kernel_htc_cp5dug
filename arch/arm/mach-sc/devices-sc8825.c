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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/ion.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mmc/sdhci.h>
#include <linux/gpio.h>
#include <linux/persistent_ram.h>

#include <linux/sprd_cproc.h>
#include <linux/sipc.h>
#include <linux/spipe.h>
#include <linux/spool.h>
#include <linux/seth.h>

#include <sound/saudio.h>

#include <asm/pmu.h>

#include <mach/hardware.h>
#include <mach/sci_glb_regs.h>
#include <mach/irqs.h>
#include <mach/board.h>
#include <mach/kpd.h>
#include "devices.h"

//HTC_WIFI_START
#include <linux/delay.h>
#include <linux/mmc/card.h>  
#include <asm/mach/mmc.h>
#include "board-sprd-wifi.h"
#include <mach/pinmap.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

static struct resource sprd_serial_resources0[] = {
	[0] = {
		.start = SPRD_UART0_BASE,
		.end = SPRD_UART0_BASE + SPRD_UART0_SIZE-1,
		.name = "serial_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SER0_INT,
		.end = IRQ_SER0_INT,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device sprd_serial_device0 = {
	.name           = "serial_sprd",
	.id             =  0,
	.num_resources  = ARRAY_SIZE(sprd_serial_resources0),
	.resource       = sprd_serial_resources0,
};

static struct resource sprd_serial_resources1[] = {
	[0] = {
		.start = SPRD_UART1_BASE,
		.end = SPRD_UART1_BASE + SPRD_UART1_SIZE-1,
		.name = "serial_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SER1_INT,
		.end = IRQ_SER1_INT,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device sprd_serial_device1 = {
	.name           = "serial_sprd",
	.id             =  1,
	.num_resources  = ARRAY_SIZE(sprd_serial_resources1),
	.resource       = sprd_serial_resources1,
};

static struct resource sprd_serial_resources2[] = {
	[0] = {
		.start = SPRD_UART2_BASE,
		.end = SPRD_UART2_BASE + SPRD_UART2_SIZE - 1,
		.name = "serial_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SER2_INT,
		.end = IRQ_SER2_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_serial_device2 = {
	.name           = "serial_sprd",
	.id             =  2,
	.num_resources  = ARRAY_SIZE(sprd_serial_resources2),
	.resource       = sprd_serial_resources2,
};


static struct resource sprd_serial_resources3[] = {
	[0] = {
		.start = SPRD_UART3_BASE,
		.end = SPRD_UART3_BASE + SPRD_UART3_SIZE - 1,
		.name = "serial_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SER3_INT,
		.end = IRQ_SER3_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_serial_device3 = {
	.name           = "serial_sprd",
	.id             =  3,
	.num_resources  = ARRAY_SIZE(sprd_serial_resources3),
	.resource       = sprd_serial_resources3,

};

static struct resource resources_rtc[] = {
	[0] = {
		.start	= IRQ_ANA_RTC_INT,
		.end	= IRQ_ANA_RTC_INT,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device sprd_device_rtc= {
	.name	= "sprd_rtc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_rtc),
	.resource	= resources_rtc,
};

static struct eic_gpio_resource sprd_gpio_resource[] = {
       [ENUM_ID_D_GPIO] = {
               .irq            = IRQ_GPIO_INT,
               .base_addr      = CTL_GPIO_BASE,
               .chip_base      = D_GPIO_START,
               .chip_ngpio     = D_GPIO_NR,
       },
       [ENUM_ID_D_EIC] = {
               .irq            = IRQ_EIC_INT,
               .base_addr      = CTL_EIC_BASE,
               .chip_base      = D_EIC_START,
               .chip_ngpio     = D_EIC_NR,
       },
       [ENUM_ID_A_GPIO] = {
               .irq            = IRQ_ANA_GPIO_INT,
               .base_addr      = ANA_CTL_GPIO_BASE,
               .chip_base      = A_GPIO_START,
               .chip_ngpio     = A_GPIO_NR,
       },
       [ENUM_ID_A_EIC] = {
               .irq            = IRQ_ANA_EIC_INT,
               .base_addr      = ANA_CTL_EIC_BASE,
               .chip_base      = A_EIC_START,
               .chip_ngpio     = A_EIC_NR,
       },
};

struct platform_device sprd_eic_gpio_device = {
       .name = "eic-gpio",
       .id = -1,
       .dev = { .platform_data = sprd_gpio_resource },
};

static struct resource sprd_nand_resources[] = {
	[0] = {
		.start	= 7,
		.end = 7,
		.flags	= IORESOURCE_DMA,
	},
	[1] = {
		.start	= SPRD_NFC_BASE,
		.end = SPRD_NFC_BASE + SPRD_NFC_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device sprd_nand_device = {
	.name		= "sprd-nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sprd_nand_resources),
	.resource	= sprd_nand_resources,
};

static struct resource sprd_hwspinlock_resources[] = {
	[0] = {
		.start	= SPRD_HWLOCK0_BASE,
		.end = SPRD_HWLOCK0_BASE + SPRD_HWLOCK0_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device sprd_hwspinlock_device0 = {
	.name		= "sci_hwspinlock",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sprd_hwspinlock_resources),
	.resource	= sprd_hwspinlock_resources,
};

static struct resource sprd_lcd_resources[] = {
	[0] = {
		.start = SPRD_LCDC_BASE,
		.end = SPRD_LCDC_BASE + SPRD_LCDC_SIZE - 1,
		.name = "lcd_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LCDC_INT,
		.end = IRQ_LCDC_INT,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device sprd_lcd_device0 = {
	.name           = "sprd_fb",
	.id             =  0,
	.num_resources  = ARRAY_SIZE(sprd_lcd_resources),
	.resource       = sprd_lcd_resources,
};

struct platform_device sprd_lcd_device1 = {
	.name           = "sprd_fb",
	.id             =  1,
	.num_resources  = ARRAY_SIZE(sprd_lcd_resources),
	.resource       = sprd_lcd_resources,
};

struct persistent_ram_descriptor sprd_console_desc = {
	.name = "ram_console.0",
	.size = SPRD_RAM_CONSOLE_SIZE,
};

struct persistent_ram sprd_console_ram = {
	.start = SPRD_RAM_CONSOLE_START,
	.size = SPRD_RAM_CONSOLE_SIZE,
	.num_descs = 1,
	.descs = &sprd_console_desc,
};

struct platform_device sprd_ram_console = {
	.name           = "ram_console",
	.id             =  0,
};

static struct resource sprd_otg_resource[] = {
	[0] = {
		.start = SPRD_USB_BASE,
		.end   = SPRD_USB_BASE + SPRD_USB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USBD_INT,
		.end   = IRQ_USBD_INT,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device sprd_otg_device = {
	.name		= "dwc_otg",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sprd_otg_resource),
	.resource	= sprd_otg_resource,
};

#if 0
struct platform_device sprd_backlight_device = {
	.name           = "sprd_backlight",
	.id             =  -1,
};
#endif

struct platform_device led_pwm_device = {
	.name		= "lcd-backlight",
};


static struct resource sprd_i2c_resources0[] = {
	[0] = {
		.start = SPRD_I2C0_BASE,
		.end   = SPRD_I2C0_BASE + SPRD_I2C0_SIZE -1,
		.name  = "i2c0_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C0_INT,
		.end   = IRQ_I2C0_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_i2c_device0 = {
	.name           = "sprd-i2c",
	.id             =  0,
	.num_resources  = ARRAY_SIZE(sprd_i2c_resources0),
	.resource       = sprd_i2c_resources0,
};


static struct resource sprd_i2c_resources1[] = {
	[0] = {
		.start = SPRD_I2C1_BASE,
		.end   = SPRD_I2C1_BASE + SZ_4K -1,
		.name  = "i2c1_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C1_INT,
		.end   = IRQ_I2C1_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_i2c_device1 = {
	.name           = "sprd-i2c",
	.id             =  1,
	.num_resources  = ARRAY_SIZE(sprd_i2c_resources1),
	.resource       = sprd_i2c_resources1,
};


static struct resource sprd_i2c_resources2[] = {
	[0] = {
		.start = SPRD_I2C2_BASE,
		.end   = SPRD_I2C2_BASE + SZ_4K -1,
		.name  = "i2c2_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C2_INT,
		.end   = IRQ_I2C2_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_i2c_device2 = {
	.name           = "sprd-i2c",
	.id             =  2,
	.num_resources  = ARRAY_SIZE(sprd_i2c_resources2),
	.resource       = sprd_i2c_resources2,
};


static struct resource sprd_i2c_resources3[] = {
	[0] = {
		.start = SPRD_I2C3_BASE,
		.end   = SPRD_I2C3_BASE + SZ_4K -1,
		.name  = "i2c3_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C3_INT,
		.end   = IRQ_I2C3_INT,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device sprd_i2c_device3 = {
	.name           = "sprd-i2c",
	.id             =  3,
	.num_resources  = ARRAY_SIZE(sprd_i2c_resources3),
	.resource       = sprd_i2c_resources3,
};

/* 8810 SPI devices.  */
static struct resource spi0_resources[] = {
    [0] = {
        .start = SPRD_SPI0_BASE,
        .end = SPRD_SPI0_BASE + SPRD_SPI0_SIZE - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_SPI0_INT,
        .end = IRQ_SPI0_INT,
        .flags = IORESOURCE_IRQ,
    },
};


static struct resource spi1_resources[] = {
    [0] = {
        .start = SPRD_SPI1_BASE,
        .end = SPRD_SPI1_BASE + SPRD_SPI1_SIZE - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = IRQ_SPI1_INT,
        .end = IRQ_SPI1_INT,
        .flags = IORESOURCE_IRQ,
    },
};

static struct resource spi2_resources[] = {
	[0] = {
	       .start = SPRD_SPI2_BASE,
	       .end = SPRD_SPI2_BASE + SPRD_SPI2_SIZE - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_SPI2_INT,
	       .end = IRQ_SPI2_INT,
	       .flags = IORESOURCE_IRQ,
	       },
};

struct platform_device sprd_spi0_device = {
    .name = "sprd_spi",
    .id = 0,
    .resource = spi0_resources,
    .num_resources = ARRAY_SIZE(spi0_resources),
};

struct platform_device sprd_spi1_device = {
	.name = "sprd_spi",
	.id = 1,
	.resource = spi1_resources,
	.num_resources = ARRAY_SIZE(spi1_resources),
};

struct platform_device sprd_spi2_device = {
	.name = "sprd_spi",
	.id = 2,
	.resource = spi2_resources,
	.num_resources = ARRAY_SIZE(spi2_resources),
};

static struct resource sprd_ahb_bm0_res[] = {
	[0] = {
		.start = SPRD_BM0_BASE,
		.end = SPRD_BM0_BASE + SPRD_BM0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_BM_INT,
		.end = IRQ_BM_INT,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource sprd_ahb_bm1_res[] = {
	[0] = {
		.start = SPRD_BM1_BASE,
		.end = SPRD_BM1_BASE + SPRD_BM1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_BM_INT,
		.end = IRQ_BM_INT,
		.flags = IORESOURCE_IRQ,
	},

};

static struct resource sprd_ahb_bm2_res[] = {
	[0] = {
		.start = SPRD_BM2_BASE,
		.end = SPRD_BM2_BASE + SPRD_BM2_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_BM_INT,
		.end = IRQ_BM_INT,
		.flags = IORESOURCE_IRQ,
	},

};

static struct resource sprd_ahb_bm3_res[] = {
	[0] = {
		.start = SPRD_BM3_BASE,
		.end = SPRD_BM3_BASE + SPRD_BM3_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_BM_INT,
		.end = IRQ_BM_INT,
		.flags = IORESOURCE_IRQ,
	},

};

static struct resource sprd_ahb_bm4_res[] = {
	[0] = {
		.start = SPRD_BM4_BASE,
		.end = SPRD_BM4_BASE + SPRD_BM4_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_BM_INT,
		.end = IRQ_BM_INT,
		.flags = IORESOURCE_IRQ,
	},

};

static struct resource sprd_axi_bm0_res[] = {
	[0] = {
		.start =SPRD_AXIBM0_BASE,
		.end = SPRD_AXIBM0_BASE + SPRD_AXIBM0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AXIBM_INT,
		.end = IRQ_AXIBM_INT,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource sprd_axi_bm1_res[] = {
	[0] = {
		.start =SPRD_AXIBM1_BASE,
		.end = SPRD_AXIBM1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AXIBM_INT,
		.end = IRQ_AXIBM_INT,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource sprd_axi_bm2_res[] = {
	[0] = {
		.start =SPRD_AXIBM2_BASE,
		.end = SPRD_AXIBM2_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_AXIBM_INT,
		.end = IRQ_AXIBM_INT,
		.flags = IORESOURCE_IRQ,
	},
};


struct platform_device sprd_ahb_bm0_device = {
	.name = "sprd_ahb_busmonitor",
	.id = 0,
	.resource = sprd_ahb_bm0_res,
	.num_resources = ARRAY_SIZE(sprd_ahb_bm0_res),
};

struct platform_device sprd_ahb_bm1_device = {
	.name = "sprd_ahb_busmonitor",
	.id = 1,
	.resource = sprd_ahb_bm1_res,
	.num_resources = ARRAY_SIZE(sprd_ahb_bm1_res),
};

struct platform_device sprd_ahb_bm2_device = {
	.name = "sprd_ahb_busmonitor",
	.id = 2,
	.resource = sprd_ahb_bm2_res,
	.num_resources = ARRAY_SIZE(sprd_ahb_bm2_res),
};

struct platform_device sprd_ahb_bm3_device = {
	.name = "sprd_ahb_busmonitor",
	.id = 3,
	.resource = sprd_ahb_bm3_res,
	.num_resources = ARRAY_SIZE(sprd_ahb_bm3_res),
};

struct platform_device sprd_ahb_bm4_device = {
	.name = "sprd_ahb_busmonitor",
	.id = 4,
	.resource = sprd_ahb_bm4_res,
	.num_resources = ARRAY_SIZE(sprd_ahb_bm4_res),

};

struct platform_device sprd_axi_bm0_device = {
	.name = "sprd_axi_busmonitor",
	.id = 0,
	.resource = sprd_axi_bm0_res,
	.num_resources = ARRAY_SIZE(sprd_axi_bm0_res),
};

struct platform_device sprd_axi_bm1_device = {
	.name = "sprd_axi_busmonitor",
	.id = 1,
	.resource = sprd_axi_bm1_res,
	.num_resources = ARRAY_SIZE(sprd_axi_bm1_res),
};

struct platform_device sprd_axi_bm2_device = {
	.name = "sprd_axi_busmonitor",
	.id = 2,
	.resource = sprd_axi_bm2_res,
	.num_resources = ARRAY_SIZE(sprd_axi_bm2_res),
};

//keypad 
#if defined(CONFIG_MACH_SP8825EB) || defined(CONFIG_MACH_SP8825EA) || defined(CONFIG_MACH_SP8825EA_TIGER_EVM) || defined(CONFIG_MACH_SP6825GA) || defined(CONFIG_MACH_SP6825GB)
#define CUSTOM_KEYPAD_ROWS          (SCI_ROW2)
#define CUSTOM_KEYPAD_COLS          (SCI_COL2)
#define ROWS	(2)
#define COLS	(2)

static const unsigned int test_keymap[] = {
	KEY(1, 0, KEY_VOLUMEUP),
	KEY(0, 0, KEY_VOLUMEDOWN),
	KEY(0, 1, KEY_CAMERA),
};
#elif defined(CONFIG_MACH_GARDA)
#define CUSTOM_KEYPAD_ROWS          (SCI_ROW3)
#define CUSTOM_KEYPAD_COLS          (SCI_COL2)
#define ROWS	(3)
#define COLS	(2)

static const unsigned int test_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),
	KEY(1, 2, KEY_HOMEPAGE),
};
#else
#define CUSTOM_KEYPAD_ROWS          (SCI_ROW7 | SCI_ROW6 | SCI_ROW5 | SCI_ROW4 | SCI_ROW3 |SCI_ROW2)
#define CUSTOM_KEYPAD_COLS          (SCI_COL7 | SCI_COL6 | SCI_COL5 | SCI_COL4 | SCI_COL3 |SCI_COL2)
#define ROWS	(8)
#define COLS	(8)

static const unsigned int test_keymap[] = {
	KEY(0, 0, KEY_F1),

	KEY(0, 3, KEY_COFFEE),
	KEY(0, 2, KEY_QUESTION),
	KEY(2, 3, KEY_CONNECT),
	KEY(1, 2, KEY_SHOP),
	KEY(1, 1, KEY_PHONE),

	KEY(0, 1, KEY_DELETE),
	KEY(2, 2, KEY_PLAY),
	KEY(1, 0, KEY_PAGEUP),
	KEY(1, 3, KEY_PAGEDOWN),
	KEY(2, 0, KEY_EMAIL),
	KEY(2, 1, KEY_STOP),

	KEY(0, 7, KEY_KP1),
	KEY(0, 6, KEY_KP2),
	KEY(0, 5, KEY_KP3),
	KEY(1, 7, KEY_KP4),
	KEY(1, 6, KEY_KP5),
	KEY(1, 5, KEY_KP6),
	KEY(2, 7, KEY_KP7),
	KEY(2, 6, KEY_KP8),
	KEY(2, 5, KEY_KP9),
	KEY(3, 6, KEY_KP0),
	KEY(3, 7, KEY_KPASTERISK),
	KEY(3, 5, KEY_KPDOT),
	KEY(7, 2, KEY_NUMLOCK),
	KEY(7, 1, KEY_KPMINUS),
	KEY(6, 1, KEY_KPPLUS),
	KEY(7, 6, KEY_KPSLASH),
	KEY(6, 0, KEY_ENTER),

	KEY(7, 4, KEY_CAMERA),

	KEY(0, 4, KEY_F2),
	KEY(1, 4, KEY_F3),
	KEY(2, 4, KEY_F4),
	KEY(7, 7, KEY_F5),
	KEY(7, 5, KEY_F6),

	KEY(3, 4, KEY_Q),
	KEY(3, 3, KEY_W),
	KEY(3, 2, KEY_E),
	KEY(3, 1, KEY_R),
	KEY(3, 0, KEY_T),
	KEY(4, 7, KEY_Y),
	KEY(4, 6, KEY_U),
	KEY(4, 5, KEY_I),
	KEY(4, 4, KEY_O),
	KEY(4, 3, KEY_P),
	KEY(4, 2, KEY_A),
	KEY(4, 1, KEY_S),
	KEY(4, 0, KEY_D),
	KEY(5, 7, KEY_F),
	KEY(5, 6, KEY_G),
	KEY(5, 5, KEY_H),
	KEY(5, 4, KEY_J),
	KEY(5, 3, KEY_K),
	KEY(5, 2, KEY_L),
	KEY(5, 1, KEY_Z),
	KEY(5, 0, KEY_X),
	KEY(6, 7, KEY_C),
	KEY(6, 6, KEY_V),
	KEY(6, 5, KEY_B),
	KEY(6, 4, KEY_N),
	KEY(6, 3, KEY_M),
	KEY(6, 2, KEY_SPACE),
	KEY(7, 0, KEY_LEFTSHIFT),
	KEY(7, 3, KEY_LEFTCTRL),
};
#endif

static const struct matrix_keymap_data test_keymap_data = {
	.keymap = test_keymap,
	.keymap_size = ARRAY_SIZE(test_keymap),
};

struct sci_keypad_platform_data sci_keypad_data = {
	.rows_choose_hw = CUSTOM_KEYPAD_ROWS,
	.cols_choose_hw = CUSTOM_KEYPAD_COLS,
	.rows = ROWS,
	.cols = COLS,
	.keymap_data = &test_keymap_data,
	.support_long_key = 1,
	.repeat = 0,
	.debounce_time = 5000,
	.keyup_test_interval = 50,
	.controller_ver = 0,
};

static struct resource sci_keypad_resources[] = {
	{
	        .start = IRQ_KPD_INT,
	        .end = IRQ_KPD_INT,
	        .flags = IORESOURCE_IRQ,
	},
};

struct platform_device sprd_keypad_device = {
	.name = "sci-keypad",
        .id             = -1,
	.dev = {
		.platform_data = &sci_keypad_data,
		},
	.num_resources = ARRAY_SIZE(sci_keypad_resources),
	.resource = sci_keypad_resources,
};

struct platform_device sprd_audio_platform_vbc_pcm_device = {
	.name           = "sprd-vbc-pcm-audio",
	.id             =  -1,
};

struct platform_device sprd_audio_cpu_dai_vaudio_device = {
	.name           = "vaudio",
	.id             =  -1,
};

struct platform_device sprd_audio_cpu_dai_vbc_device = {
	.name           = "vbc",
	.id             =  -1,
};

struct platform_device sprd_audio_codec_sprd_codec_device = {
	.name           = "sprd-codec",
	.id             =  -1,
};

static struct resource sprd_battery_resources[] = {
        [0] = {
                .start = EIC_CHARGER_DETECT,
                .end = EIC_CHARGER_DETECT,
                .flags = IORESOURCE_IO,
        }
};

struct platform_device sprd_battery_device = {
        .name           = "sprd-battery",
        .id             =  0,
        .num_resources  = ARRAY_SIZE(sprd_battery_resources),
        .resource       = sprd_battery_resources,
};

struct platform_device sprd_vsp_device = {
	.name	= "sprd_vsp",
	.id	= -1,
};

#ifdef CONFIG_ION
static struct ion_platform_data ion_pdata = {
	.nr = 2,
	.heaps = {
		{
			.id	= ION_HEAP_TYPE_CARVEOUT,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= "ion_carveout_heap",
			.base   = SPRD_ION_BASE,
			.size   = SPRD_ION_SIZE,
		},
		{
			.id	= ION_HEAP_TYPE_CARVEOUT + 1,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= "ion_carveout_heap_overlay",
			.base   = SPRD_ION_OVERLAY_BASE,
			.size   = SPRD_ION_OVERLAY_SIZE,
		}
	}
};

struct platform_device sprd_ion_dev = {
	.name = "ion-sprd",
	.id = -1,
	.dev = { .platform_data = &ion_pdata },
};
#endif

static struct resource sprd_dcam_resources[] = {
	{
		.start	= SPRD_ISP_BASE,
		.end	= SPRD_ISP_BASE + SPRD_ISP_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_ISP_INT,
		.end	= IRQ_ISP_INT,
		.flags	= IORESOURCE_IRQ,
	},
};
struct platform_device sprd_dcam_device = {
	.name		= "sprd_dcam",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(sprd_dcam_resources),
	.resource	= sprd_dcam_resources,
};
struct platform_device sprd_scale_device = {
	.name	= "sprd_scale",
	.id	= -1,
};

struct platform_device sprd_rotation_device = {
	.name	= "sprd_rotation",
	.id	= -1,
};

struct platform_device sprd_sensor_device = {
	.name	= "sprd_sensor",
	.id	= -1,
};
struct platform_device sprd_isp_device = {
	.name = "sprd_isp",
	.id = -1,
};

static struct resource sprd_sdio0_resources[] = {
	[0] = {
		.start = SPRD_SDIO0_BASE,
		.end = SPRD_SDIO0_BASE + SPRD_SDIO0_SIZE-1,
		.name = "sdio0_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDIO0_INT,
		.end = IRQ_SDIO0_INT,
		.flags = IORESOURCE_IRQ,
	}
};

static struct sprd_host_platdata sprd_sdio0_pdata = {
	.hw_name = "sprd-sdcard",
	.detect_gpio = 31,
	.vdd_name = "vddsd0",
	.clk_name = "clk_sdio0",
	.clk_parent = "clk_sdio_src",
	.max_clock = 50000000,
	.enb_bit = BIT_SDIO0_EB,
	.rst_bit = BIT_SD0_SOFT_RST,
};

struct platform_device sprd_sdio0_device = {
	.name           = "sprd-sdhci",
	.id             =  0,
	.num_resources  = ARRAY_SIZE(sprd_sdio0_resources),
	.resource       = sprd_sdio0_resources,
	.dev = { .platform_data = &sprd_sdio0_pdata },
};

static struct resource sprd_sdio1_resources[] = {
	[0] = {
		.start = SPRD_SDIO1_BASE,
		.end = SPRD_SDIO1_BASE + SPRD_SDIO1_SIZE-1,
		.name = "sdio1_res",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDIO1_INT,
		.end = IRQ_SDIO1_INT,
		.flags = IORESOURCE_IRQ,
	}
};

static struct sprd_host_platdata sprd_sdio1_pdata = {
	.hw_name = "sprd-sdio1",
	.clk_name = "clk_sdio1",
	.clk_parent = "clk_sdio_src",
	.vdd_name = "vddsd1",
	.max_clock = 2000000,
	.enb_bit = BIT_SDIO1_EB,
	.rst_bit = BIT_SD1_SOFT_RST,
	.regs.is_valid = 1,
};

struct platform_device sprd_sdio1_device = {
	.name           = "sprd-sdhci",
	.id             =  1,
	.num_resources  = ARRAY_SIZE(sprd_sdio1_resources),
	.resource       = sprd_sdio1_resources,
	.dev = { .platform_data = &sprd_sdio1_pdata },
};

static struct resource sprd_sdio2_resources[] = {
	[0] = {
	       .start = SPRD_SDIO2_BASE,
	       .end = SPRD_SDIO2_BASE + SPRD_SDIO2_SIZE - 1,
	       .name = "sdio2_res",
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_SDIO2_INT,
	       .end = IRQ_SDIO2_INT,
	       .flags = IORESOURCE_IRQ,
	       }
};

/*
 * SDIO2 (SDIO WLAN)
 */
//HTC_WIFI_START

/* BCM4329 returns wrong sdio_vsn(1) when we read cccr,
 * we use predefined value (sdio_vsn=2) here to initial sdio driver well
 */
static struct embedded_sdio_data sprd_wifi_emb_data = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	}
};

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int
sprd_wifi_status_register(void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;

	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}
static int sprd_wifi_cd;      /* WiFi virtual 'card detect' status */

static unsigned int sprd_wifi_status(struct device *dev)
{
	return sprd_wifi_cd;
}

int sprd_wifi_set_carddetect(int val)
{
	printk(KERN_INFO "%s: %d\n", __func__, val);
	sprd_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(sprd_wifi_set_carddetect);


//pull up 4.7k
static int clk_pin_init(void)
{

	
	uint32_t clk_pin_val = __raw_readl(CTL_PIN_BASE + REG_PIN_CTRL3);
	printk(KERN_INFO "[WIFI]---0x%x\n", clk_pin_val);
	clk_pin_val |= 0x2F;//0010 1111
	__raw_writel(clk_pin_val, CTL_PIN_BASE + REG_PIN_CTRL3);

	return 0;
}

static void wlan_clk_init(void)
{
	struct clk *wlan_clk;
	struct clk *clk_parent;
/*8825ea/openphone use aux0 garda use aux1*/
#ifdef CONFIG_MACH_GARDA
	wlan_clk = clk_get(NULL, "clk_aux1");
	if (IS_ERR(wlan_clk)) {
		printk("clock: failed to get clk_aux1\n");
	}
#else
	wlan_clk = clk_get(NULL, "clk_aux0");
	if (IS_ERR(wlan_clk)) {
		printk("clock: failed to get clk_aux0\n");
	}
#endif
	clk_parent = clk_get(NULL, "ext_32k");
	if (IS_ERR(clk_parent)) {
		printk("failed to get parent ext_32k\n");
	}

	clk_set_parent(wlan_clk, clk_parent);
	clk_set_rate(wlan_clk, 32768);
	clk_enable(wlan_clk);
}

static void power_on_cmmb1v8(void)
{
      //u32 value;
	  int enable;
      struct regulator* cmmb_regulator_1v8;
	  cmmb_regulator_1v8 = regulator_get(NULL, "vddcmmb1p8");

	  enable = regulator_is_enabled(cmmb_regulator_1v8);

      printk("[WIFI] power_on_cmmb1v8 is %d\n",enable);
	  
      regulator_set_voltage(cmmb_regulator_1v8, 1800000, 1800000);
      //regulator_disable(cmmb_regulator_1v8);
      //msleep(3);
      // value = cmmb_regulator_1v8->rdev->use_count;
      //printk("[WIFI] power_on_cmmb1v8 is %d\n",value);
      regulator_set_mode(cmmb_regulator_1v8, REGULATOR_MODE_STANDBY);
      regulator_enable(cmmb_regulator_1v8);
      msleep(5);
}

//we will adjust later
int sprd_wifi_power(int on)
{
	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on)
	{
		clk_pin_init();
        wlan_clk_init();
	    power_on_cmmb1v8();
	}
	else
	{
	}	

	mdelay(1); /* Delay 1 ms, Recommand by Hardware */
	//gpio_set_value(SPRD_WIFI_PMENA_GPIO, on); /* WIFI_SHUTDOWN */
	gpio_request(SPRD_WIFI_PMENA_GPIO,"wifi_pwd");
    gpio_direction_output(SPRD_WIFI_PMENA_GPIO, on);
	gpio_free(SPRD_WIFI_PMENA_GPIO);
	mdelay(120);
	return 0;
}
EXPORT_SYMBOL(sprd_wifi_power);

int sprd_wifi_reset(int on)
{
	printk(KERN_INFO "%s: do nothing\n", __func__);
	return 0;
}
//HTC_WIFI_END
static unsigned int sprd_wifislot_type = MMC_TYPE_SDIO_WIFI;

static struct sprd_host_platdata sprd_sdio2_pdata = {
	.hw_name = "sprd-sdio2",
	.clk_name = "clk_sdio2",
	.clk_parent = "clk_192m", //WIFI NEED modify
        .max_clock = 192000000,
	//.clk_parent = "clk_48m",
	//.vdd_name = "vdd18",
	.enb_bit = BIT_SDIO2_EB,
	.rst_bit = BIT_SD2_SOFT_RST,
	//HTC_WIFI_START
	.status                 = sprd_wifi_status,
	.register_status_notify = sprd_wifi_status_register,
	.embedded_sdio          = &sprd_wifi_emb_data,  //We didnot use on cp2
	.slot_type	= &sprd_wifislot_type,
      //HTC_WIFI_END
};

static struct saudio_init_data  sprd_saudio_td={
	"VIRTUAL AUDIO",
	SIPC_ID_CPT,
	SMSG_CH_VBC,
	SMSG_CH_PLAYBACK,
	SMSG_CH_CAPTURE,
};

struct platform_device sprd_saudio_td_device = {
	.name       = "saudio",
	.id         = 0,
	.dev        = {.platform_data=&sprd_saudio_td},
};

struct platform_device sprd_sdio2_device = {
	.name = "sprd-sdhci",
	.id = 2,
	.num_resources = ARRAY_SIZE(sprd_sdio2_resources),
	.resource = sprd_sdio2_resources,
	.dev = { .platform_data = &sprd_sdio2_pdata },
};
static struct resource sprd_emmc_resources[] = {
	[0] = {
	       .start = SPRD_EMMC_BASE,
	       .end = SPRD_EMMC_BASE + SPRD_EMMC_SIZE - 1,
	       .name = "sdio3_res",
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_EMMC_INT,
	       .end = IRQ_EMMC_INT,
	       .flags = IORESOURCE_IRQ,
	       }
};

static struct sprd_host_platdata sprd_emmc_pdata = {
	.hw_name = "sprd-emmc",
	.vdd_name = "vddsd3",
	.clk_name = "clk_emmc",
	.clk_parent = "clk_384m",
	.max_clock = 384000000,
	.enb_bit = BIT_EMMC_EB,
	.rst_bit = BIT_EMMC_SOFT_RST,
	.regs.is_valid = 1,
};

struct platform_device sprd_emmc_device = {
	.name = "sprd-sdhci",
	.id = 3,
	.num_resources = ARRAY_SIZE(sprd_emmc_resources),
	.resource = sprd_emmc_resources,
	.dev = { .platform_data = &sprd_emmc_pdata },
};


#define TD_REG_CLK_ADDR				(SPRD_AHB_BASE + 0x250)
#define TD_REG_RESET_ADDR			(SPRD_AHB_BASE + 0x254)
#define TD_CTL_ENABLE				(0x01)
#define TD_CTL_DISENABLE			(0x00)
#define TD_CTL_CLK_VAL				(0x0F)

static int native_tdmodem_start(void *arg)
{
	u32 cpdata[3] = {0xe59f0000, 0xe12fff10, 0x80500000};
	/*disbale cp clock */
	__raw_writel(TD_CTL_DISENABLE, TD_REG_CLK_ADDR);
	/* hold stop cp */
	__raw_writel(TD_CTL_DISENABLE, TD_REG_RESET_ADDR);
	/*cp iram select to ap */
	memcpy((volatile u32*)SPRD_TDPROC_BASE, cpdata, sizeof(cpdata));
	/*enbale cp clock */
	__raw_writel(TD_CTL_CLK_VAL, TD_REG_CLK_ADDR);
	/* reset cp */
	__raw_writel(TD_CTL_ENABLE, TD_REG_RESET_ADDR);

	return 0;
}
static struct cproc_init_data sprd_cproc_td_pdata = {
	.devname	= "cproc_td",
	.base		= 0x80000000,
	.maxsz		= 0x02000000,
	.start		= native_tdmodem_start,
	.segnr		= 4,
	.segs		= {
		{
			.name  = "modem",
			.base  = 0x80500000,
			.maxsz = 0x00800000,
		},
		{
			.name  = "dsp",
			.base  = 0x80020000,
			.maxsz = 0x003E0000,
		},
		{
			.name  = "cmdline",
			.base  = 0x8047F400,
			.maxsz = 0x400,
		},
		{
			.name  = "dlnv",
			.base  = 0x80480000,
			.maxsz = 64 * 1024,
		},
#if 0
		{
			.name  = "fixnv",
			.base  = 0x80020000,
			.maxsz = 0x003E0000,
		},
		{
			.name  = "rumtimenv",
			.base  = 0x80020000,
			.maxsz = 0x003E0000,
		},
		{
			.name  = "protnv",
			.base  = 0x80020000,
			.maxsz = 0x003E0000,
		},
		{
			.name  = "rf",
			.base  = 0x80020000,
			.maxsz = 0x003E0000,
		},
#endif
	},
};

struct platform_device sprd_cproc_td_device = {
	.name           = "sprd_cproc",
	.id             = 0,
	.dev		= {.platform_data = &sprd_cproc_td_pdata},
};
static struct spipe_init_data sprd_spipe_td_pdata = {
	.name		= "spipe_td",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_PIPE,
	.ringnr		= 8,
	.txbuf_size	= 4096,
	.rxbuf_size	= 4096,
};
struct platform_device sprd_spipe_td_device = {
	.name           = "spipe",
	.id             = 0,
	.dev		= {.platform_data = &sprd_spipe_td_pdata},
};

static struct spipe_init_data sprd_slog_td_pdata = {
	.name		= "slog_td",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_PLOG,
	.ringnr		= 1,
	.txbuf_size	= 32 * 1024,
	.rxbuf_size	= 256 * 1024,
};
struct platform_device sprd_slog_td_device = {
	.name           = "spipe",
	.id             = 1,
	.dev		= {.platform_data = &sprd_slog_td_pdata},
};

static struct spipe_init_data sprd_stty_td_pdata = {
	/* to be compatible with vlx mux dev nodes */
	.name		= "ts0710mux",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_TTY,
	.ringnr		= 32,
	.txbuf_size	= 1024,
	.rxbuf_size	= 1024,
};
struct platform_device sprd_stty_td_device = {
	.name           = "spipe",
	.id             = 2,
	.dev		= {.platform_data = &sprd_stty_td_pdata},
};

static struct spool_init_data sprd_spool_td_pdata = {
    .name       = "spool_td",
    .dst        = SIPC_ID_CPT,
    .channel    = SMSG_CH_CTRL, /*TODO*/
	.txblocknum = 64,
    .txblocksize = 1516,
    .rxblocknum = 64,
    .rxblocksize = 1516,
};
struct platform_device sprd_spool_td_device = {
    .name           = "spool",
    .id             = 0,
    .dev            = {.platform_data = &sprd_spool_td_pdata},
};


static struct seth_init_data sprd_seth0_td_pdata = {
	.name		= "veth0",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_DATA0,
};
struct platform_device sprd_seth0_td_device = {
	.name           = "seth",
	.id             =  0,
	.dev		= {.platform_data = &sprd_seth0_td_pdata},
};

static struct seth_init_data sprd_seth1_td_pdata = {
	.name		= "veth1",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_DATA1,
};

struct platform_device sprd_seth1_td_device = {
	.name           = "seth",
	.id             =  1,
	.dev		= {.platform_data = &sprd_seth1_td_pdata},
};

static struct seth_init_data sprd_seth2_td_pdata = {
	.name		= "veth2",
	.dst		= SIPC_ID_CPT,
	.channel	= SMSG_CH_DATA2,
};
struct platform_device sprd_seth2_td_device = {
	.name           = "seth",
	.id             =  2,
	.dev		= {.platform_data = &sprd_seth2_td_pdata},
};

static struct resource sprd_pmu_resource[] = {
	[0] = {
		.start		= IRQ_CA5PMU_NCT_INT0,
		.end		= IRQ_CA5PMU_NCT_INT0,
		.flags		= IORESOURCE_IRQ,
	},
	[1] = {
		.start		= IRQ_CA5PMU_NCT_INT1,
		.end		= IRQ_CA5PMU_NCT_INT1,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device sprd_pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.resource = sprd_pmu_resource,
	.num_resources	= ARRAY_SIZE(sprd_pmu_resource),
};

static void sprd_init_pmu(void)
{
	platform_device_register(&sprd_pmu_device);
}
arch_initcall(sprd_init_pmu);

static struct seth_init_data sprd_seth_td_pdata = {
	.name       = "veth0",
	.dst        = SIPC_ID_CPT,
	.channel    = SMSG_CH_DATA0, //FIXME: conflict with 8825,3.4 linux/sipc.h is different with 3.0 ...
};

struct platform_device sprd_seth_td_device = {
	.name       = "seth",
	.id         =  0,
	.dev        = {.platform_data = &sprd_seth_td_pdata},
};

struct platform_device sprd_peer_state_device = {
	.name       = "peer_state",
	.id         = -1,
};


/*  sprd ram layout
 *
 *   |-----------|------------------|- pmem -|----------------------|
 *   |           |                                                  |
 *   |   MODEM   |            LINUX KERNEL MEMORY                   |
 *   |           |                                                  |
 *   |-----------|------------------|--------|----------------------|
 * 0x80000000  0x82000000           |    0x90000000         0x9fffffff
 *                        SPRD_RAM_CONSOLE_START
 */
struct sysdump_mem sprd_dump_mem[] = {
	{
		.paddr		= (CONFIG_PHYS_OFFSET & (~(SZ_256M - 1))),
		.vaddr		= 0,
		.soff		= 0xffffffff,
		.size		= (CONFIG_PHYS_OFFSET & (SZ_256M - 1)),
		.type	 	= SYSDUMP_MODEM,
	},
	{
		.paddr		= CONFIG_PHYS_OFFSET,
		.vaddr		= PAGE_OFFSET,
		.soff		= 0xffffffff,
		.size		= SPRD_RAM_CONSOLE_START - CONFIG_PHYS_OFFSET,
		.type	 	= SYSDUMP_RAM,
	},
	{
		.paddr		= CONFIG_PHYS_OFFSET + SZ_256M - CPT_TOTAL_SIZE,
		.vaddr		= PAGE_OFFSET + SZ_256M - CPT_TOTAL_SIZE,
		.soff		= 0xffffffff,
		.size		= 0, /* fill this dynamically according to real ram size */
		.type		= SYSDUMP_RAM,
	},
	{
		.paddr		= SPRD_AHB_PHYS,
		.vaddr		= SPRD_AHB_BASE,
		.soff		= 0x0,
		.size		= SPRD_AHB_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_INTC0_PHYS,
		.vaddr		= SPRD_INTC0_BASE,
		.soff		= 0x0,
		.size		= SPRD_INTC0_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_GPTIMER_PHYS,
		.vaddr		= SPRD_GPTIMER_BASE,
		.soff		= 0x0,
		.size		= SPRD_GPTIMER_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_ADI_PHYS,
		.vaddr		= SPRD_ADI_BASE,
		.soff		= 0x0,
		.size		= SPRD_ADI_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_GPIO_PHYS,
		.vaddr		= SPRD_GPIO_BASE,
		.soff		= 0x0,
		.size		= SPRD_GPIO_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_EIC_PHYS,
		.vaddr		= SPRD_EIC_BASE,
		.soff		= 0x0,
		.size		= SPRD_EIC_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
	{
		.paddr		= SPRD_GREG_PHYS,
		.vaddr		= SPRD_GREG_BASE,
		.soff		= 0x0,
		.size		= SPRD_GREG_SIZE,
		.type		= SYSDUMP_IOMEM,
	},
};
int sprd_dump_mem_num = ARRAY_SIZE(sprd_dump_mem);
