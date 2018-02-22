#include <linux/kernel.h>
#include <mach/hardware.h>
#include <mach/pinmap.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cbp_sdio.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/module.h>

/* GPIO setting */
#define GPIO_AP_WAKEUP_CP 147
#define GPIO_AP_READDY 131
#define GPIO_CP_WAKEUP_AP 124
#define GPIO_CP_READY 123

#define GPIO_MDM_RST_IND 120
#define GPIO_MDM_DATA_ACK 126
#define GPIO_MDM_FLOW_CTRL 125

#define GPIO_MDM_PWR_EN 134
#define GPIO_MDM_PMIC 122
#define GPIO_MDM_RST 121

#define GPIO_MDM_LV_SHIFT -1
#define GPIO_MDM_ETS_SEL 127
#define GPIO_MDM_ETS_SEL1 128
#define GPIO_MDM_USB_SWITCH 231
#define GPIO_MDM_SIM_SWAP 152
#define GPIO_SIM_DETECT 181

#define GPIO_ANT_CTRL 29

typedef struct {
	uint32_t reg;
	uint32_t val;
} pinmap_t;

static pinmap_t pinmap_init[] = {
	{REG_PIN_NFWPN,		BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO147 -- AP_WAKEUP_CP
	{REG_PIN_LCD_D23,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO131 -- AP_READY
	{REG_PIN_LCD_D16,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO124 -- CP_WAKEUP_AP
	{REG_PIN_LCD_D15,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO123 -- CP_READY

	{REG_PIN_LCD_D12,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO120 -- MDM_RST_IND
	{REG_PIN_LCD_D18,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO126 -- DATA_ACK
	{REG_PIN_LCD_D17,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO125 -- FLOW_CTRL

	{REG_PIN_LCD_D14,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO122 -- MDM_PMIC
	{REG_PIN_LCD_D13,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO121 -- MDM_RST
	{REG_PIN_SPI2_DI,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO134 -- PWR_ON

	{REG_PIN_LCD_D19,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO127 -- ETS_SEL
	{REG_PIN_LCD_D20,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO128 -- ETS_SEL1
	{REG_PIN_LCD_D21,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO129 -- WAKEUP_UART
	{REG_PIN_LCD_D22,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO130 -- AP_UART_RDY

	{REG_PIN_NFCLE,		BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // UART TXD
	{REG_PIN_NFALE,		BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // UART RXD
	{REG_PIN_NFREN,		BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // UART CTS
	{REG_PIN_NFWEN,		BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // UART RTS

	/* --SDIO-- */
	{REG_PIN_NFD9,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},
	{REG_PIN_NFD10,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},
	{REG_PIN_NFD11,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},
	{REG_PIN_NFD12,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},

	{REG_PIN_NFD14,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // CMD
	{REG_PIN_NFD15,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // CLK

	{REG_PIN_TRACECTRL,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, //GPIO 231 -- MDM_USB_SWITCH
	{REG_PIN_NFCEN1,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE},//GPIO152 MDM_SIM_SWAP
};

static pinmap_t pinmap[] = {
	{REG_PIN_NFWPN,		BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO147 -- AP_WAKEUP_CP
	{REG_PIN_LCD_D23,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO131 -- AP_READY
	{REG_PIN_LCD_D16,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO124 -- CP_WAKEUP_AP
	{REG_PIN_LCD_D15,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO123 -- CP_READY

	{REG_PIN_LCD_D12,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO120 -- MDM_RST_IND
	{REG_PIN_LCD_D18,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO126 -- DATA_ACK
	{REG_PIN_LCD_D17,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO125 -- FLOW_CTRL

	{REG_PIN_LCD_D14,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO122 -- MDM_PMIC
	{REG_PIN_LCD_D13,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO121 -- MDM_RST
	{REG_PIN_SPI2_DI,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO134 -- PWR_ON

	{REG_PIN_LCD_D19,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO127 -- ETS_SEL
	{REG_PIN_LCD_D20,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO128 -- ETS_SEL1
	{REG_PIN_LCD_D21,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO129 -- WAKEUP_UART
	{REG_PIN_LCD_D22,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO130 -- AP_UART_RDY
	{REG_PIN_TRACECTRL,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, //GPIO 231 -- MDM_USB_SWITCH

	{REG_PIN_NFCLE,		BITS_PIN_DS(1)|BITS_PIN_AF(1)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // UART TXD
	{REG_PIN_NFALE,		BITS_PIN_DS(1)|BITS_PIN_AF(1)|BIT_PIN_WPU|BIT_PIN_SLP_WPU|BIT_PIN_SLP_IE}, // UART RXD
	{REG_PIN_NFREN,		BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // UART CTS
	{REG_PIN_NFWEN,		BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // UART RTS

	{REG_PIN_NFCEN1,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE},//GPIO152 MDM_SIM_SWAP
#if 0
	{REG_PIN_CCIRD5,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},//GPIO181 SIM_DETECT
#endif
	/* --SDIO-- */
	{REG_PIN_NFD9,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(1)|BIT_PIN_WPU|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},
	{REG_PIN_NFD10,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(1)|BIT_PIN_WPU|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},
	{REG_PIN_NFD11,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(1)|BIT_PIN_WPU|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},
	{REG_PIN_NFD12,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(1)|BIT_PIN_WPU|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},

	{REG_PIN_NFD14,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(1)|BIT_PIN_WPU|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // CMD
	{REG_PIN_NFD15,		BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(1)|BIT_PIN_WPD|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // CLK

	{REG_PIN_CLK_AUX0,	BIT_PIN_NULL|BITS_PIN_DS(1)|BITS_PIN_AF(0)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE},//GPIO 209 -- CBP_SLEEP_CLK
};
#if 0
static void cbp_clk_exit(void)
{
	struct clk *cbp_clk;

	cbp_clk = clk_get(NULL,"clk_aux0");
	clk_disable(cbp_clk);
	clk_put(cbp_clk);
}
#endif

static void cbp_clk_init(void)
{
        struct clk *cbp_clk;
        struct clk *clk_parent;

	cbp_clk = clk_get(NULL, "clk_aux0");

        if (IS_ERR(cbp_clk)) {
                printk(KERN_ERR "clock: failed to get clk_aux0\n");
        }
        clk_parent = clk_get(NULL, "ext_32k");
        if (IS_ERR(clk_parent)) {
                printk(KERN_ERR "failed to get parent ext_32k\n");
        }

        clk_set_parent(cbp_clk, clk_parent);
        clk_set_rate(cbp_clk, 32768);
        clk_enable(cbp_clk);
}

void cbp_power_init(int enable)
{
	int i = 0;
	if (enable) {
		for (i = 0; i < ARRAY_SIZE(pinmap); i++) {
			__raw_writel(pinmap[i].val, CTL_PIN_BASE + pinmap[i].reg);
		}
	} else {
#if 0
		for (i = 0; i < ARRAY_SIZE(pinmap_init); i++) {
			__raw_writel(pinmap_init[i].val, CTL_PIN_BASE + pinmap_init[i].reg);
		}
#endif
	}
}
EXPORT_SYMBOL(cbp_power_init);

static int __init pin_init(void)
{
	int i;
	uint32_t addr;
	for (i = 0; i < ARRAY_SIZE(pinmap_init); i++) {
		__raw_writel(pinmap_init[i].val, CTL_PIN_BASE + pinmap_init[i].reg);
	}
#define U4CTS_PIN_IN_SEL(_x_)	( ((_x_) << 5) & (BIT(5)) )
#define U4RXD_PIN_IN_SEL(_x_)	( ((_x_) << 3) & (BIT(3)|BIT(4)) )
	addr = CTL_PIN_BASE + REG_PIN_CTRL1;
	__raw_writel(readl(addr) | U4CTS_PIN_IN_SEL(1) | U4RXD_PIN_IN_SEL(1), addr);
#define UART4_SYS_SEL_MASK	( ~(BIT(16)|BIT(17)|BIT(18)) )
	addr = CTL_PIN_BASE + REG_PIN_CTRL2;
	__raw_writel(readl(addr) & UART4_SYS_SEL_MASK, addr);
	return 0;
}

static struct cbp_platform_data cbp_data = {
	.bus = "sdio",
	.host_id = "sprd-sdhci.2",
	.ipc_enable = false,
	.rst_ind_enable = false,
	.data_ack_enable = false,
	.flow_ctrl_enable = false,
	.tx_disable_irq = true,

	.gpio_ap_wkup_cp = GPIO_AP_WAKEUP_CP,
	.gpio_ap_ready = GPIO_AP_READDY,
	.gpio_cp_wkup_ap = GPIO_CP_WAKEUP_AP,
	.gpio_cp_ready = GPIO_CP_READY,

	.gpio_sync_polar = 0,

	.gpio_rst_ind = GPIO_MDM_RST_IND,
	.gpio_rst_ind_polar = 0,

	.gpio_data_ack = GPIO_MDM_DATA_ACK,
	.gpio_data_ack_polar = 1,

	.gpio_flow_ctrl = GPIO_MDM_FLOW_CTRL,
	.gpio_flow_ctrl_polar = 0,

	.gpio_pwr_on = GPIO_MDM_PWR_EN,
	.gpio_pmic = GPIO_MDM_PMIC,
	.gpio_rst = GPIO_MDM_RST,
	//for the level transfor chip fssd06
	.gpio_lv_shift = GPIO_MDM_LV_SHIFT,

	.gpio_ets_sel = GPIO_MDM_ETS_SEL,
	.gpio_ets_sel1 = GPIO_MDM_ETS_SEL1,
	.gpio_usb_switch = GPIO_MDM_USB_SWITCH,
	.gpio_sim_swap = -1,
	.gpio_ant_ctrl = -1,
#ifdef CONFIG_CBP_SIM_HOTPLUG
	.gpio_sim_detect = -1,
#endif
};

static struct platform_device cbp_device = {
	.name = CBP_DRIVER_NAME,
	.dev = {
		.platform_data = &cbp_data,
	},
};

static int __init cbp_init(void) {
	pin_init();
	cbp_clk_init();
	if (platform_device_register(&cbp_device)) {
		printk(KERN_ERR "CBP device register fail.\n");
		return -1;
	}
	return 0;
}

device_initcall(cbp_init);
