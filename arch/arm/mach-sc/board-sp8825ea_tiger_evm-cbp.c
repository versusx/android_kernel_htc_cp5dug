#include <linux/kernel.h>
#include <mach/hardware.h>
#include <mach/pinmap.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cbp_sdio.h>

/* GPIO setting */
#define GPIO_AP_WAKEUP_CP 58
#define GPIO_AP_READDY 196
#define GPIO_CP_WAKEUP_AP 117
#define GPIO_CP_READY 48

#define GPIO_MDM_RST_IND 51
#define GPIO_MDM_DATA_ACK 56
#define GPIO_MDM_FLOW_CTRL 57

#define GPIO_MDM_PWR_EN 62
#define GPIO_MDM_PMIC 143
#define GPIO_MDM_RST 141

#define GPIO_MDM_LV_SHIFT -1
#define GPIO_MDM_ETS_SEL -1
#define GPIO_MDM_USB_SWITCH -1
#define GPIO_MDM_SIM_SWAP -1
#define GPIO_SIM_DETECT -1

typedef struct {
	uint32_t reg;
	uint32_t val;
} pinmap_t;

static pinmap_t __initconst pinmap[] = {
	{REG_PIN_KEYIN4,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO58 -- AP_WAKEUP_CP
	{REG_PIN_RFCTL2,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO196 -- AP_READY
	{REG_PIN_LCD_D11,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO117 -- CP_WAKEUP_AP
	{REG_PIN_KEYOUT2,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO48 -- CP_READY

	{REG_PIN_KEYOUT5,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},
	//{REG_PIN_KEYOUT7,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_WPD|BIT_PIN_SLP_IE}, // GPIO53 -- MDM_RST_IND
	{REG_PIN_KEYIN2,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_WPU|BIT_PIN_SLP_WPU|BIT_PIN_SLP_IE}, // GPIO56 -- DATA_ACK
	{REG_PIN_KEYIN3,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE}, // GPIO57 -- FLOW_CTRL

	{REG_PIN_NFCLE,		BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_WPD|BIT_PIN_SLP_OE}, // GPIO62 -- PWR_ON
	{REG_PIN_GPIO143,	BITS_PIN_DS(1)|BITS_PIN_AF(0)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO143 -- MDM_PMIC
	{REG_PIN_CLK_AUX0,	BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_OE}, // GPIO141 -- MDM_RST

	{REG_PIN_U2TXD,               BITS_PIN_DS(1)|BITS_PIN_AF(0)|BIT_PIN_NUL|BIT_PIN_SLP_WPU|BIT_PIN_SLP_IE},
	{REG_PIN_U2RXD,               BITS_PIN_DS(1)|BITS_PIN_AF(0)|BIT_PIN_WPU|BIT_PIN_SLP_NUL|BIT_PIN_SLP_Z},
	{REG_PIN_U2CTS,               BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},
	{REG_PIN_U2RTS,               BITS_PIN_DS(1)|BITS_PIN_AF(3)|BIT_PIN_NUL|BIT_PIN_SLP_NUL|BIT_PIN_SLP_IE},
};

static int __init pin_init(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pinmap); i++) {
		__raw_writel(pinmap[i].val, CTL_PIN_BASE + pinmap[i].reg);
	}
	return 0;
}

static struct cbp_platform_data cbp_data = {
	.bus = "sdio",
	.host_id = "sprd-sdhci.1",
	.ipc_enable = false,
	.rst_ind_enable = false,
	.data_ack_enable = false,
	.flow_ctrl_enable = false,
	.tx_disable_irq = true,

	.gpio_ap_wkup_cp = GPIO_AP_WAKEUP_CP,
	.gpio_ap_ready = GPIO_AP_READDY,
	.gpio_cp_wkup_ap = GPIO_CP_WAKEUP_AP,
	.gpio_cp_ready = GPIO_CP_READY,

	.gpio_sync_polar = 1,

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
	.gpio_usb_switch = GPIO_MDM_USB_SWITCH,
	.gpio_sim_swap = GPIO_MDM_SIM_SWAP,
#ifdef CONFIG_CBP_SIM_HOTPLUG
	.gpio_sim_detect = GPIO_SIM_DETECT,
#endif
};

static struct platform_device cbp_device = {
	.name = CBP_DRIVER_NAME,
	.dev = {
		.platform_data = &cbp_data,
	},
};

static void __init cbp_init(void) {
	pin_init();
	if (platform_device_register(&cbp_device)) {
		printk(KERN_ERR "CBP device register fail.\n");
	}
}

device_initcall(cbp_init);
