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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/pm_runtime.h>

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <mach/regulator.h>
#include <mach/arch_misc.h>

#include "sc8830.h"

#define     SDIO_MAX_CLK            32000000

#define     SDIO_VDD_VOLT_1V8       1800000
#define     SDIO_VDD_VOLT_3V0       3000000
#define     SDIO_VDD_VOLT_3V3       3300000

#define MMC_RESTORE_REGS


#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
#include <mach/pinmap.h>
static unsigned int sdio_wakeup_irq;
#define HOST_WAKEUP_GPIO     22
#endif

#ifdef CONFIG_MMC_BUS_SCAN
static struct sdhci_host *sdhci_host_g = NULL;
#endif

extern void mmc_power_off(struct mmc_host* mmc);
extern void mmc_power_up(struct mmc_host* mmc);
extern void sd_buffer_pre_alloc(void);

static void *sdhci_get_platdata(struct sdhci_host *host)
{
	return ((struct sprd_host_data *)sdhci_priv(host))->platdata;
}

int is_wifi_slot(struct sdhci_host *host)
{
	struct sprd_host_data *host_data = sdhci_priv(host);
	if ((host_data->platdata->slot_type) && (*host_data->platdata->slot_type == MMC_TYPE_SDIO_WIFI))
	{
	    
		return 1;
	}

	return 0;
}

int is_cbp_slot(struct sdhci_host *host)
{
	struct sprd_host_data *host_data = sdhci_priv(host);

	if ((host_data->platdata->slot_type) && (*host_data->platdata->slot_type == MMC_TYPE_SDIO_CBP))
	{
		return 1;
	}
	return 0;
}

int is_sd_slot(struct sdhci_host *host)
{
	struct sprd_host_data *host_data = sdhci_priv(host);

	if ((host_data->platdata->slot_type) && (*host_data->platdata->slot_type == MMC_TYPE_SD))
	{
		return 1;
	}
	return 0;
}

#ifdef MMC_RESTORE_REGS
#if 0
static void sdhci_save_regs(struct sdhci_host *host)
{
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);
	if (!host_pdata->regs.is_valid) return;
	host_pdata->regs.addr = sdhci_readl(host, SDHCI_DMA_ADDRESS);
	host_pdata->regs.blk_size = sdhci_readw(host, SDHCI_BLOCK_SIZE);
	host_pdata->regs.blk_cnt = sdhci_readw(host, SDHCI_BLOCK_COUNT);
	host_pdata->regs.arg = sdhci_readl(host, SDHCI_ARGUMENT);
	host_pdata->regs.tran_mode = sdhci_readw(host, SDHCI_TRANSFER_MODE);
	host_pdata->regs.ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	host_pdata->regs.power = sdhci_readb(host, SDHCI_POWER_CONTROL);
	host_pdata->regs.clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
}

static void sdhci_restore_regs(struct sdhci_host *host)
{
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);
	if (!host_pdata->regs.is_valid) return;
	sdhci_writel(host, host_pdata->regs.addr, SDHCI_DMA_ADDRESS);
	sdhci_writew(host, host_pdata->regs.blk_size, SDHCI_BLOCK_SIZE);
	sdhci_writew(host, host_pdata->regs.blk_cnt, SDHCI_BLOCK_COUNT);
	sdhci_writel(host, host_pdata->regs.arg, SDHCI_ARGUMENT);
	sdhci_writew(host, host_pdata->regs.tran_mode, SDHCI_TRANSFER_MODE);
	sdhci_writeb(host, host_pdata->regs.ctrl, SDHCI_HOST_CONTROL);
	sdhci_writeb(host, host_pdata->regs.power, SDHCI_POWER_CONTROL);
	sdhci_writew(host, host_pdata->regs.clk, SDHCI_CLOCK_CONTROL);
}
#endif
#ifdef CONFIG_MMC_DEBUG
static void sdhci_dump_saved_regs(struct sdhci_host *host)
{
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);
	if (!host_pdata->regs.is_valid) return;
	printk("%s, host_addr:0x%x\n", host->hw_name, host_pdata->regs.addr);
	printk("%s, host_blk_size:0x%x\n", host->hw_name, host_pdata->regs.blk_size);
	printk("%s, host_blk_cnt:0x%x\n", host->hw_name, host_pdata->regs.blk_cnt);
	printk("%s, host_arg:0x%x\n", host->hw_name, host_pdata->regs.arg);
	printk("%s, host_tran_mode:0x%x\n", host->hw_name, host_pdata->regs.tran_mode);
	printk("%s, host_ctrl:0x%x\n", host->hw_name, host_pdata->regs.ctrl);
	printk("%s, host_power:0x%x\n", host->hw_name, host_pdata->regs.power);
	printk("%s, host_clk:0x%x\n", host->hw_name, host_pdata->regs.clk);
}
#endif
#endif

#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
static irqreturn_t sdhci_wakeup_irq_handler(int irq, void *dev)
{
	printk("sdhci_wakeup_irq_handler\n");
	
	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

void sdhci_set_data1_to_gpio(struct sdhci_host *host)
{
	unsigned int val;
	
	val = BITS_PIN_DS(1) | BITS_PIN_AF(3)  |
		BIT_PIN_WPU  | BIT_PIN_SLP_WPU |
		BIT_PIN_SLP_IE ;
	__raw_writel( val, CTL_PIN_BASE + REG_PIN_SD2_D1 );

	printk("%s, PIN_SD2_D1_REG:0x%x\n", __func__, __raw_readl(REG_PIN_SD2_D1));
	printk("sdhci_set_data1_to_gpio done\n");
}

void sdhci_set_gpio_to_data1(struct sdhci_host *host)
{
	unsigned int val;
	
	val = __raw_readl( CTL_PIN_BASE + REG_PIN_SD2_D1 );
	val = BITS_PIN_DS(1) | BITS_PIN_AF(0)  |
		BIT_PIN_WPU  | BIT_PIN_SLP_NUL |
		BIT_PIN_SLP_Z ;
	__raw_writel( val, CTL_PIN_BASE + REG_PIN_SD2_D1 );

	printk("%s, REG_PIN_SD2_D1:0x%x\n", __func__, __raw_readl(REG_PIN_SD2_D1));
	printk("sdhci_set_gpio_to_data1 done\n");
}


static void  sdhci_host_wakeup_set( struct sdhci_host *host )
{
	unsigned int val;
	int ret;


	if( (host->mmc->card )&& mmc_card_sdio(host->mmc->card) ){
		sdhci_set_data1_to_gpio(host);
		gpio_request(HOST_WAKEUP_GPIO, "host_wakeup_irq");
		sdio_wakeup_irq = gpio_to_irq(HOST_WAKEUP_GPIO);
		gpio_direction_input(HOST_WAKEUP_GPIO);
		ret = request_threaded_irq(sdio_wakeup_irq, sdhci_wakeup_irq_handler, NULL,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "host_wakeup_irq", host);
		if(ret){
			printk(KERN_ERR "%s, request threaded irq error:%d\n",
				mmc_hostname(host->mmc), ret);
			return;
		}
		enable_irq_wake(sdio_wakeup_irq);
	}
	return;
}

static void  sdhci_host_wakeup_clear(struct sdhci_host *host)
{
       if( (host->mmc->card )&& mmc_card_sdio(host->mmc->card) ){
		disable_irq_wake(sdio_wakeup_irq);
		free_irq(sdio_wakeup_irq, host);
		gpio_free(HOST_WAKEUP_GPIO);
		sdhci_set_gpio_to_data1(host);
        }
	return;
}
#endif

#ifdef CONFIG_MMC_BUS_SCAN
void sdhci_bus_scan(void)
{
#ifdef CONFIG_MMC_BUS_SCAN
	if(sdhci_host_g && (sdhci_host_g->mmc)){
		printk("%s, entry\n", __func__);
		if (sdhci_host_g->ops->set_clock) {
			sdhci_host_g->ops->set_clock(sdhci_host_g, 1);
		}

		
		mmc_detect_change(sdhci_host_g->mmc, 0);
	}
#endif
	return;
}
EXPORT_SYMBOL_GPL(sdhci_bus_scan);

int sdhci_device_attach(int on)
{
	struct mmc_host *mmc = NULL;
	if(sdhci_host_g && (sdhci_host_g->mmc)){
		mmc = sdhci_host_g->mmc;
		if(mmc->card){
			sdhci_host_g->dev_attached = on;
			if(!on){
				mmc_power_off(mmc);
			}else{
				mmc_power_up(mmc);
			}
		}else{
			
			sdhci_host_g->dev_attached = 0;
			return -1;
		}
		return 0;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sdhci_device_attach);

int sdhci_device_attached(void)
{
	struct mmc_host *mmc = NULL;
	if(sdhci_host_g && (sdhci_host_g->mmc)){
		mmc = sdhci_host_g->mmc;
		if(mmc->card){
			return true;
		}else{
			return false;
		}
	}
	return false;
}
EXPORT_SYMBOL_GPL(sdhci_device_attached);
#endif

static unsigned int sdhci_sprd_get_max_clk(struct sdhci_host *host)
{
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);

	if (host_pdata->max_clock)
		return host_pdata->max_clock;
	else
		return SDIO_MAX_CLK;						
}

static void sdhci_sprd_set_base_clock(struct sdhci_host *host)
{
#ifndef CONFIG_MACH_SPX35FPGA
	struct clk *clk_parent;
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);

	if(is_wifi_slot(host))
        printk("[WLAN] read chip id is 0x%x \n",sci_get_chip_id());

	
	if (((0 == strcmp(host_pdata->hw_name, "sprd-sdio1"))
		|| (0 == strcmp(host_pdata->hw_name, "sprd-sdio2")))
		&& ( soc_is_scx35_v0())){
		host_pdata->clk_parent =  "clk_48m";
		host_pdata->max_clock = 48000000;
	}

	
	host->clk = clk_get(NULL, host_pdata->clk_name);
	clk_parent = clk_get(NULL, host_pdata->clk_parent);
	BUG_ON(IS_ERR(host->clk) || IS_ERR(clk_parent));
	clk_set_parent(host->clk, clk_parent);
#endif
	return;
}

static void sdhci_sprd_enable_clock(struct sdhci_host *host, unsigned int clock)
{
#ifndef CONFIG_MACH_SPX35FPGA
	struct sprd_host_data *host_data= sdhci_priv(host);
	if(clock == 0){
		if (host_data->clk_enable) {
			sdhci_sdclk_enable(host, 0);
			clk_disable(host->clk);
			pr_debug("******* %s, call  clk_disable*******\n", mmc_hostname(host->mmc));
			host->clock = 0;
			host_data->clk_enable = 0;
		}
	}else{
		if (0 == host_data->clk_enable) {
			clk_enable(host->clk);
			pr_debug("******* %s, call  clk_enable*******\n", mmc_hostname(host->mmc));
			sdhci_sdclk_enable(host, 1);
			host_data->clk_enable = 1;
		}
	}
	pr_debug("clock:%d, host->clock:%d, AHB_CTL1:0x%x\n", clock,host->clock,
			sci_glb_raw_read(REG_AP_AHB_AHB_EB));
#endif
	return;
}

#if 0
static int __regulator_force_disable(struct regulator *regulator)
{
	int i = 0;
	while(1 == regulator_is_enabled(regulator)) {
		regulator_disable(regulator);
		i++;
	};
	printk("__regulator_force_disable count= %d\n",i);
	return 0;
}
#endif

static void sdhci_sprd_set_power(struct sdhci_host *host, unsigned int power)
{
	unsigned int volt_level = 0;
	unsigned int volt_ext_level = SDIO_VDD_VOLT_3V0;
	int ret;
	struct sprd_host_platdata *host_pdata = sdhci_get_platdata(host);

	if(host->vmmc == NULL){
		host->vmmc = regulator_get(NULL, host_pdata->vdd_name);
		if (IS_ERR(host->vmmc)) {
			printk(KERN_ERR "%s: no vmmc regulator found\n",
						mmc_hostname(host->mmc));
			host->vmmc = NULL;
		}
	}

	switch(power){
	case SDHCI_POWER_180:
		volt_level = SDIO_VDD_VOLT_1V8;
		break;
	case SDHCI_POWER_300:
		volt_level = SDIO_VDD_VOLT_3V0;
		break;
	case SDHCI_POWER_330:
		volt_level = SDIO_VDD_VOLT_3V3;
		break;
	default:
		;
	}

	pr_debug("%s, power:%d, set regulator voltage:%d\n",
			mmc_hostname(host->mmc), power, volt_level);
	if(volt_level == 0){
		if (host->vmmc)
			ret = regulator_disable(host->vmmc);
		if (host->vmmc_ext)
			ret = regulator_disable(host->vmmc_ext);
	}else{
		if(host->vmmc){
			ret = regulator_set_voltage(host->vmmc, volt_level,
								volt_level);
			if(ret){
				printk(KERN_ERR "%s, set voltage error:%d\n",
					mmc_hostname(host->mmc), ret);
				return;
			}
			printk(KERN_ERR "%s, enabel regulator vmmc\n",mmc_hostname(host->mmc));
			ret = regulator_enable(host->vmmc);
			if(ret){
				printk(KERN_ERR "%s, enabel regulator error:%d\n",
					mmc_hostname(host->mmc), ret);
				return;
			}
		}
		if(host->vmmc_ext){
			ret = regulator_set_voltage(host->vmmc_ext, volt_ext_level,
								volt_ext_level);
			if(ret){
				printk(KERN_ERR "%s, vmmc_ext set voltage error:%d\n",
					mmc_hostname(host->mmc), ret);
				return;
			}
			printk(KERN_ERR "%s, enabel regulator vmmc_ext\n",mmc_hostname(host->mmc));
			ret = regulator_enable(host->vmmc_ext);
			if(ret){
				printk(KERN_ERR "%s, vmmc_ext enabel regulator error:%d\n",
					mmc_hostname(host->mmc), ret);
				return;
			}
		}
	}
	return;
}

static struct sdhci_ops sdhci_sprd_ops = {
	.get_max_clock		= sdhci_sprd_get_max_clk,
	.set_clock		= sdhci_sprd_enable_clock,
	.set_power		= sdhci_sprd_set_power,
};

static void sdhci_module_init(struct sdhci_host* host)
{
	struct sprd_host_platdata *host_pdata;
	host_pdata = sdhci_get_platdata(host);
	
	sci_glb_set(REG_AP_AHB_AHB_EB, host_pdata->enb_bit);

	
	sdhci_sdclk_enable(host, 0);

	
	sci_glb_set(REG_AP_AHB_AHB_RST, host_pdata->rst_bit);
	udelay(200);
	sci_glb_clr(REG_AP_AHB_AHB_RST, host_pdata->rst_bit);
	sdhci_sprd_set_base_clock(host);
	host->ops->set_clock(host, true);

}

static void sdhci_status_notify_cb(int card_present, void *dev_id)
{
	struct sdhci_host *host = dev_id;
	unsigned int status;
	struct sprd_host_data *host_data = sdhci_priv(host);

	printk("[WLAN] %s: card_present %d\n", mmc_hostname(host->mmc),
			card_present);

	status = host_data->platdata->status(mmc_dev(host->mmc));
	host->eject = !status;
	pr_info("[WLAN] %s: Slot status change detected (%d -> %d)\n",
			mmc_hostname(host->mmc), host->oldstat, status);
	if (status != host->oldstat) {
		if (status)
			
			mmc_detect_change(host->mmc, 0);
		else
			mmc_detect_change(host->mmc, msecs_to_jiffies(100));
	}
	host->oldstat = status;
}

static int __devinit sdhci_sprd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct resource *res;
	int ret, irq;
#ifdef CONFIG_MMC_CARD_HOTPLUG
	int sd_detect_gpio;
	int detect_irq;
#endif
	struct sprd_host_data *host_data;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory specified\n");
		return -ENOENT;
	}

	if(pdev->id == 0)
	{
		sd_buffer_pre_alloc();
	}

	host = sdhci_alloc_host(dev, sizeof(struct sprd_host_data));
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}

	host_data = sdhci_priv(host);
	host_data->platdata = dev_get_platdata(dev);
	host_data->clk_enable = 0;
	BUG_ON(NULL == host_data->platdata);
	printk("sdio probe %s, vdd %s (%d), clk %s parent %s\n",
		host_data->platdata->hw_name,
		host_data->platdata->vdd_name,	host_data->platdata->volt_level,
		host_data->platdata->clk_name, host_data->platdata->clk_parent);

	platform_set_drvdata(pdev, host);
	host->vmmc = NULL;
	host->vmmc_ext = NULL;
	host->ioaddr = (void __iomem *)res->start;
	printk("sdio: host->ioaddr:0x%x\n", (u32)host->ioaddr);
	host->hw_name = (host_data->platdata->hw_name)?
		host_data->platdata->hw_name:pdev->name;
	host->ops = &sdhci_sprd_ops;
	host->quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |\
		SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |\
		SDHCI_QUIRK_BROKEN_CARD_DETECTION|\
		SDHCI_QUIRK_INVERTED_WRITE_PROTECT;
	host->irq = irq;
#ifdef CONFIG_MMC_CARD_HOTPLUG
	sd_detect_gpio = host_data->platdata->detect_gpio;
	if(sd_detect_gpio > 0){
		pr_info("%s, sd_detect_gpio:%d\n", __func__, sd_detect_gpio);

		if (0 == pdev->id){
			ret = gpio_request(sd_detect_gpio, "sdio0_detect");
		}else{
			ret = gpio_request(sd_detect_gpio, "sdio1_detect");
		}

		if (ret) {
			dev_err(dev, "cannot request gpio\n");
			return -1;
		}

		ret = gpio_direction_input(sd_detect_gpio);
		if (ret) {
			dev_err(dev, "gpio can not change to input\n");
			return -1;
		}

		detect_irq = gpio_to_irq(sd_detect_gpio);
		if (detect_irq < 0){
			dev_err(dev, "cannot alloc detect irq\n");
			return -1;
		}
		host_data->detect_irq = detect_irq;
	}else if (host_data->platdata->register_status_notify) {
		host_data->platdata->register_status_notify(sdhci_status_notify_cb, host);
	} else if (!host_data->platdata->status) {
		printk("%s: No card detect facilities available\n",
				mmc_hostname(host->mmc));
	}else{
		printk("%s, sd_detect_gpio == 0 \n", __func__ );
	}
#endif

	if(host_data->platdata->vdd_name) {
		host->vmmc = regulator_get(NULL, host_data->platdata->vdd_name);
		BUG_ON(IS_ERR(host->vmmc));
		
		if(!is_sd_slot(host))
			regulator_enable(host->vmmc);
	}
	if(host_data->platdata->vdd_ext_name) {
		host->vmmc_ext = regulator_get(NULL, host_data->platdata->vdd_ext_name);
		BUG_ON(IS_ERR(host->vmmc_ext));
		regulator_enable(host->vmmc_ext);
	}

	host->clk = NULL;
	sdhci_module_init(host);

	switch(pdev->id) {
		case 0:
			host->mmc->pm_flags |= MMC_PM_ONLY_USED_SDIO0_SHARK;
			host->caps = sdhci_readl(host, SDHCI_CAPABILITIES) & (~(SDHCI_CAN_VDD_330 | SDHCI_CAN_VDD_180));
			host->quirks |= SDHCI_QUIRK_MISSING_CAPS;
			host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
			break;
		case 1:
		        host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY|MMC_PM_KEEP_POWER |MMC_PM_DISABLE_TIMEOUT_IRQ;
                host->mmc->pm_caps |= MMC_PM_KEEP_POWER |MMC_PM_DISABLE_TIMEOUT_IRQ;
				#ifdef CONFIG_SDIO_CARD
                host->mmc->caps |= MMC_CAP_4_BIT_DATA |MMC_CAP_NONREMOVABLE | MMC_CAP_POWER_OFF_CARD;
				#else
				host->mmc->caps |= MMC_CAP_4_BIT_DATA |MMC_CAP_NONREMOVABLE;
				#endif
			break;
		case 2:
			host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY | MMC_PM_KEEP_POWER;
			host->mmc->caps |= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD;
			break;
		case 3:
			host->caps = sdhci_readl(host, SDHCI_CAPABILITIES) & (~(SDHCI_CAN_VDD_330 | SDHCI_CAN_VDD_300));
			host->quirks |= SDHCI_QUIRK_MISSING_CAPS;
			host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY | MMC_PM_KEEP_POWER;
			host->mmc->caps |= MMC_CAP_NONREMOVABLE | MMC_CAP_8_BIT_DATA | MMC_CAP_HW_RESET;
			break;
		default:
			BUG();
			break;
	}

#ifdef CONFIG_PM_RUNTIME
		switch(pdev->id) {
			case 1:
			case 2:
                        #ifdef CONFIG_SDIO_CARD
			pm_runtime_set_active(&pdev->dev);
			pm_runtime_set_autosuspend_delay(&pdev->dev, 100);
			pm_runtime_use_autosuspend(&pdev->dev);
			pm_runtime_enable(&pdev->dev);
			pm_runtime_no_callbacks(mmc_classdev(host->mmc));
			pm_suspend_ignore_children(mmc_classdev(host->mmc), true);
			pm_runtime_set_active(mmc_classdev(host->mmc));
			pm_runtime_enable(mmc_classdev(host->mmc));
                        #endif
			break;
			case 3:
			case 0:
			pm_suspend_ignore_children(&pdev->dev, true);
			pm_runtime_set_active(&pdev->dev);
			pm_runtime_set_autosuspend_delay(&pdev->dev, 100);
			pm_runtime_use_autosuspend(&pdev->dev);
			pm_runtime_enable(&pdev->dev);
			default:
			break;
			}
#endif

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		goto err_add_host;
	}

#ifdef CONFIG_MMC_BUS_SCAN
	if (pdev->id == 1)
		sdhci_host_g = host;
#endif

	return 0;

err_add_host:
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&(pdev)->dev);
	pm_runtime_set_suspended(&(pdev)->dev);
#endif
	if (host_data->clk_enable) {
		clk_disable(host->clk);
		host_data->clk_enable = 0;
	}
	sdhci_free_host(host);
	return ret;
}

static int __devexit sdhci_sprd_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sprd_host_data *host_data = sdhci_priv(host);

#ifdef CONFIG_PM_RUNTIME
	if (pm_runtime_suspended(&(pdev)->dev))
		pm_runtime_resume(&(pdev)->dev);
#endif
	sdhci_remove_host(host, 1);
	sdhci_free_host(host);

	if (host_data->clk_enable) {
		clk_disable(host->clk);
		host_data->clk_enable = 0;
	}

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&(pdev)->dev);
	pm_runtime_set_suspended(&(pdev)->dev);
#endif
	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int sprd_mmc_host_runtime_suspend(struct device *dev) {
    int rc = -EBUSY;
    unsigned long flags;
    struct platform_device *pdev = container_of(dev, struct platform_device, dev);
    struct sdhci_host *host = platform_get_drvdata(pdev);
    
    if(dev->driver != NULL) {
        sdhci_runtime_suspend_host(host);
        spin_lock_irqsave(&host->lock, flags);
        if(host->ops->set_clock)
            host->ops->set_clock(host, 0);
        spin_unlock_irqrestore(&host->lock, flags);
        rc = 0;
    }
    return rc;
}

static int sprd_mmc_host_runtime_resume(struct device *dev) {
    unsigned long flags;
    struct platform_device *pdev = container_of(dev, struct platform_device, dev);
    struct sdhci_host *host = platform_get_drvdata(pdev);
    
    if(dev->driver != NULL) {
        if(host->ops->set_clock) {
            spin_lock_irqsave(&host->lock, flags);
            host->ops->set_clock(host, 1);
            spin_unlock_irqrestore(&host->lock, flags);
            mdelay(10);
        }
        sdhci_runtime_resume_host(host);
    }
    return 0;
}

static int sprd_mmc_host_runtime_idle(struct device *dev) {
    return 0;
}
#endif

volatile bool dhd_mmc_suspend=0;

EXPORT_SYMBOL(dhd_mmc_suspend);

static int sdhci_pm_suspend(struct device *dev) {
    int				 retval = 0;
    struct platform_device	*pdev	= container_of(dev, struct platform_device, dev);
    struct sdhci_host		*host	= platform_get_drvdata(pdev);
    printk("%s: %s enter\n", mmc_hostname(host->mmc), __func__);
    if(is_wifi_slot(host))
      {
        printk("[WLAN] suspend read REG_AP_CLK_AP_AHB_CFG is %x\n",__raw_readl(REG_AP_CLK_AP_AHB_CFG));
	dhd_mmc_suspend = 1;
	printk("[WLAN] %s,dhd_mmc_suspend=%d\n",__func__,dhd_mmc_suspend);
      }
#ifdef CONFIG_PM_RUNTIME
    if(pm_runtime_enabled(dev))
        retval = pm_runtime_get_sync(dev);
#endif
    if(retval >= 0) {
            retval = sdhci_suspend_host(host, PMSG_SUSPEND);
            if(!retval) {
                unsigned long flags;
#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
                if (pdev->id == 1)
                    sdhci_host_wakeup_set(host);
#endif
                spin_lock_irqsave(&host->lock, flags);
                if(host->ops->set_clock)
                    host->ops->set_clock(host, 0);
                spin_unlock_irqrestore(&host->lock, flags);
            } else {
#ifdef CONFIG_PM_RUNTIME
                if(pm_runtime_enabled(dev))
                    pm_runtime_put_autosuspend(dev);
#endif
            }
    }

    printk("%s: %s leave retval %d\n", mmc_hostname(host->mmc), __func__, retval);
    return retval;
}



static int sdhci_pm_resume(struct device *dev) {
    int				 retval = 0;
    unsigned long		 flags;
    struct platform_device	*pdev	= container_of(dev, struct platform_device, dev);
    struct sdhci_host		*host	= platform_get_drvdata(pdev);
    printk("%s: %s enter\n", mmc_hostname(host->mmc), __func__);
    if(is_wifi_slot(host) || is_cbp_slot(host) || is_sd_slot(host)) {
	    sdhci_sprd_set_base_clock(host);
    }
    spin_lock_irqsave(&host->lock, flags);
    if(host->ops->set_clock)
        host->ops->set_clock(host, 1);
    spin_unlock_irqrestore(&host->lock, flags);

    if(is_wifi_slot(host))
    {
        printk("[WLAN] resume read REG_AP_CLK_AP_AHB_CFG is %x\n",__raw_readl(REG_AP_CLK_AP_AHB_CFG));
		printk("[WLAN] resume read REG_AP_CLK_SDIO1_CFG is %x\n",__raw_readl(REG_AP_CLK_SDIO1_CFG));
	}
    
#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
    if (pdev->id == 1)
        sdhci_host_wakeup_clear(host);
#endif
    retval = sdhci_resume_host(host);
#ifdef CONFIG_PM_RUNTIME
    if(pm_runtime_enabled(dev))
        pm_runtime_put_autosuspend(dev);
#endif
    if(is_wifi_slot(host))
     {
        dhd_mmc_suspend	= 0;
	printk("[WLAN] %s,dhd_mmc_suspend=%d\n",__func__,dhd_mmc_suspend);
     }
    printk("%s: %s leave retval %d\n", mmc_hostname(host->mmc), __func__, retval);
    return retval;
}

static const struct dev_pm_ops sdhci_dev_pm_ops = {
	.suspend	 = sdhci_pm_suspend,
	.resume		 = sdhci_pm_resume,

	.runtime_suspend = sprd_mmc_host_runtime_suspend,
	.runtime_resume  = sprd_mmc_host_runtime_resume,
	.runtime_idle    = sprd_mmc_host_runtime_idle,
};
#else
static const struct dev_pm_ops sdhci_dev_pm_ops = {
	.suspend	 = NULL,
	.resume		 = NULL,
	.runtime_suspend = NULL,
	.runtime_resume  = NULL,
	.runtime_idle    = NULL,
};

#endif



static void sdhci_shutdown(struct platform_device *pdev)
{
       struct sdhci_host *host = platform_get_drvdata(pdev);
       u32 regVal;

       printk("SHUTDWON Emmc :pdev%x  host:%x,\n",pdev,host);

	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	regVal = sdhci_readl(host, SDHCI_CLOCK_CONTROL);
	regVal &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writel(host, regVal, SDHCI_CLOCK_CONTROL);
	spin_unlock_irqrestore(&host->lock, flags);
}



static struct platform_driver sdhci_sprd_driver = {
	.probe		= sdhci_sprd_probe,
	.driver		= {
		.owner		= THIS_MODULE,
#ifdef CONFIG_PM_RUNTIME
		.pm 		=  &sdhci_dev_pm_ops,
#endif
		.name		= "sprd-sdhci",
	},
	.shutdown	= &sdhci_shutdown,
};

static int __init sdhci_sprd_init(void)
{
	return platform_driver_register(&sdhci_sprd_driver);
}

static void __exit sdhci_sprd_exit(void)
{
	platform_driver_unregister(&sdhci_sprd_driver);
}

module_init(sdhci_sprd_init);
module_exit(sdhci_sprd_exit);

MODULE_DESCRIPTION("Spredtrum SDHCI glue");
MODULE_AUTHOR("spreadtrum.com");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sprd-sdhci");
