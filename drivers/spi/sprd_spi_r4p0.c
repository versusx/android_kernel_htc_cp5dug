/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
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
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/sched.h>
#include <linux/kthread.h>

#include <mach/hardware.h>

#include <mach/hardware.h>
#include <mach/sci_glb_regs.h>

#include "sprd_spi_r4p0.h"

#define ALIGN_UP(a, b)\
	(((a) + ((b) - 1)) & ~((b) - 1))

#define MHz(inte, dec) ((inte) * 1000 * 1000 + (dec) * 1000 * 100)

struct sprd_spi_devdata {
	void __iomem *reg_base;
	int irq_num;
	struct clk *clk;
	
	struct clk *parent_clk;
	u32 src_clk;
	spinlock_t lock;
	struct list_head msg_queue;
	struct work_struct work;
	struct workqueue_struct *work_queue;
	u32 reg_backup[11];
	bool is_active;
};

extern void clk_force_disable(struct clk *);

static void sprd_spi_wait_for_idle(void __iomem *reg_base)
{
	u32 timeout = 0;
	while (!(__raw_readl(reg_base + SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY))
	{
		if (++timeout > SPI_TIME_OUT) {
			
			printk("spi send timeout!\n");
			BUG_ON(1);
		}
	}

	while (__raw_readl(reg_base + SPI_STS2) & SPI_TX_BUSY)
	{
		if (++timeout > SPI_TIME_OUT) {
			
			printk("spi send timeout!\n");
			BUG_ON(1);
		}
	}
}

enum spi_transfer_mode {
	tx_mode,
	rx_mode,
	rt_mode,
};

static int sprd_spi_transfer_full_duplex(struct spi_device *spi_dev,
	struct spi_transfer *trans_node)
{
	int i;
	u32 reg_val;
	u32 bytes_per_word, block_num;
	u32 send_data_msk;
	u8 *tx_u8_p, *rx_u8_p;
	u16 *tx_u16_p, *rx_u16_p;
	u32 *tx_u32_p, *rx_u32_p;
	const void *src_buf;
	void *dst_buf;
	u32 transfer_mode;
	struct sprd_spi_devdata *spi_chip;

	spi_chip = spi_master_get_devdata(spi_dev->master);
#if 0
	if (unlikely(trans_node->bits_per_word != spi_dev->bits_per_word)) {
		printk("%s %d\n", __func__, __LINE__);
		reg_val = __raw_readl(spi_chip->reg_base + SPI_CTL0);
		reg_val &= ~(0x1f << 2);
		if (trans_node->bits_per_word != MAX_BITS_PER_WORD) {
			reg_val |= trans_node->bits_per_word << 2;
			printk("%s %d\n", __func__, __LINE__);
		}
		__raw_writel(reg_val, spi_chip->reg_base + SPI_CTL0);
	}
	
	bytes_per_word = ALIGN_UP(trans_node->bits_per_word, 8) >> 3;
#else
	bytes_per_word = ALIGN_UP(spi_dev->bits_per_word, 8) >> 3;
#endif
	
	writel(0x144, SPRD_PIN_BASE + 0x164);
	writel(0x18a, SPRD_PIN_BASE + 0x168);
	writel(0x101, SPRD_PIN_BASE + 0x16c);
	writel(0x101, SPRD_PIN_BASE + 0x170);
	
	src_buf = trans_node->tx_buf;

	
	dst_buf = trans_node->rx_buf;

	send_data_msk = 0xffffffff;

	if (src_buf && dst_buf) {
		transfer_mode = rt_mode;
	} else {
		if (dst_buf) {
			transfer_mode = rx_mode;
			send_data_msk = 0x0;
			src_buf = trans_node->rx_buf;
		} else {
			transfer_mode = tx_mode;
		}
	}

	sprd_spi_wait_for_idle(spi_chip->reg_base);

	reg_val = __raw_readl(spi_chip->reg_base + SPI_CTL1);
	reg_val &= ~(0x3 << 12);
	
	if (transfer_mode == tx_mode) {
		reg_val |= SPI_TX_MODE;
	} else {
		reg_val |= SPI_RX_MODE |SPI_TX_MODE;
	}
	writel(reg_val, spi_chip->reg_base + SPI_CTL1);
	
	writel(0x1, spi_chip->reg_base + SPI_FIFO_RST); 
	writel(0x0, spi_chip->reg_base + SPI_FIFO_RST);

	switch (bytes_per_word) {
	case 1:
		tx_u8_p = (u8 *)src_buf;
		rx_u8_p = (u8 *)dst_buf;
		block_num = trans_node->len;

		for (; block_num >= SPRD_SPI_FIFO_SIZE; block_num -= SPRD_SPI_FIFO_SIZE)
		{
			for (i = 0; i < SPRD_SPI_FIFO_SIZE; i++, tx_u8_p++)
			{
				__raw_writeb(*tx_u8_p & send_data_msk,
					spi_chip->reg_base + SPI_TXD);
				
			}

			sprd_spi_wait_for_idle(spi_chip->reg_base);

			if (transfer_mode == tx_mode)
				continue;

			for (i = 0; i < SPRD_SPI_FIFO_SIZE; i++, rx_u8_p++)
			{
				do{
					reg_val = __raw_readb(spi_chip->reg_base + SPI_STS2);
				}while(reg_val & SPI_RX_FIFO_REALLY_EMPTY); 
				*rx_u8_p = __raw_readb(spi_chip->reg_base + SPI_TXD);
			}
		}

		for (i = 0; i < block_num; i++, tx_u8_p++)
		{
			__raw_writeb(*tx_u8_p & send_data_msk, spi_chip->reg_base + SPI_TXD);
			
		}

		sprd_spi_wait_for_idle(spi_chip->reg_base);

		if (transfer_mode == tx_mode)
			break;

		for (i = 0; i < block_num; i++, rx_u8_p++)
		{
			do{
				reg_val = __raw_readb(spi_chip->reg_base + SPI_STS2);
			}while(reg_val & SPI_RX_FIFO_REALLY_EMPTY); 
			*rx_u8_p = __raw_readb(spi_chip->reg_base + SPI_TXD);
		}

		break;
	case 2:
		tx_u16_p = (u16 *)src_buf;
		rx_u16_p = (u16 *)dst_buf;
		block_num = trans_node->len >> 1;

		for (;block_num >= SPRD_SPI_FIFO_SIZE; block_num -= SPRD_SPI_FIFO_SIZE)
		{
			for (i =0; i < SPRD_SPI_FIFO_SIZE; i++, tx_u16_p++)
			{
				__raw_writew(*tx_u16_p & send_data_msk,
					spi_chip->reg_base + SPI_TXD);
			}

			sprd_spi_wait_for_idle(spi_chip->reg_base);

			if (transfer_mode == tx_mode)
				continue;

			for (i = 0; i < SPRD_SPI_FIFO_SIZE; i++, rx_u16_p++)
			{
				*rx_u16_p = __raw_readw(spi_chip->reg_base + SPI_TXD);
			}
		}

		for (i = 0; i < block_num; i++, tx_u16_p++)
		{
			__raw_writew(*tx_u16_p & send_data_msk, spi_chip->reg_base + SPI_TXD);
		}

		sprd_spi_wait_for_idle(spi_chip->reg_base);

		if (transfer_mode == tx_mode)
			break;

		for (i = 0; i < block_num; i++, rx_u16_p++)
		{
			*rx_u16_p = __raw_readw(spi_chip->reg_base + SPI_TXD);
		}
		break;

	case 4:
		tx_u32_p = (u32 *)src_buf;
		rx_u32_p = (u32 *)dst_buf;
		block_num = trans_node->len >> 2;

		for (;block_num >= SPRD_SPI_FIFO_SIZE; block_num -= SPRD_SPI_FIFO_SIZE)
		{
			for (i = 0; i < SPRD_SPI_FIFO_SIZE; i++, tx_u32_p++)
			{
				__raw_writel(*tx_u32_p & send_data_msk,
					spi_chip->reg_base + SPI_TXD);
			}

			sprd_spi_wait_for_idle(spi_chip->reg_base);

			if (transfer_mode == tx_mode)
				continue;

			for (i = 0; i < SPRD_SPI_FIFO_SIZE; i++, rx_u32_p++)
			{
				*rx_u32_p = __raw_readl(spi_chip->reg_base + SPI_TXD);
			}
		}

		for (i = 0; i < block_num; i++, tx_u32_p++)
		{
			__raw_writel(*tx_u32_p & send_data_msk, spi_chip->reg_base + SPI_TXD);
		}

		sprd_spi_wait_for_idle(spi_chip->reg_base);

		if (transfer_mode == tx_mode)
			break;

		for (i = 0; i < block_num; i++, rx_u32_p++)
		{
			*rx_u32_p = __raw_readl(spi_chip->reg_base + SPI_TXD);
		}

		break;
	default:
		
		break;
	}

	reg_val = __raw_readl(spi_chip->reg_base + SPI_CTL1);
	reg_val &= ~(0x3 << 12);
	writel(reg_val, spi_chip->reg_base + SPI_CTL1);

	return trans_node->len;
}

static void  sprd_spi_transfer_work(struct work_struct *work)
{
	int ret;
	int reg_val ;
	struct sprd_spi_devdata *spi_chip;
	struct spi_message *spi_msg;
	struct spi_transfer *transfer_node;
	unsigned long flags;
	struct spi_device *spi_dev;

	spi_chip = container_of(work, struct sprd_spi_devdata, work);

	clk_enable(spi_chip->clk);

	
	spin_lock_irqsave(&spi_chip->lock, flags);

	while (!list_empty(&spi_chip->msg_queue))
	{
		spi_msg = container_of(spi_chip->msg_queue.next,
			struct spi_message, queue);
		list_del_init(&spi_msg->queue);
		spin_unlock_irqrestore(&spi_chip->lock, flags);

		spi_dev = spi_msg->spi;
	
	if (spi_dev->chip_select < SPRD_SPI_CHIP_CS_NUM) {
		reg_val = __raw_readl(spi_chip->reg_base + SPI_CTL0);
		reg_val &= ~(0x1 << (spi_dev->chip_select + 8));
		__raw_writel(reg_val, spi_chip->reg_base + SPI_CTL0);
	} else {
		
	}

		list_for_each_entry(transfer_node,
			&spi_msg->transfers, transfer_list)
		{
			if (transfer_node->tx_buf || transfer_node->rx_buf) {
				ret = sprd_spi_transfer_full_duplex(spi_msg->spi,
					transfer_node);
			}
		}

	if (spi_dev->chip_select < SPRD_SPI_CHIP_CS_NUM) {
		reg_val = __raw_readl(spi_chip->reg_base + SPI_CTL0);
		reg_val |= 0xf << 8;
		__raw_writel(reg_val, spi_chip->reg_base + SPI_CTL0);
	} else {
		
	}

		if (spi_msg->complete){
			spi_msg->status = 0;
			spi_msg->complete(spi_msg->context);
		}

		spin_lock_irqsave(&spi_chip->lock, flags);
	}

	clk_disable(spi_chip->clk);

	spin_unlock_irqrestore(&spi_chip->lock, flags);
}

static int sprd_spi_transfer(struct spi_device *spi_dev, struct spi_message *msg)
{
	struct sprd_spi_devdata *spi_chip;
	unsigned long flags;

	spi_chip = spi_master_get_devdata(spi_dev->master);

	spin_lock_irqsave(&spi_chip->lock, flags);

	list_add_tail(&msg->queue, &spi_chip->msg_queue);

	queue_work(spi_chip->work_queue, &spi_chip->work);

	spin_unlock_irqrestore(&spi_chip->lock, flags);

	return 0;
}

static void __spi_backup_config(struct sprd_spi_devdata *spi_chip)
{
	u32 reg_base;

	reg_base = spi_chip->reg_base;

	spi_chip->reg_backup[0] = __raw_readl(reg_base + SPI_CLKD);
	spi_chip->reg_backup[1] = __raw_readl(reg_base + SPI_CTL0);
	spi_chip->reg_backup[2] = __raw_readl(reg_base + SPI_CTL1);
	spi_chip->reg_backup[3] = __raw_readl(reg_base + SPI_CTL2);
	spi_chip->reg_backup[4] = __raw_readl(reg_base + SPI_CTL3);
	spi_chip->reg_backup[5] = __raw_readl(reg_base + SPI_CTL4);
	spi_chip->reg_backup[6] = __raw_readl(reg_base + SPI_CTL5);
	spi_chip->reg_backup[7] = __raw_readl(reg_base + SPI_INT_EN);
	spi_chip->reg_backup[8] = __raw_readl(reg_base + SPI_DSP_WAIT);
	spi_chip->reg_backup[9] = __raw_readl(reg_base + SPI_CTL6);
	spi_chip->reg_backup[10] = __raw_readl(reg_base + SPI_CTL7);
}

static void __spi_restore_config(struct sprd_spi_devdata *spi_chip)
{
	u32 reg_base;

	reg_base = spi_chip->reg_base;
	__raw_writel(spi_chip->reg_backup[0], reg_base + SPI_CLKD);
	__raw_writel(spi_chip->reg_backup[1], reg_base + SPI_CTL0);
	__raw_writel(spi_chip->reg_backup[2], reg_base + SPI_CTL1);
	__raw_writel(spi_chip->reg_backup[3], reg_base + SPI_CTL2);
	__raw_writel(spi_chip->reg_backup[4], reg_base + SPI_CTL3);
	__raw_writel(spi_chip->reg_backup[5], reg_base + SPI_CTL4);
	__raw_writel(spi_chip->reg_backup[6], reg_base + SPI_CTL5);
	__raw_writel(spi_chip->reg_backup[7], reg_base + SPI_INT_EN);
	__raw_writel(spi_chip->reg_backup[8], reg_base + SPI_DSP_WAIT);
	__raw_writel(spi_chip->reg_backup[9], reg_base + SPI_CTL6);
	__raw_writel(spi_chip->reg_backup[10], reg_base + SPI_CTL7);

}


static int sprd_spi_setup(struct spi_device *spi_dev)
{
	int i;
	u32 reg_val;
	u32 spi_work_clk, spi_src_clk, spi_clk_div;
	const char *src_clk_name;
	struct clk *clk_parent;
	struct sprd_spi_devdata *spi_chip;

	spi_chip = spi_master_get_devdata(spi_dev->master);

	

	
	clk_enable(spi_chip->clk);

	
	if (spi_dev->bits_per_word > MAX_BITS_PER_WORD) {
		
	}

	
	if (spi_dev->bits_per_word == MAX_BITS_PER_WORD) {
		reg_val = 0x0 << 2;
	} else {
		reg_val = spi_dev->bits_per_word << 2;
	}

	
	reg_val |= 0xf << 8;

	
	switch (spi_dev->mode & 0x3) {
	case SPI_MODE_0:
		reg_val |= BIT(1);
		break;
	case SPI_MODE_1:
		reg_val |= BIT(0);
		break;
	case SPI_MODE_2:
		reg_val |= BIT(13) | BIT(1);
		break;
	case SPI_MODE_3:
		reg_val |= BIT(13) | BIT(0);
		break;
	default:
		
		break;
	}
	__raw_writel(reg_val, spi_chip->reg_base + SPI_CTL0);
	__raw_writel(0x0, spi_chip->reg_base + SPI_CTL1);
	
	__raw_writel(0x0, spi_chip->reg_base + SPI_CTL2);
	__raw_writel(0x0, spi_chip->reg_base + SPI_CTL4);
	__raw_writel(0x0, spi_chip->reg_base + SPI_CTL5);
	__raw_writel(0x0, spi_chip->reg_base + SPI_INT_EN);
	
	__raw_writel(0x1, spi_chip->reg_base + SPI_FIFO_RST);
	for (i = 0; i < 0x20; i++);
	__raw_writel(0x0, spi_chip->reg_base + SPI_FIFO_RST);

	
	spi_work_clk = spi_dev->max_speed_hz << 1;

	if (spi_work_clk > MHz(192, 0))
		return -EINVAL;

	if (spi_work_clk <= MHz(26, 0)) {
		src_clk_name = "ext_26m";
		spi_src_clk = MHz(26, 0);
	} else {
		if (spi_work_clk <= MHz(96, 0)) {
			src_clk_name = "clk_96m";
			spi_src_clk = MHz(96, 0);
		} else {
			if (spi_work_clk <= MHz(153, 6)) {
				src_clk_name = "clk_153m6";
				spi_src_clk = MHz(153, 6);
			} else {
				src_clk_name = "clk_192m";
				spi_src_clk = MHz(192, 0);
			}
		}
	}
	clk_disable(spi_chip->clk);

	clk_parent = clk_get(&(spi_dev->master->dev), src_clk_name);
	if (IS_ERR(clk_parent)) {
		printk("Can't get the clock source: %s\n", src_clk_name);
		return -EINVAL;
	}

	clk_set_parent(spi_chip->clk, clk_parent);
	clk_set_rate(spi_chip->clk, spi_src_clk);
	clk_enable(spi_chip->clk);

	spi_clk_div = spi_src_clk / (spi_dev->max_speed_hz << 1) - 1;

	__raw_writel(spi_clk_div, spi_chip->reg_base + SPI_CLKD);

	
	msleep(5);
	__spi_backup_config(spi_chip);
	spi_chip->parent_clk = clk_parent;
	spi_chip->src_clk = spi_src_clk;

	spi_chip->is_active = true;

	
	clk_disable(spi_chip->clk);

	return 0;
}

static void sprd_spi_cleanup(struct spi_device *spi)
{
	struct sprd_spi_devdata *spi_chip;

	spi_chip = spi_master_get_devdata(spi->master);

	clk_force_disable(spi_chip->clk);
}

static int __init sprd_spi_probe(struct platform_device *pdev)
{
	int ret;
	int irq_num;
	struct resource *regs;
	struct spi_master *master;
	struct sprd_spi_devdata *spi_chip;

	printk("sprd spi probe\n");
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENXIO;

	irq_num = platform_get_irq(pdev, 0);
	if (irq_num < 0)
		return irq_num;

	master = spi_alloc_master(&pdev->dev, sizeof (*spi_chip));
	if (!master) {
		return -ENOMEM;
	}

	spi_chip = spi_master_get_devdata(master);

	
	switch (pdev->id) {
		case sprd_spi_0:
			spi_chip->clk = clk_get(&pdev->dev, "clk_spi0");
			break;
		case sprd_spi_1:
			spi_chip->clk = clk_get(&pdev->dev, "clk_spi1");
			break;
		case sprd_spi_2:
			spi_chip->clk = clk_get(&pdev->dev, "clk_spi2");
			break;
		default:
			ret = -EINVAL;
			goto err_exit;
	}
	if (IS_ERR(spi_chip->clk)) {
		ret = -ENXIO;
		goto err_exit;
	}
	spi_chip->reg_base = (void __iomem *)regs->start;
	spi_chip->irq_num  = irq_num;

	master->mode_bits = SPI_CPOL | SPI_CPHA;
	master->bus_num = pdev->id;
	master->num_chipselect = SPRD_SPI_CHIP_CS_NUM;

	master->setup = sprd_spi_setup;
#ifdef SPI_NEW_INTERFACE
	master->prepare_transfer_hardware = xxxxx;
	master->transfer_one_message = xxxx;
	master->unprepare_transfer_hardware = xxxx;
#else
	master->transfer = sprd_spi_transfer;
	master->cleanup = sprd_spi_cleanup;
#endif

	ret = spi_register_master(master);
	if (ret) {
		printk("register spi master %d failed!\n", master->bus_num);
		goto err_exit;
	}

	INIT_LIST_HEAD(&spi_chip->msg_queue);
	spin_lock_init(&spi_chip->lock);
	
	spi_chip->work_queue = create_singlethread_workqueue(pdev->name);
	INIT_WORK(&spi_chip->work, sprd_spi_transfer_work);

	platform_set_drvdata(pdev, master);

	printk("sprd spi probe, end\n");
	return 0;

err_exit:
	spi_master_put(master);
	kfree(master);

	return ret;
}

static int __exit sprd_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct sprd_spi_devdata *spi_chip = spi_master_get_devdata(master);

	destroy_workqueue(spi_chip->work_queue);

	clk_force_disable(spi_chip->clk);

	spi_unregister_master(master);

	spi_master_put(master);

	return 0;
}

#ifdef CONFIG_PM
static int sprd_spi_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct sprd_spi_devdata *spi_chip = spi_master_get_devdata(master);
	if (IS_ERR(spi_chip->clk)) {
		pr_err("can't get spi_clk when suspend()\n");
		return -1;
	}

	if (spi_chip->is_active) {
		clk_force_disable(spi_chip->clk);
	}

	return 0;
}

static int sprd_spi_resume(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);

	struct sprd_spi_devdata *spi_chip = spi_master_get_devdata(master);

	if (spi_chip->is_active) {
		
		clk_set_parent(spi_chip->clk, spi_chip->parent_clk);
		clk_set_rate(spi_chip->clk, spi_chip->src_clk);
		clk_enable(spi_chip->clk);

		__spi_restore_config(spi_chip);

		clk_force_disable(spi_chip->clk);
	}

	return 0;
}

#else
#define	sprd_spi_suspend NULL
#define	sprd_spi_resume NULL
#endif

static struct platform_driver sprd_spi_driver = {
	.driver = {
		.name = "sprd_spi",
		.owner = THIS_MODULE,
	},
	.suspend = sprd_spi_suspend,
	.resume  = sprd_spi_resume,
	.remove  = __exit_p(sprd_spi_remove),
};

static int __init sprd_spi_init(void)
{
	printk("spi master: init\n");
	return platform_driver_probe(&sprd_spi_driver, sprd_spi_probe);
}
module_init(sprd_spi_init);

static void __exit sprd_spi_exit(void)
{
	platform_driver_unregister(&sprd_spi_driver);
}
module_exit(sprd_spi_exit);

MODULE_DESCRIPTION("SpreadTrum SPI(r4p0 version) Controller driver");
MODULE_AUTHOR("Jack.Jiang <Jack.Jiang@spreadtrum.com>");
MODULE_LICENSE("GPL");
