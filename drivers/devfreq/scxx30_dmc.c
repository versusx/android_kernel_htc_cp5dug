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

#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/math64.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/opp.h>
#include <linux/devfreq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>


#ifdef CONFIG_BUS_MONITOR
#include <mach/bm_sc8830.h>
#endif

# if 0
#define CP2AP_INT_CTRL		(SPRD_IPI_BASE + 0x04)
#define CP0_AP_MCU_IRQ1_CLR	BIT(2)
#define CP1_AP_MCU_IRQ1_CLR	BIT(6)
#define CPT_SHARE_MEM		(CPT_RING_ADDR + 0x880)
#define CPW_SHARE_MEM		(CPW_RING_ADDR + 0x880)
#endif

#define DMC_FREQ_NORMAL_SCENE			0x0 
#define DMC_FREQ_EARLYSUSPEND_SCENE			0x1 
static u32 scxx30_dmc_scene = DMC_FREQ_NORMAL_SCENE;

static DEFINE_SPINLOCK(min_freq_cnt_lock);
extern u32 emc_clk_set(u32 new_clk, u32 sene);
extern u32 emc_clk_get(void);

static int boot_mode = 0;
extern int board_mfg_mode(void);

enum scxx30_dmc_type {
	TYPE_DMC_SCXX30 ,
};

enum dmcclk_level_idx {
	LV_0 = 0,
	LV_1,
	LV_2,
	LV_3,
	LV_4,
	_LV_END
};

int scxx30_stress_dfs_flags = 0;
static struct wake_lock fix_dfs_wake_lock;
struct dmc_opp_table {
	unsigned int idx;
	unsigned long clk;  
	unsigned long volt; 
	unsigned long bandwidth; 
};

static struct dmc_opp_table scxx30_dmcclk_table[] = {
	{LV_0, 500000, 1200000, 4256},
	{LV_1, 384000, 1200000, 3200},
	{LV_2, 200000, 1200000, 2656},
	{0, 0, 0},
};

struct dmcfreq_data {
	enum scxx30_dmc_type type;
	struct device *dev;
	struct devfreq *devfreq;
	bool disabled;
	struct opp *curr_opp;
	void __iomem *cpt_share_mem_base;
	void __iomem *cpw_share_mem_base;
	struct notifier_block pm_notifier;
	unsigned long last_jiffies;
	unsigned long quirk_jiffies;
	spinlock_t lock;
};

static struct dmcfreq_data *g_data = NULL;

#define SCXX30_LV_NUM (LV_3)
#define SCXX30_MAX_FREQ (500000)
#define SCXX30_MIN_FREQ (200000)
#define SCXX30_INITIAL_FREQ SCXX30_MAX_FREQ
#define MIN_FREQ_CNT   (1)
#define SCXX30_POLLING_MS (100)
#define BOOT_TIME	(80*HZ)
static unsigned long boot_done;
static unsigned int min_freq_cnt = 0;
static u32 min_freq_skip = 0;

int devfreq_request_ignore(void)
{
	if((emc_clk_get()*1000) > SCXX30_MIN_FREQ )
		return 1;
	else
		return 0;
}

void devfreq_min_freq_cnt_reset(unsigned int cnt, unsigned int skip)
{

	if(cnt >= MIN_FREQ_CNT){
		cnt = MIN_FREQ_CNT;
	}
	spin_lock(&min_freq_cnt_lock);
	min_freq_cnt = cnt;
	min_freq_skip = skip;
	spin_unlock(&min_freq_cnt_lock);
	
}


static LIST_HEAD(devfreq_dbs_handlers);
static DEFINE_MUTEX(devfreq_dbs_lock);

int devfreq_notifier_register(struct devfreq_dbs *handler)
{

	struct list_head *pos;
	struct devfreq_dbs *e;

	mutex_lock(&devfreq_dbs_lock);
	list_for_each(pos, &devfreq_dbs_handlers) {
		e = list_entry(pos, struct devfreq_dbs, link);
		if(e == handler){
			printk("***** %s, %pf already exsited ****\n",
					__func__, e->devfreq_notifier);
			return -1;
		}
	}
	list_for_each(pos, &devfreq_dbs_handlers) {
		struct devfreq_dbs *e;
		e = list_entry(pos, struct devfreq_dbs, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	mutex_unlock(&devfreq_dbs_lock);

	return 0;
}
EXPORT_SYMBOL(devfreq_notifier_register);

int devfreq_notifier_unregister(struct devfreq_dbs *handler)
{
	mutex_lock(&devfreq_dbs_lock);
	list_del(&handler->link);
	mutex_unlock(&devfreq_dbs_lock);

	return 0;
}
EXPORT_SYMBOL(devfreq_notifier_unregister);

#if 0
static unsigned int devfreq_change_notification(unsigned int state)
{
	struct devfreq_dbs *pos;
	int forbidden;

	mutex_lock(&devfreq_dbs_lock);
	list_for_each_entry(pos, &devfreq_dbs_handlers, link) {
		if (pos->devfreq_notifier != NULL) {
			pr_debug("%s: state:%u, calling %pf\n", __func__, state, pos->devfreq_notifier);
			forbidden = pos->devfreq_notifier(pos, state);
			if(forbidden){
				mutex_unlock(&devfreq_dbs_lock);
				return forbidden;
			}
			pr_debug("%s: calling %pf done \n",
						__func__, pos->devfreq_notifier );
		}
	}
	mutex_unlock(&devfreq_dbs_lock);
	return 0;
}
#endif

static void inline scxx30_set_min(struct dmcfreq_data *data);
static void inline scxx30_set_max(struct dmcfreq_data *data);
static int scxx30_switch_disable = 0;
static int scxx30_count = 0;
static unsigned long orig_freq = SCXX30_MIN_FREQ / 1000;
void scxx30_fix_max_freq(int enable)
{
	if(g_data) {
		spin_lock(&g_data->lock);
		if(enable) {
			scxx30_count++;
			BUG_ON(scxx30_count >=2 );
			scxx30_switch_disable = 1;
			scxx30_set_max(g_data);
		} else {
			scxx30_count--;
			BUG_ON(scxx30_count < 0);
			if(scxx30_count == 0) {
				scxx30_switch_disable = 0;

				if(DMC_FREQ_EARLYSUSPEND_SCENE == scxx30_dmc_scene) {
					scxx30_set_min(g_data);
					orig_freq = SCXX30_MIN_FREQ / 1000;
				}
			}
		}

		spin_unlock(&g_data->lock);
	}
}


static unsigned long scxx30_max_freq(struct dmcfreq_data *data)
{
	switch (data->type) {
	case TYPE_DMC_SCXX30:
		return SCXX30_MAX_FREQ;
	default:
		pr_err("Cannot determine the device id %d\n", data->type);
		return (-EINVAL);
	}
}

static unsigned long scxx30_min_freq(struct dmcfreq_data *data)
{
	switch (data->type) {
	case TYPE_DMC_SCXX30:
		return SCXX30_MIN_FREQ;
	default:
		pr_err("Cannot determine the device id %d\n", data->type);
		return (-EINVAL);
	}
}

static int scxx30_convert_bw_to_freq(int bw)
{
	int freq;

	freq = 0;
	
	freq = bw/2;

	return freq;
}

static int boot_done_init = 0;
static int scxx30_dmc_target(struct device *dev, unsigned long *_freq,
				u32 flags)
{
	int cnt, err = 0;
	struct platform_device *pdev = container_of(dev, struct platform_device,
							dev);
	struct dmcfreq_data *data = platform_get_drvdata(pdev);
	struct opp *opp = devfreq_recommended_opp(dev, _freq, flags);
	unsigned long freq = opp_get_freq(opp);
	unsigned long old_freq = emc_clk_get()*1000 ;

	if(time_before(jiffies, boot_done)){
		return 0;
	} else {
		
		if(4 != boot_mode) {
			if(!boot_done_init) {
				wake_unlock(&fix_dfs_wake_lock);
				boot_done_init = 1;
			}
		}
	}

	if (IS_ERR(opp))
		return PTR_ERR(opp);

	pr_debug("*** %s, old_freq:%luKHz, freq:%luKHz ***\n", __func__, old_freq, freq);

	if (old_freq == freq)
		return 0;

	
	printk("*** %s, data->quirk_jiffies:%u ***\n", __func__, data->quirk_jiffies);
	if(freq==scxx30_min_freq(data) &&
		time_in_range(jiffies, data->quirk_jiffies,  data->quirk_jiffies+msecs_to_jiffies(SCXX30_POLLING_MS)/5) ){
		printk("*** %s, return ***\n", __func__ );
		return 0;
	}
	spin_lock(&min_freq_cnt_lock);
	cnt = min_freq_cnt;
	spin_unlock(&min_freq_cnt_lock);

	if (scxx30_min_freq(data)==freq && cnt<MIN_FREQ_CNT){
		spin_lock(&min_freq_cnt_lock);
		min_freq_cnt++;
		spin_unlock(&min_freq_cnt_lock);
		freq += 1;
		opp = devfreq_recommended_opp(dev, &freq, flags);
		freq = opp_get_freq(opp);
	} else {
		spin_lock(&min_freq_cnt_lock);
		
			min_freq_cnt = 0;
		
		spin_unlock(&min_freq_cnt_lock);
#if 0
		if( freq == scxx30_max_freq(data) ){
			spin_lock(&min_freq_cnt_lock);
			min_freq_skip = 0;
			min_freq_cnt = 0;
			spin_unlock(&min_freq_cnt_lock);
		}
#endif
	}
	pr_debug("*** %s, freq:%lu, min_freq_cnt:%u, min_freq_skip:%u ***\n",
				__func__, freq, min_freq_cnt, min_freq_skip );

#if 0
	if(data->cpt_share_mem_base){
		cp_req = readb(data->cpt_share_mem_base);
	}
	if(cp_req){
		pr_debug("*** %s, cpt:cp_req:%u ***\n", __func__, cp_req);
		return 0;
	}else{
		if(data->cpw_share_mem_base){
			cp_req = readb(data->cpw_share_mem_base);
		}
		pr_debug("*** %s, cpw:cp_req:%u ***\n", __func__, cp_req);
		if(cp_req)
			return 0;
	}
#endif

	dev_dbg(dev, "targetting %lukHz %luuV\n", freq, opp_get_voltage(opp));
	freq = freq/1000; 

	
	

	spin_lock(&data->lock);
	if(data->disabled) {
		goto out;
	}

	if(scxx30_switch_disable)
		goto out;

	if(scxx30_stress_dfs_flags)
		goto out;

	err = emc_clk_set(freq, 1);
	data->curr_opp = opp;

out:
	spin_unlock(&data->lock);
	
	pr_debug("*** %s, old_freq:%luKHz, set emc done, err:%d, current freq:%uKHz ***\n",
			__func__, old_freq, err, emc_clk_get()*1000 );
	return err;
}

static int scxx30_dmc_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
#ifdef CONFIG_BUS_MONITOR
	struct dmcfreq_data *data = dev_get_drvdata(dev);
	u32 total_bw;
	u64 trans_bw;
	u32 interval;
	dmc_mon_cnt_stop();
	trans_bw = (u64)dmc_mon_cnt_bw(); 
	dmc_mon_cnt_clr();
	dmc_mon_cnt_start();
	interval = jiffies - data->last_jiffies;
	data->last_jiffies = jiffies;

	if(min_freq_skip)
		data->quirk_jiffies = jiffies;

	stat->current_frequency = emc_clk_get() * 1000; 
	
	total_bw = (stat->current_frequency)*8; 
	pr_debug("*** %s, trans_bw:%lluB, curr freq:%lu, total_bw:%uKB ***\n",
			__func__, trans_bw, stat->current_frequency, total_bw);

	if(interval){
		stat->busy_time = (u32)div_u64(trans_bw*HZ, interval); 
		stat->total_time = total_bw*350 ;   
	}else{
		stat->busy_time = (u32)div_u64(trans_bw*HZ, 1); 
		stat->total_time = total_bw*350 ;   
		
		
	}
	pr_debug("*** %s, interval:%u, busy_time:%lu, totoal_time:%lu ***\n",
				__func__, interval, stat->busy_time, stat->total_time );
#else
	stat->busy_time = 0 ;
	stat->total_time = 0 ;
#endif
	return 0;
}


static void scxx30_dmc_exit(struct device *dev)
{
	struct dmcfreq_data *data = dev_get_drvdata(dev);

	devfreq_unregister_opp_notifier(dev, data->devfreq);

	return;
}

static struct devfreq_dev_profile scxx30_dmcfreq_profile = {
	.initial_freq	= SCXX30_INITIAL_FREQ,
	.polling_ms	= SCXX30_POLLING_MS,
	.target		= scxx30_dmc_target,
	.get_dev_status	= scxx30_dmc_get_dev_status,
	.exit		= scxx30_dmc_exit,
};

static int scxx30_init_tables(struct dmcfreq_data *data)
{
	int i, err;

	switch (data->type) {
	case TYPE_DMC_SCXX30:
		for (i = LV_0; i < SCXX30_LV_NUM; i++) {
			err = opp_add(data->dev, scxx30_dmcclk_table[i].clk,
					scxx30_dmcclk_table[i].volt);
			if (err) {
				dev_err(data->dev, "Cannot add opp entries.\n");
				return err;
			}
		}
		break;
	default:
		dev_err(data->dev, "Cannot determine the device id %d\n", data->type);
		err = -EINVAL;
	}

	return err;
}

static int scxx30_dmcfreq_pm_notifier(struct notifier_block *this,
		unsigned long event, void *ptr)
{

	switch (event) {
#if 0
	case PM_SUSPEND_PREPARE:
		spin_lock(&data->lock);
		data->disabled = true;
		emc_clk_set(200, 1);
		spin_unlock(&data->lock);
		return NOTIFY_OK;

	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		
		spin_lock(&data->lock);
		data->disabled = false;
		spin_unlock(&data->lock);
		return NOTIFY_OK;
#endif
	}

	return NOTIFY_DONE;
}

static void inline scxx30_set_max(struct dmcfreq_data *data)
{
	unsigned long max;
	

	
	max = scxx30_max_freq(data);
	emc_clk_set(max/1000, 1);
	

}

static void inline scxx30_set_min(struct dmcfreq_data *data)
{
	unsigned long min;
	

	
	min = scxx30_min_freq(data);
	emc_clk_set(min/1000, 1);
	

}

#if 0
static irqreturn_t scxx30_cp0_irq_handler(int irq, void *data)
{
	struct dmcfreq_data *usr = (struct dmcfreq_data *)data;
	scxx30_set_max(usr);
	__raw_writel(CP0_AP_MCU_IRQ1_CLR, CP2AP_INT_CTRL);

	return IRQ_HANDLED;
}

static irqreturn_t scxx30_cp1_irq_handler(int irq, void *data)
{

	struct dmcfreq_data *usr = (struct dmcfreq_data *)data;
	scxx30_set_max(usr);
	__raw_writel(CP1_AP_MCU_IRQ1_CLR, CP2AP_INT_CTRL);

	return IRQ_HANDLED;
}
#endif

static __devinit int scxx30_dmcfreq_probe(struct platform_device *pdev)
{
	struct dmcfreq_data *data;
	struct opp *opp;
	struct device *dev = &pdev->dev;
	int err = 0;

	data = kzalloc(sizeof(struct dmcfreq_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}

	data->type = pdev->id_entry->driver_data;
	data->pm_notifier.notifier_call = scxx30_dmcfreq_pm_notifier;
	data->dev = dev;
	spin_lock_init(&data->lock);

	switch (data->type) {
	case TYPE_DMC_SCXX30:
		err = scxx30_init_tables(data);
		break;
	default:
		dev_err(dev, "Cannot determine the device id %d\n", data->type);
		err = -EINVAL;
	}
	if (err)
		goto err_opp_add;

	opp = opp_find_freq_floor(dev, &scxx30_dmcfreq_profile.initial_freq);
	if (IS_ERR(opp)) {
		dev_err(dev, "Invalid initial frequency %lu kHz.\n",
		       scxx30_dmcfreq_profile.initial_freq);
		err = PTR_ERR(opp);
		goto err_opp_add;
	}
	data->curr_opp = opp;
	data->last_jiffies = jiffies;
	platform_set_drvdata(pdev, data);
	data->devfreq = devfreq_add_device(dev, &scxx30_dmcfreq_profile,
					   &devfreq_ondemand, scxx30_convert_bw_to_freq);
	if (IS_ERR(data->devfreq)) {
		err = PTR_ERR(data->devfreq);
		dev_err(dev, "Failed to add device\n");
		goto err_opp_add;
	}

	devfreq_register_opp_notifier(dev, data->devfreq);

	data->devfreq->min_freq = scxx30_min_freq(data);
	data->devfreq->max_freq = scxx30_max_freq(data);

	err = register_pm_notifier(&data->pm_notifier);
	if (err) {
		dev_err(dev, "Failed to setup pm notifier\n");
		goto err_devfreq_add;
	}
#ifdef CONFIG_BUS_MONITOR
	dmc_mon_cnt_clr( );
	dmc_mon_cnt_start( );
#endif
	boot_done = jiffies + BOOT_TIME;

#if 0
	
	err = request_irq(IRQ_CP0_MCU1_INT, scxx30_cp0_irq_handler, IRQF_DISABLED, "dfs_cp0_int1", data);
	if (err) {
		printk(KERN_ERR ": failed to cp0 int1 request irq!\n");
		err = -EINVAL;
		goto err_devfreq_add;
	}
	err = request_irq(IRQ_CP1_MCU1_INT, scxx30_cp1_irq_handler, IRQF_DISABLED, "dfs_cp1_int1", data);
	if (err) {
		printk(KERN_ERR ": failed to cp1 int1 request irq!\n");
		err = -EINVAL;
		goto err_cp1_irq;
	}


	data->cpt_share_mem_base = ioremap(CPT_SHARE_MEM, 128);
	if (!data->cpt_share_mem_base){
		printk("*** %s, remap CPT_SHARE_MEM error ***\n", __func__);
		err = -ENOMEM;
		goto err_devfreq_add;
	}

	data->cpw_share_mem_base = ioremap(CPW_SHARE_MEM, 128);
	if (!data->cpw_share_mem_base){
		printk("*** %s, remap CPW_SHARE_MEM error ***\n", __func__);
		err = -ENOMEM;
		goto err_map;
	}
#endif

	g_data = data;
	pr_info(" %s done,  current freq:%lu \n", __func__, opp_get_freq(data->curr_opp));
	return 0;

	iounmap(data->cpt_share_mem_base);
#if 0
err_irq:
	free_irq(IRQ_CP1_MCU1_INT, data);
err_cp1_irq:
	free_irq(IRQ_CP0_MCU1_INT, data);
#endif
err_devfreq_add:

	devfreq_remove_device(data->devfreq);
err_opp_add:
	kfree(data);
	return err;
}

static __devexit int scxx30_dmcfreq_remove(struct platform_device *pdev)
{
	struct dmcfreq_data *data = platform_get_drvdata(pdev);

	unregister_pm_notifier(&data->pm_notifier);
	devfreq_remove_device(data->devfreq);
	kfree(data);
	free_irq(IRQ_CP0_MCU1_INT, data);
	free_irq(IRQ_CP1_MCU1_INT, data);
	iounmap(data->cpt_share_mem_base);

	if(data->cpw_share_mem_base)
		iounmap(data->cpw_share_mem_base);
	return 0;
}


void scxx30_spin_lock(void)
{
	if(g_data) {
		spin_lock(&g_data->lock);
	}
}

void scxx30_spin_unlock(void)
{
	if(g_data) {
		spin_unlock(&g_data->lock);
	}
}

static void scxx30_late_resume(struct early_suspend *h)
{
	if(g_data) {
		spin_lock(&g_data->lock);
		g_data->disabled = false;
		scxx30_dmc_scene = DMC_FREQ_NORMAL_SCENE;
		if(!scxx30_switch_disable && !scxx30_stress_dfs_flags)
			scxx30_set_max(g_data);

		spin_unlock(&g_data->lock);
#ifdef CONFIG_BUS_MONITOR
		dmc_mon_cnt_clr( );
		dmc_mon_cnt_start( );
#endif
	}
}

static void scxx30_early_suspend(struct early_suspend *h)
{
	if(g_data) {
		spin_lock(&g_data->lock);
		g_data->disabled = true;
		scxx30_dmc_scene = DMC_FREQ_EARLYSUSPEND_SCENE;
		printk("scxx30_stress_dfs_flags = %d\n", scxx30_stress_dfs_flags);
		if(!scxx30_switch_disable && !scxx30_stress_dfs_flags)
			scxx30_set_min(g_data);

		spin_unlock(&g_data->lock);
	}
}


static struct early_suspend devfreq_early_suspend_desc = {
        .level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 100,
        .suspend = scxx30_early_suspend,
        .resume = scxx30_late_resume,
};

#if 0
static int scxx30_dmcfreq_resume(struct device *dev)
{
	struct dmcfreq_data *data = dev_get_drvdata(dev);
	spin_lock(&data->lock);
	data->disabled = false;
	spin_unlock(&data->lock);
#ifdef CONFIG_BUS_MONITOR
	dmc_mon_resume();
	dmc_mon_cnt_clr( );
	dmc_mon_cnt_start( );
#endif
	return 0;
}

static const struct dev_pm_ops scxx30_dmcfreq_pm = {
	.resume	= scxx30_dmcfreq_resume,
};
#endif

static int scxx30_dmcfreq_resume(struct device *dev)
{
	struct dmcfreq_data *data = dev_get_drvdata(dev);
	spin_lock(&data->lock);
	if( SCXX30_MIN_FREQ / 1000 != orig_freq && !scxx30_stress_dfs_flags)
		emc_clk_set(orig_freq,1);
	orig_freq = SCXX30_MIN_FREQ / 1000;
	spin_unlock(&data->lock);
	return 0;
}

static int scxx30_dmfreq_suspend(struct device *dev)
{
	struct dmcfreq_data *data = dev_get_drvdata(dev);
	spin_lock(&data->lock);
	orig_freq = emc_clk_get();
	if( SCXX30_MIN_FREQ / 1000 != orig_freq && !scxx30_stress_dfs_flags)
		emc_clk_set(SCXX30_MIN_FREQ / 1000, 1);
	spin_unlock(&data->lock);
	return 0;
}

static const struct dev_pm_ops scxx30_dmcfreq_pm = {
	.resume	= scxx30_dmcfreq_resume,
	.suspend = scxx30_dmfreq_suspend,
};

static const struct platform_device_id scxx30_dmcfreq_id[] = {
	{ "scxx30-dmcfreq", TYPE_DMC_SCXX30 },
	{ },
};

static struct platform_device scxx30_dmcfreq = {
	.name = "scxx30-dmcfreq",
};

static struct platform_driver scxx30_dmcfreq_driver = {
	.probe	= scxx30_dmcfreq_probe,
	.remove	= __devexit_p(scxx30_dmcfreq_remove),
	.id_table = scxx30_dmcfreq_id,
	.driver = {
		.name	= "scxx30_dmcfreq",
		.owner	= THIS_MODULE,
		.pm	= &scxx30_dmcfreq_pm,
	},
};

static int __init scxx30_dmcfreq_init(void)
{
	int err;
	boot_mode = board_mfg_mode();
	err = platform_device_register(&scxx30_dmcfreq);
	if(err){
		pr_err(" register scxx30_dmcfreq failed, err:%d\n", err);
	}
	err = platform_driver_register(&scxx30_dmcfreq_driver);
	if(err){
		pr_err(" register scxx30_dmcfreq_driver failed, err:%d\n", err);
	}

	
	if(4 != boot_mode) {
		wake_lock_init(&fix_dfs_wake_lock, WAKE_LOCK_SUSPEND, "fix_dfs_wake_lock");
		wake_lock(&fix_dfs_wake_lock);
	}

	register_early_suspend(&devfreq_early_suspend_desc);
	return err;
}
late_initcall(scxx30_dmcfreq_init);

static void __exit scxx30_dmcfreq_exit(void)
{
	platform_driver_unregister(&scxx30_dmcfreq_driver);
}
module_exit(scxx30_dmcfreq_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SCxx30 dmcfreq driver with devfreq framework");
