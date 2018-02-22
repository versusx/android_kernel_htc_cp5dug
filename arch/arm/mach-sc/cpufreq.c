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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <linux/thermal.h>
#include <linux/regulator/consumer.h>
#include <asm/system.h>
#include <trace/events/power.h>

#include <mach/hardware.h>
#include <mach/regulator.h>
#include <mach/adi.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <mach/arch_misc.h>
#include <mach/freq_table.h>
#include <mach/perflock.h>


extern int sprd_thm_thermal_table_interrupt_registry(unsigned int curr_level);

#if defined(CONFIG_ARCH_SC8825)

#define MHz                     (1000000)
#define GR_MPLL_REFIN_2M        (2 * MHz)
#define GR_MPLL_REFIN_4M        (4 * MHz)
#define GR_MPLL_REFIN_13M       (13 * MHz)
#define GR_MPLL_REFIN_SHIFT     16
#define GR_MPLL_REFIN_MASK      (0x3)
#define GR_MPLL_N_MASK          (0x7ff)
#define GR_MPLL_MN				(REG_GLB_M_PLL_CTL0)
#define GR_GEN1					(REG_GLB_GEN1)


static void set_mcu_clk_freq(u32 mcu_freq)
{
	u32 val, rate, arm_clk_div, gr_gen1;

	rate = mcu_freq / MHz;
	switch(1000 / rate)
	{
		case 1:
			arm_clk_div = 0;
			break;
		case 2:
			arm_clk_div = 1;
			break;
		default:
			panic("set_mcu_clk_freq fault\n");
			break;
	}
	pr_debug("%s --- before, AHB_ARM_CLK: %08x, rate = %d, div = %d\n",
		__FUNCTION__, __raw_readl(REG_AHB_ARM_CLK), rate, arm_clk_div);

	gr_gen1 =  __raw_readl(GR_GEN1);
	gr_gen1 |= BIT(9);
	__raw_writel(gr_gen1, GR_GEN1);

	val = __raw_readl(REG_AHB_ARM_CLK);
	val &= 0xfffffff8;
	val |= arm_clk_div;
	__raw_writel(val, REG_AHB_ARM_CLK);

	gr_gen1 &= ~BIT(9);
	__raw_writel(gr_gen1, GR_GEN1);

	pr_debug("%s --- after, AHB_ARM_CLK: %08x, rate = %d, div = %d\n",
		__FUNCTION__, __raw_readl(REG_AHB_ARM_CLK), rate, arm_clk_div);

	return;
}

static unsigned int get_mcu_clk_freq(void)
{
	u32 mpll_refin, mpll_n, mpll_cfg = 0, rate, val;

	mpll_cfg = __raw_readl(GR_MPLL_MN);

	mpll_refin = (mpll_cfg >> GR_MPLL_REFIN_SHIFT) & GR_MPLL_REFIN_MASK;
	switch(mpll_refin){
		case 0:
			mpll_refin = GR_MPLL_REFIN_2M;
			break;
		case 1:
		case 2:
			mpll_refin = GR_MPLL_REFIN_4M;
			break;
		case 3:
			mpll_refin = GR_MPLL_REFIN_13M;
			break;
		default:
			pr_err("%s --- ERROR mpll_refin: %d\n", __FUNCTION__, mpll_refin);
	}
	mpll_n = mpll_cfg & GR_MPLL_N_MASK;
	rate = mpll_refin * mpll_n;

	
	val = __raw_readl(REG_AHB_ARM_CLK) & 0x7;
	val += 1;
	return rate / val;
}

#endif


#define FREQ_TABLE_SIZE 	10

struct cpufreq_conf {
	struct clk 					*clk;
	struct clk 					*mpllclk;
	struct clk 					*tdpllclk;
	struct regulator 				*regulator;
	struct thermal_cooling_device	*cdev;
	int 							cooling_state;
	unsigned int					limited_max_freq;
	unsigned int					orignal_freq;
	struct cpufreq_frequency_table			*freq_tbl;
	unsigned int					*vddarm_mv;
};

struct cpufreq_status {
	unsigned int	percpu_target[CONFIG_NR_CPUS];
	int		is_suspend;
};

struct cpufreq_table_data {
	struct cpufreq_frequency_table 		freq_tbl[FREQ_TABLE_SIZE];
	unsigned int				vddarm_mv[FREQ_TABLE_SIZE];
};

static struct cpufreq_table_data sc8825_cpufreq_table_data = {
	.freq_tbl =	{
		{0, 1000000},
		{1, 500000},
		{2, CPUFREQ_TABLE_END}
	},
	.vddarm_mv = {
		0
	},
};

struct cpufreq_conf sc8825_cpufreq_conf = {
	.clk = NULL,
	.regulator = NULL,
	.freq_tbl = sc8825_cpufreq_table_data.freq_tbl,
	.vddarm_mv = sc8825_cpufreq_table_data.vddarm_mv,
};

static unsigned int shark_top_frequency;
#define SHARK_TDPLL_FREQUENCY  (768000)
static unsigned int shark_limited_max_frequency;
#define SHARK_FAKE_FREQUENCY   (300000)

#if defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
static struct cpufreq_table_data sc8830_cpufreq_table_data_cs = {
	.freq_tbl = {
		{0, FREQ_HIGHEST},
		{1, FREQ_HIGH},
		{2, SHARK_TDPLL_FREQUENCY},
		{3, FREQ_LOW},
		{4, CPUFREQ_TABLE_END},
	},
	.vddarm_mv = {
		1300000,
		1200000,
		1150000,
		1100000,
		1000000,
	},
};
#else
static struct cpufreq_table_data sc8830_cpufreq_table_data_cs = {
	.freq_tbl = {
		
		{0, FREQ_HIGH},
		{1, SHARK_TDPLL_FREQUENCY},
		{2, FREQ_LOW},
		{3, CPUFREQ_TABLE_END},
	},
	.vddarm_mv = {
		
		1200000,
		1150000,
		1100000,
		1000000,
	},
};
#endif
static struct cpufreq_table_data sc8830_cpufreq_table_data_es = {
	.freq_tbl = {
		{0, FREQ_HIGH},
		{1, CPUFREQ_TABLE_END},
	},
	.vddarm_mv = {
		1150000,
		1000000,
	},
};

struct cpufreq_conf sc8830_cpufreq_conf = {
	.clk = NULL,
	.mpllclk = NULL,
	.tdpllclk = NULL,
	.regulator = NULL,
	.cdev = NULL,
	.freq_tbl = NULL,
	.vddarm_mv = NULL,
};

#define TRANSITION_LATENCY	(25 * 1000)

struct cpufreq_conf *sprd_cpufreq_conf = NULL;
struct cpufreq_status sprd_cpufreq_status = {{0}, 0};
struct cpufreq_freqs global_freqs;
static DEFINE_MUTEX(freq_lock);

#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
static unsigned int bottom_freq;
struct delayed_work plugin_work;
struct unplug_work_info {
	unsigned int cpuid;
	int need_unplug;
	struct delayed_work unplug_work;
};
static int unplug_delay = 500;
static int plugin_delay = 100;
static int enabled_dhp = 1;
static int temp_limit_cpus = 4;
static int perf_limit_cpus = 0;

static struct unplug_work_info uwi[4];
static void sprd_unplug_one_cpu(struct work_struct *work)
{
	struct unplug_work_info *puwi = container_of(work,
		struct unplug_work_info, unplug_work.work);
	if ((puwi->need_unplug) &&(num_online_cpus() > perf_limit_cpus)) {
		if (enabled_dhp) {
			pr_info("---###--- we gonna unplug cpu%d\n", puwi->cpuid);
			cpu_down(puwi->cpuid);
		}
	} else
		pr_debug("---###--- ok do nonthing for cpu%d ###\n", puwi->cpuid);


	pr_debug("### puwi->cpuid=%d ###\n", puwi->cpuid);
	return;
}

static void sprd_plugin_one_cpu(struct work_struct *work)
{
	int cpuid, i;
	unsigned int min_speed = shark_top_frequency;

	for_each_online_cpu(i) {
		min_speed = min(min_speed, sprd_cpufreq_status.percpu_target[i]);
	}

	if ((min_speed == shark_top_frequency) && (num_online_cpus() < nr_cpu_ids)) {
		cpuid = cpumask_next_zero(0, cpu_online_mask);
		if ((enabled_dhp) &&(num_online_cpus() < temp_limit_cpus)){
			pr_info("----@@@---- we gonna plug cpu%d\n", cpuid);
			cpu_up(cpuid);
		}
	} else
		pr_debug("----@@@---- do nonthing:%x,%x\n",num_online_cpus(),temp_limit_cpus);
	return;
}

static ssize_t show_enabled(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", enabled_dhp);
}

static ssize_t store_enabled(struct kobject *kobj, struct attribute *attr,
			      const char *buf, size_t n)
{
	unsigned long val;
	int cpu;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;

	enabled_dhp = val;
	smp_wmb();
	if (!enabled_dhp) {
		for_each_cpu(cpu, cpu_possible_mask) {
			if (!cpu_online(cpu))
				cpu_up(cpu);
		}
	}
	return n;
}

static ssize_t show_unplug_delay_time
(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", unplug_delay);
}

static ssize_t store_unplug_delay_time(struct kobject *kobj, struct attribute *attr,
			      const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	unplug_delay = val;
	return n;
}

static ssize_t show_plugin_delay_time
(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", plugin_delay);
}

static ssize_t store_plugin_delay_time(struct kobject *kobj, struct attribute *attr,
			      const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	plugin_delay = val;
	return n;
}

static ssize_t show_require_cpu_count
(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", perf_limit_cpus);
}

static ssize_t store_require_cpu_count(struct kobject *kobj, struct attribute *attr,
			      const char *buf, size_t n)
{
	unsigned long val;
	int cpu = 0;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if(val > perf_limit_cpus)
		perf_limit_cpus = val;
	for_each_cpu(cpu, cpu_possible_mask) {
		if (num_online_cpus() < perf_limit_cpus)
			if (!cpu_online(cpu))
				cpu_up(cpu);
	}
	return n;
}

static ssize_t show_release_cpu_count
(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", perf_limit_cpus);
}

static ssize_t store_release_cpu_count(struct kobject *kobj, struct attribute *attr,
			      const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if (val == 0)
		perf_limit_cpus =0;
	return n;
}

define_one_global_rw(enabled);
define_one_global_rw(unplug_delay_time);
define_one_global_rw(plugin_delay_time);
define_one_global_rw(require_cpu_count);
define_one_global_rw(release_cpu_count);

static struct attribute *dynamic_hp_attributes[] = {
	&enabled.attr,
	&unplug_delay_time.attr,
	&plugin_delay_time.attr,
	&require_cpu_count.attr,
	&release_cpu_count.attr,
	NULL
};

static struct attribute_group dynamic_hp_attr_group = {
	.attrs = dynamic_hp_attributes,
	.name = "dynamic_hotplug",
};
static int sprd_auto_hotplug_init(void)
{
	int rc;
	unsigned int i;
	struct unplug_work_info *puwi;
	struct cpufreq_frequency_table *pfreq = sprd_cpufreq_conf->freq_tbl;

	for (i = 0; (pfreq[i].frequency != SHARK_FAKE_FREQUENCY); i++);
	bottom_freq = pfreq[--i].frequency;
	pr_debug("%s bottom_freq=%u\n", __func__, bottom_freq);

	rc = sysfs_create_group(cpufreq_global_kobject, &dynamic_hp_attr_group);
	if (rc)
		goto sysfs_err;

	INIT_DELAYED_WORK(&plugin_work, sprd_plugin_one_cpu);
#if 0
	for_each_possible_cpu(i) {
		puwi = &per_cpu(uwi, i);
		puwi->cpuid = i;
		puwi->need_unplug = 0;
		INIT_DELAYED_WORK(&puwi->unplug_work, sprd_unplug_one_cpu);
	}
#else
	for (i = 0; i < ARRAY_SIZE(uwi); i++) {
		puwi = &uwi[i];
		puwi->cpuid = i;
		puwi->need_unplug = 0;
		INIT_DELAYED_WORK(&puwi->unplug_work, sprd_unplug_one_cpu);
	}
#endif

	return 0;
sysfs_err:
	return rc;
}
static void sprd_auto_hotplug_exit(void)
{
	sysfs_remove_group(cpufreq_global_kobject, &dynamic_hp_attr_group);
	return;
}
#endif

static unsigned int sprd_raw_get_cpufreq(void)
{
#if defined(CONFIG_ARCH_SCX35)
	return clk_get_rate(sprd_cpufreq_conf->clk) / 1000;
#elif defined(CONFIG_ARCH_SC8825)
	return get_mcu_clk_freq() / 1000;
#endif
}

static void sprd_raw_set_cpufreq(int cpu, struct cpufreq_freqs *freq, int index)
{
#if defined(CONFIG_ARCH_SCX35)
	int ret;

#define CPUFREQ_SET_VOLTAGE() \
	do { \
	    ret = regulator_set_voltage(sprd_cpufreq_conf->regulator, \
			sprd_cpufreq_conf->vddarm_mv[index], \
			sprd_cpufreq_conf->vddarm_mv[index]); \
		if (ret) \
			pr_err("cpufreq: Failed to set vdd to %d mv\n", \
				sprd_cpufreq_conf->vddarm_mv[index]); \
	} while (0)
#define CPUFREQ_SET_CLOCK() \
	do { \
		if (freq->new == SHARK_TDPLL_FREQUENCY) { \
			ret = clk_set_parent(sprd_cpufreq_conf->clk, sprd_cpufreq_conf->tdpllclk); \
			if (ret) \
				pr_err("cpufreq: Failed to set cpu parent to tdpll\n"); \
		} else { \
			if (clk_get_parent(sprd_cpufreq_conf->clk) != sprd_cpufreq_conf->tdpllclk) { \
				ret = clk_set_parent(sprd_cpufreq_conf->clk, sprd_cpufreq_conf->tdpllclk); \
				if (ret) \
					pr_err("cpufreq: Failed to set cpu parent to tdpll\n"); \
			} \
			ret = clk_set_rate(sprd_cpufreq_conf->mpllclk, (freq->new * 1000)); \
			if (ret) \
				pr_err("cpufreq: Failed to set mpll rate\n"); \
			ret = clk_set_parent(sprd_cpufreq_conf->clk, sprd_cpufreq_conf->mpllclk); \
			if (ret) \
				pr_err("cpufreq: Failed to set cpu parent to mpll\n"); \
		} \
	} while (0)
	trace_cpu_frequency(freq->new, cpu);
	pr_debug("---### freq->new=%u, freq->old=%u, real=%u, index=%d\n",
		freq->new, freq->old, sprd_raw_get_cpufreq(), index);
	if (freq->new >= sprd_raw_get_cpufreq()) {
		CPUFREQ_SET_VOLTAGE();
		CPUFREQ_SET_CLOCK();
	} else {
		CPUFREQ_SET_CLOCK();
		CPUFREQ_SET_VOLTAGE();
	}

#undef CPUFREQ_SET_VOLTAGE
#undef CPUFREQ_SET_CLOCK

#elif defined(CONFIG_ARCH_SC8825)
	set_mcu_clk_freq(freq->new * 1000);
#endif
	return;
}

static void sprd_real_set_cpufreq(int cpu, unsigned int new_speed, int index)
{
	mutex_lock(&freq_lock);

	if (global_freqs.old == new_speed) {
		mutex_unlock(&freq_lock);
		return;
	}
	pr_debug("$$$ sprd_real_set_cpufreq %u khz on cpu%d\n",
		new_speed, smp_processor_id());
	global_freqs.new = new_speed;

	for_each_online_cpu(global_freqs.cpu)
		cpufreq_notify_transition(&global_freqs, CPUFREQ_PRECHANGE);

	sprd_raw_set_cpufreq(cpu, &global_freqs, index);

	for_each_online_cpu(global_freqs.cpu)
		cpufreq_notify_transition(&global_freqs, CPUFREQ_POSTCHANGE);

	global_freqs.old = global_freqs.new;

	mutex_unlock(&freq_lock);
	return;
}

static void sprd_find_real_index(unsigned int new_speed, int *index)
{
	int i;
	struct cpufreq_frequency_table *pfreq = sprd_cpufreq_conf->freq_tbl;

	*index = pfreq[0].index;
	for (i = 0; (pfreq[i].frequency != CPUFREQ_TABLE_END); i++) {
		if (new_speed == pfreq[i].frequency) {
			*index = pfreq[i].index;
			break;
		}
	}
	return;
}

static int sprd_update_cpu_speed(int cpu,
	unsigned int target_speed, int index)
{
	unsigned int cpufreq_ceiling_speed = 0;
	unsigned int lock_speed = 0;
	int i, real_index = 0;
	unsigned int new_speed = 0;
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	unsigned int min_speed = shark_top_frequency;
#endif


	for_each_online_cpu(i) {
		new_speed = max(new_speed, sprd_cpufreq_status.percpu_target[i]);
	}
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	for_each_online_cpu(i) {
		min_speed = min(min_speed, sprd_cpufreq_status.percpu_target[i]);
	}

	if ((new_speed == min_speed) && (min_speed == shark_top_frequency))
		schedule_delayed_work_on(0, &plugin_work, msecs_to_jiffies(plugin_delay));

	if (new_speed < bottom_freq)
		new_speed = bottom_freq;
#endif

	if (new_speed > sprd_cpufreq_conf->limited_max_freq)
		new_speed = sprd_cpufreq_conf->limited_max_freq;
	else{
		if ((cpufreq_ceiling_speed = (get_cpufreq_ceiling_speed() / 1000))){
			if (new_speed >= cpufreq_ceiling_speed)
				new_speed = cpufreq_ceiling_speed;
			}
		else if ((lock_speed = (get_perflock_speed() / 1000))){
			if (new_speed <= lock_speed)
				new_speed = lock_speed;
		}
        }
	if (new_speed != sprd_cpufreq_conf->freq_tbl[index].frequency)
		sprd_find_real_index(new_speed, &real_index);
	else
		real_index = index;
	sprd_real_set_cpufreq(cpu, new_speed, real_index);
	return 0;
}

#if 0
static int sprd_cpufreq_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	int i;

	if (event == PM_SUSPEND_PREPARE || event == PM_HIBERNATION_PREPARE) {
		sprd_cpufreq_status.is_suspend = true;

		for_each_online_cpu(i) {
			sprd_cpufreq_status.percpu_target[i] =
				sprd_cpufreq_conf->orignal_freq;
		}

		for (i = 0; i < FREQ_TABLE_SIZE; i++) {
			if (CPUFREQ_TABLE_END == sprd_cpufreq_conf->freq_tbl[i].frequency)
				break;
			if (sprd_cpufreq_conf->freq_tbl[i].frequency ==
				sprd_cpufreq_conf->orignal_freq)
				break;
		}

		if (FREQ_TABLE_SIZE == i ||
				CPUFREQ_TABLE_END == sprd_cpufreq_conf->freq_tbl[i].frequency) {
			pr_err("cpufreq: Failed to find orignal cpu frequency in table\n");
		} else
			sprd_update_cpu_speed(0, sprd_cpufreq_conf->orignal_freq, i);
	} else if (event == PM_POST_SUSPEND || event == PM_POST_HIBERNATION)
		sprd_cpufreq_status.is_suspend = false;

	return NOTIFY_OK;
}

static struct notifier_block sprd_cpufreq_pm_notifier = {
	.notifier_call = sprd_cpufreq_pm_notify,
};
#endif

static int sprd_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu > CONFIG_NR_CPUS) {
		pr_err("%s --- no such cpu id %d\n", __FUNCTION__, policy->cpu);
		return -EINVAL;
	}

	return cpufreq_frequency_table_verify(policy, sprd_cpufreq_conf->freq_tbl);
}

static int sprd_cpufreq_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	int ret = -EFAULT;
	int index;
	unsigned int new_speed;
	struct cpufreq_frequency_table *table;
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	struct unplug_work_info *puwi;
#endif
	if (true == sprd_cpufreq_status.is_suspend)
		return 0;

	table = cpufreq_frequency_get_table(policy->cpu);

	if (cpufreq_frequency_table_target(policy, table,
					target_freq, relation, &index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		return -EINVAL;
	}

	pr_debug("CPU_%d target %d relation %d (%d-%d) selected %d\n",
			policy->cpu, target_freq, relation,
			policy->min, policy->max, table[index].frequency);

	new_speed = table[index].frequency;

	sprd_cpufreq_status.percpu_target[policy->cpu] = new_speed;
	pr_debug("## %s cpu:%d %u on cpu%d\n", __func__, policy->cpu, new_speed, smp_processor_id());

#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	puwi = &uwi[policy->cpu];
	if (new_speed < bottom_freq) {
		if (policy->cpu) {
			puwi->need_unplug = 1;
			pr_debug("we set need unplug here cpu%d\n", policy->cpu);
			if (enabled_dhp)
				schedule_delayed_work_on(0, &puwi->unplug_work, msecs_to_jiffies(unplug_delay));
			return 0;
		}
	} else {
		pr_debug("we ununuset need unplug here cpu%d\n", policy->cpu);
		puwi->need_unplug = 0;
	}
#endif

	ret = sprd_update_cpu_speed(policy->cpu, new_speed, index);

	return ret;
}

static unsigned int sprd_cpufreq_getspeed(unsigned int cpu)
{
	if (cpu > CONFIG_NR_CPUS) {
		pr_err("%s --- no such cpu id %d\n", __FUNCTION__, cpu);
		return -EINVAL;
	}

	return sprd_raw_get_cpufreq();
}

static int sprd_freq_table_init(void)
{
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	struct cpufreq_frequency_table *pfreq;
	unsigned int *pvdd;
	unsigned int i;
#endif

	
	if (soc_is_scx35_v0()) {
		pr_info("%s es_chip", __func__);
		sprd_cpufreq_conf->freq_tbl = sc8830_cpufreq_table_data_es.freq_tbl;
		sprd_cpufreq_conf->vddarm_mv = sc8830_cpufreq_table_data_es.vddarm_mv;
	} else if (soc_is_scx35_v1()) {
		pr_info("%s cs_chip", __func__);
		sprd_cpufreq_conf->freq_tbl = sc8830_cpufreq_table_data_cs.freq_tbl;
		sprd_cpufreq_conf->vddarm_mv = sc8830_cpufreq_table_data_cs.vddarm_mv;
	} else {
		pr_err("%s error chip id\n", __func__);
		return -EINVAL;
	}
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	
	pfreq = sprd_cpufreq_conf->freq_tbl;
	pvdd = sprd_cpufreq_conf->vddarm_mv;

	for (i = 0; (pfreq[i].frequency != CPUFREQ_TABLE_END); i++);
	pfreq[i++].frequency = SHARK_FAKE_FREQUENCY;
	pfreq[i].frequency = CPUFREQ_TABLE_END;
	pfreq[i].index = i;
#endif
	return 0;
}

static int sprd_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;

	cpufreq_frequency_table_cpuinfo(policy, sprd_cpufreq_conf->freq_tbl);
	policy->cur = sprd_raw_get_cpufreq(); 
	policy->cpuinfo.transition_latency = TRANSITION_LATENCY;
	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
	cpumask_copy(policy->related_cpus, cpu_possible_mask);

	cpufreq_frequency_table_get_attr(sprd_cpufreq_conf->freq_tbl, policy->cpu);

	sprd_cpufreq_status.percpu_target[policy->cpu] = policy->cur;

	ret = cpufreq_frequency_table_cpuinfo(policy, sprd_cpufreq_conf->freq_tbl);
	if (ret != 0)
		pr_err("%s --- Failed to config freq table: %d\n", __FUNCTION__, ret);
	pr_err("sprd_cpufreq_driver_init policy->cpu = %d, policy->cur = %u, cpu = %d, ret = %d\n",
		policy->cpu, policy->cur, smp_processor_id(), ret);

	return ret;
}

static int sprd_cpufreq_exit(struct cpufreq_policy *policy)
{
	return 0;
}

static struct freq_attr *sprd_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver sprd_cpufreq_driver = {
	.verify		= sprd_cpufreq_verify_speed,
	.target		= sprd_cpufreq_target,
	.get		= sprd_cpufreq_getspeed,
	.init		= sprd_cpufreq_init,
	.exit		= sprd_cpufreq_exit,
	.name		= "sprd",
	.attr		= sprd_cpufreq_attr,
};



static int sprd_cpufreq_policy_notifier(
	struct notifier_block *nb, unsigned long event, void *data)
{
	return NOTIFY_OK;
}

static struct notifier_block sprd_cpufreq_policy_nb = {
	.notifier_call = sprd_cpufreq_policy_notifier,
};

static int get_max_state(struct thermal_cooling_device *cdev,
			 unsigned long *state)
{
	int ret = 0;

	*state = 2;

	return ret;
}

static int get_cur_state(struct thermal_cooling_device *cdev,
			 unsigned long *state)
{
	int ret = 0;

	*state = sprd_cpufreq_conf->cooling_state;

	return ret;
}

	typedef enum{
		AP_LEVEL_NOT_CONTROL,
		AP_LEVEL_1G_4CORE,
		AP_LEVEL_1G_3CORE,
		AP_LEVEL_768M_2CORE,
		AP_LEVEL_600M_1CORE,
		AP_LEVEL_MAX,
	}thermal_state_type;


static int set_cur_state(struct thermal_cooling_device *cdev,
			 unsigned long state, thermal_table_type thermal_table_num)
{
	int ret = 0;
	
		

	pr_info("cooling operation:0x%ld\n",state);

	sprd_cpufreq_conf->cooling_state = state;
	if(state == AP_LEVEL_1G_3CORE)
	{
		sprd_cpufreq_conf->limited_max_freq = 1000000;
		#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
			temp_limit_cpus = 3;
		#endif
		if(num_online_cpus() == 4)
		{
			cpu_down(3);
		}
	}
	else if(state == AP_LEVEL_1G_4CORE)
	{
		sprd_cpufreq_conf->limited_max_freq = 1000000;
		#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
		    temp_limit_cpus =4;
		#endif
	}
	else if(state == AP_LEVEL_768M_2CORE)
	{
		sprd_cpufreq_conf->limited_max_freq = 768000;
		#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
		    temp_limit_cpus =2;
		#endif
		if(num_online_cpus() >2 ){
			if (cpu_online(2))
				cpu_down(2);
			if (cpu_online(3))
				cpu_down(3);
		}
	}
	else if(state == AP_LEVEL_600M_1CORE)
	{
		sprd_cpufreq_conf->limited_max_freq = 600000;
		#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
		    temp_limit_cpus =1;
		#endif
	       if(num_online_cpus() >1 ){
                  if (cpu_online(1))
                      cpu_down(1);
                  if (cpu_online(2))
                      cpu_down(2);
                  if (cpu_online(3))
                      cpu_down(3);
		}
	}
	else
	{
		sprd_cpufreq_conf->limited_max_freq = 1200000;
		#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
		    temp_limit_cpus=4;
		#endif
	}

	#if 0
	if (state) {
		pr_info("#########--------- cpufreq heating up\n");
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
		enabled_dhp = 0;
#endif
		sprd_cpufreq_conf->limited_max_freq = shark_limited_max_frequency;
#if defined(CONFIG_SMP)
		
		for_each_online_cpu(cpu) {
			if (cpu)
				cpu_down(cpu);
		}
#endif
	} else {
		pr_info("#########--------- cpufreq cooling down\n");
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
		enabled_dhp = 1;
#elif defined(CONFIG_SMP)
		for_each_cpu(cpu, cpu_possible_mask) {
			if (!cpu_online(cpu))
				cpu_up(cpu);
		}
#endif
		sprd_cpufreq_conf->limited_max_freq = shark_top_frequency;
	}
	#endif
	sprd_thm_thermal_table_interrupt_registry(thermal_table_num);
	return ret;
}

#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
void up_all_cpu(void){
        int cpu;
        enabled_dhp = 0;
        pr_info("#########--------- cpu -----------------\n");
        for_each_cpu(cpu, cpu_possible_mask) {
                if (!cpu_online(cpu))
                cpu_up(cpu);
        }
}
EXPORT_SYMBOL_GPL(up_all_cpu);

#if defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
void down_all_cpu(bool lock_plug){
        int i;
	if(lock_plug){
		if(num_online_cpus() >2 ){
			if (cpu_online(2))
				cpu_down(2);
			if (cpu_online(3))
				cpu_down(3);
		}
	}
	else{
		enabled_dhp = 1;
		pr_info("#########--------- cpu +++++++++++++++\n");
		for(i=0;i<4;i++)
		{
			sprd_cpufreq_status.percpu_target[i] =
				sprd_cpufreq_conf->orignal_freq;
		}
	}
}
EXPORT_SYMBOL_GPL(down_all_cpu);
#else
void down_all_cpu(void){
        int i;
        enabled_dhp = 1;
        pr_info("#########--------- cpu +++++++++++++++\n");
        for(i=0;i<4;i++)
         {
             sprd_cpufreq_status.percpu_target[i] =
                 sprd_cpufreq_conf->orignal_freq;
         }
}
EXPORT_SYMBOL_GPL(down_all_cpu);
#endif
#endif

static struct thermal_cooling_device_ops sprd_cpufreq_cooling_ops = {
	.get_max_state = get_max_state,
	.get_cur_state = get_cur_state,
	.set_cur_state = set_cur_state,
};
static int __init sprd_cpufreq_modinit(void)
{
	int ret;

#if defined(CONFIG_ARCH_SCX35)
	sprd_cpufreq_conf = &sc8830_cpufreq_conf;
#elif defined(CONFIG_ARCH_SC8825)
	sprd_cpufreq_conf = &sc8825_cpufreq_conf;
#endif

#if defined(CONFIG_ARCH_SCX35)
	ret = sprd_freq_table_init();
	if (ret)
		return ret;
	shark_top_frequency = sprd_cpufreq_conf->freq_tbl[0].frequency;
	
	shark_limited_max_frequency = sprd_cpufreq_conf->freq_tbl[0].frequency;

	sprd_cpufreq_conf->clk = clk_get_sys(NULL, "clk_mcu");
	if (IS_ERR(sprd_cpufreq_conf->clk))
		return PTR_ERR(sprd_cpufreq_conf->clk);
	if (soc_is_scx35_v0())
		sprd_cpufreq_conf->freq_tbl[0].frequency = sprd_raw_get_cpufreq();
	sprd_cpufreq_conf->mpllclk = clk_get_sys(NULL, "clk_mpll");
	if (IS_ERR(sprd_cpufreq_conf->mpllclk))
		return PTR_ERR(sprd_cpufreq_conf->mpllclk);

	sprd_cpufreq_conf->tdpllclk = clk_get_sys(NULL, "clk_tdpll");
	if (IS_ERR(sprd_cpufreq_conf->tdpllclk))
		return PTR_ERR(sprd_cpufreq_conf->tdpllclk);

	sprd_cpufreq_conf->regulator = regulator_get(NULL, "vddarm");
	if (IS_ERR(sprd_cpufreq_conf->regulator))
		return PTR_ERR(sprd_cpufreq_conf->regulator);
#if 1	
	sprd_cpufreq_conf->cdev = thermal_cooling_device_register("thermal-cpufreq-0", 0,
						&sprd_cpufreq_cooling_ops);
	if (IS_ERR(sprd_cpufreq_conf->cdev))
		return PTR_ERR(sprd_cpufreq_conf->cdev);
#endif
	
	sprd_cpufreq_conf->orignal_freq = sprd_raw_get_cpufreq();
	sprd_cpufreq_conf->limited_max_freq = shark_top_frequency;
	sprd_cpufreq_conf->cooling_state = 0;
	global_freqs.old = sprd_cpufreq_conf->orignal_freq;
	sprd_cpufreq_status.is_suspend = false;

#endif
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	sprd_auto_hotplug_init();
#endif
	ret = cpufreq_register_notifier(
		&sprd_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return ret;

	ret = cpufreq_register_driver(&sprd_cpufreq_driver);

	return ret;
}

static void __exit sprd_cpufreq_modexit(void)
{
#if defined(CONFIG_ARCH_SCX35)
	if (!IS_ERR_OR_NULL(sprd_cpufreq_conf->regulator))
		regulator_put(sprd_cpufreq_conf->regulator);
#endif
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	sprd_auto_hotplug_exit();
#endif
#if 1	
	thermal_cooling_device_unregister(sprd_cpufreq_conf->cdev);
#endif
	cpufreq_unregister_driver(&sprd_cpufreq_driver);
	cpufreq_unregister_notifier(
		&sprd_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);
	return;
}

module_init(sprd_cpufreq_modinit);
module_exit(sprd_cpufreq_modexit);

MODULE_AUTHOR("Jianjun.He <jianjun.he@spreadtrum.com>");
MODULE_DESCRIPTION("cpufreq driver for Spreadtrum");
MODULE_LICENSE("GPL");
