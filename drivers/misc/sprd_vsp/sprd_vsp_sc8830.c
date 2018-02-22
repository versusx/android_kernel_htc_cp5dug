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


#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include <video/sprd_vsp.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/globalregs.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>

#define VSP_MINOR MISC_DYNAMIC_MINOR
#define VSP_AQUIRE_TIMEOUT_MS 500
#define VSP_INIT_TIMEOUT_MS 200

#define USE_INTERRUPT

#define DEFAULT_FREQ_DIV 0x0

#define ARM_ACCESS_CTRL_OFF         0x0
#define ARM_ACCESS_STATUS_OFF   0x04
#define MCU_CTRL_SET_OFF                0x08
#define ARM_INT_STS_OFF                     0x10        
#define ARM_INT_MASK_OFF                0x14
#define ARM_INT_CLR_OFF                     0x18
#define ARM_INT_RAW_OFF                 0x1C
#define WB_ADDR_SET0_OFF                0x20
#define WB_ADDR_SET1_OFF                0x24

#define VSP_GLB_REG_BASE        (SPRD_VSP_BASE+0x1000)
#define VSP_INT_STS_OFF            0x0             
#define VSP_INT_MASK_OFF        0x04
#define VSP_INT_CLR_OFF           0x08
#define VSP_INT_RAW_OFF         0x0c

struct vsp_fh {
    int is_vsp_aquired;
    int is_clock_enabled;

    wait_queue_head_t wait_queue_work;
    int condition_work;
    int vsp_int_status;
};

struct vsp_dev {
    unsigned int freq_div;

    struct semaphore vsp_mutex;

    struct clk *vsp_clk;
    struct clk *vsp_parent_clk;
    struct clk *mm_clk;

    struct vsp_fh *vsp_fp;
};

static struct vsp_dev vsp_hw_dev;
static struct wake_lock vsp_wakelock;

struct clock_name_map_t {
    unsigned long freq;
    char *name;
};

static struct clock_name_map_t clock_name_map[] = {
    {256000000,"clk_256m"},
    {192000000,"clk_192m"},
    {128000000,"clk_128m"},
    {76800000,"clk_76m8"}
};

static int max_freq_level = ARRAY_SIZE(clock_name_map);

static char *vsp_get_clk_src_name(unsigned int freq_level)
{
    if (freq_level >= max_freq_level ) {
        printk(KERN_INFO "set freq_level to 0");
        freq_level = 0;
    }

    return clock_name_map[freq_level].name;
}

static int find_vsp_freq_level(unsigned long freq)
{
    int level = 0;
    int i;
    for (i = 0; i < max_freq_level; i++) {
        if (clock_name_map[i].freq == freq) {
            level = i;
            break;
        }
    }
    return level;
}

#if defined(CONFIG_ARCH_SCX35)
#ifdef USE_INTERRUPT
static irqreturn_t vsp_isr(int irq, void *data);
#endif
#endif
static long vsp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret;
    struct clk *clk_parent;
    char *name_parent;
    unsigned long frequency;
    struct vsp_fh *vsp_fp = filp->private_data;

    switch (cmd) {
    case VSP_CONFIG_FREQ:
        get_user(vsp_hw_dev.freq_div, (int __user *)arg);
        name_parent = vsp_get_clk_src_name(vsp_hw_dev.freq_div);
        clk_parent = clk_get(NULL, name_parent);
        if ((!clk_parent )|| IS_ERR(clk_parent)) {
            printk(KERN_ERR "clock[%s]: failed to get parent [%s] by clk_get()!\n", "clk_vsp", name_parent);
            return -EINVAL;
        }
        ret = clk_set_parent(vsp_hw_dev.vsp_clk, clk_parent);
        if (ret) {
            printk(KERN_ERR "clock[%s]: clk_set_parent() failed!","clk_vsp");
            return -EINVAL;
        } else {
            clk_put(vsp_hw_dev.vsp_parent_clk);
            vsp_hw_dev.vsp_parent_clk = clk_parent;
        }
        pr_debug(KERN_INFO "VSP_CONFIG_FREQ %d\n", vsp_hw_dev.freq_div);
        break;
    case VSP_GET_FREQ:
        frequency = clk_get_rate(vsp_hw_dev.vsp_clk);
        ret = find_vsp_freq_level(frequency);
        put_user(ret, (int __user *)arg);
        pr_debug(KERN_INFO "vsp ioctl VSP_GET_FREQ %d\n", ret);
        break;
    case VSP_ENABLE:
        pr_info("vsp ioctl VSP_ENABLE\n");
        wake_lock(&vsp_wakelock);
        ret = clk_enable(vsp_hw_dev.vsp_clk);
        if (ret) {
            printk(KERN_ERR "###:vsp_hw_dev.vsp_clk: clk_enable() failed!\n");
            return ret;
        } else {
            pr_debug("###vsp_hw_dev.vsp_clk: clk_enable() ok.\n");
        }
        vsp_fp->is_clock_enabled= 1;
        break;
    case VSP_DISABLE:
        pr_info("vsp ioctl VSP_DISABLE\n");
        clk_disable(vsp_hw_dev.vsp_clk);
        vsp_fp->is_clock_enabled = 0;
        wake_unlock(&vsp_wakelock);
        break;
    case VSP_ACQUAIRE:
        pr_info("vsp ioctl VSP_ACQUAIRE begin\n");
        ret = down_timeout(&vsp_hw_dev.vsp_mutex,
                           msecs_to_jiffies(VSP_AQUIRE_TIMEOUT_MS));
        if (ret) {
            printk(KERN_ERR "vsp error timeout\n");
            
            return ret;
        }
#ifdef RT_VSP_THREAD
        if (!rt_task(current)) {
            struct sched_param schedpar;
            int ret;
            struct cred *new = prepare_creds();
            cap_raise(new->cap_effective, CAP_SYS_NICE);
            commit_creds(new);
            schedpar.sched_priority = 1;
            ret = sched_setscheduler(current, SCHED_RR, &schedpar);
            if (ret!=0)
                printk(KERN_ERR "vsp change pri fail a\n");
        }
#endif
        vsp_fp->is_vsp_aquired = 1;
        vsp_hw_dev.vsp_fp = vsp_fp;
        pr_info("vsp ioctl VSP_ACQUAIRE end\n");
        break;
    case VSP_RELEASE:
        pr_info("vsp ioctl VSP_RELEASE\n");
        vsp_fp->is_vsp_aquired = 0;
        up(&vsp_hw_dev.vsp_mutex);
        break;
#ifdef USE_INTERRUPT
    case VSP_COMPLETE:
        pr_info("vsp ioctl VSP_COMPLETE\n");
        ret = wait_event_interruptible_timeout(
                  vsp_fp->wait_queue_work,
                  vsp_fp->condition_work,
                  msecs_to_jiffies(VSP_INIT_TIMEOUT_MS));
        if (ret == -ERESTARTSYS) {
            printk(KERN_INFO "vsp complete -ERESTARTSYS\n");
            vsp_fp->vsp_int_status |= (1<<30);
            put_user(vsp_fp->vsp_int_status, (int __user *)arg);
            ret = -EINVAL;
        } else
        {
            vsp_fp->vsp_int_status &= (~ (1<<30));
            if (ret == 0) {
                printk(KERN_ERR "vsp complete  timeout\n");
                vsp_fp->vsp_int_status |= (1<<31);
                ret = -ETIMEDOUT;
                
                __raw_writel((1<<1) |(1<<2)|(1<<4)|(1<<5), VSP_GLB_REG_BASE+VSP_INT_CLR_OFF);
                __raw_writel((1<<0)|(1<<1)|(1<<2), SPRD_VSP_BASE+ARM_INT_CLR_OFF);
            } else {
                ret = 0;
            }
            put_user(vsp_fp->vsp_int_status, (int __user *)arg);
            vsp_fp->vsp_int_status = 0;
            vsp_fp->condition_work = 0;
        }
        pr_info("vsp ioctl VSP_COMPLETE end\n");
        return ret;
        break;
#endif
    case VSP_RESET:
        pr_info("%s: before vsp ioctl VSP_RESET\n", __FUNCTION__);
#if 0
#else
        __raw_writel((1<<4) |(1<<11), SPRD_MMAHB_BASE + 0x1004);
        udelay(1);
        __raw_writel((1<<4) , SPRD_MMAHB_BASE + 0x2004);
#endif
        pr_info("%s : after vsp ioctl VSP_RESET\n", __FUNCTION__);
        break;
    case VSP_HW_INFO:
    {
        u32 mm_eb_reg;

        pr_debug("vsp ioctl VSP_HW_INFO\n");
        mm_eb_reg = sci_glb_read(SPRD_AONAPB_BASE, 0xFFFFFFFF);
        put_user(mm_eb_reg, (int __user *)arg);
    }
    break;

    default:
        return -EINVAL;
    }
    return 0;
}

#ifdef USE_INTERRUPT
static irqreturn_t vsp_isr(int irq, void *data)
{
    int int_status;
    int ret = 0xff; 

    
    int_status =  __raw_readl(VSP_GLB_REG_BASE+VSP_INT_STS_OFF);
    if((int_status >> 1) & 0x1) 
    {
        __raw_writel((1<<1), VSP_GLB_REG_BASE+VSP_INT_CLR_OFF);
        ret = (1<<1);
    } else if((int_status >> 2) & 0x1) 
    {
        __raw_writel((1<<2), VSP_GLB_REG_BASE+VSP_INT_CLR_OFF);
        ret = (1<<2);
    } else if((int_status >> 4) & 0x1) 
    {
        __raw_writel((1<<4), VSP_GLB_REG_BASE+VSP_INT_CLR_OFF);
        ret = (1<<4);
    } else if((int_status >> 5) & 0x1) 
    {
        __raw_writel((1<<5), VSP_GLB_REG_BASE+VSP_INT_CLR_OFF);
        ret = (1<<5);
    }

    
    int_status =  __raw_readl(SPRD_VSP_BASE+ARM_INT_STS_OFF);
    if ((int_status >> 2) & 0x1) 
    {
        __raw_writel((1<<2), SPRD_VSP_BASE+ARM_INT_CLR_OFF);
    }

    vsp_hw_dev.vsp_fp->vsp_int_status = ret;
    vsp_hw_dev.vsp_fp->condition_work = 1;
    wake_up_interruptible(&vsp_hw_dev.vsp_fp->wait_queue_work);

    return IRQ_HANDLED;
}
#endif

static int vsp_nocache_mmap(struct file *filp, struct vm_area_struct *vma)
{
    printk(KERN_INFO "@vsp[%s]\n", __FUNCTION__);
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    vma->vm_pgoff     = (SPRD_VSP_PHYS>>PAGE_SHIFT);
    if (remap_pfn_range(vma,vma->vm_start, vma->vm_pgoff,
                        vma->vm_end - vma->vm_start, vma->vm_page_prot))
        return -EAGAIN;
    printk(KERN_INFO "@vsp mmap %x,%x,%x\n", (unsigned int)PAGE_SHIFT,
           (unsigned int)vma->vm_start,
           (unsigned int)(vma->vm_end - vma->vm_start));
    return 0;
}

static int vsp_open(struct inode *inode, struct file *filp)
{
    int ret;
    struct vsp_fh *vsp_fp = kmalloc(sizeof(struct vsp_fh), GFP_KERNEL);
    if (vsp_fp == NULL) {
        printk(KERN_ERR "vsp open error occured\n");
        return  -EINVAL;
    }
    filp->private_data = vsp_fp;
    vsp_fp->is_clock_enabled = 0;
    vsp_fp->is_vsp_aquired = 0;

    printk(KERN_INFO "VSP mmi_clk open");
    ret = clk_enable(vsp_hw_dev.mm_clk);
    if (ret) {
        printk(KERN_ERR "###:vsp_hw_dev.mm_clk: clk_enable() failed!\n");
        return ret;
    } else {
        pr_debug("###vsp_hw_dev.mm_clk: clk_enable() ok.\n");
    }

    init_waitqueue_head(&vsp_fp->wait_queue_work);
    vsp_fp->vsp_int_status = 0;
    vsp_fp->condition_work = 0;

    printk(KERN_INFO "vsp_open %p\n", vsp_fp);
    return 0;
}

static int vsp_release (struct inode *inode, struct file *filp)
{
    int ret;
    struct vsp_fh *vsp_fp = filp->private_data;

    pr_info("%s : before disable vsp clock, dump sts %x %x %x %x\n", __FUNCTION__,
        vsp_fp->vsp_int_status, vsp_fp->condition_work,
        vsp_fp->is_vsp_aquired, vsp_fp->is_clock_enabled);

    if (vsp_fp->is_clock_enabled) {
        printk(KERN_ERR "error occured and close clock \n");
        clk_disable(vsp_hw_dev.vsp_clk);
    }

    pr_info("%s : after disable vsp clock, dump %x %x\n", __FUNCTION__,
        sci_glb_raw_read(REG_PMU_APB_PD_MM_TOP_CFG), sci_glb_raw_read(REG_AON_APB_APB_EB0));

    if (vsp_fp->is_vsp_aquired) {
        printk(KERN_ERR "error occured and up vsp_mutex \n");
        up(&vsp_hw_dev.vsp_mutex);
    }

    kfree(filp->private_data);
    filp->private_data=NULL;

    printk(KERN_INFO "VSP mmi_clk close");
    clk_disable(vsp_hw_dev.mm_clk);

    printk(KERN_INFO "vsp_release %p\n", vsp_fp);

    return 0;
}

static const struct file_operations vsp_fops =
{
    .owner = THIS_MODULE,
    .unlocked_ioctl = vsp_ioctl,
    .mmap  = vsp_nocache_mmap,
    .open = vsp_open,
    .release = vsp_release,
};

static struct miscdevice vsp_dev = {
    .minor   = VSP_MINOR,
    .name   = "sprd_vsp",
    .fops   = &vsp_fops,
};

static int vsp_probe(struct platform_device *pdev)
{
    struct clk *clk_mm_i;
    struct clk *clk_vsp;
    struct clk *clk_parent;
    char *name_parent;
    int ret_val;
    int ret;
    int cmd0;

    wake_lock_init(&vsp_wakelock, WAKE_LOCK_SUSPEND,
                   "pm_message_wakelock_vsp");

    sema_init(&vsp_hw_dev.vsp_mutex, 1);

    vsp_hw_dev.freq_div = DEFAULT_FREQ_DIV;

    vsp_hw_dev.vsp_clk = NULL;
    vsp_hw_dev.vsp_parent_clk = NULL;

#if defined(CONFIG_ARCH_SCX35)
    clk_mm_i = clk_get(NULL, "clk_mm_i");
    if (IS_ERR(clk_mm_i) || (!clk_mm_i)) {
        printk(KERN_ERR "###: Failed : Can't get clock [%s}!\n",
               "clk_mm_i");
        printk(KERN_ERR "###: clk_mm_i =  %p\n", clk_mm_i);
        ret = -EINVAL;
        goto errout;
    } else {
        vsp_hw_dev.mm_clk= clk_mm_i;
    }
#endif

    clk_vsp = clk_get(NULL, "clk_vsp");
    if (IS_ERR(clk_vsp) || (!clk_vsp)) {
        printk(KERN_ERR "###: Failed : Can't get clock [%s}!\n",
               "clk_vsp");
        printk(KERN_ERR "###: vsp_clk =  %p\n", clk_vsp);
        ret = -EINVAL;
        goto errout;
    } else {
        vsp_hw_dev.vsp_clk = clk_vsp;
    }

    name_parent = vsp_get_clk_src_name(vsp_hw_dev.freq_div);
    clk_parent = clk_get(NULL, name_parent);
    if ((!clk_parent )|| IS_ERR(clk_parent) ) {
        printk(KERN_ERR "clock[%s]: failed to get parent in probe[%s] \
by clk_get()!\n", "clk_vsp", name_parent);
        ret = -EINVAL;
        goto errout;
    } else {
        vsp_hw_dev.vsp_parent_clk = clk_parent;
    }

    ret_val = clk_set_parent(vsp_hw_dev.vsp_clk, vsp_hw_dev.vsp_parent_clk);
    if (ret_val) {
        printk(KERN_ERR "clock[%s]: clk_set_parent() failed in probe!",
               "clk_vsp");
        ret = -EINVAL;
        goto errout;
    }

    printk("vsp parent clock name %s\n", name_parent);
    printk("vsp_freq %d Hz",
           (int)clk_get_rate(vsp_hw_dev.vsp_clk));

    ret = misc_register(&vsp_dev);
    if (ret) {
        printk(KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
               VSP_MINOR, ret);
        goto errout;
    }

#ifdef USE_INTERRUPT
    
    ret = request_irq(IRQ_VSP_INT, vsp_isr, IRQF_DISABLED, "VSP", &vsp_hw_dev);
    if (ret) {
        printk(KERN_ERR "vsp: failed to request irq!\n");
        ret = -EINVAL;
        goto errout2;
    }
#endif


    return 0;

#ifdef USE_INTERRUPT
errout2:
    misc_deregister(&vsp_dev);
#endif

errout:
    if (vsp_hw_dev.vsp_clk) {
        clk_put(vsp_hw_dev.vsp_clk);
    }

    if (vsp_hw_dev.vsp_parent_clk) {
        clk_put(vsp_hw_dev.vsp_parent_clk);
    }
    return ret;
}

static int vsp_remove(struct platform_device *pdev)
{
    printk(KERN_INFO "vsp_remove called !\n");

    misc_deregister(&vsp_dev);

#ifdef USE_INTERRUPT
    free_irq(IRQ_VSP_INT, &vsp_hw_dev);
#endif

    if (vsp_hw_dev.vsp_clk) {
        clk_put(vsp_hw_dev.vsp_clk);
    }

    if (vsp_hw_dev.vsp_parent_clk) {
        clk_put(vsp_hw_dev.vsp_parent_clk);
    }

    printk(KERN_INFO "vsp_remove Success !\n");
    return 0;
}

static struct platform_driver vsp_driver = {
    .probe    = vsp_probe,
    .remove   = vsp_remove,
    .driver   = {
        .owner = THIS_MODULE,
        .name = "sprd_vsp",
    },
};

static int __init vsp_init(void)
{
    printk(KERN_INFO "vsp_init called !\n");
    if (platform_driver_register(&vsp_driver) != 0) {
        printk(KERN_ERR "platform device vsp drv register Failed \n");
        return -1;
    }
    return 0;
}

static void __exit vsp_exit(void)
{
    printk(KERN_INFO "vsp_exit called !\n");
    platform_driver_unregister(&vsp_driver);
}

module_init(vsp_init);
module_exit(vsp_exit);

MODULE_DESCRIPTION("SPRD VSP Driver");
MODULE_LICENSE("GPL");
