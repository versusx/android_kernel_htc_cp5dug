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
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sprd_cproc.h>

#define CPROC_WDT_TRUE   1
#define CPROC_WDT_FLASE  0
#define NORMAL_MSG "started\n"
#define STOP_MSG "stopped\n"
#define WDTIRQ_MSG "wdtirq\n"

enum {
	CP_NORMAL_STATUS=0,
	CP_STOP_STATUS,
	CP_WDTIRQ_STATUS,
};

struct cproc_device {
	struct miscdevice	miscdev;
	struct cproc_init_data	*initdata;
	void			*vbase;
	int 			wdtirq;
	int			wdtcnt;
	wait_queue_head_t	wdtwait;
	char *			name;
	int			status;
};

struct cproc_proc_fs {
	struct proc_dir_entry	*procdir;
	struct proc_dir_entry	*start;
	struct proc_dir_entry	*stop;
	struct proc_dir_entry	*modem;
	struct proc_dir_entry	*dsp;
	struct proc_dir_entry	*dlnv;
	struct proc_dir_entry	*fixnv;
	struct proc_dir_entry	*runtimenv;
	struct proc_dir_entry	*protnv;
	struct proc_dir_entry	*rfnv;
	struct proc_dir_entry	*cmdline;
	struct proc_dir_entry	*status;
	struct proc_dir_entry	*wdtirq;
	struct proc_dir_entry	*mem;
	struct cproc_device	*cproc;
};

static struct cproc_proc_fs proc_td_fs_entries;
static struct cproc_proc_fs proc_w_fs_entries;

static int sprd_cproc_open(struct inode *inode, struct file *filp)
{
	struct cproc_device *cproc = container_of(filp->private_data,
			struct cproc_device, miscdev);

	filp->private_data = cproc;

	pr_info("cproc %s opened!\n", cproc->initdata->devname);

	return 0;
}

static int sprd_cproc_release (struct inode *inode, struct file *filp)
{
	struct cproc_device *cproc = filp->private_data;

	pr_info("cproc %s closed!\n", cproc->initdata->devname);

	return 0;
}

static long sprd_cproc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/*struct cproc_device *cproc = filp->private_data;*/

	/* TODO: for general modem download&control */

	return 0;
}

static int sprd_cproc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/*struct cproc_device *cproc = filp->private_data;*/

	/* TODO: for general modem download&control */

	return 0;
}

static const struct file_operations sprd_cproc_fops = {
	.owner = THIS_MODULE,
	.open = sprd_cproc_open,
	.release = sprd_cproc_release,
	.unlocked_ioctl = sprd_cproc_ioctl,
	.mmap = sprd_cproc_mmap,
};

static int cproc_proc_td_open(struct inode *inode, struct file *filp)
{
	filp->private_data = PDE(inode)->data;
	return 0;
}

static int cproc_proc_td_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static ssize_t cproc_proc_td_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = proc_td_fs_entries.cproc;
	unsigned int len;
	void *vmem;
	int rval;

	pr_info("cproc proc read type: %s ppos %lld\n", type, *ppos);

	 if (strcmp(type, "mem") == 0) {
		if (cproc->initdata->maxsz < *ppos) {
			return -EINVAL;
		} else if (cproc->initdata->maxsz == *ppos) {
			return 0;
		}

		if ((*ppos + count) > cproc->initdata->maxsz) {
			count = cproc->initdata->maxsz - *ppos;
		}
		vmem = cproc->vbase + *ppos;
		if (copy_to_user(buf, vmem, count))	{
			return -EFAULT;
		}
		*ppos += count;
		return count;
	} else if (strcmp(type, "status") == 0) {
		if (CP_STOP_STATUS == cproc->status) {
			len = strlen(STOP_MSG);
			count = (len > count) ? count : len;
			if (copy_to_user(buf, STOP_MSG, count)) {
				printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
				return -EFAULT;
			} else {
				return count;
			}
		} else if (CP_WDTIRQ_STATUS == cproc->status) {
			len = strlen(WDTIRQ_MSG);
			count = (len > count) ? count : len;
			if (copy_to_user(buf, WDTIRQ_MSG, count)) {
				printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
				return -EFAULT;
			} else {
				return count;
			}
		} else {
			len = strlen(NORMAL_MSG);
			count = (len > count) ? count : len;
			if (copy_to_user(buf, NORMAL_MSG, count)) {
				printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
				return -EFAULT;
			} else {
				return count;
			}
		}
	} else if (strcmp(type, "wdtirq") == 0) {
		/* wait forever */
		rval = wait_event_interruptible(cproc->wdtwait, cproc->wdtcnt  != CPROC_WDT_FLASE);
		if (rval < 0) {
			printk(KERN_ERR "cproc_proc_read wait interrupted error !\n");
		}
		len = strlen(WDTIRQ_MSG);
		count = (len > count) ? count : len;
		if (copy_to_user(buf, WDTIRQ_MSG, count)) {
			printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
			return -EFAULT;
		} else {
			printk(KERN_INFO "cproc proc read wdtirq data !\n");
			return count;
		}
	} else {
		return -EINVAL;
	}
}

static ssize_t cproc_proc_td_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = proc_td_fs_entries.cproc;
	uint32_t base, size, offset;
	void *vmem;

	pr_info("cproc proc write type: %s\n!", type);

	if (strcmp(type, "start") == 0) {
		printk(KERN_INFO "cproc_proc_write to map cproc base start++\n");
		cproc->initdata->start(NULL);
		cproc->wdtcnt = CPROC_WDT_FLASE;
		cproc->status = CP_NORMAL_STATUS;
		printk(KERN_INFO "cproc_proc_write to map cproc base start--\n");
		return count;
	}

	if (strcmp(type, "stop") == 0) {
		printk(KERN_INFO "cproc_proc_write to map cproc base stop++\n");
		cproc->initdata->stop(NULL);
		cproc->status = CP_STOP_STATUS;
		printk(KERN_INFO "cproc_proc_write to map cproc base stop--\n");
		return count;
	}

	if (strcmp(type, "modem") == 0) {
		base = cproc->initdata->segs[0].base;
		size = cproc->initdata->segs[0].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "dsp") == 0) {
		base = cproc->initdata->segs[1].base;
		size = cproc->initdata->segs[1].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "dlnv") == 0) {
		base = cproc->initdata->segs[2].base;
		size = cproc->initdata->segs[2].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "fixnv") == 0) {
		base = cproc->initdata->segs[3].base;
		size = cproc->initdata->segs[3].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "runtimenv") == 0) {
		base = cproc->initdata->segs[4].base;
		size = cproc->initdata->segs[4].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "protnv") == 0) {
		base = cproc->initdata->segs[5].base;
		size = cproc->initdata->segs[5].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "rfnv") == 0) {
		base = cproc->initdata->segs[6].base;
		size = cproc->initdata->segs[6].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "cmdline") == 0) {
		base = cproc->initdata->segs[7].base;
		size = cproc->initdata->segs[7].maxsz;
		offset = *ppos;
	} else {
		return -EINVAL;
	}

	pr_info("cproc proc write: 0x%08x, 0x%08x!\n", base + offset, count);
	vmem = cproc->vbase + (base - cproc->initdata->base) + offset;

	if (copy_from_user(vmem, buf, count)) {
		return -EFAULT;
	}

	*ppos += count;

	return count;
}

static loff_t cproc_proc_td_lseek(struct file* filp, loff_t off, int whence )
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = proc_td_fs_entries.cproc;
	loff_t new;

	switch (whence) {
		case 0:
		new = off;
		filp->f_pos = new;
		break;
		case 1:
		new = filp->f_pos + off;
		filp->f_pos = new;
		break;
		case 2:
		 if (strcmp(type, "mem") == 0) {
			new = cproc->initdata->maxsz + off;
			filp->f_pos = new;
		} else {
			return -EINVAL;
		}
		break;
		default:
		return -EINVAL;
	}
	return (new);
}

static unsigned int cproc_proc_td_poll(struct file *filp, poll_table *wait)
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = proc_td_fs_entries.cproc;
	unsigned int mask = 0;

	pr_info("cproc proc poll  type: %s \n", type);

	if (strcmp(type, "wdtirq") == 0) {
		poll_wait(filp, &cproc->wdtwait, wait);
		if (cproc->wdtcnt  != CPROC_WDT_FLASE) {
			mask |= POLLIN | POLLRDNORM;
		}
	} else {
		printk(KERN_ERR "cproc_proc_poll file don't support poll !\n");
		return -EINVAL;
	}
	return mask;
}


static int cproc_proc_w_open(struct inode *inode, struct file *filp)
{
	filp->private_data = PDE(inode)->data;
	return 0;

}


static int cproc_proc_w_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;

}


static loff_t cproc_proc_w_lseek(struct file* filp, loff_t off, int whence)
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = proc_w_fs_entries.cproc;
	loff_t new;

	switch (whence) {
		case 0:
		new = off;
		filp->f_pos = new;
		break;
		case 1:
		new = filp->f_pos + off;
		filp->f_pos = new;
		break;
		case 2:
		 if (strcmp(type, "mem") == 0) {
			new = cproc->initdata->maxsz + off;
			filp->f_pos = new;
		} else {
			return -EINVAL;
		}
		break;
		default:
		return -EINVAL;
	}
	return (new);

}


static ssize_t cproc_proc_w_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = proc_w_fs_entries.cproc;
	unsigned int len;
	void *vmem;
	int rval;

	pr_info("cproc proc read type: %s ppos %lld\n", type, *ppos);

	 if (strcmp(type, "mem") == 0) {
		if (cproc->initdata->maxsz < *ppos) {
			return -EINVAL;
		} else if (cproc->initdata->maxsz == *ppos) {
			return 0;
		}

		if ((*ppos + count) > cproc->initdata->maxsz) {
			count = cproc->initdata->maxsz - *ppos;
		}
		vmem = cproc->vbase + *ppos;
		if (copy_to_user(buf, vmem, count)) {
			return -EFAULT;
		}
		*ppos += count;
		return count;
	} else if (strcmp(type, "status") == 0) {
		if (CP_STOP_STATUS == cproc->status) {
			len = strlen(STOP_MSG);
			count = (len > count) ? count : len;
			if (copy_to_user(buf, STOP_MSG, count)) {
				printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
				return -EFAULT;
			} else {
				return count;
			}
		} else if (CP_WDTIRQ_STATUS == cproc->status) {
			len = strlen(WDTIRQ_MSG);
			count = (len > count) ? count : len;
			if (copy_to_user(buf, WDTIRQ_MSG, count)) {
				printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
				return -EFAULT;
			} else {
				return count;
			}
		} else {
			len = strlen(NORMAL_MSG);
			count = (len > count) ? count : len;
			if (copy_to_user(buf, NORMAL_MSG, count)) {
				printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
				return -EFAULT;
			} else {
				return count;
			}
		}
	} else if (strcmp(type, "wdtirq") == 0) {
		/* wait forever */
		rval = wait_event_interruptible(cproc->wdtwait, cproc->wdtcnt  != CPROC_WDT_FLASE);
		if (rval < 0) {
			printk(KERN_ERR "cproc_proc_read wait interrupted error !\n");
		}
		len = strlen(WDTIRQ_MSG);
		count = (len > count) ? count : len;
		if (copy_to_user(buf, WDTIRQ_MSG, count)) {
			printk(KERN_ERR "cproc_proc_read copy data to user error !\n");
			return -EFAULT;
		} else {
			printk(KERN_INFO "cproc proc read wdtirq data !\n");
			return count;
		}
	} else {
		return -EINVAL;
	}

}


static ssize_t cproc_proc_w_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = proc_w_fs_entries.cproc;
	uint32_t base, size, offset;
	void *vmem;

	pr_info("cproc proc write type: %s\n!", type);

	if (strcmp(type, "start") == 0) {
		printk(KERN_INFO "cproc_proc_write to map cproc base start++\n");
		cproc->initdata->start(NULL);
		cproc->wdtcnt = CPROC_WDT_FLASE;
		cproc->status = CP_NORMAL_STATUS;
		printk(KERN_INFO "cproc_proc_write to map cproc base start--\n");
		return count;
	}

	if (strcmp(type, "stop") == 0) {
		printk(KERN_INFO "cproc_proc_write to map cproc base stop++\n");
		cproc->initdata->stop(NULL);
		cproc->status = CP_STOP_STATUS;
		printk(KERN_INFO "cproc_proc_write to map cproc base stop--\n");
		return count;
	}

	if (strcmp(type, "modem") == 0) {
		base = cproc->initdata->segs[0].base;
		size = cproc->initdata->segs[0].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "dsp") == 0) {
		base = cproc->initdata->segs[1].base;
		size = cproc->initdata->segs[1].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "dlnv") == 0) {
		base = cproc->initdata->segs[2].base;
		size = cproc->initdata->segs[2].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "fixnv") == 0) {
		base = cproc->initdata->segs[3].base;
		size = cproc->initdata->segs[3].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "runtimenv") == 0) {
		base = cproc->initdata->segs[4].base;
		size = cproc->initdata->segs[4].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "protnv") == 0) {
		base = cproc->initdata->segs[5].base;
		size = cproc->initdata->segs[5].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "rfnv") == 0) {
		base = cproc->initdata->segs[6].base;
		size = cproc->initdata->segs[6].maxsz;
		offset = *ppos;
	} else if (strcmp(type, "cmdline") == 0) {
		base = cproc->initdata->segs[7].base;
		size = cproc->initdata->segs[7].maxsz;
		offset = *ppos;
	} else {
		return -EINVAL;
	}

	pr_info("cproc proc write: 0x%08x, 0x%08x!\n", base + offset, count);
	vmem = cproc->vbase + (base - cproc->initdata->base) + offset;

	if (copy_from_user(vmem, buf, count)) {
		return -EFAULT;
	}

	*ppos += count;

	return count;

}


static unsigned int cproc_proc_w_poll(struct file *filp, poll_table *wait)
{
	char *type = (char *)filp->private_data;
	struct cproc_device *cproc = proc_w_fs_entries.cproc;
	unsigned int mask = 0;

	pr_info("cproc proc poll  type: %s \n", type);

	if (strcmp(type, "wdtirq") == 0) {
		poll_wait(filp, &cproc->wdtwait, wait);
		if (cproc->wdtcnt  != CPROC_WDT_FLASE) {
			mask |= POLLIN | POLLRDNORM;
		}
	} else {
		printk(KERN_ERR "cproc_proc_poll file don't support poll !\n");
		return -EINVAL;
	}
	return mask;

}

struct file_operations cproc_td_fs_fops = {
	.open		= cproc_proc_td_open,
	.release	= cproc_proc_td_release,
	.llseek  	= cproc_proc_td_lseek,
	.read		= cproc_proc_td_read,
	.write		= cproc_proc_td_write,
	.poll		= cproc_proc_td_poll,
};

struct file_operations cproc_w_fs_fops = {
	.open		= cproc_proc_w_open,
	.release	= cproc_proc_w_release,
	.llseek  	= cproc_proc_w_lseek,
	.read		= cproc_proc_w_read,
	.write		= cproc_proc_w_write,
	.poll		= cproc_proc_w_poll,
};


static inline void sprd_cproc_td_fs_init(struct cproc_device *cproc)
{
	proc_td_fs_entries.procdir = proc_mkdir(cproc->name, NULL);

	proc_td_fs_entries.start = proc_create_data("start", S_IWUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "start");
	proc_td_fs_entries.stop = proc_create_data("stop", S_IWUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "stop");
	proc_td_fs_entries.modem = proc_create_data("modem_bank", S_IWUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "modem");
	proc_td_fs_entries.dsp = proc_create_data("dsp_bank", S_IWUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "dsp");
	proc_td_fs_entries.dlnv = proc_create_data("nvd_bank", S_IWUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "dlnv");
	proc_td_fs_entries.fixnv = proc_create_data("nvf_bank", S_IWUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "fixnv");
	proc_td_fs_entries.runtimenv = proc_create_data("nvr_bank", S_IWUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "runtimenv");
	proc_td_fs_entries.protnv = proc_create_data("nvp_bank", S_IWUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "protnv");
	proc_td_fs_entries.rfnv = proc_create_data("nvc_bank", S_IWUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "rfnv");
	proc_td_fs_entries.cmdline = proc_create_data("cmdline_bank", S_IWUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "cmdline");
	proc_td_fs_entries.status = proc_create_data("status", S_IRUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "status");
	proc_td_fs_entries.wdtirq = proc_create_data("wdtirq", S_IRUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "wdtirq");
	proc_td_fs_entries.mem = proc_create_data("mem", S_IRUSR, proc_td_fs_entries.procdir,
								&cproc_td_fs_fops, "mem");
	proc_td_fs_entries.cproc = cproc;
}

static inline void sprd_cproc_w_fs_init(struct cproc_device *cproc)
{
	proc_w_fs_entries.procdir = proc_mkdir(cproc->name, NULL);

	proc_w_fs_entries.start = proc_create_data("start", S_IWUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "start");
	proc_w_fs_entries.stop = proc_create_data("stop", S_IWUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "stop");
	proc_w_fs_entries.modem = proc_create_data("modem_bank", S_IWUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "modem");
	proc_w_fs_entries.dsp = proc_create_data("dsp_bank", S_IWUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "dsp");
	proc_w_fs_entries.dlnv = proc_create_data("nvd_bank", S_IWUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "dlnv");
	proc_w_fs_entries.fixnv = proc_create_data("nvf_bank", S_IWUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "fixnv");
	proc_w_fs_entries.runtimenv = proc_create_data("nvr_bank", S_IWUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "runtimenv");
	proc_w_fs_entries.protnv = proc_create_data("nvp_bank", S_IWUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "protnv");
	proc_w_fs_entries.rfnv = proc_create_data("nvc_bank", S_IWUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "rfnv");
	proc_w_fs_entries.cmdline = proc_create_data("cmdline_bank", S_IWUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "cmdline");
	proc_w_fs_entries.status = proc_create_data("status", S_IRUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "status");
	proc_w_fs_entries.wdtirq = proc_create_data("wdtirq", S_IRUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "wdtirq");
	proc_w_fs_entries.mem = proc_create_data("mem", S_IRUSR, proc_w_fs_entries.procdir,
								&cproc_w_fs_fops, "mem");
	proc_w_fs_entries.cproc = cproc;
}


static inline void sprd_cproc_td_fs_exit(struct cproc_device *cproc)
{
	remove_proc_entry("mem", proc_td_fs_entries.procdir);
	remove_proc_entry("wdtirq", proc_td_fs_entries.procdir);
	remove_proc_entry("status", proc_td_fs_entries.procdir);
	remove_proc_entry("cmdline_bank", proc_td_fs_entries.procdir);
	remove_proc_entry("rfnv_bank", proc_td_fs_entries.procdir);
	remove_proc_entry("protnv_bank", proc_td_fs_entries.procdir);
	remove_proc_entry("runtimenv_bank", proc_td_fs_entries.procdir);
	remove_proc_entry("fixnv_bank", proc_td_fs_entries.procdir);
	remove_proc_entry("dlnv_bank", proc_td_fs_entries.procdir);
	remove_proc_entry("dsp_bank", proc_td_fs_entries.procdir);
	remove_proc_entry("modem_bank", proc_td_fs_entries.procdir);
	remove_proc_entry("stop", proc_td_fs_entries.procdir);
	remove_proc_entry("start", proc_td_fs_entries.procdir);
	remove_proc_entry(cproc->name, NULL);
}


static inline void sprd_cproc_w_fs_exit(struct cproc_device *cproc)
{
	remove_proc_entry("mem", proc_w_fs_entries.procdir);
	remove_proc_entry("wdtirq", proc_w_fs_entries.procdir);
	remove_proc_entry("status", proc_w_fs_entries.procdir);
	remove_proc_entry("cmdline_bank", proc_w_fs_entries.procdir);
	remove_proc_entry("rfnv_bank", proc_w_fs_entries.procdir);
	remove_proc_entry("protnv_bank", proc_w_fs_entries.procdir);
	remove_proc_entry("runtimenv_bank", proc_w_fs_entries.procdir);
	remove_proc_entry("fixnv_bank", proc_w_fs_entries.procdir);
	remove_proc_entry("dlnv_bank", proc_w_fs_entries.procdir);
	remove_proc_entry("dsp_bank", proc_w_fs_entries.procdir);
	remove_proc_entry("modem_bank", proc_w_fs_entries.procdir);
	remove_proc_entry("stop", proc_w_fs_entries.procdir);
	remove_proc_entry("start", proc_w_fs_entries.procdir);
	remove_proc_entry(cproc->name, NULL);
}


static irqreturn_t sprd_cproc_irq_handler(int irq, void *dev_id)
{
	struct cproc_device *cproc = (struct cproc_device *)dev_id;

	printk("sprd_cproc_irq_handler cp watchdog enable !\n");
	cproc->wdtcnt = CPROC_WDT_TRUE;
	cproc->status = CP_WDTIRQ_STATUS;
	wake_up_interruptible_all(&(cproc->wdtwait));
	return IRQ_HANDLED;
}

static int sprd_cproc_probe(struct platform_device *pdev)
{
	struct cproc_device *cproc;
	int rval;

	cproc = kzalloc(sizeof(struct cproc_device), GFP_KERNEL);
	if (!cproc) {
		printk(KERN_ERR "failed to allocate cproc device!\n");
		return -ENOMEM;
	}

	cproc->initdata = pdev->dev.platform_data;

	cproc->miscdev.minor = MISC_DYNAMIC_MINOR;
	cproc->miscdev.name = cproc->initdata->devname;
	cproc->miscdev.fops = &sprd_cproc_fops;
	cproc->miscdev.parent = NULL;
	cproc->name = cproc->initdata->devname;
	rval = misc_register(&cproc->miscdev);
	if (rval) {
		kfree(cproc);
		printk(KERN_ERR "failed to register sprd_cproc miscdev!\n");
		return rval;
	}

	cproc->vbase = ioremap(cproc->initdata->base, cproc->initdata->maxsz);
	if (!cproc->vbase) {
		misc_deregister(&cproc->miscdev);
		kfree(cproc);
		printk(KERN_ERR "Unable to map cproc base: 0x%08x\n", cproc->initdata->base);
		return -ENOMEM;
	}
	cproc->status = CP_NORMAL_STATUS;
	cproc->wdtcnt = CPROC_WDT_FLASE;
	init_waitqueue_head(&(cproc->wdtwait));

	/* register IPI irq */
	rval = request_irq(cproc->initdata->wdtirq, sprd_cproc_irq_handler,
			0, cproc->initdata->devname, cproc);
	if (rval != 0) {
		printk(KERN_ERR "Cproc failed to request irq %s: %d\n",
				cproc->initdata->devname, cproc->initdata->wdtirq);
		return rval;
	}

	if (!strcmp("cpt", cproc->name))
		sprd_cproc_td_fs_init(cproc);
	else if (!strcmp("cpw", cproc->name))
		sprd_cproc_w_fs_init(cproc);

	platform_set_drvdata(pdev, cproc);

	printk(KERN_INFO "cproc %s probed!\n", cproc->initdata->devname);

	return 0;
}

static int sprd_cproc_remove(struct platform_device *pdev)
{
	struct cproc_device *cproc = platform_get_drvdata(pdev);

	if (!strcmp("cpt", cproc->name))
		sprd_cproc_td_fs_exit(cproc);
	else if (!strcmp("cpw", cproc->name))
		sprd_cproc_w_fs_exit(cproc);
	iounmap(cproc->vbase);
	misc_deregister(&cproc->miscdev);
	kfree(cproc);

	printk(KERN_INFO "cproc %s removed!\n", cproc->initdata->devname);

	return 0;
}

static struct platform_driver sprd_cproc_driver = {
	.probe    = sprd_cproc_probe,
	.remove   = sprd_cproc_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "sprd_cproc",
	},
};

static int __init sprd_cproc_init(void)
{
	if (platform_driver_register(&sprd_cproc_driver) != 0) {
		printk(KERN_ERR "sprd_cproc platform drv register Failed \n");
		return -1;
	}
	return 0;
}

static void __exit sprd_cproc_exit(void)
{
	platform_driver_unregister(&sprd_cproc_driver);
}

module_init(sprd_cproc_init);
module_exit(sprd_cproc_exit);

MODULE_DESCRIPTION("SPRD Communication Processor Driver");
MODULE_LICENSE("GPL");

