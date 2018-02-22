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
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/highmem.h>
#include <linux/freezer.h>
#include <media/videobuf-vmalloc.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <linux/platform_device.h>
#include <mach/board.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>

#include "dcam_drv_sc8830.h"
#include "csi2/csi_api.h"
#include <media/rawchip_interface.h>
#include "htccallback_sysfs_init.h"
#include "camera_sysfs_init.h"


#define LOCAL

#define DCAM_MODULE_NAME                        "DCAM"
#define DCAM_MINOR                              MISC_DYNAMIC_MINOR
#define DCAM_INVALID_FOURCC                     0xFFFFFFFF
#define DCAM_MAJOR_VERSION                      1
#define DCAM_MINOR_VERSION                      0
#define DCAM_RELEASE                            0
#define DCAM_QUEUE_LENGTH                       16
#define DCAM_TIMING_LEN                         16
#define DCAM_TIMEOUT                            1500
#define DEBUG_STR                               "Error L %d, %s: \n"
#define DEBUG_ARGS                              __LINE__,__FUNCTION__
#define V4L2_RTN_IF_ERR(n)          \
	do {                        \
		if(unlikely(n)) {                \
			printk(DEBUG_STR,DEBUG_ARGS);            \
			goto exit;       \
		}                        \
	} while(0)

#define V4L2_PRINT_IF_ERR(n)          \
	do {                        \
		if(unlikely(n)) {                \
			printk(DEBUG_STR,DEBUG_ARGS);            \
		}                        \
	} while(0)

#define DCAM_VERSION \
	KERNEL_VERSION(DCAM_MAJOR_VERSION, DCAM_MINOR_VERSION, DCAM_RELEASE)

typedef int (*path_cfg_func)(enum dcam_cfg_id, void *);

enum
{
	V4L2_TX_DONE  = 0x00,
	V4L2_NO_MEM   = 0x01,
	V4L2_TX_ERR   = 0x02,
	V4L2_CSI2_ERR = 0x03,
	V4L2_SYS_BUSY = 0x04,
	V4L2_TIMEOUT  = 0x10,
	V4L2_TX_STOP  = 0xFF
};

enum
{
	PATH_IDLE  = 0x00,
	PATH_RUN,
};

int rawchip_image_width = 0;
static struct wake_lock cam_wake_lock;
static int v4l2_path1_cnt = 0;

LOCAL int video_nr = -1;
module_param(video_nr, uint, 0644);
MODULE_PARM_DESC(video_nr, "videoX start number, -1 is autodetect");

LOCAL unsigned debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "activates debug info");

LOCAL unsigned int vid_limit = 16;
module_param(vid_limit, uint, 0644);
MODULE_PARM_DESC(vid_limit, "capture memory limit in megabytes");

LOCAL LIST_HEAD(dcam_devlist);

struct dcam_format {
	char                       *name;
	uint32_t                   fourcc;
	int                        depth;
};

struct dcam_node {
	uint32_t                   irq_flag;
	uint32_t                   f_type;
	uint32_t                   index;
	uint32_t                   height;
	uint32_t                   reserved;
};

struct dcam_queue {
	struct dcam_node           node[DCAM_QUEUE_LENGTH];
	struct dcam_node           *write;
	struct dcam_node           *read;
};

struct dcam_path_spec {
	uint32_t                   is_work;
	uint32_t                   is_from_isp;
	uint32_t                   status;
	struct dcam_size           in_size;
	struct dcam_path_dec       img_deci;
	struct dcam_rect           in_rect;
	struct dcam_size           out_size;
	enum   dcam_fmt            out_fmt;
	struct dcam_endian_sel     end_sel;
	uint32_t                   pixel_depth;
	uint32_t                   frm_id_base;
	uint32_t                   frm_type;
	struct dcam_addr           frm_addr[DCAM_FRM_CNT_MAX];
	struct dcam_frame          *frm_ptr[DCAM_FRM_CNT_MAX];
	uint32_t                   frm_cnt_act;
	uint32_t                   path_frm_deci;
};

struct dcam_info {
	uint32_t                   if_mode;
	uint32_t                   sn_mode;
	uint32_t                   yuv_ptn;
	uint32_t                   data_bits;
	uint32_t                   is_loose;
	uint32_t                   lane_num;
	struct dcam_cap_sync_pol   sync_pol;
	uint32_t                   frm_deci;
	struct dcam_cap_dec        img_deci;
	struct dcam_size           cap_in_size;
	struct dcam_rect           cap_in_rect;
	struct dcam_size           cap_out_size;
	
	struct dcam_path_spec      dcam_path[DCAM_PATH_NUM];

	uint32_t                   capture_mode;
	uint32_t                   skip_number;
};

struct dcam_dev {
	struct list_head         dcam_devlist;
	struct v4l2_device       v4l2_dev;
	struct mutex             dcam_mutex;
	struct mutex             tx_mutex;
	struct semaphore         irq_sem;
	atomic_t                 users;
	struct video_device      *vfd;
	struct dcam_info         dcam_cxt;
	atomic_t                 stream_on;
	uint32_t                 stream_mode;
	struct dcam_queue        queue;
	struct timer_list        dcam_timer;
	atomic_t                 run_flag;
};

#ifndef __SIMULATOR__
LOCAL int sprd_v4l2_tx_done(struct dcam_frame *frame, void* param);
LOCAL int sprd_v4l2_tx_error(struct dcam_frame *frame, void* param);
LOCAL int sprd_v4l2_no_mem(struct dcam_frame *frame, void* param);
LOCAL int sprd_v4l2_queue_write(struct dcam_queue *queue, struct dcam_node *node);
LOCAL int sprd_v4l2_queue_read(struct dcam_queue *queue, struct dcam_node *node);
LOCAL int sprd_start_timer(struct timer_list *dcam_timer, uint32_t time_val);
LOCAL int sprd_stop_timer(struct timer_list *dcam_timer);
LOCAL int sprd_v4l2_streampause(struct file *file, uint32_t channel_id);
LOCAL int sprd_v4l2_streamresume(struct file *file, uint32_t channel_id);


LOCAL const dcam_isr_func sprd_v4l2_isr[] = {
	sprd_v4l2_tx_done,
	sprd_v4l2_tx_error,
	sprd_v4l2_no_mem
};

LOCAL struct proc_dir_entry*  v4l2_proc_file;

LOCAL struct dcam_format dcam_img_fmt[] = {
	{
		.name     = "4:2:2, packed, YUYV",
		.fourcc   = V4L2_PIX_FMT_YUYV,
		.depth    = 16,
	},
	{
		.name     = "4:2:2, packed, YVYU",
		.fourcc   = V4L2_PIX_FMT_YVYU,
		.depth    = 16,
	},
	{
		.name     = "4:2:2, packed, UYVY",
		.fourcc   = V4L2_PIX_FMT_UYVY,
		.depth    = 16,
	},
	{
		.name     = "4:2:2, packed, VYUY",
		.fourcc   = V4L2_PIX_FMT_VYUY,
		.depth    = 16,
	},
	{
		.name     = "YUV 4:2:2, planar, (Y-Cb-Cr)",
		.fourcc   = V4L2_PIX_FMT_YUV422P,
		.depth    = 16,
	},
	{
		.name     = "YUV 4:2:0 planar (Y-CbCr)",
		.fourcc   = V4L2_PIX_FMT_NV12,
		.depth    = 12,
	},
	{
		.name     = "YVU 4:2:0 planar (Y-CrCb)",
		.fourcc   = V4L2_PIX_FMT_NV21,
		.depth    = 12,
	},
	
	{
		.name     = "YUV 4:2:0 planar (Y-Cb-Cr)",
		.fourcc   = V4L2_PIX_FMT_YUV420,
		.depth    = 12,
	},
	{
		.name     = "YVU 4:2:0 planar (Y-Cr-Cb)",
		.fourcc   = V4L2_PIX_FMT_YVU420,
		.depth    = 12,
	},
	{
		.name     = "RGB565 (LE)",
		.fourcc   = V4L2_PIX_FMT_RGB565,
		.depth    = 16,
	},
	{
		.name     = "RGB565 (BE)",
		.fourcc   = V4L2_PIX_FMT_RGB565X,
		.depth    = 16,
	},
	{
		.name     = "RawRGB",
		.fourcc   = V4L2_PIX_FMT_GREY,
		.depth    = 8,
	},
	{
		.name     = "JPEG",
		.fourcc   = V4L2_PIX_FMT_JPEG,
		.depth    = 8,
	},
};
#else
LOCAL struct dcam_format dcam_img_fmt[] = {
	{"4:2:2, packed, YUYV", V4L2_PIX_FMT_YUYV, 16},
	{"4:2:2, packed, YVYU", V4L2_PIX_FMT_YVYU, 16},
	{"4:2:2, packed, UYVY", V4L2_PIX_FMT_UYVY, 16},
	{"4:2:2, packed, VYUY", V4L2_PIX_FMT_VYUY, 16},	
	{"YUV 4:2:2, planar, (Y-Cb-Cr)", V4L2_PIX_FMT_YUV422P, 16},
	{"YUV 4:2:0 planar (Y-CbCr)", V4L2_PIX_FMT_NV12, 12},
	{"YVU 4:2:0 planar (Y-CrCb)", V4L2_PIX_FMT_NV21, 12},
	{"YUV 4:2:0 planar (Y-Cb-Cr)", V4L2_PIX_FMT_YUV420, 12},
	{"YVU 4:2:0 planar (Y-Cr-Cb)", V4L2_PIX_FMT_YVU420, 12},
	{"RGB565 (LE)", V4L2_PIX_FMT_RGB565, 16},
	{"RGB565 (BE)", V4L2_PIX_FMT_RGB565X, 16},
	{"RawRGB", V4L2_PIX_FMT_GREY, 8},
	{"JPEG", V4L2_PIX_FMT_JPEG, 8}
};
#endif

LOCAL struct dcam_format *sprd_v4l2_get_format(struct v4l2_format *f)
{
	struct dcam_format       *fmt;
	uint32_t                 i;

	for (i = 0; i < ARRAY_SIZE(dcam_img_fmt); i++) {
		fmt = &dcam_img_fmt[i];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			break;
	}

	if (unlikely(i == ARRAY_SIZE(dcam_img_fmt)))
		return NULL;

	return &dcam_img_fmt[i];
}

LOCAL uint32_t sprd_v4l2_get_fourcc(struct dcam_path_spec *path)
{
	uint32_t                 fourcc = DCAM_INVALID_FOURCC;

	switch (path->out_fmt) {
	case DCAM_YUV422:
		fourcc = V4L2_PIX_FMT_YUV422P;
		break;
	case DCAM_YUV420:

		if (likely(DCAM_ENDIAN_LITTLE == path->end_sel.uv_endian))
			fourcc = V4L2_PIX_FMT_NV12;
		else 
			fourcc = V4L2_PIX_FMT_NV21;
		break;
	case DCAM_YUV420_3FRAME:
		fourcc = V4L2_PIX_FMT_YUV420;
		break;
	case DCAM_RGB565:
		if (likely(DCAM_ENDIAN_HALFBIG == path->end_sel.uv_endian))
			fourcc = V4L2_PIX_FMT_RGB565;
		else
			fourcc = V4L2_PIX_FMT_RGB565X;
		break;
	case DCAM_RAWRGB:
		fourcc = V4L2_PIX_FMT_GREY;
		break;
	case DCAM_JPEG:
		fourcc = V4L2_PIX_FMT_JPEG;
		break;
	default:
		break;
	}
	return fourcc;
}

LOCAL uint32_t sprd_v4l2_get_deci_factor(uint32_t src_size, uint32_t dst_size)
{
	uint32_t                 factor = 0;

	if (0 == src_size || 0 == dst_size) {
		return factor;
	}

	for (factor = 0; factor < DCAM_CAP_X_DECI_FAC_MAX; factor ++) {
		if (src_size < (uint32_t)(dst_size * (1 << factor))) {
			break;
		}
	}

	return factor;
}

LOCAL uint32_t sprd_v4l2_endian_sel(uint32_t fourcc, struct dcam_path_spec *path)
{
	uint32_t                 depth = 0;

	if (V4L2_PIX_FMT_YUV422P == fourcc ||
		V4L2_PIX_FMT_RGB565 == fourcc ||
		V4L2_PIX_FMT_RGB565X == fourcc) {
		depth = 16;
		if (V4L2_PIX_FMT_YUV422P == fourcc) {
			path->out_fmt = DCAM_YUV422;
		} else {
			path->out_fmt = DCAM_RGB565;
			if (V4L2_PIX_FMT_RGB565 == fourcc) {
				path->end_sel.y_endian = DCAM_ENDIAN_HALFBIG;
			} else {
				path->end_sel.y_endian = DCAM_ENDIAN_BIG;
			}
		}
	} else {
		depth = 12;
		if (V4L2_PIX_FMT_YUV420 == fourcc ||
			V4L2_PIX_FMT_YVU420 == fourcc) {
			path->out_fmt = DCAM_YUV420_3FRAME;
		} else {
			path->out_fmt = DCAM_YUV420;
			if (V4L2_PIX_FMT_NV12 == fourcc) {
				path->end_sel.uv_endian = DCAM_ENDIAN_LITTLE;
			} else {
				path->end_sel.uv_endian = DCAM_ENDIAN_HALFBIG;
			}
		}
	}

	return depth;
}

LOCAL int sprd_v4l2_check_path0_cap(uint32_t fourcc,
					struct v4l2_format *f,
					struct dcam_info   *info)
{
	struct dcam_path_spec    *path = &info->dcam_path[DCAM_PATH0];

	DCAM_TRACE("V4L2: check format for path0 \n");
	if (unlikely(V4L2_BUF_TYPE_VIDEO_OUTPUT != f->type)) {
		printk("V4L2: unsupported video type, 0x%x \n", f->type);
		return -EINVAL;
	}

	path->frm_type = f->type;
	path->is_work = 0;

	printk("V4L2: testest  fourcc=%d , info->is_loose : %d \n", fourcc, info->is_loose);

	switch (fourcc) {
	case V4L2_PIX_FMT_GREY:
		path->out_fmt = info->is_loose; 
		if (1 == info->is_loose )
			path->end_sel.y_endian = 2;
		else 
			path->end_sel.y_endian = 0;
		break;

	case V4L2_PIX_FMT_JPEG:
		path->out_fmt = 0; 
		break;

	default:
		printk("V4L2: unsupported image format for path0 0x%x \n", fourcc);
		return -EINVAL;
	}

	DCAM_TRACE("V4L2: check format for path0: out_fmt=%d, is_loose=%d \n", path->out_fmt, info->is_loose);

	path->is_work = 1;

	return 0;
}

LOCAL int sprd_v4l2_check_path1_cap(uint32_t fourcc,
					struct v4l2_format *f,
					struct dcam_info   *info)
{
	uint32_t                 maxw, maxh, tempw,temph;
	uint32_t                 depth_pixel = 0;
	struct dcam_path_spec    *path = &info->dcam_path[DCAM_PATH1];
	uint32_t                 need_recal = 0;
	
	DCAM_TRACE("V4L2: check format for path1 \n");
	if (unlikely(V4L2_BUF_TYPE_VIDEO_CAPTURE != f->type)) {
		printk("V4L2: unsupported video type, %d \n", f->type);
		return -EINVAL;
	}

	path->frm_type = f->type;
	path->is_from_isp = f->fmt.pix.priv;
	path->end_sel.y_endian = DCAM_ENDIAN_LITTLE;
	path->end_sel.uv_endian = DCAM_ENDIAN_LITTLE;
	path->is_work = 0;
	path->pixel_depth = 0;
	path->img_deci.x_factor = 0;
	path->img_deci.y_factor = 0;
	tempw = path->in_rect.w;
	temph = path->in_rect.h;
	info->img_deci.x_factor = 0;
	f->fmt.pix.sizeimage = 0;

	switch (fourcc) {
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_JPEG:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		if (unlikely(f->fmt.pix.width != tempw ||
			f->fmt.pix.height != temph)) {
			
			printk("V4L2: Can not scaling for this image fmt src %d %d, dst %d %d \n",
					tempw,
					temph,
					f->fmt.pix.width,
					f->fmt.pix.height);
			return -EINVAL;
		}

		if (unlikely(DCAM_CAP_MODE_JPEG != info->sn_mode &&
			V4L2_PIX_FMT_JPEG == fourcc)) {
			
			printk("V4L2: It's not JPEG sensor \n");
			return -EINVAL;
		}

		if (unlikely(DCAM_CAP_MODE_RAWRGB != info->sn_mode &&
			V4L2_PIX_FMT_GREY == fourcc)) {
			
			printk("V4L2: It's not RawRGB sensor \n");
			return -EINVAL;
		}

		if (V4L2_PIX_FMT_GREY == fourcc) {
			if (DCAM_CAP_IF_CSI2 == info->if_mode && 0 == info->is_loose) {
				depth_pixel = 10;
			} else {
				depth_pixel = 16;
			}
			DCAM_TRACE("V4L2: RawRGB sensor, %d %d \n", info->is_loose, depth_pixel);
		} else if (V4L2_PIX_FMT_JPEG == fourcc) {
			depth_pixel = 8;
		} else {
			depth_pixel = 16;
		}

		if (V4L2_PIX_FMT_GREY == fourcc) {
			path->out_fmt = DCAM_RAWRGB;
			path->end_sel.y_endian = DCAM_ENDIAN_BIG;
		} else {
			path->out_fmt = DCAM_JPEG;
		}
		break;
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		if (DCAM_CAP_MODE_RAWRGB == info->sn_mode &&
			path->is_from_isp) {
			if (unlikely(info->cap_out_size.w > DCAM_ISP_LINE_BUF_LENGTH)) {
				if (0 == info->if_mode) {
					
					printk("V4L2: CCIR CAP, no bining for this path, %d %d \n",
						f->fmt.pix.width,
						f->fmt.pix.height);
					return -EINVAL;
				} else {
					
					DCAM_TRACE("Need Binning \n");
					tempw = tempw >> 1;
					if (unlikely(tempw > DCAM_ISP_LINE_BUF_LENGTH)) {
						printk("V4L2: the width is out of ISP line buffer, %d %d \n",
							tempw,
							DCAM_ISP_LINE_BUF_LENGTH);
						return -EINVAL;
					}
					info->img_deci.x_factor = 1;
					f->fmt.pix.sizeimage = 1;
					DCAM_TRACE("x_factor, %d \n", info->img_deci.x_factor);
					path->in_size.w = path->in_size.w >> 1;
					path->in_rect.x = path->in_rect.x >> 1;
					path->in_rect.w = path->in_rect.w >> 1;
					path->in_rect.w = path->in_rect.w & (~3);
				}
			}
		}
		if (DCAM_CAP_MODE_YUV == info->sn_mode ||
			DCAM_CAP_MODE_SPI == info->sn_mode) {
			if (tempw != f->fmt.pix.width || temph != f->fmt.pix.height) {
				
				if (unlikely(f->fmt.pix.width > DCAM_PATH1_LINE_BUF_LENGTH)) {
					
					printk("V4L2: the output width %d can not be more than %d \n",
						f->fmt.pix.width,
						DCAM_PATH1_LINE_BUF_LENGTH);
					return -EINVAL;
				}

				
				maxw = tempw * DCAM_SC_COEFF_UP_MAX;
				maxh = temph * DCAM_SC_COEFF_UP_MAX;
				if (unlikely(f->fmt.pix.width > maxw || f->fmt.pix.height > maxh)) {
					
					printk("V4L2: the output size is too large, %d %d \n",
						f->fmt.pix.width,
						f->fmt.pix.height);
					return -EINVAL;
				}

				
				maxw = f->fmt.pix.width * DCAM_SC_COEFF_DOWN_MAX;
				if (unlikely( tempw > maxw)) {
					path->img_deci.x_factor = sprd_v4l2_get_deci_factor(tempw, maxw);
					if (path->img_deci.x_factor > DCAM_PATH_DECI_FAC_MAX) {
						printk("V4L2: the output size is too small, %d %d \n",
							f->fmt.pix.width,
							f->fmt.pix.height);
						return -EINVAL;
					}
				}

				maxh = f->fmt.pix.height * DCAM_SC_COEFF_DOWN_MAX;
				if (unlikely(temph > maxh)) {
					path->img_deci.y_factor = sprd_v4l2_get_deci_factor(temph, maxh);
					if (path->img_deci.y_factor > DCAM_PATH_DECI_FAC_MAX) {
						printk("V4L2: the output size is too small, %d %d \n",
							f->fmt.pix.width,
							f->fmt.pix.height);
						return -EINVAL;
					}
				}

				if (path->img_deci.x_factor) {
					tempw           = path->in_rect.w >> 1;
					need_recal      = 1;
				}

				if (path->img_deci.y_factor) {
					temph           = path->in_rect.h >> 1;
					need_recal      = 1;
				}

				if (need_recal && (tempw != f->fmt.pix.width || temph != f->fmt.pix.height)) {
					
					if (unlikely(f->fmt.pix.width > DCAM_PATH1_LINE_BUF_LENGTH)) {
						
						printk("V4L2: the output width %d can not be more than %d \n",
							f->fmt.pix.width,
							DCAM_PATH1_LINE_BUF_LENGTH);
						return -EINVAL;
					}
				}
			}
		} else if (DCAM_CAP_MODE_RAWRGB == info->sn_mode) {
			if (path->is_from_isp) {

				if (tempw != f->fmt.pix.width ||
					temph != f->fmt.pix.height) {
					
					maxw = f->fmt.pix.width * DCAM_SC_COEFF_DOWN_MAX;
					maxw = maxw * (1 << DCAM_PATH_DECI_FAC_MAX);
					maxh = f->fmt.pix.height * DCAM_SC_COEFF_DOWN_MAX;
					maxh = maxh * (1 << DCAM_PATH_DECI_FAC_MAX);
					if (unlikely(tempw > maxw || temph > maxh)) {
						
						printk("V4L2: the output size is too small, %d %d \n",
								f->fmt.pix.width,
								f->fmt.pix.height);
						return -EINVAL;
					}

					if (unlikely(f->fmt.pix.width > DCAM_PATH1_LINE_BUF_LENGTH)) {
						
						printk("V4L2: the output width %d can not be more than %d \n",
							f->fmt.pix.width,
							DCAM_PATH1_LINE_BUF_LENGTH);
						return -EINVAL;
					}

					maxw = tempw * DCAM_SC_COEFF_UP_MAX;
					maxh = temph * DCAM_SC_COEFF_UP_MAX;
					if (unlikely(f->fmt.pix.width > maxw || f->fmt.pix.height > maxh)) {
						
						printk("V4L2: the output size is too large, %d %d \n",
							f->fmt.pix.width,
							f->fmt.pix.height);
						return -EINVAL;
					}
				}
			} else {
				
				printk("V4L2: RawRGB sensor, no ISP, format 0x%x can't be supported \n",
						fourcc);
				return -EINVAL;
			}
		}

		depth_pixel = sprd_v4l2_endian_sel(fourcc, path);
		break;
	default:
		printk("V4L2: unsupported image format for path2 0x%x \n", fourcc);
		return -EINVAL;
	}

	path->pixel_depth = depth_pixel;
	f->fmt.pix.bytesperline = (f->fmt.pix.width * depth_pixel) >> 3;
	path->out_size.w = f->fmt.pix.width;
	path->out_size.h = f->fmt.pix.height;
	path->is_work = 1;
	return 0;
}

LOCAL int sprd_v4l2_check_path2_cap(uint32_t fourcc,
					struct v4l2_format *f,
					struct dcam_info   *info)
{
	uint32_t                 maxw, maxh, tempw,temph;
	uint32_t                 depth_pixel = 0;
	struct dcam_path_spec    *path = &info->dcam_path[DCAM_PATH2];
	uint32_t                 need_recal = 0;

	

	DCAM_TRACE("V4L2: check format for path2 \n");
	if (unlikely(V4L2_BUF_TYPE_PRIVATE != f->type)) {
		printk("V4L2: unsupported video type, %d \n", f->type);
		return -EINVAL;
	}

	path->frm_type = f->type;
	path->is_from_isp = f->fmt.pix.priv;
	path->end_sel.y_endian = DCAM_ENDIAN_LITTLE;
	path->end_sel.uv_endian = DCAM_ENDIAN_LITTLE;
	path->is_work = 0;
	path->pixel_depth = 0;
	if (info->img_deci.x_factor) {
		tempw = path->in_rect.w >> 1;
		path->in_size.w = path->in_size.w >> 1;
		path->in_rect.w = path->in_rect.w >> 1;
		path->in_rect.x = path->in_rect.x >> 1;
	} else {
		tempw = path->in_rect.w;
	}
	temph = path->in_rect.h;
	switch (fourcc) {
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:

		if (DCAM_CAP_MODE_JPEG == info->sn_mode ||
			info->sn_mode >= DCAM_CAP_MODE_MAX) {
			return -EINVAL;
		}

		if (DCAM_CAP_MODE_RAWRGB == info->sn_mode) {
			path->is_from_isp = 1;
		}

		if (DCAM_CAP_MODE_YUV == info->sn_mode ||
			DCAM_CAP_MODE_SPI == info->sn_mode) {
			if (tempw != f->fmt.pix.width || temph != f->fmt.pix.height) {
				
				if (unlikely(f->fmt.pix.width > DCAM_PATH2_LINE_BUF_LENGTH)) {
					
					printk("V4L2: the output width %d can not be more than %d \n",
						f->fmt.pix.width,
						DCAM_PATH2_LINE_BUF_LENGTH);
					return -EINVAL;
				}

				
				maxw = tempw * DCAM_SC_COEFF_UP_MAX;
				maxh = temph * DCAM_SC_COEFF_UP_MAX;
				if (unlikely(f->fmt.pix.width > maxw || f->fmt.pix.height > maxh)) {
					
					printk("V4L2: the output size is too large, %d %d \n",
						f->fmt.pix.width,
						f->fmt.pix.height);
					return -EINVAL;
				}

				
				maxw = f->fmt.pix.width * DCAM_SC_COEFF_DOWN_MAX;
				if (unlikely( tempw > maxw)) {
					path->img_deci.x_factor = sprd_v4l2_get_deci_factor(tempw, maxw);
					if (path->img_deci.x_factor > DCAM_PATH_DECI_FAC_MAX) {
						printk("V4L2: the output size is too small, %d %d \n",
							f->fmt.pix.width,
							f->fmt.pix.height);
						return -EINVAL;
					}
				}

				maxh = f->fmt.pix.height * DCAM_SC_COEFF_DOWN_MAX;
				if (unlikely(temph > maxh)) {
					path->img_deci.y_factor = sprd_v4l2_get_deci_factor(temph, maxh);
					if (path->img_deci.y_factor > DCAM_PATH_DECI_FAC_MAX) {
						printk("V4L2: the output size is too small, %d %d \n",
							f->fmt.pix.width,
							f->fmt.pix.height);
						return -EINVAL;
					}
				}

				if (path->img_deci.x_factor) {
					tempw           = path->in_rect.w >> 1;
					need_recal      = 1;
				}

				if (path->img_deci.y_factor) {
					temph           = path->in_rect.h >> 1;
					need_recal      = 1;
				}

				if (need_recal && (tempw != f->fmt.pix.width || temph != f->fmt.pix.height)) {
					
					if (unlikely(f->fmt.pix.width > DCAM_PATH2_LINE_BUF_LENGTH)) {
						
						printk("V4L2: the output width %d can not be more than %d \n",
							f->fmt.pix.width,
							DCAM_PATH2_LINE_BUF_LENGTH);
						return -EINVAL;
					}
				}
			}
		} else if (DCAM_CAP_MODE_RAWRGB == info->sn_mode) {
			if (path->is_from_isp) {

				if (tempw != f->fmt.pix.width ||
					temph != f->fmt.pix.height) {
					
					maxw = f->fmt.pix.width * DCAM_SC_COEFF_DOWN_MAX;
					maxw = maxw * (1 << DCAM_PATH_DECI_FAC_MAX);
					maxh = f->fmt.pix.height * DCAM_SC_COEFF_DOWN_MAX;
					maxh = maxh * (1 << DCAM_PATH_DECI_FAC_MAX);
					if (unlikely(tempw > maxw || temph > maxh)) {
						
						printk("V4L2: the output size is too small, %d %d \n",
								f->fmt.pix.width,
								f->fmt.pix.height);
						return -EINVAL;
					}

					if (unlikely(f->fmt.pix.width > DCAM_PATH2_LINE_BUF_LENGTH)) {
						
						printk("V4L2: the output width %d can not be more than %d \n",
							f->fmt.pix.width,
							DCAM_PATH2_LINE_BUF_LENGTH);
						return -EINVAL;
					}

					maxw = tempw * DCAM_SC_COEFF_UP_MAX;
					maxh = temph * DCAM_SC_COEFF_UP_MAX;
					if (unlikely(f->fmt.pix.width > maxw || f->fmt.pix.height > maxh)) {
						
						printk("V4L2: the output size is too large, %d %d \n",
							f->fmt.pix.width,
							f->fmt.pix.height);
						return -EINVAL;
					}
				}
			} else {
				
				printk("V4L2: RawRGB sensor, no ISP, format 0x%x can't be supported \n",
						fourcc);
				return -EINVAL;
			}
		}


		depth_pixel = sprd_v4l2_endian_sel(fourcc, path);
		path->end_sel.uv_endian = DCAM_ENDIAN_LITTLE; 
		break;
	default:
		printk("V4L2: unsupported image format for path2 0x%x \n", fourcc);
		return -EINVAL;
	}

	f->fmt.pix.priv = path->is_from_isp;
	path->pixel_depth = depth_pixel;
	f->fmt.pix.bytesperline = (f->fmt.pix.width * depth_pixel) >> 3;
	path->out_size.w = f->fmt.pix.width;
	path->out_size.h = f->fmt.pix.height;
	path->is_work = 1;

	return 0;
}

LOCAL int sprd_v4l2_cap_cfg(struct dcam_info* info)
{
	int                      ret = DCAM_RTN_SUCCESS;
	uint32_t                 param = 0;

	if (NULL == info)
		return -EINVAL;

	ret = dcam_cap_cfg(DCAM_CAP_SYNC_POL, &info->sync_pol);
	V4L2_RTN_IF_ERR(ret);

	if ((info->dcam_path[DCAM_PATH1].is_work && info->dcam_path[DCAM_PATH1].is_from_isp) || 
		(info->dcam_path[DCAM_PATH2].is_work && info->dcam_path[DCAM_PATH2].is_from_isp)) {
		param = 1;
	} else {
		param = 0;
	}
	ret = dcam_cap_cfg(DCAM_CAP_TO_ISP, &param);
	V4L2_RTN_IF_ERR(ret);

	ret = dcam_cap_cfg(DCAM_CAP_YUV_TYPE, &info->yuv_ptn);
	V4L2_RTN_IF_ERR(ret);

	ret = dcam_cap_cfg(DCAM_CAP_DATA_BITS, &info->data_bits);
	V4L2_RTN_IF_ERR(ret);

	if (DCAM_CAP_MODE_RAWRGB == info->sn_mode &&
		DCAM_CAP_IF_CSI2 == info->if_mode)
	{
		ret = dcam_cap_cfg(DCAM_CAP_DATA_PACKET, &info->is_loose);
		V4L2_RTN_IF_ERR(ret);
	}

	ret = dcam_cap_cfg(DCAM_CAP_FRM_DECI, &info->frm_deci);
	V4L2_RTN_IF_ERR(ret);	

	ret = dcam_cap_cfg(DCAM_CAP_INPUT_RECT, &info->cap_in_rect);
	V4L2_RTN_IF_ERR(ret);

	ret = dcam_cap_cfg(DCAM_CAP_FRM_COUNT_CLR, NULL);
	V4L2_RTN_IF_ERR(ret);

	ret = dcam_cap_cfg(DCAM_CAP_PRE_SKIP_CNT, &info->skip_number);
	V4L2_RTN_IF_ERR(ret);

	ret = dcam_cap_cfg(DCAM_CAP_SAMPLE_MODE, &info->capture_mode);
	V4L2_RTN_IF_ERR(ret);

	ret = dcam_cap_cfg(DCAM_CAP_IMAGE_XY_DECI, &info->img_deci);
	
exit:
	return ret;
}

LOCAL int sprd_v4l2_tx_done(struct dcam_frame *frame, void* param)
{
	int                      ret = DCAM_RTN_SUCCESS;
	struct dcam_dev          *dev = (struct dcam_dev*)param;
	struct dcam_path_spec    *path;
	struct dcam_node         node;
	uint32_t                 fmr_index;

	if (NULL == frame || NULL == param || 0 == atomic_read(&dev->stream_on))
		return -EINVAL;

	
	if(V4L2_BUF_TYPE_VIDEO_CAPTURE == frame->type){
		if(v4l2_path1_cnt++ <= 10){
			printk("cam p1 %d\n", v4l2_path1_cnt);
		}else{
			atomic_set(&dev->run_flag, 1);
		}
	}
	node.irq_flag = V4L2_TX_DONE;
	node.f_type   = frame->type;
	node.index    = frame->fid;
	node.height   = frame->height;

	DCAM_TRACE_LOW("V4L2: sprd_v4l2_tx_done, flag 0x%x type 0x%x index 0x%x \n",
		node.irq_flag, node.f_type, node.index);

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE == frame->type) {
		path = &dev->dcam_cxt.dcam_path[DCAM_PATH1];

	} else if (V4L2_BUF_TYPE_PRIVATE == frame->type) {
		path = &dev->dcam_cxt.dcam_path[DCAM_PATH2];
		if (DCAM_CAP_MODE_JPEG == dev->dcam_cxt.sn_mode) {
			dcam_cap_get_info(DCAM_CAP_JPEG_GET_LENGTH, &node.reserved);
			DCAM_TRACE("V4L2: sprd_v4l2_tx_done, JPEG length 0x%x \n", node.reserved);
		}
	} else {
		path = &dev->dcam_cxt.dcam_path[DCAM_PATH0];
	}
	fmr_index = frame->fid - path->frm_id_base;
	if (fmr_index >= path->frm_cnt_act) {
		DCAM_TRACE_LOW("V4L2: sprd_v4l2_tx_done, index error %d, actually count %d \n",
			fmr_index,
			path->frm_id_base);
	}
	path->frm_ptr[fmr_index] = frame;
	ret = sprd_v4l2_queue_write(&dev->queue, &node);
	if (ret)
		return ret;

	up(&dev->irq_sem);
	return ret;
}

LOCAL int sprd_v4l2_tx_error(struct dcam_frame *frame, void* param)
{
	int                      ret = DCAM_RTN_SUCCESS;
	struct dcam_dev          *dev = (struct dcam_dev*)param;
	struct dcam_node         node;
	printk("V4L2: tx_error f 0x%x p 0x%x strm_on %d\n", (uint32_t)frame, (uint32_t)param, atomic_read(&dev->stream_on));

	if (NULL == param || 0 == atomic_read(&dev->stream_on))
		return -EINVAL;

	atomic_set(&dev->run_flag, 1);

	memset((void*)&node, 0, sizeof(struct dcam_node));

	node.irq_flag = V4L2_TX_ERR;
	ret = sprd_v4l2_queue_write(&dev->queue, &node);
	if (ret) {
		printk("v4l2 q err \n");
		return ret;
	}

	up(&dev->irq_sem);
	return ret;
}

LOCAL int sprd_v4l2_no_mem(struct dcam_frame *frame, void* param)
{
	int                      ret = DCAM_RTN_SUCCESS;
	struct dcam_dev          *dev = (struct dcam_dev*)param;
	struct dcam_node         node;
	printk("V4L2: no mem f 0x%x p 0x%x strm_on %d\n", (uint32_t)frame, (uint32_t)param, atomic_read(&dev->stream_on));

	if (NULL == param || 0 == atomic_read(&dev->stream_on))
		return -EINVAL;

	atomic_set(&dev->run_flag, 1);

	memset((void*)&node, 0, sizeof(struct dcam_node));
	node.irq_flag = V4L2_NO_MEM;
	ret = sprd_v4l2_queue_write(&dev->queue, &node);
	if (ret) {
		printk("v4l2 q err \n");
		return ret;
	}

	up(&dev->irq_sem);

	return ret;
}

LOCAL int sprd_v4l2_csi2_error(uint32_t err_id, uint32_t err_status, void* u_data)
{
	int                      ret = DCAM_RTN_SUCCESS;
	struct dcam_dev          *dev = (struct dcam_dev*)u_data;
	struct dcam_node         node;
	printk("V4L2: csi2_error, %d 0x%x u 0x%x strm_on %d\n", err_id, err_status, (uint32_t)u_data, atomic_read(&dev->stream_on));

	(void)err_id; (void)err_status;
	if (NULL == u_data || 0 == atomic_read(&dev->stream_on))
		return -EINVAL;

	atomic_set(&dev->run_flag, 1);

	memset((void*)&node, 0, sizeof(struct dcam_node));

	node.irq_flag = V4L2_CSI2_ERR;
	ret = sprd_v4l2_queue_write(&dev->queue, &node);
	if (ret) {
		printk("v4l2 q err \n");
		return ret;
	}

	up(&dev->irq_sem);

	return ret;

}

LOCAL int sprd_v4l2_tx_stop(void* param)
{
	int                      ret = DCAM_RTN_SUCCESS;
	struct dcam_dev          *dev = (struct dcam_dev*)param;
	struct dcam_node         node;

	node.irq_flag = V4L2_TX_STOP;
	ret = sprd_v4l2_queue_write(&dev->queue, &node);
	if (ret)
		return ret;

	up(&dev->irq_sem);

	return ret;
}
LOCAL int sprd_v4l2_path0_cfg(path_cfg_func path_cfg,
				struct dcam_path_spec* path_spec)
{
	int                      ret = DCAM_RTN_SUCCESS;
	uint32_t                 param, i;

	if (NULL == path_cfg || NULL == path_spec)
		return -EINVAL;

	ret = path_cfg(DCAM_PATH_INPUT_SIZE, &path_spec->in_size);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_OUTPUT_FORMAT, &path_spec->out_fmt);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_FRAME_BASE_ID, &path_spec->frm_id_base);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_FRAME_TYPE, &path_spec->frm_type);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_FRM_DECI, &path_spec->path_frm_deci);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_DATA_ENDIAN, &path_spec->end_sel);
	V4L2_RTN_IF_ERR(ret);

	for (i = 0; i < path_spec->frm_cnt_act; i++) {
		ret = path_cfg(DCAM_PATH_OUTPUT_ADDR, &path_spec->frm_addr[i]);
		V4L2_RTN_IF_ERR(ret);
	}

	param = 1;
	ret = path_cfg(DCAM_PATH_ENABLE, &param);

exit:
	return ret;
}

LOCAL int sprd_v4l2_path_cfg(path_cfg_func path_cfg,
				struct dcam_path_spec* path_spec)
{
	int                      ret = DCAM_RTN_SUCCESS;
	uint32_t                 param, i;

	if (NULL == path_cfg || NULL == path_spec)
		return -EINVAL;

	if (path_spec->is_from_isp) {
		param = 1;
	} else {
		param = 0;
	}
	ret = path_cfg(DCAM_PATH_SRC_SEL, &param);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_INPUT_SIZE, &path_spec->in_size);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_INPUT_RECT, &path_spec->in_rect);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_OUTPUT_SIZE, &path_spec->out_size);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_OUTPUT_FORMAT, &path_spec->out_fmt);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_FRAME_BASE_ID, &path_spec->frm_id_base);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_FRAME_TYPE, &path_spec->frm_type);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_DATA_ENDIAN, &path_spec->end_sel);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_FRM_DECI, &path_spec->path_frm_deci);
	V4L2_RTN_IF_ERR(ret);

	for (i = 0; i < path_spec->frm_cnt_act; i++) {
		ret = path_cfg(DCAM_PATH_OUTPUT_ADDR, &path_spec->frm_addr[i]);
		V4L2_RTN_IF_ERR(ret);
	}

	param = 1;
	ret = path_cfg(DCAM_PATH_ENABLE, &param);

exit:
	return ret;
}

LOCAL int sprd_v4l2_path_lightly_cfg(path_cfg_func path_cfg,
				struct dcam_path_spec* path_spec)
{
	int                      ret = DCAM_RTN_SUCCESS;
	uint32_t                 param, i;

	if (NULL == path_cfg || NULL == path_spec)
		return -EINVAL;

	if (path_spec->is_from_isp) {
		param = 1;
	} else {
		param = 0;
	}
	ret = path_cfg(DCAM_PATH_SRC_SEL, &param);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_INPUT_SIZE, &path_spec->in_size);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_INPUT_RECT, &path_spec->in_rect);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_OUTPUT_SIZE, &path_spec->out_size);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_OUTPUT_FORMAT, &path_spec->out_fmt);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_FRAME_BASE_ID, &path_spec->frm_id_base);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_FRAME_TYPE, &path_spec->frm_type);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_DATA_ENDIAN, &path_spec->end_sel);
	V4L2_RTN_IF_ERR(ret);

exit:
	return ret;
}


LOCAL int sprd_v4l2_local_deinit(struct dcam_dev *dev)
{
	struct dcam_path_spec    *path = &dev->dcam_cxt.dcam_path[DCAM_PATH1];

	if (unlikely(NULL == dev || NULL == path)) {
		return -EINVAL;
	}
	path->is_work = 0;
	path->frm_cnt_act = 0;

	path = &dev->dcam_cxt.dcam_path[DCAM_PATH2];
	if (unlikely(NULL == path))
		return -EINVAL;
	path->is_work = 0;
	path->frm_cnt_act = 0;

	path = &dev->dcam_cxt.dcam_path[DCAM_PATH0];
	if (unlikely(NULL == path))
		return -EINVAL;
	path->is_work = 0;
	path->frm_cnt_act = 0;

	DCAM_TRACE("V4L2: sprd_v4l2_local_deinit, frm_cnt_act %d \n", path->frm_cnt_act);

	return 0;
}

LOCAL int sprd_v4l2_queue_init(struct dcam_queue *queue)
{
	if (NULL == queue)
		return -EINVAL;

	memset(queue, 0, sizeof(*queue));
	queue->write = &queue->node[0];
	queue->read  = &queue->node[0];

	return 0;
}

LOCAL int sprd_v4l2_queue_write(struct dcam_queue *queue, struct dcam_node *node)
{
	struct dcam_node         *ori_node;

	if (NULL == queue || NULL == node)
		return -EINVAL;

	ori_node = queue->write;

	*queue->write++ = *node;
	if (queue->write > &queue->node[DCAM_QUEUE_LENGTH-1]) {
		queue->write = &queue->node[0];
	}

	if (queue->write == queue->read) {
		queue->write = ori_node;
		printk("V4L2: warning, queue is full, cannot write, flag 0x%x type 0x%x index 0x%x \n",
			node->irq_flag, node->f_type, node->index);
	}

	return 0;
}

LOCAL int sprd_v4l2_queue_read(struct dcam_queue *queue, struct dcam_node *node)
{
	int                      ret = DCAM_RTN_SUCCESS;

	if (NULL == queue || NULL == node)
		return -EINVAL;

	if (queue->read != queue->write) {
		*node = *queue->read++;
		if (queue->read > &queue->node[DCAM_QUEUE_LENGTH-1]) {
			queue->read = &queue->node[0];
		}
	} else {
		ret = EAGAIN;
	}

	return ret;
}

LOCAL int sprd_v4l2_get_path_index(uint32_t channel_id)
{
	int path_index;
	if(DCAM_PATH0 == channel_id){
		path_index = DCAM_PATH_IDX_0;
	}else if(DCAM_PATH1 == channel_id){
		path_index = DCAM_PATH_IDX_1;
	}else if(DCAM_PATH2 == channel_id){
		path_index = DCAM_PATH_IDX_2;
	}else{
		path_index = DCAM_PATH_IDX_NONE;
		printk("V4L2: get path index error, invalid channel_id=0x%x \n", channel_id);
	}
	return path_index;
}

LOCAL int v4l2_g_parm(struct file *file,
			void *priv,
			struct v4l2_streamparm *streamparm)
{
	struct dcam_dev          *dev = video_drvdata(file);

	DCAM_TRACE("V4L2: v4l2_g_parm, capability 0x%x \n", streamparm->parm.capture.capability);

	streamparm->parm.capture.capturemode  = dev->dcam_cxt.capture_mode;
	streamparm->parm.capture.reserved[0]  = dev->dcam_cxt.skip_number;
	streamparm->parm.capture.reserved[2] = dev->dcam_cxt.cap_in_size.w;
	streamparm->parm.capture.reserved[3] = dev->dcam_cxt.cap_in_size.h;

	return 0;
}


enum dcam_parm_id {
	CAPTURE_MODE = 0x1000,
	CAPTURE_SKIP_NUM,
	CAPTURE_SENSOR_SIZE,
	CAPTURE_FRM_ID_BASE,
	
	PATH_FRM_DECI = 0x2000,
	PATH_PAUSE    = 0x2001,
	PATH_RESUME   = 0x2002,
};
LOCAL int v4l2_s_parm(struct file *file,
			void *priv,
			struct v4l2_streamparm *streamparm)
{
	struct dcam_dev          *dev = video_drvdata(file);
	uint32_t                 frm_id;
	uint32_t                 channel_id;
	struct dcam_path_spec    *path = NULL;
	int                      ret = DCAM_RTN_SUCCESS;

	DCAM_TRACE("V4L2: v4l2_s_parm, ability 0x%x \n", streamparm->parm.capture.capability);

	mutex_lock(&dev->dcam_mutex);
	
	switch (streamparm->parm.capture.capability) {
	case CAPTURE_MODE:
		dev->dcam_cxt.capture_mode = streamparm->parm.capture.capturemode;
		DCAM_TRACE("V4L2: capture mode %d \n", dev->dcam_cxt.capture_mode);
		break;
	case CAPTURE_SKIP_NUM:
		dev->dcam_cxt.skip_number  = streamparm->parm.capture.reserved[0];
		DCAM_TRACE("V4L2: cap skip number %d \n", dev->dcam_cxt.skip_number);
		break;
	case CAPTURE_SENSOR_SIZE:
		dev->dcam_cxt.cap_in_size.w  = streamparm->parm.capture.reserved[2];
		dev->dcam_cxt.cap_in_size.h  = streamparm->parm.capture.reserved[3];
		dev->dcam_cxt.cap_in_rect.x  = 0;
		dev->dcam_cxt.cap_in_rect.y  = 0;
		dev->dcam_cxt.cap_in_rect.w  = dev->dcam_cxt.cap_in_size.w;
		dev->dcam_cxt.cap_in_rect.h  = dev->dcam_cxt.cap_in_size.h;
		dev->dcam_cxt.cap_out_size.w = dev->dcam_cxt.cap_in_rect.w;
		dev->dcam_cxt.cap_out_size.h = dev->dcam_cxt.cap_in_rect.h;
		DCAM_TRACE("V4L2: sensor size %d %d \n",
			dev->dcam_cxt.cap_in_size.w,
			dev->dcam_cxt.cap_in_size.h);
		rawchip_image_width = dev->dcam_cxt.cap_in_size.w;
		break;
	case CAPTURE_FRM_ID_BASE:
		frm_id = streamparm->parm.capture.reserved[1];

		if (DCAM_PATH1 == streamparm->parm.capture.extendedmode) {
			dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_id_base = frm_id;
		} else if (DCAM_PATH2 == streamparm->parm.capture.extendedmode) {
			dev->dcam_cxt.dcam_path[DCAM_PATH2].frm_id_base = frm_id;
		} else if (DCAM_PATH0 == streamparm->parm.capture.extendedmode) {
			dev->dcam_cxt.dcam_path[DCAM_PATH0].frm_id_base = frm_id;
		} else {
			printk("V4L2: Wrong channel ID, %d  \n", streamparm->parm.capture.extendedmode);
		}
		DCAM_TRACE("V4L2: channel %d, base id 0x%x \n",
			streamparm->parm.capture.extendedmode,
			frm_id);
		break;

	case PATH_FRM_DECI:
		channel_id = streamparm->parm.capture.reserved[0];
		path = &dev->dcam_cxt.dcam_path[channel_id];
		
		path->path_frm_deci = 0;
		DCAM_TRACE("V4L2: channel %d, frm_deci=%d \n", channel_id, path->path_frm_deci);
		break;

	case PATH_PAUSE:
		channel_id = streamparm->parm.capture.reserved[0];
		sprd_v4l2_streampause(file, channel_id);
		break;

	case PATH_RESUME:
		channel_id = streamparm->parm.capture.reserved[0];
		sprd_v4l2_streamresume(file, channel_id);
		break;
		
	default:
		printk("V4L2: unsupported parm_id, 0x%x \n", streamparm->parm.capture.capability);
		ret = -EINVAL;
		break;

	}

	mutex_unlock(&dev->dcam_mutex);

	return ret;
}

LOCAL int v4l2_querycap(struct file *file,
			void  *priv,
			struct v4l2_capability *cap)
{
	struct dcam_dev          *dev = video_drvdata(file);

	DCAM_TRACE("V4L2: v4l2_querycap \n");

	strcpy(cap->driver, DCAM_MODULE_NAME);
	strcpy(cap->card, DCAM_MODULE_NAME);
	strlcpy(cap->bus_info, dev->v4l2_dev.name, sizeof(cap->bus_info));
	cap->version = DCAM_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

LOCAL int v4l2_cropcap(struct file *file, 
			void  *priv,
			struct v4l2_cropcap *cc)
{
	struct dcam_dev          *dev = video_drvdata(file);

	if (unlikely(cc->type != V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -EINVAL;

	DCAM_TRACE("V4L2: v4l2_cropcap %d %d %d %d \n",
		dev->dcam_cxt.cap_in_rect.x,
		dev->dcam_cxt.cap_in_rect.y,
		dev->dcam_cxt.cap_in_rect.w,
		dev->dcam_cxt.cap_in_rect.h);

	cc->bounds.left   = (int)dev->dcam_cxt.cap_in_rect.x;
	cc->bounds.top    = (int)dev->dcam_cxt.cap_in_rect.y;
	cc->bounds.width  = (int)dev->dcam_cxt.cap_in_rect.w;
	cc->bounds.height = (int)dev->dcam_cxt.cap_in_rect.h;
	cc->defrect       = cc->bounds;

	cc->pixelaspect.numerator   = dev->dcam_cxt.cap_in_rect.h;
	cc->pixelaspect.denominator = dev->dcam_cxt.cap_in_rect.w;

	return 0;
}

LOCAL int v4l2_s_crop(struct file *file, 
			void  *priv,
			struct v4l2_crop *crop)
{
	struct dcam_dev          *dev = video_drvdata(file);
	struct dcam_rect         *input_rect;
	struct dcam_size         *input_size;
	uint32_t                 i;

	if (unlikely(crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
		unlikely(crop->type != V4L2_BUF_TYPE_PRIVATE) &&
		unlikely(crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -EINVAL;

	if (unlikely(crop->c.left < 0 || crop->c.top < 0))
		return -EINVAL;

	if (unlikely(crop->c.left + crop->c.width > (int)dev->dcam_cxt.cap_in_size.w ||
		crop->c.top + crop->c.height > (int)dev->dcam_cxt.cap_in_size.h))
		return -EINVAL;

	DCAM_TRACE("V4L2: v4l2_s_crop, type 0x%x, window %d %d %d %d \n",
		crop->type,
		crop->c.left,
		crop->c.top,
		crop->c.width,
		crop->c.height);

	mutex_lock(&dev->dcam_mutex);

	if(crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE){
		input_size = &dev->dcam_cxt.dcam_path[DCAM_PATH1].in_size;
		input_rect = &dev->dcam_cxt.dcam_path[DCAM_PATH1].in_rect;
	}else if (crop->type == V4L2_BUF_TYPE_PRIVATE){
		input_size = &dev->dcam_cxt.dcam_path[DCAM_PATH2].in_size;
		input_rect = &dev->dcam_cxt.dcam_path[DCAM_PATH2].in_rect;
	}else{
		
		input_size = &dev->dcam_cxt.dcam_path[DCAM_PATH0].in_size;
		input_rect = &dev->dcam_cxt.dcam_path[DCAM_PATH0].in_rect;

		dev->dcam_cxt.cap_in_rect.x  = (uint32_t)crop->c.left;
		dev->dcam_cxt.cap_in_rect.y  = (uint32_t)crop->c.top;
		dev->dcam_cxt.cap_in_rect.w  = (uint32_t)crop->c.width;
		dev->dcam_cxt.cap_in_rect.h  = (uint32_t)crop->c.height;
		dev->dcam_cxt.cap_out_size.w = dev->dcam_cxt.cap_in_rect.w;
		dev->dcam_cxt.cap_out_size.h = dev->dcam_cxt.cap_in_rect.h;
		DCAM_TRACE("V4L2: v4l2_s_crop, Path 0, cap crop: cap_rect %d %d %d %d, cap_out:%d %d \n",
			dev->dcam_cxt.cap_in_rect.x, dev->dcam_cxt.cap_in_rect.y,
			dev->dcam_cxt.cap_in_rect.w, dev->dcam_cxt.cap_in_rect.h,
			dev->dcam_cxt.cap_out_size.w, dev->dcam_cxt.cap_out_size.h);
	}
	input_size->w = dev->dcam_cxt.cap_out_size.w;
	input_size->h = dev->dcam_cxt.cap_out_size.h;
	input_rect->x = (uint32_t)crop->c.left;
	input_rect->y = (uint32_t)crop->c.top;
	input_rect->w = (uint32_t)crop->c.width;
	input_rect->h = (uint32_t)crop->c.height;
#if 0
	for (i = 0; i < DCAM_PATH_NUM; i++) {
		input_size = &dev->dcam_cxt.dcam_path[i].in_size;
		input_size->w = dev->dcam_cxt.cap_out_size.w;
		input_size->h = dev->dcam_cxt.cap_out_size.h;
		input_rect = &dev->dcam_cxt.dcam_path[i].in_rect;
		input_rect->x = (uint32_t)crop->c.left;
		input_rect->y = (uint32_t)crop->c.top;
		input_rect->w = (uint32_t)crop->c.width;
		input_rect->h = (uint32_t)crop->c.height;
	}
#endif
	mutex_unlock(&dev->dcam_mutex);

	return 0;
}

LOCAL int  v4l2_g_crop(struct file *file,
			void *priv,
			struct v4l2_crop *crop)
{
	struct dcam_dev          *dev = video_drvdata(file);
	struct dcam_rect         *rect;

	if (unlikely(crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
		unlikely(crop->type != V4L2_BUF_TYPE_PRIVATE))
		return -EINVAL;

	DCAM_TRACE("V4L2: v4l2_g_crop %d %d %d %d \n",
		dev->dcam_cxt.cap_in_rect.x,
		dev->dcam_cxt.cap_in_rect.y,
		dev->dcam_cxt.cap_in_rect.w,
		dev->dcam_cxt.cap_in_rect.h);

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE == crop->type) {
		rect = &dev->dcam_cxt.dcam_path[DCAM_PATH1].in_rect;
	} else if (V4L2_BUF_TYPE_PRIVATE == crop->type) {
		rect = &dev->dcam_cxt.dcam_path[DCAM_PATH2].in_rect;
	} else {
		rect = &dev->dcam_cxt.dcam_path[DCAM_PATH0].in_rect;
	}
	crop->c.left   = (int)rect->x;
	crop->c.top    = (int)rect->y;
	crop->c.width  = (int)rect->w;
	crop->c.height = (int)rect->h;

	return 0;
}

LOCAL int v4l2_enum_fmt_vid_cap(struct file *file,
				void  *priv,
				struct v4l2_fmtdesc *f)
{
	struct dcam_format          *fmt;

	DCAM_TRACE("V4L2: v4l2_enum_fmt_vid_cap, type 0x%x index %d \n", f->type, f->index);
	
	if (unlikely(f->index >= ARRAY_SIZE(dcam_img_fmt)))
		return -EINVAL;


	fmt = &dcam_img_fmt[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

LOCAL int v4l2_g_fmt_vid_out(struct file *file,
				void *priv,
				struct v4l2_format *f)
{
	return 0;
}

LOCAL int v4l2_g_fmt_vid_cap(struct file *file,
				void *priv,
				struct v4l2_format *f)
{
	struct dcam_dev          *dev = video_drvdata(file);
	struct dcam_path_spec    *path = &dev->dcam_cxt.dcam_path[DCAM_PATH1];

	DCAM_TRACE("V4L2: v4l2_g_fmt_vid_cap %d \n", f->type);

	if (V4L2_BUF_TYPE_PRIVATE == f->type) {
		path = &dev->dcam_cxt.dcam_path[DCAM_PATH2];
	}

	f->fmt.pix.width        = path->out_size.w;
	f->fmt.pix.height       = path->out_size.h;
	f->fmt.pix.field        = V4L2_FIELD_INTERLACED;
	f->fmt.pix.pixelformat  = sprd_v4l2_get_fourcc(path);
	f->fmt.pix.bytesperline = (f->fmt.pix.width * path->pixel_depth) >> 3;

	return 0;
}

LOCAL int sprd_v4l2_update_video(struct file *file, uint32_t channel_id)
{
	struct dcam_dev          *dev = video_drvdata(file);
	struct dcam_format       *fmt;
	int                      ret = DCAM_RTN_SUCCESS;
	struct dcam_path_spec    *path = NULL;
	path_cfg_func            path_cfg;
	enum dcam_path_index     path_index;

	DCAM_TRACE("V4L2: sprd_v4l2_update_video, channel=%d \n", channel_id);

	mutex_lock(&dev->dcam_mutex);

	path = &dev->dcam_cxt.dcam_path[channel_id];
	path_index = sprd_v4l2_get_path_index(channel_id);

	if (DCAM_PATH1 == channel_id) {
		path_cfg = dcam_path1_cfg;
	}else if (DCAM_PATH2 == channel_id) {
		path_cfg = dcam_path2_cfg;
	}
#if 0
	ret = path_cfg(DCAM_PATH_INPUT_SIZE, &path->in_size);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_INPUT_RECT, &path->in_rect);
	V4L2_RTN_IF_ERR(ret);

	ret = path_cfg(DCAM_PATH_OUTPUT_SIZE, &path->out_size);
	V4L2_RTN_IF_ERR(ret);
#endif
	dcam_update_path(path_index, &path->in_size, &path->in_rect, &path->out_size);

exit:
	mutex_unlock(&dev->dcam_mutex);

	if (ret) {
		printk("V4L2: Failed to update video 0x%x \n", ret);
	}

	return ret;
}

LOCAL int v4l2_try_fmt_vid_cap(struct file *file,
				void *priv,
				struct v4l2_format *f)
{
	struct dcam_dev          *dev = video_drvdata(file);
	struct dcam_format       *fmt;
	int                      ret;
	uint32_t                 channel_id;

	DCAM_TRACE("V4L2: v4l2_try_fmt_vid_cap, type 0x%x \n", f->type);

	fmt = sprd_v4l2_get_format(f);
	if (unlikely(!fmt)) {
		printk("V4L2: Fourcc format (0x%08x) invalid. \n",
			f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE == f->type) {
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_v4l2_check_path1_cap(fmt->fourcc, f, &dev->dcam_cxt);
		mutex_unlock(&dev->dcam_mutex);
		channel_id = DCAM_PATH1;
	} else if (V4L2_BUF_TYPE_PRIVATE == f->type) {
		if (unlikely(dcam_get_resizer(0))) {
			
			printk("V4L2: path2 has been occupied by other app \n");
			return -EIO;
		}
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_v4l2_check_path2_cap(fmt->fourcc, f, &dev->dcam_cxt);
		mutex_unlock(&dev->dcam_mutex);
		if (ret) {
			
			dcam_rel_resizer();
		}
		channel_id = DCAM_PATH2;
	} else if (V4L2_BUF_TYPE_VIDEO_OUTPUT == f->type){
		mutex_lock(&dev->dcam_mutex);
		ret = sprd_v4l2_check_path0_cap(fmt->fourcc, f, &dev->dcam_cxt);
		mutex_unlock(&dev->dcam_mutex);
		channel_id = DCAM_PATH0;
	} else {
		printk("V4L2: Buf type invalid. \n");
		return -EINVAL;
	}

	if ((0 == ret) && (0 != atomic_read(&dev->stream_on))) {
		if ((DCAM_PATH1 == channel_id))
			ret = sprd_v4l2_update_video(file, channel_id);
	}

	return ret;
}

LOCAL int v4l2_s_fmt_vid_cap(struct file *file,
				void *priv,
				struct v4l2_format *f)
{
	return v4l2_try_fmt_vid_cap(file, priv, f);
}

LOCAL int v4l2_qbuf(struct file *file,
			void *priv, 
			struct v4l2_buffer *p)
{
	int                      ret = DCAM_RTN_SUCCESS;
	struct dcam_dev          *dev = video_drvdata(file);
	struct dcam_info         *info = &dev->dcam_cxt;
	struct dcam_path_spec    *path;
	uint32_t                 index;
	uint32_t                 path_cnt;

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE == p->type) {
		path = &info->dcam_path[DCAM_PATH1];
		path_cnt = DCAM_PATH_1_FRM_CNT_MAX;
	} else if (V4L2_BUF_TYPE_PRIVATE == p->type) {
		path = &info->dcam_path[DCAM_PATH2];
		path_cnt = DCAM_PATH_2_FRM_CNT_MAX;
	} else if (V4L2_BUF_TYPE_VIDEO_OUTPUT == p->type){
		path = &info->dcam_path[DCAM_PATH0];
		path_cnt = DCAM_PATH_0_FRM_CNT_MAX;
	} else {
		printk("V4L2 error: v4l2_qbuf, type 0x%x \n", p->type);
		return -EINVAL;
	}

	DCAM_TRACE_LOW("V4L2: v4l2_qbuf, type 0x%x, status=%d, frm_cnt_act=%d, y=0x%x, u=0x%x, v=0x%x \n", 
		p->type, path->status, path->frm_cnt_act, p->m.userptr, p->input, p->reserved);
		
	mutex_lock(&dev->dcam_mutex);

	if (PATH_IDLE == path->status) {
		DCAM_TRACE("V4L2: v4l2_qbuf, type 0x%x, status=%d, frm_cnt_act=%d, y=0x%x, u=0x%x, v=0x%x \n",
				p->type, path->status, path->frm_cnt_act, p->m.userptr, p->input, p->reserved);
		if (unlikely(0 == p->m.userptr)) {
			printk("V4L2: No yaddr, type 0x%x \n", p->type);
			ret = -EINVAL;
		} else if(unlikely(0xFFFFFFFF != p->m.userptr)){
			if (unlikely(path->frm_cnt_act < path_cnt)) {
				
				path->frm_addr[path->frm_cnt_act].yaddr = p->m.userptr;
				path->frm_addr[path->frm_cnt_act].uaddr = p->input;
				path->frm_addr[path->frm_cnt_act].vaddr = p->reserved;
				path->frm_cnt_act++;
			} else {
				printk("V4L2: frm_cnt_act error %d \n", path->frm_cnt_act);
				ret = -EMLINK;
			}
		}
	} else {
		if (unlikely(p->index > path->frm_id_base + path->frm_cnt_act - 1)) {
			printk("V4L2: error, index %d, frm_id_base %d frm_cnt_act %d \n",
				p->index, path->frm_id_base, path->frm_cnt_act);
			ret = -EINVAL;
		} else if (p->index < path->frm_id_base) {
			printk("V4L2: error, index %d, frm_id_base %d \n",
						p->index, path->frm_id_base);
			ret = -EINVAL;		
		} else {
			index = p->index - path->frm_id_base;
			dcam_frame_unlock(path->frm_ptr[index]);
			DCAM_TRACE_LOW("V4L2: v4l2_qbuf, type 0x%x, index = 0x%x \n", p->type, p->index);
		}
	}

	mutex_unlock(&dev->dcam_mutex);

	return ret;

}

LOCAL int v4l2_dqbuf(struct file *file,
			void *priv, 
			struct v4l2_buffer *p)
{
	struct dcam_dev          *dev = video_drvdata(file);
	struct dcam_node         node;
	struct dcam_path_spec    *path;
	int                      ret = 0;

	DCAM_TRACE_LOW("V4L2: v4l2_dqbuf \n");

	while (1) {
		ret = down_interruptible(&dev->irq_sem);
		if (0 == ret) {
			break;
		} else if (-EINTR == ret) {
			p->flags = V4L2_SYS_BUSY;
			return DCAM_RTN_SUCCESS;
		}else {
			printk("V4L2: v4l2_dqbuf, failed to down, %d \n", ret);
			return -EPERM;
		}
	}

	if (sprd_v4l2_queue_read(&dev->queue, &node)) {
		printk("V4L2: v4l2_dqbuf, failed read queue \n");
		p->flags = V4L2_TX_ERR;
		return -ERESTARTSYS;
	}

	do_gettimeofday(&p->timestamp);
	DCAM_TRACE_LOW("V4L2: time, %d %d \n", (int)p->timestamp.tv_sec, (int)p->timestamp.tv_usec);

	p->flags = node.irq_flag;
	p->type  = node.f_type;
	p->index = node.index;
	p->reserved = node.height;
	p->sequence = node.reserved;
	if (V4L2_BUF_TYPE_VIDEO_CAPTURE == p->type) {
		path = &dev->dcam_cxt.dcam_path[DCAM_PATH1];
	} else if (V4L2_BUF_TYPE_PRIVATE == p->type) {
		path = &dev->dcam_cxt.dcam_path[DCAM_PATH2];
	} else {
		path = &dev->dcam_cxt.dcam_path[DCAM_PATH0];
	}

	memcpy((void*)&p->bytesused, (void*)&path->end_sel, sizeof(struct dcam_endian_sel));

	DCAM_TRACE_LOW("V4L2: v4l2_dqbuf, flag 0x%x type 0x%x index 0x%x \n", p->flags, p->type, p->index);

	return DCAM_RTN_SUCCESS;
}

LOCAL int v4l2_streamon(struct file *file,
			void *priv, 
			enum v4l2_buf_type i_type)
{
	struct dcam_dev          *dev = video_drvdata(file);
	int                      ret = DCAM_RTN_SUCCESS;
	uint32_t                 i;
	struct dcam_path_spec    *path_0 = NULL;
	struct dcam_path_spec    *path_1 = NULL;
	struct dcam_path_spec    *path_2 = NULL;

	DCAM_TRACE("V4L2: v4l2_streamon, mode %d, buf_type 0x%x \n",
		dev->stream_mode, i_type);
	
	mutex_lock(&dev->dcam_mutex);

	path_0 = &dev->dcam_cxt.dcam_path[DCAM_PATH0];
	path_1 = &dev->dcam_cxt.dcam_path[DCAM_PATH1];
	path_2 = &dev->dcam_cxt.dcam_path[DCAM_PATH2];
	DCAM_TRACE("V4L2: streamon, is_work: path_0 = %d, path_1 = %d, path_2 = %d, stream_on = %d \n", 
		path_0->is_work, path_1->is_work, path_2->is_work, atomic_read(&dev->stream_on));

	
	if (DCAM_CAP_IF_CSI2 == dev->dcam_cxt.if_mode) {
		ret = csi_api_init();
		V4L2_RTN_IF_ERR(ret);
		ret = csi_api_start();
		V4L2_RTN_IF_ERR(ret);
		ret = csi_reg_isr(sprd_v4l2_csi2_error, (void*)dev);
		V4L2_RTN_IF_ERR(ret);
		ret = csi_set_on_lanes(dev->dcam_cxt.lane_num);
		V4L2_RTN_IF_ERR(ret);
	}

	
	ret = dcam_module_init(dev->dcam_cxt.if_mode, dev->dcam_cxt.sn_mode);
	V4L2_RTN_IF_ERR(ret);

	for (i = DCAM_TX_DONE; i < USER_IRQ_NUMBER; i++) {
		ret = dcam_reg_isr(i, sprd_v4l2_isr[i], dev);
		V4L2_RTN_IF_ERR(ret);
	}

	
	ret = sprd_v4l2_cap_cfg(&dev->dcam_cxt);
	V4L2_RTN_IF_ERR(ret);

	
	if (path_1->is_work) {
		ret = sprd_v4l2_path_cfg(dcam_path1_cfg, path_1);
		V4L2_RTN_IF_ERR(ret);
		path_1->status = PATH_RUN;
	} else {
		ret = dcam_path1_cfg(DCAM_PATH_ENABLE, &path_1->is_work);
		V4L2_RTN_IF_ERR(ret);
	}

	
	if (path_2->is_work) {
		ret = sprd_v4l2_path_cfg(dcam_path2_cfg, path_2);
		V4L2_RTN_IF_ERR(ret);
		path_2->status = PATH_RUN;
	} else {
		ret = dcam_path2_cfg(DCAM_PATH_ENABLE, &path_2->is_work);
		V4L2_RTN_IF_ERR(ret);
	}

	if (path_0->is_work) {
		ret = sprd_v4l2_path0_cfg(dcam_path0_cfg, path_0);
		V4L2_RTN_IF_ERR(ret);
		path_0->status = PATH_RUN;
	} else {
		ret = dcam_path0_cfg(DCAM_PATH_ENABLE, &path_0->is_work);
		V4L2_RTN_IF_ERR(ret);
	}

	ret = dcam_start();

    #ifndef CONFIG_MACH_DUMMY
	rawchip_start(rawchip_image_width);
	#endif

	atomic_set(&dev->stream_on, 1);
exit:

	if (ret) {
		printk("V4L2: Failed to start stream 0x%x \n", ret);
	} else {
		atomic_set(&dev->run_flag, 0);
		v4l2_path1_cnt = 0;
		sprd_start_timer(&dev->dcam_timer, DCAM_TIMEOUT);
	}

	mutex_unlock(&dev->dcam_mutex);

	return ret;
}

LOCAL int v4l2_streamoff(struct file *file,
			void *priv,
			enum v4l2_buf_type i_type)
{
	struct dcam_dev          *dev = video_drvdata(file);
	int                      ret = DCAM_RTN_SUCCESS;
	struct dcam_path_spec    *path_0 = NULL;
	struct dcam_path_spec    *path_1 = NULL;
	struct dcam_path_spec    *path_2 = NULL;

	DCAM_TRACE("V4L2: streamoff, mode %d, if mode %d, buf_type = 0x%x \n",
		dev->stream_mode, dev->dcam_cxt.if_mode, i_type);

	mutex_lock(&dev->dcam_mutex);

	path_0 = &dev->dcam_cxt.dcam_path[DCAM_PATH0];
	path_1 = &dev->dcam_cxt.dcam_path[DCAM_PATH1];
	path_2 = &dev->dcam_cxt.dcam_path[DCAM_PATH2];

	DCAM_TRACE("V4L2: streamoff, is_work: path_0 = %d, path_1 = %d, path_2 = %d, stream_on = %d \n", 
		path_0->is_work, path_1->is_work, path_2->is_work, atomic_read(&dev->stream_on));

	if (unlikely(0 == atomic_read(&dev->stream_on))) {
		printk("V4L2: stream not on\n");
		goto exit;
	}

	ret = sprd_stop_timer(&dev->dcam_timer);
	V4L2_PRINT_IF_ERR(ret);

	ret = dcam_stop();
	V4L2_PRINT_IF_ERR(ret);

	ret = dcam_stop_cap();
	V4L2_PRINT_IF_ERR(ret);

	if (path_1->is_work) {
		path_1->status = PATH_IDLE;
	}

	if (path_2->is_work) {
		path_2->status = PATH_IDLE;
		dcam_rel_resizer();
	}

	if (path_0->is_work) {
		path_0->status = PATH_IDLE;
	}

	atomic_set(&dev->stream_on, 0);
	if (DCAM_CAP_IF_CSI2 == dev->dcam_cxt.if_mode) {
		ret = csi_api_close();
		V4L2_PRINT_IF_ERR(ret);
	}

	ret = dcam_module_deinit(dev->dcam_cxt.if_mode, dev->dcam_cxt.sn_mode);
	V4L2_PRINT_IF_ERR(ret);

	ret = sprd_v4l2_local_deinit(dev);
	V4L2_PRINT_IF_ERR(ret);

exit:
	mutex_unlock(&dev->dcam_mutex);
	rawchip_stop();

	return ret;
}

LOCAL int v4l2_g_output(struct file *file,
			void *priv, 
			unsigned int *i)
{
	int                      ret = DCAM_RTN_SUCCESS;

	return ret;
}

static  int v4l2_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct dcam_dev          *dev = video_drvdata(file);
	uint32_t                 timing_param[DCAM_TIMING_LEN];
	int                      ret = 0;

	mutex_lock(&dev->dcam_mutex);

	ret = copy_from_user((void*)timing_param, (void*)ctrl->value,
		(uint32_t)(DCAM_TIMING_LEN*sizeof(uint32_t)));

	if (unlikely(ret)) {
		printk("V4L2: v4l2_s_ctrl, error, 0x%x \n", ctrl->value);
		ret = -EFAULT;
		V4L2_RTN_IF_ERR(ret);
	}

	dev->dcam_cxt.if_mode     = timing_param[DCAM_TIMING_LEN-1];
	dev->dcam_cxt.sn_mode     = timing_param[0];
	dev->dcam_cxt.yuv_ptn     = timing_param[1];
	dev->dcam_cxt.frm_deci    = timing_param[3];

	DCAM_TRACE("V4L2: interface %d, mode %d frm_deci %d \n",
		dev->dcam_cxt.if_mode,
		dev->dcam_cxt.sn_mode,
		dev->dcam_cxt.frm_deci);

	if (DCAM_CAP_IF_CCIR == dev->dcam_cxt.if_mode) {
		
		dev->dcam_cxt.sync_pol.vsync_pol = timing_param[4];
		dev->dcam_cxt.sync_pol.hsync_pol = timing_param[5];
		dev->dcam_cxt.sync_pol.pclk_pol  = timing_param[6];
		dev->dcam_cxt.data_bits          = 8;
		DCAM_TRACE("V4L2: CIR interface, vsync %d hsync %d pclk %d bits %d \n",
			dev->dcam_cxt.sync_pol.vsync_pol,
			dev->dcam_cxt.sync_pol.hsync_pol,
			dev->dcam_cxt.sync_pol.pclk_pol,
			dev->dcam_cxt.data_bits);
	} else {
		
		dev->dcam_cxt.sync_pol.need_href = timing_param[4];
		dev->dcam_cxt.is_loose           = timing_param[6];
		dev->dcam_cxt.data_bits          = timing_param[5];
		dev->dcam_cxt.lane_num           = timing_param[7];
		DCAM_TRACE("V4L2: MIPI interface, ref %d is_loose %d bits %d lanes %d \n",
			dev->dcam_cxt.sync_pol.need_href,
			dev->dcam_cxt.is_loose,
			dev->dcam_cxt.data_bits,
			dev->dcam_cxt.lane_num);
	}
	
exit:
	mutex_unlock(&dev->dcam_mutex);
	return ret;
}

LOCAL int sprd_v4l2_streampause(struct file *file, uint32_t channel_id)
{
	struct dcam_dev          *dev = video_drvdata(file);
	struct dcam_path_spec    *path = NULL;
	int                      ret = 0;
	enum dcam_path_index     path_index;

	DCAM_TRACE("V4L2: sprd_v4l2_streampause, channel=%d \n", channel_id);
		
	path = &dev->dcam_cxt.dcam_path[channel_id];
	path_index = sprd_v4l2_get_path_index(channel_id);

	if (PATH_RUN == path->status) {
		ret = dcam_stop_path(path_index);
		V4L2_PRINT_IF_ERR(ret);
		path->status = PATH_IDLE;
		
             
             
		
		

		if(DCAM_PATH2 == channel_id){
			dcam_rel_resizer();
		}
		DCAM_TRACE("V4L2: sprd_v4l2_streampause, channel=%d done \n", channel_id);
	}else{
		DCAM_TRACE("V4L2: sprd_v4l2_streampause, path %d not running, status=%d, cannot pause \n", 
			channel_id, path->status);
	}

	return ret;
}

LOCAL int sprd_v4l2_streamresume(struct file *file, uint32_t channel_id)
{
	struct dcam_dev          *dev = video_drvdata(file);
	struct dcam_path_spec    *path = NULL;
	enum dcam_path_index     path_index;
	path_cfg_func            path_cfg;
	int                      ret = 0;

	DCAM_TRACE("V4L2: sprd_v4l2_streamresume, channel=%d \n", channel_id);
	
	path = &dev->dcam_cxt.dcam_path[channel_id];
	path_index = sprd_v4l2_get_path_index(channel_id);

	if (PATH_IDLE == path->status) {
		if (path->is_work) {
			if (DCAM_PATH0 == channel_id) {
				path_cfg = dcam_path0_cfg;
			}else if (DCAM_PATH1 == channel_id) {
				path_cfg = dcam_path1_cfg;
			}else if (DCAM_PATH2 == channel_id) {
				path_cfg = dcam_path2_cfg; 
			} else {
				printk("V4L2: sprd_v4l2_streamresume, invalid channel_id=0x%x \n", channel_id);
				return -1;
			}

			if(DCAM_PATH0 == channel_id) {
				ret = sprd_v4l2_path0_cfg(path_cfg, path);
			} else {
				ret = sprd_v4l2_path_cfg(path_cfg, path);
			}
			V4L2_RTN_IF_ERR(ret);

			ret = dcam_start_path(path_index);
			V4L2_PRINT_IF_ERR(ret);

			path->status = PATH_RUN;
		}else{
			DCAM_TRACE("V4L2: sprd_v4l2_streamresume, path %d no parameter, is_work=%d, cannot resume \n", 
				channel_id, path->is_work);
		}
	}else{
		DCAM_TRACE("V4L2: sprd_v4l2_streamresume, path %d not idle, status=%d, cannot resume \n", 
			channel_id, path->status);
	}
exit:
	if (ret) {
		DCAM_TRACE("V4L2: fail sprd_v4l2_streamresume, path %d, ret = 0x%x\n", channel_id, ret);
	}
	return ret;
}

LOCAL void sprd_timer_callback(unsigned long data)
{
	struct dcam_dev          *dev = (struct dcam_dev*)data;
	struct dcam_node         node;
	int                      ret = 0;

	DCAM_TRACE("V4L2: sprd_timer_callback \n");

	if (0 == data || 0 == atomic_read(&dev->stream_on)) {
		printk("timer cb error \n");
		return;
	}

	if (0 == atomic_read(&dev->run_flag)) {
		printk("DCAM timeout.\n");
		node.irq_flag = V4L2_TIMEOUT;
		node.f_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		ret = sprd_v4l2_queue_write(&dev->queue, &node);
		if (ret) {
			printk("V4L2: timer cb write queue error \n");
		}
		up(&dev->irq_sem);
	}
}
LOCAL int sprd_init_timer(struct timer_list *dcam_timer, unsigned long data)
{
	int                      ret = 0;

	setup_timer(dcam_timer, sprd_timer_callback, data);
	return ret;
}
LOCAL int sprd_start_timer(struct timer_list *dcam_timer, uint32_t time_val)
{
	int                      ret = 0;

	DCAM_TRACE("V4L2: starting timer %ld \n",jiffies);
	ret = mod_timer(dcam_timer, jiffies + msecs_to_jiffies(time_val));
	if (ret) {
		printk("V4L2: Error in mod_timer %d \n",ret);
	}
	return ret;
}

LOCAL int sprd_stop_timer(struct timer_list *dcam_timer)
{
	int                      ret = 0;

	DCAM_TRACE("V4L2: stop timer \n");
	del_timer_sync(dcam_timer);
	return ret;
}

LOCAL int sprd_init_handle(struct file *file)
{
	struct dcam_dev          *dev = video_drvdata(file);
	struct dcam_info         *info = &dev->dcam_cxt;
	struct dcam_path_spec    *path;

	if (NULL == info) {
		printk("V4L2: init handle fail \n");
		return -EINVAL;
	}
	path = &info->dcam_path[DCAM_PATH1];
	if (NULL == path) {
		printk("V4L2: init path 1 fail \n");
		return -EINVAL;
	}

	atomic_set(&dev->stream_on, 0);

	path->frm_cnt_act = 0;
	path->status = PATH_IDLE;

	path = &info->dcam_path[DCAM_PATH2];
	if (NULL == path) {
		printk("V4L2:init path 2 fail \n");
		return -EINVAL;
	}
	path->frm_cnt_act = 0;
	path->status = PATH_IDLE;

	path = &info->dcam_path[DCAM_PATH0];
	if (NULL == path) {
		printk("V4L2:init path 0 fail \n");
		return -EINVAL;
	}
	path->frm_cnt_act = 0;
	path->status = PATH_IDLE;

	return 0;
}

LOCAL int sprd_v4l2_open(struct file *file)
{
	struct dcam_dev          *dev = video_drvdata(file);
	int                      ret = 0;

	wake_lock(&cam_wake_lock);
	printk("[CAM]wake lock >\n");

	mutex_lock(&dev->dcam_mutex);

	if (unlikely(atomic_inc_return(&dev->users) > 1)) {
		ret = -EBUSY;
		goto exit;
	}

	DCAM_TRACE("sprd_v4l2_open \n");
	ret = dcam_module_en();
	if (unlikely(0 != ret)) {
		printk("V4L2: Failed to enable dcam module \n");
		ret = -EIO;
		goto exit;
	}

	ret = dcam_reset(DCAM_RST_ALL);
	if (unlikely(0 != ret)) {
		printk("V4L2: Failed to reset dcam path1 \n");
		ret = -EIO;
		goto exit;
	}

	ret = sprd_v4l2_queue_init(&dev->queue);
	if (unlikely(0 != ret)) {
		printk("V4L2: Failed to init queue \n");
		ret = -EIO;
		goto exit;
	}

	ret = sprd_init_timer(&dev->dcam_timer,(unsigned long)dev);
	if (unlikely(0 != ret)) {
		printk("V4L2: Failed to init timer \n");
		ret = -EIO;
		goto exit;
	}

	ret = sprd_init_handle(file);
	if (unlikely(0 != ret)) {
		printk("V4L2: Failed to init queue \n");
		ret = -EIO;
		goto exit;
	}

	DCAM_TRACE("V4L2: open /dev/video%d type=%s \n", dev->vfd->num,
		v4l2_type_names[V4L2_BUF_TYPE_VIDEO_CAPTURE]);

exit:
	if (unlikely(ret)) {
		atomic_dec(&dev->users);
	}
	mutex_unlock(&dev->dcam_mutex);

	DCAM_TRACE("sprd_v4l2_open %d \n", ret);

	return ret;
}

ssize_t sprd_v4l2_read(struct file *file, char __user *u_data, size_t cnt, loff_t *cnt_ret)
{
	uint32_t                 rt_word[2];

	if (cnt < sizeof(uint32_t)) {
		printk("sprd_v4l2_read , wrong size of u_data %d \n", cnt);
		return -1;
	}

	rt_word[0] = DCAM_PATH2_LINE_BUF_LENGTH;
	rt_word[1] = DCAM_SC_COEFF_UP_MAX;
	DCAM_TRACE("sprd_v4l2_read line threshold %d, sc factor %d.\n", rt_word[0], rt_word[1]);
	(void)file; (void)cnt; (void)cnt_ret;
	return copy_to_user(u_data, (void*)rt_word, (uint32_t)(2*sizeof(uint32_t)));
}

ssize_t sprd_v4l2_write(struct file *file, const char __user * u_data, size_t cnt, loff_t *cnt_ret)
{
	struct dcam_dev          *dev = video_drvdata(file);
	int                      ret = 0;

	DCAM_TRACE("sprd_v4l2_write %d, dev 0x%x \n", cnt, (uint32_t)dev);

	mutex_lock(&dev->dcam_mutex);

	ret = sprd_v4l2_tx_stop(dev);

	if (ret)
		ret = 0;
	else
		ret = 1;

	mutex_unlock(&dev->dcam_mutex);

	return ret;
}

LOCAL int sprd_v4l2_close(struct file *file)
{
	struct dcam_dev          *dev = video_drvdata(file);
	int                      ret = 0;

	DCAM_TRACE("V4L2: sprd_v4l2_close. \n");

	mutex_lock(&dev->dcam_mutex);
	ret = dcam_module_dis();
	if (unlikely(0 != ret)) {
		printk("V4L2: Failed to enable dcam module \n");
		ret = -EIO;
	}
	sprd_stop_timer(&dev->dcam_timer);
	atomic_dec(&dev->users);
	mutex_unlock(&dev->dcam_mutex);

	wake_unlock(&cam_wake_lock);
	printk("[CAM]wake unlock <\n");

	DCAM_TRACE("V4L2: close end. \n");

	return ret;
}

LOCAL int  sprd_v4l2_proc_read(char           *page,
			char  	       **start,
			off_t          off,
			int            count,
			int            *eof,
			void           *data)
{
#if 0
	int                      len = 0, ret;
	struct dcam_dev          *dev = (struct dcam_dev*)data;
	uint32_t*                reg_buf;
	uint32_t                 reg_buf_len = 0x400;
	uint32_t                 print_len = 0, print_cnt = 0;

	(void)start; (void)off; (void)count; (void)eof;

	len += sprintf(page + len, "Context for DCAM device \n");
	len += sprintf(page + len, "********************************************* \n");
	len += sprintf(page + len, "SPRD_ION_SIZE: 0x%x \n", SPRD_ION_SIZE);
	len += sprintf(page + len, "the configuration of CAP \n");
	len += sprintf(page + len, "1. interface mode %d \n", dev->dcam_cxt.if_mode);
	len += sprintf(page + len, "2. sensor mode %d \n", dev->dcam_cxt.sn_mode);
	len += sprintf(page + len, "3. YUV pattern %d \n", dev->dcam_cxt.yuv_ptn);
	len += sprintf(page + len, "4. sync polarities v %d h %d p %d href %d \n",
			dev->dcam_cxt.sync_pol.vsync_pol, 
			dev->dcam_cxt.sync_pol.hsync_pol,
			dev->dcam_cxt.sync_pol.pclk_pol,
			dev->dcam_cxt.sync_pol.need_href);
	len += sprintf(page + len, "5. Data bit-width %d \n", dev->dcam_cxt.data_bits);
	len += sprintf(page + len, "6. need ISP %d \n", dev->dcam_cxt.dcam_path[DCAM_PATH1].is_from_isp);
	len += sprintf(page + len, "********************************************* \n");
	len += sprintf(page + len, "the configuration of PATH1 \n");
	len += sprintf(page + len, "1. input rect %d %d %d %d \n",
			dev->dcam_cxt.dcam_path[DCAM_PATH1].in_rect.x,
			dev->dcam_cxt.dcam_path[DCAM_PATH1].in_rect.y,
			dev->dcam_cxt.dcam_path[DCAM_PATH1].in_rect.w,
			dev->dcam_cxt.dcam_path[DCAM_PATH1].in_rect.h);
	len += sprintf(page + len, "2. output size %d %d \n",
		dev->dcam_cxt.dcam_path[DCAM_PATH1].out_size.w,
		dev->dcam_cxt.dcam_path[DCAM_PATH1].out_size.h);
	len += sprintf(page + len, "3. output format %d \n",
		dev->dcam_cxt.dcam_path[DCAM_PATH1].out_fmt);
	len += sprintf(page + len, "4. frame index based on 0x%x \n", dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_id_base);
	len += sprintf(page + len, "5. frame count 0x%x \n", dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_cnt_act);
	len += sprintf(page + len, "6. frame type 0x%x \n", dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_type);
	for (print_cnt = 0; print_cnt < DCAM_PATH_1_FRM_CNT_MAX; print_cnt ++) {
		len += sprintf(page + len, "%d. frame buffer0  0x%x 0x%x 0x%x \n",
			(6 + print_cnt),
			dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_addr[print_cnt].yaddr,
			dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_addr[print_cnt].uaddr,
			dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_addr[print_cnt].vaddr);
	}

	if (dev->dcam_cxt.dcam_path[DCAM_PATH2].is_work) {
		len += sprintf(page + len, "********************************************* \n");
		len += sprintf(page + len, "the configuration of PATH2 \n");
		len += sprintf(page + len, "1. input rect %d %d %d %d \n",
				dev->dcam_cxt.dcam_path[DCAM_PATH2].in_rect.x,
				dev->dcam_cxt.dcam_path[DCAM_PATH2].in_rect.y,
				dev->dcam_cxt.dcam_path[DCAM_PATH2].in_rect.w,
				dev->dcam_cxt.dcam_path[DCAM_PATH2].in_rect.h);
		len += sprintf(page + len, "2. output size %d %d \n",
			dev->dcam_cxt.dcam_path[DCAM_PATH2].out_size.w,
			dev->dcam_cxt.dcam_path[DCAM_PATH2].out_size.h);
		len += sprintf(page + len, "3. output format %d \n",
			dev->dcam_cxt.dcam_path[DCAM_PATH2].out_fmt);
		len += sprintf(page + len, "4. frame index based on 0x%x \n", dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_id_base);
		len += sprintf(page + len, "5. frame count 0x%x \n", dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_cnt_act);
		len += sprintf(page + len, "6. frame type 0x%x \n", dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_type);
		for (print_cnt = 0; print_cnt < DCAM_PATH_2_FRM_CNT_MAX; print_cnt ++) {
			len += sprintf(page + len, "%d. frame buffer %d,  0x%x 0x%x 0x%x \n",
				(6 + print_cnt),
				print_cnt,
				dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_addr[print_cnt].yaddr,
				dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_addr[print_cnt].uaddr,
				dev->dcam_cxt.dcam_path[DCAM_PATH1].frm_addr[print_cnt].vaddr);
		}

	}

	reg_buf = (uint32_t*)kmalloc(reg_buf_len, GFP_KERNEL);
	ret = dcam_read_registers(reg_buf, &reg_buf_len);
	if (ret)
		return len;

	len += sprintf(page + len, "********************************************* \n");
	len += sprintf(page + len, "dcam registers \n");
	print_cnt = 0;
	while (print_len < reg_buf_len) {
		len += sprintf(page + len, "offset 0x%x : 0x%x, 0x%x, 0x%x, 0x%x \n",
			print_len,
			reg_buf[print_cnt],
			reg_buf[print_cnt+1],
			reg_buf[print_cnt+2],
			reg_buf[print_cnt+3]);
		print_cnt += 4;
		print_len += 16;
	}
	len += sprintf(page + len, "********************************************* \n");
	len += sprintf(page + len, "The end of DCAM device \n");
	msleep(10);
	kfree(reg_buf);

	return len;
#else
	return 0;
#endif 
}

LOCAL const struct v4l2_ioctl_ops sprd_v4l2_ioctl_ops = {
	.vidioc_g_parm                = v4l2_g_parm,
	.vidioc_s_parm                = v4l2_s_parm,	
	.vidioc_querycap              = v4l2_querycap,
	.vidioc_cropcap               = v4l2_cropcap,
	.vidioc_s_crop                = v4l2_s_crop,
	.vidioc_enum_fmt_vid_cap      = v4l2_enum_fmt_vid_cap,
	.vidioc_enum_fmt_type_private = v4l2_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap         = v4l2_g_fmt_vid_cap,
	.vidioc_g_fmt_type_private    = v4l2_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap       = v4l2_try_fmt_vid_cap,
	.vidioc_try_fmt_type_private  = v4l2_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap         = v4l2_s_fmt_vid_cap,
	.vidioc_qbuf                  = v4l2_qbuf,
	.vidioc_dqbuf                 = v4l2_dqbuf,
	.vidioc_streamon              = v4l2_streamon,
	.vidioc_streamoff             = v4l2_streamoff,
	.vidioc_g_crop                = v4l2_g_crop,
	.vidioc_g_output              = v4l2_g_output,
	.vidioc_s_ctrl                = v4l2_s_ctrl,
	.vidioc_g_fmt_vid_out         = v4l2_g_fmt_vid_out,
	.vidioc_enum_fmt_vid_out      = v4l2_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_out       = v4l2_try_fmt_vid_cap
};

LOCAL const struct v4l2_file_operations sprd_v4l2_fops = {
	.owner                   = THIS_MODULE,
	.open                    = sprd_v4l2_open,
	.read                    = sprd_v4l2_read,
	.release                 = sprd_v4l2_close,
	.write                   = sprd_v4l2_write,
	.ioctl                   = video_ioctl2, 
};


LOCAL struct video_device sprd_v4l2_template = {
	.name                    = "dcam",
	.fops                    = &sprd_v4l2_fops,
	.ioctl_ops               = &sprd_v4l2_ioctl_ops,
	.minor                   = -1,
	.release                 = video_device_release,
	.tvnorms                 = V4L2_STD_525_60,
	.current_norm            = V4L2_STD_NTSC_M,
};

LOCAL int release(void)
{
	struct dcam_dev          *dev;
	struct list_head         *list;

	while (!list_empty(&dcam_devlist)) {
		list = dcam_devlist.next;
		list_del(list);
		dev = list_entry(list, struct dcam_dev, dcam_devlist);
		v4l2_info(&dev->v4l2_dev, "unregistering /dev/video%d\n",
			dev->vfd->num);
		video_unregister_device(dev->vfd);
		v4l2_device_unregister(&dev->v4l2_dev);
		kfree(dev);
	}
	return 0;
}

LOCAL int __init create_instance(int inst)
{
	struct dcam_dev          *dev;
	struct video_device      *vfd;
	int                      ret = DCAM_RTN_SUCCESS;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	DCAM_TRACE("create_instance, 0x%x", (uint32_t)dev);

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name),
		 "%s-%03d", DCAM_MODULE_NAME, inst);
	ret = v4l2_device_register(NULL, &dev->v4l2_dev);
	if (ret)
		goto free_dev;

	
	mutex_init(&dev->dcam_mutex);
	mutex_init(&dev->tx_mutex);

	sema_init(&dev->irq_sem, 0);

	ret = -ENOMEM;
	vfd = video_device_alloc();
	if (!vfd)
		goto unreg_dev;
	*vfd = sprd_v4l2_template;
	vfd->debug = debug;
	ret = video_register_device(vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0)
		goto rel_vdev;
	video_set_drvdata(vfd, dev);

	
	list_add_tail(&dev->dcam_devlist, &dcam_devlist);
	snprintf(vfd->name, sizeof(vfd->name), "%s (%i)",
		 sprd_v4l2_template.name, vfd->num);

	if (video_nr >= 0)
		video_nr++;

	dev->vfd = vfd;
	v4l2_info(&dev->v4l2_dev, "V4L2 device registered as /dev/video%d\n",
		  vfd->num);

	v4l2_proc_file = create_proc_read_entry("driver/video0",
						0444,
						NULL,
						sprd_v4l2_proc_read,
						(void*)dev);
	if (unlikely(NULL == v4l2_proc_file)) {
		printk("V4L2: Can't create an entry for video0 in /proc \n");
		ret = ENOMEM;
		goto rel_vdev;
	}

	return 0;
rel_vdev:
	video_device_release(vfd);
unreg_dev:
	v4l2_device_unregister(&dev->v4l2_dev);
free_dev:
	kfree(dev);
	return ret;
}

int sprd_v4l2_probe(struct platform_device *pdev)
{
	int                      ret = DCAM_RTN_SUCCESS;

	printk(KERN_ALERT "dcam_probe called\n");
	ret = create_instance(pdev->id);
	if (ret < 0) {
		printk(KERN_INFO "Error %d while loading dcam driver\n", ret);
		return ret;
	}

	htccallback_sysfs_init();
	camera_sysfs_init();
	
	printk(KERN_ALERT "dcam_probe Success.\n");
	return ret;
}

LOCAL int sprd_v4l2_remove(struct platform_device *dev)
{
	int                      ret = DCAM_RTN_SUCCESS;

	if (v4l2_proc_file) {
		DCAM_TRACE("V4L2: sprd_v4l2_remove \n");
		remove_proc_entry("driver/video0", NULL);
	}

	return ret;
}

LOCAL struct platform_driver sprd_v4l2_driver = {
	.probe = sprd_v4l2_probe,
	.remove = sprd_v4l2_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sprd_dcam",
		   },
};

int __init sprd_v4l2_init(void)
{
	int                      ret = DCAM_RTN_SUCCESS;

	if (platform_driver_register(&sprd_v4l2_driver) != 0) {
		printk("platform device register Failed \n");
		return -1;
	}

	wake_lock_init(&cam_wake_lock, WAKE_LOCK_SUSPEND, "sprd_camera_suspend");

	printk(KERN_INFO "Video Technology Magazine Virtual Video "
	       "Capture Board ver %u.%u.%u successfully loaded.\n",
	       (DCAM_VERSION >> 16) & 0xFF, (DCAM_VERSION >> 8) & 0xFF,
	       DCAM_VERSION & 0xFF);
	return ret;
}

void sprd_v4l2_exit(void)
{
	platform_driver_unregister(&sprd_v4l2_driver);
	release();

	wake_lock_destroy(&cam_wake_lock);
}

module_init(sprd_v4l2_init);
module_exit(sprd_v4l2_exit);

MODULE_DESCRIPTION("DCAM Driver");
MODULE_AUTHOR("REF_Image@Spreadtrum");
MODULE_LICENSE("GPL");

