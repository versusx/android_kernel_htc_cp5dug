#include <linux/init.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h> 
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/device.h> 
#include <linux/io.h>     
#include <linux/version.h>
#include <linux/timer.h>         
#include <linux/input.h>      
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#ifdef PRIMOU_REWORK
#undef PRIMOU_REWORK
#endif
#define DEBUG_ON 1

#if DEBUG_ON
#define GPSD(fmt, arg...) printk(KERN_DEBUG "[GPS].(DEBUG) "fmt"", ##arg)
#define GPSE(fmt, arg...) printk(KERN_ERR "[GPS].(ERROR) "fmt"", ##arg)
#else
#define GPSD(...) ((void *)0)
#define GPSE(fmt, arg...) printk(KERN_ERR "[GPS].(ERROR) "fmt"", ##arg)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
#define LINUX_3
#endif

#define GPS_ONOFF_VERSION "1.0.0"
#define DEVICE_NAME "gps_onoff"


#define PRIMOTD_GPIO_GPS_UART1_RX     (186)
#define PRIMOTD_GPIO_GPS_UART1_TX     (185)
#define PRIMOTD_GPIO_GPS_UART1_CTS    (187)
#define PRIMOTD_GPIO_GPS_UART1_RTS    (188)

#define GPS_ONOFF					  (121)


int gps_onoff_major = -1;
static struct class  *gps_onoff_class = NULL;   /* GPS class during class_create */
static struct device *gps_onoff_dev   = NULL;   /* GPS dev during device_create */

static struct regulator *cmmb_regulator_1v8 = NULL;

char status = '0';

static int config_gps_uart(void)
{
	return 0;
}

static void gps_clk_init(void)
{
        struct clk *gps_clk;
        struct clk *clk_parent;
/*8825ea/openphone use aux0 garda use aux1*/
        gps_clk = clk_get(NULL, "clk_aux0");
        if (IS_ERR(gps_clk)) {
                printk("clock: failed to get clk_aux0\n");
        }
        clk_parent = clk_get(NULL, "ext_32k");
        if (IS_ERR(clk_parent)) {
                printk("failed to get parent ext_32k\n");
        }

        clk_set_parent(gps_clk, clk_parent);
        clk_set_rate(gps_clk, 32768);
        clk_enable(gps_clk);
}






static int deconfig_gps_uart(void)
{
/*
	gpio_direction_output(PRIMOTD_GPIO_GPS_UART3_RTS, 0);
	gpio_direction_output(PRIMOTD_GPIO_GPS_UART3_CTS, 0);
	gpio_direction_output(PRIMOTD_GPIO_GPS_UART3_RX, 0);
	gpio_direction_output(PRIMOTD_GPIO_GPS_UART3_TX, 0);


	return -1;
*/
	return 0;
}

static void gps_onoff(int onoff)
{
	//gpio_direction_output(GPS_ONOFF, onoff);
        gpio_direction_output(GPS_ONOFF,onoff);
}

static long gps_onoff_ioctl(
#ifndef LINUX_3
		struct inode *inode,
#endif
		struct file *filp , unsigned int cmd, unsigned long arg)
{
	switch(cmd){
		case 1:
			gps_onoff(1);
			status = '1';
			GPSD("set the gps_engine on\n");
			break;
		case 0:
			gps_onoff(0);
			status = '0';
			GPSD("set the gps_engine off\n");
			break;
		default:
			GPSE("the cmd is not correct, please check \n");
			return -1;
	}
	return 0;
}

static int gps_onoff_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_ops)
{
	char data[2];
	
	if( copy_from_user(data, buf, count) ) {
		GPSE("write data error\n");
		return -1;
	}
	
	if ( data[0] == '1' ){
		gps_onoff(1);
		status = '1';
		GPSD("set GPIO OK!\n");
	}
	else{
		//gps_onoff(0);
		status = '0';
		GPSD("reset GPIO OK!\n");
	}
	
	return 1;
}

static int gps_onoff_read(struct file *filp, char *buf, size_t count, loff_t *f_ops)
{
	if( copy_to_user(buf, &status, 1)){
		GPSE("read data wrong\n");
		return -1;
	}
	
	return 1;
}


struct file_operations gps_onoff_ops={
	.owner	= THIS_MODULE,
	.write	= gps_onoff_write,
	.read	= gps_onoff_read,
#ifdef LINUX_3
	.unlocked_ioctl = 
#else
	.ioctl	= 
#endif
	gps_onoff_ioctl,
};

//static unsigned gps_onoff_config = GPIO_CFG(GPS_ONOFF, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA);




static int gps_onoff_init(struct platform_device *platdev)
{
	
	int retval;
	GPSD("In gps_onoff_init\n");

	//enable 1V8 power
	cmmb_regulator_1v8 = regulator_get(NULL, "vddcmmb1p8");
        regulator_set_voltage(cmmb_regulator_1v8, 1800000, 2200000);
        regulator_disable(cmmb_regulator_1v8);
        msleep(3);
        regulator_enable(cmmb_regulator_1v8);
        msleep(5);
	
	//for 32k sleep clock	
	gps_clk_init();


	if(config_gps_uart() < 0){
		GPSE("config to GPS uart err!\n ");
		return -1;
	}

        gpio_direction_output(GPS_ONOFF,0);
	
	gps_onoff_major = register_chrdev(0, DEVICE_NAME, &gps_onoff_ops);
	if(gps_onoff_major < 0){
		GPSE("register gps_onoff device err\n");
		goto err4;
	}
	GPSD(KERN_INFO "%s , major %d",DEVICE_NAME, gps_onoff_major);
	
	gps_onoff_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(gps_onoff_class == NULL){
		GPSE("gps_onoff class create err\n");
		goto err5;
	}
	gps_onoff_dev = device_create(gps_onoff_class, NULL, MKDEV(gps_onoff_major, 0), NULL, DEVICE_NAME);
	if(gps_onoff_dev == NULL){
		GPSE("gps_onoff dev create err\n");
		goto err6;
	}

	GPSD("init OK\n");

	return 0;

//	device_destroy(gpsdrv_class, MKDEV(gpsdrv_major, 0));
err6:
	class_destroy(gps_onoff_class);
err5:
	unregister_chrdev(gps_onoff_major, DEVICE_NAME);
err4:	
	return -1;
}

static int  gps_onoff_exit(struct platform_device *platdev )
{
	GPSD("free up\n");
	device_destroy(gps_onoff_class, MKDEV(gps_onoff_major, 0));
	class_destroy(gps_onoff_class);
	unregister_chrdev(gps_onoff_major, DEVICE_NAME);
	return 0;
}


static int gps_suspend(struct platform_device *platdev, pm_message_t state)
{
	GPSD("In gps_suspend function\n");
	
	if(deconfig_gps_uart() < 0){
		GPSE("error occured when deinit gps uart");
		return -1;
	}
	return 0;
}

static int gps_resume(struct platform_device *platdev)
{
	GPSD("In gps_resume function\n");
	
	if(config_gps_uart() < 0){
		GPSE("resume to config to GPS uart err!\n ");
		return -1;
	}

	return 0;
}


static struct platform_driver gps_ctl_drv={
	.probe = gps_onoff_init,
	.remove = gps_onoff_exit,
	.suspend = gps_suspend,
	.resume = gps_resume,
	.driver = {
		.name = "gps_ctl",
		.owner = THIS_MODULE,
	},

};


static struct platform_device gps_ctl_dev={
	    .name="gps_ctl",
		.id=0,
};

/*
static int __init gps_ctl_device_init(void)
{
	return platform_device_register(&gps_ctl_dev);
}

static void __exit gps_ctl_device_exit(void)
{
	platform_device_unregister(&gps_ctl_dev);
}
*/

static int __init gps_ctl_init(void)
{
	int ret;
	ret = platform_device_register(&gps_ctl_dev);
	if(ret < 0){
		GPSE("platform_device register error!\n");
		return ret;
	}

	ret = platform_driver_register(&gps_ctl_drv);
	if(ret < 0)
	{
		GPSE("platform_driver register error!\n");
		platform_device_unregister(&gps_ctl_dev);
		return ret;
	}
	
	return 0;
}

static void __exit gps_ctl_exit(void)
{
	platform_device_unregister(&gps_ctl_dev);
	platform_driver_unregister(&gps_ctl_drv);
}

module_init(gps_ctl_init);
module_exit(gps_ctl_exit);

MODULE_AUTHOR("Derek Guo<Derek_Guo@htc.com>");
MODULE_DESCRIPTION("GPS power control");
MODULE_LICENSE("GPL");
MODULE_VERSION(GPS_ONOFF_VERSION);
