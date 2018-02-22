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
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/termios.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <mach/hardware.h>
#include <linux/clk.h>
#include <mach/pinmap.h>
#include <mach/serial_sprd.h>
#include <linux/wakelock.h>

#include<linux/irq.h>
#include <mach/gpio.h>
#include <linux/uaccess.h>
#include <mach/board_htc.h>

#define USE_BCM_BT_CHIP
#ifdef USE_BCM_BT_CHIP	
#define BT_SERIAL_DBG
#define I_LOG(x...) printk(KERN_INFO "[BT] " x)
#define E_LOG(x...) printk(KERN_ERR "[BT][ERR] " x)

#define   bt_wakeup_pin   233
static unsigned char bt_wakeup_level = 0;
static unsigned char bt_wakeup_assert_inadvance = 0;
#define  host_wakeup_pin  232
static unsigned char host_wakeup_level = 0l;
static unsigned char host_want_sleep =0;
static unsigned int   host_wake_irq=0;

static struct wake_lock brcm_tx_wake_lock;
static struct wake_lock brcm_rx_wake_lock;
static unsigned int is_brcm_rx_wake_locked = 0;
static struct spinlock bt_rlock;

static irqreturn_t bt_hostwake_isr(int irq, void *dev_id);
static struct work_struct lock_off_w; 
static struct workqueue_struct *hsuart_wq; 

static struct hrtimer lock_off_timer;  
static ktime_t lock_off_delay;

static unsigned char suspended_flag=0;    

static unsigned char uart0_status_flag=0;  
static int uart0_pin_config(int enable);

#endif


#define IRQ_WAKEUP 	0

#define UART_NR_MAX			CONFIG_SERIAL_SPRD_UART_NR
#define SP_TTY_NAME			"ttyS"
#define SP_TTY_MINOR_START	64
#define SP_TTY_MAJOR		TTY_MAJOR

#define UART_CLK		48000000

#define ARM_UART_TXD	0x0000
#define ARM_UART_RXD	0x0004
#define ARM_UART_STS0	0x0008
#define ARM_UART_STS1	0x000C
#define ARM_UART_IEN	0x0010
#define ARM_UART_ICLR	0x0014
#define ARM_UART_CTL0	0x0018
#define ARM_UART_CTL1	0x001C
#define ARM_UART_CTL2	0x0020
#define ARM_UART_CLKD0	0x0024
#define ARM_UART_CLKD1	0x0028
#define ARM_UART_STS2	0x002C

#define SP_TX_FIFO		0x40
#define SP_RX_FIFO		0x40

#define CONFIG_BT_DECREASE_FIFO
#ifdef CONFIG_BT_DECREASE_FIFO
#define SP_TX_FIFO_BT		0x20
#define SP_RX_FIFO_BT		0x40
#define TX_HW_FLOW_CTL_THRESHOLD   0x60
#endif


#if defined(CONFIG_MACH_Z4DTG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
#define SP_RX_FIFO_EXT		0x04
#endif
#define UART_IEN_RX_FIFO_FULL	(0x1<<0)
#define UART_IEN_TX_FIFO_EMPTY	(0x1<<1)
#define UART_IEN_BREAK_DETECT	(0x1<<7)
#define UART_IEN_TIMEOUT     	(0x1<<13)

#define UART_DATA_BIT	(0x3<<2)
#define UART_DATA_5BIT	(0x0<<2)
#define UART_DATA_6BIT	(0x1<<2)
#define UART_DATA_7BIT	(0x2<<2)
#define UART_DATA_8BIT	(0x3<<2)
#define UART_STOP_1BIT	(0x1<<4)
#define UART_STOP_2BIT	(0x3<<4)
#define UART_PARITY		0x3
#define UART_PARITY_EN	0x2
#define UART_EVEN_PAR	0x0
#define UART_ODD_PAR	0x1
#define UART_LSR_OE	(0x1<<4)
#define UART_LSR_FE	(0x1<<3)
#define UART_LSR_PE	(0x1<<2)
#define UART_LSR_BI	(0x1<<7)
#define UART_LSR_DR	(0x1<<8)
#define RX_HW_FLOW_CTL_THRESHOLD	0x40
#define RX_HW_FLOW_CTL_EN		(0x1<<7)
#define TX_HW_FLOW_CTL_EN		(0x1<<8)
#define UART_STS_RX_FIFO_FULL	(0x1<<0)
#define UART_STS_TX_FIFO_EMPTY	(0x1<<1)
#define UART_STS_BREAK_DETECT	(0x1<<7)
#define UART_STS_TIMEOUT     	(0x1<<13)
#define BAUD_1200_48M	0x9C40
#define BAUD_2400_48M	0x4E20
#define BAUD_4800_48M	0x2710
#define BAUD_9600_48M	0x1388
#define BAUD_19200_48M	0x09C4
#define BAUD_38400_48M	0x04E2
#define BAUD_57600_48M	0x0314
#define BAUD_115200_48M	0x01A0
#define BAUD_230400_48M	0x00D0
#define BAUD_460800_48M	0x0068
#define BAUD_921600_48M	0x0034
#define BAUD_1000000_48M 0x0030
#define BAUD_1152000_48M 0x0029
#define BAUD_1500000_48M 0x0020
#define BAUD_2000000_48M 0x0018
#define BAUD_2500000_48M 0x0013
#define BAUD_3000000_48M 0x0010



static struct wake_lock uart_rx_lock;  
static bool is_uart_rx_wakeup;
static struct serial_data plat_data;

static inline unsigned int serial_in(struct uart_port *port, int offset)
{
	return __raw_readl(port->membase + offset);
}

static inline void serial_out(struct uart_port *port, int offset, int value)
{
	__raw_writel(value, port->membase + offset);
}

static unsigned int serial_sprd_tx_empty(struct uart_port *port)
{
	if (serial_in(port, ARM_UART_STS1) & 0xff00)
		return 0;
	else
		return 1;
}

static unsigned int serial_sprd_get_mctrl(struct uart_port *port)
{
	return TIOCM_DSR | TIOCM_CTS;
}

static void serial_sprd_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static void serial_sprd_stop_tx(struct uart_port *port)
{
	unsigned int ien, iclr;

	iclr = serial_in(port, ARM_UART_ICLR);
	ien = serial_in(port, ARM_UART_IEN);

	iclr |= UART_IEN_TX_FIFO_EMPTY;
	ien &= ~UART_IEN_TX_FIFO_EMPTY;

	serial_out(port, ARM_UART_ICLR, iclr);
	serial_out(port, ARM_UART_IEN, ien);
}

static void serial_sprd_start_tx(struct uart_port *port)
{
	unsigned int ien;

	ien = serial_in(port, ARM_UART_IEN);
	if (!(ien & UART_IEN_TX_FIFO_EMPTY)) {
		ien |= UART_IEN_TX_FIFO_EMPTY;
		serial_out(port, ARM_UART_IEN, ien);
	}
}

static void serial_sprd_stop_rx(struct uart_port *port)
{
	unsigned int ien, iclr;

	iclr = serial_in(port, ARM_UART_ICLR);
	ien = serial_in(port, ARM_UART_IEN);

	ien &= ~(UART_IEN_RX_FIFO_FULL | UART_IEN_BREAK_DETECT);
	iclr |= UART_IEN_RX_FIFO_FULL | UART_IEN_BREAK_DETECT;

	serial_out(port, ARM_UART_IEN, ien);
	serial_out(port, ARM_UART_ICLR, iclr);
}

static void serial_sprd_enable_ms(struct uart_port *port)
{
}

static void serial_sprd_break_ctl(struct uart_port *port, int break_state)
{
}

static inline void serial_sprd_rx_chars(int irq, void *dev_id)
{
	struct uart_port *port = (struct uart_port *)dev_id;
	struct tty_struct *tty = port->state->port.tty;
	unsigned int status, ch, flag, lsr, max_count = 2048;

	status = serial_in(port, ARM_UART_STS1);
	lsr = serial_in(port, ARM_UART_STS0);
	while ((status & 0x00ff) && max_count--) {
		ch = serial_in(port, ARM_UART_RXD);
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (unlikely(lsr &
		     (UART_LSR_BI | UART_LSR_PE | UART_LSR_FE | UART_LSR_OE))) {
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				port->icount.brk++;
				if (uart_handle_break(port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				port->icount.parity++;
			else if (lsr & UART_LSR_FE)
				port->icount.frame++;
			if (lsr & UART_LSR_OE)
				port->icount.overrun++;
			lsr &= port->read_status_mask;
			if (lsr & UART_LSR_BI)
				flag = TTY_BREAK;
			else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

		uart_insert_char(port, lsr, UART_LSR_OE, ch, flag);
ignore_char:
		status = serial_in(port, ARM_UART_STS1);
		lsr = serial_in(port, ARM_UART_STS0);
	}
	
	tty_flip_buffer_push(tty);
}

static inline void serial_sprd_tx_chars(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct circ_buf *xmit = &port->state->xmit;
	int count;

	if (port->x_char) {
		serial_out(port, ARM_UART_TXD, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		serial_sprd_stop_tx(port);
		return;
	}

    #ifdef CONFIG_BT_DECREASE_FIFO
		if (port->line == 0)
		{
		    count = SP_TX_FIFO_BT;
		}
		else
		{
		    count = SP_TX_FIFO;
		}
	#else
	    count = SP_TX_FIFO;
	#endif

	do {
		serial_out(port, ARM_UART_TXD, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(port);
	}
	if (uart_circ_empty(xmit)) {
		serial_sprd_stop_tx(port);
	}

}

static irqreturn_t serial_sprd_interrupt_chars(int irq, void *dev_id)
{
	struct uart_port *port = (struct uart_port *)dev_id;
	int pass_counter = 0;

	if(port->line == 0 || port->line == 3)
	{
		do {
			if (!
			    (serial_in(port, ARM_UART_STS0) &
			     serial_in(port, ARM_UART_IEN))) {
				break;
			}
			if (serial_in(port, ARM_UART_STS0) &
			    (UART_STS_RX_FIFO_FULL | UART_STS_BREAK_DETECT |
			     UART_STS_TIMEOUT)) {
				serial_sprd_rx_chars(irq, port);
			}
			serial_out(port, ARM_UART_ICLR, 0xffff2090);
			if (serial_in(port, ARM_UART_STS0) & UART_STS_TX_FIFO_EMPTY) {
				serial_sprd_tx_chars(irq, port);
			}
			serial_out(port, ARM_UART_ICLR, 0xffff006C);
		} while (pass_counter++ < 50);
	}
	else{
		do {
			if (!
			    (serial_in(port, ARM_UART_STS0) &
			     serial_in(port, ARM_UART_IEN))) {
				break;
			}
			if (serial_in(port, ARM_UART_STS0) &
			    (UART_STS_RX_FIFO_FULL | UART_STS_BREAK_DETECT |
			     UART_STS_TIMEOUT)) {
				serial_sprd_rx_chars(irq, port);
			}
			if (serial_in(port, ARM_UART_STS0) & UART_STS_TX_FIFO_EMPTY) {
				serial_sprd_tx_chars(irq, port);
			}
			serial_out(port, ARM_UART_ICLR, 0xffffffff);
		} while (pass_counter++ < 50);
	}

	return IRQ_HANDLED;
}


#define SPRD_EICINT_BASE	(SPRD_EIC_BASE+0x80)
static irqreturn_t wakeup_rx_interrupt(int irq,void *dev_id)
{
	u32 val;


	
	val = __raw_readl(SPRD_EICINT_BASE+0x10);
	if((val&BIT(0))==BIT(0))
		val &= ~BIT(0);
	else
		val |= BIT(0);
	__raw_writel(val, SPRD_EICINT_BASE+0x10);

	
	val = __raw_readl(SPRD_EICINT_BASE+0x0C);
	val |= BIT(0);
	__raw_writel(val, SPRD_EICINT_BASE+0x0C);


	
	is_uart_rx_wakeup = true;

	return IRQ_HANDLED;
}

static void serial_sprd_pin_config(void)
{
#ifndef CONFIG_ARCH_SCX35
	unsigned int value;

	value = __raw_readl(SPRD_GREG_BASE + 0x08);
	value |= 0x07 << 20;
	__raw_writel(value, SPRD_GREG_BASE + 0x08);
#endif
}

static int serial_sprd_startup(struct uart_port *port)
{
	int ret = 0;
	unsigned int ien, ctrl1;

	
	serial_sprd_pin_config();

	
#if 0 
	serial_out(port,ARM_UART_CTL2,0x801);
#endif

#ifdef USE_BCM_BT_CHIP
	if(0 == port->line &&  SERIAL_IO_PORT == port->iotype)
	{
		I_LOG("serial_sprd_startup-------------\n");

		host_want_sleep = 0;
		bt_wakeup_assert_inadvance = 0;
		suspended_flag = 0;

		host_wakeup_level = 1;
		is_brcm_rx_wake_locked = 0;

        ret = request_threaded_irq(host_wake_irq, NULL, bt_hostwake_isr,
                                IRQF_TRIGGER_LOW|IRQF_ONESHOT|IRQF_NO_SUSPEND,
                                "bluetooth hostwake", NULL);
		if (ret < 0)
		{
            E_LOG("Couldn't acquire bt_hostwake_isr err (%d)\n", ret);
	    }
		else
		{
            enable_irq_wake(host_wake_irq);
		}

		uart0_pin_config(1);
		uart0_status_flag =1;
	}
#endif


#if defined(CONFIG_MACH_Z4DTG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
    #ifdef	CONFIG_BT_DECREASE_FIFO
		if (port->line == 1) {
			serial_out(port, ARM_UART_CTL2, ((SP_TX_FIFO << 8) | SP_RX_FIFO_EXT));
		}else if (port->line == 0) {
			serial_out(port, ARM_UART_CTL2, ((TX_HW_FLOW_CTL_THRESHOLD << 8) | SP_RX_FIFO_BT));
		}else{
			serial_out(port, ARM_UART_CTL2, ((SP_TX_FIFO << 8) | SP_RX_FIFO));
		}
	#else
		if (port->line == 1) {
			serial_out(port, ARM_UART_CTL2, ((SP_TX_FIFO << 8) | SP_RX_FIFO_EXT));
		}else{
			serial_out(port, ARM_UART_CTL2, ((SP_TX_FIFO << 8) | SP_RX_FIFO));
		}
	#endif
#else
	serial_out(port, ARM_UART_CTL2, ((SP_TX_FIFO << 8) | SP_RX_FIFO));
#endif
	
	while (serial_in(port, ARM_UART_STS1) & 0x00ff) {
		serial_in(port, ARM_UART_RXD);
	}
	
	while (serial_in(port, ARM_UART_STS1) & 0xff00) ;
	
	serial_out(port, ARM_UART_IEN, 0x00);
	serial_out(port, ARM_UART_ICLR, 0xffffffff);
	
	ret =
	    request_irq(port->irq, serial_sprd_interrupt_chars, IRQF_DISABLED,
			"serial", port);
	if (ret) {
		printk(KERN_ERR "fail to request serial irq\n");
		free_irq(port->irq, port);
	}

	if(BT_RX_WAKE_UP == plat_data.wakeup_type)
	{
		int ret2 = 0;

		if (!port->line) {
			ret2 = request_irq(IRQ_WAKEUP,wakeup_rx_interrupt,IRQF_SHARED,"wakeup_rx",port);
			if(ret2)
			{
				printk("fail to request wakeup irq\n");
				free_irq(IRQ_WAKEUP,NULL);
			}
		}
	}

	ctrl1 = serial_in(port, ARM_UART_CTL1);

#if defined(CONFIG_MACH_Z4DTG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
    #ifdef	CONFIG_BT_DECREASE_FIFO
		if (port->line == 1) {
			ctrl1 |= 0x3e00 | SP_RX_FIFO_EXT;
		}else if(port->line == 0){
			ctrl1 |= 0x3e00 | SP_RX_FIFO_BT;
		}else{
			ctrl1 |= 0x3e00 | SP_RX_FIFO;
		}
	#else
		if (port->line == 1) {
			ctrl1 |= 0x3e00 | SP_RX_FIFO_EXT;
		}else{
			ctrl1 |= 0x3e00 | SP_RX_FIFO;
		}
	#endif
#else
	ctrl1 |= 0x3e00 | SP_RX_FIFO;
#endif
	serial_out(port, ARM_UART_CTL1, ctrl1);

	spin_lock(&port->lock);
	
	ien = serial_in(port, ARM_UART_IEN);
	ien |=
	    UART_IEN_RX_FIFO_FULL | UART_IEN_TX_FIFO_EMPTY |
	    UART_IEN_BREAK_DETECT | UART_IEN_TIMEOUT;
	serial_out(port, ARM_UART_IEN, ien);
	spin_unlock(&port->lock);
	return 0;
}

static void serial_sprd_shutdown(struct uart_port *port)
{
#ifdef USE_BCM_BT_CHIP
	if(0 == port->line &&  SERIAL_IO_PORT == port->iotype)
	{
		I_LOG("serial_sprd_shutdown-------------\n");
		hrtimer_cancel(&lock_off_timer);
		
		wake_lock_timeout(&brcm_tx_wake_lock, HZ / 2);
		wake_lock_timeout(&brcm_rx_wake_lock, HZ / 2);
		

		gpio_set_value(bt_wakeup_pin, 1);
		uart0_pin_config(0);
		uart0_status_flag =0;
		disable_irq_wake(host_wake_irq);
		free_irq(host_wake_irq, NULL);
	}
#endif
	serial_out(port, ARM_UART_IEN, 0x0);
	serial_out(port, ARM_UART_ICLR, 0xffffffff);
	free_irq(port->irq, port);
}

static void serial_sprd_set_termios(struct uart_port *port,
				     struct ktermios *termios,
				     struct ktermios *old)
{
	unsigned int baud, quot;
	unsigned int lcr, fc;
	
	baud = uart_get_baud_rate(port, termios, old, 1200, 3000000);

	quot = (unsigned int) ( (port->uartclk + baud / 2) / baud);

	
	lcr = serial_in(port, ARM_UART_CTL0);
	lcr &= ~UART_DATA_BIT;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr |= UART_DATA_5BIT;
		break;
	case CS6:
		lcr |= UART_DATA_6BIT;
		break;
	case CS7:
		lcr |= UART_DATA_7BIT;
		break;
	default:
	case CS8:
		lcr |= UART_DATA_8BIT;
		break;
	}
	
	lcr &= ~(UART_STOP_1BIT | UART_STOP_2BIT);
	if (termios->c_cflag & CSTOPB)
		lcr |= UART_STOP_2BIT;
	else
		lcr |= UART_STOP_1BIT;
	
	lcr &= ~UART_PARITY;
	if (termios->c_cflag & PARENB) {
		lcr |= UART_PARITY_EN;
		if (termios->c_cflag & PARODD)
			lcr |= UART_ODD_PAR;
		else
			lcr |= UART_EVEN_PAR;
	}
	
	
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_LSR_OE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;
	
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_LSR_BI;
		
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_LSR_OE;
	}
	
#if 0 
	if((termios->c_cflag & CREAD)== 0)
		port->ignore_status_mask |= UART_LSR_DR;
#endif
	
	fc = serial_in(port, ARM_UART_CTL1);
	fc &=
	    ~(RX_HW_FLOW_CTL_THRESHOLD | RX_HW_FLOW_CTL_EN | TX_HW_FLOW_CTL_EN);
	if (termios->c_cflag & CRTSCTS) {
		fc |= RX_HW_FLOW_CTL_THRESHOLD;
		fc |= RX_HW_FLOW_CTL_EN;
		fc |= TX_HW_FLOW_CTL_EN;
	}
	
	serial_out(port, ARM_UART_CLKD0, quot & 0xffff);
	
	serial_out(port, ARM_UART_CLKD1, (quot & 0x1f0000) >> 16);
	serial_out(port, ARM_UART_CTL0, lcr);
#if defined(CONFIG_MACH_Z4DTG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_CP5DUG) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY)
    #ifdef	CONFIG_BT_DECREASE_FIFO
		if (port->line == 1) {
			fc |= 0x3e00 | SP_RX_FIFO_EXT;
		}else if (port->line == 0) {
			fc |= 0x3e00 | SP_RX_FIFO_BT;
		}else{
			fc |= 0x3e00 | SP_RX_FIFO;
		}
	#else
		if (port->line == 1) {
			fc |= 0x3e00 | SP_RX_FIFO_EXT;
		}else{
			fc |= 0x3e00 | SP_RX_FIFO;
		}
	#endif
#else
	fc |= 0x3e00 | SP_RX_FIFO;
#endif
	serial_out(port, ARM_UART_CTL1, fc);
}

static const char *serial_sprd_type(struct uart_port *port)
{
	return "SPX";
}

static void serial_sprd_release_port(struct uart_port *port)
{
}

static int serial_sprd_request_port(struct uart_port *port)
{
	return 0;
}

static void serial_sprd_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE && serial_sprd_request_port(port) == 0)
		port->type = PORT_SPRD;
}

static int serial_sprd_verify_port(struct uart_port *port,
				    struct serial_struct *ser)
{
	if (unlikely(ser->type != PORT_SPRD))
		return -EINVAL;
	if (unlikely(port->irq != ser->irq))
		return -EINVAL;
	return 0;
}
static struct uart_port *serial_sprd_ports[UART_NR_MAX] = { 0 };
static struct{
	uint32_t ien;
	uint32_t ctrl0;
	uint32_t ctrl1;
	uint32_t ctrl2;
	uint32_t clkd0;
	uint32_t clkd1;
	uint32_t dspwait;
} uart_bak[UART_NR_MAX]={0};

#ifdef USE_BCM_BT_CHIP	

static int  serial_sprd_port_brcm_on(void)
{
	return 0;
}

static int  serial_sprd_port_brcm_off(void)
{
	suspended_flag =0;

	
	if (host_want_sleep) {
		if ((bt_wakeup_level == 0)
			|| (bt_wakeup_assert_inadvance == 1)) {
			gpio_set_value(bt_wakeup_pin, 1);
			bt_wakeup_level = 1;
			bt_wakeup_assert_inadvance = 0;

			I_LOG("CHK CLK OFF, BT_WAKE=HIGH\n");
		}
	}
	else
	{
		return 0;
	}
	
	
       if (!gpio_get_value(host_wakeup_pin)) {

		I_LOG("host_wakeup_pin level=0, rx need hold \n");
		hrtimer_start(&lock_off_timer, lock_off_delay, HRTIMER_MODE_REL);

		return 0;
	}


	I_LOG("wake_unlock(brcm_rx_wake_lock)\n");
	
	if (is_brcm_rx_wake_locked == 1) {
		wake_unlock(&brcm_rx_wake_lock);
		is_brcm_rx_wake_locked = 0;
	}

	return 1;

}

static int serial_sprd_ioctl_brcm(struct uart_port * port, unsigned int cmd, unsigned long arg )
{

	I_LOG("serial_sprd_ioctl port= %x, cmd=%x\n", port->line, cmd);

	if(0 == port->line &&  SERIAL_IO_PORT == port->iotype)
	{
		void __user *argp = (void __user *)arg;
		unsigned long tbt_wakeup_level;
		unsigned long flags;

		switch (cmd) {
		case 0x8003:

			
			wake_lock(&brcm_tx_wake_lock);

			spin_lock_irqsave(&bt_rlock, flags);

			hrtimer_cancel(&lock_off_timer);


			I_LOG("HOST BT_WAKE=LOW\n");

			gpio_set_value(bt_wakeup_pin, 0);
			bt_wakeup_level = 0;

			spin_unlock_irqrestore(&bt_rlock, flags);

			serial_sprd_port_brcm_on();
			host_want_sleep = 0;

			break;

		case 0x8004:
			I_LOG("HOST BT_WAKE=HIGH\n");

			host_want_sleep = 1;

			serial_sprd_port_brcm_off();

			
			wake_lock_timeout(&brcm_tx_wake_lock, HZ / 2);

			break;

		case 0x8005:
			tbt_wakeup_level = !bt_wakeup_level;
			if (copy_to_user(argp, &tbt_wakeup_level,
					sizeof(tbt_wakeup_level)))
				return -EFAULT;
			break;

		default:
			return -ENOIOCTLCMD;
		}
	}
		return -ENOIOCTLCMD;

}

static int uart0_pin_config(int enable)
{
	if(enable)
	{
		unsigned long fc = 0;
		struct uart_port *port = serial_sprd_ports[0];
		__raw_writel( (BITS_PIN_DS(2)|BITS_PIN_AF(0)|BIT_PIN_WPU|BIT_PIN_SLP_WPU|BIT_PIN_SLP_Z), (CTL_PIN_BASE + REG_PIN_U0RTS));
		fc=serial_in(port,ARM_UART_CTL1);
		fc |=(RX_HW_FLOW_CTL_EN|TX_HW_FLOW_CTL_EN);
		serial_out(port,ARM_UART_CTL1,fc);
	}
	else
	{
		unsigned long fc = 0;
		struct uart_port *port = serial_sprd_ports[0];
		fc=serial_in(port,ARM_UART_CTL1);
		fc &=~(RX_HW_FLOW_CTL_EN|TX_HW_FLOW_CTL_EN);
		serial_out(port,ARM_UART_CTL1,fc);
		__raw_writel( (BITS_PIN_DS(3)|BITS_PIN_AF(3)|BIT_PIN_WPU|BIT_PIN_SLP_WPU|BIT_PIN_SLP_OE), (CTL_PIN_BASE + REG_PIN_U0RTS));
	}
	return 0;
}

#endif

static struct uart_ops serial_sprd_ops = {
	.tx_empty = serial_sprd_tx_empty,
	.get_mctrl = serial_sprd_get_mctrl,
	.set_mctrl = serial_sprd_set_mctrl,
	.stop_tx = serial_sprd_stop_tx,
	.start_tx = serial_sprd_start_tx,
	.stop_rx = serial_sprd_stop_rx,
	.enable_ms = serial_sprd_enable_ms,
	.break_ctl = serial_sprd_break_ctl,
	.startup = serial_sprd_startup,
	.shutdown = serial_sprd_shutdown,
	.set_termios = serial_sprd_set_termios,
	.type = serial_sprd_type,
	.release_port = serial_sprd_release_port,
	.request_port = serial_sprd_request_port,
	.config_port = serial_sprd_config_port,
	.verify_port = serial_sprd_verify_port,
};

#ifdef USE_BCM_BT_CHIP	
static struct uart_ops serial_sprd_brcm_ops = {
	.tx_empty = serial_sprd_tx_empty,
	.get_mctrl = serial_sprd_get_mctrl,
	.set_mctrl = serial_sprd_set_mctrl,
	.stop_tx = serial_sprd_stop_tx,
	.start_tx = serial_sprd_start_tx,
	.stop_rx = serial_sprd_stop_rx,
	.enable_ms = serial_sprd_enable_ms,
	.break_ctl = serial_sprd_break_ctl,
	.startup = serial_sprd_startup,
	.shutdown = serial_sprd_shutdown,
	.set_termios = serial_sprd_set_termios,
	.type = serial_sprd_type,
	.release_port = serial_sprd_release_port,
	.request_port = serial_sprd_request_port,
	.config_port = serial_sprd_config_port,
	.verify_port = serial_sprd_verify_port,
	.ioctl = serial_sprd_ioctl_brcm,
};
#endif

static int clk_startup(struct platform_device *pdev)
{
	struct clk *clk;
	struct clk *clk_parent;
	char clk_name[10];
	int ret;
	int clksrc;
	struct serial_data plat_local_data;

	sprintf(clk_name,"clk_uart%d",pdev->id);
	clk = clk_get(NULL, clk_name);
	if (IS_ERR(clk)) {
		printk("clock[%s]: failed to get clock by clk_get()!\n",
				clk_name);
		return -1;
	}

	plat_local_data = *(struct serial_data *)(pdev->dev.platform_data);
	clksrc = plat_local_data.clk;

	if (clksrc == 48000000){
		clk_parent = clk_get(NULL, "clk_48m");
	} else {
		clk_parent = clk_get(NULL, "ext_26m");
	}
	if (IS_ERR(clk_parent)) {
		printk("clock[%s]: failed to get parent [%s] by clk_get()!\n",
				clk_name, "clk_48m");
		return -1;
	}

	ret= clk_set_parent(clk, clk_parent);
	if (ret) {
		printk("clock[%s]: clk_set_parent() failed!\n", clk_name);
	}
	ret = clk_enable(clk);
	if (ret) {
		printk("clock[%s]: clk_enable() failed!\n", clk_name);
	}
	return 0;
}


#ifdef USE_BCM_BT_CHIP	
static irqreturn_t bt_hostwake_isr(int irq, void *dev_id)
{

	
	unsigned long flags;
	spin_lock_irqsave(&bt_rlock, flags);

	if (host_wakeup_level == 0) {
		I_LOG("bt_hostwake_isr host can sleep\n");

		host_wakeup_level =1;
		irq_set_irq_type(host_wake_irq, IRQF_TRIGGER_LOW| IRQF_ONESHOT|IRQF_NO_SUSPEND);
	}
	else{
		I_LOG("bt_hostwake_isr host enter run\n");

		host_wakeup_level =0;
		irq_set_irq_type(host_wake_irq, IRQF_TRIGGER_HIGH| IRQF_ONESHOT|IRQF_NO_SUSPEND);

		
		if (is_brcm_rx_wake_locked == 0) {
			is_brcm_rx_wake_locked = 1;
			wake_lock(&brcm_rx_wake_lock);
			if(suspended_flag == 1) 
				hrtimer_start(&lock_off_timer,ktime_set(0, 500000000), HRTIMER_MODE_REL);
			else
				hrtimer_start(&lock_off_timer,ktime_set(0, 200000000), HRTIMER_MODE_REL);
		}

		
		if(bt_wakeup_assert_inadvance == 0) {
			gpio_set_value(bt_wakeup_pin, 0);
			bt_wakeup_assert_inadvance = 1;
			I_LOG("SET BT_WAKE=LOW IN ADV\n");
		}

	}
	spin_unlock_irqrestore(&bt_rlock, flags);

       return IRQ_HANDLED;
}



static int hs_check_lock_off(void)
{
	suspended_flag =0;

       
	if (host_want_sleep) {
		if ((bt_wakeup_level == 0)
			|| (bt_wakeup_assert_inadvance == 1)) {
			gpio_set_value(bt_wakeup_pin, 1);
			bt_wakeup_level = 1;
			bt_wakeup_assert_inadvance = 0;

			I_LOG("CHK CLK OFF, BT_WAKE=HIGH\n");
		}
	}
       else
	{
		return 0;
	}
	
	
       if (!gpio_get_value(host_wakeup_pin)) {

		I_LOG("host_wakeup_pin level=0, rx need hold \n");
		return 0;
	}


	I_LOG("wake_unlock(brcm_rx_wake_lock) work queue\n");
	
	if (is_brcm_rx_wake_locked == 1) {
		wake_unlock(&brcm_rx_wake_lock);
		is_brcm_rx_wake_locked = 0;
	}
	   
	return 1;
}

static void hsuart_lock_off_work(struct work_struct *w)
{
	if(!hs_check_lock_off())
	{
		hrtimer_start(&lock_off_timer, lock_off_delay, HRTIMER_MODE_REL);
	}
}

static enum hrtimer_restart hs_lock_off_retry(struct hrtimer *timer)
{
	queue_work(hsuart_wq, &lock_off_w);
	return HRTIMER_NORESTART;
}





static int bt_gpio_ctrl_config(void)
{
        int ret;

        ret = gpio_request(host_wakeup_pin, "bt_host_wake");
        if (ret)
                goto free;
        ret = gpio_direction_input(host_wakeup_pin);
        if (ret < 0)
                goto free;

        ret = gpio_request(bt_wakeup_pin, "bt_ext_wake");
        if (ret)
                goto free_host_wake;
        ret = gpio_direction_output(bt_wakeup_pin, 0);
        if (ret < 0) {
                goto free_bt_wake;
        }

	I_LOG("allocat BT host_wake_pin= %d\n",host_wakeup_pin);

        host_wake_irq = gpio_to_irq(host_wakeup_pin);
	I_LOG("host_wake_irq = %d", host_wake_irq);
        if (host_wake_irq < 0) {
                E_LOG("couldn't find host_wake irq\n");
                ret = -ENODEV;
                goto free_bt_wake;
        }

	host_wakeup_level = 1;
	host_want_sleep = 0;

	bt_wakeup_level = 1;
	bt_wakeup_assert_inadvance = 0;

	wake_lock_init(&brcm_tx_wake_lock, WAKE_LOCK_SUSPEND, "hs_brcm_tx");
	wake_lock_init(&brcm_rx_wake_lock, WAKE_LOCK_SUSPEND, "hs_brcm_rx");
	
	spin_lock_init(&bt_rlock);


	hsuart_wq= create_singlethread_workqueue("btuart0_wq");
	if (!hsuart_wq) {
		E_LOG("bt create_singlethread_workqueue fail \n");
	}

	INIT_WORK(&lock_off_w, hsuart_lock_off_work);

	hrtimer_init(&lock_off_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lock_off_timer.function = hs_lock_off_retry;
	lock_off_delay = ktime_set(0, 150000000);  

	return 0;

free_bt_wake:
        gpio_free(bt_wakeup_pin);
free_host_wake:
        gpio_free(host_wakeup_pin);
free:
	return -1;
}
#endif

static int serial_sprd_setup_port(struct platform_device *pdev, struct resource *mem,
				   struct resource *irq)
{
	struct serial_data plat_local_data;
	struct uart_port *up;
	up = kzalloc(sizeof(*up), GFP_KERNEL);
	if (up == NULL) {
		return -ENOMEM;
	}

	up->line = pdev->id;
	up->type = PORT_SPRD;
	up->iotype = SERIAL_IO_PORT;
	up->membase = (void *)mem->start;
	up->mapbase = mem->start;

	plat_local_data = *(struct serial_data *)(pdev->dev.platform_data);
	up->uartclk = plat_local_data.clk;

	up->irq = irq->start;
	up->fifosize = 128;
#ifdef USE_BCM_BT_CHIP	
	if(0 == pdev -> id )
	{
		up->ops = &serial_sprd_brcm_ops;
		bt_gpio_ctrl_config();
	}
	else
#endif
	up->ops = &serial_sprd_ops;
	up->flags = ASYNC_BOOT_AUTOCONF;

	serial_sprd_ports[pdev->id] = up;

	clk_startup(pdev);
	return 0;
}

#ifdef CONFIG_SERIAL_SPRD_UART_CONSOLE
static inline void wait_for_xmitr(struct uart_port *port)
{
	unsigned int status, tmout = 10000;
	
	do {
		status = serial_in(port, ARM_UART_STS1);
		if (--tmout == 0)
			break;
		udelay(1);
	} while (status & 0xff00);
}

static void serial_sprd_console_putchar(struct uart_port *port, int ch)
{
	wait_for_xmitr(port);
	serial_out(port, ARM_UART_TXD, ch);
}

static void serial_sprd_console_write(struct console *co, const char *s,
				       unsigned int count)
{
	struct uart_port *port = serial_sprd_ports[co->index];
	int ien;
	int locked = 1;
	if (oops_in_progress)
		locked = spin_trylock(&port->lock);
	else
		spin_lock(&port->lock);
	
	ien = serial_in(port, ARM_UART_IEN);
	serial_out(port, ARM_UART_IEN, 0x0);

	uart_console_write(port, s, count, serial_sprd_console_putchar);
	
	wait_for_xmitr(port);
	serial_out(port, ARM_UART_IEN, ien);
	if (locked)
		spin_unlock(&port->lock);
}

static int __init serial_sprd_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	if (unlikely(co->index >= UART_NR_MAX || co->index < 0))
		co->index = 0;

	if ((htc_get_config(HTC_DBG_FLAG_SUPERMAN) & SUPERMAN_FLAG_DBG_UART)
			!= SUPERMAN_FLAG_DBG_UART) {
		printk(KERN_WARNING "Disable console register when console is null\n");
		return -ENODEV;
	}

	if (co->index == 1) {
		port = serial_sprd_ports[co->index];
		if (port == NULL) {
			printk(KERN_INFO "srial port %d not yet initialized\n", co->index);
			return -ENODEV;
		}
		if (options)
			uart_parse_options(options, &baud, &parity, &bits, &flow);

		return uart_set_options(port, co, baud, parity, bits, flow);
	}
	else {
		printk(KERN_WARNING "Reject console register consoel:%s%d\n", co->name, co->index);
		return -ENODEV;
	}
}

static struct uart_driver serial_sprd_reg;
static struct console serial_sprd_console = {
	.name = "ttyS",
	.write = serial_sprd_console_write,
	.device = uart_console_device,
	.setup = serial_sprd_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &serial_sprd_reg,
};

#define SPRD_CONSOLE		&serial_sprd_console

#else 
#define SPRD_CONSOLE		NULL
#endif

static struct uart_driver serial_sprd_reg = {
	.owner = THIS_MODULE,
	.driver_name = "serial_sprd",
	.dev_name = SP_TTY_NAME,
	.major = SP_TTY_MAJOR,
	.minor = SP_TTY_MINOR_START,
	.nr = UART_NR_MAX,
	.cons = SPRD_CONSOLE,
};

static int serial_sprd_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *mem, *irq;


	if (unlikely(pdev->id < 0 || pdev->id >= UART_NR_MAX)) {
		dev_err(&pdev->dev, "does not support id %d\n", pdev->id);
		return -ENXIO;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!mem)) {
		dev_err(&pdev->dev, "not provide mem resource\n");
		return -ENODEV;
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (unlikely(!irq)) {
		dev_err(&pdev->dev, "not provide irq resource\n");
		return -ENODEV;
	}

	ret = serial_sprd_setup_port(pdev, mem, irq);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "setup port failed\n");
		return ret;
	}
	ret =
	    uart_add_one_port(&serial_sprd_reg, serial_sprd_ports[pdev->id]);
	if (likely(ret == 0)) {
		platform_set_drvdata(pdev, serial_sprd_ports[pdev->id]);
	}
	if (!((void *)(pdev->dev.platform_data))) {
		dev_err(&pdev->dev, "serial driver get platform data failed\n");
		return -ENODEV;
	}
	plat_data = *(struct serial_data *)(pdev->dev.platform_data);
	printk("bt host wake up type is %d, clk is %d \n",plat_data.wakeup_type,plat_data.clk);
	if(BT_RX_WAKE_UP == plat_data.wakeup_type){
		wake_lock_init(&uart_rx_lock, WAKE_LOCK_SUSPEND, "uart_rx_lock");
	}

	return ret;
}

static int serial_sprd_remove(struct platform_device *dev)
{
	struct uart_port *up = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);
	if (up) {
		uart_remove_one_port(&serial_sprd_reg, up);
		kfree(up);
		serial_sprd_ports[dev->id] = NULL;
	}
	return 0;
}

static int serial_sprd_suspend(struct platform_device *dev, pm_message_t state)
{
	
	int id = dev->id;
	struct uart_port *port;
	if(BT_RX_WAKE_UP == plat_data.wakeup_type){
		is_uart_rx_wakeup = false;
	}else if(BT_RTS_HIGH_WHEN_SLEEP == plat_data.wakeup_type){
		if(0 == id)
		{
			unsigned long fc = 0;
			struct uart_port *port = serial_sprd_ports[0];
			fc=serial_in(port,ARM_UART_CTL1);
			fc &=~(RX_HW_FLOW_CTL_EN|TX_HW_FLOW_CTL_EN);
			serial_out(port,ARM_UART_CTL1,fc);
			__raw_writel( (BITS_PIN_DS(3)|BITS_PIN_AF(3)|BIT_PIN_WPU|BIT_PIN_SLP_WPU|BIT_PIN_SLP_OE), (CTL_PIN_BASE + REG_PIN_U0RTS));

			suspended_flag = 1;
		}
	}else{
		pr_debug("BT host wake up feature has not been supported\n");
	}
	port = serial_sprd_ports[id];
	uart_bak[id].ien = serial_in(port, ARM_UART_IEN);
	uart_bak[id].ctrl0 = serial_in(port, ARM_UART_CTL0);
	uart_bak[id].ctrl1 = serial_in(port, ARM_UART_CTL1);
	uart_bak[id].ctrl2 = serial_in(port, ARM_UART_CTL2);
	uart_bak[id].clkd0 = serial_in(port, ARM_UART_CLKD0);
	uart_bak[id].clkd1 = serial_in(port, ARM_UART_CLKD1);

	return 0;
}

static int serial_sprd_resume(struct platform_device *dev)
{
	struct clk *uart_clk;
	struct clk *uart_clk_parent;

	
	int id = dev->id;
	struct uart_port *port = serial_sprd_ports[id];
	
	port = serial_sprd_ports[id];
	serial_out(port, ARM_UART_IEN, uart_bak[id].ien);
	serial_out(port, ARM_UART_CTL0, uart_bak[id].ctrl0);
	serial_out(port, ARM_UART_CTL1, uart_bak[id].ctrl1);
	serial_out(port, ARM_UART_CTL2, uart_bak[id].ctrl2);
	serial_out(port, ARM_UART_CLKD0, uart_bak[id].clkd0);
	serial_out(port, ARM_UART_CLKD1, uart_bak[id].clkd1);

    
    if (0 == id)
    {
	    uart_clk = clk_get(NULL, "clk_uart0");
		if (IS_ERR(uart_clk)) {
			printk("clock: failed to get clk_uart0\n");
		}
		uart_clk_parent = clk_get(NULL, "clk_48m");
		if (IS_ERR(uart_clk_parent)) {
			printk("failed to get parent clk_48m\n");
		}
		clk_set_parent(uart_clk, uart_clk_parent);
		clk_enable(uart_clk);
    }

	if (2 == id)
		clk_startup(dev);

	
	if(BT_RX_WAKE_UP == plat_data.wakeup_type){
		if(is_uart_rx_wakeup)
		{
		    is_uart_rx_wakeup = false;
		    wake_lock_timeout(&uart_rx_lock, HZ / 5);	
		}
	}else if(BT_RTS_HIGH_WHEN_SLEEP == plat_data.wakeup_type){

		if(0 == id && 1== uart0_status_flag)
		{
			unsigned long fc = 0;
			struct uart_port *port = serial_sprd_ports[0];
			__raw_writel( (BITS_PIN_DS(2)|BITS_PIN_AF(0)|BIT_PIN_WPU|BIT_PIN_SLP_WPU|BIT_PIN_SLP_Z), (CTL_PIN_BASE + REG_PIN_U0RTS));
			fc=serial_in(port,ARM_UART_CTL1);
			fc |=(RX_HW_FLOW_CTL_EN|TX_HW_FLOW_CTL_EN);
			serial_out(port,ARM_UART_CTL1,fc);
		}
	}else{
		pr_debug("BT host wake up feature has not been supported\n");
	}


	return 0;
}

static struct platform_driver serial_sprd_driver = {
	.probe = serial_sprd_probe,
	.remove = serial_sprd_remove,
	.suspend = serial_sprd_suspend,
	.resume = serial_sprd_resume,
	.driver = {
		   .name = "serial_sprd",
		   .owner = THIS_MODULE,
	},
};

static int __init serial_sprd_init(void)
{
	int ret = 0;

	ret = uart_register_driver(&serial_sprd_reg);
	if (unlikely(ret != 0))
		return ret;

	ret = platform_driver_register(&serial_sprd_driver);
	if (unlikely(ret != 0))
		uart_unregister_driver(&serial_sprd_reg);

	return ret;
}

static void __exit serial_sprd_exit(void)
{
	platform_driver_unregister(&serial_sprd_driver);
	uart_unregister_driver(&serial_sprd_reg);
}

module_init(serial_sprd_init);
module_exit(serial_sprd_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sprd serial driver $Revision:1.0$");
