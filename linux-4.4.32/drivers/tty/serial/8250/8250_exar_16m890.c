/*
 *  Base port operations for 8250/16550-type serial ports
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Based on tty/serial/8250/8250_port.c
 * exar 16m890 ops will override the 8250_port ops
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/ratelimit.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/pm_runtime.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "8250.h"

#define __maybe_unused __attribute__((unused))

#ifndef CDTRDSR
#define CDTRDSR	004000000000
#endif
#define XR_RS422	0100000
#define XR_RS485	0200000

/*
 * The special register set for XR16M890 UARTs.
*/
#define XR_16M890_EXTENDED_TRIG	UART_TRG
#define XR_16M890_DLD			2
#define XR_16M890_SHR			5
#define XR_16M890_SFR			6

#define XR_FCTR_DIR_CTRL  0x08  /* Auto RS-485 Direction Control (EXAR 16M890 only) */

#define LOWBAUD_TTL_DEFAULT		1
#define LOWBAUD_RTL_DEFAULT		2
#define MIDBAUD_TTL_DEFAULT		1
#define MIDBAUD_RTL_DEFAULT		5
#define HIGHBAUD_TTL_DEFAULT	8
#define HIGHBAUD_RTL_DEFAULT	48
#define FIFOSIZE_DEFAULT		64

#define LOWBAUD_THRESHOLD 2400
#define MIDBAUD_THRESHOLD 9600
#define is_LOW_BAUDRATE(baud) (baud < LOWBAUD_THRESHOLD)
#define is_MID_BAUDRATE(baud)	\
	(baud <= MIDBAUD_THRESHOLD && baud > LOWBAUD_THRESHOLD)
#define XR_EXPORT(NAME) EXPORT_SYMBOL(serialxr_##NAME)

extern const struct serial8250_config uart_config[];
extern void serial8250_do_set_mctrl(struct uart_port *port, unsigned int mctrl);
extern void serial8250_clear_fifos(struct uart_8250_port *p);
extern void set_io_from_upio(struct uart_port *p);
extern void serial_port_out_sync(struct uart_port *p, int offset, int value);
extern void wait_for_xmitr(struct uart_8250_port *up, int bits);

/*
 *@brief    set the register bit to 1
 *@param    the bit wanna to setting begian...
 *
 * For example:
 *  SETBIT(1, 2, 3);
 *      means set bit(1), bit(2), bit(3) to 1,
 * Warning!! you cannot invoke the macro SETBIT() without params
 */
#define NUMARGS(...)    (sizeof((int[]){__VA_ARGS__})/sizeof(int))
#define SETBIT(...)    (adv_set_bit(NUMARGS(__VA_ARGS__), __VA_ARGS__))
static __maybe_unused u8 adv_set_bit(int num, ...)
{
	u8 res = 0;
	va_list ap;

	va_start(ap, num);
	while(num--)
		res |= BIT(va_arg(ap, int));
	va_end(ap);

	return res;
}
#define is_TERMIOS(flag)	((termios->c_iflag) & (flag))

static void set_fifo_trigger_level(struct uart_port *port,
										unsigned int ttl,
										unsigned int rtl)
{
//	unsigned char old_fctr;
	
//	old_fctr = serial_port_in(port, UART_FCTR);
	/*
	 * Set RTL:
	 * first, set the rx fifo trigger use table-D FCTR[5:4]=1, 
	 * and FCTR[7]=0 means setup rx
	 * Then, set RTL to TRIG register
	*/
	serial_port_out(port, UART_FCTR, UART_FCTR_TRGD & ~UART_FCTR_RX);
	serial_port_out(port, XR_16M890_EXTENDED_TRIG, rtl);
	/*
	 * Set TTL:
	 * first, set the tx fifo trigger use table-D FCTR[5:4]=1
	 * and FCTR[7]=1 means setup tx
	 * Then, set TTL to TRIG register
	*/
	serial_port_out(port, UART_FCTR, UART_FCTR_TRGD | UART_FCTR_TX);
	serial_port_out(port, XR_16M890_EXTENDED_TRIG, ttl);
}

/*
 * The exar 16m890 specific startup function,
 * reference by serail8250_do_startup()
*/
int serialxr_startup(struct uart_port *port)
{
	unsigned long flags;
	unsigned char lsr, iir;
	int retval;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct exar_priv *priv = port->private_data;

	pr_info("==== %s ====\n", __func__);

	if (!port->fifosize)
		port->fifosize = uart_config[port->type].fifo_size;
	if (!up->tx_loadsz)
		up->tx_loadsz = uart_config[port->type].tx_loadsz;
	if (!up->capabilities)
		up->capabilities = uart_config[port->type].flags;
	up->mcr = 0;

	if (port->iotype != up->cur_iotype){
		set_io_from_upio(port);
	}
	serial8250_rpm_get(up);

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial8250_clear_fifos(up);


	/* First of all, enable enhance mode */
	up->lcr = serial_port_in(port, UART_LCR);
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_EFR, UART_EFR_ECB);
	/* To clear IER register, LCR[7] must be 0 */
	up->lcr &= ~UART_LCR_DLAB;
	serial_port_out(port, UART_LCR, up->lcr);
	serial_port_out(port, UART_IER, 0);
	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial8250_clear_fifos(up);
	/*
	 * Set FIFO trigger level
	*/
//	priv->TTL = LOWBAUD_TTL_DEFAULT;
//	priv->RTL = LOWBAUD_RTL_DEFAULT;
	priv->TTL = 92;
	priv->RTL = 92;
	set_fifo_trigger_level(port, priv->TTL, priv->RTL);
	
	/*
	 * Wakeup and initilaize UART
	 * enable TX FIFO count(SPR), 
	 * need to config EMSR
	 *
	 * First: FCTR[6]=1, and use table-D FCTR[5:4]
	 * Second: LCR=0 and SFR[0]=0
	 * Finally: config EMSR
	 */
	up->lcr = serial_port_in(port, UART_LCR);
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_EFR, UART_EFR_ECB);
	serial_port_out(port, UART_FCTR, UART_FCTR_SCR_SWAP | UART_FCTR_TRGD);
	serial_port_out(port, UART_LCR, 0);
	up->sfr &= ~0x01;
	serial_port_out(port, XR_16M890_SFR, up->sfr);
	serial_port_out(port, UART_EMSR, 0x01);
	/* resume lcr */
	serial_port_out(port, UART_LCR, up->lcr);

	/*
	 * Clear the interrupt registers.
	 */
	serial_port_in(port, UART_LSR);
	serial_port_in(port, UART_RX);
	serial_port_in(port, UART_IIR);
	serial_port_in(port, UART_MSR);

	/*
	 * At this point, there's no way the LSR could still be 0xff;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */
	if (!(port->flags & UPF_BUGGY_UART) &&
	    (serial_port_in(port, UART_LSR) == 0xff)) 
	{
		printk_ratelimited(KERN_INFO "ttyS%d: LSR safety check engaged!\n",
						   serial_index(port));
		retval = -ENODEV;
		goto out;
	}

	/*
	 * FIXME: What's going on?
	*/
/*********************************************************************/
	if (port->irq) {
		unsigned char iir1;
		/*
		 * Test for UARTs that do not reassert THRE when the
		 * transmitter is idle and the interrupt has already
		 * been cleared.  Real 16550s should always reassert
		 * this interrupt whenever the transmitter is idle and
		 * the interrupt is enabled.  Delays are necessary to
		 * allow register changes to become visible.
		 */
		spin_lock_irqsave(&port->lock, flags);
		if (up->port.irqflags & IRQF_SHARED)
			disable_irq_nosync(port->irq);

		wait_for_xmitr(up, UART_LSR_THRE);
		serial_port_out_sync(port, UART_IER, UART_IER_THRI);
		udelay(1); /* allow THRE to set */
		iir1 = serial_port_in(port, UART_IIR);
		serial_port_out(port, UART_IER, 0);
		serial_port_out_sync(port, UART_IER, UART_IER_THRI);
		udelay(1); /* allow a working UART time to re-assert THRE */
		iir = serial_port_in(port, UART_IIR);
		serial_port_out(port, UART_IER, 0);

		if (port->irqflags & IRQF_SHARED)
			enable_irq(port->irq);
		spin_unlock_irqrestore(&port->lock, flags);

		/*
		 * If the interrupt is not reasserted, or we otherwise
		 * don't trust the iir, setup a timer to kick the UART
		 * on a regular basis.
		 */
		if ((!(iir1 & UART_IIR_NO_INT) && (iir & UART_IIR_NO_INT)) ||
		    up->port.flags & UPF_BUG_THRE) 
		{
			up->bugs |= UART_BUG_THRE;
		}
	}
	retval = up->ops->setup_irq(up);
	if (retval) goto out;
/**********************************************************************/
	/*
	 * Now, initialize the UART
	 */
	serial_port_out(port, UART_LCR, UART_LCR_WLEN8); //reset DLAB

	spin_lock_irqsave(&port->lock, flags);
	/*
	 * Most PC uarts need OUT2 raised to enable interrupts.
	 */
	if (port->irq)
		up->port.mctrl |= TIOCM_OUT2;

	serial8250_do_set_mctrl(port, port->mctrl);
	/* 
	 * Serial over Lan (SoL) hack:
	 * Intel 8257x Gigabit ethernet chips have a
	 * 16550 emulation, to be used for Serial Over Lan.
	 * Those chips take a longer time than a normal
	 * serial device to signalize that a transmission
	 * data was queued. Due to that, the above test generally
	 * fails. One solution would be to delay the reading of
	 * iir. However, this is not reliable, since the timeout
	 * is variable. So, let's just don't test if we receive
	 * TX irq. This way, we'll never enable UART_BUG_TXEN.
	*/
	if (up->port.flags & UPF_NO_TXEN_TEST)
		goto dont_test_tx_en;

	/*
	 * Do a quick test to see if we receive an
	 * interrupt when we enable the TX irq.
	 */
	serial_port_out(port, UART_IER, UART_IER_THRI);
	lsr = serial_port_in(port, UART_LSR);
	iir = serial_port_in(port, UART_IIR);
	serial_port_out(port, UART_IER, 0);
	if (lsr & UART_LSR_TEMT && iir & UART_IIR_NO_INT) {
		if (!(up->bugs & UART_BUG_TXEN)) {
			up->bugs |= UART_BUG_TXEN;
			pr_debug("ttyS%d - enabling bad tx status workarounds\n",
				 serial_index(port));
		}
	} else {
		up->bugs &= ~UART_BUG_TXEN;
	}

dont_test_tx_en:
	spin_unlock_irqrestore(&port->lock, flags);
	/*
	 * Clear the interrupt registers again for luck, and clear the
	 * saved flags to avoid getting false values from polling
	 * routines or the previous session.
	 */
	serial_port_in(port, UART_LSR);
	serial_port_in(port, UART_RX);
	serial_port_in(port, UART_IIR);
	serial_port_in(port, UART_MSR);
	up->lsr_saved_flags = 0;
	up->msr_saved_flags = 0;

	/*
	 * Request DMA channels for both RX and TX.
	 */
	if (up->dma) {
		retval = serial8250_request_dma(up);
		if (retval) {
			pr_warn_ratelimited("ttyS%d - failed to request DMA\n",
					    serial_index(port));
			up->dma = NULL;
		}
	}
	/*
	 * Set the IER shadow for rx interrupts but defer actual interrupt
	 * enable until after the FIFOs are enabled; otherwise, an already-
	 * active sender can swamp the interrupt handler with "too much work".
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	
	if (port->flags & UPF_FOURPORT) {
		unsigned int icp;
		/* Enable interrupts on the AST Fourport board */
		icp = (port->iobase & 0xfe0) | 0x01f;
		outb_p(0x80, icp);
		inb_p(icp);
	}
	retval = 0;
out:
	serial8250_rpm_put(up);
	
	return retval;
}
XR_EXPORT(startup);

/*
 * Reference by 8250_port.c
*/ 
void serialxr_shutdown(struct uart_port *port)
{
	unsigned long flags;
	struct uart_8250_port *up = up_to_u8250p(port);

	pr_info("==== %s ====\n", __func__);

	serial8250_rpm_get(up); 
	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_port_out(port, UART_IER, 0);

	if (up->dma)
		serial8250_release_dma(up);

	spin_lock_irqsave(&port->lock, flags);

	port->mctrl &= ~TIOCM_OUT2;
	serial8250_do_set_mctrl(port, port->mctrl);

	spin_unlock_irqrestore(&port->lock, flags);
	/*
	 * Disable break condition and FIFOs
	 */
	serial_port_out(port, UART_LCR,
			serial_port_in(port, UART_LCR) & ~UART_LCR_SBC);
	serial8250_clear_fifos(up);
	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	serial_port_in(port, UART_RX);
	
	serial8250_rpm_put(up);
	/* free irq */
	up->ops->release_irq(up);
}
XR_EXPORT(shutdown);

/*
 * Reference by serial8250_compute_lcr() form 8250_port.c
*/ 
static unsigned char 
_compute_lcr(struct uart_8250_port *port, tcflag_t c_cflag)
{
	unsigned char cval;
	
	switch (c_cflag & CSIZE) {
		case CS5:
			cval = UART_LCR_WLEN5;
			break;
		case CS6:
			cval = UART_LCR_WLEN6;
			break;
		case CS7:
			cval = UART_LCR_WLEN7;
			break;
		default:
		case CS8:
			cval = UART_LCR_WLEN8;
			break;
	}
	
	if (c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	
	if (c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	
	if (!(c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif
	return cval;
}

/*
 * Reference by serial8250_get_baud_rate() from 8250_port.c 
*/
static inline unsigned int _get_baud_rate(struct uart_port *port,
										struct ktermios *termios,
										struct ktermios *old)
{
	unsigned int tolerance = port->uartclk / 100;
	
	/*
	 * Ask the core to calculate the divisor for us.
	 * Allow 1% tolerance at the upper limit so uart clks marginally
	 * slower than nominal still match standard baud rates without
	 * causing transmission errors.
	 */
	return uart_get_baud_rate(port, termios, old,
							port->uartclk/16/0xffff,
							(port->uartclk + tolerance)/16);
}

#define is_RS232()	(!is_TERMIOS(XR_RS422) && !is_TERMIOS(XR_RS485))
static void auto_flow_control(struct uart_port *port,
								struct exar_priv *priv,
								struct ktermios *termios)
{
	unsigned char efr;
	struct uart_8250_port *up = up_to_u8250p(port);

	efr = UART_EFR_ECB;
	up->lcr = serial_port_in(port, UART_LCR); 

	efr = UART_EFR_ECB;
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	if(is_TERMIOS(CRTSCTS))
		serial_port_out(port, UART_EFR, UART_EFR_CTS);
	else
		serial_port_out(port, UART_EFR, 0);

	if(port->flags & ASYNC_HARDPPS_CD)
		up->ier |= UART_IER_MSI;

	if(is_TERMIOS(CLOCAL))
		port->flags &= ~ASYNC_CHECK_CD;
	else
		port->flags |= ASYNC_CHECK_CD;
	
	serial_port_out(port, UART_LCR, up->lcr);

	if(is_TERMIOS(IXOFF)){
		/*
		 * Enable xon/xoff, LCR=0xBF & SFR[0]=0
		 * make sure SFR[0] = 0
		*/
		efr |= 0xB;
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_XON1, START_CHAR(port->state->port.tty));
		serial_port_out(port, UART_XON2, START_CHAR(port->state->port.tty));
		serial_port_out(port, UART_XOFF1, STOP_CHAR(port->state->port.tty));
		serial_port_out(port, UART_XOFF2, STOP_CHAR(port->state->port.tty));
		//resume sfr
		serial_port_out(port, UART_LCR, up->lcr);
	}

	if(is_TERMIOS(IXANY))
		up->mcr |= UART_MCR_XONANY;
	serial8250_do_set_mctrl(port, port->mctrl);

	if(is_RS232()){
		/* rs232 mode, pull the relative GPIO to high */
		gpio_direction_output(priv->gpio_sel, 1);
		
		/* enable/disable RTS/CTS */
		if(is_TERMIOS(CRTSCTS)){
			efr |= UART_EFR_CTS | UART_EFR_RTS;
			up->ier &= ~UART_IER_MSI;
		}else{
			up->ier |= UART_IER_MSI;
		}
	}else{
		/* rs422/485 mode pull the relative GPIO to low */
		gpio_direction_output(priv->gpio_sel, 0);
		if(is_TERMIOS(XR_RS485)){
			unsigned char fctr;
			/* set RS485 auto direction control */
			serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
			fctr = serial_port_in(port, UART_FCTR);
			serial_port_out(port, UART_FCTR, fctr | XR_FCTR_DIR_CTRL);
			serial_port_out(port, UART_LCR, 0);	
		}
		up->ier &= ~UART_IER_MSI;
	}

	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_EFR, efr);
	serial_port_out(port, UART_LCR, up->lcr);

	if(is_RS232()){
		/* set FCL and FCH by Hysteresis Level */
		serial_port_out(port, XR_16M890_SHR, 0x09);
	}else{
		/* set RS-485 Turn-Around Delay */
		serial_port_out(port, XR_16M890_SHR, 0);
	}
	
}
#undef is_RS232
void serialxr_set_termios(struct uart_port *port, 
							struct ktermios *termios,
		          			struct ktermios *old)
{
	unsigned char cval, dld;
	unsigned long flags;
	unsigned int baud, quot, frac;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct exar_priv *priv = port->private_data;	

	pr_info("=== %s ===\n", __func__);

	cval = _compute_lcr(up, termios->c_cflag);
	baud = _get_baud_rate(port, termios, old);
	if(unlikely(!baud)) baud = 9600;
	/*
	 * Determine divisor base on baud rate
	*/	
	quot = uart_get_divisor(port, baud);
	if(unlikely(!quot)) 
		quot = port->uartclk/16 / 9600;

	/***************************************
	 * TODO: if anybaudrate,
	 * baud = anybaudrate;
	 * quot = port->uartclk/16 / baud
	*****************************************/

	/* Finally, enable interrupts */
	up->ier = UART_IER_MSI | UART_IER_RLSI | 
			UART_IER_RDI | UART_IER_RDITO;
	/* 
	 * auto flow control setting
	*/
	auto_flow_control(port, priv, termios);
	serial_port_out(port, UART_IER, up->ier);

	/*
	 * Set RTL/TTL
	 * FIXME: assume the get_baud_rate is right 
	 * TODO: if change TTL/RTL in ioctl?
	*/
	if(is_LOW_BAUDRATE(baud)){
		up->fcr |= UART_FCR_TRIGGER_1;
		priv->TTL = LOWBAUD_TTL_DEFAULT;
		priv->RTL = LOWBAUD_RTL_DEFAULT;
	}else if(is_MID_BAUDRATE(baud)){
		up->fcr |= UART_FCR_TRIGGER_1;
		priv->TTL = MIDBAUD_TTL_DEFAULT;
		priv->RTL = MIDBAUD_RTL_DEFAULT;
	}else{
		up->fcr |= UART_FCR_TRIGGER_8;
		priv->TTL = HIGHBAUD_TTL_DEFAULT;
		priv->RTL = HIGHBAUD_RTL_DEFAULT;
	}
	up->lcr = serial_port_in(port, UART_LCR);
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	set_fifo_trigger_level(port, priv->TTL, priv->RTL);
	serial_port_out(port, UART_LCR, up->lcr);
	serial_port_out(port, UART_FCR, up->fcr);

	port->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (is_TERMIOS(INPCK))
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (is_TERMIOS(IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;
	
	/* To set fifo size */
	//TODO: if change fifo size?
	priv->fifosize = FIFOSIZE_DEFAULT;
	up->lcr = cval;			/* Save computed LCR */
	
	serial8250_rpm_get(up);
	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&port->lock, flags);
	
	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Characteres to ignore
	 */
	port->ignore_status_mask = 0;
	if (is_TERMIOS(IGNPAR))
		port->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (is_TERMIOS(IGNBRK)){
		port->ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if(is_TERMIOS(IGNPAR))
			port->ignore_status_mask |= UART_LSR_OE;
	}
	/*
	 * ignore all characters if CREAD is not set
	 */
	if (!is_TERMIOS(CREAD))
		port->ignore_status_mask |= UART_LSR_DR;
	
	/* 
	 * Set divisor
	*/
	/***************************** 
	 *TODO: if any baud rate,
	 *	need to set DLD
	********************************/
	//serial_port_out(port, XR_16M890_DLD, (quot&0x0f)|dld);
	serial_port_out(port, UART_LCR, UART_LCR_DLAB);
	serial_port_out(port, UART_DLL, quot & 0xff);	/* LS of divisor */
	serial_port_out(port, UART_DLM, quot >> 8);		/* MS of divisor */
	/*
	 * LCR DLAB must be set to enable 64-byte FIFO mode. If the FCR
	 * is written without DLAB set, this mode will be disabled.
	 */
	serial_port_out(port, UART_LCR, up->lcr);	/* reset DLAB */

	spin_unlock_irqrestore(&port->lock, flags);
	
	serial8250_rpm_put(up);
	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
}
XR_EXPORT(set_termios);

void serialxr_throttle(struct uart_port *port)
{
	//TODO: implement it!
	pr_info("%s: not implement!\n", __func__);
}
XR_EXPORT(throttle);

void serialxr_unthrottle(struct uart_port *port)
{
	//TODO: implement it!
	pr_info("%s: not implement!\n", __func__);
}
XR_EXPORT(unthrottle);

int serialxr_ioctl(struct uart_port *port,
					unsigned int cmd, 
					unsigned long arg)
{
	//TODO: implement it!
	pr_info("%s: not implement!\n", __func__);

	return 0;
}
XR_EXPORT(ioctl);
#undef is_TERMIOS


