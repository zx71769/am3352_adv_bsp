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
#include <linux/types.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "8250.h"

#define XR_RS422	0100000
#define XR_RS485	0200000

//#define ADV_DEBUG

#ifdef ADV_DEBUG
#define adv_info(str, args...) pr_info("" str, ##args)
#else
#define adv_info(str, args...) do{ }while(0)
#endif

/*
 * The special register set for XR16M890 UARTs.
*/
#define XR_16M890_EXTENDED_TRIG	UART_TRG
#define XR_16M890_DLD			2
#define XR_16M890_SHR			5
#define XR_16M890_SFR			6
#define XR_FCTR_DIR_CTRL  0x08  /* Auto RS-485 Direction Control */
#define XR_MCR_PRESCALER  0x80

/*
 * The ioctl command
*/
#define ADVSENDXON				0x546C
#define ADVSENDXOFF				0x546D
#define ADVTIOCSERGCHARTIMEOUT	0x5470
#define ADVTIOCSETRTL			0x5460
#define ADVTIOCSETTTL			0x5461
#define ADVTIOCSETFCL			0x5462
#define ADVTIOCSETFCH			0x5463
#define ADVTIOCSETFIFOSIZE		0x5464
#define ADVTIOCSETCHANGFLAG		0x5465
#define ADVTIOCRTURDFRAME		0x5471
#define ADVTIOCSETBAUDRATE		0x546E


#define LOWBAUD_TTL_DEFAULT		UART_TRG_1
#define LOWBAUD_RTL_DEFAULT		UART_TRG_2
#define MIDBAUD_TTL_DEFAULT		UART_TRG_1
#define MIDBAUD_RTL_DEFAULT		UART_TRG_5
#define HIGHBAUD_TTL_DEFAULT 	UART_TRG_8
#define HIGHBAUD_RTL_DEFAULT 	UART_TRG_48
#define XR_PASS_LIMIT			256

#define LOWBAUD_THRESHOLD		2400
#define MIDBAUD_THRESHOLD		9600
#define is_LOW_BAUDRATE(baud) (baud < LOWBAUD_THRESHOLD)
#define is_MID_BAUDRATE(baud)	\
	(baud <= MIDBAUD_THRESHOLD && baud > LOWBAUD_THRESHOLD)

#define XR_EXPORT(NAME) EXPORT_SYMBOL(serialxr_##NAME)

extern const struct serial8250_config uart_config[];
extern void serial8250_do_set_mctrl(struct uart_port *port, 
										unsigned int mctrl);
extern void serial8250_clear_fifos(struct uart_8250_port *p);
extern void set_io_from_upio(struct uart_port *p);
extern void serial_port_out_sync(struct uart_port *p, 
									int offset, int value);
extern void wait_for_xmitr(struct uart_8250_port *up, int bits);
extern unsigned int serial8250_modem_status(struct uart_8250_port *up);
extern unsigned int serial8250_get_baud_rate(struct uart_port *port, 
										struct ktermios *termios, 
										struct ktermios *old);
extern unsigned char serial8250_compute_lcr(struct uart_8250_port *up, 
										tcflag_t c_cflag);

struct irq_info {
	struct				hlist_node node;
	int					irq;  
	spinlock_t			lock;   /* Protects list not the hash */ 
	struct list_head	*head;
};

static void set_fifo_trigger_level(struct uart_port *port,
										unsigned int ttl,
										unsigned int rtl)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	
	up->fctr = serial_port_in(port, UART_FCTR);
	/*
	 * Set RTL:
	 * first, set the rx fifo trigger use table-D FCTR[5:4]=1, 
	 * and FCTR[7]=0 means setup rx
	 * Then, set RTL to TRIG register
	*/
	serial_port_out(port, UART_FCTR, 
					UART_FCTR_TRGD | (~UART_FCTR_TX & up->fctr));
	serial_port_out(port, XR_16M890_EXTENDED_TRIG, rtl);
	/*
	 * Set TTL:
	 * first, set the tx fifo trigger use table-D FCTR[5:4]=1
	 * and FCTR[7]=1 means setup tx
	 * Then, set TTL to TRIG register
	*/
	serial_port_out(port, UART_FCTR, 
				UART_FCTR_TRGD | UART_FCTR_TX | up->fctr);
	serial_port_out(port, XR_16M890_EXTENDED_TRIG, ttl);
}

static inline void private_data_init(struct uart_port *port)
{
	struct exar_priv *priv = port->private_data;
	
	priv->throttle = false;
	priv->mbusreadmode = false;
	priv->charto = 0;
}

void serialxr_rx_chars_RDI(struct uart_8250_port *up)
{
	int i;
	unsigned int bytes_in_fifo;
	unsigned char ch;
	struct uart_port *port = &up->port;
	struct tty_port *tty = &port->state->port;
	struct exar_priv *priv = port->private_data;

	adv_info("=== %s ===\n", __func__);

	if(priv->throttle) return; //need?

	bytes_in_fifo = priv->RTL-1;

	for(i = 0; i < bytes_in_fifo; ++i){
		ch = serial_port_in(port, UART_RX);
		tty_insert_flip_char(tty, ch, TTY_NORMAL);
		port->icount.rx++;
	}
	priv->rx = port->icount.rx;

	if(priv->mbusreadmode) return;

	tty_flip_buffer_push(tty);

	DEBUG_INTR(" LSR_DR...");
}

void serialxr_rx_chars(struct uart_8250_port *up, unsigned char lsr)
{
	unsigned char ch;
	int max_count = 256;
	struct uart_port *port = &up->port;
	struct tty_port *tty = &port->state->port;
	struct exar_priv *priv = port->private_data;

	adv_info("=== %s ===\n", __func__);

	if(priv->throttle) return; //need?

	do{
		if(likely(lsr & UART_LSR_DR)){
			ch = serial_port_in(port, UART_RX);
		}else{
			/*
			 * Intel 82571 has a Serial Over Lan device that will
			 * set UART_LSR_BI without setting UART_LSR_DR when
			 * it receives a break. To avoid reading from the
			 * receive buffer without UART_LSR_DR bit set, we
			 * just force the read character to be 0
			 */
			ch = 0;
		}
		
		port->icount.rx++;

		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

		tty_insert_flip_char(tty, ch, TTY_NORMAL);

ignore_char:
		lsr = serial_in(up, UART_LSR);
	}while((lsr & UART_LSR_DR) && (--max_count > 0));
	
	priv->rx = port->icount.rx;

	tty_flip_buffer_push(tty);

	DEBUG_INTR(" LSR_DR...");
}

void serialxr_tx_chars(struct uart_8250_port *up)
{
	int count;
	int bytes_in_fifo;
	struct uart_port *port = &up->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct exar_priv *priv = port->private_data;

	adv_info("=== %s ===\n", __func__);
	
	if(port->x_char){
		serial_port_out(port, UART_TX, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		priv->tx = port->icount.tx;
		return;
	}
	
	if(uart_tx_stopped(port)){
		port->ops->stop_tx(port);
		return;
	}
	
	if(uart_circ_empty(xmit)){
		port->ops->stop_tx(port);
		return;
	}
	/*
	 * We already choose scratrch pad reg. to show the
	 * how many data byte in tx fifo, so just read it
	*/
	bytes_in_fifo = serial_port_in(port, UART_SCR);
	
	count = min((unsigned int)(up->tx_loadsz), 
				(unsigned int)(port->fifosize - bytes_in_fifo));
	do{
		serial_port_out(port, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit)) break;
	}while (--count > 0);
	priv->tx = port->icount.tx;

	if(uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	DEBUG_INTR("THRE...");

	if(uart_circ_empty(xmit)){
		port->ops->stop_tx(port);
	}else{
		unsigned int THR;
		THR = serial_port_in(port, UART_SCR);
		if(THR < (bytes_in_fifo + count) &&
			THR < priv->TTL)
		{
			/* Try to re-trigger the tx empty interrupt */
			serial_port_out(port, UART_IER, up->ier & (~UART_IER_THRI));
			serial_port_out(port, UART_IER, up->ier);
		}
	}
}
XR_EXPORT(tx_chars);
/*
 * The exar 16m890 specific startup function,
 * reference by serail8250_do_startup()
*/
int serialxr_startup(struct uart_port *port)
{
	int retval;
	unsigned long flags;
	unsigned char lsr, iir, fctr;
	struct uart_8250_port *up = up_to_u8250p(port);

	adv_info("=== %s ===\n", __func__);

	if (!port->fifosize)
		port->fifosize = uart_config[port->type].fifo_size;
	if (!up->tx_loadsz)
		up->tx_loadsz = uart_config[port->type].tx_loadsz;
	if (!up->capabilities)
		up->capabilities = uart_config[port->type].flags;
	
	private_data_init(port);
	
	up->mcr = 0;

	if (port->iotype != up->cur_iotype)
		set_io_from_upio(port);

	serial8250_clear_fifos(up);

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

	/* Enable enhance mode */ 
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_EFR, UART_EFR_ECB);
	/* Set tx/rx fifo trigger level */
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	fctr = serial_in(up, UART_FCTR) & ~(UART_FCTR_RX|UART_FCTR_TX);
	serial_port_out(port, UART_FCTR,
			fctr | UART_FCTR_TRGD | UART_FCTR_RX);
	serial_port_out(port, UART_TRG, UART_TRG_96);
	serial_port_out(port, UART_FCTR,
			fctr | UART_FCTR_TRGD | UART_FCTR_TX);
	serial_port_out(port, UART_TRG, UART_TRG_96);
	serial_port_out(port, UART_LCR, 0);
	/*
	 * Wakeup and initilaize UART
	 * enable TX FIFO count(SPR),
	 * to let tx read how many data in tx-fifo dirctily
	 *
	 * First: FCTR[6]=1, and use table-D FCTR[5:4]
	 * Second: LCR=0 and SFR[0]=0
	 * Finally: config EMSR
	 */
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_EFR, UART_EFR_ECB);
	serial_port_out(port, UART_LCR, 0);
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_FCTR, UART_FCTR_SCR_SWAP | UART_FCTR_TRGD);
	serial_port_out(port, UART_LCR, 0);
	up->sfr &= ~0x01;
	serial_port_out(port, XR_16M890_SFR, up->sfr);
	serial_port_out(port, UART_EMSR, 0x01);
/*******************************************************************/
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
/**********************************************************************/
	retval = up->ops->setup_irq(up);
	if (retval) goto out;

	/*
	 * Now, initialize the UART
	 */
	serial_port_out(port, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&port->lock, flags);
	/* Most PC uarts need OUT2 raised to enable interrupts */
	if (port->irq)
		up->port.mctrl |= TIOCM_OUT2;

	serial8250_do_set_mctrl(port, port->mctrl);

	/* Serial over Lan (SoL) hack:
	   Intel 8257x Gigabit ethernet chips have a
	   16550 emulation, to be used for Serial Over Lan.
	   Those chips take a longer time than a normal
	   serial device to signalize that a transmission
	   data was queued. Due to that, the above test generally
	   fails. One solution would be to delay the reading of
	   iir. However, this is not reliable, since the timeout
	   is variable. So, let's just don't test if we receive
	   TX irq. This way, we'll never enable UART_BUG_TXEN.
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
	 * Set the IER shadow for rx interrupts but defer actual interrupt
	 * enable until after the FIFOs are enabled; otherwise, an already-
	 * active sender can swamp the interrupt handler with "too much work".
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI;

	retval = 0;
out:
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

	adv_info("==== %s ====\n", __func__);

	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_port_out(port, UART_IER, 0);

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
	
	/* free irq */
	up->ops->release_irq(up);
}
XR_EXPORT(shutdown);

/*
 * flow control, modem status and rs232/422/485 interrupt
*/
static void flow_control_handle(struct uart_port *port,
								struct exar_priv *priv,
								struct ktermios *termios)
{
	unsigned char efr;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct tty_struct *tty = port->state->port.tty;

	if(termios->c_iflag & CLOCAL)
		port->flags &= ~ASYNC_CHECK_CD;
	else
		port->flags |= ASYNC_CHECK_CD;

	efr = UART_EFR_ECB;

	if(I_IXOFF(tty)){
		/*
		 * Enable xon/xoff, LCR=0xBF & SFR[0]=0
		 * make sure SFR[0] = 0
		*/
		efr |= 0xB;
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_XON1, START_CHAR(tty));
		serial_port_out(port, UART_XON2, START_CHAR(tty));
		serial_port_out(port, UART_XOFF1, STOP_CHAR(tty));
		serial_port_out(port, UART_XOFF2, STOP_CHAR(tty));
		serial_port_out(port, UART_LCR, 0);
	}

	if(I_IXANY(tty))
		up->mcr |= UART_MCR_XONANY;

	if(termios->c_iflag & (XR_RS422 | XR_RS485)){
		adv_info("Port ttyS%d under rs422/485 mode\n", port->line);
		/* rs422/485 mode pull the relative GPIO to low */
		gpio_direction_output(priv->gpio_sel, 0);

		if(termios->c_iflag & XR_RS485){
			unsigned char fctr;
			/* set RS485 auto direction control */
			serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
			fctr = serial_port_in(port, UART_FCTR);
			serial_port_out(port, UART_FCTR, fctr | XR_FCTR_DIR_CTRL);
			serial_port_out(port, UART_LCR, 0);
			priv->type = PORT_RS485;
		}else
			priv->type = PORT_RS422;
		up->ier &= ~UART_IER_MSI;
	}else{
		adv_info("Port ttyS%d under rs232 mode\n", port->line);
		/* rs232 mode, pull the relative GPIO to high */
		gpio_direction_output(priv->gpio_sel, 1);
		
		/* enable/disable RTS/CTS */
		if(termios->c_cflag & CRTSCTS){
			efr |= UART_EFR_CTS | UART_EFR_RTS;
			up->ier &= ~UART_IER_MSI;
		}else{
			up->ier |= UART_IER_MSI;
		}
		priv->type = PORT_RS232;
	}
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_EFR, efr);
	serial_port_out(port, UART_LCR, 0);

	if(termios->c_iflag & (XR_RS422 | XR_RS485)){
		/* set RS-485 Turn-Around Delay */
		serial_port_out(port, XR_16M890_SHR, 0);
	}else{
		/* set FCL and FCH by Hysteresis Level */
		serial_port_out(port, XR_16M890_SHR, 0x09);
	}
}
/*
 * Programmable Baud Rate
 * retrun the divisor and set to DLL/DLD/DLM
*/
static int anybaudrate_get_divisor(struct uart_port *port, 
									unsigned int baudrate)
{
	int i, j;
	const int div_min = 1;
	const int div_max = 0xffff;
	const int ps[] = {1, 4};
	const int sp[] = {8, 16};
	const int dld_min = 0;
	const int dld_max = 15;
	int baud = (int)baudrate;
	int min_delta = 0;
	int dld;
	unsigned int prescaler;
	unsigned int sc;
	unsigned int divisor;
	unsigned int frag_divisor;
	unsigned int tmp_mcr, tmp_efr, tmp_lcr;

	min_delta = baud;	

	/*
	 * Try to try every possible sampling rate,
	 * and find the best!
	*/
	for(i = 0; i < ARRAY_SIZE(ps); i++){
		for(j = 0; j < ARRAY_SIZE(ps); j++){
			/* divisor = (clock/1) / (baudrate*sampling rate) */
			int tmp_div = port->uartclk / (baud*ps[i]*sp[j]);
			int tmp_baud;
			int tmp_delt;
			int head = dld_min;
			int tail = dld_max;
	
			if(tmp_div > div_max || tmp_div < div_min)
				continue;
		
			do{
				dld = head + (tail-head)/2;
				tmp_baud = (16*port->uartclk) / 
							(16*tmp_div*ps[i]*sp[j] + ps[i]*sp[j]*dld);
				tmp_delt = abs(baud-tmp_baud);
				if(tmp_delt < min_delta){
					min_delta = tmp_delt;
					prescaler = (unsigned int)ps[i];
					sc = (unsigned int)sp[j];
					divisor = (unsigned int)tmp_div;
					frag_divisor = (unsigned int)dld;
				}
				if(baud < tmp_baud)
					head = dld + 1;
				else
					tail = dld;
			}while(head < tail);
		}
	}

	adv_info("%s: anybaud %d, DLD %d, divisor %d, SC %d, prescaler %d\n",
				__func__, baud, frag_divisor, divisor, sc, prescaler);
	/* 
	 * MCR: Clock Prescaler Select 
	 * MCR[7]=0 : Divide by one
	 * MCR[7]=1 : Divide by four
	*/
	tmp_mcr = serial_port_in(port, UART_MCR);
	if(prescaler == 0x04)
		serial_port_out(port, UART_MCR, tmp_mcr | XR_MCR_PRESCALER);
	else
		serial_port_out(port, UART_MCR, tmp_mcr & ~(XR_MCR_PRESCALER));

	/* enable enhance mode */	
	tmp_lcr = serial_port_in(port, UART_LCR);
	serial_port_out(port, UART_LCR, UART_LCR_DLAB);
	tmp_efr = serial_port_in(port, UART_EFR);
	serial_port_out(port, UART_EFR, tmp_efr | UART_EFR_ECB);
	/* Set DLD, the sampling rate and fractional baud rate divisor */
	dld = 0;
	dld |= (0xf & (unsigned char)frag_divisor);
	dld &= 0xcf;	//clear DLD[4:5]
	if(sc == 4)
		dld |= 0x20;	//sampling rate 4x
	else if(sc == 8)
		dld |= 0x10;	//sampling rate 8x
	else
		pr_err("%s: sc = 16? %d\n", __func__, sc);
	
	serial_port_out(port, XR_16M890_DLD, dld);
	serial_port_out(port, UART_LCR, tmp_lcr);
	
	return divisor;
}

void serialxr_set_termios(struct uart_port *port, 
							struct ktermios *termios,
		         			struct ktermios *old)
{
	unsigned char cval;
	unsigned long flags;
	unsigned int baud, quot;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct exar_priv *priv = port->private_data;

	adv_info("==== %s ====\n", __func__);

	cval = serial8250_compute_lcr(up, termios->c_cflag);
	baud = serial8250_get_baud_rate(port, termios, old);
	if(unlikely(baud == 0)) 
		baud = 9600;
//	quot = uart_get_divisor(port, baud);

	up->lcr = cval;		/* Save computed LCR */

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&port->lock, flags);
	
	/* Update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;
	
	/* Characteres to ignore */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_LSR_OE;
	}

	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_LSR_DR;

	flow_control_handle(port, priv, termios);

	adv_info("set mcr = 0x%0X, ier = 0x%0X\n", up->mcr, up->ier);
	serial_port_out(port, UART_MCR, up->mcr);
	serial_port_out(port, UART_IER, up->ier);
	/*
	 * Set RTL/TTL
	 * TODO: if change TTL/RTL in ioctl?
	 */
	up->fcr = UART_FCR_ENABLE_FIFO;
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
	/* To set trigger level, need to enable fifo first */
	serial_port_out(port, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	set_fifo_trigger_level(port, priv->TTL, priv->RTL);
	serial_port_out(port, UART_LCR, 0);
	/* emulated UARTs (Lucent Venus 167x) need two steps */
	serial_port_out(port, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_port_out(port, UART_FCR, up->fcr);	/* set fcr */

	quot = anybaudrate_get_divisor(port, baud);
	if(unlikely(quot == 0)) 
		quot = (port->uartclk/16/2) / 9600;

	serial_port_out(port, UART_LCR, UART_LCR_DLAB);
	serial_port_out(port, UART_DLL, quot & 0xff);
	serial_port_out(port, UART_DLM, quot >> 8);
	serial_port_out(port, UART_LCR, up->lcr);	/* reset DLAB */

	serial8250_do_set_mctrl(port, port->mctrl);

	spin_unlock_irqrestore(&port->lock, flags);

	priv->baud = baud;
	
	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
}
XR_EXPORT(set_termios);

void serialxr_throttle(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	struct exar_priv *priv = port->private_data;	

	if(!priv->throttle){
		spin_lock_irq(&up->port.lock);
		up->ier &= ~(UART_IER_RLSI | UART_IER_RDI);
		serial_port_out(port, UART_IER, up->ier);
		spin_unlock_irq(&up->port.lock);
		priv->throttle = true;
	}
}
XR_EXPORT(throttle);

void serialxr_unthrottle(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	struct exar_priv *priv = port->private_data;
	
	if(priv->throttle){
		spin_lock_irq(&up->port.lock);
		up->ier |= UART_IER_RLSI | UART_IER_RDI;
		serial_port_out(port, UART_IER, up->ier);
		spin_unlock_irq(&up->port.lock);
		priv->throttle = false;
	}
}
XR_EXPORT(unthrottle);

static inline int serialxr_send1char(struct uart_port *port,
									unsigned char ch)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	port->x_char = ch;
	serialxr_tx_chars(up);
	
	return 0;
}

int serialxr_ioctl(struct uart_port *port, unsigned int cmd, 
					unsigned long arg)
{
	int ret = -ENOIOCTLCMD;
	struct tty_struct *tty = port->state->port.tty;
	struct exar_priv *priv = port->private_data;

	switch (cmd) {
		case ADVSENDXON:
			pr_info("Ioctl: send XON char\n");
			ret = serialxr_send1char(port, tty->termios.c_cc[VSTART]);
			break;
		
		case ADVSENDXOFF:
			pr_info("Ioctl: send XOFF char\n");
			ret = serialxr_send1char(port, tty->termios.c_cc[VSTOP]);
			break;
	
		case ADVTIOCSERGCHARTIMEOUT:
			if(copy_to_user((void *)arg, &priv->charto, 
									sizeof(unsigned int)))
				return -EFAULT;
			if(*((unsigned int *)arg) != 0)
				priv->charto = 0;
			ret = 0;
			break;
		
		case ADVTIOCRTURDFRAME:
		{
			unsigned long tmp;
			
			if(copy_from_user((void *)(&tmp), (void *)arg,
								sizeof(unsigned long)))
			{
				ret = -EFAULT;
				break;
			}
			if(tmp == 1)
				priv->mbusreadmode = true;
			else
				priv->mbusreadmode = false;
			
			ret = 0;
			break;
		}
		
		case ADVTIOCSETRTL:
		case ADVTIOCSETTTL:
		case ADVTIOCSETFIFOSIZE:
		case ADVTIOCSETCHANGFLAG:
			ret = 0;
			break;
	}

	return ret;
}
XR_EXPORT(ioctl);

static void check_line_status(struct uart_port *port, 
							unsigned char *status)
{
	while(*status & (UART_LSR_BI | UART_LSR_PE | 
					UART_LSR_FE | UART_LSR_OE))
	{
		do{
			if(*status & UART_LSR_BI){
				port->icount.brk++;
				break;
			}
		
			if(*status & UART_LSR_PE){
				port->icount.parity++;
				break;
			}

			if(*status & UART_LSR_FE){
				port->icount.frame++;
				break;
			}
		
			if(*status & UART_LSR_OE){
				port->icount.overrun++;
				break;
			}
		}while(0);
		*status = serial_port_in(port, UART_LSR);
	}
}

static int interrupt_select(struct uart_port *port, 
									unsigned int iir)
{
	unsigned long flags;
	unsigned char status;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct exar_priv *priv = port->private_data;

	if(iir & UART_IIR_NO_INT) return 0;

	switch(iir & 0x3F){
		case UART_IIR_RDI: /* Receiver data interrupt */
			spin_lock_irqsave(&port->lock, flags);
			status = serial_port_in(port, UART_LSR);
			if(status & UART_LSR_DR)
				serialxr_rx_chars_RDI(up);
			spin_unlock_irqrestore(&port->lock, flags);
			break;
		case UART_IIR_MSI: /* Modem status interrupt */
			serial8250_modem_status(up);
			break;
		case UART_IIR_RX_TIMEOUT: /* Receiver data time-out */
			spin_lock_irqsave(&port->lock, flags);
			status = serial_port_in(port, UART_LSR);
			if(status & UART_LSR_DR)
				serialxr_rx_chars(up, status);
			spin_unlock_irqrestore(&port->lock, flags);
			priv->charto++;
			break;
		case UART_IIR_RLSI: /* Receiver line status interrupt */
			spin_lock_irqsave(&port->lock, flags);
			status = serial_port_in(port, UART_LSR);
			check_line_status(port, &status);
			spin_unlock_irqrestore(&port->lock, flags);
			break;
		case UART_IIR_THRI: /* Transmitter holding register empty */
			serialxr_tx_chars(up);
			break;
	}
	
	return 1;
}

int serialxr_handle_irq(struct uart_port *port)
{
	unsigned int iir;

	iir = serial_port_in(port, UART_IIR);
	adv_info("Interrupt: 0x%0X\n", iir);

	return interrupt_select(port, iir);
}
XR_EXPORT(handle_irq);

void serialxr_pm(struct uart_port *port, unsigned int state,
					unsigned int old)
{
	unsigned int lcr;

	lcr = serial_port_in(port, UART_LCR);
	/*
	 * To enable sleep mode, EFR[4]=1, LCR[7]=0
	*/
	if(state){
		/* sleep mode IER[4]=1 */
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_EFR, UART_EFR_ECB);
		serial_port_out(port, UART_LCR, 0);
		serial_port_out(port, UART_IER, UART_IERX_SLEEP);
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_EFR, 0);
		serial_port_out(port, UART_LCR, lcr);
	}else{
		/* wake up mode IER[4]=0 */
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_EFR, UART_EFR_ECB);
		serial_port_out(port, UART_LCR, 0);
		serial_port_out(port, UART_IER, 0);
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_EFR, 0);
		serial_port_out(port, UART_LCR, lcr);
	}	
}
XR_EXPORT(pm);

void serialxr_proc_show(struct seq_file *m, struct uart_port *port)
{
	char *type;
	char stat_buf[32];
	unsigned int status;
	struct exar_priv *priv = port->private_data;

	if(priv->type == PORT_RS232) type = "232";
	else if(priv->type == PORT_RS422) type = "422";
	else type = "485";

	seq_printf(m, " RS%s, baud %d, ", type, priv->baud);
	seq_printf(m, " TTX:%d, TX:%lu, TRX:%d, RX:%lu",
							port->icount.tx, priv->tx,
							port->icount.rx, priv->rx);
	if(port->icount.frame)
		seq_printf(m, ", fe:%d ", port->icount.frame);
	if(port->icount.parity)
		seq_printf(m, ", pe:%d ", port->icount.parity);
	if(port->icount.brk)
		seq_printf(m, ", brk:%d ", port->icount.brk);
	if(port->icount.overrun)
		seq_printf(m, ", oe:%d ", port->icount.overrun);

	status = port->ops->get_mctrl(port);
#define INFOBIT(bit, str) \
	if (port->mctrl & (bit)) \
	strncat(stat_buf, (str), sizeof(stat_buf) - \
				strlen(stat_buf) - 2)
#define STATBIT(bit, str) \
	if (status & (bit)) \
		strncat(stat_buf, (str), sizeof(stat_buf) - \
		       strlen(stat_buf) - 2)
	stat_buf[0] = '\0';
	stat_buf[1] = '\0';
	INFOBIT(TIOCM_RTS, "|RTS");
	STATBIT(TIOCM_CTS, "|CTS");
	INFOBIT(TIOCM_DTR, "|DTR");
	STATBIT(TIOCM_DSR, "|DSR");
	STATBIT(TIOCM_CAR, "|CD");
	STATBIT(TIOCM_RNG, "|RI");
	if (stat_buf[0])
		stat_buf[0] = ' ';
	seq_puts(m, stat_buf);
	
	seq_putc(m, '\n');	
#undef INFOBIT
#undef STATBIT
}
XR_EXPORT(proc_show);
