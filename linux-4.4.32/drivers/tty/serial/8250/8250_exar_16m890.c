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
 * exar 16m890 ops will override the default 8250 ops
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

static const struct serial8250_config xr_config [] = {
	[PORT_XR16M890] = {
		.name		= "xr16m890",
		.fifo_size	= 128,
		.tx_loadsz	= 128,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.rxtrig_bytes	= {1, 4, 8, 14},
		.flags		= UART_CAP_FIFO, /* uart has fifo */
	},
};

unsigned int inline serialxr_in(struct uart_port *p, int offset)
{
	return readb(p->membase+offset);
}
EXPORT_SYMBOL_GPL(serialxr_in);

void inline serialxr_out(struct uart_port *p, int offset, int val)
{
	return writeb(val, p->membase+offset);
}
EXPORT_SYMBOL_GPL(serialxr_out);

void serialxr_set_mctrl(struct uart_port *port, 
							unsigned int mctrl)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;

	serial_port_out(port, UART_MCR, mcr);
}
EXPORT_SYMBOL_GPL(serialxr_set_mctrl);

/*
 * The exar 16m890 specific startup function
*/
int serialxr_startup(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	unsigned long flags;
	unsigned char lsr, iir;
	int retval;

	pr_info("==== %s ====\n", __func__);

	if (!port->fifosize)
		port->fifosize = xr_config[port->type].fifo_size;
	if (!up->tx_loadsz)
		up->tx_loadsz = xr_config[port->type].tx_loadsz;
	if (!up->capabilities)
		up->capabilities = xr_config[port->type].flags;
	up->mcr = 0;

	/* TODO: need to check */
	if (port->iotype != up->cur_iotype){
		set_io_from_upio(port);
	}
	if(port->handle_irq)
		pr_info("have irq!!!!\n");

	serial8250_rpm_get(up);
	/* 
	 * TODO: set RTL/TTL 
	 * I need to get the TTL & RTL by myself,
	 * and set into UART_EXAR_TXTRG/RXTRG
	*/
	//serial_port_out(port, UART_EXAR_TXTRG, TTL);
	//serial_port_out(port, UART_EXAR_RXTRG, RTL);

	/*
	 * Wakeup and initilaize UART
	 *
	 * First enable access to IER [7:5], ISR [5:4], FCR [5:4],
	 * MCR [7:5] and MSR [7:0]
	 */
	serial_port_out(port, UART_XR_EFR, UART_EFR_ECB);
	/*
	 * Make sure all interrups are masked until initialization is
	 * complete and the FIFOs are cleared
	 */
	serial_port_out(port, UART_IER, 0);
	serial_port_out(port, UART_LCR, 0);
	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
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

	/*
	 * FIXME: What's going on?
	*/
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
	/*
	 * Now, initialize the UART
	 */
	serial_port_out(port, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&port->lock, flags);
	/*
	 * Most PC uarts need OUT2 raised to enable interrupts.
	 */
	if (port->irq)
		up->port.mctrl |= TIOCM_OUT2;

	serialxr_set_mctrl(port, port->mctrl);
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
	serial_port_out(port, UART_IER, up->ier);	
	retval = 0;
out:
	serial8250_rpm_put(up);
	return retval;
}
EXPORT_SYMBOL_GPL(serialxr_startup);

void serialxr_set_termios(struct uart_port *port, 
							struct ktermios *termios,
		          			struct ktermios *old)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	unsigned char cval;
	unsigned long flags;
	unsigned int baud, quot, frac = 0;

	pr_info("=== %s ===\n", __func__);

	cval = serial8250_compute_lcr(up, termios->c_cflag);

	baud = serial8250_get_baud_rate(port, termios, old);
	quot = serial8250_get_divisor(up, baud, &frac);

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	serial8250_rpm_get(up);
	spin_lock_irqsave(&port->lock, flags);

	up->lcr = cval;					/* Save computed LCR */

	if (up->capabilities & UART_CAP_FIFO && port->fifosize > 1) {
		/* NOTE: If fifo_bug is not set, a user can set RX_trigger. */
		if ((baud < 2400 && !up->dma) || up->fifo_bug) {
			up->fcr &= ~UART_FCR_TRIGGER_MASK;
			up->fcr |= UART_FCR_TRIGGER_1;
		}
	}

	/*
	 * MCR-based auto flow control.  When AFE is enabled, RTS will be
	 * deasserted when the receive FIFO contains more characters than
	 * the trigger, or the MCR RTS bit is cleared.  In the case where
	 * the remote UART is not using CTS auto flow control, we must
	 * have sufficient FIFO entries for the latency of the remote
	 * UART to respond.  IOW, at least 32 bytes of FIFO.
	 */
	if (up->capabilities & UART_CAP_AFE && port->fifosize >= 32) {
		up->mcr &= ~UART_MCR_AFE;
		if (termios->c_cflag & CRTSCTS)
			up->mcr |= UART_MCR_AFE;
	}

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
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

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (!(up->bugs & UART_BUG_NOMSR) &&
			UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
	if (up->capabilities & UART_CAP_UUE)
		up->ier |= UART_IER_UUE;
	if (up->capabilities & UART_CAP_RTOIE)
		up->ier |= UART_IER_RTOIE;

	serial_port_out(port, UART_IER, up->ier);

	if (up->capabilities & UART_CAP_EFR) {
		unsigned char efr = 0;
		/*
		 * TI16C752/Startech hardware flow control.  FIXME:
		 * - TI16C752 requires control thresholds to be set.
		 * - UART_MCR_RTS is ineffective if auto-RTS mode is enabled.
		 */
		if (termios->c_cflag & CRTSCTS)
			efr |= UART_EFR_CTS;

		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		if (port->flags & UPF_EXAR_EFR)
			serial_port_out(port, UART_XR_EFR, efr);
		else
			serial_port_out(port, UART_EFR, efr);
	}

	serial8250_set_divisor(port, baud, quot, frac);

	/*
	 * LCR DLAB must be set to enable 64-byte FIFO mode. If the FCR
	 * is written without DLAB set, this mode will be disabled.
	 */
	if (port->type == PORT_16750)
		serial_port_out(port, UART_FCR, up->fcr);

	serial_port_out(port, UART_LCR, up->lcr);	/* reset DLAB */
	if (port->type != PORT_16750) {
		/* emulated UARTs (Lucent Venus 167x) need two steps */
		if (up->fcr & UART_FCR_ENABLE_FIFO)
			serial_port_out(port, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_port_out(port, UART_FCR, up->fcr);	/* set fcr */
	}
	serial8250_set_mctrl(port, port->mctrl);
	spin_unlock_irqrestore(&port->lock, flags);
	serial8250_rpm_put(up);

	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
}
EXPORT_SYMBOL(serialxr_set_termios);



