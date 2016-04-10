
#include <linux/delay.h>

#include "nrf24_base.h"
#include "nrf24_funcs.h"
#include "nrf24_tty.h"

static unsigned int mcr = 0;

static uint8_t get_pipe_nr(struct nrf24_chip *ts, struct nrf24_pipe *pipe)
{
	int i = 0;
	for (i = 0; i < NRF24_PIPES; i++) {
		if (ts->pipes[i] == pipe) {
			break;
		}
	}
	return i;
}

int nrf24_open(struct uart_port *port)
{
	struct nrf24_chip *ts;
	struct nrf24_pipe *pipe;
	uint8_t nr = 0;

	printk("%s\n", __func__);

	pipe = to_nrf24_pipe(port);
	if (pipe == NULL) {
		return -ECHILD;
	}
	ts = pipe->chip;
	if (ts == NULL)
		return -ECHILD;
	nr = get_pipe_nr(ts, pipe);
	if (nr >= NRF24_PIPES) {
		return -ECHILD;
	}

	if (ts->open_ports == 0) {

		powerUp(ts);

		ts->ackPayload = false;
		if (ts->workqueue) {
			/* Flush and destroy work queue */
			flush_workqueue(ts->workqueue);
			destroy_workqueue(ts->workqueue);
			ts->workqueue = NULL;
		}
		/* Initialize work queue */
		ts->workqueue = create_freezable_workqueue("nrf24");
		if (!ts->workqueue) {
			dev_err(&ts->spi->dev, "Workqueue creation failed\n");
			return -EBUSY;
		}

		INIT_WORK(&ts->work, nrf24_work_routine);

		ts->pending = 0;

		startListening(ts);
	}
	ts->open_ports |= (1 << nr);

  	return 0;
}

void nrf24_start_tx(struct uart_port *port)
{
	struct nrf24_chip *ts;
	struct nrf24_pipe *pipe;

	pipe = to_nrf24_pipe(port);
	BUG_ON(!pipe);
	ts = pipe->chip;
	BUG_ON(!ts);
	if (pipe != ts->pipes[0]) {
		dev_info(&ts->spi->dev, "No TX\n");
	}
	dev_info(&ts->spi->dev, "%s\n", __func__);

	/* Trigger work thread for sending data */
	nrf24_dowork(ts);
}

void nrf24_stop_rx(struct uart_port *port)
{
	struct nrf24_chip *ts;
	struct nrf24_pipe *pipe;

	pipe = to_nrf24_pipe(port);
	BUG_ON(!pipe);
	ts = pipe->chip;
	BUG_ON(!ts);
	dev_info(&ts->spi->dev, "%s\n", __func__);
	if (pipe != ts->pipes[0]) {
		dev_info(&ts->spi->dev, "Read only device\n");
		return;
	}

	if (assign_command(ts, NRF24_POWERDOWN) == 0) {
		nrf24_dowork(ts);
	}
	else {
		dev_info(&ts->spi->dev, "Poweroff command failed\n");
	}
}

int nrf24dev_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct nrf24_chip *ts;
	struct spi_device	*spi;
	int ret = -ENOSYS;

	struct nrf24_pipe *pipe = to_nrf24_pipe(port);
	if (pipe == NULL) {
		return -ECHILD;
	}
	ts = pipe->chip;
	if (ts == NULL) {
		return -ECHILD;
	}

	dev_info(&ts->spi->dev," Command 0x%x, pointer %x\n",cmd,(unsigned int)arg);

	spin_lock_irq(&ts->lock);
	spi = spi_dev_get(ts->spi);
	spin_unlock_irq(&ts->lock);

	switch (cmd)
	{
	case TIO_NRF24_ACKPAYLOAD:
		memcpy(&ts->nioctl, (void*)arg, sizeof(struct nrf24_ioctl));
		dev_dbg(&ts->spi->dev,"AP(%d) len: %d\n",ts->nioctl.pipe,ts->nioctl.apLen);
		ts->ackPayload = true;
		nrf24_dowork(ts);
		ret = 0;
		break;
	case TIO_NRF24_PAYLOADSIZE:
		ts->payload_size = *(char*)arg;
		dev_dbg(&ts->spi->dev,"Set payload size %d\n", ts->payload_size);
		ret = 0;
		break;
	case TIO_NRF24_GETCONFIG:
		if (getConfiguration(ts) == 0) {
			memcpy((void*)arg,&ts->radioConfig,sizeof(struct nrf24_config));
			ret = 0;
		}
		else {
			ret = -EBUSY;
		}
		break;
	case TIO_NRF24_SETCHANNEL:
		ts->radioConfig.channel = *(char*)arg;
		if (assign_command(ts, NRF24_SETCHANNEL) == 0) {
			if (!request_comm(ts, 1000)) {
				nrf24_device_control(ts);
				release_comm(ts);
				ret = 0;
			}
			else {
				ret = -EBUSY;
			}
		}
		else {
			ret = -EBUSY;
		}
		break;
#if 0
	case TCGETS:
		ret = 0;
		break;
	case TCSETSW:
		ret = 0;
		break;
#endif
	default:
		ret = -ENOIOCTLCMD;;
		break;
	}

	return ret;
}

void nrf24_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct nrf24_chip *ts;
	unsigned long flags;
	struct nrf24_pipe *pipe;

	printk("%s\n", __func__);

	pipe = to_nrf24_pipe(port);
	BUG_ON(!pipe);
	ts = pipe->chip;
	BUG_ON(!ts);

	spin_lock_irqsave(&pipe->uart.lock, flags);
	/* we are sending char from a workqueue so enable */
#ifdef KERNEL_PRE_3v9
	pipe->uart.state->port.tty->low_latency = 1;
#else
	pipe->uart.state->port.low_latency = 1;
#endif
	/* Update the per-port timeout. */
	uart_update_timeout(port, termios->c_cflag, 57600);
	spin_unlock_irqrestore(&pipe->uart.lock, flags);
	return;
}

unsigned int nrf24_get_mctrl(struct uart_port *port)
{
	printk("%s\n", __func__);
	return mcr;
}

void nrf24_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	printk("%s\n", __func__);
	mcr = mctrl;
}

const char * nrf24_type(struct uart_port *port)
{
	printk("%s\n", __func__);
	return TYPE_NAME;
}

void nrf24_release_port(struct uart_port *port)
{
	printk("%s\n", __func__);
}

int nrf24_request_port(struct uart_port *port)
{
	printk("%s\n", __func__);
	return 0;
}

void nrf24_config_port(struct uart_port *port, int flags)
{
	printk("%s\n", __func__);
	return;
}

int nrf24_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type == PORT_UNKNOWN || ser->type == 98)
		return 0;

	return -EINVAL;
}

void nrf24_shutdown(struct uart_port *port)
{
	struct nrf24_chip *ts;
	struct nrf24_pipe *pipe;
	uint8_t nr = 0;

	printk("%s\n", __func__);

	pipe = to_nrf24_pipe(port);
	BUG_ON(!pipe);
	ts = pipe->chip;
	BUG_ON(!ts);
	nr = get_pipe_nr(ts, pipe);
	if (nr >= NRF24_PIPES) {
		return;
	}
	ts->open_ports &= ~(1 << nr);
	if (ts->open_ports == 0) {
		if (request_comm(ts, 2000)) {
			dev_err(&ts->spi->dev,"Access timeout at shutdown, forcing close\n");
		}
		if (ts->workqueue) {
			/* Flush and destroy work queue */
			flush_workqueue(ts->workqueue);
			destroy_workqueue(ts->workqueue);
			ts->workqueue = NULL;
		}
		powerDown(ts);
		printk("Workqueue removed\n");
	}
	return;
}

unsigned int nrf24_tx_empty(struct uart_port *port)
{
	printk("%s\n", __func__);
	return TIOCSER_TEMT;
}

void nrf24_enable_ms(struct uart_port *port)
{
	/* Do nothing */
	printk("%s\n", __func__);
}

void nrf24_break_ctl(struct uart_port *port, int break_state)
{
	/* Do nothing */
	printk("%s\n", __func__);
}

void nrf24_stop_tx(struct uart_port *port)
{
	/* Do nothing */
	printk("%s\n", __func__);
}
