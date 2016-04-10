/*
 * nrf24_tty.h
 *
 *  Created on: Apr 5, 2016
 *      Author: ray
 */

#ifndef NRF24_TTY_H_
#define NRF24_TTY_H_

#define to_nrf24_pipe(port) \
		container_of(port, struct nrf24_pipe, uart)

int nrf24_open(struct uart_port *port);
void nrf24_start_tx(struct uart_port *port);
void nrf24_stop_rx(struct uart_port *port);
int nrf24dev_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg);
void nrf24_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old);

unsigned int nrf24_get_mctrl(struct uart_port *port);
void nrf24_set_mctrl(struct uart_port *port, unsigned int mctrl);
const char * nrf24_type(struct uart_port *port);
void nrf24_release_port(struct uart_port *port);
int nrf24_request_port(struct uart_port *port);
void nrf24_config_port(struct uart_port *port, int flags);
int nrf24_verify_port(struct uart_port *port, struct serial_struct *ser);
void nrf24_shutdown(struct uart_port *port);
unsigned int nrf24_tx_empty(struct uart_port *port);
void nrf24_enable_ms(struct uart_port *port);
void nrf24_break_ctl(struct uart_port *port, int break_state);
void nrf24_stop_tx(struct uart_port *port);

#endif /* NRF24_TTY_H_ */
