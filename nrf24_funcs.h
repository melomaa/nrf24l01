/*
 * nrf24_funcs.h
 *
 *  Created on: Feb 28, 2015
 *      Author: build
 */

#ifndef NRF24_FUNCS_H_
#define NRF24_FUNCS_H_

#include "nrf24_base.h"

uint8_t flush_rx(struct nrf24_chip *ts);

uint8_t flush_tx(struct nrf24_chip *ts);

void powerUp(struct nrf24_chip *ts);

void powerDown(struct nrf24_chip *ts);

void toggle_features(struct nrf24_chip *ts);

void enableAckPayload(struct nrf24_chip *ts);

void enableDynamicPayloads(struct nrf24_chip *ts);

void setAutoAckPipe(struct nrf24_chip *ts, uint8_t pipe, bool enable );

void setChannel(struct nrf24_chip *ts, uint8_t channel);

void setCRCLength(struct nrf24_chip *ts, rf24_crclength_e length);

bool setDataRate(struct nrf24_chip *ts, rf24_datarate_e speed);

void setPALevel(struct nrf24_chip *ts, rf24_pa_dbm_e level);

void setPayloadSize(struct nrf24_chip *ts, uint8_t size);

void setRetries(struct nrf24_chip *ts, uint8_t delay, uint8_t count);

void openWritingPipe(struct nrf24_chip *ts, uint64_t value);

void openReadingPipe(struct nrf24_chip *ts, uint8_t child, uint64_t address);

void startListening(struct nrf24_chip *ts);

void stopListening(struct nrf24_chip *ts);

void setAutoAck(struct nrf24_chip *ts, bool enable);

rf24_crclength_e getCRCLength(struct nrf24_chip *ts);

#endif /* NRF24_FUNCS_H_ */
