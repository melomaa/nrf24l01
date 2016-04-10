
#include "nrf24_funcs.h"

uint8_t flush_rx(struct nrf24_chip *ts)
{
  uint8_t status;

  u8 c = ( FLUSH_RX );
  status = nrf24_write_from_buf(ts, &c, 1);

  return status;
}

uint8_t flush_tx(struct nrf24_chip *ts)
{
  uint8_t status;

  u8 c = ( FLUSH_TX );
  status = nrf24_write_from_buf(ts, &c, 1);

  return status;
}

void powerUp(struct nrf24_chip *ts)
{
  write_register(ts, CONFIG,read_register(ts, CONFIG) | _BV(PWR_UP));
}

void powerDown(struct nrf24_chip *ts)
{
	write_register(ts, CONFIG,read_register(ts, CONFIG) & ~(_BV(PWR_UP)));
}

void toggle_features(struct nrf24_chip *ts)
{
	u8 buf[2];

	buf[0] = ACTIVATE;
	buf[1] = 0x73;
	nrf24_write_from_buf(ts, buf, 2);
	return;
}

void enableAckPayload(struct nrf24_chip *ts)
{
  //
  // enable ack payload and dynamic payload features
  //

  write_register(ts, FEATURE,read_register(ts, FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! read_register(ts, FEATURE) )
  {
    // So enable them and try again
    toggle_features(ts);
    write_register(ts, FEATURE, read_register(ts, FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
  }

  //
  // Enable dynamic payload on pipes 0 & 1
  //

  write_register(ts, DYNPD,read_register(ts, DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}

void enableDynamicPayloads(struct nrf24_chip *ts)
{
  // Enable dynamic payload throughout the system
  write_register(ts, FEATURE,read_register(ts, FEATURE) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! read_register(ts, FEATURE) )
  {
    // So enable them and try again
    toggle_features(ts);
    write_register(ts, FEATURE,read_register(ts, FEATURE) | _BV(EN_DPL) );
  }

  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  write_register(ts, DYNPD,read_register(ts, DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  ts->dynamic_payloads_enabled = true;
}

void setAutoAckPipe(struct nrf24_chip *ts, uint8_t pipe, bool enable )
{
  if ( pipe <= 6 )
  {
    uint8_t en_aa = read_register(ts, EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    write_register(ts, EN_AA, en_aa ) ;
  }
}

void setChannel(struct nrf24_chip *ts, uint8_t channel)
{
  // TODO: This method could take advantage of the 'wide_band' calculation
  // done in setChannel() to require certain channel spacing.

  const uint8_t max_channel = 127;
  write_register(ts, RF_CH,min(channel,max_channel));
}

void setCRCLength(struct nrf24_chip *ts, rf24_crclength_e length)
{
  uint8_t config = read_register(ts, CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above.
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  write_register(ts, CONFIG, config ) ;
}


bool setDataRate(struct nrf24_chip *ts, rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = read_register(ts, RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  ts->wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    ts->wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      ts->wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      ts->wide_band = false ;
    }
  }
  write_register(ts, RF_SETUP,setup);

  // Verify our result
  if ( read_register(ts, RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    ts->wide_band = false;
  }

  return result;
}

void setPALevel(struct nrf24_chip *ts, rf24_pa_dbm_e level)
{
  uint8_t setup = read_register(ts, RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_MAX )
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  write_register(ts, RF_SETUP, setup ) ;
}

void setPayloadSize(struct nrf24_chip *ts, uint8_t size)
{
  ts->payload_size = min(size,(uint8_t)MAX_PAYLOAD_SIZE);
}

void setRetries(struct nrf24_chip *ts, uint8_t delay, uint8_t count)
{
 write_register(ts, SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

void openWritingPipe(struct nrf24_chip *ts, uint64_t value)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.
  write_buffer_to_register(ts, RX_ADDR_P0, (uint8_t*)(&value), 5);
  write_buffer_to_register(ts, TX_ADDR, (uint8_t*)(&value), 5);

  write_register(ts, RX_PW_P0,min(ts->payload_size,(uint8_t)MAX_PAYLOAD_SIZE));
}

static const uint8_t child_pipe[] =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void openReadingPipe(struct nrf24_chip *ts, uint8_t child, uint64_t address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0)
    ts->pipe0_reading_address = address;

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB

	  if ( child < 2 ) {
		  write_buffer_to_register(ts, child_pipe[child], (uint8_t*)(&address), 5);
	  }
	  else {
		  write_register(ts, child_pipe[child], (uint8_t)address);
	  }
	  write_register(ts, child_payload_size[child],ts->payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register(ts, EN_RXADDR,read_register(ts, EN_RXADDR) | _BV(child_pipe_enable[child]));
  }
}

void startListening(struct nrf24_chip *ts)
{
	ce(LOW);
//	msleep(1);
  write_register(ts, CONFIG, read_register(ts, CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
  write_register(ts, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if (ts->pipe0_reading_address)
    write_buffer_to_register(ts, RX_ADDR_P0, (uint8_t*)(&ts->pipe0_reading_address), 5);

  // Flush buffers
  flush_rx(ts);
  flush_tx(ts);

  // Go!
  ce(HIGH);

  // wait for the radio to come up (130us actually only needed)
  //delayMicroseconds(130);
}

void stopListening(struct nrf24_chip *ts)
{
	  ce(LOW);
//		msleep(1);

	write_register(ts, CONFIG, read_register(ts, CONFIG) & ~_BV(PRIM_RX) );
	//write_register(EN_RXADDR,read_register(EN_RXADDR) | _BV(0)); // Enable RX on pipe0
	  ce(HIGH);

}


void setAutoAck(struct nrf24_chip *ts, bool enable)
{
  if ( enable )
    write_register(ts, EN_AA, 0b111111);
  else
    write_register(ts, EN_AA, 0);
}

rf24_crclength_e getCRCLength(struct nrf24_chip *ts)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = read_register(ts, CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

  if ( config & _BV(EN_CRC ) )
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

int getConfiguration(struct nrf24_chip *ts)
{
	uint8_t reg;
	int i;

	read_buffer_from_register(ts,RX_ADDR_P0,ts->radioConfig.rxAddr0,5);
	read_buffer_from_register(ts,RX_ADDR_P1,ts->radioConfig.rxAddr1,5);
	reg = RX_ADDR_P2;
	for (i=0; i<4; i++) {
		ts->radioConfig.rxAddrN[i] = read_register(ts, reg+i);
	}
	read_buffer_from_register(ts,TX_ADDR,ts->radioConfig.txAddr,5);
	reg = RX_PW_P0;
	for (i=0; i<6; i++) {
		ts->radioConfig.rxWidth[i] = read_register(ts, reg+i);
	}
	ts->radioConfig.autoAck = read_register(ts, EN_AA);
	ts->radioConfig.rxEnable = read_register(ts, EN_RXADDR);
	ts->radioConfig.channel = read_register(ts, RF_CH);
	ts->radioConfig.setup = read_register(ts, RF_SETUP);
	ts->radioConfig.config = read_register(ts, CONFIG);
	ts->radioConfig.dynpd = read_register(ts, DYNPD);
	ts->radioConfig.feature = read_register(ts, FEATURE);
	ts->radioConfig.retransmit = read_register(ts, SETUP_RETR);
	ts->radioConfig.addrWidth = read_register(ts, SETUP_AW);

	return 0;
}


