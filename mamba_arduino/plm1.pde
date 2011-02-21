/* plm.c */
#include <stdint.h>
#include <string.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "plm1.h"
#include "spi.h"
#include "ports.h"

#define IACK_SIZE_BYTE_MODE      5
#define IACK_SIZE_NIBBLE_MODE    IACK_SIZE_BYTE_MODE * 2
#define FIFO_MAX_DEPTH           8

static void send_next_nibble( void );
static uint8_t plm_cfg_string_pgm[] PROGMEM = PLM_CFG_STRING;


/* Structure holding variables for reception. The reception buffer is circular. */
struct plm1_rx_t {
   bool receiving;                         /* true when in the process of receiving a PLM packet. */
   bool packet_received;                   /* Signals whether an END_OF_PACKET has been received. */
   bool msb;                               /* true if next nibble to be received is the MSB. */
   bool packet_missed;                     /* true if a receiver disable had to be sent. */
   uint8_t buffer[PLM_RX_MAX_PACKET_SIZE]; /* Circular buffer for reception. */
   uint8_t buffer_index;
   uint8_t write_next;                     /* Index where the next byte will be written in the buffer. */
   uint8_t read_next;                      /* Index of next byte to read from the buffer. */
   uint8_t pkt_end;                        /* Index of the end of a packet. */
   uint8_t iack[IACK_SIZE_NIBBLE_MODE];    /* Confirmation packet for Acks. */
   uint8_t iack_index;                     /* Index of the IAck */
   uint8_t current_packet_type;            /* Current packet type. */
};

/* Structure holding variables for transmission. */
struct plm1_tx_t {
   bool request;                          /* true when the application has requested a transmission. */
   bool in_progress;                      /* true when transmission has been started. */
   bool eop_sent;                         /* true when the PLM_EOP has been sent. */
   bool finished;                         /* true when a packet has been successfully transmitted. */
   uint8_t *pkt_next_byte;                /* Points to next byte to send in buffer. */
   uint8_t nibbles_remaining;
   uint8_t pkt_length[PLM_TX_FIFO_DEPTH]; /* Packet length in bytes (byte mode) or in nibbles (nibbles mode). */
   uint8_t fifo[PLM_TX_FIFO_DEPTH][PLM_TX_MAX_PACKET_SIZE];            /* Buffer for transmission. */
   uint8_t fifo_index;
   bool txing_iack;
};



struct plm_t {
   bool int_context;    /* Set to true during plm interrupt. */
   uint8_t error;       /* Error status. */
   bool no_packet_type;
   struct plm1_rx_t rx;
   struct plm1_tx_t tx;
};


static struct plm_t plm;



static void
cs_enable( bool enable ) {

#if CSPOL == 1
	/* PLM-1 Chip-select pin active high. */
	if(enable)
	{
		SET_OUTPUT(PLM_CS);
	}
	else
	{
		CLR_OUTPUT(PLM_CS);
	}

#elif CSPOL == 0
	/* PLM-1 Chip-select pin active low. */
	if(enable)
	{
		CLR_OUTPUT(PLM_CS);
	}
	else
	{
		SET_OUTPUT(PLM_CS);
	}
#else
#error "CSPOL is not defined"
#endif

}



void
spi_init(enum spi_speed_t speed)
{
#if PLM_PCKPOL == 0
   /* Clock idle state = high, setup on falling edge, sample on rising edge. */
   SPCR = _BV( MSTR ) | _BV( SPE ) | ( speed & SPI_SPR_MASK ) | _BV(CPHA) | _BV(CPOL);
   SPSR = 0; 
#elif PLM_PCKPOL == 1

   /* SPI Master, Clock idle state = 0, set on rising, sample on falling. */
   SPCR = _BV( MSTR ) | _BV( SPE ) | ( speed & SPI_SPR_MASK ) | _BV(CPHA);
   SPSR = 0; /* Double SPI Speed Bit. */
#else
#error "PLM_PCKPOL must be defined with 0 or 1 as value."
#endif
  

}


uint8_t
spi_transfer_byte( uint8_t data )
{
   SPDR = data;

   /* Wait for transfer to complete. Reading of SPSR and SPDR are
   * crucial to the clearing of the SPIF flag in non-interrupt mode. */
   while( bit_is_clear( SPSR, SPIF ) );

   return SPDR; /* Reading data returned by slave. */
}


/******************************************************************
* Writing one nibble to the PLM, and reading one nibble from
* the PLM.
*
* INPUTS: data: nibble to send to the PLM.
* OUTPUT:       nibble read from PLM.
******************************************************************/
static uint8_t
exchange_nibble( uint8_t data )
{
   uint8_t read;

   _delay_us( 500 ); // very important for 16M crystal


   /* Disabling all interruptions (no effect in interrupt context). This
   * is necessary because exchange_nibble() can be called in non
   * interrupt context, and there is a risk that an interruption can
   * occur during that time, and this can lead to problems if that
   * interruption also uses the SPI port. */
   cli();

   cs_enable( true ); /* Enabling PLM Chip-Select. */
   read = spi_transfer_byte( data );
   cs_enable( false ); /* Disabling PLM Chip-Select. */

   /* Don't re-enable interruptions during plm interrupt. */
   if( !plm.int_context ) {
      sei();
   }

   return read; /* Return read data. */
}

/******************************************************************
* Try to initiate a new (or previously aborted) transmission.
******************************************************************/
static void
plm1_tx_start( void )
{
#if COMM_DIRECTION == MODEM_RXTX

   if( ( plm.tx.request ) && ( plm.tx.in_progress == false ) )
   {
      plm.tx.pkt_next_byte = plm.tx.fifo[plm.tx.fifo_index]; /* Next byte = start of buffer. */

#if LIBPLM_MODE == MODE_BYTE
      plm.tx.nibbles_remaining = plm.tx.pkt_length[plm.tx.fifo_index] * 2;  /* 2 nibbles in each byte. */
#elif LIBPLM_MODE == MODE_NIBBLE
      plm.tx.nibbles_remaining = plm.tx.pkt_length[plm.tx.fifo_index];      /* 1 nibble per byte. */
#endif

      plm.tx.eop_sent = false;

      if( plm.rx.receiving == false )
      {
		   plm.tx.in_progress = true;
		   send_next_nibble();                 /* Send first nibble. */
      }
   }
#endif
}

/******************************************************************
* Reset transmission variables in case of error or when a complete
* packet has been transmitted.
*
* INPUTS: n/a
******************************************************************/
static void
plm1_tx_stop( void )
{
   plm.tx.request = false;
   plm.tx.in_progress = false;
   plm.tx.fifo[plm.tx.fifo_index][0] = 0;       /* Transmission over, erase current packet. */
}

static void
transmission_over( void )
{
   plm1_tx_stop();

   /* Go to the next buffer. Wrap around. */
   if(++plm.tx.fifo_index >= PLM_TX_FIFO_DEPTH)
   {
      plm.tx.fifo_index = 0;
   }

   /* Verify whether the next slot of the fifo has a packet ready to be sent. */
   if( plm.tx.fifo[plm.tx.fifo_index][0] )
   {
      plm1_tx_start();
   }

   /* Transmission over and confirmed. */
   else
   {
      plm1_tx_stop();
      plm.tx.finished = true;
   }
}

void
plm1_abort_tx( void )
{
   plm.tx.request = false;
   transmission_over();
}


/******************************************************************
* Reset reception variables in case of error or when a complete
* packet has been received.
*
* INPUTS: packet_received: true  = EOP code received
*                          false = error handling.
******************************************************************/
static void
plm1_rx_stop( bool packet_received )
{
   plm.rx.receiving = false; /* Reception finished. */
   plm.rx.packet_received = packet_received;

   if( packet_received ) {
      plm.rx.pkt_end = plm.rx.write_next; /* Keep position of the end of packet. */

      /* If a Ack is received, send an IAck. */
      if(plm.rx.current_packet_type == PLM_TYPE_ACK && !plm.no_packet_type)
      {
#if LIBPLM_MODE == MODE_BYTE
         plm.rx.iack[0] &= 0x3F;
#elif LIBPLM_MODE == MODE_NIBBLE
         plm.rx.iack[0] &= 0x03;
#endif
         plm.tx.pkt_next_byte = plm.rx.iack; /* Next byte = start of buffer. */
         plm.tx.nibbles_remaining = IACK_SIZE_BYTE_MODE * 2;  /* 2 nibbles in each byte. */
         plm.tx.eop_sent = false;
         plm.tx.in_progress = true;
         plm.tx.txing_iack = true;
         send_next_nibble();
      }

      /* If a IACK has just been received. */
      else if(plm.rx.current_packet_type == PLM_TYPE_IACK)
      {
         transmission_over();
      }
   }

   /* In case of error, clears the circular reception buffer. */
   else {
      plm.rx.write_next = plm.rx.read_next;
   }

   /* Preparing for next packet. */
   plm.rx.msb = true;
}


static void
plm1_set_error( uint8_t error_code )
{
   plm.error = error_code;
   plm.tx.in_progress = false;
   plm1_rx_stop( false );  /* Stop reception and clear receive buffer in case of errors. */
}


/******************************************************************
* Sends the next nibble in the transmit buffer.
******************************************************************/
static void
send_next_nibble( void ) {
uint8_t data;
  
   /* If there are nibbles remaining, send the next nibble. */
   if( plm.tx.nibbles_remaining != 0 )
   {
      /* Load next data byte/nibble to send. */
      data = *plm.tx.pkt_next_byte;

#if LIBPLM_MODE == MODE_BYTE

      /* Each byte in the transmit buffer represents two data nibbles. */
      if( ( plm.tx.nibbles_remaining % 2 ) == 0 ) {
         /* An even remaining nibble means we must transmit
          * the upper nibble (MSB). Put the upper nibble into the lower
          * nibble position of data. */
         data = data >> 4;
      }
      else {
         /* Point to next byte in transmit buffer. */
         plm.tx.pkt_next_byte++;
      }

      data &= 0x0F;
#elif LIBPLM_MODE == MODE_NIBBLE

      /* Point to next byte in transmit buffer. */
      plm.tx.pkt_next_byte++;

#endif

      plm.tx.nibbles_remaining--;

      /* Make sure data is from 0x0 to 0x10 (EOF) .*/
      if( data < 0x11 )
      {
         exchange_nibble( data );   /* Sends nibble. */
      }

      else if(data == 0x11)
      {
         goto SEND_EOP;
      }

      /* An important error occurred if we were to transmit a control code. */
      else 
      {
         plm1_set_error( PLM_ERROR_TX_FAILED );
         plm.tx.request = false;
         plm.tx.in_progress = false;   
      }
   }
   else
   {
      /* When there are no bytes remaining, we must send an END_OF_PACKET.
       * We must then wait until the next TX_REGISTER_EMPTY to reset the
       * flag plm.tx.tx_request. */
      if( plm.tx.eop_sent == false )
      {
SEND_EOP:
         /* The END_OF_PACKET has not been transmitted. Do it! */
         exchange_nibble(PLM_EOP);
         plm.tx.eop_sent = true;
      }

      /* The END_OF_PACKET has been transmitted. Transmission is finished. */
      else 
      {
         if(plm.tx.txing_iack)
         {
            plm.tx.txing_iack = false;
            plm.tx.in_progress = false;

            /* Something was ready to be sent before receiving the ACK and sending the IACK. */
            if( plm.tx.fifo[plm.tx.fifo_index][0] )
            {
               plm1_tx_start();
            }
         }
         else
         {
            if(plm.tx.fifo[plm.tx.fifo_index][0] >> 6 != 0x03)
            {
               transmission_over();
            }
            else
            {
               /* If an ACK was just transmitted. We either wait for NO_RESPONSE or IACK. */
            }
         }
      }
   }
}




static void
plm1_tx_error_handling( uint8_t error_code )
{
   /* An error occurred; the device is no longer transmitting. */
   plm.tx.in_progress = false;
   plm.tx.txing_iack = false;

   /* For a receiver-only node, this code is invalid. */
#if COMM_DIRECTION == MODEM_RX
   plm1_set_error( PLM_ERROR_MODEM_RX );
#else
   plm1_set_error( error_code );
#endif

}


bool
plm1_packet_missed( void ) {
   bool packet_missed = plm.rx.packet_missed;
   plm.rx.packet_missed = false;
   return packet_missed;
}



/******************************************************************
* Check protocol in header.
*
* INPUTS: protocol: protocol to match
*         data:     pointer to start of transmit buffer.
* OUTPUT: true:     if successful
*         false:    if empty packet or if another transmission was
*                   already requested.
******************************************************************/
bool
plm1_check_header( uint8_t protocol, uint8_t *data )
{
   struct plm_pkt_t *pkt = (struct plm_pkt_t *) data;
   return pkt->protocol == protocol;
}



bool
plm1_send_data(uint8_t *data, uint8_t length) {
   return plm1_send_packet(PLM_TYPE_UNACK, PLM_PRIORITY_NORMAL, TX_PROTOCOL, 0, data, length, false);
}



bool
plm1_send_packet(int type, int priority, 
                   int protocol, int8_t repeat_limit,
                   uint8_t *data, int8_t data_length, bool raw_mode)
{
   uint8_t slot_to_fill = plm.tx.fifo_index;
   struct plm_pkt_t *pkt;

   
   /* Loop depth + 1 times so that if everything's full, slot_to_fill comes back to its initial value. */
   for(int i = 0; i <= PLM_TX_FIFO_DEPTH; ++i)
   {
      /* If the slot is empty, fill it. */
      if( !plm.tx.fifo[slot_to_fill][0] )
      {
         break;
      }

      /* If the current slot was not available, increment the slot index and wrap around,
       * except if it's the last pass through the loop, which means that all slots are full. */
      else if( i < PLM_TX_FIFO_DEPTH )
      {
         if( ++slot_to_fill >= PLM_TX_FIFO_DEPTH )
         {
            slot_to_fill = 0;
         }
      }
   }

   /* Return false if every slot is full. If slot_to_fill == 0 and this slot is not empty,
    * the full loop has been made and there's no more place in the fifo. */
   if( (slot_to_fill == plm.tx.fifo_index) && (plm.tx.fifo[plm.tx.fifo_index][0] != 0) )
   {
      return false;
   }
   else
   {
      pkt = (struct plm_pkt_t *) plm.tx.fifo[slot_to_fill];
   }

   /* In raw mode, the header is already included in the byte array. */
   if( !raw_mode )
   {
      pkt->type = type;
      pkt->priority = priority;
      pkt->reserved1 = 0;
      pkt->protocol = protocol;
	   pkt->repeat_limit = repeat_limit;

      /* Copy application data. */
      plm.tx.pkt_length[slot_to_fill] = data_length + sizeof(struct plm_pkt_t);
      memcpy( &plm.tx.fifo[slot_to_fill][sizeof(struct plm_pkt_t)], data, data_length );
   }
   else
   {
      plm.tx.pkt_length[slot_to_fill] = data_length;
      memcpy( plm.tx.fifo[slot_to_fill], data, data_length );     /* Copy packet data. */
   }

   if(!plm.tx.request)
   {
      plm.tx.request = true;
      plm1_tx_start();
   }
   return true;
}



/******************************************************************
* Check if a packet has been successfully transmitted.
*
* OUTPUT: true:  the application has transmitted a complete packet.
*                this automatically clears the packet_transmitted flag.
*         false: the application has NOT sent a complete packet.
******************************************************************/
bool
plm1_tx_finished( void )
{
   bool return_value = plm.tx.finished;

   if(return_value)
   {
      plm.tx.finished = false;      /* Clear flag. */
   }

   return return_value;
}


/******************************************************************
* Returns status of transmission.
*
* OUTPUT: true:  transmitting.
*         false: not transmitting.
******************************************************************/
bool
plm1_tx_requested( void )
{
   return plm.tx.request;
}


/******************************************************************
* Returns status of reception.
*
* OUTPUT: true:  receiving.
*         false: not receiving.
******************************************************************/
bool
plm1_receiving( void ) {
   return plm.rx.receiving;
}


/******************************************************************
* Cancel reception of a packet at the PLM level.
******************************************************************/
void
plm1_disable_receiver( void )
{
   exchange_nibble( PLM_DISABLE_RECV );
   plm1_rx_stop( false );
   plm1_tx_start(); /* Try to start a possible pending transmission. */
}


/******************************************************************
* Gets the current PLM-1 error. The application must poll
* with this function in order know when an error occurred and reset
* the part of packet it fetched from the plm buffer. The returned 
* variable is 0 if no error occurred.
*
* OUTPUT: If error: current error code (0x0 to 0xF).
*         if no error: 0
******************************************************************/
uint8_t
plm1_get_error( void )
{
   return plm.error;
}


/******************************************************************
* Resets the reception error flag. The receive buffer isn't
* cleared because some bytes of a new packet may have been received
* but not given to the application layer, to prevent it from appending
* them to the invalid packet it just received before.
******************************************************************/
void
plm1_reset_error( void )
{
   plm.error = PLM_ERROR_NONE;
}


/******************************************************************
* Check if a complete packet has been read by the aplication.
*
* OUTPUT: true:  the application has read a complete packet.
*                this automatically clears the pkt_received flag.
*         false: the application has NOT read a complete packet.
******************************************************************/
bool
plm1_rx_packet_complete( void )
{
   if( plm.rx.packet_received && (plm.rx.read_next == plm.rx.pkt_end) ) {
      plm.rx.packet_received = false;
      return true;
   }
   else {
      return false;
   }
}


/******************************************************************
* Checks if a byte in the rx buffer is available for reading.
*
* OUTPUT: true:  at least a byte is available in the rx buffer.
*         false: the rx buffer is empty.
******************************************************************/
bool
plm1_rx_data_available( void )
{
   /* There has been a reception error. */
   if( plm.error ) {
      return false;
   }

   /* The library has received an END_OF_PACKET, so the application
    * is only allowed to fetch the bytes *before* the END_OF_PACKET. */
   else if( plm.rx.packet_received && (plm.rx.read_next == plm.rx.pkt_end) ) {
      return false;
   }

   /* Data available in RX buffer. */
   else if( plm.rx.write_next != plm.rx.read_next ) {
      return true;
   }

   /* RX buffer is empty. */
   else {
      return false;
   }
}


/******************************************************************
* Copy a byte from the rx buffer to the application buffer.
*
* Before calling this function, make sure that data is available
* for reading in the rx buffer by calling
* plm1_is_rx_buf_data_available().
******************************************************************/
void
plm1_rx_get_data( uint8_t *buffer )
{
   /* Pop the next byte/nibble. */
   *buffer = plm.rx.buffer[plm.rx.read_next++];

   /* Check if we are past the end of the circular buffer. */
   if( plm.rx.read_next >= PLM_RX_MAX_PACKET_SIZE ) {
      plm.rx.read_next = 0;
   }
}


/******************************************************************
* Store a nibble received from the PLM into the rx buffer.
******************************************************************/
static void
plm1_rx_store_nibble( uint8_t nibble )
{
#if LIBPLM_MODE == MODE_BYTE

   /* Most significant nibble case. Store it in the buffer, shifting of 4 bits. */
   if( plm.rx.msb ) {
      plm.rx.buffer[plm.rx.write_next] = nibble << 4;
   }
   /* Least significant nibble case. Merge with the most significant nibble and store in buffer. */
   else {
      plm.rx.buffer[plm.rx.write_next] |= nibble;
      /* Fill up the iack packet. */
      if(plm.rx.iack_index < IACK_SIZE_BYTE_MODE) {
         plm.rx.iack[plm.rx.iack_index++] = plm.rx.buffer[plm.rx.write_next];
      }
      plm.rx.write_next++;
   }
   plm.rx.msb = !plm.rx.msb;

#elif LIBPLM_MODE == MODE_NIBBLE

   plm.rx.buffer[plm.rx.write_next++] = nibble;

#endif

   /* Circular buffer management. */
   if( plm.rx.write_next >= PLM_RX_MAX_PACKET_SIZE ) {
      plm.rx.write_next = 0;
   }
}


/******************************************************************
* Returns the state of the plm's circular FIFO buffer.
*
* OUTPUT: true:  the FIFO is full.
*         false: the FIFO is not full.
******************************************************************/
static bool
plm1_rx_is_buffer_full( void )
{
   uint8_t read_next;

   /* Special case when read_next is at index 0. */
   if( plm.rx.read_next == 0 ) {
      read_next = PLM_RX_MAX_PACKET_SIZE;
   }

   /* Normal case. */
   else {
      read_next = plm.rx.read_next;
   }

   /* The circular buffer is considered full when the index of
    * the last element is only one element before the first. */
   return ( plm.rx.write_next == (read_next - 1) );
}

#define LED C,4

/******************************************************************
* Gets nibbles from the plm and stores them in the receive buffer.
* Must be called in an interrupt subroutine.
******************************************************************/
void
plm1_interrupt( void )
{
   plm.int_context = true;

   if( plm1_rx_is_buffer_full() )
   {
      plm1_disable_receiver();
      plm.rx.packet_missed = true;
   }
   else {
      uint8_t data_received;

      data_received = exchange_nibble( PLM_NOP );

      /* Test if a special character is received. */
      if( data_received & 0x10 ) 
      {
         /* PLM special characters management. */
         switch( data_received )
         {
            case PLM_EOF:
               /* Byte mode does not support end of field. */
#if LIBPLM_MODE == MODE_BYTE
               plm1_disable_receiver();
               plm1_set_error( PLM_ERROR_EOF_IN_BYTE_MODE );
#endif
               break;

            case PLM_EOP:
               plm1_rx_stop( true );
               break;

            case PLM_PKT_ERROR:
               plm1_set_error( PLM_ERROR_RECEIVED );
               break;

            case PLM_RX_OVERRUN:
               plm1_set_error( PLM_ERROR_RECEIVER_OVERRUN );
               break;

            case PLM_COLLISION:
               plm1_tx_error_handling( PLM_ERROR_COLLISION );
               break;

            case PLM_NO_RESPONSE:
               plm.tx.in_progress = false;
#if LIBPLM_MODE == MODE_BYTE
               plm.tx.fifo[plm.tx.fifo_index][0] &= 0xBF;
#elif LIBPLM_MODE == MODE_NIBBLE
               plm.tx.fifo[plm.tx.fifo_index][0] &= 0xB;
#endif
               break;

            case PLM_PKT_TOO_LONG:
               plm1_set_error( PLM_ERROR_PACKET_TOO_LONG );
               break;

            case PLM_TX_UNDERRUN:
               plm1_tx_error_handling( PLM_ERROR_TX_UNDERRUN );
               break;

            case PLM_TX_OVERRUN:
               plm1_tx_error_handling( PLM_ERROR_TX_OVERRUN );
               break;

            /* The PLM_XRE is received between each transmitted nibble. The first PLM_XRE
             * received in a transmission indicated that the modem has gained access
             * to the communication channel. */
            case PLM_XRE:
#if COMM_DIRECTION == MODEM_RXTX
               /* Send the next nibble of the packet. */
               _delay_us(25);
               send_next_nibble();
#else
               plm1_set_error( PLM_ERROR_MODEM_RX ); /* Raise error flag and clear rx buffer. */
#endif
               break;

            case PLM_NOP:
               if(plm.tx.in_progress) {
                  plm1_tx_error_handling( PLM_ERROR_NOP );
               }
               break;

            default:
               /* Any other unsupported special code: clear the receive buffer. */
               plm1_set_error( PLM_ERROR_UNKNOWN_CODE ); /* Raise error flag and clear rx buffer. */
               break;
         }

         /* In case of error during reception, or after a complete packet has
         * been received, always try to start a pending transmission. */
         plm1_tx_start();
      }

      /* Otherwise, a data nibble is being received. */
      else 
      {
         /* First nibble fo a packet. By definition, the first nibble cannot
          * be an EOF or an EOP, so we always catch it here. */
         if( !plm.rx.receiving )
         {
            plm.rx.receiving = true; /* Now we can't initiate a transmission. */
            plm.tx.in_progress = false;
            plm.rx.current_packet_type = data_received >> 2;
            plm.rx.iack_index = 0;
         }

         /* Only data nibbles are stored in byte mode. */
#if LIBPLM_MODE == MODE_BYTE
         plm1_rx_store_nibble( data_received );
#endif
      }

      /* In nibble mode, we store everything (EOF and EOP) in the buffer when receiving. */
#if LIBPLM_MODE == MODE_NIBBLE
      if( plm.rx.receiving )
      {
         plm1_rx_store_nibble( data_received );
      }
#endif
   }
   plm.int_context = false;
}


uint8_t
plm1_receive( uint8_t *data_packet ) {
   uint8_t length;

   if( plm1_rx_data_available() != false )
   {
      /* Make sure buffer is not full. */
      if( plm.rx.buffer_index < sizeof(plm.rx.buffer) )
      {
         plm1_rx_get_data( &plm.rx.buffer[plm.rx.buffer_index++] );
      }
      else {
         plm1_disable_receiver();
         plm.rx.buffer_index = 0;
      }
   }

   /* When an error is received, reset the current packet
      * and reset the error flag of the PLM. */
   if( plm1_get_error() ) 
   {
      plm.rx.buffer_index = 0;
      plm1_reset_error();
   }

   if( plm1_rx_packet_complete() ) {
      length = plm.rx.buffer_index - 2;
      memcpy( data_packet, plm.rx.buffer + 2, length );
      plm.rx.buffer_index = 0;
   }
   else {
      length = 0;
   }
   return length;
}


static void
plm_configure_pgm( const uint8_t *data )
{
   uint8_t value;
   int8_t i = PLM_CONFIG_DATA_LENGTH;

   plm.int_context = true; /* To prevent the sei() instruction from being called
                           * during initialization (interruptions are disabled
                           * during initialization). */

   /* Sends a Soft reset command to the PLM. */
   exchange_nibble( PLM_RESET );

#if LIBPLM_MODE == MODE_BYTE

   /* Verify if IAcks need to be sent. */
   value = pgm_read_byte(&data[9]);
   plm.no_packet_type = (value & 0x20) != 0;

   do 
   {
      value = pgm_read_byte(data++);
      exchange_nibble( value >> 4 );
      exchange_nibble( value & 0x0F );
   } while( --i > 0 );

#elif LIBPLM_MODE == MODE_NIBBLE

   i *= 2;
   do 
   {
      value = pgm_read_byte(data++);
      exchange_nibble(value);
   } while( --i > 0 );

#endif

   /* Don't enable interrupts. They will be enabled at the beginning of the main loop. */
   plm.int_context = false;

   /* Reset tx/rx variables. */
   plm1_tx_stop();
   plm1_rx_stop(false);

}


/******************************************************************
* Sends configuration string to the PLM.
******************************************************************/
void
plm1_configure( uint8_t *nibbles ) {
   int8_t i = PLM_CONFIG_DATA_LENGTH * 2;

   plm.int_context = true;

   plm.no_packet_type = (nibbles[18] & 2) != 0;

   exchange_nibble( PLM_RESET );    /* Sends a Soft reset command to the PLM. */
   _delay_us( 1000 ); // very important for 16M crystal

   do {
      exchange_nibble( *nibbles++ );
   } while( --i > 0 );

   plm.int_context = false;

   /* Reset tx/rx variables. */
   plm1_tx_stop();
   plm1_rx_stop(false);

   sei();
}



/******************************************************************
* plm1 initialization. Must be called before using any plm1
* functions.
******************************************************************/
void
plm1_init( void )
{
   /* Initial static variables values (all other values are zero). */
   memset( &plm, 0x00, sizeof(struct plm_pkt_t) );
   plm.rx.msb = true;
   plm.tx.request = 0;

   PIN_OUTPUT(PLM_CS);				   /* Make sure CS pin is an output */

   CLR_OUTPUT(nPLM_RESET);  /* Toggle reset line of PLM-1. */
   _delay_us( 1000 ); // very important for 16M crystal
   SET_OUTPUT(nPLM_RESET);

   plm_configure_pgm( plm_cfg_string_pgm );

}
