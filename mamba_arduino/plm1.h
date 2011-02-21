#ifndef _LIBPLM_H_
#define _LIBPLM_H_
/************************************************************************************
 * Ariane Controls
 *
 * Libplm for Atmel MCU for gcc.
 *    by Guillaume Boucher
 *
 * History:
 *    V 1.00
 *
 ************************************************************************************/
#include <stdbool.h>
#include "spi.h"
#include "plm1cfg.h"


/************************************************************************************
 * USER PARAMETERS
 * 
 * Parameters to be modified by the user.
 ************************************************************************************/
#define PLM_CFG_STRING                 PLM_144_5930      /* From plmcfg.h */
#define PLM_CS						   B,2               /* CS pin. */
#define nPLM_RESET                     B,0               /* Reset pin. */
#define PLM_INTERRUPT_VECT_NAME        SIG_INTERRUPT0    /* Name of interrupt vector for PLM-1. */
#define PLM_CSPOL                      0		         /* Value of PLM-1 CSPOL pin. */
#define PLM_PCKPOL                     1                 /* Value of PLM-1 PCKPOL pin. */
#define PLM_TX_FIFO_DEPTH			   1                 /* Max nb of packets in the tx fifo. */
#define PLM_TX_MAX_PACKET_SIZE         63                /* Max packet size in transmission. */
#define PLM_RX_MAX_PACKET_SIZE         63                /* Rx Max packet size. */
#define COMM_DIRECTION                 MODEM_RXTX        /* Bidirectional device. */
#define LIBPLM_MODE                    MODE_BYTE         /* Library works with bytes. */
#define TX_PROTOCOL                    4                 /* */
/************************************************************************************
 * END OF USER PARAMETERS
 ************************************************************************************/



#define MODEM_RX                          0
#define MODEM_RXTX                        1

#define MODE_NIBBLE                       0
#define MODE_BYTE                         1

/* Library PLM-1 error codes. */
/* These error codes are used by the library, *not* by the PLM-1. */
#define PLM_ERROR_NONE                    0x0
#define PLM_ERROR_UNKNOWN_CODE            0x1
#define PLM_ERROR_RECEIVED                0x2
#define PLM_ERROR_RECEIVER_OVERRUN        0x3
#define PLM_ERROR_COLLISION               0x4
#define PLM_ERROR_NO_RESPONSE             0x5
#define PLM_ERROR_PACKET_TOO_LONG         0x6
#define PLM_ERROR_TX_UNDERRUN             0x7
#define PLM_ERROR_TX_FAILED               0x8
#define PLM_ERROR_TX_OVERRUN              0x9
#define PLM_ERROR_EOF_IN_BYTE_MODE        0xA
#define PLM_ERROR_MODEM_RX                0xB
#define PLM_ERROR_PACKET_MISSED           0xC
#define PLM_ERROR_NOP                     0xF

/* Control codes of the the PLM-1. */
#define PLM_EOF                          0x10 /* End of Field */
#define PLM_EOP                          0x11 /* End of Packet */
#define PLM_PKT_ERROR                    0x12
#define PLM_RX_OVERRUN                   0x13
#define PLM_COLLISION                    0x14
#define PLM_NO_RESPONSE                  0x15
#define PLM_PKT_TOO_LONG                 0x16
#define PLM_TX_UNDERRUN                  0x17
#define PLM_XRE                          0x18 /* Tx Register Empty. */
#define PLM_TX_OVERRUN                   0x19
#define PLM_TIMER_ELAPSED                0x1A
#define PLM_TIMER_OVERRUN                0x1B
#define PLM_NOP                          0x1F /* No Operation. */

/* Controls codes to send to the PLM. */
#define PLM_DISABLE_RECV  0x12                /* Disable receiver. */
#define PLM_RESET         0x16                /* Reset. */


#define PLM_CONFIG_DATA_LENGTH 19 /* bytes. */

/* Packet types */
enum plm_type_t {
   PLM_TYPE_IACK,
   PLM_TYPE_FAIL,
   PLM_TYPE_UNACK,
   PLM_TYPE_ACK
};

/* Packet priorities */
enum plm_priority_t {
   PLM_PRIORITY_HIGHEST,
   PLM_PRIORITY_HIGH,
   PLM_PRIORITY_NORMAL,
   PLM_PRIORITY_DEFERRED
};

/* Protocol IDs */
enum plm_protocol_t {
   PLM_PROTOCOL_ALL = 0x00,         /* 0 */
   PLM_PROTOCOL_ACP2,               /* 1 */
   PLM_PROTOCOL_LOGISIG,            /* 2 */
   PLM_PROTOCOL_RAW,                /* 3 */
   PLM_PROTOCOL_RANDOM,             /* 4 */
   PLM_PROTOCOL_ACP3,               /* 5 */
   PLM_PROTOCOL_CRC,                /* 6 */
   PLM_PROTOCOL_DOO1,               /* 7 */
   PLM_PROTOCOL_DOO2                /* 8 */
};


/* Standard PLM-1 header. */
struct plm_pkt_t {
  /* Byte 1 */
  uint8_t repeat_limit: 2;
  uint8_t reserved1: 2;
  uint8_t priority : 2;
  uint8_t type : 2;
  
  /* Byte 2 */
  uint8_t protocol;
};

/******************************************************************
* spi_init: Initializes the SPI settings of the MCY library according 
* to the PLM_PCKPOL value.
*
* INPUTS: None
* OUTPUT: None
******************************************************************/
void
spi_init(enum spi_speed_t speed);


/******************************************************************
* plm1_init: Initializes the PLM-1 library according to the USER 
* parameters defined on top of this file.
*
* INPUTS: None
* OUTPUT: None
******************************************************************/
void
plm1_init( void );


/******************************************************************
 * plm1_send_packet: send a packet on the powerline
 *
 * INPUTS: data = data to send: When the PLM-1 library has been initialized
 *              in byte mode (LIBPLM_MODE_BYTE), the data in the array
 *              is expected to be in bytes whereas when it is initialized
 *              in nibble mode, the data is expected to be in nibbles.
 *              Ex. byte mode: 0xA3;  nibble mode: 0x0A, 0x03
 *         length = the the length of the data array.
 *
 * OUTPUT: 
 *         true if the packet has been queued successfully
 ******************************************************************/
bool
plm1_send_data(uint8_t *data, uint8_t length);


/******************************************************************
 * plm1_send_packet: send a packet on the powerline
 *
 * INPUTS: plm_type = packet type (default PLM_TYPE_UNACK)
 *         plm_priority = packet priority (default PLM_PRIORITY_NORMAL)
 *         plm_protocol = protocol id field
 *         repeat_limit = repeat limit field. If -1, the value taken
 *              is the one specified by the function plm1_tx_set_repeat_limit.
 *         data = data to send: the first 2 bytes of the packet are determined
 *              by the first 4 arguments. The following bytes come from
 *              the data array. When the PLM-1 library has been initialized
 *              in byte mode (LIBPLM_MODE_BYTE), the data in the array
 *              is expected to be in bytes whereas when it is initialized
 *              in nibble mode, the data is expected to be in nibbles.
 *              Ex. byte mode: 0xA3;  nibble mode: 0x0A, 0x03
 *         length = the the length of the data array.
 *         raw_mode = when set to true, the first 4 arguments not considered
 *              and the data array contains all the packet.
 *
 * OUTPUT: 
 *         true if the packet has been queued successfully
 ******************************************************************/
bool
plm1_send_packet( int plm_type_t, int plm_priority_t, int plm_protocol_t,
		      int8_t repeat_limit, uint8_t *data, int8_t length, bool raw_mode );


/******************************************************************
 * plm1_receive: must be polled continuously to get received packets.
 * plm1_interrupt() must be called on every PLM-1 interrupt for
 * this function to work.
 *
 * INPUTS: None
 * OUTPUT: true if the library is loaded with a packet to transmit.
 ******************************************************************/
uint8_t
plm1_receive( uint8_t* rx_buffer );


/******************************************************************
 * plm1_tx_requested: returns true if a transmission is in progress.
 *
 * INPUTS: None
 * OUTPUT: true if the library is loaded with a packet to transmit.
 ******************************************************************/
bool
plm1_tx_requested( void );


/******************************************************************
 * plm1_abort_tx: aborts the current transmission.
 *
 * INPUTS: None
 * OUTPUT: true if the library is loaded with a packet to transmit.
 ******************************************************************/
void
plm1_abort_tx( void );


/******************************************************************
 * plm1_tx_finished: gets the tx status
 *
 * INPUTS: None
 * OUTPUT: true if a transmission has just been finished
 ******************************************************************/
bool
plm1_tx_finished( void );


/******************************************************************
 * plm1_configure: resets and configure the PLM-1
 *
 * INPUTS: PLM-1 configuration string (see PLM-1 documentation)
 * OUTPUT: None
 ******************************************************************/
void
plm1_configure( uint8_t * );


/******************************************************************
 * plm1_packet_missed: returns true upon a packet reception error.
 *
 * INPUTS: PLM-1 configuration string (see PLM-1 documentation)
 * OUTPUT: None
 ******************************************************************/
bool
plm1_packet_missed( void );


/******************************************************************
 * plm1_check_header: Check protocol received packet header: the 
 * first 2 bytes of the packet.
 *
 * INPUTS: protocol = protocol to match
 *         data = pointer to start of transmit buffer.
 * OUTPUT: true: if successful
 *         false: if empty packet or if another transmission was
 *                   already requested.
 ******************************************************************/
bool
plm1_check_header( uint8_t protocol, uint8_t *data );

/******************************************************************
 * plm1_receiving
 *
 * INPUTS: None
 * OUTPUT: true: if the library is receiving a packet
 *         false: no packet is being received
 ******************************************************************/
bool
plm1_receiving( void );


/******************************************************************
 * plm1_rx_data_available
 *
 * INPUTS: None
 * OUTPUT: true if a byte receievd is available
 *         false if no data is available
 ******************************************************************/
bool
plm1_rx_data_available( void );


/******************************************************************
 * plm1_rx_get_data: inserts data at the address pointed by the 
 * buffer argument.
 *
 * INPUTS: pointer where to insert the received byte.
 * OUTPUT: None
 ******************************************************************/
void
plm1_rx_get_data( uint8_t *buffer );


/******************************************************************
 * plm1_rx_packet_complete
 *
 * INPUTS: None
 * OUTPUT: true if the packet received is complete.
 ******************************************************************/
bool
plm1_rx_packet_complete( void );


/******************************************************************
 * plm1_disable_receiver: aborts the current reception
 *
 * INPUTS: None
 * OUTPUT: None
 ******************************************************************/
void
plm1_disable_receiver( void );


/******************************************************************
* Gets the current PLM-1 error. The application must poll
* with this function in order know when an error occurred and reset
* the part of packet it fetched from the plm buffer. The returned 
* variable is 0 if no error occurred.
*
* INPUTS: None
* OUTPUT: If error: current error code (0x0 to 0xF).
*         if no error: 0
 ******************************************************************/
uint8_t
plm1_get_error( void );


/******************************************************************
* plm1_reset_error: Resets the reception error flag. The receive buffer isn't
* cleared because some bytes of a new packet may have been received
* but not given to the application layer, to prevent it from appending
* them to the invalid packet it just received before.
*
* INPUTS: None
* OUTPUT: None
 ******************************************************************/
void
plm1_reset_error( void );


/******************************************************************
* plm1_interrupt handles the PLM-1 packet reception and transmission.
* this function must be called from the PLM-1 external interrupt
* ISR.
*
* INPUTS: None
* OUTPUT: None
 ******************************************************************/
void
plm1_interrupt( void );


#endif /* LIBPLM_H */
