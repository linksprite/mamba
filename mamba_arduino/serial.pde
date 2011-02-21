#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "serial.h"
#include "ports.h"

#define BAUD_RATE       9600
#define UART_DIVIDER    ((uint32_t)F_CPU/((uint32_t)BAUD_RATE*16) - 1)

static uint8_t serial_tx_buffer[60];
static uint8_t *serial_tx_ptr;
static uint8_t serial_tx_len;

void output_serial_data(uint8_t *data, uint8_t length)
{
   memcpy(serial_tx_buffer, data, length);         /* Copy data to internal buffer. */

   serial_tx_ptr = serial_tx_buffer;
   serial_tx_len = length;

   UDR0 = *serial_tx_ptr++;
   UCSR0B |= _BV(UDRIE0);     /* Enable interrupts. */
}


void serial_tx_buffer_empty( void ) {

   if(!serial_tx_len)
   {
      UCSR0B &= ~_BV(UDRIE0);
   }
   else
   {
      if(--serial_tx_len)
      {
         UDR0 = *serial_tx_ptr++;
      }
      else
      {
         UCSR0B &= ~_BV(UDRIE0);
      }
   }
}

/* Initializes the UART for 9600, 8, N, 1 configuration. */
void
serial_init( void ) {

   UBRR0 = UART_DIVIDER;
   UCSR0C = 0x06;                               /* 8, None, 1. */
}
