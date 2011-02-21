#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define PARITY_NONE        0
#define PARITY_ODD         1
#define PARITY_EVEN        2

void
serial_init( void );

void 
output_serial_data(uint8_t *data, uint8_t length);

void
serial_tx_buffer_empty( void );

void
serial_data_available( void );

#endif
