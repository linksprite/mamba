/******************************************************************************************
 * Sample code for plm1 library.
 *
 *
 * 
 * Project description:
 *                   This program acts as a bridge from RS-232 to Powerline and Powerline
 *                   to RS-232. It features a simple main loop and 4 ISR; 2 for the UART
 *                   one for the PLM-1 external interrupt and one for a 10 ms timer.
 *
 *                   The interrupt for the PLM-1 external interrupt is mandatory. This
 *                   ISR handles the interrupts required for transmission and reception.
 *                   The higher the configuration of the PLM-1 baud rate, the faster
 *                   the interrupts will occur.
 *
 * Project files:
 *    plm1.h         Declarations of the plm1 library functions and definitions of user
 *                   parameters for the configuration of plm1.
 *    plmcfg.h       Contains definitions of reference configuration strings for the PLM-1.
 *    ports.h        Definitions of macros used for I/O manipulation.
 *    serial.h       Declarations of functions used for serial communication.
 *    spi.h          Enumerations and macros used for SPI communication with the PLM-1.
 *
 *    main.c         Main function and interrupt handlers of the program.
 *    plm.c          Libplm library. Contains all functions used by the library and the 
 *                   SPI initialization function for use with the PLM-1.
 *    serial.c       Functions used for serial communication.
 *
 *For compatibility with newers IDE, some changes was made.
 * Changes: Changed the function set_output for the register configuration and commented the declaration 
 * of pins MISO, MOSI, SCK.
 ******************************************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "spi.h"
#include "ports.h"
#include "plm1.h"
#include "serial.h"

/*
#define MOSI            B,3
#define MISO            B,4
#define SCK             B,5
*/

/* Timer 0 Configuration */
#define TIMER0_CLOCK_SRC_HZ         ( F_CPU / 1024 )  /* Nb ticks per second = 6144000 / 1024 = 6000 */		  //15626
#define TIMER0_NB_INT_FOR_100_MS    10 //10                 /* Nb ticks per 100ms = 600 */
#define TIMER0_NB_TICKS_FOR_100_MS  156 //60                 /* Nb ticks per 100ms = (10 * 60) */
#define TIMER0_RESET_VALUE          (255 - TIMER0_NB_TICKS_FOR_100_MS)


static volatile bool uart_tx_flag = false;      /* Flag for Tx UART interrupt. */
static volatile bool uart_rx_flag = false;      /* Flag for Rx UART interrupt. */
static volatile bool timer_flag = false;        /* Flag for Timer interrupt. */
static uint8_t uart_rx_data;                    /* Rx UART data read. */

uint8_t rx_packet[PLM_RX_MAX_PACKET_SIZE];
uint8_t rx_length = 0;
uint8_t tx_packet[PLM_TX_MAX_PACKET_SIZE];
uint8_t tx_length = 0;


int
main( void )
{

   
/*
   PIN_OUTPUT(MOSI);
   PIN_OUTPUT(SCK);
   PIN_OUTPUT(nPLM_RESET);
   PIN_OUTPUT(PLM_CS);
   PIN_INPUT(MISO);
*/

   DDRB &= ~_BV(4) | ~_BV(1); //clear these bits
   DDRB |= _BV(5)| _BV(3)| _BV(2)| _BV(0); // set these bits
   
   /* SPI communication must be initialized before plm1 library. */
   spi_init( SPI_SPEED_F16 );

   /* Initialization of plm1 library. */
   plm1_init();

   /* Initialization of the serial communication. */
   serial_init();

   /* Activate external interrupt 0 for PLM-1 */
   EIFR = 0xFF;                              /* Clear external interrupt flags. */
   EICRA = _BV( ISC00 ) | _BV( ISC01 );      /* INT0 activated on a rising edge. */
   EIMSK = _BV( INT0 );                      /* Enable only external interrupt 0 (PLM). */


   /* Enable RS232 TX and RX interrupts */
   UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);

   /* Set timer 0 for interrupts every 10 ms. */
   TCCR0A = _BV(WGM01);                      /* Timer mode: Capture on Compare. */
   TCCR0B = _BV(CS02) | _BV(CS00);           /* Timer cycle = Fosc/1024 = (6144000/1024) = 6000 Hz. */
   OCR0A = 60;   










   /* Activate global interrupts. */
   sei();

   for( ;; )
   {
      /* Flag true if a Tx UART interrupt occured. */
      if(uart_tx_flag)
      {
         uart_tx_flag = false;
         serial_tx_buffer_empty();
      }

      /* Flag true if a Rx UART interrupt occured. */
      if(uart_rx_flag)
      {
         /* Flag active if interrupt UART Rx Data Available was reaised.
          * When serial data is received, a 10 ms timer is started.
          * If the timer expires before additional data is received from the
          * UART, a transmission is started. */
         uart_rx_flag = false;
         timer_flag = false;

         /* Disable timer interrupt. */
         TIMSK0 = 0x00;

         /* Fill the buffer with data from UART. */
         tx_packet[tx_length++] = uart_rx_data;

         /* Start the 10ms timer. If the timer interrupt occurs before the next data byte,
          * send the packet. Otherwise, the timer is reset. */
         if(tx_length < PLM_TX_MAX_PACKET_SIZE - 10)
         {
            TCNT0 = 0x00;                             /* Reset timer. */
            TIMSK0 = _BV(OCIE0A);                     /* Enable Compare Match A interrupt. */
         }

         /* If the packet size is getting near the maximum, start immediately the transmission. */
         else
         {
            /* plm1_send_data returns true if the library has queued the packet sucessfully. */
            if( plm1_send_data(tx_packet, tx_length) )
            {
               /* Reset tx buffer if the packet was successfully transferred to the PLM-1 library. */
               tx_length = 0;
            }
            
            /* Otherwise, start Timer interrupt. */
            else
            {
               TCNT0 = 0x00;
               TIMSK0 = _BV(OCIE0A);
            }
         }
      }

      /* Flag tue if a timer interrupt occured. */
      if(timer_flag)
      {
         timer_flag = false;

         /* plm1_send_data returns true if the library has queued the packet sucessfully. */
         if( plm1_send_data(tx_packet, tx_length) )
         {
            tx_length = 0;
            TIMSK0 = 0x00;                             /* Disable timer interrupt. */
         }
      }

      /* When a packet is received, plm1_receive returns the length of the 
       * packet and copies the contents of the packet in rx_packet. */
      if( (rx_length = plm1_receive(rx_packet)) )
      {
         /* Sends data on the serial port. */
         output_serial_data(rx_packet, rx_length);
      }
   }

   return 0;                                 /* The main() function will never return. */
}



/* Interrupt Service Routine provenant de la ligne d'interruption du PLM. */
SIGNAL(INT0_vect)
{
   plm1_interrupt();
}


/* 10 ms timer interrupt ISR. */
SIGNAL(TIMER0_COMPA_vect)
{
   timer_flag = true;
}


// Uart RX
//*********************************************
SIGNAL(USART_RX_vect)
{
   uart_rx_flag = true;
   uart_rx_data = UDR0;

}

/* USART Tx Buffer Empty ISR. */
SIGNAL(USART_UDRE_vect)
{
	// One byte sent, now try to send next
   uart_tx_flag = true;

}


