#ifndef _SPI_H_
#define _SPI_H_

#include <avr/io.h>

enum spi_speed_t {
   SPI_SPEED_F2 = 4,       /* Fosc/2 */
   SPI_SPEED_F4 = 0,       /* Fosc/4 */
   SPI_SPEED_F8 = 5,       /* Fosc/8 */
   SPI_SPEED_F16 = 1,      /* Fosc/16 */
   SPI_SPEED_F32 = 6,      /* Fosc/32 */
   SPI_SPEED_F64 = 2,      /* Fosc/64 */
   SPI_SPEED_F128 = 3      /* Fosc/128 */
};


#define SPI_SPR_MASK   0x03 /* Mask for SPI SPR0 and SPR1 bits in SPCR register. */
#define SPI_SPI2X_MASK 0x01 /* Mask for SPI Double Speed bit in SPSR register. */

#endif