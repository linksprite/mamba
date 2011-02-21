#ifndef _PORTS_H_
#define _PORTS_H_

#define BIT(p,b)        (b)

#define PORT(p,b)       (PORT ## p)
#define PIN(p,b)        (PIN ## p)
#define DDR(p,b)        (DDR ## p)

#define SET_IO_BIT(p,b) ((p) |= _BV(b))
#define CLR_IO_BIT(p,b) ((p) &= ~_BV(b))
#define TGL_IO_BIT(p,b) ((p) ^= _BV(b))
#define GET_IO_BIT(p,b) (((p) & _BV(b)) != 0)

#define SET_OUTPUT(io)  SET_IO_BIT(PORT(io),BIT(io))
#define CLR_OUTPUT(io)  CLR_IO_BIT(PORT(io),BIT(io))
#define TGL_OUTPUT(io)  TGL_IO_BIT(PORT(io),BIT(io))
#define GET_OUTPUT(io)  GET_IO_BIT(PORT(io),BIT(io))
#define GET_INPUT(io)   GET_IO_BIT(PIN(io),BIT(io))

#define PIN_OUTPUT(io)  SET_IO_BIT(DDR(io),BIT(io))
#define PIN_INPUT(io)   CLR_IO_BIT(DDR(io),BIT(io))


/* Timer 0 and 1 Prescaler definitions. */
#define TIMER01_SRC_OFF           0
#define TIMER01_SRC_CLK_DIV1      1
#define TIMER01_SRC_CLK_DIV8      2
#define TIMER01_SRC_CLK_DIV64     3
#define TIMER01_SRC_CLK_DIV256    4
#define TIMER01_SRC_CLK_DIV1024   5
#define TIMER01_SRC_T0PIN_FALLING 6
#define TIMER01_SRC_T0PIN_RISING  7

#endif