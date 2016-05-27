# msp430g2xx2_timer_uart

MSP430G2xx2 TimerA full-duplex UART echo firmware.

Some low cost MSP430G2XX2 micro-controllers do not have hardware UART, but all of them have at least Timer A. This firmware uses Timer A to implement a full-duplex UART.

The micro-controller sleeps in LPM0 while receiving UART character. The received character is pushed into a ring buffer to be echoed later. Transmit characters are pushed into ring buffer first before being slowly drained out to the UART.

### Setup:
* MSP430G2452 micro-processor
* MSP-EXP430G2 emulation board
* IAR Embedded Workbench for MSP430
* MCLK = SMCLK = 12MHz
* Baud rate = 9600

Note: This firmware is based on TI's example code.
