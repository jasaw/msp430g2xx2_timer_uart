/**
 *  MSP430G2xx2 TimerA full-duplex UART echo software at 9600 baud
 *
 *  MCLK = SMCLK = 12MHz
 *  
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "msp430g2452.h"


#define     UART_TXD              BIT1      // TXD on P1.1 (Timer0_A.OUT0)
#define     UART_RXD              BIT2      // RXD on P1.2 (Timer0_A.CCI1A)

#define     LED1                  BIT0
#define     LED2                  BIT6
#define     LED_DIR               P1DIR
#define     LED_OUT               P1OUT

#define UART_TBIT_DIV_2     (12000000 / (9600 * 2))
#define UART_TBIT           (12000000 / 9600)
#define UART_BUFFER_SIZE    (unsigned char)32

struct ring_buffer
{
    unsigned char buf[UART_BUFFER_SIZE];
    volatile unsigned char head;
    volatile unsigned char tail;
};
static struct ring_buffer uartTxBuffer = {.head = 0, .tail = 0};
static struct ring_buffer uartRxBuffer = {.head = 0, .tail = 0};
static unsigned int uartTxByte;             // UART internal variable for TX


static void TimerA_UART_init(void);
static void TimerA_UART_tx(unsigned char byte);
static void TimerA_UART_print(char *string);


static void setup_clocks(void)
{
    DCOCTL = 0x00;                          // Set DCOCLK to 12MHz
    BCSCTL1 = CALBC1_12MHZ;
    DCOCTL = CALDCO_12MHZ;
    BCSCTL2 &= ~(DIVS_3);                   // SMCLK = DCO = 12MHz
    BCSCTL3 |= LFXT1S_2;                    // ACLK = VLO
}

static void setup_io_pins(void)
{
    P1OUT = 0x00;
    P1SEL = UART_TXD + UART_RXD;
    P1DIR = UART_TXD;
    // set all unused pins to output to reduce power consumption
    //P1DIR = 0xFF & ~UART_RXD;
    P2OUT = 0x00;
    P2SEL = 0x00;
    // set all unused pins to output to reduce power consumption
    //P2DIR = 0xFF;
}

static void setup_leds(void)
{
    LED_DIR |= LED1 + LED2;
    LED_OUT &= ~(LED1 + LED2);
}


inline int ring_buffer_full(struct ring_buffer *rb)
{
    unsigned char head = rb->head;
    return ((head - rb->tail) == UART_BUFFER_SIZE) ? 1 : 0;
}

inline int ring_buffer_empty(struct ring_buffer *rb)
{
    unsigned char head = rb->head;
    return ((head - rb->tail) == 0U) ? 1 : 0;
}

inline unsigned char ring_buffer_increment_index(unsigned char index)
{
    return (index + 1) & (UART_BUFFER_SIZE - 1);
}


void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;               // Stop watchdog timer
    setup_clocks();
    setup_io_pins();
    setup_leds();

    TimerA_UART_init();
    __enable_interrupt();

    TimerA_UART_print("G2xx2 TimerA UART\r\n");
    TimerA_UART_print("READY.\r\n");

    while (1)
    {
        // Wait for incoming character
        __bis_SR_register(LPM0_bits);

        // Echo received character
        unsigned char head = uartRxBuffer.head;
        while (head != uartRxBuffer.tail)
        {
            if (ring_buffer_full(&uartTxBuffer))
                break;
            unsigned char uartRxByte = uartRxBuffer.buf[uartRxBuffer.tail];
            uartRxBuffer.tail = ring_buffer_increment_index(uartRxBuffer.tail);
            TimerA_UART_tx(uartRxByte);
            if (uartRxByte == '\r')
                TimerA_UART_tx('\n');
        }
    }
}


//------------------------------------------------------------------------------
// Function configures Timer_A for full-duplex UART operation
//------------------------------------------------------------------------------
static void TimerA_UART_init(void)
{
    TACCTL0 = OUT;                          // Set TXD Idle as Mark = '1'
    TACCTL1 = SCS + CM1 + CAP + CCIE;       // Sync, Neg Edge, Capture, Int
    TACTL = TASSEL_2 + MC_2;                // SMCLK, start in continuous mode
}

static void TimerA_UART_tx(unsigned char byte)
{
    // while buffer full, busy loop
    while (ring_buffer_full(&uartTxBuffer)) {}
    uartTxBuffer.buf[uartTxBuffer.head] = byte;
    uartTxBuffer.head = ring_buffer_increment_index(uartTxBuffer.head);
    if ((TACCTL0 & CCIE) == 0)              // If Tx timer idle, kick start it
    {
        TACCR0 = TAR;                       // Current state of TA counter
        TACCR0 += UART_TBIT;                // One bit time till first bit
        TACCTL0 = OUTMOD0 + CCIE;           // Set TXD on EQU0, Int
        uartTxByte = uartTxBuffer.buf[uartTxBuffer.tail]; // Load global variable
        uartTxBuffer.tail = ring_buffer_increment_index(uartTxBuffer.tail);
        uartTxByte |= 0x100;                // Add mark stop bit to TXData
        uartTxByte <<= 1;                   // Add space start bit
    }
}


static void TimerA_UART_print(char *string)
{
    while (*string) {
        TimerA_UART_tx(*string++);
    }
}

//------------------------------------------------------------------------------
// Timer_A UART - Transmit Interrupt Handler
//------------------------------------------------------------------------------
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
{
    static unsigned char txBitCnt = 10;

    TACCR0 += UART_TBIT;                    // Add Offset to CCRx
    if (txBitCnt == 0) {                    // All bits TXed?
        // if buffer empty, disable Tx timer
        if (ring_buffer_empty(&uartTxBuffer)) {
            TACCTL0 &= ~CCIE;               // All bits TXed, disable interrupt
            txBitCnt = 10;                  // Re-load bit counter
        } else {
            TACCTL0 = OUTMOD0 + CCIE;       // Set TXD on EQU0, Int
            txBitCnt = 10;                  // Re-load bit counter
            uartTxByte = uartTxBuffer.buf[uartTxBuffer.tail]; // Load global variable
            uartTxBuffer.tail = ring_buffer_increment_index(uartTxBuffer.tail);
            uartTxByte |= 0x100;            // Add mark stop bit to TXData
            uartTxByte <<= 1;               // Add space start bit
        }
        //__bic_SR_register_on_exit(LPM0_bits);  // Clear LPM0 bits from 0(SR)
    } else {
        if (uartTxByte & 0x01) {
          TACCTL0 &= ~OUTMOD2;              // TX Mark '1'
        } else {
          TACCTL0 |= OUTMOD2;               // TX Space '0'
        }
        uartTxByte >>= 1;
        txBitCnt--;
    }
}
//------------------------------------------------------------------------------
// Timer_A UART - Receive Interrupt Handler
//------------------------------------------------------------------------------
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void)
{
    static unsigned char rxBitCnt = 8;
    static unsigned char rxData = 0;

    switch (__even_in_range(TA0IV, TA0IV_TAIFG)) { // Use calculated branching
        case TA0IV_TACCR1:                        // TACCR1 CCIFG - UART RX
            TACCR1 += UART_TBIT;                 // Add Offset to CCRx
            if (TACCTL1 & CAP) {                 // Capture mode = start bit edge
                TACCTL1 &= ~CAP;                 // Switch capture to compare mode
                TACCR1 += UART_TBIT_DIV_2;       // Point CCRx to middle of D0
            } else {
                rxData >>= 1;
                if (TACCTL1 & SCCI) {            // Get bit waiting in receive latch
                    rxData |= 0x80;
                }
                rxBitCnt--;
                if (rxBitCnt == 0) {             // All bits RXed?
                    // if buffer not full, copy rx byte
                    if (!ring_buffer_full(&uartRxBuffer)) {
                        uartRxBuffer.buf[uartRxBuffer.head] = rxData;
                        uartRxBuffer.head = ring_buffer_increment_index(uartRxBuffer.head);
                    }
                    rxBitCnt = 8;                // Re-load bit counter
                    TACCTL1 |= CAP;              // Switch compare to capture mode
                    __bic_SR_register_on_exit(LPM0_bits);  // Clear LPM0 bits from 0(SR)
                }
            }
            break;
    }
}
//------------------------------------------------------------------------------
