/**********************************************************
*          SEGGER MICROCONTROLLER SYSTEME GmbH
*   Solutions for real time microcontroller applications
***********************************************************
File    : HIF_UART.c
Purpose : Terminal control for Flasher using USART1 on PA9/PA10
--------- END-OF-HEADER ---------------------------------*/
// changed to UART3 for NUCLEO-STM32H743ZI, DM1CR, Sep 1, 21
// USART3_RX PD9
// USART3_TX PD8

// RCC_APB1LENR offset 0x0e8
//
// Bit 18 USART3EN: USART3 Peripheral Clocks Enable
//   Set and reset by software.
//   0: USART3 peripheral clocks disable (default after reset)
//   1: USART3 peripheral clocks enabled
//   The peripheral clocks of the USART3 are: the kernel clock selected by USART234578SEL and
//   provided to usart_ker_ck input, and the rcc_pclk1 bus interface clock.
//
// RCC_AHB4ENR offset 0x0e0
//
// Bit 3 GPIODEN: GPIOD peripheral clock enable
//   Set and reset by software.
//   0: GPIOD peripheral clock disabled (default after reset)
//   1: GPIOD peripheral clock enabled

#include "SEGGER_SYSVIEW.h"

#if (SEGGER_UART_REC == 1)
#include "SEGGER_RTT.h"
#include "stm32h7xx.h"

#define OS_FSYS 400000000L   // MCU core frequency
#define RCC_BASE_ADDR            0x58024400

#define OFF_APB1ENR         0xE8
#define OFF_AHB4ENR         0xE0

#define RCC_APB1ENR         *(volatile uint32_t*)(RCC_BASE_ADDR + OFF_APB1ENR)
#define RCC_AHB4ENR         *(volatile uint32_t*)(RCC_BASE_ADDR + OFF_AHB4ENR)

#define GPIOD_BASE_ADDR     0x58020c00

#define OFF_MODER           0x00        // GPIOx_MODER    (GPIO port mode register)
#define OFF_OTYPER          0x04        // GPIOx_OTYPER   (GPIO port output type register)
#define OFF_OSPEEDR         0x08        // GPIOx_OSPEEDR  (GPIO port output speed register)
#define OFF_PUPDR           0x0C        // GPIOx_PUPDR    (GPIO port pull-up/pull-down register)
#define OFF_IDR             0x10        // GPIOx_IDR      (GPIO port input data register)
#define OFF_ODR             0x14        // GPIOx_ODR      (GPIO port output data register)
#define OFF_BSRR            0x18        // GPIOx_BSRR     (GPIO port bit set/reset register)
#define OFF_LCKR            0x1C        // GPIOx_LCKR     (GPIO port configuration lock register)
#define OFF_AFRL            0x20        // GPIOx_AFRL     (GPIO alternate function low register)
#define OFF_AFRH            0x24        // GPIOx_AFRH     (GPIO alternate function high register)

#define USART3_BASE_ADDR    0x40004800

#define OFF_CR1             0x00        // Control register 1
#define OFF_CR2             0x04        // Control register 2
#define OFF_CR3             0x08        // Control register 3
#define OFF_BRR             0x0c        // Baudrate register
#define OFF_SR              0x1c        // Interrupt Status register
#define OFF_RDR             0x24        // Receive Data register
#define OFF_TDR             0x28        // Transmit Data register


#define UART_BASECLK        OS_FSYS / 4       // USART3 runs on APB1 clock
#define GPIO_BASE_ADDR      GPIOD_BASE_ADDR   // GPIOD MODER
#define USART_BASE_ADDR     USART3_BASE_ADDR  // USART3 CR1
#define GPIO_UART_TX_BIT    8                 // USART3 TX: Pin pd8
#define GPIO_UART_RX_BIT    9                 // USART3 RX: Pin pd9
#define USART_IRQn          USART3_IRQn

#define GPIO_MODER          *(volatile uint32_t*)(GPIO_BASE_ADDR + OFF_MODER)
#define GPIO_AFRH           *(volatile uint32_t*)(GPIO_BASE_ADDR + OFF_AFRH)
#define GPIO_AFRL           *(volatile uint32_t*)(GPIO_BASE_ADDR + OFF_AFRL)

#define USART_SR            *(volatile uint32_t*)(USART_BASE_ADDR + OFF_SR)

#define USART_RDR            *(volatile uint32_t*)(USART_BASE_ADDR + OFF_RDR)
#define USART_TDR            *(volatile uint32_t*)(USART_BASE_ADDR + OFF_TDR)

#define USART_BRR           *(volatile uint32_t*)(USART_BASE_ADDR + OFF_BRR)
#define USART_CR1           *(volatile uint32_t*)(USART_BASE_ADDR + OFF_CR1)
#define USART_CR2           *(volatile uint32_t*)(USART_BASE_ADDR + OFF_CR2)
#define USART_CR3           *(volatile uint32_t*)(USART_BASE_ADDR + OFF_CR3)

#define USART_RXNE              5     // Read data register not empty flag (SR)
#define USART_TC                6     // Transmission complete flag (SR)
#define USART_RX_ERROR_FLAGS  0xB     // Parity, framing, overrun error flags (SR)
#define USART_RXNEIE            5     // Read data register not empty interrupt enable (CR1)
#define USART_TCIE              6     // Transmission complete interrupt enable (CR1)
#define USART_TXE               7     // Transmit data register empty (Does NOT indicate that byte has already been sent just indicates that it has been copied to the shift register)
#define USART_TXEIE             7     // Transmit data register empty interrupt enable


typedef void UART_ON_RX_FUNC(uint8_t Data);
typedef int  UART_ON_TX_FUNC(uint8_t* pChar);

typedef UART_ON_TX_FUNC* UART_ON_TX_FUNC_P;
typedef UART_ON_RX_FUNC* UART_ON_RX_FUNC_P;


static UART_ON_RX_FUNC_P _cbOnRx;
static UART_ON_TX_FUNC_P _cbOnTx;


void HIF_UART_Init(uint32_t Baudrate, UART_ON_TX_FUNC_P cbOnTx, UART_ON_RX_FUNC_P cbOnRx);


#define _SERVER_HELLO_SIZE        (4)
#define _TARGET_HELLO_SIZE        (4)

static const U8 _abHelloMsg[_TARGET_HELLO_SIZE] = { 'S', 'V', (SEGGER_SYSVIEW_VERSION / 10000), (SEGGER_SYSVIEW_VERSION / 1000) % 10 };  // "Hello" message expected by SysView: [ 'S', 'V', <PROTOCOL_MAJOR>, <PROTOCOL_MINOR> ]

static struct {
  U8         NumBytesHelloRcvd;
  U8         NumBytesHelloSent;
  int        ChannelID;
} _SVInfo = {0,0,1};

static void _StartSysView(void) {
  int r;

  r = SEGGER_SYSVIEW_IsStarted();
  if (r == 0) {
    SEGGER_SYSVIEW_Start();
  }
}

static void _cbOnUARTRx(U8 Data) {
  if (_SVInfo.NumBytesHelloRcvd < _SERVER_HELLO_SIZE) {  // Not all bytes of <Hello> message received by SysView yet?
    _SVInfo.NumBytesHelloRcvd++;
    goto Done;
  }
  _StartSysView();
  SEGGER_RTT_WriteDownBuffer(_SVInfo.ChannelID, &Data, 1);  // Write data into corresponding RTT buffer for application to read and handle accordingly
Done:
  return;
}

static int _cbOnUARTTx(U8* pChar) {
  int r;

  if (_SVInfo.NumBytesHelloSent < _TARGET_HELLO_SIZE) {  // Not all bytes of <Hello> message sent to SysView yet?
    *pChar = _abHelloMsg[_SVInfo.NumBytesHelloSent];
    _SVInfo.NumBytesHelloSent++;
    r = 1;
    goto Done;
  }
  r = SEGGER_RTT_ReadUpBufferNoLock(_SVInfo.ChannelID, pChar, 1);
  if (r < 0) {  // Failed to read from up buffer?
    r = 0;
  }
Done:
  return r;
}

void SEGGER_UART_init(U32 baud)
{
	HIF_UART_Init(baud, _cbOnUARTTx, _cbOnUARTRx);
}


/*********************************************************************
*
*       HIF_UART_WaitForTxEnd
*/
void HIF_UART_WaitForTxEnd(void) {
  //
  // Wait until transmission has finished (e.g. before changing baudrate).
  //
  while ((USART_SR & (1 << USART_TXE)) == 0);  // Wait until transmit buffer empty (Last byte shift from data to shift register)
  while ((USART_SR & (1 << USART_TC)) == 0);   // Wait until transmission is complete
}

/*********************************************************************
*
*       USART1_IRQHandler
*
*  Function descriptio
*    Interrupt handler.
*    Handles both, Rx and Tx interrupts
*
*  Notes
*    (1) This is a high-prio interrupt so it may NOT use embOS functions
*        However, this also means that embOS will never disable this interrupt
*/
void USART3_IRQHandler(void);
void USART3_IRQHandler(void) {
  int UsartStatus;
  uint8_t v;
  int r;

  UsartStatus = USART_SR;                              // Examine status register
  if (UsartStatus & (1 << USART_RXNE)) {               // Data received?
	  v = USART_RDR;                                      // Read data
    if ((UsartStatus & USART_RX_ERROR_FLAGS) == 0) {   // Only process data if no error occurred
      (void)v;                                         // Avoid warning in BTL
      if (_cbOnRx) {
        _cbOnRx(v);
      }
    }
  }
  if (UsartStatus & (1 << USART_TXE)) {                // Tx (data register) empty? => Send next character Note: Shift register may still hold a character that has not been sent yet.
    //
    // Under special circumstances, (old) BTL of Flasher does not wait until a complete string has been sent via UART,
    // so there might be an TxE interrupt pending *before* the FW had a chance to set the callbacks accordingly which would result in a NULL-pointer call...
    // Therefore, we need to check if the function pointer is valid.
    //
    if (_cbOnTx == NULL) {  // No callback set? => Nothing to do...
      return;
    }
    r = _cbOnTx(&v);
    if (r == 0) {                          // No more characters to send ?
      USART_CR1 &= ~(1UL << USART_TXEIE);  // Disable further tx interrupts
    } else {
      USART_SR;      // Makes sure that "transmission complete" flag in USART_SR is reset to 0 as soon as we write USART_DR. If USART_SR is not read before, writing USART_DR does not clear "transmission complete". See STM32F4 USART documentation for more detailed description.
      USART_TDR = v;  // Start transmission by writing to data register
    }
  }
}

/*********************************************************************
*
*       HIF_UART_EnableTXEInterrupt()
*/
void HIF_UART_EnableTXEInterrupt(void) {
  USART_CR1 |= (1 << USART_TXEIE);  // enable Tx empty interrupt => Triggered as soon as data register content has been copied to shift register
}



/*********************************************************************
*
*       HIF_UART_Init()
*/
void HIF_UART_Init(uint32_t Baudrate, UART_ON_TX_FUNC_P cbOnTx, UART_ON_RX_FUNC_P cbOnRx) {
  uint32_t v;
  uint32_t Div;
  //
  // Configure USART RX/TX pins for alternate function AF7
  //
  RCC_APB1ENR |= (1 <<  18);        // Enable USART3 clock
  RCC_AHB4ENR |= (1 <<  3);         // Enable IO port D clock

  v  = GPIO_AFRH;
  v &= ~((15UL << ((GPIO_UART_TX_BIT % 8) << 2)) | (15UL << ((GPIO_UART_RX_BIT % 8) << 2)));
  v |=   ((7UL << ((GPIO_UART_TX_BIT % 8) << 2)) | (7UL << ((GPIO_UART_RX_BIT % 8) << 2)));
  GPIO_AFRH = v;
  //
  // Configure USART RX/TX pins for alternate function usage
  //
  v  = GPIO_MODER;
  v &= ~((3UL << (GPIO_UART_TX_BIT << 1)) | (3UL << (GPIO_UART_RX_BIT << 1)));
  v |=  ((2UL << (GPIO_UART_TX_BIT << 1)) | (2UL << (GPIO_UART_RX_BIT << 1)));         // PA10: alternate function
  GPIO_MODER = v;
  //
  // Initialize USART
  //
  USART_CR1 = 0
            | (1 << 15)                         // OVER8  = 1; Oversampling by 8
            | (1 << 0)                          // UE     = 1; USART enabled
            | (0 << 12)                         // M      = 0; Word length is 1 start bit, 8 data bits
            | (0 << 10)                         // PCE    = 0; No parity control
            | (1 <<  5)                         // RXNEIE = 1; RXNE interrupt enabled
            | (1 <<  3)                         // TE     = 1; Transmitter enabled
            | (1 <<  2)                         // RE     = 1; Receiver enabled
            ;
  USART_CR2 = 0
            | (0 << 12)                         // STOP = 00b; 1 stop bit
            ;
  USART_CR3 = 0
            | (0 << 11)                         // ONEBIT = 0; Three sample bit method
            | (1 <<  7)                         // DMAT   = 1; DMA for transmitter enabled
            ;
  //
  // Set baudrate
  //
  Div = Baudrate * 8;                       // We use 8x oversampling.
  Div = ((2 * (UART_BASECLK)) / Div) + 1;   // Calculate divider for baudrate and round it correctly. This is necessary to get a tolerance as small as possible.
  Div = Div / 2;
  if (Div > 0xFFF) {
    Div = 0xFFF;        // Limit to 12 bit (mantissa in BRR)
  }
  if (Div >= 1) {
    USART_BRR = 0xFFF0 & (Div << 4);    // Use only mantissa of fractional divider
  }
  //
  // Setup callbacks which are called by ISR handler and enable interrupt in NVIC
  //
  _cbOnRx = cbOnRx;
  _cbOnTx = cbOnTx;
  NVIC_SetPriority(USART_IRQn, 6);  // Highest prio, so it is not disabled by embOS
  NVIC_EnableIRQ(USART_IRQn);
}

#endif
