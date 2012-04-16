#include    <syszig.h>
#include    <_zigzag.h>
#include    <msp430.h>
#include    <msp430uart1.h>

//#include <msp430x16x.h>

/*
IRQ_HANDLER( UARTRX_IRQ )
{
    uint8_t temp = URXBUF;
    uart_rx_done(temp);
}


IRQ_HANDLER( UARTTX_IRQ )
{
    // is i2c ? 
    if ((UCTL & 0x20) && (UCTL & 0x04) && (UCTL & 0x01))
        return;
    uart_tx_done();
}
*/

#define SPI_SOFT

#ifdef SPI_SOFT
#define SPI_MOSI   0x02
#define SPI_MISO   0x04
#define SPI_CLK    0x08

#define SPI_OUT    P3OUT
#define SPI_IN     P3IN
#define SPI_SEL    P3SEL
#define SPI_DIR    P3DIR
#endif

#ifdef SPI_SOFT
unsigned char SPI_tx(const uint8_t data)
{
   unsigned int i;
   unsigned char rcvd = 0;
  
//   P5OUT &= ~SPI_CLK;
   SPI_OUT &= ~SPI_CLK;
   for(i = 0x80; i; i >>= 1)
   {
      rcvd <<= 1;
      if(data & i)
         SPI_OUT |=  SPI_MOSI; //P5OUT |=  SPI_MOSI;
      else
         SPI_OUT &= ~SPI_MOSI; //P5OUT &= ~SPI_MOSI;
//      if(P5IN & SPI_MISO)
      if(SPI_IN & SPI_MISO)
         rcvd |= 0x01;
      SPI_OUT |=  SPI_CLK; //P5OUT |=  SPI_CLK;
      SPI_OUT |=  SPI_CLK; //P5OUT |=  SPI_CLK;
      SPI_OUT |=  SPI_CLK; //P5OUT |=  SPI_CLK;
      SPI_OUT |=  SPI_CLK; //P5OUT |=  SPI_CLK;
      SPI_OUT &= ~SPI_CLK; //P5OUT &= ~SPI_CLK;
   }
   return rcvd;
}
#else  // SPI_SOFT
unsigned char SPI_tx(const uint8_t data)
{
   UTXBUF = data;
   while(!(UTCTL & TXEPT));
   while ((IFG2 & URXIFG1)==0);   // wait for RX buffer (full)
   return (URXBUF);
      
}
#endif // SPI_SOFT

void SPI_init()
{
   if( ISZIGLOAD ) __critical_enter();

#ifdef SPI_SOFT
   SPI_DIR  = (SPI_DIR | SPI_MOSI | SPI_CLK) & (~SPI_MISO); //0x0a; // P5.3 (UCLK1) and P5.1 (SIMO1) - outputs
   SPI_SEL &= ~(SPI_MOSI | SPI_MISO | SPI_CLK); // 0x0a; // They peripherial I/O pins
   SPI_OUT &= ~SPI_CLK;
// ME2   &= ~USPIE1;  // USART SPI module enable
   /*
   P5DIR  = (P5DIR | SPI_MOSI | SPI_CLK) & (~SPI_MISO); //0x0a; // P5.3 (UCLK1) and P5.1 (SIMO1) - outputs
   P5SEL &= ~(SPI_MOSI | SPI_MISO | SPI_CLK); // 0x0a; // They peripherial I/O pins
   ME2   &= ~USPIE1;  // USART SPI module enable
   P5OUT &= ~SPI_CLK;
   */
#else  // SPI_SOFT
   P5DIR  =  (P5DIR | 0x0a) & ~0x04; // P5.3 (UCLK1) and P5.1 (SIMO1) - outputs
   P5SEL |=  0x0e; // They peripherial I/O pins
   UCTL |= SWRST;  
   UCTL |= CHAR | SYNC | MM;  // 8-bit char, spi-mode, USART as master
   UCTL &= ~(0x20); 
   UTCTL = STC ;     // 3-pin
   UTCTL |= CKPH | SSEL_2;    // half-cycle delayed UCLK 
   UBR0 = 0x02;   // as fast as possible
   UBR1 = 0x00;
   UMCTL = 0;
   ME2 &= ~(UTXE1 | URXE1); //USART UART module disable
   ME2 |= USPIE1;  // USART SPI module enable
   UCTL &= ~SWRST;  
#endif // SPI_SOFT
   if( ISZIGLOAD ) __critical_exit();
}

void SPI_stop()
{
   if( ISZIGLOAD ) __critical_enter();

#ifndef SPI_SOFT
   IFGx &= ~(UTXIFGx | URXIFGx);
   IEx &= ~(UTXIEx | URXIEx);  // interrupt disabled

   MEx &= ~(UTXEx | URXEx);    // USART0 UART module disable
   ME2 &= ~USPIE1;  // USART SPI module disable

   P5SEL &=  0x0a; // They peripherial I/O pins
#endif // SPI_SOFT

   if( ISZIGLOAD ) __critical_exit();
}

