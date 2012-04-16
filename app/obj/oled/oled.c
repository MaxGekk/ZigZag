/*
 * oled.c - routines for OLED display
 */

//#include <msp430x16x.h>
#include <zzPort.h>
#include "oled.h"
#include "font.h"

port_attr_t port5_attr;
port_attr_t port6_attr;

#define __strlen strlen
/*
unsigned int __strlen(const char * str)
{
   unsigned int size = 0;
   while(*str++)
      size++;
   return size;
}
*/

static void OLED_initSPI(void)
{
   /*
   IE2 &= ~(UTXIE1 | URXIE1);  // interrupt disable

   P5DIR |=  0x0a; // P5.3 (UCLK1) and P5.1 (SIMO1) - outputs
   P5SEL |=  0x0a; // They peripherial I/O pins
   
   U1CTL |= SWRST;  
   U1CTL |= CHAR | SYNC | MM;  // 8-bit char, spi-mode, USART as master
   U1CTL &= ~(0x20); 
   U1TCTL = STC ;     // 3-pin
//   U1TCTL |= CKPL | SSEL_1;    //
   U1TCTL |= CKPH | CKPL | SSEL_2;    // half-cycle delayed UCLK 
   U1BR0 = 0x02;   // as fast as possible
   U1BR1 = 0x00;
   U1MCTL = 0;
   ME2 &= ~(UTXE1 | URXE1); //USART UART module disable
   ME2 |= USPIE1;  // USART SPI module enable
   U1CTL &= ~SWRST;
   */
   
   SPI_init();
                     
}

static inline void OLED_Data(void)
{
//   P5OUT |=  OLED_D_C;
   port_write(OLED_PORT5, OLED_D_C, PIN_HI);
}

static inline void OLED_Command(void)
{
//   P5OUT &= ~OLED_D_C;
   port_write(OLED_PORT5, OLED_D_C, PIN_LO);
}

static inline void OLED_SelectDSPL(void)
{
//   P5OUT &= ~OLED_CS2;
   port_write(OLED_PORT5, OLED_CS2, PIN_LO);
}

static inline void OLED_DeselectDSPL(void)
{
//   P5OUT |=  OLED_CS2;
   port_write(OLED_PORT5, OLED_CS2, PIN_HI);
}

static inline void OLED_DSPLon(void)
{
//   P5OUT &= ~OLED_on;
   port_write(OLED_PORT5, OLED_on, PIN_LO);
}

static inline void OLED_DSPLoff(void)
{
//   P5OUT |=  OLED_on;
   port_write(OLED_PORT5, OLED_on, PIN_HI);
}

static inline void OLED_DCon(void)
{
//   P6OUT &= ~OLED_DC_on;
#ifdef EBOARD
   port_write(OLED_PORT6, OLED_DC_on, PIN_LO);
#else  // EBOARD
   port_write(OLED_PORT6, OLED_DC_on, PIN_HI);
#endif // EBOARD
}

static inline void OLED_DCoff(void)
{
//   P6OUT |=  OLED_DC_on;
#ifdef EBOARD
   port_write(OLED_PORT6, OLED_DC_on, PIN_HI);
#else  // EBOARD
   port_write(OLED_PORT6, OLED_DC_on, PIN_LO);
#endif // EBOARD
}

static void OLED_sendByte(uint8_t val)
{
   /*
   U1TXBUF = val;
   while(!(U1TCTL & TXEPT));
   */
   SPI_tx(val);
}

static void OLED_SendData(unsigned char dat)
{
  char i;
  OLED_Data();
  OLED_SelectDSPL();
  OLED_sendByte(dat);
  /*
  for (i=8;i>0;i--)
  {
    if(dat&0x80) P5OUT|=MOSI; else P5OUT&=~MOSI;
    P5OUT^=UCLK;
    dat=dat<<1;
    P5OUT^=UCLK;
  }
  */
  OLED_DeselectDSPL();
}

static void OLED_SendComm(int com)
{
  char i;
  OLED_Command();
  OLED_SelectDSPL();
  /*
  for (i=16;i>0;i--)
  {
    if(com&0x8000) P5OUT|=MOSI; else P5OUT&=~MOSI;
    P5OUT^=UCLK;
    com=com<<1;
    P5OUT^=UCLK;
  }
  */
  OLED_sendByte(com >> 8);
  OLED_sendByte(com);
 /* __DeselectDSPL();
  __SelectDSPL();
  */
  OLED_DeselectDSPL();
}

void OLED_pause(unsigned int D)
{
   volatile unsigned int i;
   for (i=D;i--;);
}

void OLED_reset(void)
{
   /*
   P5OUT |= OLED_RST;
   P5OUT ^= OLED_RST;
   OLED_pause(10000);
   P5OUT ^= OLED_RST;
   */

   port_write(OLED_PORT5, OLED_RST, PIN_HI);
   port_write(OLED_PORT5, OLED_RST, PIN_LO);
   OLED_pause(10000);
   port_write(OLED_PORT5, OLED_RST, PIN_HI);
}

void OLED_init(void)
{
   port_attr_t port_attr;
   PIN_CLEAR(port_attr.ie, 0x01);
   PIN_SET(port_attr.dir, 0x01, PIN_HI);
   PIN_CLEAR(port_attr.sel, 0x01);

   port_set_attr(4, 0x01, &port_attr);
   port_write(4, 0x01, PIN_HI);
   
   /*
   P5OUT |=   OLED_RST | OLED_D_C | OLED_on | OLED_CS2 ;
   P5DIR |=   OLED_RST | OLED_D_C | OLED_on | OLED_CS2 ;
   P5SEL &= ~(OLED_RST | OLED_D_C | OLED_on | OLED_CS2);
   */
   
   PIN_CLEAR(port5_attr.ie, OLED_D_C);
   PIN_CLEAR(port5_attr.ie, OLED_CS2);
   PIN_CLEAR(port5_attr.ie, OLED_RST);
   PIN_CLEAR(port5_attr.ie, OLED_on);

   PIN_SET(port5_attr.dir, OLED_D_C, PIN_HI);
   PIN_SET(port5_attr.dir, OLED_CS2, PIN_HI);
   PIN_SET(port5_attr.dir, OLED_RST, PIN_HI);
   PIN_SET(port5_attr.dir, OLED_on,  PIN_HI);

   PIN_CLEAR(port5_attr.sel, OLED_D_C);


   PIN_CLEAR(port5_attr.sel, OLED_CS2);
   PIN_CLEAR(port5_attr.sel, OLED_RST);
   PIN_CLEAR(port5_attr.sel, OLED_on);

   port_set_attr(OLED_PORT5, OLED_D_C | OLED_CS2 | OLED_RST | OLED_on, &port5_attr);
   port_write(OLED_PORT5, OLED_D_C | OLED_CS2 | OLED_RST | OLED_on, PIN_HI);
   
   PIN_CLEAR(port6_attr.ie, OLED_DC_on);
   PIN_SET(port6_attr.dir, OLED_DC_on,  PIN_HI);
   PIN_CLEAR(port6_attr.sel, OLED_DC_on);
   port_set_attr(OLED_PORT6, OLED_DC_on, &port6_attr);

   OLED_initSPI();
//  P5SEL=MOSI|UCLK;
/*
   P6OUT |=  OLED_DC_on;
   P6DIR |=  OLED_DC_on;
   P6SEL &= ~OLED_DC_on;
*/
   port_write(OLED_PORT6, OLED_DC_on, PIN_HI);

   OLED_DSPLon();          // Switch OLED display power supply on
   OLED_reset();           // Reset it
   OLED_DCon();            // Switch DC/DC converter on
   OLED_pause(10000);      // Wait for it to start
   OLED_SendComm(0x01A0);  // Set initial brightness 
   OLED_SendComm(0x1001);  // Enable display
   OLED_SendComm(0x1101);  // Select direction (horizontal mirroring)
   OLED_SendComm(0x123F);  // Selec number of lines to scan (64)
   OLED_SendComm(0x1503);  // Select display mode (16 gray) and RAM pointer increment mode
   OLED_SendComm(0x160B);  // Set up dimmer control
   // SendComm(0x1702);
   // SendComm(0x1A00);
   OLED_SendComm(0x1C81);  // Setup period 2
   OLED_SendComm(0x1D81);  // Setup period 3
   OLED_SendComm(0x1E89);  // Setup period 4
   OLED_cls();             // Clear screen
}

/*
void OLED_setX(uint8_t X)
{
   OLED_SendComm(0x1300 | X);
}

void OLED_setY(uint8_t Y)
{
   OLED_SendComm(0x1400 | Y);
}
*/
/*
void OLED_cls()
{
   uint16_t i, j;
   for(i = OLED_Y_SIZE; i--; )
   {
      OLED_setX(0);
      OLED_setY(i);
      for(j = OLED_X_SIZE / 2; j--; )
         OLED_SendData(0x00);
   }
   OLED_setX(0);
   OLED_setY(0);
}
*/

void OLED_clrLine(uint16_t from, uint16_t to)
{
   uint16_t j;
   while(from != to)
   {
      OLED_setX(0);
      OLED_setY(from++);
      for(j = OLED_X_SIZE / 2; j--; )
         OLED_SendData(0x00);
   }
}

void OLED_clr(uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize)
{
   uint16_t i, j;
   x /= 2;
   for(i = y + ysize; i > y; )
   {
      OLED_setX(x);
      OLED_setY(--i);
      for(j = x + xsize / 2; j > x; j--)
         OLED_SendData(0x00);
   }
}

void OLED_putBitmap(uint16_t x, uint16_t y, struct OLED_bitmap * bmp)
{
   uint16_t i, j, x_coord, pos;
   i = j = 0;
   x_coord = x >> 1; // 2 pixels per byte
   OLED_setX(x_coord); // Set zero position
   OLED_setY(y);
   for(pos = 0; pos < bmp->bmpSize; pos++)
   {
      if(x_coord < OLED_X_SIZE)
      {
         OLED_SendData(0xff - bmp->BitmapData[pos]);
         x_coord += 2;
      }
      i += 2;
      if(i >= bmp->bmpWidth)
      {
         i = 0;
         x_coord = x >> 1;
         j++;
         OLED_setX(x_coord); // Set zero position
         OLED_setY(++y);
         if(y >= OLED_Y_SIZE)
            break;
      }
   }
}

static const unsigned char OLED_2pxls[4] = {0x00, 0xf0, 0x0f, 0xff};

void OLED_puts(uint16_t x, uint16_t y, uint8_t color, uint8_t font_num, unsigned char * str)
{
   uint16_t i, j, k, pos, x_coord;
   uint8_t  bits, byte_cnt, bit_cnt;
   uint16_t str_len = __strlen(str);
   const struct fontHeader * ptrFh = getFont(font_num);
   unsigned char * fontPtr = (unsigned char *)ptrFh + sizeof(struct fontHeader) + sizeof(struct blockHeader) * ptrFh->block_num;

   color  = color & 0xf0;
   color |= color >> 4;

   for(j = 0; j < ptrFh->height; j++)
   {
      OLED_setY(j + y);
      OLED_setX(x / 2);
      x_coord = x;
//      OLED_SendComm(0x1400 | (uint8_t)(j + y));
//      OLED_SendComm(0x1300 | (uint8_t)x / 2);
      for(i = 0; i < str_len; i++)
      {
         // Get pointer to font symbol
         pos = 0;
         for(k = 0; k < ptrFh->block_num; k++)
         {
            if((str[i] >= ptrFh->bh[k].fisrtSymCode) && (str[i] < (ptrFh->bh[k].fisrtSymCode + ptrFh->bh[k].blkSize)))
               break;
            pos += ptrFh->bh[k].blkSize;
         }
         
         if(k == ptrFh->block_num)
         {
            for(k = 0; k < ptrFh->width; k += 2)
            {
               if(x_coord < OLED_X_SIZE)
               {
                  OLED_SendData(0xff);
                  x_coord += 2;
               }
            }
            /*
            OLED_SendData(0xff);
            OLED_SendData(0xff);
            OLED_SendData(0xff);
            */
         }
         else
         {
            /*
            bits = fontPtr[(pos + str[i] - ptrFh->bh[k].fisrtSymCode) * ptrFh->height + j];
            for(k = 0; k < 3; k++)
            {
               OLED_SendData((OLED_2pxls[(bits & 0xc0) >> 6]) & color);
               bits <<= 2;
            }
            */
            
            bit_cnt = 0;
            for(byte_cnt = 0; byte_cnt < (ptrFh->width / 8 + 1); byte_cnt++)
            {
               bits = fontPtr[(pos + str[i] - ptrFh->bh[k].fisrtSymCode + byte_cnt) * ptrFh->height + j];
               for(k = 0; k < 8 / 2; k++)
               {
                  if(bit_cnt >= ptrFh->width)
                     break;
                  if(x_coord < OLED_X_SIZE)
                  {
                     OLED_SendData((OLED_2pxls[(bits & 0xc0) >> 6]) & color);
                     x_coord += 2;
                  }
                  bits <<= 2;
                  bit_cnt += 2;
               }
            }
         }
      }
   }
}

