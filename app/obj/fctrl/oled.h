/*
 * oled.h - inteface to OLED display routines
 */
#ifndef _OLED_H_
#define _OLED_H_

//#define EBOARD

#define OLED_PORT5           5
#define OLED_PORT6           6

#ifndef BIT0
#define BIT0                 0x01
#define BIT1                 0x02
#define BIT2                 0x04
#define BIT3                 0x08
#define BIT4                 0x10
#define BIT5                 0x20
#define BIT6                 0x40
#define BIT7                 0x80
#endif  // BIT0

// Port5
#ifdef  OLED_SOFT_SPI
#define MOSI       BIT1
#define UCLK       BIT3
#endif  // OLED_SOFT_SPI

#define OLED_RST             BIT4 	// Reset OLED display
#define OLED_CS2             BIT5 	// Chip Select for OLED
#define OLED_D_C             BIT6 	// Data/!Command
#define OLED_on              BIT7    // PowerOn OLED (0-On/1-Off)

//Port6
#define OLED_DC_on           BIT5    // PowerOn DC/DC Convertor (0-On,1-Off)

//Macro for OLED
/*
#define OLED_Data()          P5OUT |=  OLED_D_C
#define OLED_Command()       P5OUT &= ~OLED_D_C

#define OLED_SelectDSPL()    P5OUT &= ~OLED_CS2
#define OLED_DeselectDSPL()  P5OUT |=  OLED_CS2

#define OLED_DSPLon()        P5OUT &= ~OLED_on
#define OLED_DCon()          P6OUT &= ~OLED_DC_on
#define OLED_DSPLoff()       P5OUT |=  OLED_on
#define OLED_DCoff()         P6OUT |=  OLED_DC_on
*/

#define OLED_X_SIZE          256
#define OLED_Y_SIZE          64

#define OLED_setX(X) OLED_SendComm(0x1300 |(uint8_t)(X))
#define OLED_setY(Y) OLED_SendComm(0x1400 |(uint8_t)(Y))
#define OLED_cls()   OLED_clrLine(0, OLED_Y_SIZE)


struct OLED_bitmap
{
   unsigned short bmpWidth;
   unsigned short bmpHeight;
   unsigned short bmpSize;
   unsigned char BitmapData[0];
};

void OLED_pause(unsigned int D);
void OLED_reset(void);
void OLED_init(void);
//void OLED_setX(unsigned char X);
//void OLED_setY(unsigned char Y);
//void OLED_cls();
void OLED_clrLine(uint16_t from, uint16_t to);
void OLED_clr(uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize);
//void OLED_putBitmap(uint16_t x, uint16_t y, uint16_t size, uint16_t width, uint8_t height, unsigned char * bmp);
void OLED_putBitmap(uint16_t x, uint16_t y, struct OLED_bitmap * bmp);
void OLED_puts(uint16_t x, uint16_t y, uint8_t color, uint8_t font_num, unsigned char * str);

#endif  // _OLED_H_

