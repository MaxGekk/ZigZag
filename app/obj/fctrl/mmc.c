//
//
#ifndef _MMCLIB_C
#define _MMCLIB_C
//
//---------------------------------------------------------------
#include <zzPort.h>
//#include <msp430x16x.h>
#include "mmc.h"
//#include  "msp430x16x.h"

//#define withDMA

#define spiSendByte(data) SPI_tx(data)

//extern port_attr_t port5_attr;
port_attr_t mmc_port_attr;
port_attr_t mmc_pwr_port_attr;

// Function Prototypes
unsigned char mmcGetResponse(void);
unsigned char mmcGetXXResponse(const unsigned char resp);
unsigned char mmcCheckBusy(void);
//void initSPI (void);
unsigned char spiSendByte(const unsigned char data);
unsigned char mmc_GoIdle();

unsigned char mmc_error;

// Varialbes
static unsigned char mmc_buffer[512] = { 0 };               // Buffer for mmc i/o for data and registers

unsigned short mmc_buf_cpy(unsigned char * ptr, unsigned short len, unsigned short off)
{
   if((off + len) > 512)
      len = 512 - off;
   memcpy(mmc_buffer + off, ptr, len);
//   memcopy(mmc_buffer + off, ptr, len);
   return len;
}

unsigned char mmcWriteBuf(unsigned long sector)
{
   return mmcWriteBlock(sector << 9, 512, mmc_buffer);
}

unsigned char mmcReadBuf(unsigned long sector)
{
   return mmcReadBlock(sector << 9, 512, mmc_buffer);
}

unsigned char * mmcGetBuffer(void)
{
   return mmc_buffer;
}

//---------------------------------------------------------------------

// setup usart1 in spi mode
/*
void initSPI (void)
{
  UCTL1 = SWRST;                            // 8-bit SPI Master **SWRST**
  UTCTL1 = CKPH | SSEL1 | SSEL0 | STC;      // SMCLK, 3-pin mode, clock idle low, data valid on rising edge, UCLK delayed
  UBR01 = 0x02;                             // 0x02: UCLK/2 (4 MHz), works also with 3 and 4
  UBR11 = 0x00;                             // -"-
  UMCTL1 = 0x00;                            // no modulation
  UCTL1 = CHAR | SYNC | MM | SWRST;         // 8-bit SPI Master **SWRST**
  UCTL1 &= ~SWRST;                          // clear SWRST
  ME2 |= USPIE1;                            // Enable USART1 SPI mode
  while (!(IFG2 & UTXIFG1));                // USART1 TX buffer ready (empty)?
}
*/

unsigned char mmcPower(unsigned char cmd)
{
#ifndef PORT_WRITE
   switch(cmd)
   {
   case MMC_ON:
      MMC_PWR_POUT &= ~MMC_CARD_PWR;
      break;
   case MMC_OFF:
      MMC_PWR_POUT |=  MMC_CARD_PWR;
      break;
//   default:
//      return MMC_PWR_PORT;
   }
   return MMC_PWR_POUT;
#else  // !PORT_WRITE
   unsigned char pval;
   switch(cmd)
   {
   case MMC_ON:
      port_write(MMC_PWR_PORT, MMC_CARD_PWR, PIN_LO);
      break;
   case MMC_OFF:
      port_write(MMC_PWR_PORT, MMC_CARD_PWR, PIN_HI);
      break;
//   default:
//      return MMC_PWR_PORT;
   }
   port_read(MMC_PWR_PORT, MMC_CARD_PWR, &pval);
   return pval;
#endif // !PORT_WRITE
}



// Initialize MMC card
unsigned char initMMC (void)
{

  //raise SS and MOSI for 80 clock cycles
  //SendByte(0xff) 10 times with SS high
  //RAISE SS
  int i;

  // Port 5 Function           Dir       On/Off
  //         5.1-Dout          Out       0 - off    1 - On -> init in SPI_Init
  //         5.2-Din           Inp       0 - off    1 - On -> init in SPI_Init
  //         5.3-Clk           Out       -                 -> init in SPI_Init
  //         5.0-mmcCS         Out       0 - Active 1 - none Active
/*
  P5SEL |= 0x0E;
  P5SEL &= ~0x11;
  P5OUT |= 0x11;
  P5DIR |= 0x1B;
*/
  /*
  PIN_SET(port5_attr.sel, 0x0e, PIN_HI);
  PIN_CLEAR(port5_attr.sel, 0x01);
  PIN_SET(port5_attr.dir, 0x0B, PIN_HI);
  PIN_CLEAR(port5_attr.dir, 0x04);
  port_set_attr(5, 0x0f, &port5_attr);
  port_write(5, 0x01, PIN_HI);
  */
//  PIN_SET(port5_attr.sel, 0x0e, PIN_HI);
#ifndef PORT_WRITE
  mmcPower(MMC_ON);
  MMC_PWR_PDIR |=  MMC_CARD_PWR;
  MMC_PWR_PSEL &= ~MMC_CARD_PWR;
//  MMC_POUT |=  MMC_CS;
  CS_HIGH();
  MMC_PDIR |=  MMC_CS;
  MMC_PSEL &= ~MMC_CS;
#else  // !PORT_WRITE
  PIN_SET  (mmc_pwr_port_attr.dir, MMC_CARD_PWR, PIN_HI);
  PIN_CLEAR(mmc_pwr_port_attr.sel, MMC_CARD_PWR);
  port_set_attr(MMC_PWR_PORT, MMC_CARD_PWR, &mmc_pwr_port_attr);
  mmcPower(MMC_ON);
  PIN_CLEAR(mmc_port_attr.sel, MMC_CS);
  PIN_SET  (mmc_port_attr.dir, MMC_CS, PIN_HI);
  port_set_attr(MMC_PORT, MMC_CS, &mmc_port_attr);
  CS_HIGH();
  //port_write(5, 0x01, PIN_HI);
#endif // !PORT_WRITE

//  initSPI();
//  SPI_init();
  //initialization sequence on PowerUp
//  CS_HIGH();
  for(i=0;i<=9;i++)
    spiSendByte(0xff);

  mmc_error = MMC_SUCCESS;

  return (mmc_GoIdle());
}


unsigned char mmc_GoIdle()
{
  unsigned char response=0x01;
  CS_LOW();

  //Send Command 0 to put MMC in SPI mode
  mmcSendCmd(MMC_GO_IDLE_STATE,0,0x95);
  //Now wait for READY RESPONSE
  if((mmc_error = mmcGetResponse())!=0x01)
  {
    CS_HIGH();
    return MMC_INIT_ERROR;
  }

  while(response==0x01)
  {
    CS_HIGH();
    spiSendByte(0xff);
    CS_LOW();
    mmcSendCmd(MMC_SEND_OP_COND,0x00,0xff);
    response=mmcGetResponse();
  }
  CS_HIGH();
  spiSendByte(0xff);
  return MMC_SUCCESS;
}

// mmc Get Responce
unsigned char mmcGetResponse(void)
{
  //Response comes 1-8bytes after command
  //the first bit will be a 0
  //followed by an error code
  //data will be 0xff until response
  int i=0;

  unsigned char response;

  while(i<=64)
  {
    response=spiSendByte(0xff);
    if(response==0x00)break;
    if(response==0x01)break;
    i++;
  }
  return response;
}

unsigned char mmcGetXXResponse(const unsigned char resp)
{
  //Response comes 1-8bytes after command
  //the first bit will be a 0
  //followed by an error code
  //data will be 0xff until response
  volatile int i=0;

  unsigned char response;

//  while(i<=1000)
  for(i = 0xffff; i--; )
  {
    response=spiSendByte(0xff);
    if(response==resp)break;
//    i++;
  }
  return response;
}

unsigned char mmcCheckBusy(void)
{
  //Response comes 1-8bytes after command
  //the first bit will be a 0
  //followed by an error code
  //data will be 0xff until response
  int i=0;

  unsigned char response;
  unsigned char rvalue;
  while(i<=64)
  {
    response=spiSendByte(0xff);
    response &= 0x1f;
    switch(response)
    {
      case 0x05: rvalue=MMC_SUCCESS;break;
      case 0x0b: return(MMC_CRC_ERROR);
      case 0x0d: return(MMC_WRITE_ERROR);
      default:
        rvalue = MMC_OTHER_ERROR;
        break;
    }
    if(rvalue==MMC_SUCCESS)break;
    i++;
  }
  i=0;
  do
  {
    response=spiSendByte(0xff);
    i++;
  }while(response==0);
  return response;
}

// The card will respond with a standard response token followed by a data
// block suffixed with a 16 bit CRC.

unsigned char mmcReadBlock(const unsigned long address, const unsigned long count, unsigned char *pBuffer)
{
  unsigned long i = 0;
  unsigned char rvalue = MMC_RESPONSE_ERROR;

  // Set the block length to read
  if (mmcSetBlockLength (count) == MMC_SUCCESS)   // block length could be set
  {
    // SS = LOW (on)
    CS_LOW ();
    // send read command MMC_READ_SINGLE_BLOCK=CMD17
    mmcSendCmd (MMC_READ_SINGLE_BLOCK,address, 0xFF);
    // Send 8 Clock pulses of delay, check if the MMC acknowledged the read block command
    // it will do this by sending an affirmative response
    // in the R1 format (0x00 is no errors)
    if (mmcGetResponse() == 0x00)
    {
      // now look for the data token to signify the start of
      // the data
      mmc_error = mmcGetXXResponse(MMC_START_DATA_BLOCK_TOKEN);
      if (mmc_error == MMC_START_DATA_BLOCK_TOKEN)
//      if (mmcGetXXResponse(MMC_START_DATA_BLOCK_TOKEN) == MMC_START_DATA_BLOCK_TOKEN)
      {
#ifndef withDMA
        // clock the actual data transfer and receive the bytes; spi_read automatically finds the Data Block
        for (i = 0; i < count; i++)
          pBuffer[i] = spiSendByte(0xff);   // is executed with card inserted
#else
        U1IFG &= ~(URXIFG1 + URXIFG1);      /* clear flags */
        /* Get the block */
        /* DMA trigger is UART1 receive for both DMA0 and DMA1 */
        DMACTL0 &= ~(DMA0TSEL_15 | DMA1TSEL_15);
        DMACTL0 |= (DMA0TSEL_9 | DMA1TSEL_9);
        /* Source DMA address: receive register.  */
        DMA0SA = U1RXBUF_;
        /* Destination DMA address: the user data buffer. */
        DMA0DA = (unsigned short)pBuffer;
        /* The size of the block to be transferred */
        DMA0SZ = count;
        /* Configure the DMA transfer*/
        DMA0CTL =
          DMAIE   |                         /* Enable interrupt */
          DMADT_0 |                         /* Single transfer mode */
          DMASBDB |                         /* Byte mode */
          DMAEN |                           /* Enable DMA */
          DMADSTINCR1 | DMADSTINCR0;        /* Increment the destination address */

        /* We depend on the DMA priorities here.  Both triggers occur at
           the same time, since the source is identical.  DMA0 is handled
           first, and retrieves the byte.  DMA1 is triggered next, and
           sends the next byte. */
        /* Source DMA address: constant 0xFF (don't increment)*/
        DMA1SA = U1TXBUF_;
        /* Destination DMA address: the transmit buffer. */
        DMA1DA = U1TXBUF_;
        /* Increment the destination address */
        /* The size of the block to be transferred */
        DMA1SZ = count-1;
        /* Configure the DMA transfer*/
        DMA1CTL =
          DMADT_0 |                         /* Single transfer mode */
          DMASBDB |                         /* Byte mode */
          DMAEN;                            /* Enable DMA */

        /* Kick off the transfer by sending the first byte */
        U1TXBUF = 0xFF;
//      while (DMA0CTL & DMAEN) _NOP(); //LPM0;  // wait till done
//      while (DMA0CTL & DMAEN) _EINT(); LPM0;  // wait till done
        _EINT(); LPM0;  // wait till done
#endif
        // get CRC bytes (not really needed by us, but required by MMC)
        spiSendByte(0xff);
        spiSendByte(0xff);
        rvalue = MMC_SUCCESS;
      }
      else
      {
        // the data token was never received
        rvalue = MMC_DATA_TOKEN_ERROR;      // 3
      }
    }
    else
    {
      // the MMC never acknowledge the read command
      rvalue = MMC_RESPONSE_ERROR;          // 2
    }
  }
  else
  {
    rvalue = MMC_BLOCK_SET_ERROR;           // 1
  }
  CS_HIGH ();
  spiSendByte(0xff);
  return rvalue;
}// mmc_read_block



//---------------------------------------------------------------------
//char mmcWriteBlock (const unsigned long address)
unsigned char mmcWriteBlock (const unsigned long address, const unsigned long count, unsigned char *pBuffer)
{
  unsigned long i = 0;
  unsigned char rvalue = MMC_RESPONSE_ERROR;         // MMC_SUCCESS;
  //  char c = 0x00;

  // Set the block length to read
  if (mmcSetBlockLength (count) == MMC_SUCCESS)   // block length could be set
  {
    // SS = LOW (on)
    CS_LOW ();
    // send write command
    mmcSendCmd (MMC_WRITE_BLOCK,address, 0xFF);

    // check if the MMC acknowledged the write block command
    // it will do this by sending an affirmative response
    // in the R1 format (0x00 is no errors)
    if (mmcGetXXResponse(MMC_R1_RESPONSE) == MMC_R1_RESPONSE)
    {
      spiSendByte(0xff);
      // send the data token to signify the start of the data
      spiSendByte(0xfe);
      // clock the actual data transfer and transmitt the bytes
#ifndef withDMA
      for (i = 0; i < count; i++)
        spiSendByte(pBuffer[i]);            
#else
      /* Get the block */
      /* DMA trigger is UART send */
      DMACTL0 &= ~(DMA0TSEL_15);
      DMACTL0 |= (DMA0TSEL_9);
      /* Source DMA address: the data buffer.  */
      DMA0SA = (unsigned short)pBuffer;
      /* Destination DMA address: the UART send register. */
      DMA0DA = U1TXBUF_;
      /* The size of the block to be transferred */
      DMA0SZ = count;
      /* Configure the DMA transfer*/
      DMA0CTL =
        DMAREQ  |                           /* start transfer */
        DMADT_0 |                           /* Single transfer mode */
        DMASBDB |                           /* Byte mode */
        DMAEN |                             /* Enable DMA */
        DMASRCINCR1 | DMASRCINCR0;          /* Increment the source address */
#endif
      // put CRC bytes (not really needed by us, but required by MMC)
      spiSendByte(0xff);
      spiSendByte(0xff);
      // read the data response xxx0<status>1 : status 010: Data accected, status 101: Data
      //   rejected due to a crc error, status 110: Data rejected due to a Write error.
      mmcCheckBusy();
      rvalue = MMC_SUCCESS;
    }
    else
    {
      // the MMC never acknowledge the write command
      rvalue = MMC_RESPONSE_ERROR;   // 2
    }
  }
  else
  {
    rvalue = MMC_BLOCK_SET_ERROR;   // 1
  }
  // give the MMC the required clocks to finish up what ever it needs to do
  //  for (i = 0; i < 9; ++i)
  //    spiSendByte(0xff);

  CS_HIGH ();
  // Send 8 Clock pulses of delay.
  spiSendByte(0xff);
  return rvalue;
} // mmc_write_block


//---------------------------------------------------------------------
void mmcSendCmd (const unsigned char cmd, unsigned long data, const unsigned char crc)
{
  unsigned char frame[6];
  unsigned char temp;
  int i;
  frame[0]=(cmd|0x40);
  for(i=3;i>=0;i--){
    temp=(char)(data>>(8*i));
    frame[4-i]=(temp);
  }
  frame[5]=(crc);
  for(i=0;i<6;i++)
    spiSendByte(frame[i]);
}


//--------------- set blocklength 2^n ------------------------------------------------------
unsigned char mmcSetBlockLength (const unsigned long blocklength)
{
  //  char rValue = MMC_TIMEOUT_ERROR;
  //  char i = 0;
  // SS = LOW (on)
  CS_LOW ();
  // Set the block length to read
  //MMC_SET_BLOCKLEN =CMD16
  mmcSendCmd(MMC_SET_BLOCKLEN, blocklength, 0xFF);

  // get response from MMC - make sure that its 0x00 (R1 ok response format)
  if(mmcGetResponse()!=0x00)
  { initMMC();
    mmcSendCmd(MMC_SET_BLOCKLEN, blocklength, 0xFF);
    mmcGetResponse();
  }

  CS_HIGH ();

  // Send 8 Clock pulses of delay.
  spiSendByte(0xff);

  return MMC_SUCCESS;
} // Set block_length

/*
unsigned char spiSendByte(const unsigned char data)
{
  while ((IFG2&UTXIFG1) ==0);   // wait while not ready / for RX
  TXBUF1 = data;         // write
  while ((IFG2 & URXIFG1)==0);   // wait for RX buffer (full)
  return (RXBUF1);
}
*/


// Reading the contents of the CSD and CID registers in SPI mode is a simple
// read-block transaction.
unsigned char mmcReadRegister (const char cmd_register, const unsigned char length, unsigned char *pBuffer)
{
  unsigned char uc = 0;
  unsigned char rvalue = MMC_TIMEOUT_ERROR;

  if (mmcSetBlockLength (length) == MMC_SUCCESS)
  {
    CS_LOW ();
    // CRC not used: 0xff as last byte
    mmcSendCmd(cmd_register, 0x000000, 0xff);

    // wait for response
    // in the R1 format (0x00 is no errors)
    if (mmcGetResponse() == 0x00)
    {
      if (mmcGetXXResponse(0xfe)== 0xfe)
        for (uc = 0; uc < length; uc++)
          pBuffer[uc] = spiSendByte(0xff);  //mmc_buffer[uc] = spiSendByte(0xff);
      // get CRC bytes (not really needed by us, but required by MMC)
      spiSendByte(0xff);
      spiSendByte(0xff);
      rvalue = MMC_SUCCESS;
    }
    else
      rvalue = MMC_RESPONSE_ERROR;
    // CS = HIGH (off)
    CS_HIGH ();

    // Send 8 Clock pulses of delay.
    spiSendByte(0xff);
  }
  CS_HIGH ();
  return rvalue;
} // mmc_read_register

/*
uint64_t mul_32x16(unsigned long v1, unsigned short v2, unsigned char dbg)
{
   unsigned char a1, b1, c1, d1;
   unsigned char a2, b2;
   unsigned long ul1, ul2, ul3;
   uint64_t res, tmp64;

   a1 = (unsigned char)v1;
   b1 = (unsigned char)(((unsigned short)v1) >> 8);
   c1 = (unsigned char)(v1 >> 16);
   d1 = (unsigned char)(v1 >> 24);

   a2 = (unsigned char)v2;
   b2 = (unsigned char)(v2 >> 8);

   ul1 = (unsigned long)((unsigned short)a2*b1) + (unsigned short)b2*a1;
   ul2 = (unsigned long)((unsigned short)a2*c1) + (unsigned short)b2*b1;
   ul3 = (unsigned long)((unsigned short)a2*d1) + (unsigned short)b2*c1;
   v2  = (unsigned short)a2*a1;
//   res = (uint64_t)((unsigned short)b2*d1) << 32 + (uint64_t)ul3 << 24 + (uint64_t)ul2 << 16 + ul1 << 8 + v2;
   res  = (uint64_t)((unsigned short)b2*d1) << 32;
//   res += (uint64_t)ul3 << 24;
   tmp64 = ul3; // Bug fix (problem with 64-bit left shift by 24 bits)
   tmp64 <<= 16;
   res += tmp64 << 8;
   
   res += (uint64_t)ul2 << 16;
   res += (uint64_t)ul1 << 8;
   res += (uint64_t)v2;

   return res;
}
*/


unsigned long MMC_ReadCardSize(void)
{
  // Read contents of Card Specific Data (CSD)

  unsigned long MMC_CardSize;
  unsigned short i,      // index
                 j,      // index
                 b,      // temporary variable
                 response,   // MMC response to command
                 mmc_C_SIZE;

  unsigned char mmc_READ_BL_LEN,  // Read block length
                mmc_C_SIZE_MULT;

  CS_LOW ();

  spiSendByte(MMC_READ_CSD);   // CMD 9
  for(i=4; i>0; i--)      // Send four dummy bytes
    spiSendByte(0);
  spiSendByte(0xFF);   // Send CRC byte

  response = mmcGetResponse();

  // data transmission always starts with 0xFE
  b = spiSendByte(0xFF);

  if( !response )
  {
    while (b != 0xFE) b = spiSendByte(0xFF);
    // bits 127:87
    for(j=5; j>0; j--)          // Host must keep the clock running for at
      b = spiSendByte(0xff);


    // 4 bits of READ_BL_LEN
    // bits 84:80
    b =spiSendByte(0xff);  // lower 4 bits of CCC and
    mmc_READ_BL_LEN = b & 0x0F;

    b = spiSendByte(0xff);

    // bits 73:62  C_Size
    // xxCC CCCC CCCC CC
    mmc_C_SIZE = (b & 0x03) << 10;
    b = spiSendByte(0xff);
    mmc_C_SIZE += b << 2;
    b = spiSendByte(0xff);
    mmc_C_SIZE += b >> 6;

    // bits 55:53
    b = spiSendByte(0xff);

    // bits 49:47
    mmc_C_SIZE_MULT = (b & 0x03) << 1;
    b = spiSendByte(0xff);
    mmc_C_SIZE_MULT += b >> 7;

    // bits 41:37
    b = spiSendByte(0xff);

    b = spiSendByte(0xff);

    b = spiSendByte(0xff);

    b = spiSendByte(0xff);

    b = spiSendByte(0xff);

  }

  for(j=4; j>0; j--)          // Host must keep the clock running for at
    b = spiSendByte(0xff);  // least Ncr (max = 4 bytes) cycles after
                               // the card response is received
  b = spiSendByte(0xff);
//  CS_LOW ();
  CS_HIGH ();

  MMC_CardSize = (mmc_C_SIZE + 1);
  // power function with base 2 is better with a loop
  // i = (pow(2,mmc_C_SIZE_MULT+2)+0.5);
  for(i = 2,j=mmc_C_SIZE_MULT+2; j>1; j--)
    i <<= 1;
  MMC_CardSize *= i;
//  MMC_CardSize = mul_32x16(MMC_CardSize, i);//, 0);
  // power function with base 2 is better with a loop
  //i = (pow(2,mmc_READ_BL_LEN)+0.5);
  for(i = 2,j=mmc_READ_BL_LEN; j>1; j--)
    i <<= 1;
  MMC_CardSize *= i;
//  MMC_CardSize = mul_32x16(MMC_CardSize, i);//, 1);
  
  return (MMC_CardSize);

}

/*
unsigned char mmc_ping(void)
{
  if (!(P5IN & 0x01))
    return (MMC_SUCCESS);
  else
    return (MMC_INIT_ERROR);
}
*/

#ifdef withDMA
#ifdef __IAR_SYSTEMS_ICC__
#if __VER__ < 200
interrupt[DACDMA_VECTOR] void DMA_isr(void)
#else
#pragma vector = DACDMA_VECTOR
__interrupt void DMA_isr(void)
#endif
#endif

#ifdef __CROSSWORKS__
void DMA_isr(void)   __interrupt[DACDMA_VECTOR]
#endif

#ifdef __TI_COMPILER_VERSION__
__interrupt void DMA_isr(void);
DMA_ISR(DMA_isr)
__interrupt void DMA_isr(void)
#endif
{
  DMA0CTL &= ~(DMAIFG);
  LPM3_EXIT;
}
#endif


//---------------------------------------------------------------------
#endif /* _MMCLIB_C */
