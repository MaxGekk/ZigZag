// *************************************************************************************
//
// Filename:  mmc.h: 
// Declarations for Communication with the MMC (see mmc.c) in unprotected SPI mode.
//
// Version 1.1
//    added ul declaration in macros mmcWriteSector and mmcReadSector
// *************************************************************************************

#ifndef _MMCLIB_H
#define _MMCLIB_H


// macro defines
#define HIGH(a) ((a>>8)&0xFF)               // high byte from word
#define LOW(a)  (a&0xFF)                    // low byte from word

#define PORT_WRITE


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


#define MMC_CS        BIT0

#define VARIANT_0_2
#ifdef VARIANT_0_2
#define MMC_CARD_PWR  BIT4
#else
#define MMC_CARD_PWR  BIT2 // Second test hardware variant
#endif

#define MMC_OFF       0x00
#define MMC_ON        0x01
#define MMC_PWR_STAT  0xff

#ifndef PORT_WRITE

#define MMC_POUT      P3OUT
#define MMC_PIN       P3IN
#define MMC_PSEL      P3SEL
#define MMC_PDIR      P3DIR
#define MMC_PWR_POUT  P4OUT
#define MMC_PWR_PSEL  P4SEL
#define MMC_PWR_PDIR  P4DIR

#define CS_LOW()      MMC_POUT &= ~MMC_CS              // Card Select
#define CS_HIGH()     MMC_POUT |=  MMC_CS  // Card Deselect

#else  // !PORT_WRITE

#define MMC_PORT      3
#define MMC_PWR_PORT  4

#define CS_LOW()      port_write(MMC_PORT, MMC_CS, PIN_LO)   // Card Select
#define CS_HIGH()     port_write(MMC_PORT, MMC_CS, PIN_HI)  // Card Deselect

#endif // !PORT_WRITE

#define SPI_RXC       (IFG2 & URXIFG1)
#define SPI_TXC       (IFG2 & UTXIFG1)

#define SPI_RX_COMPLETE (IFG2 & URXIFG1)
#define SPI_TX_READY    (IFG2 & UTXIFG1)
#define SPI_TX_DONE     while((U1TCTL & TXEPT)==0)

#define DUMMY 0xff


// Tokens (necessary  because at NPO/IDLE (and CS active) only 0xff is on the data/command line)
#define MMC_START_DATA_BLOCK_TOKEN          0xfe   // Data token start byte, Start Single Block Read
#define MMC_START_DATA_MULTIPLE_BLOCK_READ  0xfe   // Data token start byte, Start Multiple Block Read
#define MMC_START_DATA_BLOCK_WRITE          0xfe   // Data token start byte, Start Single Block Write
#define MMC_START_DATA_MULTIPLE_BLOCK_WRITE 0xfc   // Data token start byte, Start Multiple Block Write
#define MMC_STOP_DATA_MULTIPLE_BLOCK_WRITE  0xfd   // Data toke stop byte, Stop Multiple Block Write


// an affirmative R1 response (no errors)
#define MMC_R1_RESPONSE       0x00


// this variable will be used to track the current block length
// this allows the block length to be set only when needed
// unsigned long _BlockLength = 0;

// error/success codes
#define MMC_SUCCESS           0x00
#define MMC_BLOCK_SET_ERROR   0x01
#define MMC_RESPONSE_ERROR    0x02
#define MMC_DATA_TOKEN_ERROR  0x03
#define MMC_INIT_ERROR        0x04
#define MMC_CRC_ERROR         0x10
#define MMC_WRITE_ERROR       0x11
#define MMC_OTHER_ERROR       0x12
#define MMC_TIMEOUT_ERROR     0xFF


// commands: first bit 0 (start bit), second 1 (transmission bit); CMD-number + 0ffsett 0x40
#define MMC_GO_IDLE_STATE          0x40     //CMD0
#define MMC_SEND_OP_COND           0x41     //CMD1
#define MMC_READ_CSD               0x49     //CMD9
#define MMC_SEND_CID               0x4a     //CMD10
#define MMC_STOP_TRANSMISSION      0x4c     //CMD12
#define MMC_SEND_STATUS            0x4d     //CMD13
#define MMC_SET_BLOCKLEN           0x50     //CMD16 Set block length for next read/write
#define MMC_READ_SINGLE_BLOCK      0x51     //CMD17 Read block from memory
#define MMC_READ_MULTIPLE_BLOCK    0x52     //CMD18
#define MMC_CMD_WRITEBLOCK         0x54     //CMD20 Write block to memory
#define MMC_WRITE_BLOCK            0x58     //CMD24
#define MMC_WRITE_MULTIPLE_BLOCK   0x59     //CMD25
#define MMC_WRITE_CSD              0x5b     //CMD27 PROGRAM_CSD
#define MMC_SET_WRITE_PROT         0x5c     //CMD28
#define MMC_CLR_WRITE_PROT         0x5d     //CMD29
#define MMC_SEND_WRITE_PROT        0x5e     //CMD30
#define MMC_TAG_SECTOR_START       0x60     //CMD32
#define MMC_TAG_SECTOR_END         0x61     //CMD33
#define MMC_UNTAG_SECTOR           0x62     //CMD34
#define MMC_TAG_EREASE_GROUP_START 0x63     //CMD35
#define MMC_TAG_EREASE_GROUP_END   0x64     //CMD36
#define MMC_UNTAG_EREASE_GROUP     0x65     //CMD37
#define MMC_EREASE                 0x66     //CMD38
#define MMC_READ_OCR               0x67     //CMD39
#define MMC_CRC_ON_OFF             0x68     //CMD40


extern unsigned char mmc_error;

unsigned char mmcPower(unsigned char cmd);
// mmc init
unsigned char initMMC (void);

// check if MMC card is present
unsigned char mmc_ping(void);

// send command to MMC
void mmcSendCmd (const unsigned char cmd, unsigned long data, const unsigned char crc);

// set MMC in Idle mode
unsigned char mmc_GoIdle();

// set MMC block length of count=2^n Byte
unsigned char mmcSetBlockLength (const unsigned long);

unsigned short mmc_buf_cpy(unsigned char * ptr, unsigned short len, unsigned short off);
// read a size Byte big block beginning at the address.
unsigned char mmcReadBlock(const unsigned long address, const unsigned long count, unsigned char *pBuffer);
#define mmcReadSector(sector, pBuffer) mmcReadBlock(sector*512ul, 512, pBuffer)

// write a 512 Byte big block beginning at the (aligned) address
unsigned char mmcWriteBlock (const unsigned long address, const unsigned long count, unsigned char *pBuffer);
#define mmcWriteSector(sector, pBuffer) mmcWriteBlock(sector*512ul, 512, pBuffer)

unsigned char mmcWriteBuf(unsigned long sector); 
unsigned char mmcReadBuf(unsigned long sector);
unsigned char * mmcGetBuffer(void);

// Read Register arg1 with Length arg2 (into the buffer)
unsigned char mmcReadRegister(const char, const unsigned char, unsigned char *pBuffer);

// Read the Card Size from the CSD Register
unsigned long MMC_ReadCardSize(void);


#endif /* _MMCLIB_H */
