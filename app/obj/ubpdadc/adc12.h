#ifndef __msp430_headers_adc12_h
#define __msp430_headers_adc12_h

/* adc12.h
 *
 * mspgcc project: MSP430 device headers
 * ADC12 module header
 *
 * (c) 2002 by M. P. Ashton <data@ieee.org>
 * Originally based in part on work by Texas Instruments Inc.
 *
 * $Id: adc12.h,v 1.9 2004/06/15 13:50:15 coppice Exp $
 */

/* Switches: none */

#define ADC12CTL0_          0x01A0  /* ADC12 Control 0 */
sfrw(ADC12CTL0,ADC12CTL0_);
#define ADC12CTL1_          0x01A2  /* ADC12 Control 1 */
sfrw(ADC12CTL1,ADC12CTL1_);
#define ADC12IFG_           0x01A4  /* ADC12 Interrupt Flag */
sfrw(ADC12IFG,ADC12IFG_);
#define ADC12IE_            0x01A6  /* ADC12 Interrupt Enable */
sfrw(ADC12IE,ADC12IE_);
#define ADC12IV_            0x01A8  /* ADC12 Interrupt Vector Word */
sfrw(ADC12IV,ADC12IV_);

#ifndef _GNU_ASSEMBLER_
/* Structured declaration */
typedef struct {
  volatile unsigned
    adc12sc:1,
    enc:1,
    adc12tovie:1,
    adc12ovie:1,
    adc12on:1,
    refon:1,
    r2_5v:1,
    msc:1,
    sht0:4,
    sht1:4;
} __attribute__ ((packed)) adc12ctl0_t;

typedef struct {
  volatile unsigned
    adc12busy:1,
    conseq:2,
    adc12ssel:2,
    adc12div:3,
    issh:1,
    shp:1,
    shs:2,
    cstartadd:4;
} __attribute__ ((packed)) adc12ctl1_t;

typedef struct {
  volatile unsigned
    bit0:1,
    bit1:1,
    bit2:1,
    bit3:1,
    bit4:1,
    bit5:1,
    bit6:1,
    bit7:1,
    bit8:1,
    bit9:1,
    bit10:1,
    bit11:1,
    bit12:1,
    bit13:1,
    bit14:1,
    bit15:1;
} __attribute__ ((packed)) adc12xflg_t;

/* The adc12 declaration itself */
struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};

#ifdef __cplusplus
extern "C" struct adc12_t adc12 asm("0x01A0");
#else //__cplusplus
struct adc12_t adc12 asm("0x01A0");
#endif //__cplusplus

#endif

#define ADC12MEM_           0x0140  /* ADC12 Conversion Memory */
#ifdef _GNU_ASSEMBLER_
#define ADC12MEM            ADC12MEM_ /* ADC12 Conversion Memory (for assembler) */
#else
#define ADC12MEM            ((int*) ADC12MEM_) /* ADC12 Conversion Memory (for C) */
#endif
#define ADC12MEM0_          ADC12MEM_ /* ADC12 Conversion Memory 0 */
sfrw(ADC12MEM0,ADC12MEM0_);
#define ADC12MEM1_          0x0142  /* ADC12 Conversion Memory 1 */
sfrw(ADC12MEM1,ADC12MEM1_);
#define ADC12MEM2_          0x0144  /* ADC12 Conversion Memory 2 */
sfrw(ADC12MEM2,ADC12MEM2_);
#define ADC12MEM3_          0x0146  /* ADC12 Conversion Memory 3 */
sfrw(ADC12MEM3,ADC12MEM3_);
#define ADC12MEM4_          0x0148  /* ADC12 Conversion Memory 4 */
sfrw(ADC12MEM4,ADC12MEM4_);
#define ADC12MEM5_          0x014A  /* ADC12 Conversion Memory 5 */
sfrw(ADC12MEM5,ADC12MEM5_);
#define ADC12MEM6_          0x014C  /* ADC12 Conversion Memory 6 */
sfrw(ADC12MEM6,ADC12MEM6_);
#define ADC12MEM7_          0x014E  /* ADC12 Conversion Memory 7 */
sfrw(ADC12MEM7,ADC12MEM7_);
#define ADC12MEM8_          0x0150  /* ADC12 Conversion Memory 8 */
sfrw(ADC12MEM8,ADC12MEM8_);
#define ADC12MEM9_          0x0152  /* ADC12 Conversion Memory 9 */
sfrw(ADC12MEM9,ADC12MEM9_);
#define ADC12MEM10_         0x0154  /* ADC12 Conversion Memory 10 */
sfrw(ADC12MEM10,ADC12MEM10_);
#define ADC12MEM11_         0x0156  /* ADC12 Conversion Memory 11 */
sfrw(ADC12MEM11,ADC12MEM11_);
#define ADC12MEM12_         0x0158  /* ADC12 Conversion Memory 12 */
sfrw(ADC12MEM12,ADC12MEM12_);
#define ADC12MEM13_         0x015A  /* ADC12 Conversion Memory 13 */
sfrw(ADC12MEM13,ADC12MEM13_);
#define ADC12MEM14_         0x015C  /* ADC12 Conversion Memory 14 */
sfrw(ADC12MEM14,ADC12MEM14_);
#define ADC12MEM15_         0x015E  /* ADC12 Conversion Memory 15 */
sfrw(ADC12MEM15,ADC12MEM15_);

#define ADC12MCTL_          0x0080  /* ADC12 Memory Control */
#ifdef _GNU_ASSEMBLER_
#define ADC12MCTL           ADC12MCTL_ /* ADC12 Memory Control (for assembler) */
#else
#define ADC12MCTL           ((char*) ADC12MCTL_) /* ADC12 Memory Control (for C) */
#endif
#define ADC12MCTL0_         ADC12MCTL_ /* ADC12 Memory Control 0 */
sfrb(ADC12MCTL0,ADC12MCTL0_);
#define ADC12MCTL1_         0x0081  /* ADC12 Memory Control 1 */
sfrb(ADC12MCTL1,ADC12MCTL1_);
#define ADC12MCTL2_         0x0082  /* ADC12 Memory Control 2 */
sfrb(ADC12MCTL2,ADC12MCTL2_);
#define ADC12MCTL3_         0x0083  /* ADC12 Memory Control 3 */
sfrb(ADC12MCTL3,ADC12MCTL3_);
#define ADC12MCTL4_         0x0084  /* ADC12 Memory Control 4 */
sfrb(ADC12MCTL4,ADC12MCTL4_);
#define ADC12MCTL5_         0x0085  /* ADC12 Memory Control 5 */
sfrb(ADC12MCTL5,ADC12MCTL5_);
#define ADC12MCTL6_         0x0086  /* ADC12 Memory Control 6 */
sfrb(ADC12MCTL6,ADC12MCTL6_);
#define ADC12MCTL7_         0x0087  /* ADC12 Memory Control 7 */
sfrb(ADC12MCTL7,ADC12MCTL7_);
#define ADC12MCTL8_         0x0088  /* ADC12 Memory Control 8 */
sfrb(ADC12MCTL8,ADC12MCTL8_);
#define ADC12MCTL9_         0x0089  /* ADC12 Memory Control 9 */
sfrb(ADC12MCTL9,ADC12MCTL9_);
#define ADC12MCTL10_        0x008A  /* ADC12 Memory Control 10 */
sfrb(ADC12MCTL10,ADC12MCTL10_);
#define ADC12MCTL11_        0x008B  /* ADC12 Memory Control 11 */
sfrb(ADC12MCTL11,ADC12MCTL11_);
#define ADC12MCTL12_        0x008C  /* ADC12 Memory Control 12 */
sfrb(ADC12MCTL12,ADC12MCTL12_);
#define ADC12MCTL13_        0x008D  /* ADC12 Memory Control 13 */
sfrb(ADC12MCTL13,ADC12MCTL13_);
#define ADC12MCTL14_        0x008E  /* ADC12 Memory Control 14 */
sfrb(ADC12MCTL14,ADC12MCTL14_);
#define ADC12MCTL15_        0x008F  /* ADC12 Memory Control 15 */
sfrb(ADC12MCTL15,ADC12MCTL15_);

/* ADC12CTL0 */
#define ADC12SC             0x0001      /* ADC12 Start Conversion */
#define ENC                 0x0002      /* ADC12 Enable Conversion */
#define ADC12TOVIE          0x0004      /* ADC12 Timer Overflow interrupt enable */
#define ADC12OVIE           0x0008      /* ADC12 Overflow interrupt enable */
#define ADC12ON             0x0010      /* ADC12 On/enable */
#define REFON               0x0020      /* ADC12 Reference on */
#define REF2_5V             0x0040      /* ADC12 Ref 0:1.5V / 1:2.5V */ 
#define MSC                 0x0080      /* ADC12 Multiple Sample Conversion */
#define MSH                 0x0080
#define SHT00               0x0100      /* ADC12 Sample Hold 0 Select 0 */
#define SHT01               0x0200      /* ADC12 Sample Hold 0 Select 1 */
#define SHT02               0x0400      /* ADC12 Sample Hold 0 Select 2 */
#define SHT03               0x0800      /* ADC12 Sample Hold 0 Select 3 */
#define SHT10               0x1000      /* ADC12 Sample Hold 0 Select 0 */
#define SHT11               0x2000      /* ADC12 Sample Hold 1 Select 1 */
#define SHT12               0x4000      /* ADC12 Sample Hold 2 Select 2 */
#define SHT13               0x8000      /* ADC12 Sample Hold 3 Select 3 */

#define SHT0_0              (0<<8)
#define SHT0_1              (1<<8)
#define SHT0_2              (2<<8)
#define SHT0_3              (3<<8)
#define SHT0_4              (4<<8)
#define SHT0_5              (5<<8)
#define SHT0_6              (6<<8)
#define SHT0_7              (7<<8)
#define SHT0_8              (8<<8)
#define SHT0_9              (9<<8)
#define SHT0_10             (10<<8)
#define SHT0_11             (11<<8)
#define SHT0_12             (12<<8)
#define SHT0_13             (13<<8)
#define SHT0_14             (14<<8)
#define SHT0_15             (15<<8)

#define SHT1_0              (0<<12)
#define SHT1_1              (1<<12)
#define SHT1_2              (2<<12)
#define SHT1_3              (3<<12)
#define SHT1_4              (4<<12)
#define SHT1_5              (5<<12)
#define SHT1_6              (6<<12)
#define SHT1_7              (7<<12)
#define SHT1_8              (8<<12)
#define SHT1_9              (9<<12)
#define SHT1_10             (10<<12)
#define SHT1_11             (11<<12)
#define SHT1_12             (12<<12)
#define SHT1_13             (13<<12)
#define SHT1_14             (14<<12)
#define SHT1_15             (15<<12)

/* ADC12CTL1 */
#define ADC12BUSY           0x0001      /* ADC12 Busy */
#define CONSEQ0             0x0002      /* ADC12 Conversion Sequence Select 0 */
#define CONSEQ1             0x0004      /* ADC12 Conversion Sequence Select 1 */
#define ADC12SSEL0          0x0008      /* ADC12 Clock Source Select 0 */
#define ADC12SSEL1          0x0010      /* ADC12 Clock Source Select 1 */
#define ADC12DIV0           0x0020      /* ADC12 Clock Divider Select 0 */
#define ADC12DIV1           0x0040      /* ADC12 Clock Divider Select 1 */
#define ADC12DIV2           0x0080      /* ADC12 Clock Divider Select 2 */
#define ISSH                0x0100      /* ADC12 Invert Sample Hold Signal */
#define SHP                 0x0200      /* ADC12 Sample/Hold Pulse Mode */
#define SHS0                0x0400      /* ADC12 Sample/Hold Source 0 */
#define SHS1                0x0800      /* ADC12 Sample/Hold Source 1 */
#define CSTARTADD0          0x1000      /* ADC12 Conversion Start Address 0 */
#define CSTARTADD1          0x2000      /* ADC12 Conversion Start Address 1 */
#define CSTARTADD2          0x4000      /* ADC12 Conversion Start Address 2 */
#define CSTARTADD3          0x8000      /* ADC12 Conversion Start Address 3 */

#define CONSEQ_0            (0<<1)
#define CONSEQ_1            (1<<1)
#define CONSEQ_2            (2<<1)
#define CONSEQ_3            (3<<1)
#define ADC12SSEL_0         (0<<3)
#define ADC12SSEL_1         (1<<3)
#define ADC12SSEL_2         (2<<3)
#define ADC12SSEL_3         (3<<3)
#define ADC12DIV_0          (0<<5)
#define ADC12DIV_1          (1<<5)
#define ADC12DIV_2          (2<<5)
#define ADC12DIV_3          (3<<5)
#define ADC12DIV_4          (4<<5)
#define ADC12DIV_5          (5<<5)
#define ADC12DIV_6          (6<<5)
#define ADC12DIV_7          (7<<5)
#define SHS_0               (0<<10)
#define SHS_1               (1<<10)
#define SHS_2               (2<<10)
#define SHS_3               (3<<10)
#define CSTARTADD_0         (0<<12)
#define CSTARTADD_1         (1<<12)
#define CSTARTADD_2         (2<<12)
#define CSTARTADD_3         (3<<12)
#define CSTARTADD_4         (4<<12)
#define CSTARTADD_5         (5<<12)
#define CSTARTADD_6         (6<<12)
#define CSTARTADD_7         (7<<12)
#define CSTARTADD_8         (8<<12)
#define CSTARTADD_9         (9<<12)
#define CSTARTADD_10        (10<<12)
#define CSTARTADD_11        (11<<12)
#define CSTARTADD_12        (12<<12)
#define CSTARTADD_13        (13<<12)
#define CSTARTADD_14        (14<<12)
#define CSTARTADD_15        (15<<12)

/* ADC12MCTLx */
#define INCH_0               0
#define INCH_1               1
#define INCH_2               2
#define INCH_3               3
#define INCH_4               4
#define INCH_5               5
#define INCH_6               6
#define INCH_7               7
#define INCH_8               8
#define INCH_9               9
#define INCH_10             10
#define INCH_11             11
#define INCH_12             12
#define INCH_13             13
#define INCH_14             14
#define INCH_15             15

#define SREF_0              (0<<4)
#define SREF_1              (1<<4)
#define SREF_2              (2<<4)
#define SREF_3              (3<<4)
#define SREF_4              (4<<4)
#define SREF_5              (5<<4)
#define SREF_6              (6<<4)
#define SREF_7              (7<<4)
#define EOS                 0x80

#endif
