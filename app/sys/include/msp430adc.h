#ifndef     _MSP430ADC_H
#define     _MSP430ADC_H
/*! @file   msp430adc.h
 *  @brief  Регистры и константы АЦП12
 *  @author Max Gekk
 *  @date   апрель 2007
 *  @version 1
 */   

#include    <inttypes.h>

/* Управляющий регистр 0 АЦП12 */
volatile    uint16_t    ADC12CTL0   asm("0x01A0");
/* Управляющий регистр 1 АЦП12 */
volatile    uint16_t    ADC12CTL1   asm("0x01A2");

/* Регистр флагов прерываний АЦП12 */
volatile    uint16_t    ADC12IFG    asm("0x01A4");
/* Регистр разрешения прерываний АЦП12 */
volatile    uint16_t    ADC12IE     asm("0x01A6");
/* Слово вектора прерываний АЦП12 */
volatile    uint16_t    ADC12IV     asm("0x01A8");

/* Число регистров памяти АЦП12 */
#define     ADC12MEM_TOTAL  16
/* Регистр памяти 0 АЦП12 */
volatile    uint16_t    ADC12MEM0   asm("0x0140");
/* Регистр памяти 1 АЦП12 */
volatile    uint16_t    ADC12MEM1   asm("0x0142");
/* Регистр памяти 2 АЦП12 */
volatile    uint16_t    ADC12MEM2   asm("0x0144");
/* Регистр памяти 3 АЦП12 */
volatile    uint16_t    ADC12MEM3   asm("0x0146");
/* Регистр памяти 4 АЦП12 */
volatile    uint16_t    ADC12MEM4   asm("0x0148");
/* Регистр памяти 5 АЦП12 */
volatile    uint16_t    ADC12MEM5   asm("0x014A");
/* Регистр памяти 6 АЦП12 */
volatile    uint16_t    ADC12MEM6   asm("0x014C");
/* Регистр памяти 7 АЦП12 */
volatile    uint16_t    ADC12MEM7   asm("0x014E");
/* Регистр памяти 8 АЦП12 */
volatile    uint16_t    ADC12MEM8   asm("0x0150");
/* Регистр памяти 9 АЦП12 */
volatile    uint16_t    ADC12MEM9   asm("0x0152");
/* Регистр памяти 10 АЦП12 */
volatile    uint16_t    ADC12MEM10   asm("0x0154");
/* Регистр памяти 11 АЦП12 */
volatile    uint16_t    ADC12MEM11   asm("0x0156");
/* Регистр памяти 12 АЦП12 */
volatile    uint16_t    ADC12MEM12   asm("0x0158");
/* Регистр памяти 13 АЦП12 */
volatile    uint16_t    ADC12MEM13   asm("0x015A");
/* Регистр памяти 14 АЦП12 */
volatile    uint16_t    ADC12MEM14   asm("0x015C");
/* Регистр памяти 15 АЦП12 */
volatile    uint16_t    ADC12MEM15   asm("0x015E");
/* Массив регистров памяти */
volatile    uint16_t    adc12mem[ADC12MEM_TOTAL]    asm("0x140"); 

/* Управление регистром памяти 0 АЦП12 */
volatile    uint8_t     ADC12MCTL0  asm("0x0080");
/* Управление регистром памяти 1 АЦП12 */
volatile    uint8_t     ADC12MCTL1  asm("0x0081");
/* Управление регистром памяти 2 АЦП12 */
volatile    uint8_t     ADC12MCTL2  asm("0x0082");
/* Управление регистром памяти 3 АЦП12 */
volatile    uint8_t     ADC12MCTL3  asm("0x0083");
/* Управление регистром памяти 4 АЦП12 */
volatile    uint8_t     ADC12MCTL4  asm("0x0084");
/* Управление регистром памяти 5 АЦП12 */
volatile    uint8_t     ADC12MCTL5  asm("0x0085");
/* Управление регистром памяти 6 АЦП12 */
volatile    uint8_t     ADC12MCTL6  asm("0x0086");
/* Управление регистром памяти 7 АЦП12 */
volatile    uint8_t     ADC12MCTL7  asm("0x0087");
/* Управление регистром памяти 8 АЦП12 */
volatile    uint8_t     ADC12MCTL8  asm("0x0088");
/* Управление регистром памяти 9 АЦП12 */
volatile    uint8_t     ADC12MCTL9  asm("0x0089");
/* Управление регистром памяти 10 АЦП12 */
volatile    uint8_t     ADC12MCTL10  asm("0x008A");
/* Управление регистром памяти 11 АЦП12 */
volatile    uint8_t     ADC12MCTL11  asm("0x008B");
/* Управление регистром памяти 12 АЦП12 */
volatile    uint8_t     ADC12MCTL12  asm("0x008C");
/* Управление регистром памяти 13 АЦП12 */
volatile    uint8_t     ADC12MCTL13  asm("0x008D");
/* Управление регистром памяти 14 АЦП12 */
volatile    uint8_t     ADC12MCTL14  asm("0x008E");
/* Управление регистром памяти 15 АЦП12 */
volatile    uint8_t     ADC12MCTL15  asm("0x008F");
/* Массив управляющих регистров */
volatile    uint8_t     adc12mctl[ADC12MEM_TOTAL]   asm("0x0080");

/* Константы регистра ADC12CTL0 */
#define     ADC12SC     0x0001      /* Программно управляемый старт выборки-преобразования */
#define     ENC         0x0002      /* Разрешение преобразования */
#define     ADC12TOIE   0x0004      /* Разрешение прерывания по превышению времени преобразования АЦП12 */
#define     ADC12OVIE   0x0008      /* Разрешение прерывания по переполнению ADC12MEMx */
#define     ADC12ON     0x0010      /* Включение АЦП12. Модифицируется, только если ENC=0 */
#define     REFON       0x0020      /* Включение опорного генератора. Только если ENC=0 */
#define     REF2_5V     0x0040      /* Генератор опорного напряжения. 0 - 1.5 В, 1 - 2.5 В. Изм. только если ENC=0 */
/* Множественная выборка и преобразование ( для последовательных или повторных режимов ). 
 * 0 - для запуска каждой выборки-преобразования на таймер выборки подаётся фронт сигнала SHI
 * 1 - первый фронт сигнала SHI запускает таймер выборки, последующие выборки-преобразования выполняются 
 *     автоматически, сразу же после завершения предыдущего преобразования */
#define     MSC         0x0080      /* Модифицируется, только если ENC = 0 */
/* Время выборки-хранения. Время задаётся в виде констант SHTy_x, где x - число циклов ADC12CLK в периоде выборки.
 * Модифицируется, только когда ENC = 0. */
/* Для регистров с ADC12MEM0 по ADC12MEM7. */
#define     SHT0_4      0x0000
#define     SHT0_8      0x0100
#define     SHT0_16     0x0200
#define     SHT0_32     0x0300
#define     SHT0_64     0x0400
#define     SHT0_96     0x0500
#define     SHT0_128    0x0600
#define     SHT0_192    0x0700
#define     SHT0_256    0x0800
#define     SHT0_384    0x0900
#define     SHT0_512    0x0A00
#define     SHT0_768    0x0B00
#define     SHT0_1024   0x0C00
/* Для регистров с ADC12MEM8 по ADC12MEM15. */
#define     SHT1_4      0x0000
#define     SHT1_8      0x1000
#define     SHT1_16     0x2000
#define     SHT1_32     0x3000
#define     SHT1_64     0x4000
#define     SHT1_96     0x5000
#define     SHT1_128    0x6000
#define     SHT1_192    0x7000
#define     SHT1_256    0x8000
#define     SHT1_384    0x9000
#define     SHT1_512    0xA000
#define     SHT1_768    0xB000
#define     SHT1_1024   0xC000

/* Константы регистра ADC12CTL1 */
#define     ADC12BUSY   0x0001      /* Занятость АЦП12. 0 - действия не выполняются */
/* Выбор режима преобразования. */
#define     CONSEQ_xx   0x0006
#define     CONSEQ_00   0x0000      /* Одноканальный, с одним преобразованием */
#define     CONSEQ_01   0x0002      /* Последовательность каналов */
#define     CONSEQ_10   0x0004      /* Повторный одноканальный */
#define     CONSEQ_11   0x0006      /* Повторяющаяся последовательность каналов */
/* Выбор источника тактирования для АЦП 12. Модифицируется, только когда ENC = 0. */
#define     ADC12SSEL_xx    0x0018
#define     ADC12SSEL_00    0x0000  /* ADC12OSC */
#define     ADC12SSEL_01    0x0008  /* ACLK */
#define     ADC12SSEL_10    0x0010  /* MCLK */
#define     ADC12SSEL_11    0x0018  /* SMCLK */
/* Тактовый делитель. Задаётся константами вида ADC12DIV_x, где x - делитель. 
 * Модифицируется, только когда ENC = 0.*/
#define     ADC12DIV_x      0x00e0
#define     ADC12DIV_1      0x0000  /* /1 */
#define     ADC12DIV_2      0x0020  /* /2 */
#define     ADC12DIV_3      0x0040  /* /3 */
#define     ADC12DIV_4      0x0060  /* /4 */
#define     ADC12DIV_5      0x0080  /* /5 */
#define     ADC12DIV_6      0x00a0  /* /6 */
#define     ADC12DIV_7      0x00c0  /* /7 */
#define     ADC12DIV_8      0x00e0  /* /8 */

#define     ISSH            0x0100  /* Инвертирование сигнала выборки-хранения. 1 - инверсия. Модифицируется, только когда ENC = 0. */
#define     SHP             0x0200  /* Выбор импульсного режима выборки. Модифицируется, только когда ENC = 0. */
/* Выбор источника выборки-хранения. Модифицируется, только когда ENC = 0. */
#define     SHS_xx          0x0c00
#define     SHS_00          0x0000  /* Бит ADC12SC */
#define     SHS_01          0x0400  /* Выход 1 Таймера А */
#define     SHS_10          0x0800  /* Выход 0 Таймера В */
#define     SHS_11          0x0c00  /* Выход 1 Таймера В */
/* Стартовый номер ADC12MEMx для преобразования. Модифицируется, только когда ENC = 0. */
#define     CSTARTADDR_0    0x0000  // ADC12MEM0
#define     CSTARTADDR_1    0x1000  // ADC12MEM1
#define     CSTARTADDR_2    0x2000  // ADC12MEM2
#define     CSTARTADDR_3    0x3000  // ADC12MEM3
#define     CSTARTADDR_4    0x4000  // ADC12MEM4
#define     CSTARTADDR_5    0x5000  // ADC12MEM5
#define     CSTARTADDR_6    0x6000  // ADC12MEM6
#define     CSTARTADDR_7    0x7000  // ADC12MEM7
#define     CSTARTADDR_8    0x8000  // ADC12MEM8
#define     CSTARTADDR_9    0x9000  // ADC12MEM9
#define     CSTARTADDR_10   0xA000  // ADC12MEMA
#define     CSTARTADDR_11   0xB000  // ADC12MEMB
#define     CSTARTADDR_12   0xC000  // ADC12MEMC
#define     CSTARTADDR_13   0xD000  // ADC12MEMD
#define     CSTARTADDR_14   0xE000  // ADC12MEME
#define     CSTARTADDR_15   0xF000  // ADC12MEMF

/* Константы управляющих регистров памяти */
/* Выбор входного канала.  */
#define     INCH_x          0x0F
#define     INCH_0000       0x00    /* A0 */
#define     INCH_0001       0x01    /* A1 */
#define     INCH_0010       0x02    /* A2 */
#define     INCH_0011       0x03    /* A3 */
#define     INCH_0100       0x04    /* A4 */
#define     INCH_0101       0x05    /* A5 */
#define     INCH_0110       0x06    /* A6 */
#define     INCH_0111       0x07    /* A7 */
#define     INCH_1000       0x08
#define     INCH_1001       0x09    /* */
#define     INCH_1010       0x0A    /* */
#define     INCH_1011       0x0B    /* */
#define     INCH_1100       0x0C    /* */
#define     INCH_1101       0x0D    /* */
#define     INCH_1110       0x0E    /* */
#define     INCH_1111       0x0F    /* */
/* Выбор опорного источника. Модифицируется, только когда ENC = 0. */
#define     SREF_x          0x70
#define     SREF_000        0x00
#define     SREF_001        0x10
#define     SREF_010        0x20
#define     SREF_011        0x30
#define     SREF_100        0x40
#define     SREF_101        0x50
#define     SREF_110        0x60
#define     SREF_111        0x70
#define     EOS             0x80    /* Конец последовательности. Модифицируется, только когда ENC = 0. */

/* Константы регистра ADC12IE. Разрешение прерывания. Эти биты разрешают или запрещают запрос
 * прерывания для битов ADC12IFGx. */
#define     ADC12IE0        (1<<0)
#define     ADC12IE1        (1<<1)
#define     ADC12IE2        (1<<2)
#define     ADC12IE3        (1<<3)
#define     ADC12IE4        (1<<4)
#define     ADC12IE5        (1<<5)
#define     ADC12IE6        (1<<6)
#define     ADC12IE7        (1<<7)
#define     ADC12IE8        (1<<8)
#define     ADC12IE9         (1<<9)
#define     ADC12IE10        (1<<10)
#define     ADC12IE11        (1<<11)
#define     ADC12IE12        (1<<12)
#define     ADC12IE13        (1<<13)
#define     ADC12IE14        (1<<14)
#define     ADC12IE15        (1<<15)

/* Константы регистра ADC12IFG. Флаг прерывания. Эти биты устанавливаются, когда в
 * соответствующий регистр ADC12MEMx загружается результат преобразования. Биты ADC12IFGx
 * сбрасываются, если выполняется доступ к соответсвующим регистрам ADC12MEMx или же могут
 * быть сброшены программно.. */
#define     ADC12IFG0        (1<<0)
#define     ADC12IFG1        (1<<1)
#define     ADC12IFG2        (1<<2)
#define     ADC12IFG3        (1<<3)
#define     ADC12IFG4        (1<<4)
#define     ADC12IFG5        (1<<5)
#define     ADC12IFG6        (1<<6)
#define     ADC12IFG7        (1<<7)
#define     ADC12IFG8        (1<<8)
#define     ADC12IFG9         (1<<9)
#define     ADC12IFG10        (1<<10)
#define     ADC12IFG11        (1<<11)
#define     ADC12IFG12        (1<<12)
#define     ADC12IFG13        (1<<13)
#define     ADC12IFG14        (1<<14)
#define     ADC12IFG15        (1<<15)

#endif      /* _MSP430ADC_H */

