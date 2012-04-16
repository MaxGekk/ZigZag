#ifndef     _MSP430DMA_H
#define     _MSP430DMA_H
/*! @file   msp430dma.h
 *  @brief  Регистры и константы dma
 *  @author Max Gekk
 *  @date   декабрь 2007
 *  @version 1
 */   

#include    <inttypes.h>

/* Регистр 0 управления DMA */
volatile    uint16_t    DMACTL0 asm("0x0122");
/* Регистр 1 управления DMA */
volatile    uint16_t    DMACTL1 asm("0x0124");

/* Регистр управления канала 0 */
volatile    uint16_t    DMA0CTL asm("0x01e0");
/* Регистр адреса источника канала 0 */
volatile    uint16_t    DMA0SA  asm("0x01e2");
/* Регистр адреса назначения канала 0 */
volatile    uint16_t    DMA0DA  asm("0x01e4");
/* Регистр объёма переноса канала 0 */
volatile    uint16_t    DMA0SZ  asm("0x01e6");

/* Регистр управления канала 1 */
volatile    uint16_t    DMA1CTL asm("0x01e8");
/* Регистр адреса источника канала 1 */
volatile    uint16_t    DMA1SA  asm("0x01ea");
/* Регистр адреса назначения канала 1 */
volatile    uint16_t    DMA1DA  asm("0x01ec");
/* Регистр объёма переноса канала 1 */
volatile    uint16_t    DMA1SZ  asm("0x01ee");

/* Регистр управления канала 2 */
volatile    uint16_t    DMA2CTL asm("0x01f0");
/* Регистр адреса источника канала 2 */
volatile    uint16_t    DMA2SA  asm("0x01f2");
/* Регистр адреса назначения канала 2 */
volatile    uint16_t    DMA2DA  asm("0x01f4");
/* Регистр объёма переноса канала 2 */
volatile    uint16_t    DMA2SZ  asm("0x01f6");

/* Константы регистра DMACTL0 */
/* DMA0TSELx - Выбор источника сигнала запуска канала 0 */
#define     DMA0TSEL_0000   0x0000  /* Бит DMAREQ - программный запуск */
#define     DMA0TSEL_0001   0x0001  /* Бит TACCR2 CCIFG */      
#define     DMA0TSEL_0010   0x0002  /* Бит TBCCR2 CCIFG */
#define     DMA0TSEL_0011   0x0003  /* URXIFG0 (режим UART/SPI), данные приняты USART0 ( режим I2C ) */
#define     DMA0TSEL_0100   0x0004  /* UTXIFG0 (режим UART/SPI), готовность передачи USART0 ( режим I2C ) */
#define     DMA0TSEL_0101   0x0005  /* Бит DAC12IFG DAC12_0CTL */
#define     DMA0TSEL_0110   0x0006  /* Бит ADC12IFGx ADC12 */
#define     DMA0TSEL_0111   0x0007  /* Бит CCIFG TACCR0 */
#define     DMA0TSEL_1000   0x0008  /* Бит CCIFG TBCCR0 */
#define     DMA0TSEL_1001   0x0009  /* URXIFG1 */
#define     DMA0TSEL_1010   0x000A  /* UTXIFG1 */
#define     DMA0TSEL_1011   0x000B  /* Готовность умножителя */
#define     DMA0TSEL_1100   0x000C  /* Действие не производится */
#define     DMA0TSEL_1101   0x000D  /* Действие не производится */
#define     DMA0TSEL_1110   0x000E  /* Бит DMA0IFG запускает канал 1, бит DMA1IFG запускает канал 2, бит DMA2IFG запускает канал 0 */
#define     DMA0TSEL_1111   0x000F  /* Внешний запуск DMAE0 */
/* DMA1TSELx - Выбор источника сигнала запуска канала 1 */
#define     DMA1TSEL_0000   (DMA0TSEL_0000<<4)  /* Бит DMAREQ - программный запуск */
#define     DMA1TSEL_0001   (DMA0TSEL_0001<<4)  /* Бит TACCR2 CCIFG */      
#define     DMA1TSEL_0010   (DMA0TSEL_0010<<4)  /* Бит TBCCR2 CCIFG */
#define     DMA1TSEL_0011   (DMA0TSEL_0011<<4)  /* URXIFG0 (режим UART/SPI), данные приняты USART0 ( режим I2C ) */
#define     DMA1TSEL_0100   (DMA0TSEL_0100<<4)  /* UTXIFG0 (режим UART/SPI), готовность передачи USART0 ( режим I2C ) */
#define     DMA1TSEL_0101   (DMA0TSEL_0101<<4)  /* Бит DAC12IFG DAC12_0CTL */
#define     DMA1TSEL_0110   (DMA0TSEL_0110<<4)  /* Бит ADC12IFGx ADC12 */
#define     DMA1TSEL_0111   (DMA0TSEL_0111<<4)  /* Бит CCIFG TACCR0 */
#define     DMA1TSEL_1000   (DMA0TSEL_1000<<4)  /* Бит CCIFG TBCCR0 */
#define     DMA1TSEL_1001   (DMA0TSEL_1001<<4)  /* URXIFG1 */
#define     DMA1TSEL_1010   (DMA0TSEL_1010<<4)  /* UTXIFG1 */
#define     DMA1TSEL_1011   (DMA0TSEL_1011<<4)  /* Готовность умножителя */
#define     DMA1TSEL_1100   (DMA0TSEL_1100<<4)  /* Действие не производится */
#define     DMA1TSEL_1101   (DMA0TSEL_1101<<4)  /* Действие не производится */
#define     DMA1TSEL_1110   (DMA0TSEL_1110<<4)  /* Бит DMA0IFG запускает канал 1, бит DMA1IFG запускает канал 2, бит DMA2IFG запускает канал 0 */
#define     DMA1TSEL_1111   (DMA0TSEL_1111<<4)  /* Внешний запуск DMAE0 */
/* DMA2TSELx - Выбор источника сигнала запуска канала 2 */
#define     DMA2TSEL_0000   (DMA0TSEL_0000<<8)  /* Бит DMAREQ - программный запуск */
#define     DMA2TSEL_0001   (DMA0TSEL_0001<<8)  /* Бит TACCR2 CCIFG */      
#define     DMA2TSEL_0010   (DMA0TSEL_0010<<8)  /* Бит TBCCR2 CCIFG */
#define     DMA2TSEL_0011   (DMA0TSEL_0011<<8)  /* URXIFG0 (режим UART/SPI), данные приняты USART0 ( режим I2C ) */
#define     DMA2TSEL_0100   (DMA0TSEL_0100<<8)  /* UTXIFG0 (режим UART/SPI), готовность передачи USART0 ( режим I2C ) */
#define     DMA2TSEL_0101   (DMA0TSEL_0101<<8)  /* Бит DAC12IFG DAC12_0CTL */
#define     DMA2TSEL_0110   (DMA0TSEL_0110<<8)  /* Бит ADC12IFGx ADC12 */
#define     DMA2TSEL_0111   (DMA0TSEL_0111<<8)  /* Бит CCIFG TACCR0 */
#define     DMA2TSEL_1000   (DMA0TSEL_1000<<8)  /* Бит CCIFG TBCCR0 */
#define     DMA2TSEL_1001   (DMA0TSEL_1001<<8)  /* URXIFG1 */
#define     DMA2TSEL_1010   (DMA0TSEL_1010<<8)  /* UTXIFG1 */
#define     DMA2TSEL_1011   (DMA0TSEL_1011<<8)  /* Готовность умножителя */
#define     DMA2TSEL_1100   (DMA0TSEL_1100<<8)  /* Действие не производится */
#define     DMA2TSEL_1101   (DMA0TSEL_1101<<8)  /* Действие не производится */
#define     DMA2TSEL_1110   (DMA0TSEL_1110<<8)  /* Бит DMA0IFG запускает канал 1, бит DMA1IFG запускает канал 2, бит DMA2IFG запускает канал 0 */
#define     DMA2TSEL_1111   (DMA0TSEL_1111<<8)  /* Внешний запуск DMAE0 */

/* Константы регистра DMACTL1 */
#define     ENNMI       0x0001      /*  Разрешение прекращения DMA переноса немаскируемым прерыванием NMI */
#define     ROUNDROBIN  0x0002      /*  Этот бит разрешает циклическое движение приоритетов каналов DMA. */
#define     DMAONFETCH  0x0003      /*  1 - dma перенос происходит при выборке следующей команды, 0 - перенос происходит немедленно */

/* DMAxCTL регистр управления каналом x */
#define     DMAREQ      0x0001      /*  Запрос DMA. Программно управляемый старт DMA. Сбрасывается автоматически после старта */
#define     DMAABORT    0x0002      /*  DMA перенос был прерван NMI прерыванием */
#define     DMAIE       0x0004      /*  Разрешение DMA прерывания */
#define     DMAIFG      0x0008      /*  Флаг DMA прерывания */
#define     DMAEN       0x0010      /*  Разрешение DMA */
#define     DMALEVEL    0x0020      /*  Условие запуска переноса: по фронту (0) или по высокому уровню (1) */
#define     DMASRCBYTE  0x0040      /*  Формат источника: слово (0) или байт (1) */
#define     DMADSTBYTE  0x0080      /*  Формат назначения: слово (0) или байт (1) */
#define     DMASRCINCR_00   0x0000  /*  Адрес источника не изменяется */  
#define     DMASRCINCR_01   0x0100  /*  Адрес источника не изменяется */
#define     DMASRCINCR_10   0x0200  /*  Адрес источника декрементируется */
#define     DMASRCINCR_11   0x0300  /*  Адрес источника инкрементируется */
#define     DMADSTINCR_00   0x0000  /*  Адрес назначения не изменяется */  
#define     DMADSTINCR_01   0x0400  /*  Адрес назначения не изменяется */
#define     DMADSTINCR_10   0x0800  /*  Адрес назначения декрементируется */
#define     DMADSTINCR_11   0x0c00  /*  Адрес назначения инкрементируется */
/* Режимы переноса */
#define     DMADT_000       0x0000  /*  Одиночный перенос */
#define     DMADT_001       0x1000  /*  Блочный перенос */
#define     DMADT_010       0x2000  /*  Пакетно-блочный перенос */
#define     DMADT_011       0x3000  /*  Пакетно-блочный перенос */
#define     DMADT_100       0x4000  /*  Повторный одиночный перенос */
#define     DMADT_101       0x5000  /*  Повторный блочный перенос */
#define     DMADT_110       0x6000  /*  Повторный пакетно-блочный перенос */
#define     DMADT_111       0x7000  /*  Повторный пакетно-блочный перенос */


#endif  /*  _MSP430DMA_H  */
