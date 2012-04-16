#ifndef _MSP430FLASH_H
#define _MSP430FLASH_H
/*! @file   msp430flash.h
 *  @brief  Регистры и константы флэш-памяти
 *  @author Max Gekk
 *  @date   декабрь 2007
 *  @version 1
 */   

#include    <inttypes.h>
#include    <msp430.h>

/*! Регистр 1 управления флэш-памятью */
volatile    uint16_t    FCTL1 asm("0x0128");
/*! Регистр 2 управления флэш-памятью */
volatile    uint16_t    FCTL2 asm("0x012A");
/*! Регистр 3 управления флэш-памятью */
volatile    uint16_t    FCTL3 asm("0x012C");

#define FRKEY               0x9600  /* Пароль флэш при вычитывании из регистра */
#define FWKEY               0xA500  /* Пароль флэш для записи в регистр */

#define ERASE               0x0002  /* Очистка одного сегмента*/
#define MERAS               0x0004  /* Очистка всех сегментов */

#define WRT                 0x0040  /* Режим записи */
#define BLKWRT              0x0080  /* Режим блочной записи */

#define FSSEL_ACLK          0x0000
#define FSSEL_MCLK          0x0040
#define FSSEL_SMCLK         0x0080

#define BUSY                0x0001  /* Состояние тактового генератора флэш: 1 - занято */
#define KEYV                0x0002  /* Ключ нарушения безопасности: 1 - записан неправильный пароль */
#define ACCVIFG             0x0004  /* Флаг прерывания при нарушении прав доступа: 1 - ожидается прерывание */
#define WAIT                0x0008  /* 1 - флэш готова для записи следующего байта/слова */
#define LOCK                0x0010  /* Бит блокировки: 1 - флэш залочена (только чтение) */
#define EMEX                0x0020  /* 1 - аварийный выход */

#endif  /* _MSP430FLASH_H */

