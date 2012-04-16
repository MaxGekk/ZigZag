//  vim:encoding=utf-8
#ifndef     _ZIGZAG_CONFIG_H
#define     _ZIGZAG_CONFIG_H
/*! @file  config.h
 *  @brief Конфигурационный заголовочный файл
 *  @author     Max Gekk
 *  @date       декабрь 2007 г.
 *  @version    1           
 *  */

/* Включить стек ZigBee */
//#define     ZC_ZIGBEE

/* Перенаправлять обработку прерываний в прикладную часть */
#define     ZC_IRQ

/* Поддержка мягких таймеров */
#define     ZC_STIMER

/* Поддержка жёстких таймеров */
#define     ZC_HTIMER

/* Производить ли инициализацию прикладной части */
#define     ZC_INIT

/* 64-битное время */
#define     ZC_TIME64

/* Операции с цифровыми входами-выходами */
#define     ZC_PORT

/* Маски разрешённых цифровых входов-выходов */
#define     ZC_PORT1    0xFF
#define     ZC_PORT2    0xFF
#define     ZC_PORT3    0xFF
#define     ZC_PORT4    0xFF
#define     ZC_PORT5    0xFF
#define     ZC_PORT6    0xFF

/* Управление энергопотреблением */
#define     ZC_POWER

/* Включать ли модуль сериализации */
//#define     ZC_SERIALIZE

/* Извещение о засыпании */
//#define     ZC_SLEEP_NOTIFY

#endif     /* _ZIGZAG_CONFIG_H */

