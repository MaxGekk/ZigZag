/*! @file   blinker.c
 *  @brief  Пример прикладного объекта: периодическое мигание светодиодом
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1
 *      Каждые две секунды на 1 секунду загорается светодиод. 
 *  Для отсчёта секундных интервалов используется синхронный таймер.
 */   

//#define DBGLOG
#define OBJ     1
#define PORT    3
#include    <zigzag.h>

#define     SYNC_TIMER  0

#define     LED_PORT     2       /* номер порта светодиодов */
#define     RED_LED     0x80    /* Ножка красного светодиода */

#define     WAIT_PERIOD     1000    /* миллисекунд */
#define     BLINK_DURATION  3277    /* Число тиков асинхронного таймера за 100 мсек */

#define     EV_PRIORITY     0   /* Приоритет события: срабатывание асинхронного таймера */
#define     EV_TYPE         0   /* Тип события о срабатывании асинхр. таймера */
#define     EV_UNIDATA      0

static uint16_t count = 0;

/* Инициализация прикладного объекта */
void    sys_init()
{
    port_attr_t   port_attr;
    /* Настройка ножки светодиода */

    PIN_SET( port_attr.dir, RED_LED, PIN_HI );    /* Направление на вывод */
    PIN_CLEAR( port_attr.sel, RED_LED);         /* Функция ввода/вывода */
    PIN_CLEAR( port_attr.ie, RED_LED);          /* Запрет прерываний */

    port_set_attr( LED_PORT, RED_LED, &port_attr );

    port_write( LED_PORT, RED_LED, PIN_LO );

    /* Запуск синхронного таймера */
    stimer_set( SYNC_TIMER , WAIT_PERIOD );
    return;
}

void    stimer_fired( const uint8_t tnum )
{
    DBG("timer fired");
    event_emit( EV_PRIORITY, EV_TYPE, EV_UNIDATA );
    return;
}

void    event_handler( event_type_t event_type, unidata_t   unidata )
{
    /* Проверяем, что данное событие от таймера */
    if( EV_TYPE == event_type ) {
        /* Запускаем таймер ещё раз */
        DBG("timer set");
        stimer_set( SYNC_TIMER, WAIT_PERIOD );

        if( count++ % 2 )
            port_write( LED_PORT, RED_LED, PIN_LO );
        else
            port_write( LED_PORT, RED_LED, PIN_HI );
    }
    return;
}

