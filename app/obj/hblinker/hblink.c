/*! @file   blinker.c
 *  @brief  Пример прикладного объекта: периодическое мигание светодиодом
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1
 *      Каждую секунду на 500 мсек загорается светодиод. 
 *  Для отсчёта 500 мсек. интервалов используется асинхронный таймер,
 */   

#define OBJ     4
#define PORT    4
#include    <zigzag.h>

#define     ASYNC_TIMER 0

#define     LED_PORT     2       /* номер порта светодиодов */
#define     GREEN_LED     0x80    /* Ножка зелёного светодиода */

#define     WAIT_PERIOD     16384  /* 500 мсек. */

#define     EV_PRIORITY     0   /* Приоритет события: срабатывание асинхронного таймера */
#define     EV_TYPE         0   /* Тип события о срабатывании асинхр. таймера */
#define     EV_UNIDATA      0

static uint16_t count = 0;

/* Инициализация прикладного объекта */
void    sys_init()
{
    port_attr_t   port_attr;
    /* Настройка ножки светодиода */

    PIN_SET( port_attr.dir, GREEN_LED, PIN_HI );    /* Направление на вывод */
    PIN_CLEAR( port_attr.sel, GREEN_LED);         /* Функция ввода/вывода */
    PIN_CLEAR( port_attr.ie, GREEN_LED);          /* Запрет прерываний */
    port_set_attr( LED_PORT, GREEN_LED, &port_attr );
    port_write( LED_PORT, GREEN_LED, PIN_LO );

    atimer_set( ASYNC_TIMER , atimer_counter() + WAIT_PERIOD );

    return;
}

void    atimer_fired( uint8_t    tnum ) 
{ 
    event_emit( EV_PRIORITY, EV_TYPE, EV_UNIDATA );
    return;
}

void    event_handler( event_type_t event_type, unidata_t   unidata )
{
    /* Проверяем, что данное событие от таймера */
    if( EV_TYPE == event_type ) {
        /* Запускаем таймер ещё раз */
        atimer_set( ASYNC_TIMER, atimer_counter() + WAIT_PERIOD );

        if( count++ % 2 )
            port_write( LED_PORT, GREEN_LED, PIN_LO );
        else
            port_write( LED_PORT, GREEN_LED, PIN_HI );
    }
    return;
}


