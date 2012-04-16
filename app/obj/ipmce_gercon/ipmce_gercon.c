/*! @file   ipmce_gercon.c
 *  @brief  Датчик-геркон, который будет установлен в ИТМиВТ
 *  @author Max Gekk
 *  @date   декабрь 2007
 *  @version 1
 *
 *      При размыкании геркона должен часто мигать красный светодиод.
 *  После замыкания геркона некоторое время ( секунд 5-10 ) должен моргать
 *  зелёный светодиод, а потом погаснуть.
 *
 *  */

#include    <8002.h>    /* Профиль цифровые входа-выходы */
#define PORT    2

#include    <zigzag.h>

#define     GERCON_PORT   2         /* Номер порта геркона */
#define     GERCON_IRQ   IRQ_PORT2  /* Прерывание он геркона */
#define     GERCON_PIN   1          /* Вход геркона на порте GERCON_PORT  */

#define     PRIORITY    (PRIORITY_LOW+2)    /* Приоритет событий от геркона */
#define     EV_TYPE_START   0   /* Начало работы */
#define     EV_TYPE_OPEN    1   /* Размыкание геркона */
#define     EV_TYPE_CLOSE   2   /* Замыкание геркона */

#define     TIMER_NUM   0

#define     RED_PORT    2       /* Номер порта красного светодиода */
#define     RED_PIN     (1<<6)  /* Маска выхода на красный светодиод */
#define     RED_PERIOD  200     /* Период в миллисекундах, с которым вкл./выкл. красный светодиод */

#define     GREEN_PORT  2       /* Номер порта зелёного светодиода */
#define     GREEN_PIN   (1<<7)  /* Маска выхода на зелёный светодиод */
#define     GREEN_PERIOD    300 /* Период в миллисекундах, с которым вкл./выкл. зелёный светодиод */

/* Состояние геркона */
static enum {
    GERCON_UNKNOW,  /* Не известное */
    GERCON_OPEN,    /* Разомкнут */
    GERCON_CLOSE    /* Замкнут */
} state = GERCON_UNKNOW;    /* По умолчанию состояние не известно */

#define     MAX_GREEN_COUNT     10  /* Максимальное число переключений зелёного светодиода */
uint16_t green_count;   /* Счётчик переключений зелёного светодиода */

/* Инициализация портов */
void    sys_init()
{
    port_attr_t   port_attr;


    /* Настройка выхода на зелёный светодиод */
    PIN_SET( port_attr.dir, GREEN_PIN, PIN_HI );    /* Направление на вывод */
    PIN_CLEAR( port_attr.sel, GREEN_PIN );          /* Функция ввода/вывода */
    port_set_attr( GREEN_PORT, GREEN_PIN, &port_attr );
    port_write( GREEN_PORT, GREEN_PIN, PIN_LO );       /* По умолчанию гасим светодиод */

    /* Настройка выхода на красный светодиод */
    PIN_SET( port_attr.dir, RED_PIN, PIN_HI );    /* Направление на вывод */
    PIN_CLEAR( port_attr.sel, RED_PIN );          /* Функция ввода/вывода */
    port_set_attr( RED_PORT, RED_PIN, &port_attr );
    port_write( RED_PORT, RED_PIN, PIN_LO );       /* По умолчанию гасим светодиод */

    /* Настройка входа от геркона  */
    {   port_t  gercon_port = 0x00;
        event_type_t    event_type = 0;
        result_t    res = ENOSYS;    

        port_reset_iflag( GERCON_PORT, GERCON_PIN );

        PIN_CLEAR( port_attr.dir, GERCON_PIN );     /* Направление на ввод */
        PIN_CLEAR( port_attr.sel, GERCON_PIN );     /* Функция ввода/вывода */ 
        PIN_SET( port_attr.ie, GERCON_PIN, PIN_HI );    /* Разрешаем прерывания от кнопки */
        PIN_SET( port_attr.ies, GERCON_PIN, PIN_HI );   /* Ловим изменение с высокого на низкий уровень */ 
        port_set_attr( GERCON_PORT, GERCON_PIN, &port_attr );

        port_read( GERCON_PORT, GERCON_PIN, &gercon_port );

        /* Определение текущего состояния геркона */
        if( PIN_IS_SET( gercon_port, GERCON_PIN) ) {
            event_type = EV_TYPE_OPEN;
        } else {   
            PIN_SET( port_attr.ies, GERCON_PIN, PIN_LO );   /* Ловим изменение с низкого на высокий уровень */ 
            port_set_attr( GERCON_PORT, GERCON_PIN, &port_attr );
            event_type = EV_TYPE_CLOSE;
        }
        res = event_emit( PRIORITY_LOW, event_type, 0 );
        if( IS_ERROR(res) ) {
            /* Не смогли начать работу, зажигаем красный светодиод */
            port_write( RED_PORT, RED_PIN, PIN_HI );
        }
    }

    return;
}

void    stimer_fired( const uint8_t tnum )
{
    port_t  ledport;
    result_t    res;

    if( tnum != TIMER_NUM ) return;

    if( state == GERCON_OPEN ) {
        port_read( RED_PORT, RED_PIN, &ledport );
        ledport ^= RED_PIN;
        port_write( RED_PORT, RED_PIN, ledport );
        res = stimer_set( TIMER_NUM, RED_PERIOD );
    } else if( state == GERCON_CLOSE ) {
        if( MAX_GREEN_COUNT <= green_count )
            port_write( GREEN_PORT, GREEN_PIN, PIN_HI );
        green_count++;
        port_read( GREEN_PORT, GREEN_PIN, &ledport );
        ledport ^= GREEN_PIN;
        port_write( GREEN_PORT, GREEN_PIN, ledport );
        res = stimer_set( TIMER_NUM, GREEN_PERIOD );
    }
    if( IS_ERROR(res) )
        port_write( RED_PORT, RED_PIN, PIN_HI );
    return;
}

void    event_handler( event_type_t event_type, unidata_t   unidata )
{
    uint16_t    input_attr = 0;
    result_t    res = EINVAL;

    if( EV_TYPE_OPEN == event_type ) {
        /* Геркон разомкнут */
        state = GERCON_OPEN;
        input_attr &= ~GERCON_PIN;
        attr_write( INPUT, &input_attr );
        res = stimer_set( TIMER_NUM, RED_PERIOD );
    } else if( EV_TYPE_CLOSE == event_type ) {
        /* Геркон замкнут */
        state = GERCON_CLOSE;
        input_attr |= GERCON_PIN;
        attr_write( INPUT, &input_attr );
        res = stimer_set( TIMER_NUM, GREEN_PERIOD );
        green_count = 0;
    } else
        return;

    port_write( RED_PORT, RED_PIN, PIN_LO );
    port_write( GREEN_PORT, GREEN_PIN, PIN_LO );

    //if( IS_ERROR(res) ) 
    //    port_write( RED_PORT, RED_PIN, PIN_HI );
    return;
}

void    irq_handler( irq_t  irq )
{
    if( GERCON_IRQ == irq ) {
        /* Обрабатываем прерывание от геркона */
        port_attr_t   port_attr;
        /* Перенастраиваем вход от геркона. Если до этого прерывание от геркона генерилось
         * на спаде, то перенастраиваем на фронт. Если же на фронте, то перенастраиваем на спад.  */
        port_get_attr( GERCON_PORT, GERCON_PIN, &port_attr );
        port_attr.ies ^= GERCON_PIN; 
        port_set_attr( GERCON_PORT, GERCON_PIN, &port_attr );
        /* Продолжаем обработку прерывания в обработчике события */
        if( PIN_IS_SET( port_attr.ies, GERCON_PIN ) )
            event_emit( PRIORITY_HIGH, EV_TYPE_OPEN, 0 );
        else
            event_emit( PRIORITY_HIGH, EV_TYPE_CLOSE, 0 );
    }
    return;
}


