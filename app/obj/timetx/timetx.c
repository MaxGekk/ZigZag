/*! @file   blinker.c
 *  @brief  Пример прикладного объекта: отсылка текущего времени
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1
 *      Через полторы секунды после отправки предыдущего сообщения отсылается новое сообщение 
 *  с текущим временем на порт 1 узлу с адресом 0x0000. Для отсчёта секундных интервалов 
 *  используется синхронный таймер.
 */   

#define DBGLOG
#define OBJ     5
#define PORT    12
#include    <zigzag.h>
#include    <msp430.h>

#define     LED_PORT    2       /* номер порта для светодиодов */
//#define     BLUE_LED    0x40    /* Синий светодиод */
#define     BLUE_LED    0x80

#define     SYNC_TIMER  1
#define     WAIT_PERIOD     2000    /* миллисекунд */

#define     EV_PRIORITY     0   /* Приоритет события: срабатывание асинхронного таймера */
#define     EV_TYPE         0   /* Тип события о срабатывании асинхр. таймера */
#define     EV_UNIDATA      0

#define     DST_PORT    0x01
#define     DST_ADDR    0x0000
#define     MSG_TYPE    0x40
#define     BODY_SIZE   sizeof(uint64_t)

static msg_t msg;

/* Инициализация прикладного объекта */
void    sys_init()
{
    port_attr_t   port_attr;
    /* Настройка ножек светодиода */

    PIN_SET( port_attr.dir, BLUE_LED, PIN_HI );   /* Направление на вывод */
    PIN_CLEAR( port_attr.sel, BLUE_LED );         /* Функция ввода/вывода */
    PIN_CLEAR( port_attr.ie, BLUE_LED );          /* Запрет прерываний */

    port_set_attr( LED_PORT, BLUE_LED, &port_attr );

    port_write( LED_PORT, BLUE_LED, PIN_LO );

    msg = msg_new( DST_ADDR, DST_PORT, MSG_TYPE, BODY_SIZE, MFLAG_DEFAULT );

    stimer_set( SYNC_TIMER , WAIT_PERIOD );

    return;
}

void    stimer_fired( const uint8_t tnum )
{
    event_emit( EV_PRIORITY, EV_TYPE, EV_UNIDATA );
    return;
}

void    event_handler( event_type_t event_type, unidata_t   unidata )
{
    /* Проверяем, что данное событие от таймера */
    if( EV_TYPE == event_type ) {
        struct msginfo info;

        msg_info( msg, &info );
        if( info.body_ptr ) {
            uint64_t time = sys_time();
            memcopy( info.body_ptr, &time, sizeof(time) );
        }
        /* Отправляем сообщение */
        msg_send( msg );

        port_write( LED_PORT, BLUE_LED, PIN_HI );

    }
    return;
}

void    msg_send_done( msg_t   msg, status_t    status ) 
{
    port_write( LED_PORT, BLUE_LED, PIN_LO );

    /* Запускаем таймер ещё раз */
    stimer_set( SYNC_TIMER, WAIT_PERIOD );

    return;
}

void    msg_recv( msg_t   msg )
{
    uint16_t i;
    combuf_t cb;
    struct msginfo minfo;

    if( IS_ERROR( msg_info( msg, &minfo ) ) )
        return;

    if( IS_ERROR( cb = combuf_create( COMBUF_DATA, minfo.body_size) ) )
        return;

    DBG1("body_size = %x", (uint16_t)minfo.body_size );
    for( i = 0; i < minfo.body_size; i++ ) {
        DBG2("body[%hhx]=%hhx",i,*(((uint8_t *)minfo.body_ptr)+i));
        combuf_write( cb, i, *(((uint8_t *)minfo.body_ptr)+i) );
    }

    combuf_send(cb);

    msg_destroy( msg );
}
