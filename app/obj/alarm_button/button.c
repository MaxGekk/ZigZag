/*! @file   button.c
 *  @brief  Пример прикладного объекта: реакция на кнопку
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1
 *  */

#define OBJ     2
#define PORT    2
#include    <zigzag.h>

#define     LED_PORT    2       /* номер порта для светодиодов */
#define     BLUE_LED    0x40    /* Синий светодиод */

#define     BUTTON_PORT 1               /* номер порта с кнопкой */
#define     BUTTON_PIN  (1 << 5)        /* ножка кнопки */

#define     BUTTON_PRIORITY     1   /* Приоритет события: прерывание от кнопки */
#define     BUTTON_TYPE         1   /* Тип события:  прерывание от кнопки */
#define     BUTTON_PRESS        1
#define     BUTTON_FREE         0

#define     DST_PORT    1
#define     DST_ADDR    0
//#define     MSG_TYPE    0x42  // alarm
#define     MSG_TYPE    0x41    // fire

#define     BODY_SIZE   sizeof(uint16_t)

static msg_t bmsg; // Button msg
static uint16_t press_count;

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

    /* Настройка кнопки */
    PIN_CLEAR( port_attr.dir, BUTTON_PIN );     /* Направление на ввод */
    PIN_CLEAR( port_attr.sel, BUTTON_PIN );     /* Функция ввода/вывода */ 
    PIN_SET( port_attr.ies, BUTTON_PIN, PIN_LO );   /* Ловим изменение с высокого на низкий уровень */ 
    PIN_SET( port_attr.ie, BUTTON_PIN, PIN_HI );    /* Разрешаем прерывания от кнопки */

    port_set_attr( BUTTON_PORT, BUTTON_PIN, &port_attr );

    bmsg = msg_new( DST_ADDR, DST_PORT, MSG_TYPE, BODY_SIZE, MFLAG_DEFAULT );
    press_count = 0;

    return;
}

void    irq_handler( irq_t  irq )
{
    /* Обрабатываем нажатие от кнопки */
    if( IRQ_PORT1 == irq ) {
        port_attr_t   port_attr;
        /* Перенастраиваем кнопку */
        port_get_attr( BUTTON_PORT, BUTTON_PIN, &port_attr );
        port_attr.ies ^= BUTTON_PIN; 
        port_set_attr( BUTTON_PORT, BUTTON_PIN, &port_attr );
        /* Продолжаем обработку прерывания в обработчике события */
        if( PIN_IS_SET( port_attr.ies, BUTTON_PIN ) )
            event_emit( BUTTON_PRIORITY, BUTTON_PRESS, 0 );
        else
            event_emit( BUTTON_PRIORITY, BUTTON_FREE, 0 );
    }
    return;
}

void    event_handler( event_type_t event_type, unidata_t   unidata )
{
    if( event_type == BUTTON_PRESS )
    {
        struct msginfo info;
        port_write( LED_PORT, BLUE_LED, PIN_LO );
        
        if(IS_OK(msg_info( bmsg, &info ))){
        //if( info.body_ptr ) {
 //           uint64_t time = sys_time();
//            memcopy( info.body_ptr, &time, sizeof(time) );
            memcopy( info.body_ptr, &press_count, sizeof(press_count) );
        }
        /* Отправляем сообщение */
        msg_send( bmsg );
        press_count++;
    }
    else if( event_type == BUTTON_FREE )
        port_write( LED_PORT, BLUE_LED, PIN_HI );
    return;
}


