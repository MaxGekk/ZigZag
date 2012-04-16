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
        port_write( LED_PORT, BLUE_LED, PIN_LO );
    else if( event_type == BUTTON_FREE )
        port_write( LED_PORT, BLUE_LED, PIN_HI );
    return;
}


