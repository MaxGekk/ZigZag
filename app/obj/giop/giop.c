/*! @file   giop.c
 *  @brief  Реализация цифровых входов-выходов
 *  @author Max Gekk
 *  @date   февраль 2008
 *  @version 1
 *  */

#include    <8002.h>
#define PORT    19

#include    <zigzag.h>
#include    <platform.h>

//#define     BLINK_ON
#define     PRIORITY    (PRIORITY_HIGH)
#define     GIOP_INTERRUPT  0   /* Номер события, который генерируется при изменении входов */
#define     INVERT_ATTR

/* Инициализация портов */
void    sys_init()
{
    port_attr_t   port_attr;
#if defined(BLINK_ON)
    PIN_SET( port_attr.dir, LED1|LED2|LED3, PIN_HI );   /* Направление на вывод */
    PIN_CLEAR( port_attr.sel, LED1|LED2|LED3 );         /* Функция ввода/вывода */
    PIN_CLEAR( port_attr.ie, LED1|LED2|LED3 );          /* Запрет прерываний */
    port_set_attr( LED_PORT, LED1|LED2|LED3, &port_attr );
    port_write( LED_PORT, LED1|LED2|LED3, PIN_LO );
#endif
#if defined(PLATFORM_FIRE_BUTTON)
    PIN_SET( port_attr.dir, CONTROL_PINS, PIN_HI );   /* Направление на вывод */
    PIN_CLEAR( port_attr.sel, CONTROL_PINS );         /* Функция ввода/вывода */
    PIN_CLEAR( port_attr.ie, CONTROL_PINS );          /* Запрет прерываний */
    port_set_attr( CONTROL_PORT, CONTROL_PINS, &port_attr );
    port_write( CONTROL_PORT, CONTROL_PINS, PINT1 );
#endif
    /* Настройка входа от геркона  */
    {   port_t  iport = 0x00;
        event_type_t    event_type = 0;
        result_t    res = ENOSYS;    

        port_reset_iflag( IPORT, IPINS );

        PIN_CLEAR( port_attr.dir, IPINS );     /* Направление на ввод */
        PIN_CLEAR( port_attr.sel, IPINS );     /* Функция ввода/вывода */ 
        PIN_SET( port_attr.ie, IPINS, PIN_HI );    /* Разрешаем прерывания от кнопки */
        PIN_SET( port_attr.ies, IPINS, PIN_LO );   /* Ловим изменение с низкого на высокий уровень */ 
        port_set_attr( IPORT, IPINS, &port_attr );

        /* Определение текущего состояния входов */
        port_read( IPORT, IPINS, &iport );
        /* У тех входов, которые уже в 1 настраиваем прерывания на спад */
        PIN_SET( port_attr.ies, iport, PIN_HI );   /* Ловим изменение с высокого на низкий уровень */ 
        port_set_attr( IPORT, iport, &port_attr );
        res = event_emit( PRIORITY, GIOP_INTERRUPT, (unidata_t)iport );
        if( IS_ERROR(res) ) {
            port_write( LED_PORT, LED1, PIN_HI );
        }
    }
    return;
}

void    irq_handler( irq_t  irq )
{
    /* Обрабатываем изменение состояний входов */
    if( IPORT_IRQ == irq ) {
        port_attr_t   port_attr;
        port_t  ifg, in;
        /* Перенастраиваем прерывания от входов */
        port_get_attr( IPORT, IPINS, &port_attr );
        port_get_iflag( IPORT, IPINS, &ifg );   /* Маска флагов прерываний */
        if( ifg ) {
            port_reset_iflag( IPORT, ifg );         /* Сбрасываем флаги прерываний */
            port_attr.ies ^= ifg;   /* Меняем условие генерации прерывания, у входов, у которых изменилось состояние */
            port_set_attr( IPORT, IPINS, &port_attr );
            port_read( IPORT, IPINS, &in );         /* Вычитываем состояние входов, чтобы записать их в атрибут */
            /* Продолжаем обработку прерывания в обработчике события */
            event_emit( PRIORITY, GIOP_INTERRUPT, in );
        }
    }
    return;
}

void    event_handler( event_type_t event_type, unidata_t   unidata )
{
    if( event_type == GIOP_INTERRUPT ) {
        /* Изменилось состояние входов */
#if defined(BLINK_ON)
        port_t  ledpin;
        port_read( LED_PORT, LED1, &ledpin );
        ledpin ^= LED1; 
        port_write( LED_PORT, LED1, ledpin );
#endif
#if defined(INVERT_ATTR)
        unidata = ~unidata;
#endif        
        attr_write( INPUT, &unidata );
    } else if( event_type == EV_AWRITTEN ){
        if( unidata == OUTPUT ) {
            /* Изменён атрибут цифровых выходов */
            port_t  out;
            result_t res = attr_read( OUTPUT, &out );
            if( IS_OK(res) ) {
                /* Записываем в порт новое значение атрибута */
                port_write( OPORT, OPINS, out );
            }
#if defined(BLINK_ON)
            port_t  ledpin;
            port_read( LED_PORT, LED2, &ledpin );
            ledpin ^= LED2; 
            port_write( LED_PORT, LED2, ledpin );
#endif
        }
    }
    return;
}

