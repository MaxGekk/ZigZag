/*! @file   fbut.c
 *  @brief  Реализация цифровых входов-выходов
 *  @author Max Gekk
 *  @date   февраль 2008
 *  @version 1
 *  */

#include    <8002.h>
#define PORT    19

#include    <zigzag.h>

//#define     PLATFORM_USVD
#define     PLATFORM_FIRE_BUTTON
//#define     PLATFORM_ALARM_BUTTON
//#define     BLINK_ON
#define     PRIORITY    (PRIORITY_HIGH)
#define     GIOP_INTERRUPT  0   /* Номер события, который генерируется при изменении входов */

#if defined(PLATFORM_UBPD)

#define     IPORT   2       /* Номер порта цифровых входов */
#define     IPORT_IRQ   IRQ_PORT2
#define     IPINS   0x0F    /* XXX Маска цифровых входов */

#define     OPORT   5       /* Номер порта цифровых выходов */
#define     OPINS   0x0F    /* Маска цифровых выходов */

#if defined(BLINK_ON)
#define     LED_PORT    6       /* номер порта для светодиодов */
#define     LED1        1
#define     LED2        2
#define     LED3        4
#endif

#elif   defined(PLATFORM_USVD)

#define     IPORT   1       /* Номер порта цифровых входов */
#define     IPORT_IRQ   IRQ_PORT1
#define     IPINS   0x0F    /* XXX Маска цифровых входов */

#define     OPORT   5       /* Номер порта цифровых выходов */
#define     OPINS   0xFF    /* Маска цифровых выходов */

#elif   (defined(PLATFORM_FIRE_BUTTON) || defined(PLATFORM_ALARM_BUTTON))
#define     IPORT   1
#define     IPORT_IRQ   IRQ_PORT1
#define     IPINS   0x20

#define     OPORT   5
#define     OPINS   0xFE

#ifdef PLATFORM_FIRE_BUTTON
#define     FIRE_LED_PORT   5
#define     FIRE_LED_PIN    0x01
#else
#define     FIRE_LED_PORT   2
#define     FIRE_LED_PIN    0x10
#endif

#define     CONTROL_PORT    2
#define     PINT1           0x02
#define     PINT2           0x01
#define     CONTROL_PINS    (PINT1|PINT2)
#endif


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
//#if defined(PLATFORM_FIRE_BUTTON)
#if   (defined(PLATFORM_FIRE_BUTTON) || defined(PLATFORM_ALARM_BUTTON))
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
        PIN_SET( port_attr.ies, IPINS, PIN_LO );   /* Ловим изменение с низкого на высокий уровень */ 
        port_set_attr( IPORT, IPINS, &port_attr );

        /* Определение текущего состояния входов */
        port_read( IPORT, IPINS, &iport );
        /* У тех входов, которые уже в 1 настраиваем прерывания на спад */
        PIN_SET( port_attr.ies, iport, PIN_HI );   /* Ловим изменение с высокого на низкий уровень */ 
        port_set_attr( IPORT, iport, &port_attr );
        PIN_SET( port_attr.ie, IPINS, PIN_HI );    /* Разрешаем прерывания от кнопки */
        /*
        res = event_emit( PRIORITY, GIOP_INTERRUPT, (unidata_t)iport );
        if( IS_ERROR(res) ) {
            port_write( LED_PORT, LED1, PIN_HI );
        }
        */
//#if defined(PLATFORM_FIRE_BUTTON)
#if   (defined(PLATFORM_FIRE_BUTTON) || defined(PLATFORM_ALARM_BUTTON))
        PIN_SET( port_attr.dir, FIRE_LED_PIN, PIN_HI );   /* Направление на вывод */
        PIN_CLEAR( port_attr.sel, FIRE_LED_PIN );         /* Функция ввода/вывода */
        PIN_CLEAR( port_attr.ie, FIRE_LED_PIN );          /* Запрет прерываний */
        port_set_attr( FIRE_LED_PORT, FIRE_LED_PIN, &port_attr );
        port_write( FIRE_LED_PORT, FIRE_LED_PIN, iport ? PIN_LO : PIN_HI );
#endif
    }
    return;
}

void    irq_handler( irq_t  irq )
{
    /* Обрабатываем изменение состояний входов */
    if( IPORT_IRQ == irq ) {
        port_attr_t   port_attr;
        port_t  ifg, in, in1;
        char ev_emit_flag = 0;
        char eq_cnt = 0;
        in = in1 = ifg = 0;
        /* Перенастраиваем прерывания от входов */
        port_get_attr( IPORT, IPINS, &port_attr );
        do
        {
           ifg = 0;
           port_get_iflag( IPORT, IPINS, &ifg );   /* Маска флагов прерываний */
           //port_reset_iflag( IPORT, ifg );         /* Сбрасываем флаги прерываний */
           port_read( IPORT, IPINS, &in );         /* Вычитываем состояние входов, чтобы записать их в атрибут */
           if( ifg )
           {
               ev_emit_flag = 1;
               port_attr.ies ^= ifg;   /* Меняем условие генерации прерывания, у входов, у которых изменилось состояние */
               port_set_attr( IPORT, IPINS, &port_attr );
           }
           port_reset_iflag( IPORT, ifg );         /* Сбрасываем флаги прерываний */
           port_read( IPORT, IPINS, &in1 );         /* Вычитываем состояние входов, чтобы записать их в атрибут */
           if(in == in1)
              eq_cnt++;
           else
              eq_cnt = 0;
        }while(eq_cnt < 2);
        //}while(in != in1);
        /* Продолжаем обработку прерывания в обработчике события */
        if(ev_emit_flag)
           event_emit( PRIORITY, GIOP_INTERRUPT, in );
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
        attr_write( INPUT, &unidata );
//#if defined(PLATFORM_FIRE_BUTTON)
#if   (defined(PLATFORM_FIRE_BUTTON) || defined(PLATFORM_ALARM_BUTTON))
        port_write( FIRE_LED_PORT, FIRE_LED_PIN, unidata ? PIN_LO : PIN_HI );
#endif
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

