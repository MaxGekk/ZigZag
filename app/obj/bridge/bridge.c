/*! @file   bridge.c
 *  @brief  шлюз UART<->ZigBee. 
 *      При передаче по UART перед пакетом ZigSwarm вставляется поле с адресом назначения или источника. 
 *      Модуль использует прямые вызовы приема/отправки пакетов ZigBee, поэтому конфликтует с network.c 
 *  @author Max Gekk
 *  @author Timofei Istomin
 *  @date   июль 2007
 *  @version 1
 */   

//#define DBGLOG
#define OBJ     1
#define PORT    1
#include    <zigzag.h>
#include <msp430.h>

#define MAX_PAYLOAD_SIZE 90

//#define     BLINK_ON    /* Включать ли светодиоды ВНИМАНИЕ: с ними координатор виснет при подключении узла */
#define     BLINK_DURATION  200     /* Продолжительность горения красного светодиода при приёме и передаче */
#define     TIMER_NUM   0

#define     LED_PORT  2       /* Номер порта зелёного светодиода */
#define     GREEN_PIN   (1<<6)    /* Маска выхода на зелёный светодиод */
#define     RED_PIN     (1<<7)    /* Маска выхода на красный светодиод */

result_t combuf_recv(combuf_t combuf,  combuf_size_t payload_size)
{
    uint16_t dst_addr;
    msg_t msg;
    struct msginfo minfo;
    uint8_t i;
    uint8_t dst_port;
    //uint8_t src_port;
    uint8_t msg_type;
    result_t result;
    uint8_t msg_size;
    uint8_t position;


#if defined(BLINK_ON)    
    port_write( LED_PORT, RED_PIN, PIN_HI );     
    stimer_set( TIMER_NUM, BLINK_DURATION );
#endif

    if (payload_size > MAX_PAYLOAD_SIZE)
        return;

    dst_addr = ((combuf_read(combuf, 1)<<8)) | combuf_read(combuf, 0);
    
    //DBG1("psize %d", payload_size);
    position = 2;
    while ((payload_size - position) >= MSG_HEADER_SIZE )
    {
        dst_port = combuf_read(combuf, position++);
        position++; //игнорируем src_port. Будет использован порт данного объекта.
        msg_type = combuf_read(combuf, position++);
        msg_size = combuf_read(combuf, position++);

        //if (msg_size > payload_size - position)
        //    return;

        msg = msg_new(dst_addr, dst_port, msg_type, msg_size, MFLAG_NO_CONFIRM); // пакет будет удален автоматически после отправки
        if (! IS_OK(msg) )
            return;

        msg_info(msg, &minfo); // не проверяем результат, так как сообщение было только что создано
        for (i=0; i<msg_size; i++)
            ((uint8_t*)minfo.body_ptr)[i] = combuf_read(combuf, position++);

        msg_send(msg);
    }
}

void    msg_recv( msg_t   msg )
{
    uint16_t i;
    uint8_t position;
    combuf_t cb;
    struct msginfo minfo;


    if( IS_ERROR( msg_info( msg, &minfo ) ) )
        return;

    if( IS_ERROR( cb = combuf_create( COMBUF_DATA, minfo.body_size + MSG_HEADER_SIZE + 10 /*адреса*/) ) )
        return;

#if defined(BLINK_ON)    
    port_write( LED_PORT, RED_PIN, PIN_HI );     
    stimer_set( TIMER_NUM, BLINK_DURATION );
#endif

    position = 0;
    /*заголовок проводного протокола*/
    /*nwk addr*/
    combuf_write(cb, position++, (uint8_t)(minfo.src_addr));
    combuf_write(cb, position++, (uint8_t)(minfo.src_addr>>8));
    /*ieee addr*/
    {
        uint64_t addr = minfo.src_ext_addr;
        for (i=0; i<8; i++)
        {
            combuf_write(cb, position++, (uint8_t)(addr));
            addr >>= 8;
        }
    }
    
    /*восстановление заголовка ZigSwarm*/
    combuf_write(cb, position++, minfo.dst_port);
    combuf_write(cb, position++, minfo.src_port);
    combuf_write(cb, position++, minfo.msg_type);
    combuf_write(cb, position++, minfo.body_size);

    /* остаток пакета */
    for( i = 0; i < minfo.body_size; i++ ) {
        combuf_write( cb, position++, *(((uint8_t *)minfo.body_ptr)+i) );
    }

    combuf_send(cb);

    msg_destroy( msg );
}

REG_COMBUF_RECV(combuf_recv);

#if defined(BLINK_ON)    
void    stimer_fired( const uint8_t tnum )
{
    if( tnum == TIMER_NUM )
        port_write( LED_PORT, RED_PIN, PIN_LO );     
}
#endif


void sys_init()
{
#if defined(BLINK_ON)
    port_attr_t   port_attr;

    /* Настройка выхода на зелёный светодиод */
    PIN_SET( port_attr.dir, (GREEN_PIN|RED_PIN), PIN_HI );    /* Направление на вывод */
    PIN_CLEAR( port_attr.sel, GREEN_PIN|RED_PIN );          /* Функция ввода/вывода */
    port_set_attr( LED_PORT, GREEN_PIN|RED_PIN, &port_attr );
    port_write( LED_PORT, GREEN_PIN, PIN_LO );
    port_write( LED_PORT, RED_PIN, PIN_LO );     
#endif    

}

