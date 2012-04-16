/*! @file   mbascii_bridge.c
 *  @brief  Шлюз ZigBee <-> ModBus ASCII
 *  @author Max Gekk
 *  @date   март 2008
 *  @version 1
 */

#define OBJ     1
#define PORT    1
#include    <zigzag.h>
#include    <platform.h>

#define     MODBUS_ADDRESS      1
#define     MB_FCODE_DATA       102
#define     MAX_PAYLOAD_SIZE    80
#define     BLINK_ON

#ifdef      BLINK_ON
#define     BLINK_DURATION  200     /* Продолжительность горения красного светодиода при приёме и передаче */
#define     TIMER_NUM   0
#define     GREEN_LED   LED0    /* Маска выхода на зелёный светодиод */
#define     RED_LED     LED1    /* Маска выхода на красный светодиод */
#endif

void mbascii_rx_notify( unsigned char mb_addr,
        unsigned char func_code,
        int size, void  *data )
{
    msg_t msg;
    struct msginfo minfo;
    uint16_t    dst_net_addr;
    uint8_t     dst_port, msg_type, msg_size;

    deserialize( data, size, "2:1:1:1:1:", 
            &dst_net_addr, &dst_port,0,&msg_type, &msg_size );
    msg = msg_new( dst_net_addr, dst_port,msg_type,msg_size, MFLAG_NO_CONFIRM );
    if( IS_ERROR(msg) ) 
        return;
    msg_info(msg, &minfo); // не проверяем результат, так как сообщение было только что создано

    memcopy( minfo.body_ptr, (uint8_t *)data+6, msg_size );

    msg_send(msg);

#if defined(BLINK_ON)    
    port_write( LED_PORT, RED_LED, PIN_HI );     
    stimer_set( TIMER_NUM, BLINK_DURATION );
#endif
}

void    msg_recv( msg_t   msg )
{
    enum { HDR_SIZE = 14 };
    unsigned char   data[ MAX_PAYLOAD_SIZE ];
    struct msginfo minfo;

    if( IS_OK( msg_info( msg, &minfo ) ) ) {
        serialize( data, MAX_PAYLOAD_SIZE, "2:8:1:1:1:1:",
                minfo.src_addr, minfo.src_ext_addr,
                minfo.dst_port, minfo.src_port,
                minfo.msg_type, minfo.body_size );

        if( (HDR_SIZE+minfo.body_size)<= MAX_PAYLOAD_SIZE ) {
            memcopy( &data[HDR_SIZE], minfo.body_ptr, minfo.body_size );
            mbascii_tx( MODBUS_ADDRESS, MB_FCODE_DATA, HDR_SIZE+minfo.body_size, data );
            //port_write( LED_PORT, RED_LED, PIN_HI );
        }
    }
    msg_destroy( msg );

#if defined(BLINK_ON)    
    port_write( LED_PORT, RED_LED, PIN_HI );     
    stimer_set( TIMER_NUM, BLINK_DURATION );
#endif
}

#ifdef  BLINK_ON

void    stimer_fired( const uint8_t tnum )
{
    if( tnum == TIMER_NUM )
        port_write( LED_PORT, RED_LED, PIN_LO );     
}

void sys_init()
{
    port_attr_t   port_attr;
    PIN_SET( port_attr.dir, (RED_LED|GREEN_LED), PIN_HI );
    PIN_CLEAR( port_attr.sel, (RED_LED|GREEN_LED));       
    PIN_CLEAR( port_attr.ie, (RED_LED|GREEN_LED));        
    port_set_attr( LED_PORT, RED_LED|GREEN_LED, &port_attr );
    port_write( LED_PORT, RED_LED|GREEN_LED, GREEN_LED );
}
#endif

