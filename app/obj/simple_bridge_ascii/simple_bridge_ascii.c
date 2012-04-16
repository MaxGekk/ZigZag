
#define PORT 11
#include <8006.h>
#include <zigzag.h>
#include <msp430.h>


#define     MODBUS_ADDRESS      1
#define     MB_FCODE_DATA       102

#define DATA_MSG_TYPE 0x20

#define DEST_PORT PORT

#define MAX_ZIGSWARM_PAYLOAD 80

/*
#define     LED_PORT     4       
#define     RED_PIN     0x1
#define     BLUE_PIN    0x10
#define     GREEN_PIN   0x8
*/

void    sys_init()
{
    //P2SEL &= 0x10;
    //P2DIR |= 0x10;
    //P2OUT |= 0x10;
    /*port_attr_t   port_attr;

    PIN_SET( port_attr.dir, (BLUE_PIN|GREEN_PIN|RED_PIN), PIN_HI );  
    PIN_CLEAR( port_attr.sel, BLUE_PIN|GREEN_PIN|RED_PIN );          
    port_set_attr( LED_PORT, BLUE_PIN|GREEN_PIN|RED_PIN, &port_attr );
    port_write( LED_PORT, BLUE_PIN|RED_PIN|GREEN_PIN, PIN_HI );
    */
    // тут ничего нет
}


void mbascii_rx_notify( unsigned char mb_addr,
        unsigned char func_code,
        int size, void  *buf )
{
	msg_t msg;
	struct msginfo msg_inf;
	
    //P2OUT ^= 0x10;
	if (size <= MAX_ZIGSWARM_PAYLOAD)
    {
        msg = msg_new(DEST_ADDR, DEST_PORT, DATA_MSG_TYPE, size, MFLAG_NO_CONFIRM);


        msg_info(msg, &msg_inf);

        memcpy(msg_inf.body_ptr, buf, size);
        msg_send(msg);
    }
}


void    msg_recv( msg_t   msg )
{
    struct  msginfo minfo;
    uint8_t *txbuf;
    unsigned char   data[ MAX_ZIGSWARM_PAYLOAD ];


    if( IS_ERROR(msg_info(msg, &minfo)) )
    {
        goto exit;
    }

    memcpy(data, minfo.body_ptr, minfo.body_size);
    mbascii_tx(MODBUS_ADDRESS, MB_FCODE_DATA, minfo.body_size);

exit:
    msg_destroy(msg);
}

