
#define PORT 11
#include <8006.h>
#include <zigzag.h>
#include <modbus.h>
#include <msp430.h>


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
    /*port_attr_t   port_attr;

    PIN_SET( port_attr.dir, (BLUE_PIN|GREEN_PIN|RED_PIN), PIN_HI );  
    PIN_CLEAR( port_attr.sel, BLUE_PIN|GREEN_PIN|RED_PIN );          
    port_set_attr( LED_PORT, BLUE_PIN|GREEN_PIN|RED_PIN, &port_attr );
    port_write( LED_PORT, BLUE_PIN|RED_PIN|GREEN_PIN, PIN_HI );
    */
    // тут ничего нет
}


void    mb_rx_notify( uint8_t   *buf, uint8_t   size )
{
	msg_t msg;
	struct msginfo msg_inf;
	
    //P4OUT ^= 0x10;
	if (size <= MAX_ZIGSWARM_PAYLOAD)
    {
        msg = msg_new(DEST_ADDR, DEST_PORT, DATA_MSG_TYPE, size, MFLAG_NO_CONFIRM);


        msg_info(msg, &msg_inf);

        memcpy(msg_inf.body_ptr, buf, size);
        msg_send(msg);
    }
    mb_rx_enable(0);
}


void    msg_recv( msg_t   msg )
{
    struct  msginfo minfo;
    uint8_t *txbuf;


    if( IS_ERROR(msg_info(msg, &minfo)) )
    {
        goto exit;
    }

    if( (uint8_t *)0 == (txbuf = mb_get_txbuf()) )
    {
        goto exit;
    }

    memcpy(txbuf, minfo.body_ptr, minfo.body_size);
    mb_tx(minfo.body_size);

exit:
    msg_destroy(msg);
}

void mb_tx_done(result_t result)
{
    mb_rx_enable(0);
}

