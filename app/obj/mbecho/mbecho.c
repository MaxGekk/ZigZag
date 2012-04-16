/*! @file   mbecho.c
 *  @brief  Эхо-объект по modbus RTU
 *  @author Max Gekk
 *  @date   январь 2008
 *  @version 1
 */   

#define OBJ     1
#define PORT    17

#include    <zigzag.h>
#include    <modbus.h>

void    mb_rx_notify( uint8_t   *buf, uint8_t   size )
{
    __label__ exit;
    uint8_t *txbuf,i;

    if( (uint8_t *)0 == (txbuf = mb_get_txbuf()) )
        goto exit;

    for( i=0; i<size; i++ ) {
        txbuf[i] = buf[i];
        //txbuf[i] = '0'+size;
    }

    if( IS_ERROR(mb_tx( size )) )
        goto exit;

return;

exit:   
    mb_rx_enable(0);
    return;
}

void mb_tx_done(result_t result)
{
    mb_rx_enable(0);
}
