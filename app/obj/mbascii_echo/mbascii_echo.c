/*! @file   mbascii_echo.c
 *  @brief  Эхо-объект по modbus ASCII
 *  @author Max Gekk
 *  @date   март 2008
 *  @version 1
 */   

#define OBJ     1
#define PORT    16

#include    <zigzag.h>
#include    <modbus.h>

void mbascii_rx_notify( unsigned char mb_addr,
        unsigned char func_code,
        int size, void  *data )
{
    mbascii_tx( mb_addr, func_code, size, data );
}

