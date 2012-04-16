/*! @file   settime.c
 *  @brief  Установка времени
 *  @author Max Gekk
 *  @date   апрель 2008
 *  @version 1
 *  */

#define     OBJ     2
#define     PORT    2
#include    <zigzag.h>

void    msg_recv( msg_t   msg )
{
    struct msginfo minfo;

    if( IS_OK( msg_info( msg, &minfo ) ) ) {
        if( minfo.msg_type == 0x20 ) {
            uint64_t    time;
            deserialize( minfo.body_ptr, minfo.body_size, "8:", &time);
            set_sys_time( time );
        }
    }

    msg_destroy( msg );
    
}
