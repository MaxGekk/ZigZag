#define OBJ     255
#define PORT    255

#include    <zigzag.h>

void    event_handler( event_type_t event_type, unidata_t   unidata ) 
{
    return;
}

void    msg_send_done( msg_t   msg, status_t    status )
{
    return;
}

size_t    msg_recv( msg_t   msg )
{
    return 0; 
}

void    sys_init()
{
    return;
}

void    atimer_fired( uint8_t    tnum )
{
    return;
}

void    irq_handler( irq_t  irq )
{
    return;
}

void    stimer_fired( const uint8_t tnum ) 
{
    return;
}

