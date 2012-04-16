//#define     DBGLOG
#include    <_zigzag.h>
#include    <syszig.h>

#include    <zzDebug.h>

int32_t port2offset( uint8_t port )
{
    const bind_t    *bind_ptr = &_begin_bind;
    for(; bind_ptr != &_end_bind; bind_ptr++ ) {
       if( bind_ptr->port_num == port ) 
           return (uint16_t)( bind_ptr - &_begin_bind );
    }
    return EINVAL;
}

uint8_t offset2port( uint16_t obj_offset )
{
    const bind_t    *bind_ptr = &_begin_bind;
    bind_ptr += obj_offset;
    return bind_ptr->port_num;
}

int32_t onum2offset( uint16_t obj_num )
{
    const bind_t    *bind_ptr = &_begin_bind;
    for(; bind_ptr != &_end_bind; bind_ptr++ )
    {
       if( bind_ptr->obj_num == obj_num ) 
           return (uint16_t)( bind_ptr - &_begin_bind );
    }
    return EINVAL;
}

