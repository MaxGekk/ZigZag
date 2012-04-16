/* Реализация интерфейса асинхронного таймера */
#include    <_zigzag.h>
#include    <syszig.h>

struct {
    uint16_t    obj_offset;
} atimers[ HARD_TIMER_TOTAL ];

uint32_t    _atimer_counter() { return ISZIGLOAD ?__atimer_counter():0; }

result_t    _atimer_set( const uint16_t obj_offset, const uint8_t tnum, const uint32_t tpoint )
{
    if( ISZIGLOAD ) {
        atimers[ tnum ].obj_offset = obj_offset;
        if( IS_ERROR( __atimer_set( tnum, tpoint ) ) )
            return EINVAL;
        return ENOERR;
    }
    return ENOSYS;
}

result_t    _atimer_stop( const uint16_t  obj_offset, const uint8_t tnum )
{
    if( ISZIGLOAD ) {
        if( obj_offset != atimers[ tnum ].obj_offset )
            return EACCESS;
        if( IS_ERROR( __atimer_stop( tnum ) ) )
            return EINVAL;
        return ENOERR;
    }
    return ENOSYS;
}

result_t    _atimer_info( const uint8_t tnum, struct timerinfo  *const info )
{
    if( 0 == info ) return EINVAL;
    if( ISZIGLOAD ) {
        int16_t is_set;
        if( IS_ERROR( is_set = __atimer_is_set( tnum ) ) )
            return EINVAL;
        info->is_set = is_set;
        info->tpoint = __atimer_point( tnum );
        return ENOERR;
    }
    return ENOSYS;
}

__attribute__((used)) void __atimer_fired( uint8_t tnum )
{
    const atimer_fired_ft *tf =  &_begin_atimer_fired;
    tf += atimers[ tnum ].obj_offset;
    (*tf)( tnum );         
    return;
}

__attribute__((weak)) void    atimer_fired( uint8_t    tnum ) { }

