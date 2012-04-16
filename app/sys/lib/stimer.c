/* Реализация интерфейса синхронного таймера */
#include    <_zigzag.h>
#include    <syszig.h>

static struct {
    uint16_t    obj_offset;
} stimer[ SOFT_TIMER_TOTAL ];

result_t    _stimer_set( const uint16_t obj_offset, const uint8_t tnum, const uint32_t milli_sec )
{
    if( SOFT_TIMER_TOTAL <= tnum )
        return EINVAL;
    if( !ISZIGLOAD )
        return ENOSYS;
    stimer[ tnum ].obj_offset = obj_offset;
    __stimer_set( tnum, milli_sec );

    return ENOERR;
}

__attribute__((used)) void __stimer_fired( const uint8_t tnum ) 
{
    const stimer_fired_ft *stf = &_begin_stimer_fired;
    if( SOFT_TIMER_TOTAL <= tnum )
        return;
    stf += stimer[tnum].obj_offset;
    (* stf)( tnum );
    return;
}

__attribute__((weak)) void    stimer_fired( const uint8_t tnum ) {}

