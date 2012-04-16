#ifndef     __ZZ_ATIMER_H
#define     __ZZ_ATIMER_H

#include    <zzTypes.h>
#include    <_zzMacro.h>

uint32_t    _atimer_counter();

result_t    _atimer_set( const uint16_t obj_offset, const uint8_t tnum, const uint32_t tpoint );   

result_t    _atimer_stop( const uint16_t  obj_offset, const uint8_t tnum );

result_t    _atimer_info( const uint8_t tnum, struct timerinfo  *const info );

typedef void (* atimer_fired_ft)( uint8_t );

extern const atimer_fired_ft    _begin_atimer_fired;
extern const atimer_fired_ft    _end_atimer_fired;

#endif  /*  __ZZ_ATIMER_H   */

