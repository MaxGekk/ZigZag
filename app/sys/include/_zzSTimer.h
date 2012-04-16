#ifndef     __ZZ_STIMER_H
#define     __ZZ_STIMER_H

#include    <zzTypes.h>
#include    <_zzMacro.h>

result_t    _stimer_set( const uint16_t obj_offset, const uint8_t tnum, const uint32_t milli_sec );

typedef     void (* stimer_fired_ft)( uint8_t );

extern const    stimer_fired_ft     _begin_stimer_fired;
extern const    stimer_fired_ft     _end_stimer_fired;

#endif  /*  __ZZ_STIMER_H */
