#ifndef     __ZZ_SYS_H
#define     __ZZ_SYS_H

#include    <zzTypes.h>
#include    <_zzMacro.h>
#include    <serialize.h>

typedef void (* sys_init_ft)();

extern  const sys_init_ft     _begin_sys_init;
extern  const sys_init_ft     _end_sys_init;

uint64_t    _sys_time();
int16_t     _set_sys_time( uint64_t time );

void memcopy( void *const dst, const void *const src, const uint8_t len );

#define     to_digit(c)     ((c)-'0')

#endif  /*  __ZZ_SYS_H   */

