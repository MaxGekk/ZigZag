#ifndef     __ZZ_ATTR_H
#define     __ZZ_ATTR_H

#include    <zzTypes.h>
#include    <_zzMacro.h>

#define     ATTR_FUNC( fn )  _ATTR_FUNC(fn, OBJ )
#define     _ATTR_FUNC( fn, OBJ ) __ATTR_FUNC( fn, OBJ )
#define     __ATTR_FUNC( fn, OBJ ) fn##_##OBJ


#define ATTR_HOOK \
__attribute__((used)) void ATTR_FUNC( attr_hook )() \
{ \
    ATTR_FUNC( attr_size )(0); \
    ATTR_FUNC( attr_set )( 0,0 ); \
    ATTR_FUNC( attr_get )( 0,0 ); \
}

#define REG_ATTR_FUNC( fn ) _REG_ATTR_FUNC(fn, OBJ )
#define _REG_ATTR_FUNC( fn, OBJ ) __REG_ATTR_FUNC( fn, OBJ )
#define __REG_ATTR_FUNC( fn, OBJ ) \
static hword_t _##OBJ##_##fn __attribute__ ((unused,__section__ ( #OBJ "." #fn ))) = (hword_t)fn##_##OBJ;

typedef size_t  (* _attr_size_ft)( const uint8_t attr_num );

extern  const _attr_size_ft     _begin_attr_size;
extern  const _attr_size_ft     _end_attr_size;
size_t  _attr_size( uint16_t obj_offset, const uint8_t attr_num );

typedef size_t  (* _attr_set_ft)( const uint8_t attr_num, const void *const from );

extern  const _attr_set_ft      _begin_attr_set;
extern  const _attr_set_ft      _end_attr_set;  
size_t  _attr_set( uint16_t obj_offset, const uint8_t attr_num, const void  *const from );

typedef size_t  (* _attr_get_ft)( const uint8_t attr_num, void  *const to );

extern  const _attr_get_ft      _begin_attr_get;
extern  const _attr_get_ft      _end_attr_get;  
size_t  _attr_get( uint16_t obj_offset, const uint8_t attr_num, void  *const to );

size_t  _msg_attr_recv( uint16_t obj_offset, msg_t  imsg_id );

#endif  /*  __ZZ_ATTR_H    */

