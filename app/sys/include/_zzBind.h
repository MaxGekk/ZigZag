#ifndef     __ZZ_BIND_H
#define     __ZZ_BIND_H

#include    <zzTypes.h>

#define     ALL_OBJ     0xff
#define     ALL_PORT    0xff
#define     ALL_OBJOFFSET   0xffff

extern const bind_t _begin_bind;
extern const bind_t _end_bind;

#define	OBJ_INFO( obj, port ) _OBJ_INFO( obj, port )
#define _OBJ_INFO( obj, port ) __OBJ_INFO( obj, port )
#define __OBJ_INFO( objnum, port ) const static bind_t _##objnum##_obj_info __attribute__ ((unused,__section__ (#objnum "." "obj_info" ))) = {objnum, port };

#define OBJ_OFFSET0( obj ) _OBJ_OFFSET( obj )
#define	_OBJ_OFFSET( obj ) __OBJ_OFFSET( obj )
#define	__OBJ_OFFSET( objnum ) (uint16_t)( &_##objnum##_obj_info - &_begin_bind )

int32_t port2offset( uint8_t port );
uint8_t offset2port( uint16_t obj_offset );
int32_t onum2offset( uint16_t obj_num );

#endif  /*  __ZZ_BIND_H */

