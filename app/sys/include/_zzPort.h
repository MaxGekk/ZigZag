#ifndef     __ZZ_PORT_H
#define     __ZZ_PORT_H

#include    <zzTypes.h>
#include    <_zzMacro.h>

result_t    _port_write( uint16_t obj_offset, const uint8_t  port_num, const port_t mask, const port_t value );

result_t    _port_read( uint16_t obj_offset, const uint8_t port_num, const port_t mask, port_t *const value_ptr);

result_t    _port_set_attr( uint16_t obj_offset, const uint8_t port_num, const port_t mask, 
        const port_attr_t *const port_attr_ptr );

result_t    _port_get_attr( uint16_t obj_offset, const uint8_t port_num, const port_t mask, 
        port_attr_t *const port_attr_ptr );

result_t    _port_get_iflag( uint16_t obj_offset, const uint8_t port_num, const port_t mask, 
        port_t *const iflag_ptr );

result_t    _port_reset_iflag( uint16_t obj_offset, const uint8_t port_num, const port_t mask );

#endif  /*  __ZZ_PORT_H */
