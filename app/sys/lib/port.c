#include    <_zigzag.h>
#include    <syszig.h>

static const struct {
    volatile port_t    *const in;
    volatile port_t    *const out;
    volatile port_t    *const dir;
    volatile port_t    *const ifg;
    volatile port_t    *const ies;
    volatile port_t    *const ie;
    volatile port_t    *const sel;
} paddr[ PORT_TOTAL ] = {
    { (port_t *)0x20, (port_t *)0x21, (port_t *)0x22, (port_t *)0x23, (port_t *)0x24, (port_t *)0x25, (port_t *)0x26 },
    { (port_t *)0x28, (port_t *)0x29, (port_t *)0x2a, (port_t *)0x2b, (port_t *)0x2c, (port_t *)0x2d, (port_t *)0x2e },
    { (port_t *)0x18, (port_t *)0x19, (port_t *)0x1a, (port_t *)0x00, (port_t *)0x00, (port_t *)0x00, (port_t *)0x1b },
    { (port_t *)0x1c, (port_t *)0x1d, (port_t *)0x1e, (port_t *)0x00, (port_t *)0x00, (port_t *)0x00, (port_t *)0x1f },
    { (port_t *)0x30, (port_t *)0x31, (port_t *)0x32, (port_t *)0x00, (port_t *)0x00, (port_t *)0x00, (port_t *)0x33 },
    { (port_t *)0x34, (port_t *)0x35, (port_t *)0x36, (port_t *)0x00, (port_t *)0x00, (port_t *)0x00, (port_t *)0x37 }
};

result_t    port_write( const uint8_t  port_num, const port_t mask, const port_t value )
{
    if( PORT_TOTAL < port_num )
        return EINVAL;
    if( !PIN_IS_SET(*paddr[ port_num-1 ].dir, mask ) )
        return EPERM;
    if( !ISZIGLOAD )
        return ENOSYS;
    if( IS_ERROR( __port_perm( port_num, mask, OP_WRITE ) ) )
        return EACCESS;


    PIN_SET( *paddr[ port_num-1 ].out, mask, value );

    return ENOERR;
}

result_t    port_read( const uint8_t port_num, const port_t mask, port_t *const value_ptr)
{
    if( (PORT_TOTAL < port_num)||(0 == value_ptr) )
        return EINVAL;
    if( !ISZIGLOAD )
        return ENOSYS;
    if( IS_ERROR( __port_perm( port_num, mask, OP_READ ) ) )
        return EACCESS;

    *value_ptr = *paddr[ port_num-1 ].in & mask;

    return ENOERR;
}

result_t    port_set_attr( const uint8_t port_num, const port_t mask, const port_attr_t *const port_attr_ptr )
{
    if( (PORT_TOTAL < port_num)||(0 == port_attr_ptr) )
        return EINVAL;
    if( !ISZIGLOAD )
        return ENOSYS;
    if( IS_ERROR( __port_perm( port_num, mask, OP_SET_ATTR ) ) )
        return EACCESS;

    if( port_num < 3 )
        PIN_CLEAR( *paddr[ port_num-1 ].ie, mask );

    PIN_SET( *paddr[ port_num-1 ].dir, mask, port_attr_ptr->dir );
    PIN_SET( *paddr[ port_num-1 ].sel, mask, port_attr_ptr->sel );

    if( port_num < 3 ) {
        PIN_SET( *paddr[ port_num-1 ].ies, mask, port_attr_ptr->ies );
        PIN_CLEAR( *paddr[ port_num-1 ].ifg, mask );
        PIN_SET( *paddr[ port_num-1 ].ie, mask, port_attr_ptr->ie );
    }

    return ENOERR;

}

result_t    port_get_attr( const uint8_t port_num, const port_t mask, port_attr_t *const port_attr_ptr )
{
    if( (PORT_TOTAL < port_num)||(0 == port_attr_ptr) )
        return EINVAL;
    if( !ISZIGLOAD )
        return ENOSYS;
    if( IS_ERROR( __port_perm( port_num, mask, OP_GET_ATTR ) ) )
        return EACCESS;

    port_attr_ptr->dir = *paddr[ port_num-1 ].dir & mask;
    port_attr_ptr->sel = *paddr[ port_num-1 ].sel & mask;
    if( port_num < 3 ) {
        port_attr_ptr->ie = *paddr[ port_num-1 ].ie;
        port_attr_ptr->ies = *paddr[ port_num-1 ].ies;
    }

    return ENOERR;
}

result_t    port_get_iflag( const uint8_t port_num, const port_t mask, port_t *const iflag_ptr )
{
    if( (2 < port_num)||(0 == iflag_ptr) )
        return EINVAL;
    if( !ISZIGLOAD )
        return ENOSYS;
    if( IS_ERROR( __port_perm( port_num, mask, OP_GET_IFLAG ) ) )
        return EACCESS;

    *iflag_ptr = *paddr[ port_num-1 ].ifg & mask;

    return ENOERR;
}

result_t    port_reset_iflag( const uint8_t port_num, const port_t mask )
{
    if( 2 < port_num )
        return EINVAL;
    if( !ISZIGLOAD )
        return ENOSYS;
    if( IS_ERROR( __port_perm( port_num, mask, OP_RESET_IFLAG ) ) )
        return EACCESS;

    PIN_CLEAR( *paddr[ port_num-1 ].ifg, mask );

    return ENOERR;
}


