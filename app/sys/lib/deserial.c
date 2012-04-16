#include    <stdarg.h>
#include    <_zzSys.h>

size_t deserialize( const void *src, size_t size, const char *fmt, ... )
{
    uint8_t num = 0;
    void    *dst;
    va_list args;

    va_start( args, fmt );

    while( '\0' != *fmt ) {
        if( ':' == *fmt ) {
            if( 0 < num ) {
                if( size < num )
                    break;
                dst = (void *)va_arg( args, unsigned int );
                memcopy( dst, src, num );
                size -= num;
                src = ((uint8_t *)src) + num;
            }
            num = 0;
        } else
            num = to_digit( *fmt );
        ++fmt;
    }

    va_end( args );

    return size;
}

