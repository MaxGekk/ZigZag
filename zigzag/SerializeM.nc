#include    <stdarg.h>

module SerializeM {
} implementation {

#define     to_digit(c)     ((c)-'0')

static void memcopy( void *const dst, const void *const src, const uint8_t len )
{
    uint8_t i;
    for( i=0; i<len; i++)
        ((uint8_t *)dst)[i] = ((uint8_t *)src)[i];
}

size_t serialize( void *dst, size_t max_size, const char *fmt, ... )  @spontaneous() @C()
{
    uint8_t len = 0, num = 0;
    va_list args;
    va_start( args, fmt );
    while( '\0' != *fmt ) {
        if( *fmt == ':' ) {
            if( 0 < num) {
                if( max_size < (len+num) )
                    break;
                switch( num ) {
                    case 1: { unsigned char src = (unsigned char)va_arg( args, unsigned int );
                        memcopy( dst, &src, sizeof(src) ); 
                        }
                        break;
                    case 2: { unsigned int src = va_arg( args, unsigned int );
                        memcopy( dst, &src, sizeof(src) ); 
                        }
                        break;
                    case 4: { unsigned int src = va_arg( args, unsigned long int );
                        memcopy( dst, &src, sizeof(src) ); 
                        }
                        break;
                    case 8: { unsigned long long int src = va_arg( args, unsigned long long int );
                        memcopy( dst, &src, sizeof(src) ); 
                        }
                        break;
                    default: break;
                }
                len += num;
                dst = ((uint8_t *)dst) + num;
            }
            num = 0;
        } else 
            num = to_digit( *fmt );
        ++fmt;
    }
    va_end( args );
    return len;
}

size_t deserialize( const void *src, size_t size, const char *fmt, ... ) @spontaneous() @C() 
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
    
}

