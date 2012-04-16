#include    <stdarg.h>
#include    <_zzSys.h>

size_t serialize( void *dst, size_t max_size, const char *fmt, ... )
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

