#include    <_zigzag.h>

void memcopy( void *const dst, const void *const src, const uint8_t len )
{
    uint8_t i;
    for( i=0; i<len; i++)
        ((uint8_t *)dst)[i] = ((uint8_t *)src)[i];
}
