
#include    <_zigzag.h>
#include    <syszig.h>
#include    <infomem.h>
#include    <zzTypes.h>
#include    <8007.h>

#define     OBJOFFSET    onum2offset(OBJ)

size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to );
size_t  ATTR_FUNC( attr_set )( const uint8_t attr_num, const void *const from );
size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num );
common_attr_t common_attrs_32775;

static uint8_t vibro_state __attribute__ ((unused,__section__ (".module_infomem" ))) = 0;
static uint16_t vibro_threshold_high __attribute__ ((unused,__section__ (".module_infomem" ))) = 34868;
static uint16_t vibro_threshold_low __attribute__ ((unused,__section__ (".module_infomem" ))) = 8718;
static uint8_t vibro_state_change_num __attribute__ ((unused,__section__ (".module_infomem" ))) = 1;

static const struct attributes_t {
    uint8_t     num;
    uint8_t     size;
    void        *ptr;
} attributes[] = {
    {VIBRO_STATE,VIBRO_STATE_SIZE,&vibro_state},
    {VIBRO_THRESHOLD_HIGH,VIBRO_THRESHOLD_HIGH_SIZE,&vibro_threshold_high},
    {VIBRO_THRESHOLD_LOW,VIBRO_THRESHOLD_LOW_SIZE,&vibro_threshold_low},
    {VIBRO_STATE_CHANGE_NUM,VIBRO_STATE_CHANGE_NUM_SIZE,&vibro_state_change_num},
};
#define    TOTAL_ATTRIBUTES    (sizeof(attributes)/sizeof(struct attributes_t))

size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to )
{
    void * from = NULL;
    size_t size = 0;
    uint8_t i;

    // сначала обработка общих атрибутов
    size = common_attr_get(&common_attrs_32775, attr_num, to);
    if (size != 0)
        return size;
        
    // иначе обработка атрибутов данного профиля
    for( i=0; i<TOTAL_ATTRIBUTES; i++ ) 
        if( attributes[i].num == attr_num ) {
            from = attributes[i].ptr;
            size = attributes[i].size;
            break;
        }
    if( TOTAL_ATTRIBUTES <= i )
        return 0;
    
    memcopy(to, from, size);
    return size;
}
REG_ATTR_FUNC( attr_get );

size_t  ATTR_FUNC( attr_set )( const uint8_t attr_num, const void *const from )
{
    __label__ __report_change;
    void * to = NULL;
    size_t size = 0;
    uint8_t i;

    // сначала обработка общих атрибутов
    size = common_attr_set(&common_attrs_32775, attr_num, from);
    if (size != 0)
        goto __report_change;
        
    // иначе обработка атрибутов данного профиля
    for( i=0; i<TOTAL_ATTRIBUTES; i++ ) 
        if( attributes[i].num == attr_num ) {
            to = attributes[i].ptr;
            size = attributes[i].size;
            break;
        }
    if( TOTAL_ATTRIBUTES <= i )
        return 0;
    
    umemcopy(to, from, size);
__report_change:
    _attr_on_change(OBJOFFSET, &common_attrs_32775, attr_num);
    return size;
}
REG_ATTR_FUNC( attr_set );

size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num )
{
    uint8_t i;
    // сначала обработка общих атрибутов
    size_t size = common_attr_size(&common_attrs_32775, attr_num);
    if (size != 0)
        return size;
    
    for( i=0; i<TOTAL_ATTRIBUTES; i++ ) 
        if( attributes[i].num == attr_num )
            return attributes[i].size;

    return 0;
}
REG_ATTR_FUNC( attr_size );

void init_obj_32775()
{
    common_attrs_init(&common_attrs_32775);
}

INIT(init_obj_32775);
    
