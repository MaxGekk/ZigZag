
#include    <_zigzag.h>
#include    <syszig.h>
#include    <infomem.h>
#include    <zzTypes.h>
#include    <./8003.h>

#define     OBJOFFSET    onum2offset(OBJ)

size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to );
size_t  ATTR_FUNC( attr_set )( const uint8_t attr_num, const void *const from );
size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num );
common_attr_t common_attrs_32771;

uint16_t analog_input1;
uint16_t analog_input1_thr_active_levels = 3;
uint16_t analog_input1_thr_triggered_levels;
uint16_t analog_input1_upper_thr_threshold1 = 2000;
uint16_t analog_input1_lower_thr_threshold1 = 1000;
uint16_t analog_input2;
uint16_t analog_input2_thr_active_levels = 3;
uint16_t analog_input2_thr_triggered_levels;
uint16_t analog_input2_upper_thr_threshold1 = 2000;
uint16_t analog_input2_lower_thr_threshold1 = 1000;
uint16_t analog_input3;
uint16_t analog_input3_thr_active_levels = 3;
uint16_t analog_input3_thr_triggered_levels;
uint16_t analog_input3_upper_thr_threshold1 = 2000;
uint16_t analog_input3_lower_thr_threshold1 = 1000;
uint16_t analog_input4;
uint16_t analog_input4_thr_active_levels = 3;
uint16_t analog_input4_thr_triggered_levels;
uint16_t analog_input4_upper_thr_threshold1 = 2000;
uint16_t analog_input4_lower_thr_threshold1 = 1000;
uint16_t analog_input1_test = 0;
uint16_t analog_input2_test = 0;
uint16_t analog_input3_test = 0;
uint16_t analog_input4_test = 0;

static const struct attributes_t {
    uint8_t     num;
    uint8_t     size;
    void        *ptr;
} attributes[] = {
    {ANALOG_INPUT1,ANALOG_INPUT1_SIZE,&analog_input1},
    {ANALOG_INPUT1_THR_ACTIVE_LEVELS,ANALOG_INPUT1_THR_ACTIVE_LEVELS_SIZE,&analog_input1_thr_active_levels},
    {ANALOG_INPUT1_THR_TRIGGERED_LEVELS,ANALOG_INPUT1_THR_TRIGGERED_LEVELS_SIZE,&analog_input1_thr_triggered_levels},
    {ANALOG_INPUT1_UPPER_THR_THRESHOLD1,ANALOG_INPUT1_UPPER_THR_THRESHOLD1_SIZE,&analog_input1_upper_thr_threshold1},
    {ANALOG_INPUT1_LOWER_THR_THRESHOLD1,ANALOG_INPUT1_LOWER_THR_THRESHOLD1_SIZE,&analog_input1_lower_thr_threshold1},
    {ANALOG_INPUT2,ANALOG_INPUT2_SIZE,&analog_input2},
    {ANALOG_INPUT2_THR_ACTIVE_LEVELS,ANALOG_INPUT2_THR_ACTIVE_LEVELS_SIZE,&analog_input2_thr_active_levels},
    {ANALOG_INPUT2_THR_TRIGGERED_LEVELS,ANALOG_INPUT2_THR_TRIGGERED_LEVELS_SIZE,&analog_input2_thr_triggered_levels},
    {ANALOG_INPUT2_UPPER_THR_THRESHOLD1,ANALOG_INPUT2_UPPER_THR_THRESHOLD1_SIZE,&analog_input2_upper_thr_threshold1},
    {ANALOG_INPUT2_LOWER_THR_THRESHOLD1,ANALOG_INPUT2_LOWER_THR_THRESHOLD1_SIZE,&analog_input2_lower_thr_threshold1},
    {ANALOG_INPUT3,ANALOG_INPUT3_SIZE,&analog_input3},
    {ANALOG_INPUT3_THR_ACTIVE_LEVELS,ANALOG_INPUT3_THR_ACTIVE_LEVELS_SIZE,&analog_input3_thr_active_levels},
    {ANALOG_INPUT3_THR_TRIGGERED_LEVELS,ANALOG_INPUT3_THR_TRIGGERED_LEVELS_SIZE,&analog_input3_thr_triggered_levels},
    {ANALOG_INPUT3_UPPER_THR_THRESHOLD1,ANALOG_INPUT3_UPPER_THR_THRESHOLD1_SIZE,&analog_input3_upper_thr_threshold1},
    {ANALOG_INPUT3_LOWER_THR_THRESHOLD1,ANALOG_INPUT3_LOWER_THR_THRESHOLD1_SIZE,&analog_input3_lower_thr_threshold1},
    {ANALOG_INPUT4,ANALOG_INPUT4_SIZE,&analog_input4},
    {ANALOG_INPUT4_THR_ACTIVE_LEVELS,ANALOG_INPUT4_THR_ACTIVE_LEVELS_SIZE,&analog_input4_thr_active_levels},
    {ANALOG_INPUT4_THR_TRIGGERED_LEVELS,ANALOG_INPUT4_THR_TRIGGERED_LEVELS_SIZE,&analog_input4_thr_triggered_levels},
    {ANALOG_INPUT4_UPPER_THR_THRESHOLD1,ANALOG_INPUT4_UPPER_THR_THRESHOLD1_SIZE,&analog_input4_upper_thr_threshold1},
    {ANALOG_INPUT4_LOWER_THR_THRESHOLD1,ANALOG_INPUT4_LOWER_THR_THRESHOLD1_SIZE,&analog_input4_lower_thr_threshold1},
    {ANALOG_INPUT1_TEST,ANALOG_INPUT1_TEST_SIZE,&analog_input1_test},
    {ANALOG_INPUT2_TEST,ANALOG_INPUT2_TEST_SIZE,&analog_input2_test},
    {ANALOG_INPUT3_TEST,ANALOG_INPUT3_TEST_SIZE,&analog_input3_test},
    {ANALOG_INPUT4_TEST,ANALOG_INPUT4_TEST_SIZE,&analog_input4_test},
};
#define    TOTAL_ATTRIBUTES    (sizeof(attributes)/sizeof(struct attributes_t))

size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to )
{
    void * from = NULL;
    size_t size = 0;
    uint8_t i;

    // сначала обработка общих атрибутов
    size = common_attr_get(&common_attrs_32771, attr_num, to);
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
    size = common_attr_set(&common_attrs_32771, attr_num, from);
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
    _attr_on_change(OBJOFFSET, &common_attrs_32771, attr_num);
    return size;
}
REG_ATTR_FUNC( attr_set );

size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num )
{
    uint8_t i;
    // сначала обработка общих атрибутов
    size_t size = common_attr_size(&common_attrs_32771, attr_num);
    if (size != 0)
        return size;
    
    for( i=0; i<TOTAL_ATTRIBUTES; i++ ) 
        if( attributes[i].num == attr_num )
            return attributes[i].size;

    return 0;
}
REG_ATTR_FUNC( attr_size );

void init_obj_32771()
{
    common_attrs_init(&common_attrs_32771);
}

INIT(init_obj_32771);
    
