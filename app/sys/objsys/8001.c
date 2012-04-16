
#include    <_zigzag.h>
#include    <syszig.h>
#include    <infomem.h>
#include    <zzTypes.h>
#include    <8001.h>

#define     OBJOFFSET    onum2offset(OBJ)

size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to );
size_t  ATTR_FUNC( attr_set )( const uint8_t attr_num, const void *const from );
size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num );
common_attr_t common_attrs_32769;

uint16_t smoke_density;
uint16_t smoke_density_thr_active_levels = 15;
uint16_t smoke_density_thr_triggered_levels;
uint16_t smoke_density_upper_thr_pre_fire_day = 120;
uint16_t smoke_density_lower_thr_pre_fire_day = 100;
uint16_t smoke_density_upper_thr_fire_day = 170;
uint16_t smoke_density_lower_thr_fire_day = 150;
uint16_t smoke_density_upper_thr_pre_fire_night = 70;
uint16_t smoke_density_lower_thr_pre_fire_night = 50;
uint16_t smoke_density_upper_thr_fire_night = 120;
uint16_t smoke_density_lower_thr_fire_night = 100;
uint16_t failure;
uint16_t failure_thr_active_levels = 255;
uint16_t failure_thr_triggered_levels;
uint16_t failure_upper_thr_pre_contamination = 140;
uint16_t failure_lower_thr_pre_contamination = 100;
uint16_t failure_upper_thr_contamination = 170;
uint16_t failure_lower_thr_contamination = 120;
uint16_t failure_upper_thr_pre_failure = -100;
uint16_t failure_lower_thr_pre_failure = -140;
uint16_t failure_upper_thr_failure = -120;
uint16_t failure_lower_thr_failure = -170;
uint16_t current_state;
uint16_t day_time = 480;
uint16_t night_time = 1200;
static uint16_t background_signal __attribute__ ((unused,__section__ (".module_infomem" ))) = 0x00D2;
static uint16_t full_range_signal __attribute__ ((unused,__section__ (".module_infomem" ))) = 0x0741;
static uint16_t contamination_offset __attribute__ ((unused,__section__ (".module_infomem" ))) = 0;
static uint16_t max_contamination_offset __attribute__ ((unused,__section__ (".module_infomem" ))) = 0x07ED;
static uint16_t min_contamination_offset __attribute__ ((unused,__section__ (".module_infomem" ))) = 0xFF6E;
uint8_t failure_indication_mode = 0xFF;
uint8_t alarm_indication_mode = 0x7F;

static const struct attributes_t {
    uint8_t     num;
    uint8_t     size;
    void        *ptr;
} attributes[] = {
    {SMOKE_DENSITY,SMOKE_DENSITY_SIZE,&smoke_density},
    {SMOKE_DENSITY_THR_ACTIVE_LEVELS,SMOKE_DENSITY_THR_ACTIVE_LEVELS_SIZE,&smoke_density_thr_active_levels},
    {SMOKE_DENSITY_THR_TRIGGERED_LEVELS,SMOKE_DENSITY_THR_TRIGGERED_LEVELS_SIZE,&smoke_density_thr_triggered_levels},
    {SMOKE_DENSITY_UPPER_THR_PRE_FIRE_DAY,SMOKE_DENSITY_UPPER_THR_PRE_FIRE_DAY_SIZE,&smoke_density_upper_thr_pre_fire_day},
    {SMOKE_DENSITY_LOWER_THR_PRE_FIRE_DAY,SMOKE_DENSITY_LOWER_THR_PRE_FIRE_DAY_SIZE,&smoke_density_lower_thr_pre_fire_day},
    {SMOKE_DENSITY_UPPER_THR_FIRE_DAY,SMOKE_DENSITY_UPPER_THR_FIRE_DAY_SIZE,&smoke_density_upper_thr_fire_day},
    {SMOKE_DENSITY_LOWER_THR_FIRE_DAY,SMOKE_DENSITY_LOWER_THR_FIRE_DAY_SIZE,&smoke_density_lower_thr_fire_day},
    {SMOKE_DENSITY_UPPER_THR_PRE_FIRE_NIGHT,SMOKE_DENSITY_UPPER_THR_PRE_FIRE_NIGHT_SIZE,&smoke_density_upper_thr_pre_fire_night},
    {SMOKE_DENSITY_LOWER_THR_PRE_FIRE_NIGHT,SMOKE_DENSITY_LOWER_THR_PRE_FIRE_NIGHT_SIZE,&smoke_density_lower_thr_pre_fire_night},
    {SMOKE_DENSITY_UPPER_THR_FIRE_NIGHT,SMOKE_DENSITY_UPPER_THR_FIRE_NIGHT_SIZE,&smoke_density_upper_thr_fire_night},
    {SMOKE_DENSITY_LOWER_THR_FIRE_NIGHT,SMOKE_DENSITY_LOWER_THR_FIRE_NIGHT_SIZE,&smoke_density_lower_thr_fire_night},
    {FAILURE,FAILURE_SIZE,&failure},
    {FAILURE_THR_ACTIVE_LEVELS,FAILURE_THR_ACTIVE_LEVELS_SIZE,&failure_thr_active_levels},
    {FAILURE_THR_TRIGGERED_LEVELS,FAILURE_THR_TRIGGERED_LEVELS_SIZE,&failure_thr_triggered_levels},
    {FAILURE_UPPER_THR_PRE_CONTAMINATION,FAILURE_UPPER_THR_PRE_CONTAMINATION_SIZE,&failure_upper_thr_pre_contamination},
    {FAILURE_LOWER_THR_PRE_CONTAMINATION,FAILURE_LOWER_THR_PRE_CONTAMINATION_SIZE,&failure_lower_thr_pre_contamination},
    {FAILURE_UPPER_THR_CONTAMINATION,FAILURE_UPPER_THR_CONTAMINATION_SIZE,&failure_upper_thr_contamination},
    {FAILURE_LOWER_THR_CONTAMINATION,FAILURE_LOWER_THR_CONTAMINATION_SIZE,&failure_lower_thr_contamination},
    {FAILURE_UPPER_THR_PRE_FAILURE,FAILURE_UPPER_THR_PRE_FAILURE_SIZE,&failure_upper_thr_pre_failure},
    {FAILURE_LOWER_THR_PRE_FAILURE,FAILURE_LOWER_THR_PRE_FAILURE_SIZE,&failure_lower_thr_pre_failure},
    {FAILURE_UPPER_THR_FAILURE,FAILURE_UPPER_THR_FAILURE_SIZE,&failure_upper_thr_failure},
    {FAILURE_LOWER_THR_FAILURE,FAILURE_LOWER_THR_FAILURE_SIZE,&failure_lower_thr_failure},
    {CURRENT_STATE,CURRENT_STATE_SIZE,&current_state},
    {DAY_TIME,DAY_TIME_SIZE,&day_time},
    {NIGHT_TIME,NIGHT_TIME_SIZE,&night_time},
    {BACKGROUND_SIGNAL,BACKGROUND_SIGNAL_SIZE,&background_signal},
    {FULL_RANGE_SIGNAL,FULL_RANGE_SIGNAL_SIZE,&full_range_signal},
    {CONTAMINATION_OFFSET,CONTAMINATION_OFFSET_SIZE,&contamination_offset},
    {MAX_CONTAMINATION_OFFSET,MAX_CONTAMINATION_OFFSET_SIZE,&max_contamination_offset},
    {MIN_CONTAMINATION_OFFSET,MIN_CONTAMINATION_OFFSET_SIZE,&min_contamination_offset},
    {FAILURE_INDICATION_MODE,FAILURE_INDICATION_MODE_SIZE,&failure_indication_mode},
    {ALARM_INDICATION_MODE,ALARM_INDICATION_MODE_SIZE,&alarm_indication_mode},
};
#define    TOTAL_ATTRIBUTES    (sizeof(attributes)/sizeof(struct attributes_t))

size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to )
{
    void * from = NULL;
    size_t size = 0;
    uint8_t i;

    // сначала обработка общих атрибутов
    size = common_attr_get(&common_attrs_32769, attr_num, to);
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
    size = common_attr_set(&common_attrs_32769, attr_num, from);
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
    _attr_on_change(OBJOFFSET, &common_attrs_32769, attr_num);
    return size;
}
REG_ATTR_FUNC( attr_set );

size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num )
{
    uint8_t i;
    // сначала обработка общих атрибутов
    size_t size = common_attr_size(&common_attrs_32769, attr_num);
    if (size != 0)
        return size;
    
    for( i=0; i<TOTAL_ATTRIBUTES; i++ ) 
        if( attributes[i].num == attr_num )
            return attributes[i].size;

    return 0;
}
REG_ATTR_FUNC( attr_size );

void init_obj_32769()
{
    common_attrs_init(&common_attrs_32769);
}

INIT(init_obj_32769);
    
