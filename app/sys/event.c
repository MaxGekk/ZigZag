#include    <config.h>
#include    <_zigzag.h>
#include    <syszig.h>

/* Очередь событий */
struct {
   unidata_t    unidata;
   uint16_t     obj_offset;     
   priority_t   priority;
   event_type_t     event_type;
   bool_t     free;
} ev_queue[ EVENT_QUEUE_SIZE ];

__attribute__((used)) void __event_handler()
{
    uint8_t max_pr_index = EVENT_QUEUE_SIZE;
    priority_t priority ,max_priority = 0;
    uint8_t i;
    bool_t free_place;

    for(i=0;i<EVENT_QUEUE_SIZE;i++) {
        priority = ev_queue[i].priority;
        free_place = ev_queue[i].free;
        if( (FALSE == free_place ) && ( priority >= max_priority ) ) {
            max_pr_index = i;
            max_priority = priority; 
        }
    }
    if( max_pr_index < EVENT_QUEUE_SIZE ) {
        event_type_t event_type;
        unidata_t unidata;
        event_type = ev_queue[ max_pr_index ].event_type;
        unidata = ev_queue[ max_pr_index ].unidata; 
        ev_queue[ max_pr_index ].free = TRUE;
        { /* Вызов обработчика события соответствующего объекта */
         const event_handler_ft *eh = &_begin_event_handler;
         if( ev_queue[ max_pr_index ].obj_offset == ALL_OBJOFFSET ) {
            for(; eh < &_end_event_handler; eh++ )   
                (*eh)( event_type, unidata );         
         } else {
            eh += ev_queue[ max_pr_index ].obj_offset;
            (*eh)( event_type, unidata );         
         }
        }
    }
    return;
}

result_t   _event_emit( uint16_t obj_offset, priority_t  priority, 
        event_type_t  event_type, unidata_t   unidata )
{
    uint8_t i;
    bool_t free_place;
    for(i=0;i<EVENT_QUEUE_SIZE;i++) {

        if(ISZIGLOAD)   __critical_enter();
        else    return ENOSYS;

        free_place = ev_queue[i].free;
        if( TRUE == free_place )
            ev_queue[i].free = FALSE;

        __critical_exit();  /* Загружен ли ZigZag проверено выше */

        if( TRUE == free_place ) {
            ev_queue[i].event_type = event_type;  
            ev_queue[i].unidata = unidata;
            ev_queue[i].obj_offset = obj_offset;
            ev_queue[i].priority = priority;
            break;
        }
    }
    if( EVENT_QUEUE_SIZE == i )
        return   ENOMEM;
    if( IS_ERROR( __post_task( &__event_handler ) ) ) { 
        ev_queue[i].free = TRUE;
        return  ENOMEM;
    }
    return  ENOERR;
}

#ifdef  ZC_SLEEP_NOTIFY
int16_t     __net_sleep( uint32_t duration )
{
    _event_emit( ALL_OBJOFFSET, PRIORITY_LOW, EV_SLEEP, duration );
    return 0;
}
#endif

void ev_queue_init()
{
    uint8_t i = 0;
    for(;i<EVENT_QUEUE_SIZE;i++)
        ev_queue[i].free = 1;
    return;
}
INIT( ev_queue_init );

__attribute__((weak)) void    event_handler( event_type_t event_type, unidata_t   unidata ) {}

