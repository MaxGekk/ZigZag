#ifndef     __ZZ_EVENT_H
#define     __ZZ_EVENT_H

#include    <zzTypes.h>
#include    <_zzMacro.h>

typedef void    (* event_handler_ft)( event_type_t, unidata_t );

extern const event_handler_ft   _begin_event_handler;
extern const event_handler_ft   _end_event_handler;

result_t   _event_emit( const uint16_t obj_offset, 
        priority_t  priority, event_type_t  event_type, unidata_t   unidata );

#endif  /*  __ZZ_EVENT_H */

