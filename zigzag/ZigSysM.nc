#include    <syszig.h>

module  ZigSysM {
    provides interface   StdControl;
    uses interface   ILocalTime64;
} implementation {

#ifdef  ZC_PORT
static const uint8_t en_mask[ PORT_TOTAL ] = { ZC_PORT1, ZC_PORT2, ZC_PORT3, ZC_PORT4, ZC_PORT5, ZC_PORT6 };

int16_t __port_perm( const uint8_t port_num, const uint8_t mask, const uint8_t op ) @spontaneous() @C()
{
    if( PORT_TOTAL < port_num )
        return -1;
    if( ( en_mask[ port_num-1 ] | mask ) ^ en_mask[ port_num-1 ] )
        return -1;
    return 0;
}
#endif

#ifdef  ZC_TIME64
uint64_t __sys_time( ) @spontaneous() @C()
{
    return (uint64_t)(call ILocalTime64.getLocalTime());
}

int16_t __set_sys_time( uint64_t time )@spontaneous() @C()
{
    call ILocalTime64.setLocalTime( time );
    return 0;
}
#endif

task void   task_app_init()
{
    if( ISMODLOAD )
        __app_init();
    return;
}

bool TOS_post(void (*tp) ()) @spontaneous() @C();
int16_t __post_task( task_ft task_func ) @spontaneous() @C()
{
    if( TRUE == TOS_post( task_func ) )
        return 0;
    return -1;
}

static volatile __nesc_atomic_t reenable_interrupt;
void __critical_enter() @spontaneous() @C()
{
    reenable_interrupt = __nesc_atomic_start();
    return;
}

void __critical_exit() @spontaneous() @C()
{
    __nesc_atomic_end( reenable_interrupt );
    return;
}

command result_t    StdControl.init() { return SUCCESS; }    
command result_t    StdControl.start()
{
#ifdef  ZC_INIT
    post    task_app_init();    
#endif
    return SUCCESS;
}

command result_t    StdControl.stop() { return SUCCESS; }    

}
