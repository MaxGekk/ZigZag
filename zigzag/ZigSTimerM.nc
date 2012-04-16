#include    <syszig.h>

module  ZigSTimerM {
    uses {
        interface TimerMilli as Timer0;
        interface TimerMilli as Timer1;
        interface TimerMilli as Timer2;    
    }
} implementation {

int16_t __stimer_set( const uint8_t timer_num, const uint32_t   milli_sec ) @spontaneous() @C()
{
    result_t res;
    switch( timer_num ) {
        case 0: res = call Timer0.setOneShot( milli_sec ); break;
        case 1: res = call Timer1.setOneShot( milli_sec ); break;  
        case 2: res = call Timer2.setOneShot( milli_sec ); break; 
        default: res = FAIL;
    }
    if( SUCCESS == res )
       return 0;
    return -1;
}


event result_t Timer0.fired()
{
    if( ISMODLOAD )
        __stimer_fired( 0 );
    return SUCCESS;
}

event result_t Timer1.fired()
{
    if( ISMODLOAD )
        __stimer_fired( 1 );
    return SUCCESS;
}

event result_t Timer2.fired()
{
    if( ISMODLOAD )
        __stimer_fired( 2 );
    return SUCCESS;
}

}

