#include    <syszig.h>

module ZigHTimerM {
    provides {
        interface StdControl;
    }
    uses {

        interface MSP430Timer as Timer;

        interface MSP430TimerControl as AlarmControl0;
        interface MSP430Compare as AlarmCompare0;

        interface MSP430TimerControl as AlarmControl1;
        interface MSP430Compare as AlarmCompare1;

        interface MSP430TimerControl as AlarmControl2;
        interface MSP430Compare as AlarmCompare2;

    }
} implementation {

enum { 
    ZH_TIMER_TOTAL = 3,
    ZH_COUNTER_MAX = 0xffff,
    ZH_COUNTER_MIN = 2  
};

norace struct {
   uint32_t     tpoint;
   uint32_t     remain;
   uint8_t      is_set;
} zh_timer[ ZH_TIMER_TOTAL ]; 

uint16_t    hi_counter;

void setControlAsCompare( const uint8_t tnum );
void disableEvents( const uint8_t tnum );
void enableEvents( const uint8_t tnum );
void clearPendingInterrupt( const uint8_t tnum );
void setEvent( const uint8_t tnum, const uint16_t jiffy);

async event void Timer.overflow()
{
    uint8_t i;
    uint16_t tar = 0;
    for( i = 0; i < ZH_TIMER_TOTAL; i++ ) {
       if( FALSE == zh_timer[i].is_set ) continue;
       tar = TAR+1; 
       if( 0 == zh_timer[i].remain )
          *(((uint16_t *)&TACCTL0)+i) |= 1;
       else
          zh_timer[i].remain -= 1;
       if( tar < *(((uint16_t *)&TACCR0)+i) )
          clearPendingInterrupt(i);
       if( 0 == zh_timer[i].remain )
          enableEvents(i);
    }
    ++hi_counter;  
    while( TBR < tar );
    return;
}

uint16_t    timer_overflow()
{
    uint16_t ot;

    atomic {
        ot = hi_counter;        
        if( call Timer.isOverflowPending() )
            ++ot;
    }
    
    return ot;
    
}

uint32_t    __atimer_counter() @spontaneous() @C() 
{
    uint16_t _hi_counter, _lo_counter;

    atomic {
        _lo_counter = call Timer.read();
        _hi_counter = timer_overflow();
    }
    
    return ((uint32_t)_hi_counter << 16) | _lo_counter;
}

int16_t     __atimer_set( const uint8_t tnum, uint32_t tpoint ) @spontaneous() @C()
{
    uint32_t hi_tpoint, ot;    

    if( ZH_TIMER_TOTAL <= tnum )
        return -1;

    disableEvents( tnum );
    
    clearPendingInterrupt( tnum );
    setEvent( tnum, tpoint & 0xffff );
    ot = timer_overflow();
    hi_tpoint = (uint32_t)tpoint >> 16;
    if( ot <= hi_tpoint )
       zh_timer[ tnum ].remain = hi_tpoint - ot;
    else 
       zh_timer[ tnum ].remain = ( hi_tpoint + ZH_COUNTER_MAX + 1 ) - ot;

    zh_timer[ tnum ].tpoint = tpoint;
    zh_timer[ tnum ].is_set = 1;   

    if( 0 == zh_timer[ tnum ].remain )
       enableEvents( tnum );
    return 0; 
}

int16_t     __atimer_stop( const uint8_t tnum ) @spontaneous() @C()
{
    if( ZH_TIMER_TOTAL <= tnum )
        return -1;

    atomic {
       zh_timer[ tnum ].is_set = 0;
       disableEvents( tnum );  
    }

    return 0;
}  
 
int16_t     __atimer_is_set( const uint8_t tnum ) @spontaneous() @C()
{
    if( ZH_TIMER_TOTAL <= tnum )
        return -1;

    return zh_timer[ tnum ].is_set;
}

uint32_t    __atimer_point( const uint8_t tnum ) @spontaneous() @C()
{
    if( ZH_TIMER_TOTAL <= tnum )
        return 0;

    return zh_timer[ tnum ].tpoint;
}

void timerfired( const uint8_t tnum )
   {
    __atimer_stop( tnum );
    if( ISMODLOAD )
        __atimer_fired( tnum );
    return;
   }

      
command result_t StdControl.init()
   {
    uint8_t i;
    for( i=0; i < ZH_TIMER_TOTAL; i++ ) {
       setControlAsCompare( i );
       disableEvents( i );
    }
    return SUCCESS;
   }

command result_t StdControl.start()
   {
    uint8_t i;
    for( i = 0; i < ZH_TIMER_TOTAL; i++ ) {
       zh_timer[ i ].is_set = FALSE;
       disableEvents( i ); 
    }
    return SUCCESS;
   }

command result_t StdControl.stop()
   {
    uint8_t i;
    for( i = 0; i < ZH_TIMER_TOTAL; i++ ) {
       zh_timer[ i ].is_set = FALSE;
       disableEvents( i ); 
    }
    return SUCCESS;
   }

void setControlAsCompare( const uint8_t tnum )
{
    switch( tnum ) {
       case 0: call AlarmControl0.setControlAsCompare(); return;
       case 1: call AlarmControl1.setControlAsCompare(); return;
       case 2: call AlarmControl2.setControlAsCompare(); return;
    }
}

void disableEvents( const uint8_t tnum )
{
    switch( tnum ) {
       case 0: call AlarmControl0.disableEvents(); return;
       case 1: call AlarmControl1.disableEvents(); return;
       case 2: call AlarmControl2.disableEvents(); return;
    }
}
   
void enableEvents( const uint8_t tnum )
{
    switch( tnum ) {
       case 0: call AlarmControl0.enableEvents(); return;
       case 1: call AlarmControl1.enableEvents(); return;
       case 2: call AlarmControl2.enableEvents(); return;
    }
}

void clearPendingInterrupt( const uint8_t tnum )
{
    switch( tnum ) {
       case 0: call AlarmControl0.clearPendingInterrupt(); return;
       case 1: call AlarmControl1.clearPendingInterrupt(); return;
       case 2: call AlarmControl2.clearPendingInterrupt(); return;
    }
}
   
void setEvent( const uint8_t tnum, const uint16_t jiffy )
{
    switch( tnum ) {
       case 0: call AlarmCompare0.setEvent( jiffy ); return;
       case 1: call AlarmCompare1.setEvent( jiffy ); return;
       case 2: call AlarmCompare2.setEvent( jiffy ); return;
    }
}

async event void AlarmCompare0.fired() {  timerfired( 0 ); }
async event void AlarmCompare1.fired() {  timerfired( 1 ); }
async event void AlarmCompare2.fired() {  timerfired( 2 ); }

}

