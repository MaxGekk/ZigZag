#include <syszig.h>

module  ZigTimerAIrqM   {
    uses {
        interface MSP430Timer as Timer;
        interface MSP430Compare as Compare0;
        interface MSP430Compare as Compare1;
        interface MSP430Compare as Compare2;
        interface MSP430Capture as Capture0;
        interface MSP430Capture as Capture1;
        interface MSP430Capture as Capture2;
    }
} implementation {

#define MOD_HANDLER(a) { if( ISMODLOAD ) __process_irq(a); }

async event void Timer.overflow() { MOD_HANDLER( TIMERA1_VECTOR ) }
async event void Compare0.fired() { MOD_HANDLER( TIMERA0_VECTOR ) }
async event void Compare1.fired() { MOD_HANDLER( TIMERA1_VECTOR ) }
async event void Compare2.fired() { MOD_HANDLER( TIMERA1_VECTOR ) }
async event void Capture0.captured(uint16_t time) { MOD_HANDLER( TIMERA0_VECTOR ) }
async event void Capture1.captured(uint16_t time) { MOD_HANDLER( TIMERA1_VECTOR ) }
async event void Capture2.captured(uint16_t time) { MOD_HANDLER( TIMERA1_VECTOR ) }

}

