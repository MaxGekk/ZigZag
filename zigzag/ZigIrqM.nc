#include <syszig.h>

module ZigIrqM {
   provides interface StdControl;
   uses {

      interface MSP430Interrupt as Port10;
      interface MSP430Interrupt as Port11;
      interface MSP430Interrupt as Port12;
      interface MSP430Interrupt as Port13;
      interface MSP430Interrupt as Port14;
      interface MSP430Interrupt as Port15;
      interface MSP430Interrupt as Port16;
      interface MSP430Interrupt as Port17;

      interface MSP430Interrupt as Port20;
      interface MSP430Interrupt as Port21;
      interface MSP430Interrupt as Port22;
      interface MSP430Interrupt as Port23;
      interface MSP430Interrupt as Port24;
      interface MSP430Interrupt as Port25;
      interface MSP430Interrupt as Port26;
      interface MSP430Interrupt as Port27;

   }
} implementation {

#define MOD_HANDLER(a) { if( ISMODLOAD ) __process_irq(a); }

async event void Port10.fired() { MOD_HANDLER( PORT1_VECTOR ) }
async event void Port11.fired() { MOD_HANDLER( PORT1_VECTOR ) }
async event void Port12.fired() { MOD_HANDLER( PORT1_VECTOR ) }
async event void Port13.fired() { MOD_HANDLER( PORT1_VECTOR ) }
async event void Port14.fired() { MOD_HANDLER( PORT1_VECTOR ) }
async event void Port15.fired() { MOD_HANDLER( PORT1_VECTOR ) }
async event void Port16.fired() { MOD_HANDLER( PORT1_VECTOR ) }
async event void Port17.fired() { MOD_HANDLER( PORT1_VECTOR ) }

async event void Port20.fired() { MOD_HANDLER( PORT2_VECTOR ) }
async event void Port21.fired() { MOD_HANDLER( PORT2_VECTOR ) }
async event void Port22.fired() { MOD_HANDLER( PORT2_VECTOR ) }
async event void Port23.fired() { MOD_HANDLER( PORT2_VECTOR ) }
async event void Port24.fired() { MOD_HANDLER( PORT2_VECTOR ) }
async event void Port25.fired() { MOD_HANDLER( PORT2_VECTOR ) }
async event void Port26.fired() { MOD_HANDLER( PORT2_VECTOR ) }
async event void Port27.fired() { MOD_HANDLER( PORT2_VECTOR ) }

TOSH_SIGNAL(DACDMA_VECTOR) { MOD_HANDLER( DACDMA_VECTOR ) }
TOSH_SIGNAL(ADC12_VECTOR) { MOD_HANDLER( ADC12_VECTOR ) }
TOSH_SIGNAL(COMPARATORA_VECTOR) { MOD_HANDLER( COMPARATORA_VECTOR ) }

#if !defined( BUSY_USART0 )
TOSH_SIGNAL(UART0RX_VECTOR) { MOD_HANDLER( UART0RX_VECTOR ) }
TOSH_SIGNAL(UART0TX_VECTOR) { MOD_HANDLER( UART0TX_VECTOR ) }
#endif

#if !defined( BUSY_USART1 )
TOSH_SIGNAL(UART1RX_VECTOR) { MOD_HANDLER( UART1RX_VECTOR ) }
TOSH_SIGNAL(UART1TX_VECTOR) { MOD_HANDLER( UART1TX_VECTOR ) }
#endif

command result_t StdControl.init() { return SUCCESS; }
command result_t StdControl.start()
   {
    /*   
    call Port10.disable(); call Port11.disable();
    call Port12.disable(); call Port13.disable();
    call Port14.disable(); call Port15.disable();
    call Port16.disable(); call Port17.disable(); 

    call Port20.disable(); call Port21.disable();
    call Port22.disable(); call Port23.disable();
    call Port24.disable(); call Port25.disable();
    call Port26.disable(); call Port27.disable(); 
    */
    DAC12_0CTL &= 0xfff7;
    CACTL1 &= 0xfd;
    ADC12CTL0 &= 0xfff3;

    return SUCCESS;
   }
command result_t StdControl.stop() { return SUCCESS; }

}

