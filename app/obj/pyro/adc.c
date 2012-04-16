#include <msp430x16x.h>
#include "adc.h"

uint16_t adc_result; // Storage for last conversion results

void init_adc(void)
{
   P4SEL &= ~0x08; // P4.0 as digital I/O
   P4DIR |=  0x08; // P4.0 (PHOTO_EN) as output
   P4OUT |=  0x08; // Switch off Photodiode
   enable_pyro(0);

}


int init_PyroMeasurement(uint8_t forceStop)
{
   int res = ADC_RES_OK;

   enable_pyro(1);

   if(adc12.ctl1.adc12busy)
   {
      if(forceStop)
      {
         adc12.ctl0.enc = 0;
         adc12.ctl0.adc12sc = 0;
      }
      else
         return ADC_RES_BUSY;
   }

   // Prepare ADC12 for conversion and sleep until reference generator to stabilize
   adc12.ctl0.enc = 0; // Reset ENC bit to be able to change neccessary fields of following registers
   ADC12CTL0  = ADC12ON | SHT0_2 | ADC12SC | ADC12OVIE | ADC12TOVIE; // Set Sample and hold timer to 32 clocks
   ADC12CTL1  = CSTARTADD_0 | SHS_0 | SHP | ADC12DIV_0 | ADC12SSEL_0 | CONSEQ_0; // Start from ADC12MEM0, SH from ADC12SC, ADC12OSC, single conversion
   ADC12MCTL0 = EOS + SREF_0 + INCH_1; // Set EOS for this register, select range from VSS to AVCC, channel A2 (P6.1)
   ADC12IFG   = 0x00; // Clear all ADC12MEMx interupt flags
   ADC12IE    = 0x01; // Interrupt for ADC12MEM0

   // ADC12 is now prepared for conversion. Let's start it
   ADC12CTL0 |= ENC | ADC12SC;// adc12.ctl0.enc = 1;

   return res;
}

// This function is used for handling ADC interrupts
// If ADC interrupt is enabled and used it should be called only
// from system interrupt handler (not strict condition)
inline uint16_t adc_int_handler(uint16_t adc_vector)
{
   switch(adc_vector)
   {
   case 0x00: // No interrupt pending... Something strange
      return 0 ;
   case 0x02: // ADC12MEMx overflow
      return ADC_RES_OVERFLOW; // I haven't decided yet what to do
   case 0x04: // Conversion time overflow
      return ADC_RES_TIMEOVERFLOW; // I haven't decided yet what to do
   case 0x06: // ADC12MEM1 flag
      return  ADC12MEM0;
   case 0x08: // ADC12MEM1 flag
      return  ADC12MEM1;
   default:
      return 0; // Unsupported vector
   }
}

void enable_pyro(uint8_t enable)
{
   if(enable)
      P4OUT &= ~0x08; // Enable pyroelectric sensor 
   else
      P4OUT |=  0x08; // Disable pyroelectric sensor
}

