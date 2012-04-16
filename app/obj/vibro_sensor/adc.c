#include <msp430x16x.h>
#include "adc.h"


uint16_t adc_result; // Storage for last conversion results

void init_adc(void)
{
   P6SEL  =  0x36; // P6.1, P6.2, P6.4, P6.5 - ADC inputs, all others are digital I/O
   P6DIR  =  0xC9; // P6.1, P6.2, P6.4, P6.5 - inputs, all others - outputs 
   adc_result = 0; // Reset consversion result

}

int init_ADXLMeasurement(uint8_t forceStop, uint8_t channel)
{
   int res = ADC_RES_OK;

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

   enable_ADXL(1);
   // Prepare ADC12 for conversion and sleep until reference generator to stabilize
   adc12.ctl0.enc = 0; // Reset ENC bit to be able to change neccessary fields of following registers
   ADC12CTL0  = ADC12ON | SHT0_2 | REFON | REF2_5V; // Set Sample and hold timer to 16 clocks, internal reference 2.5V
   ADC12IFG   = 0x00; // Clear all ADC12MEMx interupt flags
   if(channel == X_IDX)
   {
      ADC12CTL1 = CSTARTADD_4 | SHS_0 | SHP | ADC12DIV_0 | ADC12SSEL_0 | CONSEQ_0; // Start from ADC12MEM4, SH from ADC12SC, ADC12OSC, single channel
      ADC12MCTL4 = EOS + SREF_1 + INCH_4; // Set EOS for this register, select range from VSS to VREF+, channel A4 (P6.4) - X channel
      ADC12IE   = 0x10; // Interrupt for ADC12MEM4
   }
   else
   {
      ADC12CTL1 = CSTARTADD_5 | SHS_0 | SHP | ADC12DIV_0 | ADC12SSEL_0 | CONSEQ_0; // Start from ADC12MEM5, SH from ADC12SC, ADC12OSC, single channel
      ADC12MCTL5 = EOS + SREF_1 + INCH_5; // Set EOS for this register, select range from VSS to VREF+, channel A5 (P6.5) - Y channel
      ADC12IE   = 0x20; // Interrupt for ADC12MEM5
   }

   // ADC12 is now prepared for conversion. We should wait for Reference Generator to stabilize for about 17 ms.
   // It's caller responsibility to wait or sleep while expecting Ref stabilization
   // and then to initialize A/D conversion

   return res;
}

// Starts single conersion if ADC isn't busy
int start_singleConversion(void)
{
   // If Reference Generator is used ADC12 is expected to be ON
   if(adc12.ctl0.refon && !adc12.ctl0.adc12on)
      return ADC_RES_FAIL;
   if(adc12.ctl1.adc12busy)
      return ADC_RES_BUSY;
   ADC12CTL0 |= ENC | ADC12SC;
   return ADC_RES_OK;
}

// Stops ADC and shuts down the Reference Generator
int adc_shutdown(uint8_t forceStop)
{
   if(adc12.ctl1.adc12busy)
   {
      // If conversion is underway we may stop it or wait for it to complete
      // It's up to the user to make proper choice
      if(!forceStop)
         return ADC_RES_BUSY;
   }

   adc12.ctl0.enc = 0;
   ADC12CTL0 &= ~(REFON | ADC12ON | ADC12SC);

   return ADC_RES_OK;
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
      return ADC12MEM0;
   case 0x08: // ADC12MEM1 flag
      return ADC12MEM1;
   case 0x0E:
      return ADC12MEM4;
   case 0x10:
      return ADC12MEM5;
   default:
      return 0; // Unsupported vector
   }
}

// Enables or disables adxl sensor
void enable_ADXL(uint8_t enable)
{
   
   if(!enable)
      P6OUT &= ~0x08; // Enable ADXL 
   else
      P6OUT |=  0x08; // Disable ADXL
}

