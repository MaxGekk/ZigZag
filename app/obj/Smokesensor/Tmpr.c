#include  "msp430x16x.h"
#include "typedefs.h"
#include "Tmpr.h"

uint TemperatureScan(void)
{
  uint Tmpr;
	ADC12CTL0 |= (SHT0_8 + ADC12ON);       // Reconfigure ADC Module for Temp Sensor Scan
  ADC12CTL1 |= SHP;                      // Enable sample timer
  ADC12MCTL0 = SREF_1+INCH_10;           // Vr+=Vref+, Input Channel 0x0A
  ADC12CTL0 |= ENC;                      // Enable conversions
  ADC12CTL0 |= ADC12SC;                  // Start conversion
  while ((ADC12IFG & BIT0)==0);
  Tmpr=ADC12MEM0;
  ADC12CTL0 &= ~ENC;                     // Disable conversions
  ADC12CTL0 &= ~(SHT0_8 + ADC12ON);      // Turn Off ADC12 Module
  ADC12CTL1 &= ~SHP;                     // Disable sample timer
	return Tmpr;
}

void TemperatureProcessing(uint* Tmpr)
{
}

void TmprInit(void)
{
}
