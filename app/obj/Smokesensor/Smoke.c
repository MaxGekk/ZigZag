#include <msp430.h>
#include "typedefs.h"
#include "Smoke.h"
#include "RFSensor.h"
#include "biquad.h"
#include "Mem.h"

uchar	CntmCompTmr=0;
uchar	SumCompTmr=0;
uint  Samples[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
sint	SmokeSignal=0;
slong CntmCompVal=0;
uchar	SmokeState=SmokeDutyState;

const TSmokeParam IEESmokeParam =
{
	0x0200,									//	sint	InitSignal;			//Initial Signal (Nadc)
	0x0500, 								//	sint	FRngOffset;			//Full Range Offset == 100% == 0,2dB/m (Nadc)
	0x0000, 								//	sint	CntmOffset;			//Contamination Offset (Nadc)
	0x0800, 								//	sint	MaxCntmOffset;	//Maximal Contamination Offset (Nadc)
	-0x0100,								//	sint	MinCntmOffset;	//Minimal Contamination Offset (Nadc)
	0x0F00, 								//	sint	MaxSignal;			//Maximal Permissible Signal (Nadc)
	0x0100, 								//	sint	MinSignal;			//Minimal Permissible Signal (Nadc)
	0x0400, 								//	sint	MaxInitSignal;	//Maximal Permissible Initial Signal (Nadc)
	0x0180, 								//	sint	MinInitSignal;	//Minimal Permissible Initial Signal (Nadc)
	1321,										//	sint	TmprCompRef;		//Reference for computing Temperature compensation
	1613,										//	sint	TmprCompOffset;	//Offset for No temperature Compensation Mode
	2,											//	sint	CntmMaxUpd;			//Maximal Single Contamination Update Value
	160,										//	uchar	DAlarmThrld;		//Day Alarm Threshold (%)
	100,										//	uchar	DPreAlarmThrld;	//Day Prealarm Threshold (%)
	100,										//	uchar	NAlarmThrld;		//Night Alarm Threshold (%)
	80,											//	uchar	NPreAlarmThrld;	//Night Prealarm Threshold (%)
	160,										//	uchar	CntmThrld;			//Contamination Threshold (%)
	100,										//	uchar	PreCntmThrld;		//PreContamination Threshold (%)
	160,										//	uchar	FaultThrld;			//Fault Threshold (%)
	100,										//	uchar	PreFaultThrld;	//PreFault Threshold (%)
	10,											//	uchar	CntmUpdatePr;		//Contamination Update Period
	10,											//	uchar	SumUpdatePr;		//Integration Sum Update Period
	{1,1,1,1,1,1,1}					//	TSmokeCtrl Control;		//Control Register
};

void SmokeScan(uint* Tmpr)
{
  uchar i;
  uint DACVal;
	PinConfigure(SENS_PORT, SENS_BIT, DIR_OUT+PINVAL_LO+MODE_EXT+IRQ_DS+EDG_FIL);
	PinConfigure(IR_PLS_PORT, IR_PLS_BIT, DIR_OUT+PINVAL_LO+MODE_EXT+IRQ_DS+EDG_FIL);
  if(IEESmokeParam.Control.TmprCompEn)									// Compute Temperature Compensation Value
	{
		DACVal = (*Tmpr) - IEESmokeParam.TmprCompRef;
	}
	else DACVal=IEESmokeParam.TmprCompOffset;
  ADC12CTL0 |= (SHT0_2 + ADC12ON);											// Set sampling time, turn on ADC12
  ADC12CTL1 = SHP;																			// Use sampling timer
  ADC12MCTL0 = SREF_1+INCH_0;														// Vr+=Vref+, Input ADC Channel 0
  DAC12_0DAT=0;																					// Presetting DAC Value
  DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;					// Internal ref gain 1
  ADC12CTL0 |= ENC;																			// Enable conversions
  ADC12IFG &= ~BIT0;																		// Dummy Clear ADC Interrupt
  i=0;
	while (i<16)
  {
    if((i>4)&&(i<11)) DAC12_0DAT=DACVal;								// Set IR Pulse
    else DAC12_0DAT=0;
    ADC12CTL0 |= ADC12SC;																// Start conversion
    while ((ADC12IFG & BIT0)==0);												// Wait for End of Conversion
    Samples[i++]=ADC12MEM0;															// Store ADC Value
  }
  ADC12CTL0 &= ~ENC;																		// Disable conversions
  ADC12CTL0 &= ~(SHT0_2 + ADC12ON);											// Turn Off ADC12 Module
  ADC12CTL1 = 0;
  DAC12_0CTL &= ~DAC12ENC;															// Disable DAC conversions
  DAC12_0CTL = 0;					
	PinConfigure(SENS_PORT, SENS_BIT, DIR_OUT+PINVAL_LO+MODE_IO+IRQ_DS+EDG_FIL);
	PinConfigure(IR_PLS_PORT, IR_PLS_BIT, DIR_OUT+PINVAL_LO+MODE_IO+IRQ_DS+EDG_FIL);
}

void SmokeProcessing(void)
{
  uchar i;
	TSmokeParam SmokeParam;
	SmokeParam=IEESmokeParam;
  sint  AMax=-32767;
  sint  AMin=32767;
  i=0;
  while(i<16)																						//Filter Measurement Smoke Signal
	{																											//And Store Max and Min Values
		SmokeSignal=IIRFilter(Samples[i]);
    if( SmokeSignal > AMax ) AMax = SmokeSignal;
    if( SmokeSignal < AMin ) AMin = SmokeSignal;
		i++;
	}
  SmokeSignal=AMax-AMin;																//Compute Filtered Signal Amplitude
	if(SmokeParam.Control.CntmCompEn)
	{
		if(SumCompTmr++==SmokeParam.SumUpdatePr)
		{
			CntmCompVal+=(slong)SmokeSignal;									//Update Contamination Integral
			SumCompTmr=0;
			CntmCompTmr++;
		}
	}
	else
	{
		CntmCompVal=0;
		CntmCompTmr=0;
		SumCompTmr=0;
	}
	if(CntmCompTmr>=SmokeParam.CntmUpdatePr)							//Update If Need Contamination Value
	{
		if((SmokeState & SmokeAlarmMask) ==
			SmokeDutyState)																		//Check if Sensor is in Duty State
		{
			CntmCompVal/=(slong)CntmCompTmr;									//Compute Mid Amplitude Value on Long Integration Period
			CntmCompVal-=(slong)SmokeParam.InitSignal;				//Compute Current Contamination Offset
			AMin=((sint)CntmCompVal)-SmokeParam.CntmOffset;		//Compute subtraction of Current and last contamination Offset
			if(AMin<=-SmokeParam.CntmMaxUpd)									//Update Contamination Value only if it more than Threshold
				AMin=-SmokeParam.CntmMaxUpd;
			else if(AMin>=SmokeParam.CntmMaxUpd)
				AMin=SmokeParam.CntmMaxUpd;
			else AMin=0;
			if(AMin)
			{
				AMin=SmokeParam.CntmOffset+AMin;								//Compute New Contamination Offset
				if(AMin<=SmokeParam.MinCntmOffset)							//Check Contamination Offset on MIN Value
				{
					SmokeSetState((SmokeState&(~SmokeFaultMask))	//Sensor Absolute Fault Report
						|SmokeFaultState);												
				}
				else if (AMin>=SmokeParam.MaxCntmOffset)				//Check Contamination Offset on Max Value
				{
					SmokeSetState((SmokeState&(~SmokeFaultMask))	//Sensor Absolute High Contamination Report
						|SmokeCntmState);							
				}
				else
				{
					SmokeParam.CntmOffset=AMin;										//Set New Contamination Offset
					IEEWR((void*)&SmokeParam, 										//Store Contamination Offset
								(void*)&IEESmokeParam,			
								sizeof(SmokeParam));	
					if(AMin>=0)
					{
						AMax=(sint)((((slong)AMin)*200)/
							((slong)SmokeParam.MaxCntmOffset));				//Compute Contamination in percents
						if(AMax>SmokeParam.CntmThrld)
							SmokeSetState((SmokeState&								//Sensor Contamination Report
								(~SmokeFaultMask))		
								|SmokeCntmState);
						else if(AMax>SmokeParam.PreCntmThrld)
							SmokeSetState((SmokeState&
							(~SmokeFaultMask))|												//Sensor PreContamination Report
							SmokePreCntmState);
						else SmokeSetState(SmokeState&							//Sensor No Fault Report
							(~SmokeFaultMask));		
					}
					else if(AMin<0)
					{
						AMax=(sint)((((slong)AMin)*200)/
							((slong)SmokeParam.MinCntmOffset));				//Compute Fault in percents
						if(AMax>SmokeParam.FaultThrld)
						{
							SmokeSetState((SmokeState&								//Sensor Fault Report
								(~SmokeFaultMask))		
								|SmokeFaultState);
						}
						else if(AMax>SmokeParam.PreFaultThrld)
						{
							if(SmokeParam.Control.PreFaultEn)
							{
								SmokeSetState((SmokeState&							//Sensor PreFault Report
									(~SmokeFaultMask))		
									|SmokePreFaultState);
							}
							else
							{
								SmokeSetState(SmokeState&								//Sensor No Fault Report
									(~SmokeFaultMask));		
							}
						}
						else SmokeSetState(SmokeState&							//Sensor No Fault Report
							(~SmokeFaultMask));		
					}
				}
			}
			else SmokeSetState(SmokeState&										//Sensor No Fault Report
				(~SmokeFaultMask));		
		}
		CntmCompTmr=0;																			//Zeroing Integration Timer
		CntmCompVal=0;																			//Zeroing Long Time Integration Sum
		SumCompTmr=0;
	}
	switch(SmokeState&SmokeAlarmMask)
	{
		case SmokePreAlarmState:
		case SmokeDutyState:
			AMin=SmokeSignal;
			if(SmokeParam.Control.CntmCompEn)									//Adjust Filtered Signal with Contamination Offset
			{
				AMin-=SmokeParam.CntmOffset;
			}
			AMin-=SmokeParam.InitSignal;											//Compute Smoke Amplitude vs Initial Signal
			AMin=(sint)((((slong)AMin)*200)/
				((slong)SmokeParam.FRngOffset));								//Compute Percent representation of Smoke
			if(SmokeState&SmokeNightMode)
				AMax=SmokeParam.NAlarmThrld;										//Choose current Day/Night Mode Threshold
			else AMax=SmokeParam.DAlarmThrld;
			if(AMin>=AMax)																		//Compare Smoke with choosing threshold
				SmokeSetState((SmokeState&											//Sensor Alarm Report
					(~SmokeAlarmMask))		
					|SmokeAlarmState);
			else																							//If No Alarm, then Check for PreAlarm
			{
				if(SmokeParam.Control.PreAlarmEn)
				{
					if(SmokeState&SmokeNightMode)
						AMax=SmokeParam.NPreAlarmThrld;							//Choose current Day/Night Mode Threshold
					else AMax=SmokeParam.DPreAlarmThrld;
					if((SmokeState&SmokeAlarmMask)==
						SmokePreAlarmState)	AMax-=5;								//Set Hysteresis
					else AMax+=5;
					if(AMin>=AMax)																//Compare Smoke with choosing threshold
					{
						SmokeSetState((SmokeState&									//Sensor PreAlarm Report
							(~SmokeAlarmMask))		
							|SmokePreAlarmState);
					}
					else																					//If No PreAlarm
					{
						SmokeSetState((SmokeState&									//Sensor Duty Mode Report
							(~SmokeAlarmMask))		
							|SmokeDutyState);
					}
				}
			}
			break;
		case SmokeCleanState:
			AMin=SmokeSignal-SmokeParam.InitSignal;						//Compute New Contamination Offset
			AMin=AMin-SmokeParam.CntmOffset;									//Compute Subtraction of New and Last Contamination Offset
			if(AMin<=-SmokeParam.CntmMaxUpd)									//Amplitude limitation and equation
				AMin=-SmokeParam.CntmMaxUpd;
			else if (AMin>=SmokeParam.CntmMaxUpd)							
				AMin=SmokeParam.CntmMaxUpd;
			else AMin=0;
			if(AMin)
			{
				SmokeParam.CntmOffset+=AMin;
				if((SmokeParam.CntmOffset>SmokeParam.MinCntmOffset)&&
					 (SmokeParam.CntmOffset<SmokeParam.MaxCntmOffset))
				{
					IEEWR(&SmokeParam, (void*)&IEESmokeParam,			//Store Contamination Offset
					sizeof(SmokeParam));
				}
				if(SmokeParam.CntmOffset>=0)										//Check and Set Sensor Signal Quality State
				{
					AMin=(sint)((((slong)SmokeParam.CntmOffset)*
						200)/((slong)SmokeParam.MaxCntmOffset));
					if(AMin<SmokeParam.PreCntmThrld)
					{
						SmokeSetState((SmokeState&									//Sensor Contamination Report
							(~SmokeFaultMask))
							|SmokeCntmState);
					}
					else
					{
						SmokeSetState(SmokeState&										//Sensor Good Signal Report
							(~SmokeFaultMask));	
					}
				}
				else
				{
					AMin=(sint)((((slong)SmokeParam.CntmOffset)*
						200)/((slong)SmokeParam.MinCntmOffset));
					if(AMin>SmokeParam.PreFaultThrld)
					{
						SmokeSetState((SmokeState&
							(~SmokeFaultMask))												//Sensor Fault Report
							|SmokeFaultState);
					}
					else
					{
						SmokeSetState(SmokeState&
							(~SmokeFaultMask));												//Sensor Good Signal Report
					}
				}
			}
			break;
		case SmokeAlarmState:
			break;
		case SmokeLearnState:
			break;
	}
}

void SmokeInit(void)
{
	SmokeState=SmokeDutyState;
	CntmCompVal=0;
	CntmCompTmr=0;
	SumCompTmr=0;
}

void SmokeSetState(uchar State)
{
}
