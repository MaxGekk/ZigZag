#ifndef SMOKE_H
#define SMOKE_H
#include "typedefs.h"

//Smoke Control Register Context
typedef struct tSmokeCtrl
{
	uchar	PrcEn:1;				//Processing Enable flag
	uchar	TmprCompEn:1;		//Temperature Compensation Enable Flag
	uchar	CntmCompEn:1;		//Contamination Compensation Enable Flag
	uchar	DNSwEn:1;				//Day to Night Switch Enable Flag
	uchar	PreAlarmEn:1;		//PreAlarm Processing Enable
	uchar	PreCntmEn:1;		//PreContamination Processing Enable
	uchar	PreFaultEn:1;		//PreFault Processing Enable
} TSmokeCtrl;

typedef struct tSmokeParam
{
	sint	InitSignal;			//Initial Signal (Nadc)
	sint	FRngOffset;			//Full Range Offset == 100% == 0,2dB/m (Nadc)
	sint	CntmOffset;			//Contamination Offset (Nadc)
	sint	MaxCntmOffset;	//Maximal Contamination Offset (Nadc)
	sint	MinCntmOffset;	//Minimal Contamination Offset (Nadc)
	sint	MaxSignal;			//Maximal Permissible Signal (Nadc)
	sint	MinSignal;			//Minimal Permissible Signal (Nadc)
	sint	MaxInitSignal;	//Maximal Permissible Initial Signal (Nadc)
	sint	MinInitSignal;	//Minimal Permissible Initial Signal (Nadc)
	sint	TmprCompRef;		//Reference for computing Temperature compensation
	sint	TmprCompOffset;	//Offset for No temperature Compensation Mode
	sint	CntmMaxUpd;			//Maximal Single Contamination Update Value
	uchar	DAlarmThrld;		//Day Alarm Threshold (%)
	uchar	DPreAlarmThrld;	//Day Prealarm Threshold (%)
	uchar	NAlarmThrld;		//Night Alarm Threshold (%)
	uchar	NPreAlarmThrld;	//Night Prealarm Threshold (%)
	uchar	CntmThrld;			//Contamination Threshold (%)
	uchar	PreCntmThrld;		//PreContamination Threshold (%)
	uchar	FaultThrld;			//Fault Threshold (%)
	uchar	PreFaultThrld;	//PreFault Threshold (%)
	uchar	CntmUpdatePr;		//Contamination Update Period
	uchar	SumUpdatePr;		//Integration Sum Update Period
	TSmokeCtrl Control;		//Control Register
} TSmokeParam;

enum SmokeStates
{
	SmokeDutyState=0x00,
	SmokePreAlarmState=0x01,
	SmokeAlarmState=0x02,
	SmokeLearnState=0x03,
	SmokeCleanState=0x04,
	SmokePreFaultState=0x08,
	SmokeFaultState=0x10,
	SmokePreCntmState=0x18,
	SmokeCntmState=0x20,
	SmokeNightMode=0x40,
	ZRLearnComplete=0x08,		//Valid only in Learn Mode
	FRLearnComplete=0x10		//Valid only in Learn Mode
};

#define SmokeAlarmMask	0x07
#define SmokeFaultMask	0x38

enum SmokeFuncList
{
	SmokeReset,							//Sensor Always switch to SmokeDutyMode with cleaning status bits (valid in All Modes)
	SmokeDayModeSet,				//Switch to Day Mode (valid only in Duty Mode)
	SmokeNightModeSet,			//Switch to Night Mode (valid only in Duty Mode)
	SmokeLearnStart,				//Switch to Learn Mode (valid only in Duty Mode)
	SmokeLearnComplete,			//Store Learned Variables and switch to Duty Mode
	SmokeCleanStart,				//Switch to Clean Mode (valid only in Duty Mode)
	SmokeCleanComplete,			//Store New Contamination Offset Variables and switch to Duty Mode
	SmokeZrSave,						//Zero (Initial) Signal Save (valid only in Learn Mode)
	SmokeFRSave,						//Full Range Signal Save (valid only in Learn Mode)
	SmokeRAWGet,						//RAW Smoke Sensor Signal Get (valid in All Modes)
	SmokeDensGet,						//Smoke Density request (valid only in Duty Mode)
	SmokeCntmGet,						//Contamination Request (valid only in Duty Mode)
	SmokeCThrldSet,					//Contamination Threshold Set (valid only in Learn Mode)
	SmokeCThrldGet,					//Contamination Threshold Request (valid only in Learn Mode)
	SmokePCThrldSet,				//PreContamination Threshold Set (valid only in Learn Mode)
	SmokePCThrldGet,				//PreContamination Threshold Request (valid only in Learn Mode)
	SmokeDAThrldSet,				//Day Alarm Threshold Set (valid only in Learn Mode)
	SmokeDAThrldGet,				//Day Alarm Threshold Request (valid only in Learn Mode)
	SmokeDPAThrldSet,				//Day PreAlarm Threshold Set (valid only in Learn Mode)
	SmokeDPAThrldGet,				//Day PreAlarm Threshold Request (valid only in Learn Mode)
	SmokeNAThrldSet,				//Night Alarm Threshold Set (valid only in Learn Mode)
	SmokeNAThrldGet,				//Night Alarm Threshold Request (valid only in Learn Mode)
	SmokeNPAThrldSet,				//Night PreAlarm Threshold Set (valid only in Learn Mode)
	SmokeNPAThrldGet				//Night PreAlarm Threshold Request (valid only in Learn Mode)
};

//#define TmprCompRef			1321		//Reference for computing Temperature compensation
//#define TmprCompOffset	1613		//Offset for No temperature Compensation Mode

void SmokeScan(uint* Tmpr);
void SmokeProcessing(void);
void SmokeInit(void);
void SmokeSetState(uchar State);

#endif
