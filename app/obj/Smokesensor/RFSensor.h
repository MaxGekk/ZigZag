#ifndef RFSENSOR_H
#define RFSENSOR_H

#include  "msp430x16x.h"
#include "typedefs.h"

#define OPAMP_ON_BIT  0x01
#define OPAMP_ON_PORT	0x02

#define HLY_BIT				0x01
#define HLY_PORT			0x04

#define HLR_BIT				0x02
#define HLR_PORT			0x04

#define IR_PLS_BIT		0x40
#define IR_PLS_PORT		0x06

#define SENS_BIT			0x01
#define SENS_PORT			0x06

#define TXD0_BIT			0x10
#define TXD0_PORT			0x03

#define RXD0_BIT			0x20
#define RXD0_PORT			0x03

#define TXD1_BIT			0x40
#define TXD1_PORT			0x03

#define RXD1_BIT			0x80
#define RXD1_PORT			0x03

#define RF_ON_BIT			0x40
#define RF_ON_PORT		0x01

#define HI_PWR_BIT		0x80
#define HI_PWR_PORT		0x01

#define KEY_BIT				0x80
#define KEY_PORT			0x05

#define	DIR_IN				0x00
#define	DIR_OUT				0x01
#define	MODE_IO				0x00
#define	MODE_EXT			0x02
#define	PINVAL_HI			0x04
#define	PINVAL_LO			0x00
#define	IRQ_EN				0x08
#define	IRQ_DS				0x00
#define	EDG_RIS				0x00
#define	EDG_FIL				0x10

typedef struct tSensSta
{
  uint  SmokeAlarm:1;
  uint  SmokePreAlarm:1;
  uint  TmprMaxAlarm:1;
  uint  TmprMaxPreAlarm:1;
  uint  TmprRiseAlarm:1;
  uint  TmprRisePreAlarm:1;
  uint  SmokeCntm:1;
  uint  SmokePreCntm:1;
  uint  SmokeFault:1;
  uint  SmokePreFault:1;
  uint  TmprFault:1;
  uint  TmprPreFault:1;
} TSensSta;                   //Sensor Status

typedef struct tSensCntr
{
  uint  SmokeEn:1;
  uint  SmokeTmprCompEn:1;
  uint  SmokeCntmCompEn:1;
  uint  MaxTmprEn:1;
  uint  RiseTmprEn:1;
} TSensCntr;                  //Sensor Control

void  WakeUpEvent(void);
void  RefTOn(void);
void  RefTOff(void);
void	SmokeOpampOn(void);
void	SmokeOpampOff(void);
void	PinConfigure(uchar	Port, uchar	pMsk, uchar	Mode);


#endif


