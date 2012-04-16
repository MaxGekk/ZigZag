#include <msp430x16x.h>
#include "typedefs.h"
#include "RFSensor.h"
#include "Smoke.h"
#include "Tmpr.h"

#define OBJ     4
#define PORT    5
#include    <zigzag.h>
#include    <zzTypes.h>

#define     SYNC_TIMER 0
#define     WAIT_PERIOD     1000  /* 500 ЛЯЕЙ. */

#define     EV_PRIORITY     0   /* Приоритет события: срабатывание асинхронного таймера */
#define     EV_TYPE         0   /* Тип события о срабатывании асинхр. таймера */
#define     EV_UNIDATA      0

uint Tmpr;

void sys_init()
{
	PinConfigure(OPAMP_ON_PORT, OPAMP_ON_BIT, DIR_OUT+PIN_LO+MODE_IO+IRQ_DS+EDG_FIL);
	PinConfigure(HLY_PORT, HLY_BIT, DIR_OUT+PIN_LO+MODE_IO+IRQ_DS+EDG_FIL);
	PinConfigure(HLR_PORT, HLR_BIT, DIR_OUT+PIN_LO+MODE_IO+IRQ_DS+EDG_FIL);
	PinConfigure(IR_PLS_PORT, IR_PLS_BIT, DIR_OUT+PIN_LO+MODE_IO+IRQ_DS+EDG_FIL);
	PinConfigure(SENS_PORT, SENS_BIT, DIR_OUT+PIN_LO+MODE_IO+IRQ_DS+EDG_FIL);
	PinConfigure(TXD0_PORT, TXD0_BIT, DIR_OUT+PIN_HI+MODE_EXT+IRQ_DS+EDG_FIL);
	PinConfigure(RXD0_PORT, RXD0_BIT, DIR_IN+PIN_HI+MODE_EXT+IRQ_DS+EDG_FIL);
	PinConfigure(TXD1_PORT, TXD1_BIT, DIR_IN+PIN_HI+MODE_IO+IRQ_DS+EDG_FIL);
	PinConfigure(RXD1_PORT, RXD1_BIT, DIR_IN+PIN_HI+MODE_IO+IRQ_DS+EDG_FIL);
	PinConfigure(RF_ON_PORT, RF_ON_BIT, DIR_OUT+PIN_HI+MODE_IO+IRQ_DS+EDG_FIL);
	PinConfigure(HI_PWR_PORT, HI_PWR_BIT, DIR_OUT+PIN_LO+MODE_IO+IRQ_DS+EDG_FIL);
	PinConfigure(KEY_PORT, KEY_BIT, DIR_IN+PIN_HI+MODE_IO+IRQ_DS+EDG_FIL);

//Initial Scanning
  RefTOn();
	TmprInit();
	Tmpr=TemperatureScan();
	SmokeInit();
  SmokeScan(&Tmpr);
  RefTOff();
	SmokeOpampOff();
  stimer_set(SYNC_TIMER , WAIT_PERIOD);
	return;
}

void    stimer_fired( uint8_t    tnum ) 
{ 
	event_emit( EV_PRIORITY, EV_TYPE, EV_UNIDATA );
	return;
}

void    event_handler( event_type_t event_type, unidata_t   unidata )
{
	if( EV_TYPE == event_type )
	{
		stimer_set( SYNC_TIMER, WAIT_PERIOD);
		WakeUpEvent();
	}
    return;
}

void WakeUpEvent(void)
{
  SmokeOpampOn();
	RefTOn();
  SmokeProcessing();
  TemperatureProcessing(&Tmpr);
  SmokeScan(&Tmpr);
	SmokeOpampOff();
  Tmpr=TemperatureScan();
  RefTOff();
}

void  RefTOn(void)
{
  ADC12CTL0|=REFON;                   // Turn on Refererence
}

void  RefTOff(void)
{
  ADC12CTL0&=~REFON;                  // Turn off Refererence
}

void SmokeOpampOn(void)
{
	port_write(OPAMP_ON_PORT, OPAMP_ON_BIT, PIN_LO);
}

void SmokeOpampOff(void)
{
	port_write(OPAMP_ON_PORT, OPAMP_ON_BIT, PIN_HI);
}

void	PinConfigure(uchar	Port, uchar	Mask, uchar	Mode) 
{
	port_attr_t   port_attr;
	if(Mode & PINVAL_HI)	
	{
		port_write(Port , Mask, PIN_HI);
	}
	else
	{
		port_write(Port , Mask, PIN_LO);
	}
	if(Mode & DIR_OUT)
	{
		PIN_SET(port_attr.dir, Mask , Mask);
	}
	else
	{
		PIN_CLEAR(port_attr.dir, Mask);
	}
	if(Mode & MODE_EXT)
	{
		PIN_SET(port_attr.sel, Mask , Mask);
	}
	else
	{
		PIN_CLEAR(port_attr.sel, Mask);								/* тСМЙЖХЪ ББНДЮ/БШБНДЮ */
	}
	if(Mode & IRQ_EN )
	{
		PIN_SET(port_attr.ie, Mask , Mask);
	}
	else
	{
		PIN_CLEAR(port_attr.ie, Mask );									/* гЮОПЕР ОПЕПШБЮМХИ */
	}
	if(Mode & EDG_FIL)
	{
		PIN_SET(port_attr.ies, Mask , Mask);
	}
	else
	{
		PIN_CLEAR(port_attr.ies, Mask );									/* гЮОПЕР ОПЕПШБЮМХИ */
	}
	port_set_attr(Port, Mask, &port_attr);	
}
