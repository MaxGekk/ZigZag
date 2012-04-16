#include    <8002.h>
#define PORT    20

#include    <zigzag.h>
#include    "iomacros.h"
#include    "adc12.h"
#include	<msp430.h>

#include "adc.h"

#define DBG(a) {}
#define NORMAL_PRIORITY      0
#define HIGH1_PRIORITY       1

#define TIMER_EVENT          0
#define ADC12_EVENT          3

#define TIMER_0              0

#define SHOT_PERIOD          100
#define QUIET_MSG_TIMEOUT    30000

// Interrupt numbers for MSP430
#define INT_NUM_ADC12        14

//#define MOTION_HI_TH         2730           // Upper limit signal above which is supposed to be signal caused by  moving object
//#define MOTION_LO_TH         1365           // Lower limit signal below which is supposed to be signal caused by  moving object
#define MOTION_TH            107
#define MOTION_HI_TH         (middle_point + (MOTION_TH << 4)) // Upper limit signal above which is supposed to be signal caused by  moving object
#define MOTION_LO_TH         (middle_point - (MOTION_TH << 4)) // Lower limit signal below which is supposed to be signal caused by  moving object
#define MOTION_STATE_QUIET   0
#define MOTION_STATE_ALARM   1

#define STAGE_IDLE           0
#define STAGE_PYRO           5


// Variables for Ext Int input

uint8_t  stage;              // Current measurements stage
uint8_t  cur_MotionState;
uint8_t  motion_detect_count;
uint16_t middle_point;


void sys_init()
   {
    result_t err;	    

    DBG("call init_module");

    stage = STAGE_IDLE; // Wait for the system to start up


    err = event_emit( NORMAL_PRIORITY, TIMER_EVENT, 0);
    if( ENOERR != err ) DBG("FAIL: module start");

    cur_MotionState = MOTION_STATE_QUIET;
    motion_detect_count = 0;
    middle_point        = (4096 / 2) << 4;


    DBG("return init_module");    
    return; 
   }

void event_handler( event_type_t event_type, unidata_t unidata ) 
   { 
    result_t err;
    uint64_t cur_time;
    static uint64_t last_detect_time = 0;
    
    DBG("call event_handler");
    switch(event_type)
    {
    case TIMER_EVENT:
       if((stage == STAGE_IDLE) )
       {
          stage = STAGE_PYRO; 
          init_adc();
       }
       err = stimer_set(TIMER_0, SHOT_PERIOD );
       if( ENOERR != err ) DBG("FAIL: start timer");
       break;
    case ADC12_EVENT:
       switch(stage)
       {
       case STAGE_PYRO:
          cur_time = sys_time();
          unidata <<= 4;
          if((unidata < MOTION_LO_TH) || (unidata > MOTION_HI_TH))
          {
             // Motion is detected
             if(cur_MotionState == MOTION_STATE_QUIET)
             {
                uint16_t attr = 0;
                attr_write(0xE2, &attr);
             }
             cur_MotionState = MOTION_STATE_ALARM;
             last_detect_time = cur_time;
          }
          else if(cur_MotionState == MOTION_STATE_ALARM)
          {
             if((cur_time - last_detect_time) > QUIET_MSG_TIMEOUT)
             {
                uint16_t attr = 1;
                attr_write(0xE2, &attr);
                cur_MotionState = MOTION_STATE_QUIET;
             }
          }
          middle_point += (unidata - middle_point) >> 8;
          //unidata = ((uint32_t)3000 * (uint32_t)unidata) / 4095; // Calculation of voltage 
          break;
       default:
          stage = STAGE_PYRO;

       }
       break;
    default:
       DBG("UNKNOW EVENT");
    }

    DBG("return event_handler");
    return;
   }

void stimer_fired( const uint8_t timer_num)
   {
    result_t err;
    int res;

    DBG("call fired");

    if(timer_num)
    {
       DBG("Timer not supported");
       return;
    }

    if(stage == STAGE_IDLE)
    {
       err = event_emit( NORMAL_PRIORITY, TIMER_EVENT, 0);
       if( ENOERR != err ) DBG("FAIL: emit timer event");
       return;
    }

    
    res = init_PyroMeasurement(1);
   
    err = stimer_set(TIMER_0, SHOT_PERIOD);
        if( ENOERR != err ) DBG("FAIL: start usual timer");


    DBG("return fired");
    return;
   }

void irq_handler( irq_t interrupt )
{
   uint16_t tmp; // General purpose temporal variable
   
   switch(interrupt)
   {
   case INT_NUM_ADC12:
      tmp = adc_int_handler(ADC12IV);
      event_emit(NORMAL_PRIORITY, ADC12_EVENT, tmp);
      break;
   }
}

