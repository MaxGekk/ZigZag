#include    <8007.h>
#define PORT    20

#include    <zigzag.h>
#include    "iomacros.h"
#include    "adc12.h"
#include	<msp430.h>

#include "adc.h"

#define DBG(a) {}
#define NORMAL_PRIORITY      0

#define TIMER_EVENT          0
#define ADC12_EVENT          3
#define ADXL_X_EVENT         5
#define ADXL_Y_EVENT         6
#define RCV_EVENT            7

#define TIMER_0              0
#define TIMER_1              1
#define TIMER_2              2

#define SHOT_PERIOD          500
#define REF_WAIT_PERIOD      25
#define EXT_INT_DIS_PERIOD   200

// Interrupt numbers for MSP430
#define INT_NUM_DAC_DMA      0
#define INT_NUM_PORT2        2
#define INT_NUM_PORT1        8
#define INT_NUM_TIMER_A0     10
#define INT_NUM_TIMER_A1     12
#define INT_NUM_ADC12        14
#define INT_NUM_COMPAR_A     22

//#define ADXL_THRESHOLD       34868          // ~2g (squared)
//#define ADXL_HYST            26150
#define VIBRO_NORM           0
#define VIBRO_HIGH           1

#define STAGE_IDLE           0
#define STAGE_ADXL_X_INIT    5
#define STAGE_ADXL_Y_INIT    6
#define STAGE_ADXL_INIT      7
#define STAGE_ADXL_X         8
#define STAGE_ADXL_Y         9
#define STAGE_ADXL_RES       10

#define ENABLE_EXT_INT       P1IE |=  0x80
#define DISABLE_EXT_INT      P1IE &= ~0x80

uint8_t seq_num;
uint8_t timer_cnt;    // Number of regular timer events (not events for REF generator)
uint8_t wait_for_REF; // Flag showing that we are waiting for the Reference Generator to stabilize

uint8_t  stage;                // Current measurements stage
uint16_t period_count;         // Conter for measurement number
uint16_t cur_ADXL_ADCValue[2]; // ADC results by X and Y ADXL channels
int16_t  diff[2];              // Difference between current and previous ADXL value
uint8_t  cur_VibroState;       // Current vibration state
uint32_t cur_VibroLevel;       // Current vibration level (x*x + y*y)
uint8_t  cur_VibroStateChangeCount;


/* Initialization of external interrupts (waterproof contacts, electronic seals etc)
 */
void sys_init()
   {
    result_t err;	    

    DBG("call init_module");
    P6SEL  =  0x36; // P6.1, P6.2, P6.4, P6.5 - ADC inputs, all others are digital I/O
    P6DIR  =  0xC9; // P6.1, P6.2, P6.4, P6.5 - inputs, all others - outputs 
    P6OUT &= ~0x08; // Switch off ADXL

    timer_cnt    = 0;
    wait_for_REF = 0;
    stage        = STAGE_IDLE;
    period_count = 0;
    cur_VibroStateChangeCount = 0;

    err = event_emit( NORMAL_PRIORITY, TIMER_EVENT, 0);
    if( ENOERR != err ) DBG("FAIL: module start");

    DBG("return init_module");    
    return; 
   }

void event_handler( event_type_t event_type, unidata_t unidata ) 
{
    result_t err;
    int res;

    DBG("call event_handler");
    switch(event_type)
    {
    case TIMER_EVENT:
       if((stage == STAGE_IDLE) )
       {
          stage = STAGE_ADXL_X_INIT; 
          init_adc();
       }
       err = stimer_set(TIMER_0, SHOT_PERIOD );
       if( ENOERR != err ) DBG("FAIL: start timer");
       break;
    case ADC12_EVENT:
       switch(stage)
       {
       case STAGE_ADXL_X:
          diff[X_IDX] = cur_ADXL_ADCValue[X_IDX] - unidata;
          if(diff[X_IDX] < 0) diff[X_IDX] = -diff[X_IDX]; // We are interested in absolute value
       case STAGE_ADXL_X_INIT:
          stage++; // STAGE_ADXL_Y_INIT or STAGE_ADXL_Y;
          cur_ADXL_ADCValue[X_IDX] = unidata;
          init_ADXLMeasurement(1, Y_IDX); // Prepare Y axis measurement
          if((res = start_singleConversion()) != ADC_RES_OK)
             stage--; // Problem starting A<F2>D conversion. Let's try again
          break;
       case STAGE_ADXL_Y_INIT:
          stage = STAGE_ADXL_X;
          enable_ADXL(0);
          adc_shutdown(1);
          cur_ADXL_ADCValue[Y_IDX] = unidata;
          cur_VibroState = VIBRO_NORM;
          break;
       case STAGE_ADXL_Y:
          stage = STAGE_ADXL_X;
          enable_ADXL(0);
          adc_shutdown(1);
          diff[Y_IDX] = cur_ADXL_ADCValue[Y_IDX] - unidata;
          if(diff[Y_IDX] < 0) diff[Y_IDX] = -diff[Y_IDX];  // We are interested in absolute value
          cur_ADXL_ADCValue[Y_IDX] = unidata;
          cur_VibroLevel = (uint32_t)diff[X_IDX] * diff[X_IDX] + (uint32_t)diff[Y_IDX] * diff[Y_IDX];
		  {
			  uint16_t threshold;
			  uint8_t max_state_changes;
			  attr_read(VIBRO_STATE_CHANGE_NUM, &max_state_changes);
			  if(cur_VibroState == VIBRO_NORM)
			  {
				 attr_read(VIBRO_THRESHOLD_HIGH, &threshold);

	//             if((cur_VibroLevel >= ADXL_THRESHOLD) && (++cur_VibroStateChangeCount > 2))
				 if(cur_VibroLevel >= threshold)
				 {
					if(++cur_VibroStateChangeCount > max_state_changes)
					{
					   cur_VibroStateChangeCount = 0;
					   cur_VibroState = VIBRO_HIGH;
					   attr_write(VIBRO_STATE, &cur_VibroState);
					}
				 }
				 else
					cur_VibroStateChangeCount = 0;
			  }
			  else
			  {
				 attr_read(VIBRO_THRESHOLD_LOW, &threshold);
				 if(cur_VibroLevel < threshold)
				 {
					if(++cur_VibroStateChangeCount > max_state_changes)
					{
					   cur_VibroStateChangeCount = 0;
					   cur_VibroState = VIBRO_NORM;
					   attr_write(VIBRO_STATE, &cur_VibroState);
					}
				 }
				 else
					cur_VibroStateChangeCount = 0;
			  }
		  }
          break;
       default:
          stage = STAGE_IDLE;
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

    if(timer_num > TIMER_0)
    {
       DBG("Timer not supported");
       return;
    }

    if(stage == STAGE_IDLE) // Wait for the system to initialize
    {
       err = event_emit( NORMAL_PRIORITY, TIMER_EVENT, 0);
       if( ENOERR != err ) DBG("FAIL: emit timer event");
       return;
    }
    
    if(!wait_for_REF)
    {
       // Initialize measurments
       res = init_ADXLMeasurement(1, X_IDX);

       wait_for_REF = 1;
       err = stimer_set(TIMER_0, REF_WAIT_PERIOD);
       if( ENOERR != err ) DBG("FAIL: start REF Wait timer");
    }
    else
    {
       // We are here cause REF_WAIT_PERIOD timer expired
       // Reference Generator should be stable already
       // No we are able to start conversion
       wait_for_REF = 0;
       if((res = start_singleConversion()) != ADC_RES_OK)
       {
          // If somthing is wrong (e.g. ADC is busy) emit event about this
          err = event_emit( NORMAL_PRIORITY, ADC12_EVENT, res);

          if( ENOERR != err ) DBG("FAIL: emit ADC12 (err) event");
       }
       period_count++;
       err = event_emit( NORMAL_PRIORITY, TIMER_EVENT, 0);
   
       if( ENOERR != err ) DBG("FAIL: emit timer event");

//       DBG("return fired");
//       return;
    }
    DBG("return fired");
    return;
   }


void irq_handler( irq_t interrupt )
{
   uint16_t tmp; // General purpose temporal variable
   result_t err;
   
   switch(interrupt)
   {
   case INT_NUM_TIMER_A0:
      break;
   case INT_NUM_TIMER_A1:
      break;
   case INT_NUM_ADC12:
      tmp = adc_int_handler(ADC12IV);
      event_emit(NORMAL_PRIORITY, ADC12_EVENT, tmp);
      break;
   case INT_NUM_COMPAR_A:
      break;
   }
}

uint16_t isGT(uint8_t attr_num1, uint8_t attr_num2)
{
    uint16_t val1, val2;
    attr_read(attr_num1, &val1);
    attr_read(attr_num2, &val2);

    return val1 > val2;
}
