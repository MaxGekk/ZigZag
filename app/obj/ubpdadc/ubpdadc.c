/*! @file   ubpd.c
 *  @brief  Устройство беспроводной передачи данных
 *  @author Max Gekk
 *  @date   февраль 2008
 *  @version 1
 *  */

#include    <8003.h>
#define PORT    20

#include    <zigzag.h>
#include    "iomacros.h"
#include    "adc12.h"

#define     IPORT   2       /* Номер порта цифровых входов */
#define     IPORT_IRQ   IRQ_PORT2
#define     IPINS   0x0F    /* XXX Маска цифровых входов */

#define     OPORT   5       /* Номер порта цифровых выходов */
#define     OPINS   0x0F    /* Маска цифровых выходов */

#define     PRIORITY    (PRIORITY_HIGH)
#define     GIOP_INTERRUPT  0   /* Номер события, который генерируется при изменении входов */

#define BLINK_ON
#if defined(BLINK_ON)
#define     LED_PORT    6       /* номер порта для светодиодов */
#define     LED3        4
#endif

#define     APORT                 6
#define     APINS                 0x78  // Analog inputs mask
#define     APINSNUM              4
#define     AP_1ST_IN_NUM         3

#define     ADC_RES_OK             0
#define     ADC_RES_OVERFLOW      -1
#define     ADC_RES_TIMEOVERFLOW  -2
#define     ADC_RES_BUSY          -3
#define     ADC_RES_UNSUPPORTED   -4
#define     ADC_RES_FAIL          -5

#define     ADC_TIMER_ID          0
#define     ADC_PERIOD            1000 
#define     ADC_REF_TIMER_ID      1
#define     ADC_REFON_STARTUP     20

#define     EV_ADC                1
#define     STAGE_INIT_ADC        1
#define     STAGE_START_ADC       2
#define     STAGE_ADC_RESULT      3

uint16_t adc_stage;
uint16_t adc_result[APINSNUM]; // Storage for last conversion results

void init_adc(void)
{
   port_attr_t adc_pattr;
   PIN_CLEAR(adc_pattr.dir, APINS);
   PIN_SET(adc_pattr.sel, APINS, PIN_HI);
   memset(&adc_result, 0, sizeof(adc_result));
   adc_stage = STAGE_INIT_ADC;
}

int init_ADConversion(uint8_t forceStop)
{
   int res;

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
   ADC12CTL0  = ADC12ON | SHT0_2 | REFON | REF2_5V | MSC; // Set Sample and hold timer to 16 clocks, internal reference 2.5V
   ADC12CTL1  = CSTARTADD_0 | SHS_0 | SHP | ADC12DIV_0 | ADC12SSEL_0 | CONSEQ_1; // Start from ADC12MEM1, SH from ADC12SC, ADC12OSC, sequence of channels
   ADC12MCTL0 = SREF_1 + AP_1ST_IN_NUM + 0; // Select range from VSS to VREF+, first channel
   ADC12MCTL1 = SREF_1 + AP_1ST_IN_NUM + 1; // Select range from VSS to VREF+, second channel
   ADC12MCTL2 = SREF_1 + AP_1ST_IN_NUM + 2; // Select range from VSS to VREF+, third channel
   ADC12MCTL3 = SREF_1 + AP_1ST_IN_NUM + 3 + EOS; // Set EOS for this register, select range from VSS to VREF+, fourth channel
   ADC12IFG   = 0x00; // Clear all ADC12MEMx interupt flags
   ADC12IE    = 0x0f; // Interrupt for ADC12MEM0 - ADCMEM3


   // ADC12 is now prepared for conversion. We should wait for Reference Generator to stabilize for about 17 ms.
   // It's caller responsibility to wait or sleep while expecting Ref stabilization
   // and then to initialize A/D conversion

   return res;
}

// Starts single conersion if ADC isn't busy
int start_ADConversion(void)
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


uint16_t isGT(uint8_t attr_num1, uint8_t attr_num2)
{
    uint16_t val1, val2;
    attr_read(attr_num1, &val1);
    attr_read(attr_num2, &val2);

    return val1 > val2;
}

/* Инициализация портов */
void    sys_init()
{
    port_attr_t   port_attr;
#if defined(BLINK_ON)
    PIN_SET( port_attr.dir, LED3, PIN_HI );   /* Направление на вывод */
    PIN_CLEAR( port_attr.sel, LED3 );         /* Функция ввода/вывода */
    PIN_CLEAR( port_attr.ie, LED3);          /* Запрет прерываний */
    port_set_attr( LED_PORT, LED3, &port_attr );
    port_write( LED_PORT, LED3, PIN_LO );
#endif
    init_adc();
    event_emit(PRIORITY, EV_ADC, 0);
    //stimer_set(ADC_TIMER_ID, ADC_PERIOD);
    return;
}

void    stimer_fired( const uint8_t tnum )
{
   if(tnum == ADC_TIMER_ID)
   {
      //port_t led_port;
      //port_read(LED_PORT, LED3, &led_port);
      //port_write(LED_PORT, LED3, led_port ^ LED3);
      stimer_set(ADC_TIMER_ID, ADC_PERIOD);
      event_emit(PRIORITY, EV_ADC, adc_stage);
      //port_write( LED_PORT, LED3, PIN_HI );
   }
   else if(tnum == ADC_REF_TIMER_ID)
      event_emit(PRIORITY, EV_ADC, adc_stage);
}


void    irq_handler( irq_t  irq )
{
    /* Обрабатываем изменение состояний входов */
    if( IPORT_IRQ == irq ) {
        port_attr_t   port_attr;
        port_t  ifg, in;
        /* Перенастраиваем прерывания от входов */
        port_get_attr( IPORT, IPINS, &port_attr );
        port_get_iflag( IPORT, IPINS, &ifg );   /* Маска флагов прерываний */
        port_reset_iflag( IPORT, ifg );         /* Сбрасываем флаги прерываний */
        port_attr.ies ^= ifg;   /* Меняем условие генерации прерывания, у входов, у которых изменилось состояние */
        port_set_attr( IPORT, IPINS, &port_attr );
        port_read( IPORT, IPINS, &in );         /* Вычитываем состояние входов, чтобы записать их в атрибут */
        /* Продолжаем обработку прерывания в обработчике события */
        event_emit( PRIORITY, GIOP_INTERRUPT, in );
    }
    else if(irq == IRQ_ADC12)
    {
       uint16_t adcres;

       adcres = ADC12IV;
       switch(adcres)
       {
       case 0x00:
          break;
       case 0x02: // ADC12MEMx overflow
          break;
       case 0x04: // Conversion time oferflow
          break;
       case 0x06: // ADC12MEM0
          adc_result[0] = ADC12MEM0;
          break;
       case 0x08: // ADC12MEM1
          adc_result[1] = ADC12MEM1;
          break;
       case 0x0a: // ADC12MEM2
          adc_result[2] = ADC12MEM2;
          break;
       case 0x0c: // ADC12MEM3
          adc_result[3] = ADC12MEM3;
          event_emit(PRIORITY, EV_ADC, STAGE_ADC_RESULT);
          break;
       }
    }
    return;
}

void    event_handler( event_type_t event_type, unidata_t   unidata )
{
   if(event_type == EV_ADC)
   {
      port_t led_port;
      //port_write( LED_PORT, LED3, PIN_HI );
      switch(unidata)
      {
      case STAGE_INIT_ADC:
         init_ADConversion(1);
         stimer_set(ADC_REF_TIMER_ID, ADC_REFON_STARTUP);
         adc_stage = STAGE_START_ADC;
         break;
      case STAGE_START_ADC:
         adc_stage = STAGE_ADC_RESULT;
         start_ADConversion();
         break;
      case STAGE_ADC_RESULT:
         adc_stage = STAGE_INIT_ADC;
         adc_shutdown(1);

         /*
         attr_write(ANALOG_INPUT1_TEST, &adc_result[0]);
         attr_write(ANALOG_INPUT2_TEST, &adc_result[1]);
         attr_write(ANALOG_INPUT3_TEST, &adc_result[2]);
         attr_write(ANALOG_INPUT4_TEST, &adc_result[3]);
         */
         
         attr_write(ANALOG_INPUT1, &adc_result[0]);
         attr_write(ANALOG_INPUT2, &adc_result[1]);
         attr_write(ANALOG_INPUT3, &adc_result[2]);
         attr_write(ANALOG_INPUT4, &adc_result[3]);
      
         port_read(LED_PORT, LED3, &led_port);
         port_write(LED_PORT, LED3, led_port ^ LED3);
         break;
      default:
         adc_stage = STAGE_INIT_ADC;
         stimer_set(ADC_TIMER_ID, ADC_PERIOD);
      }
   }

   return;
}

