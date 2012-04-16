#ifndef _ADC_H_
#define _ADC_H_

#define ADC_RES_OK            0
#define ADC_RES_OVERFLOW      -1
#define ADC_RES_TIMEOVERFLOW  -2
#define ADC_RES_BUSY          -3
#define ADC_RES_UNSUPPORTED   -4
#define ADC_RES_FAIL          -5


extern uint16_t adc_result; // Storage for conversion results

void init_adc(void);
int init_PyroMeasurement(uint8_t forceStop);
uint16_t adc_int_handler(uint16_t adc_vector);
void enable_pyro(uint8_t enable);

#endif // _ADC_H_
