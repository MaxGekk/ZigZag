#ifndef _ADC_H_
#define _ADC_H_


#define ADC_RES_OK            0
#define ADC_RES_OVERFLOW      -1
#define ADC_RES_TIMEOVERFLOW  -2
#define ADC_RES_BUSY          -3
#define ADC_RES_UNSUPPORTED   -4
#define ADC_RES_FAIL          -5

#define X_IDX                0              // definitions for accessing corresponding array elements
#define Y_IDX                1

extern uint16_t adc_result; // Storage for conversion results

int init_SupplyMonitor(uint8_t forceStop, uint8_t useInterrupt);
uint16_t getSupplyVoltage(void);

void init_adc(void);
int init_LightMeasurement(uint8_t forceStop);
int init_TemperatureMeasurement(uint8_t forceStop);
int init_ADXLMeasurement(uint8_t forceStop, uint8_t channel);
int start_singleConversion(void);
int adc_shutdown(uint8_t forceStop);
uint16_t adc_int_handler(uint16_t adc_vector);
void enable_photo(uint8_t enable);
void enable_temperature(uint8_t enable);
void enable_ADXL(uint8_t enable);

#endif // _ADC_H_
