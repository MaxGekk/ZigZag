/*! @file   tire_analog.c
 *  @brief  Мониторинг аналогового входа
 *  @author Max Gekk
 *  @date   апрель 2008
 *  @version 1
 *  */

#include    <8003.h>
#define     PORT    14
#include    <zigzag.h>
/* Предполагается, что в platform.h определен номер порта для 
 * светодиодов LED_PORT, а также маски для LED0, LED1, LED2 */
#include    <platform.h>

/* Включить отладочное мигание светодиодами */
#define     BLINK_ON
/* Отладочный светодиод */
#define     DBG_LED     LED0

/* Номер используемого таймера */
#define     SOFT_TIMER      0
/* Период опроса дискретных входов в миллисекундах*/
#define     POLLING_PERIOD  500
/* Светодиод, зажигаемый при ошибке */
#define     ERROR_LED       LED2
/* Время инициализации АЦП */
#define     ADC_INIT_TIME   50

#ifdef  BLINK_ON
static port_t dbg_led = 0; 
#define     TOGGLE_DBG_LED \
{ port_write( LED_PORT, DBG_LED, dbg_led ); \
  dbg_led ^= DBG_LED;\
}
#else
#define     TOGGLE_DBG_LED
#endif

void    sys_init()
{
    result_t    result = ENOERR;
    
    {  /* Отправлять показания с АЦП при каждом изменении */
        const uint16_t report_mode = 0x4444;
        attr_write( ATTR_THRESHOLD_REPORT_MODE, &report_mode );
    }

    /* Подготовка параметров инициализации АЦП */
    /* Инициализация АЦП */
    result = adc_init( SINGLE_CONVERSION );

    if( IS_ERROR(result) ) {
        port_write( LED_PORT, ERROR_LED, PIN_HI );
        return;
    }
    result = stimer_set( SOFT_TIMER, ADC_INIT_TIME );

    if( IS_ERROR(result) )
        port_write( LED_PORT, ERROR_LED, PIN_HI );

}

void    event_handler( event_type_t event_type, unidata_t   unidata )
{

    if( event_type == EV_ADC ) {
        const uint16_t  channel = unidata;
        int32_t    adcval;
        result_t    result;

        /* Останавливаем АЦП для уменьшения энергопотребления */
        result = adc_stop( TRUE );
        if( IS_ERROR(result) )
            port_write( LED_PORT, ERROR_LED, PIN_HI );

       
        /* Вычитываем все результаты выборки-преобразования по каналу */
        while( IS_OK( adcval = adc_read(channel) ) ) {
            attr_write( ANALOG_INPUT1, &adcval );
            //attr_write( ANALOG_INPUT1_TEST, &adcval );
        }
       
        /* Уходим на следующую итерацию */
        result = stimer_set( SOFT_TIMER, POLLING_PERIOD );
        if( IS_ERROR(result) )
            port_write( LED_PORT, ERROR_LED, PIN_HI );

        TOGGLE_DBG_LED
    }
}

void    stimer_fired( const uint8_t tnum )
{
    result_t result = ENOERR;

    if( tnum != SOFT_TIMER )
         port_write( LED_PORT, ERROR_LED, PIN_HI );

    /* Запуск АЦП */
    result = adc_start( FALSE );
    if( IS_OK(result) ) {
        return;
    }
    /* Не удалось запустить АЦП */
    port_write( LED_PORT, ERROR_LED, PIN_HI );
    /* Пробуем ещё раз через некоторое время */
    stimer_set( SOFT_TIMER, 4*POLLING_PERIOD );

}

uint16_t isGT(uint8_t attr_num1, uint8_t attr_num2)
{
    uint16_t val1, val2;
    attr_read(attr_num1, &val1);
    attr_read(attr_num2, &val2);

    return val1 > val2;
}

