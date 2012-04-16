/*! @file   tire_digio.c
 *  @brief  Мониторинг 24 цифровых входов
 *  @author Max Gekk
 *  @date   апрель 2008
 *  @version 1
 *  Заказчик: <<Нижнекамскшина>>.
 *  Требования к УБПД с дискретными входами:
 *      1. УБПД с дискретными входами должен обеспечивать прием и передачу информации 
 *      об изменении состояния входных сигналов через базовый узел на ПК, а так же 
 *      являться ретранслятором для других УБПД.
 *      2. УБПД должен иметь 24 дискретных входов напряжением 5В постоянного тока с общим минусом:
 *          * 10 сигналов – закодированные значения дисбаланса и угла; 
 *          * 2 сигнала – разрешение на чтение кода;
 *          * 4 сигнала – резерв;
 *          * 8 сигналов – код текущего типоразмера покрышки;
 *      3. При поступлении сигнала из внешней системы устройство должно подтвердить его прием миганием 
 *         светодиода, размещенного на корпусе.
 *      4. Настройки УБПД должны содержаться во внутренней микропрограмме устройства и не могут быть 
 *         изменены при помощи внешних команд.
 *  */

#include    <8005.h>
#define PORT    15

#include    <zigzag.h>
/* Предполагается, что в platform.h определены номера портов дискретных входов:
 * INPUT_PORT_1, INPUT_PORT_2 и INPUT_PORT_3
 * А для светодиодов LED_PORT, LED1, LED2, LED3 */
#include    <platform.h>

/* Включить отладочное мигание светодиодами */
#define     BLINK_ON
/* Индикация съёма данных */
#define     DATA_LED    LED1
/* Отладочный светодиод */
#define     DBG_LED     LED2

/* Период опроса дискретных входов в миллисекундах*/
#define     POLLING_PERIOD  200
/* Номер используемого таймера */
#define     SOFT_TIMER      0
/* Светодиод, зажигаемый при ошибке */
#define     ERROR_LED       LED0
/* Число портов ввода */
#define     TOTAL_IPORT     3

static union {
    uint32_t    val32;
    uint8_t     val8[sizeof(uint32_t)];
} input;

static  const uint8_t port_num[ TOTAL_IPORT ] = {INPUT_PORT_1,INPUT_PORT_2,INPUT_PORT_3};

#ifdef  BLINK_ON
static port_t dbg_led = 0; 
#define     TOGGLE_DBG_LED \
{ port_write( LED_PORT, DBG_LED, dbg_led ); \
  dbg_led ^= DBG_LED;\
}
static port_t data_led = 0; 
#define     TOGGLE_DATA_LED \
{ port_write( LED_PORT, DATA_LED, data_led ); \
  data_led ^= DATA_LED;\
}
#else
#define     TOGGLE_DBG_LED
#define     TOGGLE_DATA_LED
#endif

void    sys_init()
{
    result_t    result;
    port_attr_t   port_attr;

    input.val32 = 0;

    result = stimer_set( SOFT_TIMER, POLLING_PERIOD );

    if( IS_ERROR(result) )
        port_write( LED_PORT, ERROR_LED, PIN_HI );

    TOGGLE_DBG_LED
}

void    stimer_fired( const uint8_t tnum )
{
    result_t result;
    int i;
    uint32_t old_val = input.val32;   

    for( i=0; i<TOTAL_IPORT; i++ ) {
        result = port_read( port_num[i] , 0xFF, &(input.val8[i]) );

        if( IS_ERROR(result) )
            port_write( LED_PORT, ERROR_LED, PIN_HI );

    }

    if( old_val != input.val32 ) {
        attr_write( INPUT, &(input.val32) );
        TOGGLE_DBG_LED
    }

    result = stimer_set( SOFT_TIMER, POLLING_PERIOD );

    if( IS_ERROR(result) )
        port_write( LED_PORT, ERROR_LED, PIN_HI );

    TOGGLE_DATA_LED
}

