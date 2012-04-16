#include    <config.h>
#if !defined(_ZZ_ADC_H) 
//&& defined(ZC_ADC)

/*! @file   zzADC.h
 *  @brief  Интерфейс для работы с АЦП
 *  @author Max Gekk
 *  @date   апрель 2007
 *  @version 1
 */   

#include    <zzTypes.h>

/*! @defgroup ZIG_ADC Интерфейс аналого-цифрового преобразователя
 *  @ingroup ZIGZAG
 *  @{
 * */

/*! @typedef    adc_mode_t
 *  @brief Режимы работы АЦП */
typedef enum {
    SINGLE_CONVERSION
} adc_mode_t;

/*! @fn result_t    adc_init( void  *cfg );
 *  @brief  Инициализация АЦП
 *  @param  mode - режим, под который будет проинициализирован АЦП
 *  @return В случае успеха возвращается ENOERR.
 * */
result_t    adc_init( adc_mode_t mode, ... );

/*! @fn result_t    adc_start( bool_t only_prepare );
 *  @brief  Подготовка и запуск выборки-преобразования
 *  @param only_prepare - Если значение параметра равно TRUE, то функция
 *  должна только подготовить АЦП к процедуре выборки и преобразования.
 *  @return В случае успешного запуска выборки возвращается ENOERR, иначе
 *  значение < 0.
 * */
result_t    adc_start( bool_t only_prepare );

/*! @fn int32_t     adc_read( uint16_t channel );
 *  @brief Вычитывание преобразованного значения
 *  @param channel - номер канала
 *  @return Возвращается очередное значение преобразования или значение < 0 в
 *  случае ошибки.
 * */
int32_t     adc_read( uint16_t channel );

/*! @fn result_t    adc_stop();
 *  @brief  Остановить АЦП
 *  @param  force - принудительная остановка АЦП
 *  @return В случае успешной остановки АЦП возвращается ENOERR, иначе значение меньшее 0.
 * */
result_t    adc_stop( bool_t force );

/*! @} */
#endif

