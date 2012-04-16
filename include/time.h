#ifndef     _TIME_H
#define     _TIME_H

/*! @file   time.h
 *  @brief  Операции с таймерами и локальным временем
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1 
 */

#include    <inttypes.h>
#include    <zzConst.h>

/*! @defgroup sys_time Получение локального времени @{ */

/*! @fn uint64_t    __sys_time();
 *  @brief Локальное время в миллисекундах от 1 января 1970 года
 * */
uint64_t    __sys_time();

/*! @fn int16_t    __set_sys_time( uint64_t time );
 *  @brief Установка локального времи в миллисекундах от 1 января 1970 года
 * */
int16_t    __set_sys_time( uint64_t time );

/*! @} */

/*! @defgroup stimer Интерфейс синхронного таймера 
 *      Число доступных синхронных таймеров определяется константой SOFT_TIMER_TOTAL
 *! @{  */

/*! @fn int16_t     __stimer_set( uint8_t tnum, uint32_t milli_sec );
 *  @brief  Установка синхронного таймера
 *  @param  tnum - номер таймера из диапазона от 0 до SOFT_TIMER_TOTAL-1
 *  @param  milli_sec - интервал в миллисекундах начиная от текущего момента, на который 
 *  устанавливается таймер.
 *  @return Возвращается 0 в случае успешного вызова, иначе -1.
 * */
int16_t     __stimer_set( const uint8_t tnum, const uint32_t milli_sec );

/*! @fn void    __stimer_fired( uint8_t tnum );
 *  @brief  Срабатывание таймера
 *  @param tnum - номер таймера
 * */
void    __stimer_fired( const uint8_t tnum );

/*! @} */

/*! @defgroup atimer Интерфейс асинхронного таймера 
 *      Число доступных фсинхронных таймеров определяется константой HARD_TIMER_TOTAL
 *! @{  */

/*! @fn uint32_t    __atimer_counter();
 *  @brief  Текущее значение счётчика таймера
 *  */
uint32_t    __atimer_counter();

/*! @fn int16_t __atimer_set( const uint8_t tnum, uint32_t tpoint );
 *  @brief  Установка асинхронного таймера
 *  @param tnum - номер таймера из диапазона от 0 до HARD_TIMER_TOTAL-1
 *  @param tpoint - значение счётчика таймера, при котором должно произойти срабатывание
 *  @return Возвращается 0 в случае успешной установки таймера, иначе -1.
 *  */
int16_t __atimer_set( const uint8_t tnum, uint32_t tpoint );

/*! @fn int16_t __atimer_stop( const uint8_t tnum );
 *  @brief Остановка асинхронного таймера таймера
 *  @param tnum - номер асинхронного таймера
 *  @return Возвращается 0 в случае успешной остановки таймера, иначе -1.
 * */
int16_t __atimer_stop( const uint8_t tnum );

/*! @fn int16_t     __atimer_is_set( const uint8_t tnum );
 *  @brief  Проверка: установлен ли таймер.
 *  @param tnum - номер проверяемого таймера
 *  @return 1 - если таймер установлен, 0 - в случае если не установлен, -1 -
 *  если произошла ошибка.
 * */
int16_t     __atimer_is_set( const uint8_t tnum );

/*! @fn uint32_t    __atimer_point( const uint8_t tnum );
 *  @brief  Получение значение счётчика, на которое установлен таймер.
 *  @param tnum - номер таймера
 *  @return Значение счётчика таймера 
 * */
uint32_t    __atimer_point( const uint8_t tnum );

/*! @fn void    __atimer_fired( uint8_t tnum );
 *  @brief  Срабатывание асинхронного таймера
 *  @param tnum - номер сработавшего таймера
 * */
void    __atimer_fired( uint8_t tnum );

/*! @} */

#endif  /*  _TIME_H */

