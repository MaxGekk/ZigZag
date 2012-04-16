#include    <config.h>
#if !defined(_ZZ_STIMER_H) && defined(ZC_STIMER)
#define     _ZZ_STIMER_H

/*! @file   zzSTimer.h
 *  @brief  Объявление интерфейса синхронных таймеров
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1
 */   

#include    <zzTypes.h>
#include    <_zzSTimer.h>

/*! @defgroup ZIG_STIMER Интерфейс синхронных таймеров
 *  @ingroup ZIGZAG  @{
 *      Число доступных синхронных таймеров в системе определяется константой SOFT_TIMER_TOTAL
 * */

/*! @fn result_t    stimer_set( const uint8_t tnum, const uint32_t milli_sec )
 *  @brief  Установка синхронного таймера
 *  @param  tnum - номер таймера из диапазона от 0 до SOFT_TIMER_TOTAL-1
 *  @param  milli_sec - интервал в миллисекундах начиная от текущего момента, на который 
 *  устанавливается таймер.
 *  @return Возвращается ENOERR в случае успешного вызова, иначе:
 *      - EINVAL - некорректный номер таймера,
 *      - ENOSYS - данная функция не поддерживается системой
 * */
__syscall   result_t    stimer_set( const uint8_t tnum, const uint32_t milli_sec )
{ return _stimer_set( OBJ_OFFSET, tnum, milli_sec ); }

/*! @fn void    stimer_fired( const uint8_t tnum );
 *  @brief  Срабатывание таймера
 *  @param tnum - номер таймера
 * */
__callback  void    stimer_fired( const uint8_t tnum );
REG_CALLBACK( stimer_fired );

/*! @} */
#endif  /*  _ZZ_STIMER_H    */

