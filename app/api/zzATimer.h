#include    <config.h>
#if !defined(_ZZ_ATIMER_H) && defined(ZC_ATIMER)
#define     _ZZ_ATIMER_H

/*! @file   zzATimer.h
 *  @brief  Объявление интерфейса асинхронных таймеров
 *  @author Max Gekk
 *  @date   июнь 2007
 *  @version 5
 */   

#include    <zzTypes.h>
#include    <_zzATimer.h>

/*! @defgroup ZIG_TIMER Интерфейс асинхронных таймеров
 *  @ingroup ZIGZAG
 *  @{
 * */

/*! @fn atimer_counter
 *  @brief  Функция возвращает значение счётчика таймера
 * */
__syscall uint32_t    atimer_counter() 
{ return _atimer_counter(); }

/*! @fn result_t    atimer_set( const uint8_t    tnum, const uint32_t    tpoint )
 *  @brief   Установка таймера
 *  @param tnum - номер таймера
 *  @param tpoint - момент времени срабатывания таймера
 *  @return Результат выполнения функции
 * */
__syscall result_t    atimer_set( const uint8_t    tnum, const uint32_t    tpoint )
{ return _atimer_set( OBJ_OFFSET, tnum, tpoint ); }

/*! @fn result_t    timer_stop( const uint8_t   tnum )
 *  @brief   Останов таймера
 *  @param   tnum - номер останавливаемого таймера
 */
__syscall result_t    atimer_stop( const uint8_t   tnum )
{ return _atimer_stop( OBJ_OFFSET, tnum ); }

/*! @fn void    timer_fired( uint8_t    tnum );
 *  @brief  Срабатывание таймера
 *  @param  tnum - номер сработавшего таймера
 *  */
__callback void    atimer_fired( uint8_t    tnum );
REG_CALLBACK( atimer_fired );

/*! @fn result_t    timer_info( const   uint8_t tnum, struct timerinfo  *const info )
 *  @brief  Получение информации о таймере
 *  @param  tnum - номер таймера
 *  @param  info - указатель на структуру, в которую будет скопирована информация о таймере
 *  @return Результат вызова функции
 *  */
__syscall result_t    atimer_info( const   uint8_t tnum, struct timerinfo  *const info )
{ return _atimer_info( tnum, info ); }

/*! @} */
#endif  /*  _ZZ_ATIMER_H    */

