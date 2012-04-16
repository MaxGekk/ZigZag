#ifndef     _ZZ_SYS_H
#define     _ZZ_SYS_H

/*! @file   zzSys.h
 *  @brief  Объявления функций общего назначения
 *  @author Max Gekk
 *  @date   июнь 2007
 *  @version 5
 */   

#include    <zzTypes.h>
#include    <_zzSys.h>

/*! @defgroup ZIG_SYS Интерфейс общесистемных функций
 *  @ingroup ZIGZAG
 *  @{
 * */

/*! @fn void    sys_init();
 *  @brief  Инициализация прикладного объекта
 * */
__callback void    sys_init();
REG_CALLBACK( sys_init );

/*! @fn uint64_t    sys_time();
 *  @brief  Функция возвращает локальное время в миллисекундах 
 * */
__syscall uint64_t    sys_time() { return _sys_time(); }

/*! @fn uint64_t    set_sys_time();
 *  @brief  Установка локального времени.
 *  @param  time - новое локальное время в миллисекундах относительно 1 января 1970 года
 *  @return 0 - в случае успеха, иначе -1.
 * */
volatile uint64_t* __workaround;
__syscall int16_t    set_sys_time( uint64_t time ) {
   __workaround = &time;
   return _set_sys_time(time); }

/*! @fn void memcopy( void *const dst, const void *const src, const uint8_t len );
 *  @brief  Копирование блока данных из одной области памяти в другую
 *  @param dst - указатель на область памяти, в которую будут скопированы данные
 *  @param src - указатель на область памяти, из которой будут скопированы данные
 *  @param len - размер копируемых данных в байтах
 * */
void memcopy( void *const dst, const void *const src, const uint8_t len );

/*! @} */
#endif  /*  _ZZ_SYS_H   */

