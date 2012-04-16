#include    <config.h>
#if     !defined(_ZZ_PORT_H) && defined(ZC_PORT)
#define     _ZZ_PORT_H

/*! @file   zzPort.h
 *  @brief  Интерфейс для работы с портами ввода/вывода
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1
 */   

#include    <zzTypes.h>
#include    <_zzPort.h>

/*! @defgroup ZIG_PORT Интерфейс портов ввода/вывода
 *  Число доступных в системе портов определяется константой PORT_TOTAL. 
 *  @ingroup ZIGZAG @{
 * */

/*! @fn result_t    port_write( const uint8_t  port_num, const port_t mask, const port_t value );
 *  @brief  Запись в порт
 *  @param port_num - номер порта из диапазона от 0 до PORT_TOTAL-1
 *  @param mask - маска, задающая выходы, которые должны быть записаны
 *  @param value - значение, которое записывается в порт ( в соответствии с маской )
 *  @return В случае успешной записи в порт возвращается ENOERR, иначе
 *      - EINVAL - неправильный номер порта
 *      - EACCESS - запрешёна запись в порт
 *      - ENOSYS - функция не поддерживается системой
 * */
result_t    port_write( const uint8_t  port_num, const port_t mask, const port_t value );

/*! @fn result_t    port_read( uint8_t port_num, port_t mask, port_t *const value_ptr);
 *  @brief  Чтение из порта
 *  @param port_num - номер порта
 *  @param mask - маска, задающая входы, которые должны быть прочитаны.
 *  @param value_ptr - указатель на область памяти, в которую будет записано состояние порта в 
 *  соответствии с маской
 *  @return В случае успешного чтения из порта возвращается ENOERR, иначе
 *      - EINVAL - неправильный номер порта
 *      - EACCESS - запрещено чтение из порта
 *      - ENOSYS - функция не поддерживается системой
 * */
result_t    port_read( const uint8_t port_num, const port_t mask, port_t *const value_ptr);

/*! @fn result_t    port_set_attr( uint8_t port_num, port_t mask, const port_attr_t *const port_attr_ptr );
 *  @brief  Установка атрибутов ( параметров ) порта
 *  @param port_num - номер порта
 *  @param mask - маска, задающая входа/выходы порта, атрибуты которых должны быть установлены
 *  @param port_attr_ptr - указатель на структуру, задающую атрибуты порта
 *  @return В случае успешной установки атрибутов возвращается ENOERR, иначе
 *      - EINVAL - неправильный номер порта или установки порта
 *      - EACCESS - нарушен доступ к порту
 *      - ENOSYS - функция не поддерживается системой
 * */
result_t    port_set_attr( const uint8_t port_num, const port_t mask, const port_attr_t *const port_attr_ptr );

/*! @fn result_t    port_get_attr( const uint8_t port_num, const port_t mask, port_attr_t *const port_attr_prt );
 *  @brief Получение текущих значений атрибутов порта
 *  @param port_num - номер порта
 *  @param mask -  маска, задающая входа/выходы порта, атрибуты которых должны быть получены
 *  @param port_attr_ptr - указатель на структуру, в которую будут записаны текущие значения атрибутов
 *  @return В случае успеха вызова возвращается ENOERR, иначе
 *      - EINVAL - неправильный номер порта
 *      - EACCESS - нарушен режим доступа к порту
 *      - ENOSYS - функция не поддерживается системой
 * */
result_t    port_get_attr( const uint8_t port_num, const port_t mask, port_attr_t *const port_attr_ptr );

/*! @fn result_t    port_get_iflag( const uint8_t port_num, const port_t mask, port_t *const iflag_ptr );
 *  @brief  Получение флагов прерываний
 *  @param port_num - номер порта
 *  @param mask - маска, задающая те входы/выходы порта, флаги прерываний которых должны быть получены
 *  @param iflag_ptr - указатель на область памяти, в которую будут записаны флаги прерываний порта.
 *  @return Функция возвращает ENOERR в случае успешного вызова, иначе:
 *      - EINVAL - неправильный номер порта
 *      - EACCESS - порт не поддерживает флаги прерываний
 *      - ENOSYS - функция не поддерживается системой
 * */
result_t    port_get_iflag( const uint8_t port_num, const port_t mask, port_t *const iflag_ptr );

/*! @fn result_t    port_reset_iflag( const uint8_t port_num, const port_t mask );
 *  @brief  Сброс флагов прерываний порта
 *  @param port_num - номер порта
 *  @param mask - маска, задающая входы порта, флаги прерываний которых должны быть сброшены.
 *  @return В случае успешного сброса флагов возвращается ENOERR, иначе
 *      - EINVAL - неверный номер порта
 *      - EACCESS - порт не поддерживает флаги прерываний
 *      - ENOSYS - функция не поддерживается системой
 * */
result_t    port_reset_iflag( const uint8_t port_num, const port_t mask );

/*  @}  */
#endif  /*  _ZZ_PORT_H   */

