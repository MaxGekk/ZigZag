#ifndef     _ZZ_MSG_H
#define     _ZZ_MSG_H

/*! @file   zzMsg.h
 *  @brief  Объявление интерфейса обмена сообщениями
 *  @author Max Gekk
 *  @date   июнь 2007
 *  @version 5
 */   

#include    <zzTypes.h>
#include    <_zzMsg.h>

/*! @defgroup ZIG_MSG Интерфейс обмена сообщениями
 *  @ingroup ZIGZAG
 *  @{
 * */

/*! Флаги управления сообщениями */
#define MFLAG_DEFAULT       _MFLAG_DEFAULT      /*!< Все флаги по умолчанию сброшены */
#define MFLAG_NO_CONFIRM    _MFLAG_NO_CONFIRM   /*!< Не извещать об окончании процедуры отправки сообщения */
#define MFLAG_RAW           _MFLAG_RAW          /*!< Тело сообщения уже содержит заголовок */

/*! @fn msg_t msg_new( net_addr_t dst_addr, app_port_t    dst_port, uint8_t msg_type, size_t  body_size, uint8_t flags) 
 *  @brief   Создание нового сообщения
 *  @param dst_addr - сетевой адрес узла-назначения
 *  @param dst_port - порт объекта-назначения
 *  @param msg_type - тип сообщения
 *  @param body_size - размер тела сообщения
 *  @return Функция возвращает отрицательное значение в случае ошибки, иначе идентификатор сообщения 
 * */
__syscall msg_t   msg_new( net_addr_t dst_addr, app_port_t    dst_port, uint8_t msg_type, size_t  body_size, uint8_t flags )
{ return _msg_new( OBJ_OFFSET, dst_addr, dst_port, PORT, msg_type, body_size, flags ); }

/*! @fn result_t    msg_info( msg_t   msg,  struct  msginfo *info );    
 *  @brief   Получение информации о сообщении
 *  @param msg - идентификатор сообщения
 *  @param info - указательна структуру, в которую будет скопирована информация о сообщении
 *  @return Функция возвращает отрицательное значение в случае ошибки и ENOERR если вызов
 *  был успешен.
 * */
__syscall result_t    msg_info( msg_t   msg,  struct  msginfo *info )
{ return _msg_info( OBJ_OFFSET, msg, info ); }

/*! @fn result_t    msg_destroy( msg_t   msg );
 *  @brief   Удаление сообщения
 *  @param msg - идентификатор сообщения
 *  @return Функция возвращает отрицательное значение в случае ошибки и ENOERR если вызов
 *  был успешен.
 * */
__syscall result_t    msg_destroy( msg_t   msg )
{ return _msg_destroy( OBJ_OFFSET, msg ); }

/*! @fn result_t    msg_send( msg_t   msg );
 *  @brief   Передача сообщения
 *  @param msg - идентификатор отправляемого сообщения
 *  @return Функция возвращает отрицательное значение в случае ошибки и ENOERR если вызов
 *  был успешен.
 * */
__syscall result_t    msg_send( msg_t   msg )
{ return _msg_send( OBJ_OFFSET, msg ); }

/*! @fn void    msg_send_done( msg_t   msg, status_t    status );
 *  @brief   Извещение об окончании процедуры отправки сообщения
 *  @param msg - идентификатор отправляемого сообщения.
 *  @param status - статус завершения процедуры отправки 
 *  был успешен.
 * */
__callback void    msg_send_done( msg_t   msg, status_t    status );
REG_CALLBACK( msg_send_done );

/*! @fn size_t    msg_recv( msg_t   msg );
 *  @brief   Извещение о приёме сообщения
 *  @param msg - идентификатор принятого сообщения.
 * */
__callback void    msg_recv( msg_t   msg );
REG_CALLBACK( msg_recv );

/*! @} */

#endif  /*  _ZZ_MSG_H    */


