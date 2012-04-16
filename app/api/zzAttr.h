#ifndef     _ZZ_ATTR_H
#define     _ZZ_ATTR_H

/*! @file   zzAttr.h
 *  @brief  Объявление интерфейса доступа к атрибутам
 *  @author Max Gekk
 *  @date   июнь 2007
 *  @version 5
 */   

#include    <zzTypes.h>
#include    <_zzAttr.h>

/*! @defgroup ZIG_ATTR Интерфейс доступа к атрибутам
 *  @ingroup ZIGZAG
 *  @{
 * */

/*! @fn result_t    attr_read( const uint8_t    anum, void  *to );   
 *  @brief   Чтение значения атрибута
 *  @param anum - номер атрибута
 *  @param to - указатель на область памяти, в которую будет скопировано текущее значение атрибута
 *  @return Результат выполнения функции
 * */
__syscall result_t    attr_read( const uint8_t    anum, void  *to )
{
    result_t res = _attr_get( OBJ_OFFSET, anum, to );
    return res?res:EINVAL;
}


/*! @fn result_t    attr_write( const uint8_t    anum, void  *from );   
 *  @brief   Запись значения атрибута
 *  @param anum - номер атрибута
 *  @param from - указатель на область памяти, из которой будет взято новое значение атрибута
 *  @return Результат выполнения функции
 * */
__syscall result_t    attr_write( const uint8_t    anum, void  *from )
{ 
    result_t res = _attr_set( OBJ_OFFSET, anum, from ); 
    return res?res:EINVAL;
}

/*! @} */

/* Данная константа позволяет слинковать системные функции доступа к атрибутам */
ATTR_HOOK

#endif  /*  _ZZ_ATTR_H    */

