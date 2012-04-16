#ifndef     _ZZ_EVENT_H
#define     _ZZ_EVENT_H

/*! @file   zzEvent.h
 *  @brief  Объявление интерфейса системных событий
 *  @author Max Gekk
 *  @date   июнь 2007
 *  @version 5
 */   

#include    <zzTypes.h>
#include    <_zzEvent.h>

/*! @defgroup ZIG_EVENT Интерфейс системных событий
 *  @ingroup ZIGZAG
 *  @{
 * */

/*! @fn result_t   event_emit( priority_t  priority, event_type_t  event_type, unidata_t   unidata );
 *  @brief  Генерация события
 *  @param  priority - приоритет события. Чем больше значение аргумента, тем выше приоритет.
 *  @param  event_type - тип события.
 *  @param  unidata - данные, связанные с событием.
 * */
__syscall result_t   event_emit( priority_t  priority, event_type_t  event_type, unidata_t   unidata )
{ return _event_emit( OBJ_OFFSET, priority, event_type, unidata ); }

/*! @fn void    event_handler( event_type_t event_type, unidata_t   unidata );
 *  @brief  Обработчик события
 *  @param  event_type - тип события.
 *  @param  unidata - данные, связанные с событием.
 * */
__callback void    event_handler( event_type_t event_type, unidata_t   unidata );
REG_CALLBACK( event_handler );

/* @} */
#endif  /*  _ZZ_EVENT_H */

