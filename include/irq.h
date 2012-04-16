#ifndef   _IRQ_H
#define   _IRQ_H

/*! @file   irq.h
 *  @brief  Работа с прерываниями
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1 
 */

#include    <inttypes.h>

/*  @defgroup critical Критические секции
 *  @{ 
 *      Данный интерфейс реализует механизм критических секций.
 *  Критическая секция - это блок кода, внутри которого отключены аппаратные прерывания.
 *  Время нахождения в критической секции должно быть сведено к минимуму. Допустимы
 *  вложенные критические секции.
 * */

/*! @fn void __critical_enter()
 *  @brief Вход в критическую секцию.
 * */
void __critical_enter();

/*! @fn void __critical_exit()
 *  @brief Выход из критической секции.
 * */
void __critical_exit();

/*! @def critical_block( block )
 *  @brief Размещение блока кода @a block в критической секции.
 * */
#define __critical_block( block )   __critical_enter(); { block;} __critical_exit();
/*! @}*/

/*  @defgroup interrupt Обработка прерываний
 *  @{ 
 *      Интерфейс обработчика прерывания.
 * */

/*! @fn void    __process_irq( uint16_t irq_num );
 *  @brief Обработчик прерываний
 *  @param irq_num - номер прерывания
 * */
void    __process_irq( uint16_t irq_num );

/*! @} */

#endif  /* _IRQ_H */

