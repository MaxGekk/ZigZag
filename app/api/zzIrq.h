#if !defined(_ZZ_IRQ_H) && defined(ZC_IRQ)
#define     _ZZ_IRQ_H

/*! @file   zzIrq.h
 *  @brief  Объявление интерфейса аппаратных прерываний
 *  @author Max Gekk
 *  @date   июнь 2007
 *  @version 5
 */   

#include    <zzTypes.h>
#include    <_zzIrq.h>

/*! @defgroup ZIG_IRQ Интерфейс аппаратных прерываний
 *  @ingroup ZIGZAG
 *  @{
 * */

/*! @fn void    irq_handler( irq_t  irq );
 *  @brief   Обработчик прерываний
 *  @param irq - номер прерывания
 * */
__callback void    irq_handler( irq_t  irq );
REG_CALLBACK( irq_handler );

/*  @} */

#endif  /*  _ZZ_IRQ_H */

