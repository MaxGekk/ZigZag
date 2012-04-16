#ifndef     __ZZ_IRQ_H
#define     __ZZ_IRQ_H

#include    <zzTypes.h>
#include    <_zzMacro.h>

typedef void    (* irq_handler_ft)(irq_t); /* Пользовательский обработчик прерывания */

extern  const irq_handler_ft    _begin_irq_handler;
extern  const irq_handler_ft    _end_irq_handler;

typedef void    (*irq_handler_t)(); /* Системный обработчик прерывания */

extern const irq_handler_t __irq_start; /*!< Указатель на первую функцию-обработчик прерывания */

#define IRQ_HANDLER( irq_num ) _IRQ_HANDLER_( irq_num )
#define _IRQ_HANDLER_( irq_num ) __attribute__((used)) __##irq_num##_interrupt()

#endif  /*  __ZZ_IRQ_H */

