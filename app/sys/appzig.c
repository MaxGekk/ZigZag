#include    <_zigzag.h>
#include    <syszig.h>

__attribute__((used)) void __process_irq( uint16_t irq_num )
{
    { /* Если зарегистрирован системный обработчик прерывания, то
         вызываем его и выходим */
     const irq_handler_t *irq_handler = &__irq_start;
     irq_handler += (irq_num >> 1); /* поскольку передаётся смещение вектора прерывания в байтах */
     if( EXISTS( *irq_handler ) ) {
        (*irq_handler)();
        return;
     }
    }
    {/* Вызываем все обработчики прерываний прикладных объектов */
     const irq_handler_ft *handler = &_begin_irq_handler;
     for(; handler < &_end_irq_handler; handler++ )
        (*handler)((irq_t)irq_num);
    }
    return;
}
__attribute__((weak)) void    irq_handler( irq_t  irq ) {}

uint64_t    _sys_time() { return ISZIGLOAD ?__sys_time():0; }

volatile uint64_t* __workaround2;
int16_t     _set_sys_time( uint64_t  time )
{
   __workaround2 = &time;
    if( ISZIGLOAD )
        return __set_sys_time(time);
    return -1;
}

__attribute__((used, noinline, section(".init8"))) void __app_init()
{
    {/* Инициализация системы */
     init_func_t *init_func = &__init_start;
     for (; init_func < &__init_end; init_func++)
        (*init_func) ();
    }
    {/*  инициализация прикладных объектов */
     const sys_init_ft *init_func = &_begin_sys_init;
     for (; init_func < &_end_sys_init; init_func++)
        (*init_func)();
    }
}

__attribute__((weak)) void    sys_init() {}

int main() {}

