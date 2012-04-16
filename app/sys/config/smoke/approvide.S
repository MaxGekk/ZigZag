#include    "config.h"
	.section .interface.app, "ax", @progbits
	.global	approvides

approvides:
    br  #_reset_vector__
#ifdef  ZC_IRQ
    br	#__process_irq
#else
    br	#__app_stab
#endif
#ifdef  ZC_STIMER
    br  #__stimer_fired
#else
    br  #__app_stab
#endif
#ifdef  ZC_ATIMER
    br  #__atimer_fired
#else
    br  #__app_stab
#endif
    br  #__net_send_done
    br  #__net_recv
    br  #__net_enter
    br  #__net_exit
    br  #__net_child_join
    br  #__net_child_leave
#ifdef  ZC_SLEEP_NOTIFY
    br  #__net_sleep
#else
    br  #__app_stab
#endif

    .section .text, "ax", @progbits
    .global __app_stab
__app_stab:
    mov #-1,r15
    ret
