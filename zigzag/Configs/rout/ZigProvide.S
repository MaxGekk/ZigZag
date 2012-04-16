#include "config.h"
	.section .interface.zig, "ax", @progbits
	.global	zigrovides

zigrovides:
    br  #__critical_enter
    br  #__critical_exit
#ifdef  ZC_TIME64
    br	#__sys_time
#else
    br	#__zig_stab
#endif
#ifdef  ZC_ZIGBEE
    br  #__net_addr
    br  #__net_send
#else
    br  #__zig_stab
    br  #__zig_stab
#endif
#ifdef  ZC_HTIMER
    br  #__atimer_counter
    br  #__atimer_set
    br  #__atimer_stop
    br  #__atimer_is_set
    br  #__atimer_point
#else
    br  #__zig_stab
    br  #__zig_stab
    br  #__zig_stab
    br  #__zig_stab
    br  #__zig_stab
#endif
#ifdef  ZC_STIMER
    br  #__stimer_set
#else
    br  #__zig_stab
#endif
#ifdef  ZC_PORT
    br  #__port_perm
#else
    br  #__zig_stab
#endif
    br  #__post_task
#ifdef  ZC_TIME64
    br  #__set_sys_time
#else
    br	#__zig_stab
#endif

    .section .text, "ax", @progbits
    .global __zig_stab
__zig_stab:
    mov #-1,r15
    ret

