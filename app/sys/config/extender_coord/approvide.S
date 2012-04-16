	.section .interface.app, "ax", @progbits
	.global	approvides

approvides:
    br  #_reset_vector__
    br	#__process_irq
    br  #__stimer_fired
    br  #__atimer_fired
    br  #__net_send_done
    br  #__net_recv
    br  #__net_enter
    br  #__net_exit
    br  #__net_child_join
    br  #__net_child_leave
    br  #__app_stab

    .section .text, "ax", @progbits
    .global __app_stab
__app_stab:
    mov #-1,r15
    ret
