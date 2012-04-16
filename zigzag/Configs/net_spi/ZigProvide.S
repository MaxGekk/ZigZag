	.section .interface.zig, "ax", @progbits
	.global	zigrovides

zigrovides:
    br  #__critical_enter
    br  #__critical_exit
    br	#__sys_time
    br  #__net_addr
    br  #__net_send
    br  #__atimer_counter
    br  #__atimer_set
    br  #__atimer_stop
    br  #__atimer_is_set
    br  #__atimer_point
    br  #__stimer_set
    br  #__port_perm
    br  #__post_task
    br  #__zig_stab

    .section .text, "ax", @progbits
    .global __zig_stab
__zig_stab:
    mov #-1,r15
    ret

