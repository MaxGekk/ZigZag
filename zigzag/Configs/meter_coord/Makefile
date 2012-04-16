#-*-Makefile-*- vim:syntax=make
COMPONENT=ZigZagC
top_dir:=../../../

NWK_DIR:=$(top_dir)/zigzag/ZigBee
STD_802_15_4:=$(top_dir)/zigzag/IEEE802_15_4
include $(NWK_DIR)/Makerules

# mac addresses:
# 11..19 old small sensor nodes
# 20..29 old big MKNT nodes
# 31..44 new small sensor nodes (with radio switch)
#MAC_ADDRESS:=100

CFLAGS:= -DBASE_CLK_8MHZ $(CFLAGS)
CFLAGS:= -DNEW_LEDS $(CFLAGS)		# for small sensor nodes
CFLAGS:= -DRADIO_POWER_PIN $(CFLAGS)		# for new small sensor nodes
CFLAGS:= -DTOSH_MAX_TASKS_LOG2=4 $(CFLAGS)
CFLAGS:= -DBUSY_USART0 $(CFLAGS)

CFLAGS:=-I$(top_dir)/include -I$(top_dir)app/api -I$(top_dir)/zigzag $(CFLAGS)
CFLAGS:=ldscript ZigProvide.S $(CFLAGS)
CFLAGS:=$(CFLAGS) -_fnesc-include=config

