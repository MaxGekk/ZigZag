#-*-Makefile-*- vim:syntax=make
CC:=msp430-gcc
SIZE:=msp430-size
CFLAGS:=-O -w -mmcu=msp430x1611 -mdisable-hwmul
sys_dirs:= sys
conf:=default
include_dirs:= api $(sys_dirs)/include $(sys_dirs)/config/$(conf) $(sys_dirs)/objsys $(sys_dirs)/platforms/$(platform)

override obj:=$(addprefix obj/, $(obj))
cfiles:= $(foreach dir,$(obj),$(wildcard $(dir)/*.c))
ofiles:= $(sort $(patsubst %.c,%.o,$(cfiles)) $(foreach dir,$(obj),$(wildcard $(dir)/*.o)) )
sysofiles:=$(foreach dir,$(sys_dirs) $(sys_dirs)/config/$(conf),$(wildcard $(dir)/*.o))
ldscript:=$(sys_dirs)/config/$(conf)/ldscript

CFLAGS:= $(CFLAGS) $(addprefix -I,$(include_dirs)) -DZZ_NODE_KIND=$(kind)

.PHONY: all clean

all:app

app: $(ofiles) $(sysofiles) $(sys_dirs)/objsys/libobj.a $(sys_dirs)/lib/libzig.a
	$(CC) $(CFLAGS) -Xlinker -T $(ldscript) $^ -o ../bin/appzig_$(conf)
	$(SIZE) ../bin/appzig_$(conf)

$(ofiles): $(cfiles)

flash:
	sys/config/$(conf)/flash.sh ../bin/appzig_$(conf)

clean:
	rm -f $(ofiles)


