#-*-Makefile-*- vim:syntax=make
CC:=msp430-gcc
AR:=msp430-ar
OBJCOPY:=msp430-objcopy
CFLAGS:= -Os -mmcu=msp430x1611 -mdisable-hwmul -nostdinc -nostdlib
CFLAGS:=$(CFLAGS) -fexpensive-optimizations -frename-registers -frerun-loop-opt -fstrength-reduce -pipe
addr:=100
conf:=default
platform:=bbox
kind:=0

.PHONY: appsys zig clean app clean_app

target_dirs:= app

ifeq ($(MAKECMDGOALS),appsys)
override target_dirs:= app
endif

ifeq ($(MAKECMDGOALS),clean)
override target_dirs:= app
endif

src_dirs:= $(target_dirs)/sys
include_dirs:= include $(target_dirs)/api $(src_dirs)/include $(src_dirs)/config/$(conf) $(src_dirs)/objsys $(src_dirs)/platforms/$(platform)
cfiles:= $(foreach dir,$(src_dirs),$(wildcard $(dir)/*.c))
Sfiles:= $(foreach dir,$(src_dirs)/config/$(conf),$(wildcard $(dir)/*.S))
ofiles:= $(patsubst %.c,%.o,$(cfiles)) $(patsubst %.S,%.o,$(Sfiles))

CFLAGS:= $(CFLAGS) $(addprefix -I,$(include_dirs)) -DZZ_NODE_KIND=$(kind)

appsys: $(ofiles) $(src_dirs)/lib/libzig.a

app: $(src_dirs)/objsys/libobj.a
	$(MAKE) -C app app platform=$(platform) kind=$(kind)

clean: clean_app
	rm -rf bin/*

clean_app:
	rm -f $(src_dirs)/lib/libzig.a
	rm -f $(src_dirs)/objsys/libobj.a
	rm -f $(ofiles)
	rm -f $(src_dirs)/lib/*.o
	rm -f $(src_dirs)/objsys/*.o
	$(MAKE) -C app clean

zig:
	PLATFORMS=$(platform) $(MAKE) -C zigzag/Configs/$(conf) $(platform)
	if test -d bin/$(platform); then echo; else mkdir bin/$(platform); fi
	$(OBJCOPY) --set-section-flags .infomem=noload zigzag/Configs/$(conf)/build/$(platform)/main.exe bin/$(platform)/zig_$(conf)

infomem:
	cd zigzag/Configs/$(conf);./infomem.sh $(addr) $(channel)

flash:
	zigzag/Configs/$(conf)/flash.sh bin/$(platform)/zig_$(conf)

flash_app:
	$(MAKE) -C app flash

flash_full:
	tools/flash_full.sh bin/$(platform)/zig_$(conf)

$(src_dirs)/lib/libzig.a: $(patsubst %.c,%.o, $(foreach dir,$(src_dirs)/lib,$(wildcard $(dir)/*.c)) )
	$(AR) -q -s $@ $^

$(src_dirs)/objsys/libobj.a: $(patsubst %.c,%.o, $(foreach dir,$(src_dirs)/objsys,$(wildcard $(dir)/*.c)) )
	$(AR) -q -s $@ $^

deb: tools/prep_deb.sh
	tools/prep_deb.sh $(shell pwd)
	dpkg-deb --build ./debian zigzag-2.0.deb

deb-empty: tools/prep_deb.sh
	tools/prep_deb.sh $(shell pwd) -e
	dpkg-deb --build ./debian zigzag-2.0.e.deb

