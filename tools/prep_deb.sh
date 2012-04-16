#!/usr/bin/env bash
topdir=$1
debdir=${topdir}/debian
install  -d ${debdir}/bin ${debdir}/app ${debdir}/tools ${debdir}/zigzag

install  -m a=r  ${topdir}/Makefile ${debdir}/Makefile
install  -m a=r  ${topdir}/readme.txt ${debdir}/readme.txt

install  -m a=rx ${topdir}/tools/comcat.py ${debdir}/tools/comcat.py
install  -m a=rx ${topdir}/tools/elf.py ${debdir}/tools/elf.py
install  -m a=rx ${topdir}/tools/flash_full.sh ${debdir}/tools/flash_full.sh
install  -m a=rx ${topdir}/tools/infomem.py ${debdir}/tools/infomem.py

install  -d ${debdir}/zigzag/Configs 

if [ "$2" == "-e" ]
then
install  -d ${debdir}/zigzag/Configs/empty

install  -m a=rx ${topdir}/zigzag/Configs/empty/flash.sh ${debdir}/zigzag/Configs/empty/flash.sh
install  -m a=rx ${topdir}/zigzag/Configs/empty/infomem.sh ${debdir}/zigzag/Configs/empty/infomem.sh
install  -m a=r  ${topdir}/zigzag/Configs/empty/infomem.cfg ${debdir}/zigzag/Configs/empty/infomem.cfg

install  -m a=rx ${topdir}/bin/zig_empty.exe ${debdir}/bin/zig_empty.exe
else
install  -d ${debdir}/zigzag/Configs/rout ${debdir}/zigzag/Configs/coord
install  -m a=rx ${topdir}/zigzag/Configs/coord/flash.sh ${debdir}/zigzag/Configs/coord/flash.sh
install  -m a=rx ${topdir}/zigzag/Configs/coord/infomem.sh ${debdir}/zigzag/Configs/coord/infomem.sh
install  -m a=r  ${topdir}/zigzag/Configs/coord/infomem.cfg ${debdir}/zigzag/Configs/coord/infomem.cfg

install  -m a=rx ${topdir}/zigzag/Configs/rout/flash.sh ${debdir}/zigzag/Configs/rout/flash.sh
install  -m a=rx ${topdir}/zigzag/Configs/rout/infomem.sh ${debdir}/zigzag/Configs/rout/infomem.sh
install  -m a=r  ${topdir}/zigzag/Configs/rout/infomem.cfg ${debdir}/zigzag/Configs/rout/infomem.cfg

install  -m a=rx ${topdir}/bin/zig_rout.exe ${debdir}/bin/zig_rout.exe
install  -m a=rx ${topdir}/bin/zig_coord.exe ${debdir}/bin/zig_coord.exe
fi

install  -d ${debdir}/doc
install  -m a=r  ${topdir}/doc/zig_swarm/zig_swarm.pdf ${debdir}/doc/zig_swarm.pdf

sappdir=${topdir}/app
dappdir=${debdir}/app

install  -m a=r ${sappdir}/Makefile ${dappdir}/Makefile
install  -m a=r ${sappdir}/readme.txt ${dappdir}/readme.txt

install  -d ${dappdir}/api ${dappdir}/doc ${dappdir}/obj ${dappdir}/sys
install  -m a=r  ${sappdir}/api/inttypes.h ${dappdir}/api/inttypes.h
install  -m a=r  ${sappdir}/api/zigzag.h ${dappdir}/api/zigzag.h
install  -m a=r  ${sappdir}/api/zzATimer.h ${dappdir}/api/zzATimer.h
install  -m a=r  ${sappdir}/api/zzAttr.h ${dappdir}/api/zzAttr.h
install  -m a=r  ${sappdir}/api/zzBind.h ${dappdir}/api/zzBind.h
install  -m a=r  ${sappdir}/api/zzComBuf.h ${dappdir}/api/zzComBuf.h
install  -m a=r  ${sappdir}/api/zzConst.h ${dappdir}/api/zzConst.h
install  -m a=r  ${sappdir}/api/zzDebug.h ${dappdir}/api/zzDebug.h
install  -m a=r  ${sappdir}/api/zzEvent.h ${dappdir}/api/zzEvent.h
install  -m a=r  ${sappdir}/api/zzIrq.h ${dappdir}/api/zzIrq.h
install  -m a=r  ${sappdir}/api/zzMsg.h ${dappdir}/api/zzMsg.h
install  -m a=r  ${sappdir}/api/zzPort.h ${dappdir}/api/zzPort.h
install  -m a=r  ${sappdir}/api/zzSTimer.h ${dappdir}/api/zzSTimer.h
install  -m a=r  ${sappdir}/api/zzSys.h ${dappdir}/api/zzSys.h
install  -m a=r  ${sappdir}/api/zzTypes.h ${dappdir}/api/zzTypes.h

install  -m a=r  ${sappdir}/doc/zigzag_api/zigzag_api.pdf ${dappdir}/doc/zigzag_api.pdf

ssysdir=${sappdir}/sys
dsysdir=${dappdir}/sys
install  -d ${dsysdir}/include ${dsysdir}/ldx ${dsysdir}/lib ${dsysdir}/objsys

install  -m a=r  ${ssysdir}/approvide.o ${dsysdir}/approvide.o
install  -m a=r  ${ssysdir}/appzig.o ${dsysdir}/appzig.o
install  -m a=r  ${ssysdir}/atimer.o ${dsysdir}/atimer.o
install  -m a=r  ${ssysdir}/attr.o ${dsysdir}/attr.o
install  -m a=r  ${ssysdir}/bind.o ${dsysdir}/bind.o
install  -m a=r  ${ssysdir}/event.o ${dsysdir}/event.o
install  -m a=r  ${ssysdir}/network.o ${dsysdir}/network.o
install  -m a=r  ${ssysdir}/stimer.o ${dsysdir}/stimer.o

sinclude=${ssysdir}/include
dinclude=${dsysdir}/include
install  -m a=r  ${sinclude}/_zigzag.h ${dinclude}/_zigzag.h
install  -m a=r  ${sinclude}/_zzATimer.h ${dinclude}/_zzATimer.h
install  -m a=r  ${sinclude}/_zzAttr.h ${dinclude}/_zzAttr.h
install  -m a=r  ${sinclude}/_zzBind.h ${dinclude}/_zzBind.h
install  -m a=r  ${sinclude}/_zzEvent.h ${dinclude}/_zzEvent.h
install  -m a=r  ${sinclude}/_zzInit.h ${dinclude}/_zzInit.h
install  -m a=r  ${sinclude}/_zzIrq.h ${dinclude}/_zzIrq.h
install  -m a=r  ${sinclude}/_zzMacro.h ${dinclude}/_zzMacro.h
install  -m a=r  ${sinclude}/_zzMsg.h ${dinclude}/_zzMsg.h
install  -m a=r  ${sinclude}/_zzPort.h ${dinclude}/_zzPort.h
install  -m a=r  ${sinclude}/_zzSTimer.h ${dinclude}/_zzSTimer.h
install  -m a=r  ${sinclude}/_zzSys.h ${dinclude}/_zzSys.h
install  -m a=r  ${sinclude}/msp430.h ${dinclude}/msp430.h
install  -m a=r  ${sinclude}/msp430uart0.h ${dinclude}/msp430uart0.h
install  -m a=r  ${sinclude}/msp430uart1.h ${dinclude}/msp430uart1.h

install  -m a=rx ${ssysdir}/ldx/default.sh ${dsysdir}/ldx/default.sh
install  -m a=r  ${ssysdir}/ldx/default.ldx ${dsysdir}/ldx/default.ldx
install  -m a=rx ${ssysdir}/ldx/coord.sh ${dsysdir}/ldx/coord.sh
install  -m a=r  ${ssysdir}/ldx/coord.ldx ${dsysdir}/ldx/coord.ldx
install  -m a=rx ${ssysdir}/ldx/rout.sh ${dsysdir}/ldx/rout.sh
install  -m a=r  ${ssysdir}/ldx/rout.ldx ${dsysdir}/ldx/rout.ldx

install  -m a=r  ${ssysdir}/lib/libzig.a ${dsysdir}/lib/libzig.a
install  -m a=r  ${ssysdir}/objsys/libobj.a ${dsysdir}/objsys/libobj.a

install  -d ${dappdir}/obj/blinker ${dappdir}/obj/button
install  -m a=rw ${sappdir}/obj/blinker/blinker.c ${dappdir}/obj/blinker/blinker.c
install  -m a=rw ${sappdir}/obj/button/button.c ${dappdir}/obj/button/button.c

