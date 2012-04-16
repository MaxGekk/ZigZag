#include    <ZigParam.h>
configuration ZigZagC {
} implementation {
    components
                Main
#ifdef  ZC_ZIGBEE
            ,   ZigCoordC as ZigBeeCoordinator
#endif
#ifdef  ZC_POWER
            ,   PowerManagerM
#endif
#ifdef  ZC_IRQ
            ,   ZigIrqC as  ZigIrq
#endif
#if defined(ZC_INIT) || defined(ZC_PORT) || defined(ZC_TIME64)
            ,   ZigSysC as  ZigSys
#endif
#ifdef  ZC_STIMER
            ,   ZigSTimerC
#endif
#ifdef  ZC_HTIMER
            ,   ZigHTimerC  as  ZigHTimer
#else
            ,   ZigTimerAIrqC
#endif
#ifdef  ZC_ZIGBEE
            ,   ZigNetM as  ZigNet
            ,   ZigMonitorC as  ZigMonitor

            ,   ChildSupervisorC as ChildSupervisor
	    
            ,   MacCoordC   as  MAC
            ,   NwkRootC    as  NWK	    
#endif
            ;

#ifdef  ZC_POWER
    Main.StdControl -> PowerManagerM;
#endif
#ifdef  ZC_ZIGBEE
    Main.StdControl -> ZigBeeCoordinator;
#endif
#ifdef  ZC_IRQ
    Main.StdControl ->  ZigIrq;
#endif
#ifdef  ZC_HTIMER
    Main.StdControl ->  ZigHTimer;
#endif
#if defined(ZC_INIT) || defined(ZC_PORT) || defined(ZC_TIME64)
    Main.StdControl ->  ZigSys;
#endif

#ifdef  ZC_ZIGBEE
    ZigNet.IMacGET  ->  MAC;
    ZigNet.NLDE_Data    ->  NWK;
    ZigNet.NLME_Reset -> NWK;
    ZigNet.NLME_JoinParent -> NWK;
    ZigNet.NLME_Leave -> ChildSupervisor;
#ifdef  ZC_SLEEP_NOTIFY
    ZigNet.sleepIndication <- MAC.sleepIndication; 
#endif
    ZigMonitor.NLME_JoinParent -> NWK;    
#endif
}

