configuration ZigZagC {
} implementation {
   components   
                Main
            ,   ZigCoordC as ZigBeeCoordinator
            ,   PowerManagerM
            ,   ZigIrqC as  ZigIrq
            ,   ZigSysC as  ZigSys
            ,   ZigSTimerC
#if     defined(ZC_HTIMER)
            ,   ZigHTimerC  as  ZigHTimer
#else
            ,   ZigTimerAIrqC
#endif
            ,   ZigNetM as  ZigNet
            ,   ZigMonitorC as  ZigMonitor

            ,   ChildSupervisorC as ChildSupervisor
	    
            ,   MacCoordC   as  MAC
            ,   NwkRootC    as  NWK	    
            ;

    Main.StdControl -> PowerManagerM;	      
    Main.StdControl -> ZigBeeCoordinator;

    Main.StdControl ->  ZigIrq;
#if     defined(ZC_HTIMER)
    Main.StdControl ->  ZigHTimer;
#endif
    Main.StdControl ->  ZigSys;
    
    ZigNet.IMacGET  ->  MAC;
    ZigNet.NLDE_Data    ->  NWK;
    ZigNet.NLME_Reset -> NWK;
    ZigNet.NLME_JoinParent -> NWK;
    ZigNet.NLME_Leave -> ChildSupervisor;

    ZigMonitor.NLME_JoinParent -> NWK;    
   
}

