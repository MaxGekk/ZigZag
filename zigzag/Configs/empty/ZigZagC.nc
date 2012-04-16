configuration ZigZagC {
} implementation {
   components   
                Main
            ,   PowerManagerM
            ,   ZigIrqC as  ZigIrq
            ,   ZigSysC as  ZigSys
            ,   SysTimerC
            ,   ZigSTimerC
            ,   ZigHTimerC  as  ZigHTimer
 //           ,   ZigMonitorC as  ZigMonitor
            ;

    Main.StdControl -> PowerManagerM;	      
    Main.StdControl -> SysTimerC;

    Main.StdControl ->  ZigIrq;
    Main.StdControl ->  ZigHTimer;
    Main.StdControl ->  ZigSys;

//    ZigMonitor.NLME_JoinParent -> NWK;    
   
}

