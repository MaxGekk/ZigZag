configuration ZigZagC {
} implementation {
   components   
                Main
            ,   PowerManagerM
            ,   ZigIrqC as  ZigIrq
            ,   ZigSysC as  ZigSys
            ,   SysTimerC
            ,   ZigSTimerC
            //,   ZigHTimerC  as  ZigHTimer
            ,   ZigTimerAIrqC
            ,   ZigSpi2AppC
            ;
	    
    Main.StdControl -> PowerManagerM;	      
    Main.StdControl -> SysTimerC;
    Main.StdControl -> ZigSpi2AppC;


    Main.StdControl ->  ZigIrq;
    //Main.StdControl ->  ZigHTimer;
    Main.StdControl ->  ZigSys;

}

