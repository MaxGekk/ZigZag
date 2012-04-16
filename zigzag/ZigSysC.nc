configuration ZigSysC {
    provides    interface   StdControl;
} implementation {
    components  
                ZigSysM as  ZigSys
            ,   SysTimerC   as  SysTimer
            ;

    StdControl  =   ZigSys;

    ZigSys.ILocalTime64 -> SysTimer;
                
}

