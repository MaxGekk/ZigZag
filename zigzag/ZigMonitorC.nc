configuration ZigMonitorC {
    uses {
        interface   NLME_JoinParent;
    }
} implementation {

    components  
                ZigMonitorM as ZigMonitor
            ,   ChildSupervisorC
            ;

    NLME_JoinParent = ZigMonitor.NLME_JoinParent;

    ZigMonitor.NLME_Leave -> ChildSupervisorC;

}

