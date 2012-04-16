configuration ZigCoordC {
    provides    interface StdControl;
} implementation {
   components  
                ZigCoordM as ZigBeeCoordinator
              , NwkRootC as NWK	      
              , ChildSupervisorC
              ;

   StdControl = ZigBeeCoordinator;
   StdControl = NWK;

   ZigBeeCoordinator.IZigReset -> NWK.NLME_Reset;
   ZigBeeCoordinator.IZigNetFormation -> NWK.NLME_NetworkFormation;
   ZigBeeCoordinator.NIB -> NWK;
   ZigBeeCoordinator.IZigPermitJoining -> NWK.NLME_PermitJoining;

   ZigBeeCoordinator.ChildSupervisorControl -> ChildSupervisorC;
   ZigBeeCoordinator.NLME_Leave -> ChildSupervisorC;
    
}

