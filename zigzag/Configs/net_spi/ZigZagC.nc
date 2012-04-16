configuration ZigZagC {
} implementation {
   components   
                Main
            ,   ZigCoordC as ZigBeeCoordinator
            ,   PowerManagerM
            ,   ChildSupervisorC as ChildSupervisor
            ,   MacCoordC   as  MAC
            ,   NwkRootC    as  NWK	    
            ,   ZigNet2SpiC as  ZigNet2Spi
            ;

    Main.StdControl -> PowerManagerM;	      
    Main.StdControl -> ZigBeeCoordinator;
    Main.StdControl -> ZigNet2Spi;

    ZigNet2Spi.IMacGET  ->  MAC;
    ZigNet2Spi.NLDE_Data    ->  NWK;
    ZigNet2Spi.NLME_Reset -> NWK;
    ZigNet2Spi.NLME_JoinParent -> NWK;
    ZigNet2Spi.NLME_NetworkFormation -> NWK;
}

