#include"ZigParam.h"
#include"types.h"

module ZigCoordM {
   provides {
      interface StdControl;
   }
   uses {
      interface NLME_NetworkFormation as IZigNetFormation;
	  interface NLME_PermitJoining as IZigPermitJoining;
	  interface NIB;
	  interface StdControl as ChildSupervisorControl;
      interface NLME_Leave;
      interface NLME_Reset as IZigReset;
   }
} implementation {


	
#define SCAN_DURATION   1
#define BATTERY_LIFE_EXTENSION   FALSE 
	
task void startDemo()
{
    DBG("Start ZigBee coordinator.");
    call IZigReset.request();
    return;
}
 
task void networkFormation()
{
    DBG("ZigBee network formation...");
    call NIB.setNwkMaxChildren( Z_MAX_CHILDREN );
    call NIB.setNwkMaxRouters( Z_MAX_ROUTERS );
    call NIB.setNwkMaxDepth(Z_MAX_DEPTH);

    call IZigNetFormation.request(
      Z_CHANNELS,
      SCAN_DURATION,
      Z_BEACON_ORDER,
      Z_SUPERFRAME_ORDER,
      Z_PAN_ID,
      BATTERY_LIFE_EXTENSION );
    return;
}


event void IZigReset.confirm(NwkStatus status)
{
    post networkFormation();
    return;
}

   
event void IZigNetFormation.confirm( NwkStatus status )
   {
    if( NWK_SUCCESS == status ) {
       DBG("SUCCESS network formation.");
	   ASSERT(call IZigPermitJoining.request(0xff /*forever*/) == NWK_SUCCESS, "permit joining failure");
	   call ChildSupervisorControl.start();
       return;
    } else {
       DBG("FAILURE network formation.");
       // networkFormation();
       return;
    }
   }
   
 
command result_t StdControl.init()
   {
    return SUCCESS;
   }
command result_t StdControl.start()
   {
    post startDemo();
    return SUCCESS;
   }
command result_t StdControl.stop()
   {
	call ChildSupervisorControl.stop();
    return SUCCESS;
   }

event void NLME_Leave.indication(IEEEAddr DeviceAddr) {}

}

