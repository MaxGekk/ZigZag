#include    <syszig.h>
#include    <types.h>

module ZigRouterM
{
	provides {
        interface StdControl;
    }
	uses {
        interface NLME_PermitJoining;
	    interface NLME_JoinChild;
        interface NLME_StartRouter;
        interface NLME_NetworkDiscovery;
		interface NeighborTable;
        interface NLME_Reset;
        interface NLME_Sync;
        interface NIB;
        interface NLME_Leave;
        interface StdControl as AliveSenderControl;
        interface StdControl as ChildSupervisorControl;
        interface TimerMilli;
    }
} implementation {

enum {
	IDLE_SCAN_INTERVAL	=	20,
	REJOIN_INTERVAL		=	2
};


bool idle_scan;

task void join();
task void sync();
task void start_router();
task void start_discovery();

task void restart()
{
	DBG("=-=-=-=");
	call ChildSupervisorControl.stop();
	call AliveSenderControl.stop();
	call NLME_Reset.request();
}

event void NLME_Reset.confirm(NwkStatus status)
{
	call TimerMilli.setOneShot((idle_scan?IDLE_SCAN_INTERVAL:REJOIN_INTERVAL)*1000);

    return;
}

command result_t StdControl.start()
{
	DBG("NODE %lu", (uint32_t)MAC_AEXTENDED_ADDRESS);
	idle_scan = FALSE;
	//post start_discovery(); //  works buggy this way :(
	post restart();

	return SUCCESS;
}

event result_t TimerMilli.fired()
{
	post start_discovery();

	return SUCCESS;
}

task void start_discovery()
{
	call NIB.setNwkMaxChildren(Z_MAX_CHILDREN);
	call NIB.setNwkMaxRouters(Z_MAX_ROUTERS);
	call NIB.setNwkMaxDepth(Z_MAX_DEPTH);
	idle_scan = FALSE;
	call NLME_NetworkDiscovery.request(Z_CHANNELS, Z_SCAN_ORDER);

    return;
}

event void NLME_NetworkDiscovery.confirm(
		uint8_t 				networkCount,
		NwkNetworkDescriptor* 	networkDescriptors,
		NwkStatus 				status
		)
{
	DBG("discovery status %hhx, networks %hhu", status, networkCount);
	if (status == NWK_SUCCESS) {
		NwkNeighbor* pnbr;

		while (call NeighborTable.getNextPtr(&pnbr) == SUCCESS)
		{
            if (pnbr->panID == Z_PAN_ID && pnbr->permitJoining)
			{
				post join();
                return;
			}
        }
	}
	idle_scan = TRUE;
	post restart();

}

task void join()
{
	call NLME_JoinChild.request(
			Z_PAN_ID,
			TRUE, /*as router*/
			FALSE, /*rejoin*/
			0, /*scan channels - ignored*/
			0, /*scan duration - ignored*/
			0, /*power source*/
			TRUE, /*rx on when idle*/
			FALSE /*security*/);

    return;
}


event void NLME_JoinChild.confirm(
		PanID_t 	panID,
		NwkStatus 	status
		)
{
	DBG("join confirm: %hhx", status);
	if (status != NWK_SUCCESS)
		post restart();
	else {
		call AliveSenderControl.start();
		post start_router();
	}
	return;
}

task void start_router()
{
    #ifndef ZC_END_DEVICE
    call NLME_StartRouter.request( 
			0, /*forced slot number (0 == not forced)*/
			FALSE);
    #endif

    return;
}

event void NLME_StartRouter.confirm(NwkStatus status)
{
	DBG("start router: %hhx", status);

	if (status == NWK_SUCCESS)	{
	 	call NLME_PermitJoining.request(0xff /*forever*/);
		call ChildSupervisorControl.start();
	} else if (status == NWK_INVALID_REQUEST) {
	 	call NLME_PermitJoining.request(0 /*never*/);
	} else
		post restart();

    return;
}

event void NLME_Sync.indication()
{
	DBG("SYNC LOSS");
	post restart();

    return;
}

event void NLME_Leave.indication( IEEEAddr deviceAddr )
{
	if (deviceAddr == MAC_AEXTENDED_ADDRESS) {
		DBG("leaving... ");
		post restart();
	}

    return;
}

command result_t StdControl.init()
{
	return SUCCESS;
}
command result_t StdControl.stop()
{
	return SUCCESS;
}

event void NLME_Sync.confirm(NwkStatus status) {}

}

