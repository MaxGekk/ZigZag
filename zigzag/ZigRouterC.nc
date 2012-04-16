configuration ZigRouterC{
   provides interface StdControl;
} implementation {
	components 
                   Main
                 , ZigRouterM as ZigRouter
                 , NwkRouterC
                 , SysTimerC
                 , AliveSenderC
                 , ChildSupervisorC
                 ;

	StdControl = ZigRouter;
	StdControl = NwkRouterC;
	ZigRouter.NLME_PermitJoining 	-> NwkRouterC;
	ZigRouter.NLME_JoinChild		-> NwkRouterC;
	ZigRouter.NLME_StartRouter 	-> NwkRouterC;
	ZigRouter.NLME_NetworkDiscovery	-> NwkRouterC;
	ZigRouter.NLME_Reset	-> NwkRouterC;
	ZigRouter.NLME_Sync	-> NwkRouterC;
	ZigRouter.NIB			-> NwkRouterC;
	ZigRouter.NLME_Leave	-> AliveSenderC;
    ZigRouter.NeighborTable -> NwkRouterC;
	
	ZigRouter.ChildSupervisorControl	-> ChildSupervisorC;
	ZigRouter.AliveSenderControl		-> AliveSenderC;

	ZigRouter.TimerMilli 	-> SysTimerC.TimerMilli[unique("TimerMilli")];
	
}

