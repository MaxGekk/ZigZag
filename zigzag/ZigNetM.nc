#include    <types.h>
#include    <syszig.h>

module ZigNetM {
#ifdef  ZC_SLEEP_NOTIFY
    provides {
      event void sleepIndication( TMilliSec sleep_duration );
    }
#endif
    uses {
        interface   IMacGET;
        interface   NLDE_Data;
        interface   NLME_JoinChild;
        interface   NLME_Reset;
        interface   NLME_JoinParent;
        interface   NLME_Leave;
    }
} implementation {

#define   MAX_HOPS   0xff
#define   DISCOVER_ROUTE   0x0
#define   SECURITY_ENABLE   FALSE

int16_t __net_send( const uint16_t dst_addr, const uint16_t data_size,
        const uint8_t *const data, uint8_t handle ) @spontaneous() @C()
{
	result_t res;
    res = call NLDE_Data.request(
        dst_addr, 
        data_size, (uint8_t *)data, handle,
        MAX_HOPS, DISCOVER_ROUTE, SECURITY_ENABLE
    );
    
    return (res==SUCCESS)?0:-5;
}

event void NLDE_Data.confirm( uint8_t nsduHandle, NwkStatus status)
{
    if( ISMODLOAD )
        __net_send_done( nsduHandle, ( NWK_SUCCESS == status )?0:-1 );

    return;
}

event void NLDE_Data.indication( NwkAddr srcAddr, IEEEAddr srcExtAddr, uint8_t nsduLength,
    uint8_t* nsdu,	uint8_t linkQuality)
{
    if( ISMODLOAD )
        __net_recv( (uint16_t)srcAddr, srcExtAddr, (uint16_t)nsduLength, nsdu, linkQuality );

    return;
}

uint16_t    __net_addr() @spontaneous() @C()
{
    TMacStatus   status;
    TMacShortAddress   shortAddress = call IMacGET.GetmacShortAddress( &status );
    if(  MAC_SUCCESS == status )
       return (uint16_t)shortAddress;
    return 0xffff;
}

event void NLME_JoinChild.confirm( PanID_t   panID,  NwkStatus   status) 
{
    if( (NWK_SUCCESS == status)&&( ISMODLOAD) )
        __net_enter( (uint16_t)panID );
    return;
}

event void NLME_Reset.confirm(NwkStatus status)
{
    if( ISMODLOAD )
        __net_exit();
    return;
}

event void NLME_JoinParent.indication(	NwkAddr	shortAddr,	IEEEAddr extendedAddr,	
    NwkCapabilityInfo	capabilityInformation,	bool	secureJoin, uint8_t lqi	)
{
    if( ISMODLOAD )
        __net_child_join( shortAddr, extendedAddr, lqi );
    return;
}

event void NLME_Leave.indication(IEEEAddr DeviceAddr)
{
    if( ISMODLOAD )
        __net_child_leave( DeviceAddr );
    return;
}

#ifdef  ZC_SLEEP_NOTIFY
event void sleepIndication( TMilliSec sleep_duration )
{
    if( ISMODLOAD )
        __net_sleep( sleep_duration );
} 
#endif

}

