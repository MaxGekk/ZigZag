module  ZigMonitorM {
    uses {
        interface NLME_JoinParent;
        interface NLME_Leave;
    }
} implementation {

event void NLME_JoinParent.indication( NwkAddr   shortAddr, IEEEAddr   extendedAddr,
    NwkCapabilityInfo   capabilityInformation,  bool   secureJoin, uint8_t lqi ) 
{
    return;
}

event void NLME_Leave.indication( IEEEAddr deviceAddr )
{
    return;
}

}

