#!/bin/bash

if [[ -z $1 ]]
then 
    MAC_ADDRESS="MAC_ADDRESS=3";
    echo no mac address specified, keeping the existing one;
else 
    MAC_ADDRESS="MAC_ADDRESS=$1";
fi
echo flashing $MAC_ADDRESS

if [[ -z $2 ]]
then 
    CHANNELS="Z_CHANNELS=0x8000";
    echo no channel set specified, setting default;
else 
    CHANNELS="Z_CHANNELS=$2";
fi
echo flashing $CHANNELS

../../../tools/infomem.py $MAC_ADDRESS $CHANNELS Z_PAN_ID=243 Z_MAX_CHILDREN=4 Z_MAX_ROUTERS=4 Z_MAX_DEPTH=7 Z_BEACON_ORDER=6 Z_SUPERFRAME_ORDER=0 Z_SCAN_ORDER=6 ALIVE_SEND_PERIOD=20 ALIVE_SEND_ATTEMPTS=3
