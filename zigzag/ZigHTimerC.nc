configuration ZigHTimerC {
    provides {
        interface StdControl;
    }
} implementation {

    components      ZigHTimerM  as ZigHTimer
                ,   MSP430TimerC
                ;

    StdControl  = ZigHTimer;
    
    ZigHTimer.Timer -> MSP430TimerC.TimerA;
 
    ZigHTimer.AlarmControl0 -> MSP430TimerC.ControlA0;
    ZigHTimer.AlarmCompare0 -> MSP430TimerC.CompareA0;

    ZigHTimer.AlarmControl1 -> MSP430TimerC.ControlA1;
    ZigHTimer.AlarmCompare1 -> MSP430TimerC.CompareA1;

    ZigHTimer.AlarmControl2 -> MSP430TimerC.ControlA2;
    ZigHTimer.AlarmCompare2 -> MSP430TimerC.CompareA2;

}

