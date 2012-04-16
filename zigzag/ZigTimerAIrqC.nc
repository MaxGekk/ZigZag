configuration ZigTimerAIrqC {
} implementation {

    components      ZigTimerAIrqM  as ZigTimerIrq
                ,   MSP430TimerC
                ;

    ZigTimerIrq.Timer -> MSP430TimerC.TimerA;
 
    ZigTimerIrq.Capture0 -> MSP430TimerC.CaptureA0;
    ZigTimerIrq.Compare0 -> MSP430TimerC.CompareA0;

    ZigTimerIrq.Capture1 -> MSP430TimerC.CaptureA1;
    ZigTimerIrq.Compare1 -> MSP430TimerC.CompareA1;

    ZigTimerIrq.Capture2 -> MSP430TimerC.CaptureA2;
    ZigTimerIrq.Compare2 -> MSP430TimerC.CompareA2;

}

