configuration ZigSTimerC {
} implementation {
    components  
                ZigSTimerM  as ZigSTimer
            ,   SysTimerC as Timer
            ;

    ZigSTimer.Timer0 -> Timer.TimerMilli[unique("TimerMilli")];
    ZigSTimer.Timer1 -> Timer.TimerMilli[unique("TimerMilli")];
    ZigSTimer.Timer2 -> Timer.TimerMilli[unique("TimerMilli")];

}

