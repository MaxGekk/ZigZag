configuration ZigIrqC {
   provides interface StdControl;
} implementation {
   components       ZigIrqM as ZigIrq
                ,   MSP430InterruptC
                ;

   StdControl = ZigIrq.StdControl;

   ZigIrq.Port10 -> MSP430InterruptC.Port10;
   ZigIrq.Port11 -> MSP430InterruptC.Port11;
   ZigIrq.Port12 -> MSP430InterruptC.Port12;
   ZigIrq.Port13 -> MSP430InterruptC.Port13;
   ZigIrq.Port14 -> MSP430InterruptC.Port14;
   ZigIrq.Port15 -> MSP430InterruptC.Port15;
   ZigIrq.Port16 -> MSP430InterruptC.Port16;
   ZigIrq.Port17 -> MSP430InterruptC.Port17;

   ZigIrq.Port20 -> MSP430InterruptC.Port20;
   ZigIrq.Port21 -> MSP430InterruptC.Port21;
   ZigIrq.Port22 -> MSP430InterruptC.Port22;
   ZigIrq.Port23 -> MSP430InterruptC.Port23;
   ZigIrq.Port24 -> MSP430InterruptC.Port24;
   ZigIrq.Port25 -> MSP430InterruptC.Port25;
   ZigIrq.Port26 -> MSP430InterruptC.Port26;
   ZigIrq.Port27 -> MSP430InterruptC.Port27;

}

