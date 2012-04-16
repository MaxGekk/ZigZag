module PowerManagerM {
   provides interface StdControl;
} implementation {

command result_t StdControl.init() { return SUCCESS; }
command result_t StdControl.start()
   {
    LPMode_enable();
    return SUCCESS;
   }
command result_t StdControl.stop()
   {
    LPMode_disable();
    return SUCCESS;
   }
   
}

