#include "timer.h"

void tima_1_init(void)
{
	 NVIC_ClearPendingIRQ(TIMER_10ms_INST_INT_IRQN);
   NVIC_EnableIRQ(TIMER_10ms_INST_INT_IRQN);
   DL_TimerG_startCounter(TIMER_10ms_INST);  
}