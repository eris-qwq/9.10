#include "ramp.h"

uint32_t Get_TimerTick() {
    return HAL_GetTick();
}

float Slope(Ramp_Typedef *Ramp) {
    if (!Ramp->flag) {
        Ramp->StartTick = Get_TimerTick();
        Ramp->flag = 1;
    }
    if (Get_TimerTick() > (Ramp->StartTick + Ramp->RampTime))return 1.0f;
    return ((Get_TimerTick() - Ramp->StartTick) / (float) Ramp->RampTime);
}

float Slope_1(float current, float expect, float k) //D¡À??
{
	if(current < expect)
	{
		if(expect - current < k)
			current += (expect - current);
		else
			current += k;
	}
	if(current > expect)
	{
		if(current - expect < k)
			current -= (current - expect);
		else 
			current -= k;
	}
	return current;
}
