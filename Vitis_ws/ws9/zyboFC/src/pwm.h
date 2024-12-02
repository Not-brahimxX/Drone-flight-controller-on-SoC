#ifndef PWM_H
#define PWM_H

#include "xparameters.h"
#include "xttcps.h"
#include "xil_printf.h"

#define PWM_PERIOD           5000 // Period of the PWM signal in microseconds
#define PWM_DUTY_CYCLE       50      // Duty cycle of the PWM signal in percent
int TtcPwmSetup(XTtcPs *TtcPsInstance, u16 DeviceId);
void setPwmPulseWidth(XTtcPs *TtcPsInstance, u32 PulseWidthMicroseconds);
#endif // PWM_H
