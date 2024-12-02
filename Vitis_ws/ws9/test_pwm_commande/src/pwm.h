#ifndef PWM_H
#define PWM_H

#include "xparameters.h"
#include "xttcps.h"
#include "xil_printf.h"

#define TTC_DEVICE_ID0        XPAR_XTTCPS_0_DEVICE_ID
#define TTC_DEVICE_ID1        XPAR_XTTCPS_1_DEVICE_ID
#define TTC_DEVICE_ID2        XPAR_XTTCPS_2_DEVICE_ID
#define PWM_PERIOD           1000 // Period of the PWM signal in microseconds
#define PWM_DUTY_CYCLE       50      // Duty cycle of the PWM signal in percent

int TtcPwmSetup(XTtcPs *TtcPsInstance, u16 DeviceId, u8 Counter);

#endif // PWM_H
