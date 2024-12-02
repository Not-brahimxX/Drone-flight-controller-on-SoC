#include "pwm.h"

int TtcPwmSetup(XTtcPs *TtcPsInstance, u16 DeviceId, u8 Counter) {
    XTtcPs_Config *Config;
    XTtcPs *TtcPsInst = TtcPsInstance;
    u32 Options = XTTCPS_OPTION_INTERVAL_MODE | XTTCPS_OPTION_MATCH_MODE;
    u16 Interval;
    u8 Prescaler = 0; // Adjust as necessary
    u32 MatchValue;
    u32 ClockFreq = XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ; // Use TTC clock frequency from xparameters.h

    // Initialize the device configuration
    Config = XTtcPs_LookupConfig(DeviceId);
    if (Config == NULL) {
        return XST_FAILURE;
    }

    // Initialize the TTC driver
    int Status = XTtcPs_CfgInitialize(TtcPsInst, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Set the prescaler
    XTtcPs_SetPrescaler(TtcPsInst, Prescaler);

    // Calculate the Interval and MatchValue
    Interval = (ClockFreq / PWM_PERIOD) >> Prescaler;
    MatchValue = Interval * PWM_DUTY_CYCLE / 100;

    // Set the interval and match registers
    XTtcPs_SetOptions(TtcPsInst, Options);
    XTtcPs_SetInterval(TtcPsInst, Interval);
    XTtcPs_SetMatchValue(TtcPsInst, 0, MatchValue);

    // Start the timer
    XTtcPs_Start(TtcPsInst);

    return XST_SUCCESS;
}
