#include "pwm.h"

int TtcPwmSetup(XTtcPs *TtcPsInstance, u16 DeviceId) {
    XTtcPs_Config *Config;
    XTtcPs *TtcPsInst = TtcPsInstance;
    u32 Options = XTTCPS_OPTION_INTERVAL_MODE | XTTCPS_OPTION_MATCH_MODE;

    u16 Interval;
    u8 Prescaler; // Adjust as necessary
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

    // Calculate the Total Division Factor
    double TotalDivisionFactor = (double)ClockFreq / 400;

    // Choose the Prescaler as a power of 2, for example, 2^9 = 512
    Prescaler = 9;

    // Calculate the Interval
    Interval = (u16)(TotalDivisionFactor / (1 << Prescaler));

    // Calculate the MatchValue for a 50% duty cycle
    MatchValue = Interval / 2;

    // Set the prescaler
    XTtcPs_SetPrescaler(TtcPsInst, Prescaler);

    // Set the interval and match registers
    XTtcPs_SetOptions(TtcPsInst, Options);
    XTtcPs_SetInterval(TtcPsInst, Interval);
    XTtcPs_SetMatchValue(TtcPsInst, 0, MatchValue);

    // Start the timer
    XTtcPs_Start(TtcPsInst);

    return XST_SUCCESS;
}


void setPwmPulseWidth(XTtcPs *TtcPsInstance, u32 PulseWidthMicroseconds) {
    u32 ClockFreq = XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ;
    u8 Prescaler = 9; // Adjust as necessary
    u16 Interval;
    u32 MatchValue;

    // Calculate the Interval based on desired frequency
    double TotalDivisionFactor = (double)ClockFreq / 400;
    Interval = (u16)(TotalDivisionFactor / (1 << Prescaler));

    // Calculate the MatchValue based on PulseWidthMicroseconds
    MatchValue = (u32)(Interval-Interval*PulseWidthMicroseconds/5000);

    // Set the prescaler
    XTtcPs_SetPrescaler(TtcPsInstance, Prescaler);

    // Set the interval and match registers
    XTtcPs_SetInterval(TtcPsInstance, Interval);
    XTtcPs_SetMatchValue(TtcPsInstance, 0, MatchValue);
}
