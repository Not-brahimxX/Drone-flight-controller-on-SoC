#include "pwm.h"
#include <unistd.h>
XTtcPs TtcPsInst0; // Instance of the TTC peripheral for PWM
XTtcPs TtcPsInst1; // Instance of the TTC peripheral for PWM
XTtcPs TtcPsInst2; // Instance of the TTC peripheral for PWM

// Function to set PWM duty cycle
void setPwmDutyCycle(XTtcPs *TtcPsInstance, u8 dutyCycle) {
    u32 ClockFreq = XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ;
    u8 Prescaler = 0; // Adjust as necessary
    u16 Interval = (ClockFreq / PWM_PERIOD) >> Prescaler;
    u32 MatchValue = Interval * dutyCycle / 100;
    XTtcPs_SetMatchValue(TtcPsInstance, 0, MatchValue);
}

int main(void) {
    int Status;

    xil_printf("DC Motor Control Example\r\n");

    // Initialize and configure the TTC0 counter 0 for PWM generation
    Status = TtcPwmSetup(&TtcPsInst0, TTC_DEVICE_ID0, 0);
    if (Status != XST_SUCCESS) {
        xil_printf("TTC0 PWM setup for counter 0 failed with status %d\r\n", Status);
        return XST_FAILURE;
    }

    // Initialize and configure the TTC1 counter 1 for PWM generation
    Status = TtcPwmSetup(&TtcPsInst1, TTC_DEVICE_ID1, 1);
    if (Status != XST_SUCCESS) {
        xil_printf("TTC1 PWM setup for counter 1 failed with status %d\r\n", Status);
        return XST_FAILURE;
    }

    // Initialize and configure the TTC2 counter 2 for PWM generation
    Status = TtcPwmSetup(&TtcPsInst2, TTC_DEVICE_ID2, 2);
    if (Status != XST_SUCCESS) {
        xil_printf("TTC2 PWM setup for counter 2 failed with status %d\r\n", Status);
        return XST_FAILURE;
    }

    xil_printf("PWM setup successful\r\n");

    // Example: Set motor speed to 50% duty cycle
    setPwmDutyCycle(&TtcPsInst0, 50); // Adjust the channel index and duty cycle as needed
    sleep(5); // Run motor at 50% speed for 2 seconds

    // Example: Set motor speed to 75% duty cycle
    setPwmDutyCycle(&TtcPsInst0, 75); // Adjust the channel index and duty cycle as needed
    sleep(5); // Run motor at 75% speed for 2 seconds

    // Example: Set motor speed to 25% duty cycle
    setPwmDutyCycle(&TtcPsInst0, 25); // Adjust the channel index and duty cycle as needed
    sleep(5); // Run motor at 25% speed for 2 seconds

    setPwmDutyCycle(&TtcPsInst0, 50); // Adjust the channel index and duty cycle as needed
    sleep(5); // Run motor at 50% speed for 2 seconds

    // Example: Set motor speed to 75% duty cycle
    setPwmDutyCycle(&TtcPsInst0, 75); // Adjust the channel index and duty cycle as needed
    sleep(5); // Run motor at 75% speed for 2 seconds

    // Example: Set motor speed to 25% duty cycle
    setPwmDutyCycle(&TtcPsInst0, 25); // Adjust the channel index and duty cycle as needed
    sleep(5); // Run motor at 25% speed for 2 seconds

    // Add more motor control logic as needed

    return XST_SUCCESS;
}
