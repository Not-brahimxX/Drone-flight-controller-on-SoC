#include <stdio.h>
#include "xparameters.h"
#include "xuartps.h"
#include "xiicps.h"
#include "mpu6050.h"

#define UART_DEVICE_ID XPAR_XUARTPS_0_DEVICE_ID
#define IIC_DEVICE_ID XPAR_XIICPS_0_DEVICE_ID

int main() {
    int Status;
    XIicPs IicInstance;
    XUartPs Uart_Ps;
    XUartPs_Config *UartConfig;
    XIicPs_Config *Config;
    float AccX, AccY, GyroZ;

    // Initialize UART
    UartConfig = XUartPs_LookupConfig(UART_DEVICE_ID);
    Status = XUartPs_CfgInitialize(&Uart_Ps, UartConfig, UartConfig->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Initialize IIC
    Config = XIicPs_LookupConfig(IIC_DEVICE_ID);
    Status = XIicPs_CfgInitialize(&IicInstance, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Perform a self-test to ensure that the hardware was built correctly
    Status = XIicPs_SelfTest(&IicInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Set the IIC serial clock rate
    XIicPs_SetSClk(&IicInstance, 100000);

    // Initialize the MPU6050 sensor
    MPU6050_Init(&IicInstance);

    while (1) {
        MPU6050_ReadAccel(&IicInstance, &AccX, &AccY);
        MPU6050_ReadGyro(&IicInstance, &GyroZ);

        // Print the values on the serial monitor
        printf("AccX: %f / AccY: %f / GyroZ: %f\n", AccX, AccY, GyroZ);


    }

    return 0;
}
