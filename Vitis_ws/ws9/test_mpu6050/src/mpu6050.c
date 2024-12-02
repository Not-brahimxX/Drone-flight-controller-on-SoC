#include "mpu6050.h"

void MPU6050_Init(XIicPs *IicInstance) {
    u8 sendBuffer[2];

    sendBuffer[0] = 0x6B; // Register 6B
    sendBuffer[1] = 0x00; // Reset

    XIicPs_MasterSendPolled(IicInstance, sendBuffer, 2, MPU6050_ADDRESS);
    while (XIicPs_BusIsBusy(IicInstance));
}

void MPU6050_ReadAccel(XIicPs *IicInstance, float *AccX, float *AccY) {
    u8 recvBuffer[6];
    u8 sendBuffer[1] = {0x3B}; // Start with register 0x3B (ACCEL_XOUT_H)

    XIicPs_MasterSendPolled(IicInstance, sendBuffer, 1, MPU6050_ADDRESS);
    while (XIicPs_BusIsBusy(IicInstance));
    XIicPs_MasterRecvPolled(IicInstance, recvBuffer, 6, MPU6050_ADDRESS);

    *AccX = ((recvBuffer[0] << 8 | recvBuffer[1]) / 16384.0); // X-axis value
    *AccY = ((recvBuffer[2] << 8 | recvBuffer[3]) / 16384.0); // Y-axis value
}

void MPU6050_ReadGyro(XIicPs *IicInstance, float *GyroZ) {
    u8 recvBuffer[2];
    u8 sendBuffer[1] = {0x43}; // Gyro data first register address 0x43

    XIicPs_MasterSendPolled(IicInstance, sendBuffer, 1, MPU6050_ADDRESS);
    while (XIicPs_BusIsBusy(IicInstance));
    XIicPs_MasterRecvPolled(IicInstance, recvBuffer, 2, MPU6050_ADDRESS);

    *GyroZ = ((recvBuffer[0] << 8 | recvBuffer[1]) / 131.0); // Z-axis value
}
