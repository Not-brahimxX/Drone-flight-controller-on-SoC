#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "xil_types.h"
#include "xiicps.h"

// MPU6050 structure
typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;
    double Gx_cal;
    double Gy_cal;
    double Gz_cal;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

int WriteToMPU6050Register(XIicPs *Iic, u8 DeviceAddress, u8 RegisterAddress, u8 Data);

int ReadFromMPU6050Register(XIicPs *Iic, u8 DeviceAddress, u8 RegisterAddress, u8 *BufferPtr, u8 ByteCount);

int MPU6050_Init(XIicPs *Iic);

int MPU6050_Read_Accel(XIicPs *Iic, MPU6050_t *DataStruct);

int MPU6050_Read_Gyro(XIicPs *Iic, MPU6050_t *DataStruct);

int MPU6050_Read_Temp(XIicPs *Iic, MPU6050_t *DataStruct);

int MPU6050_Read_All(XIicPs *Iic, MPU6050_t *DataStruct);

int MPU6050_Read(XIicPs *Iic, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

void MPU6050_Calibrate(XIicPs *Iic, MPU6050_t *DataStruct, uint8_t gyro_manual);

#endif /* INC_MPU6050_H_ */
