#ifndef MPU6050_H_
#define MPU6050_H_

#include "xiicps.h"
#include "xparameters.h"

#define MPU6050_ADDRESS 0x68

void MPU6050_Init(XIicPs *IicInstance);
void MPU6050_ReadAccel(XIicPs *IicInstance, float *AccX, float *AccY);
void MPU6050_ReadGyro(XIicPs *IicInstance, float *GyroZ);

#endif /* MPU6050_H_ */
