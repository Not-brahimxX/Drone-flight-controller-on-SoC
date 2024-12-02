#include "mpu6050.h"
#include "xtime_l.h"
#include <math.h>
#include "xiicps.h"
#include "sleep.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define MPU6050_ADDRESSESS     0x68
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_CONFIG_REG    0x1C
#define PWR_MGMT_1_REG      0x6B

// Setup MPU6050
#define MPU6050_ADDRESS 0x68
const uint16_t i2c_timeout = 100;
const double Accel_X_corrector = 16384.0;
const double Accel_Y_corrector = 16384.0;
const double Accel_Z_corrector = 14418.0;

XTime currentTime;
uint32_t timer;


Kalman_t KalmanX = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f };

Kalman_t KalmanY = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f, };



int WriteToMPU6050Register(XIicPs *Iic, u8 DeviceAddress, u8 RegisterAddress, u8 Data)
{
    u8 WriteBuffer[2];

    WriteBuffer[0] = RegisterAddress;
    WriteBuffer[1] = Data;

    // Send register address and data to MPU6050
    XIicPs_MasterSendPolled(Iic, WriteBuffer, 2, DeviceAddress);

    // Wait for transfer to complete
    while (XIicPs_BusIsBusy(Iic));

    return XST_SUCCESS;
}


int ReadFromMPU6050Register(XIicPs *Iic, u8 DeviceAddress, u8 RegisterAddress, u8 *BufferPtr, u8 ByteCount) {
    int Status;

    // Set register address to the specified register
    Status = WriteToMPU6050Register(Iic, DeviceAddress, RegisterAddress, 0);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Read the specified number of bytes starting from the specified register
    Status = XIicPs_MasterRecvPolled(Iic, BufferPtr, ByteCount, DeviceAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

int MPU6050_Init(XIicPs *Iic)
{
    int Status;

    // Wake up the MPU6050 as it starts in sleep mode
    Status = WriteToMPU6050Register(Iic, MPU6050_ADDRESS, PWR_MGMT_1_REG, 0x00);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    Status = WriteToMPU6050Register(Iic, MPU6050_ADDRESS, 0x19,
    				0x07);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    // Configure gyroscope range to ±250 degrees/sec
    Status = WriteToMPU6050Register(Iic, MPU6050_ADDRESS, GYRO_CONFIG_REG, 0x00);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Configure accelerometer range to ±2g
    Status = WriteToMPU6050Register(Iic, MPU6050_ADDRESS, ACCEL_CONFIG_REG, 0x00);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

int MPU6050_Read_All(XIicPs *Iic, MPU6050_t *DataStruct) {
	u8 Rec_Data[6];
	int Status;

	// Set register address to ACCEL_XOUT_H
	Status = WriteToMPU6050Register(Iic, MPU6050_ADDRESS, 0x3B, 0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Read 6 bytes of accelerometer data starting from ACCEL_XOUT_H register
	Status = XIicPs_MasterRecvPolled(Iic, Rec_Data, 6, MPU6050_ADDRESS);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	DataStruct->Accel_X_RAW =  (Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Accel_Y_RAW =  (Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Accel_Z_RAW =  (Rec_Data[4] << 8 | Rec_Data[5]);

	// Set register address to GYRO_XOUT_H
	Status = WriteToMPU6050Register(Iic, MPU6050_ADDRESS, 0x43, 0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Read 6 bytes of gyroscope data starting from GYRO_XOUT_H register
	Status = XIicPs_MasterRecvPolled(Iic, Rec_Data, 6, MPU6050_ADDRESS);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	//Correcting the Received Data
	DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
	DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;

	DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;


	/*
	// Kalman angle solve

	XTime_GetTime(&currentTime);

	double dt = (double) (currentTime - timer) / COUNTS_PER_SECOND;
	timer = currentTime;

	double roll;
	double roll_sqrt = sqrt(DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
	if (roll_sqrt != 0.0) {
		roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
	} else {
		roll = 0.0;
	}
	double pitch = atan2(-DataStruct->Accel_X_RAW,
			DataStruct->Accel_Z_RAW) * RAD_TO_DEG;

	if ((pitch < -90 && DataStruct->KalmanAngleY > 90)
			|| (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
		KalmanY.angle = pitch;
		DataStruct->KalmanAngleY = pitch;
	} else {
		DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch,
				DataStruct->Gy, dt);
	}
	if (fabs(DataStruct->KalmanAngleY) > 90)
		DataStruct->Gx = -DataStruct->Gx;
	DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx,dt);*/
	return XST_SUCCESS;
}



int MPU6050_Read(XIicPs *Iic, MPU6050_t *DataStruct) {
    u8 Rec_Data[14];
    u8 sendBuffer[1] = {0x3B}; // Start with register 0x3B (ACCEL_XOUT_H)
    int Status;

    // Set register address to ACCEL_XOUT_H
    XIicPs_MasterSendPolled(Iic, sendBuffer, 1, MPU6050_ADDRESS);
    while (XIicPs_BusIsBusy(Iic));
    XIicPs_MasterRecvPolled(Iic, Rec_Data, 14, MPU6050_ADDRESS);

    // Process the received data
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    int16_t temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // Convert raw data to meaningful units
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / 340.0 + 36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    return XST_SUCCESS;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate,
		double dt) {
	double rate = newRate - Kalman->bias;
	Kalman->angle += dt * rate;
	Kalman->P[0][0] += dt
			* (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0]
					+ Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;
	double S = Kalman->P[0][0] + Kalman->R_measure;
	double K[2];
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;
	double y = newAngle - Kalman->angle;
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;
	double P00_temp = Kalman->P[0][0];
	double P01_temp = Kalman->P[0][1];
	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;
	return Kalman->angle;
}



void MPU6050_Calibrate(XIicPs *Iic, MPU6050_t *DataStruct,uint8_t gyro_manual){
	int cal_int = 0;
	if (gyro_manual)
		{
			cal_int = 2000;                                          //If manual calibration is used set cal_int to 2000 to skip the calibration.
			DataStruct->Gx_cal = 0;                                                            //Divide the roll total by 2000.
			DataStruct->Gy_cal = 0;                                                           //Divide the pitch total by 2000.
	        DataStruct->Gz_cal = 0;
		}
	  else {
	    cal_int = 0;                                                                      //If manual calibration is not used.
	    DataStruct->Gx_cal = 0; //Set the manual pitch calibration variable to 0.
	    DataStruct->Gy_cal = 0; //Set the manual roll calibration variable to 0.
	    DataStruct->Gz_cal = 0; //Set the manual yaw calibration variable to 0.

	    if (cal_int != 2000) {
	        //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
	        for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
	          MPU6050_Read_All(Iic, DataStruct);                                                               //Read the gyro output.
	          DataStruct->Gx_cal +=DataStruct->Gx;                                                     //Ad roll value to gyro_roll_cal.
	          DataStruct->Gy_cal += DataStruct->Gy;                                                   //Ad pitch value to gyro_pitch_cal.
	          DataStruct->Gz_cal += DataStruct->Gz;                                                       //Ad yaw value to gyro_yaw_cal.
	          usleep(4000);                                                                       //Small delay to simulate a 250Hz loop during calibration.
	        }                                                                 //Set output PB3 low.
	        //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
	        DataStruct->Gx_cal /= 2000;                                                            //Divide the roll total by 2000.
	        DataStruct->Gy_cal /= 2000;                                                           //Divide the pitch total by 2000.
	        DataStruct->Gz_cal /= 2000;                                                             //Divide the yaw total by 2000.
	      }
	  }
}

