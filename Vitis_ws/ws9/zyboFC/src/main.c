#include "pwm.h"
#include <unistd.h>
#include "mpu6050.h"  // Include MPU6050 functions header
#include "xil_printf.h"
#include "xparameters.h"
#include <math.h>

/* Peripherals Definition */
#define IIC_DEVICE_ID    XPAR_XIICPS_0_DEVICE_ID
#define TTC_DEVICE_ID0        XPAR_XTTCPS_0_DEVICE_ID
#define TTC_DEVICE_ID1        XPAR_XTTCPS_1_DEVICE_ID
#define TTC_DEVICE_ID2        XPAR_XTTCPS_2_DEVICE_ID
#define IIC_SCLK_RATE 400000 // 400 kHz in Hz

#define IBUS_USER_CHANNELS		6		// Use 6 channels

/* Handlers Definitions */
int IicPsSelfTestExample(u16 DeviceId);

XIicPs Iic; /* Instance of the IIC Device */
MPU6050_t GYRO;

XTtcPs TtcPsInst0; // Instance of the TTC peripheral for PWM
XTtcPs TtcPsInst1; // Instance of the TTC peripheral for PWM
XTtcPs TtcPsInst2; // Instance of the TTC peripheral for PWM

/********* VARIABLES ********/
// PID gain and limit settings
// Roll PID
float pid_p_gain_roll = 1.3; // P-gain for roll PID controller
float pid_i_gain_roll = 0.04; // I-gain for roll PID controller
float pid_d_gain_roll = 18.0; // D-gain for roll PID controller
int pid_max_roll = 400; // Maximum output of the roll PID controller (+/-)

// Pitch PID (using same gains and limits as roll for simplicity)
float pid_p_gain_pitch = 1.3;
float pid_i_gain_pitch = 0.04;
float pid_d_gain_pitch = 18.0;
int pid_max_pitch = 400;

// Yaw PID
float pid_p_gain_yaw = 4.0; // P-gain for yaw PID controller
float pid_i_gain_yaw = 0.02; // I-gain for yaw PID controller
float pid_d_gain_yaw = 0.0; // D-gain for yaw PID controller
int pid_max_yaw = 400; // Maximum output of the yaw PID controller (+/-)

// Other variables
int16_t esc_1, esc_2, esc_3, esc_4; // ESC pulse values
int16_t throttle; // Throttle input
double gyro_pitch, gyro_roll, gyro_yaw; // Gyroscope measurements

// Loop timing variables
uint32_t time1, time2, timex;
uint32_t loop_timer, error_timer;
uint32_t startTime, endTime;
uint32_t loopExecutionTime = 0;
uint8_t error, error_counter, error_led;
// PID variable
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll,
		pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch,
		pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw,
		pid_last_yaw_d_error;
float roll_level_adjust, pitch_level_adjust;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float acc_x, acc_y, acc_z, acc_total_vector;
uint16_t ibus_data[IBUS_USER_CHANNELS];

void calculate_pid(void) {
	//Roll calculations
	pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

	//Limiting the I term
	if (pid_i_mem_roll > 100)
		pid_i_mem_roll = 100;
	else if (pid_i_mem_roll < 100 * -1)
		pid_i_mem_roll = 100 * -1;

	pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll
			+ pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);

	//Limiting the output
	if (pid_output_roll > pid_max_roll)
		pid_output_roll = pid_max_roll;
	else if (pid_output_roll < pid_max_roll * -1)
		pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_error_temp;

	//Pitch calculations
	pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

	//Limiting the I term
	if (pid_i_mem_pitch > 100)
		pid_i_mem_pitch = 100;
	else if (pid_i_mem_pitch < 100 * -1)
		pid_i_mem_pitch = 100 * -1;

	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch
			+ pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

	//Limiting the output
	if (pid_output_pitch > pid_max_pitch)
		pid_output_pitch = pid_max_pitch;
	else if (pid_output_pitch < pid_max_pitch * -1)
		pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_error_temp;

	//Yaw calculations
	pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;

	if (pid_i_mem_yaw > 100)
		pid_i_mem_yaw = 100;
	else if (pid_i_mem_yaw < 100 * -1)
		pid_i_mem_yaw = 100 * -1;

	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw
			+ pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);

	//Limiting the output
	if (pid_output_yaw > pid_max_yaw)
		pid_output_yaw = pid_max_yaw;
	else if (pid_output_yaw < pid_max_yaw * -1)
		pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;
}

void gyro_signalen() {
	gyro_roll = GYRO.Gx - GYRO.Gx_cal;
	gyro_pitch = GYRO.Gy - GYRO.Gy_cal;
	gyro_yaw = GYRO.Gz - GYRO.Gz_cal;

	acc_x = GYRO.Ax;
	acc_y = GYRO.Ay;
	acc_z = GYRO.Az;

	// Calculate pitch and roll angles using gyro readings
	angle_pitch += (float) gyro_pitch / 250;  // Calculate traveled pitch angle
	angle_roll += (float) gyro_roll / 250;    // Calculate traveled roll angle

	// Correct for yaw effect on pitch and roll angles
	angle_pitch -= angle_roll * sin((float) gyro_yaw * 0.0000697778); // Transfer roll angle to pitch angle if yawed
	angle_roll += angle_pitch * sin((float) gyro_yaw * 0.0000697778); // Transfer pitch angle to roll angle if yawed

	// Calculate pitch and roll angles using accelerometer readings
	acc_total_vector = sqrt(
			(acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); // Calculate total accelerometer vector
	if (abs(acc_y) < acc_total_vector) {
		angle_pitch_acc = asin((float) acc_y / acc_total_vector) * 57.295; // Calculate pitch angle
	}
	if (abs(acc_x) < acc_total_vector) {
		angle_roll_acc = asin((float) acc_x / acc_total_vector) * 57.295; // Calculate roll angle
	}

	// Correct gyro drift using accelerometer readings
	angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; // Correct gyro drift in pitch angle
	angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004; // Correct gyro drift in roll angle

	// Calculate pitch and roll angle corrections
	pitch_level_adjust = angle_pitch * 15;  // Calculate pitch angle correction
	roll_level_adjust = angle_roll * 15;     // Calculate roll angle correction
}

int main(void) {
	int Status;

	xil_printf("Test PWM\r\n");

	// Initialize and configure the TTC0 counter 0 for PWM generation
	Status = TtcPwmSetup(&TtcPsInst0, TTC_DEVICE_ID0);
	if (Status != XST_SUCCESS) {
		xil_printf("TTC0 PWM setup for counter 0 failed with status %d\r\n",
				Status);
		return XST_FAILURE;
	}

	// Initialize and configure the TTC1 counter 1 for PWM generation
	Status = TtcPwmSetup(&TtcPsInst1, TTC_DEVICE_ID1);
	if (Status != XST_SUCCESS) {
		xil_printf("TTC1 PWM setup for counter 1 failed with status %d\r\n",
				Status);
		return XST_FAILURE;
	}

	// Initialize and configure the TTC2 counter 2 for PWM generation
	Status = TtcPwmSetup(&TtcPsInst2, TTC_DEVICE_ID2);
	if (Status != XST_SUCCESS) {
		xil_printf("TTC2 PWM setup for counter 2 failed with status %d\r\n",
				Status);
		return XST_FAILURE;
	}

	xil_printf("PWM setup successful\r\n");
	/*
	 // Example: Set motor speed to 50% duty cycle
	 setPwmPulseWidth(&TtcPsInst0, 1000);
	 sleep(5); // Run motor at 50% speed for 2 seconds

	 setPwmPulseWidth(&TtcPsInst0, 1500);
	 sleep(5); // Run motor at 50% speed for 2 seconds

	 setPwmPulseWidth(&TtcPsInst0, 2000);
	 sleep(5); // Run motor at 50% speed for 2 seconds
	 */

	xil_printf("IIC Self Test Example \r\n");
	Status = IicPsSelfTestExample(IIC_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("IIC Self Test Failed\r\n");
		return XST_FAILURE;
	}
	Status = XIicPs_SetSClk(&Iic, IIC_SCLK_RATE);
	if (Status != XST_SUCCESS) {
		// Handle clock configuration error
		return XST_FAILURE;
	}

	// Initialize MPU6050
	Status = MPU6050_Init(&Iic);
	if (Status != XST_SUCCESS) {
		xil_printf("MPU6050 initialization failed\r\n");
		return XST_FAILURE;
	}

	xil_printf("MPU6050 initialized successfully\r\n");

	MPU6050_Calibrate(&Iic, &GYRO, 0);
	roll_level_adjust = 0;
	pitch_level_adjust = 0;
	pid_pitch_setpoint = 0;
	pid_yaw_setpoint = 0;
	pid_roll_setpoint = 0;

	gyro_roll_input = 0;
	gyro_pitch_input = 0;
	gyro_yaw_input = 0;

	pid_error_temp = 0;

	pid_i_mem_pitch = 0;
	pid_i_mem_roll = 0;
	pid_i_mem_yaw = 0;

	pid_last_pitch_d_error = 0;
	pid_last_yaw_d_error = 0;
	pid_last_roll_d_error = 0;

	pid_i_mem_roll = 0;
	pid_last_roll_d_error = 0;
	pid_i_mem_pitch = 0;
	pid_last_pitch_d_error = 0;
	pid_i_mem_yaw = 0;
	pid_last_yaw_d_error = 0;

	pid_roll_setpoint = 0;
	pid_yaw_setpoint = 0;
	pid_pitch_setpoint = 0;

	angle_pitch = 0;
	angle_roll = 0;
	while (1) {
		// Read MPU6050 accelerometer data

		if (Status != XST_SUCCESS) {
			xil_printf("Failed to read accelerometer data\r\n");
			return XST_FAILURE;
		}
		Status = MPU6050_Read(&Iic, &GYRO);
		if (Status != XST_SUCCESS) {
					xil_printf("Failed to read accelerometer data\r\n");
					return XST_FAILURE;
		}
		gyro_signalen();
		pid_roll_setpoint = 0;

		pid_roll_setpoint -= roll_level_adjust;
		pid_roll_setpoint /= 3; //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

		//The PID set point in degrees per second is determined by the pitch receiver input.
		//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_pitch_setpoint = 0;


		pid_pitch_setpoint -= pitch_level_adjust; //Subtract the angle correction from the standardized receiver pitch input value.
		pid_pitch_setpoint /= 3; //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

		//The PID set point in degrees per second is determined by the yaw receiver input.
		//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_yaw_setpoint = 0;

		gyro_roll_input = (gyro_roll_input * 0.7) + (((float) gyro_roll) * 0.3); //Gyro pid input is deg/sec.
		gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float) gyro_pitch) * 0.3); //Gyro pid input is deg/sec.
		gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float) gyro_yaw) * 0.3); //Gyro pid input is deg/sec.

		throttle = 1500;

		calculate_pid();
		if (throttle > 1800)
			throttle = 1800;

		esc_1 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW).
		esc_2 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW).
		esc_3 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW).
		esc_4 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW).

		if (esc_1 < 1100)
			esc_1 = 1100;                             //Keep the motors running.
		if (esc_2 < 1100)
			esc_2 = 1100;                             //Keep the motors running.
		if (esc_3 < 1100)
			esc_3 = 1100;                             //Keep the motors running.
		if (esc_4 < 1100)
			esc_4 = 1100;                             //Keep the motors running.

		if (esc_1 > 2000)
			esc_1 = 2000;                     //Limit the esc-1 pulse to 2000us.
		if (esc_2 > 2000)
			esc_2 = 2000;                     //Limit the esc-2 pulse to 2000us.
		if (esc_3 > 2000)
			esc_3 = 2000;                     //Limit the esc-3 pulse to 2000us.
		if (esc_4 > 2000)
			esc_4 = 2000;                     //Limit the esc-4 pulse to 2000us.
		setPwmPulseWidth(&TtcPsInst0, esc_1);
		setPwmPulseWidth(&TtcPsInst1, esc_2);
		setPwmPulseWidth(&TtcPsInst2, esc_3);
		// Print MPU6050 data
		xil_printf("esc1: %d \t esc2: %d \t esc3: %d\t esc4: %d \t roll: %d \t pitch: %d\r\n", esc_1,esc_2,esc_3,esc_4,(int)(angle_roll),(int)(angle_pitch));
		usleep(2000);
	}
	return XST_SUCCESS;
}

int IicPsSelfTestExample(u16 DeviceId) {
	int Status;
	XIicPs_Config *Config;

	/*
	 * Initialize the IIC driver so that it's ready to use
	 * Look up the configuration in the config table, then initialize it.
	 */
	Config = XIicPs_LookupConfig(DeviceId);
	if (NULL == Config) {
		return XST_FAILURE;
	}

	Status = XIicPs_CfgInitialize(&Iic, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test.
	 */
	Status = XIicPs_SelfTest(&Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}
