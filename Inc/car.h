/*
 * car.h
 *
 *  Created on: Jan 3, 2017
 *      Author: ben
 */

#ifndef CAR_H_
#define CAR_H_

#include "main.h"
#include "cmsis_os.h"
#include "PedalBox.h"
#include "stm32f4xx_hal.h"
#include "CANProcess.h"
#include "BMS.h"
#include "motor_controller_functions.h"
//#include "accelerometer.h"
#include <math.h>

#define PERIOD_ACCELRO				50 / portTICK_RATE_MS
#define TICK_MAIN				 	20 / portTICK_RATE_MS
#define HEARTBEAT_PULSEWIDTH		200 / portTICK_RATE_MS
#define HEARTBEAT_PERIOD			100 / portTICK_RATE_MS
#define POLL_DELAY					50 / portTICK_RATE_MS
#define LC_THRESHOLD				10			// todo lc threshold DUMMY VALUE
#define LAUNCH_CONTROL_INTERVAL_MS	10


//rtos parameter defines
#define QUEUE_SIZE_RXCAN			16
#define QUEUE_SIZE_TXCAN			10
#define QUEUE_SIZE_MCFRAME			3


//launch control
typedef enum {
	LC_ACTIVATED,
	LC_DISABLED
} LC_status_t;

typedef enum {
	CAR_STATE_INIT,
	CAR_STATE_PREREADY2DRIVE,
	CAR_STATE_READY2DRIVE,
	CAR_STATE_ERROR,
	CAR_STATE_RESET,
	CAR_STATE_RECOVER
} Car_state_t;

typedef enum {
	PEDALBOX_MODE_ANALOG,
	PEDALBOX_MODE_DIGITAL
} Pedalbox_mode_t;

typedef enum {
	CALIBRATE_NONE,
	CALIBRATE_THROTTLE_MIN,
	CALIBRATE_THROTTLE_MAX,
	CALIBRATE_BRAKE_MIN,
	CALIBRATE_BRAKE_MAX
} Calibrate_flag_t;


int BCparam;
int actualTorque0700;
int actualTorque1508;
int actualDC;
int actualV;
int DCLimit;
int calcTorqueLimit;
int pedalTorque;
int actualTorque;
int torque_to_send;
int speedActual;
int currentActual;
int commandCurrent;
int dcBusVoltage;
int motorTemperature;
int powerStageTemperature;
int airTemperature;
int actualCurrentLimit;
int errBitMap1;

typedef struct {

	Car_state_t 			state;
	uint8_t					errorFlags;
	//calibration values

	Pedalbox_mode_t			pb_mode;					//determines whether pb will be analog or CAN
	Calibrate_flag_t		calibrate_flag;

	LC_status_t				lc_status;
	//Pedalbox_msg_t 			pb_current_msg;

	//RTOS objects, initialized in initRTOSObjects
	QueueHandle_t			q_rxcan;
	QueueHandle_t			q_txcan;
	QueueHandle_t	 		q_pedalboxmsg;
	QueueHandle_t			q_mc_frame;

	SemaphoreHandle_t		m_CAN;						//mutex for CAN peripheral

	CAN_HandleTypeDef *		phcan;						//pointer to car's CAN peripheral handle

} Car_t;

extern volatile Car_t car;
extern CAN_HandleTypeDef hcan1;

void carInit();
void taskCarMainRoutine();
int SendTorqueTask();
int mainModuleWatchdogTask();
int taskHeartbeat();
void taskSoundBuzzer(uint32_t* time);
void initRTOSObjects();
void taskBlink(void* can);
void taskSendAccelero();
void ISR_StartButtonPressed();


#endif /* CAR_H_ */
