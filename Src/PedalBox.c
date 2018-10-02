/*
 * PedalBox.c
 *
 *  Created on: Jan 12, 2017
 *      Author: ben
 */

#include "PedalBox.h"
#include "car.h"

//TODO think about how to make these thread safe with a lock or possibly a request flag
float 			throttle;				//sum of car's intended throttle messages from pedalbox since last cmd sent to MC
int16_t				throttle_cnt;				//number of throttle messages in accumulator
float 				brake;						//car's intended brake position
Pedalbox_status_t		apps_status_imp;			//the last pedalbox message imp state
Pedalbox_status_t		apps_status_bp_plaus;	//apps-brake plausibility status
Pedalbox_status_t		apps_status_eor;			//apps-brake plausibility status
Pedalbox_status_t		apps_status_timeout;		//apps-brake plausibility status
uint32_t				pb_msg_rx_time;				//indicates when a pedalbox message was last received
uint32_t				apps_imp_first_time_ms;		//indicates when the first imp error was received
int32_t				throttle1_min; 				//this is a higher value than max
int32_t				throttle1_max; 				//this is a lower value than min
int32_t				throttle2_min;
int32_t				throttle2_max;
int32_t				brake1_min;
int32_t				brake1_max;
int32_t				brake2_min;
int32_t				brake2_max;
QueueHandle_t	 	q_pedalboxmsg;


float apps_getThrottlePos()
{
	return throttle;
}

float apps_getBrake()
{
	return brake;
}

void apps_init()
/***************************************************************************
*
*     Function Information
*
*     Name of Function: pedalBoxMessageHandler
*
*     Programmer's Name: 	Kai Strubel
*     						Ben Ng			xbenng@gmail.com
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.Pedalbox_msg_t msg
			brake_level from pedalbox potentiometer
*			throttle_level from pedalbox potentiometer
*			APPS_Implausible flag
*			EOR flag
*		2.
*
*      Global Dependents:
*
*     Function Description:
*			Initializes apps variables and queues and starts apps tasks
***************************************************************************/
{
	throttle1_min = 0x0f90;
	throttle1_max = 0x07e0;
	throttle2_min = 0x0ed0;
	throttle2_max = 0x06c0;
	brake1_min = 0x027c;
	brake1_max = 0x0900;
	brake2_min = 0x026f;
	brake2_max = 0x0900;

	throttle = 0;
	brake = 0;
	pb_msg_rx_time = 4294967295;
	apps_status_bp_plaus = PEDALBOX_STATUS_ERROR;
	apps_status_eor = PEDALBOX_STATUS_ERROR;
	apps_status_imp = PEDALBOX_STATUS_ERROR;
	apps_status_timeout = PEDALBOX_STATUS_ERROR;

	q_pedalboxmsg = xQueueCreate(QUEUE_SIZE_PEDALBOXMSG, sizeof(Pedalbox_msg_t));
	xTaskCreate(taskRXCANProcess, "RX CAN", 256, NULL, 1, NULL);
}

void taskPedalBoxMsgHandler() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: pedalBoxMessageHandler
*
*     Programmer's Name: 	Kai Strubel
*     						Ben Ng			xbenng@gmail.com
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.Pedalbox_msg_t msg
			brake_level from pedalbox potentiometer
*			throttle_level from pedalbox potentiometer
*			APPS_Implausible flag
*			EOR flag
*		2.
*
*      Global Dependents:
*
*     Function Description:
*			Takes message from pedal box, runs safetly check, sets throttle.
*			Designed this way so pedalboxmsg's can be generated multiple ways,
*			and not disrupt the logic behind processing the data.
***************************************************************************/

	Pedalbox_msg_t pedalboxmsg;		//struct to store pedalbox msg
	while (1) {

		if(xQueueReceive(q_pedalboxmsg, &pedalboxmsg, PEDALBOX_TIMEOUT))
		{
			//now relying on queue timeout, previously: update time stamp, indicates when a pedalbox message was last received
			//get current time in ms
			uint32_t current_time_ms = xTaskGetTickCount() / portTICK_PERIOD_MS;;
			//pb_msg_rx_time = current_time_ms;

			//calibration should not be done here.
			//check if calibration values should be updated
			//dynamic calibration not implemented yet.
//			if (car.calibrate_flag == CALIBRATE_THROTTLE_MIN) {
//				car.throttle1_min = pedalboxmsg.throttle1_raw;
//				car.throttle2_min = pedalboxmsg.throttle2_raw;
//				car.calibrate_flag = CALIBRATE_NONE;
//			} else if (car.calibrate_flag == CALIBRATE_THROTTLE_MAX) {
//				car.throttle1_max = pedalboxmsg.throttle1_raw;
//				car.throttle2_max = pedalboxmsg.throttle2_raw;
//				car.calibrate_flag = CALIBRATE_NONE;
//			} else 	if (car.calibrate_flag == CALIBRATE_BRAKE_MIN) {
//				car.brake1_min = pedalboxmsg.brake1_raw;
//				car.brake2_min = pedalboxmsg.brake2_raw;
//				car.calibrate_flag = CALIBRATE_NONE;
//			} else if (car.calibrate_flag == CALIBRATE_BRAKE_MAX) {
//				car.brake1_max = pedalboxmsg.brake1_raw;
//				car.brake2_max = pedalboxmsg.brake2_raw;
//				car.calibrate_flag = CALIBRATE_NONE;
//			}else{
//
//			}

			/////////////PROCESS DATA///////////////
			int throttle1_sign = (throttle1_max - throttle1_min) / fabs(throttle1_max - throttle1_min);
			int throttle2_sign = (throttle2_max - throttle2_min) / fabs(throttle2_max - throttle2_min);

			float			throttle1_cal = ((float)(pedalboxmsg.throttle1_raw - throttle1_min)) / (throttle1_max - throttle1_min);  //value 0-1, throttle 1 calibrated between min and max
			float			throttle2_cal = ((float)(pedalboxmsg.throttle2_raw - throttle2_min)) / (throttle2_max - throttle2_min);;  //value 0-1, throttle 2 calibrated between min and max
			float 			brake1_cal	  = ((float)(pedalboxmsg.brake1_raw - brake1_min)) / (brake1_max - brake1_min);  //value 0-1, brake 1 calibrated between min and max
			float 			brake2_cal	  = ((float)(pedalboxmsg.brake2_raw - brake2_min)) / (brake2_max - brake2_min);  //value 0-1, brake 2 calibrated between min and max
			float 			throttle_avg  = (throttle1_cal + throttle2_cal) / 2.0;
			float			brake_avg     = (brake1_cal + brake2_cal) / 2.0;


			// EV 2.4.6: Encoder out of range
			if (
				//check this math
//				!(throttle1_sign * pedalboxmsg.throttle1_raw >= throttle1_sign * car.throttle1_min &&
//				 throttle1_sign * pedalboxmsg.throttle1_raw <= throttle1_sign * car.throttle1_max)
//				||
//				!(throttle2_sign * pedalboxmsg.throttle2_raw >= throttle2_sign * car.throttle2_min &&
//				throttle2_sign * pedalboxmsg.throttle2_raw <= throttle2_sign * car.throttle2_max)
				pedalboxmsg.throttle1_raw >= 0x0e70 ||
				pedalboxmsg.throttle1_raw <= 0x019a ||
				pedalboxmsg.throttle2_raw >= 0x0e70 ||
				pedalboxmsg.throttle2_raw <= 0x019a
			)
			{
				apps_status_eor = PEDALBOX_STATUS_ERROR;
			} else {
				apps_status_eor = PEDALBOX_STATUS_NO_ERROR;
			}

			//APPS Implausibility error handling, EV 2.3.5,
			//	error if throttle sensors disagree more than 10% travel
			//	for more than 100ms
			if (fabs(throttle1_cal - throttle2_cal) > .1 ) //legacy: (pedaboxmsg.APPS_Implausible == PEDALBOX_STATUS_ERROR)
			{
				//if error is persistent
				if (apps_status_imp == PEDALBOX_STATUS_ERROR_APPSIMP_PREV)  //<<is this a typo
				{
					//if time between first error and this error >= 100ms
					if (apps_imp_first_time_ms - current_time_ms >= 100)
					{
						apps_status_imp = PEDALBOX_STATUS_ERROR;
					}
				} else {  //else this is the first message to have an imp error
					//record the time
					apps_status_imp = PEDALBOX_STATUS_ERROR_APPSIMP_PREV;
					apps_imp_first_time_ms = current_time_ms;
				}
			} else {
				apps_status_imp = PEDALBOX_STATUS_NO_ERROR;

			}




//			BRAKE PLAUSIBILITY check
			if (throttle_avg >= .25 && brake_avg >= BRAKE_PRESSED_THRESHOLD)
			{
				apps_status_bp_plaus = PEDALBOX_STATUS_ERROR;
			}
			else if (throttle_avg <= APPS_BP_PLAUS_RESET_THRESHOLD) {//latch until this condition
				//EV 2.5.1, reset apps-brake pedal plausibility error only if throttle level is less than the .05
					apps_status_bp_plaus = PEDALBOX_STATUS_NO_ERROR;
			}

			brake = brake_avg;
			if (throttle_avg >= THROTTLE_DEADZONE_LOW)
			{
				if (throttle_avg >= 1 - THROTTLE_DEADZONE_HIGH)
				{
					throttle = 1; //MAX_THROTTLE_LEVEL;
				}
				else
				{
					//adjust throttle for full range inside deadzones
					throttle = (throttle_avg - THROTTLE_DEADZONE_LOW) / (1 - (THROTTLE_DEADZONE_LOW + THROTTLE_DEADZONE_HIGH));//((throttle_avg - 0.1) * MAX_THROTTLE_LEVEL / 0.8);
				}
			//car.throttle_cnt ++;
			}
			else
			{
				throttle = 0;
			}

		}
		else
		{
			//queue hasn't received for PEDALBOX_TIMEOUT, set timeout error.
			apps_status_timeout = PEDALBOX_STATUS_ERROR;
		}
	}

	//if this task breaks from the loop kill it
	vTaskDelete(NULL);
}

Pedalbox_status_t apps_getStatus_imp()
{
	return apps_status_imp;
}

Pedalbox_status_t apps_getStatus_bp()
{
	return apps_status_bp_plaus;
}

Pedalbox_status_t apps_getStatus_eor()
{
	return apps_status_eor;
}

Pedalbox_status_t apps_getStatus_timeout()
{
	return apps_status_timeout;
}

Brake_light_status_t brake_isPressed()
{
	return apps_getBrake() >= BRAKE_PRESSED_THRESHOLD;
}

//int storeToFlash() {
//	HAL_FLASH_Unlock();
//	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
//)
//};


