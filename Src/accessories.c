/*
 * accessories.c
 *
 *  Created on: Oct 2, 2018
 *      Author: ben
 */

#include "accessories.h"

/*private functions*/
static void taskSoundBuzzer(uint32_t* time_ms);


void carSetBrakeLight(Brake_light_status_t status)
/***************************************************************************
*
*     Function Information
*
*     Name of Function: setBrakeLight
*
*     Programmer's Name: Ben Ng xbenng@gmail.com
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. Brake_light_status_t status, value to write to GPIO pin
*
*      Global Dependents:
*
*     Function Description:
*			turns brakelight on or off
***************************************************************************/
{
	HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, status);
}

void carSetPump(Pump_status_t status)
/***************************************************************************
*
*     Function Information
*
*     Name of Function: carSetPump
*
*     Programmer's Name: Ben Ng xbenng@gmail.com
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. Pump_status_t status, value to write to GPIO pin
*
*      Global Dependents:
*
*     Function Description:
*			turns pump on or off
***************************************************************************/
{
	HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, status);
}

void carSetBattFan(BattFan_status_t status)
/***************************************************************************
*
*     Function Information
*
*     Name of Function: setBrakeLight
*
*     Programmer's Name: Ben Ng xbenng@gmail.com
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. Brake_light_status_t status, value to write to GPIO pin
*
*      Global Dependents:
*
*     Function Description:
*			turns brakelight on or off
***************************************************************************/
{
	HAL_GPIO_WritePin(BATT_FAN_GPIO_Port, BATT_FAN_Pin, status);
}


void carSoundBuzzer(uint32_t timeMS)
/***************************************************************************
*
*     Function Information
*
*     Name of Function: setBrakeLight
*
*     Programmer's Name: Ben Ng xbenng@gmail.com
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. uint32_t timeMS, time to sound buzzer
*
*      Global Dependents:
*
*     Function Description:
*			asynchronously sounds buzzer for timeMS number of milliseconds
*			immediately returns after starting task
***************************************************************************/
{
	xTaskCreate(taskSoundBuzzer, "SoundBuzzer", 256, &timeMS, 10, NULL);
}

static void taskSoundBuzzer(uint32_t* time_ms) {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskSoundBuzzer
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*     Global Dependents:
*
*     Function Description:
*		ready to drive sound task
*
***************************************************************************/
	while (1) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET); //turn on buzzer
		//enable FRG/RUN 0.5s after RFE.
		vTaskDelay(*time_ms / portTICK_RATE_MS);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); //turn off buzzer
		vTaskDelete(NULL);
	}
}

PC_status_t pc_getStatus()
/***************************************************************************
*
*     Function Information
*
*     Name of Function: pc_isComplete()
*
*     Programmer's Name: Kai Strubel
*     					 Ben Ng			xbenng@gmail.com
*
*     Function Return Type: PC_status_t
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*
*     Function Description:
*		Returns status of precharge
*
***************************************************************************/

{
	return HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin);
}

