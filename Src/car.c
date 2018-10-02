/***************************************************************************
*
*     File Information
*
*     Name of File: car.c
*
*     Authors (Include Email):
*       1. Ben Ng				xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. car.h
*
*     File Description:
*     	Functions to control the physical car
*
***************************************************************************/

#include "car.h"
#include "accelerometer.h"
#include "PedalBox.h"

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
	return;
}

void carInit() {
	car.state = CAR_STATE_INIT;
	car.pb_mode = PEDALBOX_MODE_DIGITAL;

	car.phcan = &hcan1;
	car.calibrate_flag = CALIBRATE_NONE;

	apps_init();
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: ISR_StartButtonPressed
*
*     Programmer's Name: Ben Ng			xbenng@gmail.com
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*
*     Function Description:
*     		called when start button is pressed.
*
***************************************************************************/
void ISR_StartButtonPressed() {
	if (car.state == CAR_STATE_INIT)
	{
		if (
			brake_isPressed() //check if brake is pressed before starting car
			&& pc_isComplete() //check if precharge has finished
		)
		{
			car.state = CAR_STATE_PREREADY2DRIVE;  //put car into preready to drive state
		}
	} else {
		car.state = CAR_STATE_RESET;
	}
}

PC_status_t pc_isComplete()
{
	return HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) == PC_COMPLETE;
}

//TODO Potential MC ping function
//TODO BMS functions

int mainModuleWatchdogTask() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: mainModuleTimeCheckIdle
*
*     Programmer's Name: Kai Strubel
*     					 Ben Ng			xbenng@gmail.com
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*	    1.bool launchControl
*		2.float MMPB_TIME time pedal box message handler function was last run
*		3.float MMWM_TIME time wheel module handler function was last run
*		4.float torque
*		5.float currentTime
*
*     Function Description:
*		Checks if wheel module and pedal box are still communicating
*
***************************************************************************/
	while (1) {
		/*
		//check how old the wheel module data is, if its too old, then turn off LC
		if (current_time_ms - MMWM_TIME > LC_THRESHOLD) {
			launchControl = 0;
			//error
		}*/
		vTaskDelay(500);
	}
}

int taskHeartbeat() {
/***************************************************************************
*.
*     Function Information
*
*     Name of Function: heartbeatIdle
*
*     Programmer's Name: Kai Strubel
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*
*     Function Description:
*		Heart beat to communicate that main module is alive
*
***************************************************************************/
	// write to GPIO
	while (1) {
		HAL_GPIO_TogglePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin);
		vTaskDelay(HEARTBEAT_PERIOD);
	}
}

void initRTOSObjects() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: startTasks
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*     Global Dependents:
*
*     Function Description:
*		all xTaskCreate calls
*		all xQueueCreate calls
*
***************************************************************************/

	/* Create Queues */

	car.q_rxcan = 			xQueueCreate(QUEUE_SIZE_RXCAN, sizeof(CanRxMsgTypeDef));
	car.q_txcan = 			xQueueCreate(QUEUE_SIZE_TXCAN, sizeof(CanTxMsgTypeDef));
//	car.q_mc_frame = 		xQueueCreate(QUEUE_SIZE_MCFRAME, sizeof(CanRxMsgTypeDef));

	car.m_CAN =				xSemaphoreCreateMutex(); //mutex to protect CAN peripheral

	/* Create Tasks */

	//todo optimize stack depths http://www.freertos.org/FAQMem.html#StackSize
	xTaskCreate(taskPedalBoxMsgHandler, "PedalBoxMsgHandler", 256, NULL, 1, NULL);
	xTaskCreate(taskCarMainRoutine, "CarMain", 256 , NULL, 1, NULL);
	xTaskCreate(taskTXCAN, "TX CAN", 256, NULL, 1, NULL);
	xTaskCreate(taskBlink, "blink", 256, NULL, 1, NULL);
	xTaskCreate(taskSendAccelero, "accelro", 256, NULL, 1, NULL);
	//xTaskCreate(taskMotorControllerPoll, "Motor Poll", 256, NULL, 1, NULL);
 }

void taskBlink(void* can)
{
	while (1)
	{
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);  //orange LED

		CanTxMsgTypeDef tx;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.StdId = 0x200;
		tx.DLC = 1;
		tx.Data[0] = 0;

		xQueueSendToBack(car.q_txcan, &tx, 100);

		//bamocar timeout message
		//req regid 40
		//mcCmdTransmissionRequestSingle(0x40);

		vTaskDelay(250 / portTICK_RATE_MS);
	}
}


void taskSoundBuzzer(uint32_t* time_ms) {
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




void taskCarMainRoutine() {
	while (1)
	{
		float torque_to_send = 0;
		//always active block (do this no matter what state.)

		//check if brake level is greater than the threshold level
		if (brake_isPressed()) {
			//brake is presssed
			carSetBrakeLight(BRAKE_LIGHT_ON);  //turn on brake light
		} else {
			//brake is not pressed
			carSetBrakeLight(BRAKE_LIGHT_OFF);  //turn off brake light
		}

		if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) == GPIO_PIN_SET && car.state == CAR_STATE_READY2DRIVE)
		{
			car.state = CAR_STATE_RESET;
		}

		mcCmdTorqueFake(apps_getThrottlePos());

		CanTxMsgTypeDef tx;
		tx.StdId = ID_PEDALBOX_ERRORS;
		tx.Data[0] = apps_getStatus_bp();
		tx.Data[1] = apps_getStatus_eor();
		tx.Data[2] = apps_getStatus_imp();
		tx.Data[3] = apps_getStatus_timeout();
		tx.DLC = 4;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		xQueueSendToBack(car.q_txcan, &tx, 100);

		//state dependent block
		if (car.state == CAR_STATE_INIT)
		{
			disableMotor();
			//HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET); //turn on pump

			//assert these pins always
			HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_PIN_SET); //close SDC
			//HAL_GPIO_WritePin(Motor_Controller_Relay_CTRL_GPIO_Port, Motor_Controller_Relay_CTRL_Pin, GPIO_PIN_SET); //turn on mc
			//car.state = CAR_STATE_PREREADY2DRIVE;  //car is started

		}
		else if (car.state == CAR_STATE_PREREADY2DRIVE)
		{

			HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET); //turn on pump
			//bamocar 5.2
			//Contacts of the safety device closed,
			//enable FRG/RUN 0.5s after RFE.
			enableMotorController();
			//turn on buzzer
			carSoundBuzzer(2500);
			car.state = CAR_STATE_READY2DRIVE;  //car is started
			HAL_GPIO_WritePin(BATT_FAN_GPIO_Port, BATT_FAN_Pin, GPIO_PIN_SET);
			if (HAL_GPIO_ReadPin(P_AIR_STATUS_GPIO_Port, P_AIR_STATUS_Pin) == PC_COMPLETE)
			{
				car.state = CAR_STATE_READY2DRIVE;  //car is started
			}
		}
		else if (car.state == CAR_STATE_READY2DRIVE)
		{
			if (apps_getStatus_bp() == PEDALBOX_STATUS_NO_ERROR &&
				apps_getStatus_eor() == PEDALBOX_STATUS_NO_ERROR &&
				apps_getStatus_imp() == PEDALBOX_STATUS_NO_ERROR &&
				apps_getStatus_timeout()== PEDALBOX_STATUS_NO_ERROR)
			{
				torque_to_send = apps_getThrottlePos();
			} else {
				torque_to_send = 0;
			}
		}
		else if (car.state == CAR_STATE_ERROR)
		{
			//disableMotor();
		}
		else if (car.state == CAR_STATE_RESET)
		{
			HAL_GPIO_WritePin(PUMP_GPIO_Port,PUMP_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RFE_GPIO_Port,RFE_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FRG_RUN_GPIO_Port,FRG_RUN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BATT_FAN_GPIO_Port,BATT_FAN_Pin,GPIO_PIN_RESET);
			car.state = CAR_STATE_INIT;

		}
		else if (car.state == CAR_STATE_RECOVER)
		{
			HAL_GPIO_WritePin(RFE_GPIO_Port,RFE_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FRG_RUN_GPIO_Port,FRG_RUN_Pin,GPIO_PIN_RESET);
			vTaskDelay((uint32_t) 500 / portTICK_RATE_MS);
			enableMotorController();
			car.state = CAR_STATE_READY2DRIVE;
		}
		// calculate
//			calcTorqueLimit = (80000 / (actualDC * 10 * actualV * 10)); //(DCLimit / (actualDC * 10)) * actualTorque;
//			if(torque_to_send/MAX_THROTTLE_LEVEL > calcTorqueLimit)
//			{
//				torque_to_send = calcTorqueLimit * torque_to_send;
//			}

		mcCmdTorque(torque_to_send);  //command the MC to move the motor

		//wait until
		vTaskDelay(PERIOD_TORQUE_SEND);

	}

}
