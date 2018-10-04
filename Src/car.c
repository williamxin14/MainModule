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
#include "motor_controller_functions.h"
#include "accessories.h"



void carInit() {
	car.state = CAR_STATE_INIT;
	car.pb_mode = PEDALBOX_MODE_DIGITAL;

	car.phcan = &hcan1;
	car.calibrate_flag = CALIBRATE_NONE;

	apps_init();
}

//TODO BMS functions


void initRTOSObjects() {  //TODO this should be done in each module respectively
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
	xTaskCreate(taskCarMainRoutine, "CarMain", 256 , NULL, 1, NULL);
	xTaskCreate(taskRXCANProcess, "RX CAN", 256, NULL, 1, NULL);
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
				brake_isPressed() &&
				pc_getStatus() == PC_COMPLETE
			)
		{
			car.state = CAR_STATE_PREREADY2DRIVE;  //put car into preready to drive state
		}
	} else {
		car.state = CAR_STATE_RESET;
	}
}


void sendErrorFrame()
{
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
}


void taskCarMainRoutine() {
	while (1)
	{
		float torque_to_send = 0;
		//always active block (do this no matter what state.)

		//check if brake level is greater than the threshold level
		if (brake_isPressed() == BRAKE_STATUS_PRESSED) {
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
		sendErrorFrame();

		//state dependent block
		if (car.state == CAR_STATE_INIT)
		{
			mc_disableMotor();
			//assert these pins always
			HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_PIN_SET); //close SDC
		}
		else if (car.state == CAR_STATE_PREREADY2DRIVE)
		{
			carSetBattFan(BATTFAN_ON);
			carSetPump(PUMP_ON);

			mc_enableMotor();
			//turn on buzzer
			carSoundBuzzer(2500);

			car.state = CAR_STATE_READY2DRIVE;  //car is started
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
			carSetBattFan(BATTFAN_OFF);
			carSetPump(PUMP_OFF);
			mc_disableMotor();
			car.state = CAR_STATE_INIT;

		}
		else if (car.state == CAR_STATE_RECOVER)
		{
			mc_disableMotor();
			vTaskDelay((uint32_t) 500 / portTICK_RATE_MS);
			mc_enableMotor();
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
		vTaskDelay(TICK_MAIN);

	}
}
