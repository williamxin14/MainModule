/***************************************************************************
*
*     File Information
*
*     Name of File: CANRXProcess.c
*
*     Authors (Include Email):
*       1. Ben Ng,       xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. FreeRTOS.h
* 		2. stm32f7xx_hal_can.h
* 		3. CANRXProcess.h
*
*     File Description: Used for interpreting incoming CAN messages on
*						main module
*
***************************************************************************/
#include <CANProcess.h>

#include "car.h"
#include "PedalBox.h"


/***************************************************************************
*
*     Function Information
*
*     Name of Function: HAL_CAN_RxFifo0MsgPendingCallback
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type:
*
*     Parameters (list data type, name, and comment one per line):
*
*     Global Dependents:
*	  1. ;
*
*     Function Description:
*			To be called by HAL_CAN_IRQHandler when a CAN message is received.
*
***************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

	CanRxMsgTypeDef rx;
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, 0, &header, rx.Data);
	rx.DLC = header.DLC;
	rx.StdId = header.StdId;
	xQueueSendFromISR(car.q_rxcan, &rx, NULL);

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

	CanRxMsgTypeDef rx;
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, 1, &header, rx.Data);
	rx.DLC = header.DLC;
	rx.StdId = header.StdId;
	xQueueSendFromISR(car.q_rxcan, &rx, NULL);

}


/***************************************************************************
*
*     Function Information
*
*     Name of Function: RXCANProcessTask
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type:
*
*     Parameters (list data type, name, and comment one per line):
*       1. CAN_HandleTypeDef *hcan, hcan structure address to add filter to
*
*     Global Dependents:
*	    1.
*
*     Function Description: Filter Configuration.
*
***************************************************************************/
void CANFilterConfig()
{


	  CAN_FilterTypeDef FilterConf;
	  FilterConf.FilterIdHigh =         ID_PEDALBOX2 << 5; // 2 num
	  FilterConf.FilterIdLow =          ID_DASHBOARD << 5; // 0
	  FilterConf.FilterMaskIdHigh =     0x7ff;       // 3
	  FilterConf.FilterMaskIdLow =      0x7fe;       // 1
	  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
	  FilterConf.FilterBank = 0;
	  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
	  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
	  FilterConf.FilterActivation = ENABLE;
	  HAL_CAN_ConfigFilter(car.phcan, &FilterConf);
}


/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskTXCAN
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*	   1.
*
*     Function Description:
*     	Task function to send CAN messages using the CAN peripheral
*
***************************************************************************/
void taskTXCAN()
{
	CanTxMsgTypeDef tx;

	for (;;)
	{
		//check if this task is triggered
		if (xQueuePeek(car.q_txcan, &tx, portMAX_DELAY) == pdTRUE)
		{
			xQueueReceive(car.q_txcan, &tx, portMAX_DELAY);  //actually take item out of queue
			CAN_TxHeaderTypeDef header;
			header.DLC = tx.DLC;
			header.IDE = tx.IDE;
			header.RTR = tx.RTR;
			header.StdId = tx.StdId;
			header.TransmitGlobalTime = DISABLE;
			uint32_t mailbox;
			HAL_CAN_AddTxMessage(car.phcan, &header, tx.Data, &mailbox);
			while (!HAL_CAN_GetTxMailboxesFreeLevel(car.phcan)); // while mailboxes not free
		}
	}
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskRXCANprocess
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*	   1.-
*
*     Function Description:
*     	Task function to process received CAN Messages.
*     	CanRxMsgTypeDef are sent here to the q_rxcan queue to be processed
*     	from the CAN RX interrupt handler.
*     	The data is process and handled according to what kind of message is received
*
***************************************************************************/
void taskRXCANProcess()
{

	CanRxMsgTypeDef rx;  //CanRxMsgTypeDef to be received on the queue
	while (1)
	{

		//if there is a CanRxMsgTypeDef in the queue, pop it, and store in rx
		if (xQueueReceive(car.q_rxcan, &rx, portMAX_DELAY) == pdTRUE)
		{
			//A CAN message has been recieved
			//check what kind of message we received
			switch (rx.StdId)
			{
				case ID_PEDALBOX2:  //if pedalbox1 message
				{
					processPedalboxFrame(&rx);
					break;
				}
				case ID_PEDALBOXCALIBRATE: {
					processCalibrate(&rx);
					break;
				}
				case ID_BAMOCAR_STATION_RX: { //if bamocar message
					//forward frame to mc frame queue
					processBamoCar(&rx);
				}
				case  	ID_WHEEL_FR:
				case	ID_WHEEL_FL:
				case	ID_WHEEL_RR:
				case	ID_WHEEL_RL: //todo add all the other wheel module IDs
				{
					//processWheelModuleFrame(&rx);
					break;
				}
				case	ID_DASHBOARD:
				{
					ISR_StartButtonPressed();
					break;
				}
				case	ID_DASHBOARD1:
				{
					if (car.state == CAR_STATE_READY2DRIVE)
					{
						car.state = CAR_STATE_RECOVER;
					}
					break;
				}
				case	ID_DASHBOARD2:
				{
					HAL_GPIO_TogglePin(PUMP_GPIO_Port, PUMP_Pin);
					break;
				}
				case ID_BMS_PACK_CUR_VOL:
					actualDC = rx.Data[1] | (rx.Data[1] << 8);
					actualV = rx.Data[3] | (rx.Data[2]<< 8);
					break;
			}
		}
	}
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: processPedalboxFrame
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*		1. CanRxMsgTypeDef* rx, CAN frame to be converted into a pedalbox message
*      Global Dependents:
*	    1.
*
*     Function Description:
*     	Converts CAN frame into a pedalbox message and sends it to pedalboxmsg handler
*
***************************************************************************/
void processPedalboxFrame(CanRxMsgTypeDef* rx)
{
	xQueueSendToBack(q_pedalboxmsg, rx, 100);
}

void processCalibrate(CanRxMsgTypeDef* rx) {
	//set the calibration flag, so calibration values are updated upon reception of new pedalboxmsg
	if 		  (rx->Data[0] == 0x01) {
		car.calibrate_flag = CALIBRATE_THROTTLE_MAX; //calibrate high
	} else if (rx->Data[0] == 0x02) {
		car.calibrate_flag = CALIBRATE_THROTTLE_MIN; //calibrate low
	} else if (rx->Data[0] == 0x03) {
		car.calibrate_flag = CALIBRATE_BRAKE_MAX; //calibrate high
	} else if (rx->Data[0] == 0x04) {
		car.calibrate_flag = CALIBRATE_BRAKE_MIN; //calibrate low
	} else {
		car.calibrate_flag = CALIBRATE_NONE;
	}

}
/***************************************************************************
*
*     Function Information
*
*     Name of Function: processBamoCar
*
*     Programmer's Name: Raymond Dong, raymonddonghome@gmail.com
*
*     Function Return Type: none
*
*     Parameters:
*		1. CanRxMsgTypeDef* rx, CAN frame to be converted into a bamocar message
*
*     Global Dependents:
*	    1. actualTorque
*		2. actualDC
*		3. DCLimit
*		4. pedalTorque
*
*     Function Description:
*     	Updates variables
*
***************************************************************************/
void processBamoCar(CanRxMsgTypeDef* rx)
{
	/*
	{actual torque}				{actual dc current}			{dc current limit}
	{calculated torque limit}	{pedal torque}
	{calculated torque limit} = {dc current limit}/{actual dc current} * {actual torque}
	if {calculated torque limit} > {pedal torque}
		send {calculated torque limit}
	else
		send {pedal torque}
	*/
//	if (rx->Data[0] == REGID_I_ACT)
//	{
//		actualTorque0700 = rx->Data[1];
//		actualTorque1508 = rx->Data[2];
//		actualTorque = actualTorque0700 | (actualTorque1508 << 8);
//		BCparam = 1;	//actualTorque received
//	}
//	if (rx->Data[0] == ID_BMS_PACK_CUR_VOL)
//	{
//
//		BCparam = 2;	//actualDc received
//	}
//	if (rx->Data[0] == ID_BMS_DCL)
//	{
//
//		BCparam = 3;	//DCLimit received
//	}
	if (rx->Data[0] == REGID_I_ACT)
	    {
	        actualTorque0700 = rx->Data[1];
	        actualTorque1508 = rx->Data[2];
	        actualTorque = actualTorque0700 | (actualTorque1508 << 8);
	        BCparam = 1;    //actual torque received
	    }
	    if (rx->Data[0] == REGID_SPEED_ACTUAL)
	    {
	        speedActual = (rx -> Data[1] << 8 | rx -> Data[2]);
	        BCparam = 2;    //speed actual received
	    }
	    if (rx->Data[0] == REGID_I_IST)
	    {
	        currentActual = (rx -> Data[1] << 8 | rx -> Data[2]);
	        BCparam = 3;    //current actual received
	    }
	    if (rx->Data[0] == REGID_I_SOLL)
	    {
	        commandCurrent = (rx -> Data[1] << 8 | rx -> Data[2]);
	        BCparam = 4;    //command current received
	    }
	    if (rx->Data[0] == REGID_DC_BUS)
	    {
	        dcBusVoltage = (rx -> Data[1] << 8 | rx -> Data[2]);
	        BCparam = 5;    //current actual received
	    }
	    if (rx->Data[0] == REGID_T_MOTOR)
	    {
	        motorTemperature = (rx -> Data[1] << 8 | rx -> Data[2]);
	        BCparam = 6;    //motor temperature received
	    }
	    if (rx->Data[0] == REGID_T_IGBT)
	    {
	        powerStageTemperature = (rx -> Data[1] << 8 | rx -> Data[2]);
	        BCparam = 7;    //power stage temperature received
	    }
	    if (rx->Data[0] == REGID_T_AIR)
	    {
	        airTemperature = (rx -> Data[1] << 8 | rx -> Data[2]);
	        BCparam = 8;    //air temperature received
	    }
	    if (rx->Data[0] == REGID_I_REDA)
	    {
	        actualCurrentLimit = (rx -> Data[1] << 8 | rx -> Data[2]);
	        BCparam = 9;    //actual current limit received
	    }
	    if (rx->Data[0] == REGID_ERR_BITMAP1)
	    {
	        errBitMap1 = (rx -> Data[1] << 8 | rx -> Data[2]);
	        BCparam = 10;    //errBitMap1 received
	    }
}
