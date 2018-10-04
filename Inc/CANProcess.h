/*
 * CANRXProcess.h
 *
 *  Created on: Dec 22, 2016
 *      Author: ben
 */

#ifndef CANPROCESS_H_
#define CANPROCESS_H_

//includes
#include "motor_controller_functions.h"
//#include "WheelModule.h"

//defines for reading data from RxCanMsgTypeDef
#define ID_PEDALBOX1							0x500
#define ID_PEDALBOX2							0x501
#define	ID_PEDALBOXCALIBRATE					0x503
#define ID_BAMOCAR_STATION_TX					0x210	//message recieved by MC
#define ID_BAMOCAR_STATION_RX					0x190	//message sent by MC
//#define ID_BMS_PACK_VOLTAGE						0x400
#define ID_WHEEL_FR								0x100	// wheel module IDs
#define ID_WHEEL_FL								0x101
#define ID_WHEEL_RR								0x102
#define ID_WHEEL_RL								0x103
#define ID_DASHBOARD							0x350
#define ID_DASHBOARD1							0x351
#define ID_DASHBOARD2							0x352
#define ID_BMS_PACK_CUR_VOL						0x03B //current and voltage
#define ID_BMS_DCL								0x03C
#define ID_PEDALBOX_ERRORS						0x601

//wheel module defines
#define WM_SPEED_7_0_BYTE						2
#define WM_SPEED_11_8_BYTE						1
#define WM_SPEED_11_8_MASK						0x0F00






/**
  * @brief  CAN Tx message structure definition
  */
typedef struct
{
  uint32_t StdId;    /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId;    /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC;      /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8];   /*!< Contains the data to be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

}CanTxMsgTypeDef;

/**
  * @brief  CAN Rx message structure definition
  */
typedef struct
{
  uint32_t StdId;       /*!< Specifies the standard identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId;       /*!< Specifies the extended identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE;         /*!< Specifies the type of identifier for the message that will be received.
                             This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR;         /*!< Specifies the type of frame for the received message.
                             This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC;         /*!< Specifies the length of the frame that will be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8];      /*!< Contains the data to be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FMI;         /*!< Specifies the index of the filter the message stored in the mailbox passes through.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FIFONumber;  /*!< Specifies the receive FIFO number.
                             This parameter can be CAN_FIFO0 or CAN_FIFO1 */

}CanRxMsgTypeDef;

void ISR_RXCAN();
void CANFilterConfig();
void taskRXCANProcess();
void taskTXCAN();
void taskRXCAN();
void processBamoCar(CanRxMsgTypeDef* rx);
void processWheelModuleFrame(CanRxMsgTypeDef* rx);
void processPedalboxFrame(CanRxMsgTypeDef* rx);


void processCalibrate(CanRxMsgTypeDef* rx);

#endif /* CANPROCESS_H_ */
