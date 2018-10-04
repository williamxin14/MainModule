/*
 * PedalBox.h
 *
 *  Created on: Jan 12, 2017
 *      Author: ben
 */

#ifndef PEDALBOX_H_
#define PEDALBOX_H_

#include "stdint.h"
#include "FreeRTOS.h"
#include "queue.h"


#define QUEUE_SIZE_PEDALBOXMSG		16
#define PEDALBOX_TIMEOUT			500 / portTICK_RATE_MS
#define THROTTLE_DEADZONE_LOW			.1
#define THROTTLE_DEADZONE_HIGH			.1
#define BRAKE_PRESSED_THRESHOLD	.3
#define APPS_BP_PLAUS_RESET_THRESHOLD .05  //EV 2.5
#define APPS_BP_PLAUS_THRESHOLD .25  //EV 2.5

//pedalbox CAN message //todo not sure if better to send whole frame or just pbmsg.
#define PEDALBOX1_FILTER 						0	//filter number corresponding to the PEDALBOX1 message
#define PEDALBOX1_THROT1_7_0_BYTE				1
#define PEDALBOX1_THROT1_7_0_OFFSET				0
#define PEDALBOX1_THROT1_7_0_MASK				0b11111111
#define PEDALBOX1_THROT1_11_8_BYTE				0
#define PEDALBOX1_THROT1_11_8_OFFSET			0
#define PEDALBOX1_THROT1_11_8_MASK				0b00001111
#define PEDALBOX1_THROT2_7_0_BYTE				3
#define PEDALBOX1_THROT2_7_0_OFFSET				0
#define PEDALBOX1_THROT2_7_0_MASK				0b11111111
#define PEDALBOX1_THROT2_11_8_BYTE				2
#define PEDALBOX1_THROT2_11_8_OFFSET				0
#define PEDALBOX1_THROT2_11_8_MASK				0b00001111
//brake
#define PEDALBOX1_BRAKE1_7_0_BYTE				5
#define PEDALBOX1_BRAKE1_7_0_OFFSET				0
#define PEDALBOX1_BRAKE1_7_0_MASK				0b11111111
#define PEDALBOX1_BRAKE1_11_8_BYTE				4
#define PEDALBOX1_BRAKE1_11_8_OFFSET			0
#define PEDALBOX1_BRAKE1_11_8_MASK				0b00001111
#define PEDALBOX1_BRAKE2_7_0_BYTE				7
#define PEDALBOX1_BRAKE2_7_0_OFFSET				0
#define PEDALBOX1_BRAKE2_7_0_MASK				0b11111111
#define PEDALBOX1_BRAKE2_11_8_BYTE				6
#define PEDALBOX1_BRAKE2_11_8_OFFSET			0
#define PEDALBOX1_BRAKE2_11_8_MASK				0b00001111

#define PEDALBOX1_EOR_BYTE						3
#define PEDALBOX1_EOR_OFFSET					0
#define PEDALBOX1_EOR_MASK						0b00000001
#define PEDALBOX1_IMP_BYTE						3
#define PEDALBOX1_IMP_OFFSET					1
#define PEDALBOX1_IMP_MASK						0b00000010


typedef enum {
	PEDALBOX_STATUS_ERROR = 1, //generic error
	PEDALBOX_STATUS_ERROR_EOR, // encoder out of range
	PEDALBOX_STATUS_ERROR_APPSIMP,  //APPS Implausibility error, EV 2.3.5,
	PEDALBOX_STATUS_ERROR_APPSIMP_PREV,  //APPS Implausibility error, provisional (before it has lasted .1 second)
	PEDALBOX_STATUS_ERROR_BPIMP,	//brake pedal implaus //EV 2.5.1,
	PEDALBOX_STATUS_NO_ERROR = 0
} Pedalbox_status_t;

typedef enum {
	BRAKE_STATUS_PRESSED = 1,
	BRAKE_STATUS_NOT_PRESSED = 0
} Brake_pressed_status_t;

// Structure to hold data passed through the queue to pedalBoxMsgHandler
typedef struct _pedalbox_msg {
	int 				throttle1_raw;		// raw throttle data from pedalbox
	int 				throttle2_raw;
	int  				brake1_raw;
	int 				brake2_raw;

} Pedalbox_msg_t;


/*public functions*/
extern void apps_init();
extern float apps_getThrottlePos();
extern float apps_getBrakePos();
extern Brake_pressed_status_t brake_isPressed();
extern Pedalbox_status_t apps_getStatus_imp();
extern Pedalbox_status_t apps_getStatus_bp();
extern Pedalbox_status_t apps_getStatus_eor();
extern Pedalbox_status_t apps_getStatus_timeout();

/*public variables*/
extern QueueHandle_t q_pedalboxmsg;



#endif /* PEDALBOX_H_ */
