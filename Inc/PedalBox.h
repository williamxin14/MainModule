/*
 * PedalBox.h
 *
 *  Created on: Jan 12, 2017
 *      Author: ben
 */

#ifndef PEDALBOX_H_
#define PEDALBOX_H_

#include "stdint.h"

#define QUEUE_SIZE_PEDALBOXMSG		16
#define PEDALBOX_TIMEOUT			500 / portTICK_RATE_MS
#define MAX_BRAKE_LEVEL 			0xFFF
#define THROTTLE_DEADZONE_LOW			.1
#define THROTTLE_DEADZONE_HIGH			.1
#define BRAKE_PRESSED_THRESHOLD	.3
#define APPS_BP_PLAUS_RESET_THRESHOLD .05  //EV 2.5
#define APPS_BP_PLAUS_THRESHOLD .25  //EV 2.5


typedef enum {
	PEDALBOX_STATUS_ERROR = 1, //generic error
	PEDALBOX_STATUS_ERROR_EOR, // encoder out of range
	PEDALBOX_STATUS_ERROR_APPSIMP,  //APPS Implausibility error, EV 2.3.5,
	PEDALBOX_STATUS_ERROR_APPSIMP_PREV,  //APPS Implausibility error, provisional (before it has lasted .1 second)
	PEDALBOX_STATUS_ERROR_BPIMP,	//brake pedal implaus //EV 2.5.1,
	PEDALBOX_STATUS_NO_ERROR = 0
} Pedalbox_status_t;

typedef enum {
	BRAKE_STATUS_PRESSED,
	BRAKE_STATUS_NOT_PRESSED
} Brake_status_t;

// Structure to hold data passed through the queue to pedalBoxMsgHandler
typedef struct _pedalbox_msg {
	int 				throttle1_raw;		// raw throttle data from pedalbox
	int 				throttle2_raw;
	int  				brake1_raw;
	int 				brake2_raw;

} Pedalbox_msg_t;

float apps_getThrottlePos();
float apps_getBrake();
void apps_init();
Pedalbox_status_t apps_getStatus_imp();
Pedalbox_status_t apps_getStatus_bp();
Pedalbox_status_t apps_getStatus_eor();
Pedalbox_status_t apps_getStatus_timeout();


#endif /* PEDALBOX_H_ */
