/*
 * accessories.h
 *
 *  Created on: Oct 2, 2018
 *      Author: ben
 */

#ifndef ACCESSORIES_H_
#define ACCESSORIES_H_

#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

typedef enum
{
	BRAKE_LIGHT_OFF = GPIO_PIN_RESET,
  	BRAKE_LIGHT_ON = GPIO_PIN_SET
} Brake_light_status_t;

typedef enum
{
	PUMP_OFF = GPIO_PIN_RESET,
  	PUMP_ON = GPIO_PIN_SET
} Pump_status_t;

typedef enum
{
	BATTFAN_OFF = GPIO_PIN_RESET,
  	BATTFAN_ON = GPIO_PIN_SET
} BattFan_status_t;

typedef enum
{
	PC_INPROGRESS = GPIO_PIN_SET,
	PC_COMPLETE = GPIO_PIN_RESET
} PC_status_t;

/*public functions*/
extern void carSetBrakeLight(Brake_light_status_t status);
extern void carSetPump(Pump_status_t status);
extern void carSetBattFan(BattFan_status_t status);
extern void carSoundBuzzer(uint32_t timeMS);
extern PC_status_t pc_getStatus();


#endif /* ACCESSORIES_H_ */
