/*
 * motor_ctrl.h
 *
 *  Created on: May 20, 2019
 *      Author: quack
 */

#ifndef PROJECTS_SPEED_CONTROL_INC_MOTOR_CTRL_H_
#define PROJECTS_SPEED_CONTROL_INC_MOTOR_CTRL_H_

#include "sct_hal.h"
#include "gpio_hal.h"

typedef enum {FORWARD,BACKWARD} dir_t;

typedef struct{
	sctPin_t pwm_pin;
	gpioPin_t dir_pin;
}motor_t;



void MotorInit();

void MotorCfg(motor_t * motor,sctPinId sct_id,gpioNumber_t n);

void MotorSet(motor_t * motor, uint8_t pwm, dir_t dir);

void MotorRst(motor_t * motor);



#endif /* PROJECTS_SPEED_CONTROL_INC_MOTOR_CTRL_H_ */
