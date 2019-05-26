/*
 * motor_ctrl.c
 *
 *  Created on: May 20, 2019
 *      Author: quack
 */

#include "motor_ctrl.h"


void MotorInit(){
	 SctInit(100); // Set the frequency of the PWM
	 SctStart();   // Start the PWM timer
}

void MotorCfg(motor_t * motor,sctPinId sct_id,gpioNumber_t n){

	motor->pwm_pin.id = sct_id;
	motor->pwm_pin.duty_cycle = 0;

	motor->dir_pin.n = n;
	motor->dir_pin.dir = GPIO_OUT;
	motor->dir_pin.init_st = GPIO_LOW;

	SctConfig(&motor->pwm_pin); // Configures the SCTimer
	GpioConfig(&motor->dir_pin); // Configures the GPIO to control the directions
}



void MotorSet(motor_t * motor, uint8_t pwm, dir_t dir){

	pwm =pwm%101;

	if(dir == FORWARD){
		GpioWrite(&motor->dir_pin,GPIO_LOW);
		SctSetDutyCycle(&motor->pwm_pin,pwm);
	}else if(dir == BACKWARD){
		GpioWrite(&motor->dir_pin,GPIO_HIGH);
		SctSetDutyCycle(&motor->pwm_pin,100-pwm);
	}
}

void MotorRst(motor_t * motor){
	GpioWrite(&motor->dir_pin,GPIO_LOW);
	SctSetDutyCycle(&motor->pwm_pin,0);
}
