/*
 * optical_enc.h
 *
 *  Created on: May 20, 2019
 *      Author: quack
 */

#ifndef PROJECTS_SPEED_CONTROL_INC_OPTICAL_ENC_H_
#define PROJECTS_SPEED_CONTROL_INC_OPTICAL_ENC_H_

#include "gpio_hal.h"

#define NOF_ENCODERS	4
typedef enum{ ENC_1,ENC_2,ENC_3,ENC_4} optEncId_t;


typedef struct{
	optEncId_t 	id;
	gpioPin_t   input_pin;
	uint32_t 	distance;
	float   	speed;
} optEnc_t;

void EncoderInit();

void EncoderCfg(optEnc_t *enc, optEncId_t id,gpioNumber_t n,uint8_t pin_int_num);

void EncoderUpdate(optEnc_t *enc);

void EncoderOdomToZero(optEnc_t *enc);

uint32_t EncoderGetDistance(optEnc_t *enc);

float EncoderGetSpeed(optEnc_t *enc);

#endif /* PROJECTS_SPEED_CONTROL_INC_OPTICAL_ENC_H_ */
