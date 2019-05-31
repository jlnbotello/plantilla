/*
 * optical_enc.c
 *
 *  Created on: May 20, 2019
 *      Author: quack
 */
#include "optical_enc.h"
#include "sapi_timer.h"

#define TIMER_FREQ 		204000000
#define MAX_COUNTS 		TIMER_FREQ/5 // 1 [s]
#define WHEEL_DIAM 		22 // [cm]
#define DISK_TEETH 		20
#define PULSE_DISTANCE 	WHEEL_DIAM/DISK_TEETH
#define SPEED_CONSTANT	224400000//TIMER_FREQ * WHEEL_DIAM/DISK_TEETH



typedef struct{
	optEnc_t *enc;
	uint32_t last_tc;
	uint8_t  timer_roll;
} optEncInternalData_t;

optEncInternalData_t encoders[NOF_ENCODERS];

static void EncoderTimerRoll();


void EncoderInit(){
	Timer_Init(TIMER0, MAX_COUNTS, EncoderTimerRoll);

}


void EncoderCfg(optEnc_t *enc,optEncId_t id,gpioNumber_t n,uint8_t pin_int_num){
	enc->id = id;
	encoders[enc->id].enc = enc;
	encoders[enc->id].last_tc = 0;
	enc->input_pin.n = n;
	enc->input_pin.dir = GPIO_IN_PULLUP;
	GpioConfig(&enc->input_pin);
	GpioInterruptConfig(&enc->input_pin,GPIO_IRQ_EDGE_FALL,pin_int_num);
	enc->distance=0;
	enc->speed=0;
}

void EncoderUpdate(optEnc_t *enc){
	uint32_t tc = Timer_ReadCount(TIMER0);
	uint32_t diff;

	if(encoders[enc->id].timer_roll == TRUE){
		diff = tc + (MAX_COUNTS - encoders[enc->id].last_tc);
		encoders[enc->id].timer_roll = FALSE;
	}else{
		diff = tc - (encoders[enc->id].last_tc);
	}
	if(diff>=MAX_COUNTS){
		enc->speed = 0; // no movement within 1 second
	}else{
		enc->speed = SPEED_CONSTANT/diff; //[cm/s]
	}
	encoders[enc->id].last_tc = tc;
	enc->distance += PULSE_DISTANCE;
}

void EncoderOdomToZero(optEnc_t *enc){
	enc->distance = 0;
}


uint32_t EncoderGetDistance(optEnc_t *enc){
	return enc->distance;
}

float EncoderGetSpeed(optEnc_t *enc){
	return enc->speed;
}


static void EncoderTimerRoll(){



	for(uint8_t i=0; i<NOF_ENCODERS;i++){
		if(encoders[i].timer_roll == TRUE){

			if(encoders[i].enc!=NULL)
				encoders[i].enc->speed=0;
			encoders[i].timer_roll = FALSE;
		}else{
			encoders[i].timer_roll = TRUE;
		}
	}
}

