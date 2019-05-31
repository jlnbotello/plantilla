/*
 * pwm.c
 *
 *  Created on: May 17, 2019
 *      Author: quack
 */


/*==================[inclusions]=============================================*/
#include "../inc/sct_hal.h"
#include "chip.h"

/*==================[macros and definitions]=================================*/

/**
 * @brief Internal typedef for SCT pin configuration
 */
typedef struct {
	uint8_t port; 	/**< Hardware Port */
	uint8_t pin;  	/**< Hardware Pin */
	uint16_t func;	/**< Pin Function */
	uint8_t ch ;	/**< SCT timer channel*/
	uint8_t out;	/**< SCT out number*/
}stcCfg_t;

/*==================[internal data definition]===============================*/

static const stcCfg_t sct[] ={		/* Silkscreen	| Function 	*/
	{0x04, 1,SCU_MODE_FUNC1,1,1}, 	/* 	T_FIL1 		| CTOUT_1	*/
	{0x04, 2,SCU_MODE_FUNC1,2,0}, 	/* 	T_FIL2 		| CTOUT_0 	*/
	{0x07, 4,SCU_MODE_FUNC1,3,13}, 	/* 	T_COL1 		| CTOUT_13 	*/
	{0x07, 5,SCU_MODE_FUNC1,4,12},	/* 	T_COL2 		| CTOUT_12 	*/
};

/**
 * @brief Initializes the SCT(State Configurable Timer) peripheral
 *
 * @param[in] frequency	Frequency of all PWM channels
 */
void SctInit(uint16_t frequency){

	Chip_SCTPWM_Init(LPC_SCT);
	Chip_SCTPWM_SetRate(LPC_SCT, frequency);
}

/**
* @brief Maps the timer to the outputs
*
* @param[in] pin	Pointer to pin configuration
*/
void SctConfig(sctPin_t * pin){

	Chip_SCU_PinMux(sct[pin->id].port,sct[pin->id].pin, SCU_MODE_INACT, sct[pin->id].func);
	Chip_SCTPWM_SetOutPin(LPC_SCT, sct[pin->id].ch , sct[pin->id].out);
	pin->duty_cycle = 0;
	Chip_SCTPWM_SetDutyCycle(LPC_SCT,sct[pin->id].ch, Chip_SCTPWM_PercentageToTicks(LPC_SCT, 0));
}

/**
 * @brief Starts the timer
 */
void SctStart(){
	Chip_SCTPWM_Start(LPC_SCT);
}

/**
 * @brief Stops the timer
 */
void SctStop(){
	Chip_SCTPWM_Stop(LPC_SCT);
}

/**
* @param[in] pin		Pointer to pin configuration
* @param[in] duty_cycle	Duty cycle
*/
void SctSetDutyCycle(sctPin_t * pin, uint8_t duty_cycle){
	Chip_SCTPWM_SetDutyCycle(LPC_SCT,sct[pin->id].ch , Chip_SCTPWM_PercentageToTicks(LPC_SCT, duty_cycle));
	pin->duty_cycle = duty_cycle;
}



