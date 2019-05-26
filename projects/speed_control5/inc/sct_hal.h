/*
 * sct_hal.h
 *
 *  Created on: May 20, 2019
 *      Author: quack
 */

#ifndef PROJECTS_SPEED_CONTROL_SCT_HAL_H_
#define PROJECTS_SPEED_CONTROL_SCT_HAL_H_

#include "stdint.h"

typedef enum {
	T_FIL1,
	T_FIL2,
	T_COL1,
	T_COL2
}sctPinId;

/**
 * @brief SCT pin configuration structure
 *
 */
typedef struct {
	sctPinId id; 		/**< Silkscreen on EDU-CIAA board*/
	uint8_t duty_cycle;	/**< Current duty cycle of channel */
} sctPin_t;

/**
 * @brief Initializes the SCT(State Configurable Timer) peripheral
 *
 * @param[in] frequency	Frequency of all PWM channels
 */
void SctInit(uint16_t frequency);

/**
* @brief Maps the timer to the outputs
*
* @param[in] pin	Pointer to pin configuration
*/
void SctConfig(sctPin_t * pin);

/**
 * @brief Starts the timer
 */
void SctStart();

/**
 * @brief Stops the timer
 */
void SctStop();

/**
* @param[in] pin		Pointer to pin configuration
* @param[in] duty_cycle	Duty cycle
*/
void SctSetDutyCycle(sctPin_t * pin, uint8_t duty_cycle);

#endif /* PROJECTS_SPEED_CONTROL_SCT_HAL_H_ */
