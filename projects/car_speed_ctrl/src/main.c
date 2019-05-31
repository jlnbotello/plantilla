/* Copyright 2017, Esteban Volentini - Facet UNT, Fi UNER
 * Copyright 2014, 2015 Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** @file blinking.c
 **
 ** @brief Ejemplo de un led parpadeando
 **
 ** Ejemplo de un led parpadeando utilizando la capa de abstraccion de 
 ** hardware y sin sistemas operativos.
 ** 
 ** | RV | YYYY.MM.DD | Autor       | Descripción de los cambios              |
 ** |----|------------|-------------|-----------------------------------------|
 ** |  2 | 2017.10.16 | evolentini  | Correción en el formato del archivo     |
 ** |  1 | 2017.09.21 | evolentini  | Version inicial del archivo             |
 ** 
 ** @defgroup ejemplos Proyectos de ejemplo
 ** @brief Proyectos de ejemplo de la Especialización en Sistemas Embebidos
 ** @{ 
 */

/* === Inclusiones de cabeceras ============================================ */

#include "optical_enc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "soc.h"
#include "led.h"
#include "switch.h"
#include "gpio_hal.h"
#include "textUtils.h"
#include "uart.h"
#include "sapi_timer.h"
#include "motor_ctrl.h"
#include "event_groups.h"
#include "semphr.h"


/* === Definicion y Macros ================================================= */

#define PIN_INT_0 0
#define PIN_INT_1 1
#define TIMER0 0
//#define NUMERO_INSTRUCCIONES 	10
#define K 						1

#define EV_REACHED_R			(1<<0)
#define EV_REACHED_L 			(1<<1)
#define EV_START				(1<<2) // USER EVENT
#define EV_STOP					(1<<3) // USER EVENT
#define EV_RESET				(1<<4) // USER EVENT
#define EV_NEXT_INSTRUCTION	    (1<<5)
#define FLAG_CAR_RUNNING	    (1<<6)


/* === Declaraciones de tipos de datos internos ============================ */

typedef struct {
	optEnc_t *encoder;
	motor_t *motor;
	uint8_t speed;
	uint32_t distance_to_reach;
	dir_t direction;
	uint32_t ev_rst_odo;
	uint32_t ev_distance_reached;
	SemaphoreHandle_t semphr;
}wheel_t;

typedef enum{
	ADELANTE,ATRAS,GIRO_H,GIRO_AH
}command_t;

typedef struct {
	command_t command;
	uint8_t speed;
	uint32_t distance;
	uint16_t angle;

}instruction_t;



/* === Declaraciones de funciones internas ================================= */

/** @brief Función que implementa una tarea de baliza
 **
 ** @parameter[in] parametros Puntero a una estructura que contiene el led
 **                           y la demora entre encendido y apagado.
 */
void Blinking(void * parametros);



/***COMPLETAR TODO***/




/* === Definiciones de variables internas ================================== */

motor_t motL; 	/* Left Motor	*/
motor_t motR;   /* Right Motor	*/
optEnc_t encL; 	/* Left Encoder */
optEnc_t encR;	/* Right Encoder*/
wheel_t wheelL; /* Left Motor + Left Encoder	*/
wheel_t wheelR; /* Right Motor + Right Encoder	*/

EventGroupHandle_t carEvents;

uint8_t pwmBiasLUT[] = {
		13,  /* 0 */
		14,  /* 1 */
		16,  /* 2 */
		19,  /* 3 */
	    24,	 /* 4 */
	    34,	 /* 5 */
	    45,  /* 6 */
		70,  /* 7 */
		100,  /* 8 */
		100   /* 9 */
};



#define TURN_SPEED 20
#define LINEAR_SPEED 70

instruction_t instructionList[]={
	{.command=ADELANTE	, .speed=LINEAR_SPEED	, .distance=50	, .angle=0},
	{.command=GIRO_H	, .speed=TURN_SPEED		, .distance=12	, .angle=0},
	{.command=ADELANTE	, .speed=LINEAR_SPEED	, .distance=50	, .angle=0},
	{.command=GIRO_H	, .speed=TURN_SPEED		, .distance=12	, .angle=0},
	{.command=ADELANTE	, .speed=LINEAR_SPEED	, .distance=50	, .angle=0},
	{.command=GIRO_H	, .speed=TURN_SPEED		, .distance=12	, .angle=0},
	{.command=ADELANTE	, .speed=LINEAR_SPEED	, .distance=50	, .angle=0},
	{.command=GIRO_H	, .speed=TURN_SPEED		, .distance=12	, .angle=0},


};

uint8_t nOfInstruction = sizeof(instructionList)/sizeof(instruction_t);

uint8_t instructionIndex = 0;

SemaphoreHandle_t xSemaphoreL = NULL;
SemaphoreHandle_t xSemaphoreR = NULL;
SemaphoreHandle_t xSemaphoreCarAvailable = NULL;

/* === Definiciones de variables externas ================================== */

/* === Definiciones de funciones internas ================================== */

void setOdo(wheel_t * wheel,uint32_t distance){
	wheel->distance_to_reach = distance;
}

void rstOdo(wheel_t * wheel){
	EncoderOdomToZero(wheel->encoder);
}

void carForward (uint8_t spd){
	wheelL.direction = wheelR.direction = FORWARD;
	wheelL.speed = wheelR.speed= spd;
}

void carBackward (uint8_t spd){
	wheelL.direction = wheelR.direction = BACKWARD;
	wheelL.speed = wheelR.speed= spd;
}

void carRotateCW(uint8_t spd){
	wheelR.direction = BACKWARD;
	wheelL.direction = FORWARD;
	wheelR.speed = wheelL.speed= spd;
}

void carRotateACW(uint8_t spd){
	wheelR.direction = FORWARD;
	wheelL.direction = BACKWARD;
	wheelR.speed = wheelL.speed= spd;
}

void carStop(){
	wheelL.speed = wheelR.speed= 0;
	MotorRst(&motR);
	MotorRst(&motL);
}


/*-----------   TASKS ------------*/

void Blinking(void * parametros) {
   while(1) {
      Led_On(YELLOW_LED);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      Led_Off(YELLOW_LED);
      vTaskDelay(500 / portTICK_PERIOD_MS);
   }
}


void Teclado(void * parametros){
	uint8_t tecla;
	uint8_t anterior = 0;
	while(1){

		tecla = Read_Switches();
		if (tecla != anterior) {
			switch (tecla) {
			case TEC1:
				//Damos inicio a la ejecucion de instrucciones
				xEventGroupSetBits(carEvents,EV_START);
				break;
			case TEC2:
				//Paramos la ejecucion de instrucciones
				xEventGroupSetBits(carEvents,EV_STOP);
				break;
			case TEC3:
				//Paramos la ejecucion de instrucciones y se apunta la primera instruccion
				xEventGroupSetBits(carEvents,EV_RESET);
				break;
			case TEC4:

				break;
			default:
				break;
			}
			anterior = tecla;
		}
		vTaskDelay( 20/ portTICK_PERIOD_MS);
	}
}

void Director(void *parametros){
	EventBits_t evBits;
	uint8_t rightWheelReachGoal = FALSE;
	uint8_t leftWheelReachGoal = FALSE;
	uint8_t nextInstruction = FALSE;
	xSemaphoreGive( xSemaphoreCarAvailable );
	while(1){
		evBits = xEventGroupWaitBits( carEvents, EV_START|EV_STOP|EV_RESET|EV_REACHED_L|EV_REACHED_R, pdTRUE, pdFALSE, 500/ portTICK_PERIOD_MS);

		if((evBits & EV_START) != 0){
			if(xSemaphoreTake( xSemaphoreCarAvailable, portMAX_DELAY ) == pdTRUE){ // if car isn't running
				xEventGroupSetBits(carEvents, EV_NEXT_INSTRUCTION|FLAG_CAR_RUNNING);
			}

		}else if((evBits & (EV_STOP|EV_RESET)) != 0){ // OR
			carStop();
			xEventGroupClearBits(carEvents, FLAG_CAR_RUNNING);
			xSemaphoreGive( xSemaphoreCarAvailable );
			if((evBits & EV_RESET) != 0)
				instructionIndex = 0;

		}else if( ( evBits & (EV_REACHED_R|EV_REACHED_L )) != 0 ){
			if((evBits & EV_REACHED_R) != 0)
				rightWheelReachGoal = TRUE;
			else
				leftWheelReachGoal = TRUE;

			if(rightWheelReachGoal == TRUE && leftWheelReachGoal == TRUE){
				xEventGroupSetBits(carEvents, EV_NEXT_INSTRUCTION);
				leftWheelReachGoal = FALSE;
				rightWheelReachGoal = FALSE;
			}

		}else if( ( evBits & EV_NEXT_INSTRUCTION ) != 0 ){

			xEventGroupClearBits(carEvents, EV_NEXT_INSTRUCTION);

			if(instructionIndex<nOfInstruction){

				carStop();
				vTaskDelay(500/ portTICK_PERIOD_MS);
				command_t cmdActual=instructionList[instructionIndex].command;
				uint8_t speedActual=instructionList[instructionIndex].speed;
				uint32_t distanciaActual=instructionList[instructionIndex].distance;
				uint16_t angleActual=instructionList[instructionIndex].angle;
				instructionIndex++;
				taskENTER_CRITICAL();
				switch( cmdActual ){
				case ADELANTE:
					/* Se recibe el msj de sin comando. */
					carForward (speedActual);
					break;

				case ATRAS:
					/* Se llama a la función que ejecuta un giro. con el respectivo dato de velocidad*/
					carBackward(speedActual);
					break;

				case GIRO_H:
					/* Se llama a la función que ejecita un avance con la respectiva velocidad asociada*/
					carRotateCW(speedActual);
					break;
				case GIRO_AH:
					/* Se llama a la función que ejecita un avance con la respectiva velocidad asociada*/
					carRotateACW(speedActual);
					break;

				default:
					carStop();
				}


				rstOdo(&wheelR);
				rstOdo(&wheelL);

				setOdo(&wheelR,distanciaActual);
				setOdo(&wheelL,distanciaActual);

				xSemaphoreGive( xSemaphoreL );
				xSemaphoreGive( xSemaphoreR );
				taskEXIT_CRITICAL();
			}else{
				carStop();
				xEventGroupClearBits(carEvents, FLAG_CAR_RUNNING);
				xSemaphoreGive( xSemaphoreCarAvailable );
			}

		}else{
				Led_Toggle(RED_LED); // TIMEOUT: Director is alive, but doing nothing
		}
	}
}

void WheelSpeedControl(void * parametros){
	wheel_t * wheel = parametros;
	float cur_spd,last1,last2,last3 = 0;
	uint8_t iLUT,pwm,bias;
	EventBits_t evBits;
	while(1){

		xEventGroupWaitBits( carEvents,FLAG_CAR_RUNNING, pdFALSE, pdFALSE, portMAX_DELAY);

		if(wheel->speed > 0 && wheel->speed < 100){
			iLUT = (uint8_t) wheel->speed/10;
			bias  = pwmBiasLUT [iLUT];
			//average
			last3 = last2;
			last2 = last1;
			last1 = EncoderGetSpeed(wheel->encoder);
			cur_spd = (last1 + last2 + last3)/3;

			pwm = (uint8_t) (bias + ((float)(K*(wheel->speed-cur_spd))));

			if(pwm>100){
				pwm = 100;
			}else if(pwm<0){
				pwm = 0;
			}
			MotorSet(wheel->motor,pwm,wheel->direction);

		}else{
			MotorRst(wheel->motor);

		}
		vTaskDelay(20/ portTICK_PERIOD_MS);
	}
}

void WheelOdometer(void *parametros){
	wheel_t * wheel = parametros;
	EventBits_t evBits;
	while(1){

		xEventGroupWaitBits( carEvents,FLAG_CAR_RUNNING, pdFALSE, pdFALSE,portMAX_DELAY);

		if(EncoderGetDistance(wheel->encoder) > (wheel->distance_to_reach)){
			if( xSemaphoreTake( wheel->semphr, portMAX_DELAY ) == pdTRUE ){
				EncoderOdomToZero(wheel->encoder);
				xEventGroupSetBits(carEvents,wheel->ev_distance_reached);
				MotorRst(wheel->motor);
				wheel->speed = 0;
			}
		}
		vTaskDelay(20/ portTICK_PERIOD_MS);
	}
}





/* === Definiciones de funciones externas ================================== */

/*ISR for GPIO0*/
void GPIO0_IRQHandler(){
	NVIC_DisableIRQ( PIN_INT0_IRQn);
	EncoderUpdate(&encR);
/*
	uint8_t data[5];
	uint32ToASCII((uint32_t)encR.distance,data);
	SendString_Uart_Ftdi("Distance R: ");
	SendString_Uart_Ftdi(data);
	SendString_Uart_Ftdi(" cm | Speed R: ");
	uint32ToASCII((uint32_t)encR.speed,data);
	SendString_Uart_Ftdi(data);
	SendString_Uart_Ftdi(" cm/s | Set speed R: ");
	uint32ToASCII((uint32_t)wheelR.speed,data);
	SendString_Uart_Ftdi(data);
	SendString_Uart_Ftdi(" cm/s \n");
*/
	Led_Toggle(GREEN_LED);

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(PIN_INT_0));
	NVIC_ClearPendingIRQ( PIN_INT0_IRQn);
	NVIC_EnableIRQ( PIN_INT0_IRQn);
}


void GPIO1_IRQHandler(){
	NVIC_DisableIRQ( PIN_INT1_IRQn);
	EncoderUpdate(&encL);
/*
	uint8_t data[5];
	uint32ToASCII((uint32_t)encL.distance,data);
	SendString_Uart_Ftdi("Distance L: ");
	SendString_Uart_Ftdi(data);
	SendString_Uart_Ftdi(" cm | Speed L: ");
	uint32ToASCII((uint32_t)encL.speed,data);
	SendString_Uart_Ftdi(data);
	SendString_Uart_Ftdi(" cm/s | Set speed L: ");
	uint32ToASCII((uint32_t)wheelL.speed,data);
	SendString_Uart_Ftdi(data);
	SendString_Uart_Ftdi(" cm/s \n");
/**/
	Led_Toggle(RGB_B_LED);

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(PIN_INT_1));
	NVIC_ClearPendingIRQ( PIN_INT1_IRQn);
	NVIC_EnableIRQ( PIN_INT1_IRQn);
}

void WheelInit(wheel_t * wheel, optEnc_t *encoder, motor_t *motor, uint32_t ev_dist_reached , SemaphoreHandle_t sem){

	 wheel->encoder = encoder;
	 wheel->motor =  motor;
	 wheel->ev_distance_reached = ev_dist_reached;
	 wheel->semphr = sem;
	 wheel->direction = FORWARD;
	 wheel->speed = 0;
	 wheel->distance_to_reach = 0;
}












/** @brief Función principal del programa
 **
 ** @returns 0 La función nunca debería termina
 **
 ** @remarks En un sistema embebido la función main() nunca debe terminar.
 **          El valor de retorno 0 es para evitar un error en el compilador.
 */

int main(void) {
   /* Inicializaciones y configuraciones de dispositivos */

   Init_Leds();
   Init_Switches();
   SctInit(100); // 100 Hz
   GpioInit();
   Init_Uart_Ftdi();

   MotorInit();
   MotorCfg(&motR,T_FIL1,GPIO_3);
   MotorCfg(&motL,T_FIL2,GPIO_4);

   EncoderInit();
   EncoderCfg(&encR,ENC_1,GPIO_1,PIN_INT_0);
   EncoderCfg(&encL,ENC_2,GPIO_2,PIN_INT_1);



   carEvents = xEventGroupCreate();
   xSemaphoreL = xSemaphoreCreateBinary();
   xSemaphoreR = xSemaphoreCreateBinary();
   xSemaphoreCarAvailable = xSemaphoreCreateBinary();

   WheelInit(&wheelR, &encR, &motR,EV_REACHED_R,xSemaphoreR);
   WheelInit(&wheelL, &encL, &motL,EV_REACHED_L,xSemaphoreL);



   /* Creación de las tareas */

   xTaskCreate(Blinking, "Verde", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(Teclado, "Teclado", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(Director, "MainDirector", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1,NULL);
   xTaskCreate(WheelSpeedControl, "SpeedCtrlR", configMINIMAL_STACK_SIZE, &wheelR, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(WheelSpeedControl, "SpeedCtrlL", configMINIMAL_STACK_SIZE, &wheelL, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(WheelOdometer, "OdometerR", configMINIMAL_STACK_SIZE, &wheelR, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(WheelOdometer, "OdometerL", configMINIMAL_STACK_SIZE, &wheelL, tskIDLE_PRIORITY + 1, NULL );


   SisTick_Init();
   /* Arranque del sistema operativo */
   vTaskStartScheduler();
   /* Arranque del systick*/

   /* vTaskStartScheduler solo retorna si se detiene el sistema operativo */
   while(1);

   /* El valor de retorno es solo para evitar errores en el compilador*/
   return 0;
}
/* === Ciere de documentacion ============================================== */
/** @} Final de la definición del modulo para doxygen */
