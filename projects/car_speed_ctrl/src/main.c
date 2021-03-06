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

/** @file main.c
 **
 ** @brief Aplicación que controla un robot difirencial (dos ruedas motorizadas + rueda giratoria)
 **  que sigue intrucciones precargadas.
 **
 ** La aplicación realiza un control retroalimentado de velocidad de cada rueda. La velocidad se
 ** determina midiendo el tiempo entre dos interrupciones del encoder en un GPIO. La potencia entregada
 ** a los motores se regula a través de una señal de PWM generada por hardware (SCTimer).
 ** 
 ** | RV | YYYY.MM.DD | Autor       	| Descripción de los cambios              |
 ** |----|------------|-----------------|-----------------------------------------|
 ** |  1 | 2019.06.04 | botello-escher  | Version inicial de la aplicación        |
 ** 
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

#define PIN_INT_0 		0  	/*!< Interrupción encoder derecho*/
#define PIN_INT_1 		1  	/*!< Interrupción encoder izquierdo*/
#define TIMER0 			0 	/*!< Timer para encoders*/


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
	uint16_t angle; //TODO: funcion de conversión ángulo -> distancia
}instruction_t;



/* === Declaraciones de funciones internas ================================= */

/* === Definiciones de variables internas ================================== */

motor_t motL; 	/* Left Motor	*/
motor_t motR;   /* Right Motor	*/
optEnc_t encL; 	/* Left Encoder */
optEnc_t encR;	/* Right Encoder*/
wheel_t wheelL; /* Left Motor + Left Encoder	*/
wheel_t wheelR; /* Right Motor + Right Encoder	*/

EventGroupHandle_t carEvents;

#define TURN_SPEED 15
#define LINEAR_SPEED 25

instruction_t instructionList[]={
	{.command=ADELANTE	, .speed=LINEAR_SPEED	, .distance=50	, .angle=0},
	{.command=GIRO_H	, .speed=TURN_SPEED		, .distance=9	, .angle=0},
	{.command=ADELANTE	, .speed=LINEAR_SPEED	, .distance=50	, .angle=0},
	{.command=GIRO_H	, .speed=TURN_SPEED		, .distance=9	, .angle=0},
	{.command=ADELANTE	, .speed=LINEAR_SPEED	, .distance=50	, .angle=0},
	{.command=GIRO_H	, .speed=TURN_SPEED		, .distance=9	, .angle=0},
	{.command=ADELANTE	, .speed=LINEAR_SPEED	, .distance=50	, .angle=0},
	{.command=GIRO_H	, .speed=TURN_SPEED		, .distance=9	, .angle=0},


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

		}else if((evBits & (EV_STOP|EV_RESET)) != 0){ // STOP OR RESET
			carStop();
			xEventGroupClearBits(carEvents, FLAG_CAR_RUNNING);
			xSemaphoreGive( xSemaphoreCarAvailable );
			if((evBits & EV_RESET) != 0) // ALSO IF RESET
				instructionIndex = 0;

		}else if( ( evBits & (EV_REACHED_R|EV_REACHED_L )) != 0 ){
			if((evBits & EV_REACHED_R) != 0)
				rightWheelReachGoal = TRUE;
			else
				leftWheelReachGoal = TRUE;

			if(rightWheelReachGoal == TRUE && leftWheelReachGoal == TRUE){ // BOTH REACHED THE GOAL DISTANCE
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
				taskENTER_CRITICAL();  // NEXT LINES MUST RUN AT ONCE
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

				xSemaphoreGive( xSemaphoreL ); // NOW LEFT WHEEL HAS AN INSTRUCTION. ODO CAN START COMPARING
				xSemaphoreGive( xSemaphoreR ); // NOW RIGHT WHEEL HAS AN INSTRUCTION. ODO CAN START COMPARING
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

#define kP 0.01
#define kI 0.01
#define kD 0.01

void WheelSpeedControl(void * parametros){
	wheel_t * wheel = parametros;
	float current_speed=0,average_speed=0,pwm=0, target_speed=0,error=0,last_error=0,integral=0,derivative=0,last1=0,last2=0,last3 = 0;
	//uint8_t iLUT,bias;
	EventBits_t evBits;
	uint8_t data;
	while(1){
		taskENTER_CRITICAL();
		xEventGroupWaitBits( carEvents,FLAG_CAR_RUNNING, pdFALSE, pdFALSE, portMAX_DELAY);

		target_speed = (float) wheel->speed;
		if(target_speed==0){
			integral = 0;
		}

		current_speed = EncoderGetSpeed(wheel->encoder);
		if (current_speed>100){
			current_speed=100;
		}

		average_speed = (current_speed + last1 + last2 + last3)/4;
		last3 = last2;
		last2 = last1;
		last1 = current_speed;

		//error =  target_speed - current_speed;
		error =  target_speed - average_speed;
		integral = error + integral;
		if(integral<0){
			integral=0;
		}
		derivative = error - last_error;

		last_error = error;

		pwm = kP*error + kI*integral + kD*derivative;

		if(pwm>100){
			pwm = 100;
		}else if(pwm<0){
			pwm = 0;
		}


		MotorSet(wheel->motor,(uint8_t)pwm,wheel->direction);
		/* To make the graphics with serialplot. Use with one wheel task
		data = (uint8_t) current_speed;
		SendByte_Uart_Ftdi(&data);
		data = (uint8_t) average_speed;
		SendByte_Uart_Ftdi(&data);
		data = (uint8_t) pwm;
		SendByte_Uart_Ftdi(&data);
		*/
		taskEXIT_CRITICAL();

		vTaskDelay(20/ portTICK_PERIOD_MS);
	}
}

void WheelOdometer(void *parametros){
	wheel_t * wheel = parametros;
	EventBits_t evBits;
	while(1){

		xEventGroupWaitBits( carEvents,FLAG_CAR_RUNNING, pdFALSE, pdFALSE,portMAX_DELAY);

		if(EncoderGetDistance(wheel->encoder) > (wheel->distance_to_reach)){
				EncoderOdomToZero(wheel->encoder);
			if( xSemaphoreTake( wheel->semphr, portMAX_DELAY ) == pdTRUE ){ // ODO DISTANCE > DISTANCE_TO_REACH AND THERE IS AN INSTRUCTION TO COMPLETE
				//EncoderOdomToZero(wheel->encoder);
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
	Led_Toggle(GREEN_LED);

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(PIN_INT_0));
	NVIC_ClearPendingIRQ( PIN_INT0_IRQn);
	NVIC_EnableIRQ( PIN_INT0_IRQn);
}

/*ISR for GPIO1*/
void GPIO1_IRQHandler(){
	NVIC_DisableIRQ( PIN_INT1_IRQn);

	EncoderUpdate(&encL);
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
   xTaskCreate(Director, "MainDirector", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2,NULL);
   xTaskCreate(WheelSpeedControl, "SpeedCtrlR", configMINIMAL_STACK_SIZE, &wheelR, tskIDLE_PRIORITY + 3, NULL);
   xTaskCreate(WheelSpeedControl, "SpeedCtrlL", configMINIMAL_STACK_SIZE, &wheelL, tskIDLE_PRIORITY + 3, NULL);
   xTaskCreate(WheelOdometer, "OdometerR", configMINIMAL_STACK_SIZE, &wheelR, tskIDLE_PRIORITY + 2, NULL);
   xTaskCreate(WheelOdometer, "OdometerL", configMINIMAL_STACK_SIZE, &wheelL, tskIDLE_PRIORITY + 2, NULL );


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
