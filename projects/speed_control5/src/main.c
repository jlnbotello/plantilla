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
#include "optical_enc.h"
#include "event_groups.h"


/* === Definicion y Macros ================================================= */

#define PIN_INT_0 0
#define PIN_INT_1 1
#define TIMER0 0
#define NUMERO_INSTRUCCIONES 	10
#define K 						1

#define EV_ODO_RST_R 			(1<<0)
#define EV_ODO_RST_L 			(1<<1)
#define EV_ODO_OBJ_R  			(1<<2)
#define EV_ODO_OBJ_L  			(1<<3)
#define EV_RUN_L			    (1<<4)
#define EV_RUN_R			    (1<<5)
#define EV_STP_L			    (1<<6)
#define EV_STP_R			    (1<<7)
#define EV_START				(1<<8)

/* === Declaraciones de tipos de datos internos ============================ */

typedef struct {
	optEnc_t *enc;
	motor_t *mot;
	uint8_t ref_spd;
	dir_t dir;
}wheel_t;

typedef struct {
	uint32_t rst_ev;
	uint32_t obj_ev;
	uint32_t obj;
	optEnc_t *enc;
}odo_t;

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

/* === Definiciones de variables internas ================================== */

motor_t motFL; /* Front Left Motor	*/
motor_t motFR; /* Front Right Motor	*/
motor_t motBL; /* Back Left Motor 	*/
motor_t motBR; /* Back Right Motor 	*/
optEnc_t encBL;/* Back Left Encoder */
optEnc_t encBR;/* Back Right Encoder*/

uint8_t pwmBiasLut[] = {
		12,  /* 0 */
		13,  /* 1 */
		16,  /* 2 */
		19,  /* 3 */
	    24,	 /* 4 */
	    34,	 /* 5 */
	    45,  /* 6 */
		70,  /* 7 */
		100,  /* 8 */
		100   /* 9 */
};

EventGroupHandle_t carEvents;

wheel_t wheelFL;
wheel_t wheelFR;
wheel_t wheelBL;
wheel_t wheelBR;

instruction_t instructionList[NUMERO_INSTRUCCIONES]={

	//BACKWARD,CLOCKWISE,ANTICLOCKWISE
	//cmd, spped , distance ,angle
	{.command=ADELANTE, .speed=40, .distance=50,.angle=0},
	{.command=GIRO_H, .speed=40, .distance=20,.angle=0},
	{.command=ADELANTE, .speed=40, .distance=50,.angle=0},
	{.command=GIRO_H, .speed=40, .distance=20,.angle=0},
	{.command=ADELANTE, .speed=40, .distance=50,.angle=0},
	{.command=GIRO_H, .speed=40, .distance=20,.angle=0},
	{.command=ADELANTE, .speed=40, .distance=50,.angle=0}

};

uint8_t instructionIndex = 0;

odo_t rightOdo = {
		.enc = &encBR,
		.rst_ev = EV_ODO_RST_R,
		.obj_ev = EV_STP_R
};
odo_t leftOdo = {
		.enc = &encBL,
		.rst_ev = EV_ODO_RST_L,
		.obj_ev = EV_STP_L
};

TaskHandle_t hMainDirector;


/* === Definiciones de variables externas ================================== */

/* === Definiciones de funciones internas ================================== */

void setOdo(odo_t * odo,uint32_t distance){
	odo->obj = distance;

}


void rstOdo(odo_t * odo){
	EncoderOdomToZero(odo->enc);
}

void carForward (uint8_t spd){
	wheelFR.dir = wheelBR.dir = FORWARD;
	wheelFL.dir  = wheelBL.dir = FORWARD;
	wheelFL.ref_spd  = wheelFR.ref_spd  = wheelBL.ref_spd  = wheelBR.ref_spd = spd;
}

void carBackward (uint8_t spd){
	wheelFR.dir  = wheelBR.dir = BACKWARD;
	wheelFL.dir  = wheelBL.dir = BACKWARD;
	wheelFL.ref_spd  = wheelFR.ref_spd  = wheelBL.ref_spd  = wheelBR.ref_spd = spd;
}

void carRotateCW(uint8_t spd){
	wheelFR.dir  = wheelBR.dir = BACKWARD;
	wheelFL.dir  = wheelBL.dir = FORWARD;
	wheelFL.ref_spd  = wheelFR.ref_spd  = wheelBL.ref_spd  = wheelBR.ref_spd = spd;
}

void carRotateACW(uint8_t spd){
	wheelFR.dir  = wheelBR.dir = FORWARD;
	wheelFL.dir  = wheelBL.dir = BACKWARD;
	wheelFL.ref_spd  = wheelFR.ref_spd  = wheelBL.ref_spd  = wheelBR.ref_spd = spd;
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
	uint8_t speed = 0;
	while(1){

		tecla = Read_Switches();
		if (tecla != anterior) {
			switch (tecla) {
			case TEC1:
				//Damos inicio a la ejecucion de instrucciones
				xEventGroupSetBits(carEvents,EV_STP_L|EV_STP_R);
				xEventGroupSetBits(carEvents,EV_START);
				//carForward(40);
				break;
			case TEC2:
				//Paramos las instrucciones
				xEventGroupClearBits(carEvents,EV_START);
				//carForward(0);
				break;
			case TEC3:
				//Se detienen las instrucciones y se inicia el index del contador
				//xEventGroupSetBits(carEvents,EV_RESET);
				instructionIndex=0;
				//carBackward(40);
				break;
			case TEC4:
				carBackward(0);
				break;
			default:
				break;
			}
			anterior = tecla;
		}
		vTaskDelay( 20/ portTICK_PERIOD_MS);
	}
}



void WheelCtrl(void * parametros){
	wheel_t * wheel = parametros;
	int8_t cur_spd,last1,last2,last3 = 0;
	uint8_t iLUT,pwm,bias;
	EventBits_t evBits;
	while(1){

		evBits = xEventGroupWaitBits( carEvents,EV_START, pdFALSE, pdFALSE, 10/ portTICK_PERIOD_MS);
		if( ( evBits & EV_START ) != 0 ){
			last3 = last2;
			last2 = last1;
			last1 = EncoderGetSpeed(wheel->enc);
			cur_spd = (last1 + last2 + last3)/3;


			if(wheel->ref_spd > 0 && wheel->ref_spd < 100){
				iLUT = (uint8_t) wheel->ref_spd/10;
				bias  = pwmBiasLut [iLUT];
			}else{
				bias = 0;
			}

			pwm = (uint8_t) bias + ((float)(K*(wheel->ref_spd-cur_spd)));

			if(pwm>100){
				pwm = 100;
			}else if(pwm<0){
				pwm = 0;
			}

			MotorSet(wheel->mot,pwm,wheel->dir);
		}else{
			MotorSet(wheel->mot,0,wheel->dir);
		}
		//vTaskDelay( 20/ portTICK_PERIOD_MS);
	}
}

void Odometer(void *parametros){
	odo_t * odo = parametros;
	EventBits_t evBits;
	while(1){
		evBits = xEventGroupWaitBits( carEvents, odo->rst_ev, pdTRUE, pdFALSE, 100/ portTICK_PERIOD_MS);
		if( ( evBits & odo->rst_ev ) != 0 ){
			EncoderOdomToZero(odo->enc);
		}else{
			if(EncoderGetDistance(odo->enc) > odo->obj){
				xEventGroupSetBits(carEvents,EV_STP_L|EV_STP_R);
			}
		}
	}
}


void Director(void *parametros){
	EventBits_t evBits;
	while(1){
		//Espera el bit que setea TECLA1
		evBits = xEventGroupWaitBits( carEvents, EV_START, pdFALSE, pdFALSE, portMAX_DELAY);//

		//if(evBits & EV_START !=0){


		xEventGroupWaitBits( carEvents, EV_STP_L|EV_STP_R, pdTRUE, pdTRUE, portMAX_DELAY);//



		command_t cmdActual=instructionList[instructionIndex].command;
		uint8_t speedActual=instructionList[instructionIndex].speed;
		uint32_t distanciaActual=instructionList[instructionIndex].distance;
		uint16_t angleActual=instructionList[instructionIndex].angle;
		rstOdo(&leftOdo);
		rstOdo(&rightOdo);
		setOdo(&leftOdo,distanciaActual);
		setOdo(&rightOdo,distanciaActual);

		if(instructionIndex<NUMERO_INSTRUCCIONES)instructionIndex++;

		switch( cmdActual )
		{
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
		}
		//}
		//else{
			//carForward (0);
		//}

		}


}


/* === Definiciones de funciones externas ================================== */

/*ISR for GPIO0*/
void GPIO0_IRQHandler(){
	NVIC_DisableIRQ( PIN_INT0_IRQn);
	EncoderUpdate(&encBR);

	/*uint8_t data[5];
	uint32ToASCII((uint32_t)encBR.distance,data);
	SendString_Uart_Ftdi("Distance: ");
	SendString_Uart_Ftdi(data);
	SendString_Uart_Ftdi(" cm | Speed: ");
	uint32ToASCII((uint32_t)encBR.speed,data);
	SendString_Uart_Ftdi(data);
	SendString_Uart_Ftdi(" cm/s | Set speed: ");
	uint32ToASCII((uint32_t)ref_r_spd,data);
	SendString_Uart_Ftdi(data);
	SendString_Uart_Ftdi(" cm/s \n");
*/

	Led_Toggle(GREEN_LED);;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(PIN_INT_0));
	NVIC_ClearPendingIRQ( PIN_INT0_IRQn);
	NVIC_EnableIRQ( PIN_INT0_IRQn);
}


void GPIO1_IRQHandler(){
	NVIC_DisableIRQ( PIN_INT1_IRQn);
	EncoderUpdate(&encBL);
	Led_Toggle(RGB_B_LED);;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(PIN_INT_1));
	NVIC_ClearPendingIRQ( PIN_INT1_IRQn);
	NVIC_EnableIRQ( PIN_INT1_IRQn);
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
   EncoderInit();
   MotorInit();

   MotorCfg(&motFR,T_FIL1,GPIO_4);
   MotorCfg(&motFL,T_FIL2,GPIO_5);
   MotorCfg(&motBR,T_COL1,GPIO_6);
   MotorCfg(&motBL,T_COL2,GPIO_7);
   EncoderCfg(&encBR,ENC_1,GPIO_0,PIN_INT_0);
   EncoderCfg(&encBL,ENC_2,GPIO_1,PIN_INT_1);

   wheelFR.enc = &encBR;
   wheelFL.enc = &encBL;
   wheelBR.enc = &encBR;
   wheelBL.enc = &encBL;

   wheelFR.mot = &motFR;
   wheelFL.mot = &motFL;
   wheelBR.mot = &motBR;
   wheelBL.mot = &motBL;

   carEvents = xEventGroupCreate();

   /* Creación de las tareas */

   xTaskCreate(Blinking, "Verde", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(Teclado, "Teclado", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(WheelCtrl, "FR Wheel", configMINIMAL_STACK_SIZE, &wheelFR, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(WheelCtrl, "FL Wheel", configMINIMAL_STACK_SIZE, &wheelFL, tskIDLE_PRIORITY +1, NULL);
   xTaskCreate(WheelCtrl, "BR Wheel", configMINIMAL_STACK_SIZE, &wheelBR, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(WheelCtrl, "BL Wheel", configMINIMAL_STACK_SIZE, &wheelBL, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(Director, "MainDirector", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1,NULL);
   xTaskCreate(Odometer, "OdometerR", configMINIMAL_STACK_SIZE, &rightOdo, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(Odometer, "OdometerL", configMINIMAL_STACK_SIZE, &leftOdo, tskIDLE_PRIORITY + 1, NULL );


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
