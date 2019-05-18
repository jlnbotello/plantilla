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
//#include "unt.h"
#include "soc.h"
#include "led.h"
#include "switch.h"
#include "sct_hal.h"
#include "gpio_hal.h"
#include "textUtils.h"
#include "uart.h"
#include "sapi_timer.h"

/* === Definicion y Macros ================================================= */

#define PIN_INT_0 0
#define PIN_INT_1 1
#define TIMER0 0
sctPin_t motFL; /* Front Left Motor	*/
sctPin_t motFR; /* Front Right Motor	*/
sctPin_t motBL; /* Back Left Motor 	*/
sctPin_t motBR; /* Back Right Motor 	*/


/* === Declaraciones de tipos de datos internos ============================ */

/* === Declaraciones de funciones internas ================================= */

/** @brief Función que implementa una tarea de baliza
 ** 
 ** @parameter[in] parametros Puntero a una estructura que contiene el led
 **                           y la demora entre encendido y apagado.
 */ 
void Blinking(void * parametros);

/* === Definiciones de variables internas ================================== */

/* === Definiciones de variables externas ================================== */

/* === Definiciones de funciones internas ================================== */

void Blinking(void * parametros) {
   while(1) {
      Led_On(GREEN_LED);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      Led_Off(GREEN_LED);
      vTaskDelay(500 / portTICK_PERIOD_MS);
   }
}

void Teclado(void * parametros){
	uint8_t tecla;
	uint8_t anterior = 0;
	uint8_t dc1,dc2,dc3,dc4 = 0;

	while(1){

		tecla = Read_Switches();
		if (tecla != anterior) {
			switch (tecla) {
			case TEC1:
				dc2 = 0;
				SctSetDutyCycle(&motBR, dc2);
				break;
			case TEC2:
				if(dc2<100){
					dc2+=10;
				} else{
					dc2 = 0;
				}
				SctSetDutyCycle(&motBR, dc2);
				break;
			case TEC3:
				if(dc3 <= 100){
					dc3 += 15;
				} else{
					dc3 = 0;
				}
				SctSetDutyCycle(&motFL, dc3);
				break;
			case TEC4:
				if(dc4 <= 100){
					dc4 += 15;
				} else{
					dc4 = 0;
				}
				SctSetDutyCycle(&motFR, dc4);
				break;
			default:
				break;
			}
			anterior = tecla;
		}
		vTaskDelay( 20/ portTICK_PERIOD_MS);
	}
}

/* === Definiciones de funciones externas ================================== */

/** @brief Función principal del programa
 **
 ** @returns 0 La función nunca debería termina
 **
 ** @remarks En un sistema embebido la función main() nunca debe terminar.
 **          El valor de retorno 0 es para evitar un error en el compilador.
 */
uint8_t pulsos = 0;
uint8_t vueltas = 0;
uint32_t speed = 0;

/*ISR for GPIO0*/
void GPIO0_IRQHandler(){
	NVIC_DisableIRQ( PIN_INT0_IRQn);

	Led_Toggle(RGB_B_LED);
	pulsos ++;
	uint32_t counts = Timer_ReadCount(0);
	Timer_Reset(0);

	if(pulsos == 20){
		pulsos=0;
		vueltas++;
		if(counts>0)
		speed = 224400000/counts;
		uint8_t data[5];
		uint32ToASCII(speed,data);
		SendString_Uart_Ftdi(data);
		SendString_Uart_Ftdi(" cm/s \n");
		Led_Toggle(RED_LED);
	}




	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(PIN_INT_0));
	NVIC_ClearPendingIRQ( PIN_INT0_IRQn);
	NVIC_EnableIRQ( PIN_INT0_IRQn);
}

void CallbackTimer(){
	speed=0; // si en 500ms no hubo una deteccion del detector la velocidad es cero
	Led_Toggle(YELLOW_LED);
}

int main(void) {
   /* Inicializaciones y configuraciones de dispositivos */

   Init_Leds();
   Init_Switches();
   SctInit(100); // 100 Hz
   GpioInit();
   Init_Uart_Ftdi();
   Timer_Init(0, 102000000, CallbackTimer);




   motFR.id = T_FIL1;
   motFL.id = T_FIL2;
   motBR.id = T_COL1;
   motBL.id = T_COL2;

   SctConfig(&motFR);
   SctConfig(&motFL);
   SctConfig(&motBR);
   SctConfig(&motBL);

   SctStart();


	/* Falling Edge Sensitive */
	gpioPin_t gpio0;
	gpio0.n=GPIO_0;
	gpio0.dir=GPIO_IN_PULLUP;
	GpioConfig(&gpio0);
	GpioInterruptConfig(&gpio0,GPIO_IRQ_EDGE_FALL,PIN_INT_0);





   /* Creación de las tareas */

   xTaskCreate(Blinking, "Verde", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(Teclado, "Teclado", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

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
