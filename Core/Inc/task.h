/*
 * task.h
 *
 *  Created on: 2 nov. 2020
 *      Author: nicoc
 */


#ifndef INC_TASK_H_
#define INC_TASK_H_

#include "main.h"

#define MODE_KEY			GPIOB, inputMode_Pin
#define HOLD_KEY			GPIOB, inputHold_Pin
#define ZERO_KEY			GPIOB, inputZero_Pin
#define	LED_ALARMA			GPIOB, outputLed_Pin

#define	NO_KEY				0
#define MODE				1
#define ZERO				2
#define HOLD				3
#define REBOTES_MIN			2

#define MPU6050_ADDR 		0xD0
#define MAX_LEN_TASK_LIST	(7)
#define TICK_SISTEMA		(10)
#define PROMEDIOS			25		//valor anterior: 50

//Display
#define POS_GRADOS			10,30	//30,30
#define POS_TITULO			5,0
#define POS_LINEA1			20,15
#define POS_LINEA2			20,40
#define POS_GRADOSA			20,40

//ESTADOS - MODOS
#define CANT_MODOS			2
#define MODO_MEDIR			0
#define MODO_CAL			1

//CALIBRACION
#define TOTAL_CAL_POINTS	19		//voy de -90 a +90 con saltos de a 10

#define ON					1
#define OFF					0

#define	TRUE	1
#define	FALSE	0

#define THRESHOLD			0.01	//valor de td2: 0.13
#define	UMBRAL_ALARMA		0.5

/*
 * Prototipos de las funciones que hacen al sistema.
 */

void tarea_iwdg(void *p);
void tarea_led_blinking(void *p);
void tarea_display(void *p);
void tarea_orienta(void *p);
void tarea_refresh(void *p);
void tarea_pulsadores(void *p);
void tarea_modos(void *p);
//void tarea_alarmas(void *p);

/*
 * Funciones necesarias para algunas tareas
 */

uint8_t antirebote (uint8_t );
void lastAlarm(char *);
uint8_t alarmCheck (float );
double corregir (double );
void tareas_Init();

#endif /* INC_TASK_H_ */
