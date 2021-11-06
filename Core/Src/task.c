/*
 * task.c
 *
 *  Created on: 2 nov. 2020
 *      Author: nicoc
 *  Version 2021: fcascan, oct 2021
 */

#include "task.h"
#include "mpu_6050.h"
#include "ssd1306.h"
#include "fonts.h"
#include "stdio.h"
#include <math.h>
#include "EEPROM.h"

//Variables globales:
extern I2C_HandleTypeDef hi2c1;
MPU6050_t MPU6050;
double lectura;				//Valor de medicion actual del modulo sin la correccion
double medicion;			//Valor de medicion una vez aplicada la correccion
uint8_t modo = MODO_MEDIR;
float offset = 0;
float valuetoSave = 0;
uint8_t keyboard_buffer = NO_KEY;
//float customAlarms[CANT_ALARMAS] = {30,45,90,EEPROM_EMPTY,EEPROM_EMPTY,EEPROM_EMPTY,EEPROM_EMPTY};
//float customAlarms[CANT_ALARMAS] = {-90.0,-85.0,-80.0,-75.0,-70.0,-65.0,-60.0,-55.0,-50.0,-45.0,-40.0,
//									-35.0,-30.0,-25.0,-20.0,-15.0,-10.0,-5.0,0,5,10,15,20,25,30,35,40,45,
//									50,55,60,65,70,75,80,85,90};

//Flags globales:
uint8_t f_hold = OFF;		//Para freezar el display cuando pulso HOLD
uint8_t f_zero = OFF;		//
uint8_t f_2lineas = OFF;	//Para diferenciar cuando visualizar en 1 o 2 lineas del display
uint8_t f_alarmas = OFF;	//Para dar aviso de si tener en cuenta el LED o no en el modo actual
uint8_t f_default = OFF;	//Flag para dar reset a toda la calibracion cargada en la EEPROM

//Calibracion:
uint8_t indice_cal = 0;
float cal_points[TOTAL_CAL_POINTS]={-90.0,-80.0,-70.0,-60.0,-50.0,-40.0,-30.0,-20.0,-10.0,0,
									10,20,30,40,50,60,70,80,90};	//Valores a mostrar en pantalla en MODO2
float cal_offsets[TOTAL_CAL_POINTS+1]={0};	//Valores a modificar por la calibracion
float cal_default[TOTAL_CAL_POINTS+1]={0};	//Valores de calibracion default para pisar cal_offsets

////////////////////////////
//	 FUNCION ANTIREBOTE	  //
////////////////////////////
//Esta funcion se ejecuta siempre con la tarea de lectura de pulsadores, incluso si no se pulso nada.
//Se recibe la tecla actualmente leida, y retorna la letra considerada como valida segun la cant min de rebotes.
uint8_t antirebote (uint8_t teclaActual)
{
	static uint8_t cont = 0;

//	//Cada vez que entro a la funcion, y si la tecla sigue estando pulsada pero no llego al min de rebotes:
//	if(cont < REBOTES_MIN && teclaAnt == teclaActual){
//		cont++;
//		blocked = NO;
//		return NO_KEY;
//	}
//
//	//Si ya conté la cantidad de rebotes minimos:
//	else{
//		cont = 0;
//		// Si no estoy bloqueando:
//		if(teclaActual == teclaAnt && blocked == NO){
//			blocked = YES;
//			return teclaActual;
//		}
//		// Si estaba bloqueando
//		else{
//			teclaAnt = teclaActual;
//			blocked = NO;
//			return NO_KEY;
//		}
//	}
//
//	//Aca no tengo que llegar, reseteo todos los valores:
//	cont = 0;
//	teclaAnt = NO_KEY;
//	blocked = NO;
//	return NO_KEY;

	//Si todavia no se vacio el buffer de teclado, recargo el buffer con el mismo valor:
//	if(keyboard_buffer != NO_KEY){
//		return keyboard_buffer;
//	}

	//Tecla no pulsada (o porque esta rebotando):
	if(teclaActual == NO_KEY){
		cont = 0;
		return NO_KEY;
	}
	//Lecturas de tecla distintas de NO_KEY:
	//pero no llegue a la cant min de rebotes:
	if(cont < REBOTES_MIN){
		cont++;
		return keyboard_buffer;
	}
	//Tecla aceptada y cargada al buffer solo una vez:
	if(cont == REBOTES_MIN){
		cont++;
		return teclaActual;
	}
	//Quedo bloqueado hasta que deje de presionarse la tecla:
	if(cont > REBOTES_MIN){
		return keyboard_buffer;
	}
	//En teoria aca no tengo que llegar nunca:
	cont = 0;
	return keyboard_buffer;
}


////////////////////////////
//	 FUNCION LASTALARM	  //
////////////////////////////
//
void lastAlarm(char *string)
{
	float aux = 0;

	for(int i = 0; i < TOTAL_CAL_POINTS; i++){
		if(cal_offsets[i] != EEPROM_EMPTY)
			aux = cal_offsets[i];
		else
			break;
	}
	if(aux!=90){
		if(aux < 10){
			sprintf(string,"0%.2f", aux);
		}
		else{
			sprintf(string,"%.2f", aux);
		}
	}
	else{	//aux==90
		sprintf(string,"%d    ", (int)aux);
	}
}

////////////////////////////
//	 FUNCION ALARMCHECK	  //
////////////////////////////
//
uint8_t alarmCheck (float value)
{
	for(int i = 0; i < TOTAL_CAL_POINTS; i++){
		if(cal_offsets[i] >= value-UMBRAL_ALARMA	&&	cal_offsets[i] <= value+UMBRAL_ALARMA)
			return TRUE;
	}
	return FALSE;
}

////////////////////////////
//	 FUNCION CORRECCION	  //
////////////////////////////
//Corrige en forma lineal la lectura obtenida por el sensor segun los offsets cargados en la calibracion.
//Se considera que entre dos angulos de 10° hay una correccion lineal a aplicar ya que puede pasar que la
//correccion necesaria en un punto no sea la misma que en que le sigue.
//Recibe la lectura a corregir. Retorna la medicion ya corregida.
double corregir (double valor){
	float x1, x2, y1, y2, m=0, b=0;

	//Si el valor es menor a -90°: Uso directamente la correccion de -90°
	if(valor<=cal_points[0]){
		return valor + cal_offsets[0];
	}
	//Si el valor es mayor a +90°: Uso directamente la correccion de +90°
	if(cal_points[TOTAL_CAL_POINTS]<=valor){
		return valor + cal_offsets[TOTAL_CAL_POINTS-1];
	}
	//Valores intermedios, calcular la pendiente y origen de la recta
	for(int i=0; i<TOTAL_CAL_POINTS-1; i++){
		if(cal_points[i]<=valor && valor<=cal_points[i+1]){
			x1 = cal_points[i];
			x2 = cal_points[i+1];
			y1 = cal_offsets[i];
			y2 = cal_offsets[i+1];
			m = (y2-y1)/(x2-x1);
			b = y1-(m*x1);
			break;
		}
	}
	//Retorno el valor sumado a su correccion segun la recta obtenida:
	return valor + (m*valor) + b;
}

////////////////////////////
//  FUNCION tareas_Init   //
////////////////////////////
//Funcion que realiza tareas de inicializacion al encendido del dispositivo
void tareas_Init(){
	memset(cal_offsets, 0, sizeof(cal_offsets));
	memset(cal_default, 0, sizeof(cal_default));
	cal_offsets[TOTAL_CAL_POINTS]=EEPROM_EMPTY;	//Refresco el fin del vector de calibracion
	cal_default[TOTAL_CAL_POINTS]=EEPROM_EMPTY;	//Refresco el fin del vector default

}


////////////////////////////
//	 TAREA PULSADORES	  //
////////////////////////////
// En esta tarea se lee individualmente a cada uno de los pulsadores y se realiza el antirebote.
// La interaccion esta pensada para monousuario y monotecla.
void tarea_pulsadores (void *p)
{
	uint8_t tecla_leida = NO_KEY;
	if(HAL_GPIO_ReadPin(MODE_KEY)){
		tecla_leida = MODE;
	}
	if(HAL_GPIO_ReadPin(ZERO_KEY)){
		tecla_leida = ZERO;
	}
	if(HAL_GPIO_ReadPin(HOLD_KEY)){
		tecla_leida = HOLD;
	}
	keyboard_buffer = antirebote(tecla_leida);
}

////////////////////////////
//	    TAREA MODOS 	  //
////////////////////////////
//Imprime en el display los titulos de los modos en la linea superior.
//Y esta pendiente del buffer de teclas para hacer lo que corresponda en cada modo.
void tarea_modos(void *p)
{
	static uint8_t clean = OFF;

	//Si tengo una lectura de la tecla MODE cambio de modo al que sigue
	if(keyboard_buffer == MODE){
		modo++;
		modo%=CANT_MODOS;
		clean = OFF;
		indice_cal = 0;
		keyboard_buffer = NO_KEY;
	}

	//Limpieza de pantalla en cada cambio de modo:
	if(clean == OFF){
		SSD1306_Fill(SSD1306_COLOR_BLACK); /* Clear screen */
		offset = 0;
		clean = ON;
		f_hold = OFF;
	}

	//Realizo lo correspondiente a cada modo:
	switch(modo){
	case MODO_MEDIR:	//MODO1
		SSD1306_GotoXY (POS_TITULO);
		SSD1306_Puts("----Medicion----", &Font_7x10, SSD1306_COLOR_WHITE);
		f_2lineas = OFF;
		f_alarmas = OFF;	//todo: poner ON e implementar
		f_default = OFF;
		if(keyboard_buffer == ZERO){
			f_zero = ON;
//			keyboard_buffer = NO_KEY; No vacio el buffer porque me conviene poder mantener pulsado
		}
		if(keyboard_buffer == HOLD){
			f_hold = ~ f_hold; // toggle flag
			keyboard_buffer = NO_KEY;
		}
		break;

	case MODO_CAL:	//MODO2
		SSD1306_GotoXY (POS_TITULO);
		SSD1306_Puts("---Calibracion---", &Font_7x10, SSD1306_COLOR_WHITE);
		f_2lineas = ON;
		f_alarmas = OFF;
		if(keyboard_buffer == ZERO){	// Pasar al siguiente angulo a calibrar
			indice_cal++;
			indice_cal %= TOTAL_CAL_POINTS+1;
			if(indice_cal == TOTAL_CAL_POINTS)
				f_default = ON;
			else
				f_default = OFF;
			keyboard_buffer = NO_KEY;
		}
		if(keyboard_buffer == HOLD){
			if(f_default == ON){	// Si se solicita resetear la calibracion, cargo los offsets por defecto
				for(int i=0; i<TOTAL_CAL_POINTS; i++){
					cal_offsets[i] = cal_default[i];	//Cargo valores default (todos ceros)
				}
				indice_cal = 0;
				modo = MODO_MEDIR;
				clean = OFF;
			}
			else{	// Cargo solo el offset de la lectura actual (lectura - angulo que se solicitado)
				cal_offsets[indice_cal] = lectura - cal_points[indice_cal];
			}
//			EEPROM_refresh(EEPROM_SAVE);	//Guardo en EEPROM cal_offsets completo, no solo el ultimo valor modificado
			if(writeEEPROM() == EEPROM_ERR){ //todo, decomentar esto y ver porque me cambia el modo
				while(1);
			}
//			writeEEPROM();
			f_default = OFF;
			keyboard_buffer = NO_KEY;
		}
		break;

	default: //Error, vuelvo al MODO1
		modo = MODO_MEDIR;
		break;
	}
}

////////////////////////////
//	   TAREA DISPLAY	  //
////////////////////////////
//Se encarga de preparar los valores de interes leidos, y luego los muestra en pantalla en forma adecuada.
//El area disponible se reparte en un reglon unico con fuente grande, o en dos lineas con fuentes mas pequeñas.
void tarea_display(void *p)
{
	static int cont = 0;		//Contador para los promedios
	static float prom = 0;		//Acumulador para los promedios
	static float valorAnt = 0;
	float prom_offset = 0;
	char signo;
	char str12[8] = {0};		//String para MODO1
	char str12_aux[8] = {0};
	char str1[9] = {0};			//String de Linea1 para MODO2
	char str1_aux[7] = {0};
	char str2[9] = {0};			//String de Linea2 para MODO2
	char str2_aux[7] = {0};

	//Realizo una cierta cantidad de promedios antes de mostrar en pantalla, para que no actualice como loco:
	if(cont >= PROMEDIOS){
		cont = 0;
		prom /= PROMEDIOS;

		//Graficado del signo:
		if(offset != 0){
			prom = fabs(prom);
			if((prom_offset) < 0)
				signo = (f_2lineas? '+': '>'); // >	todo: estan bien el sentido de los signos?
			else
				signo = (f_2lineas? '-': '<'); // <
		}
		else{
			if(prom < 0)
				signo = (f_2lineas? '+': '>'); // >
			else
				signo = (f_2lineas? '-': '<'); // <
			prom = fabs(prom);
		}
		str12[0] = signo;
		str1[0] = signo;

		prom_offset = prom - offset;

		//Zero:
		if(f_zero == ON){
			offset = prom;
			f_zero = OFF;
		}

		//Umbral:
		if(fabs(prom_offset - valorAnt) < THRESHOLD )
			prom_offset = valorAnt;

		//Valor Anterior:
		valorAnt = prom_offset;

		//Redondeo:
		if(fabs(prom_offset) >= 10 ){
			valuetoSave = prom_offset;
			sprintf(str12_aux,"%.2f@ ", roundf(fabs(prom_offset) * 100)/100);
			strcat(str12, str12_aux);
		}
		else{
			valuetoSave = prom_offset;
			sprintf(str12_aux,"0%.2f@", roundf(fabs(prom_offset) * 100)/100);
			strcat(str12, str12_aux);
		}

		//Visualizacion de valores:
		if(f_2lineas == OFF){	// Modo Medicion (1 linea)
			if(f_alarmas == OFF){
				SSD1306_GotoXY (POS_GRADOS);
				SSD1306_Puts(str12, &Font_16x26, SSD1306_COLOR_WHITE);
			}
			else{
				SSD1306_GotoXY(POS_LINEA1);
				SSD1306_Puts(str12, &Font_11x18, SSD1306_COLOR_WHITE);
//				SSD1306_GotoXY(POS_GRADOSA);
//				lastAlarm(str2);
//				SSD1306_Puts(str2, &Font_11x18, SSD1306_COLOR_WHITE);
			}
			if(alarmCheck(prom_offset)){
				HAL_GPIO_WritePin(LED_ALARMA, SET);
			}
			else{
				HAL_GPIO_WritePin(LED_ALARMA, RESET);
			}
		}
		else{	// Modo Calibracion (2 lineas)
			if(f_default==ON){
//				SSD1306_GotoXY(POS_LINEA1);
//				SSD1306_Puts("HOLD TO", &Font_11x18, SSD1306_COLOR_WHITE);
				SSD1306_GotoXY(POS_LINEA2);
				SSD1306_Puts("RESET CAL", &Font_11x18, SSD1306_COLOR_WHITE);
			}
			else{
				SSD1306_GotoXY(POS_LINEA1);
				sprintf(str1_aux, "%.2f*", cal_offsets[indice_cal]);
				strcat(str1, str1_aux);
				SSD1306_Puts(str1, &Font_11x18, SSD1306_COLOR_WHITE);
				SSD1306_GotoXY(POS_LINEA2);
				sprintf(str2_aux, "%i)", indice_cal+1);
				strcat(str2, str2_aux);
				sprintf(str2_aux, "%.2f*", cal_points[indice_cal]);
				strcat(str2, str2_aux);
				SSD1306_Puts(str2, &Font_11x18, SSD1306_COLOR_WHITE);
			}
		}
		prom = 0;
	}
	//Si no llegue a la cantidad de promedios necesarios, acumulo mediciones
	else{
		lectura = MPU6050.KalmanAngleY;	//todo, como el eje esta al reves, capaz debo ver el signo al reves
		medicion = corregir(lectura);
		prom += medicion;
		cont++;
	}
}

////////////////////////////
//	   TAREA ORIENTA	  //
////////////////////////////
// En esta tarea solamente se llama a la función MPU6050_Read_All de manera tal que
// se actualiza la estructura global donde se guardan todas las lecturas respectivas
// del acelerómetro.
void tarea_orienta(void *p)
{
	MPU6050_Read_All(&hi2c1, &MPU6050);
}

////////////////////////////
//	  TAREA BLINKING	  //
////////////////////////////
// Esta tarea hace conmutar el estado del led de la bluepill
void tarea_led_blinking(void *p)
{
	HAL_GPIO_TogglePin(GPIOC, Led_Blink_Pin);
}

////////////////////////////
//	   TAREA REFRESH	  //
////////////////////////////
// Esta tarea se encarga de refrescar la medición mostrada en pantalla
// en caso de que no se haya pulsado el botón de HOLD en los modos 1 y 2.
void tarea_refresh(void *p)
{
	if(f_hold == OFF)
		SSD1306_UpdateScreen();
}
