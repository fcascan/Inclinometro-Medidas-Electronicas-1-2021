/*
 * EEPROM.c
 *
 *  Created on: 2 nov. 2020
 *      Author: nicoc
 *  Version 2021: fcascan, oct 2021
 */

#include "main.h"
#include "EEPROM.h"
#include "EEPROM_21.h"

extern I2C_HandleTypeDef hi2c1;
extern float cal_offsets[TOTAL_CAL_POINTS+1];


////////////////////////////
//	 FUNCION readEEPROM  //
////////////////////////////
//Funcion que lee de la EEPROM la parte entera y decimal de cada calibracion a realizar y lo guarda en el vector
//auxiliar cal_offsetsInt, para luego rearmarlo en el vector cal_offsets de manera que quedaria guardado asi:
//(calibracion1, calibracion2, calibracion3, ...)
//Solo se ejecuta al arranque del dispositivo debido a la funcion EEPROM_Init
uint8_t readEEPROM(void){
	//Leo de la EEPROM pagina a pagina y cargo en el vector cal_offsets:
	for(int i=0; i<TOTAL_CAL_POINTS; i++){
		cal_offsets[i] = EEPROM_Read_NUM (i,0);	//(number of start page, start byte in the page)
//		EEPROM_Read(0, 0, cal_offsets, TOTAL_CAL_POINTS);
	}
	return EEPROM_OK;
}


////////////////////////////
//	 FUNCION writeEEPROM  //
////////////////////////////
//Funcion que lee el vector cal_offsets completo, lo desarma en parte entera y decimal en un vector auxiliar
//llamado cal_offsetsInt para luego guardar en la EEPROM ese contenido de manera que quedaria guardado asi:
//(Parte_entera1, Parte_decimal1, Parte_entera2, Parte_decimal2, ...)
uint8_t writeEEPROM(void){
	//Escribo en la EEPROM el vector cal_offsets de a una pagina por cada nro float:
	for(int i=0; i<TOTAL_CAL_POINTS || cal_offsets[i]==EEPROM_EMPTY; i++){
		//number of start page, start byte in the page, data to write (int or float)
		EEPROM_Write_NUM (i,0,cal_offsets[i]);
//		EEPROM_Write(0, 0, cal_offsets, TOTAL_CAL_POINTS);
	}
	return EEPROM_OK;
}


////////////////////////////
//	 FUNCION writeEEPROM  //
////////////////////////////
//Funcion que se ejecuta solo al inicio del dispositivo.
//Solo se encarga de levantar los valores de la EEPROM y cargarlos en la variable cal_offsets.
//Si por algun motivo falla, se queda en un bucle que no permite avanzar al resto del programa.
void EEPROM_Init(void){
	if(readEEPROM() == EEPROM_ERR){
		while(1);
	}
//	readEEPROM();
}

