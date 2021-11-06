/*
 * EEPROM.c
 *
 *  Created on: 2 nov. 2020
 *      Author: nicoc
 *  Version 2021: fcascan, oct 2021
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

//Caracteristicas acordes a la EEPROM 24LC64:
//400KHz(Max fclock), 32bytes per page, 8192 x 8bit memory organization, page write time = 5ms,
//entonces tengo 8192bytes total, asi que 8192/32 = 256 pages de 32bytes
#define EEPROM_ADDRESS			0xA0	//De hoja de datos: 10100000 (donde: XXXX A2 A1 A0 RW)

#define EEPROM_ERR				0
#define EEPROM_OK				1

#define EEPROM_EMPTY			250		// valor que nunca se mide

#define	INICIO				0
#define TOTAL_CAL_POINTS	19
#define OK					0
#define	NO_OK				1

uint8_t readEEPROM( void );
uint8_t writeEEPROM( void );
void EEPROM_Init( void );

#endif /* INC_EEPROM_H_ */
