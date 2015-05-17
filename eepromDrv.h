/*
 * eepromDrv.h
 *
 *	драйвер eeprom для stm8
 *  реализует отлаженную запись через очередь
 *
 *  Created on: 04.04.2013
 *      Author: dima
 */

#ifndef EEPROMDRV_H_
#define EEPROMDRV_H_
#include "stdint.h"

#define QEUE_LENGTH	3	// длинна очереди

typedef struct{
	uint16_t addr;
	uint16_t data;
}eepromQeue_type;

typedef struct{
	eepromQeue_type writeQeue[QEUE_LENGTH];	// очередь на запись
	uint8_t qeueStart;		// начало очерди
	uint8_t qeueEnd;		// конец очереди
	uint8_t qeueNums;		// количество данных в очереди
}eeprom_type;


void eepromPoll();
void eepromInit();
void eepromWrite(uint16_t addr, uint16_t data);
uint16_t eepromRead(uint16_t addr);

eepromQeue_type * getFromQeue(eeprom_type *epr);
void eepromWriteSpi(uint16_t addr, uint16_t data);

uint8_t isEepromReady(void);


#endif /* EEPROMDRV_H_ */
