/*
 * драйвер eeprom памяти
 * режим доступа пословный (32 бита)
 * адресация и данные модбаса по 16 бит
 * адрес умножается на 2
 * используется библиотека драйверов от ST
 */

#include "./STM8S_StdPeriph_Driver/inc/stm8s.h"
#include "./STM8S_StdPeriph_Driver/inc/stm8s_flash.h"
#include "eepromDrv.h"
#include "main.h"

extern eeprom_type eeprom;

/*
 * периодически вызывается в цикле
 * проверяет очередь на запись
 * если необходимо инициирует запись в eeprom
 */
void eepromPoll()
{
	eepromQeue_type *tekQeue;
	tekQeue = getFromQeue(&eeprom);
	if (tekQeue != NULL) {
		if (isEepromReady()) {
			eepromWriteSpi(tekQeue->addr, tekQeue->data);
		}
	}
}

/*
 * инициализация
 * инициализируется очередь, все порты
 */
void eepromInit()
{

    // плохая идея разлочить на всегда
    // бешенный указатель может все похерить
	FLASH_Unlock(FLASH_MEMTYPE_DATA);
}

/*
 * функция записи
 * помещает запрос в очередь
 * потом eepromPoll выгребает
 */
void eepromWrite(uint16_t addr, uint16_t data)
{
	eepromQeue_type *tekQeue;
	eeprom_type *epr = &eeprom;

	tekQeue = &(epr->writeQeue[epr->qeueEnd]);

	tekQeue->addr = addr;
	tekQeue->data = data;

	if (epr->qeueNums < QEUE_LENGTH) {
		epr->qeueNums++;
	}
	if (epr->qeueEnd < QEUE_LENGTH-1) {
		epr->qeueEnd++;
	}
	else {
		epr->qeueEnd = 0;
	}
}

/*
 * непосредственная запись в eeprom
 */
void eepromWriteSpi(uint16_t addr, uint16_t data)
{
    FLASH_ProgramWord(addr*4 + FLASH_DATA_START_PHYSICAL_ADDRESS, data);
}

/*
 * чтение, сразу выдает данные из eeprom
 */
uint16_t eepromRead(uint16_t addr)
{
	uint16_t data;

	data = FLASH_ReadByte(addr*4+3 + FLASH_DATA_START_PHYSICAL_ADDRESS);
	data |= FLASH_ReadByte(addr*4+2 + FLASH_DATA_START_PHYSICAL_ADDRESS) << 8;

	return data;
}

/*
 * проверка на занятость
 */
uint8_t isEepromReady(void)
{
    // по ходу здесь процессор останавливается на вермя пока все не запишет
    // время около 6 мс
	return 1;
}

/*
 * берем данные с очереди
 * если очередь пустая возвращается NULL
 */
eepromQeue_type * getFromQeue(eeprom_type *epr)
{
	eepromQeue_type *tekQeue;

	if (epr->qeueNums != 0) {
		tekQeue = &(epr->writeQeue[epr->qeueStart]);
		if (epr->qeueStart < QEUE_LENGTH-1) {
			epr->qeueStart++;
		}
		else {
			epr->qeueStart = 0;
		}
		epr->qeueNums--;
	}
	else {
		tekQeue = NULL;
	}

	return tekQeue;
}
