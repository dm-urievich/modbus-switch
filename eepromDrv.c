/*
 * ������� eeprom ������
 * ����� ������� ��������� (32 ����)
 * ��������� � ������ ������� �� 16 ���
 * ����� ���������� �� 2
 * ������������ ���������� ��������� �� ST
 */

#include "./STM8S_StdPeriph_Driver/inc/stm8s.h"
#include "./STM8S_StdPeriph_Driver/inc/stm8s_flash.h"
#include "eepromDrv.h"
#include "main.h"

extern eeprom_type eeprom;

/*
 * ������������ ���������� � �����
 * ��������� ������� �� ������
 * ���� ���������� ���������� ������ � eeprom
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
 * �������������
 * ���������������� �������, ��� �����
 */
void eepromInit()
{

    // ������ ���� ��������� �� ������
    // �������� ��������� ����� ��� ��������
	FLASH_Unlock(FLASH_MEMTYPE_DATA);
}

/*
 * ������� ������
 * �������� ������ � �������
 * ����� eepromPoll ���������
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
 * ���������������� ������ � eeprom
 */
void eepromWriteSpi(uint16_t addr, uint16_t data)
{
    FLASH_ProgramWord(addr*4 + FLASH_DATA_START_PHYSICAL_ADDRESS, data);
}

/*
 * ������, ����� ������ ������ �� eeprom
 */
uint16_t eepromRead(uint16_t addr)
{
	uint16_t data;

	data = FLASH_ReadByte(addr*4+3 + FLASH_DATA_START_PHYSICAL_ADDRESS);
	data |= FLASH_ReadByte(addr*4+2 + FLASH_DATA_START_PHYSICAL_ADDRESS) << 8;

	return data;
}

/*
 * �������� �� ���������
 */
uint8_t isEepromReady(void)
{
    // �� ���� ����� ��������� ��������������� �� ����� ���� ��� �� �������
    // ����� ����� 6 ��
	return 1;
}

/*
 * ����� ������ � �������
 * ���� ������� ������ ������������ NULL
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
