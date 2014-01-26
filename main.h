#include "stm8s_type.h"
#include "stdint.h"

#define NULL 0

typedef enum{
    buttPressed,
    buttRelase,
    buttFalling,     // ������ ������
    buttRaising,     // ������ ���������
}buttonStates_enum;

// �������� �� ������� ������
typedef enum{
    buttNon,      // ��� ��������
    buttSwitch,   
    buttOn,
    buttOff,
}buttonEction_enum;

// ������ �� ������������
typedef enum{
    ledOff_en,  // 0
    ledOn_en,   // 1
    ledModbus_en, // 2
    ledDout_en, // 3
    ledNotDout_en, // 4
    ledDin_en,  // 5
    ledNotDin_en, // 6
    ledManual_en, // 7
}ledMode_enum;

#define BUTT_FILTER_TIME    100  // ����� ���������������� ������� � ��


uint16_t buttEction(buttonEction_enum butt, uint16_t state);
buttonStates_enum getButtState(void);
void buttonPoll(void);
void buttonInit(void);
buttonStates_enum getPinButtonState(void);

typedef enum{
    toggleOnOff = 0,    // ����� ����� ����������� �����
    fallingEdgeOnOff,   // ������ ������� ����������
}buttonMode_enum;

void TimModbusEn(void);
void TimModbusDis(void);
void UART_Send(u8 *buff, u8 len);

//============================================
// ��������� ��������
//============================================
typedef enum {
	readOnly,
	writeOnly,
	readWrite,
}rwMask_enum;

typedef struct {
	uint16_t *pnt;		// ��������� �� ������
	uint16_t minLim;		// ����������� ��������
	uint16_t maxLim;		// ������������ ��������
	rwMask_enum rwMask;		// ����� ������/������
	uint8_t eeprom;		// ������� �������� ������ � eeprom
	uint16_t defVal;	// ���������� �������� ��������
}modbusReg_type;

uint8_t mbRegsInit(void);
uint8_t findReg(uint16_t *pnt);

void LEDon(void);
void LEDoff(void);

void setTx(void);
void setRx(void);
