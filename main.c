//#define HSE_VALUE 16000000

#include "stm8s.h"
#include "intrinsics.h"   // ����� ������� ������� ������
#include "main.h"
#include "eepromDrv.h"
#include "mb.h"
#include "stm8s_tim4.h"
#include "stm8s_tim2.h"
#include "stm8s_adc1.h"

u16 reservReg;

uint16_t state;          // ������� ��������� ������
uint16_t switcher;       // "������������", ���� ����������
uint16_t buttModeFall;     // ����� ������ ��������� ������ �� ������� 
uint16_t buttModeRais;     // ����� ������ ��������� ������ �� ����������

// ���������� �����������
uint16_t ledState;      // �������� ����������
uint16_t ledMode;       // ����� ������

uint16_t mbAddr;         // ��������� �����
uint16_t mbBaudrate;     // ��������, �������� ��������� �������
uint16_t eprVirgin;	    // ������� ������������������ eeprom

uint16_t button;         // ������, ������� ���������, ���������������
uint16_t adcData;       // ������ � ���

// ���������� �������������, ����� ����������
uint16_t idModule_1;
uint16_t idModule_2;

// ��� �������, ���� ������, ������ ��������, ������ �����������
uint16_t typeDev;

eeprom_type eeprom;

//****************************************************************************
const char modbusIdString[] = "BYCE eSwith - v1.2";
//****************************************************************************
modbusReg_type tableRegs[] = {
/*0x0000*/  {(uint16_t*) &state, 	0x0000, 0x0001, readWrite, 0, 0x0000},  // ��������� ������ Int
/*0x0001*/  {(uint16_t*) &switcher, 	0x0000, 0x0001, readWrite, 0, 0x0000},  // ������������� Int
/*0x0002*/  {(uint16_t*) &button,       0x0000, 0x0001, readWrite, 0, 0x0000},  // ��������� �����. ��. Int
/*0x0003*/  {(uint16_t*) &buttModeFall, 0x0000, 0x00FF, readWrite, 1, 0x0000},  // ������ ����� ����. ��. Text
                                                                                // 0 - ������ �� ������
                                                                                // 1 - �����������
                                                                                // 2 - ��������
                                                                                // 3 - ���������
/*0x0004*/  {(uint16_t*) &buttModeRais, 0x0000, 0x00FF, readWrite, 1, 0x0000},  // �������� ����� ����. ��. Text
                                                                                // 0 - ������ �� ������
                                                                                // 1 - �����������
                                                                                // 2 - ��������
                                                                                // 3 - ���������	
/*0x0005*/  {(uint16_t*) &adcData,   0x0000, 0xFFFF, readWrite, 0, 0x0000},     // ������ � ��� Int
/*0x0006*/  {(uint16_t*) &ledState,  0x0000, 0x0001, readWrite, 0, 0x0000},     // ��������� ����������  Int
/*0x0007*/  {(uint16_t*) &ledMode,   0x0000, 0x0008, readWrite, 1, 0x0000},     // ����� ������ ����������  Text
                                                                                // 0 - ������ ��������
                                                                                // 1 - ������ �������
                                                                                // 2 - ��������� ������
                                                                                // 3 - ���������� ������
                                                                                // 4 - �������� ������
                                                                                // 5 - ���������� �����. ��.
                                                                                // 6 - �������� �����. ��.
                                                                                // 7 - ������ ����������
/*0x0008*/  {(uint16_t*) &reservReg, 0x0000, 0xFFFF, readWrite, 0, 0x0000},     // ������  Reserv
/*0x0009*/  {(uint16_t*) &mbAddr,    0x0000, 0x00FF, readWrite, 1, 0x0001},     // ����� � ���� Int
/*0x000A*/  {(uint16_t*) &mbBaudrate,0x0000, 0xFFFF, readWrite, 1, 1152},       // �������� ������ Text
                                                                                // 96 - 9600
                                                                                // 192 - 19200
                                                                                // 384 - 38400
                                                                                // 576 - 57600
                                                                                // 1152 - 115200
/*0x000A*/  {(uint16_t*) &idModule_1,0x0000, 0xFFFF, readWrite, 1, 0x0000},     // ���������� ������������� 1 ����� Int
/*0x000A*/  {(uint16_t*) &idModule_2,0x0000, 0xFFFF, readWrite, 1, 0x0000},     // ���������� ������������� 2 ����� Int
/*0x000A*/  {(uint16_t*) &typeDev   ,0x0000, 0xFFFF, readWrite, 1, 0x0000},     // ��� ������� Int
/*0x000B*/  {(uint16_t*) &eprVirgin, 0x0000, 0xFFFF, readWrite, 1, 0xAA55},     // ������  Reserv
/*0x000C*/
};

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START 0
#define REG_HOLDING_START REG_INPUT_START

// ���������� ��������� � �������
static u8 usRegInputNregs = sizeof(tableRegs) / sizeof(modbusReg_type);

void main( void )
{
    buttonStates_enum tekButton;
  
  eMBErrorCode eStatus;
	//�������������� CLK
	CLK_HSECmd(ENABLE);
        
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO,
                         CLK_SOURCE_HSE,
                         DISABLE,
                         CLK_CURRENTCLOCKSTATE_ENABLE);

  //CLK->SWCR |= CLK_SWCR_SWEN;  //��������� ���������������� ��������� Clock ��� ������������� ����������

  //CLK->CKDIVR = 0;             //�������� ������� ����������� � �������� ���������� �� 1 - ������� ���� ������������
    CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART2, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);

   // ���������
  GPIOB->DDR = 1 << 5;   // ����� PD0 ��������������� �� �����
  GPIOB->CR1 = 1 << 5;   // ����� ���� Push-pull
  GPIOB->CR2 = 1 << 5;

  // ����������
  GPIOC->DDR = 1 << 1;   // ����� PD0 ��������������� �� �����
  GPIOC->CR1 = 1 << 1;   // ����� ���� Push-pull
  GPIOC->CR2 = 1 << 1;

    eepromInit();

    mbRegsInit();

    eStatus = eMBInit( MB_RTU, (uint8_t)mbAddr, 0, (uint32_t)mbBaudrate * 100, MB_PAR_NONE );
   //eStatus = eMBInit( MB_RTU, 0x01, 0, 115200, MB_PAR_NONE );
	/* Enable the Modbus Protocol Stack. */
	eStatus = eMBEnable(  );

    setRx();

    buttonInit();
    
    enableInterrupts();

    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
              ADC1_CHANNEL_0,
              ADC1_PRESSEL_FCPU_D8,
              ADC1_EXTTRIG_TIM,
              DISABLE,
              ADC1_ALIGN_RIGHT,
              ADC1_SCHMITTTRIG_CHANNEL0,
              ENABLE);
    ADC1_StartConversion();
    
    for(;;)              // ����������� ����
    {
        //GPIOB->ODR ^= 1 << 5;   // ������������ ������ ���������� �� ����� �� ���������������
                                      // ��� ������ �������� ����������� ��� (XOR)

        // ��� ��� ��� ���������
        if (ADC1_GetFlagStatus(ADC1_FLAG_EOC)) {
            adcData = ADC1_GetConversionValue();
            ADC1_ClearFlag(ADC1_FLAG_EOC);   
            ADC1_Cmd(DISABLE);
        }
      
        eMBPoll(  );
    	eepromPoll();
    	buttonPoll();

    	tekButton = getButtState();
        switch (tekButton) {
        case buttPressed : 
          break;
        case buttRelase :
          break;
        case buttFalling :
          state = buttEction((buttonEction_enum) buttModeFall, state);
          break;
        case buttRaising :
          state = buttEction((buttonEction_enum) buttModeRais, state);
          break;
        default : break;
        }

        if (switcher) {
            state ^= 1;
            switcher = 0;
        }

        if (state){
            GPIOC->ODR |= (1 << 1);
        }
        else {
            GPIOC->ODR &= ~(1 << 1);
        }
        
        switch (ledMode) {
        case ledOff_en :
          LEDoff();
          break;
        case ledOn_en :
          LEDon();
          break;
        case ledModbus_en :
          break;
        case ledDout_en :
          if (state)
            LEDon();
          else
            LEDoff();
          break;
        case ledNotDout_en :
          if (!state)
            LEDon();
          else
            LEDoff();
          break;
        case ledDin_en :
          if (button)
            LEDon();
          else
            LEDoff();
          break;
        case ledNotDin_en :
          if (!button)
            LEDon();
          else
            LEDoff();
          break;
        case ledManual_en :
          if (ledState)
            LEDon();
          else
            LEDoff();
          break;
        default :
          LEDoff();
          break;
        }
    }
}

// ��������� ������� ������
uint16_t buttEction(buttonEction_enum butt, uint16_t state)
{
  switch (butt) {
  case buttNon : 
    break;
  case buttSwitch :
    state ^= 1;
    break;
  case buttOn :
    state = 1;
    break;
  case buttOff :
    state = 0;
    break;
  default : break;
  }
  
  return state;
}
  
// ����� �� �����
void setRx(void)
{
  GPIOD->ODR &= ~(1 << 7);
}

// ����� �� ��������
void setTx(void)
{
  GPIOD->ODR |= (1 << 7);
}

//===============================================
// 4 �������, ������ ���������
//===============================================
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    usAddress--;  // � ���������� ������-�� ���� �������� ++
    
    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + usRegInputNregs ) )
    {
        iRegIndex = ( int )( usAddress - REG_INPUT_START );
        while( usNRegs > 0 )
        {
        	if (tableRegs[iRegIndex].rwMask == readOnly || tableRegs[iRegIndex].rwMask == readWrite) {
				*pucRegBuffer++ =
					( unsigned char )( *(tableRegs[iRegIndex].pnt) >> 8 );
				*pucRegBuffer++ =
					( unsigned char )( *(tableRegs[iRegIndex].pnt) & 0xFF );
        	}
        	else {
        		eStatus = MB_ENOREG;	// �������� �������� �� ������ ������
        	}
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

//================================================
// ������ � ������ ��������
// ������������ � �������� 3, 23 (0x17), 6, 16 (0x10)
//================================================
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    usAddress--;  // � ���������� ������-�� ���� �������� ++
    
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + usRegInputNregs ) )
    {
        iRegIndex = ( int )( usAddress - REG_HOLDING_START );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
        	// � ��� �� ����������� Holding � Input ��������
        	// ������� ������� ������ ���������
        	eStatus = eMBRegInputCB(pucRegBuffer, usAddress+1, usNRegs);
        	/*
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            */
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
            	if (tableRegs[iRegIndex].rwMask == writeOnly || tableRegs[iRegIndex].rwMask == readWrite) {
            		*(tableRegs[iRegIndex].pnt) = *pucRegBuffer++ << 8;
            		*(tableRegs[iRegIndex].pnt) |= *pucRegBuffer++;

            		if (tableRegs[iRegIndex].eeprom) {
            			eepromWrite(iRegIndex, *(tableRegs[iRegIndex].pnt));
            		}
            	}
            	else {
            		eStatus = MB_ENOREG;	// �������� �������� �� ������ ������
            	}
            	iRegIndex++;
            	usNRegs--;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

//================================================
// ��� ������ � ������, � ���� �� ������������
//================================================
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

//================================================
// 2 �������, ������ ���������� ������
// �� ������������
//================================================
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}

/*
 * ����� ������� � ������� ���������� ��������
 * (��� ������ � eeprom, ����� ��� ����� �������� � eeprom)
 */
uint8_t findReg(uint16_t *pnt)
{
	uint8_t i;
	for (i = 0; i < usRegInputNregs; i++) {
		if (tableRegs[i].pnt == pnt) {
			return i;
		}
	}
	return 0xFF;
}

/*
 * ������������� ��������� ���������
 * ���� eeprom ����� - ������ ���������� �������
 */
uint8_t mbRegsInit(void)
{
	int i;
	i = findReg(&eprVirgin);
	eprVirgin = eepromRead(i);
	if (eprVirgin != tableRegs[i].defVal) {	// eeprom �� ������ ��� ���������� ��������
		for (i = 0; i < usRegInputNregs; i++) {
			if (tableRegs[i].eeprom) {				// ���������� ������ �� ��� ������ ��� ���������
				eepromWriteSpi(i, tableRegs[i].defVal);
				while (!isEepromReady())
					;
			}
		}
	}

	// �������������� ���� ���������� ���������
	for (i = 0; i < usRegInputNregs; i++) {
		if (tableRegs[i].eeprom) {
			*(tableRegs[i].pnt) = eepromRead(i);
		}
		else {
			*(tableRegs[i].pnt) = tableRegs[i].defVal;
		}
	}

	return 1;
}

void LEDon(void)
{
    GPIOB->ODR |= (1 << 5);
    ledState = 1;
}

void LEDoff(void)
{
    GPIOB->ODR &= ~(1 << 5);
    ledState = 0;
}

/*
* ���������� ��������� ������
*/
buttonStates_enum getButtState(void)
{
    static uint8_t button_prev = 0;
    uint8_t event;
    buttonStates_enum ret;
    
    event = button << 1 | button_prev;

    switch (event) {
        case 0 : ret = buttPressed;
            break;
        case 1 : ret =  buttFalling;
            break;
        case 2 : ret = buttRaising;
            break;
        case 3 : ret = buttRelase;
            break;
    }

    button_prev = button;

    return ret;
}

/*
* ��������������� ������
*/
void buttonPoll(void)
{
    static uint16_t button_count = 0;
    uint8_t butt;

    if (TIM2_GetFlagStatus(TIM2_FLAG_UPDATE) == SET) {
        TIM2_ClearFlag(TIM2_FLAG_UPDATE);
        butt = getPinButtonState();
        if (butt) {
            if (button_count > BUTT_FILTER_TIME) {
                button = 1;
            }
            else {
                button_count++;
            }
        }
        else {
            if (button_count == 0) {
                button = 0;
            }
            else {
                button_count--;
            }
        }
        
        // ��� ���������� ��� �����. ���� �� �����������
        // ��� ����������� � �������� ����� ����� ���������
        // ���������������, ��� �������������� ������ ����������
        // ��������� ����
        ADC1_Cmd(ENABLE);
        ADC1_StartConversion();
    }
}

/*
* ������������� �����, ������� ��� ������
*/
void buttonInit(void)
{
    TIM2_TimeBaseInit(TIM2_PRESCALER_64, 250);      // ���� ��� ��� 1 ��
    TIM2_Cmd(ENABLE);

    GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);
    
    button = getPinButtonState();
    
    getButtState();
    getButtState();
}

/*
* ��������� ��������� ����� � �������� ��������� ������
*/
buttonStates_enum getPinButtonState(void)
{
  if (GPIO_ReadInputPin(GPIOB, GPIO_PIN_0) == SET)
     return buttRelase;
  else 
    return buttPressed;
}

