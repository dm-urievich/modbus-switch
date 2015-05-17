//#define HSE_VALUE 16000000

#include "stm8s.h"
#include "intrinsics.h"   // здесь функция итерапт энейбл
#include "main.h"
#include "eepromDrv.h"
#include "mb.h"
#include "stm8s_tim4.h"
#include "stm8s_tim2.h"
#include "stm8s_adc1.h"

u16 reservReg;

uint16_t state;          // текущее состояние выхода
uint16_t switcher;       // "переключалка", сама обнуляется
uint16_t buttModeFall;     // режим работы локальной кнопки по нажатию 
uint16_t buttModeRais;     // режим работы локальной кнопки по отпусканию

// управление светодиодом
uint16_t ledState;      // сотояние светодиода
uint16_t ledMode;       // режим работы

uint16_t mbAddr;         // модбасный адрес
uint16_t mbBaudrate;     // скорость, согласно принятому формату
uint16_t eprVirgin;	    // признак неисполозованности eeprom

uint16_t button;         // кнопка, текущее состояние, отфильтрованное
uint16_t adcData;       // данные с АЦП

// уникальный идентификатор, может пригодится
uint16_t idModule_1;
uint16_t idModule_2;

// тип девайса, типа геркон, датчик движения, датчик температуры
uint16_t typeDev;

eeprom_type eeprom;

//****************************************************************************
const char modbusIdString[] = "BYCE eSwith - v1.2";
//****************************************************************************
modbusReg_type tableRegs[] = {
/*0x0000*/  {(uint16_t*) &state, 	0x0000, 0x0001, readWrite, 0, 0x0000},  // Состояние выхода Int
/*0x0001*/  {(uint16_t*) &switcher, 	0x0000, 0x0001, readWrite, 0, 0x0000},  // Переключатель Int
/*0x0002*/  {(uint16_t*) &button,       0x0000, 0x0001, readWrite, 0, 0x0000},  // Состояние дискр. вх. Int
/*0x0003*/  {(uint16_t*) &buttModeFall, 0x0000, 0x00FF, readWrite, 1, 0x0000},  // Задний фротн диск. вх. Text
                                                                                // 0 - Ничего не делать
                                                                                // 1 - Переключить
                                                                                // 2 - Включить
                                                                                // 3 - Выключить
/*0x0004*/  {(uint16_t*) &buttModeRais, 0x0000, 0x00FF, readWrite, 1, 0x0000},  // Передний фротн диск. вх. Text
                                                                                // 0 - Ничего не делать
                                                                                // 1 - Переключить
                                                                                // 2 - Включить
                                                                                // 3 - Выключить	
/*0x0005*/  {(uint16_t*) &adcData,   0x0000, 0xFFFF, readWrite, 0, 0x0000},     // Данные с АЦП Int
/*0x0006*/  {(uint16_t*) &ledState,  0x0000, 0x0001, readWrite, 0, 0x0000},     // Состояние светодиода  Int
/*0x0007*/  {(uint16_t*) &ledMode,   0x0000, 0x0008, readWrite, 1, 0x0000},     // Режим работы светодиода  Text
                                                                                // 0 - Всегда выключен
                                                                                // 1 - Всегда включен
                                                                                // 2 - Индикация обмена
                                                                                // 3 - Повторение выхода
                                                                                // 4 - Инверсия выхода
                                                                                // 5 - Повторение дискр. вх.
                                                                                // 6 - Инверсия дискр. вх.
                                                                                // 7 - Ручное управление
/*0x0008*/  {(uint16_t*) &reservReg, 0x0000, 0xFFFF, readWrite, 0, 0x0000},     // Резерв  Reserv
/*0x0009*/  {(uint16_t*) &mbAddr,    0x0000, 0x00FF, readWrite, 1, 0x0001},     // Адрес в сети Int
/*0x000A*/  {(uint16_t*) &mbBaudrate,0x0000, 0xFFFF, readWrite, 1, 1152},       // Скорость обмена Text
                                                                                // 96 - 9600
                                                                                // 192 - 19200
                                                                                // 384 - 38400
                                                                                // 576 - 57600
                                                                                // 1152 - 115200
/*0x000A*/  {(uint16_t*) &idModule_1,0x0000, 0xFFFF, readWrite, 1, 0x0000},     // Уникальный идентификатор 1 слово Int
/*0x000A*/  {(uint16_t*) &idModule_2,0x0000, 0xFFFF, readWrite, 1, 0x0000},     // Уникальный идентификатор 2 слово Int
/*0x000A*/  {(uint16_t*) &typeDev   ,0x0000, 0xFFFF, readWrite, 1, 0x0000},     // Тип девайса Int
/*0x000B*/  {(uint16_t*) &eprVirgin, 0x0000, 0xFFFF, readWrite, 1, 0xAA55},     // Резерв  Reserv
/*0x000C*/
};

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START 0
#define REG_HOLDING_START REG_INPUT_START

// количество регистров в модбасе
static u8 usRegInputNregs = sizeof(tableRegs) / sizeof(modbusReg_type);

void main( void )
{
    buttonStates_enum tekButton;
  
  eMBErrorCode eStatus;
	//Инициализируем CLK
	CLK_HSECmd(ENABLE);
        
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO,
                         CLK_SOURCE_HSE,
                         DISABLE,
                         CLK_CURRENTCLOCKSTATE_ENABLE);

  //CLK->SWCR |= CLK_SWCR_SWEN;  //Разрешаем автопереключение источника Clock при неисправности генератора

  //CLK->CKDIVR = 0;             //Делители частоты внутреннего и внешнего генератора на 1 - частота ядра максимальная
    CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART2, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);

   // светодиод
  GPIOB->DDR = 1 << 5;   // Ножка PD0 конфигурируется на вывод
  GPIOB->CR1 = 1 << 5;   // Выход типа Push-pull
  GPIOB->CR2 = 1 << 5;

  // транзистор
  GPIOC->DDR = 1 << 1;   // Ножка PD0 конфигурируется на вывод
  GPIOC->CR1 = 1 << 1;   // Выход типа Push-pull
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
    
    for(;;)              // Бесконечный цикл
    {
        //GPIOB->ODR ^= 1 << 5;   // Переключение уровня напряжения на ножке на противоположное
                                      // при помощи операции Исключающее ИЛИ (XOR)

        // тут еще АЦП считываем
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

// обработка события кнопки
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
  
// ножку на прием
void setRx(void)
{
  GPIOD->ODR &= ~(1 << 7);
}

// ножку на передачу
void setTx(void)
{
  GPIOD->ODR |= (1 << 7);
}

//===============================================
// 4 функция, чтение регистров
//===============================================
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    usAddress--;  // в библиотеке какого-то хера делатеся ++
    
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
        		eStatus = MB_ENOREG;	// возможно заменить на другую ошибку
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
// чтение и запись регистра
// используется в функциях 3, 23 (0x17), 6, 16 (0x10)
//================================================
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    usAddress--;  // в библиотеке какого-то хера делатеся ++
    
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + usRegInputNregs ) )
    {
        iRegIndex = ( int )( usAddress - REG_HOLDING_START );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
        	// у мен не различаются Holding и Input регистры
        	// вызываю функцию чтения регистров
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
            		eStatus = MB_ENOREG;	// возможно заменить на другую ошибку
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
// для работы с битами, у меня не используется
//================================================
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

//================================================
// 2 функция, чтение дискретных входов
// не используется
//================================================
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}

/*
 * поиск индекса в массиве модбасного регистра
 * (для записи в eeprom, идекс это адрес регистра в eeprom)
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
 * инициализация модбасных регистров
 * если eeprom чиста - запись дефолтовых знаений
 */
uint8_t mbRegsInit(void)
{
	int i;
	i = findReg(&eprVirgin);
	eprVirgin = eepromRead(i);
	if (eprVirgin != tableRegs[i].defVal) {	// eeprom не юзаная или специально обнулена
		for (i = 0; i < usRegInputNregs; i++) {
			if (tableRegs[i].eeprom) {				// записываем только то что должно там храниться
				eepromWriteSpi(i, tableRegs[i].defVal);
				while (!isEepromReady())
					;
			}
		}
	}

	// инициализацмия всех переменных программы
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
* возврашает состояние кнопки
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
* антидребезговый фильтр
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
        
        // при включенном АЦП дискр. вход не считывается
        // АЦП выключается в основном цикле после окончания
        // преоборазования, что гарантированно раньше следующего
        // попадания сюда
        ADC1_Cmd(ENABLE);
        ADC1_StartConversion();
    }
}

/*
* инициализация ножки, таймера для кнопки
*/
void buttonInit(void)
{
    TIM2_TimeBaseInit(TIM2_PRESCALER_64, 250);      // один тик это 1 мс
    TIM2_Cmd(ENABLE);

    GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);
    
    button = getPinButtonState();
    
    getButtState();
    getButtState();
}

/*
* возращает состояние порта к которому подлючена кнопка
*/
buttonStates_enum getPinButtonState(void)
{
  if (GPIO_ReadInputPin(GPIOB, GPIO_PIN_0) == SET)
     return buttRelase;
  else 
    return buttPressed;
}

