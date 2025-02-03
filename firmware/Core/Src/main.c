/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include<stdio.h>
#include "BME.h"
#include <string.h>
#include "Logic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
int i=0;
int val=0;
uint8_t result=0;
uint8_t tempread[8];
uint8_t config[2];
uint8_t comp[32];
uint16_t ADCResult[6];
uint8_t _sensorID[7];
int i;
float tfloat;
volatile int temperature_raw, pressure_raw, humidity_raw = 0;
int finaltemp = 0;
uint32_t finalpressure, final_humidity = 0;
unsigned char dig_H1, dig_H3;
signed char dig_H6;
unsigned short dig_T1, dig_P1;
signed short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;
bmp280_calib_data _bmp280_calib[7];
int32_t t_fine;
int32_t p_fine;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	int i=0;
	for(i=0 ; i<len ; i++)
		ITM_SendChar((*ptr++));
	return len;
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	__NOP();
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//	__NOP();
	//  printf("adc=%d %d %d %d %d %d\n",ADCResult[0],ADCResult[1],ADCResult[2],ADCResult[3],ADCResult[4],ADCResult[5]);

}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t               RxData[8];
	CAN_RxHeaderTypeDef   RxHeader;
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
  printf("header=%d \n",RxData[0]);

	if(RxHeader.StdId!=0x281)return;
//	  SetServoPosition(RxData);
//	  CanSend(0x12);
//  TargetPoition=RxData[0];
//  TargetPoition<<=8;
//  TargetPoition+=RxData[1];
//  printf("setpoint=%d\n",TargetPoition);
	can_data_received(RxHeader.StdId,RxData);


}
void CanSend(uint32_t id)
{
	CAN_TxHeaderTypeDef   TxHeader;
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t               TxData[8];

	uint32_t              TxMailbox;
	  /*##-4- Start the Transmission process #####################################*/
	  TxHeader.StdId =id;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.DLC = 2;
	  TxHeader.TransmitGlobalTime = DISABLE;
	  TxData[0] = 0xCA;
	  TxData[1] = 0xFE;

	  /* Request transmission */
	//  if(HAL_CAN_AddTxMessage(hcan, TxHeader, aData, pTxMailbox)(hcan, TxMailboxes), TxMailboxes))
	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData, &TxMailbox) != HAL_OK)
	  {
	    /* Transmission request Error */
		  printf("can send error\n");
	  }



}
uint8_t CanSendArray(uint32_t id,uint8_t *data, uint32_t len)
{
	CAN_TxHeaderTypeDef   TxHeader;
	uint32_t              TxMailbox;
	  /*##-4- Start the Transmission process #####################################*/
	  TxHeader.StdId =id;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.DLC = 2;
	  TxHeader.TransmitGlobalTime = DISABLE;

	  /* Request transmission */
	//  if(HAL_CAN_AddTxMessage(hcan, TxHeader, aData, pTxMailbox)(hcan, TxMailboxes), TxMailboxes))
	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader,data, &TxMailbox) != HAL_OK)
	  {
	    /* Transmission request Error */
		  return 1;
	  }
	  return 0;



}
void SetDirection(bool dir)
{
	GPIOB->ODR &=~(1<<12);
	if(dir)GPIOB->ODR |=(1<<12);
}
//value is between 0 to 5
void SelectDriver(uint8_t value)
{
	value&=0x07;
	GPIOB->ODR &=~(0x07<<13);
	GPIOB->ODR |=(value<<13);
}
//minus means clockwise
void set_motor_speed(uint8_t motor,int32_t speed )
{


	if(speed<0) {
		SetDirection(0);
		speed*=-1;
	}
	else{
	SetDirection(1);

	}
	speed=1900-speed;
	if(speed>1900)speed=1900;
	if(speed<2)speed =2;

	SelectDriver(motor);

	 TIM4->CCR3=speed;



}
void SelectSensor(int8_t index)
{

	GPIOB->ODR &= ~ ((0x07)<<9);
	GPIOB->ODR |= ((0x07 & index)<<9);
}

void SpiWrite(uint8_t index,uint8_t *data,int len)
{
	int offset=-1;
	//if(index==1)offset=1;
	SelectSensor(index+offset);
	HAL_SPI_Transmit(&hspi1, data, len, 1000); //CONFIG
	SelectSensor(index);
}

void SPIReadWrite(uint8_t index,uint8_t *txBuffer,uint8_t txLen,uint8_t *rxBuffer,uint8_t rxLen)
{
	HAL_Delay(5);
	int offset=-1;
	//if(index==1)offset=1;
	SelectSensor(index+offset);
	HAL_SPI_Transmit(&hspi1, txBuffer, txLen, 10);
	HAL_SPI_Receive(&hspi1, rxBuffer, rxLen, 200);
	SelectSensor(index);
	HAL_Delay(5);

}
void BME280_CONFIG_SETUP(int index)
{

	config[0] = CTRLMEASREG;
	config[1] = CTRLMEASVAL;
	SpiWrite(index,config,2);

	config[0] = CONFIGREG;
	config[1] = CONFIGVAL;
	SpiWrite(index,config,2);

	config[0] = CTRLHUMREG;
	config[1] = CTRLHUMVAL;
	SpiWrite(index,config,2);

}

void BME280_GET_COMP_VALS(int index)
{
	config[0] = COMPTEMPPRES;
	SPIReadWrite(index,config,1,comp,24);
	dig_T1 = (comp[0])+(comp[1]<<8);
	dig_T2 = (comp[2])+(comp[3]<<8);
	dig_T3 = (comp[4])+(comp[5]<<8);
	dig_P1 = (comp[6])+(comp[7]<<8);
	dig_P2 = (comp[8])+(comp[9]<<8);
	dig_P3 = (comp[10])+(comp[11]<<8);
	dig_P4 = (comp[12])+(comp[13]<<8);
	dig_P5 = (comp[14])+(comp[15]<<8);
	dig_P6 = (comp[16])+(comp[17]<<8);
	dig_P7 = (comp[18])+(comp[19]<<8);
	dig_P8 = (comp[20])+(comp[21]<<8);
	dig_P9 = (comp[22])+(comp[23]<<8);

	config[0] = COMPHUMINIT;
	SPIReadWrite(index,config,1, &comp[24],1);
	dig_H1 = comp[24];

	config[0] = COMPHUMREST;

	SPIReadWrite(index,config,1, &comp[25],7);


	dig_H2 = (comp[25])+(comp[26]<< 8);
	dig_H3 = comp[27];
	dig_H4 = (comp[28] << 4) +(comp[29] & 0xF);
	dig_H5 = (comp[29] & 0xF0) +(comp[30]<< 4);
	dig_H6 = comp[31];
}

void BME280_GET_RAW_VALS(int index)
{
	BME280_CONFIG_SETUP(index);

	config[0] = RAWREAD;

	SPIReadWrite(index, config, 1, tempread, 8);

	temperature_raw =(tempread[3]<<12)+(tempread[4]<<4)+(tempread[5]>>4);
	pressure_raw = (tempread[0]<<12)+(tempread[1]<<4)+(tempread[2]>>4);
	humidity_raw = (tempread[6] << 8) + (tempread[7]);
}
void BME280_CALC_FINAL_VALS(){
	int var1, var2;
	var1 = ((((temperature_raw >> 3) - ((int)dig_T1 << 1))) * ((int)dig_T2)) >> 11;
	var2 = (((((temperature_raw >> 4) - ((int)dig_T1)) * ((temperature_raw >> 4) - ((int)dig_T1))) >> 12) * ((int)dig_T3)) >> 14;
	t_fine = (var1 + var2);
	finaltemp = (t_fine * 5 + 128) >> 8;

	var1 = (((int)t_fine) >> 1) - 64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int)dig_P6);
	var2 = var2 + ((var1 * ((int)dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int) dig_P4) << 16);
	var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int) dig_P2) * var1) >> 1 )) >> 18;
	var1 = ((((32768 + var1)) * ((int)dig_P1)) >> 15);
	if (var1 == 0)
	{
		finalpressure = 0;
	}
	else{
		finalpressure = (((uint32_t) (((int)1048576)-pressure_raw) - (var2 >> 12))) * 3125;
		if (finalpressure < 0x80000000){
			finalpressure = (finalpressure << 1) / (( uint32_t)var1);
		}
		else{
			finalpressure = (finalpressure / (uint32_t)var1) * 2;
		}
		var1 = (((int)dig_P9) * ((int) ((( finalpressure >> 3) * ( finalpressure >> 3)) >> 13))) >> 12;
		var2 = (((int) (finalpressure >> 2)) * ((int)dig_P8)) >> 13;
		finalpressure = ((uint32_t)((int)finalpressure + ((var1 + var2 + dig_P7) >> 4)))/100; //kPA
	}

	var1 = (t_fine - ((int) 76800));
	var1 = (((((humidity_raw << 14) - (((int) dig_H4) << 20) - (((int)dig_H5) * var1)) + ((int) 16384)) >> 15) * \
			(((((((var1 * ((int) dig_H6)) >> 10) * (((var1 * ((int) dig_H3)) >> 11) + ((int) 32768))) >> 10) + \
					((int) 2097152)) * ((int) dig_H2) + 8192) >> 14));

	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int)dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419330400 : var1);
	final_humidity = (var1 >> 12)/1024;
}

bool write8(int8_t reg,uint8_t val,int index)
{
	uint8_t txBuffer[2];
	txBuffer[0]=reg& ~ 0x80;
	txBuffer[1]=val;
	SpiWrite(index,txBuffer,2);
	return true;

}

uint8_t read8(int8_t reg,int index)
{
	uint8_t rxBuffer[1];
	SPIReadWrite(index,&reg, 1, rxBuffer, 1);
	return rxBuffer[0];
}

uint32_t read24(int8_t reg,int index)
{
	uint32_t result=0;
	uint8_t txBuffer[1];
	uint8_t rxBuffer[3];
	txBuffer[0]=(reg|0x80);
	SPIReadWrite(index, txBuffer, 1, rxBuffer, 3);
	result=rxBuffer[0];
	result<<=8;
	result|=rxBuffer[1];
	result<<=8;
	result|=rxBuffer[2];
	return result;
}

int32_t read16(uint8_t reg,int index)
{
	int32_t result=0;
	uint8_t txBuffer[1];
	uint8_t rxBuffer[2];
	txBuffer[0]=reg;
	SPIReadWrite(index, txBuffer, 1, rxBuffer, 2);
	result=rxBuffer[0];
	result<<=8;
	result|=rxBuffer[1];
	return result;
}

uint16_t read16_LE(uint8_t reg,int index)
{
	uint16_t temp = read16(reg,index);
	return (temp >> 8) | (temp << 8);
}

int16_t readS16_LE(uint8_t reg,int index)
{
	return (int16_t)read16_LE(reg,index);
}

void readCoefficients(int index) {
	_bmp280_calib[index].dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1,index);
	_bmp280_calib[index].dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2,index);
	_bmp280_calib[index].dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3,index);

	_bmp280_calib[index].dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1,index);
	_bmp280_calib[index].dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2,index);
	_bmp280_calib[index].dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3,index);
	_bmp280_calib[index].dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4,index);
	_bmp280_calib[index].dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5,index);
	_bmp280_calib[index].dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6,index);
	_bmp280_calib[index].dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7,index);
	_bmp280_calib[index].dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8,index);
	_bmp280_calib[index].dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9,index);
}

float readTemperature(int index)
{

	int32_t var1, var2=0;
	  if (!_sensorID[index])
	    return 0; // begin() not called yet

	int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA,index);
	adc_T >>= 4;

	var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib[index].dig_T1 << 1))) *
			((int32_t)_bmp280_calib[index].dig_T2)) >>
					11;

	var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib[index].dig_T1)) *
			((adc_T >> 4) - ((int32_t)_bmp280_calib[index].dig_T1))) >>
			12) *
			((int32_t)_bmp280_calib[index].dig_T3)) >>
					14;

	t_fine = var1 + var2;

	float T = (t_fine * 5 + 128) >> 8;
	return T / 100;
}

void setSampling(int index,uint8_t mode,
		uint8_t tempSampling,
		uint8_t pressSampling,
		uint8_t filter,
		uint8_t duration)
{

	  if (!_sensorID[index])
	    return; // begin() not called yet

	uint8_t ctrl_meas=(tempSampling << 5) | (pressSampling << 2) | mode;
	uint8_t configReg=(duration << 5) | (filter << 2) | 0;
	//
	write8(BMP280_REGISTER_CONFIG,configReg ,index);
	write8(BMP280_REGISTER_CONTROL, ctrl_meas,index);
}

float readPressure(int index)
{
	int64_t var1, var2, p;
	  if (!_sensorID[index])
	    return 0;
	// Must be done first to get the t_fine variable set up
	readTemperature(index);

	int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA,index);
	adc_P >>= 4;

	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)_bmp280_calib[index].dig_P6;
	var2 = var2 + ((var1 * (int64_t)_bmp280_calib[index].dig_P5) << 17);
	var2 = var2 + (((int64_t)_bmp280_calib[index].dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)_bmp280_calib[index].dig_P3) >> 8) +
			((var1 * (int64_t)_bmp280_calib[index].dig_P2) << 12);
	var1 =
			(((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib[index].dig_P1) >> 33;

	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)_bmp280_calib[index].dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)_bmp280_calib[index].dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib[index].dig_P7) << 4);
	return (float)p / 256;
}

bool init(int index)
{
	_sensorID[index] = read8(BMP280_REGISTER_CHIPID,index);
	if (_sensorID[index] != 0x58)
	return false;

	readCoefficients(index);

	// write8(BMP280_REGISTER_CONTROL, 0x3F); /* needed? */
	setSampling(index,MODE_NORMAL,SAMPLING_X2,SAMPLING_X16,FILTER_X16,STANDBY_MS_500);
	HAL_Delay(100);
	return true;
}

//void DoPID()
//{
//	float ki=0.00,kp=1,kd=0.00;
//
//	int error=0;
//	int intcmd;
//	int dt=10;
//	static int prevError=0;
//	float command=1200;
//	float Dterm=0;
//	static float Iterm=0;
//
//	CurrentPos=ADCResult[4];//UpdateEncoder();
//
//	if(CurrentPos<50 || CurrentPos>3900 )
//	{
//		set_motor_speed(7, 0);
//		return;
//	}
//
////	printf("adc5=%d\n",CurrentPos);
//	error=TargetPoition-CurrentPos;
//	Dterm=(error-prevError)/dt;
//	Iterm+=(error)*dt;
//	if(Iterm>1200)Iterm=1200;
//	if(Iterm<-1200)Iterm=-1200;
//
//	command=kp*error+kd*Dterm+ki*Iterm;
//
//	prevError=error;
//		if(command>1900)command=1900;
//		if(command<-1900)	command=-1900;
//		intcmd=command;
////	printf("err=%d command=%d\n", error,intcmd);
//	 HAL_Delay(10);
//	//threshold
////	if(command<300)command=300;
////	if(command>2300)command=2300;
//
//		set_motor_speed(5, command);
//
//
//}

void hal_read_adc(uint32_t * data, uint32_t length)
{

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCResult, length*sizeof(uint32_t));
	memcpy(data,ADCResult,length);
//	printf("adc=%d %d %d %d %d %d\n",adc_values[0],adc_values[1],adc_values[2],adc_values[3],adc_values[4],adc_values[5]);


}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SelectDriver(5);
  if (HAL_CAN_Start(&hcan) != HAL_OK)
     {
       /* Start Error */
 	  printf("can start error\n");
     }
   else{
 	  printf("can start ok!!!!!\n");
   }
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  CanSend(0xaa);

	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 register_adc_callback(hal_read_adc);
	 register_delay(HAL_Delay);
	 register_finger_motros_callback(set_motor_speed);
	 register_can_send(CanSendArray);
	 logic_start();

	  TIM4->CCR3=1900;
  printf("start\n");
	for(i=0;i<6;i++)
	{
		init(i+1);
		HAL_Delay(100);
	}
	  set_motor_speed(5,-100);
//	 set_motor_speed(5,1900);
  while (1)
  {
	 logic_loop();

//	  DoPID();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfil; //declare CAN filter structure
  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfil.FilterIdHigh = 0;
  canfil.FilterIdLow = 0;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;

  	HAL_CAN_ConfigFilter(&hcan, &canfil); //configure CAN filter
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 360;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 360;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	 printf("Wrong parameters value: file %s on line %d\r\n", file, line)
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
