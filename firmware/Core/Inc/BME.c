/*
 * BME.c
 *
 *  Created on: Feb 15, 2024
 *      Author: amin
 */
#include "BME.h"
static uint8_t _sensorID[7];
static bmp280_calib_data _bmp280_calib[7];
static int32_t t_fine;
static int32_t p_fine;
static float tfloat;
static volatile int temperature_raw, pressure_raw, humidity_raw = 0;
static int finaltemp = 0;
static uint32_t finalpressure, final_humidity = 0;
static unsigned char dig_H1, dig_H3;
static signed char dig_H6;
static unsigned short dig_T1, dig_P1;
static uint8_t tempread[8];
static uint8_t config[2];
static uint8_t comp[32];
static signed short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;

uint8_t  (*spi_transmit)(uint8_t* data, uint16_t len, uint32_t timeout);
uint8_t  (*spi_receive)( uint8_t *data, uint16_t len, uint32_t timeout);


void SpiWrite(uint8_t index,uint8_t *data,int len)
{
	int offset=-1;
	//if(index==1)offset=1;
	SelectSensor(index+offset);
//	HAL_SPI_Transmit(&hspi1, data, len, 1000); //CONFIG
	spi_transmit(data, len, 1000);
	SelectSensor(index);
}

void SPIReadWrite(uint8_t index,uint8_t *txBuffer,uint8_t txLen,uint8_t *rxBuffer,uint8_t rxLen)
{
	HAL_Delay(5);
	int offset=-1;
	//if(index==1)offset=1;
	SelectSensor(index+offset);
//	HAL_SPI_Transmit(&hspi1, txBuffer, txLen, 10);
	spi_transmit(txBuffer,txLen,10);
//	HAL_SPI_Receive(&hspi1, rxBuffer, rxLen, 200);
	spi_receive( rxBuffer, rxLen, 200);
	SelectSensor(index);
	HAL_Delay(5);

}

void register_spi_transmit_function( uint8_t (*spi_transmit_callback)(uint8_t* data, uint16_t len, uint32_t timeout))
{
	spi_transmit=spi_transmit_callback;
}


void register_spi_receive_function( uint8_t (*spi_receive_callback)(uint8_t* data, uint16_t len, uint32_t timeout))
{
	spi_receive=spi_receive_callback;
}
void SelectSensor(int8_t index)
{

	GPIOB->ODR &= ~ ((0x07)<<9);
	GPIOB->ODR |= ((0x07 & index)<<9);
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


void BME280_GET_RAW_VALS(int index)
{
	BME280_CONFIG_SETUP(index);

	config[0] = RAWREAD;

	SPIReadWrite(index, config, 1, tempread, 8);

	temperature_raw =(tempread[3]<<12)+(tempread[4]<<4)+(tempread[5]>>4);
	pressure_raw = (tempread[0]<<12)+(tempread[1]<<4)+(tempread[2]>>4);
	humidity_raw = (tempread[6] << 8) + (tempread[7]);
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



bool bmp_sensors_init(int index)
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


