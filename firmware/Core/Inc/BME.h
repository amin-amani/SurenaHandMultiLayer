#ifndef BME_H
#define BME_H
#include <inttypes.h>
#include <stdbool.h>
#include  "stm32f1xx_hal.h"

#define CTRLMEASREG 0x74
#define CTRLMEASVAL 0x25
#define CONFIGREG 0x75
#define CONFIGVAL 0xA0
#define CTRLHUMREG 0x72
#define CTRLHUMVAL 0x01
#define COMPTEMPPRES 0x88
#define COMPHUMINIT 0xA0
#define COMPHUMREST 0xE1
#define RAWREAD 0xF7


enum {
  BMP280_REGISTER_DIG_T1 = 0x88,
  BMP280_REGISTER_DIG_T2 = 0x8A,
  BMP280_REGISTER_DIG_T3 = 0x8C,
  BMP280_REGISTER_DIG_P1 = 0x8E,
  BMP280_REGISTER_DIG_P2 = 0x90,
  BMP280_REGISTER_DIG_P3 = 0x92,
  BMP280_REGISTER_DIG_P4 = 0x94,
  BMP280_REGISTER_DIG_P5 = 0x96,
  BMP280_REGISTER_DIG_P6 = 0x98,
  BMP280_REGISTER_DIG_P7 = 0x9A,
  BMP280_REGISTER_DIG_P8 = 0x9C,
  BMP280_REGISTER_DIG_P9 = 0x9E,
  BMP280_REGISTER_CHIPID = 0xD0,
  BMP280_REGISTER_VERSION = 0xD1,
  BMP280_REGISTER_SOFTRESET = 0xE0,
  BMP280_REGISTER_CAL26 = 0xE1, /**< R calibration = 0xE1-0xF0 */
  BMP280_REGISTER_STATUS = 0xF3,
  BMP280_REGISTER_CONTROL = 0xF4,
  BMP280_REGISTER_CONFIG = 0xF5,
  BMP280_REGISTER_PRESSUREDATA = 0xF7,
  BMP280_REGISTER_TEMPDATA = 0xFA,
};

typedef struct {
  uint16_t dig_T1; /**< dig_T1 cal register. */
  int16_t dig_T2;  /**<  dig_T2 cal register. */
  int16_t dig_T3;  /**< dig_T3 cal register. */

  uint16_t dig_P1; /**< dig_P1 cal register. */
  int16_t dig_P2;  /**< dig_P2 cal register. */
  int16_t dig_P3;  /**< dig_P3 cal register. */
  int16_t dig_P4;  /**< dig_P4 cal register. */
  int16_t dig_P5;  /**< dig_P5 cal register. */
  int16_t dig_P6;  /**< dig_P6 cal register. */
  int16_t dig_P7;  /**< dig_P7 cal register. */
  int16_t dig_P8;  /**< dig_P8 cal register. */
  int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;

enum sensor_sampling {
  /** No over-sampling. */
  SAMPLING_NONE = 0x00,
  /** 1x over-sampling. */
  SAMPLING_X1 = 0x01,
  /** 2x over-sampling. */
  SAMPLING_X2 = 0x02,
  /** 4x over-sampling. */
  SAMPLING_X4 = 0x03,
  /** 8x over-sampling. */
  SAMPLING_X8 = 0x04,
  /** 16x over-sampling. */
  SAMPLING_X16 = 0x05
};

/** Operating mode for the sensor. */
enum sensor_mode {
  /** Sleep mode. */
  MODE_SLEEP = 0x00,
  /** Forced mode. */
  MODE_FORCED = 0x01,
  /** Normal mode. */
  MODE_NORMAL = 0x03,
  /** Software reset. */
  MODE_SOFT_RESET_CODE = 0xB6
};

/** Filtering level for sensor data. */
enum sensor_filter {
  /** No filtering. */
  FILTER_OFF = 0x00,
  /** 2x filtering. */
  FILTER_X2 = 0x01,
  /** 4x filtering. */
  FILTER_X4 = 0x02,
  /** 8x filtering. */
  FILTER_X8 = 0x03,
  /** 16x filtering. */
  FILTER_X16 = 0x04
};

/** Standby duration in ms */
enum standby_duration {
  /** 1 ms standby. */
  STANDBY_MS_1 = 0x00,
  /** 62.5 ms standby. */
  STANDBY_MS_63 = 0x01,
  /** 125 ms standby. */
  STANDBY_MS_125 = 0x02,
  /** 250 ms standby. */
  STANDBY_MS_250 = 0x03,
  /** 500 ms standby. */
  STANDBY_MS_500 = 0x04,
  /** 1000 ms standby. */
  STANDBY_MS_1000 = 0x05,
  /** 2000 ms standby. */
  STANDBY_MS_2000 = 0x06,
  /** 4000 ms standby. */
  STANDBY_MS_4000 = 0x07
};


void BME280_CONFIG_SETUP(int);
void BME280_GET_COMP_VALS(int);
void BME280_GET_RAW_VALS(int index);
void BME280_CALC_FINAL_VALS(void);
void SelectSensor(int8_t index);
void SpiWrite(uint8_t index,uint8_t *data,int len);

void SPIReadWrite(uint8_t index,uint8_t *txBuffer,uint8_t txLen,uint8_t *rxBuffer,uint8_t rxLen);
void BME280_CONFIG_SETUP(int index);

void BME280_GET_COMP_VALS(int index);

void BME280_GET_RAW_VALS(int index);
void BME280_CALC_FINAL_VALS();
bool write8(int8_t reg,uint8_t val,int index);

uint8_t read8(int8_t reg,int index);

uint32_t read24(int8_t reg,int index);

int32_t read16(uint8_t reg,int index);

uint16_t read16_LE(uint8_t reg,int index);
int16_t readS16_LE(uint8_t reg,int index);

void readCoefficients(int index);

float readTemperature(int index);

void setSampling(int index,uint8_t mode,
		uint8_t tempSampling,
		uint8_t pressSampling,
		uint8_t filter,
		uint8_t duration);
float readPressure(int index);

bool init(int index);

#endif
