#ifndef BMP_280_H
#define BMP_280_H

#include <stm32f1xx.h>

typedef  int32_t BMP280_S32_t;
typedef  uint32_t BMP280_U32_t;

extern BMP280_S32_t t_fine;

typedef struct
{
	uint8_t dev_address;
	I2C_HandleTypeDef * hi2c;

	// variables store NVM data from sensors (datasheet)
	uint16_t dig_T1, dig_P1;
	int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	uint8_t osrs_t, osrs_p, osrs_h,  mode,  t_sb,  filter;

	uint8_t chip_id;
	int32_t pRaw, tRaw;

	float temperature, pressure;

} BMP280_TypeDef;


// Private methods
static void _trimRead(BMP280_TypeDef * const me);
static int _BMP280_readRaw(BMP280_TypeDef * const me);
static BMP280_S32_t _BMP280_compensate_T_int32(BMP280_TypeDef * const me, BMP280_S32_t adc_T);
static BMP280_U32_t _BMP280_compensate_P_int32(BMP280_TypeDef * const me, BMP280_S32_t adc_P);

// Public methods
void BMP280_init(BMP280_TypeDef * const me, I2C_HandleTypeDef * hi2c, uint8_t device_address);
int BMP280_config(BMP280_TypeDef * const me, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter);
void BMP280_wakeUp(BMP280_TypeDef * const me);
void BMP280_measure (BMP280_TypeDef * const me);
float BMP280_getTemperature_Celc(BMP280_TypeDef * const me);
float BMP280_getPressure_Pa(BMP280_TypeDef * const me);



// Oversampling definitions
#define OSRS_OFF    	0x00
#define OSRS_1      	0x01
#define OSRS_2      	0x02
#define OSRS_4      	0x03
#define OSRS_8      	0x04
#define OSRS_16     	0x05

// MODE Definitions
#define MODE_SLEEP      0x00
#define MODE_FORCED     0x01
#define MODE_NORMAL     0x03

// Standby Time
#define T_SB_0p5    	0x00
#define T_SB_62p5   	0x01
#define T_SB_125    	0x02
#define T_SB_250    	0x03
#define T_SB_500    	0x04
#define T_SB_1000   	0x05
#define T_SB_10     	0x06
#define T_SB_20     	0x07

// IIR Filter Coefficients
#define IIR_OFF     	0x00
#define IIR_2       	0x01
#define IIR_4       	0x02
#define IIR_8       	0x03
#define IIR_16      	0x04


// REGISTERS DEFINITIONS
#define ID_REG      	0xD0
#define RESET_REG  		0xE0
#define CTRL_HUM_REG    0xF2
#define STATUS_REG      0xF3
#define CTRL_MEAS_REG   0xF4
#define CONFIG_REG      0xF5
#define PRESS_MSB_REG   0xF7



#endif
