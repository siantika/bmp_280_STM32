#include "bmp_280.h"

void BMP280_init(BMP280_TypeDef * const me, I2C_HandleTypeDef * hi2c, uint8_t device_address)
{
	me->hi2c = hi2c;
	me->dev_address = device_address;
}

void _trimRead(BMP280_TypeDef * const me)
{
	uint8_t _trim_data[25];

	// Read NVM from 0x88 to 0xA1
	HAL_I2C_Mem_Read(me->hi2c, me->dev_address, 0x88, 1, _trim_data, 25, HAL_MAX_DELAY);

	// Arrange the data as per the datasheet (page no. 21)
	me->dig_T1 = (_trim_data[1]<<8) | _trim_data[0];
	me->dig_T2 = (_trim_data[3]<<8) | _trim_data[2];
	me->dig_T3 = (_trim_data[5]<<8) | _trim_data[4];
	me->dig_P1 = (_trim_data[7]<<8) | _trim_data[5];
	me->dig_P2 = (_trim_data[9]<<8) | _trim_data[6];
	me->dig_P3 = (_trim_data[11]<<8) | _trim_data[10];
	me->dig_P4 = (_trim_data[13]<<8) | _trim_data[12];
	me->dig_P5 = (_trim_data[15]<<8) | _trim_data[14];
	me->dig_P6 = (_trim_data[17]<<8) | _trim_data[16];
	me->dig_P7 = (_trim_data[19]<<8) | _trim_data[18];
	me->dig_P8 = (_trim_data[21]<<8) | _trim_data[20];
	me->dig_P9 = (_trim_data[23]<<8) | _trim_data[22];
}


