#include "bmp_280.h"

void BMP280_init(BMP280_TypeDef * const me, I2C_HandleTypeDef * hi2c, uint8_t device_address)
{
	me->hi2c = hi2c;
	me->dev_address = device_address;
}

static void _trimRead(BMP280_TypeDef * const me)
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


static int _BMP280_readRaw(BMP280_TypeDef * const me)
{
	uint8_t _raw_data[6];

	// Check the chip ID before reading
	HAL_I2C_Mem_Read(me->hi2c, me->dev_address, ID_REG, 1, &me->chip_id, 1, 1000);

	if (me->chip_id == 0x58)
	{
		// Read the Registers 0xF7 to 0xFE
		HAL_I2C_Mem_Read(me->hi2c, me->dev_address, PRESS_MSB_REG, 1, _raw_data, 6, HAL_MAX_DELAY);

		/* Calculate the Raw data for the parameters
		 * Here the Pressure and Temperature are in 20 bit format and humidity in 16 bit format
		 */
		me->pRaw = (_raw_data[0]<<12)|(_raw_data[1]<<4)|(_raw_data[2]>>4);
		me->tRaw = (_raw_data[3]<<12)|(_raw_data[4]<<4)|(_raw_data[5]>>4);

		return 0;
	}

	else return -1;
}





int BMP280_config(BMP280_TypeDef * const me, uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
	uint8_t _data_to_write = 0;
	uint8_t _data_check = 0;

	_trimRead(me);

	// reset the device
	if (HAL_I2C_Mem_Write(me->hi2c, me->dev_address, RESET_REG, 1, (uint8_t *) 0xB6, 1, 1000) != HAL_OK)
	{
		return -1;
	}
	HAL_Delay(100);

	// set standby and filter IIR
	_data_to_write = (t_sb << 5) | (filter << 2);

	if (HAL_I2C_Mem_Write(me->hi2c, me->dev_address, CONFIG_REG, 1,&_data_to_write , 1, 1000) != HAL_OK)
	{
		return -1;
	}
	HAL_Delay(100);

	HAL_I2C_Mem_Read(me->hi2c, me->dev_address, CONFIG_REG, 1,&_data_check , 1, 1000);
	if (_data_check != _data_to_write)
	{
		return -1;
	}
	HAL_Delay(100);


	// oversampling config for temp and pressure
	_data_to_write = (osrs_t << 5) | (osrs_p << 2) | mode;

	if (HAL_I2C_Mem_Write(me->hi2c, me->dev_address, CTRL_MEAS_REG, 1, &_data_to_write, 1, 1000) != HAL_OK)
	{
		return -1;
	}
	HAL_Delay(100);

	HAL_I2C_Mem_Read(me->hi2c, me->dev_address, CTRL_MEAS_REG, 1, &_data_check , 1, 1000);
	if (_data_check != _data_to_write)
	{
		return -1;
	}
	HAL_Delay(100);

	return 0;
}


void BMP280_wakeUp(BMP280_TypeDef * const me)
{
	uint8_t _data_to_write;
	HAL_I2C_Mem_Read(me->hi2c, me->dev_address, CTRL_MEAS_REG, 1, &_data_to_write, 1, 1000);

	_data_to_write |= MODE_FORCED;

	HAL_I2C_Mem_Write(me->hi2c, me->dev_address, CTRL_MEAS_REG, 1, &_data_to_write, 1, 1000);
	HAL_Delay(100);
}



