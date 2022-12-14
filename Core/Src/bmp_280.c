/**
  ******************************************************************************
  * @file    bmp_280.c
  * @author  I Putu Pawesi Siantika, S.T. December, 2022.
  * @brief   bmp_280 Bosch library.
  *          This is a modification from https://controllerstech.com/bme280-with-stm32/ .
  *          The modifications are: make it suitable for bmp280 device, make it OOP,
  *          added "get temperature in celcius"-method,
  *          and added "get pressure in pa" - method.
  * @datasheet : https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
  *
  *

  ==============================================================================
                     ##### How to use this library #####
  ==============================================================================
    [..]
    	Steps:
    	* Pre-code:
    		1. Include "bmp_280.c" file inside "Src" folder and "bmp_280.h" file inside "Inc" folder.
    		2. Hardware: determining I2C address:
    			*) SDIO pin is grounded        = 0x76
    			*) SDIO pin is pull up to VCC  = 0x77;
    	*code :
    		1. In "main.c" file, declare BMP280_TypeDef bmp280_attributes.
    		2. Invoke  a method " BMP280_init(&bmp280_attributes, &hi2c1, BMP280_ADDRESS)" once.
    		3. BMP280_config(&bmp280_attributes, OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16) once.
    			parameters determine over sampling feature, power mode, standby time, and IIR filter. please
    			check data sheet and bmp_280.h file for custom setting .
    		4. Invoke the measurement method "BMP280_measure(&bmp280_attributes)"
    		5. The result (temperature and pressure) can be achieved by calling method float BMP280_getTemperature_Celc().
float          BMP280_getPressure_Pa(). Temperature and Pressure are float type-data.
    [..]

  */


#include "bmp_280.h"


// global variables
BMP280_S32_t t_fine;


// constructor method
void BMP280_init(BMP280_TypeDef * const me, I2C_HandleTypeDef * hi2c, uint8_t device_address)
{
	me->hi2c = hi2c;
	me->dev_address = device_address;
}


// Private Methods
static void _trimRead(BMP280_TypeDef * const me)
{
	uint8_t _trim_data[25];

	// Read NVM from 0x88 to 0xA1
	HAL_I2C_Mem_Read(me->hi2c, me->dev_address, 0x88, 1, _trim_data, 25, HAL_MAX_DELAY);

	// Arrange the data as per the data sheet (page no. 21)
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

/*
 * @note	: This code based on the data sheet page 44.
 */
static BMP280_S32_t _BMP280_compensate_T_int32(BMP280_TypeDef * const me, BMP280_S32_t adc_T)
{
	BMP280_S32_t _var1, _var2, _T;

	_var1 = ((((adc_T>>3) - ((BMP280_S32_t)me->dig_T1<<1))) * ((BMP280_S32_t)me->dig_T2)) >> 11;
	_var2 = (((((adc_T>>4) - ((BMP280_S32_t)me->dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)me->dig_T1))) >> 12) * ((BMP280_S32_t)me->dig_T3)) >> 14;
	t_fine = _var1 + _var2;
	_T = (t_fine * 5 + 128) >> 8;
	return _T;

}

static BMP280_U32_t _BMP280_compensate_P_int32(BMP280_TypeDef * const me, BMP280_S32_t adc_P)
{
	BMP280_S32_t _var1, _var2;
	BMP280_U32_t _P;
	_var1 = ((t_fine)>>1) - (BMP280_S32_t)64000;
	_var2 = (((_var1>>2) * (_var1>>2)) >> 11 ) * ((BMP280_S32_t)me->dig_P6);
	_var2 = _var2 + ((_var1*((BMP280_S32_t)me->dig_P5))<<1);
	_var2 = (_var2>>2)+(((BMP280_S32_t)me->dig_P4)<<16);
	_var1 = (((me->dig_P3 * (((_var1>>2) * (_var1>>2)) >> 13 )) >> 3) + ((((BMP280_S32_t)me->dig_P2) * _var1)>>1))>>18;
	_var1 =((((32768+_var1))*((BMP280_S32_t)me->dig_P1))>>15);
	if (_var1 == 0)
	{
		return 0; // avoid exce_Ption caused by division by zero
	}
	_P = (((BMP280_U32_t)(((BMP280_S32_t)1048576)-adc_P)-(_var2>>12)))*3125;
	if (_P < 0x80000000)
	{
		_P = (_P << 1) / ((BMP280_U32_t)_var1);
	}
	else
	{
		_P = (_P / (BMP280_U32_t)_var1) * 2;
	}
	_var1 = (((BMP280_S32_t)me->dig_P9) * ((BMP280_S32_t)(((_P>>3) * (_P>>3))>>13)))>>12;
	_var2 = (((BMP280_S32_t)(_P>>2)) * ((BMP280_S32_t)me->dig_P8))>>13;
	_P = (BMP280_U32_t)((BMP280_S32_t)_P + ((_var1 + _var2 + me->dig_P7) >> 4));
	return _P;
}

/* End of code copied from data sheet */




/* Public Methods */

int BMP280_config(BMP280_TypeDef * const me, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
	uint8_t _data_to_write = 0;
	uint8_t _data_check = 0;

	// read NVM device and store it to struct.
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


	// over sampling configuration for temperature and pressure
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

/*
 * @note : it uses in FORCE MODE (read data once and put device to sleep).
 * 			it needs to re-invoke after called to wake the sensor from sleep and capture the data.
 */
void BMP280_wakeUp(BMP280_TypeDef * const me)
{
	uint8_t _data_to_write;
	HAL_I2C_Mem_Read(me->hi2c, me->dev_address, CTRL_MEAS_REG, 1, &_data_to_write, 1, 1000);

	_data_to_write |= MODE_FORCED;

	HAL_I2C_Mem_Write(me->hi2c, me->dev_address, CTRL_MEAS_REG, 1, &_data_to_write, 1, 1000);
	HAL_Delay(100);
}




void BMP280_measure (BMP280_TypeDef * const me)
{
	if (_BMP280_readRaw(me) == 0)
	{
		if (me->tRaw == 0x800000) me->temperature = 0; // value in case temp measurement was disabled
		else
		{
			me->temperature = (_BMP280_compensate_T_int32 (me, me->tRaw))/100.0;  // as per datasheet, the temp is x100
		}

		if (me->pRaw == 0x800000) me->pressure = 0; // value in case temp measurement was disabled
		else
		{

			me->pressure = (_BMP280_compensate_P_int32 (me, me->pRaw));  // as per datasheet, the pressure is Pa

		}
	}

	// if the device is detached
	else
	{
		me->temperature = me->pressure = 0;
	}

}


float BMP280_getTemperature_Celc(BMP280_TypeDef * const me)
{
	return me->temperature;
}

float BMP280_getPressure_Pa(BMP280_TypeDef * const me)
{
	return me->pressure;
}


