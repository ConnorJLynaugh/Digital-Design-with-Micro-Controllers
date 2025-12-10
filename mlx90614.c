#include "mlx90614.h"
#include "mlx90614Config.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <string.h>

#define mlx90614_delay(x)   sleep_ms(x)

// Minimal HAL compatibility layer for Pico SDK
typedef enum {
  HAL_OK = 0,
  HAL_ERROR = 1
} HAL_StatusTypeDef;

#define I2C_MEMADD_SIZE_8BIT 1

#define MLX90614_REGISTER_TA      0x06
#define MLX90614_REGISTER_TOBJ1	  0x07
#define MLX90614_REGISTER_TOBJ2	  0x08
#define MLX90614_REGISTER_TOMAX   0x20
#define MLX90614_REGISTER_TOMIN   0x21
#define MLX90614_REGISTER_PWMCTRL 0x22
#define MLX90614_REGISTER_TARANGE 0x23
#define MLX90614_REGISTER_KE      0x24
#define MLX90614_REGISTER_CONFIG  0x25
#define MLX90614_REGISTER_ADDRESS 0x2E
#define MLX90614_REGISTER_ID0     0x3C
#define MLX90614_REGISTER_ID1     0x3D
#define MLX90614_REGISTER_ID2     0x3E
#define MLX90614_REGISTER_ID3     0x3F
#define MLX90614_REGISTER_SLEEP   0xFF

// Pico SDK replacements for the STM32 HAL helpers used by the original driver
static HAL_StatusTypeDef HAL_I2C_IsDeviceReady(i2c_inst_t *i2c, uint8_t addr8, uint32_t trials, uint32_t timeout_ms)
{
  uint8_t addr7 = addr8 >> 1;
  uint8_t reg = MLX90614_REGISTER_TA;

  while (trials--)
  {
    int ret = i2c_write_timeout_us(i2c, addr7, &reg, 1, false, timeout_ms * 1000);
    if (ret == 1)
      return HAL_OK;
    sleep_ms(2);
  }

  return HAL_ERROR;
}

static HAL_StatusTypeDef HAL_I2C_Mem_Read(i2c_inst_t *i2c, uint8_t addr8, uint16_t mem_addr, uint16_t mem_add_size, uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  (void)mem_add_size;
  uint8_t addr7 = addr8 >> 1;
  uint8_t reg = (uint8_t)mem_addr;

  int ret = i2c_write_timeout_us(i2c, addr7, &reg, 1, true, timeout_ms * 1000);
  if (ret != 1)
    return HAL_ERROR;

  ret = i2c_read_timeout_us(i2c, addr7, data, len, false, timeout_ms * 1000);
  return (ret == (int)len) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef HAL_I2C_Mem_Write(i2c_inst_t *i2c, uint8_t addr8, uint16_t mem_addr, uint16_t mem_add_size, uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  (void)mem_add_size;

  // The driver only ever writes 3 bytes at a time (LSB, MSB, PEC)
  if (len > 3)
    return HAL_ERROR;

  uint8_t addr7 = addr8 >> 1;
  uint8_t buffer[4];
  buffer[0] = (uint8_t)mem_addr;
  memcpy(&buffer[1], data, len);

  int ret = i2c_write_timeout_us(i2c, addr7, buffer, len + 1, false, timeout_ms * 1000);
  return (ret == (int)(len + 1)) ? HAL_OK : HAL_ERROR;
}

MLX90614_t  mlx90614;
//###################################################################################################
uint8_t mlx90614_crc8(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  while (len--)
  {
    uint8_t inbyte = *data++;
    for (uint8_t i = 8; i; i--)
    {
      uint8_t carry = (crc ^ inbyte) & 0x80;
      crc <<= 1;
      if (carry)
        crc ^= 0x7;
      inbyte <<= 1;
    }
  }
  return crc;
}
//###################################################################################################
bool mlx90614_read16(uint8_t address, int16_t *data)
{
  uint8_t d[3];
  if(HAL_I2C_Mem_Read(_MLX90614_I2C, _MLX90614_I2C_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, d, 3, 100) != HAL_OK)
    return false;
  *data = d[0] | (d[1] << 8);
  return true;  
}
//###################################################################################################
bool mlx90614_write16(uint8_t address, int16_t data)
{
  uint8_t d[5];
  d[0] = _MLX90614_I2C_ADDRESS;
  d[1] = address;
  d[2] = 0;
  d[3] = 0;
  d[4] = mlx90614_crc8(d, 4);
  if(HAL_I2C_Mem_Write(_MLX90614_I2C, _MLX90614_I2C_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, &d[2], 3, 100) != HAL_OK)
    return false;
  mlx90614_delay(10);
  d[2] = data & 0x00FF;
  d[3] = data >> 8;
  d[4] = mlx90614_crc8(d, 4);    
  mlx90614_delay(10);
  if(HAL_I2C_Mem_Write(_MLX90614_I2C, _MLX90614_I2C_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, &d[2], 3, 100) != HAL_OK)
    return false;
  mlx90614_delay(10);
  int16_t comp = 0;
  if(mlx90614_read16(address, &comp) == false)
    return false;
  if(comp != data)
    return false;
  return true;
}
//###################################################################################################
int16_t mlx90614_calcRawTemp(float calcTemp)
{
	int16_t rawTemp; 
	if (mlx90614.unit == MLX90614_UNIT_RAW)
	{
		rawTemp = (int16_t) calcTemp;
	}
	else
	{
		float tempFloat;
		if (mlx90614.unit == MLX90614_UNIT_F)
		{
			tempFloat = (calcTemp - 32.0f) * 5.0f / 9.0f + 273.15f;
		}
		else if (mlx90614.unit == MLX90614_UNIT_C)
		{
			tempFloat = calcTemp + 273.15f;
		}
		else if (mlx90614.unit == MLX90614_UNIT_K)
		{
			tempFloat = calcTemp;
		}
		tempFloat *= 50.0f;
		rawTemp = (int16_t) tempFloat;
	}
	return rawTemp;
}
//###################################################################################################
float mlx90614_calcTemperature(int16_t rawTemp)
{
	float retTemp;
	if(mlx90614.unit == MLX90614_UNIT_RAW)
	{
		retTemp = (float) rawTemp;
	}
	else
	{
		retTemp = (float)(rawTemp) * 0.02f;
    if(mlx90614.unit != MLX90614_UNIT_K)
		{
			retTemp -= 273.15f;
			if(mlx90614.unit == MLX90614_UNIT_F)
			{
				retTemp = retTemp * 9.0f / 5.0f + 32.0f;
			}
		}
	}	
	return retTemp;
}
//###################################################################################################
bool mlx90614_init(void)
{
  memset(&mlx90614, 0, sizeof(mlx90614)); 
  mlx90614.unit = MLX90614_UNIT_C;
  if(HAL_I2C_IsDeviceReady(_MLX90614_I2C, _MLX90614_I2C_ADDRESS, 1, 100) != HAL_OK)
    return false;  
  mlx90614_read16(MLX90614_REGISTER_CONFIG, (int16_t*)&mlx90614.configReg);  
  if(mlx90614_readID(NULL) == false)
    return false;
  if(mlx90614_getEmissivity(NULL) == false)
    return false;
  if(mlx90614_getMax(NULL) == false)
    return false;
  if(mlx90614_getMin(NULL) == false)
    return false;  
  
  return true;
}
//###################################################################################################
void mlx90614_setUnit(MLX90614_UNIT_t MLX90614_UNIT_)
{
  mlx90614.unit = MLX90614_UNIT_;
}
//###################################################################################################
bool mlx90614_readID(int16_t *id)
{	
	for (int i=0; i<4; i++)
	{
		int16_t temp = 0;
		if (!mlx90614_read16(MLX90614_REGISTER_ID0 + i, &temp))
			return false;
    if(id != NULL)
      id[i] = (uint16_t)temp;
    mlx90614.id[i] = (uint16_t)temp;
	}
	return true;
}
//###################################################################################################
bool mlx90614_getEmissivity(float *emissivity)
{
	if(mlx90614_read16(MLX90614_REGISTER_KE, &mlx90614.rawEmissivity))
	{
    mlx90614.emissivity = (((float)((uint16_t)mlx90614.rawEmissivity)) / 65535.0f);
		if(emissivity != NULL)
      *emissivity = (((float)((uint16_t)mlx90614.rawEmissivity)) / 65535.0f);
		return true;
	}
	return false;
}
//###################################################################################################
bool mlx90614_setEmissivity(float emissivity)
{
	if ((emissivity > 1.0f) || (emissivity < 0.1f))
		return false;
	mlx90614.rawEmissivity = (uint16_t)(65535.0f * emissivity);
	if(mlx90614.rawEmissivity < 0x2000)
    mlx90614.rawEmissivity = 0x2000;
	return mlx90614_write16(MLX90614_REGISTER_KE, (int16_t)mlx90614.rawEmissivity);
}
//###################################################################################################
bool mlx90614_setMax(float maxTemp)
{
	mlx90614.rawMax = mlx90614_calcRawTemp(maxTemp);
	return mlx90614_write16(MLX90614_REGISTER_TOMAX, mlx90614.rawMax);
}
//###################################################################################################
bool mlx90614_setMin(float minTemp)
{
	mlx90614.rawMin = mlx90614_calcRawTemp(minTemp);
	return mlx90614_write16(MLX90614_REGISTER_TOMIN, mlx90614.rawMin);
}
//###################################################################################################
bool mlx90614_getMax(float *maxTemp)
{
	if(mlx90614_read16(MLX90614_REGISTER_TOMAX, &mlx90614.rawMax))
	{
    if(maxTemp != NULL)
      *maxTemp = mlx90614.rawMax;
		return true;
	}
	return false;
}
//###################################################################################################
bool mlx90614_getMin(float *minTemp)
{
	if(mlx90614_read16(MLX90614_REGISTER_TOMIN, &mlx90614.rawMin))
	{
    if(minTemp != NULL)
      *minTemp = mlx90614.rawMin;
		return true;
	}
	return false;
}
//###################################################################################################
bool mlx90614_getAmbient(float *ambientTemp)
{
	if (mlx90614_read16(MLX90614_REGISTER_TA, &mlx90614.rawAmbient))
	{
    if(ambientTemp != NULL)
      *ambientTemp = mlx90614_calcTemperature(mlx90614.rawAmbient);
		return true;
	}
	return false; 
}
//###################################################################################################
bool mlx90614_getObject1(float *objectTemp)
{
	if(mlx90614_read16(MLX90614_REGISTER_TOBJ1, &mlx90614.rawObject1))
	{
		if(mlx90614.rawObject1 & 0x8000)
			return false; 
		if(objectTemp != NULL)
      *objectTemp =  mlx90614_calcTemperature(mlx90614.rawObject1);
		return true;
	}
	return false;	
}
//###################################################################################################
bool mlx90614_getObject2(float *objectTemp)
{
	if(mlx90614_read16(MLX90614_REGISTER_TOBJ2, &mlx90614.rawObject2))
	{
		if(mlx90614.rawObject2 & 0x8000)
			return false; 
		if(objectTemp != NULL)
      *objectTemp =  mlx90614_calcTemperature(mlx90614.rawObject2);
		return true;
	}
	return false;	
}
//###################################################################################################
bool mlx90614_ping(void)
{
  return HAL_I2C_IsDeviceReady(_MLX90614_I2C, _MLX90614_I2C_ADDRESS, 1, 100) == HAL_OK;
}
//###################################################################################################
bool mlx90614_read_both_temps(float *objectTemp, float *ambientTemp)
{
  float obj, amb;

  if(mlx90614_getObject1(&obj) == false)
    return false;
  if(mlx90614_getAmbient(&amb) == false)
    return false;

  if(objectTemp != NULL)
    *objectTemp = obj;
  if(ambientTemp != NULL)
    *ambientTemp = amb;
  return true;
}
//###################################################################################################
float mlx90614_celsius_to_fahrenheit(float celsius)
{
  return (celsius * 9.0f / 5.0f) + 32.0f;
}
//###################################################################################################
