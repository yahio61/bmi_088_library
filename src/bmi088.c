/*
 * bmi088.c
 *
 *  Created on: May 6, 2024
 *      Author: yahya
 */

#include "bmi088.h"
#include "math.h"
#include "i2c.h"
#include "stdlib.h"

//#define SELFTEST_ENABLED

static uint8_t is_time_updated = 0;
static uint8_t is_starded = 0;
static uint8_t is_gyro_renewed = 0;

static int errorLine = 0;


#ifdef SELFTEST_ENABLED
#include <string.h>
#include <stdio.h>
#endif

#ifdef SELFTEST_ENABLED
static void bmi088_selfTest()
{
	HAL_Delay(100);

	HAL_StatusTypeDef retVal = HAL_OK;

	uint8_t buf[1];

	buf[0] = ACC_RESET;
	retVal |= HAL_I2C_Mem_Write(BMI_I2c, ACC_I2C_ADD, ACC_SOFTRESET, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); // Accel reset
	HAL_Delay(100);

	buf[0] = ACC_RANGE_24G;
	retVal |= HAL_I2C_Mem_Write(BMI_I2c, ACC_I2C_ADD, ACC_RANGE, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //acc range set to 24G

	buf[0] = (0x01 << 7) | (0x02 << 4) | ACC_ODR_1600;
	retVal |= HAL_I2C_Mem_Write(BMI_I2c, ACC_I2C_ADD, ACC_CONF, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //acc some configs
	HAL_Delay(4); // wait for 3 ms

	//positive self test for accel
	buf[0] = ACC_SELF_TEST_POSITIVE;
	retVal |= HAL_I2C_Mem_Write(BMI_I2c, ACC_I2C_ADD, ACC_SELF_TEST, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //acc positive self test activated
	HAL_Delay(70); // wait for min 50ms

	retVal |= HAL_I2C_Mem_Read(BMI_I2c, ACC_I2C_ADD, ACC_X_LSB, I2C_MEMADD_SIZE_8BIT, BMI->rawDatas.accel, 6, 20);
	BMI->testVals.selfTest_acc_z = (BMI->rawDatas.accel[5] << 8) | BMI->rawDatas.accel[4];
	BMI->testVals.selfTest_acc_y = (BMI->rawDatas.accel[3] << 8) | BMI->rawDatas.accel[2];
	BMI->testVals.selfTest_acc_x = (BMI->rawDatas.accel[1] << 8) | BMI->rawDatas.accel[0];

	BMI->testVals.acc_z[0] = (double)BMI->testVals.selfTest_acc_z / 32768.0 * 1000.0 * 1.5 * pow(2.0, (double)(ACC_RANGE_24G + 1));
	BMI->testVals.acc_y[0] = (double)BMI->testVals.selfTest_acc_y / 32768.0 * 1000.0 * 1.5 * pow(2.0, (double)(ACC_RANGE_24G + 1));
	BMI->testVals.acc_x[0] = (double)BMI->testVals.selfTest_acc_x / 32768.0 * 1000.0 * 1.5 * pow(2.0, (double)(ACC_RANGE_24G + 1));

	//negative self test for accel
	buf[0] = ACC_SELF_TEST_NEGATIVE;
	retVal |= HAL_I2C_Mem_Write(BMI_I2c, ACC_I2C_ADD, ACC_SELF_TEST, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //acc negative self test activated
	HAL_Delay(70); // wait for min 50ms

	retVal |= HAL_I2C_Mem_Read(BMI_I2c, ACC_I2C_ADD, ACC_X_LSB, I2C_MEMADD_SIZE_8BIT, BMI->rawDatas.accel, 6, 20);
	BMI->testVals.selfTest_acc_z = (BMI->rawDatas.accel[5] << 8) | BMI->rawDatas.accel[4];
	BMI->testVals.selfTest_acc_y = (BMI->rawDatas.accel[3] << 8) | BMI->rawDatas.accel[2];
	BMI->testVals.selfTest_acc_x = (BMI->rawDatas.accel[1] << 8) | BMI->rawDatas.accel[0];

	BMI->testVals.acc_z[1] = (float)BMI->testVals.selfTest_acc_z / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(ACC_RANGE_24G + 1));
	BMI->testVals.acc_y[1] = (float)BMI->testVals.selfTest_acc_y / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(ACC_RANGE_24G + 1));
	BMI->testVals.acc_x[1] = (float)BMI->testVals.selfTest_acc_x / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(ACC_RANGE_24G + 1));

	buf[0] = ACC_SELF_TEST_OFF;
	retVal |= HAL_I2C_Mem_Write(BMI_I2c, ACC_I2C_ADD, ACC_SELF_TEST, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //acc self test off
	HAL_Delay(70);	//wait for steady state.
	if(retVal != HAL_OK)
		Error_Handler();



#ifdef SELFTEST_ENABLED
	bmi088_selfTest();
	uint8_t buffer[100];
	sprintf((char*)buffer, "self test positive:  a_x: %f  a_y: %f  a_z: %f\r\n", BMI->testVals.acc_x[0], BMI->testVals.acc_y[0], BMI->testVals.acc_z[0]);
	HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer), 50);
	sprintf((char*)buffer, "self test negative:  a_x: %f  a_y: %f  a_z: %f\r\n", BMI->testVals.acc_x[1], BMI->testVals.acc_y[1], BMI->testVals.acc_z[1]);
	HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer), 50);
	while(1);
#endif

}
#endif

#define GYRO_X_OFFSET (float)0.0
#define GYRO_Y_OFFSET (float)0.0
#define GYRO_Z_OFFSET (float)0.0
/*
static void bmi088_poke()
{

	HAL_I2C_DeInit(BMI_I2c);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);
	HAL_Delay(1000);
	HAL_I2C_Init(BMI_I2c);

}
*/
uint8_t bmi088_init(bmi088_struct_t* BMI)
{
	//quaternionSet_zero();
	uint8_t ret_val = 0;
	BMI->flags.isGyroUpdated = 0;
	BMI->flags.isAccelUpdated = 0;
	is_time_updated = 0;
	is_starded = 0;
	uint8_t buf[2];
	buf[0] = 0;

	if(BMI->device_config.offsets == NULL)
	{
		BMI->device_config.offsets = calloc(sizeof(*BMI->device_config.offsets), 1);

	}

	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_CHIP_ID, I2C_MEMADD_SIZE_8BIT, buf, 1, 50);
	ret = HAL_I2C_Mem_Read(BMI->device_config.BMI_I2c, GYRO_I2C_ADD, GYRO_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &buf[1], 1, 50);
	UNUSED(ret);
	if(!(buf[0] == 0x1E))
	{
		ret_val = ret_val | 0x01;
	}
	if(!(buf[1] == 0x0F))
	{
		ret_val = ret_val | 0x02;
	}

	return ret_val;
}

void bmi088_config(bmi088_struct_t* BMI)
{
	HAL_NVIC_DisableIRQ(BMI->device_config.acc_IRQ);
	HAL_NVIC_DisableIRQ(BMI->device_config.gyro_IRQ);

	HAL_StatusTypeDef retVal = HAL_OK;
	uint8_t buf[1];

	buf[0] = ACC_PWR_SAVE_ULTRA;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_PWR_CONF, I2C_MEMADD_SIZE_8BIT, buf, 1, 100); // power save ultra

	buf[0] = ACC_DISABLE;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_PWR_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 100); // accel disable
	HAL_Delay(20);

	buf[0] = ACC_RESET;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_SOFTRESET, I2C_MEMADD_SIZE_8BIT, buf, 1, 100); // Accel reset
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(10);

	buf[0] = FIFO_RESET;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_SOFTRESET, I2C_MEMADD_SIZE_8BIT, buf, 1, 100); // FIFO reset
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(10);

	buf[0] = GYRO_RESET;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, GYRO_I2C_ADD, GYRO_SOFT_RESET, I2C_MEMADD_SIZE_8BIT, buf, 1, 100); //Gyro reset
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(10);

	//Gyroscope configuration.
	buf[0] = BMI->device_config.gyro_range;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, GYRO_I2C_ADD, GYRO_RANGE, I2C_MEMADD_SIZE_8BIT, buf, 1, 100); //Gyro range config
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = BMI->device_config.gyro_bandWidth;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, GYRO_I2C_ADD, GYRO_BANDWITH, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro bandwidth config
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = BMI->device_config.gyro_powerMode;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, GYRO_I2C_ADD, GYRO_LPM1, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro power mode config.
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(20);

	//gyro interrupt
	buf[0] = GYRO_INT_ENABLE;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, GYRO_I2C_ADD, GYRO_INT_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro interrupt enabled.
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = (GYRO_INT_IO_PP << 1) | (GYRO_INT_ACT_HIGH << 0) | (GYRO_INT_ACT_HIGH << 2);
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, GYRO_I2C_ADD, GYRO_INT_3_4_IO_CONF, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro interrupt 3 config
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = GYRO_INT_MAP_BOTH;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, GYRO_I2C_ADD, GYRO_INT_3_4_IO_MAP, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //Gyro interrupt pin 3 mapped.
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	//Accelerometer configuration.
	buf[0] = ACC_ENABLE;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_PWR_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); // Accel on
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(8);

	buf[0] = BMI->device_config.acc_powerMode;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_PWR_CONF, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel mode active
	retVal != HAL_OK ? errorLine =__LINE__ : 0;
	HAL_Delay(8);

	buf[0] = (BMI->device_config.acc_bandwith << 4) | BMI->device_config.acc_outputDateRate;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_CONF, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel bandwith and odr selection
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = BMI->device_config.acc_range;
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_RANGE, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel range config.
	retVal != HAL_OK ? errorLine =__LINE__ : 0;

	//accel interrupt
	buf[0] = (0x01 << 3) | (ACC_INT1_OD_PP << 2) | (ACC_INT1_LVL_ACT_HIGH << 1);
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_INT1_IO_CTRL, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel interrupt config.
	//retVal != HAL_OK ? errorLine =__LINE__ : 0;

	buf[0] = (0x01 << 2);
	retVal |= HAL_I2C_Mem_Write(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_INT_MAP_DATA, I2C_MEMADD_SIZE_8BIT, buf, 1, 20); //accel interrupt DRDY map to pin1.
	//retVal != HAL_OK ? errorLine =__LINE__ : 0;

	HAL_NVIC_EnableIRQ(BMI->device_config.acc_IRQ);
	HAL_NVIC_EnableIRQ(BMI->device_config.gyro_IRQ);
	HAL_Delay(70);
}


void bmi088_update(bmi088_struct_t* BMI)
{
	HAL_StatusTypeDef ret_val = HAL_OK;

		if(BMI->flags.isAccelUpdated)
		{
			uint8_t raw_accel[9];
			uint8_t	raw_temp[2];

			ret_val = HAL_I2C_Mem_Read(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_X_LSB, I2C_MEMADD_SIZE_8BIT, raw_accel, 9, 20);
			ret_val = HAL_I2C_Mem_Read(BMI->device_config.BMI_I2c, ACC_I2C_ADD, ACC_TEMP_MSB, I2C_MEMADD_SIZE_8BIT, raw_temp, 2, 20);

			uint16_t Temp_uint11 = (raw_temp[0] << 3) | (raw_temp[1] >> 5);
			int16_t Temp_int11 = 0;
			if (Temp_uint11 > 1023){
				Temp_int11 = Temp_uint11 - 2048;
			}
			else{
				Temp_int11 = Temp_uint11;
				BMI->datas.temp = (float)Temp_int11 * 0.125 + 23.0;
			}
			uint32_t sensorTime = (raw_accel[8] << 16) | (raw_accel[7] << 8) | raw_accel[6];

			BMI->datas.current_time= (float)sensorTime * 39.0625 / 1000000.0;

			int16_t acc_z_16 = (raw_accel[5] << 8) | raw_accel[4];
			int16_t acc_y_16 = (raw_accel[3] << 8) | raw_accel[2];
			int16_t acc_x_16 = (raw_accel[1] << 8) | raw_accel[0];

			BMI->datas.acc_z = (float)acc_z_16 / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(BMI->device_config.acc_range + 1)) - ACCEL_Z_OFFSET;
			BMI->datas.acc_y = (float)acc_y_16 / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(BMI->device_config.acc_range + 1)) - ACCEL_Y_OFFSET;
			BMI->datas.acc_x = (float)acc_x_16 / 32768.0 * 1000.0 * 1.5 * pow(2.0, (float)(BMI->device_config.acc_range + 1)) - ACCEL_X_OFFSET;

			if(is_starded)
			{
				BMI->datas.delta_time = BMI->datas.current_time - BMI->datas.last_time < 0 ? 0.0 : BMI->datas.current_time - BMI->datas.last_time;
			}
			else
			{
				is_starded = 1;
			}

			BMI->datas.last_time = BMI->datas.current_time;
			BMI->flags.isAccelUpdated = 0;
			is_time_updated = 1;
		}

		if(BMI->flags.isGyroUpdated && is_time_updated)
		{
			if(is_starded){
				uint8_t	raw_gyro[6];
				ret_val = HAL_I2C_Mem_Read(BMI->device_config.BMI_I2c, GYRO_I2C_ADD, GYRO_RATE_X_LSB, I2C_MEMADD_SIZE_8BIT, raw_gyro, 6, 10);

				if(ret_val)
					return;

				int16_t gyro_x_16 = (raw_gyro[1] << 8) | raw_gyro[0];
				int16_t gyro_y_16 = (raw_gyro[3] << 8) | raw_gyro[2];
				int16_t gyro_z_16 = (raw_gyro[5] << 8) | raw_gyro[4];
/*
				BMI->delta_angle_z = (((float)gyro_z_16 / 32767.0) * (float)(2000 >> BMI->device_config.gyro_range) - g_offset[0][2]) * BMI->deltaTime;
				BMI->delta_angle_y = (((float)gyro_y_16 / 32767.0) * (float)(2000 >> BMI->device_config.gyro_range) - g_offset[0][1]) * BMI->deltaTime;
				BMI->delta_angle_x = (((float)gyro_x_16 / 32767.0) * (float)(2000 >> BMI->device_config.gyro_range) - g_offset[0][0]) * BMI->deltaTime;
*/
				BMI->datas.gyro_x = (((float)gyro_x_16 / 32767.0) * (float)(2000 >> BMI->device_config.gyro_range) - BMI->device_config.offsets->gyro_offset[0]);
				BMI->datas.gyro_y = (((float)gyro_y_16 / 32767.0) * (float)(2000 >> BMI->device_config.gyro_range) - BMI->device_config.offsets->gyro_offset[1]);
				BMI->datas.gyro_z = (((float)gyro_z_16 / 32767.0) * (float)(2000 >> BMI->device_config.gyro_range) - BMI->device_config.offsets->gyro_offset[2]);

				BMI->datas.gyro_x_angle += (BMI->datas.gyro_x) * BMI->datas.delta_time;
				BMI->datas.gyro_y_angle += (BMI->datas.gyro_y) * BMI->datas.delta_time;
				BMI->datas.gyro_z_angle += (BMI->datas.gyro_z) * BMI->datas.delta_time;

				//update_quaternion(q, BMI->gyro_x, BMI->gyro_y, BMI->gyro_z, BMI->deltaTime);
				//calculateQuaternion(q, BMI->gyro_x, BMI->gyro_y, BMI->gyro_z, BMI->deltaTime, vector);

				/*
				updateQuaternion(-BMI->gyro_z * M_PI / 180.0, BMI->gyro_x * M_PI / 180.0, -BMI->gyro_y * M_PI / 180.0, BMI->deltaTime);
				quaternionToEuler();
				*/
				is_gyro_renewed = 1;
			}
			BMI->flags.isGyroUpdated = 0;
			is_time_updated = 0;
		}
}


void bmi088_set_accel_INT(bmi088_struct_t* BMI)
{
	BMI->flags.isAccelUpdated = 1;
}

void bmi088_set_gyro_INT(bmi088_struct_t* BMI)
{
	BMI->flags.isGyroUpdated = 1;
}

uint8_t bmi088_getGyroChipId(bmi088_struct_t* BMI)
{
	uint8_t data = 0;
	HAL_I2C_Mem_Read(BMI->device_config.BMI_I2c, GYRO_I2C_ADD, GYRO_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);
	return data;
}

void get_offset(bmi088_struct_t* BMI)
{
	int offsetCounter = 0;

	while(1)
	{
		bmi088_update(BMI);
		if(is_gyro_renewed == 1)
		{
			if(offsetCounter < 1000)
			{
				BMI->device_config.offsets->gyro_offset[0] += BMI->datas.gyro_x;
				BMI->device_config.offsets->gyro_offset[1] += BMI->datas.gyro_y;
				BMI->device_config.offsets->gyro_offset[2] += BMI->datas.gyro_z;
				offsetCounter++;
			}
			else
			{
				BMI->device_config.offsets->gyro_offset[0] /= 1000.0;
				BMI->device_config.offsets->gyro_offset[1] /= 1000.0;
				BMI->device_config.offsets->gyro_offset[2] /= 1000.0;
				//quaternionSet_zero();
				break;
				//Error_Handler();
			}
			is_gyro_renewed = 0;
		}

	}
}



