#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"


#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "bmi088.h"

typedef struct bckp_sram_datas
{
	bmi088_offsets_t offsets;
}bckp_sram_datas_t;

#define IMU_I2C_HNDLR	hi2c3 //put your I2C handler
#define TTL_HNDLR		huart1  //put your UART handler

bckp_sram_datas_t* backup_datas = (bckp_sram_datas_t*) BKPSRAM_BASE;
bmi088_struct_t bmi_imu_s;
uint8_t str[200];

uint8_t bmi_imu_init(void);
void serial_println(char* str);

int main(void)
{
  __disable_irq();

  //For turning on the backup sram.
  HAL_PWR_EnableBkUpAccess();
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
  HAL_PWR_EnableBkUpReg();

  HAL_NVIC_DisableIRQ(EXTI4_IRQn);
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);

  __enable_irq();

  HAL_Delay(10);

  uint8_t bmi_ret = bmi_imu_init();
  if(bmi_ret)
  {
    serial_println("bmi fail");
    if((bmi_ret & 0x01) == 0x01)
    {
      sprintf((char*)str, "error in accel");
      serial_println((char*) str);
    }
    if((bmi_ret & 0x02) == 0x02)
    {
      sprintf((char*)str, "error in gyro");
      serial_println((char*) str);
    }

  }
  else
  {
    serial_println("bmi success");
  }

  bmi088_config(&bmi_imu_s);
  get_offset(&bmi_imu_s);

  while (1)
  {

	  bmi088_update(&bmi_imu_s);	//this funstion called continuosly in while. It takes and updates IMU datas.

	  sprintf((char*)str, "acc_x= %f \t acc_y= %f \t acc_z= %f \t gyro_x= %f \t gyro_y= %f \t gyro_z= %f \t temp= %f",
			  bmi_imu_s.datas.acc_x, bmi_imu_s.datas.acc_y, bmi_imu_s.datas.acc_z,
			  bmi_imu_s.datas.gyro_x, bmi_imu_s.datas.gyro_y, bmi_imu_s.datas.gyro_z,
			  bmi_imu_s.datas.temp);
			  
	  serial_println((char*)str);
	  HAL_Delay(200);
  }

}


//BMI sensor struct filled with configuration settings. Then called bmi088_init function.
uint8_t bmi_imu_init(void)
{
	//Acc config
	bmi_imu_s.device_config.acc_bandwith = ACC_BWP_OSR4;
	bmi_imu_s.device_config.acc_outputDateRate = ACC_ODR_200;
	bmi_imu_s.device_config.acc_powerMode = ACC_PWR_SAVE_ACTIVE;
	bmi_imu_s.device_config.acc_range = ACC_RANGE_24G;

	// Gyro config
	bmi_imu_s.device_config.gyro_bandWidth = GYRO_BW_116;
	bmi_imu_s.device_config.gyro_range = GYRO_RANGE_2000;
	bmi_imu_s.device_config.gyro_powerMode = GYRO_LPM_NORMAL;

	bmi_imu_s.device_config.acc_IRQ = EXTI4_IRQn;
	bmi_imu_s.device_config.gyro_IRQ = EXTI3_IRQn;
	bmi_imu_s.device_config.BMI_I2c = &IMU_I2C_HNDLR;
	bmi_imu_s.device_config.offsets = &backup_datas->offsets;	//Offset datas stored in backup sram for saving them unwanted reset.

	return	bmi088_init(&bmi_imu_s);
}

void serial_println(char* str)
{

	HAL_UART_Transmit(&TTL_HNDLR, (uint8_t*)str, strlen(str), 50);
	HAL_UART_Transmit(&TTL_HNDLR, (uint8_t*)"\r\n", 2, 50);
}

//Bmi interrupt flags setted in this callback function.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == INT_ACC_Pin)
	{
		bmi088_set_accel_INT(&bmi_imu_s);
	}
	if(GPIO_Pin == INT_GYRO_Pin)
	{
		bmi088_set_gyro_INT(&bmi_imu_s);
	}
}

