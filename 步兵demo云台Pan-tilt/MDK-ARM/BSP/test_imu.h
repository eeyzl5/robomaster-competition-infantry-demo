/**
  *@file test_imu.h
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#ifndef _TEST__IMU_H
#define _TEST__IMU_H

#include "stm32f4xx_hal.h"
#include "main.h"

#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

#define M_PI  (float)3.1415926535

extern volatile float yaw_angle,pitch_angle,roll_angle; //使用到的角度值

typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;

extern uint8_t MPU_id;
extern IMUDataTypedef imu_data;
void usart1_send_char(uint8_t c);
void usart1_niming_report(uint8_t fun,uint8_t*data,uint8_t len);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
uint8_t MPU6500_Init(void);
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len);
void IMU_Get_Data(void);
uint8_t IST8310_Init(void);
void IMU_Show_Data(void);
void Init_Quaternion(void);
void IMU_getYawPitchRoll(volatile float * ypr); //更新姿态
void GetPitchYawGxGyGz(void);
void Calibrate_IST(void);
void IMU_init(void);


#endif

