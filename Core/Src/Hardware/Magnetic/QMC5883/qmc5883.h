//
// Created by jianai on 2025/6/8.
//

#ifndef TEST_QMC5883_H
#define TEST_QMC5883_H

#include "main.h"
#include "../Src/sys/Config/config.h"
#include "../Src/Conmunication/IIC_moni/iic_moni.h"

#define QMC_HARD    0      // 0: 不使用硬件IIC，1 使用硬件IIC

#if QMC_HARD==1//硬件I2C
#include "i2c.h"

#else//软件I2C
//SCL
#define QMC5883L_I2C_SCL_Port     GPIOB
#define QMC5883L_I2C_SCL_Pin      GPIO_PIN_8
//SDA
#define QMC5883L_I2C_SDA_Port     GPIOB
#define QMC5883L_I2C_SDA_Pin      GPIO_PIN_9
#endif

//QMC5883L 句柄
typedef struct
{
    uint8_t ModeConfigData;						//记录模式配置值
    int16_t yaw;                                //偏航角
    int16_t temp;                               // 温度
    uint8_t (*init)(void);                      // 初始化
    int16_t (*getYaw)(void);                    // 获取偏航角
    int16_t (*getTemp)(void);
}QMC5883L_HANDLE;

#define	QMC5883L_WRITE                     0x1A
#define	QMC5883L_READ	                0x1B

//QMC5883L寄存器列表
#define QMC5883L_DATA_X_LSB			0x00		//X LSB			只读
#define QMC5883L_DATA_X_MSB			0x01		//X MSB			只读
#define QMC5883L_DATA_Y_LSB			0x02		//Y LSB			只读
#define QMC5883L_DATA_Y_MSB			0x03		//Y MSB			只读
#define QMC5883L_DATA_Z_LSB			0x04		//Z LSB			只读
#define QMC5883L_DATA_Z_MSB			0x05		//Z MSB			只读

#define QMC5883L_STATUS_REG			0x06		//状态寄存器	只读

#define QMC5883L_TEMP_LSB			0x07		//温度 LSB		只读-温度只是相对值，没有实际用途
#define QMC5883L_TEMP_MSB			0x08		//温度 MSB		只读-温度只是相对值，没有实际用途

#define QMC5883L_CONTROL1_REG		0x09		//控制寄存器1	读写
#define QMC5883L_CONTROL2_REG		0x0A		//控制寄存器2	读写-BIT7为复位
#define QMC5883L_SET_RESET_PERIOD	0x0B		//时间寄存器，设置为0x01
#define QMC5883L_CHIPID				0x0D		//芯片ID		只读
//#define QMC5883L_CHIPID				0x1C		//芯片ID		只读





#define QMC5883L_ID			0xFF				//QMC5883L 芯片ID

//QMC5883L工作模式
typedef enum
{
    QMC5883L_MODE_STANDBY		=	0x00,	//待机模式
    QMC5883L_MODE_CONTINUOUS	=	0x01,	//连续工作模式
}QMC5883L_MODE;


//QMC5883L输出速率
typedef enum
{
    QMC5883L_ODR_10HZ			=	0x00,	//10Hz
    QMC5883L_ODR_50HZ			=	0x01,	//50Hz
    QMC5883L_ODR_100HZ			=	0x02,	//100Hz
    QMC5883L_ODR_200HZ			=	0x03,	//200Hz
}QMC5883L_ODR;

//QMC5883L测量范围
typedef enum
{
    QMC5883L_RNG_2G				=	0x00,	//2G
    QMC5883L_RNG_8G				=	0x01,	//8G
}QMC5883L_RNG;

//QMC5883L过采样
typedef enum
{
    QMC5883L_OSR_512			=	0x00,	//512
    QMC5883L_OSR_256			=	0x01,	//256
    QMC5883L_OSR_128			=	0x02,	//128
    QMC5883L_OSR_64				=	0x03,	//64
}QMC5883L_OSR;

extern QMC5883L_HANDLE qmc5883;

uint8_t QMC5883L_init(void);
int16_t QMC5883L_GetTemper(void);
uint8_t QMC5883L_GetMagneticData(int16_t *mx, int16_t *my ,int16_t *mz);
int16_t QMC5883_GetData(void);


#endif //TEST_QMC5883_H
