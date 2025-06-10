//
// Created by jianai on 2025/6/8.
//

#include <math.h>
#include "qmc5883.h"

QMC5883L_HANDLE qmc5883 = {
        0,
        0,
        0,
        QMC5883L_init,
        QMC5883_GetData,
        QMC5883L_GetTemper
};

uint8_t QMC5883L_ReadOneReg(uint8_t RegAddr, uint8_t *pData){
    uint8_t ret = 0;
#if QMC_HARD==1//硬件I2C

#else//软件I2C
    ret = iic[QMC5883_IIC].ReadReg(QMC5883_IIC, (QMC5883L_CHIPID << 1 | 0), (RegAddr), pData, 1);
#endif
    return ret;
}

uint8_t QMC5883L_ReadMultReg(uint8_t RegAddr, uint8_t RegNum, uint8_t DataBuff[]){
    uint8_t ret = 0;
#if QMC_HARD==1//硬件I2C

#else//软件I2C
    ret = iic[QMC5883_IIC].ReadReg(QMC5883_IIC, (QMC5883L_CHIPID << 1 | 0), RegAddr, DataBuff, RegNum);
#endif
    return ret;
}

uint8_t QMC5883L_WriteOneReg(uint8_t RegAddr,uint8_t data){
    uint8_t ret = 0;
#if QMC_HARD==1//硬件I2C

#else//软件I2C
    ret = iic[QMC5883_IIC].WriteReg(QMC5883_IIC, (QMC5883L_CHIPID << 1 | 0), RegAddr, &data, 1);
#endif
    return ret;
}

// todo 控制寄存器1的模式设置
uint8_t QMC5883L_SetMode(QMC5883L_MODE mode, QMC5883L_ODR odr, QMC5883L_RNG rng, QMC5883L_OSR osr)
{
    u8 tempreg = 0; // 0x11
    tempreg = ((mode&0x03)<<0) | ((odr&0x03)<<2) | ((rng&0x03)<<4) | ((osr&0x03)<<6);
    qmc5883.ModeConfigData = tempreg;
    return QMC5883L_WriteOneReg(QMC5883L_CONTROL1_REG, tempreg);
}


uint8_t QMC5883L_init(void){
#if QMC_HARD==1//硬件I2C

#else//软件I2C
    iic_attach(QMC5883_IIC,
               QMC5883L_I2C_SCL_Port,
               QMC5883L_I2C_SCL_Pin,
               QMC5883L_I2C_SDA_Port,
               QMC5883L_I2C_SDA_Pin,
               QMC_HARD);
#endif

    u8 tempreg;
    if(QMC5883L_ReadOneReg(QMC5883L_CHIPID, &tempreg) == FALSE){
        printf("QMC5883L初始化失败，读取芯片ID失败\r\n");
        return FALSE;
    }
    if(tempreg != QMC5883L_ID){
        printf("QMC5883L初始化失败，无效的ID:0x%X\r\n", tempreg);
        return FALSE;
    }
    printf("QMC5883L初始化成功，ID:0x%X\r\n", tempreg);

    QMC5883L_WriteOneReg(QMC5883L_CONTROL2_REG, 0X80);			//复位，要延时一定时间
    delay_ms(100);
    QMC5883L_WriteOneReg(QMC5883L_SET_RESET_PERIOD, 0X01);		//复位后必须写入0x01,否则温度为0，磁场数据也不对
    QMC5883L_SetMode(QMC5883L_MODE_CONTINUOUS, QMC5883L_ODR_50HZ, QMC5883L_RNG_2G, QMC5883L_OSR_64);//设置工作模式
    return 1;
}


int16_t QMC5883L_GetTemper(void)
{
    uint8_t buff[2];
    uint16_t temp;

    if(QMC5883L_ReadMultReg(QMC5883L_TEMP_LSB, 2, buff) == FALSE) return FALSE;
    temp = ((uint16_t)buff[1]<<8)|buff[0];
    qmc5883.temp = (int16_t)temp;

    return qmc5883.temp;
}

uint8_t QMC5883L_GetMagneticData(int16_t *mx, int16_t *my ,int16_t *mz)
{
    u8 buff[6];

    if(QMC5883L_ReadMultReg(QMC5883L_DATA_X_LSB, 6, buff) == FALSE) return FALSE;
    *mx = (int16_t)(((u16)buff[1]<<8)|buff[0]);
    *my = (int16_t)(((u16)buff[3]<<8)|buff[2]);
    *mz = (int16_t)(((u16)buff[5]<<8)|buff[4]);

    return TRUE;
}

int16_t QMC5883_GetData(void){
    int16_t X, Y,Z;
    QMC5883L_GetMagneticData(&X, &Y, &Z);
    qmc5883.yaw = (atan2(Y,X) * (180 / 3.14159265) + 180);
    return qmc5883.yaw;
}
