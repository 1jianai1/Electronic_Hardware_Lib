//
// Created by jianai on 2024/8/5.
//

#include "MPU6050.h"
MPU6050_Datas mpu6050_Datas;
static int16_t Mpu6050Addr = 0x68;

#if IS_I2C_HARDWARE==0


#else
void MPU6050_WriteReg(uint16_t DevAddr,uint16_t reg_add,uint8_t* reg_dat)
{

    //HAL_I2C_Master_Transmit(&hi2c1, MPU6050_SLAVE_ADDRESS, (uint8_t*)reg_add, 1, 10000);
    //HAL_I2C_Master_Transmit(&hi2c1, MPU6050_SLAVE_ADDRESS, &reg_dat, 1, 10000);
//直接写入
    HAL_I2C_Mem_Write(&MPU6050_I2C, DevAddr,reg_add,I2C_MEMADD_SIZE_8BIT, reg_dat,1,1000);

}

/**
  * @brief   从MPU6050寄存器读取数据
  * @param
  * @retval
  */
void MPU6050_ReadDatas(uint16_t DevAddr,uint16_t reg_add,uint8_t *Read,uint16_t num)
{
    /*
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_SLAVE_ADDRESS, &reg_add, 1, 10000);
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_SLAVE_ADDRESS+1, Read, num-1, 10000);
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_SLAVE_ADDRESS+1,Read+num-1 ,1, 10000);
    */
    HAL_I2C_Mem_Read(&MPU6050_I2C, DevAddr, reg_add, I2C_MEMADD_SIZE_8BIT, Read, num, 10000);
}
uint8_t MPU6050_ReadData(uint8_t reg_add)
{
    uint8_t Data;
    //MyI2C_Start();						//I2C起始
    //MyI2C_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
    //MyI2C_ReceiveAck();					//接收应答
    //MyI2C_SendByte(RegAddress);			//发送寄存器地址
    //MyI2C_ReceiveAck();					//接收应答
    //MyI2C_Start();						//I2C重复起始
    //MyI2C_SendByte(MPU6050_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
    //MyI2C_ReceiveAck();					//接收应答
    //Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
    //MyI2C_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
    //MyI2C_Stop();						//I2C终止
    HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_SLAVE_ADDRESS, reg_add, 1, &Data, 1, 10000);
    return Data;
}
#endif

/**
  * @brief   读取MPU6050的ID
  * @param
  * @retval
  */
int16_t MPU6050ReadID(void)
{
    uint8_t Re = 0;
    //MPU6050_ReadDatas(MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
    for(uint8_t i=0;i<255;i++)
    {
        if(HAL_I2C_IsDeviceReady(&MPU6050_I2C,i,1,1000)==HAL_OK){
            Mpu6050Addr = i;
            return i;
        }
    }
    return 0xD1;
    /*
    if(Re != 0x68)
    {
        return 0;
    }
    else
    {
        return 1;
    }*/
    return Re;
}

/**
  * @brief   初始化MPU6050芯片
  * @param
  * @retval
  */
void MPU6050_Init(void)
{
    uint16_t i=0,j=0;

    //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
    for(i=0;i<1000;i++)
    {
        for(j=0;j<1000;j++)
        {
            ;
        }
    }

    //MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	    //解除休眠状态
    //MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //陀螺仪采样率，1KHz
    //MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	        //低通滤波器20hz
    //MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x08);	  //配置加速度传感器工作在±4g模式，不自检
    //MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //陀螺仪自检及测量范围,2000deg/s
    /*
    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x01);				//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_2, 0x00);				//电源管理寄存器2，保持默认值0，所有轴均不待机
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);				//采样率分频寄存器，配置采样率
    MPU6050_WriteReg(MPU6050_RA_CONFIG, 0x06);					//配置寄存器，配置DLPF
    MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG, 0x18);			//陀螺仪配置寄存器，选择满量程为±2000°/s
    MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);			//加速度计配置寄存器，选择满量程为±16g
    */
    uint8_t check;
    int16_t Address=MPU6050ReadID();
    HAL_I2C_Mem_Read(&MPU6050_I2C,Address,MPU6050_RA_WHO_AM_I,1,&check,1,1000 );
    if(check==0x68){
        check = 0x00;
        MPU6050_WriteReg(Address,MPU6050_RA_PWR_MGMT_1,&check); 	    // 唤醒
        check = 0x07;
        MPU6050_WriteReg(Address,MPU6050_SMPLRT_DIV,&check);	    // 1Khz的速率
        check = 0x00;
        MPU6050_WriteReg(Address,MPU6050_RA_ACCEL_CONFIG,&check);	 	// 加速度配置
        check = 0x00;
        MPU6050_WriteReg(Address,MPU6050_RA_GYRO_CONFIG,&check);		// 陀螺配置
    }
}



/**
  *todo:
  * @brief   读取MPU6050的加速度数据
  * @param
  * @retval
  */
void MPU6050_ReadAcc()
{
    uint8_t buf[6]={0};
    MPU6050_ReadDatas(Mpu6050Addr,MPU6050_ACC_OUT, buf, 6);//读取加速度寄存器数据
    //数据合并
    mpu6050_Datas.Accel_X = (short)(buf[0] << 8) | buf[1];
    mpu6050_Datas.Accel_Y = (short)(buf[2] << 8) | buf[3];
    mpu6050_Datas.Accel_Z = (short)(buf[4] << 8) | buf[5];
}

/**
  *todo
  * @brief   读取MPU6050的角加速度数据
  * @param
  * @retval
  */
void MPU6050_ReadGyro()
{
    uint8_t buf[6]={0};
    MPU6050_ReadDatas(Mpu6050Addr,MPU6050_GYRO_OUT,buf,6);//读取对应的寄存器

    mpu6050_Datas.Gyro_X = (buf[0] << 8) | buf[1];//数据合并
    mpu6050_Datas.Gyro_Y = (buf[2] << 8) | buf[3];
    mpu6050_Datas.Gyro_Z = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的原始温度数据
  * @param
  * @retval
  */
void MPU6050_ReadTemp()
{
    uint8_t buf[2]={0};
    MPU6050_ReadDatas(Mpu6050Addr,MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
    mpu6050_Datas.Temp = (int16_t)(buf[0] << 8) | buf[1];
    mpu6050_Datas.Temp = (double) mpu6050_Datas.Temp/340.0+36.53f;
}

/*
 float Pitch=1,Roll=1,Yaw=1;						//角度
short gyrox,gyroy,gyroz;				//陀螺仪-
short aacx,aacy,aacz;						//加速度计
int main(void)
{
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

MPU6050_Init();//我们只需要将mpu6050初始化就行，其底层无需其他的函数
mpu_dmp_init();//数据的读取全都在dmp库文件力

while (1)
{

mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
}
}
*/
