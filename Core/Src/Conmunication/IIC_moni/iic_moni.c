//
// Created by jianai on 2025/4/24.
//
#include "iic_moni.h"
#include "../Src/sys/delay/delay.h"

IICINTERFACE iic[IIC_SUM]={
        0
};

void W_SCL(IIC_ID id, uint8_t x){
    HAL_GPIO_WritePin(iic[id].SCL_PORT, iic[id].SCL, x);
}
void W_SDA(IIC_ID id, uint8_t x){
    HAL_GPIO_WritePin(iic[id].SDA_PORT,iic[id].SDA, x);
}
uint8_t R_SDA(IIC_ID id){
    return HAL_GPIO_ReadPin(iic[id].SDA_PORT, iic[id].SDA);
}


/**
  * @brief  I2C开始
  * @param  无
  * @retval 无
  */
static void I2C_Start(IIC_ID id)    // SCL = 1 时 SDA 1->0
{	W_SDA(id, 1);

    W_SCL(id, 1);
    W_SDA(id, 0);

    W_SCL(id, 0);
}

/**
  * @brief  I2C停止
  * @param  无
  * @retval 无
  */
static void I2C_Stop(IIC_ID id)     // SCL = 1, SDA 0->1
{
    W_SDA(id, 0);

    W_SCL(id, 1);
    W_SDA(id, 1);
}

/**
  * @brief  I2C发送一个字节
  * @param  Byte 要发送的字节
  * @retval 无
  */
static void I2C_SendByte(IIC_ID id, unsigned char Byte)
{
    unsigned char i;
    for(i=0;i<8;i++)
    {
        W_SDA(id, Byte&(0x80>>i));
        W_SCL(id, 1);
        W_SCL(id, 0);
    }
}

/**
  * @brief  I2C接收一个字节
  * @param  无
  * @retval 接收到的一个字节数据
  */
static unsigned char I2C_ReceiveByte(IIC_ID id)
{
    unsigned char i,Byte=0x00;
    W_SDA(id, 1);
    for(i=0;i<8;i++)
    {
        W_SCL(id, 1);
        if(R_SDA(id)){Byte|=(0x80>>i);}
        W_SCL(id, 0);
    }
    return Byte;
}

/**
  * @brief  I2C发送应答
  * @param  AckBit 应答位，0为应答，1为非应答
  * @retval 无
  */
static void I2C_SendAck(IIC_ID id, unsigned char AckBit)
{
    W_SDA(id, AckBit);
    W_SCL(id, 1);
    W_SCL(id, 0);
}

/**
  * @brief  I2C接收应答位
  * @param  无
  * @retval 接收到的应答位，0为应答，1为非应答
  */
static unsigned char I2C_ReceiveAck(IIC_ID id)
{
    unsigned char AckBit;
    W_SDA(id, 1);
    W_SCL(id, 1);
    AckBit=R_SDA(id);
    W_SCL(id, 0);
    return AckBit;
}


static uint8_t I2C_WriteByte(IIC_ID id, uint8_t addr, uint8_t reg,uint8_t Data)
{
    I2C_Start(id);
    I2C_SendByte(id, addr);
    if(I2C_ReceiveAck(id)){
        I2C_Stop(id);
        return 0;
    }

    I2C_SendByte(id, reg);
    if(I2C_ReceiveAck(id)){
        I2C_Stop(id);
        return 0;
    }

    I2C_SendByte(id, Data);
    if(I2C_ReceiveAck(id)){
        I2C_Stop(id);
        return 0;
    }

    I2C_Stop(id);
    return 1;
}

uint8_t I2C_WriteDatas(IIC_ID id, uint8_t addr, uint8_t reg, uint8_t *pdata, uint16_t pNum){
    for(uint8_t i = 0; i < pNum; i++){
        uint8_t ret = I2C_WriteByte(id, addr, reg, *pdata);
        if(!ret) return 0;
        pdata++;
    }
    return 1;
}

uint8_t I2C_ReadDatas(IIC_ID id, uint8_t addr, uint8_t reg, uint8_t *pdata, uint16_t pNum){
    unsigned char Data;
    I2C_Start(id);
    I2C_SendByte(id, addr);
    if(I2C_ReceiveAck(id)){     // 从机未应答
        I2C_Stop(id);
        return 0;
    }

    I2C_SendByte(id, reg);
    if(I2C_ReceiveAck(id)){
        I2C_Stop(id);
        return 0;
    }

    I2C_Start(id);
    I2C_SendByte(id, addr|0x01);
    if(I2C_ReceiveAck(id)){
        I2C_Stop(id);
        return 0;
    }

    for(uint8_t i = 0; i < pNum; i++){
        *pdata++ = I2C_ReceiveByte(id);
        if(i < pNum - 1){
            I2C_SendAck(id, 0); // 0 应答
        }
    }
    I2C_SendAck(id, 1); // 非应答
    I2C_Stop(id);
    return 1;
}

void iic_attach(
        IIC_ID id,
        GPIO_TypeDef* SCL_PORT,
        uint16_t SCL,
        GPIO_TypeDef* SDA_PORT,
        uint16_t SDA,
        IIC_MODE mode
        ){
    IICINTERFACE* iic_inter = &iic[id];
    iic_inter->SCL_PORT = SCL_PORT;
    iic_inter->SCL = SCL;
    iic_inter->SDA_PORT = SDA_PORT;
    iic_inter->SDA = SDA;
    iic_inter->mode = mode;

    iic_inter->W_SDA = W_SDA;
    iic_inter->W_SCL = W_SCL;
    iic_inter->R_SDA = R_SDA;
    iic_inter->WriteReg = I2C_WriteDatas;
    iic_inter->ReadReg = I2C_ReadDatas;

    __HAL_RCC_GPIOB_CLK_ENABLE();//内部时钟初始化
    //GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    //SCL
    GPIO_InitStruct.Pin = iic_inter->SCL;
    HAL_GPIO_Init(iic_inter->SCL_PORT, &GPIO_InitStruct);
    //SDA
    GPIO_InitStruct.Pin = iic_inter->SDA;
    HAL_GPIO_Init(iic_inter->SDA_PORT, &GPIO_InitStruct);
}



















