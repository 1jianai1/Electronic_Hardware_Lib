//
// Created by jianai on 2025/6/11.
//

#include "spi_moni.h"
#include <stdio.h>

SPIINTERFACE spi[SPI_SUM];
/*引脚配置层*/

/**
  * 函    数：SPI写SS引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SS的电平，范围0~1
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SS为低电平，当BitValue为1时，需要置SS为高电平
  */
static void W_SS(SPI_ID id, uint8_t BitValue)
{
    HAL_GPIO_WritePin(spi[id].CE_PORT, spi[id].CE, BitValue);		//根据BitValue，设置SS引脚的电平
}

/**
  * 函    数：SPI写SCK引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SCK的电平，范围0~1
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SCK为低电平，当BitValue为1时，需要置SCK为高电平
  */
static void W_SCK(SPI_ID id, uint8_t BitValue)
{
    HAL_GPIO_WritePin(spi[id].SCL_PORT, spi[id].SCL, BitValue);		//根据BitValue，设置SCK引脚的电平
}

/**
  * 函    数：SPI写MOSI引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入MOSI的电平，范围0~0xFF
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置MOSI为低电平，当BitValue非0时，需要置MOSI为高电平
  */
void W_MOSI(SPI_ID id, uint8_t BitValue)
{
    if(!spi[id].MOSI_PORT){
        printf("MOSI为空\r\n");
        return;
    }
    HAL_GPIO_WritePin(spi[id].MOSI_PORT, spi[id].MOSI, BitValue);		//根据BitValue，设置MOSI引脚的电平，BitValue要实现非0即1的特性
}

/**
  * 函    数：I2C读MISO引脚电平
  * 参    数：无
  * 返 回 值：协议层需要得到的当前MISO的电平，范围0~1
  * 注意事项：此函数需要用户实现内容，当前MISO为低电平时，返回0，当前MISO为高电平时，返回1
  */
uint8_t R_MISO(SPI_ID id)
{
    return HAL_GPIO_ReadPin(spi[id].MISO_PORT, spi[id].MISO);			//读取MISO电平并返回
}			//读取MISO电平并返回

/*协议层*/

/**
  * 函    数：SPI起始
  * 参    数：无
  * 返 回 值：无
  */
static void SPI_Start(SPI_ID id)
{
    W_SS(id, 0);				//拉低SS，开始时序
}

/**
  * 函    数：SPI终止
  * 参    数：无
  * 返 回 值：无
  */
static void SPI_Stop(SPI_ID id)
{
    W_SS(id, 1);				//拉高SS，终止时序
}

/**
  * 函    数：SPI交换传输一个字节，使用SPI模式0
  * 参    数：ByteSend 要发送的一个字节
  * 返 回 值：接收的一个字节
  */
static uint8_t SPI_SwapByte(SPI_ID id, uint8_t ByteSend)
{
    uint8_t i, ByteReceive = 0x00;					//定义接收的数据，并赋初值0x00，此处必须赋初值0x00，后面会用到

    for (i = 0; i < 8; i ++)						//循环8次，依次交换每一位数据
    {
        W_MOSI(id, ByteSend & (0x80 >> i));		//使用掩码的方式取出ByteSend的指定一位数据并写入到MOSI线
        W_SCK(id, 1);								//拉高SCK，上升沿移出数据
        if (R_MISO(id) == 1){ByteReceive |= (0x80 >> i);}	//读取MISO数据，并存储到Byte变量
        //当MISO为1时，置变量指定位为1，当MISO为0时，不做处理，指定位为默认的初值0
        W_SCK(id, 0);								//拉低SCK，下降沿移入数据
    }

    return ByteReceive;								//返回接收到的一个字节数据
}

static uint8_t SPI_SwapDatas(SPI_ID id, uint8_t* sendData, uint16_t length){
    SPI_Start(id);
    for(uint16_t i = 0; i < length; i++){
        sendData[i] = SPI_SwapByte(id, sendData[i]);
    }
    SPI_Stop(id);
    return 1;
}

static uint8_t SendDatas(SPI_ID id, uint8_t* sendData, uint16_t length){
    SPI_Start(id);
    for(uint16_t i = 0; i < length; i++){
        SPI_SwapByte(id, sendData[i]);
    }
    SPI_Stop(id);
    return 1;
}

static uint8_t ReceiveDatas(SPI_ID id, uint8_t* sendData, uint16_t length){
    SPI_Start(id);
    for(uint16_t i = 0; i < length; i++){
        sendData[i] = SPI_SwapByte(id, 0xFF);
    }
    SPI_Stop(id);
    return 1;
}

uint8_t spi_attach(
        SPI_ID id,
        GPIO_TypeDef* SCL_PORT,
        uint16_t SCL,               // 时钟
        GPIO_TypeDef* MOSI_PORT,
        uint16_t MOSI,              // 发送
        GPIO_TypeDef* MISO_PORT,
        uint16_t MISO,              // 接收
        GPIO_TypeDef* CE_PORT,
        uint16_t CE,                //  片选
        SPI_MODE mode
){
    SPIINTERFACE* spi_inter = &spi[id];
    spi_inter->SCL_PORT = SCL_PORT;
    spi_inter->SCL = SCL;
    spi_inter->MOSI_PORT = MOSI_PORT;
    spi_inter->MOSI = MOSI;
    spi_inter->MISO_PORT = MISO_PORT;
    spi_inter->MISO = MISO;
    spi_inter->CE_PORT = CE_PORT;
    spi_inter->CE = CE;
    spi_inter->mode = mode;

    spi_inter->W_SS = W_SS;
    spi_inter->W_SCL = W_SCK;
    spi_inter->W_MOSI = W_MOSI;
    spi_inter->R_MISO = R_MISO;
    spi_inter->SwapByte = SPI_SwapByte;
    spi_inter->SwapDatas = SPI_SwapDatas;
    spi_inter->SendDatas = SendDatas;
    spi_inter->ReceiveDatas = ReceiveDatas;

    switch (spi_inter->mode) {
        case FULL:
            printf("软件SPI, 收发模式\r\n");
            if(!spi_inter->SCL_PORT ||
            !spi_inter->MOSI_PORT ||
            !spi_inter->MISO_PORT ||
            !spi_inter->CE_PORT
            ){
                printf("存在引脚指向置空, 重新初始化spi\r\n");
                return 0;
            }

            break;
        case ONLY_SEND:
            printf("软件SPI, 仅发送模式\r\n");
            if(!spi_inter->SCL_PORT ||
               !spi_inter->MOSI_PORT ||
               !spi_inter->CE_PORT
               ){
                printf("存在引脚指向置空, 重新初始化spi\r\n");
                return 0;
            }
            break;
        case ONLY_RECEIVE:
            if(!spi_inter->SCL_PORT ||
               !spi_inter->MISO_PORT ||
               !spi_inter->CE_PORT
                    ){
                printf("存在引脚指向置空, 重新初始化spi\r\n");
                return 0;
            }
            printf("软件SPI, 仅接收模式\r\n");
            break;
        case HARDWARE:
            printf("硬件SPI, 自行指定\r\n");
            break;
        default:
            printf("未指定spi格式, 空指针调用!!!\r\n");
            return 0;
    }

    // init()
    __HAL_RCC_GPIOB_CLK_ENABLE();//内部时钟初始化
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    //SCL
    GPIO_InitStruct.Pin = spi_inter->SCL;
    HAL_GPIO_Init(spi_inter->SCL_PORT, &GPIO_InitStruct);
    //CE
    GPIO_InitStruct.Pin = spi_inter->CE;
    HAL_GPIO_Init(spi_inter->CE_PORT, &GPIO_InitStruct);
    if(spi_inter->MOSI_PORT){
        GPIO_InitStruct.Pin = spi_inter->MOSI;
        HAL_GPIO_Init(spi_inter->MOSI_PORT, &GPIO_InitStruct);
    }
    if(spi_inter->MISO_PORT){
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pin = spi_inter->MISO;
        HAL_GPIO_Init(spi_inter->MISO_PORT, &GPIO_InitStruct);
    }
    /*设置默认电平*/
    spi_inter->W_SS(id,1);											//SS默认高电平
    spi_inter->W_SCL(id,0);											//SCK默认低电平
    spi_inter->W_MOSI(id, 1);
    printf("spi初始化完成\r\n");
    return 1;
}
