# 简介
IIC是一个非常常见的通信协议，写这个库的原因主要是在网上各种模块的IIC驱动大多数人写的都有一定的差异同时在看一些IIC驱动的模块的博客文章时大多都没有给IIC的驱动库，故写了一个软件IIC库供我自己方便移植使用，下面来看看移植流程。

# 重写初始化函数
代码在 **iic_moni.c的最后一个函数iic_attach** 因为是软件IIC，仅仅是初始化GPIO引脚就行了，这里也只需要修改成你的硬件平台的初始化代码就行.
这里也是将对应的**函数指针指向了实现的函数**，你可以直接**跳转**到对应的实现函数当中，可以根据实际需要修改底层实现函数.
```c
void iic_attach(
        IIC_ID id,
        GPIO_TypeDef* SCL_PORT,
        uint16_t SCL,
        GPIO_TypeDef* SDA_PORT,
        uint16_t SDA,
        IIC_MODE mode
        ){
    // todo 接口
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

    // todo GPIO引脚修改部分
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
```

# 重写引脚置为函数
W_SCL, W_SDA, R_SDA 这三个函数，位于 **iic_moni.c文件的最上面**，替换成你的硬件平台的库函数即可，无非就是拉高低电平以及读电平
```c
void W_SCL(IIC_ID id, uint8_t x){
    HAL_GPIO_WritePin(iic[id].SCL_PORT, iic[id].SCL, x);
}
void W_SDA(IIC_ID id, uint8_t x){
    HAL_GPIO_WritePin(iic[id].SDA_PORT,iic[id].SDA, x);
}
uint8_t R_SDA(IIC_ID id){
    return HAL_GPIO_ReadPin(iic[id].SDA_PORT, iic[id].SDA);
}
```
当然还有一点，我的函数是基于STM32 HAL库写的，在读写引脚电平的时候只需要初始化为ouput就行，有些单片机如果不初始化为 input 模式就无法读电平，这意味这在每次调用 读/写操作的时候需要**重新更改GPIO模式**，这个也很简单，在上面函数前面添加即可。

# 重写引脚类型
对于非hal库的GPIO类型和引脚的定义可能不一样，这个移植比较麻烦，我这里并没有做适配，意味着你可能需要一个一个的改，当然现在这些编译软件有一键修改匹配项的功能，也不算特别麻烦。

# 使用
## 简单演示
对于库的调用并非是传统的调用一个函数的名称可以，考虑到一个工程可能需要使用多个IIC设备，这里我是用**结构体数组 + 函数指针的方式进行调用**的，如下：
```c
iic[OLED_IIC].WriteReg(OLED_IIC, 0x78, 0x00, &Command, 1)   // 向OLED发送命令

iic[OLED_IIC].W_SCL(OLED_IIC, 1); 
iic[OLED_IIC].W_SDA(OLED_IIC, 1);   // 引脚置位
```
## 添加引脚
首先需要使用**iic_attach函数添加引脚**，比如在OLED驱动库中需要添加下面这句函数：
```c
    iic_attach(OLED_IIC,            // IIC 编号
               OLED_I2C_SCL_Port,   // 
               OLED_I2C_SCL_Pin,
               OLED_I2C_SDA_Port,
               OLED_I2C_SDA_Pin,
               0);       // 直接给0就行
```

## 重写写命令/数据函数
对于不同的模块有不同的访问寄存器的方式，但是通信协议都是一样的，以江科大的OLED库为例，需要更改下面这两个函数就行了
```c
void OLED_WriteCommand(uint8_t Command)
{
#if I2C_HARDWARE == 1
    //todo 硬件方式发送命令
    uint8_t datas[]={0x00,Command};
    HAL_I2C_Master_Transmit(&OLED_I2C,OLED_ADDRESS,datas,sizeof(datas),0xFF);
#else
    if(!iic[OLED_IIC].WriteReg(OLED_IIC, 0x78, 0x00, &Command, 1)){
        printf("发送出错\r\n");
    }
//	OLED_I2C_Start();
//	OLED_I2C_SendByte(0x78);		//从机地址
//	OLED_I2C_SendByte(0x00);		//写命令
//	OLED_I2C_SendByte(Command);
//	OLED_I2C_Stop();
#endif

}

void OLED_WriteData(uint8_t Data)
{
#if I2C_HARDWARE == 1
    //todo 硬件方式发送命令
    uint8_t datas[]={0x40,Data};
    HAL_I2C_Master_Transmit(&OLED_I2C,OLED_ADDRESS,datas,sizeof(datas),0xFF);
#else
    if(!iic[OLED_IIC].WriteReg(OLED_IIC, 0x78, 0x40, &Data, 1)){
        printf("发送出错\r\n");
    }
//	OLED_I2C_Start();
//	OLED_I2C_SendByte(0x78);		//从机地址
//	OLED_I2C_SendByte(0x40);		//写数据
//	OLED_I2C_SendByte(Data);
//	OLED_I2C_Stop();
#endif
}
```
然后正常调用库函数即可

