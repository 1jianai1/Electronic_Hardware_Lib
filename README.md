这是我为提高嵌入式开发效率封装的一些库, 这些库我要求具有更好的移植性以及方便使用.\
主要的语言为**C语言**(日后也可能会添加C++版本), 结合了**面向对象**的编程思想和**arduino的使用方式**进行封装.

**硬件平台: STM32F411CEU6**\
**软件平台: stm32cubemx + CLion + OpenOCD**\
**烧录器: DAPLink**

所有的外设我会基于此项目进行测试, 因此库文件是嵌入在工程里面的, 具体的外设文件的路径是:\
**Core/Src/Hardware**\
对于单个模块我在 **Core/SRC/Hardware/use.c/** 这个路径下面设立了使用方法\
目前包含的模块有:
 
Hardware:
- mpu6050 + dmp
- qmc5883
- motor直流电机: tb6612 和 drv8701驱动
- stepmotor步进电机：tb6600驱动
- oled，i2c接口\

conmunication(通信):
- 软件I2C Core/Src/Conmunication/iic_moni
- 软件SPI Core/Src/Conmunication/spi_moni  (施工中)
- 串口调试协议 Core/Src/Conmunication/serial

sys(开发预设)
- config:
  - 串口重定向
  - 全局变量结构体
  - gpio拉高低
- delay: ms 和 us

通过文件名应该可以很方便的指导这个文件是干什么的, 而且每个外设文件夹里面我会尽可能添加**说明文件**方便阅读, 和日后的使用
