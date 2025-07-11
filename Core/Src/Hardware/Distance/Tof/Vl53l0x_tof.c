////
//// Created by jianai on 2025/7/9.
////
//
#include "Vl53l0x_tof.h"
#include "stdio.h"
#include "math.h"
#include "../Src/sys/Config/config.h"
////检查VL53L0x是否正常
////检查成功返回0
////检查失败返回1
////uint8_t VL53L0x_RecData[4]={0};
//struct VL53L0xData  VL53L0x_Bottom;		//底部连接在IIC2的VL53L0x的数据
//
//uint8_t VL53L0x_Check()
//{
//    uint8_t VL53L0x_SendData[2] = {0x01};
//    uint8_t VL53L0x_RecData[5] ;
//
//    //Revision ID:
//    iic[VL53_IIC].ReadReg(VL53_IIC, VL53L0x_add, VL53L0X_REG_IDENTIFICATION_REVISION_ID, VL53L0x_RecData, 1);
//    //Device ID:
//    iic[VL53_IIC].ReadReg(VL53_IIC, VL53L0x_add, VL53L0X_REG_IDENTIFICATION_MODEL_ID, VL53L0x_RecData+1, 1);
//    //PRE_RANGE_CONFIG_VCSEL_PERIOD =
//    iic[VL53_IIC].ReadReg(VL53_IIC, VL53L0x_add, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, VL53L0x_RecData+2, 1);
//    //FINAL_RANGE_CONFIG_VCSEL_PERIOD=
//    iic[VL53_IIC].ReadReg(VL53_IIC, VL53L0x_add, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, VL53L0x_RecData+3, 1);
//
//    iic[VL53_IIC].ReadReg(VL53_IIC, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, VL53L0x_SendData, 1);
//    HAL_Delay(100);
//
//    VL53L0x_SendData[1] = 100;
//    while(VL53L0x_SendData[1]--)
//    {
//        HAL_Delay(10);
//        iic[VL53_IIC].ReadReg(VL53_IIC, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, VL53L0x_RecData+4, 1);
//        if (VL53L0x_RecData[4] & 0x01) break;
//    }
//
//    if (VL53L0x_RecData[4] & 0x01)
//        return 0;				//检查成功返回0
//    else return 1;	//检查失败返回1
//}
//
//
//void vl53l0x_init(void){
//    iic_attach(VL53_IIC,
//               VL_I2C_SCL_Port,
//               VL__I2C_SCL_Pin,
//               VL__I2C_SDA_Port,
//               VL__I2C_SDA_Pin,
//               VL_HARD);
//    //(IIC_ID id, uint8_t addr, uint8_t reg, uint8_t *pdata, uint16_t pNum)
//    uint8_t VL53L0x_SendData[2] = {0x01};
//    uint8_t VL53L0x_RecData[5];
//    uint8_t send_byte = 1;
//    iic[VL53_IIC].WriteReg(VL53_IIC, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, VL53L0x_SendData ,1);
//    VL53L0x_SendData[1] = 100;
//    while(VL53L0x_SendData[1]--)
//    {
//        HAL_Delay(1);
//        iic[VL53_IIC].ReadReg(VL53_IIC, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, VL53L0x_RecData+4, 1);
//        if (VL53L0x_RecData[4] & 0x01) break;
//    }
//    if(!VL53L0x_Check()){
//        printf("s\r\n");
//    }
//}
//
////VL53L0x转换一次
//void VL53L0x_StartConversion(void)
//{
//    uint8_t VL53L0x_SendData[1] = {0x01};
//    iic[VL53_IIC].WriteReg(VL53_IIC,VL53L0x_add, VL53L0X_REG_SYSRANGE_START, VL53L0x_SendData, 1);
//}
//
//uint16_t makeuint16(int lsb, int msb)
//{
//    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
//}
//
//
////VL53L0x读取距离等数据反馈信息
//void VL53L0x_ReadDistance(struct VL53L0xData *VL_temp) {
//    //记录历史有效值
//    VL_temp->dist_last = VL_temp->distValid;
//
//    iic[VL53_IIC].ReadReg(VL53_IIC, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, VL_temp->vtemp, 12);
//
//    VL_temp->acnt = makeuint16(VL_temp->vtemp[7], VL_temp->vtemp[6]);
//    VL_temp->scnt = makeuint16(VL_temp->vtemp[9], VL_temp->vtemp[8]);
//    VL_temp->dist = makeuint16(VL_temp->vtemp[11], VL_temp->vtemp[10]);
//    VL_temp->DeviceRangeStatusInternal = ((VL_temp->vtemp[0] & 0x78) >> 3);
//
//
//    //提取有效值
//    if (VL_temp->dist <= 0x0014)        //距离数据无效
//        VL_temp->distValid = VL_temp->dist_last;
//    else    //有效
//        VL_temp->distValid = VL_temp->dist;
//
///*	下面的代码具体干嘛的时间长忘记了，功能是根据对地倾角求得飞机的高度，大家可以直接删掉 */
//
//    uint8_t i, j;
//
//    for (i = 14; i > 0; i--) {
//        VL_temp->dist_buff[i] = VL_temp->dist_buff[i - 1];
//    }
//    VL_temp->dist_buff[0] = VL_temp->distValid;
//
//    i = 0;
//    j = 0;
//    for (i = 0; i < 6; i++) {
//        if (VL_temp->dist_buff[i] >= 2000)
//            j++;
//    }
//
//    if (j >= 2)        // 1/3的溢出率
//        VL_temp->distValidFinal = 2000;
//    else        //从数组里筛选出最近的有效值
//    {
//        i = 0;
//        for (i = 0; i < 15; i++) {
//            if (VL_temp->dist_buff[i] < 2000)
//                break;
//        }
//        VL_temp->distValidFinal = VL_temp->dist_buff[i];
//    }
//
////    VL_temp->GroundDis_last = VL_temp->GroundDis;
////    VL_temp->GroundDis = (float) VL_temp->distValidFinal / 1000;
////    VL_temp->GroundDis = VL_temp->GroundDis * cos(ABS((PostureAngle.Pitch / 180 * 3.14159))) *
////                         cos(ABS((PostureAngle.Roll / 180 * 3.14159)));
//
//}


#include <math.h>


struct VL53L0xData  VL53L0x_Bottom;		//底部连接在IIC2的VL53L0x的数据


void VL53L0x_init(I2C_HandleTypeDef *hi2c)
{
    uint8_t VL53L0x_SendData[2] = {0x01};
    uint8_t VL53L0x_RecData[5] ;

/*	//Revision ID:
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_IDENTIFICATION_REVISION_ID, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData, 1, 10);
	//Device ID:
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_IDENTIFICATION_MODEL_ID, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+1, 1, 10);
	//PRE_RANGE_CONFIG_VCSEL_PERIOD =
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+2, 1, 10);
	//FINAL_RANGE_CONFIG_VCSEL_PERIOD=
	HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+3, 1, 10);
	*/
    HAL_I2C_Mem_Write(hi2c, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, I2C_MEMADD_SIZE_8BIT, VL53L0x_SendData, 1, 1);
    //HAL_Delay(500);

    VL53L0x_SendData[1] = 100;
    while(VL53L0x_SendData[1]--)
    {
        HAL_Delay(1);
        HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+4, 1, 1);
        if (VL53L0x_RecData[4] & 0x01) break;
    }
}

//检查VL53L0x是否正常
//检查成功返回0
//检查失败返回1
uint8_t VL53L0x_Check(I2C_HandleTypeDef *hi2c)
{
    uint8_t VL53L0x_SendData[2] = {0x01};
    uint8_t VL53L0x_RecData[5] ;

    //Revision ID:
    HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_IDENTIFICATION_REVISION_ID, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData, 1, 1);
    //Device ID:
    HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_IDENTIFICATION_MODEL_ID, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+1, 1, 1);
    //PRE_RANGE_CONFIG_VCSEL_PERIOD =
    HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+2, 1, 1);
    //FINAL_RANGE_CONFIG_VCSEL_PERIOD=
    HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+3, 1, 1);

    HAL_I2C_Mem_Write(hi2c, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, I2C_MEMADD_SIZE_8BIT, VL53L0x_SendData, 1, 1);
    HAL_Delay(500);

    VL53L0x_SendData[1] = 100;
    while(VL53L0x_SendData[1]--)
    {
        HAL_Delay(10);
        HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, I2C_MEMADD_SIZE_8BIT, VL53L0x_RecData+4, 1, 1);
        if (VL53L0x_RecData[4] & 0x01) break;
    }

    if (VL53L0x_RecData[4] & 0x01)
        return 0;				//检查成功返回0
    else return 1;	//检查失败返回1
}

//VL53L0x转换一次
void VL53L0x_StartConversion(I2C_HandleTypeDef *hi2c)
{
    uint8_t VL53L0x_SendData[1] = {0x01};
    HAL_I2C_Mem_Write(hi2c, VL53L0x_add, VL53L0X_REG_SYSRANGE_START, I2C_MEMADD_SIZE_8BIT, VL53L0x_SendData, 1, 1);
}

uint16_t makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}


//VL53L0x读取距离等数据反馈信息
void VL53L0x_ReadDistance(I2C_HandleTypeDef *hi2c, struct VL53L0xData *VL_temp)
{
    //记录历史有效值
    VL_temp->dist_last = VL_temp->distValid;

    HAL_I2C_Mem_Read(hi2c, VL53L0x_add, VL53L0X_REG_RESULT_RANGE_STATUS, I2C_MEMADD_SIZE_8BIT, VL_temp->vtemp, 12, 1);

    VL_temp->acnt = makeuint16(VL_temp->vtemp[7], VL_temp->vtemp[6]);
    VL_temp->scnt = makeuint16(VL_temp->vtemp[9], VL_temp->vtemp[8]);
    VL_temp->dist = makeuint16(VL_temp->vtemp[11], VL_temp->vtemp[10]);
    VL_temp->DeviceRangeStatusInternal = ((VL_temp->vtemp[0] & 0x78) >> 3);


    //提取有效值
    if(VL_temp->dist <= 0x0014)		//距离数据无效
        VL_temp->distValid = VL_temp->dist_last;
    else 	//有效
        VL_temp->distValid = VL_temp->dist;

/*	下面的代码具体干嘛的时间长忘记了，功能是根据对地倾角求得飞机的高度，大家可以直接删掉 */

    uint8_t i,j;

    for(i = 14;i>0;i--)
    {
        VL_temp->dist_buff[i] = VL_temp->dist_buff[i-1];
    }
    VL_temp->dist_buff[0] = VL_temp->distValid;

    i =0;
    j=0;
    for(i=0;i<6;i++)
    {
        if(VL_temp->dist_buff[i] >=2000)
            j++;
    }

    if(j >= 2)		// 1/3的溢出率
        VL_temp->distValidFinal = 2000;
    else		//从数组里筛选出最近的有效值
    {
        i=0;
        for(i=0;i<15;i++)
        {
            if(VL_temp->dist_buff[i] <2000)
                break;
        }
        VL_temp->distValidFinal = VL_temp->dist_buff[i];
    }

//    VL_temp->GroundDis_last = VL_temp->GroundDis;
//    VL_temp->GroundDis = (float)VL_temp->distValidFinal /1000;
//    VL_temp->GroundDis = VL_temp->GroundDis *cos(ABS((PostureAngle.Pitch /180 *3.14159))) *cos(ABS((PostureAngle.Roll /180 *3.14159)));
}