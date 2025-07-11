//
// Created by jianai on 2025/7/10.
//

#include "wit_elec.h"

// 读取欧拉角，在串口中断中不断调用即可, angle为角度，data为接受的一个数据
void getwitData(int16_t *angle, uint8_t data) {
    static uint8_t state = 0;      // 状态机状态
    static uint8_t count = 0;      // 数据字节计数器
    static uint16_t sum = 0;       // 校验和累加器
    static uint8_t raw[6];         // 原始角度数据缓存

    switch (state) {
        case 0: // 等待帧头0x55
            if (data == 0x55) {
                sum = data;       // 初始化校验和
                state = 1;
            }
            break;

        case 1: // 等待帧头0x53
            if (data == 0x53) {
                sum += data;
                state = 2;
                count = 0;
            } else {
                state = (data == 0x55) ? 1 : 0; // 处理连续0x55的情况
            }
            break;

        case 2: // 接收数据帧
            if (count < 8) sum += data;  // 前8字节参与校验和

            if (count < 6) {  // 保存前6字节（角度数据）
                raw[count] = data;
            }

            if (count == 8) {  // 到达校验和字节
                if ((sum & 0xFF) == data) {  // 校验和验证
                    // 组合角度数据（先低8位，后高8位）
                    angle[0] = (int16_t)((raw[1] << 8) | raw[0]); // Roll
                    angle[1] = (int16_t)((raw[3] << 8) | raw[2]); // Pitch
                    angle[2] = (int16_t)((raw[5] << 8) | raw[4]); // Yaw
                }
                state = 0;  // 无论校验是否通过，都重置状态机
            }
            count++;
            break;
    }
}
