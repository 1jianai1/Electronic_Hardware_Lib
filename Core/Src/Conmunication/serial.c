//
// Created by jianai on 2025/4/23.
//
#include "serial.h"
#include "../Hardware/Motor/control.h"

void debug_serial(uint8_t* data){
    char ch = data[0];
    switch (ch) {
        case 'P':
            motor_speed_pid[M1].Kp += 1;
            break;
        case 'p':
            motor_speed_pid[M1].Kp -= 1;
            break;
        case 'I':
            motor_speed_pid[M1].Ki += 0.1;
            break;
        case 'i':
            motor_speed_pid[M1].Ki -= 0.1;
            break;
        case 'D':
            motor_speed_pid[M1].Kd += 1;
            break;
        case 'd':
            motor_speed_pid[M1].Kp -= 1;
            break;
    }
}
