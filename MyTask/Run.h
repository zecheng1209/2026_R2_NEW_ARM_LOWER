#ifndef __RUN_H__
#define __RUN_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "RobStride2.h"
#include "motorEx.h"
#include "Task_Init.h"
#define Transmission 2.8f

////#pragma pack(1)

////typedef struct{
////	float rad;
////	float omega;
////	float torque;
////}Motor_t;

////typedef struct{
////	int pack_type;
////	Motor_t joints[5];
////	uint8_t air_pump;
////}Arm_t;

////#pragma pack()


// 设置结构体按1字节对齐
#pragma pack(1)

typedef struct{
    float rad;      // 电机的关节角度
    float omega;    // 电机的角速度
    float torque;   // 电机输出力矩
}Motor_t;

typedef struct{
    int pack_type; // 数据包类型
    Motor_t joints[6]; // 机械臂关节
    unsigned char air_pump; //使能气泵
	  uint8_t worked;//一段任务结束
}ArmTarget_t;

typedef struct{
    int pack_type; // 数据包类型
    Motor_t joints[6]; // 机械臂关节
	  uint8_t message;//拷入串口数据
}ArmState_t;

// 将字节对齐设置恢复为默认值（通常是8字节）
#pragma pack()


typedef struct{
    RobStride_t   Rs_motor;
    float pos_offset;
    int8_t inv_motor;// 电机方向取反标志位

    float exp_rad;   // 期望弧度
    float exp_omega; // 期望角速度
    float exp_torque;// 期望扭矩

    PID pos_pid;     // 位置环PID
    PID vel_pid;     // 速度环PID
}Joint_t;


extern ArmState_t armstate_t;

void Motor_Drive(void *param);
void MotorSendTask(void *param);// 将电机的数据发送到PC上
void MotorRecTask(void *param);// 从PC接收电机的期望值

#endif
