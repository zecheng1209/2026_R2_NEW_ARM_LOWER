#include "Task_Init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "Run.h"
#include "math.h"
#include "stdbool.h"
#include "usart.h"
#include "infrared_host.h"
extern Joint_t Joint[5];
float Motor_Init[5] = {0};
uint8_t usart4_dma_buff[3];
Message_t Mesg;
extern TaskHandle_t Motor_Drive_Handle;
extern TaskHandle_t MotorSendTask_Handle;
extern TaskHandle_t MotorRecTask_Handle;
TaskHandle_t Motor_Reset_Handle;

void MotorInit(void);// 电机初始化
void Motor_reset(void *param);// 电机复位

bool Float_S(float a, float b) // 浮点数比较
{
	return fabsf(a - b) < 0.03f;
}

uint8_t F_buf[5] = {0};
bool Joint_FinInit() // 检查所有关节是否都初始化完成
{
	F_buf[0] = Float_S(Joint[0].Rs_motor.state.rad, 0 + Joint[0].pos_offset); // 检查关节0的弧度是否为0
	F_buf[1] = Float_S(Joint[1].Rs_motor.state.rad, 0 + Joint[1].pos_offset);
	F_buf[2] = Float_S(Joint[2].Rs_motor.state.rad, 0 + Joint[2].pos_offset);
	F_buf[3] = Float_S(Joint[3].Rs_motor.state.rad, 0 + Joint[3].pos_offset);
	F_buf[4] = Float_S(Joint[4].Rs_motor.state.rad, 0 + Joint[4].pos_offset);

	if (F_buf[0] && F_buf[1] && F_buf[2] && F_buf[3] && F_buf[4]) // 检查所有关节的弧度是否都为0
		return true;
	else
		return false;
}

void Task_Init(void)
{
	//遥控器
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 接收完成中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY); // 发送完成中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
	
  MotorInit();
	/////重置角度
	//RobStrideResetAngle(&Joint[1].Rs_motor);
	
	//IR_Init(&hcan2,(uint8_t[]){0x10,0x11,0x12},3);
	xTaskCreate(Motor_Drive, "Motor_Drive", 628, NULL, 4, &Motor_Drive_Handle);		  // 驱动
	xTaskCreate(Motor_reset, "Motor_reset", 300, NULL, 4, &Motor_Reset_Handle);		  // 复位
	xTaskCreate(MotorSendTask, "MotorSendTask", 128, NULL, 4, &MotorSendTask_Handle); // 将数据发送到PC
}

void RampToTarget(float *val, float target, float step) // 斜坡
{
	float diff = target - *val;

	if (fabsf(diff) < step)
	{
		*val = target;
	}
	else
	{
		*val += (diff > 0 ? step : -step);
	}
}

uint8_t ready = 0;

void Motor_reset(void *param)// 复位电机
{
	TickType_t Last_wake_time = xTaskGetTickCount();

	vTaskDelay(20);

	Motor_Init[0] = Joint[0].Rs_motor.state.rad;
	Motor_Init[1] = Joint[1].Rs_motor.state.rad;
	Motor_Init[2] = Joint[2].Rs_motor.state.rad;
	Motor_Init[3] = Joint[3].Rs_motor.state.rad;
	Motor_Init[4] = Joint[4].Rs_motor.state.rad;

	Joint[0].exp_rad = Motor_Init[0] - Joint[0].pos_offset;
	Joint[1].exp_rad = Motor_Init[1] - Joint[1].pos_offset;
	Joint[2].exp_rad = Motor_Init[2] - Joint[2].pos_offset;
	Joint[3].exp_rad = Motor_Init[3] - Joint[3].pos_offset;
	Joint[4].exp_rad = Motor_Init[4] - Joint[4].pos_offset;
	for (;;)
	{
		RampToTarget(&Joint[0].exp_rad, 0, 0.001f);
		RampToTarget(&Joint[1].exp_rad, 0, 0.001f);
		RampToTarget(&Joint[2].exp_rad, 0, 0.0012f);
		RampToTarget(&Joint[3].exp_rad, 0, 0.001f);
		RampToTarget(&Joint[4].exp_rad, 0, 0.001f);

		if (Joint_FinInit())
		{
			xTaskCreate(MotorRecTask, "MotorRecTask", 200, NULL, 4, &MotorRecTask_Handle);   //PC接收数据
			vTaskDelete(NULL);
		}

		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
	}
}

void PID_Init_Pos(Joint_t *Joint, float kp, float ki, float kd, float limit, float pid_out)
{
	Joint->pos_pid.Kp = kp;
	Joint->pos_pid.Ki = ki;
	Joint->pos_pid.Kd = kd;
	Joint->pos_pid.limit = limit;
	Joint->pos_pid.output_limit = pid_out;
}

void PID_Init_Vel(Joint_t *Joint, float kp, float ki, float kd, float limit, float pid_out)
{
	Joint->vel_pid.Kp = kp;
	Joint->vel_pid.Ki = ki;
	Joint->vel_pid.Kd = kd;
	Joint->vel_pid.limit = limit;
	Joint->vel_pid.output_limit = pid_out;
}

void RS_Offest_inv(Joint_t *Joint, int8_t inv_motor, float pos_offset)
{
	Joint->inv_motor = inv_motor;
	Joint->pos_offset = pos_offset;
}

void MotorInit(void)
{
	vTaskDelay(2000);

	PID_Init_Pos(&Joint[0], 30.0f, 0.0f, 0.0f, 100.0f, 5.0f); // 位置pid//云台
	PID_Init_Vel(&Joint[0], 8.0f, 0.8f, 0.0f, 50.0f, 20.0f); // 速度pid3.6   3.2
	RS_Offest_inv(&Joint[0], 1, 2.26959395f);				 // 方向和偏移值   //

	PID_Init_Pos(&Joint[1], 5.0f, 0.0f, 2.0f, 100.0f, 4.0f); // 大臂  //p d50
	PID_Init_Vel(&Joint[1], 15.0f, 0.1f, 0.0f, 100.0f, 30.0f);         //p9.0
	RS_Offest_inv(&Joint[1], 1, 1.1826614f);             //1.1826614

	PID_Init_Pos(&Joint[2], 5.0,0.0f,0.0f, 500.0f, 2.5f); // 小臂
	PID_Init_Vel(&Joint[2], 2.7f, 0.1f, 0.0f, 70.0f, 15.0f); //2.8 0.002
	RS_Offest_inv(&Joint[2], -1, 2.13690066f);       //上

	PID_Init_Pos(&Joint[3], 60.0f, 0.0f, 0.0f, 20.0f, 20.0f); // 手腕  
	PID_Init_Vel(&Joint[3], 3.5f, 0.8f, 0.0f, 20.0f, 15.0f);
	RS_Offest_inv(&Joint[3], 1, -0.010738194f);        //上5.51 下1.71  

	PID_Init_Pos(&Joint[4], 60.0f, 0.0f, 0.0f, 0.0f, 30.0f); // 末端
	PID_Init_Vel(&Joint[4], 3.0f, 0.3f, 0.0f, 10.0f, 2.0f);  //不宜给过大，容易电压过低然后电机无输出力矩
	RS_Offest_inv(&Joint[4], 1, 5.37676668f);        //左6.03  右4.78

	vTaskDelay(100);
	RobStrideInit(&Joint[0].Rs_motor, &hcan1, 0x01, RobStride_06);	 // 云台
	RobStrideInit(&Joint[1].Rs_motor, &hcan1, 0x02, RobStride_03);	 // 大臂
	RobStrideInit(&Joint[2].Rs_motor, &hcan2, 0x03, RobStride_01);	 // 小臂
	RobStrideInit(&Joint[3].Rs_motor, &hcan2, 0x04, RobStride_02);	 // 末端
	RobStrideInit(&Joint[4].Rs_motor, &hcan2, 0x05, RobStride_EL05); // 末端

	RobStrideSetMode(&Joint[0].Rs_motor, RobStride_MotionControl);	
	vTaskDelay(1);
	RobStrideSetMode(&Joint[1].Rs_motor, RobStride_MotionControl);
	vTaskDelay(1);
	RobStrideSetMode(&Joint[2].Rs_motor, RobStride_MotionControl);
	vTaskDelay(1);
	RobStrideSetMode(&Joint[3].Rs_motor, RobStride_MotionControl);
	vTaskDelay(1);
	RobStrideSetMode(&Joint[4].Rs_motor, RobStride_MotionControl);
	vTaskDelay(200);
	RobStrideEnable(&Joint[0].Rs_motor);
	vTaskDelay(1);
	RobStrideEnable(&Joint[1].Rs_motor);
	vTaskDelay(1);
	RobStrideEnable(&Joint[2].Rs_motor);
	vTaskDelay(1);
	RobStrideEnable(&Joint[3].Rs_motor);
	vTaskDelay(1);
	RobStrideEnable(&Joint[4].Rs_motor);

	vTaskDelay(2000);
}
