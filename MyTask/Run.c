#include "Run.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "infrared_host.h"

extern uint8_t ready;					   // 电机是否就绪标志位
Joint_t Joint[5];						   // 5个关节
uint8_t enable_Joint[5] = {1, 1, 1, 1, 1}; // 5个关节的使能标志位  {1, 1, 1, 1, 1};   {0, 0, 0, 0, 0};
uint8_t enable_feedforward[5] = {1, 1, 1, 1, 1};	   // 5个关节的前馈使能标志位
GPIO_PinState sttb=0; 				       // 吸盘开关状态
TaskHandle_t Motor_Drive_Handle;
void Motor_Drive(void *param)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
  // 开启吸盘
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	for (;;)
	{
    // 吸盘开关使能
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, sttb);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, sttb);

		for (uint8_t i = 0; i < 5; i++)
		{
			PID_Control(Joint[i].Rs_motor.state.rad, Joint[i].exp_rad + Joint[i].pos_offset, &Joint[i].pos_pid);
			PID_Control(Joint[i].Rs_motor.state.omega, Joint[i].pos_pid.pid_out + Joint[i].exp_omega, &Joint[i].vel_pid);
			RobStrideMotionControl(&Joint[i].Rs_motor,Joint[i].Rs_motor.motor_id , ((Joint[i].vel_pid.pid_out * enable_Joint[i])+ (Joint[i].exp_torque*enable_feedforward[i])), 0, 0, 0,0);
//			if(i==1||i==3||i==4)
//			vTaskDelay(2);
		}
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(1));
	}
}

ArmTarget_t armtarget_t;
ArmState_t armstate_t;
TaskHandle_t MotorSendTask_Handle;

SemaphoreHandle_t cdc_recv_semphr;
//Arm_t arm_Rec_t;
uint16_t cur_recv_size;

void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
	cur_recv_size = size;
	if (((ArmTarget_t *)src)->pack_type == 0x01)
	{
		memcpy(&armtarget_t, src, sizeof(armtarget_t));
		xSemaphoreGive(cdc_recv_semphr);
	}
}

void MotorSendTask(void *param) //    将电机的数据发送到PC上
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
	armstate_t.pack_type = 1;

	for (;;)
  {
		for (uint8_t i = 0; i < 5; i++)
	{
		float raw_rad   = (Joint[i].Rs_motor.state.rad - Joint[i].pos_offset) * Joint[i].inv_motor;
		float raw_omega = Joint[i].Rs_motor.state.omega * Joint[i].inv_motor;
		float raw_torque = Joint[i].Rs_motor.state.torque * Joint[i].inv_motor;

//		if (i == 0) // 云台关节有 2.8 的减速比
//		{
//        float ratio = 2.8f;
//        armstate_t.joints[i].rad    = raw_rad / ratio;
//        armstate_t.joints[i].omega  = raw_omega / ratio;  
//        armstate_t.joints[i].torque = raw_torque * ratio; 
//		}
//		else 
//		{
        armstate_t.joints[i].rad    = raw_rad;
        armstate_t.joints[i].omega  = raw_omega;
        armstate_t.joints[i].torque = raw_torque;
//		}
		if(Mesg.state==1)
		{
		armstate_t.message=Mesg.state;
		Mesg.state=0;
		}
	}
		CDC_Transmit_FS((uint8_t *)&armstate_t, sizeof(armstate_t));
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(20));
  }
}

uint8_t count = 0; // 接收数据的次数
TaskHandle_t MotorRecTask_Handle;
void MotorRecTask(void *param) // 从PC接收电机数据
{
	TickType_t last_wake_time = xTaskGetTickCount();

	cdc_recv_semphr = xSemaphoreCreateBinary();
	xSemaphoreTake(cdc_recv_semphr, 0);

	for (;;)
	{
		
		/*================== 气泵、电磁阀控制 ==================*/
      static uint8_t last_airpump = 0;
      static uint8_t valve_timing = 0;
      static TickType_t valve_start_tick = 0;
			sttb = armtarget_t.air_pump;

			if (sttb == 0)
			{
        /* 气泵关闭 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

        /* 从1变0时，启动电磁阀3秒 */
        if ((last_airpump == 1) && (valve_timing == 0))
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
            valve_start_tick = xTaskGetTickCount();
            valve_timing = 1;
						last_airpump = 0;
        }
        if (valve_timing &&
            (xTaskGetTickCount() - valve_start_tick >= pdMS_TO_TICKS(3000)))
        {
            /* 电磁阀关闭 */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
            valve_timing = 0;
        }
			}
			else
			{
        /* 气泵打开 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

        /* 电磁阀关闭 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

        valve_timing = 0;
				last_airpump = 1;
			}	  

		if (xSemaphoreTake(cdc_recv_semphr, pdMS_TO_TICKS(200)) == pdTRUE)
		{
			count++;
			Joint[0].exp_rad = armtarget_t.joints[0].rad* Joint[0].inv_motor;
			Joint[0].exp_omega = armtarget_t.joints[0].omega* Joint[0].inv_motor;
			Joint[0].exp_torque = armtarget_t.joints[0].torque* Joint[0].inv_motor;

			Joint[1].exp_rad = armtarget_t.joints[1].rad* Joint[1].inv_motor;
			Joint[1].exp_omega = armtarget_t.joints[1].omega* Joint[1].inv_motor;
			Joint[1].exp_torque = armtarget_t.joints[1].torque* Joint[1].inv_motor;

			Joint[2].exp_rad = armtarget_t.joints[2].rad * Joint[2].inv_motor;
			Joint[2].exp_omega = armtarget_t.joints[2].omega * Joint[2].inv_motor;
			Joint[2].exp_torque = armtarget_t.joints[2].torque * Joint[2].inv_motor;

			Joint[3].exp_rad = armtarget_t.joints[3].rad * Joint[3].inv_motor;
			Joint[3].exp_omega = armtarget_t.joints[3].omega * Joint[3].inv_motor;
			Joint[3].exp_torque = armtarget_t.joints[3].torque * Joint[3].inv_motor;
			
			Joint[4].exp_rad = armtarget_t.joints[4].rad * Joint[4].inv_motor;
			Joint[4].exp_omega = armtarget_t.joints[4].omega* Joint[4].inv_motor;
			Joint[4].exp_torque = armtarget_t.joints[4].torque* Joint[4].inv_motor;
			
			if(armtarget_t.worked ==1)
				{  
					 armstate_t.message =0;
					 armtarget_t.worked =0;
			}
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		uint8_t buf[8];
		uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
		CAN_RxHeaderTypeDef rx_header;
//		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, buf) == HAL_OK)
//		{
//			uint32_t ID = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
//			IR_OnCanRx(&rx_header, buf);

		RobStrideRecv_Handle(&Joint[0].Rs_motor, &hcan1, ID, buf);
		RobStrideRecv_Handle(&Joint[1].Rs_motor, &hcan1, ID, buf);
//		}
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN2)
	{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t buf[8];
		uint8_t buf2[8];
		uint32_t ID = CAN_Receive_DataFrame(&hcan2, buf);
//		if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &rx_header, buf) == HAL_OK)
//		{
//			uint32_t ID = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
//			//IR_OnCanRx(&rx_header, buf);
	  	RobStrideRecv_Handle(&Joint[2].Rs_motor, &hcan2, ID, buf);
			RobStrideRecv_Handle(&Joint[3].Rs_motor, &hcan2, ID, buf);
			RobStrideRecv_Handle(&Joint[4].Rs_motor, &hcan2, ID, buf);
		//}
	}
}
