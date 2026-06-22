#ifndef __TASK_INIT_H_
#define __TASK_INIT_H_

#include "main.h"

typedef struct{
	uint8_t pack;
	uint8_t state;
	uint8_t back;
}Message_t;

	
extern Message_t Mesg;
void Task_Init(void);
extern uint8_t usart4_dma_buff[3];
#endif
