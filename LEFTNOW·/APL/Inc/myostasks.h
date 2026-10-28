#ifndef __MYOSTASKS_H
#define __MYOSTASKS_H

#include "cmsis_os2.h"
#include "main.h"
#include "Led.h"
#include "Beep.h"
#include "CAN_IRQHandler.h"

extern uint8_t BeepAlarmTimes;
extern uint8_t flag;
extern uint8_t flag1;
extern uint8_t flag2;
extern uint8_t flag3;
extern uint8_t flag4;
extern uint8_t flag5;
extern uint8_t flag6;
extern uint8_t flag7;

volatile extern uint8_t Master_Command;
// 主机命令

/* 单机械臂的机构服务任务，周期推进状态机。 */
void BlockArmServiceTask(void *argument);

#endif // __MYOSTASKS_H
