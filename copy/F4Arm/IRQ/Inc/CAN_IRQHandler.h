/*
 * @Author: Frt001 2067314783@qq.com
 * @Date: 2026-08-13 10:00:01
 * @LastEditors: Frt001 2067314783@qq.com
 * @LastEditTime: 2026-08-24 11:26:40
 * @FilePath: \f4_show\IRQ\Inc\CAN_IRQHandler.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CAN_IRQHANDLER_H
#define CAN_IRQHANDLER_H

#include "EXTI_IRQHandler.h"
#include "main.h"
#include "can.h"

typedef enum{
    Master_Default = 0U,
    Master_Disable,
    Master_Enable,
    Master_Reset,
    Master_LowPick,
    Master_HighPick,
    Master_SecondPick,
    Master_PlaceBottom,
    Master_PlaceLevel1,
    Master_PlaceLevel2,
    Master_Release,
} Command_of_Master;  // 主人的任务（）

/* 从任务上下文回 finish（不带 hcan/TxHeader 等中间变量）。 */
bool Comm_SendFinish(uint8_t process);

/* 周期确认主控连续点击次数，并转换为具体的取块/放块命令。 */
void Comm_ClickProcess(void);

/* 主控命令 FIFO：入队（ISR 与任务均可调用）与出队（任务 5ms 周期调用）。
 * 替代原 volatile 单值,消除任务消费前第二条命令覆盖第一条的丢帧窗口。 */
void Comm_Enqueue(uint8_t cmd);
uint8_t Comm_Dequeue(void);

#endif /* CAN_IRQHANDLER_H */
