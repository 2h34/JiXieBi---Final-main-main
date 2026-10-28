/*
 * @Author: Frt001 2067314783@qq.com
 * @Date: 2026-08-13 10:00:13
 * @LastEditors: Frt001 2067314783@qq.com
 * @LastEditTime: 2026-08-25 16:43:21
 * @FilePath: \f4_show\IRQ\Src\CAN_IRQHandler.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "CAN_IRQHandler.h"
#include "DJmotor.h"
#include "ZDrive.h"
#include "BlockArm.h"
#include "BlockVacuum.h"

typedef enum{
    LEFT_START_PICK = 0x03U,
    // FINE_ADJUST_START,
    // FINE_ADJUST_STOP,
    // LEFT_CONFIRM_GRAB,
    LEFT_START_PLACE,
    RIGHT_START_PICK,
    RIGHT_START_PLACE,
    LEFT_CONFIRM_RELEASE,
    RIGHT_CONFIRM_RELEASE,
    // STOP,
} signal_type;

#define RESET       0xFFU
#define ERROR       0xEEU
#define DISABLE     0x00U
#define ENABLE      0x01U
#define LOWPICK     0x00U
#define HIGHPICK    0x01U
#define BOTTOMPLACE 0x00U
#define LEVEL1PLACE 0x01U
#define LEVEL2PLACE 0x02U
#define BOTHARM     0x00U
#define LEFTARM     0x01U
#define RIGHTARM    0x02U
#define GET_COMMAND 0x00U
#define FINISH      0x01U
#define CLICK_GAP_MS 1000U

volatile uint8_t Master_Command = 0U;

volatile uint8_t s_click_signal = 0U;
volatile uint8_t s_click_count = 0U;
volatile uint32_t s_last_click_tick = 0U;

// 发送消息
void send_disable_message(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader,
    uint8_t TxData[], uint32_t *TxMailbox, uint8_t process)
{
    TxHeader->IDE   = CAN_ID_EXT;
    TxHeader->RTR   = CAN_RTR_DATA;
    TxHeader->ExtId = 0x05010100 | process;
    TxHeader->DLC = 2;
    TxHeader->TransmitGlobalTime = DISABLE; 			
    TxData[0] = 0x02;
    TxData[1] = process;

    HAL_CAN_AddTxMessage(&hcan1, TxHeader, TxData, TxMailbox);
}

void send_busy_message(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader,
    uint8_t TxData[], uint32_t *TxMailbox, uint8_t process)
{
    TxHeader->IDE   = CAN_ID_EXT;
    TxHeader->RTR   = CAN_RTR_DATA;
    TxHeader->ExtId = 0x05010100 | process;
    TxHeader->DLC = 2;
    TxHeader->TransmitGlobalTime = DISABLE; 
    TxData[0] = 0x02;
    TxData[1] = process;

    HAL_CAN_AddTxMessage(&hcan1, TxHeader, TxData, TxMailbox);
}

void send_receive_message(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader,
    uint8_t TxData[], uint32_t *TxMailbox, uint8_t process)
{
    TxHeader->IDE   = CAN_ID_EXT;
    TxHeader->RTR   = CAN_RTR_DATA;
    TxHeader->ExtId = 0x05010100 | process;
    TxHeader->DLC = 2;
    TxHeader->TransmitGlobalTime = DISABLE; 
    TxData[0] = GET_COMMAND;
    TxData[1] = process;

    HAL_CAN_AddTxMessage(&hcan1, TxHeader, TxData, TxMailbox);
}

bool send_finish_message(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader,
    uint8_t TxData[], uint32_t *TxMailbox, uint8_t process)
{
    TxHeader->IDE   = CAN_ID_EXT;
    TxHeader->RTR   = CAN_RTR_DATA;
    TxHeader->ExtId = 0x05010100 | process;
    TxHeader->DLC = 2;
    TxHeader->TransmitGlobalTime = DISABLE; 
    TxData[0] = FINISH;
    TxData[1] = process;

    return HAL_CAN_AddTxMessage(hcan,
                                TxHeader,
                                TxData,
                                TxMailbox) == HAL_OK;
}

/* 从任务上下文回 finish：复用 send_finish_message，省去中间变量。 */
bool Comm_SendFinish(uint8_t process)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];
    return send_finish_message(&hcan1, &TxHeader, TxData, &TxMailbox, process);
}

/* 每收到一帧对应的取块或放块命令，就记录为一次点击。 */
static void Comm_RecordClick(uint8_t signal)
{
    if (s_click_count == 0U)
    {
        s_click_signal = signal;
        s_click_count = 1U;
    }
    else if (s_click_signal == signal)
    {
        if (s_click_count < 4U)
        {
            s_click_count++;
        }
    }
    else
    {
        return;
    }

    s_last_click_tick = HAL_GetTick();
}

/* 最后一次点击后等待一段时间，再把点击次数转换成具体任务。 */
void Comm_ClickProcess(void)
{
    uint8_t signal;
    uint8_t count;
    uint8_t command = Master_Default;
    uint32_t now;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailBox;
    uint8_t TxData[8];

    if (s_click_count == 0U)
    {
        return;
    }

    __disable_irq();
    now = HAL_GetTick();
    if ((now - s_last_click_tick) < CLICK_GAP_MS)
    {
        __enable_irq();
        return;
    }

    signal = s_click_signal;
    count = s_click_count;
    s_click_signal = 0U;
    s_click_count = 0U;
    __enable_irq();

    if (signal == LEFT_START_PICK || signal == RIGHT_START_PICK)
    {
        if (count == 1U)
        {
            command = Master_LowPick;
        }
        else if (count == 2U)
        {
            command = Master_HighPick;
        }
        else if (count == 3U)
        {
            command = Master_SecondPick;
        }
    }
    else if (signal == LEFT_START_PLACE || signal == RIGHT_START_PLACE)
    {
        if (count == 1U)
        {
            command = Master_PlaceBottom;
        }
        else if (count == 2U)
        {
            command = Master_PlaceLevel1;
        }
        else if (count == 3U)
        {
            command = Master_PlaceLevel2;
        }
    }

    if (command != Master_Default)
    {
        send_receive_message(&hcan1, &TxHeader, TxData, &TxMailBox, signal);
        Master_Command = command;
    }
    else
    {
        send_busy_message(&hcan1, &TxHeader, TxData, &TxMailBox, signal);
    }
}

// 接收关于左臂的消息
void Master_message_handler_left(CAN_HandleTypeDef *hcan, 
    CAN_RxHeaderTypeDef RxHeader, uint8_t *RxData)
{
    uint8_t signal = RxHeader.ExtId & 0xFFU;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailBox;
    uint8_t TxData[8];

    if (is_busy() && signal != 0x01 && signal != 0x00)
    {
        send_busy_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
        return;
    }

    switch (signal){

        case 0x00 :{
            if (RxHeader.DLC == 0U){
                TxHeader.IDE   = CAN_ID_EXT;
                TxHeader.RTR   = CAN_RTR_DATA;
                TxHeader.ExtId = 0x05010100;
                TxHeader.DLC = 2;
                TxHeader.TransmitGlobalTime = DISABLE; 

                if (is_disabled()){
                    TxData[0] = 0x02;
                    TxData[1] = 0x00;
                }else if (is_busy()){
                    TxData[0] = 0x01;
                    TxData[1] = 0x01;
                }else {
                    TxData[0] = 0x00;
                    TxData[1] = 0x01;
                }

                HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
            }
            break;
        }

        case 0x01 :{
            if (RxHeader.DLC == 2){
                if (RxData[0] == DISABLE){
                    Master_Command = Master_Disable;
                    TxHeader.IDE   = CAN_ID_EXT;
                    TxHeader.RTR   = CAN_RTR_DATA;
                    TxHeader.ExtId = 0x05010101;
                    TxHeader.DLC = 2;
                    TxHeader.TransmitGlobalTime = DISABLE; 

                    TxData[0] = 0x00;
                    TxData[1] = 0x00;

                    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
                }
                else if (RxData[0] == ENABLE){
                    Master_Command = Master_Enable;

                    TxHeader.IDE   = CAN_ID_EXT;
                    TxHeader.RTR   = CAN_RTR_DATA;
                    TxHeader.ExtId = 0x05010101;
                    TxHeader.DLC = 2;
                    TxHeader.TransmitGlobalTime = DISABLE; 

                    TxData[0] = 0x01;
                    TxData[1] = 0x00;

                    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
                }
            }
            break;
        }

        case RESET :{
    
            if (is_disabled()){
                send_disable_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
                break;
            }

            /* 清除尚未确认的取块/放块点击。 */
            s_click_signal = 0U;
            s_click_count = 0U;
            s_last_click_tick = 0U;
            
            send_receive_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
            Master_Command = Master_Reset;
            
            break;
        }

        case ERROR:{
             if (RxHeader.DLC != 2U)
            {
                break;
            }
            send_disable_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
            break;
        }

        case LEFT_START_PICK:{
            
            if (is_disabled()){
                send_disable_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
                break;
            }
            
            Comm_RecordClick(signal);
            break;
        }

        // case 0x05:{
        //     // 暂无，未来可能有
        // }

        // case 0x06:{
        //     // 暂无，未来可能有
        // }

        // case CONFIRM_GRAB:{
        //     if (RxData[1] == 0x00)
        //     {
        //         BlockVacuum_Grab();
        //     }
        //     break;
        // }

        case LEFT_START_PLACE:{

            if (is_disabled()){
                send_disable_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
                break;
            }

            Comm_RecordClick(signal);
            break;
        }

        // case CONFIRM_RELEASE:{
        //     if (RxData[1] == 0x00){
        //         BlockVacuum_Release();
        //     }
        //     break;
        // }

        case LEFT_CONFIRM_RELEASE:{

            if (is_disabled()){
                send_disable_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
                break;
            }

            
            send_receive_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
            Master_Command = Master_Release;            
            
            break;
        }

        // case STOP:{
        //     if (RxData[1] == 0x00){
        //         BlockArm_Stop();
        //     }
        // }
    }
}

// 接收关于右臂的消息
void Master_message_handler_right(CAN_HandleTypeDef *hcan, 
    CAN_RxHeaderTypeDef RxHeader, uint8_t *RxData)
{
    uint8_t signal = RxHeader.ExtId & 0xFFU;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailBox;
    uint8_t TxData[8];

    if (is_busy() && signal != 0x01 && signal != 0x00)
    {
        send_busy_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
        return;
    }

    switch (signal){

        case 0x00 :{
            if (RxHeader.DLC == 0U){
                TxHeader.IDE   = CAN_ID_EXT;
                TxHeader.RTR   = CAN_RTR_DATA;
                TxHeader.ExtId = 0x05010100;
                TxHeader.DLC = 2;
                TxHeader.TransmitGlobalTime = DISABLE; 

                if (is_disabled()){
                    TxData[0] = 0x02;
                    TxData[1] = 0x00;
                }else if (is_busy()){
                    TxData[0] = 0x01;
                    TxData[1] = 0x01;
                }else {
                    TxData[0] = 0x00;
                    TxData[1] = 0x01;
                }

                HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
            }
            break;
        }

        case 0x01 :{
            if (RxHeader.DLC == 2){
                if (RxData[0] == DISABLE){
                    Master_Command = Master_Disable;
                    TxHeader.IDE   = CAN_ID_EXT;
                    TxHeader.RTR   = CAN_RTR_DATA;
                    TxHeader.ExtId = 0x05010101;
                    TxHeader.DLC = 2;
                    TxHeader.TransmitGlobalTime = DISABLE; 

                    TxData[0] = 0x00;
                    TxData[1] = 0x00;

                    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
                }
                else if (RxData[0] == ENABLE){
                    Master_Command = Master_Enable;

                    TxHeader.IDE   = CAN_ID_EXT;
                    TxHeader.RTR   = CAN_RTR_DATA;
                    TxHeader.ExtId = 0x05010101;
                    TxHeader.DLC = 2;
                    TxHeader.TransmitGlobalTime = DISABLE; 

                    TxData[0] = 0x01;
                    TxData[1] = 0x00;

                    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
                }
            }
            break;
        }

        case RESET :{
            
            if (is_disabled()){
                send_disable_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
                break;
            }

            /* 清除尚未确认的取块/放块点击。 */
            s_click_signal = 0U;
            s_click_count = 0U;
            s_last_click_tick = 0U;
            
            send_receive_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
            Master_Command = Master_Reset;
            
            break;
        }

        case ERROR:{
             if (RxHeader.DLC != 2U)
            {
                break;
            }
            send_disable_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
            break;
        }

        case RIGHT_START_PICK:{

            if (is_disabled()){
                send_disable_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
                break;
            }

            Comm_RecordClick(signal);
            break;
        }

        // case 0x05:{
        //     // 暂无，未来可能有
        // }

        // case 0x06:{
        //     // 暂无，未来可能有
        // }

        // case CONFIRM_GRAB:{
        //     if (RxData[1] == 0x00)
        //     {
        //         BlockVacuum_Grab();
        //     }
        //     break;
        // }

        case RIGHT_START_PLACE:{

            if (is_disabled()){
                send_disable_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
                break;
            }

            Comm_RecordClick(signal);
            break;
        }

        // case CONFIRM_RELEASE:{
        //     if (RxData[1] == 0x00){
        //         BlockVacuum_Release();
        //     }
        //     break;
        // }

        case RIGHT_CONFIRM_RELEASE:{

            if (is_disabled()){
                send_disable_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
                break;
            }

            
            send_receive_message(hcan, &TxHeader, TxData, &TxMailBox, signal);
            Master_Command = Master_Release;
            
            break;
        }

        // case STOP:{
        //     if (RxData[1] == 0x00){
        //         BlockArm_Stop();
        //     }
        // }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];    

    if (hcan->Instance == CAN1) 
    {
        // 从 FIFO 0 把数据捞出来，存到 RxData 数组里
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
        #if USE_DJ && (MOTOR_DJI_CAN_BUS == 0U)
            DJmotor_Receive(RxHeader, RxData);
        #endif
        }
    } else if (hcan->Instance == CAN2) 
    {
        
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // 处理 CAN2 的消息...
        }
    }
    
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
        {
            if (RxHeader.IDE == CAN_ID_EXT)
            {
                if (RxHeader.ExtId >> 8 == 0x010105)
                {
                    // Master_message_handler_left(hcan, RxHeader, RxData);
                    Master_message_handler_right(hcan, RxHeader, RxData);
                     /*如果在右臂上烧录代码，则把这个从注释里捞出来并注释掉上一句*/
                }
            }          
        }
    }
    else if (hcan->Instance == CAN2)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
        {
        #if USE_ZMDR
            ZdriveReceive(RxHeader, RxData, 1U);
        #endif
        }
    }
}
