#include "PickBlockTask.h"

#include <stdbool.h>

#include "BlockArm.h"
#include "BlockVacuum.h"

PickBlockTaskState_t pick_block_task_state;
uint16_t grab_count = 0;

/* 判断当前 BlockArm 状态是否允许发起新的自动动作。 */
static bool PickBlockTask_ArmCanStart(void)
{
    BlockArmState_t arm_state = BlockArm_GetState();

    return arm_state == BLOCK_ARM_READY ||
           arm_state == BLOCK_ARM_REACHED ||
           arm_state == BLOCK_ARM_STOPPED;
}

/* 判断当前 Task 生命周期是否允许重新启动。 */
static bool PickBlockTask_CanStart(void)
{
    return pick_block_task_state == PICK_BLOCK_TASK_IDLE ||
           pick_block_task_state == PICK_BLOCK_TASK_DONE;
}

void PickBlockTask_Init(void)
{
    grab_count = 0U;
    pick_block_task_state = PICK_BLOCK_TASK_IDLE;
}

/*启动低位取块任务*/
void PickBlockTask_StartLowPick(void)
{
    if (!PickBlockTask_CanStart() ||
        !PickBlockTask_ArmCanStart() ||
        (BlockVacuum_GetState() != BLOCK_VACUUM_RELEASED && BlockVacuum_GetState() != BLOCK_VACUUM_GRABBED)
    )
    {
        return;
    }

    BlockArm_MoveToLowPickReady();
    pick_block_task_state = PICK_BLOCK_TASK_MOVE_TO_PICK_PREPARE;
}

/*启动高位取块任务*/
void PickBlockTask_StartHighPick(void)
{
    if (!PickBlockTask_CanStart() ||
        !PickBlockTask_ArmCanStart() ||
        (BlockVacuum_GetState() != BLOCK_VACUUM_RELEASED && BlockVacuum_GetState() != BLOCK_VACUUM_GRABBED)
    )
    {
        return;
    }

    BlockArm_MoveToHighPickReady();
    pick_block_task_state = PICK_BLOCK_TASK_MOVE_TO_PICK_PREPARE;
}

/*启动二层取块任务*/
void PickBlockTask_StartSecondPick(void)
{
    if (!PickBlockTask_CanStart() ||
        !PickBlockTask_ArmCanStart() ||
        (BlockVacuum_GetState() != BLOCK_VACUUM_RELEASED && BlockVacuum_GetState() != BLOCK_VACUUM_GRABBED)
    )
    {
        return;
    }

    BlockArm_MoveToSecondPickReady();
    pick_block_task_state = PICK_BLOCK_TASK_MOVE_TO_PICK_PREPARE;
}

/*微调完毕后用于确认抓取动作，吸盘开始抓取*/
void PickBlockTask_ConfirmGrab(void)
{
    if (pick_block_task_state != PICK_BLOCK_TASK_MANUAL_ALIGN)
    {
        return;
    }

    BlockArm_StopFineAdjust();
    BlockVacuum_Grab();
    pick_block_task_state = PICK_BLOCK_TASK_GRAB;
}

PickBlockTaskState_t PickBlockTask_GetState(void)
{
    return pick_block_task_state;
}

void PickBlockTask_Process(void)
{
    BlockArmState_t arm_state = BlockArm_GetState();
    BlockVacuumState_t vacuum_state = BlockVacuum_GetState();

    if (pick_block_task_state != PICK_BLOCK_TASK_IDLE &&
        pick_block_task_state != PICK_BLOCK_TASK_DONE &&
        pick_block_task_state != PICK_BLOCK_TASK_FAULT &&
        (arm_state == BLOCK_ARM_FAULT ||
         vacuum_state == BLOCK_VACUUM_FAULT))
    {
        pick_block_task_state = PICK_BLOCK_TASK_FAULT;
        return;
    }

    switch (pick_block_task_state)
    {
        case PICK_BLOCK_TASK_IDLE:
            break;

        case PICK_BLOCK_TASK_MOVE_TO_PICK_PREPARE:
            if (arm_state == BLOCK_ARM_REACHED)
            {
                grab_count = 0;
                pick_block_task_state = PICK_BLOCK_TASK_MANUAL_ALIGN;
            }
            break;

         case PICK_BLOCK_TASK_MANUAL_ALIGN:
            /* 等待操作手微调并调用 PickBlockTask_ConfirmGrab() */
            if (grab_count++ >= 150)
            {
                PickBlockTask_ConfirmGrab();
            }
            //目前是移到大概位置后直接抓取，不需要等待微调
            break;

        case PICK_BLOCK_TASK_GRAB:
            if (vacuum_state == BLOCK_VACUUM_GRABBED)
            {
                
                // BlockArm_MoveToSafe();
                pick_block_task_state = PICK_BLOCK_TASK_DONE;
            }
            break;

        // case PICK_BLOCK_TASK_MOVE_TO_SAFE:
        //     if (arm_state == BLOCK_ARM_REACHED)
        //     {
        //         pick_block_task_state = PICK_BLOCK_TASK_DONE;
        //     }
        //     break;
        
        case PICK_BLOCK_TASK_RESET_TO_SAFE:
            if (arm_state == BLOCK_ARM_READY)
            {
                pick_block_task_state = PICK_BLOCK_TASK_IDLE;
            }
            break;

        case PICK_BLOCK_TASK_DONE:
        case PICK_BLOCK_TASK_FAULT:
            break;

        /* 任务完成或异常，等待外部重新启动任务*/    
        case PICK_BLOCK_TASK_ABORTED:
            break;

        default:
            pick_block_task_state = PICK_BLOCK_TASK_FAULT;
            break;
    }
}

void PickBlockTask_Stop(void)
{
    if (pick_block_task_state == PICK_BLOCK_TASK_IDLE ||
        pick_block_task_state == PICK_BLOCK_TASK_DONE ||
        pick_block_task_state == PICK_BLOCK_TASK_FAULT ||
        pick_block_task_state == PICK_BLOCK_TASK_ABORTED)
    {
        return;
    }
    BlockArm_Stop();
    pick_block_task_state = PICK_BLOCK_TASK_ABORTED;
}

/*只复位任务状态，配合BlockArm_Reset()使用*/
void PickBlockTask_Reset(void)
{
    grab_count = 0U;
    pick_block_task_state = PICK_BLOCK_TASK_RESET_TO_SAFE;
}