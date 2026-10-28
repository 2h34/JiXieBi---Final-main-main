#include "PlaceBlockTask.h"

#include <stdbool.h>

#include "BlockArm.h"
#include "BlockVacuum.h"

 PlaceBlockTaskState_t place_block_task_state;

/* 判断当前 BlockArm 状态是否允许发起新的自动动作。 */
static bool PlaceBlockTask_ArmCanStart(void)
{
    BlockArmState_t arm_state = BlockArm_GetState();

    return arm_state == BLOCK_ARM_READY ||
           arm_state == BLOCK_ARM_REACHED ||
           arm_state == BLOCK_ARM_STOPPED;
}

/* 判断当前 Task 生命周期是否允许重新启动。 */
static bool PlaceBlockTask_CanStart(void)
{
    return place_block_task_state == PLACE_BLOCK_TASK_IDLE ||
           place_block_task_state == PLACE_BLOCK_TASK_DONE;
}

void PlaceBlockTask_Init(void)
{
    place_block_task_state = PLACE_BLOCK_TASK_IDLE;
}

void PlaceBlockTask_StartBottom(void)
{
    if (!PlaceBlockTask_CanStart() ||
        !PlaceBlockTask_ArmCanStart() ||
        BlockVacuum_GetState() != BLOCK_VACUUM_GRABBED)
    {
        return;
    }

    BlockArm_MoveToPlaceBottomReady();
    place_block_task_state = PLACE_BLOCK_TASK_MOVE_TO_PLACE_PREPARE;
}

void PlaceBlockTask_StartLevel1(void)
{
    if (!PlaceBlockTask_CanStart() ||
        !PlaceBlockTask_ArmCanStart() ||
        BlockVacuum_GetState() != BLOCK_VACUUM_GRABBED)
    {
        return;
    }

    BlockArm_MoveToPlaceLevel1Ready();
    place_block_task_state = PLACE_BLOCK_TASK_MOVE_TO_PLACE_PREPARE;
}

void PlaceBlockTask_StartLevel2(void)
{
    if (!PlaceBlockTask_CanStart() ||
        !PlaceBlockTask_ArmCanStart() ||
        BlockVacuum_GetState() != BLOCK_VACUUM_GRABBED)
    {
        return;
    }

    BlockArm_MoveToPlaceLevel2Ready();
    place_block_task_state = PLACE_BLOCK_TASK_MOVE_TO_PLACE_PREPARE;
}

void PlaceBlockTask_ConfirmRelease(void)
{
    BlockArm_StopFineAdjust();
    BlockVacuum_Release();
    place_block_task_state = PLACE_BLOCK_TASK_RELEASE;
}

PlaceBlockTaskState_t PlaceBlockTask_GetState(void)
{
    return place_block_task_state;
}

void PlaceBlockTask_Process(void)
{
    BlockArmState_t arm_state = BlockArm_GetState();
    BlockVacuumState_t vacuum_state = BlockVacuum_GetState();

    if (place_block_task_state != PLACE_BLOCK_TASK_IDLE &&
        place_block_task_state != PLACE_BLOCK_TASK_DONE &&
        place_block_task_state != PLACE_BLOCK_TASK_FAULT &&
        (arm_state == BLOCK_ARM_FAULT ||
         vacuum_state == BLOCK_VACUUM_FAULT))
    {
        place_block_task_state = PLACE_BLOCK_TASK_FAULT;
        return;
    }

    switch (place_block_task_state)
    {
        case PLACE_BLOCK_TASK_IDLE:
            break;

        case PLACE_BLOCK_TASK_MOVE_TO_PLACE_PREPARE:
            if (arm_state == BLOCK_ARM_REACHED)
            {
                place_block_task_state = PLACE_BLOCK_TASK_MANUAL_ALIGN;
            }
            break;

        case PLACE_BLOCK_TASK_MANUAL_ALIGN:
            /* 等待操作手微调并调用 PlaceBlockTask_ConfirmRelease()。 */
            break;

        case PLACE_BLOCK_TASK_RELEASE:
            if (vacuum_state == BLOCK_VACUUM_RELEASED)
            {
                // BlockArm_MoveToSafe();
                place_block_task_state = PLACE_BLOCK_TASK_DONE;
            }
            break;

        // case PLACE_BLOCK_TASK_MOVE_TO_SAFE:
        //     if (arm_state == BLOCK_ARM_REACHED)
        //     {
        //         place_block_task_state = PLACE_BLOCK_TASK_DONE;
        //     }
        //     break;

        case PLACE_BLOCK_TASK_RESET_TO_SAFE:

            if (arm_state == BLOCK_ARM_READY)
            {
                place_block_task_state = PLACE_BLOCK_TASK_IDLE;
            }
            break;

        case PLACE_BLOCK_TASK_DONE:
        case PLACE_BLOCK_TASK_FAULT:
            break;

        /* 任务完成或异常，等待外部重新启动任务*/    
        case PLACE_BLOCK_TASK_ABORTED:
            break;

        default:
            place_block_task_state = PLACE_BLOCK_TASK_FAULT;
            break;
    }
}

void PlaceBlockTask_Stop(void)
{
    if (place_block_task_state == PLACE_BLOCK_TASK_IDLE ||
        place_block_task_state == PLACE_BLOCK_TASK_DONE ||
        place_block_task_state == PLACE_BLOCK_TASK_FAULT ||
        place_block_task_state == PLACE_BLOCK_TASK_ABORTED)
    {
        return;
    }

    BlockArm_Stop();
    place_block_task_state = PLACE_BLOCK_TASK_ABORTED;
}


/*只复位任务状态，配合BlockArm_Reset()使用*/
void PlaceBlockTask_Reset(void)
{
    place_block_task_state = PLACE_BLOCK_TASK_RESET_TO_SAFE;
}
