#ifndef PICK_BLOCK_TASK_H
#define PICK_BLOCK_TASK_H

/* 拾取大地块 Task 的流程状态。 */
typedef enum
{
    PICK_BLOCK_TASK_IDLE,
    PICK_BLOCK_TASK_MOVE_TO_PICK_PREPARE,
    PICK_BLOCK_TASK_MANUAL_ALIGN,
    PICK_BLOCK_TASK_GRAB,
    PICK_BLOCK_TASK_MOVE_TO_SAFE,
    PICK_BLOCK_TASK_DONE,
    PICK_BLOCK_TASK_FAULT,
    PICK_BLOCK_TASK_ABORTED,
    PICK_BLOCK_TASK_RESET_TO_SAFE
} PickBlockTaskState_t;


void PickBlockTask_Init(void);

/* 启动天空块的拾取流程。 */
void PickBlockTask_StartLowPick(void);
/* 启动一层大地块的拾取流程。 */
void PickBlockTask_StartHighPick(void);
/* 启动二层大地块的拾取流程。 */
void PickBlockTask_StartSecondPick(void);

/* 操作手确认侧面对准完成，允许 Task 开始建立真空。 */
void PickBlockTask_ConfirmGrab(void);

/* 操作手发现异常状况，用于停止当前的拾取任务。 */
void PickBlockTask_Stop(void);

/* 重置任务状态为 IDLE并复位机械臂，允许重新启动该任务。*/
void PickBlockTask_Reset(void);

PickBlockTaskState_t PickBlockTask_GetState(void);
void PickBlockTask_Process(void);

#endif /* PICK_BLOCK_TASK_H */
