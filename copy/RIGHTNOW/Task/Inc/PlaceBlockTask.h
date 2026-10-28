#ifndef PLACE_BLOCK_TASK_H
#define PLACE_BLOCK_TASK_H

/* 放置大地块 Task 的流程状态。 */
typedef enum
{
    PLACE_BLOCK_TASK_IDLE,
    PLACE_BLOCK_TASK_MOVE_TO_PLACE_PREPARE,
    PLACE_BLOCK_TASK_MANUAL_ALIGN,
    PLACE_BLOCK_TASK_RELEASE,
    PLACE_BLOCK_TASK_MOVE_TO_SAFE,

    PLACE_BLOCK_TASK_DONE,
    PLACE_BLOCK_TASK_FAULT,
    PLACE_BLOCK_TASK_ABORTED,
    PLACE_BLOCK_TASK_RESET_TO_SAFE
} PlaceBlockTaskState_t;

/* 初始化放置 Task 自身状态，不重复初始化 Mechanism。 */
void PlaceBlockTask_Init(void);

/* 启动把大地块放到底层的流程。 */
void PlaceBlockTask_StartBottom(void);

/* 启动把大地块放到第一层的流程。 */
void PlaceBlockTask_StartLevel1(void);

/* 启动把大地块放到第二层的流程。 */
void PlaceBlockTask_StartLevel2(void);

/* 操作手确认方块已有下方支撑，允许 Task 释放真空。 */
void PlaceBlockTask_ConfirmRelease(void);
    
/*操作手发现异常状况，用于停止当前的放置任务*/
void PlaceBlockTask_Stop(void);

/*重置任务状态为 IDLE并复位机械臂，允许重新启动该任务。*/
void PlaceBlockTask_Reset(void);

PlaceBlockTaskState_t PlaceBlockTask_GetState(void);
void PlaceBlockTask_Process(void);

#endif /* PLACE_BLOCK_TASK_H */
