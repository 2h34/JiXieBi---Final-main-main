#ifndef BLOCK_ARM_H
#define BLOCK_ARM_H

#include <stdint.h>

#include "DJmotor.h"
#include "ZDrive.h"

/* BlockArm 对外运行状态。 */
typedef enum
{
    BLOCK_ARM_DISABLED,   /* 调试急停:彻底失能,不再下发任何目标 */
    BLOCK_ARM_UNHOMED,
    BLOCK_ARM_HOMING,
    BLOCK_ARM_HOME_TO_SAFE,
    BLOCK_ARM_READY,
    BLOCK_ARM_MOVING,
    BLOCK_ARM_REACHED,
    BLOCK_ARM_STOPPED,
    BLOCK_ARM_FAULT
} BlockArmState_t;

/* 人工微调时，操作手希望吸盘末端移动的方向。 */
typedef enum
{
    BLOCK_ARM_FINE_FORWARD,
    BLOCK_ARM_FINE_BACKWARD,
    BLOCK_ARM_FINE_UP,
    BLOCK_ARM_FINE_DOWN
} BlockArmFineAdjustDirection_t;


/*
 * 绑定模板中实际使用的一台 DJI 电机和一台 ZDrive 电机。
 * 初始化后仍需执行 Home；本函数不直接启动电机。
 */
void BlockArm_Init(DJMotor *dji_motor, Zdrive *zdrive_motor);

/* 启动 BlockArm 的归零流程，直到两个电机都到达归零位置。
 * 归零完成后，BlockArm 进入HOME_TO_SAFE状态，等待移动到安全位置后，进入 READY 状态。
 * 若当前状态不允许归零，则忽略本调用。*/
void BlockArm_Home(void);

/* 停止 BlockArm 的当前动作，使其进入 STOPPED 状态。
 * 若当前状态不允许停止，则忽略本调用。*/
void BlockArm_Stop(void);


/* 移动到单层天空块的取块粗定位姿态。 */
void BlockArm_MoveToLowPickReady(void);

/* 移动到双层上方大地块的取块粗定位姿态。 */
void BlockArm_MoveToHighPickReady(void);

/* 移动到二层大地块的取块粗定位姿态。 */
void BlockArm_MoveToSecondPickReady(void);

        
/* 移动到放置位置的粗定位姿态。 */
void BlockArm_MoveToPlaceBottomReady(void);
void BlockArm_MoveToPlaceLevel1Ready(void);
void BlockArm_MoveToPlaceLevel2Ready(void);

/* 启动到安全搬运姿态的复合运动。 */
void BlockArm_MoveToSafe(void);

/* 按指定末端方向启动低速人工微调（需一直按住）*/
void BlockArm_StartFineAdjust(BlockArmFineAdjustDirection_t direction);

/* 停止人工微调，并由后续实现保持当前机构姿态（松开按键后执行） */
void BlockArm_StopFineAdjust(void);

/* 启动回 Safe 的复位动作；到达 Safe 后由 Process() 进入 READY。 */
void BlockArm_Reset(void);

/* 彻底失能:两台电机都切到 Disable 模式,并进入 DISABLED 状态。
 * DISABLED 状态下 Process() 不再下发任何目标,用于调试时的紧急断电。 */
void BlockArm_Disable(void);
/*用于解除DISABLE*/
void BlockArm_Enable(void);
/*用于判断是否上电*/
bool is_disabled(void);
/*用于判断是否忙碌*/
bool is_busy(void);


BlockArmState_t BlockArm_GetState(void);
void BlockArm_Process(void);

#endif /* BLOCK_ARM_H */
