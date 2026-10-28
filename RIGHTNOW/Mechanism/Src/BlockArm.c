#include "BlockArm.h"

#include <stdbool.h>
#include <stddef.h>
#include <math.h>

/** 人工微调的间隔时间，单位为毫秒 */
#define BLOCK_ARM_FINE_ADJUST_INTERVAL_MS 20U

/** 连续到位计数器的阈值 */
#define BLOCK_ARM_REACHED_COUNT 10U

/* 自动粗定位动作的目标位置，只在 BlockArm 内部使用。 */
typedef enum
{
    BLOCK_ARM_TARGET_LOW_PICK_READY,
    BLOCK_ARM_TARGET_HIGH_PICK_READY,
    BLOCK_ARM_TARGET_SECOND_PICK_READY,
    BLOCK_ARM_TARGET_PLACE_BOTTOM_READY,
    BLOCK_ARM_TARGET_PLACE_LEVEL1_READY,
    BLOCK_ARM_TARGET_PLACE_LEVEL2_READY,
    BLOCK_ARM_TARGET_SAFE
} BlockArmTarget_t;

/* 不同粗定位姿态附近可使用不同的双电机微调组合。 */
typedef enum
{
    BLOCK_ARM_FINE_PROFILE_NONE,
    BLOCK_ARM_FINE_PROFILE_LOW_PICK,
    BLOCK_ARM_FINE_PROFILE_HIGH_PICK,
    BLOCK_ARM_FINE_PROFILE_SECOND_PICK,
    BLOCK_ARM_FINE_PROFILE_PLACE_BOTTOM,
    BLOCK_ARM_FINE_PROFILE_PLACE_LEVEL1,
    BLOCK_ARM_FINE_PROFILE_PLACE_LEVEL2
} BlockArmFineAdjustProfile_t;

typedef struct
{
    DJMotor *dji_motor;
    Zdrive *zdrive_motor;

    BlockArmState_t state;
    float dji_current_position;
    float zdrive_current_position;
    float dji_target_position;
    float zdrive_target_position;
    float dji_zero_position;  /** DJI 电机零点偏移 */
    float zdrive_zero_position; /** ZDrive 电机零点偏移 */

    float tolerance; /* 位置误差容忍度，单位为角度。 */
    uint16_t reached_count;  /* 连续到位计数器，用于判断是否真正到位。 */
    uint32_t homing_count;  /*归零计数器，用于判断超时*/
    uint32_t motion_count; /* 记录连续运动周期数，用于判断超时。 */

    BlockArmFineAdjustProfile_t fine_adjust_profile;  /* 当前粗定位姿态对应的微调组合。 */
    BlockArmFineAdjustDirection_t fine_adjust_direction; /* 当前微调方向。 */
    bool fine_adjust_active; /* 标记是否正在执行人工微调动作 */

    uint32_t fine_adjust_last_ms; /*记录上次微调的时间 */

    bool reset_to_safe_active; /* 标记是否正在执行复位到安全姿态的动作 */
} BlockArm_t;

BlockArm_t block_arm; /*后续考虑升级为结构体数组*/

/*检查是否绑定两个正确电机*/
static bool BlockArm_DriversBound(void)
{
    return block_arm.dji_motor != NULL &&
           block_arm.zdrive_motor != NULL;
}

/* 读取模板驱动已经更新的输出端位置反馈（状态机周期更新）*/
static void BlockArm_UpdateFeedback(void)
{
    if (block_arm.dji_motor != NULL)
    {
        block_arm.dji_current_position =
            block_arm.dji_motor->valNow.angle_deg - block_arm.dji_zero_position;
    }

    if (block_arm.zdrive_motor != NULL)
    {
        block_arm.zdrive_current_position =
            block_arm.zdrive_motor->valReal.pos_deg - block_arm.zdrive_zero_position;
    }
}

/*
 * 两种电机均由电机层接口切换到位置模式并接收输出端角度目标。
 * 模板电机切换模式时会暂时覆盖目标为当前位置，因此机构处于运动/保持时，
 * 每次 Process() 都重复下发本次目标；模式确认后的下一周期即可生效。
 */
static void BlockArm_ApplyPositionTargets(void)
{
    /* 将目标位置加上零点偏移后下发给两个电机 */
    DJmotor_SetPositionTarget(block_arm.dji_motor,block_arm.dji_target_position + block_arm.dji_zero_position);
    Zdrive_SetPositionTarget(block_arm.zdrive_motor,block_arm.zdrive_target_position + block_arm.zdrive_zero_position);
}

/*设置粗固定姿态目标位置（设置blockarm的目标位置）*/
static void BlockArm_SetAutoTarget(BlockArmTarget_t target)
{
    switch (target)
    {
        case BLOCK_ARM_TARGET_LOW_PICK_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_LOW_PICK;
            /* TODO: 写入低位取块姿态对应的 DJI / ZDrive 目标。 */
            block_arm.dji_target_position = 12.0f;  
            block_arm.zdrive_target_position = 116.0f;
            break;

        case BLOCK_ARM_TARGET_HIGH_PICK_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_HIGH_PICK;
            /* TODO: 写入高位取块姿态对应的 DJI / ZDrive 目标。 */
            block_arm.dji_target_position = 28.0f;  
            block_arm.zdrive_target_position = 105.0f;
            break;

        case BLOCK_ARM_TARGET_SECOND_PICK_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_SECOND_PICK;
            /* TODO: 写入二层取块姿态对应的 DJI / ZDrive 目标。 */
            block_arm.dji_target_position = -20.0f;  
            block_arm.zdrive_target_position = 45.0f;
            break;

        case BLOCK_ARM_TARGET_PLACE_BOTTOM_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_PLACE_BOTTOM;
            /* TODO: 写入底层放块姿态对应的 DJI / ZDrive 目标。 */
            block_arm.dji_target_position = 20.0f;  
            block_arm.zdrive_target_position = 95.0f;
            break;

        case BLOCK_ARM_TARGET_PLACE_LEVEL1_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_PLACE_LEVEL1;
            /* TODO: 写入第一层放块姿态对应的 DJI / ZDrive 目标。 */
            block_arm.dji_target_position = -40.0f;  
            block_arm.zdrive_target_position = 50.0f;
            break;

        case BLOCK_ARM_TARGET_PLACE_LEVEL2_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_PLACE_LEVEL2;
            /* TODO: 写入第二层放块姿态对应的 DJI / ZDrive 目标。 */
            block_arm.dji_target_position = -84.0f;  
            block_arm.zdrive_target_position = 44.0f;
            break;

        case BLOCK_ARM_TARGET_SAFE:
            // if (block_arm.fine_adjust_profile == BLOCK_ARM_FINE_PROFILE_PLACE_LEVEL2)
            // {
            //     /* 二层放置后的专用安全位置，避免碰到物块 */
            //     block_arm.dji_target_position = -82.0f;  /* 实测角度 */
            //     block_arm.zdrive_target_position = 44.0f;  /* 实测角度 */
            // }
            // else
            // {
            //     /* 其他粗定位姿态的通用安全位置 */ // }
            block_arm.dji_target_position = -25.0f;  /* 实测角度 */
            block_arm.zdrive_target_position = 40.0f;  /* 实测角度 */
           
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_NONE;
            break;

        default:
            block_arm.state = BLOCK_ARM_FAULT;
            return;
    }
}

/* 启动自动移动到指定目标姿态 */
static void BlockArm_StartAutoMove(BlockArmTarget_t target)
{
    if ((block_arm.state != BLOCK_ARM_READY &&
         block_arm.state != BLOCK_ARM_REACHED &&
         block_arm.state != BLOCK_ARM_STOPPED) ||
        !BlockArm_DriversBound())
    {
        return;
    }

    /* 复位到安全姿态的动作标记清除，避免干扰后续动作。
    每次普通自动运动先取消旧的“回 Safe 复位任务”身份；
    只有 BlockArm_Reset() 才能重新赋予这次运动该身份。*/
    block_arm.reset_to_safe_active = false;

    BlockArm_UpdateFeedback(); /*立刻更新当前位置，以便后续操作*/

    /*
     * 未填入机械标定目标前，先以当前位置作为临时保持目标。
     * 这避免框架代码把默认 0 度误下发给真实机构。
     */
    block_arm.dji_target_position = block_arm.dji_current_position;
    block_arm.zdrive_target_position = block_arm.zdrive_current_position;
    BlockArm_SetAutoTarget(target);

    block_arm.fine_adjust_active = false;
    block_arm.reached_count = 0U;
    block_arm.motion_count = 0U;
    block_arm.state = BLOCK_ARM_MOVING;
}

static bool BlockArm_IsReached(void)
{
    /*
     * 1. 比较 DJI / ZDrive 反馈与本次目标；
     * 2. 连续满足误差范围后返回 true；
     * 3. 不允许只根据一次采样判定到位。
     */
    if (fabsf(block_arm.dji_current_position - block_arm.dji_target_position) < block_arm.tolerance &&
        fabsf(block_arm.zdrive_current_position - block_arm.zdrive_target_position) < block_arm.tolerance)
    {
        block_arm.reached_count++;
        if (block_arm.reached_count >= BLOCK_ARM_REACHED_COUNT)
        {
            return true;
        }
    }
    else
    {
        block_arm.reached_count = 0U;
    }
    return false;
}

/* 执行人工微调动作，按指定方向移动末端。
 * 该函数在 Process() 中周期调用（只有按下按键时调用），
 * 间隔时间由 BLOCK_ARM_FINE_ADJUST_INTERVAL_MS 决定。
 */
static void BlockArm_ApplyFineAdjust(void)
{
    uint32_t now_ms = HAL_GetTick();
    if (now_ms - block_arm.fine_adjust_last_ms <
        BLOCK_ARM_FINE_ADJUST_INTERVAL_MS)
    {
        return;
    }
    block_arm.fine_adjust_last_ms = now_ms;

    switch (block_arm.fine_adjust_profile)
    {
        case BLOCK_ARM_FINE_PROFILE_LOW_PICK:
            /* TODO: 低位取块区域的前、后、上、下双电机组合。 */
        {
            switch (block_arm.fine_adjust_direction)
            {
            case BLOCK_ARM_FINE_FORWARD:
            {
                /* 设置两个电机的组合移动角度 */
                block_arm.dji_target_position += 1.0f; /* 例如前进 1 度 */
                block_arm.zdrive_target_position += 1.0f; /* 例如前进 1 度 */
            }
            break;

            case BLOCK_ARM_FINE_BACKWARD:
            /* 设置两个电机的组合移动角度 */
            break;

            case BLOCK_ARM_FINE_UP:
            /* 设置两个电机的组合移动角度 */
            break;

            case BLOCK_ARM_FINE_DOWN:
            /* 设置两个电机的组合移动角度 */
            break;
            }
        }
            break;

        case BLOCK_ARM_FINE_PROFILE_HIGH_PICK:
            /* TODO: 高位取块区域的前、后、上、下双电机组合。 */
            break;

        case BLOCK_ARM_FINE_PROFILE_SECOND_PICK:
            /* TODO: 二层取块区域的前、后、上、下双电机组合。 */
            break;

        case BLOCK_ARM_FINE_PROFILE_PLACE_BOTTOM:
            /* TODO: 底层放块区域的前、后、上、下双电机组合。 */
            break;

        case BLOCK_ARM_FINE_PROFILE_PLACE_LEVEL1:
            /* TODO: 第一层放块区域的前、后、上、下双电机组合。 */
            break;

        case BLOCK_ARM_FINE_PROFILE_PLACE_LEVEL2:
            /* TODO: 第二层放块区域的前、后、上、下双电机组合。 */
            break;

        case BLOCK_ARM_FINE_PROFILE_NONE:
        default:
            break;
    }
}

/* 将当前反馈设为新的保持目标，仍通过位置模式提供保持力。 */
static void BlockArm_StopAndHold(void)
{
    if (!BlockArm_DriversBound())
    {
        block_arm.state = BLOCK_ARM_FAULT;
        return;
    }

    BlockArm_UpdateFeedback();
    block_arm.dji_target_position = block_arm.dji_current_position;
    block_arm.zdrive_target_position = block_arm.zdrive_current_position;
}

void BlockArm_Init(DJMotor *dji_motor, Zdrive *zdrive_motor)
{
    block_arm.reset_to_safe_active = false;

    block_arm.dji_motor = dji_motor;
    block_arm.zdrive_motor = zdrive_motor;
    block_arm.state = BLOCK_ARM_DISABLED; /*初始失能*/

    block_arm.dji_current_position = 0.0f;
    block_arm.zdrive_current_position = 0.0f;
    block_arm.dji_target_position = 0.0f;
    block_arm.zdrive_target_position = 0.0f;
    block_arm.dji_zero_position = 0.0f;
    block_arm.zdrive_zero_position = 0.0f;
    block_arm.tolerance = 1.0f;
    block_arm.reached_count = 0U;
    block_arm.homing_count = 0U;
    block_arm.motion_count = 0U;
    block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_NONE;
    block_arm.fine_adjust_direction = BLOCK_ARM_FINE_FORWARD;
    block_arm.fine_adjust_active = false;
}

void BlockArm_Home(void)
{
    if ((block_arm.state != BLOCK_ARM_UNHOMED &&
         block_arm.state != BLOCK_ARM_STOPPED) ||
        !BlockArm_DriversBound())
    {
        return;
    }

    block_arm.homing_count = 0U;
    block_arm.reached_count = 0U;
    block_arm.fine_adjust_active = false;
    block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_NONE;
    block_arm.state = BLOCK_ARM_HOMING;

     DJmotor_SetZeromode(block_arm.dji_motor);
     Zdrive_SetZeromode(block_arm.zdrive_motor);
}

void BlockArm_Stop(void)
{
    if (block_arm.state == BLOCK_ARM_DISABLED)
    {
        return;
    }
    if (block_arm.state == BLOCK_ARM_HOMING)
    {
        BlockArm_StopAndHold();
        if (block_arm.state == BLOCK_ARM_FAULT)
        {
            return;
        }
        BlockArm_ApplyPositionTargets();  /*暂停归零并使能保持原位置*/
        block_arm.fine_adjust_active = false;
        block_arm.state = BLOCK_ARM_UNHOMED;
    }
    else if (block_arm.state == BLOCK_ARM_UNHOMED)
    {
        /* 已经在未归零状态，无需进一步操作。 */
    }
    else if (block_arm.state == BLOCK_ARM_FAULT)
    {
        /* 已经在故障状态，无需进一步操作。 */
    }
    else
    {
        BlockArm_StopAndHold();
        if (block_arm.state == BLOCK_ARM_FAULT)
        {
            return;
        }
        block_arm.fine_adjust_active = false;
        block_arm.state = BLOCK_ARM_STOPPED;
    }
}



/*均为对外接口*/
void BlockArm_MoveToLowPickReady(void)
{
    BlockArm_StartAutoMove(BLOCK_ARM_TARGET_LOW_PICK_READY);
}

void BlockArm_MoveToHighPickReady(void)
{
    BlockArm_StartAutoMove(BLOCK_ARM_TARGET_HIGH_PICK_READY);
}

void BlockArm_MoveToSecondPickReady(void)
{
    BlockArm_StartAutoMove(BLOCK_ARM_TARGET_SECOND_PICK_READY);
}

void BlockArm_MoveToPlaceBottomReady(void)
{
    BlockArm_StartAutoMove(BLOCK_ARM_TARGET_PLACE_BOTTOM_READY);
}

void BlockArm_MoveToPlaceLevel1Ready(void)
{
    BlockArm_StartAutoMove(BLOCK_ARM_TARGET_PLACE_LEVEL1_READY);
}

void BlockArm_MoveToPlaceLevel2Ready(void)
{
    BlockArm_StartAutoMove(BLOCK_ARM_TARGET_PLACE_LEVEL2_READY);
}

void BlockArm_MoveToSafe(void)
{
    BlockArm_StartAutoMove(BLOCK_ARM_TARGET_SAFE);
    /* TODO: 必要时在此目标内部扩展经过验证的撤离路径。 */
}

/* 按指定末端方向启动低速人工微调（需一直按住）*/
void BlockArm_StartFineAdjust(BlockArmFineAdjustDirection_t direction)
{
    if (direction > BLOCK_ARM_FINE_DOWN ||
        block_arm.fine_adjust_profile == BLOCK_ARM_FINE_PROFILE_NONE)
    {
        return;
    }

    if (block_arm.fine_adjust_active)
    {
        block_arm.fine_adjust_direction = direction;
        BlockArm_ApplyFineAdjust();
        return;
    }

    if (block_arm.state != BLOCK_ARM_REACHED &&
        block_arm.state != BLOCK_ARM_STOPPED)
    {
        return;
    }

    block_arm.fine_adjust_direction = direction;
    block_arm.fine_adjust_active = true;
    /* 让本次按下时，首次微调必定立即通过 20 ms 判断。 */
    block_arm.fine_adjust_last_ms = HAL_GetTick() - BLOCK_ARM_FINE_ADJUST_INTERVAL_MS;
    block_arm.state = BLOCK_ARM_MOVING;
    BlockArm_ApplyFineAdjust();
}

void BlockArm_StopFineAdjust(void)
{
    if (!block_arm.fine_adjust_active)
    {
        return;
    }

    BlockArm_StopAndHold();
    if (block_arm.state == BLOCK_ARM_FAULT)
    {
        return;
    }
    block_arm.fine_adjust_active = false;
    block_arm.state = BLOCK_ARM_STOPPED;
}

BlockArmState_t BlockArm_GetState(void)
{
    return block_arm.state;
}

void BlockArm_Process(void)
{
    BlockArm_UpdateFeedback();

    switch (block_arm.state)
    {
        case BLOCK_ARM_UNHOMED:
            break;

        case BLOCK_ARM_HOMING:
            block_arm.homing_count++;
            
            /* 检查两个电机是否都已完成寻零 */
            if (DJmotor_IsZeroDone(block_arm.dji_motor) && Zdrive_IsZeroDone(block_arm.zdrive_motor))
            {
                /* 两个电机寻零完成后，记录归零位置作为后续保持目标的零点偏移。 */
                block_arm.dji_zero_position = block_arm.dji_motor->valNow.angle_deg;
                block_arm.zdrive_zero_position = block_arm.zdrive_motor->valReal.pos_deg;
                
                block_arm.zdrive_motor->zero.done = false;

                /* TODO: 将电机目标位置设为安全位置。 */
                block_arm.dji_target_position = 0.0f;  /* 安全位置的 DJI 角度 */
                block_arm.zdrive_target_position = 10.0f; /* 安全位置的 ZDrive 角度 */

                block_arm.state = BLOCK_ARM_HOME_TO_SAFE;
            }
            else if (block_arm.homing_count > 10000U)
            {
                block_arm.state = BLOCK_ARM_FAULT;
            }
            break;
        
        case BLOCK_ARM_HOME_TO_SAFE:
            BlockArm_ApplyPositionTargets();
            if (BlockArm_IsReached())  /*归零后移动到安全位置并上电，转换为Ready状态*/
            {
                block_arm.state = BLOCK_ARM_READY;
            }
            break;

        case BLOCK_ARM_READY:
            // BlockArm_ApplyPositionTargets();
            break;

        case BLOCK_ARM_MOVING:
            if (block_arm.fine_adjust_active)
            {
                BlockArm_ApplyFineAdjust();
                BlockArm_ApplyPositionTargets();
            }
            else
            {
                BlockArm_ApplyPositionTargets();
                block_arm.motion_count++;

                if (BlockArm_IsReached())
                {
                    block_arm.state = BLOCK_ARM_REACHED;
                }

                /* TODO: 增加超时、掉线和机构异常判断。 */
            }
            break;

        case BLOCK_ARM_REACHED:
        {
            if (block_arm.reset_to_safe_active)  /* 如果是复位到安全姿态的动作，到达reached状态后转换为Ready状态 */
            {
                block_arm.state = BLOCK_ARM_READY;
                block_arm.fine_adjust_active = false;
                block_arm.reached_count = 0U;
                block_arm.motion_count = 0U;
                block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_NONE;
                block_arm.fine_adjust_direction = BLOCK_ARM_FINE_FORWARD;
                block_arm.reset_to_safe_active = false;
            }
        }
            break;
        
        case BLOCK_ARM_STOPPED:
            /* 停止状态，不执行任何动作，电机使能保持原姿势 */
            BlockArm_ApplyPositionTargets();
            break;

        case BLOCK_ARM_DISABLED:
            break;

        case BLOCK_ARM_FAULT:
            break;

        default:
            block_arm.state = BLOCK_ARM_FAULT;
            break;
    }
}

/*一般配合BlockArm_Stop()使用，用于将机械臂复位到安全位置*/
void BlockArm_Reset(void)
{
    if (block_arm.state != BLOCK_ARM_STOPPED && block_arm.state != BLOCK_ARM_REACHED)
    {
        return;
    }
    BlockArm_MoveToSafe();
    if (block_arm.state == BLOCK_ARM_MOVING)
    {
    block_arm.reset_to_safe_active = true;
    }
}

void BlockArm_Disable(void)
{
    // 紧急断电失能。需要重启程序或者Enable才能继续。
    if (!BlockArm_DriversBound())
    {
        block_arm.state = BLOCK_ARM_FAULT;
        return;
    }

    /* 两台电机都切到 Disable 模式,释放力矩。 */
    DJmotor_Disable(block_arm.dji_motor);
    Zdrive_DDisable(block_arm.zdrive_motor);

    block_arm.fine_adjust_active = false;
    block_arm.reset_to_safe_active = false;
    block_arm.state = BLOCK_ARM_DISABLED;
}

void BlockArm_Enable(void)
{
    if (block_arm.state != BLOCK_ARM_DISABLED)
    {
        return;
    }

    block_arm.state = BLOCK_ARM_UNHOMED;
}

bool is_disabled(void){
    return block_arm.state == BLOCK_ARM_DISABLED;
}

bool is_busy(void){
    switch (block_arm.state){
        case BLOCK_ARM_HOMING:
        case BLOCK_ARM_HOME_TO_SAFE:
        case BLOCK_ARM_MOVING:
            return true;
        default:
            break;
    }

    return false;
}