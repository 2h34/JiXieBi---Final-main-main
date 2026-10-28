#include "BlockArm.h"
#include "arm_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <math.h>

/** 人工微调的间隔时间，单位为毫秒 */
#define BLOCK_ARM_FINE_ADJUST_INTERVAL_MS 20U

/** 连续到位计数器的阈值 */
#define BLOCK_ARM_REACHED_COUNT 10U

/* 复位前用于识别当前粗定位姿态的单轴最大偏差。 */
#define BLOCK_ARM_RESET_POSE_TOLERANCE_DEG 5.0f

/* 3508轨迹速度前馈比例；0.30表示仅使用理论速度的30%，便于保守实机调试。 */
#define BLOCK_ARM_DJI_VELOCITY_FF_GAIN 0.30f

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
    float dji_target_velocity_deg_s;
    float zdrive_target_velocity_deg_s;
    float dji_zero_position;  /** DJI 电机零点偏移 */
    float zdrive_zero_position; /** ZDrive 电机零点偏移 */

    float tolerance; /* 位置误差容忍度，单位为角度。 */
    uint16_t reached_count;  /* 连续到位计数器，用于判断是否真正到位。 */
    uint32_t homing_count;  /*归零计数器，用于判断超时*/
    uint32_t motion_count; /* 记录连续运动周期数，用于判断超时。 */

    const ArmPose_t *active_motion_pose; /* non-NULL while a timed auto move is executing */
    uint32_t motion_duration_ms; /* 当前定时轨迹的持续时间 */
    uint32_t motion_start_ms;
    float motion_start_dji;
    float motion_start_zdrive;

    BlockArmFineAdjustProfile_t fine_adjust_profile;  /* 当前粗定位姿态对应的微调组合。 */
    BlockArmFineAdjustDirection_t fine_adjust_direction; /* 当前微调方向。 */
    bool fine_adjust_active; /* 标记是否正在执行人工微调动作 */

    uint32_t fine_adjust_last_ms; /*记录上次微调的时间 */

    bool reset_to_safe_active; /* 标记是否正在执行复位到安全姿态的动作 */
} BlockArm_t;

BlockArm_t block_arm; /*后续考虑升级为结构体数组*/

/* 左右臂姿态表指针：由编译宏选择（arm_config.h 声明、arm_poses.c 定义） */
static const ArmPose_t *ARM_POSES = NULL;

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
    float dji_target_with_zero =
        block_arm.dji_target_position + block_arm.dji_zero_position;
    float zdrive_target_with_zero =
        block_arm.zdrive_target_position + block_arm.zdrive_zero_position;

    /* 将目标位置加上零点偏移后下发给两个电机 */
    if (block_arm.active_motion_pose != NULL)
    {
        /*
         * 3508轨迹速度由deg/s除以6换算为输出轴rpm，再乘保守前馈比例。
         * AK80撤回未验证的PVT，轨迹期间仍按5 ms周期下发平滑位置目标。
         */
        DJmotor_SetPositionTargetWithFeedforward(
            block_arm.dji_motor,
            dji_target_with_zero,
            block_arm.dji_target_velocity_deg_s / 6.0f *
                BLOCK_ARM_DJI_VELOCITY_FF_GAIN);
    }
    else
    {
        DJmotor_SetPositionTarget(block_arm.dji_motor, dji_target_with_zero);
    }

    Zdrive_SetPositionTarget(block_arm.zdrive_motor, zdrive_target_with_zero);
}

/* Quintic time scaling: zero velocity and acceleration at both endpoints. */
static float BlockArm_SmoothStep5(float u)
{
    float u2;
    float u3;

    if (u <= 0.0f)
    {
        return 0.0f;
    }
    if (u >= 1.0f)
    {
        return 1.0f;
    }

    u2 = u * u;
    u3 = u2 * u;
    return 10.0f * u3 - 15.0f * u3 * u + 6.0f * u3 * u2;
}

/* Quintic time-scaling derivative ds/du; zero at both endpoints. */
static float BlockArm_SmoothStep5Derivative(float u)
{
    float u2;
    float one_minus_u;

    if (u <= 0.0f || u >= 1.0f)
    {
        return 0.0f;
    }

    u2 = u * u;
    one_minus_u = 1.0f - u;
    return 30.0f * u2 * one_minus_u * one_minus_u;
}

/* Update the current command without blocking the 5 ms BlockArm task. */
static void BlockArm_UpdateAutoMotion(void)
{
    const ArmPose_t *pose = block_arm.active_motion_pose;
    uint32_t duration_ms = block_arm.motion_duration_ms;
    uint32_t elapsed_ms;
    float u;
    float s;
    float ds_dt;

    if (pose == NULL || duration_ms == 0U)
    {
        return;
    }

    elapsed_ms = HAL_GetTick() - block_arm.motion_start_ms;
    if (elapsed_ms >= duration_ms)
    {
        block_arm.dji_target_position = pose->dji_deg;
        block_arm.zdrive_target_position = pose->zdrive_deg;
        block_arm.dji_target_velocity_deg_s = 0.0f;
        block_arm.zdrive_target_velocity_deg_s = 0.0f;
        block_arm.active_motion_pose = NULL;
        block_arm.motion_duration_ms = 0U;
        block_arm.reached_count = 0U;
        return;
    }

    u = (float)elapsed_ms / (float)duration_ms;
    s = BlockArm_SmoothStep5(u);
    ds_dt = BlockArm_SmoothStep5Derivative(u) *
            1000.0f / (float)duration_ms;

    block_arm.dji_target_position =
        block_arm.motion_start_dji +
        (pose->dji_deg - block_arm.motion_start_dji) * s;
    block_arm.zdrive_target_position =
        block_arm.motion_start_zdrive +
        (pose->zdrive_deg - block_arm.motion_start_zdrive) * s;
    block_arm.dji_target_velocity_deg_s =
        (pose->dji_deg - block_arm.motion_start_dji) * ds_dt;
    block_arm.zdrive_target_velocity_deg_s =
        (pose->zdrive_deg - block_arm.motion_start_zdrive) * ds_dt;
}

static bool BlockArm_IsNearPose(BlockArmTarget_t target, float tolerance)
{
    const ArmPose_t *pose = &ARM_POSES[target];

    return fabsf(block_arm.dji_current_position - pose->dji_deg) <= tolerance &&
           fabsf(block_arm.zdrive_current_position - pose->zdrive_deg) <= tolerance;
}

/*
 * 在已知粗定位姿态中选择与当前反馈最接近的一个。
 * 两个关节中任意一个超出识别容差时，不自动套用任何复位轨迹。
 */
static bool BlockArm_DetectCurrentPose(BlockArmTarget_t *detected_target)
{
    BlockArmTarget_t target;
    bool found = false;
    float best_error = 0.0f;

    if (detected_target == NULL)
    {
        return false;
    }

    for (target = BLOCK_ARM_TARGET_LOW_PICK_READY;
         target <= BLOCK_ARM_TARGET_SAFE;
         target = (BlockArmTarget_t)((uint32_t)target + 1U))
    {
        float dji_error;
        float zdrive_error;
        float total_error;

        if (!BlockArm_IsNearPose(target, BLOCK_ARM_RESET_POSE_TOLERANCE_DEG))
        {
            continue;
        }

        dji_error = block_arm.dji_current_position - ARM_POSES[target].dji_deg;
        zdrive_error = block_arm.zdrive_current_position - ARM_POSES[target].zdrive_deg;
        total_error = dji_error * dji_error + zdrive_error * zdrive_error;

        if (!found || total_error < best_error)
        {
            found = true;
            best_error = total_error;
            *detected_target = target;
        }
    }

    return found;
}

/* The currently fitted outbound trajectories are defined from the configured SAFE pose. */
static bool BlockArm_IsAtSafePose(void)
{
    return BlockArm_IsNearPose(BLOCK_ARM_TARGET_SAFE, block_arm.tolerance);
}

/*设置粗固定姿态目标位置（设置blockarm的目标位置）*/
static void BlockArm_SetAutoTarget(BlockArmTarget_t target)
{
    switch (target)
    {
        case BLOCK_ARM_TARGET_LOW_PICK_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_LOW_PICK;
            block_arm.dji_target_position = ARM_POSES[BLOCK_ARM_TARGET_LOW_PICK_READY].dji_deg;
            block_arm.zdrive_target_position = ARM_POSES[BLOCK_ARM_TARGET_LOW_PICK_READY].zdrive_deg;
            break;

        case BLOCK_ARM_TARGET_HIGH_PICK_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_HIGH_PICK;
            block_arm.dji_target_position = ARM_POSES[BLOCK_ARM_TARGET_HIGH_PICK_READY].dji_deg;
            block_arm.zdrive_target_position = ARM_POSES[BLOCK_ARM_TARGET_HIGH_PICK_READY].zdrive_deg;
            break;

        case BLOCK_ARM_TARGET_SECOND_PICK_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_SECOND_PICK;
            block_arm.dji_target_position = ARM_POSES[BLOCK_ARM_TARGET_SECOND_PICK_READY].dji_deg;
            block_arm.zdrive_target_position = ARM_POSES[BLOCK_ARM_TARGET_SECOND_PICK_READY].zdrive_deg;
            break;

        case BLOCK_ARM_TARGET_PLACE_BOTTOM_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_PLACE_BOTTOM;
            block_arm.dji_target_position = ARM_POSES[BLOCK_ARM_TARGET_PLACE_BOTTOM_READY].dji_deg;
            block_arm.zdrive_target_position = ARM_POSES[BLOCK_ARM_TARGET_PLACE_BOTTOM_READY].zdrive_deg;
            break;

        case BLOCK_ARM_TARGET_PLACE_LEVEL1_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_PLACE_LEVEL1;
            block_arm.dji_target_position = ARM_POSES[BLOCK_ARM_TARGET_PLACE_LEVEL1_READY].dji_deg;
            block_arm.zdrive_target_position = ARM_POSES[BLOCK_ARM_TARGET_PLACE_LEVEL1_READY].zdrive_deg;
            break;

        case BLOCK_ARM_TARGET_PLACE_LEVEL2_READY:
            block_arm.fine_adjust_profile = BLOCK_ARM_FINE_PROFILE_PLACE_LEVEL2;
            block_arm.dji_target_position = ARM_POSES[BLOCK_ARM_TARGET_PLACE_LEVEL2_READY].dji_deg;
            block_arm.zdrive_target_position = ARM_POSES[BLOCK_ARM_TARGET_PLACE_LEVEL2_READY].zdrive_deg;
            break;

        case BLOCK_ARM_TARGET_SAFE:
            block_arm.dji_target_position = ARM_POSES[BLOCK_ARM_TARGET_SAFE].dji_deg;
            block_arm.zdrive_target_position = ARM_POSES[BLOCK_ARM_TARGET_SAFE].zdrive_deg;
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
    const ArmPose_t *pose;

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
    pose = &ARM_POSES[target];
    block_arm.motion_start_dji = block_arm.dji_current_position;
    block_arm.motion_start_zdrive = block_arm.zdrive_current_position;
    block_arm.motion_start_ms = HAL_GetTick();

    block_arm.dji_target_position = block_arm.dji_current_position;
    block_arm.zdrive_target_position = block_arm.zdrive_current_position;
    block_arm.dji_target_velocity_deg_s = 0.0f;
    block_arm.zdrive_target_velocity_deg_s = 0.0f;
    BlockArm_SetAutoTarget(target);

    if (pose->move_time_ms > 0U && BlockArm_IsAtSafePose())
    {
        block_arm.active_motion_pose = pose;
        block_arm.motion_duration_ms = pose->move_time_ms;
        block_arm.dji_target_position = block_arm.motion_start_dji;
        block_arm.zdrive_target_position = block_arm.motion_start_zdrive;
        block_arm.dji_target_velocity_deg_s = 0.0f;
        block_arm.zdrive_target_velocity_deg_s = 0.0f;
    }
    else
    {
        block_arm.active_motion_pose = NULL;
        block_arm.motion_duration_ms = 0U;
    }

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
    block_arm.dji_target_velocity_deg_s = 0.0f;
    block_arm.zdrive_target_velocity_deg_s = 0.0f;
}

void BlockArm_Init(DJMotor *dji_motor, Zdrive *zdrive_motor)
{
    /* 按编译宏选择左右臂姿态表 */
#if defined(ARM_LEFT)
    ARM_POSES = ARM_POSES_LEFT;
#elif defined(ARM_RIGHT)
    ARM_POSES = ARM_POSES_RIGHT;
#endif

    block_arm.reset_to_safe_active = false;

    block_arm.dji_motor = dji_motor;
    block_arm.zdrive_motor = zdrive_motor;
    block_arm.state = BLOCK_ARM_DISABLED; /*初始失能*/

    block_arm.dji_current_position = 0.0f;
    block_arm.zdrive_current_position = 0.0f;
    block_arm.dji_target_position = 0.0f;
    block_arm.zdrive_target_position = 0.0f;
    block_arm.dji_target_velocity_deg_s = 0.0f;
    block_arm.zdrive_target_velocity_deg_s = 0.0f;
    block_arm.dji_zero_position = 0.0f;
    block_arm.zdrive_zero_position = 0.0f;
    block_arm.tolerance = 1.0f;
    block_arm.reached_count = 0U;
    block_arm.homing_count = 0U;
    block_arm.motion_count = 0U;
    block_arm.active_motion_pose = NULL;
    block_arm.motion_duration_ms = 0U;
    block_arm.motion_start_ms = 0U;
    block_arm.motion_start_dji = 0.0f;
    block_arm.motion_start_zdrive = 0.0f;
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
    block_arm.active_motion_pose = NULL;
    block_arm.motion_duration_ms = 0U;
    block_arm.dji_target_velocity_deg_s = 0.0f;
    block_arm.zdrive_target_velocity_deg_s = 0.0f;
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
    block_arm.active_motion_pose = NULL;
    block_arm.motion_duration_ms = 0U;
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
    BlockArmTarget_t current_pose;

    BlockArm_UpdateFeedback();
    if (!BlockArm_DetectCurrentPose(&current_pose))
    {
        /* 未知位置不盲目直达 SAFE，保持当前姿态等待人工处理。 */
        return;
    }

    BlockArm_StartAutoMove(BLOCK_ARM_TARGET_SAFE);
    if (block_arm.state != BLOCK_ARM_MOVING)
    {
        return;
    }

    if (current_pose != BLOCK_ARM_TARGET_SAFE &&
        ARM_POSES[current_pose].move_time_ms > 0U)
    {
        /*
         * 已配置轨迹的粗定位姿态 -> SAFE 使用对应 SAFE 出发曲线的反向时间参数化。
         * 从实时反馈起步，在端点存在少量跟踪误差时仍能平滑并入原路径。
         */
        block_arm.active_motion_pose = &ARM_POSES[BLOCK_ARM_TARGET_SAFE];
        block_arm.motion_duration_ms = ARM_POSES[current_pose].move_time_ms;
        block_arm.dji_target_position = block_arm.motion_start_dji;
        block_arm.zdrive_target_position = block_arm.motion_start_zdrive;
        block_arm.dji_target_velocity_deg_s = 0.0f;
        block_arm.zdrive_target_velocity_deg_s = 0.0f;
    }

    /*
     * 其他已识别姿态在对应 CSV 拟合加入前仍保留原有直达 SAFE 行为；
     * 后续在此处按 current_pose 增加各自的复位轨迹。
     */
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
                block_arm.dji_target_position = ARM_POSES[BLOCK_ARM_TARGET_SAFE].dji_deg;
                block_arm.zdrive_target_position = ARM_POSES[BLOCK_ARM_TARGET_SAFE].zdrive_deg;
                block_arm.dji_target_velocity_deg_s = 0.0f;
                block_arm.zdrive_target_velocity_deg_s = 0.0f;
                block_arm.active_motion_pose = NULL;
                block_arm.motion_duration_ms = 0U;
                block_arm.reached_count = 0U;

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
            else if (block_arm.active_motion_pose != NULL)
            {
                /* Reached detection starts only after the timed trajectory ends. */
                BlockArm_UpdateAutoMotion();
                BlockArm_ApplyPositionTargets();
                block_arm.motion_count++;
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
    block_arm.active_motion_pose = NULL;
    block_arm.motion_duration_ms = 0U;
    block_arm.dji_target_velocity_deg_s = 0.0f;
    block_arm.zdrive_target_velocity_deg_s = 0.0f;
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
