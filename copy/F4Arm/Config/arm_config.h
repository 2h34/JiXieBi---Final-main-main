#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* 合并后单工程双固件：左右差异唯一源头。
 * Keil Target Define：f4_show_LEFT → ARM_LEFT ；f4_show_RIGHT → ARM_RIGHT
 * 信号组保持现状（协议交叉为有意实现，不回改）：
 *   左臂(LETNOW) 响应 0x05/06/08；右臂(RIGHTNOW) 响应 0x03/04/07
 */
#if defined(ARM_LEFT)
  #define ARM_SOLENOID_CHANNEL   1U      /* 左臂：solenoid_init(1)   原 LEFTNOW· */
  #define ARM_DJI_INDEX          0U      /* 左臂：&DJmotor[0]        原 LEFTNOW· */
  #define ARM_PICK_SIGNAL        0x05U   /* 左臂响应组（原 left 工程启用 right handler） */
  #define ARM_PLACE_SIGNAL       0x06U
  #define ARM_RELEASE_SIGNAL     0x08U
#elif defined(ARM_RIGHT)
  #define ARM_SOLENOID_CHANNEL   2U      /* 右臂：solenoid_init(2U)  原 RIGHTNOW */
  #define ARM_DJI_INDEX          1U      /* 右臂：&DJmotor[1]        原 RIGHTNOW */
  #define ARM_PICK_SIGNAL        0x03U   /* 右臂响应组（原 right 工程启用 left handler） */
  #define ARM_PLACE_SIGNAL       0x04U
  #define ARM_RELEASE_SIGNAL     0x07U
#else
  #error "Define ARM_LEFT or ARM_RIGHT in Keil target"
#endif

#define ARM_ZDRIVE_INDEX         0U      /* 两板相同 */

/* 7 个姿态角（顺序必须与 BlockArm.c 内 BlockArmTarget_t 一致：
   LOW_PICK, HIGH_PICK, SECOND_PICK, PLACE_BOTTOM, PLACE_LEVEL1, PLACE_LEVEL2, SAFE）*/
typedef struct
{
    float dji_deg;
    float zdrive_deg;
    uint32_t move_time_ms; /* 0: direct target; non-zero: SAFE <-> pose smooth duration */
} ArmPose_t;

extern const ArmPose_t ARM_POSES_LEFT[7];
extern const ArmPose_t ARM_POSES_RIGHT[7];

#endif /* ARM_CONFIG_H */
