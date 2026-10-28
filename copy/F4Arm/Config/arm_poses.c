#include "arm_config.h"

/* 左臂姿态角：原 LEFTNOW· BlockArm.c 实测值 */
const ArmPose_t ARM_POSES_LEFT[7] = {
    { -12.0f, 118.0f,    0U },   /* LOW_PICK      */
    { -21.0f, 101.0f,    0U },   /* HIGH_PICK     */
    {  20.0f,  45.0f,    0U },   /* SECOND_PICK   */
    { -20.0f,  90.0f,    0U },   /* PLACE_BOTTOM  */
    {  45.0f,  50.0f,    0U },   /* PLACE_LEVEL1  */
    {  85.0f,  44.0f,    0U },   /* PLACE_LEVEL2  */
    {  20.0f,  40.0f,    0U },   /* SAFE          */
};

/* 右臂姿态角：原 RIGHTNOW BlockArm.c 实测值 */
const ArmPose_t ARM_POSES_RIGHT[7] = {
    {  12.0f, 116.0f, 1000U },   /* LOW_PICK: SAFE -> LOW_PICK fitted path */
    {  28.0f, 105.0f, 1000U },   /* HIGH_PICK: SAFE <-> HIGH_PICK fitted path */
    { -20.0f,  45.0f,    0U },   /* SECOND_PICK   */
    {  20.0f,  95.0f, 1000U },   /* PLACE_BOTTOM: SAFE <-> PLACE_BOTTOM fitted path */
    { -40.0f,  50.0f, 1000U },   /* PLACE_LEVEL1: SAFE <-> PLACE_LEVEL1 fitted path */
    { -84.0f,  44.0f, 1000U },   /* PLACE_LEVEL2: SAFE <-> PLACE_LEVEL2 fitted path */
    { -25.0f,  40.0f,    0U },   /* SAFE          */
};
