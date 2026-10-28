/**
 * @file    DJmotor.h
 * @brief   DJI M2006/M3508 CAN motor driver, ported from R2 chassis.
 */
#ifndef DJMOTOR_H
#define DJMOTOR_H

#include <stdbool.h>
#include "main.h"
#include "pid.h"
#include "motor_config.h"
#include "can.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define M3508_NUM MOTOR_M3508_COUNT
#define M2006_NUM MOTOR_M2006_COUNT
#define USE_DJNUM MOTOR_DJI_COUNT
#define M2006_RATIO MOTOR_M2006_REDUCTION_RATIO
#define M3508_RATIO MOTOR_M3508_REDUCTION_RATIO
#define Zero_Distance 15

#define BLOCK_ARM_DJI_INDEX          1U


    typedef enum
    {
        DJ_Disable = 0,  /* 关: transmit 0 current */
        DJ_RPM = 1,      /* 速度 mode          */
        DJ_Position = 2, /* 位置 mode       */
        DJ_Zero = 3,     /* 寻零 mode       */
        DJ_Current = 4,  /* 电流/扭矩        */

    } DJmotor_mode_t;

    typedef struct
    {
        volatile int16_t current_raw;  // 直接设置电流
        volatile float angle_deg;      // 输出角度, degree
        volatile int16_t speed_rpm;    // valSet: 输出轴 rpm;valNow: 转子 rpm(原始反馈),m2006额定转速为416rpm,m3508为469rpm
        volatile float current_A;      // 反馈电流, A
        volatile int16_t PulseRead;    // raw encoder pulse
        volatile int16_t PulseGap;     // pulse delta
        volatile int32_t PulseTotal;   // accumulated pulse
        volatile int8_t temperature_C; // ℃
    } DJmotorVal;

    typedef struct
    {
        uint16_t PulsePerRound;   //           8191
        float Gear_ratio;         // mechanism ratio
        float Reduction_ratio;    // motor reducer ratio
        uint32_t ParamID;         // CAN receive ID base
        int16_t CurrentLimit_raw; // output current limit, raw
    } DJmotorParam;

    typedef struct
    {
        bool RPMLimitFlag;
        bool PosAngleLimitFlag;
        bool PosRPMFlag;
        bool CurrentLimitFlag;
        float MaxAngle_deg;           // degree
        float MinAngle_deg;           // degree
        int16_t SpeedRPMLimit;        // rpm
        int32_t PosRPMLimit;          // rpm
        int16_t ZeroRPMLimit;         // rpm
        int16_t ZeroCurrentLimit_raw; // raw
        bool IsLooseStuck;
    } DJmotorLimit;

    typedef struct
    {
        volatile bool ZeroFlag;  /* 寻零完成标志 */
        volatile bool Overtimeflag;
        volatile bool StuckFlag;
        volatile bool IsSetZero;  /* 请求下一次反馈时重置位置”的内部标志 */
    } DJmotorStatus;

    typedef struct
    {
        volatile int32_t pulseLock; // 原先用于使能固定的字段,现在已无用
        uint16_t zeroCnt;
        uint16_t GapCnt;
    } DJmotorArgum;

    typedef struct
    {
        volatile uint32_t lastRxTime;
        uint16_t stuckCount;
        uint16_t timeoutCount;
    } DJmotorError;

    typedef struct
    {
        PIDType posPID;
        PIDType velPID;
    } DJmotorPID;

    typedef struct
    {
        uint8_t ID;
        volatile bool Begin;              // true 运行 MODE;false 失能
        volatile DJmotor_mode_t MODE_Set; // DJ_Disable 即失能(发 0 电流)
        volatile DJmotor_mode_t MODE_Cur; // 实际运行模式,任务层可读

        DJmotorParam param;
        DJmotorVal valSet;
        DJmotorVal valNow;
        DJmotorVal valPre;
        DJmotorStatus statusFlag;
        DJmotorLimit limit;
        DJmotorArgum argum;
        DJmotorError error;
        PIDType posPID;
        PIDType velPID;
    } DJMotor, *DJMotorPointer;

#if USE_DJ
    extern DJMotor DJmotor[USE_DJNUM];

    void DJmotor_Init(void);
    void DJmotor_Func(void);
    void DJmotor_Receive(CAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data);
    void DJmotor_PID_Reload(DJMotorPointer motor, DJmotorPID pid_reload);

    /* 请求位置模式并写入输出端角度目标，可由机构层周期调用。 */
    void DJmotor_SetPositionTarget(DJMotorPointer motor, float target_angle_deg);

    /* 请求速度模式并写入输出端速度目标，可由机构层周期调用。 */
    void DJmotor_SetVelocityTarget(DJMotorPointer motor, int16_t target_velocity_rpm);

    /*启动寻零模式*/
    void DJmotor_SetZeromode(DJMotorPointer motor);
    /* 判断寻零是否完成*/
    bool DJmotor_IsZeroDone(DJMotorPointer motor);
     /* 请求彻底失能:退出所有运动模式并每周期发送 0 电流,用于调试急停。 */
    void DJmotor_Disable(DJMotorPointer motor);



#endif /* USE_DJ */

#ifdef __cplusplus
}
#endif

#endif /*DJMOTOR_H */
