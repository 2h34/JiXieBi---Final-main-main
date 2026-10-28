#include "myostasks.h"

#include "BlockArm.h"
#include "BlockVacuum.h"
#include "PickBlockTask.h"
#include "PlaceBlockTask.h"
#include "Solenoid.h"

#define BLOCK_ARM_PROCESS_PERIOD_MS  5U

uint8_t BeepAlarmTimes = 0;
uint8_t flag = 0;
uint8_t flag1 = 0;
uint8_t flag2 = 0;
uint8_t flag3 = 0;
uint8_t flag4 = 0;
uint8_t flag5 = 0;
uint8_t flag6 = 0;
uint8_t flag7 = 0;

void LedWaterTask(void *argument)
{
  for(;;)
  {
    Led_Water();
  }
}


void BeepAlarmTask(void *argument)
{
  for(;;)
  {
    uint8_t count = BeepAlarmTimes;
    BeepAlarmTimes = 0;
    for(uint8_t i = 0; i < count; i++)
    {
        BEEP_ON();
        osDelay(40);
        BEEP_OFF();
        osDelay(40);
    }
    osDelay(1);
  }
}


/*
 * 通信 finish 回执：s_finish_process 保存"在途命令"对应的原始 signal
 * （0x01/0x03/0x04/0x07/0xFF），0 表示没有在途命令。
 * 当对应状态机到达终态时，回一次 finish 并清空。
 */
static uint8_t s_finish_process = 0U;

static void Comm_CheckFinish(void)
{
    if (s_finish_process == 0U)
    {
        return;
    }

    uint8_t done = 0U;
    switch (s_finish_process)
    {
        // case 0x01U:   /* Enable：回零完成 = arm READY */
        case 0xFFU:   /* Reset：复位到安全位 = arm READY */
            done =
    BlockArm_GetState() == BLOCK_ARM_READY &&
    PickBlockTask_GetState() == PICK_BLOCK_TASK_IDLE &&
    PlaceBlockTask_GetState() == PLACE_BLOCK_TASK_IDLE ;
    // && BlockVacuum_GetState() == BLOCK_VACUUM_RELEASED;
            break;

        case 0x05U:   /* Pick：取块全流程走完 = pick DONE */
            done = (PickBlockTask_GetState() == PICK_BLOCK_TASK_DONE);
            break;

        case 0x06U:   /* Place: 机械臂移动到放块位置，进入微调状态 */
            done = (PlaceBlockTask_GetState() == PLACE_BLOCK_TASK_MANUAL_ALIGN);
            break;
        case 0x08U:   /* Release：放块全流程走完 = place DONE */
            done = (PlaceBlockTask_GetState() == PLACE_BLOCK_TASK_DONE);
            break;

        default:      /* 未知 process，放弃回执 */
            s_finish_process = 0U;
            return;
    }

    if (done)
    {
        if (Comm_SendFinish(s_finish_process))
        {
            s_finish_process = 0U;
        }
    }
}



/*
 * 单机械臂阶段的固定运行入口。
 * 电机驱动，任务和机构已在 main() 中初始化
 * 不自动调用 BlockArm_Home()，归零仍由上层命令在后续接入。
 */
void BlockArmServiceTask(void *argument)
{
  (void)argument;
  uint32_t next_wake = osKernelGetTickCount();

  

  for (;;)
  {
    BlockArm_Process();
    BlockVacuum_Process();
    PickBlockTask_Process();
    PlaceBlockTask_Process();

    /* 连续点击结束后，将点击次数转换成具体的取块或放块命令。 */
    Comm_ClickProcess();

    /* 消费 ISR 投递的命令：读完立刻清掉，避免重复执行，也避免漏掉中断新写来的命令。 */
    uint8_t cmd = Master_Command;
    Master_Command = Master_Default;

    switch (cmd)
    {
      case Master_Disable:
      BlockArm_Disable();
      BlockVacuum_Reset();
      PickBlockTask_Init();
      PlaceBlockTask_Init();
      s_finish_process = 0U;
      break;


      case Master_Enable:
        BlockArm_Enable();
        BlockArm_Home();
        // s_finish_process = 0x01U;
        break;

      case Master_Reset:
      {
        BlockArmState_t arm_state;

        /* 1. 终止当前Pick/Place流程，使机械臂进入STOPPED。 */
        PickBlockTask_Stop();
        PlaceBlockTask_Stop();
        arm_state = BlockArm_GetState();
        if (arm_state == BLOCK_ARM_STOPPED ||
            arm_state == BLOCK_ARM_REACHED)
        {
          BlockArm_Reset();          /* 机构只复位一次 */
        }
        
        // /* 3. 关闭吸盘并清除真空机构状态。 */
        // BlockVacuum_Reset();

        /* 4. 两个任务进入复位状态，等待Arm到达READY。 */
        PickBlockTask_Reset();
        PlaceBlockTask_Reset();

        /* 5. 登记Reset完成回执。 */
        s_finish_process = 0xFFU;
        break;
      }
       
      
      case Master_LowPick:
        PickBlockTask_StartLowPick();
        //现在的设计是，低位取块不需要确认抓取，直接抓取
        s_finish_process = 0x05U;
        break;

      case Master_HighPick:
        PickBlockTask_StartHighPick();
        //现在的设计是，高位取块不需要确认抓取，直接抓取
        s_finish_process = 0x05U;
        break;

      case Master_SecondPick:
        PickBlockTask_StartSecondPick();
        //现在的设计是，二层取块不需要确认抓取，直接抓取
        s_finish_process = 0x05U;
        break;

      case Master_PlaceBottom:
        PlaceBlockTask_StartBottom();
        s_finish_process = 0x06U;
        break;

      case Master_PlaceLevel1:
        PlaceBlockTask_StartLevel1();
        s_finish_process = 0x06U;
        break;

      case Master_PlaceLevel2:
        PlaceBlockTask_StartLevel2();
        s_finish_process = 0x06U;
        break;

      case Master_Release:
        PlaceBlockTask_ConfirmRelease();
        s_finish_process = 0x08U;
        break;

      default:
        break;
    }

    /* 动作真正做完再回 finish。 */
    Comm_CheckFinish();


    if( flag1 == 1)
    {
      PlaceBlockTask_StartLevel1();
      flag1 = 0;
    }
    if (flag == 1)
    {
      BlockArm_Enable();
      BlockArm_Home();
      flag = 0;
    }
    if (flag2 == 1)
    {
      PlaceBlockTask_ConfirmRelease();
      flag2 = 0;
    }
    if (flag3 == 1)
    {
      PickBlockTask_StartHighPick();
      flag3 = 0;
    }
    if (flag4 == 1)
    {
      // 现在的設計是，高位取塊不需要確認抓取，直接抓取
      PickBlockTask_ConfirmGrab();
      flag4 = 0;
    }
    if (flag5 == 1)
    {
       BlockArmState_t arm_state;
      PickBlockTask_Stop();
        PlaceBlockTask_Stop();
        arm_state = BlockArm_GetState();
        if (arm_state == BLOCK_ARM_STOPPED ||
            arm_state == BLOCK_ARM_REACHED)
        {
          BlockArm_Reset();          /* 机构只复位一次 */
        }
        
        // /* 3. 关闭吸盘并清除真空机构状态。 */
        // BlockVacuum_Reset();

        /* 4. 两个任务进入复位状态，等待Arm到达READY。 */
        PickBlockTask_Reset();
        PlaceBlockTask_Reset();
      flag5 = 0;
    }
    if (flag6 == 1)
    {
      solenoid_on(1U, 0x0FU);
      flag6 = 0;
    }
    if (flag7 == 1)
    {
      solenoid_on(1U, 0x00U);
      flag7 = 0;
    }
    next_wake += BLOCK_ARM_PROCESS_PERIOD_MS;
    osDelayUntil(next_wake);
  }
}
