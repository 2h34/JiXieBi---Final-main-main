#include "BlockVacuum.h"
#include "Solenoid.h"
/* BlockVacuum 需要长期保存的最小内部数据。 */
typedef struct
{
    BlockVacuumState_t state;
    uint32_t grab_count;
    uint32_t release_count;
} BlockVacuum_t;

BlockVacuum_t block_vacuum;

void BlockVacuum_Init(void)
{
    block_vacuum.state = BLOCK_VACUUM_RELEASED;
    block_vacuum.grab_count = 0U;
    block_vacuum.release_count = 0U;

    solenoid_on(1U, 0x00U);
}

void BlockVacuum_Grab(void)
{
    if (block_vacuum.state != BLOCK_VACUUM_RELEASED && block_vacuum.state != BLOCK_VACUUM_GRABBED)
    {
        return;
    }

    block_vacuum.grab_count = 0U;
    block_vacuum.state = BLOCK_VACUUM_GRABBING;

    /* TODO: 向真实泵阀 Driver 发出建立真空命令。 */
    solenoid_on(1U, 0x0FU);
}

void BlockVacuum_Release(void)
{
    if (block_vacuum.state != BLOCK_VACUUM_GRABBED && block_vacuum.state != BLOCK_VACUUM_RELEASED)
    {
        return;
    }

    block_vacuum.release_count = 0U;
    block_vacuum.state = BLOCK_VACUUM_RELEASING;

    /* TODO: 向真实泵阀 Driver 发出释放真空命令。 */
    solenoid_on(1U, 0x00U);
}

BlockVacuumState_t BlockVacuum_GetState(void)
{
    return block_vacuum.state;
}

void BlockVacuum_Process(void)
{
    switch (block_vacuum.state)
    {
        case BLOCK_VACUUM_RELEASED:
            break;

        case BLOCK_VACUUM_GRABBING:
            if (block_vacuum.grab_count++ > 400U)
            {
                block_vacuum.state = BLOCK_VACUUM_GRABBED;
            }
    
            break;

        case BLOCK_VACUUM_GRABBED:
            /* TODO: 如有压力反馈，在此持续监测吸附是否丢失。 */
            break;

        case BLOCK_VACUUM_RELEASING:
            if (block_vacuum.release_count++ > 500U){
                block_vacuum.state = BLOCK_VACUUM_RELEASED;
            }
            break;

        case BLOCK_VACUUM_FAULT:
            /* TODO: 根据真实气路设计维持安全状态。 */
            break;

        default:
            block_vacuum.state = BLOCK_VACUUM_FAULT;
            break;
    }
}

void BlockVacuum_Reset(void)
{
    solenoid_on(1U, 0x00U);

    block_vacuum.grab_count = 0U;
    block_vacuum.release_count = 0U;
    block_vacuum.state = BLOCK_VACUUM_RELEASED;
}