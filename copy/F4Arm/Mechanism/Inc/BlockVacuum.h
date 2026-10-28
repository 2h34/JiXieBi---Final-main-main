#ifndef BLOCK_VACUUM_H
#define BLOCK_VACUUM_H

#include <stdint.h>

typedef enum
{
    BLOCK_VACUUM_RELEASED,
    BLOCK_VACUUM_GRABBING,
    BLOCK_VACUUM_GRABBED,
    BLOCK_VACUUM_RELEASING,
    BLOCK_VACUUM_FAULT
} BlockVacuumState_t;

/* 初始化真空机构的软件状态；不在此处假定具体泵阀硬件。 */
void BlockVacuum_Init(void);

/* 启动建立真空；函数返回不代表已经吸住方块。 */
void BlockVacuum_Grab(void);

/* 启动释放真空；函数返回不代表方块已经脱离。 */
void BlockVacuum_Release(void);

/* 复位真空机构的软件状态 */
void BlockVacuum_Reset(void);


BlockVacuumState_t BlockVacuum_GetState(void);
void BlockVacuum_Process(void);

#endif /* BLOCK_VACUUM_H */
