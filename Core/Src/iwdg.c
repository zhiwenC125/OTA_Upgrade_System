#include "iwdg.h"
#include <stdio.h>

/* ============================================================================
 * IWDG 独立看门狗 — 寄存器直接操作
 *
 * LSI = 40KHz (典型值，实际 30~60KHz)
 * 分频 /64 → 计数频率 625Hz
 * RLR = 2499 → 超时 = (2499+1) / 625 = 4.0s
 * ============================================================================ */

void IWDG_Init(void)
{
    IWDG->KR = 0x5555U;    /* 解锁 PR 和 RLR 寄存器 */
    IWDG->PR = 4U;         /* 分频系数 /64 (PR=4 → 2^(4+2)=64) */
    IWDG->RLR = 2499U;     /* 重载值：(2499+1)/625 = 4.0s */

    /* 等待 PR 和 RLR 寄存器更新完成 */
    while (IWDG->SR != 0U)
    {
        /* SR.PVU 和 SR.RVU 清零表示更新完成 */
    }

    IWDG->KR = 0xCCCCU;    /* 启动 IWDG */
    IWDG->KR = 0xAAAAU;    /* 首次喂狗（重载计数器） */

    printf("[IWDG] Init OK (4.0s timeout)\r\n");
}

void IWDG_Feed(void)
{
    IWDG->KR = 0xAAAAU;
}
