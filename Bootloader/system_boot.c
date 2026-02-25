/**
 * Bootloader 专用 SystemInit
 *
 * Bootloader 从 0x08000000 启动，向量表无需偏移。
 * 提供最小化的 SystemInit() 供启动文件调用。
 */

#include "stm32f1xx.h"

uint32_t SystemCoreClock = 8000000;
const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U]  = {0, 0, 0, 0, 1, 2, 3, 4};

void SystemInit(void)
{
    /* Bootloader 在 0x08000000，使用默认向量表地址，无需重定位 */
}

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 8000000;
}
