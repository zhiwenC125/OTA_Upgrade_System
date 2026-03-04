#ifndef __IWDG_H
#define __IWDG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ============================================================================
 * IWDG 独立看门狗驱动（寄存器直接操作，无需 HAL IWDG 模块）
 *
 * STM32F103 IWDG 特性：
 *   - 由 LSI（~40KHz）驱动，独立于系统时钟
 *   - 一旦启动无法停止（仅掉电复位才清除）
 *   - 软件复位（SystemReset）后 IWDG 仍在运行
 *     → Bootloader 中也必须喂狗，否则 Flash 拷贝期间会被复位
 *
 * 配置：分频 /64，RLR=2499 → 超时 ~4.0s
 *   LSI=40KHz / 64 = 625Hz → (2499+1) / 625 = 4.0s
 * ============================================================================ */

/* IWDG 超时时间（仅供参考，实际由寄存器决定） */
#define IWDG_TIMEOUT_MS     4000U

/**
 * @brief  启动 IWDG，超时 ~4.0s
 * @note   启动后无法停止，调用前请确认系统已准备好定期喂狗
 */
void IWDG_Init(void);

/**
 * @brief  喂狗（重载计数器）
 * @note   必须在超时前调用，否则触发系统复位
 *         即使 IWDG 未启动，调用此函数也无副作用
 */
void IWDG_Feed(void);

#ifdef __cplusplus
}
#endif

#endif /* __IWDG_H */
