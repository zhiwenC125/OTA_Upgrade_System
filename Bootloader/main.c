/**
 * STM32F103C8T6 Bootloader (方案 A — 外部 Download)
 */

#include "stm32f1xx_hal.h"

/* ======================== SysTick 处理（覆盖 startup 中的 Default_Handler 弱定义） ======================== */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ======================== 分区地址定义 ======================== */

#define FLASH_APP_ADDR          ((uint32_t)0x08002000)
#define FLASH_APP_SIZE          ((uint32_t)0xDC00)   /* 55KB */
#define FLASH_APP_PAGES         (FLASH_APP_SIZE / FLASH_PAGE_SIZE)  /* 55 pages */

#define FLASH_FLAG_ADDR         ((uint32_t)0x0800FC00)

#define OTA_FLAG_UPGRADE_PENDING  ((uint32_t)0xAA55AA55)
#define OTA_FLAG_UPGRADE_DONE     ((uint32_t)0x55AA55AA)  /* 兼容旧版 */
#define OTA_FLAG_BOOT_COUNTING    ((uint32_t)0xC3A5C3A5)
#define OTA_FLAG_CONFIRMED        ((uint32_t)0x5A5AA5A5)

#define OTA_BOOT_COUNT_MAX        3U

#define W25Q_DOWNLOAD_ADDR      ((uint32_t)0x000000)
#define W25Q_BACKUP_ADDR        ((uint32_t)0x010000)
#define W25Q_META_ADDR          ((uint32_t)0x020000)

#define OTA_META_MAGIC          ((uint32_t)0x4D455441)  /* "META" */

/* ======================== UART1 极简调试输出（PA9, 115200, 8MHz HSI） ======================== */

static void Boot_UART_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;

    /* PA9: AF push-pull output (50MHz) -- CRH bits [7:4] */
    GPIOA->CRH &= ~(0xFU << 4);
    GPIOA->CRH |=  (0xBU << 4);

    /* 115200 @ 8MHz HSI: BRR = 8000000/115200 = 69 */
    USART1->BRR = 69U;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void Boot_PutChar(char c)
{
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint8_t)c;
}

static void Boot_Print(const char *s)
{
    while (*s) Boot_PutChar(*s++);
    while (!(USART1->SR & USART_SR_TC));
}

static void Boot_PrintHex8(uint8_t v)
{
    const char h[] = "0123456789ABCDEF";
    Boot_PutChar(h[v >> 4]);
    Boot_PutChar(h[v & 0xF]);
}

static void Boot_PrintHex32(uint32_t v)
{
    Boot_Print("0x");
    Boot_PrintHex8((v >> 24) & 0xFF);
    Boot_PrintHex8((v >> 16) & 0xFF);
    Boot_PrintHex8((v >>  8) & 0xFF);
    Boot_PrintHex8( v        & 0xFF);
}

/* ======================== SPI1 精简驱动 ======================== */

#define W25Q_CS_LOW()   (GPIOA->BRR  = GPIO_PIN_4)
#define W25Q_CS_HIGH()  (GPIOA->BSRR = GPIO_PIN_4)

static SPI_HandleTypeDef hspi1;

static void Boot_SPI_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_4;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    W25Q_CS_HIGH();

    gpio.Pin   = GPIO_PIN_5 | GPIO_PIN_7;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin  = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; /* 8MHz/4=2MHz */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);
}

static void Boot_W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t cmd[4];
    cmd[0] = 0x03;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >>  8) & 0xFF;
    cmd[3] =  addr        & 0xFF;

    W25Q_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_SPI_Receive(&hspi1, buf, len, 5000);
    W25Q_CS_HIGH();
}

/* ======================== 内部 Flash 操作 ======================== */

static HAL_StatusTypeDef Boot_EraseApp(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_err = 0;

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_APP_ADDR;
    erase.NbPages     = FLASH_APP_PAGES;

    return HAL_FLASHEx_Erase(&erase, &page_err);
}

static HAL_StatusTypeDef Boot_CopyW25QToAppFrom(uint32_t w25q_src_addr)
{
    uint8_t  page_buf[256];
    uint32_t remaining = FLASH_APP_SIZE;
    uint32_t src_addr  = w25q_src_addr;
    uint32_t dest_addr = FLASH_APP_ADDR;

    while (remaining > 0)
    {
        uint32_t chunk = (remaining > 256) ? 256 : remaining;

        Boot_W25Q_Read(src_addr, page_buf, chunk);

        for (uint32_t i = 0; i < chunk; i += 2)
        {
            uint16_t halfword;
            if (i + 1 < chunk)
                halfword = (uint16_t)page_buf[i] | ((uint16_t)page_buf[i + 1] << 8);
            else
                halfword = (uint16_t)page_buf[i] | 0xFF00;

            HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                                      dest_addr, halfword);
            if (st != HAL_OK)
                return st;
            dest_addr += 2;
        }

        src_addr  += chunk;
        remaining -= chunk;
    }

    return HAL_OK;
}

static uint32_t Boot_GetFlag(void)
{
    return *(__IO uint32_t *)FLASH_FLAG_ADDR;
}

static uint32_t Boot_GetBootCount(void)
{
    return *(__IO uint32_t *)(FLASH_FLAG_ADDR + 4U);
}

/* Boot_SetFlag 仅写 state word（原有函数保留） */
/* Boot_SetFlagEx 同时写 state + boot_count 两个 word */
static HAL_StatusTypeDef Boot_SetFlagEx(uint32_t state, uint32_t count)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_err = 0;

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_FLAG_ADDR;
    erase.NbPages     = 1;

    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &page_err);
    if (st != HAL_OK) return st;

    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                           FLASH_FLAG_ADDR, state & 0xFFFFU);
    if (st != HAL_OK) return st;
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                           FLASH_FLAG_ADDR + 2U, (state >> 16) & 0xFFFFU);
    if (st != HAL_OK) return st;
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                           FLASH_FLAG_ADDR + 4U, count & 0xFFFFU);
    if (st != HAL_OK) return st;
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                           FLASH_FLAG_ADDR + 6U, (count >> 16) & 0xFFFFU);
    return st;
}

/* ======================== 跳转到 App ======================== */

typedef void (*pFunction)(void);

static void Boot_JumpToApp(void)
{
    uint32_t app_msp   = *(__IO uint32_t *)FLASH_APP_ADDR;
    uint32_t app_reset = *(__IO uint32_t *)(FLASH_APP_ADDR + 4);

    Boot_Print("[BOOT] App MSP=");
    Boot_PrintHex32(app_msp);
    Boot_Print(" ResetVec=");
    Boot_PrintHex32(app_reset);
    Boot_Print("\r\n");

    if ((app_msp & 0x2FFE0000) != 0x20000000)
    {
        Boot_Print("[BOOT] ERROR: Invalid MSP! App is empty/corrupt.\r\n");
        return;
    }

    Boot_Print("[BOOT] Jumping to App...\r\n");

    __disable_irq();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    __HAL_RCC_SPI1_CLK_DISABLE();

    SCB->VTOR = FLASH_APP_ADDR;
    __set_MSP(app_msp);

    pFunction jump = (pFunction)app_reset;
    jump();

    while (1);
}

/* ======================== 主程序入口 ======================== */

int main(void)
{
    HAL_Init();
    Boot_UART_Init();

    Boot_Print("\r\n===== Bootloader =====\r\n");

    uint32_t flag = Boot_GetFlag();
    Boot_Print("[BOOT] Flag=");
    Boot_PrintHex32(flag);
    Boot_Print("\r\n");

    if (flag == OTA_FLAG_UPGRADE_PENDING)
    {
        Boot_Print("[BOOT] PENDING: starting OTA copy\r\n");

        Boot_SPI_Init();

        /* 读取 Meta 区，打印新固件版本号 */
        {
            uint8_t meta_buf[20];
            Boot_W25Q_Read(W25Q_META_ADDR, meta_buf, 20);
            uint32_t magic = (uint32_t)meta_buf[0]       |
                             ((uint32_t)meta_buf[1] << 8) |
                             ((uint32_t)meta_buf[2] << 16)|
                             ((uint32_t)meta_buf[3] << 24);
            if (magic == OTA_META_MAGIC) {
                Boot_Print("[BOOT] New FW ver=");
                Boot_PrintHex8(meta_buf[4]); Boot_PutChar('.');
                Boot_PrintHex8(meta_buf[5]); Boot_PutChar('.');
                Boot_PrintHex8(meta_buf[6]);
                Boot_Print("\r\n");
            }
        }

        /* 读取 W25Q32 前 8 字节验证 SPI 是否正常 */
        uint8_t probe[8] = {0};
        Boot_W25Q_Read(0, probe, 8);
        Boot_Print("[BOOT] W25Q32[0..7]=");
        for (int i = 0; i < 8; i++) {
            Boot_PrintHex8(probe[i]);
            Boot_PutChar(' ');
        }
        Boot_Print("\r\n");

        /* 若全为 0xFF，说明 SPI 失败或 W25Q32 为空 */
        uint8_t all_ff = 1;
        for (int i = 0; i < 8; i++) {
            if (probe[i] != 0xFF) { all_ff = 0; break; }
        }
        if (all_ff)
        {
            Boot_Print("[BOOT] ERROR: W25Q32 reads all 0xFF (SPI fail or empty)\r\n");
            Boot_Print("[BOOT] Clearing PENDING flag, skip copy\r\n");
            HAL_FLASH_Unlock();
            FLASH_EraseInitTypeDef e = { FLASH_TYPEERASE_PAGES, FLASH_FLAG_ADDR, 1 };
            uint32_t err = 0;
            HAL_FLASHEx_Erase(&e, &err);
            HAL_FLASH_Lock();
            goto do_jump;
        }

        HAL_FLASH_Unlock();

        Boot_Print("[BOOT] Erasing App (55KB)...\r\n");
        if (Boot_EraseApp() != HAL_OK)
        {
            Boot_Print("[BOOT] ERROR: Erase failed\r\n");
            HAL_FLASH_Lock();
            goto do_jump;
        }
        Boot_Print("[BOOT] Erase OK\r\n");

        Boot_Print("[BOOT] Copying Download -> Flash...\r\n");
        if (Boot_CopyW25QToAppFrom(W25Q_DOWNLOAD_ADDR) != HAL_OK)
        {
            Boot_Print("[BOOT] ERROR: Flash write failed\r\n");
            HAL_FLASH_Lock();
            goto do_jump;
        }
        Boot_Print("[BOOT] Copy OK\r\n");

        /* 设置 BOOT_COUNTING(1)：等待 App 确认，最多允许 OTA_BOOT_COUNT_MAX 次 */
        Boot_SetFlagEx(OTA_FLAG_BOOT_COUNTING, 1U);
        HAL_FLASH_Lock();
        Boot_Print("[BOOT] Flag -> BOOT_COUNTING(1)\r\n");
    }
    else if (flag == OTA_FLAG_BOOT_COUNTING)
    {
        uint32_t boot_count = Boot_GetBootCount();
        Boot_Print("[BOOT] BOOT_COUNTING: count=");
        Boot_PrintHex8((uint8_t)boot_count);
        Boot_Print("\r\n");

        if (boot_count >= OTA_BOOT_COUNT_MAX)
        {
            /* 新固件连续失败，触发回滚 */
            Boot_Print("[BOOT] Too many failed boots! Rolling back from Backup...\r\n");

            Boot_SPI_Init();

            HAL_FLASH_Unlock();

            Boot_Print("[BOOT] Erasing App (55KB)...\r\n");
            if (Boot_EraseApp() != HAL_OK)
            {
                Boot_Print("[BOOT] ERROR: Erase failed during rollback\r\n");
                HAL_FLASH_Lock();
                goto do_jump;
            }

            Boot_Print("[BOOT] Restoring Backup -> Flash...\r\n");
            if (Boot_CopyW25QToAppFrom(W25Q_BACKUP_ADDR) != HAL_OK)
            {
                Boot_Print("[BOOT] ERROR: Restore failed\r\n");
                HAL_FLASH_Lock();
                goto do_jump;
            }

            Boot_SetFlagEx(OTA_FLAG_CONFIRMED, 0U);
            HAL_FLASH_Lock();
            Boot_Print("[BOOT] Rollback complete. Flag -> CONFIRMED\r\n");
        }
        else
        {
            /* 还有机会，累加计数再试 */
            HAL_FLASH_Unlock();
            Boot_SetFlagEx(OTA_FLAG_BOOT_COUNTING, boot_count + 1U);
            HAL_FLASH_Lock();
            Boot_Print("[BOOT] Incremented boot count, jumping to App\r\n");
        }
    }
    else if (flag == OTA_FLAG_UPGRADE_DONE || flag == OTA_FLAG_CONFIRMED)
    {
        Boot_Print("[BOOT] Flag=CONFIRMED/DONE, normal boot\r\n");
    }
    else
    {
        Boot_Print("[BOOT] No OTA flag, normal boot\r\n");
    }

do_jump:
    Boot_JumpToApp();
    while (1);
}
