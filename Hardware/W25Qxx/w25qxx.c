#include "w25qxx.h"
#include <stdio.h>

/* ============================================================================
 * W25Qxx SPI NOR Flash 驱动（Polling 模式）
 *
 * 基于 SPI1，PA4 软件控制 CS。
 * 后续性能优化可改为 DMA 模式（已预留 SPI1 DMA 中断）。
 * ============================================================================ */

/* 全局 SPI 句柄 */
SPI_HandleTypeDef hspi1;

/* 芯片信息（Init 后填充） */
W25Qxx_Info_t w25qxx_info = {0};

/* ===================== 底层 SPI 收发 ===================== */

static uint8_t SPI_TransmitReceive(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

/* ===================== SPI1 + CS 初始化 ===================== */

static void W25Qxx_SPI_Init(void)
{
    /* SPI1 时钟 + GPIO 时钟 */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* PA5=SCK, PA7=MOSI: 复用推挽 */
    gpio.Pin   = GPIO_PIN_5 | GPIO_PIN_7;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA6=MISO: 浮空输入 */
    gpio.Pin  = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA4=CS: 通用推挽输出（软件控制） */
    gpio.Pin   = W25QXX_CS_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(W25QXX_CS_PORT, &gpio);

    /* CS 拉高（取消选中） */
    W25QXX_CS_HIGH();

    /* 配置 SPI1 */
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;     /* CPOL=0 */
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;      /* CPHA=0 → Mode 0 */
    hspi1.Init.NSS               = SPI_NSS_SOFT;         /* 软件 CS */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; /* APB2/4 = 16MHz */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        while (1);
    }
}

/* ===================== 芯片操作 ===================== */

/**
 * @brief  写使能（每次写入/擦除前必须调用）
 */
static void W25Qxx_WriteEnable(void)
{
    W25QXX_CS_LOW();
    SPI_TransmitReceive(W25X_WriteEnable);
    W25QXX_CS_HIGH();
}

/**
 * @brief  等待芯片空闲（轮询 Status Register 1 的 BUSY 位）
 */
void W25Qxx_WaitBusy(void)
{
    W25QXX_CS_LOW();
    SPI_TransmitReceive(W25X_ReadStatusReg1);
    while ((SPI_TransmitReceive(0xFF) & 0x01) == 0x01)
    {
        /* BUSY=1, 继续等 */
    }
    W25QXX_CS_HIGH();
}

/**
 * @brief  读取 JEDEC ID
 */
uint32_t W25Qxx_ReadJedecID(void)
{
    uint32_t id = 0;

    W25QXX_CS_LOW();
    SPI_TransmitReceive(W25X_JedecDeviceID);
    id  = (uint32_t)SPI_TransmitReceive(0xFF) << 16;  /* Manufacturer */
    id |= (uint32_t)SPI_TransmitReceive(0xFF) << 8;   /* Memory Type */
    id |= (uint32_t)SPI_TransmitReceive(0xFF);         /* Capacity */
    W25QXX_CS_HIGH();

    return id;
}

/**
 * @brief  初始化
 */
int W25Qxx_Init(void)
{
    W25Qxx_SPI_Init();

    /* 读取芯片 ID */
    w25qxx_info.jedec_id = W25Qxx_ReadJedecID();

    switch (w25qxx_info.jedec_id)
    {
    case W25Q32_ID:
        w25qxx_info.capacity_bytes = 4 * 1024 * 1024;  /* 4MB */
        break;
    case W25Q64_ID:
        w25qxx_info.capacity_bytes = 8 * 1024 * 1024;  /* 8MB */
        break;
    case W25Q128_ID:
        w25qxx_info.capacity_bytes = 16 * 1024 * 1024; /* 16MB */
        break;
    default:
        printf("[W25Qxx] Unknown ID: 0x%06lX\r\n",
               (unsigned long)w25qxx_info.jedec_id);
        return -1;
    }

    w25qxx_info.sector_count = w25qxx_info.capacity_bytes / W25QXX_SECTOR_SIZE;

    printf("[W25Qxx] Detected ID: 0x%06lX, %luMB, %u sectors\r\n",
           (unsigned long)w25qxx_info.jedec_id,
           (unsigned long)(w25qxx_info.capacity_bytes / 1024 / 1024),
           w25qxx_info.sector_count);

    return 0;
}

/* ===================== 数据读取 ===================== */

void W25Qxx_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    W25QXX_CS_LOW();
    SPI_TransmitReceive(W25X_ReadData);
    SPI_TransmitReceive((addr >> 16) & 0xFF);
    SPI_TransmitReceive((addr >> 8) & 0xFF);
    SPI_TransmitReceive(addr & 0xFF);

    /* 连续读取 */
    HAL_SPI_Receive(&hspi1, buf, (uint16_t)len, HAL_MAX_DELAY);
    W25QXX_CS_HIGH();
}

/* ===================== 数据写入 ===================== */

/**
 * @brief  页编程（最多 256 字节，不可跨页边界）
 */
void W25Qxx_WritePage(uint32_t addr, const uint8_t *data, uint16_t len)
{
    W25Qxx_WriteEnable();

    W25QXX_CS_LOW();
    SPI_TransmitReceive(W25X_PageProgram);
    SPI_TransmitReceive((addr >> 16) & 0xFF);
    SPI_TransmitReceive((addr >> 8) & 0xFF);
    SPI_TransmitReceive(addr & 0xFF);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, HAL_MAX_DELAY);
    W25QXX_CS_HIGH();

    W25Qxx_WaitBusy();
}

/**
 * @brief  任意长度写入（自动分页，写入前需确保区域已擦除）
 *
 * 自动处理跨页边界：
 *   若 addr=0x0F0, len=512 → 先写 0x0F0~0x0FF(16B), 再写 0x100~0x1FF(256B),
 *   再写 0x200~0x2EF(240B)
 */
void W25Qxx_Write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t offset = 0;

    while (offset < len)
    {
        /* 当前页内剩余空间 */
        uint16_t page_remain = W25QXX_PAGE_SIZE - (addr % W25QXX_PAGE_SIZE);
        uint32_t to_write    = len - offset;

        if (to_write > page_remain)
            to_write = page_remain;

        W25Qxx_WritePage(addr, data + offset, (uint16_t)to_write);

        addr   += to_write;
        offset += to_write;
    }
}

/* ===================== 擦除操作 ===================== */

void W25Qxx_EraseSector(uint32_t sector_addr)
{
    /* 对齐到 4KB 边界 */
    sector_addr &= ~(W25QXX_SECTOR_SIZE - 1);

    W25Qxx_WriteEnable();

    W25QXX_CS_LOW();
    SPI_TransmitReceive(W25X_SectorErase);
    SPI_TransmitReceive((sector_addr >> 16) & 0xFF);
    SPI_TransmitReceive((sector_addr >> 8) & 0xFF);
    SPI_TransmitReceive(sector_addr & 0xFF);
    W25QXX_CS_HIGH();

    W25Qxx_WaitBusy();  /* 扇区擦除约 60-400ms */
}

void W25Qxx_EraseBlock64(uint32_t block_addr)
{
    block_addr &= ~(W25QXX_BLOCK_SIZE - 1);

    W25Qxx_WriteEnable();

    W25QXX_CS_LOW();
    SPI_TransmitReceive(W25X_BlockErase64);
    SPI_TransmitReceive((block_addr >> 16) & 0xFF);
    SPI_TransmitReceive((block_addr >> 8) & 0xFF);
    SPI_TransmitReceive(block_addr & 0xFF);
    W25QXX_CS_HIGH();

    W25Qxx_WaitBusy();  /* 块擦除约 0.15-2s */
}

void W25Qxx_EraseChip(void)
{
    W25Qxx_WriteEnable();

    W25QXX_CS_LOW();
    SPI_TransmitReceive(W25X_ChipErase);
    W25QXX_CS_HIGH();

    W25Qxx_WaitBusy();  /* 全片擦除约 20-100s */
}
