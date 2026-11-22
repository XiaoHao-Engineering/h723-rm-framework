/**
 * @file bsp_flash.c
 * @brief 内部 Flash 读写(STM32H723)
 */

#include "bsp_flash.h"
#include "bsp_log.h"

#include <string.h>

/* 判断地址是否在内部 Flash 范围内 */
static uint8_t BSPFlashAddrValid(uint32_t address)
{
    return (address >= BSP_FLASH_START_ADDR) && (address < BSP_FLASH_END_ADDR);
}

uint8_t BSPFlashGetSector(uint32_t address)
{
    if (!BSPFlashAddrValid(address))
        return 0xFF;
    return (uint8_t)((address - BSP_FLASH_START_ADDR) / BSP_FLASH_SECTOR_SIZE);
}

/* 解锁 -> 操作 -> 上锁 */
static int8_t BSPFlashUnlock(void)
{
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        LOGERROR("[bsp_flash] flash unlock failed");
        return -1;
    }
    return 0;
}

static void BSPFlashLock(void)
{
    HAL_FLASH_Lock();
}

int8_t BSPFlashEraseSector(uint32_t address)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0;

    uint8_t sector = BSPFlashGetSector(address);
    if (sector == 0xFF)
    {
        LOGERROR("[bsp_flash] illegal address 0x%x", (unsigned)address);
        return -1;
    }

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1; /* H723 为单 Bank, 8 个 128KB 扇区 */
    erase.Sector = sector;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7V ~ 3.6V */

    if (BSPFlashUnlock() != 0)
        return -1;

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
    {
        LOGERROR("[bsp_flash] erase sector %d failed", (int)sector);
        BSPFlashLock();
        return -1;
    }

    BSPFlashLock();
    return 0;
}

int8_t BSPFlashEraseRange(uint32_t start_addr, uint32_t end_addr)
{
    if (start_addr > end_addr)
    {
        LOGERROR("[bsp_flash] erase range error");
        return -1;
    }

    uint8_t first = BSPFlashGetSector(start_addr);
    uint8_t last = BSPFlashGetSector(end_addr == BSP_FLASH_END_ADDR ? end_addr - 1U : end_addr);
    if (first == 0xFF || last == 0xFF)
    {
        LOGERROR("[bsp_flash] erase range out of flash");
        return -1;
    }

    for (uint8_t s = first; s <= last; ++s)
    {
        if (BSPFlashEraseSector(BSP_FLASH_START_ADDR + (uint32_t)s * BSP_FLASH_SECTOR_SIZE) != 0)
            return -1;
    }
    return 0;
}

/**
 * @brief 写 flash word(H7 每次必须写满 32 字节, 因此这里按 32 字节分批)
 */
int8_t BSPFlashWrite(uint32_t address, const uint32_t *data, uint32_t words)
{
    uint32_t aligned_buf[BSP_FLASH_WORD_SIZE / 4U];

    if (data == NULL || words == 0)
        return -1;

    /* 地址与长度都必须是 flash word(32B) 的整数倍 */
    if ((address % BSP_FLASH_WORD_SIZE) != 0 || (words % (BSP_FLASH_WORD_SIZE / 4U)) != 0)
    {
        LOGERROR("[bsp_flash] address/len must be aligned to %d bytes", (int)BSP_FLASH_WORD_SIZE);
        return -1;
    }

    if (BSPFlashUnlock() != 0)
        return -1;

    for (uint32_t i = 0; i < words; i += BSP_FLASH_WORD_SIZE / 4U)
    {
        /* HAL 要求源地址 32 字节对齐, 这里统一拷贝到对齐的临时缓冲区 */
        memcpy(aligned_buf, &data[i], BSP_FLASH_WORD_SIZE);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              address + i * 4U,
                              (uint32_t)aligned_buf) != HAL_OK)
        {
            LOGERROR("[bsp_flash] program failed at 0x%x", (unsigned)(address + i * 4U));
            BSPFlashLock();
            return -1;
        }
    }

    BSPFlashLock();
    return 0;
}

int8_t BSPFlashRead(uint32_t address, uint32_t *buf, uint32_t words)
{
    if (buf == NULL || words == 0)
        return -1;

    if (!BSPFlashAddrValid(address) || !BSPFlashAddrValid(address + words * 4U - 1U))
    {
        LOGERROR("[bsp_flash] read out of flash");
        return -1;
    }

    memcpy(buf, (const void *)address, words * 4U);
    return 0;
}
