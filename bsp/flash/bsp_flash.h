#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include "main.h"

/**
 * @file bsp_flash.h
 * @brief 内部 Flash 读写(STM32H723, 1MB, 单 Bank, 8 个 128KB 扇区)
 *
 * @note 与 F4 版(F407: 16K/64K/128K 混合扇区 + 16 位半字编程)不同:
 *       H7 的 Flash 编程单位是 32 字节的 "flash word", 且擦除粒度是 128KB 扇区,
 *       因此接口按 "扇区擦除 + flash word 写入" 重新设计.
 *       做参数保存时请把数据放在最后一个扇区(0x080E0000 ~ 0x080FFFFF),
 *       避免和程序代码冲突.
 */

#define BSP_FLASH_START_ADDR    0x08000000UL
#define BSP_FLASH_END_ADDR      0x08100000UL
#define BSP_FLASH_SECTOR_SIZE   0x20000UL /* 128KB */
#define BSP_FLASH_SECTOR_CNT    8U
#define BSP_FLASH_WORD_SIZE     32U /* H7 的 flash word = 256bit = 32byte */

/* 建议用于保存参数的区域 */
#define BSP_FLASH_USER_START_ADDR (BSP_FLASH_START_ADDR + (BSP_FLASH_SECTOR_CNT - 1U) * BSP_FLASH_SECTOR_SIZE)

/**
 * @brief 由地址计算扇区号
 * @return 0~7, 地址非法返回 0xFF
 */
uint8_t BSPFlashGetSector(uint32_t address);

/**
 * @brief 擦除地址所在的扇区
 * @return 0 成功, -1 失败
 */
int8_t BSPFlashEraseSector(uint32_t address);

/**
 * @brief 擦除 [start_addr, end_addr) 覆盖到的所有扇区
 * @return 0 成功, -1 失败
 */
int8_t BSPFlashEraseRange(uint32_t start_addr, uint32_t end_addr);

/**
 * @brief 写入数据(长度必须是 32 字节的整数倍, 且地址 32 字节对齐)
 * @param data 源数据指针(不要求对齐: 内部会做一次对齐拷贝)
 * @param words 写入的 32bit 字数, 必须是 8 的整数倍(8 words = 32 bytes)
 * @return 0 成功, -1 失败
 */
int8_t BSPFlashWrite(uint32_t address, const uint32_t *data, uint32_t words);

/**
 * @brief 读取数据
 * @return 0 成功, -1 失败(地址越界)
 */
int8_t BSPFlashRead(uint32_t address, uint32_t *buf, uint32_t words);

#endif /* BSP_FLASH_H */
