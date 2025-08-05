#include <my_flash.h>
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"

// // 擦除指定地址的 Flash 页
// static HAL_StatusTypeDef flash_erase(void) 
// {
//     FLASH_EraseInitTypeDef erase_init;
//     uint32_t page_error = 0;
//     // 获取地址所在的扇区
//     uint32_t sector = HAL_FLASH_OB_GetSector(address);
//     erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
//     erase_init.Sector = FLASH_SECTOR_4;
//     erase_init.NbPages = 1;
//     erase_init.Banks = FLASH_BANK_2;
//     erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;


//     HAL_FLASH_Unlock();
//     HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
//     HAL_FLASH_Lock();

//     return status;
// }

// // 写入数据到指定 Flash 地址
// static HAL_StatusTypeDef flash_write(uint32_t address, uint32_t *data, uint32_t size)
//  {
//     HAL_FLASH_Unlock();
//     HAL_StatusTypeDef status = HAL_OK;

//     for (uint32_t i = 0; i < size; i++) 
//     {
//         status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address + i * 4, data[i]);
//         if (status != HAL_OK) 
//         {
//             break;
//         }
//     }

//     HAL_FLASH_Lock();
//     return status;
// }

// // 从指定 Flash 地址读取数据
// static void flash_read(uint32_t address, uint32_t *data, uint32_t size) 
// {
//     for (uint32_t i = 0; i < size; i++) 
//     {
//         data[i] = *(uint32_t *)(address + i * 4);
//     }
// }



// 擦除指定地址所在的扇区
static HAL_StatusTypeDef flash_erase(uint32_t address) 
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = 4;
    erase_init.NbSectors = 1;
    erase_init.Banks = FLASH_BANK_2; 
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    HAL_FLASH_Lock();

    return status;
}



// 向指定地址写入数据
static HAL_StatusTypeDef flash_write(uint32_t address, uint32_t *data, uint32_t size) 
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_OK;

        // 检查地址是否对齐
    if (address % FLASH_WORD_SIZE != 0) 
    {
        // 调整地址以满足对齐要求
        address += FLASH_WORD_SIZE - (address % FLASH_WORD_SIZE);
    }

    for (uint32_t i = 0; i < size; i++) 
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address + i * 4,(uint32_t) &data[i]);
        if (status != HAL_OK) 
        {
            break;
        }
    }

    HAL_FLASH_Lock();
    return status;
}

// 从指定地址读取数据
static void flash_read(uint32_t address, uint32_t *data, uint32_t size) 
{
    for (uint32_t i = 0; i < size; i++) 
    {
        data[i] = *(uint32_t *)(address + i * 4);
    }
}

// 写入 origin_point 数组到 Flash
HAL_StatusTypeDef write_origin_point_to_flash(int32_t* origin_point, uint32_t size) 
{
    HAL_StatusTypeDef erase_status = flash_erase(FLASH_SAVE_ADDRESS);
    if (erase_status != HAL_OK) 
    {
        return erase_status;
    }

    return flash_write(FLASH_SAVE_ADDRESS, (uint32_t *)origin_point , size);
}

// 从 Flash 读取数据到 origin_point 数组
void read_origin_point_from_flash(int32_t *origin_point, uint32_t size) 
{
    flash_read(FLASH_SAVE_ADDRESS, (uint32_t *)origin_point, size);
}
