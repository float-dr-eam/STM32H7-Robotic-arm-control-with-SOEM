#ifndef __MY_FLASH__
#define __MY_FLASH__

#include "LD3M_ec7010.h"
#include "main.h"

#define FLASH_SAVE_ADDRESS 0x08188000 // Flash保存地址

#define FLASH_WORD_SIZE 32  // 32 字节对齐  0x20



// 写入 origin_point 数组到 Flash
HAL_StatusTypeDef write_origin_point_to_flash(int32_t *origin_point, uint32_t size);

// 从 Flash 读取数据到 origin_point 数组
void read_origin_point_from_flash(int32_t *origin_point, uint32_t size);


#endif








