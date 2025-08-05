/**
 ****************************************************************************************************
 * @file        pcf8574.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-09-06
 * @brief       PCF8574 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 阿波罗 H743开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20220906
 * 第一次发布
 *
 ****************************************************************************************************
 */
 
#ifndef __PCF8574_H
#define __PCF8574_H
#include "stm32h7xx_hal.h"
//#include "./SYSTEM/sys/sys.h"
#include "myiic.h"

/******************************************************************************************/
/* 引脚 定义 */

#define PCF8574_GPIO_PORT                  GPIOB
#define PCF8574_GPIO_PIN                   GPIO_PIN_12
#define PCF8574_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)      /* PB口时钟使能 */

/******************************************************************************************/

#define PCF8574_INT  HAL_GPIO_ReadPin(PCF8574_GPIO_PORT,PCF8574_GPIO_PIN)                   /* PCF8574 INT脚 */

#define PCF8574_ADDR  0X40      /* PCF8574地址(左移了一位) */

/* PCF8574各个IO的功能 */
#define BEEP_IO         0       /* 蜂鸣器控制引脚        P0 */
#define AP_INT_IO       1       /* AP3216C中断引脚       P1 */
#define DCMI_PWDN_IO    2       /* DCMI的电源控制引脚    P2 */
#define USB_PWR_IO      3       /* USB电源控制引脚       P3 */
#define EX_IO           4       /* 扩展IO,自定义使用     P4 */
#define MPU_INT_IO      5       /* SH3001中断引脚        P5 */
#define RS485_RE_IO     6       /* RS485_RE引脚          P6 */
#define ETH_RESET_IO    7       /* 以太网复位引脚        P7 */

/******************************************************************************************/

uint8_t pcf8574_init(void); 
uint8_t pcf8574_read_byte(void);
void pcf8574_write_byte(uint8_t data);
void pcf8574_write_bit(uint8_t bit, uint8_t sta);
uint8_t pcf8574_read_bit(uint8_t bit);

#endif

