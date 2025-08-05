#ifndef __KEY_H
#define __KEY_H

#include "LD3M_ec7010.h"
#include "stm32h7xx_hal.h"
/******************************************************************************************/
/* 使能HAL_TIM_MODULE */
#define USE_HAL_TIM_MODULE 1   

/* 引脚 定义 */
#define KEY0_GPIO_PORT                  GPIOH
#define KEY0_GPIO_PIN                   GPIO_PIN_3
#define KEY0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)     /* PH口时钟使能 */

#define KEY1_GPIO_PORT                  GPIOH
#define KEY1_GPIO_PIN                   GPIO_PIN_2
#define KEY1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)     /* PH口时钟使能 */

#define KEY2_GPIO_PORT                  GPIOC
#define KEY2_GPIO_PIN                   GPIO_PIN_13
#define KEY2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)     /* PC口时钟使能 */

#define WKUP_GPIO_PORT                  GPIOA
#define WKUP_GPIO_PIN                   GPIO_PIN_0
#define WKUP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

#define KEY0_INDEX  0              
#define KEY1_INDEX  1             
#define KEY2_INDEX  2             
#define WK_UP_INDEX 3             

#define KEY_NUM   4
/******************************************************************************************/
// 按键过程状态定义
typedef enum
{
    KEY_STATE_IDLE,
    KEY_STATE_PRESSED,
    KEY_STATE_RELEASED,
    KEY_STATE_WAIT_DOUBLE_CLICK
} KeyState;

// 按键结果状态定义
typedef enum
{
    KEY_NO_EVENT,
    KEY_SINGLE_CLICK,
    KEY_DOUBLE_CLICK,
    KEY_LONG_PRESS
} KeyResult;
// 按键信息结构体
typedef struct
{
    KeyState state;        // 按键状态
    uint8_t click_count;     // 按键点击次数
    uint16_t press_time;     // 按键按下时间（单位：10ms）
    uint8_t Read_level;      // 按键读取电平
    KeyResult key_result;    // 按键结果状态
} KeyInfo;

extern KeyInfo key[KEY_NUM];

void key_init(void);                    /* 按键初始化函数 */
void handle_key_event(KeyInfo *key, uint8_t key_index); /* 定时器处理按键事件 */

void key_state_machine(KeyInfo *key_info, uint8_t key_state, uint8_t key_pres_value);// 按键状态机处理函数
void key_event_handler(uint8_t index , KeyResult key_result);   /* 按键事件处理函数 */
void key_function(void);                /* 按键函数 */

#endif



