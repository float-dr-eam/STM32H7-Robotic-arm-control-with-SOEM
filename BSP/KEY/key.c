#include "key.h"
#include "tim.h"
#include "stdio.h"
KeyInfo key[KEY_NUM];

// 定义长按时间阈值（单位：10ms）
#define LONG_PRESS_TIME 60 // 600ms
// 定义双击间隔时间阈值（单位：10ms）
#define DOUBLE_CLICK_TIME 20 // 200ms

/**
 * @brief       按键初始化函数
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct; /* GPIO配置参数存储变量 */
    KEY0_GPIO_CLK_ENABLE();            /* KEY0时钟使能 */
    KEY1_GPIO_CLK_ENABLE();            /* KEY1时钟使能 */
    KEY2_GPIO_CLK_ENABLE();            /* KEY2时钟使能 */
    WKUP_GPIO_CLK_ENABLE();            /* WKUP时钟使能 */

    gpio_init_struct.Pin = KEY0_GPIO_PIN;             /* KEY0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;          /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;              /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    /* 高速 */
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct); /* KEY0引脚模式设置,上拉输入 */

    gpio_init_struct.Pin = KEY1_GPIO_PIN;             /* KEY1引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;          /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;              /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    /* 高速 */
    HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct); /* KEY1引脚模式设置,上拉输入 */

    gpio_init_struct.Pin = KEY2_GPIO_PIN;             /* KEY2引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;          /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;              /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    /* 高速 */
    HAL_GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct); /* KEY2引脚模式设置,上拉输入 */

    gpio_init_struct.Pin = WKUP_GPIO_PIN;             /* WKUP引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;          /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLDOWN;            /* 下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    /* 高速 */
    HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct); /* WKUP引脚模式设置,下拉输入 */

    // 初始化按键信息
    for (int i = 0; i < KEY_NUM; i++)
    {
        key[i].state = KEY_STATE_IDLE;
        key[i].click_count = 0;
        key[i].press_time = 0;
        key[i].Read_level = 0;
        key[i].key_result = KEY_NO_EVENT;
    }
#if USE_HAL_TIM_MODULE
    HAL_TIM_Base_Start_IT(&htim7);
#endif
}

void read_keys(uint8_t key_index)
{
    if (key_index >= KEY_NUM) // 检查按键索引是否有效
    {
        return;
    }
    switch (key_index) // 根据按键索引读取对应的按键状态
    {
        case KEY0_INDEX:
            key[KEY0_INDEX].Read_level = HAL_GPIO_ReadPin(KEY0_GPIO_PORT, KEY0_GPIO_PIN);
            break;
        case KEY1_INDEX:
            key[KEY1_INDEX].Read_level = HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_GPIO_PIN);
            break;
        case KEY2_INDEX:
            key[KEY2_INDEX].Read_level = HAL_GPIO_ReadPin(KEY2_GPIO_PORT, KEY2_GPIO_PIN);
            break;
        case WK_UP_INDEX:
            key[WK_UP_INDEX].Read_level = HAL_GPIO_ReadPin(WKUP_GPIO_PORT, WKUP_GPIO_PIN);
            break;
        default:
            break;
    }
}

void handle_key_event(KeyInfo *key, uint8_t key_index)
{
    if (key_index >= KEY_NUM) // 检查按键索引是否有效
    {
        return;
    }
    read_keys(key_index);
    uint8_t key_pres_value = key_index == WK_UP_INDEX ? 1 : 0;
    key_state_machine(key, key->Read_level, key_pres_value);
}
// 按键状态机处理函数
void key_state_machine(KeyInfo *key_info, uint8_t key_state, uint8_t key_pres_value)
{

    switch (key_info->state)
    {
    case KEY_STATE_IDLE:
        if (key_state == key_pres_value)
        {
            key_info->state = KEY_STATE_PRESSED;
            key_info->press_time = 0;
        }
        break;
    case KEY_STATE_PRESSED:
        if (key_state == key_pres_value)
        {
            key_info->press_time++;
            if (key_info->press_time >= LONG_PRESS_TIME)
            {
                // 长按事件
                key_info->key_result = KEY_LONG_PRESS;
                // 这里可以添加长按处理代码
                key_info->state = KEY_STATE_IDLE;
                key_info->click_count = 0;
                key_info->press_time = 0;
            }
        }
        else
        {
            if (key_info->press_time < LONG_PRESS_TIME)
            {
                key_info->state = KEY_STATE_RELEASED;
                key_info->click_count++;
            }
            else
            {
                // 长按结束后直接回到空闲状态
                key_info->state = KEY_STATE_IDLE;
                key_info->click_count = 0;
                key_info->press_time = 0;
            }
        }
        break;
    case KEY_STATE_RELEASED:
        if (key_info->click_count == 1)
        {
            key_info->state = KEY_STATE_WAIT_DOUBLE_CLICK;
            key_info->press_time = 0;
        }
        else if (key_info->click_count == 2)
        {
            // 双击事件
            key_info->key_result = KEY_DOUBLE_CLICK;
            // 这里可以添加双击处理代码
            key_info->state = KEY_STATE_IDLE;
            key_info->click_count = 0;
        }
        break;
    case KEY_STATE_WAIT_DOUBLE_CLICK:
        key_info->press_time++;
        if (key_info->press_time >= DOUBLE_CLICK_TIME)
        {
            // 单击事件
            key_info->key_result = KEY_SINGLE_CLICK;
            // 这里可以添加单击处理代码
            key_info->state = KEY_STATE_IDLE;
            key_info->click_count = 0;
        }
        else if (key_state == key_pres_value)
        {
            key_info->state = KEY_STATE_PRESSED;
            key_info->press_time = 0;
        }
        break;
    default:
        key_info->state = KEY_STATE_IDLE;
        key_info->click_count = 0;
        key_info->press_time = 0;
        key_info->key_result = KEY_NO_EVENT;
        break;
    }
}

void key_function(void)
{
    for(int i = 0; i < KEY_NUM; i++)
    {
        if (key[i].key_result != KEY_NO_EVENT)
        {
            key_event_handler(i,key[i].key_result);
            key[i].key_result = KEY_NO_EVENT; // 重置按键结果状态
        }
    }
}
void key_event_handler(uint8_t index , KeyResult key_result)
{
    static KeyResult Key_last_Result; 
    if(Key_last_Result ==  KEY_LONG_PRESS && key_result == KEY_SINGLE_CLICK)
    {
        return;
    }
    Key_last_Result = key_result;// 保存上一次的按键结果状态
    switch (key_result)
    {
    case KEY_SINGLE_CLICK:
    
        switch (index)
        {
            case KEY0_INDEX:
                // 处理 KEY0 单击事件
                printf("KEY0 Single Click\n");
                set_motor_target_degree(0, 22.5f); // 设置电机0目标角度
                break;  
            case KEY1_INDEX:    
                // 处理 KEY1 单击事件
                printf("KEY1 Single Click\n");
                set_motor_target_degree(0, -22.5f); // 设置电机1目标角度
                break;
            case KEY2_INDEX:
                // 处理 KEY2 单击事件   
                printf("KEY2 Single Click\n");
                emergency_stop();    // 急停所有电机

                break;  
            case WK_UP_INDEX:
                // 处理 WK_UP 单击事件  
                printf("WK_UP Single Click\n");
                restart_motors(); // 重启所有电机
                break;
            default:
                break;
        }
        // 处理单击事件
        break;
    case KEY_DOUBLE_CLICK:  
        switch (index)  
        {
            case KEY0_INDEX:
                // 处理 KEY0 双击事件   
                printf("KEY0 Double Click\n");
                break;
            case KEY1_INDEX:
                // 处理 KEY1 双击事件   
                printf("KEY1 Double Click\n");
                break;
            case KEY2_INDEX:
                // 处理 KEY2 双击事件   
                printf("KEY2 Double Click\n");
                break;
            case WK_UP_INDEX:
                // 处理 WK_UP 双击事件  
                printf("WK_UP Double Click\n"); 
                break;
            default:
                break;
        }
        // 处理双击事件
        break;
    case KEY_LONG_PRESS:    
        switch (index)
        {
            case KEY0_INDEX:
                // 处理 KEY0 长按事件   
                printf("KEY0 Long Press\n");
                break;  
            case KEY1_INDEX:
                // 处理 KEY1 长按事件   
                printf("KEY1 Long Press\n");
                break;
            case KEY2_INDEX:
                // 处理 KEY2 长按事件       
                printf("KEY2 Long Press\n");
                break;  
            case WK_UP_INDEX:
                // 处理 WK_UP 长按事件  
                printf("WK_UP Long Press\n");
                break;
            default:
                break;
        }
        // 处理长按事件
        break;
    default:
        break;
    }
}
