#include "key.h"
#include "tim.h"
#include "stdio.h"
KeyInfo key[KEY_NUM];

// ���峤��ʱ����ֵ����λ��10ms��
#define LONG_PRESS_TIME 60 // 600ms
// ����˫�����ʱ����ֵ����λ��10ms��
#define DOUBLE_CLICK_TIME 20 // 200ms

/**
 * @brief       ������ʼ������
 * @param       ��
 * @retval      ��
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct; /* GPIO���ò����洢���� */
    KEY0_GPIO_CLK_ENABLE();            /* KEY0ʱ��ʹ�� */
    KEY1_GPIO_CLK_ENABLE();            /* KEY1ʱ��ʹ�� */
    KEY2_GPIO_CLK_ENABLE();            /* KEY2ʱ��ʹ�� */
    WKUP_GPIO_CLK_ENABLE();            /* WKUPʱ��ʹ�� */

    gpio_init_struct.Pin = KEY0_GPIO_PIN;             /* KEY0���� */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;          /* ���� */
    gpio_init_struct.Pull = GPIO_PULLUP;              /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    /* ���� */
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct); /* KEY0����ģʽ����,�������� */

    gpio_init_struct.Pin = KEY1_GPIO_PIN;             /* KEY1���� */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;          /* ���� */
    gpio_init_struct.Pull = GPIO_PULLUP;              /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    /* ���� */
    HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct); /* KEY1����ģʽ����,�������� */

    gpio_init_struct.Pin = KEY2_GPIO_PIN;             /* KEY2���� */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;          /* ���� */
    gpio_init_struct.Pull = GPIO_PULLUP;              /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    /* ���� */
    HAL_GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct); /* KEY2����ģʽ����,�������� */

    gpio_init_struct.Pin = WKUP_GPIO_PIN;             /* WKUP���� */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;          /* ���� */
    gpio_init_struct.Pull = GPIO_PULLDOWN;            /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    /* ���� */
    HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct); /* WKUP����ģʽ����,�������� */

    // ��ʼ��������Ϣ
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
    if (key_index >= KEY_NUM) // ��鰴�������Ƿ���Ч
    {
        return;
    }
    switch (key_index) // ���ݰ���������ȡ��Ӧ�İ���״̬
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
    if (key_index >= KEY_NUM) // ��鰴�������Ƿ���Ч
    {
        return;
    }
    read_keys(key_index);
    uint8_t key_pres_value = key_index == WK_UP_INDEX ? 1 : 0;
    key_state_machine(key, key->Read_level, key_pres_value);
}
// ����״̬��������
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
                // �����¼�
                key_info->key_result = KEY_LONG_PRESS;
                // ���������ӳ����������
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
                // ����������ֱ�ӻص�����״̬
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
            // ˫���¼�
            key_info->key_result = KEY_DOUBLE_CLICK;
            // ����������˫���������
            key_info->state = KEY_STATE_IDLE;
            key_info->click_count = 0;
        }
        break;
    case KEY_STATE_WAIT_DOUBLE_CLICK:
        key_info->press_time++;
        if (key_info->press_time >= DOUBLE_CLICK_TIME)
        {
            // �����¼�
            key_info->key_result = KEY_SINGLE_CLICK;
            // ���������ӵ����������
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
            key[i].key_result = KEY_NO_EVENT; // ���ð������״̬
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
    Key_last_Result = key_result;// ������һ�εİ������״̬
    switch (key_result)
    {
    case KEY_SINGLE_CLICK:
    
        switch (index)
        {
            case KEY0_INDEX:
                // ���� KEY0 �����¼�
                printf("KEY0 Single Click\n");
                set_motor_target_degree(0, 22.5f); // ���õ��0Ŀ��Ƕ�
                break;  
            case KEY1_INDEX:    
                // ���� KEY1 �����¼�
                printf("KEY1 Single Click\n");
                set_motor_target_degree(0, -22.5f); // ���õ��1Ŀ��Ƕ�
                break;
            case KEY2_INDEX:
                // ���� KEY2 �����¼�   
                printf("KEY2 Single Click\n");
                emergency_stop();    // ��ͣ���е��

                break;  
            case WK_UP_INDEX:
                // ���� WK_UP �����¼�  
                printf("WK_UP Single Click\n");
                restart_motors(); // �������е��
                break;
            default:
                break;
        }
        // �������¼�
        break;
    case KEY_DOUBLE_CLICK:  
        switch (index)  
        {
            case KEY0_INDEX:
                // ���� KEY0 ˫���¼�   
                printf("KEY0 Double Click\n");
                break;
            case KEY1_INDEX:
                // ���� KEY1 ˫���¼�   
                printf("KEY1 Double Click\n");
                break;
            case KEY2_INDEX:
                // ���� KEY2 ˫���¼�   
                printf("KEY2 Double Click\n");
                break;
            case WK_UP_INDEX:
                // ���� WK_UP ˫���¼�  
                printf("WK_UP Double Click\n"); 
                break;
            default:
                break;
        }
        // ����˫���¼�
        break;
    case KEY_LONG_PRESS:    
        switch (index)
        {
            case KEY0_INDEX:
                // ���� KEY0 �����¼�   
                printf("KEY0 Long Press\n");
                break;  
            case KEY1_INDEX:
                // ���� KEY1 �����¼�   
                printf("KEY1 Long Press\n");
                break;
            case KEY2_INDEX:
                // ���� KEY2 �����¼�       
                printf("KEY2 Long Press\n");
                break;  
            case WK_UP_INDEX:
                // ���� WK_UP �����¼�  
                printf("WK_UP Long Press\n");
                break;
            default:
                break;
        }
        // �������¼�
        break;
    default:
        break;
    }
}
