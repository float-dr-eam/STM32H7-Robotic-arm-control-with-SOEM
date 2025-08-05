#ifndef __KEY_H
#define __KEY_H

#include "LD3M_ec7010.h"
#include "stm32h7xx_hal.h"
/******************************************************************************************/
/* ʹ��HAL_TIM_MODULE */
#define USE_HAL_TIM_MODULE 1   

/* ���� ���� */
#define KEY0_GPIO_PORT                  GPIOH
#define KEY0_GPIO_PIN                   GPIO_PIN_3
#define KEY0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)     /* PH��ʱ��ʹ�� */

#define KEY1_GPIO_PORT                  GPIOH
#define KEY1_GPIO_PIN                   GPIO_PIN_2
#define KEY1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)     /* PH��ʱ��ʹ�� */

#define KEY2_GPIO_PORT                  GPIOC
#define KEY2_GPIO_PIN                   GPIO_PIN_13
#define KEY2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)     /* PC��ʱ��ʹ�� */

#define WKUP_GPIO_PORT                  GPIOA
#define WKUP_GPIO_PIN                   GPIO_PIN_0
#define WKUP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA��ʱ��ʹ�� */

#define KEY0_INDEX  0              
#define KEY1_INDEX  1             
#define KEY2_INDEX  2             
#define WK_UP_INDEX 3             

#define KEY_NUM   4
/******************************************************************************************/
// ��������״̬����
typedef enum
{
    KEY_STATE_IDLE,
    KEY_STATE_PRESSED,
    KEY_STATE_RELEASED,
    KEY_STATE_WAIT_DOUBLE_CLICK
} KeyState;

// �������״̬����
typedef enum
{
    KEY_NO_EVENT,
    KEY_SINGLE_CLICK,
    KEY_DOUBLE_CLICK,
    KEY_LONG_PRESS
} KeyResult;
// ������Ϣ�ṹ��
typedef struct
{
    KeyState state;        // ����״̬
    uint8_t click_count;     // �����������
    uint16_t press_time;     // ��������ʱ�䣨��λ��10ms��
    uint8_t Read_level;      // ������ȡ��ƽ
    KeyResult key_result;    // �������״̬
} KeyInfo;

extern KeyInfo key[KEY_NUM];

void key_init(void);                    /* ������ʼ������ */
void handle_key_event(KeyInfo *key, uint8_t key_index); /* ��ʱ���������¼� */

void key_state_machine(KeyInfo *key_info, uint8_t key_state, uint8_t key_pres_value);// ����״̬��������
void key_event_handler(uint8_t index , KeyResult key_result);   /* �����¼������� */
void key_function(void);                /* �������� */

#endif



