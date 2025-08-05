/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.c
 * @brief   This file provides code for the configuration
 *          of the TIM instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "key.h"
extern KeyInfo key[KEY_NUM];
/* USER CODE BEGIN 0 */
#include "DcSync.h"
#include "ethercat.h"
#include "servoS7test.h"
#include "LD3M_ec7010.h"
#include "usart.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 240;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}
/* TIM5 init function */
void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 240;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = sync_time - 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
}
/* TIM7 init function */
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 240 - 1; 
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000; 
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{

  if (tim_baseHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM5)
  {
    /* USER CODE BEGIN TIM5_MspInit 0 */

    /* USER CODE END TIM5_MspInit 0 */
    /* TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    /* TIM5 interrupt Init */
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    /* USER CODE BEGIN TIM5_MspInit 1 */

    /* USER CODE END TIM5_MspInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM7) 
  {
    // 使能TIM7时钟
    __HAL_RCC_TIM7_CLK_ENABLE();
    // 可以在此处添加NVIC中断配置，如果使用中断的话
    // 例如：
    HAL_NVIC_SetPriority(TIM7_IRQn, 0x01, 0x01);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{

  if (tim_baseHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */

    /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    /* USER CODE BEGIN TIM2_MspDeInit 1 */

    /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM5)
  {
    /* USER CODE BEGIN TIM5_MspDeInit 0 */

    /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /* TIM5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
    /* USER CODE BEGIN TIM5_MspDeInit 1 */

    /* USER CODE END TIM5_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* TIM7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

uint32_t time_s, time_ms, time_us,time_10ms=0;
extern int dorun;
extern int64 integra;
extern int64 toff;
extern int64 cycletime;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) // 1 s
  {
    LED0_Toggle;
    time_s++;
  }

  if (htim->Instance == TIM5) // 1ms
  {
    time_ms++;
    if (dorun == 1)
    {
      LD3M_loop();
      ec_sync(ec_DCtime, cycletime, &toff); //ec_DCtime为ecat的周期
      TIM5->ARR = (uint32_t)(cycletime + toff);
    }
  }

  if(htim->Instance == TIM7)// 10ms
  {
    time_10ms++;
    for(int i = 0; i < KEY_NUM; i++)
    {
      handle_key_event(&key[i], i);
    }
  }
}
uint32_t GetSec(void)
{
  return time_s;
}

uint32_t GetUSec(void)
{
  time_us = (TIM2->CNT) % 1000000;
  return time_us;
}

/*


//计时器 通过读取CNT值实现计时 避免中断的影响 TIM2 TIM3
//设置TIM2主模式，输入时钟1mhz，产生1hz的TRGO信号
//设置TIM3为从模式，时间源为TIM2更新信号
void PhyTim_EtherCAT_Init(void)
{
    //-----------------TIM2初始化------------------//
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;                //定时器主模式配置
    TIM_SlaveConfigTypeDef sSlaveConfig;                //定时器从模式配置

    TIM2_Handler.Instance = TIM2;                        //通用定时器2
    TIM2_Handler.Init.Prescaler=199;                     //分频
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM2_Handler.Init.Period=1000000-1;                  //自动装载值
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    TIM2_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;        //自动重装载 写入新值后，计数器完成当前旧的计数后，再开始新的计数周期
    HAL_TIM_Base_Init(&TIM2_Handler);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;        //使用内部时钟源 外部触发预分频器 关闭预分频器  TIM2->SMCR
    HAL_TIM_ConfigClockSource(&TIM2_Handler, &sClockSourceConfig);        //定时器时钟配置

    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;                //TIM2设置为主模式
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;                            // 选择更新事件作为触发输出 (TRGO)
    HAL_TIMEx_MasterConfigSynchronization(&TIM2_Handler, &sMasterConfig);

    HAL_TIM_Base_Start(&TIM2_Handler);                 //使能定时器2

    //-----------------TIM5初始化------------------//
    TIM3_Handler.Instance = TIM5;                                      //通用定时器5
    TIM3_Handler.Init.Prescaler = 0;                                   //不分频
    TIM3_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;                //向上计数器
    TIM3_Handler.Init.Period = 50000;                                //自动装载值
    TIM3_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;          //时钟分频因子
    HAL_TIM_Base_Init(&TIM5_Handler);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ITR1;  //定时器时钟源选择
    HAL_TIM_ConfigClockSource(&TIM5_Handler, &sClockSourceConfig);

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;    //外部时钟模式1 由所选触发信号 (TRGI) 的上升沿提供计数器时钟 TIM3->SMCR[2;0]=111
    sSlaveConfig.InputTrigger = TIM_TS_ITR1;                        // 输入触发：选择 ITR0 作为输入源
    sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;// 触发极性：上升沿
    sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;// 触发预分频：无
    sSlaveConfig.TriggerFilter = 0x0;                         // 滤波：不需要任何滤
    HAL_TIM_SlaveConfigSynchronization(&TIM5_Handler, &sSlaveConfig);   //定时器从模式配置

    HAL_TIM_Base_Start(&TIM5_Handler);
}



//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM6)      //0.5ms   TIM6
    {
        __HAL_RCC_TIM6_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn,3,1);
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
    else if(htim->Instance==TIM7)   //2ms   TIM7
    {
        __HAL_RCC_TIM7_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM7_IRQn,4,1);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
    }
    else if(htim->Instance==TIM2)   // TIM2
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
    else if(htim->Instance==TIM5)   // TIM3
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
    }
}

//读取时间
uint32_t GetSec(void)   //获取时间/秒
{
    return TIM5->CNT;
}


uint32_t GetUSec(void)   //获取时间/微妙
{
    return TIM2->CNT;
}
*/

void Delay_us(uint32_t nus)
{
  uint16_t tnow, told, tcnt =nus * (SystemCoreClock / 1000000); // 这里*480是因为Systick的时钟来源是HLCK，
                                         // 而HLCK是480M也就是480*10^6，按照s和us的换算就是72
  uint32_t load = SysTick->LOAD;
  told = SysTick->VAL;
  while (1)
  {
    tnow = SysTick->VAL;

    if (tnow > told)
    {
      tcnt -= load + (tnow - told);
    }
    else if (tnow < told)
    {
      tcnt -= told - tnow;
    }
    told = tnow;
    if (tcnt <= 0)
      break;
  }
}
// void Delay_us(uint32_t nus)
// {
//     uint32_t ticks;
//     uint32_t start, end;
//     uint32_t load = SysTick->LOAD;
//     uint32_t reload = load + 1;
    
//     // 计算需要的时钟周期数
//     ticks = nus * (SystemCoreClock / 1000000);
    
//     // 获取当前计数器值
//     start = SysTick->VAL;
    
//     while(1)
//     {
//         end = SysTick->VAL;
        
//         if(end != start)
//         {
//             if(end < start)
//             {
//                 // 计数器从0重新加载
//                 ticks -= (start - end);
//             }
//             else
//             {
//                 // 正常计数
//                 ticks -= (reload - end + start);
//             }
            
//             if(ticks <= 0)
//             {
//                 break;
//             }
            
//             start = end;
//         }
//     }
// }
/* USER CODE END 1 */
