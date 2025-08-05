/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "eth.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "osal.h"
#include "ethercat.h"

#include "simpletest.h"
#include "slaveinfo.h"
#include "servoS7test.h"
#include "LD3M_ec7010.h"
#include "my_arm.h"

#include "sdram.h"
#include "lcd.h"
#include "key.h"
#include "ltdc.h"
#include "touch.h"
#include "myiic.h"
#include "pcf8574.h"
#include "usmart.h"
#include "malloc.h"
#include "mpu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint32_t time_s, time_ms, time_us;
extern int linkState;
extern int phyEvent;
char IOmap[IOmap_Length]; /* EtherCAT process data mapping */
int dorun;                /* loop control */
bool update_flag = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  time_s = 0;
  time_ms = 0;
  time_us = 0;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
 // mpu_memory_protection();                 /* 保护相关存储区域 */
  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  StartReceiving();

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);
  // HAL_TIM_Base_Start_IT(&htim7);

  /* 初始化USMART */
  usmart_dev.init(240); /* 初始化USMART, 240Mhz */
  /* 初始化内存管理控制器 */
  sdram_init();            /* SDRAM初始化 */
  lcd_init();              /* LCD初始化 */
  mallco_dev.init(SRAMEX); /* 初始化内存管理控制器,管理SRAMIN区域的内存 */

 
  lcd_show_string(200, 10, 200, 16, 16, "lcd ok\n", BLACK);
  key_init(); /* 按键初始化 */
  //	tp_dev.init();
  while (pcf8574_init()) /* PCF8574 */
  {
    printf("PCF8574 Check Failed!\n");
    Delay_us(1000);
    printf("Please Check!      \n");
  }
  printf("PCF8574 Check Success!\n");
  lcd_show_string(200, 30, 200, 16, 16, "PCF8574 Check Success!\n", BLACK);
  MX_ETH_Init();       /* 以太网初始化 */
  get_eth_linkstate(); /* 获得ETH连接状态 */
  linkState = FALSE;
  /* USER CODE END 2 */
  // uint8 u8val = 0;
  bool flag = 1;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    key_function(); /* 按键处理函数 */

    PhyTick();
    if (phyEvent)
    {
      phyEvent = FALSE;
      PhyEventHandler();  /* 处理物理层事件 */
      LD3M_test("hello"); /* LD3M测试 */
      lcd_show_string(100, 1, 20, 16, 16, "1", BLUE);
    }
    else
    {
      time_ms++;
      if (time_ms >= 1000)
      {
        time_ms = 0;
        ecatcheck();
      }
      lcd_show_string(100, 1, 20, 16, 16, "2", BLUE);
      LED1_Toggle;
      HAL_Delay(200);
    }
    for (uint8_t i = 0; i < ec_slavecount; i++)
    {
      // 计算当前行号和列号
      uint8_t row = i / 3;
      uint8_t col = i % 3;

      char str[20];

      // 计算 cur_mode 显示位置
      int y_cur_mode = 250 + row * 90;
      int x_cur_mode = 10 + col * 200;
      // 限制写入字符串长度，防止缓冲区溢出
      snprintf(str, sizeof(str), "cur_mode[%d]=%d    ", i, LD3M_all.cur_mode[i]);
      lcd_show_string(x_cur_mode, y_cur_mode, 200, 16, 16, str, BLACK);

      // 计算 cur_pos 显示位置
      int y_cur_pos = 270 + row * 90;
      int x_cur_pos = 10 + col * 200;
      // 限制写入字符串长度，防止缓冲区溢出
      snprintf(str, sizeof(str), "cur_pos[%d]=%d    ", i, LD3M_all.cur_pos[i]);
      lcd_show_string(x_cur_pos, y_cur_pos, 200, 16, 16, str, BLACK);

      // 计算 cur_vel 显示位置
      int y_cur_vel = 290 + row * 90;
      int x_cur_vel = 10 + col * 200;
      // 限制写入字符串长度，防止缓冲区溢出
      snprintf(str, sizeof(str), "cur_vel[%d]=%d    ", i, LD3M_all.cur_vel[i]);
      lcd_show_string(x_cur_vel, y_cur_vel, 200, 16, 16, str, BLACK);

      // 计算 cur_degree 显示位置
      int y_status_word = 310 + row * 90;
      int x_status_word = 10 + col * 200;
      // 限制写入字符串长度，防止缓冲区溢出
      snprintf(str, sizeof(str), "cur_degree[%d]=%6.4f    ", i, LD3M_all.cur_degree[i]);
      lcd_show_string(x_status_word, y_status_word, 200, 16, 16, str, BLACK);
    }
    if (flag)
    {
      flag = 0;
      my_float degree[NUM_SLAVES] = {0, 90, 0, 90, 0, -90, 0}; /* 机械臂各关节角度 */
      //arm_forward_kinematics2(degree, RIGHT);                 /* 计算机械臂运动学 */

      arm_forward_kinematics(degree);
      all_motors_return_to_origin();
      //HAL_Delay(10000);
      //simple_demo();
    }
    if (update_flag)
    {
      update_flag = 0;
      arm_forward_kinematics(LD3M_all.cur_degree); /* 计算机械臂运动学 */
    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
