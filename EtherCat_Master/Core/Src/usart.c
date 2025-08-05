/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
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
#include "usart.h"
#include "tim.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;              // 16倍过采样
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;     // 单比特采样
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;             // 时钟分频因子
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // 高级功能初始化
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if (uartHandle->Instance == USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */

    /* USER CODE END USART1_MspInit 0 */

    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Stream0;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Stream1;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspInit 1 */
    /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{

  if (uartHandle->Instance == USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */

    /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspDeInit 1 */

    /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include "stdio.h"

#if !defined(__MICROLIB)
// 不使用微库的话就需要添加下面的函数
/* 如果开启（条件成立），并且编译器版本大于等于 6010050（AC6） */
#if (__ARMCC_VERSION >= 6010050)
__asm(".global __use_no_semihosting\n\t"); /* 关闭半主机调试模式 */
__asm(".global __ARM_use_no_argv \n\t");   /* 如果使用 AC6 编译器，禁止 main 函数使用argv和环境变量 */

#else
/* 如果使用 AC5 编译器，需要定义 __FILE 来避免报错，启动不支持半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
  int handle;
  /* 这里定义了文件结构体，通常只需要标准输出就可以使用printf */
};

#endif

/* 如果没有调试输出支持（关闭半主机），会使用此函数 */
int _ttywrch(int ch)
{
  ch = ch;
  return ch;
}

/* 模拟退出系统的行为，这里空实现 */
void _sys_exit(int x)
{
  x = x;
}

/* 空的命令字符串处理函数 */
char *_sys_command_string(char *cmd, int len)
{
  return NULL;
}

/* 定义标准输出流 */
FILE __stdout;

// #define TX_BUF_SIZE 1024
// #define DMA_MIN_TRANSFER 16 // DMA最小传输长度，避免频繁启动DMA
// uint8_t tx_buf[TX_BUF_SIZE];
// volatile uint16_t tx_head = 0;  // 用于跟踪缓冲区的写入位置
// volatile uint16_t tx_tail = 0;  // 用于跟踪缓冲区的读取位置
// volatile uint8_t uart_busy = 0; // 用于跟踪 DMA 传输是否繁忙
// // 定义一个全局变量来标记是否需要继续发送剩余数据
// volatile uint8_t need_continue_transmit = 0;

// // 在文件开头添加
// #define ENTER_CRITICAL()    uint32_t primask = __get_PRIMASK(); __disable_irq()
// #define EXIT_CRITICAL() __set_PRIMASK(primask)

// uint16_t tx_available()
// {
//   ENTER_CRITICAL();
//   uint16_t avail = (tx_head >= tx_tail) ? (TX_BUF_SIZE - (tx_head - tx_tail) - 1) : (tx_tail - tx_head - 1);
//   EXIT_CRITICAL();
//   return avail;
// }
// /**
//  * @brief 启动DMA传输（必须在临界区内调用）
//  */
// static void start_dma_transfer(void)
// {
//   uint16_t len;
//   if (tx_head > tx_tail)
//   {
//     len = tx_head - tx_tail;
//   }
//   else
//   {
//     len = TX_BUF_SIZE - tx_tail;
//   }
//   uart_busy = 1;
//   //len = (len < DMA_MIN_TRANSFER) ? len : DMA_MIN_TRANSFER; // 确保最小传输长度
//   if (HAL_UART_Transmit_DMA(&huart1, &tx_buf[tx_tail], len) != HAL_OK)
//   {
//     uart_busy = 0; // 传输启动失败
//   }
// }
// int fputc(int ch, FILE *f) {
//     ENTER_CRITICAL();
//     uint16_t next = (tx_head + 1) % TX_BUF_SIZE;
    
//     if(next == tx_tail) { // 缓冲区满
//         EXIT_CRITICAL();
//         // 可选：等待或丢弃数据
//         while(tx_available() == 0) {
//             if(!uart_busy && tx_head != tx_tail) {
//                 ENTER_CRITICAL();
//                 start_dma_transfer();
//                 EXIT_CRITICAL();
//             }
//             Delay_us(100);
//         }
//         ENTER_CRITICAL();
//         next = (tx_head + 1) % TX_BUF_SIZE;
//     }

//     tx_buf[tx_head] = (uint8_t)ch;
//     tx_head = next;
    
//     if (!uart_busy && (tx_head != tx_tail)) {
//         start_dma_transfer();
//     }
//     EXIT_CRITICAL();
//     return ch;
// }

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if (huart == &huart1)
//   {
//     ENTER_CRITICAL();
//     // 替代方案：使用HAL提供的API获取剩余计数
//     uint16_t remaining = __HAL_DMA_GET_COUNTER(&hdma_usart1_tx);
//     uint16_t sent_len = huart->TxXferSize - remaining;
//     // 更新tail位置（确保数据确实已发送）
//     tx_tail = (tx_tail + sent_len) % TX_BUF_SIZE;
//     // 检查是否还有数据要发送
//     if (tx_head != tx_tail)
//     {
//       start_dma_transfer();
//     }
//     else
//     {
//       uart_busy = 0;
//     }
//     EXIT_CRITICAL();
//   }
// }

/* fputc 函数用于将数据输出到 USART */
int fputc(int ch, FILE *f)
{
  while ((USART1->ISR & 0X40) == 0) ;                        /* 等待USART1的发送缓冲区为空 */
  USART1->TDR = (uint8_t)ch; /* 将字符写入USART的数据寄存器 */
  return ch;
}

/**
 * fgetc函数用于从USART接收字符
 * 通过 HAL_UART_Receive 函数从 USART1 接收字符
 */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  // HAL_UART_Receive(&huart1, &ch, 1, 0xffff); /* 接收一个字节数据 */
  HAL_UART_Receive_DMA(&huart1, &ch, 1);
  return ch;
}
#endif

/* 定义接收缓冲区 */
uint8_t receivedByte;
uint16_t n = 0;
uint8_t receivedBuff[receivedBuff_Length]; // 接收缓冲区

/*接收状态---------------------空闲中断
bit15，  接收完成标志         -
bit14~0，接收到的有效字节数目*/

/*  接收状态
 *  bit15，      接收完成标志
 *  bit14，      接收到0x0d
 *  bit13~0，    接收到的有效字节数目
 */
uint16_t USART_RX_STA = 0; // 接收状态标记

uint16_t USART_RX_CNT = 0; // 接收的字节数

/* 启动异步接收 */
void StartReceiving(void)
{
  // HAL_UART_Receive_IT(&huart1, &receivedByte, 1); // 启动接收一个字节
  HAL_UART_Receive_DMA(&huart1, receivedBuff, receivedBuff_Length);

  __HAL_UART_CLEAR_IDLEFLAG(&huart1);          // 清除IDLE标志
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // 使能 IDLE中断
}

// HAL_UART_Transmit_DMA(&huart1,data_character, sizeof(data_character));
// HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

void print_buff(void)
{
  // 在字符串末尾添加 '\0'，确保它是一个有效的字符串
  USART_RX_CNT = USART_RX_STA & 0x7fff; // 计算长度
  receivedBuff[USART_RX_CNT] = '\0';
  // 打印缓冲区中的内容
  printf("Buffer: %s\n", receivedBuff);
}

void finish_one_receieve(void)
{
  if (USART_RX_STA & 0x8000) // 接收到数据
  {
    USART_RX_CNT = USART_RX_STA & 0x7fff;                       // 计算长度
    HAL_UART_Transmit_DMA(&huart1, receivedBuff, USART_RX_CNT); // 发送接收到的数据
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET)
      ; // 等待发送完成
    printf("\r\n");
    USART_RX_STA = 0;
  }
}
// DMA发送函数
void UART_DMA_Send(uint8_t *data, uint16_t size) {
    // 等待上一次传输完成
    while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {}
    
    // 启动DMA传输
    HAL_UART_Transmit_DMA(&huart1, data, size);
    
    // 等待传输完成
    while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {}
}

// DMA接收函数
void UART_DMA_Receive(uint8_t *data, uint16_t size) {
    // 清除IDLE标志
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    
    // 使能IDLE中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    
    // 启动DMA接收
    HAL_UART_Receive_DMA(&huart1, data, size);
}

// 串口DMA接收重启函数
void UART_DMA_Receive_Restart(void) {
    // 停止DMA传输
    HAL_UART_DMAStop(&huart1);
    
    // 清空接收状态
    USART_RX_STA = 0;
    USART_RX_CNT = 0;
    
    // 重新启动DMA接收
    HAL_UART_Receive_DMA(&huart1, receivedBuff, receivedBuff_Length);
    
    // 清除IDLE标志并使能IDLE中断
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

// 串口接收空闲中断回调函数，这个是从hal里面新加的
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (USART_RX_STA >= 1)
    {
      USART_RX_STA |= 0x8000; // 已经触发空闲中断回调，剩下内容由用户操作
      printf("ok\r\n");
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (HAL_UART_GetError(huart) & HAL_UART_ERROR_PE)
  { /*!< Parity error            */
    // 奇偶校验错误
    __HAL_UART_CLEAR_PEFLAG(huart);
  }
  else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_NE)
  { /*!< Noise error             */
    // 噪声错误
    __HAL_UART_CLEAR_NEFLAG(huart);
  }
  else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_FE)
  { /*!< Frame error             */
    // 帧格式错误
    __HAL_UART_CLEAR_FEFLAG(huart);
  }
  else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
  { /*!< Overrun error           */
    // 数据太多串口来不及接收错误
    __HAL_UART_CLEAR_OREFLAG(huart);
  }
  // 当这个串口发生了错误，一定要在重新使能接收中断
  if (huart->Instance == USART1)
  {
    // 重置DMA和状态
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }
    //uart_busy = 0;
  }
  // 其他串口......
}

/* USER CODE END 1 */
