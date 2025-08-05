/**
 ****************************************************************************************************
 * @file        pcf8574.c
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

#include "pcf8574.h"
#include "tim.h"

/**
 * @brief       初始化PCF8574
 * @param       无
 * @retval      0, 成功;
                1, 失败;
 */
uint8_t pcf8574_init(void)
{
    uint8_t temp = 0;
    GPIO_InitTypeDef gpio_init_struct;
    PCF8574_GPIO_CLK_ENABLE();                            /* 使能GPIOB时钟 */

    gpio_init_struct.Pin = PCF8574_GPIO_PIN;              /* PB12 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;              /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                  /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;             /* 高速 */
    HAL_GPIO_Init(PCF8574_GPIO_PORT, &gpio_init_struct);  /* 初始化 */
    iic_init();                                           /* IIC初始化 */

    /* 检查PCF8574是否在位 */
    iic_start();
    iic_send_byte(PCF8574_ADDR);                          /* 写地址 */
    temp = iic_wait_ack();                                /* 等待应答,通过判断是否有ACK应答,来判断PCF8574的状态 */
    iic_stop();                                           /* 产生一个停止条件 */
    pcf8574_write_byte(0XFF);                             /* 默认情况下所有IO输出高电平 */
    return temp;
}

/**
 * @brief       读取PCF8574的8位IO值
 * @param       无
 * @retval      返回值:读到的数据
 */
uint8_t pcf8574_read_byte(void)
{ 
    uint8_t temp = 0;

    iic_start();
    iic_send_byte(PCF8574_ADDR | 0X01);                   /* 进入接收模式 */
    iic_wait_ack();
    temp = iic_read_byte(0);
    iic_stop();                                           /* 产生一个停止条件 */

    return temp;
}

/**
 * @brief       向PCF8574写入8位IO值
 * @param       data:要写入的数据
 * @retval      无
 */
void pcf8574_write_byte(uint8_t data)
{
    iic_start();  
    iic_send_byte(PCF8574_ADDR | 0X00);   /* 发送器件地址0X40,写数据 */
    iic_wait_ack();
    iic_send_byte(data);                  /* 发送字节 */
    iic_wait_ack();
    iic_stop();                           /* 产生一个停止条件  */
    Delay_us(10); 
}

/**
 * @brief       设置PCF8574某个IO的高低电平
 * @param       bit  : 要设置的IO编号,0~7
 * @param       sta  : IO的状态;0或1
 * @retval      无
 */
void pcf8574_write_bit(uint8_t bit, uint8_t sta)
{
    uint8_t data;

    data = pcf8574_read_byte();     /* 先读出原来的设置 */
    if (sta == 0)
    {
        data &= ~(1 << bit);
    }
    else
    {
        data |= 1 << bit;
    }
    pcf8574_write_byte(data);       /* 写入新的数据 */
}

/**
 * @brief       读取PCF8574的某个IO的值
 * @param       bit：要读取的IO编号,0~7
 * @retval      返回值:此IO的值,0或1
 */
uint8_t pcf8574_read_bit(uint8_t bit)
{
    uint8_t data;

    data = pcf8574_read_byte();     /* 先读取这个8位IO的值  */
    if (data & (1 << bit))
    {
        return 1;
    }
    else 
    {
        return 0;
    }
}  

