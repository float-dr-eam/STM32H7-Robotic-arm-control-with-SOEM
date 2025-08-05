/**
 ****************************************************************************************************
 * @file        lwip_comm.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-08-01
 * @brief       lwIP配置驱动
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
 * V1.0 20211202
 * 第一次发布
 *
 ****************************************************************************************************
 */
 
#ifndef _LWIP_COMM_H
#define _LWIP_COMM_H 
#include "./BSP/ETHERNET/ethernet.h"
#include "netif/etharp.h"
#include "lwip/timeouts.h"
#include "lwip/snmp.h"


/* DHCP进程状态 */
#define LWIP_DHCP_OFF                   (uint8_t) 0     /* DHCP服务器关闭状态 */
#define LWIP_DHCP_START                 (uint8_t) 1     /* DHCP服务器启动状态 */
#define LWIP_DHCP_WAIT_ADDRESS          (uint8_t) 2     /* DHCP服务器等待分配IP状态 */
#define LWIP_DHCP_ADDRESS_ASSIGNED      (uint8_t) 3     /* DHCP服务器地址已分配状态 */
#define LWIP_DHCP_TIMEOUT               (uint8_t) 4     /* DHCP服务器超时状态 */
#define LWIP_DHCP_LINK_DOWN             (uint8_t) 5     /* DHCP服务器链接失败状态 */

/* 链接状态 */
#define LWIP_LINK_OFF                   (uint8_t) 0     /* 链接关闭状态 */
#define LWIP_LINK_ON                    (uint8_t) 1     /* 链接开启状态 */
#define LWIP_LINK_AGAIN                 (uint8_t) 2     /* 重复开启 */

/* DHCP服务器最大重试次数 */
#define LWIP_MAX_DHCP_TRIES             (uint8_t) 4

typedef void (*display_fn)(uint8_t index);

/*lwip控制结构体*/
typedef struct  
{
    uint8_t mac[6];                                     /* MAC地址 */
    uint8_t remoteip[4];                                /* 远端主机IP地址 */ 
    uint8_t ip[4];                                      /* 本机IP地址 */
    uint8_t netmask[4];                                 /* 子网掩码 */
    uint8_t gateway[4];                                 /* 默认网关的IP地址 */
    uint8_t dhcpstatus;                                 /* dhcp状态
                                                        0, 未获取DHCP地址
                                                        1, 进入DHCP获取状态
                                                        2, 成功获取DHCP地址
                                                        0XFF,获取失败 */
    uint8_t link_status;                                /* 连接状态 */
    display_fn lwip_display_fn;                         /* 显示函数指针 */
}__lwip_dev;

extern __lwip_dev g_lwipdev;                            /* lwip控制结构体 */

void    lwip_comm_default_ip_set(__lwip_dev *lwipx);    /* lwip 默认IP设置 */
uint8_t lwip_comm_init(void);                           /* LWIP初始化(LWIP启动的时候使用) */
#endif
