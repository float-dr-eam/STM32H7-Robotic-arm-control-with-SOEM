/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    eth.c
 * @brief   This file provides code for the configuration
 *          of the ETH instances.
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
#include "eth.h"

#if defined(__ICCARM__) /*!< IAR Compiler */

#pragma location = 0x30040000
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location = 0x30040060
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined(__CC_ARM) /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined(__GNUC__) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection"))); /* Ethernet Tx DMA Descriptors */

#endif

__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */
__attribute__((at(0x30044000))) uint8_t Tx_Buff[ETH_TX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */
/*
ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT * 2U];
uint8_t Rx_Buff[ETH_RX_DESC_CNT][1528];*/
ETH_TxPacketConfig TxConfig;

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "pcf8574.h"
#include "stm32h7xx_hal_eth.h"
#include "tim.h"
#include "lcd.h"

#define ETH_CHIP_SW_RESET_TO ((uint32_t)500U)
#define ETH_CHIP_MAX_DEV_ADDR ((uint32_t)31U) 
#define ETH_DMA_TRANSMIT_TIMEOUT (200U)    

#if PHY_AUTO_SELECT

#else
#define YT8512C_AND_RTL8201BL_PHYREGISTER2 0x0000
#define SR8201F_PHYREGISTER2               0x001C
#define LAN8720A_PHYREGISTER2              0x0007

int PHY_TYPE;
uint16_t ETH_CHIP_PHYSCSR;
uint16_t ETH_CHIP_SPEED_STATUS;
uint16_t ETH_CHIP_DUPLEX_STATUS;

uint8_t RecvLength=0;
uint32_t current_pbuf_idx =0;
/**
 * @brief  Configure the MPU attributes
 * @param  None
 * @retval None
 */
void NETMPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    HAL_MPU_Disable();
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x30040000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER5;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

#endif
/* USER CODE END 0 */

ETH_HandleTypeDef heth;

/* ETH init function */
void MX_ETH_Init(void)
{

    /* USER CODE BEGIN ETH_Init 0 */
    NETMPU_Config(); // MPU configuration

    /* USER CODE END ETH_Init 0 */

    static uint8_t MACAddr[6];

    /* USER CODE BEGIN ETH_Init 1 */

    /* USER CODE END ETH_Init 1 */
    heth.Instance = ETH;
    MACAddr[0] = 0x00;
    MACAddr[1] = 0x80;
    MACAddr[2] = 0xE1;
    MACAddr[3] = 0x00;
    MACAddr[4] = 0x00;
    MACAddr[5] = 0x00;
    heth.Init.MACAddr = &MACAddr[0];
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    heth.Init.TxDesc = DMATxDscrTab;
    heth.Init.RxDesc = DMARxDscrTab;
    heth.Init.RxBuffLen = ETH_MAX_PACKET_SIZE;//1528

    /* USER CODE BEGIN MACADDRESS */

    /* USER CODE END MACADDRESS */

    if (HAL_ETH_Init(&heth) != HAL_OK)
    {
        Error_Handler();
    }
    memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD; // Tx feature configuration
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;                 // Calculate IP header and payload checksum
    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;                                            // Insert CRC padding at end of packet
    /* USER CODE BEGIN ETH_Init 2 */
    uint32_t idx = 0;
    ETH_MACFilterConfigTypeDef filterDef; // ETH MAC filter configuration

    HAL_ETH_GetMACFilterConfig(&heth, &filterDef); // Get ETH MAC filter configuration
    filterDef.PromiscuousMode = ENABLE;            // Enable promiscuous mode
    HAL_ETH_SetMACFilterConfig(&heth, &filterDef); // Set ETH MAC filter configuration

    for (idx = 0; idx < ETH_RX_DESC_CNT; idx++)
    {
        HAL_ETH_DescAssignMemory(&heth, idx, Rx_Buff[idx], NULL);//Assign RX descriptor memory
    }
    HAL_ETH_SetMDIOClockRange(&heth); // Set MDIO clock range
    /* USER CODE END ETH_Init 2 */
}

void HAL_ETH_MspInit(ETH_HandleTypeDef *ethHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (ethHandle->Instance == ETH)
    {
        /* USER CODE BEGIN ETH_MspInit 0 */

        /* USER CODE END ETH_MspInit 0 */
        /* ETH clock enable */
        __HAL_RCC_ETH1MAC_CLK_ENABLE();
        __HAL_RCC_ETH1TX_CLK_ENABLE();
        __HAL_RCC_ETH1RX_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();
        /**ETH GPIO Configuration
        PC1     ------> ETH_MDC
        PA1     ------> ETH_REF_CLK
        PA2     ------> ETH_MDIO
        PA7     ------> ETH_CRS_DV
        PC4     ------> ETH_RXD0
        PC5     ------> ETH_RXD1
        PB11     ------> ETH_TX_EN
        PG13     ------> ETH_TXD0
        PG14     ------> ETH_TXD1
        */
        GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

        /* ETH interrupt Init */
        HAL_NVIC_SetPriority(ETH_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ETH_IRQn);
        /* USER CODE BEGIN ETH_MspInit 1 */
        PhyReset();
        /* USER CODE END ETH_MspInit 1 */
    }
}
void HAL_ETH_MspDeInit(ETH_HandleTypeDef *ethHandle)
{

    if (ethHandle->Instance == ETH)
    {
        /* USER CODE BEGIN ETH_MspDeInit 0 */

        /* USER CODE END ETH_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ETH1MAC_CLK_DISABLE();
        __HAL_RCC_ETH1TX_CLK_DISABLE();
        __HAL_RCC_ETH1RX_CLK_DISABLE();

        /**ETH GPIO Configuration
        PC1     ------> ETH_MDC
        PA1     ------> ETH_REF_CLK
        PA2     ------> ETH_MDIO
        PA7     ------> ETH_CRS_DV
        PC4     ------> ETH_RXD0
        PC5     ------> ETH_RXD1
        PB11     ------> ETH_TX_EN
        PG13     ------> ETH_TXD0
        PG14     ------> ETH_TXD1
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

        HAL_GPIO_DeInit(GPIOG, GPIO_PIN_13 | GPIO_PIN_14);

        /* ETH interrupt Deinit */
        HAL_NVIC_DisableIRQ(ETH_IRQn);
        /* USER CODE BEGIN ETH_MspDeInit 1 */

        /* USER CODE END ETH_MspDeInit 1 */
    }
}

uint32_t ethernet_read_phy(uint16_t reg)
{
    uint32_t regval;
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, reg, &regval);
    return regval;
}
void ethernet_write_phy(uint16_t reg, uint16_t value)
{
    uint32_t temp = value;
    HAL_ETH_WritePHYRegister(&heth, ETH_CHIP_ADDR, reg, temp);
}

/* Get Ethernet chip speed */
uint8_t ethernet_chip_get_speed(void)
{
    uint8_t speed;
    if (PHY_TYPE == LAN8720)
        speed = ~((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS));    
    else if (PHY_TYPE == SR8201F)
        speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 13);
    else if (PHY_TYPE == YT8512C)
        speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 14);
    else if (PHY_TYPE == RTL8201)
        speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 1); 
    return speed;
}

int32_t eth_chip_get_link_state(void)
{
    uint32_t readval = 0;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_PHYSCSR, &readval) != HAL_OK)
    {
        return ETH_CHIP_STATUS_READ_ERROR;
    }

    if (((readval & ETH_CHIP_SPEED_STATUS) != ETH_CHIP_SPEED_STATUS) && ((readval & ETH_CHIP_DUPLEX_STATUS) != 0))
    {
        return ETH_CHIP_STATUS_100MBITS_FULLDUPLEX;
    }
    else if (((readval & ETH_CHIP_SPEED_STATUS) != ETH_CHIP_SPEED_STATUS))
    {
        return ETH_CHIP_STATUS_100MBITS_HALFDUPLEX;
    }
    else if (((readval & ETH_CHIP_BCR_DUPLEX_MODE) != ETH_CHIP_BCR_DUPLEX_MODE))
    {
        return ETH_CHIP_STATUS_10MBITS_FULLDUPLEX;
    }
    else
    {
        return ETH_CHIP_STATUS_10MBITS_HALFDUPLEX;
    }
}
/**
 * @brief  Set the link state of the Ethernet chip.
 * @param  linkstate: The desired link state, can be one of the following:
 *         - ETH_CHIP_STATUS_100MBITS_FULLDUPLEX
 *         - ETH_CHIP_STATUS_100MBITS_HALFDUPLEX
 *         - ETH_CHIP_STATUS_10MBITS_FULLDUPLEX
 *         - ETH_CHIP_STATUS_10MBITS_HALFDUPLEX
 * @retval ETH_CHIP_STATUS_OK on success, or an error code on failure.
 */
int32_t eth_chip_set_link_state(uint32_t linkstate)
{
    uint32_t bcrvalue = 0;
    int32_t status = ETH_CHIP_STATUS_OK;

    if (HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BCR, &bcrvalue) == HAL_OK)
    {
        bcrvalue &= ~(ETH_CHIP_BCR_AUTONEGO_EN | ETH_CHIP_BCR_SPEED_SELECT | ETH_CHIP_BCR_DUPLEX_MODE);

        if (linkstate == ETH_CHIP_STATUS_100MBITS_FULLDUPLEX)
        {
            bcrvalue |= (ETH_CHIP_BCR_SPEED_SELECT | ETH_CHIP_BCR_DUPLEX_MODE);
        }
        else if (linkstate == ETH_CHIP_STATUS_100MBITS_HALFDUPLEX)
        {
            bcrvalue |= ETH_CHIP_BCR_SPEED_SELECT;
        }
        else if (linkstate == ETH_CHIP_STATUS_10MBITS_FULLDUPLEX)
        {
            bcrvalue |= ETH_CHIP_BCR_DUPLEX_MODE;
        }
        else
        {
            status = ETH_CHIP_STATUS_ERROR;
        }
    }
    else
    {
        status = ETH_CHIP_STATUS_READ_ERROR;
    }

    if (status == ETH_CHIP_STATUS_OK)
    {
        if (HAL_ETH_WritePHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BCR, bcrvalue) == HAL_OK)
        {
            status = ETH_CHIP_STATUS_WRITE_ERROR;
        }
    }

    return status;
}
void get_eth_linkstate(void)
{
    uint8_t eth_linkstate;
    do
    {
        eth_linkstate = ethernet_read_phy(ETH_CHIP_BSR);
        HAL_Delay(1000);

        if ((eth_linkstate & ETH_CHIP_BSR_LINK_STATUS) == 4)
        {
            printf("NET_LINE Success!!\r\n");
            lcd_show_string(200, 50, 200, 16, 16, "NET_LINE Success!!\n", BLACK);
        }
        else
        {
            printf("NET_LINE FAIL!\r\n");
            lcd_show_string(200, 50, 200, 16, 16, "NET_LINE FAIL!   \n", BLACK);
        }

    } while ((eth_linkstate & ETH_CHIP_BSR_LINK_STATUS) == 0);
}

/*  SR8201F     Register 2    0x001C
                Register 3    0xC016

    YT8512C     Register 2    0x0000
                Register 3    0x0128

    LAN8720A    Register 2    0x0007
                Register 3    0xC0F0

    RTL8201BL   Register 2    0x0000
                Register 3    0x8201 */
void PhyReset(void)
{
    uint32_t regvalue;
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, PHY_REGISTER2, &regvalue);
    PHY_TYPE = regvalue;
    printf("phy id is %d\n", PHY_TYPE);

if ((PHY_TYPE & 0xFFF) == 0xFFF) /*LAN8720A*/
    {
        pcf8574_write_bit(ETH_RESET_IO, 1); // Enable auto-negotiation
        osal_usleep(100000);                // Wait for PHY reset
        pcf8574_write_bit(ETH_RESET_IO, 0); // Release PHY reset

    }
    else /*YT8512C*/
    {
        pcf8574_write_bit(ETH_RESET_IO, 0); // Release PHY reset
        osal_usleep(100000);                // Wait for PHY reset
        pcf8574_write_bit(ETH_RESET_IO, 1); // Enable auto-negotiation
        osal_usleep(100000);
    }
    __set_PRIMASK(0); // Enable interrupts
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, PHY_REGISTER2, &regvalue);
    switch (regvalue)
    {
    case YT8512C_AND_RTL8201BL_PHYREGISTER2:
        HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, PHY_REGISTER3, &regvalue);
        if (regvalue == 0x128)
        {
            // Identified as YT8512C
            ETH_CHIP_PHYSCSR = ((uint16_t)0x11);
            ETH_CHIP_SPEED_STATUS = ((uint16_t)0x4010);
            ETH_CHIP_DUPLEX_STATUS = ((uint16_t)0x2000);
            PHY_TYPE = YT8512C;
        }
        else
        {
            // Identified as RTL8201
            ETH_CHIP_PHYSCSR = ((uint16_t)0x10);
            ETH_CHIP_SPEED_STATUS = ((uint16_t)0x0022);
            ETH_CHIP_DUPLEX_STATUS = ((uint16_t)0x0004);
            PHY_TYPE = RTL8201;
        }
        break;
    case SR8201F_PHYREGISTER2:
        // Identified as SR8201F
        ETH_CHIP_PHYSCSR = ((uint16_t)0x00);
        ETH_CHIP_SPEED_STATUS = ((uint16_t)0x2020);
        ETH_CHIP_DUPLEX_STATUS = ((uint16_t)0x0100);
        PHY_TYPE = SR8201F;
        break;
    case LAN8720A_PHYREGISTER2:
        // Identified as LAN8720A
        ETH_CHIP_PHYSCSR = ((uint16_t)0x1F);
        ETH_CHIP_SPEED_STATUS = ((uint16_t)0x0004);
        ETH_CHIP_DUPLEX_STATUS = ((uint16_t)0x0010);
        PHY_TYPE = LAN8720;
        break;
    }
}

int linkState;
int phyEvent;

void PhyEventHandler(void)
{
    uint32_t value;
    ETH_MACConfigTypeDef macConfig = {0};
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BSR, &value);
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BSR, &value);

    if ((value & ETH_CHIP_BSR_LINK_STATUS) != 0)
    {
        int32_t readval = 0;
        uint32_t duplex = 0;
        uint32_t speed = 0;

        readval = eth_chip_get_link_state();
        if (readval == ETH_CHIP_STATUS_READ_ERROR)
        {
            linkState = FALSE; 
            printf("Link state read error\n");
        }
        else
        {
            switch (readval)
            {
            case ETH_CHIP_STATUS_100MBITS_FULLDUPLEX:
                duplex = ETH_FULLDUPLEX_MODE;
                speed = ETH_SPEED_100M;
                printf("ETH_CHIP_STATUS_100MBITS_FULLDUPLEX\n");
                lcd_show_string(200, 210, 300, 16, 16, "ETH_CHIP_STATUS_100MBITS_FULLDUPLEX\n",BLUE);
                break;
            case ETH_CHIP_STATUS_100MBITS_HALFDUPLEX:
                duplex = ETH_HALFDUPLEX_MODE;
                speed = ETH_SPEED_100M;
                printf("ETH_CHIP_STATUS_100MBITS_HALFDUPLEX\n");
                break;
            case ETH_CHIP_STATUS_10MBITS_FULLDUPLEX:
                duplex = ETH_FULLDUPLEX_MODE;
                speed = ETH_SPEED_10M;
                printf("ETH_CHIP_STATUS_10MBITS_FULLDUPLEX\n");
                break;
            case ETH_CHIP_STATUS_10MBITS_HALFDUPLEX:
                duplex = ETH_HALFDUPLEX_MODE;
                speed = ETH_SPEED_10M;
                printf("ETH_CHIP_STATUS_10MBITS_HALFDUPLEX\n");
                break;
            default:
                duplex = ETH_FULLDUPLEX_MODE;
                speed = ETH_SPEED_100M;
                printf("default:ETH_CHIP_STATUS_100MBITS_FULLDUPLEX\n");
                break;
            }

            /* Release PHY reset */
            HAL_ETH_GetMACConfig(&heth, &macConfig);
            macConfig.DuplexMode = duplex;
            macConfig.Speed = speed;
            macConfig.TransmitQueueMode = ETH_TRANSMITTHRESHOLD_128; // ETH transmit queue mode
            HAL_ETH_SetMACConfig(&heth, &macConfig);
            linkState = TRUE; // Link established
            if (HAL_ETH_Start_IT(&heth) == HAL_OK)
                printf("HAL_ETH_Start_IT_SUCCESS\r\n");
            else
                printf("HAL_ETH_Start_IT_ERROR\r\n");

            HAL_ETH_BuildRxDescriptors(&heth);  // Release PHY reset
        }
    }
    else
    {
        linkState = FALSE; // Link down
    }
}

extern int dorun;

void PhyTick(void)
{
    uint32_t value;
    int link;

    // Read PHY status register
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BSR, &value);
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BSR, &value);

    // Parse PHY link status
    link = (value & ETH_CHIP_BSR_LINK_STATUS) ? TRUE : FALSE;

    //printf("PHY link = %d, linkState = %d\n", link, linkState);
    // Handle link status change
    if (link && !linkState)
    {
        phyEvent = TRUE; // Link established event
        dorun = 1;       // Run flag
        printf("Link up event\n");
    }
    else if (!link && linkState)
    {
        phyEvent = TRUE; // Link down event
        dorun = 0;       // Run flag
        printf("Link down event\n");
    }
}


uint32_t sendfinishflag=0;
void low_level_output(uint8_t *p,uint32_t length)
{  
    uint32_t framelen = 0;
    HAL_StatusTypeDef HalStatus;
    ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT]; // Transmit buffer

    memset(Txbuffer, 0, ETH_TX_DESC_CNT * sizeof(ETH_BufferTypeDef));
    Txbuffer[0].buffer = p;
    Txbuffer[0].len = length;
    framelen += length;
        
    TxConfig.Length = framelen;
    TxConfig.TxBuffer = Txbuffer;
    SCB_CleanInvalidateDCache();
    
    HalStatus = HAL_ETH_Transmit(&heth, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT);
    if (HalStatus != HAL_OK)
    {
        printf("HAL_ETH_TransmitFrame err %d\n", HalStatus);    
		lcd_show_string(200, 190, 200, 16, 16, "ETH_Send Error!  \n", BLACK);
    }
	else
	{
		lcd_show_string(200, 190, 200, 16, 16, "ETH_Send Success!\n", BLACK);
        sendfinishflag = 1;
	}

}

void low_level_input()
{
    
}
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
    low_level_input();
}
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
    sendfinishflag = 0;// Send complete flag
}

int bfin_EMAC_send (void *packet, int length)
{
    memcpy(&Tx_Buff[0][0],packet,length);
    low_level_output(Tx_Buff[0],length);
    return 0;
}

int bfin_EMAC_recv (uint8_t * packet, size_t size)
{
    ETH_BufferTypeDef RxBuff;
    uint32_t framelength = 0;
    SCB_CleanInvalidateDCache();
    HAL_StatusTypeDef status = HAL_ETH_GetRxDataBuffer(&heth, &RxBuff);
    
    if( status == HAL_OK) 
    {
        HAL_ETH_GetRxDataLength(&heth, &framelength);
        
        SCB_InvalidateDCache_by_Addr((uint32_t *)Rx_Buff, (ETH_RX_DESC_CNT*ETH_MAX_PACKET_SIZE));
        memcpy(packet, RxBuff.buffer, framelength);
        /* Invalidate data cache for ETH Rx Buffers */
        HAL_ETH_BuildRxDescriptors(&heth);
        return framelength;
    }
    return -1;
}

/* USER CODE END 1 */
