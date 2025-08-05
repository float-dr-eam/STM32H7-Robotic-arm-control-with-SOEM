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

#define ETH_CHIP_SW_RESET_TO ((uint32_t)500U) /* 闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶姊洪崹顕呭剰缂佸墎鍋熼埀顒€绠嶉崕杈┾偓姘煎枤缁梻鈧潧鎽滅壕鍏肩箾閹寸儑渚涙俊鎻掓憸缁辨帡宕滄担鍛婄亪閻庢鍣崳锝呯暦婵傜ǹ鍗抽柣鏇炲€婚崥锟� */
#define ETH_CHIP_INIT_TO ((uint32_t)2000U)    /* 闂傚倷绀侀幉锛勬暜濡ゅ啯宕查柟鐗堟緲缁狀垶鏌ㄩ悤鍌涘?闂傚倷绀侀幉锟犳偋濡ゅ懏鍋嬫繝濠傜墛閸嬨倝鏌熺€涙绠ラ柛銈嗩殜閺屾稑鈽夐崡鐐寸亶濡炪伅浣告处閻撱儵鏌ｉ弴鐐测偓鍦偓姘炬嫹? */
#define ETH_CHIP_MAX_DEV_ADDR ((uint32_t)31U) /* PHY闂傚倷绶氬濠氭⒔閸曨偒鐔嗘俊顖欒閻掍粙鏌涢幇闈涙灍闁稿骸绉归弻娑㈠即閵娿儱顫繛瀵稿У閿氭い顓炴健瀹曞ジ宕卞Δ鍐冿綁鎮楀鐐? */
#define ETH_DMA_TRANSMIT_TIMEOUT (200U)     /* DMA婵犵數鍋熼ˉ鎰板磻閹邦厽鍙忛柛鎾楀嫷鍋ㄥ銈嗘尵婵敻骞嗛妷鈺傜厱婵炴垵宕弸鐔搞亜锜婚崶銊у幐闂佸憡鍔戦崝搴ㄥ春閿濆棔绻嗘い鎰靛墯鐎氾拷 */

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
    NETMPU_Config(); // MPU婵犵數鍎戠徊钘壝洪敂鐐床闁告劏鏅滈崣蹇涙煏韫囧鐏┑顖氥偢閺屾盯骞樺Δ鈧幊蹇涙倵閿燂拷

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
    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD; // Tx闂傚倷娴囧銊╂嚄閼稿灚娅犳俊銈傚亾闁伙絽鐏氱粭鐔煎焵椤掆偓閻ｇ兘鎮╃拠鎻掔獩濡炪倖鐗楅妵娑㈠磻閹捐鍐€闁靛ě灞剧カ闂佽崵濮撮幖顐﹀箹椤愶讣缍栭柣妯肩帛閻撱儵鏌ｉ弴鐐测偓鍦偓姘炬嫹?
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;                 // 闂傚倷绀佸﹢杈ㄧ仚濠电偛鐪伴崐婵嬪箖閻愵兙鍋呴柛鎰╁妼閸炪劑姊虹捄銊ユ灁濠殿喖鍢查悾鐑芥晲婢跺鍘卞┑鐘诧工閸燁垰锕㈤幍顔藉枑闁哄鐏濋弳锝団偓瑙勬礃椤ㄥ﹥淇婇悜鑺ユ櫇闁逞屽墯缁傛帒螣婵傝棄缍婇幃鈺傜瑹椤栨碍娅嶇紓鍌欑瑜板宕￠幎钘夋瀬闁瑰墽绮弲鎼佹煥閻曞倹瀚�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘 IP 闂傚倷绀侀幖顐ょ矓閺夋嚚娲Χ婢跺á锕傛煛閸愩劎澧曢柛鎰ㄥ亾闂備浇娉曢崰鎾存叏瀹勬壆鏆﹂柨婵嗩槹閻撴洘绻濇繝鍌氭殺閻庢矮鍗抽弻鐔兼憥閸屾艾绁悗娈垮枛閻忔艾顕ラ崟顓涘亾閿濆骸浜濋柣婵撶節閺岋綁鎮╅崘鎻掝潕闂佸摜濮靛ú鏍箒闂佹悶鍎洪崜娆撳礃閳ь剟姊虹捄銊ユ灁濠殿喖鍢查悾鐑芥晲婢跺鍘卞┑鐘诧工閸熶即鎮為崸妤佺厵闁告垯鍊栫€氾拷?缂傚倸鍊烽懗鑸垫叏閻㈡悶鈧啴宕ㄩ弶鎴狀唵婵炴潙鍚嬪娆戠不閻㈠憡鐓欓柣鎴灻悘銉р偓娈垮櫙閹凤拷
    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;                                            // 闂傚倷绀佸﹢杈ㄧ仚濠电偛鐪伴崐婵嬪箖閸撗呯婵烇絻鈧倝姊绘担鍛婂暈缂侇喖鐭傞、妤呫€呴崲楣冩⒒娴ｅ憡璐＄紒顕呭灣閺侇噣骞掑Δ鈧惌妤呮煕閳╁啰鈽夋俊鐐扮矙閹﹢鎮欓棃娑楀缂備礁鍊搁惌鍌炲蓟閻旇偐宓侀柛顭戝枤娴犻箖鏌ｉ姀鈺佺仩闁绘牕銈稿顐﹀箻缂佹ɑ娅㈤梺璺ㄥ櫐閹凤拷?

    /* USER CODE BEGIN ETH_Init 2 */
    uint32_t idx = 0;
    ETH_MACFilterConfigTypeDef filterDef; // ETH MAC濠电姷鏁告慨瀛樼仚闂佺ǹ锕ラ悧婊勭缁嬪簱鏀介悗锝庝邯濡兘姊洪棃娴ゆ盯宕ㄩ鑲╂闂傚倷绀侀幖顐︻敄閸涱垪鍋撳鐓庡⒋闁诡喚鍋撻妶锝夊礃閳哄倹顏熼梻浣芥硶閸ｏ箓骞忛敓锟�?

    HAL_ETH_GetMACFilterConfig(&heth, &filterDef); // 闂傚倷绀侀崥瀣磿閹惰棄搴婇柤鑹扮堪娴滃綊鏌ㄩ悤鍌涘 ETH MAC 闂備礁鎼ˇ顐﹀疾濞戞◤娲晝閸屾氨顔呴梺闈涚墕椤︻垶姊婚鐐寸厪濠电偟鍋撳▍鍛存偨椤栨稑鈻曢柡灞诲姂閹倝宕掑☉姗嗕紦?
    filterDef.PromiscuousMode = ENABLE;            // 闂備礁鎼ˇ顐﹀疾濞戞◤娲晝閸屾氨顔呴梺闈涚墕椤︻垶姊婚鐐寸厪濠电偛鐏濋埀顒佹礋閸┿垼绠涘☉娆戝幘閻庤娲栧ú銊╁Χ閻ф洘绻濈喊妯活潑闁割煈浜崺鈧い鎺嗗亾缁剧虎鍘惧☉鐢稿焵閿燂拷
    //filterDef.ReceiveAllMode = ENABLE;             // 闂備礁鎼ˇ顐﹀疾濞戞◤娲晝閸屾氨顔呴梺闈涚墕椤︻垶姊婚鐐寸厪濠电偛鐏濋埀顒佹礋閸┿垼绠涘☉娆戝幘闁诲骸婀辨慨纾嬵暱闂佹眹鍩勯崹閬嶅箰閸愬樊鍤曢柕濞炬櫆閸ゅ姊婚崼鐔剁繁闁哄棴绻濆娲传閸曨厼顣哄銈忛檮婵粙宕鐐粹拺閻犳亽鍔屽▍鎰版煙閸戙倖瀚�?
    HAL_ETH_SetMACFilterConfig(&heth, &filterDef); // 闂備浇宕垫慨宕囩矆娴ｈ娅犲ù鐘差儐閸嬵亪鏌涘⿰鍛簷H MAC 闂備礁鎼ˇ顐﹀疾濞戞◤娲晝閸屾氨顔呴梺闈涚墕椤︿即寮查幖浣圭叆闁绘洖鍊圭€氾拷?
    for (idx = 0; idx < ETH_RX_DESC_CNT; idx++)
    {
        HAL_ETH_DescAssignMemory(&heth, idx, Rx_Buff[idx], NULL);//闂備浇顕х换鎰崲閹邦儵娑樜旈崨顓″煘闁诲海鏁哥涵鍫曞磻閹炬枼妲堟繛鍡橆焽閸旈绱撴担鐟版暰缂佺姵鎸搁悾宄邦潨閳ь剟鐛幘璇茬闁硅揪闄勯鍐⒒娴ｅ憡鍟為悽顖涘笒铻炴繝闈涱儏閸屻劑鏌曢崼婵囧窛缁炬儳鍚嬮幈銊ノ旈崘銊︻唨A Rx闂傚倷鑳堕、濠囶敋瑜忛幑銏犖旈崨顓㈠敹濡炪倕绻愰悧濠囧疾閹间焦鐓ラ柣鏇炲€圭€氾拷?
    }
    HAL_ETH_SetMDIOClockRange(&heth); // 闂傚倸鍊烽悞锕€顭垮Ο鑲╃煋闁割偅娲橀崑顏堟煕濠婂懌浜橦 MDIO闂傚倷娴囬～澶嬬娴犲纾块弶鍫亖娴滆绻涢幋娆忕仾闁稿骸绉归弻娑㈠即閵娿儱顫銇礁娲﹂埛鎴︽煟閿濆懓瀚伴柡瀣懇閹宕归銈呪拰闂佽桨鐒﹂幑鍥极閹剧粯鏅搁柨鐕傛嫹?
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

/**
 * @breif       闂傚倷绀侀崥瀣磿閹惰棄搴婇柤纰卞墯椤愪粙寮堕崼姘珖缁鹃箖绠栭弻鐔衡偓鐢登瑰暩闂佺顫夊ú鐔煎蓟閵娾晜鍋嗛柛灞剧☉椤忥拷?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柛搴＄Ч閺屾盯寮撮妸銉т粴闂佺ǹ顑嗛幑鍥箖閵忕姷鏆嬮梺顓ㄧ畱閺佽姤绻濈喊妯活潑闁割煈浜崺鈧い鎺嗗亾缁剧虎鍘惧☉鐢稿焵閿燂拷
 * @param       闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?
 * @retval      1:100M
                0:10M
 */
uint8_t ethernet_chip_get_speed(void)
{
    uint8_t speed;
    if (PHY_TYPE == LAN8720)
        speed = ~((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS));     /* 婵犵數鍋涢顓熸叏閹绢喗濯兼い銈呮А8720闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?31闂傚倷绀侀幉锟犳偡閵夆晛搴婇柡灞诲劜閸庡﹪骞栨潏鍓у矝闁稿鎸搁埥澶娾枍椤撗傜盎闁宠绉归獮鎺懳旀担瑙勵仧闂備浇娉曢崳锕傚箯閿燂拷??闂傚倷绀侀幉锟犳偡閿曞倹鍋嬫繝濠傛噽缁€濠傤熆鐠鸿櫣鐏辩痪鎯у悑缁绘繃绻濋崒婊冾暤闂佺ǹ顑嗛幑鍥箖閵忕姷鏆嬮梺顓ㄧ畱閺佷粙姊绘担鍛婂暈缂侇喖鐭傞幊鐔碱敍濮樼厧娈ㄩ柣鐘叉搐濡﹪宕崨瀛樼厪濠㈠厜鏅濈换婵堟濮樿泛鏋侀柟鍓х帛閺呮悂鏌ㄩ悤鍌涘? */
    else if (PHY_TYPE == SR8201F)
        speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 13); /* 婵犵數鍋涢顓熸叏閺夋嚚鍝勨攽閿燂拷8201F闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?0闂傚倷绀侀幉锟犳偡閵夆晛搴婇柡灞诲劜閸庡﹪骞栨潏鍓у矝闁稿鎸搁埥澶娾枍椤撗傜盎闁宠绉归獮鎺懳旀担瑙勵仧闂備浇娉曢崳锕傚箯閿燂拷??闂傚倷绀侀幉锟犳偡閿曞倹鍋嬫繝濠傛噽缁€濠傤熆鐠鸿櫣鐏辩痪鎯у悑缁绘繃绻濋崒婊冾暤闂佺ǹ顑嗛幑鍥箖閵忕姷鏆嬮梺顓ㄧ畱閺佷粙姊绘担鍛婂暈缂侇喖鐭傞幊鐔碱敍濮樼厧娈ㄩ柣鐘叉搐濡﹪宕崨瀛樼厪濠㈠厜鏅濈换婵堟濮樿泛鏋侀柟鍓х帛閺呮悂鏌ㄩ悤鍌涘? */
    else if (PHY_TYPE == YT8512C)
        speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 14); /* 婵犵數鍋涢顓熸叏閹绢喗鍎夋俊銈忔嫹8512C闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?17闂傚倷绀侀幉锟犳偡閵夆晛搴婇柡灞诲劜閸庡﹪骞栨潏鍓у矝闁稿鎸搁埥澶娾枍椤撗傜盎闁宠绉归獮鎺懳旀担瑙勵仧闂備浇娉曢崳锕傚箯閿燂拷??闂傚倷绀侀幉锟犳偡閿曞倹鍋嬫繝濠傛噽缁€濠傤熆鐠鸿櫣鐏辩痪鎯у悑缁绘繃绻濋崒婊冾暤闂佺ǹ顑嗛幑鍥箖閵忕姷鏆嬮梺顓ㄧ畱閺佷粙姊绘担鍛婂暈缂侇喖鐭傞幊鐔碱敍濮樼厧娈ㄩ柣鐘叉搐濡﹪宕崨瀛樼厪濠㈠厜鏅濈换婵堟濮樿泛鏋侀柟鍓х帛閺呮悂鏌ㄩ悤鍌涘? */
    else if (PHY_TYPE == RTL8201)
        speed = ((ethernet_read_phy(ETH_CHIP_PHYSCSR) & ETH_CHIP_SPEED_STATUS) >> 1); /* 婵犵數鍋涢顓熸叏閹绢喖绀冪紒顐ｃ偛8201闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?16闂傚倷绀侀幉锟犳偡閵夆晛搴婇柡灞诲劜閸庡﹪骞栨潏鍓у矝闁稿鎸搁埥澶娾枍椤撗傜盎闁宠绉归獮鎺懳旀担瑙勵仧闂備浇娉曢崳锕傚箯閿燂拷??闂傚倷绀侀幉锟犳偡閿曞倹鍋嬫繝濠傛噽缁€濠傤熆鐠鸿櫣鐏辩痪鎯у悑缁绘繃绻濋崒婊冾暤闂佺ǹ顑嗛幑鍥箖閵忕姷鏆嬮梺顓ㄧ畱閺佷粙姊绘担鍛婂暈缂侇喖鐭傞幊鐔碱敍濮樼厧娈ㄩ柣鐘叉搐濡﹪宕崨瀛樼厪濠㈠厜鏅濈换婵堟濮樿泛鏋侀柟鍓х帛閺呮悂鏌ㄩ悤鍌涘? */
    return speed;
}

int32_t eth_chip_get_link_state(void)
{
    uint32_t readval = 0;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_PHYSCSR, &readval) != HAL_OK)
    {
        return ETH_CHIP_STATUS_READ_ERROR;
    }

    // 闂傚倷绀侀幖顐ょ矓閻戞枻缍栧璺猴功閺嗐倕霉閿濆洤鍔嬮悗姘哺閹綊宕堕妸銉хシ缂備焦褰冨锟犲蓟濞戙垹绠抽柟鐐墯濡棗鈹戦悙鑼闁搞劌娼￠獮鍐煛閸涱喖娈濈紒鍓у閿氬ù婊勫劤闇夐柨婵嗘瑜版帗鍊堕柡灞诲劜閻撱儵鏌ｉ弴鐐测偓鍦偓姘炬嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺屻劑鎮ら崒娑橆伓?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺屻劑鎮ら崒娑橆伓?
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
  * @brief       闂備浇宕垫慨宕囩矆娴ｈ娅犲ù鐘差儐閸嬵亪鏌涘⿰鍛簷H_CHIP闂備浇宕垫慨鎶芥倿閿曞倸绠繝闈涱儏缁狀垶鏌ㄩ悤鍌涘?闂傚倷鐒﹂惇褰掑礉瀹€鈧埀顒佺煯閸楁娊鐛Δ浣洪檮闁告稑锕ュ▍銏ゆ⒑鐠恒劌娅愰柟鍑ゆ嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺屻劑鎮ら崒娑橆伓?
  * @param       pobj: 闂備浇宕垫慨鎶芥倿閿曞倸绠繝闈涱儏缁狀垶鏌ㄩ悤鍌涘?闂備浇顕уù鐑藉极閹间降鈧焦绻濋崶銊ョ樁闂佽法鍣﹂幏锟�
  * @param       pLinkState: 闂傚倷绀佸﹢杈ㄧ仚濠电偛鐪伴崐婵嬬嵁閸愩劉鍫柛顐ゅ枔閸欏嫰姊虹涵鍜佹綈闁告梹顨婇幃銏ゅ川鐎涙鍘藉┑鈽嗗灠閻忔繈鎮￠幇鐗堢厓鐟滄粓宕楀☉銏″殑閻犻缚銆€閺嬫棃鏌熸潏鍓х暠缂佺姳鍗抽弻娑㈠Ψ椤旂厧顫柣搴㈢啲閹凤拷
  * @retval      ETH_CHIP_STATUS_OK闂傚倷鐒︾€笛呯矙閹烘鍋勬い鎺戝缁狀垶鏌ㄩ悤鍌涘?闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺屻劑鎮ら崒娑橆伓?
                 ETH_CHIP_STATUS_ERROR 闂傚倷鐒︾€笛呯矙閹烘鍋勬い鎺戝缁狀垶鏌ㄩ悤鍌涘?闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺屻劑鎮ら崒娑橆伓?
                 ETH_CHIP_STATUS_READ_ERROR闂傚倷鐒︾€笛呯矙閹烘鍎楁い鏂垮⒔缁犳柨顭块懜闈涘闁搞倕瀚伴弻锟犲磼濡　鍋撻弽顐熷亾濮樼偓瀚�?闂傚倷绀侀幉锟犳偡閿曞倹鍋嬮柡鍥ュ灪閸庡﹪骞栨潏鍓у矝闁稿鎸搁埥澶娾枍椤撗傜盎闁宠绉归弫鎾绘晸閿燂拷
                 ETH_CHIP_STATUS_WRITE_ERROR 闂傚倷鐒︾€笛呯矙閹烘鍎楁い鏂垮⒔缁犳柨顭块懜闈涘闁搞倕瀚伴弻锕€螣娓氼垱笑闂佸搫鎳忛幃鍌炲蓟濞戞粎鐤€閻庯綆浜滄慨搴ㄦ⒑娴兼瑧鐣辨繛瀵稿厴閸┾偓妞ゆ帊鑳堕埊鏇㈡煥濮橆厾绠鹃柛婊冨暞鐎氾拷
  */
int32_t eth_chip_set_link_state(uint32_t linkstate)
{
    uint32_t bcrvalue = 0;
    int32_t status = ETH_CHIP_STATUS_OK;

    if (HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BCR, &bcrvalue) == HAL_OK)
    {
        /* 缂傚倸鍊风粈渚€宕愰悜鑺ュ殑闁割偅娲栭弸渚€鏌曢崼婵愭Ч闁绘搫缍侀弻鐔烘喆閸曨偄顫庨梺缁樻煥閹虫ê顫忓ú顏勭骇閻犳亽鍓遍妶澶嬬厽闁靛牆绻戠€氾拷(闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝嗩棄缁绢厸鍋撻梻濠庡亜濞诧箓骞愮拠娴嬫瀺鐎光偓閸曨剛鍙嗗┑鐐村灦椤洭鎮為幖浣圭厓鐟滄粓宕滃☉銏犵獥閹艰揪绲洪崑鎾舵喆閸曨偀鏋欓悗瑙勬处娴滅偞绂掗敃鍌涘癄濠㈣泛瀛╅幉浼存⒒娴ｈ姤纭堕柛锝忕畵楠炲繘鏁撻敓锟�?) */
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
            /* 闂傚倸鍊烽悞锔锯偓绗涘洦鍋￠梺顒€绉寸粻顖炴煥閻曞倹瀚�?闂傚倷鐒﹂惇褰掑礉瀹€鈧埀顒佺煯閸楁娊鐛Δ浣洪檮闁告稑锕ュ▍銏ゆ⒑鐠恒劌娅愰柟鍑ゆ嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝嗩棄缂佺姵鍨甸湁闁挎繂姣ラ悷鎷旀稑顫滈埀顒勫蓟閵娾晜鍋嗛柛灞剧☉椤忥拷? */
            status = ETH_CHIP_STATUS_ERROR;
        }
    }
    else
    {
        status = ETH_CHIP_STATUS_READ_ERROR;
    }

    if (status == ETH_CHIP_STATUS_OK)
    {
        /* 闂傚倷绀侀幉锟犲礉閺嶎厽鍋￠柕澶嗘櫅閻鏌涢埄鍐姇闁绘搫缍侀弻鐔烘喆閸曨偄顫庨梺缁樻煥閹虫﹢寮婚敐鍛傜喎鈻庨幆褎娅嶉梻浣烘嚀閸ゆ牠骞忛敓锟�? */
        if (HAL_ETH_WritePHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BCR, bcrvalue) == HAL_OK)
        {
            status = ETH_CHIP_STATUS_WRITE_ERROR;
        }
    }

    return status;
}
void get_eth_linkstate(void)
{
    uint8_t eth_linkstate; /* 缂傚倸鍊搁崐鎼佸疮椤栫偛鍨傜憸鐗堝笒閸氬綊骞栧ǎ顒€鐏ù婧垮€濋弻锝夊箛椤撶喓绋囨繝銏ｆ硾缁夊綊寮婚敐鍛傜喖宕归鎯у缚闂備線鈧偛鑻崢鎼佹煟閹虹偟鐣甸柟顔哄劦閺佹劙宕奸姀銏℃緫婵犵绱曢崑娑欐櫠閻ｅ瞼鐭撻柨鐕傛嫹 */
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
    // 闂備浇宕垫慨鏉懨洪埡鍜佹晪鐟滄垿濡甸幇鏉垮簥婵犲﹦顕為梻浣筋嚙闁帮絽顭囪閳ь剚鍑归崜鐔煎箖濞差亜惟闁冲搫鍊瑰▍銏ゆ⒑鐠恒劌娅愰柟鍑ゆ嫹?2闂傚倷鐒﹂惇褰掑礉瀹€鈧埀顒佸嚬閸撶喖宕洪埀顒併亜閹烘垵鈧爼鍩€椤掆偓椤戝洤危閹邦兘鏀介柛銉㈡櫓濞村嫰姊洪棃娑辨Ф闁告柨鐬肩划濠氼敍閻愬鍘卞┑鐐叉閸旀牠鎳撶缓鍍� ID
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, PHY_REGISTER2, &regvalue);
    PHY_TYPE = regvalue;
    printf("phy id is %d\n", PHY_TYPE);

    /*ethernet_write_phy(ETH_CHIP_BCR, ETH_CHIP_BCR_SOFT_RESET); // 闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶姊洪崹顕呭剰缂佸墎鍋熼埀顒€绠嶉崕杈┾偓姘煎枤缁棃鏁撻敓锟� 闂傚倷绀侀幉锛勬暜閸ヮ剙纾归柡宥庡幖閽冪喖鏌涢妷銏℃澒闁稿鎹囧Λ鍐ㄢ槈濮楀棔鎮ｉ梻浣割吔閺夊灝顫囬悗瑙勬礃婵炲﹪鍨鹃敃鍌氱闁圭偓鍓氭导锟�?闂傚倷绀侀幉锟犲礉閺嶎厽鍋￠柨鏇楀亾閸楅亶鏌熼梻瀵割槮閻熸瑱绠撻弻銊╁棘閸喗鍊柣搴㈢啲閹凤拷?闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?

    regvalue = ethernet_read_phy(ETH_CHIP_BCR);
    while (regvalue & ETH_CHIP_BCR_SOFT_RESET)
    {
        regvalue = ethernet_read_phy(ETH_CHIP_BCR);
        HAL_Delay(10);
        timeout++;
        if (timeout >= 500)
        {
            printf("soft_reset_time_out");
            break; // 闂備胶鍎甸崜婵堟暜閹烘绠犻煫鍥ㄦ惄濞撳鏌涚仦鍓х煁鐎规洖顦甸弻娑㈠箛椤撶偟绁峰┑顕嗙秶閹凤拷,5S
        }
    }
    // 闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶姊洪崹顕呭剰缂佸墎鍋熼埀顒€绠嶉崕杈┾偓姘煎枤缁棃鏁撻敓锟�
    __set_PRIMASK(1); // 闂傚倷鑳舵灙缂佺粯鍨剁换娑欑節閸嬭姤鐩獮姗€顢欓懖鈺婂悑闂備線鈧偛鑻晶瀵糕偓娈垮枛閻忔艾顕ラ崟顒€绶為幖绮光偓宕囩暠闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶姊洪崹顕呭剰缂佸墎鍋熼埀顒€绠嶉崕杈┾偓姘煎枤缁棃骞橀鑲╅獓闂佸疇妗ㄩ悞锕€顔忓┑瀣厓閻犲洦褰冮幃鎴犵磼鐎ｎ亶妲告い鎾炽偢瀹曠厧鈹戦崱妤冾吅闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝呅撻柡鍛倐閺屻劑鎮ら崒娑橆伓?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘
    // 闂傚倷绀侀幉锛勬暜閸ヮ剙纾归柡宥庡幖閽冪喖鏌涢姀锛勭处Y缂傚倸鍊风欢锟犲磻婢舵劦鏁嬬憸鏃堝箖濡ゅ懏鍊婚柤鎭掑劜濞呮牠鏌ｈ箛鏇炰粶闁逞屽墰閸犲酣鎮甸鐐寸厸濠㈣泛妫欏▍鍡涙煕閹邦剦鐓兼鐐茬箻閺佹捇鏁撻敓锟�?闂傚倷鑳堕崕鐢稿疾濠靛洨浠氱紓鍌欓檷閸斿秹鎮￠敓鐘茬畾闁告劦鍠栫粈瀣煠閹间焦娑фい蹇ユ嫹?婵犵數鍋犻幓顏嗗緤閻ｅ瞼鐭撻柣銏⑶圭粻褰掓煟閹达絽袚闁哄懏鎮傞弻銊╂偆閸屾稑顏�?*/
    if ((PHY_TYPE & 0xFFF) == 0xFFF) /*LAN8720A*/
    {
        pcf8574_write_bit(ETH_RESET_IO, 1); // 闂備浇宕垫慨宕囩矆娴ｈ娅犲ù鐘差儐閸嬵亪鏌涢埄鍏狀亞娆㈤悙纰樺亾楠炲灝鍔氶悗姘煎枤缁梻鈧數纭堕崑鎾斥枔閸喗鐏嶅┑鐐插级缁诲牆鐣峰⿰鍫濈闁兼祴鏂侀弸鏍р攽閻愬弶鈻曞ù婊呭仩閵囨劙顢涢悙瀵稿幗闂侀潧绻堥崐鏍р槈瑜斿娲棘閵堝棗顏�
        osal_usleep(100000);                // 闂佽娴烽崑锝夊磹濞戙垹鏄ラ柡宓本瀵岄梺璺ㄥ櫐閹凤拷10闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹??
        pcf8574_write_bit(ETH_RESET_IO, 0); // 闂備浇宕垫慨宕囩矆娴ｈ娅犲ù鐘差儐閸嬵亪鏌涢埄鍏狀亞娆㈤悙纰樺亾楠炲灝鍔氶悗姘煎枤缁梻鈧數纭堕崑鎾斥枔閸喗鐏嶅┑鐐插级缁诲牆鐣峰⿰鍫濈闁兼祴鏂侀弸鏍煛婢跺苯浠﹀鐟版缁棃鎮介崨濠勫幗闂侀潧绻堥崐鏍р槈瑜斿娲棘閵堝棗顏�
        osal_usleep(100000);                // 闂傚倷绀侀幉锟犲礉閺囩姷鐭欓柛鏇ㄥ灠缁狀垶鏌ㄩ悤鍌涘?闂佽娴烽崑锝夊磹濞戙垹鏄ラ柡宓本瀵岄梺璺ㄥ櫐閹凤拷10闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹??
    }
    else /*YT8512C*/
    {
        pcf8574_write_bit(ETH_RESET_IO, 0); // 闂備浇宕垫慨宕囩矆娴ｈ娅犲ù鐘差儐閸嬵亪鏌涢埄鍏狀亞娆㈤悙纰樺亾楠炲灝鍔氶悗姘煎枤缁梻鈧數纭堕崑鎾斥枔閸喗鐏嶅┑鐐插级缁诲牆鐣峰⿰鍫濈闁兼祴鏂侀弸鏍煛婢跺苯浠﹀鐟版缁棃鎮介崨濠勫幗闂侀潧绻堥崐鏍р槈瑜斿娲棘閵堝棗顏�
        osal_usleep(100000);                // 闂佽娴烽崑锝夊磹濞戙垹鏄ラ柡宓本瀵岄梺璺ㄥ櫐閹凤拷10闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹??
        pcf8574_write_bit(ETH_RESET_IO, 1); // 闂備浇宕垫慨宕囩矆娴ｈ娅犲ù鐘差儐閸嬵亪鏌涢埄鍏狀亞娆㈤悙纰樺亾楠炲灝鍔氶悗姘煎枤缁梻鈧數纭堕崑鎾斥枔閸喗鐏嶅┑鐐插级缁诲牆鐣峰⿰鍫濈闁兼祴鏂侀弸鏍р攽閻愬弶鈻曞ù婊呭仩閵囨劙顢涢悙瀵稿幗闂侀潧绻堥崐鏍р槈瑜斿娲棘閵堝棗顏�
        osal_usleep(100000);                // 闂傚倷绀侀幉锟犲礉閺囩姷鐭欓柛鏇ㄥ灠缁狀垶鏌ㄩ悤鍌涘?闂佽娴烽崑锝夊磹濞戙垹鏄ラ柡宓本瀵岄梺璺ㄥ櫐閹凤拷10闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹??
    }
    __set_PRIMASK(0); // 闂佽瀛╅鏍窗閹烘纾婚柟鍓х帛閻撱儵鏌ｉ弴鐐测偓鍦偓姘炬嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝嗩棄閻庢艾顦…璺ㄦ崉娓氼垱歇闂佸憡锕╅崜鐔煎蓟閵娾晜鍋嗛柛灞剧☉椤忥拷?
    // 闂傚倷绀侀幉锟犲礉閺囩姷鐭欓柛鏇ㄥ灠缁狀垶鏌ㄩ悤鍌涘?闂備浇宕垫慨鏉懨洪埡鍜佹晪鐟滄垿濡甸幇鏉垮簥婵犲﹦顕為梻浣筋嚙闁帮絽顭囪閳ь剚鍑归崜鐔煎箖濞差亜惟闁冲搫鍊瑰▍銏ゆ⒑鐠恒劌娅愰柟鍑ゆ嫹?2闂傚倷鐒﹂惇褰掑礉瀹€鈧埀顒佸嚬閸撶喖宕洪埀顒併亜閹烘垵鈧爼鍩€椤掆偓椤戝洤危閹邦兘鏀介柛銉㈡櫓濞村嫰姊洪棃娑崇础闁逞屽墴閸┾偓妞ゆ帊绶￠悞浠嬫煃瑜滈崜婵嬶綖婢舵劖鍤屽Δ锝呭暙閻鏌熺粙鍨劉濠殿垱鎸抽弻鐔煎箲閹邦剛鍘梺浼欑悼濞差搸Y闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垰顪冪€ｎ亝鎹ｉ悘蹇曟暬閹綊宕堕鍕闂侀€炲苯澧鹃柟鍑ゆ嫹
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, PHY_REGISTER2, &regvalue);
    switch (regvalue)
    {
    case YT8512C_AND_RTL8201BL_PHYREGISTER2:
        HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, PHY_REGISTER3, &regvalue);
        if (regvalue == 0x128)
        {
            // 闂傚倸鍊烽悞锕€顭垮Ο鑲╃煋闁割偅娲橀崑顏堟煕閳╁喚鐒介柍缁樻⒒缁辨帗骞婄憸甯嫹8512C
            ETH_CHIP_PHYSCSR = ((uint16_t)0x11);
            ETH_CHIP_SPEED_STATUS = ((uint16_t)0x4010);
            ETH_CHIP_DUPLEX_STATUS = ((uint16_t)0x2000);
            PHY_TYPE = YT8512C;
        }
        else
        {
            // 闂傚倸鍊烽悞锕€顭垮Ο鑲╃煋闁割偅娲橀崑顏堟煕閳╁喚鐒介柍缁樻⒒缁辨帡鏁嶉崟顐㈠箠L8201
            ETH_CHIP_PHYSCSR = ((uint16_t)0x10);
            ETH_CHIP_SPEED_STATUS = ((uint16_t)0x0022);
            ETH_CHIP_DUPLEX_STATUS = ((uint16_t)0x0004);
            PHY_TYPE = RTL8201;
        }
        break;
    case SR8201F_PHYREGISTER2:
        // 闂傚倸鍊烽悞锕€顭垮Ο鑲╃煋闁割偅娲橀崑顏堟煕閳╁喚鐒介柍缁樻⒒缁辨帡鏁嶉崟顐ｎ唸8201F
        ETH_CHIP_PHYSCSR = ((uint16_t)0x00);
        ETH_CHIP_SPEED_STATUS = ((uint16_t)0x2020);
        ETH_CHIP_DUPLEX_STATUS = ((uint16_t)0x0100);
        PHY_TYPE = SR8201F;
        break;
    case LAN8720A_PHYREGISTER2:
        // 闂傚倸鍊烽悞锕€顭垮Ο鑲╃煋闁割偅娲橀崑顏堟煕閳╁喚鐒介柍缁樻⒒缁辨帡鍩勯崘鈺呮N8720A
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
    // 闂備浇宕垫慨鏉懨洪埡鍜佹晪鐟滄垿濡甸幇鏉跨倞闁靛ě鈧弸鏍⒑闂堟稓澧曠紒缁樺灩閳ь剚鐔幏锟�?BMSR闂備浇顕ч柊锝咁焽瑜忛埀顒佸嚬閸撶喖骞冨ú顏勎╅柍杞扮窔濡兘姊洪棃娴ゆ盯鍩€椤掑嫬鐒垫い鎺嶅閹寸姷鐭氶柛妤冨亹閺€钘夆攽閻樻彃鈧顢欓崱娑欏€甸柣銏ゆ涧椤ｅ吋銇勯妸銉﹀殗闁诡喖娼″畷鐔碱敍濞戞帗瀵栭梻浣告啞娓氭宕归柆宥嗗仼闂侇剙绉甸埛鎴︽煟閹伴潧澧紒鈧崘顔界厽妞ゆ挶鍨婚悾娲煙椤旇娅囬柟宄邦儑閹叉挳宕熼顐＄处?
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BSR, &value);
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BSR, &value);

    // 濠电姷顣藉Σ鍛村磻閳ь剟鏌涚€ｎ偅宕岄柡宀嬬磿娴狅妇鎷犻幓鎺懶曢梻浣侯攰濡嫰濡堕幖浣告瀬闁瑰墽绮弲鎼佹煥閻曞倹瀚�?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ｉ幋锝嗩棄閻熸瑱绠撻弻銊╁即濡も偓娴滈箖姊洪崨濠傜亶闁逞屽墯閸撴岸宕甸崼銉︾叄闊洦鎸荤拹锛勬喐鐢喗瀚�
    if ((value & ETH_CHIP_BSR_LINK_STATUS) != 0)
    {
        int32_t readval = 0;
        uint32_t duplex = 0;
        uint32_t speed = 0;

        readval = eth_chip_get_link_state();
        if (readval == ETH_CHIP_STATUS_READ_ERROR)
        {
            linkState = FALSE; // 闂傚倷绀侀幖顐⒚洪妶澶嬪仱闁靛ň鏅涢拑鐔封攽閻樺弶澶勯柣鎿勭秮閺岀喓鎲撮崟顐㈩潕闂佺粯鏌ㄩ幊姗€寮婚敐鍛傜喖宕归鎯у缚闂備線鈧偛鑻崢鍛婁繆閻愭潙绗ч柟宄扮秺閹垽宕楅崗鍏碱仧闂備浇娉曢崳锕傚箯閿燂拷?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶鏌ㄩ悤鍌涘
            printf("Link state read error\n");
        }
        else
        {
            // 闂傚倷绀侀幖顐ょ矓閻戞枻缍栧璺猴功閺嗐倕銆掑锝呬壕闂佸搫鐫欓崶褏顔夐梺褰掑亰閸犳岸宕幘顔解拺缂備焦銆掕ぐ鎺戞槬闁哄稁鍘奸悞鍨亜閹达絾纭剁紒娑樼箻閺岀喖宕欓妶鍡楊伓?闂傚倷娴囧▔鏇㈠闯閿曞倸绠柨鐕傛嫹?闂傚倷娴囧▔鏇㈠窗閹版澘鍑犲┑鐘宠壘缁狀垶姊洪崹顕呭剱闁稿海鍠栭弻銊╁即濡も偓娴滈箖姊虹涵鍛吂闁告鍟块悾鐑芥晲閸ワ附鍕冮梺鐟扮摠鐢偤鎯屽Δ鈧埞鎴﹀煡閸℃浠氬┑鈩冨絻鐎氫即鐛箛娑欐櫢闁跨噦鎷�?
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

            /* 闂傚倸鍊烽悞锕€顭垮Ο鑲╃煋闁割偅娲橀崑顏堟煕濠靛棗顕 */
            HAL_ETH_GetMACConfig(&heth, &macConfig);
            macConfig.DuplexMode = duplex;
            macConfig.Speed = speed;
            macConfig.TransmitQueueMode = ETH_TRANSMITTHRESHOLD_128; // ETH婵犵數鍋熼ˉ鎰板磻閹邦厽鍙忛柛鎾楀嫷鍋ㄥ銈嗘尵閸婏綁鎮㈤悡搴ｄ紜閻庤娲栧ú銊┿€侀敓锟� 128闂備浇顕х€涒晝绮欓幒妞尖偓鍐╁緞鐎ｂ晝绠氶梺璺ㄥ櫐閹凤拷
            HAL_ETH_SetMACConfig(&heth, &macConfig);
            linkState = TRUE; // 闂傚倷绀侀幖顐⒚洪妶澶嬪仱闁靛ň鏅涢拑鐔封攽閻樺弶澶勯柣鎿勭秮閺岀喓鎲撮崟顐㈩潕闂佺粯鏌ㄩ幊姗€寮婚敐鍛傜喖宕归鎯у缚闂備線鈧偛鑻崢鍛婁繆閻愭潙绗ч柟宄扮秺閹垽宕ㄦ繝浣瑰攭闂備胶绮崝鏍ь焽濞嗘劕绶為柨鐕傛嫹
                              // HAL_ETH_Start(&heth);
            if (HAL_ETH_Start_IT(&heth) == HAL_OK)
                printf("HAL_ETH_Start_IT_SUCCESS\r\n");
            else
                printf("HAL_ETH_Start_IT_ERROR\r\n");

            HAL_ETH_BuildRxDescriptors(&heth);  // 闂傚倸鍊烽悞锕併亹閸愵亞鐭撻柣銏⑶归拑鐔封攽閻樺弶鎼愰柛鎴犲█閺屾盯寮撮妸銉ょ敖缂備焦鍞荤粻鎾诲蓟閻旂厧绠掗柟鐑樺灥婵洟姊烘潪鎵槮缂傚秴锕悰顕€骞橀崷顓犳澑濠殿喗锕╅崜娑欐櫠椤栨粎纾藉ù锝堫嚃濞堬綁鏌熼崙銈嗗
        }
    }
    else
    {
        linkState = FALSE; // 闂傚倷绀侀幖顐⒚洪妶澶嬪仱闁靛ň鏅涢拑鐔封攽閻樺弶澶勯柣鎿勭秮閺岀喓鎲撮崟顐㈩潕闂佺粯鏌ㄩ幊姗€寮婚敐鍛傜喖宕归鎯у缚闂備線鈧偛鑻崢鍛婁繆閻愭潙绗ч柟宄扮秺閺佹捇鏁撻敓锟�    
    }
}

extern int dorun;

void PhyTick(void)
{
    uint32_t value;
    int link;

    // 闂備浇宕垫慨鏉懨洪埡鍜佹晪鐟滄垿濡甸幇鏉跨倞闁靛ě鈧弸鏍⒑闂堟稓澧曠紒缁樺灩閳ь剚鐔幏锟�?BMSR闂備浇顕ч柊锝咁焽瑜忛埀顒佸嚬閸撶喖骞冨ú顏勎╅柍杞扮窔濡兘姊洪棃娴ゆ盯鍩€椤掑嫬鐒垫い鎺嶅閹寸姷鐭氶柛妤冨亹閺€钘夆攽閻樻彃鈧顢欓崱娑欏€甸柣銏ゆ涧椤ｅ吋銇勯妸銉﹀殗闁诡喖娼″畷鐔碱敍濞戞帗瀵栭梻浣告啞娓氭宕归柆宥嗗仼闂侇剙绉甸埛鎴︽煟閹伴潧澧紒鈧崘顔界厽妞ゆ挶鍨婚悾娲煙椤旇娅囬柟宄邦儑閹叉挳宕熼顐＄处? 
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BSR, &value);
    HAL_ETH_ReadPHYRegister(&heth, ETH_CHIP_ADDR, ETH_CHIP_BSR, &value);

    // 闂傚倷绀侀崥瀣磿閹惰棄搴婇柤鑹扮堪娴滃綊鏌涢妷顔荤暗濞存粌缍婇弻鐔煎箚瑜嶉弳杈ㄣ亜閵堝懏鍤囨慨濠冩そ閹瑩顢楅埀顒傜矆閸愵喗鐓熸い鎾卞灮閻ｆ椽鏌熼瑙勬珖闁瑰嘲顑囬幉鎾礋椤愵偂绱�?
    link = (value & ETH_CHIP_BSR_LINK_STATUS) ? TRUE : FALSE;

    // 濠电姷顣藉Σ鍛村磻閳ь剟鏌涚€ｎ偅宕岄柡宀嬬磿娴狅妇鎷犻幓鎺戭潥婵犵鈧啿绾ч柟顔煎€搁悾鐑藉Ψ閳哄倹娅嗛梺鍏间航閸庢娊鎮鹃幎鑺モ拻濞达絽鎼禍鍓р偓瑙勬礃閿曘垽骞嗗畝鍕€锋い鎺戝€甸弸鏍ㄧ節閻㈤潧孝闁稿﹤鎽滄竟鏇㈠箮閼恒儳鍘甸柣搴㈢⊕椤洦鏅惰缁辨帒螖閳ь剟骞婂Ο渚綎闁芥ê顦介崥瀣煕閺囥劌骞樻繛鍫涘€栫换娑氣偓鐢登归鎾绘煙閸戙倖瀚�
    if (link && !linkState)
    {
        phyEvent = TRUE; // 闂備浇宕垫慨宕囩矆娴ｈ娅犲ù鐘差儐閸嬵亪鏌涢埄鍐剧劷妞も晜鐓￠幃妤呮晲閸屾稒鐝曞銈嗗姃缁瑩寮婚垾鎰佸悑闁告侗鍠楅幃娆戠磽娴ｉ涓查柟鍑ゆ嫹
        printf("Link up event\n");
    }
    else if (!link && linkState)
    {
        phyEvent = TRUE; // 闂備浇宕垫慨宕囩矆娴ｈ娅犲ù鐘差儐閸嬵亪鏌涢埄鍐剧劷妞も晜鐓￠幃妤呮晲閸屾稒鐝曞銈嗗姃缁瑩寮婚垾鎰佸悑闁告侗鍠楅幃娆戠磽娴ｉ涓查柟鍑ゆ嫹
        dorun = 0;       // 闂傚倷鑳舵灙妞ゆ垵妫涢崚鎺戭吋婢舵ɑ鏅㈠┑鐐村灦閻燂附绂嶉妶澶嬬厵闂侇叏绠戦悘閬嶆煛閳ь剟鏁撻敓锟�
        printf("Link down event\n");
    }
}


uint32_t sendfinishflag=0;
void low_level_output(uint8_t *p,uint32_t length)
{  
    uint32_t framelen = 0;
    HAL_StatusTypeDef HalStatus;
    ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT]; // 闂傚倷绀侀幉锟犳偡閿曞倸鍨傞柛褎顨呴悞鍨亜閹达絽鍔甸柛蹇撶灱缁辨帡骞夌€ｎ剛鏆ら悗瑙勬礃閿曘垽鐛幘璇茬闁硅揪闄勯锟�

    // 濠电姷鏁搁崑鐐哄箰閹间礁绠犻柟鐗堟緲閻撴﹢鏌ㄩ悤鍌涘 Txbuffer 闂傚倷娴囧銊╂倿閿旂晫鐝堕柛鈩冪懃閸ㄦ繈鏌ㄩ悤鍌涘
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
    sendfinishflag = 0;// 闂傚倷绀侀幉锟犳偡閿曞倸鍨傞柛褎顨呴悞鍨亜閹达絾纭舵い锔肩畵閺岋綁鏁愭惔鈥斥拫濡ょ姷鍋涢澶愮嵁閸ヮ剦鏁囬柣妯垮皺娴煎洭姊绘繝搴′簻婵炶绠撴俊鐢告倷閻㈢數顦梺鍛婁緱閸樺綊鍩€椤掍礁娴€规洏鍔庨埀顒佺⊕椤洭濡堕敓锟�
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
