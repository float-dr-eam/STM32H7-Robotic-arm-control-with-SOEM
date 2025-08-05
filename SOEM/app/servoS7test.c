#include "servoS7test.h"
#include <stdio.h>
#include "osal.h"
#include "ethercat.h"
#include "slaveinfo.h"

#include "lcd.h"

// 声明外部变量 IOmap，用于存储过程数据映射
extern char IOmap[4096];

// 声明外部变量 dorun，用于控制循环过程数据的激活状态
extern int dorun;
// 声明外部变量 cur_pos，用于存储当前位置
 int32 cur_pos;
// 声明外部变量 cur_mode，用于存储当前模式
 uint8 cur_mode;
// 定义指向 S7 从站输出 PDO 结构体的指针
S7_PDO_Output *outputS7;
// 定义指向 S7 从站输入 PDO 结构体的指针
S7_PDO_Input *inputS7;
// 定义 S7 从站状态标志变量
uint8 S7flag;

/**
 * @brief 对 S7 从站进行 PDO（过程数据对象）配置设置。
 * 
 * 该函数通过 EtherCAT 的 SDO（服务数据对象）协议，向指定的 S7 从站写入一系列配置参数，
 * 包括接收和发送 PDO 的映射配置。
 * 
 * @param slave 从站编号，指定要配置的 S7 从站。
 * @return 固定返回 1，表示函数执行完毕。
 */
int ServoS7setup(uint16 slave)
{
    // 定义返回值，用于累加 SDO 写入操作的结果
    int retval;
    // 定义 16 位无符号整数变量
    uint16 u16val;
    // 定义 8 位无符号整数变量
    uint8  u8val;
    // 定义 32 位无符号整数变量
    uint32 u32val;
    // 初始化返回值为 0
    retval = 0;

    // 配置 0x1c12 索引下的 PDO 映射
    u8val = 0;
    // 向从站的 0x1c12 索引，子索引 0x00 写入 u8val
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1600;
    // 向从站的 0x1c12 索引，子索引 0x01 写入 u16val
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    // 向从站的 0x1c12 索引，子索引 0x00 写入 u8val
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    // 配置 0x1600 索引下的 PDO 映射
    u8val = 0;
    // 向从站的 0x1600 索引，子索引 0x00 写入 u8val
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u32val = 0x60400010;
    // 向从站的 0x1600 索引，子索引 0x01 写入 u32val
    retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x607A0020;
    // 向从站的 0x1600 索引，子索引 0x02 写入 u32val
    retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60600008;
    // 向从站的 0x1600 索引，子索引 0x03 写入 u32val
    retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x5FFE0008;
    // 向从站的 0x1600 索引，子索引 0x04 写入 u32val
    retval += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u8val = 4;
    // 向从站的 0x1600 索引，子索引 0x00 写入 u8val
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    // 配置 0x1c13 索引下的 PDO 映射
    u8val = 0;
    // 向从站的 0x1c13 索引，子索引 0x00 写入 u8val
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1a00;
    // 向从站的 0x1c13 索引，子索引 0x01 写入 u16val
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    // 向从站的 0x1c13 索引，子索引 0x00 写入 u8val
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    // 配置 0x1A00 索引下的 PDO 映射
    u8val = 0;
    // 向从站的 0x1A00 索引，子索引 0x00 写入 u8val
    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u32val = 0x60410010;
    // 向从站的 0x1A00 索引，子索引 0x01 写入 u32val
    retval += ec_SDOwrite(slave, 0x1A00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60640020;
    // 向从站的 0x1A00 索引，子索引 0x02 写入 u32val
    retval += ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x606C0020;
    // 向从站的 0x1A00 索引，子索引 0x03 写入 u32val
    retval += ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x603F0010;
    // 向从站的 0x1A00 索引，子索引 0x04 写入 u32val
    retval += ec_SDOwrite(slave, 0x1A00, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60610008;
    // 向从站的 0x1A00 索引，子索引 0x05 写入 u32val
    retval += ec_SDOwrite(slave, 0x1A00, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x5FFF0008;
    // 向从站的 0x1A00 索引，子索引 0x06 写入 u32val
    retval += ec_SDOwrite(slave, 0x1A00, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u8val = 6;
    // 向从站的 0x1A00 索引，子索引 0x00 写入 u8val
    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    return 1;
}

/**
 * @brief 进行 S7 伺服测试的主函数。
 * 
 * 该函数初始化 EtherCAT 网络，查找并配置从站，设置从站状态为操作状态，
 * 并打印从站的相关信息。如果所有从站成功进入操作状态，则激活循环过程数据。
 * 
 * @param ifname 网络接口名称，用于绑定 EtherCAT 通信的套接字。
 */
void servoS7test(char *ifname)
{
    // 定义计数器变量
    int cnt, i, j;
    // 定义 16 位无符号整数变量，用于存储 SII 通用部分的索引
    uint16 ssigen;

    // 打印测试开始信息
    printf("Starting Servo test\n");

    /* initialise SOEM, bind socket to ifname */
    // 初始化 SOEM 库，并将套接字绑定到指定的网络接口
    if (ec_init(ifname))
    {
        // 打印初始化成功信息
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */
        // 查找并自动配置 EtherCAT 从站
        if ( ec_config_init(FALSE) > 0 )
        {
            // 初始化循环过程数据激活标志为 0
            dorun = 0;
            // 初始化 S7 从站状态标志为 1
            S7flag = 1;
            // 打印找到并配置的从站数量
            printf("%d slaves found and configured.\n",ec_slavecount);

            /* read indevidual slave state and store in ec_slave[] */
            // 读取每个从站的状态并存储在 ec_slave 数组中
            ec_readstate();
            for(cnt = 1; cnt <= ec_slavecount ; cnt++)
            {
                // 打印从站的厂商 ID
                printf("ec_slave[%d].eep_man = %x\n",cnt,ec_slave[cnt].eep_man);
                // 打印从站的设备 ID
                printf("ec_slave[%d].eep_id = %x\n",cnt,ec_slave[cnt].eep_id);
                /* check for EL2004 or EL2008 */
                // 检查从站是否为 EL2004 或 EL2008 类型
                if((ec_slave[cnt].eep_id == 0x00000748) || (ec_slave[cnt].eep_id == 0x00000006))
                {
                    // 为符合条件的从站设置 PDO 配置函数
                    ec_slave[cnt].PO2SOconfig = &ServoS7setup;
                }
            }

            // 映射过程数据到 IOmap 数组
            ec_config_map(&IOmap);
            /* configure DC options for every DC capable slave found in the list */
            // 为列表中支持分布式时钟（DC）的从站配置 DC 选项
            ec_configdc();
            // 配置从站 1 的 SYNC0 信号
            ec_dcsync0(1, TRUE, 1000000, 250000); // SYNC0 on slave 1

            /* wait for all slaves to reach SAFE_OP state */
            // 等待所有从站进入安全操作状态
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
            do
            {
                // 发送过程数据
                ec_send_processdata();
                // 接收过程数据
                ec_receive_processdata(EC_TIMEOUTRET);
                // 检查从站 0 的状态
                ec_statecheck(0, EC_STATE_SAFE_OP, 50000);
                // 检查从站 1 的状态
                ec_statecheck(1, EC_STATE_SAFE_OP, 50000);
            }
            // 循环直到从站 0 和从站 1 都进入安全操作状态
            while ((ec_slave[0].state != EC_STATE_SAFE_OP)&&(ec_slave[1].state != EC_STATE_SAFE_OP));
            // 打印从站 0 的状态
            printf("Slave 0 State=0x%04x\r\n",ec_slave[0].state);
            // 打印从站 1 的状态
            printf("Slave 1 State=0x%04x\r\n",ec_slave[1].state);

            // 程序休眠 100000 微秒
            osal_usleep(100000);
            // 定义返回值，用于累加 SDO 写入操作的结果
            int retval;
            // 定义 16 位无符号整数变量
            uint16 u16val;
            // 定义 8 位无符号整数变量
            uint8  u8val;
            u8val = 8;
            // 向从站 1 的 0x6060 索引，子索引 0x00 写入 u8val
            retval += ec_SDOwrite(1, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
            u16val = 0x80;
            // 向从站 1 的 0x6040 索引，子索引 0x00 写入 u16val
            retval += ec_SDOwrite(1, 0x6040, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);

            /* Print som information on the mapped network */
            // 打印映射网络的相关信息
            for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
            {
                // 打印从站的基本信息
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                        cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                        ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                // 打印从站的配置地址
                printf(" Configured address: %x\n", ec_slave[cnt].configadr);
                // 打印从站输出数据的地址
                printf(" Outputs address: %x\n", ec_slave[cnt].outputs);
                // 打印从站输入数据的地址
                printf(" Inputs address: %x\n", ec_slave[cnt].inputs);

                // 打印从站 FMMU（现场总线内存管理单元）的信息
                for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
                {
                    printf(" FMMU%1d Ls:%x Ll:%4d Lsb:%d Leb:%d Ps:%x Psb:%d Ty:%x Act:%x\n", j,
                           (int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
                           ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
                           ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
                }
                // 打印从站 FMMU 功能信息
                printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
                // 打印从站邮箱的长度和协议信息
                printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl, ec_slave[cnt].mbx_proto);
                // 查找从站的 SII 通用部分的索引
                ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
                /* SII general section */
                // 如果找到 SII 通用部分的索引
                if (ssigen)
                {
                    // 读取从站的 CoE 详细信息
                    ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
                    // 读取从站的 FoE 详细信息
                    ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
                    // 读取从站的 EoE 详细信息
                    ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
                    // 读取从站的 SoE 详细信息
                    ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
                    // 检查从站是否只支持 LRD/LWR 协议
                    if((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
                    {
                        ec_slave[cnt].blockLRW = 1;
                        ec_slave[0].blockLRW++;
                    }
                    // 读取从站的 EtherCAT 总线电流信息
                    ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
                    ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
                    ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
                }
                // 打印从站的 CoE、FoE、EoE 和 SoE 详细信息
                printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
                       ec_slave[cnt].CoEdetails, ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
                // 打印从站的 EtherCAT 总线电流和是否只支持 LRD/LWR 协议的信息
                printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
                       ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);
                // 如果从站支持 CoE 协议，则调用 si_sdo 函数
                if ((ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE) && 1)
                    si_sdo(cnt);
                if(1)
                {
                    // 如果从站支持 CoE 协议，则调用 si_map_sdo 函数
                    if (ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE)
                        si_map_sdo(cnt);
                    else
                        // 否则调用 si_map_sii 函数
                        si_map_sii(cnt);
                }
            }

            // 打印请求所有从站进入操作状态的信息
            printf("Request operational state for all slaves\n");
            // 设置从站 0 的状态为操作状态
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            // 发送一次有效的过程数据，使从站的输出正常工作
            ec_send_processdata();
            // 接收过程数据
            ec_receive_processdata(EC_TIMEOUTRET);

            /* request OP state for all slaves */
            // 请求所有从站进入操作状态
            ec_writestate(0);
            /* wait for all slaves to reach OP state */
            // 等待所有从站进入操作状态
            ec_statecheck(0, EC_STATE_OPERATIONAL,  5 * EC_TIMEOUTSTATE);
            // 如果从站 0 成功进入操作状态
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                // 打印所有从站进入操作状态的信息
                printf("Operational state reached for all slaves.\n");
                // 激活循环过程数据
                dorun = 1;
                // 将从站 1 的输出数据地址转换为 S7_PDO_Output 结构体指针
                outputS7 = (S7_PDO_Output *)ec_slave[1].outputs;
                // 将从站 1 的输入数据地址转换为 S7_PDO_Input 结构体指针
                inputS7  = (S7_PDO_Input *)ec_slave[1].inputs;
            }
            else
            {
                // 打印不是所有从站都进入操作状态的信息
                printf("Not all slaves reached operational state.\n");
                // 读取所有从站的状态
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    // 如果从站未进入操作状态，则打印从站的状态和状态码信息
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            // 打印未找到从站的信息
            printf("No slaves found!\n");
            // 在 LCD 上显示未找到从站的信息
            lcd_show_string(200, 170, 300, 16, 16, "servos7test: No slaves found!\n", BLACK);
        }
    }
    else
    {
        // 打印套接字连接失败的信息
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

// 定义加速度计数器变量
int32 acc_cnt;

/**
 * @brief S7 伺服系统的循环处理函数。
 * 
 * 该函数根据 S7 从站的状态字和当前位置，控制从站的目标位置和控制字，
 * 实现 S7 伺服系统的运动控制。
 */
void servoS7_loop(void)
{
    // 接收过程数据
    ec_receive_processdata(EC_TIMEOUTRET);

    // 定义 16 位无符号整数变量，用于存储当前状态字
    uint16 cur_status;

    // 读取从站的状态字
    cur_status = inputS7->StatusWord;//0x6041
    // 读取从站的当前位置
    cur_pos = inputS7->CurrentPosition;//0x6064

    // 如果 S7 从站状态标志为 1
    if(S7flag == 1){
        // 根据状态字判断从站状态，并设置相应的控制字和目标位置
        if((cur_status & 0x004f) == 0x0040)
        {
            outputS7->ControlWord = 0x06;//0x6040
            //printf("0x06\n");
        }
        else if((cur_status & 0x006f) == 0x0021)
        {
            outputS7->ControlWord = 0x07;
            outputS7->TargetPos = cur_pos;//0x607A
            //printf("0x07\n");
        }
        else if((cur_status & 0x006f) == 0x023)
        {
            outputS7->ControlWord = 0x0F;
            outputS7->TargetPos = cur_pos;//0x607A
            //printf("0x0F\n");
        }
        else if((cur_status & 0x006f) == 0x0027)
        {
            outputS7->ControlWord = 0x1F;
            //printf("0x1F\n");

            outputS7->TargetPos = cur_pos;//0x607A
            // 更新 S7 从站状态标志为 2
            S7flag = 2;
            // 初始化加速度计数器为 1
            acc_cnt = 1;
        }
    }

    // 如果 S7 从站状态标志为 2
    if(S7flag == 2){
        // 根据加速度计数器的值更新目标位置
        if(acc_cnt < 16000)
        {
            outputS7->TargetPos = cur_pos + acc_cnt*2;//0x607A
        }
        else
        {
            // 更新 S7 从站状态标志为 3
            S7flag = 3;
            // 打印加速度过程结束的信息
            printf("acc over\n");
        }

        // 设置控制字为 0x1F
        outputS7->ControlWord = 0x1F;
        // 加速度计数器加 1
        acc_cnt++;
    }

    // 如果 S7 从站状态标志为 3
    if(S7flag == 3){
        // 设置目标位置
        outputS7->TargetPos = cur_pos + 32000;//0x607A
        // 设置控制字为 0x1F
        outputS7->ControlWord = 0x1F;
    }

    // 设置目标模式为 0x8
    outputS7->TargetMode = 0x8;
    // 发送过程数据
    ec_send_processdata();
}



