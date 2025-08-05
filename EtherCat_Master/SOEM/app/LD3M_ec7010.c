#include "LD3M_ec7010.h"
#include <stdio.h>
#include "osal.h"
#include "ethercat.h"
#include "slaveinfo.h"
#include "lcd.h"
#include "malloc.h"
#include "usart.h"
#include "tim.h"
// 声明外部变量 IOmap，用于存储过程数据映射
extern char IOmap[IOmap_Length];
extern int dorun;

LD3M_All LD3M_all={0}; // 定义一个全局变量，用于存储所有从站的 PDO 数据
int32 origin_point[NUM_SLAVES+1];

#define saved_flag 123456789 

/*
1. 将 EtherCAT 状态机切换到预操作，此状态下可以用 SD0 来配置 PDO 映射。
2. 清除 PDO 指定对象的 PDO 映射对象，即设置 1C12-00h/1C13-00h 为 0。
3. 使 PDO 映射对象无效，即对 1600h~1603h/1A00h~1A01h 的子索引0赋值为 0。
4. 重新配置 PDO 映射内容，将映射对象按表 8-3 式写入到 1600-01h~1600-08h、1601-01h~1601-08h、1602-01h~1602-08h、1603-01h~1603-08h(1600-01h 开始写入的为 RXPDO 映射内容)、1A00-01h~1A00-08h 或 1A01-01h~1A01-08h(1A00-01h 开始写入的为 TXPDO 映射内容)范围的对象中。
5. 设置 PDO 映射对象的总个数，即将映射对象的个数写入到 1600-00h、1601-00h、1602-00h、1603-00h、1A00-00h 或 1A01-00h 中，未配置映射内容的 PDO 映射对象总个数将为 0。
6. 写有效的 PDO 映射对象索引到 PDO 指定对象，即将有效的 RXPDO 映射对象索引1600h~1603h 写入到 1C12-01h~1C12-04h 中，将有效的 TXPDO 映射对象索引 1A00h、1A01h 写入到 1C13-01h、1C13-02h 中。
7. 设置 PDO 指定对象的总个数，即将映射对象个数写入到 1C12-00h、1C13-00h。
8. 转换 EtherCAT 状态机到安全操作或以上，配置的 PDO 映射将有效。状态转换图。
*/
int LD3M_setup(uint16 slave)
{
    int retval;
    uint16 u16val;
    uint8  u8val;
    uint32 u32val;
    retval = 0;

    // PDO 指 定 对 象(RX)
    u8val = 0;
    // 向从站的 0x1c12 索引，子索引 0x00 写入 0，清除原有映射数量
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    // 步骤 2: 设置 0x1c12 索引下的第一个 PDO 映射对象为 0x1600
    u16val = 0x1600;
    // 向从站的 0x1c12 索引，子索引 0x01 写入 0x1600，指定映射对象
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    // 步骤 3: 更新 0x1c12 索引下的 PDO 映射对象数量为 1
    u8val = 1;
    // 向从站的 0x1c12 索引，子索引 0x00 写入 1，设置新的映射数量
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //PDO 映射对象索引
    u8val = 0;
    // 向从站的 0x1600 索引，子索引 0x00 写入 0，清除原有映射数量
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    // 步骤 2: 设置 0x1600 索引下的第一个 PDO 映射对象为 0x60400010（控制字）
    u32val = 0x60400010;
    // 向从站的 0x1600 索引，子索引 0x01 写入 0x60400010，指定第一个映射对象
    retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // 步骤 3: 设置 0x1600 索引下的第二个 PDO 映射对象为 0x607A0020（目标位置）
    u32val = 0x607A0020;
    // 向从站的 0x1600 索引，子索引 0x02 写入 0x607A0020，指定第二个映射对象
    retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // 步骤 4: 设置 0x1600 索引下的第三个 PDO 映射对象为 0x60600008（操作模式）
    u32val = 0x60600008;
    // 向从站的 0x1600 索引，子索引 0x03 写入 0x60600008，指定第三个映射对象
    retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u8val = 3;
    // 向从站的 0x1600 索引，子索引 0x00 写入 3，设置新的映射总数
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);


    // PDO 指 定 对 象(TX)
    u8val = 0;
    // 向从站的 0x1c13 索引，子索引 0x00 写入 0，清除原有映射数量
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    // 步骤 2: 设置 0x1c13 索引下的第一个 PDO 映射对象为 0x1a00
    u16val = 0x1a00;
    // 向从站的 0x1c13 索引，子索引 0x01 写入 0x1a00，指定映射对象
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    // 步骤 3: 更新 0x1c13 索引下的 PDO 映射对象数量为 1
    u8val = 1;
    // 向从站的 0x1c13 索引，子索引 0x00 写入 1，设置新的映射数量
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    //PDO 映射对象索引
    u8val = 0;
    // 向从站的 0x1A00 索引，子索引 0x00 写入 0，清除原有映射数量
    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    // 步骤 2: 设置 0x1A00 索引下的第一个 PDO 映射对象为 0x60410010（状态字）
    u32val = 0x60410010;
    // 向从站的 0x1A00 索引，子索引 0x01 写入 0x60410010，指定第一个映射对象
    retval += ec_SDOwrite(slave, 0x1A00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // 步骤 3: 设置 0x1A00 索引下的第二个 PDO 映射对象为 0x60640020（当前位置）
    u32val = 0x60640020;
    // 向从站的 0x1A00 索引，子索引 0x02 写入 0x60640020，指定第二个映射对象
    retval += ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // 步骤 4: 设置 0x1A00 索引下的第三个 PDO 映射对象为 0x606C0020(实际速度)
    u32val = 0x606C0020;
    // 向从站的 0x1A00 索引，子索引 0x03 写入 0x606C0020，指定第三个映射对象
    retval += ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // 步骤 5: 设置 0x1A00 索引下的第四个 PDO 映射对象为 0x603F0010（错误码）
    u32val = 0x603F0010;
    // 向从站的 0x1A00 索引，子索引 0x04 写入 0x603F0010，指定第四个映射对象
    retval += ec_SDOwrite(slave, 0x1A00, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    // 步骤 6: 设置 0x1A00 索引下的第五个 PDO 映射对象为 0x60610008（操作模式显示）
    u32val = 0x60610008;
    // 向从站的 0x1A00 索引，子索引 0x05 写入 0x60610008，指定第五个映射对象
    retval += ec_SDOwrite(slave, 0x1A00, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);

    // 步骤 8: 更新 0x1A00 索引下的 PDO 映射对象总数为 5
    u8val = 5;
    // 向从站的 0x1A00 索引，子索引 0x00 写入 5，设置新的映射总数
    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    return 1;
}

/**
 * @brief 进行LD3M伺服测试的主函数。
 * 
 * 该函数初始化 EtherCAT 网络，查找并配置从站，设置从站状态为操作状态，
 * 并打印从站的相关信息。如果所有从站成功进入操作状态，则激活循环过程数据。
 * 
 * @param ifname 网络接口名称，用于绑定 EtherCAT 通信的套接字。
 */
void LD3M_test(char *ifname)
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
            // 初始化LD3M从站状态标志为 1
            for(cnt = 0; cnt < NUM_SLAVES; cnt++)
            {
                LD3M_all.LD3Mflag[cnt] = 1;
            }
            // 打印找到并配置的从站数量
            printf("%d slaves found and configured.\n",ec_slavecount);

            /* read indevidual slave state and store in ec_slave[] */
            // 读取每个从站的状态并存储在 ec_slave 数组中
            ec_readstate();
            for(cnt = 1; cnt <= ec_slavecount ; cnt++)
            {
                // 打印从站的厂商 ID
                printf("ec_slave[%d].eep_man = %x\n",cnt,ec_slave[cnt].eep_man);
                lcd_show_string(200, 50 + cnt * 20, 200, 16, 16,"Find_Slave" , BLACK);
                // 打印从站的设备 ID
                printf("ec_slave[%d].eep_id = %x\n",cnt,ec_slave[cnt].eep_id);
                // if((ec_slave[cnt].eep_id == 0x0000000a))// 检查从站是否为 LD3M 类型
                // {
                    // 为符合条件的从站设置 PDO 配置函数
                    ec_slave[cnt].PO2SOconfig = &LD3M_setup;
                // }
            }

            // 映射过程数据到 IOmap 数组
            ec_config_map(&IOmap);//会使用PO2SOconfig函数来配置从站的PDO映射
            // 为列表中支持分布式时钟（DC）的从站配置 DC 选项
            ec_configdc();
            // SYNC0 是一种由分布式时钟（DC）产生的固定周期触发的同步信号
            // SYNC0 的周期时间为 500000 纳秒（0.5 毫秒），当偏移时间设置为 20000 纳秒（0.02 毫秒）时，SYNC0 同步信号会在原本触发时刻的基础上延迟 0.02 毫秒触发。
            for (cnt = 1; cnt <= ec_slavecount; cnt++)
            {
                // 检查从站是否支持分布式时钟
                if (ec_slave[cnt].hasdc)
                {
                    // 打印从站的 DC 支持信息
                    printf("Slave %d has DC support\n", cnt);
                    // 设置从站的分布式时钟（DC）同步信号
                    ec_dcsync0(cnt, TRUE, 1000*sync_time, 20000); // SYNC0 on slave cnt
                }
            }

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
            // 循环直到从站 0  都进入安全操作状态
            while ((ec_slave[0].state != EC_STATE_SAFE_OP)&&(ec_slave[1].state != EC_STATE_SAFE_OP));
            for( cnt = 0 ; cnt <= ec_slavecount ; cnt++)
            {
                // 打印从站的基本信息
                printf("Slave %d State=0x%04x\r\n",cnt,ec_slave[cnt].state);
            }

            // 程序休眠 100000 微秒
            osal_usleep(100000);

            // 定义返回值，用于累加 SDO 写入操作的结果
            int retval;
            // 定义 16 位无符号整数变量
            uint16 u16val;
            // 定义 8 位无符号整数变量
            uint8  u8val;
            u8val = 8;
            // 向从站 1 的 0x6060 索引，子索引 0x00 写入 8
            retval += ec_SDOwrite(1, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);    //循环同步位置模式
            u16val = 0x80;
            // 向从站 1 的 0x6040 索引，子索引 0x00 写入 u16val
            retval += ec_SDOwrite(1, 0x6040, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM); //控制字

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
                   // si_sdo(cnt);

                // 如果从站支持 CoE 协议，则调用 si_map_sdo 函数
                if (ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE)
                    si_map_sdo(cnt);
                else
                    // 否则调用 si_map_sii 函数
                    si_map_sii(cnt);
             
                // set_motor_target(cnt, 0); // 设置电机目标位置为当前位置
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
        
                //由于内存布局的一致性，inputLD3M 指针可以正确地访问从站输入数据存储区域中的各个成员变量。
                // 将从站 1 的输出数据地址转换为 LD3M_PDO_Output 结构体指针
                for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
                {
                    LD3M_all.outputs[cnt-1] = (LD3M_PDO_Output *)ec_slave[cnt].outputs;
                    // 将从站 1 的输入数据地址转换为 LD3M_PDO_Input 结构体指针
                    LD3M_all.inputs[cnt-1]  = (LD3M_PDO_Input *)ec_slave[cnt].inputs;
                    printf("LD3M_all.outputs[%d] = %x\n",cnt,LD3M_all.outputs[cnt-1]);
                    printf("LD3M_all.inputs[%d]  = %x\n",cnt,LD3M_all.inputs[cnt-1]);
                }

                //读取flash中的原点位置
                read_origin_point_from_flash(origin_point, NUM_SLAVES + 1);

                for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
                {
                    int readlen = sizeof(LD3M_all.cur_pos[cnt-1]);
                    ec_SDOread(cnt, 0x6064, 0x00,0,&readlen ,&LD3M_all.cur_pos[cnt-1], EC_TIMEOUTRXM);
                    LD3M_all.target_pos[cnt-1] =  LD3M_all.cur_pos[cnt-1]; // 设置电机目标位置为当前位置

                    if( origin_point[0]!=saved_flag)//没有保存过
                    {
                        origin_point[cnt] = LD3M_all.cur_pos[cnt-1];  // 初始化原点位置为当前位置
                        LD3M_all.cur_degree[cnt-1] = 0;               // 初始化当前角度为0
                    }
                    else//保存过
                    {
                       LD3M_all.cur_degree[cnt-1] = (my_float)(LD3M_all.cur_pos[cnt-1] - origin_point[cnt]) *rotate_sign[cnt-1]/ UNIT_PULSES[cnt-1]; // 计算当前角度 °
                    }

                    LD3M_all.has_new_target[cnt-1] = false; 
                    LD3M_all.LD3Mflag[cnt-1] = 1;
                    printf("LD3M_all.cur_degree[%d] = %f\n",cnt-1,LD3M_all.cur_degree[cnt-1]);
                }

                if( origin_point[0]!=saved_flag)//没有保存过    
                {
                    origin_point[0]=saved_flag;
                    HAL_StatusTypeDef state = write_origin_point_to_flash(origin_point, NUM_SLAVES + 1);  //写入flash
                    if (state == HAL_OK) 
                    {
                        printf("Origin point saved successfully.\n");
                    } 
                    else 
                    {
                        printf("Failed to save origin point.\n");
                    }
                }

                // 激活循环过程数据
                dorun = 1;
                
                
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
/*
位置模式	PP/HM/CSP	    0：旋转方向与位置指令一致
                           128：旋转方向与位置指令相反
		
速度模式	PV/CSV	        0：旋转方向与位置指令一致
                           64：旋转方向与位置指令相反
		
转矩模式	PT/CST	        0：旋转方向与位置指令一致
                           32：旋转方向与位置指令相反
		
所有模式		            0：旋转方向与位置指令一致
                           224：旋转方向与位置指令相反
uint8 u8val= 128;
ec_SDOwrite(1, 0x607E, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM); //运转方向*/


/**
 * @brief 设置指定电机的目标位置
 * 
 * @param slave_index 电机的从站索引
 * @param target 相对目标位置
 */
void set_motor_target(uint8 slave_index, int32 target) 
{
    if (slave_index < ec_slavecount) 
    {
        LD3M_all.target_pos[slave_index] = target;
        LD3M_all.has_new_target[slave_index] = true;
    }
    else 
    {
        printf("Invalid slave index: %d\n", slave_index);
    }
}

extern bool update_flag;
/**
 * @brief 设置指定电机的目标角度
 *
 * @param slave_index 电机的从站索引
 * @param target_degree 目标角度
 */
void set_motor_target_degree(uint8 slave_index, my_float target_degree)  
{
    if (slave_index < ec_slavecount) 
    {
        printf("\nMotor %d current position: %d\n", slave_index, LD3M_all.cur_pos[slave_index]);
        // 先进行浮点数运算，最后转换为 int32 类型
        LD3M_all.target_pos[slave_index] = (LD3M_all.cur_pos[slave_index] + (int32)rotate_sign[slave_index]*UNIT_PULSES[slave_index] * target_degree);
        LD3M_all.has_new_target[slave_index] = true;
        LD3M_all.cur_degree[slave_index] += target_degree;
        printf("\nMotor %d target position set to %d\n", slave_index, LD3M_all.target_pos[slave_index]);
        update_flag=1;
    }
    else 
    {
        printf("Invalid slave index: %d\n", slave_index);
    }
}

/**
 * @brief 设置指定电机的目标角度
 *
 * @param slave_index 电机的从站索引
 * @param target_degree 目标角度
 */
void set_motor_target_int_degree(uint8 slave_index, int32 target_degree)  
{
    if (slave_index < ec_slavecount) 
    {
        printf("\nMotor %d current position: %d\n", slave_index, LD3M_all.cur_pos[slave_index]);
        LD3M_all.target_pos[slave_index] = (LD3M_all.cur_pos[slave_index] + (int32)rotate_sign[slave_index]*UNIT_PULSES[slave_index] * target_degree);
        LD3M_all.has_new_target[slave_index] = true;
        LD3M_all.cur_degree[slave_index] += target_degree;
        printf("\nMotor %d target position set to %d\n", slave_index, LD3M_all.target_pos[slave_index]);
        update_flag=1;
    }
    else 
    {
        printf("Invalid slave index: %d\n", slave_index);
    }
}

/**
 * @brief 让所有电机回到原点位置
 */
void all_motors_return_to_origin(void)
{
    for(uint8 i = 0; i < ec_slavecount; i++)
    {
        // 设置目标位置为原点位置
        LD3M_all.target_pos[i] = origin_point[i+1];
        LD3M_all.has_new_target[i] = true;
        // 更新当前角度为0
        LD3M_all.cur_degree[i] = 0;
    }
}

/**
 * @brief 让电机运行到给定角度数组中的角度
 * @param angles 目标角度数组
 */
void motors_run_to_angles(my_float *angles)
{

    // 为每个电机设置目标角度
    for(uint8 i = 0; i < ec_slavecount; i++) 
    {
        // 计算目标位置
        int32 target = origin_point[i+1] + (int32)(rotate_sign[i] * UNIT_PULSES[i] * angles[i]);
        
        // 设置目标位置
        LD3M_all.target_pos[i] = target;
        LD3M_all.has_new_target[i] = true;
        
        // 更新当前角度
        LD3M_all.cur_degree[i] = angles[i];
        
        printf("Motor %d target angle set to %.2f degrees\n", i, angles[i]);
    }
    update_flag = 1;
}
/**
 * @brief 所有电机急停函数
 * 
 * 该函数将所有电机的控制字设置为0x02,使电机立即停止运行
 */
void emergency_stop(void)
{
    // 遍历所有从站
    for(uint8 cnt = 0; cnt < ec_slavecount; cnt++)
    {
        // 设置控制字为0x02,使电机立即停止
        LD3M_all.outputs[cnt]->ControlWord = 0x02;
        
        // 将目标位置设为当前位置,防止电机继续运动
        LD3M_all.outputs[cnt]->TargetPos = LD3M_all.cur_pos[cnt];
        LD3M_all.target_pos[cnt] = LD3M_all.cur_pos[cnt]; // 设置目标位置为当前位置
        // 清除新目标标志
        LD3M_all.has_new_target[cnt] = false;
        
        // 重置状态标志
        LD3M_all.LD3Mflag[cnt] = 1;
    }
    
    // 立即发送过程数据
    ec_send_processdata();
}
/**
 * @brief 重新启动所有电机
 */
void restart_motors(void)
{
    // 遍历所有从站
    for(uint8 cnt = 0; cnt < ec_slavecount; cnt++)
    {
        // 重置控制字为0x06,使电机重新启动
        LD3M_all.outputs[cnt]->ControlWord = 0x06;
        
        // 将目标位置设为当前位置
        LD3M_all.outputs[cnt]->TargetPos = LD3M_all.cur_pos[cnt];
        LD3M_all.target_pos[cnt] = LD3M_all.cur_pos[cnt];
        
        // 重置新目标标志
        LD3M_all.has_new_target[cnt] = false;
        
        // 重置状态标志为1,重新进入初始化流程
        LD3M_all.LD3Mflag[cnt] = 1;
    }
    
    // 立即发送过程数据
    ec_send_processdata();
}
/**
 * @brief 设置指定电机的目标速度
 * 
 * @param slave_index 电机的从站索引
 * @param target 目标速度
 */
// void set_motor_target_vel(uint8 slave_index, int32 target)
// {
//     if (slave_index < ec_slavecount) 
//     {
//         LD3M_all.target_vel[slave_index] = target;
//         LD3M_all.has_new_target[slave_index] = true;
//     }
//     else 
//     {
//         printf("Invalid slave index: %d\n", slave_index);
//     }
//     // 设置目标速度
//     printf("Motor %d target velocity set to %d\n", slave_index, target);
// }
/**
 * @briefLD3M伺服系统的循环处理函数。
 * 
 * 该函数根据LD3M从站的状态字和当前位置，控制从站的目标位置和控制字，
 * 实现LD3M伺服系统的运动控制。
 */
// char string[100];
void LD3M_loop(void)
{
    // 接收过程数据，等待接收过程数据的超时时间为EC_TIMEOUTRET
    ec_receive_processdata(EC_TIMEOUTRET);
    uint16 cur_status;
    uint8 cnt;
    for( cnt = 0 ; cnt < ec_slavecount ; cnt++)
    {
        // 读取从站的状态字，状态字对应对象字典索引0x6041
        cur_status = LD3M_all.inputs[cnt]->StatusWord;   
        // 读取从站的当前位置，当前位置对应对象字典索引0x6064
        LD3M_all.cur_pos[cnt] = LD3M_all.inputs[cnt]->CurrentPosition;
        LD3M_all.cur_mode[cnt] = LD3M_all.inputs[cnt]->CurrentMode;    
        LD3M_all.cur_vel[cnt] = LD3M_all.inputs[cnt]->CurrentVelocity; 
        // 如果有新的目标位置，更新状态标志
        if (LD3M_all.has_new_target[cnt]) 
        {
            LD3M_all.LD3Mflag[cnt] = 1;
            LD3M_all.has_new_target[cnt] = false;
        }
        // 如果LD3M从站状态标志为 1
        if(LD3M_all.LD3Mflag[cnt] == 1)   
        {
            // 根据状态字判断从站状态，并设置相应的控制字和目标位置 
            // 当状态字与0x004f按位与结果为0x0040时 
            if((cur_status & 0x004f) == 0x0040)            //伺服不可运行   
            {
                // 设置控制字为0x06，控制字对应对象字典索引0x6040
                LD3M_all.outputs[cnt]->ControlWord = 0x06;            //接通主回路电源
            }       
            else if((cur_status & 0x006f) == 0x0021)       //伺服准备好 快速停机无效
            {
                LD3M_all.outputs[cnt]->ControlWord = 0x07;            //可以开启伺服运行 接通主回路电 快速停机无效
                LD3M_all.outputs[cnt]->TargetPos = LD3M_all.cur_pos[cnt];     
            }   
            else if((cur_status & 0x006f) == 0x023)        //伺服准备好 可以开启伺服运行 快速停机无效
            {
                LD3M_all.outputs[cnt]->ControlWord = 0x0F;           //4-6运行模式相关	与各伺服运行模式相关
                LD3M_all.outputs[cnt]->TargetPos = LD3M_all.cur_pos[cnt];    
            }   
            else if((cur_status & 0x006f) == 0x0027)      //伺服准备好 可以开启伺服运行 伺服运行 快速停机无效   
            {
                LD3M_all.outputs[cnt]->ControlWord = 0x1F;           //伺服运行   4-6运行模式相关	与各伺服运行模式相关
                LD3M_all.outputs[cnt]->TargetPos = LD3M_all.cur_pos[cnt];         
                // 更新LD3M从站状态标志为 2
                LD3M_all.LD3Mflag[cnt] = 2;
            }   

        }
        // else if(LD3M_all.LD3Mflag[cnt] == 2)   
        // {
        //     // 加速度计数器加 1 0
        //     a_cnt[cnt]+=10;    
        //     // 根据加速度计数器的值更新目标位置 
        //     if(a_cnt[cnt] < a_cnt[cnt])//5*0.5ms=2.5ms    
        //     {
        //         // 设置目标位置为当前位置加加速度计数器的值，目标位置对应对象字典索引0x607A 
        //         LD3M_all.outputs[cnt]->TargetPos = LD3M_all.cur_pos[cnt] + a_cnt[cnt]*2; 
        //     }   
        //     else
        //     {
        //         LD3M_all.LD3Mflag[cnt] = 3;
        //     }   
        //     // 设置控制字为 0x1F
        //     LD3M_all.outputs[cnt]->ControlWord = 0x1F;   //伺服运行   4-6运行模式相关	与各伺服运行模式相关     
        // }   
        else if(LD3M_all.LD3Mflag[cnt] == 2)   
        {
            // sprintf(string, "1:T: %d, C: %d\r\n", LD3M_all.outputs[cnt]->TargetPos, LD3M_all.cur_pos[cnt]);
            // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)string, strlen(string));

            if (a_cnt[cnt] < LD3M_all.target_pos[cnt] - LD3M_all.cur_pos[cnt])          //正转
            {
                LD3M_all.outputs[cnt]->TargetPos = LD3M_all.cur_pos[cnt] + a_cnt[cnt];  // °/0.5ms
            } 
            else if (LD3M_all.target_pos[cnt] - LD3M_all.cur_pos[cnt] < -a_cnt[cnt])    //反转
            {
                LD3M_all.outputs[cnt]->TargetPos = LD3M_all.cur_pos[cnt] - a_cnt[cnt];  // °/0.5ms
            } 
            else if (LD3M_all.target_pos[cnt] - LD3M_all.cur_pos[cnt] > 0)           //正转结束   
            {
                LD3M_all.outputs[cnt]->TargetPos = LD3M_all.target_pos[cnt];
            } 
            else if (LD3M_all.target_pos[cnt] - LD3M_all.cur_pos[cnt] < 0)           //反转结束
            {
                LD3M_all.outputs[cnt]->TargetPos = LD3M_all.target_pos[cnt];
            }
            else 
            {
                LD3M_all.outputs[cnt]->TargetPos = LD3M_all.target_pos[cnt];
            }
            LD3M_all.outputs[cnt]->ControlWord = 0x1F;   //伺服运行   4-6运行模式相关	与各伺服运行模式相关  
            // sprintf(string, "2:T: %d, C: %d\r\n", LD3M_all.outputs[cnt]->TargetPos, LD3M_all.cur_pos[cnt]);
            // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)string, strlen(string));
        }   
        // 设置目标模式为 0x08  
        LD3M_all.outputs[cnt]->TargetMode = 0x08;
    }
    // 发送过程数据
    ec_send_processdata();
}

    // // 读取从站的状态字，状态字对应对象字典索引0x6041
    // cur_status = inputLD3M->StatusWord;  
    // // 读取从站的当前位置，当前位置对应对象字典索引0x6064
    // cur_pos = inputLD3M->CurrentPosition;

    // cur_mode[cnt] = inputLD3M->CurrentMode;
    // cur_vel = inputLD3M->CurrentVelocity;     
    // // 如果LD3M从站状态标志为 1
    // if(LD3M_all.LD3Mflag[cnt-1] == 1)
    // {
    //     // 根据状态字判断从站状态，并设置相应的控制字和目标位置
    //     // 当状态字与0x004f按位与结果为0x0040时
    //     if((cur_status & 0x004f) == 0x0040)            //伺服不可运行   
    //     {
    //         // 设置控制字为0x06，控制字对应对象字典索引0x6040
    //         outputLD3M->ControlWord = 0x06;            //接通主回路电源
    //     }
    //     // 当状态字与0x006f按位与结果为0x0021时
    //     else if((cur_status & 0x006f) == 0x0021)       //伺服准备好 快速停机无效
    //     {
    //         outputLD3M->ControlWord = 0x07;            //可以开启伺服运行 接通主回路电 快速停机无效
    //         // 设置目标位置为当前位置，目标位置对应对象字典索引0x607A
    //         outputLD3M->TargetPos = cur_pos;  
    //     }
    //     // 当状态字与0x006f按位与结果为0x023时
    //     else if((cur_status & 0x006f) == 0x023)        //伺服准备好 可以开启伺服运行 快速停机无效
    //     {
    //         outputLD3M->ControlWord = 0x0F;           //4-6运行模式相关	与各伺服运行模式相关
    //         // 设置目标位置为当前位置，目标位置对应对象字典索引0x607A
    //         outputLD3M->TargetPos = cur_pos;  
    //     }
    //     // 当状态字与0x006f按位与结果为0x0027时
    //     else if((cur_status & 0x006f) == 0x0027)      //伺服准备好 可以开启伺服运行 伺服运行 快速停机无效
    //     {
    //         outputLD3M->ControlWord = 0x1F;           //伺服运行   4-6运行模式相关	与各伺服运行模式相关
    //         // 设置目标位置为当前位置，目标位置对应对象字典索引0x607A
    //         outputLD3M->TargetPos = cur_pos;  
    //         // 更新LD3M从站状态标志为 2
    //         LD3M_all.LD3Mflag[cnt-1] = 2;
    //         // 初始化加速度计数器为 1
    //         a_cnt = 1;
    //     }
    // }

    // // 如果LD3M从站状态标志为 2
    // if(LD3M_all.LD3Mflag[cnt-1] == 2)
    // { 
    //     // 加速度计数器加 1
    //     a_cnt++;
    //     // 根据加速度计数器的值更新目标位置
    //     if(a_cnt < 500)//500*0.5ms=250ms
    //     {
    //         // 目标位置为当前位置加上加速度计数器乘以2的值，目标位置对应对象字典索引0x607A
    //         outputLD3M->TargetPos = cur_pos + a_cnt*2;  
    //     }
    //     else
    //     {
    //         // 更新LD3M从站状态标志为 3
    //         LD3M_all.LD3Mflag[cnt-1] = 3;
    //         // 打印加速度过程结束的信息
    //         printf("acc over\n");
    //     }

    //     // 设置控制字为 0x1F
    //     outputLD3M->ControlWord = 0x1F;   //伺服运行   4-6运行模式相关	与各伺服运行模式相关

    // }

    // // 如果LD3M从站状态标志为 3
    // if(LD3M_all.LD3Mflag[cnt-1] == 3)
    // {
    //     // 设置目标位置为当前位置加上32000的值，目标位置对应对象字典索引0x607A
    //     //outputLD3M->TargetPos = cur_pos + 32000;  
    //     outputLD3M->TargetPos = cur_pos + a_cnt;  
    //     // 设置控制字为 0x1F
    //     outputLD3M->ControlWord = 0x1F;  //伺服运行   4-6运行模式相关	 与各伺服运行模式相关
    // }

    // // 设置目标模式为 0x8
    // outputLD3M->TargetMode = 0x8;//CSP	循环同步位置模式
    // // 发送过程数据
    // ec_send_processdata();



 /*
//EtherCAT状态机
void EtherCAT_ctrl_state(void)
{ 
    switch (outputLD3M->controlword)          //获取当前状态 0x6041状态字   
    {
        case 0:                             //Switch on disabled
            outputLD3M->controlword = 6;      //Shutdown
        break;
        
        case 6:                             //Ready to switch on
            outputLD3M->controlword = 7;      //Disable voltage  +  Quick stop
        break;
        
        case 7:                             //Switched on
            outputLD3M->TargetPos = inputs1->CurrentPosition;       //查询【6064h： Position actual value】来获取电机实际位置反馈
            outputLD3M->controlword = 0x0f;   // Enable operation
        break;
        
        case 0x0f:                          //Operation enabled
            if( OpenReady == 0 )
                OpenReady = 1;
            else
                OpenReady = 0;
        break;
            
        default:
            outputLD3M->controlword = 6;      //Shutdown
        break;        
    }   
}*/
