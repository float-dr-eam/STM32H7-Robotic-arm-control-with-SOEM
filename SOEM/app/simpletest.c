#include "simpletest.h"
#include "ethercat.h"
#include <stdio.h>


#include "lcd.h"

#define EC_TIMEOUTMON 500

extern char IOmap[4096];
boolean needlf;
boolean inOP;
int wkc;
int expectedWKC;
uint8 currentgroup = 0;

void simpletest(char *ifname)
{
	int i, j, oloop, iloop, chk;

	needlf = FALSE;
	inOP = FALSE;

	printf("Starting simple test\n");

	/* initialise SOEM, bind socket to ifname */
	if (ec_init(ifname))
	{
		printf("ec_init on %s succeeded.\n",ifname);
		/* find and auto-config slaves */


		if ( ec_config_init(FALSE) > 0 )
		{
			printf("%d slaves found and configured.\n",ec_slavecount);

			ec_config_map(&IOmap);

			ec_configdc();

			printf("Slaves mapped, state to SAFE_OP.\n");
			/* wait for all slaves to reach SAFE_OP state */
			ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);//等待所有从站进入安全运行状态

			oloop = ec_slave[0].Obytes;	//输出字节数
			if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;	//输出位
			if (oloop > 8) oloop = 8;	//输出字节数限制
			iloop = ec_slave[0].Ibytes;	//输入字节数
			if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;	//输入位
			if (iloop > 8) iloop = 8;	//输入字节数限制

			printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

			printf("Request operational state for all slaves\n");
			expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
			printf("Calculated workcounter %d\n", expectedWKC);
			ec_slave[0].state = EC_STATE_OPERATIONAL;	//设置主站状态为操作状态
			/* send one valid process data to make outputs in slaves happy*/
			ec_send_processdata();                 //发送一个有效的进程数据包
			ec_receive_processdata(EC_TIMEOUTRET); //接收从站的响应
			/* request OP state for all slaves */
			ec_writestate(0);	                   //请求所有从站进入操作状态
			chk = 200;
			/* wait for all slaves to reach OP state */
			do
			{
				ec_send_processdata();	                      //发送一个有效的进程数据包
				ec_receive_processdata(EC_TIMEOUTRET);	      //接收从站的响应
				ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);//等待所有从站进入操作状态
			}
			while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
			if (ec_slave[0].state == EC_STATE_OPERATIONAL )	//检查所有从站是否进入操作状态
			{
				printf("Operational state reached for all slaves.\n");
				inOP = TRUE;
				/* cyclic loop */	//循环发送进程数据
				for(i = 1; i <= 1000; i++)
				{
					ec_send_processdata();
					wkc = ec_receive_processdata(EC_TIMEOUTRET);

					if(wkc >= expectedWKC)
					{
						printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

						for(j = 0 ; j < oloop; j++)
						{
							printf(" %2.2x", *(ec_slave[0].outputs + j));
						}

						printf(" I:");
						for(j = 0 ; j < iloop; j++)
						{
							printf(" %2.2x", *(ec_slave[0].inputs + j));
						}
						printf(" T:%lld\n",ec_DCtime);
						needlf = TRUE;
					}
					osal_usleep(5000);//5ms
				}
				inOP = FALSE;
			}
			else
			{
				printf("Not all slaves reached operational state.\n");
				ec_readstate();
				for(i = 1; i<=ec_slavecount ; i++)
				{
					if(ec_slave[i].state != EC_STATE_OPERATIONAL)
					{
						printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
						i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
					}
				}
			}
			printf("\nRequest init state for all slaves\n");
			ec_slave[0].state = EC_STATE_INIT;
			/* request INIT state for all slaves */
			ec_writestate(0);
		}
		else
		{
			printf("No slaves found!\n");
			lcd_show_string(200, 70, 300, 16, 16, "simpletest: No slaves found!\n", BLACK);
		}
		printf("End simple test, close socket\n");

	}
	else
	{
		printf("No socket connection on %s\nExcecute as root\n",ifname);
	}
}

/**
 * @brief 检查 EtherCAT 从站状态并处理异常情况。
 *
 * 该函数在 EtherCAT 系统处于操作状态时，检查工作计数器和从站状态标志，
 * 若发现从站状态异常，则根据不同的异常状态采取相应的处理措施，
 * 如确认错误、恢复从站配置或重新发现丢失的从站等。
 */
void ecatcheck(void)
{
    int slave;
    // 检查系统是否处于操作状态，并且工作计数器小于预期值或者需要检查从站状态
    if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
    {
        // 如果需要换行，则进行换行并重置标志
        if (needlf)
        {
           needlf = FALSE;
           printf("\n");
        }
        /* one ore more slaves are not responding */
        // 重置检查状态标志
        ec_group[currentgroup].docheckstate = FALSE;
        // 读取所有从站的状态
        ec_readstate();
        // 遍历所有从站
        for (slave = 1; slave <= ec_slavecount; slave++)
        {
           // 检查当前从站是否属于当前组，并且状态不是操作状态
           if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
           {
              // 设置需要检查状态标志
              ec_group[currentgroup].docheckstate = TRUE;
              // 若从站状态为安全操作状态加上错误状态
              if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
              {
                 // 输出错误信息并尝试确认错误
                 printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                 ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                 ec_writestate(slave);
              }
              // 若从站状态为安全操作状态
              else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
              {
                 // 输出警告信息并尝试将从站状态改为操作状态
                 printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                 ec_slave[slave].state = EC_STATE_OPERATIONAL;
                 ec_writestate(slave);
              }
              // 若从站状态大于无状态
              else if(ec_slave[slave].state > EC_STATE_NONE)
              {
                 // 尝试重新配置从站
                 if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                 {
                    // 标记从站未丢失并输出信息
                    ec_slave[slave].islost = FALSE;
                    printf("MESSAGE : slave %d reconfigured\n",slave);
                 }
              }
              // 若从站未标记为丢失
              else if(!ec_slave[slave].islost)
              {
                 /* re-check state */
                 // 重新检查从站状态
                 ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                 // 若从站状态为无状态，则标记从站丢失
                 if (ec_slave[slave].state == EC_STATE_NONE)
                 {
                    ec_slave[slave].islost = TRUE;
                    printf("ERROR : slave %d lost\n",slave);
                 }
              }
           }
           // 若从站标记为丢失
           if (ec_slave[slave].islost)
           {
              // 若从站状态为无状态
              if(ec_slave[slave].state == EC_STATE_NONE)
              {
                 // 尝试恢复丢失的从站
                 if (ec_recover_slave(slave, EC_TIMEOUTMON))
                 {
                    // 标记从站未丢失并输出信息
                    ec_slave[slave].islost = FALSE;
                    printf("MESSAGE : slave %d recovered\n",slave);
                 }
              }
              else
              {
                 // 标记从站未丢失并输出信息
                 ec_slave[slave].islost = FALSE;
                 printf("MESSAGE : slave %d found\n",slave);
              }
           }
        }
        // 若所有从站都恢复到操作状态，则输出信息
        if(!ec_group[currentgroup].docheckstate)
           printf("OK : all slaves resumed OPERATIONAL.\n");
    }
}
