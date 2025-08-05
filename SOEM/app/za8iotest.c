/*
 * za8iotest.c
 *
 *  Created on: 2025年1月1日
 *      Author: chenlenan
 */
#include "za8iotest.h"

#include "osal.h"
#include "ethercat.h"

#include "lcd.h"
extern char IOmap[4096];

extern int dorun;
ZA8IO_PDO_Output *output_ZA8IO;
ZA8IO_PDO_Input *input_ZA8IO;


void za8iotest(char *ifname)
{
	int cnt, i, j;

	printf("Starting Servo test\n");

	/* initialise SOEM, bind socket to ifname */
	if (ec_init(ifname))
	{
		printf("ec_init on %s succeeded.\n",ifname);
		/* find and auto-config slaves */
		if ( ec_config_init(FALSE) > 0 )
		{
			dorun = 0;
			printf("%d slaves found and configured.\n",ec_slavecount);

			/* read indevidual slave state and store in ec_slave[] */
			ec_readstate();


			ec_config_map(&IOmap);
			/* configure DC options for every DC capable slave found in the list */
			ec_configdc();
			ec_dcsync0(1, TRUE, 1000000, 250000); // SYNC0 on slave 1


			/* wait for all slaves to reach SAFE_OP state */
			ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
			do
			{
				ec_send_processdata();
				ec_receive_processdata(EC_TIMEOUTRET);
				ec_statecheck(0, EC_STATE_SAFE_OP, 50000);
				ec_statecheck(1, EC_STATE_SAFE_OP, 50000);
			}
			while ((ec_slave[0].state != EC_STATE_SAFE_OP)&&(ec_slave[1].state != EC_STATE_SAFE_OP));
			printf("Slave 0 State=0x%04x\r\n",ec_slave[0].state);
			printf("Slave 1 State=0x%04x\r\n",ec_slave[1].state);

			osal_usleep(100000);


			/* Print som information on the mapped network */
			for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
			{
				printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
											cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
											ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
				printf(" Configured address: %x\n", ec_slave[cnt].configadr);
				printf(" Outputs address: %x\n", ec_slave[cnt].outputs);
				printf(" Inputs address: %x\n", ec_slave[cnt].inputs);

				for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
				{
					printf(" FMMU%1d Ls:%x Ll:%4d Lsb:%d Leb:%d Ps:%x Psb:%d Ty:%x Act:%x\n", j,
									(int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
									ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
									ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
				}
				printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);

			}

			printf("Request operational state for all slaves\n");
			ec_slave[0].state = EC_STATE_OPERATIONAL;
			/* send one valid process data to make outputs in slaves happy*/
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);


			/* request OP state for all slaves */
			ec_writestate(0);
			/* wait for all slaves to reach OP state */
			ec_statecheck(0, EC_STATE_OPERATIONAL,  5 * EC_TIMEOUTSTATE);
			if (ec_slave[0].state == EC_STATE_OPERATIONAL )
			{
				printf("Operational state reached for all slaves.\n");
				dorun = 1;/* activate cyclic process data */
				output_ZA8IO = (ZA8IO_PDO_Output *)ec_slave[1].outputs;
				input_ZA8IO  = (ZA8IO_PDO_Input *)ec_slave[1].inputs;
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

		}
		else
		{
			printf("No slaves found!\n");
			lcd_show_string(200, 150, 300, 16, 16, "za8iotest: No slaves found!\n", BLACK);
		}

	}
	else
	{
		printf("No socket connection on %s\nExcecute as root\n",ifname);
	}
}


void za8io_loop(void)
{
	int static ioflag = 0;
	ec_receive_processdata(EC_TIMEOUTRET);

	ioflag++;
	if(ioflag < 500)
	{
		output_ZA8IO->var3101 = 0xf0;
	}
	else if(ioflag > 1000)
	{
		ioflag = 0;
	}
	else
	{
		output_ZA8IO->var3101 = 0x0f;
	}

	ec_send_processdata();

}


