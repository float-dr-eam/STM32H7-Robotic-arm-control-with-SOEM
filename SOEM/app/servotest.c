/*
 * servotest.c
 *
 *  Created on: 2024年12月9日
 *      Author: chenlenan
 */
#include "servotest.h"
#include "osal.h"
#include "ethercat.h"
#include "lcd.h"
extern char IOmap[4096];

extern int dorun;
extern int32 cur_pos;
extern uint8 cur_mode;
PDO_Output *outputs1;
PDO_Input *inputs1;
uint8 flag;

int Servosetup(uint16 slave)
{
	int retval;
	uint16 u16val;
	uint8  u8val;
	uint32 u32val;
	retval = 0;

	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u16val = 0x1600;
	retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
	u8val = 1;
	retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);


	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u32val = 0x60400010;
	retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x607A0020;
	retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60600008;
	retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u8val = 3;
	retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);


	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u16val = 0x1a00;
	retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
	u8val = 1;
	retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u32val = 0x60410010;
	retval += ec_SDOwrite(slave, 0x1A00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60640020;
	retval += ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x606C0020;
	retval += ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x603F0010;
	retval += ec_SDOwrite(slave, 0x1A00, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60610008;
	retval += ec_SDOwrite(slave, 0x1A00, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u8val = 5;
	retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);


	return 1;
}

void servotest(char *ifname)
{
	int cnt, i, j;

	printf("Starting Servo test\n");

	/* initialise SOEM, bind socket to ifname */
	if (ec_init(ifname))
	{
		printf("ec_init on %s succeeded.\n",ifname);
		/*char t[100];
		sprintf(t,"ec_init on %s succeeded.\n" ,ifname);
		lcd_show_string(100, 90, 200, 16, 16, t, BLUE);*/
		/* find and auto-config slaves */
		if ( ec_config_init(FALSE) > 0 )
		{
			dorun = 0;
			flag = 1;
			printf("%d slaves found and configured.\n",ec_slavecount);

			/* read indevidual slave state and store in ec_slave[] */
			ec_readstate();
			for(cnt = 1; cnt <= ec_slavecount ; cnt++)
			{
				printf("ec_slave[%d].eep_man = %x\n",cnt,ec_slave[cnt].eep_man);
				printf("ec_slave[%d].eep_id = %x\n",cnt,ec_slave[cnt].eep_id);
				/* check for EL2004 or EL2008 */
				if((ec_slave[cnt].eep_id == 0x00004321) || (ec_slave[cnt].eep_id == 0x00008100))
				{
					ec_slave[cnt].PO2SOconfig = &Servosetup;
				}
			}


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
			int retval;
			uint16 u16val;
			uint8  u8val;
			u8val = 8;
			retval += ec_SDOwrite(1, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
			u16val = 0x03E8;//峰值电流1A
			retval += ec_SDOwrite(1, 0x2000, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
			//		u16val = 10000;
			//		retval += ec_SDOwrite(1, 0x2001, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
			u16val = 0x80;
			retval += ec_SDOwrite(1, 0x6040, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);


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
				outputs1 = (PDO_Output *)ec_slave[1].outputs;
				inputs1  = (PDO_Input *)ec_slave[1].inputs;
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
			lcd_show_string(200, 110, 300, 16, 16, "servotest; No slaves found!\n", BLUE);
		}

	}
	else
	{
		printf("No socket connection on %s\nExcecute as root\n",ifname);
	}
}

void ecat_loop(void)
{
	ec_send_processdata();
	ec_receive_processdata(EC_TIMEOUTRET);

	uint16 cur_status;
	cur_status = inputs1->StatusWord;//0x6041

	if(flag == 1){
		if((cur_status & 0x004f) == 0x0040)
		{
			outputs1->ControlWord = 0x06;//0x6040
			//printf("0x06\n");
		}
		else if((cur_status & 0x006f) == 0x0021)
		{
			outputs1->ControlWord = 0x07;
			//printf("0x07\n");
		}
		else if((cur_status & 0x006f) == 0x023)
		{
			outputs1->ControlWord = 0x0F;
			//printf("0x0F\n");
		}
		else if((cur_status & 0x006f) == 0x0027)
		{
			outputs1->ControlWord = 0x1F;
			//printf("0x1F\n");
			cur_pos = inputs1->CurrentPosition;//0x6064
			outputs1->TargetPos = cur_pos;//0x607A

			flag = 2;
		}
	}

	if(flag == 2){
		cur_pos += 50;
		outputs1->TargetPos = cur_pos;//0x607A
		outputs1->ControlWord = 0x1F;
	}

	outputs1->TargetMode = 0x8;
}





