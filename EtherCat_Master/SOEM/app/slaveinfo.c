/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : slaveinfo [ifname] [-sdo] [-map]
 * Ifname is NIC interface, f.e. eth0.
 * Optional -sdo to display CoE object dictionary.
 * Optional -map to display slave PDO mapping
 *
 * This shows the configured slave data.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"
#include "slaveinfo.h"


#include "lcd.h"

extern char IOmap[4096];//数据映射表，把从站的输入输出数据和主站内存里的逻辑地址关联起来。
ec_ODlistt ODlist;//对象字典列表
ec_OElistt OElist;//对象字典条目列表
boolean printSDO = FALSE;
boolean printMAP = FALSE;
char usdo[128]; // CoE SDO read buffer
char hstr[1024];// CoE SDO string buffer

char* dtype2string(uint16 dtype)//将数据类型转换为字符串
{
    switch(dtype)
    {
        case ECT_BOOLEAN:
            sprintf(hstr, "BOOLEAN");
            break;
        case ECT_INTEGER8:
            sprintf(hstr, "INTEGER8");
            break;
        case ECT_INTEGER16:
            sprintf(hstr, "INTEGER16");
            break;
        case ECT_INTEGER32:
            sprintf(hstr, "INTEGER32");
            break;
        case ECT_INTEGER24:
            sprintf(hstr, "INTEGER24");
            break;
        case ECT_INTEGER64:
            sprintf(hstr, "INTEGER64");
            break;
        case ECT_UNSIGNED8:
            sprintf(hstr, "UNSIGNED8");
            break;
        case ECT_UNSIGNED16:
            sprintf(hstr, "UNSIGNED16");
            break;
        case ECT_UNSIGNED32:
            sprintf(hstr, "UNSIGNED32");
            break;
        case ECT_UNSIGNED24:
            sprintf(hstr, "UNSIGNED24");
            break;
        case ECT_UNSIGNED64:
            sprintf(hstr, "UNSIGNED64");
            break;
        case ECT_REAL32:
            sprintf(hstr, "REAL32");
            break;
        case ECT_REAL64:
            sprintf(hstr, "REAL64");
            break;
        case ECT_BIT1:
            sprintf(hstr, "BIT1");
            break;
        case ECT_BIT2:
            sprintf(hstr, "BIT2");
            break;
        case ECT_BIT3:
            sprintf(hstr, "BIT3");
            break;
        case ECT_BIT4:
            sprintf(hstr, "BIT4");
            break;
        case ECT_BIT5:
            sprintf(hstr, "BIT5");
            break;
        case ECT_BIT6:
            sprintf(hstr, "BIT6");
            break;
        case ECT_BIT7:
            sprintf(hstr, "BIT7");
            break;
        case ECT_BIT8:
            sprintf(hstr, "BIT8");
            break;
        case ECT_VISIBLE_STRING:
            sprintf(hstr, "VISIBLE_STRING");
            break;
        case ECT_OCTET_STRING:
            sprintf(hstr, "OCTET_STRING");
            break;
        default:
            sprintf(hstr, "Type 0x%4.4X", dtype);
    }
    return hstr;
}

char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype)//将数据转换为字符串
{
   int l = sizeof(usdo) - 1, i;
   uint8 *u8;
   int8 *i8;
   uint16 *u16;
   int16 *i16;
   uint32 *u32;
   int32 *i32;
   uint64 *u64;
   int64 *i64;
   float *sr;
   double *dr;
   char es[32];

   memset(&usdo, 0, 128);
   ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
   if (EcatError)
   {
      return ec_elist2string();
   }
   else
   {
      switch(dtype)
      {
         case ECT_BOOLEAN:
            u8 = (uint8*) &usdo[0];
            if (*u8) sprintf(hstr, "TRUE");
             else sprintf(hstr, "FALSE");
            break;
         case ECT_INTEGER8:
            i8 = (int8*) &usdo[0];
            sprintf(hstr, "0x%2.2x %d", *i8, *i8);
            break;
         case ECT_INTEGER16:
            i16 = (int16*) &usdo[0];
            sprintf(hstr, "0x%4.4x %d", *i16, *i16);
            break;
         case ECT_INTEGER32:
         case ECT_INTEGER24:
            i32 = (int32*) &usdo[0];
            sprintf(hstr, "0x%8.8x %d", *i32, *i32);
            break;
         case ECT_INTEGER64:
            i64 = (int64*) &usdo[0];
            sprintf(hstr, "0x%16.16"PRIx64" %"PRId64, *i64, *i64);
            break;
         case ECT_UNSIGNED8:
            u8 = (uint8*) &usdo[0];
            sprintf(hstr, "0x%2.2x %u", *u8, *u8);
            break;
         case ECT_UNSIGNED16:
            u16 = (uint16*) &usdo[0];
            sprintf(hstr, "0x%4.4x %u", *u16, *u16);
            break;
         case ECT_UNSIGNED32:
         case ECT_UNSIGNED24:
            u32 = (uint32*) &usdo[0];
            sprintf(hstr, "0x%8.8x %u", *u32, *u32);
            break;
         case ECT_UNSIGNED64:
            u64 = (uint64*) &usdo[0];
            sprintf(hstr, "0x%16.16"PRIx64" %"PRIu64, *u64, *u64);
            break;
         case ECT_REAL32:
            sr = (float*) &usdo[0];
            sprintf(hstr, "%f", *sr);
            break;
         case ECT_REAL64:
            dr = (double*) &usdo[0];
            sprintf(hstr, "%f", *dr);
            break;
         case ECT_BIT1:
         case ECT_BIT2:
         case ECT_BIT3:
         case ECT_BIT4:
         case ECT_BIT5:
         case ECT_BIT6:
         case ECT_BIT7:
         case ECT_BIT8:
            u8 = (uint8*) &usdo[0];
            sprintf(hstr, "0x%x", *u8);
            break;
         case ECT_VISIBLE_STRING:
            strcpy(hstr, usdo);
            break;
         case ECT_OCTET_STRING:
            hstr[0] = 0x00;
            for (i = 0 ; i < l ; i++)
            {
               sprintf(es, "0x%2.2x ", usdo[i]);
               strcat( hstr, es);
            }
            break;
         default:
            sprintf(hstr, "Unknown type");
      }
      return hstr;
   }
}
/**
 * @brief 读取 PDO 分配结构，并根据从站编号、PDO 分配地址、输出数据的偏移量以及当前的字节偏移量来完成映射操作。
 * 
 * 该函数通过 SDO 协议从指定从站读取 PDO 分配信息，包括 PDO 的数量、每个 PDO 的索引和子索引，
 * 以及每个子索引对应的 SDO 信息。同时，会打印出映射信息，并返回找到的 PDO 的总位长度。
 * 
 * @param[in] slave       从站编号，指定要操作的 EtherCAT 从站。
 * @param[in] PDOassign   PDO 分配地址，用于标识 PDO 分配对象字典的索引。
 * @param[in] mapoffset   输出数据在 IO 映射表中的偏移量，单位为字节。
 * @param[in] bitoffset   当前的字节偏移量，单位为位。
 * @return 找到的 PDO 的总位长度，单位为位。
 */
int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset)
{
    // 定义循环变量和存储读取数据的变量
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    // 存储子索引计数
    uint8 subcnt;
    // 工作计数器，用于记录 SDO 操作的结果
    int wkc, bsize = 0, rdl;
    // 存储 32 位读取数据
    int32 rdat2;
    // 存储 SDO 的位长度
    uint8 bitlen, obj_subidx;
    // 存储对象字典的索引
    uint16 obj_idx;
    // 计算绝对偏移量（字节）和绝对位偏移量
    int abs_offset, abs_bit;

    // 初始化读取数据长度和数据
    rdl = sizeof(rdat); 
    rdat = 0;
    /* read PDO assign subindex 0 ( = number of PDO's) */
    // 通过 SDO 协议读取 PDO 分配对象字典的子索引 0，获取 PDO 的数量
    wkc = ec_SDOread(slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
    // 将读取的数据从网络字节序转换为主机字节序
    rdat = etohs(rdat);
    /* positive result from slave ? */
    // 检查 SDO 操作是否成功且 PDO 数量大于 0
    if ((wkc > 0) && (rdat > 0))
    {
        // 获取可用的子索引数量，即 PDO 的数量
        nidx = rdat;
        // 初始化找到的 PDO 的总位长度为 0
        bsize = 0;
        /* read all PDO's */
        // 遍历所有 PDO
        for (idxloop = 1; idxloop <= nidx; idxloop++)
        {
            // 初始化读取数据长度和数据
            rdl = sizeof(rdat); 
            rdat = 0;
            /* read PDO assign */
            // 通过 SDO 协议读取当前 PDO 分配对象字典的子索引，获取 PDO 的索引
            wkc = ec_SDOread(slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
            /* result is index of PDO */
            // 将读取的数据从网络字节序转换为主机字节序，得到 PDO 的索引
            idx = etohs(rdat);
            // 检查 PDO 索引是否大于 0
            if (idx > 0)
            {
                // 初始化读取数据长度和子索引计数
                rdl = sizeof(subcnt); 
                subcnt = 0;
                /* read number of subindexes of PDO */
                // 通过 SDO 协议读取当前 PDO 对象字典的子索引 0，获取子索引的数量
                wkc = ec_SDOread(slave, idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
                // 将子索引计数赋值给 subidx
                subidx = subcnt;
                /* for each subindex */
                // 遍历当前 PDO 的所有子索引
                for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
                {
                    // 初始化读取数据长度和 32 位数据
                    rdl = sizeof(rdat2); 
                    rdat2 = 0;
                    /* read SDO that is mapped in PDO */
                    // 通过 SDO 协议读取当前 PDO 子索引对应的 SDO 数据
                    wkc = ec_SDOread(slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
                    // 将读取的数据从网络字节序转换为主机字节序
                    rdat2 = etohl(rdat2);
                    /* extract bitlength of SDO */
                    // 提取 SDO 的位长度
                    bitlen = LO_BYTE(rdat2);
                    // 累加找到的 PDO 的总位长度
                    bsize += bitlen;
                    // 提取对象字典的索引
                    obj_idx = (uint16)(rdat2 >> 16);
                    // 提取对象字典的子索引
                    obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
                    // 计算绝对偏移量（字节）
                    abs_offset = mapoffset + (bitoffset / 8);
                    // 计算绝对位偏移量
                    abs_bit = bitoffset % 8;
                    // 设置对象字典列表的从站编号
                    ODlist.Slave = slave;
                    // 设置对象字典列表的索引
                    ODlist.Index[0] = obj_idx;
                    // 初始化对象字典条目列表的条目数量为 0
                    OElist.Entries = 0;
                    // 初始化工作计数器为 0
                    wkc = 0;
                    /* read object entry from dictionary if not a filler (0x0000:0x00) */
                    // 如果对象字典的索引和子索引不为 0，则从对象字典中读取单个条目
                    if(obj_idx || obj_subidx)
                        wkc = ec_readOEsingle(0, obj_subidx, &ODlist, &OElist);
                    // 打印映射信息，包括绝对偏移量、对象字典的索引和子索引、位长度
                    printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                    // 检查是否成功读取对象字典条目
                    if((wkc > 0) && OElist.Entries)
                    {
                        // 打印数据类型和条目名称
                        printf(" %-12s %s\n", dtype2string(OElist.DataType[obj_subidx]), OElist.Name[obj_subidx]);
                    }
                    else
                        // 若未成功读取，换行
                        printf("\n");
                    // 更新当前的字节偏移量
                    bitoffset += bitlen;
                };
            };
        };
    };
    /* return total found bitlength (PDO) */
    // 返回找到的 PDO 的总位长度
    return bsize;
}


int si_map_sdo(int slave)
{
    int wkc, rdl;
    int retVal = 0;
    uint8 nSM, iSM, tSM;
    int Tsize, outputs_bo, inputs_bo;
    uint8 SMt_bug_add;

    printf("PDO mapping according to CoE :\n");// 根据CoE进行PDO映射
    SMt_bug_add = 0;
    outputs_bo = 0;
    inputs_bo = 0;
    rdl = sizeof(nSM); nSM = 0;
    /* read SyncManager Communication Type object count */// 读取同步管理器通信类型对象计数
    wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
    /* positive result from slave ? */
    if ((wkc > 0) && (nSM > 2))
    {
        /* make nSM equal to number of defined SM */
        nSM--;
        /* limit to maximum number of SM defined, if true the slave can't be configured */
        if (nSM > EC_MAXSM)
            nSM = EC_MAXSM;
        /* iterate for every SM type defined */
        for (iSM = 2 ; iSM <= nSM ; iSM++)
        {
            rdl = sizeof(tSM); tSM = 0;
            /* read SyncManager Communication Type */// 读取同步管理器通信类型
            wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
            if (wkc > 0)
            {
                if((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave! // SM2具有类型2 == 邮箱输出，这是从站的错误！
                {
                    SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
                    printf("Activated SM type workaround, possible incorrect mapping.\n");
                }
                if(tSM)
                    tSM += SMt_bug_add; // only add if SMt > 0//错误纠正

                if (tSM == 3) // outputs  iSM=2
                {
                    /* read the assign RXPDO */// 读取分配的RXPDO
                    printf("  SM%1d outputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].outputs - (uint8 *)&IOmap[0]), outputs_bo );
                    outputs_bo += Tsize;
                }
                if (tSM == 4) // inputs   iSM=3
                {
                    /* read the assign TXPDO */// 读取分配的TXPDO
                    printf("  SM%1d inputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    //计算从站输入数据在 IOmap 中的偏移量
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].inputs - (uint8 *)&IOmap[0]), inputs_bo );
                    inputs_bo += Tsize;
                }
            }
        }
    }

    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}


int si_map_sii(int slave)
{
    int retVal = 0;
    int Tsize, outputs_bo, inputs_bo;

    printf("PDO mapping according to SII :\n");// 根据SII进行PDO映射

    outputs_bo = 0;
    inputs_bo = 0;
    /* read the assign RXPDOs */
    Tsize = si_siiPDO(slave, 1, (int)(ec_slave[slave].outputs - (uint8*)&IOmap), outputs_bo );
    outputs_bo += Tsize;
    /* read the assign TXPDOs */
    Tsize = si_siiPDO(slave, 0, (int)(ec_slave[slave].inputs - (uint8*)&IOmap), inputs_bo );
    inputs_bo += Tsize;
    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}


int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset)// 读取SII PDO
{
    uint16 a , w, c, e, er, Size;
    uint8 eectl;
    uint16 obj_idx;
    uint8 obj_subidx;
    uint8 obj_name;
    uint8 obj_datatype;
    uint8 bitlen;
    int totalsize;
    ec_eepromPDOt eepPDO;
    ec_eepromPDOt *PDO;
    int abs_offset, abs_bit;
    char str_name[EC_MAXNAME + 1];

    eectl = ec_slave[slave].eep_pdi;
    Size = 0;
    totalsize = 0;
    PDO = &eepPDO;
    PDO->nPDO = 0;
    PDO->Length = 0;
    PDO->Index[1] = 0;
    for (c = 0 ; c < EC_MAXSM ; c++) PDO->SMbitsize[c] = 0;
    if (t > 1)
        t = 1;
    PDO->Startpos = ec_siifind(slave, ECT_SII_PDO + t);
    if (PDO->Startpos > 0)
    {
        a = PDO->Startpos;
        w = ec_siigetbyte(slave, a++);
        w += (ec_siigetbyte(slave, a++) << 8);
        PDO->Length = w;
        c = 1;
        /* traverse through all PDOs */
        do
        {
            PDO->nPDO++;
            PDO->Index[PDO->nPDO] = ec_siigetbyte(slave, a++);
            PDO->Index[PDO->nPDO] += (ec_siigetbyte(slave, a++) << 8);
            PDO->BitSize[PDO->nPDO] = 0;
            c++;
            /* number of entries in PDO */
            e = ec_siigetbyte(slave, a++);
            PDO->SyncM[PDO->nPDO] = ec_siigetbyte(slave, a++);
            a++;
            obj_name = ec_siigetbyte(slave, a++);
            a += 2;
            c += 2;
            if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
            {
                str_name[0] = 0;
                if(obj_name)
                  ec_siistring(str_name, slave, obj_name);
                if (t)
                  printf("  SM%1d RXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                else
                  printf("  SM%1d TXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                printf("     addr b   index: sub bitl data_type    name\n");
                /* read all entries defined in PDO */
                for (er = 1; er <= e; er++)
                {
                    c += 4;
                    obj_idx = ec_siigetbyte(slave, a++);
                    obj_idx += (ec_siigetbyte(slave, a++) << 8);
                    obj_subidx = ec_siigetbyte(slave, a++);
                    obj_name = ec_siigetbyte(slave, a++);
                    obj_datatype = ec_siigetbyte(slave, a++);
                    bitlen = ec_siigetbyte(slave, a++);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;

                    PDO->BitSize[PDO->nPDO] += bitlen;
                    a += 2;

                    /* skip entry if filler (0x0000:0x00) */
                    if(obj_idx || obj_subidx)
                    {
                       str_name[0] = 0;
                       if(obj_name)
                          ec_siistring(str_name, slave, obj_name);

                       printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                       printf(" %-12s %s\n", dtype2string(obj_datatype), str_name);
                    }
                    bitoffset += bitlen;
                    totalsize += bitlen;
                }
                PDO->SMbitsize[ PDO->SyncM[PDO->nPDO] ] += PDO->BitSize[PDO->nPDO];
                Size += PDO->BitSize[PDO->nPDO];
                c++;
            }
            else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
            {
                c += 4 * e;
                a += 8 * e;
                c++;
            }
            if (PDO->nPDO >= (EC_MAXEEPDO - 1)) c = PDO->Length; /* limit number of PDO entries in buffer */
        }
        while (c < PDO->Length);
    }
    if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
    return totalsize;
}

void si_sdo(int cnt)// 读取SDO，打印对象字典
{
    int i, j;

    ODlist.Entries = 0;
    memset(&ODlist, 0, sizeof(ODlist));
    if( ec_readODlist(cnt, &ODlist))
    {
        printf(" CoE Object Description found, %d entries.\n",ODlist.Entries);  //打印ODlist.Entries的值
        for( i = 0 ; i < ODlist.Entries ; i++)  //遍历ODlist.Entries的值
        {
            ec_readODdescription(i, &ODlist);//读取对象描述
            while(EcatError) printf("%s", ec_elist2string());
            printf(" Index: %4.4x Datatype: %4.4x Objectcode: %2.2x Name: %s\n",
                ODlist.Index[i], ODlist.DataType[i], ODlist.ObjectCode[i], ODlist.Name[i]); //打印ODlist.Index[i]的值：索引
            memset(&OElist, 0, sizeof(OElist));
            ec_readOE(i, &ODlist, &OElist);
            while(EcatError) printf("%s", ec_elist2string());
            for( j = 0 ; j < ODlist.MaxSub[i]+1 ; j++) //遍历ODlist.MaxSub[i]的值：子索引
            {
                if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0))
                {
                    printf("  Sub: %2.2x Datatype: %4.4x Bitlength: %4.4x Obj.access: %4.4x Name: %s\n",
                        j, OElist.DataType[j], OElist.BitLength[j], OElist.ObjAccess[j], OElist.Name[j]);
                    if ((OElist.ObjAccess[j] & 0x0007))
                    {
                        printf("          Value :%s\n", SDO2string(cnt, ODlist.Index[i], j, OElist.DataType[j]));
                    }
                }
            }
        }
    }
    else
    {
        while(EcatError) printf("%s", ec_elist2string());
    }
}

//void slaveinfo(char *ifname)
//{
//	printSDO = TRUE;
//	printMAP = TRUE;
//	
//	
//   int cnt, i, j, nSM;
//    uint16 ssigen;
//    int expectedWKC;

//   printf("Starting slaveinfo\n");

//   /* initialise SOEM, bind socket to ifname */
//   if (ec_init(ifname))
//   {
//      printf("ec_init on %s succeeded.\n",ifname);
//      /* find and auto-config slaves */
//      if ( ec_config_init(FALSE) > 0 )
//      {
//		  printf("%d slaves found and configured.\n",ec_slavecount);
//		  ec_config_map(&IOmap);
//         ec_configdc();
//         while(EcatError) printf("%s", ec_elist2string());
//         printf("%d slaves found and configured.\n",ec_slavecount);
//         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
//         printf("Calculated workcounter %d\n", expectedWKC);
//         /* wait for all slaves to reach SAFE_OP state */
//         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 3);
//         if (ec_slave[0].state != EC_STATE_SAFE_OP )
//         {
//            printf("Not all slaves reached safe operational state.\n");
//            ec_readstate();
//            for(i = 1; i<=ec_slavecount ; i++)
//            {
//               if(ec_slave[i].state != EC_STATE_SAFE_OP)
//               {
//                  printf("Slave %d State=%2x StatusCode=%4x : %s\n",
//                     i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
//               }
//            }
//         }


//         ec_readstate();
//         for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
//         {
//            printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
//                  cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
//                  ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
//            if (ec_slave[cnt].hasdc) printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
//            printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0 ,
//                                         (ec_slave[cnt].activeports & 0x02) > 0 ,
//                                         (ec_slave[cnt].activeports & 0x04) > 0 ,
//                                         (ec_slave[cnt].activeports & 0x08) > 0 );
//            printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);
//            printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);
//            for(nSM = 0 ; nSM < EC_MAXSM ; nSM++)
//            {
//               if(ec_slave[cnt].SM[nSM].StartAddr > 0)
//                  printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n",nSM, etohs(ec_slave[cnt].SM[nSM].StartAddr), etohs(ec_slave[cnt].SM[nSM].SMlength),
//                         etohl(ec_slave[cnt].SM[nSM].SMflags), ec_slave[cnt].SMtype[nSM]);
//            }
//            for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
//            {
//               printf(" FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
//                       etohl(ec_slave[cnt].FMMU[j].LogStart), etohs(ec_slave[cnt].FMMU[j].LogLength), ec_slave[cnt].FMMU[j].LogStartbit,
//                       ec_slave[cnt].FMMU[j].LogEndbit, etohs(ec_slave[cnt].FMMU[j].PhysStart), ec_slave[cnt].FMMU[j].PhysStartBit,
//                       ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
//            }
//            printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
//                     ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
//            printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl, ec_slave[cnt].mbx_proto);
//            ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
//            /* SII general section */
//            if (ssigen)
//            {
//               ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
//               ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
//               ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
//               ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
//               if((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
//               {
//                  ec_slave[cnt].blockLRW = 1;
//                  ec_slave[0].blockLRW++;
//               }
//               ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
//               ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
//               ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
//            }
//            printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
//                    ec_slave[cnt].CoEdetails, ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
//            printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
//                    ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);
//            if ((ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE) && printSDO)
//                    si_sdo(cnt);
//                if(printMAP)
//            {
//                    if (ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE)
//                        si_map_sdo(cnt);
//                    else
//                        si_map_sii(cnt);
//            }
//         }
//      }
//      else
//      {
//         printf("No slaves found!\n");
//      }
//      printf("End slaveinfo, close socket\n");
//      /* stop SOEM, close socket */
//      ec_close();
//   }
//   else
//   {
//      printf("No socket connection on %s\nExcecute as root\n",ifname);
//   }
//}

void slaveinfo(char *ifname)
{
    printSDO = TRUE;  // 设置标志位以打印SDO信息
    printMAP = TRUE;  // 设置标志位以打印映射信息

    int cnt, i, j, nSM;  // 定义循环计数器和其他变量
    uint16 ssigen;  // 存储SII（Slave Information Interface）通用部分的索引
    int expectedWKC;  // 预期的工作计数器值

    printf("Starting slaveinfo\n");  // 打印开始信息

    /* 初始化SOEM（Simple Open EtherCAT Master），并将套接字绑定到指定接口 */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);  // 初始化成功提示
        /* 查找并自动配置从站 */
        if (ec_config_init(FALSE) > 0)  // 如果找到并配置了一个或多个从站
        {
            printf("%d slaves found and configured.\n", ec_slavecount);              // 打印找到并配置的从站数量
			
            ec_config_map(&IOmap);  // 映射I/O数据
            ec_configdc();          // 配置分布式时钟（DC）
			
            while(EcatError) printf("%s", ec_elist2string());                        // 打印任何错误信息
            printf("%d slaves found and configured.\n", ec_slavecount);              // 再次确认从站数量
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;      // 计算预期的工作计数器值
            printf("Calculated workcounter %d\n", expectedWKC);                      // 打印计算出的工作计数器值
            /* 等待所有从站进入SAFE_OP状态 */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3);
            if (ec_slave[0].state != EC_STATE_SAFE_OP)                               // 检查是否所有从站都进入了SAFE_OP状态
            {
                printf("Not all slaves reached safe operational state.\n");
                ec_readstate();                                                      // 读取从站状态
                for(i = 1; i <= ec_slavecount ; i++)  // 遍历所有从站
                {
                    if(ec_slave[i].state != EC_STATE_SAFE_OP)  // 如果某个从站没有进入SAFE_OP状态
                    {
                        printf("Slave %d State=%2x StatusCode=%4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            ec_readstate();                                                          // 再次读取从站状态
            for(cnt = 1; cnt <= ec_slavecount; cnt++)  // 遍历所有从站
            {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                if (ec_slave[cnt].hasdc) printf(" DCParentport:%d\n", ec_slave[cnt].parentport);  // 打印是否有分布式时钟以及父端口
                printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0,
                                                       (ec_slave[cnt].activeports & 0x02) > 0,
                                                       (ec_slave[cnt].activeports & 0x04) > 0,
                                                       (ec_slave[cnt].activeports & 0x08) > 0 );
                printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);                  // 打印配置地址
                printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);  // 打印制造商、ID和版本号
                for(nSM = 0; nSM < EC_MAXSM; nSM++)      // 遍历所有状态机（SM）
                {
                    if(ec_slave[cnt].SM[nSM].StartAddr > 0)
                        printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n", nSM, etohs(ec_slave[cnt].SM[nSM].StartAddr), etohs(ec_slave[cnt].SM[nSM].SMlength),
                               etohl(ec_slave[cnt].SM[nSM].SMflags), ec_slave[cnt].SMtype[nSM]);
                }
                for(j = 0; j < ec_slave[cnt].FMMUunused; j++)   // 遍历所有FMMU（Function Memory Mapping Unit）
                {
                    printf(" FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
                           etohl(ec_slave[cnt].FMMU[j].LogStart), etohs(ec_slave[cnt].FMMU[j].LogLength), ec_slave[cnt].FMMU[j].LogStartbit,
                           ec_slave[cnt].FMMU[j].LogEndbit, etohs(ec_slave[cnt].FMMU[j].PhysStart), ec_slave[cnt].FMMU[j].PhysStartBit,
                           ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
                }
                printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
                       ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
                printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl, ec_slave[cnt].mbx_proto);// 打印MBX（Message Box）长度和协议
                ssigen = ec_siifind(cnt, ECT_SII_GENERAL);  // 查找SII通用部分的索引
                /* SII通用部分 */
                if (ssigen)
                {
                    ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);  // 获取CoE细节
                    ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);  // 获取FoE细节
                    ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);  // 获取EoE细节
                    ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);  // 获取SoE细节
                    if((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
                    {
                        ec_slave[cnt].blockLRW = 1;  // 标记仅使用LRD/LWR
                        ec_slave[0].blockLRW++;      // 全局计数增加
                    }
                    ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);        // 获取E-Bus电流低字节
                    ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;  // 获取E-Bus电流高字节
                    ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;                 // 累加全局E-Bus电流
                }
                printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
                       ec_slave[cnt].CoEdetails, ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
                printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
                       ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);
                if ((ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE) && printSDO)
                    si_sdo(cnt);  // 打印SDO信息
                if(printMAP)
                {
                    if (ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE)
                        si_map_sdo(cnt);  // 通过SDO协议打印映射信息
                    else
                        si_map_sii(cnt);  // 通过SII协议打印映射信息
                }
            }
            char str[100];
            sprintf(str, "%d slaves found and configured.\n", ec_slavecount);  // 打印找到的从站数量
            lcd_show_string(200, 90, 200, 16, 16, str, BLACK);  // 在LCD上显示找到的从站数量        
        }
        else
        {
            printf("No slaves found!\n");  // 没有找到从站的情况
			lcd_show_string(200, 90, 200, 16, 16, "slaveinfo:No slaves found!\n", BLACK);
        }
        printf("End slaveinfo, close socket\n");  // 结束提示
        /* 停止SOEM，关闭套接字 */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);  // 初始化失败提示
    }
}

