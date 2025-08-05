/*
 * DcSync.c
 *
 *  Created on: 2025年1月1日
 *      Author: chenlenan
 */
#include "DcSync.h"
#include "tim.h"
int64 integral=0;
int64 toff=0;
int64 cycletime = sync_time;

/**
 * @brief 执行 EtherCAT 分布式时钟（DC）同步操作，计算并更新时间偏移量。
 * 
 * 该函数根据给定的参考时间、循环时间和时间偏移量指针，计算出时间差值，
 * 并根据差值调整积分项，最终更新时间偏移量。示例中设置 Linux 同步点比 DC 同步点晚 50us。
 * 
 * @param reftime 分布式时钟的参考时间，单位通常为微秒。
 * @param cycletime 循环时间，即一个同步周期的时长，单位通常为微秒。
 * @param offsettime 指向时间偏移量的指针，用于存储计算得到的新的时间偏移量。
 */
//test\linux\ebox\ebox.c
/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if(delta> (cycletime /2)) { delta= delta - cycletime; }
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }
   *offsettime = -(delta / 100) - (integral /20);
}

