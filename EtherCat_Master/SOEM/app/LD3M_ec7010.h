#ifndef __LD3M__EC7010_H__
#define __LD3M__EC7010_H__

#include "osal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "my_arm.h"
#include "math.h"
#include "arm_math.h"
#include "my_flash.h"



#if USE_DOUBLE_PRECISION
  #ifndef my_float
    typedef double my_float;
  #endif
#else
   #ifndef my_float
      typedef float my_float;
   #endif
#endif

#define IOmap_Length 1024
#define NUM_SLAVES 7
// #define MAX_CNT 36000 // 最大速度值  °/0.5ms

// 定义每个电机的单位脉冲数组,用于初始化One_Degree
static const int32 UNIT_PULSES[NUM_SLAVES] =
{
   30583, // 电机1的单位脉冲数
   59710, // 电机2的单位脉冲数
   59710, // 电机3的单位脉冲数
   59710, // 电机4的单位脉冲数
   72817, // 电机5的单位脉冲数
   72817, // 电机6的单位脉冲数
   30583, // 电机7的单位脉冲数
};
// 定义每个电机的最大速度值
static const int32 a_cnt[NUM_SLAVES] =
{
   8000, // 电机1的最大速度值
   20000, // 电机2的最大速度值
   20000, // 电机3的最大速度值
   20000, // 电机4的最大速度值
   30000, // 电机5的最大速度值
   30000, // 电机6的最大速度值
   8000  // 电机7的最大速度值
};
// 定义每个电机的相对正转值
static const int8_t rotate_sign[NUM_SLAVES] =
{
   1,
   -1,
   1,
   1,
   1,
   -1,
   1
};

extern int32 origin_point[NUM_SLAVES+1];

/**PACKED 宏用于告诉编译器在结构体中不进行字节对齐，而是按照成员变量的定义顺序进行存储。
 *
 * @brief 定义 LD3M 设备的输出过程数据对象 (PDO) 结构体。
 * 该结构体用于封装要发送到 LD3M 设备的过程数据。（7字节）
 */
typedef struct PACKED
{
   uint16 ControlWord; // 控制字，用于向 LD3M 设备发送控制命令
   int32 TargetPos;    // 目标位置，指定 LD3M 设备要移动到的目标位置
   uint8 TargetMode;   // 目标模式，指定 LD3M 设备的运行模式
} LD3M_PDO_Output;

/**
 * @brief 定义 LD3M 设备的输入过程数据对象 (PDO) 结构体。
 * 该结构体用于封装从 LD3M 设备接收的过程数据。（13字节）
 */
typedef struct PACKED
{
   uint16 StatusWord;     // 状态字，反映 LD3M 设备的当前状态
   int32 CurrentPosition; // 当前位置，指示 LD3M 设备当前所处的位置
   int32 CurrentVelocity; // 当前速度，指示 LD3M 设备当前的运行速度
   uint16 ErrorCode;      // 错误代码，当 LD3M 设备出现故障时，反映具体的错误类型
   uint8 CurrentMode;     // 当前模式，指示 LD3M 设备当前的运行模式
} LD3M_PDO_Input;

typedef struct
{
   LD3M_PDO_Output *outputs[NUM_SLAVES]; // 从站输出 PDO 数组
   LD3M_PDO_Input *inputs[NUM_SLAVES];   // 从站输入 PDO 数组

   int32 cur_pos[NUM_SLAVES]; // 用于存储每个电机的当前位置
   int32 target_pos[NUM_SLAVES];

   int32 cur_vel[NUM_SLAVES]; // 用于存储每个电机的当前速度
   int32 target_vel[NUM_SLAVES];

   uint8 cur_mode[NUM_SLAVES];    // 用于存储每个电机的当前模式
   uint8 target_mode[NUM_SLAVES]; // 用于存储每个电机的目标模式

   uint16 status_word[NUM_SLAVES]; // 用于存储每个电机的状态字
   uint16 error_code[NUM_SLAVES];  // 用于存储每个电机的错误代码
   uint8 control_word[NUM_SLAVES]; // 用于存储每个电机的控制字

   bool has_new_target[NUM_SLAVES]; // 用于标记每个电机是否有新的目标位置
   uint8 LD3Mflag[NUM_SLAVES];      // 用于存储每个电机的状态机的标志器
   my_float cur_degree[NUM_SLAVES];    // 用于存储每个电机的目标位置（角度）

} LD3M_All;

extern LD3M_All LD3M_all; // 定义一个全局变量，用于存储所有从站的 PDO 数据

void LD3M_test(char *ifname);
void LD3M_loop(void);
void set_motor_target(uint8 slave_index, int32 target);
void set_motor_target_degree(uint8 slave_index, my_float target_degree);
void set_motor_target_int_degree(uint8 slave_index, int32 target_degree);


void all_motors_return_to_origin(void);//所有电机返回原点
void motors_run_to_angles(my_float *angles);//所有电机运行到指定角度
void emergency_stop(void);//所有电机急停
void restart_motors(void);//所有电机重新启动
#endif /* SERVOS7TEST_H_ */
