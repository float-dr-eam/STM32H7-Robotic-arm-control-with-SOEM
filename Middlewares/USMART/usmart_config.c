#include "usmart.h"
#include "usmart_str.h"

/******************************************************************************************/
/* 用户配置区
 * 这下面要包含所用到的函数所申明的头文件(用户自己添加)
 */

#include "main.h"
#include "tim.h"
#include "malloc.h"
#include "lcd.h"
#include "LD3M_ec7010.h"
#include "my_arm.h"

void lcd_show_static_string(void)
{
    // 显示字符串的函数实现 
    lcd_show_string(200, 800, 200, 16, 16, "o(#'-'#)o :Hello World!", MAGENTA);
}
void calculate_arm()
{
    arm_forward_kinematics(LD3M_all.cur_degree);
}
/* 函数名列表初始化(用户自己添加)
 * 用户直接在这里输入要执行的函数名及其查找串
 */
struct _m_usmart_nametab usmart_nametab[] =
{
#if USMART_USE_WRFUNS == 1      /* 如果使能了读写操作 */
    (void *)read_addr, "uint32_t read_addr(uint32_t addr)",
    (void *)write_addr, "void write_addr(uint32_t addr, uint32_t val)",
#endif
    //(void *)HAL_Delay, "void HAL_Delay(uint16_t nms)",
    (void *)Delay_us, "void Delay_us(uint32_t nus)",
    (void *)mymalloc, "void *mymalloc(uint8_t memx, uint32_t size)",
    (void *)myfree, "void myfree(uint8_t memx, void *ptr)",

    (void *)lcd_show_static_string,"void lcd_show_static_string(void)", 
    (void *)set_motor_target_int_degree,"void set_motor_target_int_degree(uint8 slave_index, int32 target_degree)",
    (void *)set_motor_target,"void set_motor_target(uint8 slave_index, int32 target)",
    (void *)calculate_arm,"void calculate_arm()",
    (void *)all_motors_return_to_origin,"void all_motors_return_to_origin(void)",
    (void *)emergency_stop,"void emergency_stop()",
    (void *)restart_motors,"void restart_motors()",
    (void *)simple_demo,"void simple_demo(void)",
    (void *)set_max_velocity,"void set_max_velocity(uint8 slave_index, int32 target_vel)",
};

/******************************************************************************************/

/* 函数控制管理器初始化
 * 得到各个受控函数的名字
 * 得到函数总数量
 */
struct _m_usmart_dev usmart_dev =
{
    usmart_nametab,
    usmart_init,
    usmart_cmd_rec,
    usmart_exe,
    usmart_scan,
    sizeof(usmart_nametab) / sizeof(struct _m_usmart_nametab), /* 函数数量 */
    0,      /* 参数数量 */
    0,      /* 函数ID */
    1,      /* 参数显示类型,0,10进制;1,16进制 */
    0,      /* 参数类型.bitx:,0,数字;1,字符串 */
    0,      /* 每个参数的长度暂存表,需要MAX_PARM个0初始化 */
    0,      /* 函数的参数,需要PARM_LEN个0初始化 */
};



















