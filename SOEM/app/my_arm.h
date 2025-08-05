#ifndef __MY__ARM_H__
#define __MY__ARM_H__

#include <stdio.h>
#include "LD3M_ec7010.h"
#include "math.h"
#include "arm_math.h"
#include "arm_const_structs.h"

// 定义浮点数类型
#define USE_DOUBLE_PRECISION 0 // 设置为0使用单精度，设置为1使用双精度

#if USE_DOUBLE_PRECISION
    typedef float64_t my_float;
    #define FLOAT_ALIGN   ALIGNED(16)   
    static inline my_float my_sin(my_float x) { return sin(x); }
    static inline my_float my_cos(my_float x) { return cos(x); }
    static inline my_float my_abs(my_float x) { return fabs(x); }
    static inline my_float my_atan2(my_float x,my_float y) { return atan2(x,y); }
    static inline my_float my_sqrt(my_float x) { return sqrt(x); }
    static inline my_float my_fmax(my_float x, my_float y) { return fmax(x, y); }
    static inline my_float my_fmin(my_float x, my_float y) { return fmin(x, y); }
    static inline my_float my_acos(my_float x) { return acos(x); }
    static inline my_float my_asin(my_float x) { return asin(x); }
    static inline my_float my_tan(my_float x) { return tan(x); }

#else
    typedef float32_t my_float;
    #define FLOAT_ALIGN   ALIGNED(8)
    // static inline my_float my_sin(my_float x) { return arm_sin_f32(x); }
    // static inline my_float my_cos(my_float x) { return arm_cos_f32(x); }
    static inline my_float my_sin(my_float x) { return sinf(x); }
    static inline my_float my_cos(my_float x) { return cosf(x); }
    static inline my_float my_abs(my_float x) { return fabsf(x); }
    static inline my_float my_atan2(my_float x,my_float y) { return atan2f(x,y); }
    static inline my_float my_sqrt(my_float x) { return sqrtf(x); }
    static inline my_float my_fmax(my_float x, my_float y) { return fmaxf(x, y); }
    static inline my_float my_fmin(my_float x, my_float y) { return fminf(x, y); }
    static inline my_float my_acos(my_float x) { return acosf(x); }
    static inline my_float my_asin(my_float x) { return asinf(x); }
    static inline my_float my_tan(my_float x) { return tanf(x); }
#endif

#ifndef NUM_SLAVES
     #define NUM_SLAVES 7 // 关节数量
     
#endif

#define LEFT 0
#define RIGHT 1

#define DOF NUM_SLAVES
#define MAX_ITER 1000
#define EPSILON 1e-4f    // 允许误差
#define LAMBDA 0.05f     // 正则化参数
#define STEP_SIZE 0.1f   // 步长
#define IF_PRINT 1      // 打印信息

// 使用 CMSIS-DSP 的三角函数
#define DEG2RAD(x) ((x) * PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f / PI)

// DH参数
typedef struct 
{
    my_float alpha;  // 关节alpha参数
    my_float a;      // 关节长度
    my_float d;      // 关节偏移量
    my_float theta;  // 关节角度
    my_float offset; // 关节角度偏移量
} DHParam;

// 机器人结构体
typedef struct 
{
    DHParam dh_params[DOF];
    my_float q_min[DOF];  // 关节最小角度
    my_float q_max[DOF];  // 关节最大角度
} Robot;

// 定义旋转轴类型
typedef enum 
{
    X, // 绕X轴旋转
    Y, // 绕Y轴旋转
    Z  // 绕Z轴旋转
} RotationAxis;

typedef struct
{
    my_float data[4][4];  
} Matrix4x4_t;


// 根据配置文件生成的代码
#if defined(__CC_ARM)    // MDK ARM Compiler
    #define ALIGNED(x)    __align(x)
#elif defined(__GNUC__)  // GNU Compiler
    #define ALIGNED(x)    __attribute__((aligned(x)))
#elif defined(__ICCARM__) // IAR Compiler
    #define ALIGNED(x)    _Pragma(STRINGIFY(data_alignment=x))
#else
    #define ALIGNED(x)
#endif


void dcm2euler_zyx(const my_float *T, my_float *z, my_float *y, my_float *x);
//void dcm2euler_zyx_fpu(const Matrix4x4_t T06, my_float *z, my_float *y, my_float *x);
//void arm_forward_kinematics2(const my_float degree[NUM_SLAVES], bool side);// 计算机器人运动学
void assign_matrix_to_float_ptr(Matrix4x4_t *src, my_float **dst); // 将Matrix4x4_t类型的矩阵指针赋值给my_float指针

void init_robot(Robot *robot) ;// 初始化机器人模型
void matrix_multiply(my_float *A, my_float *B, my_float *C, int m, int n, int p);   // 矩阵乘法
void matrix_transpose(my_float *A, my_float *AT, int m, int n); // 矩阵转置
my_float vector_norm(my_float *v, int n);// 向量范数
void compute_jacobian(Robot *robot, my_float *q, my_float *J);// 计算雅可比矩阵
void compute_transform(DHParam *dh, my_float *T);// 计算DH变换矩阵
void compute_pose_error(my_float *T_current,const my_float *T_desired, my_float *error); // 计算位姿误差


void forward_kinematics(Robot *robot, my_float *q, my_float *T_result);// 计算正运动学
int inverse_kinematics ( Robot *robot,const my_float *T_desired, my_float *q_init, my_float *q_result, int max_iter, my_float epsilon, my_float lambda_max, my_float step_size, int verbose);// 逆运动学

void arm_forward_kinematics(const my_float *q_init);// 机器人正运动学
void arm_inverse_kinematics(const my_float *T_desired, const my_float *q_init, my_float *q_result);// 机器人逆运动学

void simple_demo(void);// 简单演示


Matrix4x4_t mat4_mul_fpu(const Matrix4x4_t *A, const Matrix4x4_t *B); // 矩阵乘法
Matrix4x4_t multiply_matrix(Matrix4x4_t a, Matrix4x4_t b);// 矩阵乘法
Matrix4x4_t transl(my_float x, my_float y, my_float z) ;// 创建平移变换矩阵
Matrix4x4_t rot(RotationAxis axis, my_float angle_deg);// 创建旋转变换矩阵（角度制），根据指定的轴旋转
void print_matrix(Matrix4x4_t m) ;

#endif // MY_ARM_H_

