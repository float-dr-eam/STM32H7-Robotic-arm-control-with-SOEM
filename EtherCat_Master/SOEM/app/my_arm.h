#ifndef __MY__ARM_H__
#define __MY__ARM_H__

#include <stdio.h>
#include "LD3M_ec7010.h"
#include "math.h"
#include "arm_math.h"
#include "arm_const_structs.h"

// ѡ��ʹ�õĸ���������
#define USE_DOUBLE_PRECISION 0  // ����Ϊ0ʹ�õ����ȣ�����Ϊ1ʹ��˫����

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
#endif

#ifndef NUM_SLAVES
     #define NUM_SLAVES 7 // �ؽ�����
     
#endif

#define LEFT 0
#define RIGHT 1

#define DOF NUM_SLAVES
#define MAX_ITER 5000
#define EPSILON 1e-6    // �����ֵ
#define LAMBDA 0.01     // ���򻯲���
#define STEP_SIZE 0.5   // ����
#define IF_PRINT 1      // ��ӡ��Ϣ
// ʹ�� CMSIS-DSP �����Ǻ���
#define DEG2RAD(x) ((x) * PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f / PI)
// DH����
typedef struct {
    my_float alpha;  // �ؽ�Ťת��
    my_float a;      // �ؽڳ���
    my_float d;      // �ؽ�ƫ����
    my_float theta;  // �ؽڽǶ�
    my_float offset; // �ؽڽǶ�ƫ����
} DHParam;

// ��е�۽ṹ��
typedef struct 
{
    DHParam dh_params[DOF];
    my_float q_min[DOF];  // �ؽ���С�Ƕ�
    my_float q_max[DOF];  // �ؽ����Ƕ�
} Robot;

// typedef struct
// {
//     my_float data[4][4];  
// } Matrix4x4_t;


// ���������ļ����ɵĴ���
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
//void arm_forward_kinematics2(const my_float degree[NUM_SLAVES], bool side);// �����е�����˶�ѧ


void init_robot(Robot *robot) ;// ��ʼ����е��
void matrix_multiply(my_float *A, my_float *B, my_float *C, int m, int n, int p);   // ����˷�
void matrix_transpose(my_float *A, my_float *AT, int m, int n); // ����ת��
void vector_subtract(my_float *a, my_float *b, my_float *result, int n);// ��������
my_float vector_norm(my_float *v, int n);// ��������
void compute_jacobian(Robot *robot, my_float *q, my_float *J);// �����ſɱȾ���
void compute_transform(DHParam *dh, my_float *T);// ����DH�任����
void compute_pose_error(my_float *T_current,const my_float *T_desired, my_float *error); // ����λ�����


void forward_kinematics(Robot *robot, my_float *q, my_float *T_result);// ���������˶�ѧ
int inverse_kinematics ( Robot *robot,const my_float *T_desired, my_float *q_init, my_float *q_result, int max_iter, my_float epsilon, my_float lambda, my_float step_size, int verbose);// ���˶�ѧ

void arm_forward_kinematics(const my_float *q_init);// ��е�����˶�ѧ
void arm_inverse_kinematics(const my_float *T_desired, const my_float *q_init, my_float *q_result);// ��е�����˶�ѧ


void simple_demo(void);// ����ʾ
#endif // MY_ARM_H
