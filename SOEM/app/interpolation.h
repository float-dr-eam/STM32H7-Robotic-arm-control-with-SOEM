#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <stdint.h>
#include <stdbool.h>
#include "my_arm.h"


// 四元数结构体
typedef struct {
    my_float w;
    my_float x;
    my_float y;
    my_float z;
} Quaternion;

// 三维向量结构体
typedef struct {
    my_float x;
    my_float y;
    my_float z;
} Vector3D;

// 轨迹结构体
typedef struct {
    Matrix4x4_t* transforms;  // 变换矩阵数组
    Vector3D* positions;      // 轨迹点位置数组
    int num_points;           // 轨迹点数量
} Trajectory;
// 关节轨迹结构体
typedef struct {
    my_float** joint_angles;  // 关节角度数组，存储每个时刻的关节角度
    int num_points;         // 轨迹点数量
} JointTrajectory;


// 运动状态机枚举
typedef enum {
    MOTION_IDLE,// 空闲状态
    MOTION_SEND_ANGLE,// 发送角度状态
    MOTION_WAIT_ARRIVE,// 等待到达状态
    //MOTION_WAIT_DELAY// 等待延迟状态
} MotionState;



// 旋转矩阵转换为轴角表示
void rotm_to_axisangle(Matrix4x4_t rotm, Vector3D *axis, my_float *angle);
// 轴角表示转换为旋转矩阵
Matrix4x4_t axisangle_to_rotm(Vector3D axis, my_float angle);
// 向量叉积
Vector3D cross_product(Vector3D a, Vector3D b);
// 向量点积
my_float dot_product(Vector3D a, Vector3D b);
// 向量归一化
Vector3D normalize(Vector3D v);
// 向量减法
Vector3D vector_subtract(Vector3D a, Vector3D b);
// 向量模长
my_float vector_magnitude(Vector3D v);
// 向量加法
Vector3D vector_add(Vector3D a, Vector3D b);
// 向量缩放
Vector3D vector_scale(Vector3D v, my_float s);
// 从变换矩阵提取位置
Vector3D extract_position(Matrix4x4_t transform);
// 从变换矩阵提取旋转矩阵
Matrix4x4_t extract_rotation(Matrix4x4_t transform);
// 旋转矩阵转换为四元数
Quaternion rotm_to_quat(Matrix4x4_t rotm);
// 四元数转换为旋转矩阵
Matrix4x4_t quat_to_rotm(Quaternion q);
// 四元数球面线性插值(SLERP)
Quaternion slerp(Quaternion q1, Quaternion q2, my_float t);
// 创建变换矩阵
Matrix4x4_t create_transform(Vector3D position, Matrix4x4_t rotation);
// 五次多项式插值-直线规划
Trajectory *quintic_line_trajectory(Matrix4x4_t T_start, Matrix4x4_t T_end, int num_points);
// 圆弧轨迹规划
Trajectory *arc_trajectory(Matrix4x4_t T_start, Vector3D p_mid, Matrix4x4_t T_end, int num_points, my_float *radius, Vector3D *center);
// 打印轨迹信息
void print_trajectory(Trajectory *traj, const char *name);
// 释放轨迹占用的内存
void free_trajectory(Trajectory *traj);
// 轨迹规划测试函数
void trajectory_planning_test(void);
// 轨迹规划验证函数
void validate_trajectory_planning(void);
// 在main函数中的示例调用
//JointTrajectory* demo(void);
// 释放关节轨迹占用的内存
void free_joint_trajectory(JointTrajectory *joint_traj);
// 关节轨迹规划函数
void trajectory_planning_with_joints(void);

extern void start_nonblocking_motion(JointTrajectory* traj);// 开始非阻塞运动
#endif
