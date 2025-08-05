#include "my_arm.h"
#include "malloc.h"

void matrix_multiply(my_float *A, my_float *B, my_float *C, int m, int n, int p);
void matrix_transpose(my_float *A, my_float *AT, int m, int n);
void vector_subtract(my_float *a, my_float *b, my_float *result, int n);
my_float vector_norm(my_float *v, int n);

// 初始化机械臂
void init_robot(Robot *robot)
{
    // 关节1
    robot->dh_params[0].alpha = 0.0;
    robot->dh_params[0].a = 0.0;
    robot->dh_params[0].d = 0.1299;
    robot->dh_params[0].theta = 0.0; // 偏移=0
    robot->dh_params[0].offset = 0.0;

    // 关节2
    robot->dh_params[1].alpha = -PI / 2;
    robot->dh_params[1].a = 0.0;
    robot->dh_params[1].d = 0.0;
    robot->dh_params[1].theta = 0.0; // 偏移=-pi/2
    robot->dh_params[1].offset = -PI / 2;
    // 关节3
    robot->dh_params[2].alpha = PI / 2;
    robot->dh_params[2].a = 0.0;
    robot->dh_params[2].d = 0.2153;
    robot->dh_params[2].theta = 0.0; // 偏移=pi/2
    robot->dh_params[2].offset = PI / 2;
    // 关节4
    robot->dh_params[3].alpha = PI / 2;
    robot->dh_params[3].a = 0.0;
    robot->dh_params[3].d = 0.0;
    robot->dh_params[3].theta = 0.0; // 偏移=0
    robot->dh_params[3].offset = 0.0;
    // 关节5
    robot->dh_params[4].alpha = -PI / 2;
    robot->dh_params[4].a = 0.0;
    robot->dh_params[4].d = 0.2163;
    robot->dh_params[4].theta = 0.0; // 偏移=0
    robot->dh_params[4].offset = 0.0;
    // 关节6
    robot->dh_params[5].alpha = PI / 2;
    robot->dh_params[5].a = 0.0;
    robot->dh_params[5].d = 0.0;
    robot->dh_params[5].theta = 0.0; // 偏移=0
    robot->dh_params[5].offset = 0.0;
    // 关节7
    robot->dh_params[6].alpha = -PI / 2;
    robot->dh_params[6].a = 0.0;
    robot->dh_params[6].d = 0.1206;
    robot->dh_params[6].theta = 0.0; // 偏移=0
    robot->dh_params[6].offset = 0.0;
    // 设置关节角度范围
    for (int i = 0; i < DOF; i++)
    {
        robot->q_min[i] = -PI;
        robot->q_max[i] = PI;
    }
}

// 计算DH变换矩阵
void compute_transform(DHParam *dh, my_float *T)
{
    my_float ct = cos(dh->theta + dh->offset);
    my_float st = sin(dh->theta + dh->offset);
    my_float ca = cos(dh->alpha);
    my_float sa = sin(dh->alpha);

    // 标准DH变换矩阵
    T[0] = ct;
    T[1] = -st;
    T[2] = 0;
    T[3] = dh->a;
    T[4] = st * ca;
    T[5] = ct * ca;
    T[6] = -sa;
    T[7] = -dh->d * sa;
    T[8] = st * sa;
    T[9] = ct * sa;
    T[10] = ca;
    T[11] = dh->d * ca;
    T[12] = 0;
    T[13] = 0;
    T[14] = 0;
    T[15] = 1;
}

// 正向运动学
void forward_kinematics(Robot *robot, my_float *q, my_float *T_result)
{
    my_float T[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    my_float T_i[16];
    my_float T_temp[16];

    // 更新关节角度
    for (int i = 0; i < DOF; i++)
    {
        robot->dh_params[i].theta = q[i];
    }

    // 计算变换矩阵
    for (int i = 0; i < DOF; i++)
    {
        compute_transform(&robot->dh_params[i], T_i);
        memcpy(T_temp, T, 16 * sizeof(my_float));
        matrix_multiply(T_temp, T_i, T, 4, 4, 4);
    }

    // 输出结果
    memcpy(T_result, T, 16 * sizeof(my_float));
}

// 计算雅可比矩阵
void compute_jacobian(Robot *robot, my_float *q, my_float *J)
{
    // 计算关节位置和轴向
    my_float T[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    my_float T_i[16];
    my_float T_temp[16];
    my_float joint_positions[DOF][3];
    my_float joint_axes[DOF][3];
    my_float end_position[3];

    // 更新关节角度
    for (int i = 0; i < DOF; i++)
    {
        robot->dh_params[i].theta = q[i];
    }

    // 计算变换矩阵
    for (int i = 0; i < DOF; i++)
    {
        compute_transform(&robot->dh_params[i], T_i);
        memcpy(T_temp, T, 16 * sizeof(my_float));
        matrix_multiply(T_temp, T_i, T, 4, 4, 4);

        // 计算关节i的末端位置
        joint_positions[i][0] = T[3];
        joint_positions[i][1] = T[7];
        joint_positions[i][2] = T[11];

        // 计算关节i的轴向
        joint_axes[i][0] = T[2];
        joint_axes[i][1] = T[6];
        joint_axes[i][2] = T[10];
    }

    // 计算末端位置
    end_position[0] = T[3];
    end_position[1] = T[7];
    end_position[2] = T[11];

    // 计算雅可比矩阵
    for (int i = 0; i < DOF; i++)
    {
        // 计算关节位置和轴向
        my_float r[3] =
            {
                end_position[0] - joint_positions[i][0],
                end_position[1] - joint_positions[i][1],
                end_position[2] - joint_positions[i][2]};

        // 计算雅可比矩阵
        J[i] = joint_axes[i][1] * r[2] - joint_axes[i][2] * r[1];
        J[i + DOF] = joint_axes[i][2] * r[0] - joint_axes[i][0] * r[2];
        J[i + 2 * DOF] = joint_axes[i][0] * r[1] - joint_axes[i][1] * r[0];

        // 计算雅可比矩阵
        J[i + 3 * DOF] = joint_axes[i][0];
        J[i + 4 * DOF] = joint_axes[i][1];
        J[i + 5 * DOF] = joint_axes[i][2];
    }
}

// 计算旋转误差（旋转向量，3维）
void rotation_error(my_float *R_current,const my_float *R_target, my_float *w)
{
    // dR = R_target * R_current'
    my_float dR[9];
    // R_current'（转置）
    my_float R_current_T[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R_current_T[i * 3 + j] = R_current[j * 4 + i]; // 注意4x4矩阵的步长

    // dR = R_target * R_current'
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            dR[i * 3 + j] = 0;
            for (int k = 0; k < 3; k++)
                dR[i * 3 + j] += R_target[i * 4 + k] * R_current_T[k * 3 + j];
        }

    // 由dR提取旋转向量
    my_float theta = acos(fmin(fmax((dR[0] + dR[4] + dR[8] - 1) / 2, -1.0), 1.0));
    if (fabs(theta) < 1e-8)
    {
        w[0] = w[1] = w[2] = 0;
        return;
    }
    w[0] = (dR[7] - dR[5]) / (2 * sin(theta));
    w[1] = (dR[2] - dR[6]) / (2 * sin(theta));
    w[2] = (dR[3] - dR[1]) / (2 * sin(theta));
    w[0] *= theta;
    w[1] *= theta;
    w[2] *= theta;
}

// 计算6维位姿误差
void compute_pose_error(my_float *T_current, const my_float *T_desired, my_float *error)
{
    // 位置误差
    error[0] = T_desired[3] - T_current[3];
    error[1] = T_desired[7] - T_current[7];
    error[2] = T_desired[11] - T_current[11];

    // 姿态误差
    my_float w[3];
    rotation_error(T_current, T_desired, w);
    error[3] = w[0];
    error[4] = w[1];
    error[5] = w[2];
}
// arm_mat_inverse_f32(T, T_inv);
//  计算n阶方阵A的逆，结果存到Ainv，返回1成功，0失败
int mat_inverse(my_float *A, my_float *Ainv, int n)
{
    my_float temp;
    int i, j, k;
    // 初始化Ainv为单位阵
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            Ainv[i * n + j] = (i == j) ? 1.0 : 0.0;
    // 拷贝A到临时数组
    my_float *B = (my_float *)mymalloc(SRAMEX, n * n * sizeof(my_float));
    memcpy(B, A, n * n * sizeof(my_float));
    // 高斯消元
    for (i = 0; i < n; i++)
    {
        // 主元为0，找下面一行交换
        if (fabs(B[i * n + i]) < 1e-12)
        {
            int swap = -1;
            for (j = i + 1; j < n; j++)
            {
                if (fabs(B[j * n + i]) > 1e-12)
                {
                    swap = j;
                    break;
                }
            }
            if (swap == -1)
            {
                myfree(SRAMEX, B);
                return 0;
            }
            // 交换行
            for (k = 0; k < n; k++)
            {
                temp = B[i * n + k];
                B[i * n + k] = B[swap * n + k];
                B[swap * n + k] = temp;
                temp = Ainv[i * n + k];
                Ainv[i * n + k] = Ainv[swap * n + k];
                Ainv[swap * n + k] = temp;
            }
        }
        // 归一化主元行
        temp = B[i * n + i];
        for (k = 0; k < n; k++)
        {
            B[i * n + k] /= temp;
            Ainv[i * n + k] /= temp;
        }
        // 消元
        for (j = 0; j < n; j++)
        {
            if (j == i)
                continue;
            temp = B[j * n + i];
            for (k = 0; k < n; k++)
            {
                B[j * n + k] -= temp * B[i * n + k];
                Ainv[j * n + k] -= temp * Ainv[i * n + k];
            }
        }
    }
    myfree(SRAMEX, B);
    return 1;
}

// 阻尼最小二乘法计算伪逆（修正版）
void damped_pseudoinverse(my_float *J, my_float *J_pinv, int m, int n, my_float lambda)
{
    // 输入检查
    if (m <= 0 || n <= 0 || J == NULL || J_pinv == NULL)
    {
        printf("Invalid input parameters\n");
        return;
    }

    // 分配内存（含错误检查）
    my_float *JT = (my_float *)mymalloc(SRAMEX, n * m * sizeof(my_float));
    my_float *JJT = (my_float *)mymalloc(SRAMEX, m * m * sizeof(my_float));
    my_float *JJT_lam = (my_float *)mymalloc(SRAMEX, m * m * sizeof(my_float));
    my_float *JJT_inv = (my_float *)mymalloc(SRAMEX, m * m * sizeof(my_float));
    if (JT == NULL || JJT == NULL || JJT_lam == NULL || JJT_inv == NULL)
    {
        printf("Memory allocation failed\n");
        myfree(SRAMEX, JT);
        myfree(SRAMEX, JJT);
        myfree(SRAMEX, JJT_lam);
        myfree(SRAMEX, JJT_inv);
        return;
    }

    // 1. 计算J的转置JT
    matrix_transpose(J, JT, m, n);

    // 2. 计算J*JT得到JJT
    matrix_multiply(J, JT, JJT, m, n, m);

    // 3. 添加阻尼项lambda?*I
    for (int i = 0; i < m * m; ++i)
        JJT_lam[i] = JJT[i];
    for (int i = 0; i < m; ++i)
        JJT_lam[i * m + i] += lambda * lambda;

    // 4. 计算JJT_lam的逆矩阵（高斯消元法）
    if (!mat_inverse(JJT_lam, JJT_inv, m))
    {
        printf("JJT_lam不可逆\n");
        // 你可以选择return或继续用对角近似
        for (int i = 0; i < m * m; ++i)
            JJT_inv[i] = 0.0;
        for (int i = 0; i < m; ++i)
            JJT_inv[i * m + i] = 1.0f / JJT_lam[i * m + i];
    }

    // 5. 计算右伪逆J_pinv = JT * JJT_inv
    matrix_multiply(JT, JJT_inv, J_pinv, n, m, m);

    // 6. 释放内存
    myfree(SRAMEX, JT);
    myfree(SRAMEX, JJT);
    myfree(SRAMEX, JJT_lam);
    myfree(SRAMEX, JJT_inv);
}

// 零空间优化目标（以靠近中位为例）
void nullspace_objective(my_float *q, my_float *q_mid, my_float *grad)
{
    for (int i = 0; i < DOF; ++i)
        grad[i] = q[i] - q_mid[i];
}

// 逆运动学主循环
int inverse_kinematics(
    Robot *robot,const my_float *T_desired, my_float *q_init, my_float *q_result, int max_iter, my_float epsilon, my_float lambda, my_float step_size, int verbose)
{
    my_float *q = (my_float *)mymalloc(SRAMEX, DOF * sizeof(my_float));
    memcpy(q, q_init, DOF * sizeof(my_float));
    my_float *T_current = (my_float *)mymalloc(SRAMEX, 16 * sizeof(my_float));
    my_float *J = (my_float *)mymalloc(SRAMEX, 6 * DOF * sizeof(my_float));
    my_float *J_pinv = (my_float *)mymalloc(SRAMEX, DOF * 6 * sizeof(my_float));
    my_float *error = (my_float *)mymalloc(SRAMEX, 6 * sizeof(my_float));
    my_float *dq = (my_float *)mymalloc(SRAMEX, DOF * sizeof(my_float));
    my_float *q_mid = (my_float *)mymalloc(SRAMEX, DOF * sizeof(my_float));
    my_float *error_history = (my_float *)mymalloc(SRAMEX, MAX_ITER * sizeof(my_float));

    if (error_history == NULL || q == NULL || T_current == NULL || J == NULL || J_pinv == NULL || error == NULL || dq == NULL || q_mid == NULL)
    {
        printf("内存分配失败！\n");
        myfree(SRAMEX, q);
        myfree(SRAMEX, T_current);
        myfree(SRAMEX, J);
        myfree(SRAMEX, J_pinv);
        myfree(SRAMEX, error);
        myfree(SRAMEX, dq);
        myfree(SRAMEX, q_mid);
        myfree(SRAMEX, error_history);
        return -1; // 内存分配失败
    }

    for (int iter = 0; iter < max_iter; ++iter)
    {
        // 1. 正解
        forward_kinematics(robot, q, T_current);
        // 2. 误差
        compute_pose_error(T_current, T_desired, error);

        my_float err_norm = vector_norm(error, 6);

        if (verbose && ((iter + 1) % 100 == 0))
            printf("iter %d, error norm = %.8f\n", iter + 1, err_norm);

        if (err_norm < epsilon)
        {
            memcpy(q_result, q, DOF * sizeof(my_float));
            myfree(SRAMEX, q);
            myfree(SRAMEX, T_current);
            myfree(SRAMEX, J);
            myfree(SRAMEX, J_pinv);
            myfree(SRAMEX, error);
            myfree(SRAMEX, dq);
            myfree(SRAMEX, q_mid);
            myfree(SRAMEX, error_history);
            return iter + 1; // 收敛
        }
        // 3. 雅可比
        compute_jacobian(robot, q, J);

        // 4. 阻尼伪逆
        if (iter > 1)
        {
            if (err_norm < error_history[iter - 1])
                lambda = fmax(0.0001, lambda * 0.7f);
            else
                lambda = fmin(0.1, lambda * 1.3f);
        }

        damped_pseudoinverse(J, J_pinv, 6, DOF, lambda);
        // 5. 零空间优化梯度
        my_float grad[DOF];
        nullspace_objective(q, q_mid, grad);
        // 6. 零空间投影
        my_float N[DOF * DOF]; // N = I - J_pinv * J
        // 先算 J_pinv * J
        my_float JPJ[DOF * DOF];
        matrix_multiply(J_pinv, J, JPJ, DOF, 6, DOF);
        // N = I - JPJ
        // 先将N初始化为单位阵
        for (int i = 0; i < DOF * DOF; ++i)
            N[i] = 0.0;
        for (int i = 0; i < DOF; ++i)
            N[i * DOF + i] = 1.0;
        // 再减去JPJ
        for (int i = 0; i < DOF * DOF; ++i)
            N[i] -= JPJ[i];

        // 7. dq = J_pinv * error - alpha * N * grad
        my_float dq1[DOF], dq2[DOF];
        matrix_multiply(J_pinv, error, dq1, DOF, 6, 1);
        matrix_multiply(N, grad, dq2, DOF, DOF, 1);
        my_float alpha = 0.0; // 零空间优化权重

        // 步长限制
        for (int i = 0; i < DOF; ++i)
        {
            dq[i] = step_size * (dq1[i] - alpha * dq2[i]);
        }

        my_float dq_norm = vector_norm(dq, DOF);
        if (dq_norm > 0.5f)
        {
            for (int i = 0; i < DOF; ++i)
                dq[i] *= 0.5f / dq_norm;
        }
        // 8. 更新关节角
        for (int i = 0; i < DOF; ++i)
        {
            q[i] += dq[i];
            // 关节限位
            if (q[i] < robot->q_min[i])
                q[i] = robot->q_min[i];
            if (q[i] > robot->q_max[i])
                q[i] = robot->q_max[i];
        }
        error_history[iter] = err_norm;
    }
    // 未收敛
    memcpy(q_result, q, DOF * sizeof(my_float));
    myfree(SRAMEX, q);
    myfree(SRAMEX, T_current);
    myfree(SRAMEX, J);
    myfree(SRAMEX, J_pinv);
    myfree(SRAMEX, error);
    myfree(SRAMEX, dq);
    myfree(SRAMEX, q_mid);
    myfree(SRAMEX, error_history);
    return 0;
}

// 矩阵乘法
void matrix_multiply(my_float *A, my_float *B, my_float *C, int m, int n, int p)
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < p; j++)
        {
            C[i * p + j] = 0;
            for (int k = 0; k < n; k++)
            {
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

// 矩阵转置
void matrix_transpose(my_float *A, my_float *AT, int m, int n)
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            AT[j * m + i] = A[i * n + j];
        }
    }
}

// 向量减法
void vector_subtract(my_float *a, my_float *b, my_float *result, int n)
{
    for (int i = 0; i < n; i++)
    {
        result[i] = a[i] - b[i];
    }
}

// 向量范数
my_float vector_norm(my_float *v, int n)
{
    my_float sum = 0;
    for (int i = 0; i < n; i++)
    {
        sum += v[i] * v[i];
    }
    return sqrt(sum);
}


void arm_forward_kinematics(const my_float *q_init)
{
    Robot *robot = (Robot *)mymalloc(SRAMEX, sizeof(Robot));
    my_float *T_result = (my_float *)mymalloc(SRAMEX, 16 * sizeof(my_float));
    my_float *q_rad = (my_float *)mymalloc(SRAMEX, DOF * sizeof(my_float));
    if (q_init == NULL)
    {
        printf("Invalid input parameters!\n");
        return;
    }
    if (robot == NULL || q_rad == NULL || T_result == NULL)
    {
        printf("Memory allocation failed!\n");
        myfree(SRAMEX, robot);
        myfree(SRAMEX, q_rad);
        myfree(SRAMEX, T_result);
        return;
    }

    // 初始化机械臂
    init_robot(robot);

    // 角度转弧度
    for (int i = 0; i < DOF; i++)
    {
        q_rad[i] = DEG2RAD(q_init[i]);
    }

    // 正向运动学计算
    forward_kinematics(robot, q_rad, T_result);

    // 打印结果
    printf("Forward kinematics result:\n");
    printf("Transform matrix:\n");
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%10.6f ", T_result[i * 4 + j]);
        }
        printf("\n");
    }

    printf("Position (x,y,z):\n");
    printf("%10.6f %10.6f %10.6f\n", T_result[3], T_result[7], T_result[11]);
    my_float z, y, x;
    dcm2euler_zyx(T_result, &z, &y, &x);
    printf("Euler angles (ZYX):\n");
    printf("%10.6f %10.6f %10.6f\n", RAD2DEG(z), RAD2DEG(y), RAD2DEG(x));
    
    myfree(SRAMEX, robot);
    myfree(SRAMEX, q_rad);
    myfree(SRAMEX, T_result);
}

void arm_inverse_kinematics(const my_float *T_desired, const my_float *q_init, my_float *q_result)
{
    Robot *robot = (Robot *)mymalloc(SRAMEX, sizeof(Robot));
    my_float *J = (my_float *)mymalloc(SRAMEX, 6 * DOF * sizeof(my_float));
    my_float *q_rad = (my_float *)mymalloc(SRAMEX, DOF * sizeof(my_float));
    my_float *T_current = (my_float *)mymalloc(SRAMEX, 16 * sizeof(my_float));
    if (T_desired == NULL || q_init == NULL || q_result == NULL)
    {
        printf("Memory  error!\n");
        return; // 内存分配失败
    }
    if (J == NULL || robot == NULL || q_rad == NULL || T_current == NULL)
    {
        printf("Memory allocation failed!\n");
        myfree(SRAMEX, J);
        myfree(SRAMEX, robot);
        myfree(SRAMEX, q_rad);
        myfree(SRAMEX, T_current);
        return; // 内存分配失败
    }
    // //初始化机械臂
    init_robot(robot);
    for (int i = 0; i < DOF; i++)
    {
        q_rad[i] = DEG2RAD(q_init[i]);
    }
    // 逆运动学求解
    int n;
    if ((n = inverse_kinematics(robot, T_desired, q_rad, q_result, MAX_ITER, EPSILON, LAMBDA, STEP_SIZE, IF_PRINT)) != 0)
    {
        printf("逆运动学收敛！\n");
        printf("关节角结果：\n");
        for (int i = 0; i < DOF; i++)
        {
            printf("q[%d] = %10.6f 弧度\n", i, q_result[i]);
        }
        printf("迭代次数：%d\n", n);
        my_float error[6];
        forward_kinematics(robot, q_result, T_current);
        printf("T_current: \n");
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                printf("%10.6f ", T_current[i * 4 + j]);
            }
            printf("\n");
        }
        printf("\n");
        compute_pose_error(T_current, T_desired, error);
        my_float error_norm = vector_norm(error, 6);
        printf("误差：%f\n", error_norm);
        for (int i = 0; i < DOF; i++)
        {
            q_result[i] = RAD2DEG(q_result[i]); // 转换为角度
        }
    }
    else
    {
        printf("逆运动学未收敛！\n");
    }
    myfree(SRAMEX, J);
    myfree(SRAMEX, robot);
    myfree(SRAMEX, q_rad);
    myfree(SRAMEX, T_current);
}

void simple_demo(void)
{
    my_float *T_desired = (my_float *)mymalloc(SRAMEX, 16 * sizeof(my_float));
    my_float *q_result = (my_float *)mymalloc(SRAMEX, DOF * sizeof(my_float));
    if ( T_desired == NULL || q_result == NULL)
    {
        printf("Memory allocation failed!\n");
        myfree(SRAMEX, T_desired);
        myfree(SRAMEX, q_result);
        return; // 内存分配失败
    }
    T_desired[0] = 0;    T_desired[1] = -1;   T_desired[2] = 0;     T_desired[3] = 0;
    T_desired[4] = 1;    T_desired[5] = 0;    T_desired[6] = 0;     T_desired[7] = -0.216;
    T_desired[8] = 0;    T_desired[9] = 0;    T_desired[10] = 1;    T_desired[11] = 0.466;
    T_desired[12] = 0;   T_desired[13] = 0;   T_desired[14] = 0;    T_desired[15] = 1;


    arm_inverse_kinematics(T_desired, LD3M_all.cur_degree, q_result);
    motors_run_to_angles(q_result); // 运行到目标角度
    myfree(SRAMEX, T_desired);

}


void dcm2euler_zyx(const my_float *T, my_float *z, my_float *y, my_float *x)
{
    // 姿态计算
    my_float R[3][3] =
        {
            {T[0], T[1], T[2]},
            {T[4], T[5], T[6]},
            {T[8], T[9], T[10]}};
    *y = my_sin(-R[2][0]);
    if (my_abs(my_cos(*y)) > 1e-6f)
    {
        *x = my_atan2(R[2][1], R[2][2]);
        *z = my_atan2(R[1][0], R[0][0]);
    }
    else
    {
        *x = my_atan2(-R[1][2], R[1][1]);
        *z = 0.0f;
    }
}
// // 矩阵乘法
// void mat4_mul_fpu(const Matrix4x4_t *A, const Matrix4x4_t *B, Matrix4x4_t *C)
// {
// #if USE_my_float_PRECISION
//     arm_matrix_instance_f64 matA = {4, 4, (my_float *)A->data};
//     arm_matrix_instance_f64 matB = {4, 4, (my_float *)B->data};
//     arm_matrix_instance_f64 matC = {4, 4, (my_float *)C->data};

//     arm_mat_mult_f64(&matA, &matB, &matC);
// #else
//     arm_matrix_instance_f32 matA = {4, 4, (my_float *)A->data};
//     arm_matrix_instance_f32 matB = {4, 4, (my_float *)B->data};
//     arm_matrix_instance_f32 matC = {4, 4, (my_float *)C->data};
//     arm_mat_mult_f32(&matA, &matB, &matC);
// #endif
// }

// // 方向余弦矩阵转ZYX欧拉角（使用 CMSIS-DSP 函数）
// void dcm2euler_zyx_fpu(const Matrix4x4_t T, my_float *z, my_float *y, my_float *x)
// {
//     // 姿态计算
//     my_float R[3][3] =
//         {
//             {T.data[0][0], T.data[0][1], T.data[0][2]},
//             {T.data[1][0], T.data[1][1], T.data[1][2]},
//             {T.data[2][0], T.data[2][1], T.data[2][2]}};
//     *y = my_sin(-R[2][0]);
//     if (my_abs(my_cos(*y)) > 1e-6f)
//     {
//         *x = my_atan2(R[2][1], R[2][2]);
//         *z = my_atan2(R[1][0], R[0][0]);
//     }
//     else
//     {
//         *x = my_atan2(-R[1][2], R[1][1]);
//         *z = 0.0f;
//     }
// }
// // 初始化变换矩阵的函数
// void init_transform_matrices(const my_float q[NUM_SLAVES], Matrix4x4_t *T01, Matrix4x4_t *T12, Matrix4x4_t *T23, Matrix4x4_t *T34, Matrix4x4_t *T45, Matrix4x4_t *T56, Matrix4x4_t *T67, bool side)
// {
//     // 预计算所有关节的sin和cos值
//     my_float s0 = my_sin(q[0]), c0 = my_cos(q[0]);
//     my_float s1 = my_sin(q[1]), c1 = my_cos(q[1]);
//     my_float s2 = my_sin(q[2]), c2 = my_cos(q[2]);
//     my_float s3 = my_sin(q[3]), c3 = my_cos(q[3]);
//     my_float s4 = my_sin(q[4]), c4 = my_cos(q[4]);
//     my_float s5 = my_sin(q[5]), c5 = my_cos(q[5]);
//     my_float s6 = my_sin(q[6]), c6 = my_cos(q[6]);

//     // 关节1 - 左右两侧相同
//     T01->data[0][0] = c0;
//     T01->data[0][1] = -s0;
//     T01->data[0][2] = 0.0;
//     T01->data[0][3] = 0.0;
//     T01->data[1][0] = s0;
//     T01->data[1][1] = c0;
//     T01->data[1][2] = 0.0;
//     T01->data[1][3] = 0.0;
//     T01->data[2][0] = 0.0;
//     T01->data[2][1] = 0.0;
//     T01->data[2][2] = 1.0;
//     T01->data[2][3] = 0.1299;
//     T01->data[3][0] = 0.0;
//     T01->data[3][1] = 0.0;
//     T01->data[3][2] = 0.0;
//     T01->data[3][3] = 1.0;

//     if (side == LEFT)
//     {
//         // 左侧关节2
//         T12->data[0][0] = -s1;
//         T12->data[0][1] = -c1;
//         T12->data[0][2] = 0.0;
//         T12->data[0][3] = 0.0;
//         T12->data[1][0] = 0.0;
//         T12->data[1][1] = 0.0;
//         T12->data[1][2] = -1.0;
//         T12->data[1][3] = 0.0;
//         T12->data[2][0] = c1;
//         T12->data[2][1] = -s1;
//         T12->data[2][2] = 0.0;
//         T12->data[2][3] = 0.0;
//         T12->data[3][0] = 0.0;
//         T12->data[3][1] = 0.0;
//         T12->data[3][2] = 0.0;
//         T12->data[3][3] = 1.0;

//         // 左侧关节3
//         T23->data[0][0] = s2;
//         T23->data[0][1] = c2;
//         T23->data[0][2] = 0.0;
//         T23->data[0][3] = 0.0;
//         T23->data[1][0] = 0.0;
//         T23->data[1][1] = 0.0;
//         T23->data[1][2] = 1.0;
//         T23->data[1][3] = 0.2153;
//         T23->data[2][0] = c2;
//         T23->data[2][1] = -s2;
//         T23->data[2][2] = 0.0;
//         T23->data[2][3] = 0.0;
//         T23->data[3][0] = 0.0;
//         T23->data[3][1] = 0.0;
//         T23->data[3][2] = 0.0;
//         T23->data[3][3] = 1.0;

//         // 左侧关节4
//         T34->data[0][0] = c3;
//         T34->data[0][1] = -s3;
//         T34->data[0][2] = 0.0;
//         T34->data[0][3] = 0.0;
//         T34->data[1][0] = 0.0;
//         T34->data[1][1] = 0.0;
//         T34->data[1][2] = 1.0;
//         T34->data[1][3] = 0.0;
//         T34->data[2][0] = -s3;
//         T34->data[2][1] = -c3;
//         T34->data[2][2] = 0.0;
//         T34->data[2][3] = 0.0;
//         T34->data[3][0] = 0.0;
//         T34->data[3][1] = 0.0;
//         T34->data[3][2] = 0.0;
//         T34->data[3][3] = 1.0;

//         // 左侧关节5
//         T45->data[0][0] = c4;
//         T45->data[0][1] = -s4;
//         T45->data[0][2] = 0.0;
//         T45->data[0][3] = 0.0;
//         T45->data[1][0] = 0.0;
//         T45->data[1][1] = 0.0;
//         T45->data[1][2] = -1.0;
//         T45->data[1][3] = -0.2163;
//         T45->data[2][0] = s4;
//         T45->data[2][1] = c4;
//         T45->data[2][2] = 0.0;
//         T45->data[2][3] = 0.0;
//         T45->data[3][0] = 0.0;
//         T45->data[3][1] = 0.0;
//         T45->data[3][2] = 0.0;
//         T45->data[3][3] = 1.0;

//         // 左侧关节6
//         T56->data[0][0] = c5;
//         T56->data[0][1] = -s5;
//         T56->data[0][2] = 0.0;
//         T56->data[0][3] = 0.0;
//         T56->data[1][0] = 0.0;
//         T56->data[1][1] = 0.0;
//         T56->data[1][2] = 1.0;
//         T56->data[1][3] = 0.0;
//         T56->data[2][0] = -s5;
//         T56->data[2][1] = -c5;
//         T56->data[2][2] = 0.0;
//         T56->data[2][3] = 0.0;
//         T56->data[3][0] = 0.0;
//         T56->data[3][1] = 0.0;
//         T56->data[3][2] = 0.0;
//         T56->data[3][3] = 1.0;

//         // 左侧关节7
//         T67->data[0][0] = c6;
//         T67->data[0][1] = -s6;
//         T67->data[0][2] = 0.0;
//         T67->data[0][3] = 0.0;
//         T67->data[1][0] = 0.0;
//         T67->data[1][1] = 0.0;
//         T67->data[1][2] = -1.0;
//         T67->data[1][3] = -0.1206;
//         T67->data[2][0] = s6;
//         T67->data[2][1] = c6;
//         T67->data[2][2] = 0.0;
//         T67->data[2][3] = 0.0;
//         T67->data[3][0] = 0.0;
//         T67->data[3][1] = 0.0;
//         T67->data[3][2] = 0.0;
//         T67->data[3][3] = 1.0;
//     }
//     else
//     {
//         // 右侧关节2
//         T12->data[0][0] = s1;
//         T12->data[0][1] = c1;
//         T12->data[0][2] = 0.0;
//         T12->data[0][3] = 0.0;
//         T12->data[1][0] = 0.0;
//         T12->data[1][1] = 0.0;
//         T12->data[1][2] = 1.0;
//         T12->data[1][3] = 0.0;
//         T12->data[2][0] = c1;
//         T12->data[2][1] = -s1;
//         T12->data[2][2] = 0.0;
//         T12->data[2][3] = 0.0;
//         T12->data[3][0] = 0.0;
//         T12->data[3][1] = 0.0;
//         T12->data[3][2] = 0.0;
//         T12->data[3][3] = 1.0;

//         // 右侧关节3
//         T23->data[0][0] = -s2;
//         T23->data[0][1] = -c2;
//         T23->data[0][2] = 0.0;
//         T23->data[0][3] = 0.0;
//         T23->data[1][0] = 0.0;
//         T23->data[1][1] = 0.0;
//         T23->data[1][2] = -1.0;
//         T23->data[1][3] = -0.2153;
//         T23->data[2][0] = c2;
//         T23->data[2][1] = -s2;
//         T23->data[2][2] = 0.0;
//         T23->data[2][3] = 0.0;
//         T23->data[3][0] = 0.0;
//         T23->data[3][1] = 0.0;
//         T23->data[3][2] = 0.0;
//         T23->data[3][3] = 1.0;

//         // 右侧关节4
//         T34->data[0][0] = c3;
//         T34->data[0][1] = -s3;
//         T34->data[0][2] = 0.0;
//         T34->data[0][3] = 0.0;
//         T34->data[1][0] = 0.0;
//         T34->data[1][1] = 0.0;
//         T34->data[1][2] = -1.0;
//         T34->data[1][3] = 0.0;
//         T34->data[2][0] = s3;
//         T34->data[2][1] = c3;
//         T34->data[2][2] = 0.0;
//         T34->data[2][3] = 0.0;
//         T34->data[3][0] = 0.0;
//         T34->data[3][1] = 0.0;
//         T34->data[3][2] = 0.0;
//         T34->data[3][3] = 1.0;

//         // 右侧关节5
//         T45->data[0][0] = c4;
//         T45->data[0][1] = -s4;
//         T45->data[0][2] = 0.0;
//         T45->data[0][3] = 0.0;
//         T45->data[1][0] = 0.0;
//         T45->data[1][1] = 0.0;
//         T45->data[1][2] = 1.0;
//         T45->data[1][3] = 0.2163;
//         T45->data[2][0] = -s4;
//         T45->data[2][1] = -c4;
//         T45->data[2][2] = 0.0;
//         T45->data[2][3] = 0.0;
//         T45->data[3][0] = 0.0;
//         T45->data[3][1] = 0.0;
//         T45->data[3][2] = 0.0;
//         T45->data[3][3] = 1.0;

//         // 右侧关节6
//         T56->data[0][0] = c5;
//         T56->data[0][1] = -s5;
//         T56->data[0][2] = 0.0;
//         T56->data[0][3] = 0.0;
//         T56->data[1][0] = 0.0;
//         T56->data[1][1] = 0.0;
//         T56->data[1][2] = -1.0;
//         T56->data[1][3] = 0.0;
//         T56->data[2][0] = s5;
//         T56->data[2][1] = c5;
//         T56->data[2][2] = 0.0;
//         T56->data[2][3] = 0.0;
//         T56->data[3][0] = 0.0;
//         T56->data[3][1] = 0.0;
//         T56->data[3][2] = 0.0;
//         T56->data[3][3] = 1.0;

//         // 右侧关节7
//         T67->data[0][0] = c6;
//         T67->data[0][1] = -s6;
//         T67->data[0][2] = 0.0;
//         T67->data[0][3] = 0.0;
//         T67->data[1][0] = 0.0;
//         T67->data[1][1] = 0.0;
//         T67->data[1][2] = 1.0;
//         T67->data[1][3] = 0.1206;
//         T67->data[2][0] = -s6;
//         T67->data[2][1] = -c6;
//         T67->data[2][2] = 0.0;
//         T67->data[2][3] = 0.0;
//         T67->data[3][0] = 0.0;
//         T67->data[3][1] = 0.0;
//         T67->data[3][2] = 0.0;
//         T67->data[3][3] = 1.0;
//     }
// }



// void arm_forward_kinematics2(const my_float degree[NUM_SLAVES], bool side)
// {
//     // 使用对齐的矩阵结构体
//     Matrix4x4_t T01, T12, T23, T34, T45, T56, T67;
//     Matrix4x4_t T, T06;
//     my_float rad[NUM_SLAVES];
//     for (int i = 0; i < NUM_SLAVES; i++)
//     {
//         rad[i] = DEG2RAD(degree[i]); // 将角度转换为弧度
//     }

//     // 初始化变换矩阵
//     init_transform_matrices(rad, &T01, &T12, &T23, &T34, &T45, &T56, &T67, side);

//     // 使用 CMSIS-DSP 的矩阵乘法
//     mat4_mul_fpu(&T01, &T12, &T); // T06为中间变量
//     mat4_mul_fpu(&T, &T23, &T06);
//     mat4_mul_fpu(&T06, &T34, &T);
//     mat4_mul_fpu(&T, &T45, &T06);
//     mat4_mul_fpu(&T06, &T56, &T);
//     mat4_mul_fpu(&T, &T67, &T06); // T06为末端位姿

//     if (side == LEFT)
//     {
//         // 输出结果
//         printf("左臂末端执行器位姿矩阵:\n");
//         for (int i = 0; i < 4; i++)
//         {
//             for (int j = 0; j < 4; j++)
//             {
//                 printf("%10.6f ", T06.data[i][j]); //(float)
//             }
//             printf("\n");
//         }

//         // 末端位置
//         printf("左臂末端位置 (m):\n");
//         printf("%10.6f %10.6f %10.6f\n",
//                T06.data[0][3],
//                T06.data[1][3],
//                T06.data[2][3]);

//         my_float ez, ey, ex;
//         dcm2euler_zyx_fpu(T06, &ez, &ey, &ex);
//         printf("左臂末端姿态 (ZYX欧拉角, 角度):\n");
//         printf("%10.6f %10.6f %10.6f\n", RAD2DEG(ez), RAD2DEG(ey), RAD2DEG(ex));
//     }
//     else
//     {
//         // 输出结果
//         printf("右臂末端执行器位姿矩阵:\n");
//         for (int i = 0; i < 4; i++)
//         {
//             for (int j = 0; j < 4; j++)
//             {
//                 printf("%10.6f ", T06.data[i][j]);
//             }
//             printf("\n");
//         }

//         // 末端位置
//         printf("右臂末端位置 (m):\n");
//         printf("%10.6f %10.6f %10.6f\n",
//                T06.data[0][3],
//                T06.data[1][3],
//                T06.data[2][3]);

//         my_float ez, ey, ex;
//         dcm2euler_zyx_fpu(T06, &ez, &ey, &ex);
//         printf("右臂末端姿态 (ZYX欧拉角, 角度):\n");
//         printf("%10.6f %10.6f %10.6f\n", RAD2DEG(ez), RAD2DEG(ey), RAD2DEG(ex));
//     }
// }
