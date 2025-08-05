#include "my_arm.h"
#include "malloc.h"

void matrix_multiply(my_float *A, my_float *B, my_float *C, int m, int n, int p);
void matrix_transpose(my_float *A, my_float *AT, int m, int n);
my_float vector_norm(my_float *v, int n);

// 初始化机械臂
void init_robot(Robot *robot)
{
    // 关节1
    robot->dh_params[0].alpha = 0.0;
    robot->dh_params[0].a = 0.0;
    robot->dh_params[0].d = 0.1299;
    robot->dh_params[0].theta = 0.0; // 关节1旋转角度
    robot->dh_params[0].offset = 0.0;
    // 关节2
    robot->dh_params[1].alpha = -PI / 2;
    robot->dh_params[1].a = 0.0;
    robot->dh_params[1].d = 0.0;
    robot->dh_params[1].theta = 0.0; // 关节2旋转角度
    robot->dh_params[1].offset = -PI / 2;
    // 关节3
    robot->dh_params[2].alpha = PI / 2;
    robot->dh_params[2].a = 0.0;
    robot->dh_params[2].d = 0.2153;
    robot->dh_params[2].theta = 0.0; // 关节3旋转角度
    robot->dh_params[2].offset = PI / 2;
    // 关节4
    robot->dh_params[3].alpha = PI / 2;
    robot->dh_params[3].a = 0.0;
    robot->dh_params[3].d = 0.0;
    robot->dh_params[3].theta = 0.0; // 关节4旋转角度
    robot->dh_params[3].offset = 0.0;
    // 关节5
    robot->dh_params[4].alpha = -PI / 2;
    robot->dh_params[4].a = 0.0;
    robot->dh_params[4].d = 0.2163;
    robot->dh_params[4].theta = 0.0; // 关节5旋转角度
    robot->dh_params[4].offset = 0.0;
    // 关节6
    robot->dh_params[5].alpha = PI / 2;
    robot->dh_params[5].a = 0.0;
    robot->dh_params[5].d = 0.0;
    robot->dh_params[5].theta = 0.0; // 关节6旋转角度
    robot->dh_params[5].offset = 0.0;
    // 关节7
    robot->dh_params[6].alpha = -PI / 2;
    robot->dh_params[6].a = 0.0;
    robot->dh_params[6].d = 0.1206;
    robot->dh_params[6].theta = 0.0; // 关节7旋转角度
    robot->dh_params[6].offset = 0.0;
    // 初始化关节角度范围   
    for (int i = 0; i < DOF; i++)
    {
        robot->q_min[i] = -PI;
        robot->q_max[i] = PI;
    }
}

// 计算变换矩阵
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

// 计算前向运动学
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
    // 计算变换矩阵
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

        // 计算关节i位置
        joint_positions[i][0] = T[3];
        joint_positions[i][1] = T[7];
        joint_positions[i][2] = T[11];

        // 计算关节i轴
        joint_axes[i][0] = T[2];
        joint_axes[i][1] = T[6];
        joint_axes[i][2] = T[10];
    }

    // 计算末端执行器位置
    end_position[0] = T[3];
    end_position[1] = T[7];
    end_position[2] = T[11];

    // 计算雅可比矩阵
    for (int i = 0; i < DOF; i++)
    {
        // 计算关节i到末端执行器的向量
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
            R_current_T[i * 3 + j] = R_current[j * 4 + i]; // 计算R_current的转置矩阵

    // dR = R_target * R_current'
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            dR[i * 3 + j] = 0;
            for (int k = 0; k < 3; k++)
                dR[i * 3 + j] += R_target[i * 4 + k] * R_current_T[k * 3 + j];
        }

    // 由dR提取旋转向量
    my_float theta = acos(my_fmin(my_fmax((dR[0] + dR[4] + dR[8] - 1) / 2, -1.0), 1.0));
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

// 计算位姿误差（位置和旋转）
void compute_pose_error(my_float *T_current, const my_float *T_desired, my_float *error)
{
    // 位置误差（3维）
    error[0] = T_desired[3] - T_current[3];
    error[1] = T_desired[7] - T_current[7];
    error[2] = T_desired[11] - T_current[11];

    // 旋转误差（3维）
    my_float w[3];
    rotation_error(T_current, T_desired, w);
    error[3] = w[0];
    error[4] = w[1];
    error[5] = w[2];
}
// arm_mat_inverse_f32(T, T_inv);
// 计算n阶方阵A的逆，结果存到Ainv，返回1成功，0失败
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
        printf("JJT_lam不可逆\n");//失败
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

// // 零空间优化目标（以靠近中位为例）
// void nullspace_objective(my_float *q, my_float *q_mid, my_float *grad)
// {
//     for (int i = 0; i < DOF; ++i)
//         grad[i] = q[i] - q_mid[i];
// }

void nullspace_objective(my_float *q, my_float *q_min, my_float *q_max, my_float *grad) 
{
    const my_float k = 0.05; // 增益系数
    for (int i = 0; i < DOF; ++i)
    {
        // 计算归一化关节位置
        my_float q_mid = (q_max[i] + q_min[i]) / 2.0f;
        my_float range = q_max[i] - q_min[i];
        my_float m = (q[i] - q_mid) / range;

        // 避免m为0导致的问题
        my_float m_safe = m;
        if (fabs(m_safe) < 1e-6)
        {
            m_safe = 1e-6 * (m_safe >= 0 ? 1 : -1);
        }

        // 计算关节速度：k × ln((1+m)/m) 或 k × ln((1-m)/(-m))
        if (m_safe > 0)
        {
            grad[i] = -k * log((1 + m_safe) / m_safe);
        }
        else
        {
            grad[i] = k * log((1 - m_safe) / (-m_safe));
        }
    }
}

// 矩阵行列式计算（适用于6x6矩阵）
my_float matrix_determinant(my_float *A, int n) {
    my_float det = 0.0;
    my_float temp[6][6];
    int i, j, k, sign = 1;
    
    // 仅支持6x6矩阵
    if (n != 6) return 0.0;
    
    // 复制矩阵
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            temp[i][j] = A[i*n + j];

    // 进行高斯消元
    for (i = 0; i < n; i++) {
        // 寻找主元
        int pivot = i;
        for (j = i; j < n; j++) {
            if (fabs(temp[j][i]) > fabs(temp[pivot][i]))
                pivot = j;
        }
        
        if (pivot != i) {
            // 交换行
            for (k = 0; k < n; k++) {
                my_float t = temp[i][k];
                temp[i][k] = temp[pivot][k];
                temp[pivot][k] = t;
            }
            sign *= -1;
        }

        // 进行高斯消元
        for (j = i+1; j < n; j++) {
            my_float factor = temp[j][i] / temp[i][i];
            for (k = i; k < n; k++) {
                temp[j][k] -= factor * temp[i][k];
            }
        }
    }
    
    // 计算对角元素乘积
    det = sign;
    for (i = 0; i < n; i++)
        det *= temp[i][i];
    
    return det;
}

// SVD分解（简化实现，适用于6x7矩阵）
void matrix_svd(my_float *A, int m, int n, my_float *U, my_float *S, my_float *V) {
    // 简化实现，实际应用需使用完整SVD算法
    // 此处仅为演示框架，返回对角线元素作为奇异值近似
    memset(U, 0, m*m*sizeof(my_float));
    memset(V, 0, n*n*sizeof(my_float));
    memset(S, 0, fmin(m,n)*sizeof(my_float));
    
    // 初始化U和V为单位矩阵
    for (int i = 0; i < m; i++) U[i*m + i] = 1.0;
    for (int i = 0; i < n; i++) V[i*n + i] = 1.0;
    
    // 计算J*J^T的特征值作为奇异值平方的近似
    my_float *JT = (my_float*)mymalloc(SRAMEX, n*m*sizeof(my_float));
    my_float *JJT = (my_float*)mymalloc(SRAMEX, m*m*sizeof(my_float));
    
    matrix_transpose(A, JT, m, n);
    matrix_multiply(A, JT, JJT, m, n, m);

    // 计算奇异值
    for (int i = 0; i < m; i++) {
        S[i] = sqrt(fabs(JJT[i*m + i]));
    }
    
    // 对奇异值排序（从大到小）
    for (int i = 0; i < m; i++) {
        for (int j = i+1; j < m; j++) {
            if (S[i] < S[j]) {
                my_float t = S[i];
                S[i] = S[j];
                S[j] = t;
            }
        }
    }
    
    myfree(SRAMEX, JT);
    myfree(SRAMEX, JJT);
}
// 逆运动学主循环（返回弧度值结果）
int inverse_kinematics(
    Robot *robot,const my_float *T_desired, my_float *q_init, my_float *q_result, int max_iter, my_float epsilon, my_float lambda_max, my_float step_size, int verbose)
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

    my_float *U = (my_float *)mymalloc(SRAMEX, 6*6 * sizeof(my_float));  // 6x6 matrix
    my_float *S = (my_float *)mymalloc(SRAMEX, 6 * sizeof(my_float));     // 6-element vector
    my_float *V = (my_float *)mymalloc(SRAMEX, 7*7 * sizeof(my_float));  // 7x7 matrix
    my_float *JT = (my_float *)mymalloc(SRAMEX, 7*6 * sizeof(my_float)); // 7x6 matrix
    my_float *JJT = (my_float *)mymalloc(SRAMEX, 6*6 * sizeof(my_float));// 6x6 matrix
    my_float manipulability, manipulability_threshold = 0.01f;
    my_float sigma_0 = 100.0f, sigma1, sigma2, sigma,lambda;
    if (error_history == NULL || q == NULL || T_current == NULL || J == NULL || J_pinv == NULL || error == NULL || dq == NULL || q_mid == NULL||U==NULL||S==NULL||V==NULL||JT==NULL||JJT==NULL)
    {
        printf("inverse_kinematics: memory allocation failed\n");
        myfree(SRAMEX, q);
        myfree(SRAMEX, T_current);
        myfree(SRAMEX, J);
        myfree(SRAMEX, J_pinv);
        myfree(SRAMEX, error);
        myfree(SRAMEX, dq);
        myfree(SRAMEX, q_mid);
        myfree(SRAMEX, error_history);
        myfree(SRAMEX, U);
        myfree(SRAMEX, S);
        myfree(SRAMEX, V);
        myfree(SRAMEX, JT);
        myfree(SRAMEX, JJT);
        return -1; // inverse_kinematics: memory allocation failed
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
            myfree(SRAMEX, U);
            myfree(SRAMEX, S);
            myfree(SRAMEX, V);
            myfree(SRAMEX, JT);
            myfree(SRAMEX, JJT);
            return iter + 1; // 收敛
        }
        // 3. 计算雅可比矩阵
        compute_jacobian(robot, q, J);

        // // 4. 计算当前位姿与期望位姿之间的误差
        // my_float prev_err_norm = (iter > 0) ? error_history[iter-1] : INFINITY;
        // if (err_norm < prev_err_norm)
        //     lambda = my_fmax(0.0001f, lambda * 0.7f);
        // else
        //     lambda = my_fmin(0.1f, lambda * 1.3f);

        // error_history[iter] = err_norm; 

        // damped_pseudoinverse(J, J_pinv, 6, DOF, lambda);
        // // 5. 计算当前位姿与期望位姿之间的误差
        // my_float grad[DOF];
        // my_float q_mid[DOF];//中间关节角度
        // nullspace_objective(q, q_mid, grad);
        // 4. 计算关节空间的目标函数
        matrix_transpose(J, JT, 6, DOF);
        matrix_multiply(J, JT, JJT, 6, DOF, 6);

        // 计算可操作度 sqrt(det(J*J^T))
        manipulability = sqrt(fabs(matrix_determinant(JJT, 6)));

        // SVD分解
        matrix_svd(J, 6, DOF, U, S, V);

        // 计算奇异值比 sigma = sigma2/sigma1
        sigma1 = S[5]; // 最小奇异值
        sigma2 = S[0]; // 最大奇异值
        sigma = sigma2 / sigma1;

        // 计算可操作度
        if (manipulability > manipulability_threshold) 
        {
            // 适应性调整
            lambda = 0.0001; // 最小阻尼系数
        } 
        else 
        {
            
            if (sigma <= sigma_0) 
            {
                lambda = 0.0001;
            } 
            else // 处于奇异位置，使用阻尼最小二乘法
            {
                lambda = lambda_max * sqrt((1 - sigma_0/sigma) * (1 - sigma_0/sigma));
            }
        }
        
        // 5. 计算伪逆
        damped_pseudoinverse(J, J_pinv, 6, DOF, lambda);
        
        // 5. 零空间优化梯度
        my_float grad[DOF];
        nullspace_objective(q, robot->q_min, robot->q_max, grad);

        // 6. 零空间投影
        my_float N[DOF * DOF]; // N = I - J_pinv * J
        // 先算 J_pinv * J
        my_float JPJ[DOF * DOF];
        matrix_multiply(J_pinv, J, JPJ, DOF, 6, DOF);
        // N = I - JPJ
        // 计算零空间投影矩阵 N
        for (int i = 0; i < DOF * DOF; ++i)
            N[i] = 0.0f;
        for (int i = 0; i < DOF; ++i)
            N[i * DOF + i] = 1.0f;
        for (int i = 0; i < DOF * DOF; ++i)
            N[i] -= JPJ[i];

        // 7. dq = J_pinv * error - alpha * N * grad
        my_float dq1[DOF], dq2[DOF];
        matrix_multiply(J_pinv, error, dq1, DOF, 6, 1);
        matrix_multiply(N, grad, dq2, DOF, DOF, 1);
        my_float alpha = 0.01f; // 零空间优化系数

        // 零空间优化
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
        // 8. 更新关节角度
        for (int i = 0; i < DOF; ++i)
        {
            q[i] += dq[i];
            // 限制关节角度在范围内
            if (q[i] < robot->q_min[i])
                q[i] = robot->q_min[i];
            if (q[i] > robot->q_max[i])
                q[i] = robot->q_max[i];
        }
        //error_history[iter] = err_norm;
    }
    // 复制最终关节角度到结果
    memcpy(q_result, q, DOF * sizeof(my_float));
    myfree(SRAMEX, q);
    myfree(SRAMEX, T_current);
    myfree(SRAMEX, J);
    myfree(SRAMEX, J_pinv);
    myfree(SRAMEX, error);
    myfree(SRAMEX, dq);
    myfree(SRAMEX, q_mid);
    myfree(SRAMEX, error_history);
    myfree(SRAMEX, U);
    myfree(SRAMEX, S);
    myfree(SRAMEX, V);
    myfree(SRAMEX, JT);
    myfree(SRAMEX, JJT);
    return 0;
}

// 矩阵乘法函数 
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

// 矩阵转置函数
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



// 向量范数计算函数
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

    // 初始化机器人模型
    init_robot(robot);

    // 计算关节角度的弧度值
    for (int i = 0; i < DOF; i++)
    {
        q_rad[i] = DEG2RAD(q_init[i]);
    }

    // 计算前向运动学
    forward_kinematics(robot, q_rad, T_result);

    // 输出前向运动学结果
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
//返回角度制
void arm_inverse_kinematics(const my_float *T_desired, const my_float *q_init, my_float *q_result)
{
    Robot *robot = (Robot *)mymalloc(SRAMEX, sizeof(Robot));
    my_float *J = (my_float *)mymalloc(SRAMEX, 6 * DOF * sizeof(my_float));
    my_float *q_rad = (my_float *)mymalloc(SRAMEX, DOF * sizeof(my_float));
    my_float *T_current = (my_float *)mymalloc(SRAMEX, 16 * sizeof(my_float));
    if (T_desired == NULL || q_init == NULL || q_result == NULL)
    {
        printf("Memory  error!\n");
        return; // 内存错误
    }
    if (J == NULL || robot == NULL || q_rad == NULL || T_current == NULL)
    {
        printf("Memory allocation failed!\n");
        myfree(SRAMEX, J);
        myfree(SRAMEX, robot);
        myfree(SRAMEX, q_rad);
        myfree(SRAMEX, T_current);
        return; // 内存错误
    }
    // 初始化机器人模型
    init_robot(robot);
    for (int i = 0; i < DOF; i++)
    {
        q_rad[i] = DEG2RAD(q_init[i]);
    }
    // 计算逆向运动学
    int n;
    if ((n = inverse_kinematics(robot, T_desired, q_rad, q_result, MAX_ITER, EPSILON, LAMBDA, STEP_SIZE, IF_PRINT)) != 0)
    {
        printf((const char *)"逆向运动学求解失败，错误代码：%d\n", n);
        printf("请检查输入的期望位姿和初始关节角度是否正确\n");
        for (int i = 0; i < DOF; i++)
        {
            printf("q[%d] = %10.6f 度\n", i, q_result[i]);
        }
        printf("求解失败，错误代码：%d\n", n);
        my_float error[6];
        forward_kinematics(robot, q_result, T_current);
        // printf("T_current: \n");
        // for (int i = 0; i < 4; i++)
        // {
        //     for (int j = 0; j < 4; j++)
        //     {
        //         printf("%10.6f ", T_current[i * 4 + j]);
        //     }
        //     printf("\n");
        // }
        // printf("\n");
        compute_pose_error(T_current, T_desired, error);
        my_float error_norm = vector_norm(error, 6);
        printf("位姿误差范数：%f\n", error_norm);
        for (int i = 0; i < DOF; i++)
        {
            q_result[i] = RAD2DEG(q_result[i]); // 弧度值转换为角度值
        }
    }
    else
    {
        printf("逆向运动学求解成功\n");
        my_float error[6];
        forward_kinematics(robot, q_result, T_current);
        compute_pose_error(T_current, T_desired, error);
        my_float error_norm = vector_norm(error, 6);
        printf("位姿误差范数：%f\n", error_norm);
    }
    myfree(SRAMEX, J);
    myfree(SRAMEX, robot);
    myfree(SRAMEX, q_rad);
    myfree(SRAMEX, T_current);
}

void simple_demo(void)
{
    my_float *T_desired= (my_float *)mymalloc(SRAMEX, 16 * sizeof(my_float));
   
    // Matrix4x4_t translation1 = transl(0.0, -0.216, 0.466);
    // Matrix4x4_t rotation1 = rot(Z,90);
    // Matrix4x4_t T1 = mat4_mul_fpu(&translation1, &rotation1);
    // print_matrix(T1);
    // assign_matrix_to_float_ptr(&T1, &T_desired);

    // Matrix4x4_t start_pose = transl(-0.5522, 0, 0.1299);
    // Matrix4x4_t rotation1 = rot(X, -90);
    // Matrix4x4_t rotation2 = rot(Y, -90);
    // start_pose = multiply_matrix(start_pose, rotation1);
    // start_pose = multiply_matrix(start_pose, rotation2);
    // assign_matrix_to_float_ptr(&start_pose, &T_desired);
    // 计算期望位姿 T_desired
    // T_desired[0] = 0.0000f; T_desired[1] = -0.0314f; T_desired[2] = -0.9995f; T_desired[3] = -0.5522f;
    // T_desired[4] = 1.0000f; T_desired[5] = 0.0000f; T_desired[6] = 0.0000f; T_desired[7] = -0.0000f;
    // T_desired[8] = 0.0000f; T_desired[9] = -0.9995f; T_desired[10] = 0.0314f; T_desired[11] = 0.1299f;
    // T_desired[12] = 0.0000f; T_desired[13] = 0.0000f; T_desired[14] = 0.0000f; T_desired[15] = 1.0000f;

    T_desired[0] = -0.0117f; T_desired[1] = -0.0233f; T_desired[2] = -0.9997f; T_desired[3] = -0.5522f;
    T_desired[4] = 0.9999f; T_desired[5] = -0.0117f; T_desired[6] = -0.0114f; T_desired[7] = 0.0000f;
    T_desired[8] = -0.0114f; T_desired[9] = -0.9997f; T_desired[10] = 0.0234f; T_desired[11] = 0.1299f;
    T_desired[12] = 0.0000f; T_desired[13] = 0.0000f; T_desired[14] = 0.0000f; T_desired[15] = 1.0000f;


    my_float *q_result = (my_float *)mymalloc(SRAMEX, DOF * sizeof(my_float));
    if (q_result == NULL)
    {
        printf("Memory allocation failed!\n");
        myfree(SRAMEX, q_result);
        return; // 内存错误
    }
    //arm_inverse_kinematics(T_desired, LD3M_all.cur_degree, q_result);
     // 计算关节角度的弧度值
    my_float q_init[DOF] = {0, 0, 0, 0, 0, 0, 0};
    arm_inverse_kinematics(T_desired, q_init, q_result);
    motors_run_to_angles(q_result); // 关节运动到目标角度
    myfree(SRAMEX, T_desired);

}


void dcm2euler_zyx(const my_float *T, my_float *z, my_float *y, my_float *x)
{
    // 计算旋转矩阵
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


// 4*4矩阵相乘
Matrix4x4_t mat4_mul_fpu(const Matrix4x4_t *A, const Matrix4x4_t *B)
{
    Matrix4x4_t C;
#if USE_DOUBLE_PRECISION
    arm_matrix_instance_f64 matA = {4, 4, (my_float *)A->data};
    arm_matrix_instance_f64 matB = {4, 4, (my_float *)B->data};
    arm_matrix_instance_f64 matC = {4, 4, (my_float *)C.data};

    arm_mat_mult_f64(&matA, &matB, &matC);
#else
    arm_matrix_instance_f32 matA = {4, 4, (my_float *)A->data};
    arm_matrix_instance_f32 matB = {4, 4, (my_float *)B->data};
    arm_matrix_instance_f32 matC = {4, 4, (my_float *)C.data};
    arm_mat_mult_f32(&matA, &matB, &matC);
#endif
return C;
}
// 4*4矩阵相乘
Matrix4x4_t multiply_matrix(Matrix4x4_t a, Matrix4x4_t b) 
{
    Matrix4x4_t result;
    
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            result.data[i][j] = 0.0;
            for (int k = 0; k < 4; k++) 
            {
                result.data[i][j] += a.data[i][k] * b.data[k][j];
            }
        }
    }
    
    return result;
}
// 4*4矩阵平移
Matrix4x4_t transl(my_float x, my_float y, my_float z) 
{
    Matrix4x4_t result;
    // 计算平移矩阵
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            result.data[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    // 计算平移矩阵
    result.data[0][3] = x;
    result.data[1][3] = y;
    result.data[2][3] = z;
    
    return result;
}

// 计算旋转矩阵
Matrix4x4_t rot(RotationAxis axis, my_float angle_deg) 
{
    Matrix4x4_t result;
    // 角度转换为弧度
    my_float angle_rad = angle_deg * PI / 180.0f;
    my_float cos_val = cos(angle_rad);
    my_float sin_val = sin(angle_rad);
    
    // 计算旋转矩阵
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result.data[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    // 计算旋转矩阵
    switch (axis) {
        case X:  // 绕X轴旋转
            result.data[1][1] = cos_val;
            result.data[1][2] = -sin_val;
            result.data[2][1] = sin_val;
            result.data[2][2] = cos_val;
            break;
            
        case Y:  // 绕Y轴旋转
            result.data[0][0] = cos_val;
            result.data[0][2] = sin_val;
            result.data[2][0] = -sin_val;
            result.data[2][2] = cos_val;
            break;

        case Z:  // 绕Z轴旋转
            result.data[0][0] = cos_val;
            result.data[0][1] = -sin_val;
            result.data[1][0] = sin_val;
            result.data[1][1] = cos_val;
            break;
    }
    
    return result;
}
// 将 Matrix4x4_t 结构体中的数据赋值给 my_float 指针
void assign_matrix_to_float_ptr(Matrix4x4_t *src, my_float **dst) 
{
    *dst = (my_float *)src->data;
}
// 将 my_float 指针中的数据赋值给 Matrix4x4_t 结构体
void assign_float_ptr_to_matrix(my_float *src, Matrix4x4_t *dst) 
{
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            dst->data[i][j] = src[i * 4 + j];
        }
    }
}

//打印矩阵
void print_matrix(Matrix4x4_t m) 
{
    for (int i = 0; i < 4; i++) 
    {
        printf("[ ");
        for (int j = 0; j < 4; j++) 
        {
            printf("%7.4f ", m.data[i][j]);
        }
        printf("]\n");
    }
    printf("\n");
}
