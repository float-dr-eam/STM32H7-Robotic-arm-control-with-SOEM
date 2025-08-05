#include "interpolation.h"
#include "malloc.h"

// 将旋转矩阵转换为轴角表示
void rotm_to_axisangle(Matrix4x4_t rotm, Vector3D* axis, my_float* angle) {
    // 计算迹象
    my_float trace = rotm.data[0][0] + rotm.data[1][1] + rotm.data[2][2];
    *angle = my_acos((trace - 1.0f) / 2.0f);
    
    // 如果角度接近0，则返回默认轴
    if (my_abs(*angle) < 1e-10f || my_abs(*angle - PI) < 1e-10f) {
        axis->x = 0.0f;
        axis->y = 0.0f;
        axis->z = 1.0f; 
        return;
    }

    // 计算旋转轴
    axis->x = rotm.data[2][1] - rotm.data[1][2];
    axis->y = rotm.data[0][2] - rotm.data[2][0];
    axis->z = rotm.data[1][0] - rotm.data[0][1];

    // 归一化旋转轴
    my_float norm = my_sqrt(axis->x * axis->x + axis->y * axis->y + axis->z * axis->z);
    axis->x /= norm;
    axis->y /= norm;
    axis->z /= norm;
}
// 将轴角表示转换为旋转矩阵
Matrix4x4_t axisangle_to_rotm(Vector3D axis, my_float angle) {
    Matrix4x4_t result;
    
    // 归一化旋转轴
    my_float norm = my_sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    axis.x /= norm;
    axis.y /= norm;
    axis.z /= norm;

    // 计算四元数表示
    my_float a = my_cos(angle/2);
    my_float b = -axis.x * my_sin(angle/2);
    my_float c = -axis.y * my_sin(angle/2);
    my_float d = -axis.z * my_sin(angle/2);
    
    // 计算旋转矩阵
    result.data[0][0] = a*a+b*b-c*c-d*d;
    result.data[0][1] = 2*(b*c+a*d);
    result.data[0][2] = 2*(b*d-a*c);
    
    result.data[1][0] = 2*(b*c-a*d);
    result.data[1][1] = a*a-b*b+c*c-d*d;
    result.data[1][2] = 2*(c*d+a*b);
    
    result.data[2][0] = 2*(b*d+a*c);
    result.data[2][1] = 2*(c*d-a*b);
    result.data[2][2] = a*a-b*b-c*c+d*d;
    
    // 计算平移向量
    result.data[0][3] = 0.0f;
    result.data[1][3] = 0.0f;
    result.data[2][3] = 0.0f;
    result.data[3][0] = 0.0f;
    result.data[3][1] = 0.0f;
    result.data[3][2] = 0.0f;
    result.data[3][3] = 1.0f;
    
    return result;
}
// 计算叉积
Vector3D cross_product(Vector3D a, Vector3D b) {
    Vector3D result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

// 计算点积
my_float dot_product(Vector3D a, Vector3D b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 计算向量的模
Vector3D normalize(Vector3D v) {
    my_float norm = my_sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    Vector3D result;
    if (norm < 1e-10f) {
        result.x = 0.0f;
        result.y = 0.0f;
        result.z = 0.0f;
    } else {
        result.x = v.x / norm;
        result.y = v.y / norm;
        result.z = v.z / norm;
    }
    return result;
}

// 计算向量减法
Vector3D vector_subtract(Vector3D a, Vector3D b) 
{
    Vector3D result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

// 计算向量的模
my_float vector_magnitude(Vector3D v) 
{
    return my_sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// 计算向量加法
Vector3D vector_add(Vector3D a, Vector3D b) {
    Vector3D result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}
// 计算向量的缩放
Vector3D vector_scale(Vector3D v, my_float s) {
    Vector3D result;
    result.x = v.x * s;
    result.y = v.y * s;
    result.z = v.z * s;
    return result;
}


// 提取变换矩阵中的位置信息
Vector3D extract_position(Matrix4x4_t transform) {
    Vector3D position;
    position.x = transform.data[0][3];
    position.y = transform.data[1][3];
    position.z = transform.data[2][3];
    return position;
}

// 提取变换矩阵中的旋转信息
Matrix4x4_t extract_rotation(Matrix4x4_t transform) {
    Matrix4x4_t rotation;

    // 提取3x3旋转矩阵部分
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rotation.data[i][j] = transform.data[i][j];
        }

    }

    // 设置齐次坐标的平移部分
    rotation.data[0][3] = 0.0f;
    rotation.data[1][3] = 0.0f;
    rotation.data[2][3] = 0.0f;
    rotation.data[3][0] = 0.0f;
    rotation.data[3][1] = 0.0f;
    rotation.data[3][2] = 0.0f;
    rotation.data[3][3] = 1.0f;
    
    return rotation;
}

// 旋转矩阵转四元数
Quaternion rotm_to_quat(Matrix4x4_t rotm) {
    Quaternion q;
    my_float trace = rotm.data[0][0] + rotm.data[1][1] + rotm.data[2][2];
    
    if (trace > 0) {
        my_float s = 0.5f / my_sqrt(trace + 1.0f);
        q.w = 0.25f / s;
        q.x = (rotm.data[2][1] - rotm.data[1][2]) * s;
        q.y = (rotm.data[0][2] - rotm.data[2][0]) * s;
        q.z = (rotm.data[1][0] - rotm.data[0][1]) * s;
    } else {
        if (rotm.data[0][0] > rotm.data[1][1] && rotm.data[0][0] > rotm.data[2][2]) {
            my_float s = 2.0f * my_sqrt(1.0f + rotm.data[0][0] - rotm.data[1][1] - rotm.data[2][2]);
            q.w = (rotm.data[2][1] - rotm.data[1][2]) / s;
            q.x = 0.25f * s;
            q.y = (rotm.data[0][1] + rotm.data[1][0]) / s;
            q.z = (rotm.data[0][2] + rotm.data[2][0]) / s;
        } else if (rotm.data[1][1] > rotm.data[2][2]) {
            my_float s = 2.0f * my_sqrt(1.0f + rotm.data[1][1] - rotm.data[0][0] - rotm.data[2][2]);
            q.w = (rotm.data[0][2] - rotm.data[2][0]) / s;
            q.x = (rotm.data[0][1] + rotm.data[1][0]) / s;
            q.y = 0.25f * s;

            q.z = (rotm.data[1][2] + rotm.data[2][1]) / s;
        } else {
            my_float s = 2.0f * my_sqrt(1.0f + rotm.data[2][2] - rotm.data[0][0] - rotm.data[1][1]);
            q.w = (rotm.data[1][0] - rotm.data[0][1]) / s;
            q.x = (rotm.data[0][2] + rotm.data[2][0]) / s;
            q.y = (rotm.data[1][2] + rotm.data[2][1]) / s;
            q.z = 0.25f * s;
        }
    }
    
    return q;
}

// 提取变换矩阵中的旋转信息
Matrix4x4_t quat_to_rotm(Quaternion q) {
    Matrix4x4_t rotm;

    // 提取旋转部分
    my_float norm = my_sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    
    // 计算旋转矩阵
    my_float xx = q.x * q.x;
    my_float xy = q.x * q.y;
    my_float xz = q.x * q.z;
    my_float xw = q.x * q.w;
    
    my_float yy = q.y * q.y;
    my_float yz = q.y * q.z;
    my_float yw = q.y * q.w;
    
    my_float zz = q.z * q.z;
    my_float zw = q.z * q.w;
    
    rotm.data[0][0] = 1 - 2 * (yy + zz);
    rotm.data[0][1] = 2 * (xy - zw);
    rotm.data[0][2] = 2 * (xz + yw);
    rotm.data[0][3] = 0.0f;

    
    rotm.data[1][0] = 2 * (xy + zw);
    rotm.data[1][1] = 1 - 2 * (xx + zz);
    rotm.data[1][2] = 2 * (yz - xw);
    rotm.data[1][3] = 0.0f;
    
    rotm.data[2][0] = 2 * (xz - yw);
    rotm.data[2][1] = 2 * (yz + xw);
    rotm.data[2][2] = 1 - 2 * (xx + yy);
    rotm.data[2][3] = 0.0f;
    
    rotm.data[3][0] = 0.0f;
    rotm.data[3][1] = 0.0f;
    rotm.data[3][2] = 0.0f;
    rotm.data[3][3] = 1.0f;
    
    return rotm;
}

// 计算四元数之间的球面线性插值(SLERP)
Quaternion slerp(Quaternion q1, Quaternion q2, my_float t) 
{
    Quaternion result;

    // 计算四元数之间的点积
    my_float dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

    // 如果点积为负，则反转第一个四元数
    if (dot < 0.0f) 
    {
        q1.w = -q1.w;
        q1.x = -q1.x;
        q1.y = -q1.y;
        q1.z = -q1.z;
        dot = -dot;
    }

    // 如果两个四元数几乎相等，则使用线性插值
    if (dot > 0.9995f) {
        result.w = (1.0f - t) * q1.w + t * q2.w;
        result.x = (1.0f - t) * q1.x + t * q2.x;
        result.y = (1.0f - t) * q1.y + t * q2.y;
        result.z = (1.0f - t) * q1.z + t * q2.z;
        
        // 归一化结果四元数
        my_float norm = my_sqrt(result.w*result.w + result.x*result.x + result.y*result.y + result.z*result.z);
        result.w /= norm;
        result.x /= norm;
        result.y /= norm;
        result.z /= norm;
        
        return result;
    }
    
    // 计算插值角度
    my_float theta_0 = my_acos(dot);
    my_float theta = theta_0 * t;
    my_float sin_theta = my_sin(theta);
    my_float sin_theta_0 = my_sin(theta_0);
    my_float s0 = my_cos(theta) - dot * sin_theta / sin_theta_0;
    my_float s1 = sin_theta / sin_theta_0;
    
    result.w = s0 * q1.w + s1 * q2.w;
    result.x = s0 * q1.x + s1 * q2.x;
    result.y = s0 * q1.y + s1 * q2.y;
    result.z = s0 * q1.z + s1 * q2.z;
    
    return result;
}

// 创建变换矩阵
Matrix4x4_t create_transform(Vector3D position, Matrix4x4_t rotation) {
    Matrix4x4_t transform;

    // 提取旋转部分
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            transform.data[i][j] = rotation.data[i][j];
        }
    }

    // 设置齐次坐标的平移部分
    transform.data[0][3] = position.x;
    transform.data[1][3] = position.y;
    transform.data[2][3] = position.z;
    
    // 设置齐次坐标的旋转部分
    transform.data[3][0] = 0.0f;
    transform.data[3][1] = 0.0f;
    transform.data[3][2] = 0.0f;
    transform.data[3][3] = 1.0f;
    
    return transform;

}
// 创建五次多项式轨迹
Trajectory* quintic_line_trajectory(Matrix4x4_t T_start, Matrix4x4_t T_end, int num_points) 
{
    Trajectory* traj = (Trajectory*)mymalloc(SRAMEX, sizeof(Trajectory));
    traj->num_points = num_points;
    traj->transforms = (Matrix4x4_t*)mymalloc(SRAMEX, num_points * sizeof(Matrix4x4_t));
    traj->positions = (Vector3D*)mymalloc(SRAMEX, num_points * sizeof(Vector3D));
    
    // 提取起始点和结束点位置
    Vector3D p_start = extract_position(T_start);
    Vector3D p_end = extract_position(T_end);
    
    // 提取起始点和结束点旋转部分
    Matrix4x4_t R_start = extract_rotation(T_start);
    Matrix4x4_t R_end = extract_rotation(T_end);
    
    // 创建四元数
    Quaternion q_start = rotm_to_quat(R_start);
    Quaternion q_end = rotm_to_quat(R_end);
    
    // 设置时间参数
    my_float tf = 1.0; // 初始时间
    Vector3D a0 = p_start;
    //Vector3D a1 = {0, 0, 0};
    //Vector3D a2 = {0, 0, 0};
    Vector3D a3 = vector_scale(vector_subtract(p_end, p_start), 10.0f / (tf*tf*tf));
    Vector3D a4 = vector_scale(vector_subtract(p_end, p_start), -15.0f / (tf*tf*tf*tf));
    Vector3D a5 = vector_scale(vector_subtract(p_end, p_start), 6.0f / (tf*tf*tf*tf*tf));

    // 计算每个时间点的变换矩阵
    for (int i = 0; i < num_points; i++) 
    {
        my_float t = (my_float)i / (num_points - 1); // 当前时间 [0,1]
        my_float t2 = t * t;
        my_float t3 = t2 * t;
        my_float t4 = t3 * t;
        my_float t5 = t4 * t;

        // 计算位置
        Vector3D term3 = vector_scale(a3, t3);
        Vector3D term4 = vector_scale(a4, t4);
        Vector3D term5 = vector_scale(a5, t5);
        Vector3D p = a0;
        p = vector_add(p, term3);
        p = vector_add(p, term4);
        p = vector_add(p, term5);

        // 存储关键帧位置
        traj->positions[i] = p;

        // SLERP球面线性插值
        Quaternion q = slerp(q_start, q_end, t);

        // 创建旋转矩阵
        Matrix4x4_t R = quat_to_rotm(q);
        
        // 缓存变换矩阵
        traj->transforms[i] = create_transform(p, R);
    }
    
    return traj;
}

// 创建弧线轨迹
Trajectory* arc_trajectory(  Matrix4x4_t T_start, Vector3D p_mid , Matrix4x4_t T_end, int num_points,my_float* radius,Vector3D* center) 
{
    Vector3D p_start=extract_position(T_start);
    Vector3D p_end=extract_position(T_end);
    Trajectory* traj = (Trajectory*)mymalloc(SRAMEX, sizeof(Trajectory));
    traj->num_points = num_points;
    traj->transforms = (Matrix4x4_t*)mymalloc(SRAMEX, num_points * sizeof(Matrix4x4_t));
    traj->positions = (Vector3D*)mymalloc(SRAMEX, num_points * sizeof(Vector3D));
    
    // 1. 计算中间点位置
    my_float a = vector_magnitude(vector_subtract(p_end, p_mid));
    my_float b = vector_magnitude(vector_subtract(p_end, p_start));
    my_float c = vector_magnitude(vector_subtract(p_mid, p_start));
    
    // 2. 计算弧线的半径
    my_float s = (a + b + c) / 2.0f;
    *radius = a * b * c / (4.0f * my_sqrt(s * (s - a) * (s - b) * (s - c)));

    // 3. 计算圆心
    // 创建矩阵 [p_start; p_mid; p_end] 的逆
    my_float P[3][3] = {
        {p_start.x, p_start.y, p_start.z},
        {p_mid.x, p_mid.y, p_mid.z},
        {p_end.x, p_end.y, p_end.z}
    };

    // 计算权重
    my_float w1 = a * a * (b * b + c * c - a * a);
    my_float w2 = b * b * (a * a + c * c - b * b);
    my_float w3 = c * c * (a * a + b * b - c * c);
    my_float wtot = w1 + w2 + w3;
    
    // 计算圆心
    center->x = (w1 * P[0][0] + w2 * P[1][0] + w3 * P[2][0]) / wtot;
    center->y = (w1 * P[0][1] + w2 * P[1][1] + w3 * P[2][1]) / wtot;
    center->z = (w1 * P[0][2] + w2 * P[1][2] + w3 * P[2][2]) / wtot;
    
    // 4. 计算起始点和结束点的单位向量
    Vector3D v_start = vector_subtract(p_start, *center);
    Vector3D v_end = vector_subtract(p_end, *center);
    // 计算起始点和结束点的单位向量
    Vector3D u_start = normalize(v_start);
    Vector3D u_end = normalize(v_end);
    
    // 5. 计算旋转轴和角度
    // 计算旋转轴和角度
    Vector3D rotation_axis = cross_product(u_start, u_end);
    rotation_axis = normalize(rotation_axis);
    
    // 计算旋转角度
    my_float theta = my_acos(dot_product(u_start, u_end));
    
    // 计算中间点的角度
    Vector3D u_mid = normalize(vector_subtract(p_mid, *center));
    my_float theta_mid = my_acos(dot_product(u_start, u_mid));

    // 计算旋转矩阵 - 如果使用某个函数来计算旋转矩阵，可以将其封装成一个函数
    Matrix4x4_t test_rot = axisangle_to_rotm(rotation_axis, theta_mid);
    Vector3D test_vector = {
        test_rot.data[0][0] * v_start.x + test_rot.data[0][1] * v_start.y + test_rot.data[0][2] * v_start.z,
        test_rot.data[1][0] * v_start.x + test_rot.data[1][1] * v_start.y + test_rot.data[1][2] * v_start.z,
        test_rot.data[2][0] * v_start.x + test_rot.data[2][1] * v_start.y + test_rot.data[2][2] * v_start.z
    };
    
    Vector3D test_point = vector_add(*center, test_vector);
    if (vector_magnitude(vector_subtract(test_point, p_mid)) > 0.1f * (*radius)) 
    {
        theta = 2 * PI - theta;
    }

    // 6. 提取起始点和结束点的旋转矩阵
    Matrix4x4_t R_start = extract_rotation(T_start);
    Matrix4x4_t R_end = extract_rotation(T_end);

    // 创建四元数
    Quaternion q_start = rotm_to_quat(R_start);
    Quaternion q_end = rotm_to_quat(R_end);

    // 7. 生成关键帧
    my_float theta_step = theta / (num_points - 1);
    
    for (int i = 0; i < num_points; i++) {
        // 计算当前角度
        my_float current_angle = i * theta_step;

        // 计算旋转矩阵
        Matrix4x4_t R_current = axisangle_to_rotm(rotation_axis, current_angle);
        
        // printf("R_current%d:\n",i);
        // print_matrix(R_current);
        // 计算当前点的位移向量
        my_float v_startx = v_start.x;
        my_float v_starty = v_start.y;
        my_float v_startz = v_start.z;
        
        Vector3D v_current;
        v_current.x = R_current.data[0][0] * v_startx + R_current.data[0][1] * v_starty + R_current.data[0][2] * v_startz;
        v_current.y = R_current.data[1][0] * v_startx + R_current.data[1][1] * v_starty + R_current.data[1][2] * v_startz;
        v_current.z = R_current.data[2][0] * v_startx + R_current.data[2][1] * v_starty + R_current.data[2][2] * v_startz;
        // printf("v_current:%d:\n",i);
        // printf("%.4f,%.4f,%.4f\n",v_current.x,v_current.y,v_current.z);
        // 计算当前点的位置
        Vector3D p_current = vector_add(*center, v_current);
        // printf("p_current:%d:\n",i);
        // printf("%.4f,%.4f,%.4f\n",p_current.x,p_current.y,p_current.z);
        // 存储轨迹点位置
        traj->positions[i] = p_current;

        // 使用SLERP插值四元数
        my_float t = (my_float)i / (num_points - 1);
        Quaternion q_current = slerp(q_start, q_end, t);
        
        // 转换为旋转矩阵
        Matrix4x4_t R_pose = quat_to_rotm(q_current);
        
        // 存储轨迹点旋转矩阵
        traj->transforms[i] = create_transform(p_current, R_pose);
    }
    
    return traj;
}

// 打印轨迹信息
void print_trajectory(Trajectory* traj, const char* name) 
{
    printf("\n=== %s轨迹信息 ===\n", name);
    printf("轨迹点数量: %d\n", traj->num_points);
    
    // 打印部分轨迹点（如果点很多，只打印开始、中间和结束部分）
    int num_to_print = (traj->num_points <= 10) ? traj->num_points : 10;
    
    for (int i = 0; i < num_to_print; i++) 
    {
        int idx;
        if (traj->num_points <= 10) 
        {
            idx = i;
        } 
        else if (i < 3) 
        {

            idx = i; // 打印开始部分
        } 
        else if (i < 6) 
        {
            idx = traj->num_points / 2 - 3 + i - 3; // 打印中间部分
        } 
        else 
        {
            idx = traj->num_points - 10 + i; // 打印结束部分
        }
        
        printf("\n轨迹点 %d:\n", idx);
        printf("位置: (%.4f, %.4f, %.4f)\n", 
               traj->positions[idx].x, 
               traj->positions[idx].y, 
               traj->positions[idx].z);
        printf("旋转矩阵:\n");
        print_matrix(traj->transforms[idx]);
    }
}

// 释放轨迹内存
void free_trajectory(Trajectory* traj) {
    myfree(SRAMEX, traj->transforms);
    myfree(SRAMEX, traj->positions);
    myfree(SRAMEX, traj);
}

// 轨迹规划测试函数
void trajectory_planning_test() {
    printf("\n=== 轨迹规划测试 ===\n");
    
    // 1. 直线轨迹测试
    printf("\n--- 直线轨迹测试 ---\n");
    Matrix4x4_t start_pose = transl(-0.5522f, 0.0f, 0.1299f);
    Matrix4x4_t rotation1 = rot(X, -90.0f);
    Matrix4x4_t rotation2 = rot(Y, -90.0f);
    start_pose = multiply_matrix(start_pose, rotation1);
    start_pose = multiply_matrix(start_pose, rotation2);
    
    Matrix4x4_t end_pose = transl(0.0f, -0.216f, 0.466f);
    Matrix4x4_t rotation3 = rot(Z, 90.0f);
    end_pose = multiply_matrix(end_pose, rotation3);
    
    printf("起始位姿:\n");
    print_matrix(start_pose);
    
    printf("目标位姿:\n");
    print_matrix(end_pose);
    
    int num_points = 51;
    Trajectory* line_traj = quintic_line_trajectory(start_pose, end_pose, num_points);
    
    print_trajectory(line_traj, "直线轨迹");
    
    // 2. 弧线轨迹测试
    printf("\n--- 弧线轨迹测试 ---\n");
    //Vector3D p_start = {-0.5522f, 0.0f, 0.1299f};//起始点
    Vector3D p_mid = {-0.2f, -0.20f, 0.20f};
    //Vector3D p_end = {0.0f, -0.216f, 0.466f};
    my_float radius;
    Vector3D center;
    Trajectory* arc_traj = arc_trajectory( start_pose, p_mid, end_pose, num_points,&radius,&center);
    printf("弧线轨迹半径: %.4f\n", radius);
    printf("弧线轨迹中心: (%.4f, %.4f, %.4f)\n", center.x, center.y, center.z);
    print_trajectory(arc_traj, "弧线轨迹");
    
    // 释放内存
    free_trajectory(line_traj);
    free_trajectory(arc_traj);
}

// 轨迹规划验证函数
void validate_trajectory_planning() {
    printf("\n=== 轨迹规划验证 ===\n");
    
    // 1. 直线轨迹验证
    printf("\n--- 直线轨迹验证 ---\n");
    Matrix4x4_t start_pose = transl(-0.5522f, 0.0f, 0.1299f);
    Matrix4x4_t rotation1 = rot(X, -90.0f);
    Matrix4x4_t rotation2 = rot(Y, -90.0f);
    start_pose = multiply_matrix(start_pose, rotation1);
    start_pose = multiply_matrix(start_pose, rotation2);
    
    Matrix4x4_t end_pose = transl(0.0f, -0.216f, 0.466f);
    Matrix4x4_t rotation3 = rot(Z, 90.0f);

    end_pose = multiply_matrix(end_pose, rotation3);
    
    int num_points = 51;
    Trajectory* line_traj = quintic_line_trajectory(start_pose, end_pose, num_points);
    
    // 验证起始点和结束点
    Vector3D start_pos = extract_position(start_pose);
    Vector3D end_pos = extract_position(end_pose);
    Vector3D traj_start = line_traj->positions[0];
    Vector3D traj_end = line_traj->positions[num_points-1];
    
    printf("起点验证:\n");
    printf("设定起点: (%.6f, %.6f, %.6f)\n", start_pos.x, start_pos.y, start_pos.z);
    printf("轨迹起点: (%.6f, %.6f, %.6f)\n", traj_start.x, traj_start.y, traj_start.z);
    printf("误差: %.9f\n", vector_magnitude(vector_subtract(start_pos, traj_start)));
    
    printf("终点验证:\n");
    printf("设定终点: (%.6f, %.6f, %.6f)\n", end_pos.x, end_pos.y, end_pos.z);
    printf("轨迹终点: (%.6f, %.6f, %.6f)\n", traj_end.x, traj_end.y, traj_end.z);
    printf("误差: %.9f\n", vector_magnitude(vector_subtract(end_pos, traj_end)));
    
    // 验证直线性质
    printf("直线性质验证:\n");
    Vector3D direction = normalize(vector_subtract(end_pos, start_pos));
    my_float max_deviation = 0.0f;
    int worst_point = 0;
    
    for (int i = 1; i < num_points-1; i++) {
        Vector3D v = vector_subtract(line_traj->positions[i], start_pos);
        my_float proj_length = dot_product(v, direction);
        Vector3D proj_point = {
            start_pos.x + direction.x * proj_length,
            start_pos.y + direction.y * proj_length,
            start_pos.z + direction.z * proj_length
        };
        my_float deviation = vector_magnitude(vector_subtract(line_traj->positions[i], proj_point));
        
        if (deviation > max_deviation) {
            max_deviation = deviation;
            worst_point = i;
        }
    }
    
    printf("最大偏离直线距离: %.9f (点 %d)\n", max_deviation, worst_point);
    
    // 2. 圆弧轨迹验证
    printf("\n--- 圆弧轨迹验证 ---\n");
    Vector3D p_mid = {-0.2f, -0.20f, 0.20f};
    my_float radius;
    Vector3D center;
    Trajectory* arc_traj = arc_trajectory(start_pose, p_mid, end_pose, num_points, &radius, &center);
    
    // 验证半径一致性
    printf("圆半径验证:\n");
    printf("计算半径: %.6f\n", radius);
    my_float max_radius_error = 0.0f;
    int worst_radius_point = 0;
    
    for (int i = 0; i < num_points; i++) {
        my_float point_radius = vector_magnitude(vector_subtract(arc_traj->positions[i], center));
        my_float radius_error = my_abs(point_radius - radius);
        
        if (radius_error > max_radius_error) {
            max_radius_error = radius_error;
            worst_radius_point = i;
        }
    }
    
    printf("最大半径误差: %.9f (点 %d)\n", max_radius_error, worst_radius_point);
    
    // 验证三点共圆
    printf("三点共圆验证:\n");
    my_float start_radius = vector_magnitude(vector_subtract(traj_start, center));
    my_float mid_point_radius = vector_magnitude(vector_subtract(p_mid, center));
    my_float end_radius = vector_magnitude(vector_subtract(traj_end, center));

    printf("起点到圆心距离: %.6f (误差: %.9f)\n", start_radius, fabs(start_radius - radius));
    printf("中间点到圆心距离: %.6f (误差: %.9f)\n", mid_point_radius, fabs(mid_point_radius - radius));
    printf("终点到圆心距离: %.6f (误差: %.9f)\n", end_radius, fabs(end_radius - radius));
    
    // 验证中间点是否在轨迹的范围内
    my_float min_dist_to_mid = INFINITY;
    int closest_point = 0;
    
    for (int i = 0; i < num_points; i++) {
        my_float dist = vector_magnitude(vector_subtract(arc_traj->positions[i], p_mid));
        if (dist < min_dist_to_mid) {
            min_dist_to_mid = dist;
            closest_point = i;
        }
    }
    
    printf("中间点到轨迹的最小距离: %.9f (点 %d)\n", min_dist_to_mid, closest_point);
    
    // 释放内存
    free_trajectory(line_traj);
    free_trajectory(arc_traj);
}


// 将位姿轨迹转换为关节轨迹
JointTrajectory* trajectory_to_joint_angles(Trajectory* traj, Robot* robot) 
{
    JointTrajectory* joint_traj = (JointTrajectory*)mymalloc(SRAMEX, sizeof(JointTrajectory));
    joint_traj->num_points = traj->num_points;
    joint_traj->joint_angles = (my_float**)mymalloc(SRAMEX, traj->num_points * sizeof(my_float*));
    
    // 初始化关节角度
    my_float q_init[DOF] = {0, 0, 0, 0, 0, 0, 0};
    
    // 逆向运动学求解每个点的关节角度
    for (int i = 0; i < traj->num_points; i++) 
    {
        // 为每个关节分配内存
        joint_traj->joint_angles[i] = (my_float*)mymalloc(SRAMEX, DOF * sizeof(my_float));
        
        // 将位姿转换为4x4的变换矩阵
        my_float T_desired[16];
        for (int row = 0; row < 4; row++) 
        {
            for (int col = 0; col < 4; col++) 
            {
                T_desired[row*4 + col] = traj->transforms[i].data[row][col];
            }
        }
        // 使用逆运动学计算关节角度
        // 对于第一个点，使用初始猜测；对于后续点，使用前一点的结果作为初始猜测
        my_float* q_guess = (i == 0) ? q_init : joint_traj->joint_angles[i-1];
        my_float T_current[16];
        my_float error[6];
        int iter = inverse_kinematics(robot, T_desired, q_guess, joint_traj->joint_angles[i],  1000, 1e-6f, 0.1f, 0.5f, 1); // 璋冪敤閫嗚繍鍔拷?姹傝В
        if (iter <= 0) 
        {
            forward_kinematics(robot, joint_traj->joint_angles[i], T_current);
            compute_pose_error(T_current, T_desired, error);
            my_float error_norm = vector_norm(error, 6);
            printf("第%d次迭代，点%d的逆运动学求解失败,误差：%f\n", iter, i, error_norm);
        }
        else
        {
            // 计算当前点的正向运动学
            forward_kinematics(robot, joint_traj->joint_angles[i], T_current);
            // 计算当前点的误差
            compute_pose_error(T_current, T_desired, error);
            // 计算误差的范数
            my_float error_norm = vector_norm(error, 6);
            printf("第%d次迭代，点%d的逆运动学求解成功，误差：%f\n", iter, i, error_norm);
        }
    }
    
    return joint_traj;
}

// 打印关节轨迹
void print_joint_trajectory(JointTrajectory* joint_traj) {
    printf("\n=== 关节角度轨迹 ===\n");
    printf("轨迹点数量: %d\n", joint_traj->num_points);
    
    // 打印部分轨迹点（如果点很多，只打印开始、中间和结束部分）
    int num_to_print = (joint_traj->num_points <= 10) ? joint_traj->num_points : 10;
    
    for (int i = 0; i < num_to_print; i++) {
        int idx;
        if (joint_traj->num_points <= 10) {
            idx = i;
        } else if (i < 3) {
            idx = i; // 前3个点
        } else if (i < 6) {
            idx = joint_traj->num_points / 2 - 3 + i - 3; // 中间3个点
        } else {
            idx = joint_traj->num_points - 10 + i; // 最后3个点
        }
        
        printf("\n点 %d 关节角度:\n", idx);
        for (int j = 0; j < DOF; j++) {
            printf("关节%d: %.6f 弧度 (%.2f 度)\n", j, 
                   joint_traj->joint_angles[idx][j],
                   joint_traj->joint_angles[idx][j] * 180.0f / PI);
        }
    }
}

// 释放关节轨迹内存
void free_joint_trajectory(JointTrajectory* joint_traj) {
    for (int i = 0; i < joint_traj->num_points; i++) {
        myfree(SRAMEX, joint_traj->joint_angles[i]);
    }
    myfree(SRAMEX, joint_traj->joint_angles);
    myfree(SRAMEX, joint_traj);
}

// 在轨迹规划验证函数中添加关节角度计算
void trajectory_planning_with_joints(void) {
    Robot robot;
    init_robot(&robot);

    printf("\n=== 轨迹规划与关节轨迹验证 ===\n");

    // 1. 直线轨迹
    //printf("\n--- 直线轨迹 ---\n");
    Matrix4x4_t start_pose = transl(-0.5522f, 0.0f, 0.1299f);
    Matrix4x4_t rotation1 = rot(X, -90.0f);
    Matrix4x4_t rotation2 = rot(Y, -90.0f);
    start_pose = multiply_matrix(start_pose, rotation1);
    start_pose = multiply_matrix(start_pose, rotation2);
    
    Matrix4x4_t end_pose = transl(0.0f, -0.216f, 0.466f);
    Matrix4x4_t rotation3 = rot(Z, 90.0f);
    end_pose = multiply_matrix(end_pose, rotation3);
    
    int num_points = 51;
    // Trajectory* line_traj = quintic_line_trajectory(start_pose, end_pose, num_points);// 计算直线轨迹

    // // 计算关节角度轨迹
    // JointTrajectory* line_joint_traj = trajectory_to_joint_angles(line_traj, &robot);
    // print_joint_trajectory(line_joint_traj);
    
    // 2. 圆弧轨迹
    printf("\n--- 圆弧轨迹 ---\n");
    Vector3D p_mid = {-0.2f, -0.20f, 0.20f};
    my_float radius;
    Vector3D center;
    Trajectory* arc_traj = arc_trajectory(start_pose, p_mid, end_pose, num_points, &radius, &center);// 计算圆弧轨迹

    // 计算关节角度轨迹（弧度制）
    JointTrajectory* arc_joint_traj = trajectory_to_joint_angles(arc_traj, &robot);
    print_joint_trajectory(arc_joint_traj);
    //改为角度制
    for (size_t i = 0; i < arc_joint_traj->num_points; i++)//50
    {
        for (size_t j = 0; j < DOF; j++)//7
        {
            arc_joint_traj->joint_angles[i][j] *= 180.0f / PI;
        }
    }

    // 释放轨迹内存
    //free_trajectory(line_traj);
    free_trajectory(arc_traj);

    start_nonblocking_motion(arc_joint_traj); // 开始非阻塞运动
    //free_joint_trajectory(line_joint_traj);
    //free_joint_trajectory(arc_joint_traj);
}

// // 轨迹规划与关节轨迹验证demo函数
// JointTrajectory* demo(void) 
// {
//     //trajectory_planning_test();
//     // validate_trajectory_planning();
//     //JointTrajectory* arc_joint_traj = 

//     //free_joint_trajectory(arc_joint_traj);

//     return trajectory_planning_with_joints();
// }


