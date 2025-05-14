#include "pi_imu_ekf.h"
#include <cmath>

// 基础矩阵运算工具函数
Matrix3d _skew_symmetric3(Vector3d v)
{
    Matrix3d m;
    m << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return m;
}

// 构造函数：初始化基本变量
img_imu_ekf::img_imu_ekf()
{
    is_init_done = false;
    is_atti_init_done = false;
    this->q.setIdentity();
    this->pos.setZero();
    this->vel.setZero();
    this->img.setZero();
    this->acc_bias.setZero();
    this->gyro_bias.setZero();

    this->Phi = StateCov::Identity();
    this->G = NoiseMatrix::Zero();

    // 添加R_cb初始化
    R_cb << 0,  0,  1,
            -1,  0,  0,
            0, -1,  0;
}

img_imu_ekf::~img_imu_ekf()
{
}

// 初始化状态估计器
void img_imu_ekf::sensor_init(const Vector3d& pos, const Vector3d& vel, const Vector2d& img)
{
    VectorXd x0(8);
    // p_r初始化为[5,5,5]，v_r初始化为[0.5,0.5,0.5]
    x0.segment(0,3) = Vector3d(5.0, 5.0, 5.0);
    x0.segment(3,3) = Vector3d(0.5, 0.5, 0.5);
    x0.segment(6,2) = img;
    // std::cout << "[sensor_init] x0: " << x0.transpose() << std::endl;
    kf.init_state(x0);

    this->set_P_matrix();
    this->set_Q_matrix();
}

// 非线性状态预测函数
void img_imu_ekf::predict_state_nonlinear(const Quaterniond& q, const Vector3d& acc, 
                                         const Vector3d& gyro, double dt)
{
    Vector3d p_r = kf.x.segment(0, 3);
    Vector3d v_r = kf.x.segment(3, 3);
    Vector2d img = kf.x.segment(6, 2);
    
    Matrix3d R_eb = compute_R_eb(q);
    Vector3d a_e = R_eb * acc;
    
    p_r = p_r + v_r * dt + 0.5 * a_e * dt * dt;
    v_r = v_r + a_e * dt;
    
    Matrix3d R_ec = R_eb * R_cb.transpose();
    Vector3d p_c = R_ec.transpose() * (-p_r);

    // 硬性除0保护
    if (std::abs(p_c(2)) < 1e-6) {
        if (p_c(2) >= 0)
            p_c(2) = 1e-6;
        else
            p_c(2) = -1e-6;
    }
    
    Matrix<double,2,6> Ls = compute_Ls_matrix(img, p_c);
    
    Eigen::Matrix<double, 6, 1> u;
    u.head<3>() = R_ec * v_r;      // 速度部分
    u.tail<3>() = R_cb.transpose() * gyro;     // 角速度部分

    img = img + dt * Ls * u;

    kf.x.segment(0, 3) = p_r;
    kf.x.segment(3, 3) = v_r;
    kf.x.segment(6, 2) = img;
}

img_imu_ekf::VelocityMatrix img_imu_ekf::compute_Ls_matrix(const Vector2d& img, const Vector3d& p_c) const
{
    VelocityMatrix Ls;
    double z = p_c(2);
    // 硬性除0保护
    if (std::abs(z) < 1e-6) {
        if (z >= 0)
            z = 1e-6;
        else
            z = -1e-6;
    }
    double inv_z = 1.0 / z;

    Ls << -inv_z, 0, img(0)*inv_z, img(0)*img(1), -(1+img(0)*img(0)), img(1),
          0, -inv_z, img(1)*inv_z, 1+img(1)*img(1), -img(0)*img(1), -img(0);
          
    return Ls;
}

Matrix3d img_imu_ekf::compute_R_eb(const Quaterniond& q) const
{
    Matrix3d R;
    double q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
    
    R << q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2),
         2*(q1*q2+q0*q3), q0*q0-q1*q1+q2*q2-q3*q3, 2*(q2*q3-q0*q1),
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0*q0-q1*q1-q2*q2+q3*q3;
         
    return R.transpose();
}

// EKF协方差更新
void img_imu_ekf::update_covariance_ekf(double dt)
{
    compute_jacobian_matrix(acc, omega, dt);
    kf.predict(Phi, G, Q);
}

// 延迟状态更新
void img_imu_ekf::update_delayed_state(const StateVector& delayed_state, 
                                      const StateCov& delayed_cov,
                                      const Vector2d& measurement)
{
    MatrixXd H = MatrixXd::Zero(2, 8);
    H.block(0,6,2,2) = Matrix2d::Identity();
    kf.update(delayed_state, delayed_cov, measurement, H, R);
}

void img_imu_ekf::compute_jacobian_matrix(const Vector3d& acc, const Vector3d& gyro, double dt)
{
    Vector3d p_r = kf.x.segment<3>(0);
    Vector3d v_r = kf.x.segment<3>(3);
    Vector2d img = kf.x.segment<2>(6);

    Matrix3d R_eb = compute_R_eb(q_eb);
    Matrix3d R_ec = R_eb * R_cb;
    Vector3d p_c = -R_ec.transpose() * p_r;
    Vector3d v_c = R_ec.transpose() * v_r;
    Vector3d w_c = R_cb.transpose() * gyro;

    if (std::abs(p_c(2)) < 1e-6) {
        if (p_c(2) >= 0)
            p_c(2) = 1e-6;
        else
            p_c(2) = -1e-6;
    }

    Phi.setIdentity(8, 8);
    Phi.block<3,3>(0, 3) = Matrix3d::Identity() * dt;

    double inv_z = 1.0 / p_c(2);
    Matrix<double,2,3> F_v_img;
    F_v_img << -inv_z, 0, img(0)*inv_z,
               0, -inv_z, img(1)*inv_z;
    // 修正：先右乘再赋值，不能用 *=
    F_v_img = F_v_img * (R_cb.transpose() * R_eb.transpose() * dt);

    Phi.block<2,3>(6, 3) = F_v_img;

    MatrixXd F_img_img(2, 2);
    F_img_img << v_c(2)*inv_z + img(1)*w_c(0) - 2*img(0)*w_c(1), img(0)*w_c(0) + w_c(2),
                 -img(1)*w_c(1) - w_c(2), v_c(2)*inv_z + 2*img(1)*w_c(0) - img(0)*w_c(1);
    Phi.block(6, 6, 2, 2) = Matrix2d::Identity() + F_img_img * dt;

    MatrixXd G_q_pr = compute_G_q_pr(acc, q_eb, dt);
    MatrixXd G_q_vr = compute_G_q_vr(acc, q_eb, dt);
    MatrixXd G_q_img = compute_G_q_img(img, v_r, q_eb, p_c(2), dt);
    MatrixXd G_w_img = compute_G_w_img(img, R_cb);

    G = NoiseMatrix::Zero(8, 10);

    G.block(0, 0, 3, 4) = G_q_pr;
    G.block(0, 7, 3, 3) = 0.5 * Matrix3d::Identity() * dt * dt;

    G.block(3, 0, 3, 4) = G_q_vr;
    G.block(3, 7, 3, 3) = Matrix3d::Identity() * dt;

    G.block(6, 0, 2, 4) = G_q_img;
    G.block(6, 4, 2, 3) = G_w_img;
}

// 辅助函数实现M1、M2、M3矩阵组 (用于G_q_pr和G_q_vr)
MatrixXd img_imu_ekf::compute_M123_matrix(const Vector3d& acc, const Quaterniond& q) const 
{
    double q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
    
    Matrix<double,4,3> M1;
    M1 << q0, -q3,  q2,
          q1,  q2,  q3,
         -q2,  q1,  q0,
         -q3, -q0,  q1;

    Matrix<double,4,3> M2;
    M2 << q3,  q0, -q1,
          q2, -q1, -q0,
          q1,  q2,  q3,
          q0, -q3,  q2;

    Matrix<double,4,3> M3;
    M3 << -q2,  q1,  q0,
          q3,  q0, -q1,
         -q0,  q3, -q2,
          q1,  q2,  q3;

    Vector3d b_a = acc;
    Matrix<double,4,9> M123;
    M123.block<4,3>(0,0) = M1;
    M123.block<4,3>(0,3) = M2;
    M123.block<4,3>(0,6) = M3;
    
    return M123;
}

Matrix<double,4,3> img_imu_ekf::compute_M4_matrix(const Vector2d& img, const Quaterniond& q) const
{
    double q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
    double px = img(0);
    
    Matrix<double,4,3> M4;
    M4 << 2*px*q0+2*q3, 2*px*q3-2*q0, -2*px*q2-2*q1,
          2*px*q1-2*q2, 2*px*q2+2*q1, 2*px*q3-2*q0,
          2*px*q2-2*q1, 2*px*q1-2*q2, -2*px*q0-2*q3,
          2*px*q3+2*q0, 2*px*q0+2*q3, 2*px*q1-2*q2;
    return M4;
}

Matrix<double,4,3> img_imu_ekf::compute_M5_matrix(const Vector2d& img, const Quaterniond& q) const
{
    double q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
    double py = img(1);
    
    Matrix<double,4,3> M5;
    M5 << 2*py*q0-2*q2, 2*py*q3+2*q1, -2*py*q2-2*q0,
          2*py*q1-2*q3, 2*py*q2+2*q0, 2*py*q3+2*q1,
          2*py*q2-2*q0, 2*py*q1-2*q3, -2*py*q0+2*q2,
          2*py*q3-2*q1, 2*py*q0-2*q2, 2*py*q1-2*q3;
    return M5;
}

MatrixXd img_imu_ekf::compute_G_q_pr(const Vector3d& acc, const Quaterniond& q, double dt) const 
{
    MatrixXd M123 = compute_M123_matrix(acc, q);
    
    Vector3d b_a = acc;
    Matrix<double,4,3> M123_ba;
    M123_ba.col(0) = M123.block<4,3>(0,0) * b_a;
    M123_ba.col(1) = M123.block<4,3>(0,3) * b_a;
    M123_ba.col(2) = M123.block<4,3>(0,6) * b_a;
    
    return M123_ba.transpose() * dt * dt;
}

MatrixXd img_imu_ekf::compute_G_q_vr(const Vector3d& acc, const Quaterniond& q, double dt) const 
{
    MatrixXd M123 = compute_M123_matrix(acc, q);
    
    Vector3d b_a = acc;
    Matrix<double,4,3> M123_ba;
    M123_ba.col(0) = M123.block<4,3>(0,0) * b_a;
    M123_ba.col(1) = M123.block<4,3>(0,3) * b_a;
    M123_ba.col(2) = M123.block<4,3>(0,6) * b_a;
    
    return 2.0 * M123_ba.transpose() * dt;
}

MatrixXd img_imu_ekf::compute_G_q_img(const Vector2d& img, const Vector3d& v_r, 
                                     const Quaterniond& q, double pz, double dt) const 
{
    if (std::abs(pz) < 1e-6) {
        if (pz >= 0)
            pz = 1e-6;
        else
            pz = -1e-6;
    }
    
    Matrix<double,4,3> M4 = compute_M4_matrix(img, q);
    Matrix<double,4,3> M5 = compute_M5_matrix(img, q);
    
    Matrix<double,4,1> M4v = M4 * v_r;
    Matrix<double,4,1> M5v = M5 * v_r;
    
    Matrix<double,4,2> M45v;
    M45v.col(0) = M4v;
    M45v.col(1) = M5v;
    
    return (1.0/pz) * M45v.transpose() * dt;
}

MatrixXd img_imu_ekf::compute_G_w_img(const Vector2d& img, const Matrix3d& R_cb) const 
{
    double px = img(0);
    double py = img(1);
    
    Matrix<double,2,3> G_w;
    G_w << px*py, -(1 + px*px), py,
           1 + py*py, -px*py, -px;
    
    return G_w * R_cb.transpose();
}

// 初始化噪声矩阵
void img_imu_ekf::set_Q_matrix()
{
    Q = NoiseCov::Zero(10, 10);
    Matrix3d I3 = Matrix3d::Identity();
    Matrix4d I4 = Matrix4d::Identity();
    
    Q.block(0, 0, 4, 4) = I4 * (GYRO_NOISE_DENSITY) * (GYRO_NOISE_DENSITY);
    Q.block(4, 4, 3, 3) = I3 * (OMEGA_NOISE_DENSITY) * (OMEGA_NOISE_DENSITY);
    Q.block(7, 7, 3, 3) = I3 * (ACC_NOISE_DENSITY) * (ACC_NOISE_DENSITY);

    R = Matrix<double,2,2>::Zero();
    double sigma_px = IMG_NOISE_DENSITY;
    double sigma_py = IMG_NOISE_DENSITY;
    R(0,0) = sigma_px * sigma_px;
    R(1,1) = sigma_py * sigma_py;
}

void img_imu_ekf::set_P_matrix()
{
    VectorXd p = VectorXd::Ones(8);
    p.segment(0, 3) = Vector3d::Ones() * GPS_POS_NOISE_DENSITY * GPS_POS_NOISE_DENSITY;
    p.segment(3, 3) = Vector3d::Ones() * GPS_VEL_NOISE_DENSITY * GPS_VEL_NOISE_DENSITY;
    p.segment(6, 2) = Vector2d::Ones() * IMG_NOISE_DENSITY * IMG_NOISE_DENSITY;

    kf.init_covariance(p);
}
