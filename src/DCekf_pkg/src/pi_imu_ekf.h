#ifndef __PI_IMU_EKF_H
#define __PI_IMU_EKF_H

#include "DCeskf.h"
#include "DCkf_config.h"
#include <Eigen/Dense>
#include <deque>

using namespace Eigen;

class img_imu_ekf
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef eskf::StateVector StateVector;
    typedef eskf::StateCov StateCov;
    typedef eskf::MeasMatrix MeasMatrix;
    typedef eskf::NoiseMatrix NoiseMatrix;
    typedef eskf::MeasVector MeasVector;
    typedef eskf::NoiseCov NoiseCov;

    typedef std::deque<StateVector, Eigen::aligned_allocator<StateVector>> StateBuffer;
    typedef std::deque<StateCov, Eigen::aligned_allocator<StateCov>> CovBuffer;

    static const int BUFFER_SIZE = 100;
    static const int DELAY_STEPS = 3;

    eskf kf;

    Vector3d p_r;
    Vector3d v_r;
    Vector2d img;

    Quaterniond q_eb;
    Vector3d omega;
    Vector3d acc;

    const int dim = 8;
    double img_f = 320;

    const double gyro_noise = GYRO_NOISE_DENSITY;
    const double acc_noise = ACC_NOISE_DENSITY;
    const double img_noise = IMG_NOISE_DENSITY;

    StateCov Phi;
    NoiseMatrix G;
    NoiseCov Q;
    Matrix<double,2,2> R;

    Matrix3d R_cb;

    Vector3d mav_pos;
    Vector3d mav_vel;
    Vector2d mav_img;
    Vector2d img0;
    bool get_sensor;
    bool is_img_come;

    bool is_init_done;
    bool is_atti_init_done;
    Quaterniond q;
    Vector3d pos;
    Vector3d vel;
    Vector3d acc_bias;
    Vector3d gyro_bias;

    img_imu_ekf();
    ~img_imu_ekf();

    void sensor_init(const Vector3d& pos, const Vector3d& vel, const Vector2d& img);

    void predict_state_nonlinear(const Quaterniond& q, const Vector3d& acc,
                                 const Vector3d& gyro, double dt);
    void update_covariance_ekf(double dt);
    void update_delayed_state(const StateVector& delayed_state,
                              const StateCov& delayed_cov,
                              const Vector2d& mav_img);

    VectorXd get_current_state() const { return kf.x; }
    MatrixXd get_current_covariance() const { return kf.P; }

    void set_Q_matrix();
    void set_P_matrix();

protected:
    void compute_jacobian_matrix(const Vector3d& acc, const Vector3d& gyro, double dt);

    typedef Matrix<double,2,6> VelocityMatrix;

    StateBuffer state_buffer;
    CovBuffer cov_buffer;

    VelocityMatrix compute_Ls_matrix(const Vector2d& img, const Vector3d& p_c) const;
    Matrix3d compute_R_eb(const Quaterniond& q) const;

    Matrix<double,4,3> compute_M4_matrix(const Vector2d& img, const Quaterniond& q) const;
    Matrix<double,4,3> compute_M5_matrix(const Vector2d& img, const Quaterniond& q) const;
    MatrixXd compute_M123_matrix(const Vector3d& acc, const Quaterniond& q) const;
    MatrixXd compute_G_q_pr(const Vector3d& acc, const Quaterniond& q, double dt) const;
    MatrixXd compute_G_q_vr(const Vector3d& acc, const Quaterniond& q, double dt) const;
    MatrixXd compute_G_q_img(const Vector2d& img, const Vector3d& v_r,
                             const Quaterniond& q, double pz, double dt) const;
    MatrixXd compute_G_w_img(const Vector2d& img, const Matrix3d& R_cb) const;
};

#endif