#ifndef __ESKF_H
#define __ESKF_H

#include <Eigen/Dense>

class eskf {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum {
        STATE_DIM = 8,
        MEAS_DIM = 2,
        NOISE_DIM = 10
    };

    typedef Eigen::Matrix<double, STATE_DIM, 1> StateVector;
    typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> StateCov;
    typedef Eigen::Matrix<double, MEAS_DIM, STATE_DIM> MeasMatrix;
    typedef Eigen::Matrix<double, STATE_DIM, NOISE_DIM> NoiseMatrix;
    typedef Eigen::Matrix<double, MEAS_DIM, 1> MeasVector;
    typedef Eigen::Matrix<double, NOISE_DIM, NOISE_DIM> NoiseCov;

    eskf();
    ~eskf();

    void init_state(const StateVector& x0);
    void init_covariance(const StateVector& p0);

    void predict(const StateCov& Phi, const NoiseMatrix& G, const NoiseCov& Q);

    void update(const StateVector& delayed_state, const StateCov& delayed_cov,
                const MeasVector& measurement, const MeasMatrix& H,
                const Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>& R);

    void update(const MeasVector& measurement, const MeasMatrix& H,
                const Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>& R);

    // 获取接口
    const StateVector& get_state() const { return x; }
    const StateCov& get_covariance() const { return P; }

    StateVector x;
    StateCov P;
};

#endif