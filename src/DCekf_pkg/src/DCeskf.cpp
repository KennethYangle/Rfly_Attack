#include "DCeskf.h"

eskf::eskf() {
    x.setZero();
    P.setIdentity();
}

eskf::~eskf() {}

void eskf::init_state(const StateVector& x0) {
    x = x0;
}

void eskf::init_covariance(const StateVector& p0) {
    P.setZero();
    for (int i = 0; i < p0.size(); ++i) {
        P(i, i) = p0(i);
    }
}

void eskf::predict(const StateCov& Phi, const NoiseMatrix& G, const NoiseCov& Q) {
    // x = Phi * x;
    P = Phi * P * Phi.transpose() + G * Q * G.transpose();
}

void eskf::update(const StateVector& delayed_state, const StateCov& delayed_cov,
                  const MeasVector& measurement, const MeasMatrix& H,
                  const Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>& R) {
    // 用延迟状态和协方差进行更新
    Eigen::Matrix<double, MEAS_DIM, MEAS_DIM> S = H * delayed_cov * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = delayed_cov * H.transpose() * S.inverse();
    x = delayed_state + K * (measurement - H * delayed_state);
    P = (StateCov::Identity() - K * H) * delayed_cov;
}

void eskf::update(const MeasVector& measurement, const MeasMatrix& H,
                  const Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>& R) {
    Eigen::Matrix<double, MEAS_DIM, MEAS_DIM> S = H * P * H.transpose() + R;
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = P * H.transpose() * S.inverse();
    x = x + K * (measurement - H * x);
    P = (StateCov::Identity() - K * H) * P;
}