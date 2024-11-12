#include "kalman_filter.h"
#include <cmath>
#include <Eigen/Dense>
#include <vector>

KalmanFilter::KalmanFilter(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance) {
    x = initial_state;
    P = initial_covariance;

    // Process covariance matrix Q
    Q = Eigen::MatrixXd::Zero(5, 5);
    Q.diagonal() << 0.3, 0.3, 0.04, 0.1, 0.04;

    // Covariance matrices for GPS and IMU
    R_gps = Eigen::MatrixXd::Zero(4, 4);
    R_imu = Eigen::MatrixXd::Zero(4, 4);
    R_imu(2, 2) = 0.0159;  // IMU orientation tuning
    R_imu(3, 3) = 0.04;    // IMU angular velocity tuning

    initial_x = 0;
    initial_y = 0;
    initial_theta = 0;
}

void KalmanFilter::predict(double a_x) {
    double dt = 0.01;
    double theta = x(2);
    double v = x(3);
    double omega = x(4);

    // Update state based on the system model
    x(0) += v * std::cos(theta) * dt;
    x(1) += v * std::sin(theta) * dt;
    x(2) += omega * dt;
    x(3) += a_x * dt;
    x(4) = omega;

    // Jacobian matrix F
    Eigen::MatrixXd F(5, 5);
    F << 1, 0, -v * std::sin(theta) * dt, std::cos(theta) * dt, 0,
         0, 1, v * std::cos(theta) * dt, std::sin(theta) * dt, 0,
         0, 0, 1, 0, dt,
         0, 0, 0, 1, 0,
         0, 0, 0, 0, 1;

    // Predict covariance
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    // Measurement matrix H
    Eigen::MatrixXd H(4, 5);
    H << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0,
         0, 0, 1, 0, 0,
         0, 0, 0, 0, 1;

    // Combined GPS and IMU measurement covariance
    Eigen::MatrixXd R = R_gps + R_imu;

    // Measurement residual (innovation)
    Eigen::VectorXd y = z - H * x;

    // Innovation covariance S
    Eigen::MatrixXd S = H * P * H.transpose() + R;

    // Kalman gain K
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    // Update state and covariance
    x = x + K * y;
    P = P - K * H * P;
}

void KalmanFilter::initializeGPS(double x_utm, double y_utm) {
    initial_x = x_utm;
    initial_y = y_utm;
    x(0) = x_utm;
    x(1) = y_utm;
}

std::vector<double> KalmanFilter::expandCovarianceMatrix() {
    std::vector<double> covariance(36, 0.0);

    // Fill covariance matrix with required values
    covariance[0] = P(0, 0);
    covariance[1] = P(0, 1);
    covariance[6] = P(0, 2);
    covariance[7] = P(1, 1);
    covariance[14] = P(2, 2);
    covariance[21] = P(3, 3);
    covariance[28] = P(4, 4);
    covariance[11] = P(1, 2);

    // Symmetric elements
    covariance[5] = P(1, 0);
    covariance[15] = P(2, 0);
    covariance[16] = P(2, 1);

    return covariance;
}
