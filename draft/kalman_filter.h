#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>
#include <vector>

class KalmanFilter {
public:
    KalmanFilter(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance);
    
    void predict(double a_x);
    void update(const Eigen::VectorXd& z);
    void initializeGPS(double x_utm, double y_utm);
    std::vector<double> expandCovarianceMatrix();

private:
    Eigen::VectorXd x;  // State vector
    Eigen::MatrixXd P;  // Covariance matrix
    Eigen::MatrixXd Q;  // Process covariance matrix
    Eigen::MatrixXd R_gps;  // GPS covariance matrix
    Eigen::MatrixXd R_imu;  // IMU covariance matrix

    double initial_x, initial_y, initial_theta;
};

#endif
