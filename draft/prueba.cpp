#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cmath>
#include <Eigen/Dense>
#include <utm/utm.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class EKFNode {
public:
    EKFNode() : first_gps_reading(true), first_imu_reading(true), gps_received(false), dt(0.01) {
        // Inicialización del estado y las matrices de covarianza
        x = VectorXd(5);
        x << 0, 0, 0, 0, 0; // [x, y, theta, v, omega]

        P = MatrixXd::Zero(5, 5);
        P.diagonal() << 0.70, 0.70, 0.01, 0.01, 0.04;

        Q = MatrixXd::Zero(5, 5);
        Q.diagonal() << 0.3, 0.3, 0.04, 0.1, 0.04;

        R_gps = MatrixXd::Zero(4, 4);
        R_imu = MatrixXd::Zero(4, 4);
        R_imu(2, 2) = 0.0159; // Orientación
        R_imu(3, 3) = 0.04;   // Velocidad angular

        // Inicialización de ROS
        ros::NodeHandle nh;
        imu_sub = nh.subscribe("/imu/data", 10, &EKFNode::imuCallback, this);
        gps_sub = nh.subscribe("/fix", 10, &EKFNode::gpsCallback, this);
        pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ekf_pose", 10);
        timer = nh.createTimer(ros::Duration(0.01), &EKFNode::broadcastOdomToBaseLink, this); // 100Hz

        ROS_INFO("EKFNode initialized.");
    }

private:
    VectorXd x;  // Estado
    MatrixXd P;  // Covarianza del estado
    MatrixXd Q;  // Covarianza del proceso
    MatrixXd R_gps, R_imu;  // Covarianza de GPS e IMU
    double dt;   // Intervalo de tiempo
    bool first_gps_reading, first_imu_reading, gps_received;
    double initial_x, initial_y, initial_theta;
    double latest_gps_x, latest_gps_y;

    ros::Subscriber imu_sub, gps_sub;
    ros::Publisher pose_pub;
    ros::Timer timer;

    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

    VectorXd predict(double a_x) {
        double theta = x(2);
        double v = x(3);
        double omega = x(4);

        VectorXd x_pred(5);
        x_pred(0) = x(0) + v * std::cos(theta) * dt;
        x_pred(1) = x(1) + v * std::sin(theta) * dt;
        x_pred(2) = x(2) + omega * dt;
        x_pred(3) = v + a_x * dt;
        x_pred(4) = omega;

        MatrixXd F(5, 5);
        F << 1, 0, -v * std::sin(theta) * dt, std::cos(theta) * dt, 0,
             0, 1,  v * std::cos(theta) * dt, std::sin(theta) * dt, 0,
             0, 0, 1, 0, dt,
             0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;

        P = F * P * F.transpose() + Q;
        return x_pred;
    }

    VectorXd update(const VectorXd& z, const MatrixXd& R) {
        MatrixXd H(4, 5);
        H << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0,
             0, 0, 1, 0, 0,
             0, 0, 0, 0, 1;

        VectorXd y = z - H * x;  // Residual
        MatrixXd S = H * P * H.transpose() + R;  // Innovación
        MatrixXd K = P * H.transpose() * S.inverse();  // Ganancia de Kalman
        x = x + K * y;
        P = P - K * H * P;

        return x;
    }

    std::vector<double> expandCovarianceMatrix() {
        std::vector<double> covariance(36, 0.0);

        covariance[0] = P(0, 0); // P(0,0)
        covariance[1] = P(0, 1); // P(0,1)
        covariance[6] = P(0, 2); // P(0,2)
        covariance[7] = P(1, 1); // P(1,1)
        covariance[11] = P(1, 2); // P(1,2)
        covariance[14] = P(2, 2); // P(2,2)
        covariance[21] = P(3, 3); // P(3,3)
        covariance[28] = P(4, 4); // P(4,4)

        covariance[5] = P(1, 0); // P(1,0)
        covariance[15] = P(2, 0); // P(2,0)
        covariance[16] = P(2, 1); // P(2,1)

        return covariance;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        double a_x = msg->linear_acceleration.x;
        double omega_imu = msg->angular_velocity.z;
        double theta_imu = 2 * atan2(msg->orientation.z, msg->orientation.w);

        if (first_imu_reading) {
            initial_theta = theta_imu;
            x(2) = theta_imu;
            first_imu_reading = false;
        }

        x = predict(a_x);

        if (gps_received) {
            VectorXd z(4);
            z << latest_gps_x, latest_gps_y, theta_imu, omega_imu;
            MatrixXd R = R_gps + R_imu;
            x = update(z, R);

            geometry_msgs::PoseWithCovarianceStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "odom_ekf";
            pose.pose.pose.position.x = x(0) - initial_x;
            pose.pose.pose.position.y = x(1) - initial_y;
            pose.pose.pose.orientation.z = sin((x(2) - initial_theta) / 2);
            pose.pose.pose.orientation.w = cos((x(2) - initial_theta) / 2);
            pose.pose.covariance = expandCovarianceMatrix();

            pose_pub.publish(pose);
        }
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        double latitude = msg->latitude;
        double longitude = msg->longitude;
        double x_utm, y_utm;
        int zone_number;
        char zone_letter;
        std::tie(x_utm, y_utm, zone_number, zone_letter) = utm::fromLatLon(latitude, longitude);

        if (first_gps_reading) {
            initial_x = x_utm;
            initial_y = y_utm;
            x(0) = x_utm;
            x(1) = y_utm;
            first_gps_reading = false;

            geometry_msgs::TransformStamped t;
            t.header.stamp = ros::Time::now();
            t.header.frame_id = "map";
            t.child_frame_id = "odom_ekf";
            t.transform.translation.x = initial_x;
            t.transform.translation.y = initial_y;
            t.transform.rotation.z = sin(initial_theta / 2.0);
            t.transform.rotation.w = cos(initial_theta / 2.0);
            static_tf_broadcaster.sendTransform(t);
        }

        R_gps(0, 0) = msg->position_covariance[0];
        R_gps(1, 1) = msg->position_covariance[4];

        latest_gps_x = x_utm;
        latest_gps_y = y_utm;
        gps_received = true;
    }

    void broadcastOdomToBaseLink(const ros::TimerEvent&) {
        geometry_msgs::TransformStamped t;
        t.header.stamp = ros::Time::now();
        t.header.frame_id = "odom_ekf";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x(0) - initial_x;
        t.transform.translation.y = x(1) - initial_y;
        t.transform.rotation.z = sin((x(2) - initial_theta) / 2.0);
        t.transform.rotation.w = cos((x(2) - initial_theta) / 2.0);
        tf_broadcaster.sendTransform(t);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_node");
    EKFNode node;
    ros::spin();
    return 0;
}
