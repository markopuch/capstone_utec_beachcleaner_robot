#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
import utm

class EKFNode:
    def __init__(self):
        self.x = np.array([0, 0, 0, 0, 0])  # [x, y, theta, v, omega]
        self.P = np.diag([0.70, 0.70, 0.01, 0.01, 0.04])  # Matriz de covarianza inicial
        self.Q = np.diag([0.3, 0.3, 0.04, 0.1, 0.04])  # Covarianza del proceso
        self.R_gps = np.zeros((4, 4))
        self.R_imu = np.zeros((4, 4))
        self.R_imu[2, 2] = 0.0159  # orientación
        self.R_imu[3, 3] = 0.04  # velocidad angular
        self.dt = 0.01  # Ajustado a la frecuencia del IMU

        # Estado inicial
        self.first_gps_reading = True
        self.first_imu_reading = True
        self.initial_x = 0
        self.initial_y = 0
        self.initial_theta = 0
        self.latest_gps_x = 0
        self.latest_gps_y = 0
        self.gps_received = False

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Configuración de nodos y suscriptores
        rospy.init_node('ekf_node')
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/fix', NavSatFix, self.gps_callback)
        self.pose_pub = rospy.Publisher('/ekf_pose', PoseWithCovarianceStamped, queue_size=10)
        rospy.Timer(rospy.Duration(0.01), self.broadcast_odom_to_base_link)  # 100Hz

    def predict(self, a_x):
        theta = self.x[2]
        v = self.x[3]
        omega = self.x[4]
        x_pred = np.array([
            self.x[0] + v * np.cos(theta) * self.dt,
            self.x[1] + v * np.sin(theta) * self.dt,
            self.x[2] + omega * self.dt,
            v + a_x * self.dt,
            omega
        ])

        F = np.array([
            [1, 0, -v * np.sin(theta) * self.dt, np.cos(theta) * self.dt, 0],
            [0, 1,  v * np.cos(theta) * self.dt, np.sin(theta) * self.dt, 0],
            [0, 0, 1, 0, self.dt],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        P_pred = np.dot(np.dot(F, self.P), F.T) + self.Q
        return x_pred, P_pred

    def update(self, z, R):
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1]
        ])

        y = z - np.dot(H, self.x)
        S = np.dot(np.dot(H, self.P), H.T) + R
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        x_upd = self.x + np.dot(K, y)
        P_upd = self.P - np.dot(np.dot(K, H), self.P)
        return x_upd, P_upd

    def expand_covariance_matrix(self):
        covariance = np.zeros(36)
        P = self.P

        covariance[0] = P[0, 0]
        covariance[1] = P[0, 1]
        covariance[6] = P[0, 2]
        covariance[7] = P[1, 1]
        covariance[11] = P[1, 2]
        covariance[14] = P[2, 2]
        covariance[21] = P[3, 3]
        covariance[28] = P[4, 4]

        covariance[5] = P[1, 0]
        covariance[15] = P[2, 0]
        covariance[16] = P[2, 1]

        return covariance.flatten().tolist()

    def imu_callback(self, data):
        a_x = data.linear_acceleration.x
        omega_imu = data.angular_velocity.z
        theta_imu = 2 * np.arctan2(data.orientation.z, data.orientation.w)

        if self.first_imu_reading:
            self.initial_theta = theta_imu
            self.x[2] = theta_imu
            self.first_imu_reading = False

        self.x, self.P = self.predict(a_x)

        if self.gps_received:
            z = np.array([self.latest_gps_x, self.latest_gps_y, theta_imu, omega_imu])
            R = self.R_gps + self.R_imu
            self.x, self.P = self.update(z, R)

            pose = PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom_ekf"
            pose.pose.pose.position.x = self.x[0] - self.initial_x
            pose.pose.pose.position.y = self.x[1] - self.initial_y
            pose.pose.pose.orientation.z = np.sin((self.x[2] - self.initial_theta) / 2)
            pose.pose.pose.orientation.w = np.cos((self.x[2] - self.initial_theta) / 2)
            pose.pose.covariance = self.expand_covariance_matrix()
            self.pose_pub.publish(pose)

    def gps_callback(self, data):
        latitude = data.latitude
        longitude = data.longitude
        x_utm, y_utm, _, _ = utm.from_latlon(latitude, longitude)

        if self.first_gps_reading:
            self.initial_x = x_utm
            self.initial_y = y_utm
            self.x[0] = x_utm
            self.x[1] = y_utm
            self.first_gps_reading = False

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "odom_ekf"
            t.transform.translation.x = self.initial_x
            t.transform.translation.y = self.initial_y
            t.transform.rotation.z = np.sin(self.initial_theta / 2.0)
            t.transform.rotation.w = np.cos(self.initial_theta / 2.0)
            self.static_tf_broadcaster.sendTransform(t)

        self.R_gps[0, 0] = data.position_covariance[0]
        self.R_gps[1, 1] = data.position_covariance[4]

        self.latest_gps_x = x_utm
        self.latest_gps_y = y_utm
        self.gps_received = True

    def broadcast_odom_to_base_link(self, event):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom_ekf"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x[0] - self.initial_x
        t.transform.translation.y = self.x[1] - self.initial_y
        t.transform.rotation.z = np.sin((self.x[2] - self.initial_theta) / 2.0)
        t.transform.rotation.w = np.cos((self.x[2] - self.initial_theta) / 2.0)
        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    EKFNode()
    rospy.spin()
