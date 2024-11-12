#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

class SuscriptorImu(object):
    def __init__(self, topic):
        self.sub = rospy.Subscriber(topic, Imu, self.callback_point)
        self.imu_msg = Imu()
        
    def callback_point(self, msg):
        self.imu_msg = msg

    def get_imu_data(self):
        return self.imu_msg

if __name__ == "__main__":
    try:
        rospy.init_node('node_datacollection_imu')  # Inicializar el nodo
        
        path = "/home/utec/data_imu.txt"
        data = open(path, "w")
        topic= "imu/data"
        
        # Subscribe to the IMU topic
        sub_imu = SuscriptorImu(topic)
        
        # INIT
        rate = rospy.Rate(100)
        t = 0.0
        dt = 1.0 / 100
        
        while not rospy.is_shutdown():
            imu_data = sub_imu.get_imu_data()
            
            # Extraer datos del mensaje Imu
            orientation = imu_data.orientation
            angular_velocity = imu_data.angular_velocity
            linear_acceleration = imu_data.linear_acceleration
            
            # Escribir los datos en el archivo
            data.write(str(t) + ' ' + str(imu_data.header.stamp) + ' ' +
                       str(orientation.x) + ' ' + str(orientation.y) + ' ' + str(orientation.z) + ' ' + str(orientation.w) + ' ' +
                       str(angular_velocity.x) + ' ' + str(angular_velocity.y) + ' ' + str(angular_velocity.z) + ' ' +
                       str(linear_acceleration.x) + ' ' + str(linear_acceleration.y) + ' ' + str(linear_acceleration.z) + '\n')
            
            # Imprimir los datos para verificación (opcional)
            print(t, 
                  orientation.x, orientation.y, orientation.z, orientation.w, 
                  angular_velocity.x, angular_velocity.y, angular_velocity.z, 
                  linear_acceleration.x, linear_acceleration.y, linear_acceleration.z)
            
            # wait
            t += dt
            rate.sleep()

        data.close()

    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("Exiting")
