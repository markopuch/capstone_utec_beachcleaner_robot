#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix

class SuscriptorGPS(object):
    def __init__(self, topic):
        self.sub = rospy.Subscriber(topic, NavSatFix, self.callback_point)
        self.gps_msg = NavSatFix()
        
    def callback_point(self, msg):
        self.gps_msg = msg

    def get_gps_data(self):
        return self.gps_msg

if __name__ == "__main__":
    try:
        rospy.init_node('node_datacollection_gps')  # Inicializar el nodo
        
        path = "/home/utec/data_gps.txt"
        data = open(path, "w")

        # Subscribe to the GPS topic
        sub_gps = SuscriptorGPS('fix')
        
        # INIT
        rate = rospy.Rate(100)
        t = 0.0
        dt = 1.0 / 100
        
        while not rospy.is_shutdown():
            gps_data = sub_gps.get_gps_data()
            
            # Extraer datos del mensaje NavSatFix
            latitude = gps_data.latitude
            longitude = gps_data.longitude
            altitude = gps_data.altitude
            position_covariance = gps_data.position_covariance
            
            data.write(str(t) + ' ' + str(gps_data.header.stamp) + ' ' +
                       str(latitude) + ' ' + str(longitude) + ' ' + str(altitude) + ' ' +
                       str(position_covariance[0]) + ' ' + str(position_covariance[1]) + ' ' + str(position_covariance[2]) + ' ' +
                       str(position_covariance[3]) + ' ' + str(position_covariance[4]) + ' ' + str(position_covariance[5]) + ' ' +
                       str(position_covariance[6]) + ' ' + str(position_covariance[7]) + ' ' + str(position_covariance[8]) + '\n')
            
            
            # Imprimir los datos para verificación (opcional)
            print(t, latitude, longitude, altitude, 
                  position_covariance[0], position_covariance[1], position_covariance[2],
                  position_covariance[3], position_covariance[4], position_covariance[5],
                  position_covariance[6], position_covariance[7], position_covariance[8])
            
            # wait
            t += dt
            rate.sleep()

        data.close()

    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("Exiting")
