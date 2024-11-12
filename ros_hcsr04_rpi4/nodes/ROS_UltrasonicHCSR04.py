import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

# import custom message type for Relative Velocity
from UltrasonicHCSR04 import UltrasonicHCSR04 # import our driver module

class UltrasonicHCSR04Wrapper(object):

    def __init__(self):
        self.min_range = rospy.get_param("~minimum_range", 3)
        self.max_range = rospy.get_param("~maximum_range", 400)
        self.fov = rospy.get_param("~field_of_view", 15) * 0.0174532925199433 # degree to radian conversion

        trigger_pin = rospy.get_param("~trigger_pin", 18)
        echo_pin = rospy.get_param("~echo_pin", 24)
        
        self.distance_publisher = rospy.Publisher('/ultrasonic/data',Float32, queue_size=10)
        self.ultrasonicPub = rospy.Publisher('/ultrasonic/distance', Range, queue_size=10)

        # loading the driver
        self.ultrasonic_sensor = UltrasonicHCSR04(trigger_pin, echo_pin)

        self.rate = rospy.Rate(10) # 10hz
        rospy.Timer(rospy.Duration(1.0 / 10), self.publish_current_distance)
        rospy.Timer(rospy.Duration(1.0 / 10), self.publish_current_data)
        

    def publish_current_data(self):
        distance = self.ultrasonic_sensor.distance()
        
        data = Float32()
        data.data=distance
        
        self.distance_publisher.publish(data)
        
    def publish_current_distance(self):
        distance = self.ultrasonic_sensor.distance()

        message_str = "Distance: %s cm" % distance
        rospy.loginfo(message_str)
        
        #for distance in ranges:
        r = Range()

        r.header.stamp = rospy.Time.now()
        r.header.frame_id = "/base_link"
        r.radiation_type = Range.ULTRASOUND
        r.field_of_view = self.fov # 15 degrees
        r.min_range = self.min_range
        r.max_range = self.max_range

        r.range = distance
            
        self.ultrasonicPub.publish(r)


    def stop(self):
        self.distance_publisher.unregister()
        self.ultrasonicPub.unregister()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("ultrasonic_driver", anonymous=False)

    ultrasonic_wrapper = UltrasonicHCSR04Wrapper()

    rospy.on_shutdown(ultrasonic_wrapper.stop)
    rospy.loginfo("Ultrasonic driver is now started.")

    rospy.spin()
