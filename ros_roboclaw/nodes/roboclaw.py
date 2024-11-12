#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import roboclaw_driver.roboclaw_driver as roboclaw
import time

class Suscriptor_cmdvel(object):
    def __init__(self):
        topic = 'cmd_vel'
        self.sub = rospy.Subscriber(topic, Twist, self.callback_point)
        self.cmd_vel_msg = Twist()
        
    def callback_point(self, msg):
        self.cmd_vel_msg = msg

    def update_cmd_vel(self,MAX_SPEED,BASE_WIDTH,TICKS_PER_METER):

        linear_x =  self.cmd_vel_msg.linear.x

        if linear_x > MAX_SPEED:
            linear_x = MAX_SPEED
        if linear_x < -MAX_SPEED:
            linear_x = -MAX_SPEED

        vr = linear_x + self.cmd_vel_msg.angular.z * BASE_WIDTH / 2.0  # m/s
        vl = linear_x - self.cmd_vel_msg.angular.z * BASE_WIDTH / 2.0

        vr_ticks = int(vr * TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * TICKS_PER_METER)

        rospy.logdebug("vrf_ticks:%d vrb_ticks: %d", vr_ticks, vl_ticks)

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            if vr_ticks is 0 and vl_ticks is 0:
                roboclaw.ForwardM1(address, 0)
                roboclaw.ForwardM2(address, 0)
            else:
                roboclaw.SpeedM1M2(address,vl_ticks, vr_ticks)

        except OSError as e:
            rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
            rospy.logdebug(e)

        return vr_ticks,vl_ticks

if __name__ == "__main__":

    try:
        rospy.init_node('node_roboclaw_wheels') # Inicializar el nodo
		
        rospy.loginfo("Connecting to roboclaw")
        baud_rate = int(rospy.get_param("~baud", "115200"))
        address = int(rospy.get_param("~address", "128"))
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        
        if address > 0x87 or address < 0x80:
                rospy.logfatal("Address out of range")
                rospy.signal_shutdown("Address out of range")
        
        #OPEN ROBOCLAW
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")

        #VERSION
        try:
            version = roboclaw.ReadVersion(address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
            pass

        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))
        
        #INIT MOTOR
        roboclaw.SpeedM1M2(address, 0, 0)
        roboclaw.ResetEncoders(address)

        #VALUES KINEMATIC
        MAX_SPEED = float(rospy.get_param("~max_speed", "1.0"))
        TICKS_PER_METER = float(rospy.get_param("~tick_per_meter", "24844.8"))
        BASE_WIDTH = float(rospy.get_param("~base_width", "0.315"))

        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", address)
        rospy.logdebug("max_speed %f", MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", TICKS_PER_METER)
        rospy.logdebug("base_width %f", BASE_WIDTH)
        
        #subscribe cmd_vel
        sub_cmdvel = Suscriptor_cmdvel() 
            
        #INIT
        rospy.loginfo("Starting motor drive")
        rate = rospy.Rate(100)
        t=0.0
        dt=1.0/100

        while not rospy.is_shutdown():
            
            enc1 = None
            enc2 = None

            try:
                enc1= roboclaw.ReadEncM1(address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)

            try:
                enc2= roboclaw.ReadEncM2(address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
                rospy.logdebug(e)


            sub_cmdvel.update_cmd_vel(MAX_SPEED,BASE_WIDTH,TICKS_PER_METER)

          
            rate.sleep()


    except rospy.ROSInterruptException: 
        pass
    rospy.loginfo("Exiting")
