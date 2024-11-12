#include <iostream>
#include <wiringPi.h>
#include <libhcsr04/libhcsr04.h>
#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>

class HCSR04Node
{ 
    public:
        HCSR04Node();
        void run();
        void stop();
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        HCSR04 hcsr04;
        int trigger, echo;
        int timeout;
        int minimum_range, maximum_range;
        float field_of_view;
        ros::Publisher pub_raw;
};

HCSR04Node::HCSR04Node()
{
    nh.param<int>("trigger", trigger, 18);
    nh.param<int>("echo", echo, 24);
    nh.param<int>("timeout", timeout, 1000000);
    nh.param<int>("minimum_range", minimum_range, 3);
    nh.param<int>("maximum_range", maximum_range, 400);
    nh.param<float>("field_of_view", field_of_view, 15.0f);
    hcsr04.init(trigger, echo);
    pub = nh.advertise<sensor_msgs::Range>("distance", 1);
    pub_raw=nh.advertise<std_msgs::Float32>("distance_raw",1);
}

void HCSR04Node::run()
{
    ros::Rate rate(10);
    while(ros::ok())
    {
        sensor_msgs::Range msg;
        std_msgs::Float32 msg_raw;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/base_link";
        msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        msg.field_of_view = field_of_view * 0.0174532925199433;
        msg.min_range = minimum_range;
        msg.max_range = maximum_range;
        msg.range = hcsr04.distance(timeout);

        msg_raw.data=msg.range;
        pub.publish(msg);
        pub_raw.publish(msg_raw);
        rate.sleep();
    }
}

void HCSR04Node::stop()
{
    ROS_INFO("stopping");
    if(pub) pub.shutdown();
    if(pub_raw) pub_raw.shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HCSR04");
    HCSR04Node node;
    node.run();
    return 0;
}