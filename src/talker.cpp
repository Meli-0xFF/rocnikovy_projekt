//
// Created by meli on 6. 2. 2020.
//

#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("/humanoid/joint1_position_controller/command", 1000);
    ros::Publisher chatter_pub2 = n.advertise<std_msgs::Float64>("/humanoid/joint2_position_controller/command", 1000);
    ros::Rate loop_rate(2);

    double num = 1.57, num2 = -1.57;
    while (ros::ok()) {
        std_msgs::Float64 msg, msg2;
        msg.data = num;
        msg2.data = num2;
        ROS_INFO("%lf", msg.data);
        ROS_INFO("%lf", msg2.data);
        chatter_pub.publish(msg);
        chatter_pub2.publish(msg2);

        ros::spinOnce();

        loop_rate.sleep();
        num *= -1;
        num2 = -(num);
    }
}