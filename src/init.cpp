#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>

#define USING_SERVOS 12

#define LEFT_HIP 0
#define LEFT_HIP_SERVO 1
#define LEFT_THIGH 2
#define LEFT_SHIN 3
#define LEFT_ANKLE 4
#define LEFT_FOOT 5

#define RIGHT_HIP 6
#define RIGHT_HIP_SERVO 7
#define RIGHT_THIGH 8
#define RIGHT_SHIN 9
#define RIGHT_ANKLE 10
#define RIGHT_FOOT 11

ros::Publisher left_hip_pub;
ros::Publisher left_hip_servo_pub;
ros::Publisher left_thigh_pub;
ros::Publisher left_shin_pub;
ros::Publisher left_ankle_pub;
ros::Publisher left_foot_pub;

ros::Publisher right_hip_pub;
ros::Publisher right_hip_servo_pub;
ros::Publisher right_thigh_pub;
ros::Publisher right_shin_pub;
ros::Publisher right_ankle_pub;
ros::Publisher right_foot_pub;

void set_joint_position(int joint, double position) {
    std_msgs::Float64 msg;
    msg.data = position;

    switch (joint) {
        case LEFT_HIP:          left_hip_pub.publish(msg);
                                break;
        case LEFT_HIP_SERVO:    left_hip_servo_pub.publish(msg);
                                break;
        case LEFT_THIGH:        left_thigh_pub.publish(msg);
                                break;
        case LEFT_SHIN:         left_shin_pub.publish(msg);
                                break;
        case LEFT_ANKLE:        left_ankle_pub.publish(msg);
                                break;
        case LEFT_FOOT:         left_foot_pub.publish(msg);
                                break;
        case RIGHT_HIP:         right_hip_pub.publish(msg);
                                break;
        case RIGHT_HIP_SERVO:   right_hip_servo_pub.publish(msg);
                                break;
        case RIGHT_THIGH:       right_thigh_pub.publish(msg);
                                break;
        case RIGHT_SHIN:        right_shin_pub.publish(msg);
                                break;
        case RIGHT_ANKLE:       right_ankle_pub.publish(msg);
                                break;
        case RIGHT_FOOT:        right_foot_pub.publish(msg);
                                break;
    }

    ROS_INFO("%d, %lf", joint, msg.data);
}

void stand() {
    set_joint_position(LEFT_HIP, 0.55);
    set_joint_position(RIGHT_HIP, -0.52);
    set_joint_position(LEFT_HIP_SERVO, 0.2);
    set_joint_position(RIGHT_HIP_SERVO, 0);
    set_joint_position(LEFT_THIGH, -1.1);
    set_joint_position(RIGHT_THIGH, -1.1);
    set_joint_position(LEFT_SHIN, 0.6);
    set_joint_position(RIGHT_SHIN, -0.6);
    set_joint_position(LEFT_ANKLE, 0.3);
    set_joint_position(RIGHT_ANKLE, -0.7);
    set_joint_position(LEFT_FOOT, 0);
    set_joint_position(RIGHT_FOOT, -0.1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "init");
    ros::NodeHandle n;
    left_hip_pub = n.advertise<std_msgs::Float64>("/humanoid/left_hip_position_controller/command", 1000);
    left_hip_servo_pub = n.advertise<std_msgs::Float64>("/humanoid/left_hip_servo_position_controller/command", 1000);
    left_thigh_pub = n.advertise<std_msgs::Float64>("/humanoid/left_thigh_position_controller/command", 1000);
    left_shin_pub = n.advertise<std_msgs::Float64>("/humanoid/left_shin_position_controller/command", 1000);
    left_ankle_pub = n.advertise<std_msgs::Float64>("/humanoid/left_ankle_position_controller/command", 1000);
    left_foot_pub = n.advertise<std_msgs::Float64>("/humanoid/left_foot_position_controller/command", 1000);

    right_hip_pub = n.advertise<std_msgs::Float64>("/humanoid/right_hip_position_controller/command", 1000);
    right_hip_servo_pub = n.advertise<std_msgs::Float64>("/humanoid/right_hip_servo_position_controller/command", 1000);
    right_thigh_pub = n.advertise<std_msgs::Float64>("/humanoid/right_thigh_position_controller/command", 1000);
    right_shin_pub = n.advertise<std_msgs::Float64>("/humanoid/right_shin_position_controller/command", 1000);
    right_ankle_pub = n.advertise<std_msgs::Float64>("/humanoid/right_ankle_position_controller/command", 1000);
    right_foot_pub = n.advertise<std_msgs::Float64>("/humanoid/right_foot_position_controller/command", 1000);

    ros::Rate loop_rate(20);

    double joints_data[USING_SERVOS];

    for (int i = 0; i < USING_SERVOS; i++)
        joints_data[i] = 0;

    double num = 0, num2 = 0;
    int a = 0;

    ros::Time time = ros::Time::now();

    while (ros::ok()) {
        time = ros::Time::now();
        a++;
        if (time.toSec() <= 3.0) {
            stand();
        }
        else {
            set_joint_position(LEFT_FOOT, -0.1); // 0
            set_joint_position(RIGHT_FOOT, -0.2); // -0.1
            set_joint_position(RIGHT_HIP_SERVO, 0.1); // 0
            set_joint_position(LEFT_HIP_SERVO, 0.1); // 0.2
            ros::Duration(0.3).sleep();
            set_joint_position(LEFT_SHIN, 0.7); // 0.6
            set_joint_position(RIGHT_SHIN, -0.7); // -0.6
            set_joint_position(LEFT_THIGH, -1.22); // -1.1
            ros::Duration(0.7).sleep();
            set_joint_position(LEFT_FOOT, 0.1); // 0
            set_joint_position(RIGHT_FOOT, 0); // -0.1
            set_joint_position(RIGHT_HIP_SERVO, -0.1); // 0
            set_joint_position(LEFT_HIP_SERVO, 0.3); // 0.2
            ros::Duration(0.3).sleep();
            set_joint_position(RIGHT_SHIN, -0.6); // -0.6
            set_joint_position(LEFT_SHIN, 0.6); // 0.6
            set_joint_position(LEFT_THIGH, -1.1); // -1.1
            ros::Duration(0.7).sleep();

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
