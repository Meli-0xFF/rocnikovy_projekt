#!/usr/bin/env python

import numpy as np
import utils
import sys
import os
import rospy
from rocnikovy_projekt.msg import ImuYaw
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import ConfigParser

class ImuYawPublisher:
    """
    Subsribe the raw output of the simulated IMU sensor,
    compute the yaw angle (heading) from the quaternions,
    and print it on std out to debug the imu and also,
    publish
    """

    def odom_callback(self, msg):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        quaterions = np.array([[qx], [qy], [qz], [qw]])
        yaw = utils.quaternions_to_yaw(quaterions)
        h = Header()
        h.stamp = msg.header.stamp

        self.pb_ref.publish(h, yaw)

    def hector_imu_current_bias_callback(self,msg):
        """
        If the Hector IMU plugin is used,
        """

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        quaterions = np.array([[qx], [qy], [qz], [qw]])
        yaw = utils.quaternions_to_yaw(quaterions)
        #rospy.loginfo('current yaw offset: %.3f rad', yaw)


    def imu_callback(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        quaterions = np.array([[qx], [qy], [qz], [qw]])
        yaw = utils.quaternions_to_yaw(quaterions)
        h = Header()
        h.stamp = rospy.get_rostime()

        self.pb_imu.publish(h, yaw)
        rospy.loginfo("IMU yaw angle: %f", yaw)

    def load_parameters(self, ini_file):

        config = ConfigParser.SafeConfigParser()
        if os.path.isfile(ini_file):
            config.read(ini_file)
        else:
            rospy.logerr("Error, the config file: %s not found!", ini_file)
            return False
        try:
            self.imu_topic = config.get('result_comparison', 'imu_topic')
            rospy.loginfo("imu_topic set to:%s", self.imu_topic)

            self.gt_topic = config.get('result_comparison','gt_topic')
            rospy.loginfo("gt_topic set to:%s", self.gt_topic)

            self.use_hector_imu = config.getboolean('imu_model', 'use_hector_imu')
            rospy.loginfo("use_hector_imu set to:%d",self.use_hector_imu)

        except ConfigParser.NoSectionError as e:
            rospy.logerr("NoSectionError when parsing the config file: %s", ini_file)
            rospy.logerr(e)
            return False
        except ConfigParser.NoOptionError as e:
            rospy.logerr("NoOptionError when parsing the config file: %s", ini_file)
            rospy.logerr(e)
            return False
        except ValueError as e:
            rospy.logerr("Wrong value given in config file: %s",ini_file)
            rospy.logerr(e)
            return False
        return True

    def __init__(self, ini_file):
        rospy.init_node('imu_yaw_publisher')
        if self.load_parameters(ini_file):
            self.imu_topic = '/imu'
            self.gt_topic = '/gt'
            self.hector_imu_bias_topic = '/imu/bias'
            self.use_hecstor_imu = False
            self.load_parameters(ini_file)

            rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
            rospy.Subscriber(self.gt_topic, Odometry, self.odom_callback)

            if self.use_hector_imu is True:
                rospy.Subscriber(self.hector_imu_bias_topic, Imu, self.hector_imu_current_bias_callback)


            self.pb_imu = rospy.Publisher("/imu/yaw", ImuYaw)
            self.pb_ref = rospy.Publisher("/imu/yaw_ref", ImuYaw)
            rospy.loginfo("imu yaw publisher initialized!")
            rospy.spin()
        else:
            rospy.logerr("Error loading parameters from file: %s", ini_file)

if __name__ == "__main__":
    try:
        print("HELLO MELLO")
        if len(sys.argv) > 1:
            yp = ImuYawPublisher(sys.argv[1])
        else:
            raise AssertionError
    except AssertionError:
        print("imu_yaw_publisher error: no config file given!")
