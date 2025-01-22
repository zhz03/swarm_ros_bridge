#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from std_msgs.msg import Header
from detection_msgs.msg import BevFeature  # Ensure detection_msgs is correctly built in your workspace

class TimeRecorder(object):
    def __init__(self):
        # Set the file path to save the txt file (modify this path as needed)
        self.file_path = "/home/zhaoliang/zzl/swarm_ros_ws/src/swarm_ros_bridge/scripts/ros_time.txt"
        
        # Ensure the file exists (if not, create an empty file)
        if not os.path.isfile(self.file_path):
            with open(self.file_path, "w") as f:
                f.write("")
        
        # Create subscribers for the two topics
        self.sub_bev = rospy.Subscriber('/veh_2/bev', BevFeature, self.bev_callback)
        self.sub_time_ref = rospy.Subscriber('/bev_time_ref', Header, self.time_ref_callback)

        rospy.loginfo("TimeRecorder node started.")

    def bev_callback(self, msg):
        """
        Callback triggered when a /veh_2/bev message is received.
        Records the current ROS time along with an identifier to the file.
        """
        current_time = rospy.Time.now()
        # Format the time record; you can also use current_time.to_sec() if preferred.
        time_str = "Sent (/veh_2/bev): ROS Time: %s\n" % str(current_time)
        rospy.loginfo("Received /veh_2/bev message. Recording: %s", time_str.strip())
        
        # Append the record to the file.
        with open(self.file_path, "a") as f:
            f.write(time_str)

    def time_ref_callback(self, header_msg):
        """
        Callback triggered when a /bev_time_ref message is received.
        Records the current ROS time along with an identifier to the file.
        """
        current_time = rospy.Time.now()
        time_str = "Received (/bev_time_ref): ROS Time: %s\n" % str(current_time)
        rospy.loginfo("Received /bev_time_ref message. Recording: %s", time_str.strip())

        # Append the record to the file.
        with open(self.file_path, "a") as f:
            f.write(time_str)

def main():
    rospy.init_node('time_recorder_node', anonymous=True)
    recorder = TimeRecorder()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TimeRecorder node terminated.")

if __name__ == '__main__':
    main()
