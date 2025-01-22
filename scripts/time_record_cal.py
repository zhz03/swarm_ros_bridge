#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from std_msgs.msg import Header
from detection_msgs.msg import BevFeature  # Ensure detection_msgs is correctly built in your workspace

class TimeRecorder(object):
    def __init__(self):
        # Set the file path to save the txt file (modify this path as needed)
        self.file_path = "/home/zhaoliang/zzl/swarm_ros_ws/src/swarm_ros_bridge/scripts/ros_time_cal.txt"
        
        # Ensure the file exists (if not, create an empty file)
        if not os.path.isfile(self.file_path):
            with open(self.file_path, "w") as f:
                f.write("")

        # Variables to store the timestamps of the received messages
        self.bev_time = None
        self.bev_time_ref = None

        # Create subscribers for the two topics
        self.sub_bev = rospy.Subscriber('/veh_2/bev', BevFeature, self.bev_callback)
        self.sub_time_ref = rospy.Subscriber('/bev_time_ref', Header, self.time_ref_callback)

        rospy.loginfo("TimeRecorder node started.")

    def bev_callback(self, msg):
        """
        Callback triggered when a /veh_2/bev message is received.
        Records the timestamp of the message and calculates the time difference if possible.
        """
        self.bev_time = rospy.Time.now()
        rospy.loginfo("Received /veh_2/bev message with timestamp: %s", str(self.bev_time))
        self.write_time_difference()

    def time_ref_callback(self, header_msg):
        """
        Callback triggered when a /bev_time_ref message is received.
        Records the timestamp of the message and calculates the time difference if possible.
        """
        self.bev_time_ref = rospy.Time.now()
        rospy.loginfo("Received /bev_time_ref message with timestamp: %s", str(self.bev_time_ref))
        self.write_time_difference()

    def write_time_difference(self):
        """
        Calculates the time difference between /veh_2/bev and /bev_time_ref messages.
        Writes the time difference to the file if both timestamps are available.
        """
        if self.bev_time is not None and self.bev_time_ref is not None:
            # Calculate the time difference in seconds
            time_difference_sec = (self.bev_time_ref - self.bev_time).to_sec()
            
            # Convert the time difference to milliseconds
            time_difference_ms = time_difference_sec * 1000

            # Prepare the output string
            output_str = (
                "Time difference between /veh_2/bev and /bev_time_ref: {:.6f} seconds, {:.3f} ms\n"
                .format(time_difference_sec,time_difference_ms)
            )

            # Log and write the time difference to the file
            rospy.loginfo(output_str.strip())
            with open(self.file_path, "a") as f:
                f.write(output_str)

            # Reset the timestamps to avoid duplicate calculations
            self.bev_time = None
            self.bev_time_ref = None

def main():
    rospy.init_node('time_recorder_node', anonymous=True)
    recorder = TimeRecorder()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TimeRecorder node terminated.")

if __name__ == '__main__':
    main()
