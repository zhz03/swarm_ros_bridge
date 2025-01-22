#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Header
from detection_msgs.msg import BevFeature  # Make sure the detection_msgs package is built and in your ROS workspace

class BevHeaderPublisher(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('bev_time_ref_publisher', anonymous=True)
        
        # Create a publisher for the '/bev_time_ref' topic of type std_msgs/Header
        self.pub = rospy.Publisher('/bev_time_ref', Header, queue_size=10)
        
        # Create a subscriber for the '/veh_2/bev' topic of type detection_msgs/BevFeature
        self.sub = rospy.Subscriber('/veh_2/bev', BevFeature, self.bev_callback)
        
        rospy.loginfo("bev_time_ref_publisher node started.")

    def bev_callback(self, msg):
        """
        Callback function that is triggered upon receiving a message on the '/veh_2/bev' topic.
        It extracts the header from the received message and publishes it to the '/bev_time_ref' topic.
        """
        # Extract the header from the received BevFeature message (assumes msg.header exists)
        header = msg.header

        # Optional: update the stamp to the current time if needed
        # header.stamp = rospy.Time.now()

        rospy.loginfo("Received BevFeature message with header stamp: %s", header.stamp)

        # Publish the header to '/bev_time_ref'
        self.pub.publish(header)
        rospy.loginfo("Published header to /bev_time_ref.")
        

def main():
    try:
        # Initialize the BevHeaderPublisher instance, which sets up the subscriber and publisher
        node = BevHeaderPublisher()
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("bev_time_ref_publisher node terminated.")

if __name__ == '__main__':
    main()
