#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Header
from detection_msgs.msg import BevFeature  # Ensure that the detection_msgs package is built and in your ROS workspace

class BevHeaderPublisher(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('bev_time_ref_publisher', anonymous=True)
        
        # Create a publisher that publishes to the '/bev_time_ref' topic with message type std_msgs/Header
        self.pub = rospy.Publisher('/veh_1/bev', BevFeature, queue_size=10)
        
        # Create a subscriber that subscribes to the '/veh_1/bev' topic with message type detection_msgs/BevFeature
        self.sub = rospy.Subscriber('/veh_2/bev', BevFeature, self.bev_callback)
        
        rospy.loginfo("bev_time_ref_publisher node started.")

    def bev_callback(self, msg):
        """
        Callback function triggered upon receiving a message on the '/veh_1/bev' topic.
        It modifies the header.frame_id in the received BevFeature message to a new name ("veh_1/bev"),
        and then publishes the header to the '/bev_time_ref' topic.
        """

        # Optionally update the timestamp to the current time if needed
        # header.stamp = rospy.Time.now()

        # Publish the modified header to the '/bev_time_ref' topic
        self.pub.publish(msg)
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
