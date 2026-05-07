#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from visualization_msgs.msg import Marker

def main():
    rospy.init_node('robot_visualizer')

    # TF buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Marker publisher
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Get transform from base_link to odom
            transform_base_to_odom = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0))

            # Create transform from base_link to base_true
            # base_true is at z=0 in odom frame, same x,y as base_link, horizontal orientation
            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = "base_true"
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = -transform_base_to_odom.transform.translation.z  # Offset to make z=0 in odom
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0  # Horizontal orientation

            # Publish the transform
            tf_broadcaster.sendTransform(transform)

            # Marker in base_true frame
            marker = Marker()
            marker.header.frame_id = "base_true"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "robot_model"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Set position (center of the robot at height 0.092 above base_true, which is at z=0 in odom)
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.092  # Half of height

            # Set orientation
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set scale (length, width, height)
            marker.scale.x = 0.420  # Length
            marker.scale.y = 0.310  # Width
            marker.scale.z = 0.184  # Height

            # Set color (e.g., semi-transparent blue)
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.5

            # Publish marker
            marker_pub.publish(marker)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s" % str(e))
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass