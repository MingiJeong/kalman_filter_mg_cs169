#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

# transformation base_footprint wrt odom_kf
def tf_publisher(msg):
    broadcaster = tf.TransformBroadcaster()
    trans = msg.pose.pose.position
    rot = msg.pose.pose.orientation
    broadcaster.sendTransform((trans.x,trans.y,trans.z),
        (rot.x, rot.y, rot.z, rot.w), rospy.Time.now(), "base_footprint", "odom_kf")

if __name__ == '__main__':
    rospy.init_node('tf_publisher')
    rospy.Subscriber('kalman_filter', PoseWithCovarianceStamped, tf_publisher)
    rospy.spin()
