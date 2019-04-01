#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

origin_pose = PoseStamped()
is_subscribed = False

def pose_callback(data):
    global origin_pose, is_subscribed
    origin_pose = data
    #tf_listener = TransformListener()
    rospy.loginfo(data)
    rospy.loginfo(origin_pose)
    is_subscribed = True
    #t = self.tf.getLatestCommonTime("/base_link", "/fingertip")
    #p_in_base = self.tf_listener_.transformPose("/base_link", origin_pose)
    
def time_callback(event):
    global origin_pose
    pub.publish(origin_pose)
    #rospy.loginfo(origin_pose)

if __name__ == "__main__":
    rospy.init_node('goal_republisher', anonymous=True)
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=100)
    rospy.Subscriber('/rviz_goal', PoseStamped, pose_callback)
    #rospy.Timer(rospy.Duration(0.01), time_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(origin_pose)
        if is_subscribed:
            pub.publish(origin_pose)
        rate.sleep()
        #rospy.spin()
