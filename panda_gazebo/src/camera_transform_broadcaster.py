#!/usr/bin/env
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

if __name__ == '__main__':
    rospy.init_node('camera_transform_broadcaster_node')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(1)
    transform_pub = rospy.Publisher('/camera_color_optical_frame_in_world',
                                    PoseStamped)
    pose_color = PoseStamped()
    rospy.sleep(2)
    trans = tfBuffer.lookup_transform('world', 'camera_color_optical_frame',
                                      rospy.Time())
    #pose_color.header = '/world'
    pose_color.pose.position = trans.transform.translation
    pose_color.pose.orientation = trans.transform.rotation
    while not rospy.is_shutdown():
        print('I AM PUBLISHING WWAAAAAHHHH')
        transform_pub.publish(pose_color)
        rate.sleep()
