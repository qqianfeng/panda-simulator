#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
import time


class Test:
    def __init__(self):
        rospy.init_node('test')
        self.joint_command_pub = rospy.Publisher('/hithand/joint_cmd', JointState, queue_size=1)
        self.hithand_current_joint_states_sub = rospy.Subscriber(
            '/panda_hithand/joint_states',
            JointState,
            self.callback_hithand_current_joint_state,
            tcp_nodelay=True)
        # self.hithand_current_joint_states_sub = rospy.Subscriber(
        #     'panda_hithand/thumb0_position_controller/state', JointControllerState,
        # self.callback_hithand_current_joint_state)
        self.run_rate = rospy.Rate(20)
        delta_vector = np.array([0, 0.01, 0.01, 0.01])
        self.delta_pos_joint_vector = np.tile(delta_vector, 5)
        self.cb_prev_time = time.time()

    def callback_hithand_current_joint_state(self, msg):
        # print("Actual time: " + str(time.time()))
        # print("Ros time: " + str(rospy.Time.now()))
        cb_curr_time = time.time()
        print("Time delta CB: " + str(cb_curr_time - self.cb_prev_time))
        self.cb_prev_time = cb_curr_time
        self.hithand_current_joint_state = np.array(msg.position)
        # print(msg.position[0:3])

    def close_hand(self):
        joint_pub_msg = JointState()
        # reset hithand
        zero_pos = 0 * self.delta_pos_joint_vector
        joint_pub_msg.position = zero_pos.tolist()
        for i in xrange(5):
            self.joint_command_pub.publish(joint_pub_msg)
            rospy.sleep(0.1)
        print("Reset")
        desired_pos_joints = self.hithand_current_joint_state
        prev_time = time.time()
        for i in xrange(20):
            curr_time = time.time()
            print("Time delta: " + str(curr_time - prev_time))
            prev_time = curr_time
            #Check whether hand velocity is zero

            desired_pos_joints += self.delta_pos_joint_vector
            joint_pub_msg.position = desired_pos_joints.tolist()
            self.joint_command_pub.publish(joint_pub_msg)
            self.run_rate.sleep()


if __name__ == '__main__':
    test = Test()
    prev_time = time.time()
    for i in xrange(20):
        curr_time = time.time()
        print("Time delta: " + str(curr_time - prev_time))
        prev_time = curr_time
        rospy.wait_for_message('/hithand/joint_states', JointState)
    rospy.spin()