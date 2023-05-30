#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time

panda_joints = [
    "panda_j1", "panda_j2", "panda_j3", "panda_j4", "panda_j5", "panda_j6",
    "panda_j7"
]
hithand_joints = [
    'index0', 'index1', 'index2', 'index3', 'little0', 'little1', 'little2',
    'little3', 'middle0', 'middle1', 'middle2', 'middle3', 'ring0', 'ring1',
    'ring2', 'ring3', 'thumb0', 'thumb1', 'thumb2', 'thumb3'
]


class RobotJointWrapper:
    """ This class listens to the topic hithand/joint_cmd or panda/joint_cmd, splits the command and publishes it to the individual 
    joint control topics. On the command topic only the joint positions or efforts or velocities have to be specified, not the joint names, since the joints names are hardcoded in this script.
    """
    def __init__(self):
        rospy.init_node('robot_joint_wrapper')
        rospy.loginfo('Robot joint wrapper node started.')
        self.publishers_init_flag = False
        self.control_rate = rospy.get_param('~control_rate')
        rospy.loginfo("I heard the control rate: %d" % self.control_rate)
        self.publish_prefix = rospy.get_param('~publish_prefix')
        self.listen_prefix = rospy.get_param('~listen_prefix')
        self.command_topic = rospy.get_param('~joint_command_topic')
        self.control_method = rospy.get_param('~control_method')
        self.check_control_method()

        if self.listen_prefix == 'hithand':
            self.joint_names = hithand_joints
            self.number_of_joints = len(hithand_joints)
        elif self.listen_prefix == 'panda':
            self.joint_names = panda_joints
            self.number_of_joints = len(panda_joints)
        else:
            rospy.logerr('Unknown listen prefix: Should be panda or hithand')

        self.init_control_command_publishers()

        listened_joint_command_topic = '/' + self.listen_prefix + '/joint_cmd'
        self.joint_command_subscriber = rospy.Subscriber(
            listened_joint_command_topic,
            JointState,
            self.callback_listened_joint_command,
            queue_size=100)
        self.curr_time = None
        self.prev_time = None

    def init_control_command_publishers(self):
        """ Builds a list with the publishers for the individual joint control topics.
        """
        self.control_command_publishers = []
        for i in range(self.number_of_joints):
            topic = '/' + self.publish_prefix + '/' + self.joint_names[
                i] + '_' + self.controller_name + '/command'
            self.control_command_publishers.append(
                rospy.Publisher(topic, Float64, queue_size=1))
        self.publishers_init_flag = True

    def check_control_method(self):
        """ Verify that the specified control method is position, velocity or effort.
        """
        if (self.control_method == 'p'):
            self.controller_name = 'position_controller'
        elif (self.control_method == 'v'):
            self.controller_name = 'velocity_controller'
        elif (self.control_method != 'e'):
            self.controller_name = 'effort_controller'
        else:
            rospy.logerr(
                '[RobotJointWrapper] Unknown control method: Use p, v or e')

    def callback_listened_joint_command(self, joint_command):
        """ This function listenes to the control command on the robot/joint_cmd topic splits the command and publishes it to individual topics.
        """
        self.curr_time = time.time()
        if self.prev_time is not None:
            diff = self.curr_time - self.prev_time
            print(diff)
        self.prev_time = self.curr_time
        if self.publishers_init_flag:
            # First check that the command has the right length.
            if (len(joint_command.position) != self.number_of_joints) and (len(
                    joint_command.velocity) != self.number_of_joints) and (len(
                        joint_command.effort) != self.number_of_joints):
                rospy.logerr(
                    'The command has the wrong size. The correct size is: %d' %
                    self.number_of_joints)
            # Then choose the data from the command dependant on the control type
            for i in range(self.number_of_joints):
                joint_data = Float64()
                if self.control_method == 'p':
                    joint_data.data = joint_command.position[i]
                elif self.control_method == 'e':
                    joint_data.data = joint_command.effort[i]
                elif self.control_method == 'v':
                    joint_data.data = joint_command.velocity[i]
                else:
                    rospy.logerr(
                        'The command has no position, effort or velocity entry.'
                    )
                self.control_command_publishers[i].publish(joint_data)
        else:
            rospy.logerr(
                'The joint_wrapper publishers have not been initialized yet.')


if __name__ == '__main__':
    rjw = RobotJointWrapper()
    rate = rospy.Rate(rjw.control_rate)
    while not rospy.is_shutdown():
        rate.sleep()