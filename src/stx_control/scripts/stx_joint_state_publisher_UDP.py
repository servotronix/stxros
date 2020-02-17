#!/usr/bin/env python

# license removed for brevity

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from stxlib.axis_data.encoder_reader import AxisDataReader

MC_IP = '90.0.0.1'

DEG2RAD = 0.017453292519943295

if __name__ == '__main__':
    try:

        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('stx_joint_state_publisher')
        rospy.loginfo('stx_joint_state_publisher Running UDP.............')

        print("=====================================")
        MC_IP = rospy.get_param('~MC-IP')
        print('Joint state publisher use ip : {}'.format(MC_IP))
        print("=====================================")

        joint_names = None
        try:
            joint_names = rospy.get_param('/controller_joint_names')
            # joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        except KeyError:
            rospy.logerr('/controller_joint_names param was not found!')

        msg = JointState()
        msg.name = joint_names

        num_of_joints = len(joint_names)
        reader = AxisDataReader(MC_IP)

        # start array of 5 zeros
        msg.position = [0] * len(joint_names)
        msg.velocity = [0] * len(joint_names)
        msg.effort = [0] * len(joint_names)

        rate = rospy.Rate(50)  # 50hz
        while not rospy.is_shutdown():

            msg.header = Header()
            axes_data = reader.read(num_of_joints)
            msg.position = list(map(lambda axis_data: axis_data.get_pFb() * DEG2RAD, axes_data))
            msg.velocity = list(map(lambda axis_data: axis_data.get_vFb() * DEG2RAD, axes_data))
            msg.effort = list(map(lambda axis_data: axis_data.get_tFb(), axes_data))
            msg.header.stamp = rospy.Time.now()

            # rospy.loginfo(list(map(lambda axis_data: axis_data.get_pFb(), axes_data)))
            pub.publish(msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
