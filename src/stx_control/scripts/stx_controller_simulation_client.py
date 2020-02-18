#!/usr/bin/env python
from __future__ import print_function

import threading
import time
import rospy
import math
import actionlib
import signal
import json
import os

from std_msgs.msg import Empty
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from stx_control.srv import stx_manager_service, stx_manager_serviceResponse
from stxlib.tcp_move.basic_client import BasicClient
from msg_state import MsgState

MC_IP = '90.0.0.1'
ROBOT_SPEED = 15
TCP_IP = MC_IP
TCP_PORT = 6001

MOVEMENT_TIMEOUT = 60
GRIPPER_TIMEOUT = 10
CLEAN_TABLE_TIMEOUT = 60
D_TIME_OUT = 20
I_TIME_OUT = 2

motion_status = threading.Event()
success_status = threading.Event()

my_mutex = threading.Lock()


# ----------------------------------------------------Arm---------------------------------------------------------------

class StxControllerServerArm:

    def __init__(self, pub):
        self.server = actionlib.SimpleActionServer('/arm_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction,
                                                   self.execute,
                                                   False)
        self.server.start()

        self.pub = pub

        self.planing_request_publisher = rospy.Publisher('/rviz/moveit/update_goal_state',
                                                         Empty,
                                                         queue_size=20)

    def execute(self, goal):
        """
        :type goal: FollowJointTrajectoryGoal
        """

        # get the msg counter from MC
        msg_id = tn.getMsgCounter()
        create_and_put_in_dictionary(msg_id, None, None, True)

        tn.enableGroup(65)
        rospy.loginfo('ACTION GOAL - arm_controller_tcp INVOKED!')

        if not motion_status.wait(timeout=I_TIME_OUT):
            rospy.logerr('TOOK TO LONG FOR MC TO RESPOND')
            self.server.set_aborted()
            my_mutex.acquire()
            obj = msg_state_dictionary[str(msg_id)]
            obj.dead = True
            my_mutex.release()
            motion_status.clear()
            return

        else:
            motion_status.clear()

            msg_arm, arm_msg_id = send_trajectory_to_server(goal.trajectory, self.server)

            # waiting for 60 seconds to finish moving the arm
            if not motion_status.wait(timeout=MOVEMENT_TIMEOUT):
                rospy.logerr('TOOK TO LONG TO EXECUTE MOVEMENT - EXECUTION ABORTED ')
                self.server.set_aborted()
                my_mutex.acquire()
                obj = msg_state_dictionary[str(arm_msg_id)]
                my_mutex.release()
                obj.dead = True
                motion_status.clear()
                return
            else:
                motion_status.clear()

            self.pub.set_new_points(goal.trajectory.points, msg_arm)
            # wait for publisher to finish
            success_status.clear()
            success_status.wait(timeout=MOVEMENT_TIMEOUT)
            self.server.set_succeeded()
            success_status.clear()

            self.planing_request_publisher.publish()


def send_trajectory_to_server(goal_trajectory, srv):
    """

    :type goal_trajectory: JointTrajectory
    """

    points = goal_trajectory.points  # type: list[JointTrajectoryPoint]
    last_point = points[points.__len__() - 1].positions

    last_point_deg = map(math.degrees, last_point)

    # get the msg counter from MC
    msg_id = tn.getMsgCounter()
    create_and_put_in_dictionary(msg_id, srv.set_succeeded, srv.set_aborted, False)

    # TO EDIT THE MOVING SPEED OF THE ROBOT CHANGE THE LAST PARAMETER FROM 20 TO DESIRED SPEED
    tn.grJoints(65, last_point_deg[0], last_point_deg[1], last_point_deg[2], last_point_deg[3], last_point_deg[4], ROBOT_SPEED)
    if not motion_status.wait(timeout=I_TIME_OUT):
        rospy.logerr('TOOK TOO LONG FOR MC TO RESPOND')
        my_mutex.acquire()
        obj = msg_state_dictionary[str(msg_id)]
        obj.dead = True
        my_mutex.release()
        srv.set_aborted()
        motion_status.clear()
    motion_status.clear()

    joint_names = rospy.get_param('/controller_joint_names')
    # joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

    msg_arm = JointState()
    msg_arm.name = joint_names
    msg_arm.position = points[points.__len__() - 1].positions
    msg_arm.velocity = [1.0, 1.0, 1.0, 1.0, 1.0]
    msg_arm.effort = [1.0, 1.0, 1.0, 1.0, 1.0]

    return msg_arm, msg_id


def init_arm_msg():
    msg_arm = JointState()
    msg_arm.name = rospy.get_param('/controller_joint_names')
    msg_arm.position = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg_arm.velocity = [1.0, 1.0, 1.0, 1.0, 1.0]
    msg_arm.effort = [1.0, 1.0, 1.0, 1.0, 1.0]
    return msg_arm


def init_gripper_msg():
    msg_gripper = JointState()
    msg_gripper.name = ['gripper_left_joint', 'gripper_right_joint']
    msg_gripper.position = [0.0, 0.0]
    msg_gripper.velocity = [1.0, 1.0]
    msg_gripper.effort = [1.0, 1.0]
    return msg_gripper


class JointStatePublisher:
    def __init__(self):
        self.msg_arm = init_arm_msg()
        self.msg_gripper = init_gripper_msg()
        self.new_points = False
        self.points = None

        t_joint_state_publish_arm = threading.Thread(target=self.arm_joint_state_publisher)
        t_joint_state_publish_arm.start()

        t_joint_state_publish_gripper = threading.Thread(target=self.gripper_joint_state_publisher)
        t_joint_state_publish_gripper.start()

    def arm_joint_state_publisher(self):
        try:
            pub = rospy.Publisher('joint_states', JointState, queue_size=10)
            rate = rospy.Rate(50)  # 50hz
            while not rospy.is_shutdown():
                self.msg_arm.header.stamp = rospy.Time.now()
                msg = self.msg_arm
                pub.publish(msg)
                rate.sleep()
                if self.new_points:
                    self.publish_movement(pub)
                    success_status.set()

        except Exception as e:
            print("EXITING: arm publishing thread...")

    def publish_movement(self, pub):
        msg_arm = JointState()
        msg_arm.name = rospy.get_param('/controller_joint_names')
        for i in range(self.points.__len__()):
            p = self.points[i]
            msg_arm.effort = p.effort
            msg_arm.position = p.positions
            msg_arm.header.stamp = rospy.Time.now()
            pub.publish(msg_arm)
            rospy.sleep(0.05)
        self.new_points = False

    def gripper_joint_state_publisher(self):
        try:
            pub = rospy.Publisher('joint_states', JointState, queue_size=10)
            rate = rospy.Rate(50)  # 50hz
            while not rospy.is_shutdown():
                self.msg_gripper.header.stamp = rospy.Time.now()
                pub.publish(self.msg_gripper)
                rate.sleep()

        except Exception as e:
            print("EXITING: gripper publishing thread...")

    def set_arm_mgs(self, msg):
        self.msg_arm = msg

    def set_new_points(self, points, msg):
        self.points = points
        self.new_points = True
        self.msg_arm = msg

    def set_gripper_msg(self, msg):
        self.msg_gripper = msg


# ----------------------------------------------------Gripper-----------------------------------------------------------
class StxControllerServerGripper:

    def __init__(self, pub):
        self.server = actionlib.SimpleActionServer('/gripper_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction,
                                                   self.execute,
                                                   False)
        self.server.start()

        self.msg_gripper = init_gripper_msg()

        self.pub = pub

    def execute(self, goal):
        """
        :type goal: FollowJointTrajectoryGoal
        """

        msg_id = tn.getMsgCounter()
        create_and_put_in_dictionary(msg_id, None, None, True)

        tn.enableGroup(65)
        rospy.loginfo('ACTION GOAL - gripper_controller_tcp INVOKED!')

        if not motion_status.wait(timeout=I_TIME_OUT):
            rospy.logerr('TOOK TOO LONG FOR MC TO RESPOND')
            self.server.set_aborted()
            my_mutex.acquire()
            obj = msg_state_dictionary[str(msg_id)]
            obj.dead = True
            my_mutex.release()
            motion_status.clear()
            return
        else:
            motion_status.clear()

            msg_gripper = send_gripper_action_to_server(self, goal.trajectory)

            # waiting for GRIPPER_TIMEOUT seconds to finish moving the arm
            if not motion_status.wait(timeout=GRIPPER_TIMEOUT):
                rospy.loginfo('TOOK TOO LONG TOO EXECUTE GRIPPER ACTION - EXECUTION ABORTED ')
                self.server.set_aborted()
                my_mutex.acquire()
                obj = msg_state_dictionary[str(msg_gripper)]
                obj.dead = True
                my_mutex.release()
                motion_status.clear()
                return

            motion_status.clear()
            self.pub.set_gripper_msg(msg_gripper)
            rospy.loginfo("GRIPPER ACTION IS DONE! ")


def send_gripper_action_to_server(Gripper, goal_trajectory):
    """

    :type goal_trajectory: JointTrajectory
    """

    points = goal_trajectory.points  # type: list[JointTrajectoryPoint]
    gripper_state = points[points.__len__() - 1].positions[0] + points[points.__len__() - 1].positions[1]

    # print(points[points.__len__() - 1].positions)
    if gripper_state > 0.05:
        msg_id = tn.getMsgCounter()
        create_and_put_in_dictionary(msg_id, Gripper.server.set_succeeded, Gripper.server.set_aborted, True)

        tn.gripperClose()
        rospy.loginfo("Gripper is closing")
    else:
        msg_id = tn.getMsgCounter()
        create_and_put_in_dictionary(msg_id, Gripper.server.set_succeeded, Gripper.server.set_aborted, True)

        tn.gripperOpen()
        rospy.loginfo("Gripper is: opening")

    # construct the msg to 'Joint State' topic

    joint_names = rospy.get_param('/gripper_controller_joint_names')
    # joint_names = ['gripper_left_joint', 'gripper_right_joint']

    msg_gripper = JointState()
    msg_gripper.name = joint_names
    msg_gripper.position = points[points.__len__() - 1].positions
    msg_gripper.velocity = [1.0, 1.0]
    msg_gripper.effort = [1.0, 1.0]

    return msg_gripper


# ----------------------------------------------------------------------------------------------------------------------

def handle_command(req):
    if req.command == '1':
        msg_id = tn.getMsgCounter()
        create_and_put_in_dictionary(msg_id, None, None, True)

        tn.enableGroup(65)
        if not motion_status.wait(timeout=I_TIME_OUT):
            rospy.loginfo('TOOK TO LONG FOR MC TO RESPOND ')
            my_mutex.acquire()
            obj = msg_state_dictionary[str(msg_id)]
            obj.dead = True
            my_mutex.release()
            motion_status.clear()
            return stx_manager_serviceResponse(
                'The server have NOT been Enabled - MC has not responded in reasonable time')
        motion_status.clear()
        return stx_manager_serviceResponse('The server have been Enabled')

    if req.command == '0':
        msg_id = tn.getMsgCounter()
        create_and_put_in_dictionary(msg_id, None, None, True)

        tn.disableGroup(65)
        if not motion_status.wait(timeout=I_TIME_OUT):
            rospy.loginfo('TOOK TO LONG FOR MC TO RESPOND ')
            my_mutex.acquire()
            obj = msg_state_dictionary[str(msg_id)]
            obj.dead = True
            my_mutex.release()
            motion_status.clear()
            return stx_manager_serviceResponse(
                'The server have NOT been Disabled - MC has not responded in reasonable time')
        motion_status.clear()
        return stx_manager_serviceResponse('The server have benn Disabled')

    else:
        return stx_manager_serviceResponse('Error: wrong input')


def signal_handler(signal, frame):
    tn.disableGroup(65)
    tn.disconnect()
    rospy.sleep(2)
    print("Exiting: main thread - signal handler...")
    return


def create_and_put_in_dictionary(key, success_func, abort_func, is_immediate):
    my_msg = MsgState(key, success_func, abort_func, is_immediate)
    my_mutex.acquire()
    msg_state_dictionary[str(key)] = my_msg
    my_mutex.release()


def start_listening_thread():
    try:
        while 1:
            # if no action from MC for CLEAN_TABLE_TIMEOUT seconds-> clean the msg dictionary
            signal.alarm(CLEAN_TABLE_TIMEOUT)
            ans = tn.recv()
            update_msg_state(ans)
    except Exception as e:
        print("EXITING: listener thread...")


def update_msg_state(msg):
    immediate_ack_time = int(round(time.time() * 1000))
    is_differed = False

    parsed_ans = msg.split(" ")
    msg_id = parsed_ans[0]
    if msg_id[0] == '!':
        is_differed = True
        msg_id = msg_id[1:]

    # if a msg from mc is 'Fault' event
    if msg_id[0] == '#':
        rospy.loginfo(msg)

    msg_err_code = parsed_ans[1].split("\r\n")[0]

    my_object = msg_state_dictionary[msg_id]

    my_object.is_differed = is_differed

    my_object.err_code = msg_err_code

    if my_object.dead:
        return

    if not is_differed:
        my_object.immediate_ack_time = immediate_ack_time
        motion_status.set()
        if my_object.is_immediate and my_object.success_callback is not None:
            my_object.success_callback()

        if msg_err_code != '0':
            for error in error_dict:
                if error["number"] == msg_err_code:
                    rospy.logerr(error["message"])

    # if differed we call the Success/Fail callback
    else:
        if msg_err_code == '0':
            motion_status.set()
            success_status.set()
        else:
            for error in error_dict:
                if error["number"] == msg_err_code:
                    rospy.logerr(error["message"])

            rospy.loginfo("SET FAIL CALLBACK IS CALLED!")
            my_object.fail_callback()
            motion_status.set()


def clean_dictionary(signum, frame):
    """called when read times out"""
    try:
        keys = []
        rospy.loginfo("Cleaning dictionary!!!")

        for key in msg_state_dictionary:
            msg = msg_state_dictionary[key]

            if msg.dead:
                keys.append(key)
            else:
                if msg.is_immediate or msg.is_differed:
                    keys.append(key)
                else:
                    time_from_ack = int(round(time.time() * 1000)) - msg.immediate_ack_time
                    time_from_creation = int(round(time.time() * 1000)) - msg.creation_time

                    if time_from_ack > D_TIME_OUT or time_from_creation > I_TIME_OUT:
                        keys.append(key)

        keys_no_duplicates = list(dict.fromkeys(keys))

        my_mutex.acquire()

        for key in keys_no_duplicates:
            del msg_state_dictionary[key]

        my_mutex.release()
        rospy.loginfo("Dictionary is clean!!!")

    except Exception as e:
        print("Exiting: cleaning thread...")


def time_to_ms(secs, nsecs):
    return long(secs * 1000 + nsecs * 1e-6)


if __name__ == '__main__':

    try:
        signal.signal(signal.SIGALRM, clean_dictionary)
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        rospy.init_node('arm_controller')
        rospy.loginfo('stx_controller_arm/gripper_Running_tcp...=================================================')
        print("=====================================")
        MC_IP = rospy.get_param('~MC-IP')
        print('Stx controller use ip : {}'.format(MC_IP))
        ROBOT_SPEED = rospy.get_param('~ROBOT-SPEED')
        print('robot SPEED is : {}'.format(ROBOT_SPEED))
        print("=====================================")

        tn = BasicClient(MC_IP)
        tn.connect()

        # create a hash map: key(MC msg id) value(msg state object)
        msg_state_dictionary = dict()

        # create an error dictionary from json
        mypath = os.path.join(os.path.dirname(__file__), 'stxlib/error_codes.json')
        with open(mypath, 'r') as errors:
            error_dict = json.load(errors)

        t_MC_response = threading.Thread(target=start_listening_thread)
        t_MC_response.start()

        publisher = JointStatePublisher()
        server_arm = StxControllerServerArm(publisher)
        server_gripper = StxControllerServerGripper(publisher)

        service = rospy.Service('Stx_manage_client_command', stx_manager_service, handle_command)

        rospy.spin()

        tn.disconnect()

    except Exception as e:
        print("Exiting: main thread - exception...")
