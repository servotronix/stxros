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
TCP_IP = MC_IP
TCP_PORT = 6001

MOVEMENT_TIMEOUT = 60
GRIPPER_TIMEOUT = 3
CLEAN_TABLE_TIMEOUT = 40
D_TIME_OUT = 20
I_TIME_OUT = 2

motion_status = threading.Event()
my_mutex = threading.Lock()


# ----------------------------------------------------Arm---------------------------------------------------------------

class StxControllerServerArm:

    def __init__(self):
        self.server = actionlib.SimpleActionServer('/arm_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction,
                                                   self.execute,
                                                   False)
        self.server.start()

        self.planing_request_publisher = rospy.Publisher('/rviz/moveit/update_goal_state',
                                                         Empty,
                                                         queue_size=20)

    def execute(self, goal):
        """
        :type goal: FollowJointTrajectoryGoal
        """
        try:
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

            else:
                motion_status.clear()

                arm_msg_id = send_trajectory_to_server(goal.trajectory, self.server)

                # waiting for 60 seconds to finish moving the arm
                if not motion_status.wait(timeout=MOVEMENT_TIMEOUT):
                    rospy.logerr('TOOK TO LONG TO EXECUTE MOVEMENT - EXECUTION ABORTED ')
                    self.server.set_aborted()
                    my_mutex.acquire()
                    obj = msg_state_dictionary[str(arm_msg_id)]
                    my_mutex.release()
                    obj.dead = True
                else:
                    motion_status.clear()

            self.planing_request_publisher.publish()

        except Exception as e:
            print("Exiting: SimpleActionServer...")


def send_trajectory_to_server(goal_trajectory, srv):
    """

    :type goal_trajectory: JointTrajectory
    """

    points = goal_trajectory.points  # type: list[JointTrajectoryPoint]
    last_point = points[points.__len__() - 1].positions

    print(last_point)

    last_point_deg = map(math.degrees, last_point)

    # get the msg counter from MC
    msg_id = tn.getMsgCounter()
    create_and_put_in_dictionary(msg_id, srv.set_succeeded, srv.set_aborted, False)

    # print("just inserted msg number: {}".format(msg_id))

    # TO EDIT THE MOVING SPEED OF THE ROBOT CHANGE THE LAST PARAMETER FROM 10 TO DESIRED SPEED
    speed = 10
    tn.grJoints(65, last_point_deg[0], last_point_deg[1], last_point_deg[2], last_point_deg[3], last_point_deg[4],
                speed)

    if not motion_status.wait(timeout=I_TIME_OUT):
        rospy.logerr('TOOK TO LONG FOR MC TO RESPOND')
        my_mutex.acquire()
        obj = msg_state_dictionary[str(msg_id)]
        obj.dead = True
        my_mutex.release()
        srv.set_aborted()
        motion_status.clear()
    motion_status.clear()

    return msg_id


# ----------------------------------------------------Gripper-----------------------------------------------------------
class StxControllerServerGripper:

    def __init__(self):
        self.server = actionlib.SimpleActionServer('/gripper_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction,
                                                   self.execute,
                                                   False)
        self.server.start()

        self.msg_gripper = init_gripper_msg()

        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        t_gripper_publisher = threading.Thread(target=self.publish_to_joint_state)
        t_gripper_publisher.start()

    def publish_to_joint_state(self):
        try:

            rate = rospy.Rate(50)  # 50hz/20ms
            while not rospy.is_shutdown():
                self.msg_gripper.header.stamp = rospy.Time.now()
                self.pub.publish(self.msg_gripper)
                rate.sleep()

        except Exception as e:
            print("EXITING: publisher thread...")

    def execute(self, goal):
        """
        :type goal: FollowJointTrajectoryGoal
        """
        try:
            msg_id = tn.getMsgCounter()
            create_and_put_in_dictionary(msg_id, None, None, True)

            tn.enableGroup(65)
            rospy.loginfo('ACTION GOAL - gripper_controller_tcp INVOKED!')

            if not motion_status.wait(timeout=I_TIME_OUT):
                rospy.logerr('TOOK TO LONG FOR MC TO RESPOND')
                self.server.set_aborted()
                my_mutex.acquire()
                obj = msg_state_dictionary[str(msg_id)]
                obj.dead = True
                my_mutex.release()
                motion_status.clear()
            else:
                motion_status.clear()

                msg_gripper, msg_gripper_id = send_gripper_action_to_server(self, goal.trajectory)

                # waiting for GRIPPER_TIMEOUT seconds to finish moving the Gripper
                if not motion_status.wait(timeout=GRIPPER_TIMEOUT):
                    rospy.logerr('TOOK TO LONG TO EXECUTE GRIPPER ACTION - EXECUTION ABORTED ')
                    self.server.set_aborted()
                    my_mutex.acquire()
                    obj = msg_state_dictionary[str(msg_gripper_id)]
                    obj.dead = True
                    my_mutex.release()
                    motion_status.clear()
                    return

                motion_status.clear()
                self.msg_gripper = msg_gripper
                rospy.loginfo("GRIPPER ACTION IS DONE! ")

        except Exception as e:
            print("Exiting: SimpleActionServer...")


def send_gripper_action_to_server(gripper, goal_trajectory):
    """
    :type goal_trajectory: JointTrajectory
    """

    points = goal_trajectory.points  # type: list[JointTrajectoryPoint]
    gripper_state = points[points.__len__() - 1].positions[0] + points[points.__len__() - 1].positions[1]

    # print(points[points.__len__() - 1].positions)
    if gripper_state > 0.05:
        msg_id = tn.getMsgCounter()
        create_and_put_in_dictionary(msg_id, gripper.server.set_succeeded, gripper.server.set_aborted, True)

        tn.gripperClose()
        rospy.loginfo("Gripper is closing")
    else:
        msg_id = tn.getMsgCounter()
        create_and_put_in_dictionary(msg_id, gripper.server.set_succeeded, gripper.server.set_aborted, True)

        tn.gripperOpen()
        rospy.loginfo("Gripper is: opening")

    # construct the msg to 'Joint State' topic
    joint_names = rospy.get_param('/gripper_controller_joint_names')
    # joint_names = ['gripper_left_joint', 'gripper_right_joint']

    msg_gripper = JointState()
    msg_gripper.name = joint_names
    msg_gripper.header.stamp = rospy.Time.now()
    msg_gripper.position = points[points.__len__() - 1].positions
    msg_gripper.header.stamp = rospy.Time.now()
    msg_gripper.velocity = [1.0, 1.0]
    msg_gripper.effort = [1.0, 1.0]

    return msg_gripper, msg_id


def init_gripper_msg():
    try:

        msg_gripper = JointState()
        msg_gripper.name = ['gripper_left_joint', 'gripper_right_joint']
        msg_gripper.position = [0.0, 0.0]
        msg_gripper.velocity = [1.0, 1.0]
        msg_gripper.effort = [1.0, 1.0]
        msg_gripper.header.stamp = rospy.Time.now()
        return msg_gripper

    except Exception as e:
        print("Exiting: main thread - exception...")


# ---------------------------------------Helpers------------------------------------------------------------------

def handle_service_command(req):
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
                'Operating the robot has NOT been Enabled - MC has not responded in reasonable time')
        motion_status.clear()
        return stx_manager_serviceResponse("Operating the robot from 'moveit_node'  has been Enabled")

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
                ' Operating the robot has NOT been Disabled - MC has not responded in reasonable time')
        motion_status.clear()
        return stx_manager_serviceResponse(" Operating the robot from 'moveit_node' has been Disabled")

    else:
        return stx_manager_serviceResponse('Error: wrong input')


def signal_handler(signal, frame):
    msg_id = tn.getMsgCounter()
    create_and_put_in_dictionary(msg_id, None, None, True)
    tn.disableGroup(65)
    tn.disconnect()
    # let all other threads to exit before main thread
    rospy.sleep(1)
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

                    rospy.loginfo("EXIT SYSTEM - use 'Ctl+C'")

    # if differed we call the Success/Fail callback
    else:
        if msg_err_code == '0':
            rospy.loginfo("SET SUCCESS CALLBACK IS CALLED!")
            my_object.success_callback()
            motion_status.set()
        else:
            for error in error_dict:
                if error["number"] == msg_err_code:
                    rospy.logerr(error["message"])

                    rospy.loginfo("EXIT SYSTEM - use 'Ctl+C'")

            rospy.loginfo("SET FAIL CALLBACK IS CALLED!")
            my_object.fail_callback()
            motion_status.set()


def clean_dictionary(signum, frame):
    """called when read times out"""
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


if __name__ == '__main__':

    try:
        signal.signal(signal.SIGALRM, clean_dictionary)
        signal.signal(signal.SIGINT, signal_handler)

        rospy.init_node('arm_controller')
        rospy.loginfo('stx_controller_arm/gripper_Running_tcp.............')

        print("=====================================")
        MC_IP = rospy.get_param('~MC-IP')
        print('my IP is : {}'.format(MC_IP))
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

        server_arm = StxControllerServerArm()
        server_gripper = StxControllerServerGripper()

        service = rospy.Service('Stx_manage_client_command', stx_manager_service, handle_service_command)

        rospy.spin()

        tn.disconnect()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("Exiting: main thread - exception...")
