#!/usr/bin/env python
import signal
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

from std_msgs.msg import Empty
from stx_control.srv import stx_manager_service


class MoveGroupPythonInterface(object):

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_commander_me',
                        anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name_a = "arm"
        group_a = moveit_commander.MoveGroupCommander(group_name_a)

        group_name_g = "gripper"
        group_g = moveit_commander.MoveGroupCommander(group_name_g)

        planing_request_publisher = rospy.Publisher('/rviz/moveit/update_goal_state',
                                                    Empty,
                                                    queue_size=20)

        planning_frame = group_a.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # End-effector link name for this group:
        eef_link = group_a.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # List of all the groups in the robot:
        group_names = robot.get_group_names()
        print ("============ Robot Groups: {} \n\n".format(robot.get_group_names()))

        # robot:uncomment in order to see robot state description
        # print "============ Printing robot state"
        # print robot.get_current_state()
        # print ""

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group_a = group_a
        self.group_g = group_g
        self.update_planning_goal_request = planing_request_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        group = self.group_a
        print ('Moving the robot by joint states: {} \n'.format(group.get_name()))

        # IN ORDER TO CHANGE ROBOT PLAN CHANGE THE VALUES OF THE JOINTS (RADIANS)
        # JOINTS NAME = joint1, joint2, joint3, joint4, joint5

        # degs = list([79.00002335403727, -34.979301, 100.0085445, 0.00035999999999999997, 0.00504])
        # rads = map(math.radians, [79.00002335403727, -34.979301, 100.0085445, 0.00035999999999999997, 0.00504])
        # for example: move = {"joint1": rads[0], "joint2": rads[1], "joint3": rads[2]}

        move = {"joint1": 0.4, "joint2": -1, "joint3": -1}
        plan_msg = group.plan(joints=move)

        group.execute(plan_msg=plan_msg, wait=True)

        group.stop()

        # to update Rviz planning request to be in the correct position
        self.update_planning_goal_request.publish()

    def go_to_pose(self):
        group = self.group_a
        print ('Moving the robot by defined pose : {} \n'.format(group.get_name()))

        # IN ORDER TO MOVE THE ROBOT IN TO A KNOW POINT, CHANGE THE ORIENTATION AND POSITION PARAMETERS
        pose_goal = geometry_msgs.msg.Pose()

        # Orientation of the point: defined with respect to robot location
        pose_goal.orientation.x = -0.0780599
        pose_goal.orientation.y = -0.335181
        pose_goal.orientation.z = -0.0258615
        pose_goal.orientation.w = 0.938558

        # Position of the point: defined with respect to where the tool is located in space
        pose_goal.position.x = 0.62547
        pose_goal.position.y = -0.037943
        pose_goal.position.z = 0.65212
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        group.clear_pose_targets()

        # to update Rviz planning request to be in the correct position
        self.update_planning_goal_request.publish()

    def close_gripper(self):
        group = self.group_g
        print ('Closing the {} \n'.format(group.get_name()))
        move = {"gripper_left_joint": 0.029135302365617826, "gripper_right_joint": 0.030507981286151335}

        plan_msg = group.plan(joints=move)
        group.execute(plan_msg=plan_msg, wait=True)
        group.stop()

    def open_gripper(self):
        group = self.group_g
        print ('Opening the {} \n'.format(group.get_name()))
        move = {"gripper_left_joint": 8.612488827202469e-05, "gripper_right_joint": 7.26884635630995e-05}

        plan_msg = group.plan(joints=move)
        group.execute(plan_msg=plan_msg, wait=True)
        group.stop()

    def home(self):
        group = self.group_a
        print ('Moving the robot  to Home position \n')

        # IN ORDER TO CHANGE ROBOT PLAN CHANGE THE VALUES OF THE JOINTS
        # JOINTS NAME = joint1, joint2, joint3, joint4, joint5
        move = {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0, "joint5": 0.0}
        plan_msg = group.plan(joints=move)
        group.execute(plan_msg=plan_msg, wait=True)
        group.stop()

        # to update Rviz planning request to be in the correct position
        self.update_planning_goal_request.publish()

    def run(self):
        group = self.group_a
        move1 = {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0, "joint5": 0.0}
        move2 = {"joint1": 0.5, "joint2": -0.5, "joint3": 0.5, "joint4": 0.0, "joint5": 0.0}
        move3 = {"joint1": 0.3, "joint2": -0.8, "joint3": 0.1, "joint4": 0.0, "joint5": 0.0}

        for i in range(3):
            print ('Moving the robot by joint states: {} first position \n'.format(group.get_name()))
            plan_msg = group.plan(joints=move1)
            group.execute(plan_msg=plan_msg, wait=True)
            group.stop()

            # to update Rviz planning request to be in the correct position
            self.update_planning_goal_request.publish()

            print ('Moving the robot by joint states: {} second position \n'.format(group.get_name()))
            plan_msg = group.plan(joints=move2)
            group.execute(plan_msg=plan_msg, wait=True)
            group.stop()

            # to update Rviz planning request to be in the correct position
            self.update_planning_goal_request.publish()

            print ('Moving the robot by joint states: {} third position \n'.format(group.get_name()))
            plan_msg = group.plan(joints=move3)
            group.execute(plan_msg=plan_msg, wait=True)
            group.stop()

            # to update Rviz planning request to be in the correct position
            self.update_planning_goal_request.publish()


def print_state():
    robot = moveit_commander.RobotCommander()
    print("Robot position is=======================\n ")
    state = robot.get_current_state()
    print("{} \n\n".format(state))


def stx_manager(command):
    rospy.wait_for_service('Stx_manage_client_command')
    try:
        service_manager = rospy.ServiceProxy('Stx_manage_client_command', stx_manager_service)
        resp = service_manager(command)
        print(resp.ans)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def thread_handler(signal, frame):
    return


def signal_handler(signal, frame):
    stx_manager('0')
    sys.exit(0)


def main():
    try:

        while True:
            print "============ Press `1` to begin setting up the MoveIt! commander (press ctrl-d to exit) ..."
            inp = raw_input()
            if inp == '1':
                stx_manager(inp)
                my_robot = MoveGroupPythonInterface()
                break

        signal.signal(signal.SIGINT, signal_handler)
        while True:

            print "============ Press `Enter` to execute a movement using a joint state goal by setting the joint engles ..."
            print "============ Press `Space` and 'Enter' to move the robot to a fixed point in moveit space ..."
            print "============ Press `C` and 'Enter' to close the robot's gripper ..."
            print "============ Press `O` and 'Enter' to open the robot's gripper ..."
            print "============ Press `H` and 'Enter' to for home position ..."
            print "============ Press `M` to execute planed movement of multiple points..."
            print "============ Press `P` to print current robot current state..."
            print "============ Press '0' to Disable ..."

            command = raw_input()

            if command == '0':
                stx_manager(command)
                return

            if command == '':
                my_robot.go_to_joint_state()

            if command == ' ':
                my_robot.go_to_pose()

            if command == 'C':
                my_robot.close_gripper()

            if command == 'O':
                my_robot.open_gripper()

            if command == 'H':
                my_robot.home()

            if command == 'M':
                my_robot.run()

            if command == 'P':
                print_state()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
