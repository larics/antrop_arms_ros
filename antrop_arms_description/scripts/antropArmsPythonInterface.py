import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import copy
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from std_msgs.msg import String, Header
from moveit_commander.conversions import pose_to_list
import numpy as np
import time
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, GetPositionFKResponse
from tf import TransformListener


"""
RELEVANT SOURCES:
https://github.com/ros-planning/moveit_commander/blob/indigo-devel/src/moveit_commander/move_group.py
"""

class AntropArmsPythonInterface(object):
    """
    Interface designed for interacting with the AntropArms robot model via the MoveIt + ROS + Gazebo/RVIZ development
    environment.
    Currently, runs a single thread which manipulates actions of a move group. Future iteration should have a separate
    process for each arm which should feed joint states and positions to a common data structure which will allow
    for a more intelligent comprehension regarding their relative positions (avoiding collision)
    """
    def __init__(self):
        super(AntropArmsPythonInterface, self).__init__()


        # Additional joint_state remapping, seemingly doesn't work when done from .gazebo and .urdf configurations!
        joint_state_topic = ['joint_states:=/antrop_arms/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('foo', anonymous=False)

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        # Define which group to use, available groups: "left_arm", "right_arm"
        self.group_name = "left_arm"
        group = moveit_commander.MoveGroupCommander(self.group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        planning_frame = group.get_planning_frame()
        print(f"Reference frame is: {planning_frame}")
        ee_link = group.get_end_effector_link()
        print(f"End effector is: {ee_link}")
        available_groups = robot.get_group_names()
        print(f"Available groups are: {available_groups}")
        print(group.get_current_pose())

        # Variables for practical use later in the code!
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.ee_link = ee_link
        self.available_groups = available_groups
        self.correction_matrix = np.matrix([[1, 0, 0, 0],
                                            [0, 1, 0, 0],
                                            [0, 0, 1, 1.5],
                                            [0, 0, 0, 1]])
        #self.DH_matrix = self.correction_matrix * np.matrix([[]])


    def moveToJointStateGoal(self):
        """
        This method moves the joints into specified states. Since the planning frame is set to world instead of
        base_link an additional transformation is needed in the form of a correction matrix.
        Example:
        T(world -> ee) isn't correct since we have a virtual link holding the base of our robot at an elevated z axis
        point.
        We multiply (from the left) with T(world -> base_link) in order to get T(base_link -> ee).
        """
        while 1:
            achievable_goal = [-0.2918368955004258, -0.06868186235263263, -0.194198852046922, 1.8693671028963053]
            try:
                joint_goal = self.group.get_current_joint_values()
                print(f"Current value is: {joint_goal}")
                # TODO: See if there is a way to get random values WITHIN set joint limits (revolute joints) for testing!
                attempt_pose = self.group.get_random_joint_values()
                print(f"Attempting pose: {attempt_pose}")
                self.group.go(attempt_pose, wait=True)
                self.group.stop()
                time.sleep(2)
                print(f"Reverting to neutral pose:")
                # Achievable_goal is simply a confirmed possible pose used for testing purposes!
                self.group.go(achievable_goal, wait=True)
                self.group.stop()
                time.sleep(2)
                print("Starting the cycle again!")

            except Exception as e:
                print(e)

    def moveToNamedTarget(self, namedTarget):
        """
        :param namedTarget: -> A pose defined in the MoveIt! setup assistant!
        """
        self.namedTarget = namedTarget
        self.group.set_named_target(namedTarget)
        #plan = self.group.plan()
        self.group.go(wait=True)

    def getFK(self, input_joint_states):
        """
        :param input_joint_states: -> A 4x1 array of goal joint states
        :return: response -> xyz pose and orientation of the end effector
        """
        self.input_joint_states = input_joint_states
        #http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/robot_model_and_robot_state
        # /robot_model_and_robot_state_tutorial.html#the-robotmodel-and-robotstate-classes
        # https://www.youtube.com/watch?v=_pIyXGRXMWY
        rospy.wait_for_service("compute_fk")

        try:
            moveit_fk = rospy.ServiceProxy("compute_fk", GetPositionFK)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
        request = GetPositionFKRequest()
        request.header.frame_id = "world"
        # End effector link!
        try:
            if self.group.has_end_effector_link():
                request.fk_link_names = [self.group.get_end_effector_link()]
        except Exception as e:
            print(f"No end effector link found: {e}")

        # Joints part of the used group -> check move_group functionality to fill this (or just hardcode/fill by hand)
        request.robot_state.joint_state.name = [self.group.get_joints()]
        # Pass a goal joint_state for which the FK will get computed
        request.robot_state.joint_state.position = input_joint_states
        # This is the computed pose for the end effector for given joint_states
        response = moveit_fk(request)
        print(f"Computed FK: {response}")
        return response

    def getIK(self, target, current_joint_states):
        """
        move_group.get_current_joint_values()
        :param target: -> xyz position of the robot ee, a 3x1 numpy vector + desired orientation?
        :param current_joint_states: -> 4x1 array of current joint states
        :return: 4x1 array of solver joint states to get into the target position
        """

        targetPose = PoseStamped()
        # TODO: Check in which format does the target value get passed, and fill position/orientation accordingly!
        targetPose.header.stamp = rospy.Time.now()
        targetPose.pose.position.x = target[0]
        targetPose.pose.position.y = target[1]
        # TODO: Possible fix for world -> baselink transformation is to add 1.5 to position.z (Ex. + z_translation/hover_distance)
        targetPose.pose.position.z = target[2]
        targetPose.pose.orientation.w = target[4]

        robotTemp = RobotState()
        robotTemp.joint_state.name = [self.group.get_joints()]
        robotTemp.joint_state.position = current_joint_states

        service_request = PositionIKRequest()
        service_request.group_name = self.group_name
        try:
            if self.group.has_end_effector_link():
                service_request.ik_link_name = [self.group.get_end_effector_link()]
        except Exception as e:
            print(f"No end effector link found: {e}")

        service_request.pose_stamped = targetPose
        service_request.robot_state = robotTemp
        service_request.timeout.secs = 1

        rospy.wait_for_service('compute_ik')
        compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

        resp = compute_ik(service_request)

        return list(resp.solution.joint_state.position)

    def getJacobian(self):
        pass


def main():

  try:
    print("Starting the moveToJointStateGoal functionality.")
    testing = AntropArmsPythonInterface()
    testing.moveToJointStateGoal()

  except rospy.ROSInterruptException:
    return

  finally:
      moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
  main()
