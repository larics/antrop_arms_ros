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
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, GetPositionFKResponse, GetPositionIKResponse, GetPositionIKRequest
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


        # Variables for practical use later in the code!
        self.robot = robot
        self.frame_id = "world"
        self.scene = scene
        self.group = group
        keyword = "joint"
        self.incorrect_joint_list = self.group.get_joints()
        self.joint_list = [element for element in self.incorrect_joint_list if keyword in element]
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.ee_link = []
        self.ee_link.append(ee_link)
        self.available_groups = available_groups
        self.correction_matrix = np.matrix([[1, 0, 0, 0],
                                            [0, 1, 0, 0],
                                            [0, 0, 1, 1.5],
                                            [0, 0, 0, 1]])
        #self.DH_matrix = self.correction_matrix * np.matrix([[]])
        
    def getCurrentJointStates(self):
        return self.group.get_current_joint_values()
        


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
        print("Starting the getFK method!")
        rospy.wait_for_service("compute_fk")

        try:
            moveit_fk = rospy.ServiceProxy("compute_fk", GetPositionFK)
            print("Compute FK service initiated!")
            
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            
        request = GetPositionFKRequest()
        request.header.frame_id = self.frame_id
        # End effector link!
        try:
            if self.group.has_end_effector_link():
                request.fk_link_names = self.ee_link

        except Exception as e:
            print(f"No end effector link found: {e}")

        # Fill request with a list of joint names 
        request.robot_state.joint_state.name = self.joint_list
        # Pass a goal joint_state for which the FK will get computed
        request.robot_state.joint_state.position = self.input_joint_states
        # This is the computed pose for the end effector for given joint_states
        response = moveit_fk(request)
        print(f"Computed FK: {response}")
        return response
        
    def create_pose(self, x, y, z, qx, qy, qz, qw):
        # Create a PoseStamped message
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id 
        pose.pose.position.x = x 
        pose.pose.position.y = y  
        pose.pose.position.z = z + 1.5   
        pose.pose.orientation.x = qx  
        pose.pose.orientation.y = qy  
        pose.pose.orientation.z = qz  
        pose.pose.orientation.w = qw  
        return pose

    def getIK(self, target_pose, current_joint_state):
        """
        :param target_pose: -> xyz position of the robot ee and xyzw quaternion orientation, a 7x1 array.
        :return: 4x1 array of solver joint states to get into the target position
        """
        self.target_pose = target_pose
        print(f"Target pose: {self.target_pose}")
        
        self.current_joint_state = current_joint_state
        print("Starting the getIK method!")
        rospy.wait_for_service('compute_ik')
        
        try:
            compute_ik = rospy.ServiceProxy("compute_ik", GetPositionIK)
            print("Compute IK service initiated!")  
            
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
        
        # Filling in the RobotState() object         
        robotTemp = RobotState()
        robotTemp.joint_state.header.frame_id = self.frame_id
        robotTemp.joint_state.name = self.joint_list
        robotTemp.joint_state.position = self.current_joint_state
        
        service_request = PositionIKRequest()
        service_request.group_name = self.group_name
        service_request.pose_stamped = self.target_pose
        service_request.robot_state = robotTemp
        service_request.timeout.secs = 1
        #service_request.attempts = 5

        
        try:
            if self.group.has_end_effector_link():
                service_request.ik_link_name = self.ee_link[0]
                
        except Exception as e:
            print(f"No end effector link found: {e}")

        print(f"Service request: {service_request}")

        resp = compute_ik(service_request)
        print(f"Computed IK: {resp}")
        return list(resp.solution.joint_state.position)

    def getJacobian(self, joint_state):
        """
        """
        self.joint_state = joint state
        
        currentState = RobotState()
        currentState.joint_state.header.frame_id = self.frame_id
        currentState.joint_state.name = self.joint_list
        currentState.joint_state.position = self.joint_state
        
        jacobianMatrix = np.array(self.group.get_jacobian_matrix(currentState))


def main():

  try:
    print("Starting the AntropArmsPythonInterface!")
    testing = AntropArmsPythonInterface()
    #testing.moveToJointStateGoal()
    test_input_joints = [-0.2918368955004258, -0.06868186235263263, -0.194198852046922, 1.8693671028963053]
    testing.getFK(test_input_joints)
    test_current_joint = testing.getCurrentJointStates()
    print(f"This is a possible joint_state: {test_input_joints}. This is the current joint_state: {test_current_joint}!")
    test_goal_position = testing.create_pose(-0.19,-0.12,-0.3,0,0,0,0)
    testing.getIK(test_goal_position,test_current_joint)

    

  except rospy.ROSInterruptException:
    return

  finally:
      print("Closing everything!")
      #moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
  main()
