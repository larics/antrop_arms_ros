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
        

    def moveToJointStateGoal(self, goal_joint_state):
        """
        This method moves the joints into specified states. Since the planning frame is set to world instead of
        base_link an additional transformation is needed in the form of a correction matrix.
        Example:
        T(world -> ee) isn't correct since we have a virtual link holding the base of our robot at an elevated z axis
        point.
        We multiply (from the left) with T(world -> base_link) in order to get T(base_link -> ee).
        :param goal_joint_state: Desired joint values for the move_group selected!  ->   List of 4 floats 
        Ex.: achievable_goal = [-0.2918368955004258, -0.06868186235263263, -0.194198852046922, 1.8693671028963053]
        """
        self.goal_joint_state = goal_joint_state
        print(f"The goal joint_state: {self.goal_joint_state}")
        try:
            self.group.go(self.goal_joint_state, wait=True)
            self.group.stop()
            
        except Exception as e:
                print(e)

          
    def currentPose(self):
        # Get the current end-effector pose
        pose = self.group.get_current_pose().pose
        # Print the end-effector pose
        print("End-effector pose:")
        print(f"Position: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
        print(f"Orientation: x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w}")


    def moveToNamedTarget(self, namedTarget):
        """
        :param namedTarget: A pose defined in the MoveIt! setup assistant! -> String
        """
        print("Starting the moveToNamedTarget method!")
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
        """
        # TODO: Documentation! 
        :params x,y,z position -> float
        :params qx,qy,qz,qw quaternion orientation -> float
        :return: returns a filled PoseStamped() object which is fed to other system parts 
        """
        # Create a PoseStamped message
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id 
        pose.pose.position.x = x 
        pose.pose.position.y = y  
        pose.pose.position.z = z   
        pose.pose.orientation.x = qx  
        pose.pose.orientation.y = qy  
        pose.pose.orientation.z = qz  
        pose.pose.orientation.w = qw  
        quaternion = np.array([qx, qy, qz, qw])
        quaternionLength = np.sqrt(np.sum(quaternion ** 2))

        if quaternionLength > 1.0:
            print(f"Pose orientation has to be normalized, current lenght: {quaternionLength} exceeds 1!")
            pose.pose.orientation.x = qx / quaternionLength
            pose.pose.orientation.y = qy / quaternionLength
            pose.pose.orientation.z = qz / quaternionLength
            pose.pose.orientation.w = qw / quaternionLength
            return pose

        else:
            return pose
            
            
    def moveToCartesianPose(self, ee_pose):
        """
        # TODO: Documentation!
        :param ee_pose: x, y, z, qx, qy, qz --> array (1x7) of 7 floats
        :return:
        """
        self.ee_pose = ee_pose
        print("Starting the moveToCartesianPose method!")
        poseFormattedCartesian = self.create_pose(*self.ee_pose)
        print(f"The desired pose: {poseFormattedCartesian}")
        self.group.set_pose_target(poseFormattedCartesian, self.ee_link[0])
        print("Defined the pose and attempting to move the arm!")
        # TODO: There is planning and moving as a separate feature. Go does both.
        # TODO: Might be better to split the functionality and do loop which waits for a non-error return from planning?
        self.group.go( wait=True)
        self.group.stop()
        
            
    def getIK(self, target_pose, current_joint_state):
        """
        :param target_pose: -> xyz position of the robot ee and xyzw quaternion orientation, a 7x1 array.
        :return: 4x1 array of solved joint states to get into the target position
        """
        self.target_pose = target_pose
        poseFormattedIK = self.create_pose(*self.target_pose)
        print("Starting the getIK method!")
        print(f"Target pose: {poseFormattedIK}")
        
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
        # Filling in the PositionIKRequest object
        serviceRequest = PositionIKRequest()
        serviceRequest.group_name = self.group_name
        serviceRequest.pose_stamped = poseFormattedIK
        serviceRequest.robot_state = robotTemp
        serviceRequest.timeout.secs = 1

        
        try:
            if self.group.has_end_effector_link():
                serviceRequest.ik_link_name = self.ee_link[0]
                
        except Exception as e:
            print(f"No end effector link found: {e}")

        print(f"Service request: {serviceRequest}")

        resp = compute_ik(serviceRequest)
        print(f"Computed IK: {resp}")
        return list(resp.solution.joint_state.position)


    def getEeVelocityVector(self):
        """
        Possible daemon in a separate thead calculating the ee_velocity_vector?
        """
        samplingTime = 0.1 # Consider making an input argument to the method!
        
        #! TODO: Test this functionality if it will be implemented via Python!
        
        firstSample = self.group.get_current_pose(self.ee_link[0]).pose
        time.sleep(samplingTime)
        secondSample = self.group.get_current_pose(self.ee_link[0]).pose
        
        # Linear velocity
        vx = (secondSample.position.x - firstSample.position.x) / samplingTime
        vy = (secondSample.position.y - firstSample.position.y) / samplingTime
        vz = (secondSample.position.z - firstSample.position.z) / samplingTime
        linearVelocity = [vx, vy, vz]

        # Calculate angular velocity
        wx = (secondSample.orientation.x - firstSample.orientation.x) / samplingTime
        wy = (secondSample.orientation.y - firstSample.orientation.y) / samplingTime
        wz = (secondSample.orientation.z - firstSample.orientation.z) / samplingTime
        angularVelocity = [wx, wy, wz]
        
        endEffectorVelocityVector = linearVelocity + angularVelocity
        return eeVelocityVector
        
        
    def getJacobianMatrix(self, joint_state):
        """
        :param joint_state: -> current joint position values -> array (1x4) of 4 floats
        :return: 6x4 array 
        """
        print("Starting the getJacobian method!")
        self.joint_state = joint_state 
        jacobianMatrix = self.group.get_jacobian_matrix(self.joint_state)
        return jacobianMatrix
        
     
        


def main():

  try:
    print("Starting the AntropArmsPythonInterface!")
    testing = AntropArmsPythonInterface()
    achievableJointState = [-0.2918368955004258, -0.06868186235263263, -0.194198852046922, 1.8693671028963053] 
    # Forward kinematics
    #testing.getFK(achievableJointState)
    # Move by feeding joint states
    #testing.moveToJointStateGoal(achievableJointState)
    #testing.currentPose()
    testCurrentJointStates = testing.getCurrentJointStates()
    # Working pose for "left_arm" group:
    # Position: x=-0.3196075296701223, y=0.36576859700616704, z=1.2952693892762086
    # Orientation: x=0.1446269355378438, y=0.10098839507898862, z=-0.13750360498404174, w=0.9746677137325802
    position = [-0.3196075296701223,0.36576859700616704,1.2952693892762086,0.1446269355378438,0.10098839507898862,-0.13750360498404174,0.9746677137325802]
    # Inverse kinematics
    #testing.getIK(position,testCurrentJointStates)
    # Move by feeding end-effector pose
    #testing.moveToCartesianPose(position)
    #testing.moveToCartesianPose(position) #Second one just in case the first planning fails until attempts are added!
    #testing.currentPose()
    testing.getJacobianMatrix(testCurrentJointStates)
 

  except rospy.ROSInterruptException:
    return

  finally:
      print("Closing everything!")
      #moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
  main()
