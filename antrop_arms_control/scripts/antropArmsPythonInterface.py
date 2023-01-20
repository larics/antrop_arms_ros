import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import copy
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from std_msgs.msg import String, Header, Float64
from moveit_commander.conversions import pose_to_list
import numpy as np
import time
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, GetPositionFKResponse, \
    GetPositionIKResponse, GetPositionIKRequest
from tf import TransformListener
from pid import PID
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

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
        self.formatted_group_name = None
        if self.group_name == "left_arm":
            self.formatted_group_name = "leftArm"

        group = moveit_commander.MoveGroupCommander(self.group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        planning_frame = group.get_planning_frame()
        rospy.loginfo("Reference frame is: {}".format(planning_frame))
        ee_link = group.get_end_effector_link()
        rospy.loginfo("End effector is: {}".format(ee_link))
        available_groups = robot.get_group_names()
        rospy.loginfo("Available groups are: {}".format(available_groups))

        # Variables for practical use later in the code!
        self.rate = rospy.Rate(20)
        self.deltaT = (1 / 20)
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
        # self.DH_matrix = self.correction_matrix * np.matrix([[]])
        # Reference pose
        self.reference_pose = None
        # Initiate services
        self._init_services()
        # Initiate publishers/subscribers
        self._init_publishers()
        self._init_subscribers()
        # Initiate the PID() object to compute errors
        # Position x
        self.pid_controller_x = PID()
        self.pid_controller_x.set_kp(0.1)
        self.pid_controller_x.set_ki(0)
        self.pid_controller_x.set_kd(0)
        # Position y
        self.pid_controller_y = PID()
        self.pid_controller_y.set_kp(0.1)
        self.pid_controller_y.set_ki(0)
        self.pid_controller_y.set_kd(0)
        # Position z
        self.pid_controller_z = PID()
        self.pid_controller_z.set_kp(0.1)
        self.pid_controller_z.set_ki(0)
        self.pid_controller_z.set_kd(0)
        # Active controller:
        self.active_controller = "trajectory"
        # Controllers:
        self.joint_position_controllers = [
            'base_shoulder_left_joint_position_controller',
            'shoulder_elbow_left_joint_position_controller',
            'base_shoulder_right_joint_position_controller',
            'shoulder_elbow_right_joint_position_controller',
            'elbow_forearm_left_joint_position_controller',
            'elbow_forearm_right_joint_position_controller',
            'base_shoulder_left_pitch_joint_position_controller',
            'base_shoulder_right_pitch_joint_position_controller']
        self.follow_trajectory_controllers = [
            'left_arm_controller',
            'right_arm_controller']
        self.switch_strictness = 2

    def _init_services(self):

        try:
            # Service used by getFK method!
            rospy.wait_for_service("compute_fk")
            self.moveit_fk = rospy.ServiceProxy("compute_fk", GetPositionFK)
            rospy.loginfo("Compute FK service initiated!")
            # Service used by getIK method!
            rospy.wait_for_service('compute_ik')
            self.compute_ik = rospy.ServiceProxy("compute_ik", GetPositionIK)
            rospy.loginfo("Compute IK service initiated!")
            # Service used for switching controllers
            self.switchControllerService = rospy.ServiceProxy('/antrop_arms/controller_manager/switch_controller',
                                                              SwitchController)
            rospy.loginfo("Controller manager service initiated!")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

        finally:
            rospy.loginfo("All services initiated correctly!")

    def _init_publishers(self):

        try:
            # Check for argument: latch=True
            # Current pose publisher
            self.current_pose_publisher = rospy.Publisher("/{}/pose/current".format(self.formatted_group_name), Pose,
                                                          queue_size=10)
            # Joint position publishers
            self.left_arm_shoulder = rospy.Publisher(
                "/antrop_arms/base_shoulder_left_joint_position_controller/command", Float64, queue_size=10)
            self.left_arm_shoulder_pitch = rospy.Publisher(
                "/antrop_arms/base_shoulder_left_pitch_joint_position_controller/command", Float64, queue_size=10)
            self.left_arm_shoulder_elbow = rospy.Publisher(
                "/antrop_arms/shoulder_elbow_left_joint_position_controller/command", Float64, queue_size=10)
            self.left_arm_elbow_forearm = rospy.Publisher(
                "/antrop_arms/elbow_forearm_left_joint_position_controller/command", Float64, queue_size=10)
            # Right arm joints
            self.right_arm_shoulder = rospy.Publisher(
                "/antrop_arms/base_shoulder_right_joint_position_controller/command", Float64, queue_size=10)
            self.right_arm_shoulder_pitch = rospy.Publisher(
                "/antrop_arms/base_shoulder_right_pitch_joint_position_controller/command", Float64, queue_size=10)
            self.right_arm_shoulder_elbow = rospy.Publisher(
                "/antrop_arms/shoulder_elbow_right_joint_position_controller/command", Float64, queue_size=10)
            self.right_arm_elbow_forearm = rospy.Publisher(
                "/antrop_arms/elbow_forearm_right_joint_position_controller/command", Float64, queue_size=10)

        except Exception as e:
            rospy.logerr("Publisher initialization failed: {}".format(e))

        finally:
            rospy.loginfo("All publishers initiated correctly!")

    def _init_subscribers(self):

        try:
            self.group_pose_command = rospy.Subscriber("/{}/pose/command".format(self.formatted_group_name), Pose,
                                                       self.poolReferencePose, queue_size=1)
            self.controller_manager = rospy.Subscriber("/controller/manager".format(self.formatted_group_name), String,
                                                       self.switchControllerCallback, queue_size=1)
        except Exception as e:
            rospy.logerr("Subscriber initialization failed: {}".format(e))

        finally:
            rospy.loginfo("All subscribers initiated correctly!")

    def createSwitchControllerRequest(self, start_controllers, stop_controllers):
        """
        :param start_controllers:
        :param stop_controllers:
        :return:
        """
        switchControllerRequest = SwitchControllerRequest()
        switchControllerRequest.start_controllers = start_controllers
        switchControllerRequest.stop_controllers = stop_controllers
        switchControllerRequest.strictness = self.switch_strictness
        rospy.loginfo("Updated SwitchControllerRequest: {}".format(switchControllerRequest))
        return switchControllerRequest

    def switchControllerCallback(self, selected_controller):
        if selected_controller.data == "servo":
            rospy.loginfo("Selected the servo controller, attempting to activate!")
            switchToServoRequest = self.createSwitchControllerRequest(self.joint_position_controllers,
                                                                      self.follow_trajectory_controllers)
            responseServo = self.switchControllerService(switchToServoRequest)
            rospy.loginfo("The response for calling the servoRequest: {}".format(responseServo))
            self.active_controller = "servo"
            rospy.loginfo("Started controllers: {}".format(self.active_controller))

        elif selected_controller.data == "trajectory":
            rospy.loginfo("Selected the trajectory controller, attempting to activate!")
            switchToTrajectoryRequest = self.createSwitchControllerRequest(self.follow_trajectory_controllers,
                                                                           self.joint_position_controllers)
            responseTrajectory = self.switchControllerService(switchToTrajectoryRequest)
            rospy.loginfo("The response for calling the servoRequest: {}".format(responseTrajectory))
            self.active_controller = "trajectory"
            rospy.loginfo("Started controllers: {}".format(self.active_controller))

        else:
            rospy.logerr("Incorrect controller selected! Received: {}".format(selected_controller))
            rospy.loginfo("The passed 'servo' is of type: {}".format(type(selected_controller)))

    def poolReferencePose(self, reference):
        self.reference_pose = reference

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
        :param: goal_joint_state: Desired joint values for the move_group selected!  ->   List of 4 floats
        Ex.: achievable_goal = [-0.2918368955004258, -0.06868186235263263, -0.194198852046922, 1.8693671028963053]
        """

        self.goal_joint_state = goal_joint_state
        rospy.loginfo("Starting the moveToJointStateGoal method!")
        rospy.loginfo("The goal joint_state: {}".format(self.goal_joint_state))
        try:
            self.group.go(self.goal_joint_state, wait=True)
            self.group.stop()

        except Exception as e:
            rospy.logerr(e)

    def getCurrentPose(self):
        """
        Debugging method used to log current cartesian pose
        """
        # Get the current end-effector pose
        currentPose = self.group.get_current_pose().pose
        # Log the end-effector pose
        rospy.loginfo(
            "Position: x={}, y={}, z={}".format(currentPose.position.x, currentPose.position.y, currentPose.position.z))
        rospy.loginfo("Orientation: x={}, y={}, z={}, w={}".format(currentPose.orientation.x, currentPose.orientation.y,
                                                                   currentPose.orientation.z,
                                                                   currentPose.orientation.w))
        return currentPose

    def moveToNamedTarget(self, namedTarget):
        """
        :param namedTarget: A pose defined in the MoveIt! setup assistant! -> String
        """
        rospy.loginfo("Starting the moveToNamedTarget method!")
        self.namedTarget = namedTarget
        self.group.set_named_target(namedTarget)
        # plan = self.group.plan()
        self.group.go(wait=True)

    def getFK(self, input_joint_states):
        """
        :param: input_joint_states -> A 4x1 array of goal joint states
        :return: response -> xyz pose and orientation of the end effector
        """
        self.input_joint_states = input_joint_states
        rospy.loginfo("Starting the getFK method!")

        request = GetPositionFKRequest()
        # End effector link!
        try:
            if self.group.has_end_effector_link():
                request.fk_link_names = self.ee_link

        except Exception as e:
            rospy.logerr("No end effector link found: {}".format(e))

        # Fill request with a list of joint names
        request.robot_state.joint_state.name = self.joint_list
        # Pass a goal joint_state for which the FK will get computed
        request.robot_state.joint_state.position = self.input_joint_states
        # This is the computed pose for the end effector for given joint_states
        response = self.moveit_fk(request)
        rospy.loginfo("Computed FK: {}".format(response))
        return response

    def quaternionNormalization(self, quaternion):
        """
        Method used to normalize quaternions, keeps iterating over the given parameter until the expected length is
        accomplished thus returning the correct value.
        :param quaternion: -> 1x4 array containing qx, qy, qz, qw orientation values of a pose
        :return: -> a normalized 1x4 array of length value = 1
        """
        quaternion = np.array(quaternion)
        while 1:
            quaternionLength = np.sqrt(np.sum(quaternion ** 2))
            iteration = 0
            if quaternionLength > 1.0:
                rospy.logwarn(
                    "Pose orientation has to be normalized, current length: {} exceeds 1!".format(quaternionLength))
                iteration = iteration + 1
                rospy.loginfo("Iteration {}".format(iteration))
                quaternion[0] = quaternion[0] / quaternionLength
                quaternion[1] = quaternion[1] / quaternionLength
                quaternion[2] = quaternion[2] / quaternionLength
                quaternion[3] = quaternion[3] / quaternionLength
            else:
                rospy.loginfo("Successfully normalized! Quaternion length: {}".format(quaternionLength))
                return quaternion

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
        try:
            normalizedQuaternion = self.quaternionNormalization([qx, qy, qz, qw])
            # Extract normalized values of quaternions!
            pose.pose.orientation.x = normalizedQuaternion[0]
            pose.pose.orientation.y = normalizedQuaternion[1]
            pose.pose.orientation.z = normalizedQuaternion[2]
            pose.pose.orientation.w = normalizedQuaternion[3]

        except Exception as e:
            rospy.logerr("Normalization failed: {}".format(e))

        return pose

    def moveToCartesianPose(self, ee_pose):
        """
        # TODO: Documentation!
        :param ee_pose: x, y, z, qx, qy, qz --> array (1x7) of 7 floats
        :return:
        """
        self.ee_pose = ee_pose
        rospy.loginfo("Starting the moveToCartesianPose method!")
        poseFormattedCartesian = self.create_pose(*self.ee_pose)
        rospy.loginfo("The targeted cartesian pose: {}".format(poseFormattedCartesian))
        self.group.set_pose_target(poseFormattedCartesian, self.ee_link[0])
        rospy.loginfo("Defined the pose and attempting to move the arm!")
        # TODO: There is planning and moving as a separate feature. Go does both.
        # TODO: Might be better to split the functionality and do loop which waits for a non-error return from planning?
        self.group.go(wait=True)
        self.group.stop()

    def getIK(self, target_pose, current_joint_state):
        """
        :param target_pose: -> xyz position of the robot ee and xyzw quaternion orientation, a 7x1 array.
        :return: 4x1 array of solved joint states to get into the target position
        """
        self.target_pose = target_pose
        self.current_joint_state = current_joint_state
        poseFormattedIK = self.create_pose(*self.target_pose)
        rospy.loginfo("Starting the getIK method!")
        rospy.loginfo("Target pose: {}".format(poseFormattedIK))

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
            rospy.logerr("No end effector link found: {}".format(e))

        rospy.loginfo("Service request: {}".format(serviceRequest))

        resp = self.compute_ik(serviceRequest)
        rospy.loginfo("Computed IK: {}".format(resp))
        return list(resp.solution.joint_state.position)

    def getJacobianMatrix(self, joint_state):
        """
        :param joint_state: -> current joint position values -> array (1x4) of 4 floats
        :return: 6x4 array
        """
        rospy.loginfo("Starting the getJacobianMatrix method!")
        self.joint_state = joint_state
        jacobianMatrix = self.group.get_jacobian_matrix(self.joint_state)
        rospy.loginfo("Computed Jacobian matrix for the given joint state: {}".format(jacobianMatrix))
        return jacobianMatrix

    def servoCtl(self):
        # TODO: Add all those methods for servo ctl
        pass

    def run(self):
        """
        """
        try:
            while not rospy.is_shutdown():
                rospy.loginfo("Starting the AntropArmsPythonInterface!")
                # TODO: Keep the subscribers running! Update: When spin() is on it doesn't work?
                # rospy.spin()
                rospy.loginfo("Currently activated controller: {}".format(self.active_controller))
                testCurrentJointStates = self.getCurrentJointStates()
                startTime = time.time()
                self.getJacobianMatrix(testCurrentJointStates)
                endTime = time.time()
                elapsed = endTime - startTime
                rospy.loginfo("Elapsed time: {}".format(elapsed))
                currentPose = self.getCurrentPose()
                self.current_pose_publisher.publish(currentPose)
                if self.active_controller == "servo":
                    # Necessary overhead since it has to be calculated for each iteration?
                    currentJointState = self.getCurrentJointStates()
                    inverseJacobian = np.linalg.pinv(self.getJacobianMatrix(currentJointState))
                    # Publishing the current ee pose!

                    formattedCurrentPose = np.array(
                        [currentPose.position.x, currentPose.position.y, currentPose.position.z,
                         currentPose.orientation.x, currentPose.orientation.y,
                         currentPose.orientation.z, currentPose.orientation.w])
                    rospy.loginfo("Current pose: {}".format(currentPose))
                    # print("Current pose position: {}".format(currentPose.position))

                    # Fetching the reference pose and passing it to the PID controller
                    if not self.reference_pose == None:
                        formattedReferecePose = np.array(
                            [self.reference_pose.position.x, self.reference_pose.position.y,
                             self.reference_pose.position.z,
                             self.reference_pose.orientation.x, self.reference_pose.orientation.y,
                             self.reference_pose.orientation.z, self.reference_pose.orientation.w])
                        # Calculate positional errors
                        eePositionX = self.pid_controller_x.compute(formattedReferecePose[0],
                                                                    formattedCurrentPose[0])
                        eePositionY = self.pid_controller_y.compute(formattedReferecePose[1],
                                                                    formattedCurrentPose[1])
                        eePositionZ = self.pid_controller_z.compute(formattedReferecePose[2],
                                                                    formattedCurrentPose[2])
                        # Logging
                        rospy.loginfo("This is what the normal subtraction shows:{}".format(
                            formattedReferecePose - formattedCurrentPose))
                        rospy.loginfo("EE Position error x: {}".format(eePositionX))
                        rospy.loginfo("EE Position error y: {}".format(eePositionY))
                        rospy.loginfo("EE Position error z: {}".format(eePositionZ))
                        # TODO: Remove the hard coded orientation down the line
                        eeVelocityVector = np.array([eePositionX, eePositionY, eePositionZ, 0, 0, 0])
                        rospy.loginfo("EE velocity vector: {}".format(eeVelocityVector))
                        jointVelocity = np.dot(inverseJacobian, eeVelocityVector)
                        rospy.loginfo("Joint velocity vector: {}".format(jointVelocity))
                        # Calculate the delta joint state
                        newJointState = np.dot(jointVelocity, self.deltaT)
                        # TODO: Add code for passing the joint states!
                        self.left_arm_shoulder.publish(newJointState[0])
                        self.left_arm_shoulder_pitch.publish(newJointState[1])
                        self.left_arm_shoulder_elbow.publish(newJointState[2])
                        self.left_arm_elbow_forearm.publish(newJointState[3])

                    else:
                        rospy.logwarn("No reference given!")

                elif self.active_controller == "trajectory":
                    rospy.loginfo("Again entering trajectory!")

                    if self.active_controller == "trajectory":
                        rospy.loginfo("Still no trajectory implemented, switch to servo!")
                        pass

                else:
                    rospy.logerr("The active controller: {}, isn't defined!".format(self.active_controller))

                self.rate.sleep()

            # achievableJointState = [-0.2918368955004258, -0.06868186235263263, -0.194198852046922, 1.8693671028963053]
            # Forward kinematics
            # self.getFK(achievableJointState)
            # Move by feeding joint states
            # self.moveToJointStateGoal(achievableJointState)
            # self.getCurrentPose()
            # testCurrentJointStates = self.getCurrentJointStates()
            # Working pose for "left_arm" group:
            # Position: x=-0.3196075296701223, y=0.36576859700616704, z=1.2952693892762086
            # Orientation: x=0.1446269355378438, y=0.10098839507898862, z=-0.13750360498404174, w=0.9746677137325802
            # position = [-0.3196075296701223, 0.36576859700616704, 1.2952693892762086, 0.1446269355378438, 0.10098839507898862,
            #            -0.13750360498404174, 0.9746677137325802]
            # Inverse kinematics
            # self.getIK(position, testCurrentJointStates)
            # Move by feeding end-effector pose
            # self.moveToCartesianPose(position)
            # self.moveToCartesianPose(position)  # Second one just in case the first planning fails until attempts are added!
            # self.getCurrentPose()


        except Exception as e:
            rospy.logerr("Failed during run() method execution: {}".format(e))

        finally:
            rospy.loginfo("Closing everything!")
            # moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    testing = AntropArmsPythonInterface()
    testing.run()