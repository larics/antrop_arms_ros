cmake_minimum_required(VERSION 2.8.3)
project(antrop_arms_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED message_generation std_msgs rospy roscpp dynamic_reconfigure sensor_msgs message_generation)

add_message_files(
        FILES
        PIDController.msg
        TorsoJointPositions.msg
        JointArmCmd.msg
        CartesianArmCmd.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

generate_dynamic_reconfigure_options(
	cfg/DynamicReconfigurePid.cfg
)

catkin_package(
        CATKIN_DEPENDS message_runtime roscpp rospy std_msgs hpe_ros_msgs
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
