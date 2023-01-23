cmake_minimum_required(VERSION 2.8.3)
project(antrop_arms_control)

find_package(catkin REQUIRED message_generation std_msgs rospy roscpp dynamic_reconfigure)

add_message_files(
        FILES
        PIDController.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

generate_dynamic_reconfigure_options(
	cfg/DynamicReconfigurePid.cfg
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
