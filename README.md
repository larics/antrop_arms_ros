# Antrop arms - development branch

Table of contents:

- Prerequisites & Dependencies 
- How-to

### Prerequisites & Dependencies :

For simplification of the environment setup process which is initially the biggest hurdle when beginning to work on a project with multiple team members, we use the Docker platform with a prebuilt "Docker Image". 

**Insert Docker tutorial or link to Filips' repo where he goes over this part**

### How-to:

This section handles the topic of correctly running the necessary scripts/launch files to simplfy the introduction process for future developers & users.

**Note**:
In order for the bash scripts to work properly make sure you are using the latest Docker image which will be provided in the previous chapter (Prerequisited & Dependencies) 

The simulation process requires several components to work properly, so run these commands in separate terminal tabs in the order they are given:

```bash
roslaunch antrop_arms_description gazebo.launch
```

```bash
roslaunch antrop_arms_description controller.launch
```

```bash
roslaunch antrop_arms_moveit_config move_group.launch
```

```bash
roslaunch antrop_arms_moveit_config moveit_rviz.launch
```

Currently there are two types of controllers implemented: 

* JointPositionController - which will be used for real-time servoing
* JointTrajecotryController - MoveIt calculates the inverse kinematics problem and moves the end effector to a desired pose/orientation

**Switching controllers in real time**

With the following command we switch from the JointPoisionController to the JointTrajectoryController groups:

```bash
rosservice call /antrop_arms/controller_manager/switch_controller "start_controllers:                                                                                                                                                                                                  
- 'left_arm_controller'
- 'right_arm_controller'                                      
stop_controllers:                                                   
- 'base_shoulder_left_joint_position_controller'
- 'shoulder_elbow_left_joint_position_controller'
- 'base_shoulder_right_joint_position_controller'
- 'shoulder_elbow_right_joint_position_controller'
- 'elbow_forearm_left_joint_position_controller'
- 'elbow_forearm_right_joint_position_controller'
- 'base_shoulder_left_pitch_joint_position_controller'
- 'base_shoulder_right_pitch_joint_position_controller'                                      
strictness: 2"
```

