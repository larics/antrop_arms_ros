# Antrop arms - development branch

Table of contents:

- Prerequisites & Dependencies 
- How-to

### Prerequisites & Dependencies 

For simplification of the environment setup process which is initially the biggest hurdle when beginning to work on a project with multiple team members, we use the Docker platform with a prebuilt "Docker Image". 

**Insert Docker tutorial or link to Filips' repo where he goes over this part**

### How-to

This section handles the topic of correctly running the necessary scripts/launch files to simplfy the introduction process for future developers & users.

**Note**:
In order for the bash scripts to work properly make sure you are using the latest Docker image which will be provided in the previous chapter (Prerequisited & Dependencies) 

The simulation process requires several components to work properly, so run these commands in separate terminal tabs in the order they are given:

```bash
roslaunch antrop_arms_description gazebo.launch
```

```bash
roslaunch antrop_arms_control controller.launch
```

```bash
roslaunch antrop_arms_moveit_config move_group.launch
```

```bash
roslaunch antrop_arms_moveit_config moveit_rviz.launch
```

## or just run
```
roslaunch antrop_arms_control antrop_arms_gazebo_control.launch 
```
to start everything. 

Currently there are two types of controllers implemented: 

* JointPositionController - which will be used for real-time servoing
* JointTrajecotryController - MoveIt calculates the inverse kinematics problem and moves the end effector to a desired pose/orientation

**Switching controllers in real time**

With the following command we switch to a different robot state ("servo" or "trajectory"):

```bash
rostopic pub /controller/manager std_msgs/String "data: 'servo'"
```

The reference end effector pose is published like this:

```bash
rostopic pub /leftArm/pose/command geometry_msgs/Pose '{position: {x: -0.015477585470365042, y: 0.20000465570310144, z: 0.9574995401539291}, orientation: {x: 4.290960271475719e-06, y: -2.104719545481403e-05, z: 5.425674097403264e-07, w: 0.9999999997691544}}'
```

