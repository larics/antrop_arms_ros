# antrop_arms

Repository for antropomorphic arms for aerial manipulation tasks. 

Model of antropomorphic arms has been generated using: [fusion2urdf](https://github.com/syuntoku14/fusion2urdf/blob/master/URDF_Exporter/URDF_Exporter.py). 

In order to run antropomorphic arms run following command: 
``` 
roslaunch antrop_arms_description gazebo.launch
```

In order to run controllers run following command: 
```
roslaunch antrop_arms_description controller.launch
```

TODO: 
- [ ] Add link with no inertia for base link 
- [ ] Enable controllers for the arms
- [ ] Add direct kinematics/inverse kinematics (moveIt?) 
- [ ] Integrate with human pose estimation 
