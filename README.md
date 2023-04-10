# Two Link Arm
  A simple two link arm controlled using ROS controls and simulated in Gazebo.
  
 After cloning the package and building in ROS workspace, following command can be used to run simulation in gazebo:
```
$ roslaunch two_link_arm spawn.launch
```

following command runs the controller for the arm:
```
$ rosrun two_link_arm ik_algo.py
```

The keys i, j, l, m can be used to incrementally increase or decrease x and y coordinates of end point in 3D space.

To view camera feed: 
```
$ rqt_image_view
```
and choose image/raw
