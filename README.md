# ROS_Go_Chase_It

This project consist of design the robot, house it inside the gazebo world, and program it to chase white-colored ball


drive_bot:

* my_robot ROS package hold robot, the white ball, and the world.

* Robot type is a differential drive, which defined in the Unified Robot Description Format (URDF). 

* It has two sensors: a lidar and a camera. 


ball_chaser:

* ball_chaser ROS package hold C++ nodes.

* A drive_bot C++ node will provide a ball_chaser/command_robot service to drive the robot by controlling its linear x and angular z velocities. 

* The service will publish to the wheel joints and return back the requested velocities.

* A process_image C++ node that reads robot’s camera image, analyzes it to determine the presence and position of a white ball. 

* If a white ball exists in the image, node will request a service via a client to drive the robot towards it.
          
  
  
To simulate this project:

* Copy ROS_Go_Chase_It project folder in src folder fo catkin_workspace.

* Run following command in catkin_workspace 
```sh 
$ catkin_make 
$ source devel/setup.bash
```

* Launch the world file by running 
```sh 
$ roslaunch my_robot world.launch
```

* Open new terminal 
* source it and launch the ball chaser by running 
```sh 
$ roslaunch ball_chaser ball_chaser.launch 
``` 
   
