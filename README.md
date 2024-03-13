# Detecting_Objects_and_Navigation_of_turtlebot_Using_Aruco_Markers
As the name suggests this project aims to detect objects(batteries placed around the maze), using a advanced logical camera and at the same time navigate through the maze using the Aruco Markers placed on the walls of the environment.

This project uses opencv contribution library along with other ROS2 related dependencies. It uses gazebo to simulate the environment and turtlebot3 waffle to control and move around the maze.

This project Aims to improve the understanding of various Robotics concepts like perception, Navigation/planning and Transformation/modeling.

To Run this project first, start the simulation with ros2 launch turtlebot3_gazebo maze.launch.py , which should start a Gazebo environment as seen in below figure. This environment consists of a turtlebot with an RGB camera and a logical camera.
![Images/Environment.png](https://github.com/HKyatham/Detecting_Object_and_Navigation_of_Turtlebot_Using_Aruco_Markers/blob/main/Images/Environment.png)

The RGB camera can be used to to detect Aruco markers, which are placed on some of the maze’s walls. When an Aruco marker is detected by the camera, its information is published on the topic Topic aruco_markers . An example of a message published on this topic is provided below. In the example, the marker with id 0 has been detected and the pose of the marker in the RGB camera frame camera_rgb_optical_frame is reported. marker_ids is an array.
![Images/Aruco_msg.png](https://github.com/HKyatham/Detecting_Object_and_Navigation_of_Turtlebot_Using_Aruco_Markers/blob/main/Images/Aruco_msg.png)

The logical camera can be used to detect parts located in the environment. It publishes messages mage_msgs/msg/AdvancedLogicalCameraImage on topic /mage/advanced_logical_camera/image . These parts will be floating in the thin air in the environment. The maze environment has two parts, a blue battery and a green battery. The topic output below shows a detected blue battery in the logical camera frame logical_camera_link
– part_poses is an array of parts detected at the same time by the logical camera. When a part is detected, this array is filled out, otherwise it is empty. In the snippet, the camera detected a part with color 2 and type 10, which is actually a blue battery if you inspect messages mage_msgs/msg/Part The pose of the detected part is in the camera frame logical_camera_link
– sensor_pose is the pose of the camera in the odom frame.
![Images/Part_pose.png](https://github.com/HKyatham/Detecting_Object_and_Navigation_of_Turtlebot_Using_Aruco_Markers/blob/main/Images/Part_pose.png)

## Steps Run this Project
First start the simulation, that is start the maze environment with all the necessary nodes started. To do this use the ROS2 launch command
```
ros2 launch turtlebot3_gazebo maze.launch.py
```

Once the Maze is successfully launched. We can start working on the solver node. To start the solver node we can use the similar launch file present in the solver node. 
First open a new terminal and type the below command
```
ros2 launch solver solver.launch.py
```

This will trigger the solver node which will move the turtlebot3 waffle around the maze and after successful navigation, will output the poses of the battery.
