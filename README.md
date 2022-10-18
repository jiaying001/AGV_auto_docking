# AGV auto docking

## Overivew
This is a ROS package used for agv automatically docking. The sensor used is a laser. Docking station pattern is detected by [laser_line_extraction](https://github.com/kam3k/laser_line_extraction).

这是一个用于无人车自动对接的ROS包，传感器使用2D激光雷达。对接基站是用激光点线提取形状。

AGV auto docking in gazebo：

 ![docking_gazebo](https://github.com/jiaying001/agv-auto-docking/blob/main/images/docking_gazebo.gif)
 
 AGV auto docking in rviz：
 
 ![docking_rviz](https://github.com/jiaying001/agv-auto-docking/blob/main/images/docking_rviz.gif)
 

## Usage
 ### input
 - /scan
 ### launch gazebo simulation environment
 
 `` roslaunch auto_dock four_macnum_simulation.launch  ``
 ### docking control methods
 
 - methods 1: PID control
 
 `` roslaunch auto_dock auto_dock_PIDcontrol.launch  ``
 - methods 2: move_base control (hormonic robot)
 
 `` roslaunch auto_dock auto_dock_movebase_control.launch  ``
 - methods 3: self write docking control
 
 `` roslaunch auto_dock auto_dock_control.launch  ``
 
 
 
 
