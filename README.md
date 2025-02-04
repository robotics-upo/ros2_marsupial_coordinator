# UAV-AGV-Control

  - CONTROL MANUAL
 ```
ros2 launch marsupial_simulator_ros2 marsupial_manual_simulation.launch.py world:=stage_5.world 
 ```
Para despegar el drone: 
 ```
ros2 topic pub /takeoff std_msgs/msg/Empty {} --once 
 ```
 

Para aterrizar el drone: 
 ```
ros2 topic pub /land std_msgs/msg/Empty {} --once 
 ```


  - CONTROL AUTOMATICO
