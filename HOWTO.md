## PREREQ: Run XRCE bridge
```bash
MicroXRCEAgent udp4 -p 8888
```

## PREREQ: Run Gazebo simulation
```bash
cd xxx/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500 # headless if you don't have gui
```

## PREREQ: Build PX4_ros_com package (might have to source ROS first)
```bash
colcon build
```

## PREREQ: Source setup files
```bash
source install/setup.bash
```

## Run flight example
First, start to track telemetry to verify results later.
```bash
ros2 run px4_ros_com vehicle_gps_position_listener
```

Disable the safe mode.
```bash
mavproxy.py --master=udp:127.0.0.1:14540
param set COM_FLTMODE2 7 # 7 = Offboard, as mentioned here https://docs.px4.io/main/en/advanced_config/parameter_reference.html#commander
```

Offboard control node (by default goes to 6m).
```bash
ros2 run px4_ros_com offboard_control
```

Run offboard commands on demand:
```bash
ros2 topic pub /custom_trajectory px4_msgs/msg/TrajectorySetpoint "{ position: [ 0.0, 0.0, -50.0 ], velocity: [0.0, 0.0, 0.0],  yaw: -3.14 }"
```

## Run predefined motion (up, left, forward, rotate) x2:
1.
```
ros2 run px4_ros_com offboard_control # start offboard node
```
2. Arm from QGroundCtonrol / via mavproxy by running ```arm throttle```.
3. Record
```
ros2 bag record /fmu/out/vehicle_local_position
```
4. Start motions
```
ros2 run px4_ros_com maneuver.py # start the script
```
