## PREREQ: Run XRCE bridge
```bash
MicroXRCEAgent udp4 -p 8888
```

## PREREQ: Run Gazebo simulation
```bash
cd PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500 # headless if you don't have gui
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

Run example code that should raise the drone to 500m.
```bash
ros2 run px4_ros_com offboard_control
```
