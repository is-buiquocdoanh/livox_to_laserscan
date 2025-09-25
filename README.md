# Livox to LaserScan Converter

## Introduction
This package converts **Livox LiDAR data** (published by `livox_ros_driver2`) into a **2D LaserScan** message, which can then be used in SLAM or Navigation frameworks in ROS2 (e.g., `slam_toolbox`, `cartographer`, `nav2`).

It will:
- Subcribe `CustomMsg (/livox/lidar)` to convert `pointcloud2 (/livox/points)`
- Subscribe to `/livox/points` to convert `sensor_msgs/LaserScan`
- Publish the result to the `scan` topic.
- Set the LaserScan `frame_id` to `laser`.
- Provide a static transform from `laser` → `base_link`.

---

## Requirements
- ROS2 Humble (or newer).
- `livox_ros_driver2` installed and running. [Livox_ros_driver2](https://github.com/Livox-SDK/livox_ros2_driver)
- Standard ROS2 dependencies: `rclcpp`, `sensor_msgs`, `tf2_ros`.

---

## Build & Install
```bash
cd ~/ros2_ws/src
git clone https://github.com/is-buiquocdoanh/livox_to_laserscan.git
cd ~/ros2_ws
colcon build --packages-select livox_to_laserscan
source install/setup.bash
```

### Usage
1. Run the livox driver
``` bash
ros2 launch livox_ros_driver2 msg_MID360.launch.py
```
2. Run the converter node
``` bash
ros2 launch livox_to_laserscan livox_scan.launch.py
```
3. Check available topics
```bash
ros2 topic list
ros2 topic echo /scan
```
![Livox_to_scan](docs/livox_to_scan.gif)

## Author
Name: BUI QUOC DOANH

Email: [doanh@example.com]

Project: go2_unitree

## License
This project is released under the [MIT License](https://opensource.org/license/mit)
