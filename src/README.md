## Dependencies
- ZED SDK
- OpenCV 4
- ROS Noetic
- Ubuntu 20
- JetPack 5
- RMD SDK (Check my_actuator for installation details)
- CppLinuxSerial
- YDLIDAR Driver
- Rosserial

## Nodes

| Node Name               | Package            | Functionality |
|-------------------------|--------------------|--------------|
| `script_launcher_node`  | `bt_launcher`      | Runs a bash script to launch the automated behavior tree for both corrosion scanning and grid mapping using services. |
| `trigger_servo_node`    | `corrosion_mapping` | Main node for the automated behavior tree responsible for corrosion mapping. |
| `disk_monitor_node`     | `disk_monitor`     | Monitors disk usage to prevent memory leaks. Also launches the GStreamer pipeline for both ZED and DWE cameras. |
| `bt_lac_zero_node`      | `grid_mapping`     | Main node for the automated behavior tree responsible for grid mapping. |
| `hector_slam`          | `hector_mapping`   | Runs SLAM using Hector SLAM. |
| `motor_publisher_node`  | `helper`           | Helps execute grid mapping properly in the background as necessary. |
| `pose_subscriber`       | `lidar_angle`      | Converts Cartesian Hector SLAM coordinates into angles for better human readability. |
| `joy_to_motor_direction` | `manual`          | Controls LAC and slider movement using a joystick. |
| `crawler_control_node_2` | `my_actuator`     | Main node for controlling the robot's motion. |
| `pose_to_yaw_node`      | `pose_to_yaw`      | Converts Cartesian ZED coordinates into angles for better human readability. |
| `ut_serial_node` & `intrupt_node` | `serialtoros` | Fetches Ultrasonic Thickness (UT) data. |
| `shutdown_service`      | `shutdown`         | Provides functionality to shut down or restart the Jetson device, similar to the Ubuntu GUI. |
| `saver_node`           | `status`           | Fetches various robot statistics. |
| `rosserial`            | `stm`              | Runs STM code. |
| `yaw_controller`       | `straight`         | Uses LiDAR data to ensure the robot moves in a straight line. |
| `ut_thickness_processor` | `ut_data`        | Optimizes UT data for efficient processing. |
| `wifi_strength_monitor` | `wifi_stop`       | Stops the robot if the Wi-Fi signal range is exceeded. |
| `wifi_strength_node`    | `wifi_strength`   | Publishes Wi-Fi strength data from the robot to the laptop. |

## Installation
1. Clone the repository:
   ```sh
   git clone https://github.com/octobotics/grid_mapping.git
   ```
2. Build the workspace:
   ```sh
   cd grid_mapping
   catkin build
   ```
3. Source the workspace:
   ```sh
   source devel/setup.bash
   ```

## Usage
- **Recommended:** Auto-launch using systemd.
- **Manual launch:**
  ```sh
  roslaunch tofd_launch crawler.launch
  ```

## Contributors
- **Nilanjan Chowdhury**
- **Ishan Bhatnagar**
- **Priyanshu Shrivastava**

