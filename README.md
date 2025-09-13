<div align="center">

<img width="5280" height="719" alt="fbot_navigation" src="https://github.com/user-attachments/assets/21356d4a-a9cf-455a-a2cf-98670372ec48" />

![UBUNTU](https://img.shields.io/badge/UBUNTU-22.04-orange?style=for-the-badsge&logo=ubuntu)
![python](https://img.shields.io/badge/python-3.10-blue?style=for-the-badsge&logo=python)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badsge&logo=ros)

**Navigation, mapping, localization, and path planning for BORIS robot in RoboCup@Home competitions**

[Overview](#overview) • [Architecture](#architecture) • [Installation](#installation) • [Usage](#usage) • [Development](#development)

</div>

---

## Overview

`fbot_navigation` provides ROS2-based navigation, mapping, localization, and path planning for the BORIS robot. It integrates SLAM, Nav2, and robot localization for autonomous navigation in RoboCup@Home and similar robotics competitions.

---

## Architecture

### Package Structure

```
fbot_navigation/
├── launch/         # ROS2 launch files for navigation, mapping, localization, controllers
├── maps/           # Map files (.pgm, .yaml) for navigation and SLAM
├── param/          # Parameter files for EKF, Nav2, SLAM Toolbox
├── rviz/           # RViz configuration files for mapping and navigation
├── CMakeLists.txt  # Build configuration
├── package.xml     # ROS2 package manifest
```

---

## Installation

### Prerequisites

- ROS2 Humble
- Python 3.10+
- Ubuntu 22.04
- Dependencies listed in `package.xml`

### Initial Setup

1. **Clone the repository into your ROS workspace:**
   ```bash
   cd ~/fbot_ws/src
   git clone https://github.com/fbotathome/fbot_navigation.git
   ```

2. **Install dependencies:**
   ```bash
   cd ~/fbot_ws
   sudo rosdep init  # Skip if already initialized
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   # No requirements.txt for this package
   ```

3. **Build the workspace:**
   ```bash
   cd ~/fbot_ws
   colcon build --packages-select fbot_navigation
   source install/setup.bash
   ```

---

## Usage

### Launch Navigation Stack

```bash
# Launch full navigation stack
ros2 launch fbot_navigation navigation.launch.py

# Launch mapping
ros2 launch fbot_navigation mapping.launch.py

# Launch slam
ros2 launch fbot_navigation slam.launch.py

# Launch robot localization
ros2 launch fbot_navigation robot_localization.launch.py

# Launch navigation with keepout zones
ros2 launch fbot_navigation navigation_keepout.launch.py

# Launch controllers
ros2 launch fbot_navigation start_controllers.launch.py
```

### RViz Visualization

```bash
# Open RViz for navigation
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix fbot_navigation)/share/fbot_navigation/rviz/navigation.rviz

# Open RViz for mapping
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix fbot_navigation)/share/fbot_navigation/rviz/mapping.rviz
```

---

## How to Map an Environment

```bash
# Launch SLAM
ros2 launch fbot_navigation mapping.launch.py

# Open RViz for mapping
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix fbot_navigation)/share/fbot_navigation/rviz/mapping.rviz

# Move the robot around (running teleop of just pushing the robot)

# When the map is ready just save it 
ros2 run nav2_map_server map_saver_cli -f my_map

# Do not forget to alter the name of the map on the launch file with the name you chose when saving the newly created map.

# Obs.: If the map seems quirky don't worry, sometimes moving it around a little more corrects the drift. If it doesn't help just start over again :)


```

---

## KeepOut Zones Setup

1. Get the map of the environment
2. Open gimp, or other image editor
3. Draw a black square or line over the areas where the robot should not go through
4. Save a copy of the map with a different name
5. Open the file ```bash nav2_params_keepout.yaml``` 
6. Edit the filter_mask_server param to match the name of the newly created map with keepout zones
7. Run the launch as stated previously

---

## Tips
### What to check before running

1. Is the robot on? (base and arm, whatever is needed)
2. Is the LiDAR sending data?
3. Is the emergency button not pressed?
4. Are all the USB devices recognized?
5. Are all the udev rules for USB devices set?
6. After all if nothing works try restarting.


### Param tips and other recommendations

1. When mapping remember to change the launch file to match the name of the generated map
2. For the robot to appear in the map when running nav an initial pose has to be estimated
3. To change robot's velocity you can alter the following params:
   ```
   nav2_params.yaml:
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 1.0
      max_vel_y: 0.0
      max_vel_theta: 1.0
   ```


### Common Errors:

1. Error: Either left or right wheel...index[0]
   Fix: Check if the base is on and if the base and emergency buttons wires are connected and working.
2. Error: Robot not appearing after pose estimation
   Fix: Check the section "What to check before running", however, before restarting try running ```sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger```

---

## Development

1. Create a feature branch (`git checkout -b feat/amazing-feature`)
2. Commit your changes (`git commit -m 'Add amazing feature'`)
3. Push to the branch (`git push origin feat/amazing-feature`)
4. Open a Pull Request

---

## Useful Links

### Basic Guides

1. [Basic Navigation](https://docs.nav2.org/getting_started/index.html#navigating)
2. [Navigation Concepts](https://docs.nav2.org/concepts/index.html#)
3. [Setting up Odometry](https://docs.nav2.org/setup_guides/odom/setup_odom_gz.html)
4. [Smoothing Odometry](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)
5. [Mapping and Localization Basics](https://docs.nav2.org/setup_guides/sensors/mapping_localization.html)
6. [Setting Up Robot Footprint](https://docs.nav2.org/setup_guides/footprint/setup_footprint.html)
7. [Setting Up Navigation Plugins](https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html)
8. [Nav2 Configuration Guide](https://docs.nav2.org/configuration/index.html)
9. [Nav2 Tuning Guide](https://docs.nav2.org/tuning/index.html)
10. [List of Navigation Plugins](https://docs.nav2.org/plugins/index.html#)

### Nav Tutorials

1. [Tutorial: Navigation with SLAM](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
2. [Tutorial: Dynamic Point Following](https://docs.nav2.org/tutorials/docs/navigation2_dynamic_point_following.html)
3. [Tutorial: Navigation with KeepOut Zones](https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html)
4. [Tutorial: Navigation with Speed Limits](https://docs.nav2.org/tutorials/docs/navigation2_with_speed_filter.html)
5. [Tutorial: Navigation with Rotation Shim Controller](https://docs.nav2.org/tutorials/docs/using_shim_controller.html)
5. [Tutorial: Navigation with Obstacle Noise Filtering](https://docs.nav2.org/tutorials/docs/filtering_of_noise-induced_obstacles.html)
6. [Tutorial: Adding Vector Objects to Navigation](https://docs.nav2.org/tutorials/docs/navigation2_with_vector_objects.html)
7. [Tutorial: Creating New Planner](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html)
8. [Tutorial: Creating New Controller](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html)
9. [Tutorial: Creating New Navigator](https://docs.nav2.org/plugin_tutorials/docs/writing_new_navigator_plugin.html)
10. [Tutorial: Creating New Costmap2D](https://docs.nav2.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html)

### SLAM Content

1. [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
2. [SLAM Handbook](http://asrl.utias.utoronto.ca/~tdb/slam/)
3. [Which SLAM should I choose?](https://www.slambotics.org/blog/which-slam-algorithm-should-i-choose)


---
