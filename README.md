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

### Setup

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
---

## Development

1. Create a feature branch (`git checkout -b feat/amazing-feature`)
2. Commit your changes (`git commit -m 'Add amazing feature'`)
3. Push to the branch (`git push origin feat/amazing-feature`)
4. Open a Pull Request

---
