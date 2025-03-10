# ITRC Project

![Project Image](https://www.chosun.com/resizer/v2/ZYPY6QMIOX2MLIDAHFVSAK3V4M.jpg?auth=a480083a83d7e5b8fe76e49a1e8a281f507b72ff882ef45ac6cbd58c99166ec0&width=300&height=300&smart=true)

## TODO

### Localization
- April tag detector & publisher
- Create AprilTagMsg, AprilTagArrayMsg
- Create apriltag localization

### Path planning
- Create path planning node
- Create path planning algorithm
- Create mobile base control system

### Pick-And-Place
- Create Direct-Teaching client node
- Create manipulator controll system

### State machine
- Create state machine in order to schedule all tasks

---

# Ros_bridge_server
Connect web page and ROS server
```bash
sudo apt-get install ros-humble-rosbridge-server
```
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Streaming using web_video_server
```bash
ros2 run web_video_server web_video_server
```
Building a website
```bash
python3 -m http.server
```

---

# MOVEIT2 Installation Guide

## Prerequisites

Before installing MOVEIT2, ensure you have the following dependencies installed:

```bash
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
```

Uninstall Any Pre-existing MoveIt Debians

```bash
sudo apt remove ros-$ROS_DISTRO-moveit*
```

## Installation Steps

1. **Clone the MOVEIT2 repository**:
    ```bash
    git clone https://github.com/moveit/moveit2.git -b $ROS_DISTRO
    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. **Install dependencies**:
    ```bash
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    ```

3. **Build the workspace**:
    - Before build, fix src/moveit2/moveit_configs_utils/setup.py. Make tests_require=["pytest"] disabled
    ```bash
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```


## Verifying the Installation

To verify that MOVEIT2 has been installed correctly, run the following command:
```bash
ros2 launch moveit2_tutorials demo.launch.py
```

If everything is set up correctly, you should see the MOVEIT2 demo running.

## Additional Resources

- [MOVEIT2 Documentation](https://moveit.ros.org/documentation/)
- [MOVEIT2 Tutorials](https://ros-planning.github.io/moveit_tutorials/)

For further assistance, refer to the [MOVEIT2 GitHub repository](https://github.com/ros-planning/moveit2) or join the [ROS Discourse](https://discourse.ros.org/) community.
