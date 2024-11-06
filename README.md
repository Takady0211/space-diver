# SpaceDiver

This repository contains the codes to control/simulate space-diver.

## Requirements

- ROS 2 Humble
- Git

## Setup

Create works space to build project.

```bash
mkdir spacediver_ws/src -p
cd spacediver_ws/src
```

Clone this repository and SpaceDyn, which calculates robots' dynamics.

```bash
# spacediver_ws/src
git clone --recursive git@github.com:Space-Robotics-Laboratory/space-diver.git
git submodule update --init --recursive
```

Install required packages using rosdep.

```bash
rosdep install -i -y --from-path src
```

> [!NOTE]
> If you have never used rosdep, yo need to initialize it first.
>
>```bash
>sudo apt install python3-rosdep
>sudo rosdep init
>rosdep update # do not put sudo here. if you did it mistakenly, fix it by "sudo rosdep fix-permissions"
>```

After installing required packages, build it for the first time!

```bash
cd ../ # move to spacediver_ws
colcon build --symlink-install
```

## Launch Simulation

After you built the project, you can launch gazebo simulation.

```bash
# At spacediver_ws
source install/setup.bash
ros2 launch spacediver_ros2_control spacediver.launch.py
```

## Development

Create a branch following the naming rule as `feature/*******`.

```bash
git checkout feature/******* -b
```

> [!IMPORTANT]
> It's important to avoid developing directory under develop branch, nor main branch.

## References

- [How to create a gazebo model using SDF](https://www.theconstruct.ai/gazebo-5-minutes-004-create-gazebo-model-using-sdf/)
