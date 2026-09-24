# g1_sim_docker

Docker container to run the Unitree G1 humanoid in MuJoCo with ROS2,
walking with a choice of two RL policies.

## Requirements

- Docker

## Build

```bash
git clone git@github.com:MOBILAB-UDESC/g1_sim_docker.git
cd g1_sim_docker
docker build -t g1_sim .
```

## Run

```bash
./start_g1_docker.sh
```

## Launch the simulation

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

ros2 launch g1_simulation unitree_g1.launch.py policy:=gr00t_wbc
```

Other options for `policy:=`: `unitree_rl_lab`, `gr00t_wbc` or `none` for no walking
policy.

Move the robot with with the keyboard in another terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
