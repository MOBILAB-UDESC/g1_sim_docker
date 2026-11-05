FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# --- System deps needed by mujoco_ros2_control (from earlier build errors) ---
RUN apt-get update && apt-get install -y \
    git wget curl \
    libyaml-cpp-dev libspdlog-dev libboost-all-dev \
    libglfw3-dev libx11-dev xorg-dev \
    ros-jazzy-urdf ros-jazzy-xacro ros-jazzy-rviz2 \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager \
    ros-jazzy-plotjuggler-ros \
    ros-jazzy-pcl-ros ros-jazzy-perception-pcl ros-jazzy-urdfdom-py \
    libopencv-dev ros-jazzy-pcl-conversions ros-jazzy-cv-bridge libpcl-dev \
    ros-jazzy-tl-expected ros-jazzy-diagnostic-updater \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# --- Python deps for the policy runners ---
RUN pip install --break-system-packages onnxruntime numpy

WORKDIR /workspace
RUN mkdir -p src

# --- Your custom packages, copied in from the build context ---
COPY src/g1_policy_runner src/g1_policy_runner
COPY src/gr00t_wbc_runner src/gr00t_wbc_runner
COPY src/g1_simulation src/g1_simulation
COPY src/mujoco_ros2_control src/mujoco_ros2_control

# --- Build ---
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y \
        --skip-keys franka_description \
        --skip-keys realsense2_description \
        --skip-keys ur_description \
        --skip-keys vision_opencv && \
    colcon build

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

# --- Entrypoint: source everything, then run whatever command is passed in ---
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
