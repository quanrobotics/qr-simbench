# QR-SimBench dev/runtime image: ROS 2 Jazzy + Gazebo Harmonic + Nav2 + rosbag2/MCAP (all upstream).
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-minimal-tb3-sim \
      ros-jazzy-nav2-simple-commander \
      ros-jazzy-ros-gz \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-rviz2 \
      python3-pip git procps mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Product package deps (Python 3.12 from the base image).
RUN pip3 install --no-cache-dir --break-system-packages \
      "typer>=0.12" "pydantic>=2.6" "mcap>=1.1" "mcap-ros2-support>=0.5" "jinja2>=3.1" pytest

ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1
WORKDIR /workspace
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
