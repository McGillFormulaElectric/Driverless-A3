FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-foxglove-bridge \
      ros-humble-tf2-ros \
      ros-humble-tf2-geometry-msgs \
      ros-humble-tf2-tools \
      iproute2 \
      iputils-ping \
      curl \
      git \
      vim \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy

ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42
ENV CYCLONEDDS_URI=file:///opt/a3/cyclonedds.xml

WORKDIR /opt/a3
COPY cyclonedds.xml /opt/a3/cyclonedds.xml
COPY entrypoint.sh /opt/a3/entrypoint.sh
RUN chmod +x /opt/a3/entrypoint.sh

WORKDIR /workspace
ENTRYPOINT ["/opt/a3/entrypoint.sh"]
CMD ["bash"]
