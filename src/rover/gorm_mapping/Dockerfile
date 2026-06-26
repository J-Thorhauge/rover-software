FROM ros:humble-ros-core AS aruco

WORKDIR /root/gorm_mapping

# Install pip then install this package
RUN apt-get update && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/*

COPY . /root/gorm_mapping

RUN pip3 install --no-cache-dir --index-url https://pypi.org/simple setuptools Cython wheel colcon-common-extensions

# RUN pip3 install --no-cache-dir .
RUN apt-get update && apt-get install -y python3-opencv libopencv-dev python3-pip
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-zenoh-cpp \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf-transformations \
    ros-humble-image-transport-plugins \
    ros-humble-image-pipeline \
    ros-humble-image-transport \
    ros-humble-camera-info-manager-py \
    ros-humble-camera-info-manager

# Default to an interactive shell; run nodes with `ros2 run gorm_mapping <node>`
# CMD ["ros2 launch gorm_mapping aruco_alignment.launch.py"]
