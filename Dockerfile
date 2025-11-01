ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

ENV COLCON_WS=/root/colcon_ws
ENV COLCON_WS_SRC=/root/colcon_ws/src
ENV PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"
ENV DEBIAN_FRONTEND=noninteractive

ARG GZ_VERSION

RUN apt-get update -qq \
    && apt-get install -y \
        wget git \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update -qq \
    && apt-get install -y \
        gz-${GZ_VERSION} \
        build-essential \
        python3-pip \
        libgl1-mesa-dri \
        libgl1 \
        libegl1 \
        libgles2 \
        libvulkan1 \
        mesa-utils \
        vulkan-tools \
        ros-${ROS_DISTRO}-rcl-interfaces \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-builtin-interfaces \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-sdformat-urdf \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-actuator-msgs \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-nav2* \
        ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${COLCON_WS_SRC}

COPY . ${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2

RUN cd ${COLCON_WS} \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build

WORKDIR ${COLCON_WS}

# Set environment variables for resource and plugin paths
ENV GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2/saye_description/models:${COLCON_WS}/install/saye_description/share
ENV ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2
ENV QT_XCB_GL_INTEGRATION=egl \
    LIBGL_ALWAYS_INDIRECT=0 \
    LIBGL_ALWAYS_SOFTWARE=0 \
    __GL_SYNC_TO_VBLANK=0 \
    __GL_THREADED_OPTIMIZATIONS=1 \
    GZ_SIM_RENDER_ENGINE=ogre2 \
    GZ_GUI_RENDER_ENGINE=ogre2

# Source ROS and workspace on shell startup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash", "-c", "cd /root/colcon_ws && source install/setup.bash && exec bash"]
