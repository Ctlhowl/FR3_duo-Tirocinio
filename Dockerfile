# Start with an official ROS 2 base image for the desired distribution
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=ctlfab_ros2

# Install essential packages and ROS development tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        sudo curl git wget nano \
        build-essential cmake \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        libx11-6 libxcb1 libxau6 libgl1-mesa-dev xvfb dbus-x11 x11-utils libxkbcommon-x11-0 xsltproc\
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME


# Install and setup coppeliasim
ARG COPPELIA_FILENAME=CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
ARG COPPELIA_URL=https://downloads.coppeliarobotics.com/V4_10_0_rev0/${COPPELIA_FILENAME}.tar.xz

USER root
RUN mkdir -p /opt/coppelia && chown -R $USERNAME:$USERNAME /opt/coppelia

RUN wget -v -O /tmp/coppelia.tar.xz ${COPPELIA_URL} \
    && tar -xf /tmp/coppelia.tar.xz -C /opt/coppelia --strip-components=1 \
    && rm /tmp/coppelia.tar.xz

ENV COPPELIASIM_ROOT_DIR=/ros2_ws/coppelia
ENV PATH=${PATH}:${COPPELIASIM_ROOT_DIR}
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${COPPELIASIM_ROOT_DIR}

# Install ROS 2 dependencies
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-humble-moveit \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-joint-trajectory-controller \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-franka-msgs \
        ros-humble-topic-based-ros2-control \
        ros-humble-xacro \
        ros-humble-ament-cmake-clang-format \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

RUN RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ros2_ws
COPY ./src/franka_ros2/franka.repos /ros2_ws/src/franka_ros2/franka.repos

RUN sudo chown -R $USERNAME:$USERNAME /ros2_ws \
    && vcs import src/franka_ros2 < src/franka_ros2/franka.repos --recursive --skip-existing \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

RUN rm -rf /home/$USERNAME/.ros \
    && rm -rf src \
    && mkdir -p src

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
CMD [ "/bin/bash" ]
WORKDIR /ros2_ws