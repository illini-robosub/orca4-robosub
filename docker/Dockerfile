FROM arm64v8/ros:humble-ros-base-jammy

ARG USERNAME=orca4
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y

# Install a few handy tools
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    bash-completion \
    build-essential \
    git \
    glmark2 \
    gnupg \
    iputils-ping \
    lsb-release \
    mlocate \
    software-properties-common \
    sudo \
    wget \
    vim \
  && rm -rf /var/lib/apt/lists/*



# Install Gazebo Harmonic for use with ROS2 Humble
# RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# RUN apt-get update \
#   && apt-get -y --quiet --no-install-recommends install \
#     gz-harmonic \
#     ros-humble-ros-gzharmonic \
#   && rm -rf /var/lib/apt/lists/*


# # Dillon Version of the above 
# # # Install Gazebo Harmonic for use with ROS2 Humble
# # RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# # RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# # RUN apt-get update
# # RUN apt-get install ros-humble-ros-gzharmonic

# RUN /bin/bash -c "source /opt/ros/humble/setup.sh && apt-get update \
#   && apt-get -y --quiet --no-install-recommends install \
#     gz-harmonic \
#     ros-humble-ros-gzharmonic \
#   && rm -rf /var/lib/apt/lists/*"

# # # Install necessary tools for adding repositories and downloading packages (Below is from GPT seeing if i can get it to work)
# # RUN apt-get update && apt-get install -y \
#     curl \
#     ca-certificates

#     # gnupg2 \
  
# # Add ROS 2 Humble repository for ARM64
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#     mkdir -p /ros2_ws/src && \
#     vcs import /ros2_ws/src < /tmp/ros2.repos

# # Install ROS 2 Humble dependencies
# RUN apt update && apt install -y \
#     ros-humble-desktop \
#     ros-humble-gazebo-ros-pkgs \
#     ros-humble-gazebo-msgs \
#     && rm -rf /var/lib/apt/lists/*

# # Install Gazebo repository for ARM64
# RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# # Install Gazebo Harmonic and related packages
# RUN /bin/bash -c "source /opt/ros/humble/setup.sh && apt-get update && \
#     apt-get -y --quiet --no-install-recommends install \
#     gz-harmonic \
#     ros-humble-gazebo-harmonic && \
#     rm -rf /var/lib/apt/lists/*"

# # Clean up unnecessary files
# RUN apt-get clean && rm -rf /var/lib/apt/lists/*


#Esther: install gazebo from source # change this to USER root 
USER root 
RUN apt-get update
RUN apt-get -y install python3-pip python3-venv lsb-release gnupg curl git 
RUN python3 -m venv $HOME/vcs_colcon_installation
RUN . $HOME/vcs_colcon_installation/bin/activate
RUN pip3 install vcstool colcon-common-extensions
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update
RUN apt-get install python3-vcstool python3-colcon-common-extensions
RUN mkdir -p /home/${USERNAME}/workspace/src
WORKDIR /home/${USERNAME}/workspace/src
RUN curl -O https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-harmonic.yaml
RUN vcs import < collection-harmonic.yaml
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
WORKDIR /home/${USERNAME}/workspace/src
RUN apt-get update
RUN apt-get -y install binutils-dev build-essential cmake freeglut3-dev libassimp-dev libavcodec-dev libswscale-dev libavdevice-dev libavformat-dev libavutil-dev libbenchmark-dev libdart6.13-collision-bullet-dev libdart6.13-collision-ode-dev libdart6.13-dev libdart6.13-external-ikfast-dev libdart6.13-external-odelcpsolver-dev libdart6.13-utils-urdf-dev libeigen3-dev libfreeimage-dev libgdal-dev libgflags-dev libglew-dev libgts-dev libjsoncpp-dev libogre-1.9-dev libogre-next-2.3-dev libogre-next-dev libprotobuf-dev libprotoc-dev libpython3-dev libsqlite3-dev libtinyxml2-dev liburdfdom-dev libvulkan-dev libwebsockets-dev libxi-dev libxml2-utils libxmu-dev libyaml-dev libzip-dev libzmq3-dev mesa-utils pkg-config protobuf-compiler python3-dev python3-protobuf python3-psutil python3-pybind11 python3-pytest qml-module-qt-labs-folderlistmodel qml-module-qt-labs-platform qml-module-qt-labs-settings qml-module-qtcharts qml-module-qtgraphicaleffects qml-module-qtlocation qml-module-qtpositioning qml-module-qtqml-models2 qml-module-qtquick-controls qml-module-qtquick-dialogs qml-module-qtquick-layouts qml-module-qtquick-templates2 qml-module-qtquick-window2 qml-module-qtquick2 qtbase5-dev qtdeclarative5-dev qtquickcontrols2-5-dev rubocop ruby swig uuid-dev x11-utils xvfb
#RUN apt -y install \ $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf\|binutils-dev\|build-essential\|cmake\|curl\|freeglut3-dev\|libassimp-dev\|libavcodec-dev\|libswscale-dev\|libavdevice-dev\|libavformat-dev\|libavutil-dev\|libbenchmark-dev\|libdart6.13-collision-bullet-dev\|libdart6.13-collision-ode-dev\|libdart6.13-dev\|libdart6.13-external-ikfast-dev\|libdart6.13-external-odelcpsolver-dev\|libdart6.13-utils-urdf-dev\|libeigen3-dev\|libfreeimage-dev\|libgdal-dev\|libgflags-dev\|libglew-dev\|libgts-dev\|libjsoncpp-dev\|libogre-1.9-dev\|libogre-next-2.3-dev\|libogre-next-dev\|libprotobuf-dev\|libprotoc-dev\|libpython3-dev\|libsqlite3-dev\|libtinyxml2-dev\|liburdfdom-dev\|libvulkan-dev\|libwebsockets-dev\|libxi-dev\|libxml2-utils\|libxmu-dev\|libyaml-dev\|libzip-dev\|libzmq3-dev\|mesa-utils\|pkg-config\|protobuf-compiler\|python3-dev\|python3-protobuf\|python3-psutil\|python3-pybind11\|python3-pytest\|qml-module-qt-labs-folderlistmodel\|qml-module-qt-labs-platform\|qml-module-qt-labs-settings\|qml-module-qtcharts\|qml-module-qtgraphicaleffects\|qml-module-qtlocation\|qml-module-qtpositioning\|qml-module-qtqml-models2\|qml-module-qtquick-controls\|qml-module-qtquick-dialogs\|qml-module-qtquick-layouts\|qml-module-qtquick-templates2\|qml-module-qtquick-window2\|qml-module-qtquick2\|qtbase5-dev\|qtdeclarative5-dev\|qtquickcontrols2-5-dev\|rubocop\|ruby\|swig\|uuid-dev\|x11-utils\|xvfb/d' | tr '\n' ' ')
WORKDIR /home/${USERNAME}/workspace
RUN colcon graph
RUN colcon build --merge-install
RUN . /home/${USERNAME}/workspace/install/setup.bash



# Install NVIDIA software
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/* \
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Install some ardupilot and ardupilot_gazebo prereqs (libgz-sim8-dev is for gz harmonic)
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    python3-wxgtk4.0 \
    rapidjson-dev \
    xterm \
    libgz-sim8-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user
# Required for ArduSub install, but generally a good idea
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo "\n# Added by orca4 Dockerfile:" >> /home/$USERNAME/.bashrc \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to our new user
# esther: chown might not be necessary if we remove the part before gazebo that sets the user to root, though be wary when changing that stuff because it takes 2+ hrs to build gazebo
USER $USERNAME
RUN chown -R $USER:$USER /home/${USERNAME}
ENV USER=$USERNAME
ENV ROOT=/home/${USERNAME}


# Clone ArduSub code
WORKDIR $ROOT

# Martin: Set user back to $root? seemed to work
USER root
RUN mkdir -p $ROOT && chmod 777 $ROOT
RUN git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules

USER root

RUN chown -R $USERNAME:$USERNAME /home/$USERNAME

RUN chmod 644 /home/$USERNAME/.bashrc

# Switch back to user
USER $USERNAME

# Install ArduSub prereqs (this also appends to .bashrc)
WORKDIR /home/$USERNAME/ardupilot
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y
# RUN sudo pip3 install --upgrade pip packaging setuptools wheel

# Build ArduSub
# Note: waf will capture all of the environment variables in ardupilot/.lock-waf_linux_build.
# Any change to enviroment variables will cause a re-build.
# To avoid this call sim_vehicle.py with the "--no-rebuild" option.
WORKDIR /home/$USERNAME/ardupilot
RUN modules/waf/waf-light configure --board sitl \
  && modules/waf/waf-light build --target bin/ardusub

# Clone ardupilot_gazebo code
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git

# Build for Gazebo Harmonic, define this before building ardupilot_gazebo and ros_gz
ENV GZ_VERSION=harmonic

# Build ardupilot_gazebo
RUN [ "/bin/bash" , "-c" , " \
  cd ardupilot_gazebo \
  && git checkout 4b30a3d8 \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  && make -j4" ]

# Break orca4 build into multiple stages to speed up 'docker build':
# -- work that depends on workspace.repos
# -- work that depends on orca4 package.xml files
# -- everything else

# Create colcon workspace and copy workspace.repos file
WORKDIR /home/$USERNAME
RUN mkdir -p colcon_ws/src/orca4
COPY --chown=$USER_UID:$USER_GID workspace.repos colcon_ws/src/orca4

# Get workspace repos
WORKDIR /home/$USERNAME/colcon_ws/src
RUN vcs import < orca4/workspace.repos

# Run rosdep over workspace repos
WORKDIR /home/$USERNAME/colcon_ws
RUN rosdep update \
  && rosdep install -y --from-paths . --ignore-src

# Build everything so far
# Martin: error here, i remember running into this issue before but i forgot how i fixed it -- i'll try to find it  
RUN sudo apt-get update
RUN sudo apt-get install -y ros-humble-image-geometry ros-humble-ros-gz-image ros-humble-rqt-graph ros-humble-rviz2
RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/humble/setup.bash \
  && colcon build" ]

# Copy orca4 package.xml files
COPY --chown=$USER_UID:$USER_GID orca_base/package.xml src/orca4/orca_base/
COPY --chown=$USER_UID:$USER_GID orca_bringup/package.xml src/orca4/orca_bringup/
COPY --chown=$USER_UID:$USER_GID orca_description/package.xml src/orca4/orca_description/
COPY --chown=$USER_UID:$USER_GID orca_msgs/package.xml src/orca4/orca_msgs/
COPY --chown=$USER_UID:$USER_GID orca_nav2/package.xml src/orca4/orca_nav2/
COPY --chown=$USER_UID:$USER_GID orca_shared/package.xml src/orca4/orca_shared/

# Run rosdep over orca4 package.xml files
RUN rosdep update \
  && rosdep install -y --from-paths . --ignore-src

# MAVROS depends on GeographicLib, and GeographicLib needs some datasets
RUN [ "/bin/bash" , "-c" , "\
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
  && chmod +x install_geographiclib_datasets.sh \
  && sudo ./install_geographiclib_datasets.sh" ]

# Copy everything
COPY --chown=$USER_UID:$USER_GID . src/orca4

# Build everything
RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/humble/setup.bash \
  && colcon build" ]

# Set up the environment
WORKDIR /home/$USERNAME/colcon_ws
RUN echo "export PATH=/home/$USERNAME/.local/bin:\$PATH" >> /home/$USERNAME/.bashrc \
  && echo "export PATH=/home/$USERNAME/.local/lib/python3.10/site-packages:\$PATH" >> /home/$USERNAME/.bashrc \
  && echo "source /home/$USERNAME/colcon_ws/src/orca4/setup.bash" >> /home/$USERNAME/.bashrc

# Required to use the --console option on sim_vehicle.py:
# RUN pip3 install matplotlib

USER $USERNAME


#lsusb stuff
RUN sudo apt update && sudo apt install -y usbutils
RUN sudo usermod -aG dialout orca4

USER $USERNAME


# Dillon Adding the ZedX wrapper stuff
# WORKDIR /home/$USERNAME/colcon_ws/src
# RUN git clone https://github.com/stereolabs/zed-ros2-wrapper.git
# RUN sudo apt update && sudo apt install -y ros-humble-ament-cmake-auto ros-humble-ament-cmake
# RUN colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
# WORKDIR /home/$USERNAME/colcon_ws
# RUN sudo apt update
# RUN rosdep install --from-paths src --ignore-src -r -y

# RUN echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
# RUN source ~/.bashrc 
# # may break



