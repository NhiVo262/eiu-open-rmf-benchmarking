FROM osrf/ros:jazzy-desktop@sha256:1d6f898b6ab77636c40f26298070ad3de5a9e06f0a71cf9ab066fd6b7838f151  

LABEL maintainer="EIU - School of Engineering" \
      description="Open-RMF Benchmarking and VDA5050 x Open-RMF Integration Projects" \
      version="1.0" \
      license="MIT"

# Ensure apt directories exist & use apt-get (stable for Docker)
RUN mkdir -p /var/lib/apt/lists/partial
RUN apt-get update && apt-get install -y wget unzip && \
    wget https://github.com/eclipse-zenoh/zenoh/releases/download/1.5.0/zenoh-1.5.0-x86_64-unknown-linux-gnu-standalone.zip -O /tmp/zenoh.zip && \
    unzip /tmp/zenoh.zip -d /tmp/zenoh_extracted && \
    mv /tmp/zenoh_extracted/zenohd /usr/bin/zenohd && \
    chmod +x /usr/bin/zenohd && \
    rm -rf /tmp/zenoh.zip /tmp/zenoh_extracted

# ----------------------------------------------------------
# Install basic packages
# ----------------------------------------------------------
RUN apt-get update && apt-get install -y \
    libmosquitto-dev \
    mosquitto \
    mosquitto-clients \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
    

RUN mkdir -p /var/lib/apt/lists/partial && \
    apt-get update && apt-get install -y \
    build-essential \
    python3-argcomplete \
    python3-colcon-common-extensions \
    libboost-system-dev \
    nano \
    cmake \
    python3 \
    python3-pip \
    git \
    curl \
    lsb-release \
    gnupg \
    wget \
    net-tools \
    iputils-ping \
    iproute2 \
    python3-jinja2 \
    libgnutls28-dev \
    openssl \
    libtiff-dev \
    pybind11-dev \
    python3-yaml \
    python3-ply \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------
# Add Gazebo Harmonic repository
# ----------------------------------------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

# ----------------------------------------------------------
# Install Gazebo + ROS Jazzy + RMF packages
# ----------------------------------------------------------
RUN mkdir -p /var/lib/apt/lists/partial && \
    apt-get update && apt-get install -y \
    ros-jazzy-teleop-twist-keyboard \
    gz-harmonic=1.0.0-1~noble \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-urdf-tutorial \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-controller \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-cartographer \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-nav2-route \
    ros-jazzy-dynamixel-sdk \
    ros-jazzy-hls-lfcd-lds-driver \
    ros-jazzy-ld08-driver \
    ros-jazzy-coin-d4-driver \
    ros-jazzy-camera-ros \
    ros-jazzy-urdf \
    ros-jazzy-xacro \
    ros-jazzy-nav2-amcl \
    ros-jazzy-turtlebot3-teleop \
    ros-jazzy-nav2-map-server \
    ros-jazzy-tf-transformations \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-ros2-control \
    ros-jazzy-rmf-building-map-tools=1.9.2-1noble.20260615.134753 \
    ros-jazzy-rmf-traffic-editor=1.9.2-1noble.20260519.011603 \
    ros-jazzy-rmf-traffic-ros2=2.7.2-1noble.20260615.134742 \
    ros-jazzy-rmf-visualization=2.3.2-1noble.20260615.163557 \
    ros-jazzy-rmf-fleet-adapter=2.7.2-1noble.20260615.165559 \
    ros-jazzy-rmf-robot-sim-gz-plugins=2.3.3-1noble.20260615.151247 \
    ros-jazzy-rmf-building-sim-gz-plugins=2.3.3-1noble.20260615.134737 \
    ros-jazzy-rmf-fleet-adapter-python=2.7.2-1noble.20260615.174059 \
    ros-jazzy-rmf-dev=0.1.0-1noble.20260615.174940 \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------
# Install Python requirements
# ----------------------------------------------------------
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages --no-cache-dir -r /tmp/requirements.txt

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# ----------------------------------------------------------
# Create non-root user
# ----------------------------------------------------------
RUN useradd -m eiu 
WORKDIR /home/eiu
USER eiu
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/eiu/.bashrc
RUN echo "source /home/eiu/rmf_ws/install/setup.bash" >> /home/eiu/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:/home/eiu/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/models" >> /home/eiu/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/eiu/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]