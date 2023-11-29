ARG CARLA_VERSION
FROM carlasim/carla:0.9.13 as carla_api

FROM ros:galactic-ros-base
RUN rm /etc/apt/sources.list.d/ros2*
RUN apt update && apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

SHELL [ "/bin/bash", "-c" ]
ARG CARLA_VERSION

# Set up apt repo
RUN apt-get update
RUN apt-get install -y lsb-release software-properties-common python-is-python3 iputils-ping
RUN apt-get update

# Install build system
RUN apt-get update && apt-get install -y build-essential wget curl python3-rosdep

# Install Carla Python API
RUN apt-get install -y python3-pip python3-pygame
COPY --from=carla_api --chown=root /home/carla/PythonAPI /opt/carla/PythonAPI
WORKDIR /opt/carla/PythonAPI
RUN python3 -m pip install --upgrade pip
RUN if [[ "$CARLA_VERSION" > "0.9.11" ]]; then \
    python3 -m pip install "carla==$CARLA_VERSION"; \
    fi

# Add a docker user so we that created files in the docker container are owned by a non-root user
RUN addgroup --gid 1000 docker && \
    adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
    echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# Remap the docker user and group to be the same uid and group as the host user.
# Any created files by the docker container will be owned by the host user.
RUN USER=docker && \
    GROUP=docker && \
    curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\npaths:\n  - /home/docker/" > /etc/fixuid/config.yml
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

USER docker:docker

ENV PYTHONPATH "${PYTHONPATH}:/opt/carla/PythonAPI/carla/dist/carla-${CARLA_VERSION}-py3.7-linux-x86_64.egg"
RUN if [ "${CARLA_VERSION}" = "0.9.10.1" ]; then \
      echo "export PYTHONPATH=$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg" >> ~/.bashrc ; \
    fi
ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
# Set up workspace
RUN mkdir -p /home/docker/carla-ros-bridge && cd /home/docker/carla-ros-bridge
WORKDIR /home/docker/carla-ros-bridge

RUN sudo apt-get install -y git
RUN git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge

# Download source dependencies
RUN sudo rm /etc/ros/rosdep/sources.list.d/20-default.list || true && \
    sudo rosdep init && \
    sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y

# Fix for missing rviz and pcl_ros
RUN pip install carla==0.9.13 scipy networkx mxnet-mkl==1.6.0 numpy==1.23.1
RUN sudo apt-get update
RUN sudo apt-get install -y ros-galactic-rviz2 openni2-utils libpcap-dev pcl-tools python3-opencv ros-galactic-pcl-conversions ros-galactic-cv-bridge ros-galactic-plotjuggler-ros ros-galactic-rqt-graph
WORKDIR /home/docker/carla-ros-bridge/src
RUN git clone https://github.com/astuff/astuff_sensor_msgs.git
WORKDIR /home/docker/carla-ros-bridge
RUN source /opt/ros/galactic/setup.bash && colcon build

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/carla-ros-bridge/devel/setup.bash
COPY ./ros2_entrypoint.sh /home/docker/ros2_entrypoint.sh
RUN sudo chmod +x ~/ros2_entrypoint.sh
ENTRYPOINT ["/home/docker/ros2_entrypoint.sh"]

# ================= Develop ===================
USER root:root
# Supervisor to manage multiple processes (code-server, roslaunch)
RUN apt-get install -y wget curl gdb supervisor

COPY --chown=docker ./carla_ros2_bridge/supervisord.conf /etc/supervisor/supervisord.conf
RUN chown -R docker:docker /etc/supervisor
RUN chmod 777 /var/log/supervisor/

USER docker:docker
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
RUN echo "source /home/docker/carla-ros-bridge/install/setup.sh" >> ~/.bashrc

CMD ["/usr/bin/supervisord"]