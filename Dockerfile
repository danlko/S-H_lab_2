## ==========================================================================
## Dockerfile — ArduPilot SITL + ROS 2 Humble + Gazebo Harmonic
## ==========================================================================
FROM osrf/ros:humble-desktop

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

# 1. Базові інструменти збірки
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo curl wget gnupg lsb-release git build-essential cmake \
        software-properties-common \
        python3-pip python3-vcstool python3-rosdep python3-colcon-common-extensions \
        default-jre mesa-utils libgl1-mesa-dri \
    && add-apt-repository -y universe

# 2. Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends gz-harmonic

ENV GZ_VERSION=harmonic

# 3. Користувач без root
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

# 4. Завантаження вихідного коду
RUN mkdir -p src \
    && wget -q -O ros2.repos \
        https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos \
    && vcs import --recursive src < ros2.repos \
    && wget -q -O ros2_gz.repos \
        https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos \
    && vcs import --recursive src < ros2_gz.repos
RUN pip3 install --user -U empy==3.3.4 future pexpect pyserial MAVProxy

# 5. Системні залежності ROS-пакетів
RUN sudo rosdep init || true \
    && sudo bash -c 'wget -q https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list' \
    && rosdep update \
    && sudo apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y

# 6. Micro-XRCE-DDS-Gen
RUN git clone --recurse-submodules --branch v4.7.0 \
        https://github.com/ArduPilot/Micro-XRCE-DDS-Gen.git \
    && cd Micro-XRCE-DDS-Gen && ./gradlew assemble

ENV PATH="/home/${USERNAME}/ros2_ws/Micro-XRCE-DDS-Gen/scripts:${PATH}"

# 7. Збірка робочого простору
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    MAKEFLAGS='-j2' colcon build --symlink-install --packages-up-to ardupilot_gz_bringup"

# ВАЖЛИВЕ ВИПРАВЛЕННЯ: налаштування .bashrc із розкриттям змінних і шляхами
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "export PATH=\$PATH:/home/${USERNAME}/.local/bin" >> /home/${USERNAME}/.bashrc \
    && echo "export GZ_SIM_RESOURCE_PATH=/home/${USERNAME}/ros2_ws/install/ardupilot_gazebo/share" >> /home/${USERNAME}/.bashrc

# 8. Entrypoint
COPY --chown=${USERNAME}:${USERNAME} entrypoint.sh /home/${USERNAME}/entrypoint.sh
RUN sudo chmod +x /home/${USERNAME}/entrypoint.sh

ENTRYPOINT ["/home/ros/entrypoint.sh"]
CMD ["bash"]
