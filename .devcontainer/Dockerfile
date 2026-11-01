FROM osrf/ros:jazzy-desktop

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG UUCP_GID=986

# Create user
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Add uucp group with host GID
RUN sudo groupadd -g $UUCP_GID serial || true
RUN sudo usermod -aG serial $USERNAME
RUN sudo usermod -aG uucp $USERNAME

# Base tools + Python (critical for PlatformIO)
# me cago en ubuntu y en la mar en coche tio como que tengo que installar iproute
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    cmake \
    vim \
    iproute2 \ 
    # net-utils \ jope tio no funciona nada me cago en la leche
    build-essential \
    usbutils \
    python3 \
    python3-pip \
    python3-venv \
    python-is-python3 \
    curl \
    nano \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# X11 sobre VNC
RUN apt-get update && \
    apt-get install --no-install-suggests --no-install-recommends -y \
    tigervnc-standalone-server \
    tigervnc-tools \
    icewm \
    xterm \
    menu \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:99

EXPOSE 5299

# Sudo config
RUN echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

# ROS packages
RUN sudo apt-get update && \
    sudo apt-get install -y \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-teleop-twist-joy \
        ros-${ROS_DISTRO}-tf2-tools \
        ros-${ROS_DISTRO}-cartographer-ros \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-rmw-zenoh-cpp && \
    sudo rm -rf /var/lib/apt/lists/*

EXPOSE 7447

# PlatformIO
RUN pip install platformio --break-system-packages && \
    rm -rf ~/.cache/pip/

ENV PATH="/home/$USERNAME/.local/bin:${PATH}"
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
# ENV RUST_LOG=debug
ENV RUST_LOG=info
ENV ROS_DOMAIN_ID=123

# ROS env
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

WORKDIR /home/$USERNAME

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
