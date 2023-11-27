FROM osrf/ros:noetic-desktop-full AS base

ARG USERNAME=ece484
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
    nano \
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

# Install Python Requirements:
RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        python3-dev \
        python3-tk \
        libglib2.0-0

COPY requirements.txt requirements.txt

RUN python3 -m pip install --upgrade pip

RUN --mount=type=cache,target=/root/.cache/pip pip3 install -r ./requirements.txt
# RUN python3 -m pip install -r requirements.txt

# RUN pip3 install torch torchvision torchaudio -f https://download.pytorch.org/whl/cu111/torch_stable.html

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo "\n# Added by ece484 Dockerfile:" >> /home/$USERNAME/.bashrc \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to our new user
USER $USERNAME
ENV USER=$USERNAME

WORKDIR /home/$USERNAME/catkin_ws

RUN mkdir -p /home/$USERNAME/catkin_ws/src

RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/noetic/setup.bash" ]

  
RUN echo "export PATH=/home/$USERNAME/.local/bin:\$PATH" >> /home/$USERNAME/.bashrc \
&& echo "export PATH=/home/$USERNAME/.local/lib/python3.10/site-packages:\$PATH" >> /home/$USERNAME/.bashrc && echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc && echo "source /home/$USERNAME/catkin_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc


# RUN [ "/bin/bash" , "-c" , "\
#  sudo apt-get update && sudo apt-get install -y ros-noetic-ackermann-msgs"]
  
RUN sudo apt-get update \
    && sudo apt-get -y install \
    ros-noetic-ackermann-msgs \
    ros-noetic-geographic-msgs \ 
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-rtabmap-ros
