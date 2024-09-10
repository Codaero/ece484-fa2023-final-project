# Autonomous GEM Car

Final project for ECE 484 at UIUC. This code is meant to run on a GEM E2 autonomous electric vehicle. The goal of the project is to autonomously control the vehicle to make a lap around the UIUC Highbay facility's testing track.

[Final Video](https://www.youtube.com/watch?v=DdN4_fp8pCI&ab_channel=RobertAzarcon)

## Contributors

Robert Azarcon, Eric Roth, Paul Osuma, Ved Eti

## Environment Setup w/Docker
### Requirements:

 - Ubuntu 22.04/20.04 LTS
 - Latest Nvidia GPU drivers and CUDA
 - [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) (not Docker Desktop)
 - [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
 - vcstool (sudo apt-get install python3-vcstool)

### Installation Steps:

Create the folder hierarchy:
```
mkdir -p ~/ece484_home/catkin_ws/src
```

Clone the repository and necessary dependencies:
```
cd ~/ece484_home/catkin_ws/src
git clone https://gitlab.engr.illinois.edu/vedeti2/ece484_final_project.git
cd ece484_final_project
vcs import .. < workspace.repos
```

Build the docker image and run a container:
```
cd docker
./build.sh
sudo ./run.sh
```
### Environment Usage:
There are several volume mappings specified in the run script that give the spawned Docker container access to your display and devices. The catkin_ws in the Docker container is mapped to the same catkin_ws directory you created on your host computer during installation. Additionally, a .cache folder is also mapped to save the cache generated from PyTorch. All changes made in either catkin_ws (Docker container or host) will persist between each other due to the volume mapping.

To open a new terminal connected to the Docker container run:
```
docker exec -it ece484 /bin/bash
```
**It is highly recommended to create a bash alias in your host computer's .bashrc file for this command.



> Written with [StackEdit](https://stackedit.io/).
