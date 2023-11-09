# Autonomous GEM Car

## Environment Setup w/Docker
### Requirements:

 - Ubuntu 22.04/20.04 LTS
 - Latest Nvidia GPU drivers and CUDA
 - [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) (not Docker Desktop)
 - [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

### Installation Steps:

Create the folder hierarchy:
```
mkdir -p ~/ece484_home/catkin_ws/src
```

Clone the repository and necessary dependencies:
```
cd ~/ece484_home/catkin_ws/src
git clone https://gitlab.engr.illinois.edu/vedeti2/ece484_final_project.git
```

Build the docker image and run a container:
```
cd ece484_final_project/docker
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