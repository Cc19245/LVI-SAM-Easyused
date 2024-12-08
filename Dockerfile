FROM osrf/ros:melodic-desktop-full

# Update
ARG DEBIAN_FRONTEND=noninteractive

RUN sudo apt-get update -y && sudo apt-get upgrade -y

# Install Curl
RUN sudo apt install -y curl

# Install Zsh
RUN sudo apt install -y zsh
RUN chsh -s $(which zsh)

# Install Oh-My-Zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Installing Dependencies
RUN sudo apt-get install -y ros-melodic-catkin python3-catkin-tools 

# # Upgrading CMake
# RUN sudo apt-get install -y software-properties-common
# RUN sudo add-apt-repository ppa:george-edison55/cmake-3.x
# RUN sudo apt-get upgrade -y

# Install PCL & Eigen & essential libraries
RUN apt-get update && apt-get install -y wget cmake libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev

RUN mkdir /software_libs

# Install Ceres
WORKDIR /software_libs
# Install ceres-solver
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz
RUN tar zxf 1.14.0.tar.gz
WORKDIR /software_libs/ceres-solver-1.14.0
RUN mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release ../../ceres-solver-1.14.0 && make -j2 && sudo make install

RUN sudo apt-get install -y zip unzip

# Install GTSAM
WORKDIR /software_libs
RUN wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
RUN unzip gtsam.zip
WORKDIR /software_libs/gtsam-4.0.2
RUN mkdir build && cd build && cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. && make -j2 && sudo make install

RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> /root/.zshrc 

# Sourcing ROS
RUN echo "source /opt/ros/melodic/setup.zsh" >> /root/.zshrc

WORKDIR /
CMD ["/bin/zsh"]