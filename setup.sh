#!/bin/sh

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install libgflags-dev libeigen3-dev libopencv-dev libyaml-cpp-dev ros-humble-ros-base ros-humble-cv-bridge

mkdir install && cd install || exit
wget -O gtest.tar.gz https://github.com/google/googletest/archive/refs/tags/release-1.12.1.tar.gz
wget -O glog.tar.gz https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz

tar -xzf gtest.tar.gz
tar -xzf glog.tar.gz

mkdir gtest
mkdir glog

tar -xzf gtest.tar.gz -C gtest --strip-components=1
tar -xzf glog.tar.gz -C glog --strip-components=1

mkdir gtest/build
cmake -S gtest -B gtest/build
sudo make install -C gtest/build -j$(nproc)

mkdir glog/build
cmake -S glog -B glog/build
sudo make install -C glog/build -j$(nproc)
