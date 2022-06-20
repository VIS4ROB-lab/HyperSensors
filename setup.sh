#!/bin/sh

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install libgflags-dev libeigen3-dev libopencv-dev libyaml-cpp-dev ros-noetic-ros-base ros-noetic-cv-bridge

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
