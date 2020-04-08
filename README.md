# FlightGoggles
<br>
Forked from http://flightgoggles.mit.edu. Coded by great friend, Winter-Guerra, when he was in MIT!.

<br> <br>

#### This repo is only for back up of specific commit version("8a0af6c") of flightgoggles
+ Edited Cam topic hz in flightgoggles_ros_bridge's ROSClient.hpp
+ Edited IMU topic hz in uav_dynamics_node.hpp
+ Edited IMU to publish always in flightgoggles_uav_node.cpp
+ Edited initial pose in config/drone/drone.yaml
+ Commented not to launch rviz

<br>

Thanks Winter.

<br>
<br>

~~~shell
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
cd src
wstool init
# Install FlightGoggles nodes and deps from rosinstall file
wstool merge https://raw.githubusercontent.com/mit-fast/FlightGoggles/master/flightgoggles.rosinstall
wstool update
cd ../
# Install required libraries.
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
# Install external libraries for flightgoggles_ros_bridge
sudo apt install -y libzmqpp-dev libeigen3-dev
# Install dependencies for flightgoggles renderer
sudo apt install -y libvulkan1 vulkan-utils gdb
# Build nodes and download FlightGoggles renderer binary
catkin build 
# NOTE: to avoid downloading the FlightGoggles renderer binary, use the following build command:
# catkin build --cmake-args -DFLIGHTGOGGLES_DOWNLOAD_BINARY=OFF
# Refresh workspace
source ~/.bashrc
~~~

