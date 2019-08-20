<<<<<<< HEAD
gps2ned_ws
======

Transformation of GPS(longitude-latitude-altitude) into NED(North-East-Down) using ROS.

Installation instructions
------

This demo is running on Ubuntu 14.04 with ROS indigo. To run the demo, follow these instructions:

* Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools:
```sh
  $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
  $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  $ sudo apt-get update
  $ sudo apt-get install ros-indigo-desktop-full ros-indigo-catkin python-catkin-tools 
  $ sudo rosdep init
  $ rosdep update
  $ source /opt/ros/indigo/setup.bash
```
* Initialize and compile catkin workspace:
```sh
  $ mkdir -p ~/gps
  $ cd ~/gps
  $ git clone https://github.com/longylian6/gps2ned_ws.git
  $ cd ~/gps/gps2ned_ws
  $ catkin init  # initialize your catkin workspace
  $ catkin_make  # compile your workspace
```
* Run the demo. In seperate terminals run the following commands
```sh
  $ roscore
```
```sh
  $ cd ~/gps/gps2ned_ws
  $ source devel/setup.bash
  $ rosrun gps_tran gps_tran_node
```
=======
# gps2ned_ws
GPS transform into NED.

To run the demo, follow these instructions:
```sh
  $ cd (your directory)/gps2ned_ws
  $ catkin_make
  $ source devel/setup.bash 
  $ rosrun gps_tran gps_tran_node 
```
Thanks for your attention.
>>>>>>> d767a10dea8999b6ccbc00b635472465c861e7fd
