#!/bin/sh
sleep 3
ifconfig eth0 192.168.1.1

#route velodyne ip (192.168.1.201)
sleep 3
ap-hotspot start

sleep 2
#insmod -f ~/ThermalCamera/v4l2loopback/v4l2loopback.ko
modprobe v4l2loopback
/home/groundstation1/catkin_ws/src/unmanned-research/ThermalCamera/ThermAppCam/thermapp/thermapp

#launch ros nodes
#bag command
# rosbag record -O groundstation1test1 --split --duration=1m compress --output-dir=/home/groundstation1/bagfiles *.bag --node=/ground_station_rgb_node /ground_station_therm_node /velodyne_points /mavros/global_position/raw/fixy
