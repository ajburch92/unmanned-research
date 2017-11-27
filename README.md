## unmanned-research

## deployment procedure
# bashrc

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash

#MOVED TO /etc/hosts
export ROS_IP=192.168.150.1
export ROS_HOSTNAME=192.168.150.1
export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311

alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'
alias cm='cd ~/catkin_ws && catkin_make'
alias ct='cd ~/ThermalCamera/ThermAppCam/thermapp'
alias cv='cd ~/catkin_ws/src/unmanned-research/virtual_infrastructure_pkg'

# startup script

# iea package

# opencv install

# ros install

# dependency install

# boot
/etc/rc.local

sh /home/user/startup_script.sh

# rc car control pkg

