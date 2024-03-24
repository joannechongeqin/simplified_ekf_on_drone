EE4308_WS=$HOME/ee4308
# cd $EE4308_WS
# make sure you are in the folder of this .sh file before running it.

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30 #TURTLEBOT3
source /usr/share/gazebo/setup.sh
. ~/sjtu_drone/install/setup.bash

export EE4308_TASK=proj2