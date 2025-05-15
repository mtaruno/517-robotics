source /opt/ros/noetic/setup.bash
source ~/fetch_ws/devel/setup.bash

# ROS Network Setup
export ROS_HOSTNAME=matthew-Legion-5-16IRX9
export ROS_IP=`hostname -I | awk '{print $1}'`
export ROS_MASTER_URI=http://matthew-Legion-5-16IRX9:11311