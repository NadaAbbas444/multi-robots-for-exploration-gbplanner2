# Full ROS environment for the icuas2025-larics container, usable from
# non-interactive shells too (/root/.bashrc returns early when not interactive).
# Usage:  source /ws/sim/icuas2025_larics/env.sh
source /opt/ros/noetic/setup.bash
source /root/uav_ros_simulation/ros_packages/uav_ros_stack/miscellaneous/shell_additions/shell_scripts.sh
[ -f ~/.ardupilot_env ] && source ~/.ardupilot_env
export PATH=/root/.local/bin:/usr/lib/ccache:$PATH:/root/uav_ros_simulation/firmware/ardupilot/Tools/autotest
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/root/sim_ws/build/ardupilot_gazebo
source /root/uav_ws/install/setup.bash
source /root/sim_ws/devel/setup.bash
source /root/extra_ws/devel/setup.bash
[ -f /ws/devel_larics/setup.bash ] && source /ws/devel_larics/setup.bash
# uav_ws is an install space that the devel chains drop; re-add it.
export ROS_PACKAGE_PATH=/root/uav_ws/install/share:$ROS_PACKAGE_PATH
