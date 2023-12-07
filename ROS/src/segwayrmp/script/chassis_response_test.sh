#!/usr/bin/env bash
# usage : sh ***.sh
# 
SCRIPT_DIR=$(dirname $(readlink -f $0))
WS_DIR=${SCRIPT_DIR%/*/*/*/*}

gnome-terminal --working-directory=$WS_DIR  \
--window -e 'bash -c "echo "1" | sudo -S chmod 777 -R /dev/ttyUSB0; exec bash"'   \
--tab --active -e 'bash -c "roscore; exec bash"' \
--tab --active -e 'bash -c "sleep 1; source /opt/ros/melodic/setup.bash; source devel/setup.sh; rosrun segwayrmp SmartCar; exec bash"' \
--tab --active -e 'bash -c "sleep 2; source /opt/ros/melodic/setup.bash; source devel/setup.sh; rosrun segwayrmp ChassisResponseTest; exec bash"'
