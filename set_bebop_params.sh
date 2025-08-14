#!/usr/bin/env bash

set -e

# Source ROS environment
source /opt/ros/melodic/setup.bash
source /root/bebop_ws/devel/setup.bash

NODE_NAME="/bebop/bebop_driver"

echo "Waiting for the driver node ($NODE_NAME) to be available..."

# Loop until the node is found by rosnode list
until rosnode list | grep -q "$NODE_NAME"; do
  sleep 1
done

echo "Node found. Setting dynamic parameters..."

# Set the parameters using dynamic_reconfigure
rosrun dynamic_reconfigure dynparam set "$NODE_NAME" "{
  'PilotingSettingsMaxAltitudeCurrent': 0.5,
  'PilotingSettingsMaxDistanceValue': 10.0,
  'SpeedSettingsMaxVerticalSpeedCurrent': 0.5
}"

echo "Parameters set successfully."