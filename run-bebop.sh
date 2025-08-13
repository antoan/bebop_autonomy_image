#!/usr/bin/env bash
set -e

# Store the current docker context
prev_ctx=$(sudo docker context show)

# Switch to the host engine context
echo "Switching to 'default' docker context..."
sudo docker context use default

# Allow local X11 connections
xhost +local:docker &>/dev/null || true

# Remove the old container if it exists
sudo docker rm -f bebop &>/dev/null || true

echo "Starting bebop container..."
# Run the container with all parameters
sudo docker run -it \
  --gpus all \
  --net host \
  --add-host bebop:127.0.0.1 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/docker_share:/root/share:rw \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e ROS_HOSTNAME=localhost \
  -e ROS_IP=127.0.0.1 \
  --name bebop \
  my/bebop:bionic-rosenv

# Switch back to the previous context
echo "Switching back to '$prev_ctx' docker context..."
sudo docker context use "$prev_ctx"