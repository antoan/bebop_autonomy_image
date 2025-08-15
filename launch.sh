#!/usr/bin/env bash
set -e

# Build the new Docker image if it doesn't exist
if [[ "$(sudo docker images -q bebop_autonomy:latest 2> /dev/null)" == "" ]]; then
  echo "Building new Docker image..."
  sudo docker build -t bebop_autonomy:latest -f Dockerfile.new .
fi

# Switch to the host engine context
prev_ctx=$(sudo docker context show)
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
  --name bebop \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/docker_share:/root/share:rw \
  bebop_autonomy:latest

# Switch back to the previous context
echo "Switching back to '$prev_ctx' docker context..."
sudo docker context use "$prev_ctx"