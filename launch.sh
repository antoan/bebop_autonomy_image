#!/usr/bin/env bash
set -e

# Handle --no-cache argument
BUILD_ARGS=""
# Define the build context and Dockerfile path
BUILD_CONTEXT=".."
DOCKERFILE_PATH="bebop_autonomy_image/Dockerfile.new"

if [ "$1" == "--no-cache" ]; then
  echo "Force rebuilding the Docker image with --no-cache..."
  (cd "$BUILD_CONTEXT" && sudo docker build --no-cache -t my/bebop:bionic-rosenv -f "$DOCKERFILE_PATH" .)
# Build the new Docker image if it doesn't exist
elif [[ "$(sudo docker images -q my/bebop:bionic-rosenv 2> /dev/null)" == "" ]]; then
  echo "Building new Docker image..."
  (cd "$BUILD_CONTEXT" && sudo docker build -t my/bebop:bionic-rosenv -f "$DOCKERFILE_PATH" .)
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
  -p 9090:9090 \
  --name bebop \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/docker_share:/root/share:rw \
  my/bebop:bionic-rosenv \
  tmux new-session -s bebop -d \; \
    send-keys -t 0 'roslaunch bebop_driver bebop_node.launch' \; \
    split-window -v \; \
    send-keys -t 1 'rostopic pub -1 /bebop/takeoff std_msgs/Empty "{}"' \; \
    split-window -v \; \
    send-keys -t 2 'rostopic pub --once /bebop/land std_msgs/Empty' \; \
    split-window -h \; \
    send-keys -t 3 'roslaunch rosbridge_server rosbridge_websocket.launch' C-m \; \
    select-pane -t 0 \; \
    attach

# Switch back to the previous context
echo "Switching back to '$prev_ctx' docker context..."
sudo docker context use "$prev_ctx"