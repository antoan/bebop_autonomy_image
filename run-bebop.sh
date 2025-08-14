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
  my/bebop:bionic-rosenv \
  bash -c "tmux new-session -s bebop -d \; \
    split-window -h -p 50 \; \
    select-pane -t 0 \; \
    split-window -v -p 66 \; \
    split-window -v -p 50 \; \
    select-pane -t 1 \; \
    split-window -v -p 50 \; \
    send-keys -t 0 'roscore' \; \
    send-keys -t 1 'roslaunch bebop_driver bebop_node.launch' \; \
    send-keys -t 2 'set_bebop_params.sh' \; \
    send-keys -t 3 'rostopic pub -1 /bebop/takeoff std_msgs/Empty \"{}\"' \; \
    send-keys -t 4 'rostopic pub --once /bebop/land std_msgs/Empty' \; \
    select-pane -t 0 \; \
    attach"

# Switch back to the previous context
echo "Switching back to '$prev_ctx' docker context..."
sudo docker context use "$prev_ctx"