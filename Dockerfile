FROM wdragon75/bebop_cudagl:bionic-1.0.0
SHELL ["/bin/bash","-lc"]

# Persist ROS env + workspace sourcing for root
RUN touch /root/.bashrc && \
    printf '\n# --- ROS env ---\n' >> /root/.bashrc && \
    echo '[ -f /opt/ros/melodic/setup.bash ] && source /opt/ros/melodic/setup.bash' >> /root/.bashrc && \
    echo '[ -f /root/bebop_ws/devel/setup.bash ] && source /root/bebop_ws/devel/setup.bash' >> /root/.bashrc && \
    echo '[ -f ~/.bashrc ] && . ~/.bashrc' >> /root/.bash_profile

# Install tmux
RUN apt-get update && apt-get install -y tmux

# Default ROS env (can be overridden at `docker run -e`)
ENV ROS_MASTER_URI=http://localhost:11311 \
    ROS_HOSTNAME=localhost \
    ROS_IP=127.0.0.1
