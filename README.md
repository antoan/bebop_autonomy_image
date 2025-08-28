# Bebop Autonomy Docker Environment

This project provides a reproducible Docker environment for controlling a Parrot Bebop drone with ROS. It is designed to be launched with a single command, providing a pre-configured `tmux` session with all necessary components ready to run.

## Overview

The system is designed to provide a simple, reliable way to launch the Bebop drone's ROS driver in a containerized environment. It also includes the `rosbridge_server` to allow for communication between ROS 1 and ROS 2 systems. This is primarily intended to bridge a ROS 2 `cmd_vel` publisher on the host machine to the ROS 1 `cmd_vel` subscriber in the container.

The system consists of two main components:

1.  **`Dockerfile.new`**: A file that defines how to build a custom Docker image containing all the necessary software (ROS, the Bebop driver, `rosbridge_server`, and all dependencies).
2.  **`launch.sh`**: A script on your host machine that automates the process of building the image (if needed) and running the container with a user-friendly `tmux` interface.
### Project Structure: Overriding Launch Files
 
This project uses a specific structure to manage custom configurations for the `bebop_autonomy` package without modifying the original source code. The `main.launch` file is sourced from an adjacent ROS workspace located at `../bebop_autonomy`.
 
- **`Dockerfile.new`:** During the Docker build process, the `main.launch` file from `../bebop_autonomy/bebop_driver/launch/` is copied into the container, overwriting the default file from the cloned repository.
 
This allows you to add or modify nodes (like the `web_video_server`) in a central location without altering this project's repository.
 
### 1. The Dockerfile (`Dockerfile.new`)


This file is the blueprint for our environment. When built, it creates a self-contained image that includes:

*   **Base System:** Starts from an official NVIDIA CUDA image with Ubuntu 18.04, providing GPU support.
*   **ROS Melodic:** Installs the `desktop-full` version of ROS Melodic.
*   **Bebop Autonomy Package:** Clones your fork of the `bebop_autonomy` repository from GitHub. This is where you have customized the flight parameters in the `defaults.yaml` file.
*   **rosbridge_server:** Installs the `ros-melodic-rosbridge-server` package to allow for ROS 1 to ROS 2 communication.
*   **Dependencies:** Installs all necessary system libraries, including the `parrot_arsdk` which is required for the driver to communicate with the drone.
*   **Workspace Build:** Compiles the `bebop_autonomy` ROS package using `catkin build`.
*   **Environment Configuration:** Modifies the `.bashrc` file inside the image to automatically:
    *   Source the main ROS Melodic setup script.
    *   Source the setup script for our compiled `bebop_ws` workspace.
    *   **Crucially, it sets the `LD_LIBRARY_PATH`** to include the Parrot ARSDK libraries, which fixes the `libarcommands.so` error.

### 2. The Launch Script (`launch.sh`)

This script is the user-friendly entry point for running the system. When you execute `./launch.sh`, it performs the following steps:

1.  **Checks for Image:** It first checks if the `my/bebop:bionic-rosenv` Docker image has already been built.
2.  **Builds Image (if needed):** If the image is not found, it automatically runs `docker build` using `Dockerfile.new` to create it. This ensures you are always running an up-to-date version of your environment after you make changes.
3.  **Sets up Docker:** It ensures it's using the native Docker engine (not Docker Desktop) for better performance and hardware access.
4.  **Launches Container:** It runs the Docker container with all the necessary flags:
    *   `--gpus all`: Provides access to the NVIDIA GPU.
    *   `--net=host`: Allows the container to share the host's network, which is essential for ROS communication.
    *   `-p 9090:9090`: Exposes the `rosbridge_server` websocket port to the host machine.
    *   `-e DISPLAY` and `-v /tmp/.X11-unix...`: Forwards your display so you can run GUI applications from the container.
5.  **Starts `tmux` Session:** Instead of just giving you a single shell, it starts a `tmux` session with a pre-configured 4-pane layout:
    *   **Top Pane:** Pre-types the `roslaunch bebop_driver bebop_node.launch` command for you.
    *   **Bottom-Left Pane:** Pre-types the `takeoff` command for you.
    *   **Bottom-Right Pane:** Pre-types the `land` command for you.
    *   **Far-Right Pane:** Automatically runs `roslaunch rosbridge_server rosbridge_websocket.launch`.

### Usage

1.  **Build and Launch:**
    Run the launch script from your terminal:
    ```bash
    ./launch.sh
    ```
    The first time you run this, it will build the Docker image, which will take several minutes. Subsequent launches will be much faster.

    To force a rebuild of the image without using the Docker cache, you can pass the `--no-cache` flag:
    ```bash
    ./launch.sh --no-cache
    ```

2.  **Inside the Container:**
    You will be placed in a 4-pane `tmux` session. The far-right pane will be running the `rosbridge_server`. You can switch to the other panes to run the driver, takeoff, and land commands. The `main.launch` file, which is used to launch the `rosbridge_server`, is located in the `bebop_driver` package.