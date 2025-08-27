# Bebop Autonomy Docker Environment

This project provides a reproducible Docker environment for controlling a Parrot Bebop drone with ROS. It is designed to be launched with a single command, providing a pre-configured `tmux` session with all necessary components ready to run.

## Overview

The system is designed to provide a simple, reliable way to launch the Bebop drone's ROS driver in a containerized environment. It consists of two main components:

1.  **`Dockerfile.new`**: A file that defines how to build a custom Docker image containing all the necessary software (ROS, the Bebop driver, and all dependencies).
2.  **`launch.sh`**: A script on your host machine that automates the process of building the image (if needed) and running the container with a user-friendly `tmux` interface.

### 1. The Dockerfile (`Dockerfile.new`)

This file is the blueprint for our environment. When built, it creates a self-contained image that includes:

*   **Base System:** Starts from an official NVIDIA CUDA image with Ubuntu 18.04, providing GPU support.
*   **ROS Melodic:** Installs the `desktop-full` version of ROS Melodic.
*   **Bebop Autonomy Package:** Clones your fork of the `bebop_autonomy` repository from GitHub. This is where you have customized the flight parameters in the `defaults.yaml` file.
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
    *   `-e DISPLAY` and `-v /tmp/.X11-unix...`: Forwards your display so you can run GUI applications from the container.
5.  **Starts `tmux` Session:** Instead of just giving you a single shell, it starts a `tmux` session with a pre-configured 3-pane layout:
    *   **Top Pane:** Automatically runs `roslaunch bebop_driver bebop_node.launch`. This command starts the ROS master (`roscore`) and the main driver node.
    *   **Bottom-Left Pane:** Pre-types the `takeoff` command for you.
    *   **Bottom-Right Pane:** Pre-types the `land` command for you.

### Usage

1.  **Build and Launch:**
    Run the launch script from your terminal:
    ```bash
    ./launch.sh
    ```
    The first time you run this, it will build the Docker image, which will take several minutes. Subsequent launches will be much faster.

2.  **Inside the Container:**
    You will be placed in a 3-pane `tmux` session. The top pane will be running the driver. You can switch to the bottom panes (`Ctrl-b` + `↓`) to run the takeoff and land commands.