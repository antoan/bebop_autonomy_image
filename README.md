# Bebop Autonomy Docker Environment

This project provides a reproducible Docker environment for controlling a Parrot Bebop drone with ROS. It is designed to be launched with a single command, providing a pre-configured `tmux` session with all necessary components ready to run.

## Overview

The setup consists of three main components:

1.  **`Dockerfile`**: Defines and builds the custom Docker image.
2.  **`set_bebop_params.sh`**: A script inside the container to configure the drone's flight parameters.
3.  **`run-bebop.sh`**: A host-side script that launches the Docker container and sets up the `tmux` development environment.

An alias `bebop` is also added to your `~/.zshrc` to make launching the environment easy.

### How It Works

1.  The **`Dockerfile`** starts from a base image (`wdragon75/bebop_cudagl:bionic-1.0.0`) that contains ROS Melodic and NVIDIA GPU support. It then:
    *   Installs `tmux` for terminal multiplexing.
    *   Copies the `set_bebop_params.sh` script into the image at `/usr/local/bin/` and makes it executable.
    *   Sets up the necessary ROS environment variables and sources the `bebop_ws` workspace.

2.  The **`set_bebop_params.sh`** script is designed to be run inside the container. It:
    *   Sources the ROS environment to ensure it has access to ROS commands.
    *   Waits for the `/bebop/bebop_node` to become available.
    *   Uses `dynamic_reconfigure` to set the drone's maximum altitude, maximum distance, and maximum vertical speed.

3.  The **`run-bebop.sh`** script orchestrates the entire launch process from your host machine. It:
    *   Switches the Docker context to `default` to use the native Docker engine (required for stable GPU access).
    *   Ensures the container can connect to your host's X server for GUI applications.
    *   Removes any previously running container named `bebop` to avoid conflicts.
    *   Runs the custom Docker image (`my/bebop:bionic-rosenv`).
    *   Executes a series of `tmux` commands to create a 5-pane layout inside the container.
    *   Switches the Docker context back to what it was before the script was run.

### The `tmux` Workflow

When you run the `bebop` alias, you are dropped into a `tmux` session with five panes, each with a command pre-typed and ready to be executed by pressing `Enter`.

The layout is as follows:

*   **Pane 0 (Top-Left):** `roscore`
    *   **Action:** Press `Enter` here first to start the ROS master.
*   **Pane 1 (Top-Right):** `roslaunch bebop_driver bebop_node.launch`
    *   **Action:** After `roscore` is running, switch to this pane (`Ctrl-b` + `→`) and press `Enter` to start the drone driver.
*   **Pane 2 (Middle-Left):** `set_bebop_params.sh`
    *   **Action:** After the driver is running, switch to this pane (`Ctrl-b` + `↓` from pane 0) and press `Enter` to set the flight parameters.
*   **Pane 3 (Bottom-Left):** `rostopic pub -1 /bebop/takeoff std_msgs/Empty "{}"`
    *   **Action:** Use this pane to send the takeoff command.
*   **Pane 4 (Bottom-Right):** `rostopic pub --once /bebop/land std_msgs/Empty`
    *   **Action:** Use this pane to send the land command.

### Usage

1.  **Build the Docker Image:**
    ```bash
    sudo docker build -t my/bebop:bionic-rosenv .
    ```

2.  **Launch the Environment:**
    Open a new terminal and run:
    ```bash
    bebop
    ```
    (If the alias is not found, you may need to run `source ~/.zshrc` first).

3.  **Follow the `tmux` workflow** described above to start the components in the correct order.