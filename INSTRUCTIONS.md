# Instructions for Bridging Bebop Drone Images to ROS2

This document outlines the steps required to build the Docker container, run the ROS1 Bebop driver, and bridge the camera feed to your ROS2 host system using an MJPEG stream.

## Step 1: Build the Docker Image

First, build the Docker image using the provided `Dockerfile.new`. This image contains the ROS1 Melodic environment, the `bebop_autonomy` driver, and the `web_video_server` needed to create the MJPEG stream.

Open a terminal in the project root and run:

```bash
docker build -t bebop_ros1_image -f Dockerfile.new .
```

## Step 2: Run the Bebop Container

Next, launch the container. It's crucial to use `--net=host` to ensure that the MJPEG server's port (8088) is directly accessible from your host machine.

```bash
docker run -it --rm --net=host --privileged bebop_ros1_image
```

Inside the container, launch the Bebop driver. This will also start the `web_video_server`.

```bash
# Inside the container
roslaunch bebop_driver main.launch
```

At this point, the drone's camera feed should be available as an MJPEG stream at `http://localhost:8088/stream?topic=/bebop/image_raw`.

## Step 3: Build and Run the ROS2 Bridge Node

The ROS2 bridge node is located in the `bebop2_ros2` workspace. You will need to build this workspace and then run the node.

1.  **Install Dependencies:**
    On your host machine (Kubuntu 24.04 with ROS2 Jazzy), you need to install the Python dependencies for the bridge node.
    ```bash
    sudo apt update
    sudo apt install -y python3-opencv ros-jazzy-rclpy ros-jazzy-sensor-msgs
    ```

2.  **Build the Workspace:**
    Navigate to the `bebop2_ros2` directory and build it using `colcon`.
    ```bash
    cd ../bebop2_ros2
    colcon build
    ```

3.  **Run the Node:**
    After the build is complete, source the workspace and run the bridge node using `ros2 run`.
    ```bash
    source install/setup.bash
    ros2 run bebop_image_bridge mjpeg_bridge
    ```

## Step 4: Verify the ROS2 Image Topic

To confirm that the bridge is working, you can check the ROS2 topic on your host machine.

Open a third terminal and run:

```bash
ros2 topic hz /bebop2/image/compressed
```

You should see a steady stream of messages being published, indicating that the image bridge is running successfully. You can also use `rviz2` to visualize the compressed image topic.
