# ROS2 basics in C++ (Software Development)


[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
This project provides beginner-level tutorials for ROS 2 Humble, featuring a simple publisher-subscriber setup to demonstrate core ROS 2 functionalities. It includes a Talker node that publishes messages to a topic and a Listener node that subscribes to that topic. Additionally, a service in the Talker node allows dynamic modification of the published message, while comprehensive logging across all five ROS 2 logging levels (DEBUG, INFO, WARN, ERROR, FATAL) provides detailed output. A launch file enables users to configure parameters, such as publish frequency, through command-line arguments, making this package a practical introduction to ROS 2 basics.

## Author 
Swaraj Mundruppady Rao (swarajmr@umd.edu)

## Prerequisites
- ROS2 Humble installed
- Colcon build tool
- C++17 compatible compiler

## Installation and building the setup

1. **Setup the folder structure**
    ```bash 
    cd 
    mkdir -p ros2_ws/src/beginner_tutorials 
    cd ~/ros2_ws/src/beginner_tutorials 
    ```

2. **Download the Source Code from the Release(either zip or tar.gz)**
    Unzip the file and paste the contents of the folder into this directory - `ros2_ws/src/beginner_tutorials`

3. **Check for Missing Dependencies Before Building**

    To run rosdep in the root of the workspace: 

    ```bash
    cd ~/ros2_ws 
    rosdep install -i --from-path src --rosdistro humble -y
    ```

4. **Build the package**
    Use colcon to build the package 

    ```bash
    colcon build --packages-select beginner_tutorials
    ```

5. **Source the setup**

    Source the script setup to overlay this workspace on the environment 
    ```bash
    source install/setup.bash
    ```

## Using the package / Running the Package 

1. **Running the publisher node** 

    To run the talker node, run the following command 

    ```bash
    ros2 run beginner_tutorials talker --ros-args --log-level debug
    ```

2. **Running the subscriber node**

    To run the subscriber node, run the following command 
    ```bash
    #Open a new terminal and source the setup in this terminal
    ros2 run beginner_tutorials listener --ros-args --log-level debug
    ```
3. **Running the server client node**

    To run the server_client node with a custom output, use the following command:

    ```bash
    #Open a new terminal and source the setup in this terminal (Here "Changed Ouput" is th string that is being changed, customise this if required)
    ros2 run beginner_tutorials server_client "Changed Output" --ros-args --log-level debug
    ```

## Run the same with launch file

**Launching with custom parameters**

To launch the nodes with a custom launch file and parameters, use the following command:
(Modify the frequency as required)

```bash
    #Open a new terminal and source the setup in this terminal
    ros2 launch beginner_tutorials custom_launch.yaml frequency:=1
```
Make sure the frequency value entered is an integer. This will launch publisher and subscriber nodes and server node within the publisher. The server client needs to be called separately for editing the message (Follow Step #4: Running the service client).



## TF Frame 
The `talker` node in this package now broadcasts a TF frame. The `publisher_member_function.cpp` file includes the implementation for broadcasting a static transform using the `tf2_ros::TransformBroadcaster`. This transform can be visualized in tools like RViz to understand the spatial relationship between different frames in the ROS 2 system.

To broadcast a TF frame, the `talker` node publishes a transform with the following details:

- **Parent frame**: "world"
- **Child frame**: "talker_frame"
- **Transform**: A static transform that places the `talker_frame` at a fixed position relative to the "world" frame.

This function is called periodically within the `talker` node to continuously broadcast the transform.

To run the publisher, use the following command:
```bash
cd ~/ros2_ws
# Colcon build if not done
source ./install/setup.bash
ros2 run beginner_tutorials talker
```

**To view the TF transform, run the following commands in separate terminals:**
```bash
# In a new terminal window, echo the topic that broadcasts the static frame:
ros2 topic echo /tf_static
# or 
ros2 run tf2_ros tf2_echo world talker_frame

# In another terminal window, get more information about the frames:
ros2 run tf2_tools view_frames
```

## Colcon Test File

The `test/test.cpp` file contains level 2 integration tests for the nodes in this package. These tests are designed to verify the functionality of the `talker`, `listener`, and `server_client` nodes, ensuring that they perform as expected.

### Running the Tests

To run the tests, follow these steps:

1. **Build the package with tests**:
    ```bash
    colcon build
    ```

2. **Run the tests**:
    ```bash
    colcon test
    ```

3. **View the test results**:
    ```bash
    cat log/latest_test/beginner_tutorials/stdout_stderr.log
    ```

The tests will output the results, indicating whether each test passed or failed. This helps in maintaining code quality and ensuring that changes do not introduce regressions.


## ROS2 Bag to Record and Replay

To record and replay topics, follow these steps:

1. **Build the workspace**:
    ```bash
    colcon build
    ```

2. **Source the setup**:
    ```bash
    source install/setup.bash
    ```

3. **Run the launch file to start recording**:
    ```bash
    ros2 launch beginner_tutorials bag.launch.py enable_recording:=True
    ```

    The recording will start and stop automatically after 15 seconds. Once the recording has stopped, the files `metadata.yaml` and `rosbag_0.db3` will be created under `ros2_ws/results/rosbag`.

4. **Play the recording**:
    ```bash
    ros2 bag play results/rosbag
    ```

5. **Run the listener node to see the recorded data**:
    ```bash
    ros2 run beginner_tutorials listener
    ```

6. **Echo the topic to view the messages**:
    ```bash
    ros2 topic echo /chatter
    ```

## About the Nodes

### Talker Node
The `talker` node publishes messages to the `/topic` topic. It serves as the primary publisher, broadcasting a customizable string message that can be modified by requests from the `server_client` node.

### Listener Node
The `listener` node subscribes to the `/topic` topic. It receives and displays the messages published by the `talker` node, ensuring real-time monitoring of any changes in the message content.

### Server Client Node
The `server_client` node functions as a client that sends requests to modify the message published by the `talker` node. By interacting with the server, it can update the base string that the `talker` node broadcasts to the `/topic` topic.

### Server Node
The `server` node, embedded within the `talker` node, listens for requests from the `server_client` node. Upon receiving a request, it updates the message string that the `talker` node publishes, allowing dynamic changes to the content broadcasted on the `/topic` topic.

## RQT Console Log Output 

![382980265-41d20258-ffc3-410c-aafc-44ee4b59dc09](https://github.com/user-attachments/assets/0f683457-cdfb-4381-b301-bb2241b6ba98)

## Linting and Clang formatting

cpplint has been run and the output is saved in the `cpplint_output.txt` file.

To run cpplint, use the following command:

```bash
cpplint --filter="-legal/copyright" $(find . -name *.cpp | grep -v "/build/")
```

To format the code using clang-format with the Google style, run:

```bash
clang-format -style=Google -i src/beginner_tutorials/src/*.cpp
```
The output of this is available under the results directory, the file is `cland_tidy.png`

## Additional Notes 
1. Ensure that the ROS2 environment is properly set up before building and running the package.
2. For troubleshooting and additional information, refer to the ROS 2 Humble documentation: [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)

## License

This project is licensed under the BSD-3-Clause Licence. Check the license file for details.

## Acknowledgements 

- Open Source Robotics Foundation, Inc.
- ROS2 Community
