# my_beginner_tutorials

## Overview
This project contains beginner tutorials for ROS2 Humble. It includes simple publisher and subscriber nodes to demonstrate basic ROS2 functionalities.

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

1. **Download the Source Code from the Release(either zip or tar.gz)**
    Unzip the file and paste the contents of the folder into this directory - `ros2_ws/src/beginner_tutorials`

2. **Check for Missing Dependencies Before Building**

    To run rosdep in the root of the workspace: 

    ```sh
    cd ~/ros2_ws 
    rosdep install -i --from-path src --rosdistro humble -y
    ```

3. **Build the package**
    Use colcon to build the package 

    ```sh
    colcon build --packages-select beginner_tutorials
    ```

4. **Source the setup**

    Source the script setup to overlay this workspace on the environment 
    ```sh
    source install/setup.bash
    ```

## Using the package / Running the Package 

1. **Running the publisher node** 

    To run the talker node, run the following command 

    ```sh
    ros2 run beginner_tutorials talker
    ```

2. **Running the subscriber node**

    To run the subscriber node, run the following command 
    ```sh
    #Open a new terminal and source the setup in this terminal
    ros2 run beginner_tutorials listener
    ```


## About the Nodes 

**Talker**

The ```talker``` node publishes messages to the ```topic``` topic


**Listener**

The ```listener``` node subscribes to messages from the ```topic``` topic.


## Linting

cpplint has been run and the output is saved in the ```cpplint_output.txt``` file

To run cpplint run the following command :

```sh
find src -name "*.cpp" | xargs cpplint 2>&1 | tee cpplint_output.txt
```

## License

This project is licensed under the BSD-3-Clause Licence. Check the license file for details

## Acknowledgements 

- Open Source Robotics Foundation, Inc.
- ROS2 Community