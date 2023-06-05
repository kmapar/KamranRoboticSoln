# MachinaLabs ROS2 Sensor Network

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/kmapar/ros2network)
[![License](https://img.shields.io/badge/license-MIT-blue)](https://github.com/kmapar/ros2network)


## Objective
The goal of This project is to build a ROS2 network that collects data from 3-DOF sensors and makes the filtered data available as a ROS service and topic. Since we cannot send a real sensor to all of our applicants, we made a simple simulator (sensor.py) that mimics the behavior of a real sensor but with random data.

-   The first task is to make a costume service for 3-DOF sensor
-   The second task is to make a ROS2 service server that continuously reads data from the sensor and has the latest filter data available for the client service that you make.
-    Finally, please make a simple client that calls two of these services and publishes them a topic with 500Hz. Please keep in mind that your service servers can run slower than 500Hz.
-    You can define a second server in the simulator to modify the code and run two at the same time.
-    You can check the example.py to see how to make calls to the sensor
  
 
## Prerequisites
- ROS (Robot Operating System) installed (Humble distribution)
- Python 3
- Linux Ubuntu 22.04


## Initialization 
1. Create a ROS workspace (if not already created):
   ```bash
   mkdir -p ~/ros_ws/src
   cd ~/ros_ws/src
   
2. Clone the network package into the src directory:
    ```bash
    git clone https://github.com/kmapar/ros2network.git

3. Build the ROS workspace:
    ```bash
    cd ~/ros_ws
    colcon build


## Usage
### Starting the Sensor script
To run the sensor.py script, open a terminal enter the following commands:

1. Navigate to ROS workspace 
    ```bash
    cd ~/ros_ws
    
2. Activate your ROS workspace 
    ```bash
    source install/setup.bash

3. Launch the sensor.py script
    ```bash
    python3 network/network/sensor.py

### Starting the Service Server
To run the service_server_node.py script, open a separate terminal and enter the following commands:

1. Navigate to ROS workspace
    ```bash
    cd ~/ros_ws
    
2. Activate your ROS workspace 
    ```bash
    source install/setup.bash

3. Launch the service_server_node.py script
    ```bash
    python3 network/network/service_server_node.py
   
### Starting the Custom Service
To run the custom service, open another terminal and enter the following commands:

1. Navigate to ROS workspace
    ```bash
    cd ~/ros_ws
    
2. Activate your ROS workspace 
    ```bash
    source install/setup.bash

3. Launch CustomService.srv 
    ```bash
    ros2 run service_pkg CustomService
 
 
## Important Note Regarding Connectivity Issues
**Please note:**
Unfortunately, due to connectivity issues between the sensor script and my service_server_node, I was not able to efficiently fine tune the number of samples called in the service_server_node. I have referred to various documentation, and tutorials, using different machines, and have reached out to a former teaching assistant from my robotics courses for advice. Though, despite a multitude of efforts, the sensor script and my service server node will not establish a TCP connection. I am utilizing the example code given to establish a connection but to no avail. As soon as I resolve this connectivity issue, I will be sure to optimize the sample call number and employ a second service.
    
I apologize for the delay, working on this project has been a tremendous pleasure and has offered me more insight and strengthened my ROS skillset. I appreciate the opportunity to be considered for this position, and look foward to hearing from the team soon. Thank you!


## Author

Kamran Mapar

Email: kmapar@ucsd.edu
