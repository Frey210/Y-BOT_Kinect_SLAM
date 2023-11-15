## README: Y-BOT : Mobile Robot SLAM with Kinect Camera

### Overview

Welcome to the Mobile Robot SLAM System repository! This project focuses on implementing Simultaneous Localization and Mapping (SLAM) on a mobile robot using a Kinect camera. The Robot Operating System (ROS) Noetic serves as the foundation for seamless integration and efficient communication between components, enabling the robot to map its environment while simultaneously determining its own position.

### Table of Contents

1. [Prerequisites](#prerequisites)
2. [ROS (Robot Operating System)](#ros-robot-operating-system)
3. [SLAM (Simultaneous Localization and Mapping)](#slam-simultaneous-localization-and-mapping)
4. [System Architecture](#system-architecture)
5. [Design Overview](#design-overview)
   - [Schematics](#schematics)
   - [System Block Diagram](#system-block-diagram)
   - [Robot Design](#robot-design)
6. [Our Team](#our-team)
   - [Team Members](#team-members)
   - [Contributors](#contributors)
7. [Running the Program](#running-the-program)
8. [Mapping Mode](#mapping-mode)
9. [Localization and Autonomous Mode](#localization-and-autonomous-mode)
10. [Notes](#notes)
11. [Contributing](#contributing)

### Prerequisites

Before running the SLAM system, ensure that you have the following prerequisites installed:

- ROS Noetic: Follow the [ROS Installation Guide](http://wiki.ros.org/noetic/Installation) for instructions on installing ROS Noetic.

### ROS (Robot Operating System)

ROS is an open-source middleware framework designed to develop, control, and coordinate software for robotic systems. It provides a flexible and modular architecture, making it ideal for various robotic applications.

### SLAM (Simultaneous Localization and Mapping)

SLAM is a technique used in robotics to create a map of an unknown environment while simultaneously tracking the location of the robot within that environment. It involves the integration of sensor data, such as that from a Kinect camera, to construct a map and estimate the robot's pose in real-time.

### System Architecture

The SLAM system architecture is designed to facilitate efficient mapping and navigation. The key components include:

- **Controller Node:** Handles wheel control and communication with low-level hardware (Arduino).

- **Mapping Node:** Utilizes sensor data from the Kinect camera to create a map of the environment.

- **Navigation Node:** Implements localization and autonomy, allowing the robot to navigate within the mapped environment.


### Design Overview

#### Schematics

Below is the schematic diagram of the electronic connections for the mobile robot. This includes the connections between the Arduino, motors, and other electronic components.

![Schematics](/Design/schematic.png)


#### System Block Diagram

The block diagram provides a high-level overview of the system's functional components and their interactions.

![System Block Diagram](/Design/block_diagram.png)


#### Robot Design

The design of the mobile robot is illustrated in the following diagram, showcasing the placement of sensors, the Kinect camera, and the wheel configuration.

![Robot Design](/Design/iso2.png)


### Our Team

#### Team Members

Meet the talented individuals who contributed to this project:
   ![Teams](/Design/our_team.jpg)
1. **Fariz Achmad Faizal**
2. **Abdul Salam**

#### Contributors

We appreciate the efforts of all contributors who helped make this project possible.

special thanks for our supervisor :
1. Muh Anshar, ST. M.Sc(Research), Ph. D
2. Prof. Dr. Ir. Andani, M.T.
3. Dr. A. Ejah Umraeni Salam, S.T, M.T.

### Running the Program

1. **Open Terminal:**
   Open a terminal on your system to execute the following commands.

2. **Set ROS Noetic Environment:**
   Set up the ROS Noetic environment using the following command:
   ```bash
   source /opt/ros/noetic/setup.bash
   ```

3. **Rover Control and Low-Level Communication:**
   Navigate to `iascr/home/catkin_ws/src/my_robot_pkg/launch/` and launch the controller:
   ```bash
   roslaunch controller.launch
   ```

4. **Mapping Mode**

   - Navigate to `iascr/home/catkin_ws/src/autonomous_mobile_robot/launch/` and launch the mapping node:
     ```bash
     roslaunch mapping.launch
     ```

   - To save the mapping data using `map_saver`, open a new terminal and run the following command:
     ```bash
     rosrun map_server map_saver -f my_map
     ```
     This will save the map data as `my_map.yaml` and `my_map.pgm` in the current directory.

5. **Localization and Autonomous Mode**

   - Navigate to `iascr/home/catkin_ws/src/autonomous_mobile_robot/launch/` and open the `navigation.launch` file for editing:
     ```bash
     nano navigation.launch
     ```
   - Locate the `map_server` node and update the `map_file` parameter to the desired YAML map file:
     ```xml
     <node name="map_server" pkg="map_server" type="map_server" args="$(find your_package)/maps/your_map.yaml"/>
     ```
     Save and exit the editor.

   - Launch the navigation node:
     ```bash
     roslaunch navigation.launch
     ```

### Notes

- Ensure the Kinect camera is properly connected and calibrated for accurate sensor data.
- Customize parameters in the launch files to match your robot's specifications.
- Refer to the ROS documentation for additional information on configuring and fine-tuning the SLAM system.

### Contributing

Feel free to contribute, report issues, or provide feedback to enhance the functionality and reliability of this SLAM system for mobile robots. Your contributions are valuable in improving the project and making it more robust. Happy mapping!
