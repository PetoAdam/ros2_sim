# ROS2 UR3 Simulation and Visualization

This repository contains a ROS2 package for simulating and visualizing a UR3 robot. The package includes the necessary URDF files, mesh files, and nodes to run the simulation and visualize the robot in RViz2.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
  - [Launching the Simulation](#launching-the-simulation)
  - [Visualizing with RViz2](#visualizing-with-rviz2)
  - [PID control](#pid-controller)
  - [ROS 2 Control](#ros-2-control-integration)
- [Contributing](#contributing)
- [License](#license)

## Installation

### Prerequisites

Ensure you have the following installed on your system:

- Docker
- Visual Studio Code with Remote - Containers extension

### Setting Up the Development Container

1. Clone this repository into your desired workspace directory (e.g., `~/workspace`):

    ```bash
    git clone https://github.com/PetoAdam/ros2_sim.git
    ```

2. Open the repository in Visual Studio Code:

    ```bash
    code .
    ```

3. When prompted, reopen the repository in the devcontainer. If not prompted, you can manually reopen it by pressing `F1` and selecting `Remote-Containers: Reopen in Container`.

### Building the Package

Once inside the devcontainer, build the package:

```bash
cd ~/home/ws
colcon build
```

Source the workspace
```bash
source install/setup.bash
```

# Usage

## Launching the simulation

To start the simulation, use the following command inside the devcontainer after making sure the workspace was built and sourced:

```bash
ros2 launch src/ros2_sim_simulation/launch/simulation.launch.py
```

## Visualizing with Rviz2

Rviz2 is used to render the robot inside the simulation. To launch rendering, use the following command inside the devcontainer after making sure the simulation package is running and the workspace was built and sourced:

```bash
ros2 launch src/ros2_sim_ur3_description/launch/simulation.launch.py
```

## PID controller

The PID controller package is responsible for turning desired joint positions into torque values for the joints. Currently, the gains are not perfectly tuned yet, but the controller works properly.

TODO: add back optional tuning code for the PID controller

```bash
ros2 launch src/ros2_sim_pid_controller/launch/pid_controller.launch.py
```

The PID controller can be tested via the terminal using:

```bash
ros2 topic pub /robot_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: base_link}, joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], points: [{positions: [0, 0, 0, 0, 0, 0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}, {positions: [0.8, 0.8, 0, 0, 0, 0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 3, nanosec: 0}}, {positions: [0, 0, 0, 0, 0, 0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 6, nanosec: 0}}]}" --once
```

This command sends the joints to [0.2, 0.2, 0.0, 0.0, 0.0, 0.0].

## ROS 2 Control Integration

The project also comes with its own ros2_control integration, which converts trajectory data to joint positions, which are sent towards the PID controller.

```bash
ros2 launch src/ros2_sim_control/bringup/launch/ros2_sim_control.launch.py 
```

It can be tested via supplying it this demo trajectory:

```bash
ros2 topic pub /robot_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: base_link}, joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], points: [{positions: [0, 0, 0, 0, 0, 0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}, {positions: [0.2, 0.2, 0, 0, 0, 0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 3, nanosec: 0}}, {positions: [0, 0, 0, 0, 0, 0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 6, nanosec: 0}}]}"
```


# Contributing

Contributions are welcome! Please open an issue or a pull request if you have any suggestions or improvements.

# License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.