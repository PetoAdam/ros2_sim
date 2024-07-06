# ROS2 Robot Arm Simulation and Visualization

This repository contains a ROS2 package for simulating and visualizing a UR3 robot. The package includes the necessary URDF files, mesh files, and nodes to run the simulation and visualize the robot in RViz2.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
  - [Preset Launch Configurations](#preset-launch-configurations)
  - [Launching the Simulation](#launching-the-simulation)
  - [Visualizing with RViz2](#visualizing-with-rviz2)
  - [PID control](#pid-controller)
  - [ROS 2 Control](#ros-2-control-integration)
  - [Moveit Integration](#moveit-integration)
  - [Custom Motion Planner Library](#custom-motion-planner-library)
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

## Preset launch configurations

There are preset launch configurations that can be easily run without having to open multiple terminals for every package. Current presets:
 - launch_all_with_rviz.sh: This shell script runs the simulation, the PID controller, the ros2_control, the MoveIt and the RViz Visualization (with Motion Planning enabled) packages with their correct launch files.

    + To run it, makes sure to build and source the workspace and then run: ```
    ./launch_all_with_rviz.sh``` from the root directory of the repository.

  - launch_all_with_custom_planner.sh: This shell script runs the simulation, the PID controller, the ros2_control, the motion planner and the RViz Visualization packages with their correct launch files.

    + To run it, makes sure to build and source the workspace and then run: ```
    ./launch_all_with_with_custom_planner.sh``` from the root directory of the repository.
    + To test it, run this command from another sourced terminal: ```
    ros2 action send_goal /plan_and_execute ros2_sim_msgs/action/PlanAndExecute "{target_pose: {position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {x: 1.0, y: 1.0, z: 1.0, w: 1.0}}, planning_pipeline: 'pilz_industrial_motion_planner', planner_id: 'PTP', timeout: 10.0}"```

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
ros2 topic pub /desired_positions sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}}, name: [shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], position: [0.2, 0.2, 0.0, 0.0, 0.0, 0.0]}"
```

This command sends the joints to [0.2, 0.2, 0.0, 0.0, 0.0, 0.0].

## ROS 2 Control Integration

The project also comes with its own ros2_control integration, which converts trajectory data to joint positions, which are sent towards the PID controller.

```bash
ros2 launch src/ros2_sim_control/bringup/launch/ros2_sim_control.launch.py 
```

It can be tested via supplying it this demo trajectory:

```bash
ros2 topic pub /robot_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: base_link}, joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], points: [{positions: [0, 0, 0, 0, 0, 0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}, {positions: [0.8, 0.8, 0, 0, 0, 0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 3, nanosec: 0}}, {positions: [0, 0, 0, 0, 0, 0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 6, nanosec: 0}}]}" --once
```

## MoveIt Integration

This project utilizes MoveIt2 for motion planning. To enable motion planning and execute planned trajectories, follow these steps:

The motion planner UI runs in RViz, so a new visualization windows has to be started instead of the command provided before:

```bash
ros2 launch src/ros2_sim_ur3_description/launch/planner.launch.py
```

To launch the motion planner, run:

```bash
ros2 launch ros2_sim_moveit_config move_group.launch.py
```

Planning and executing trajectories can be done via the MotionPlanning window inside RViz:
![rviz.png](resources/images/rviz.png)

## Custom Motion Planner Library

The custom motion planner library in this repository enhances the functionality of motion planning for the robot simulation. It integrates with MoveIt2 and provides additional capabilities tailored to specific scenarios. It uses a custom ros2 action to listen for advanced motion requests.

To launch the custom motion planner, make sure the default moveit_config motion planner library is not running and run:

```bash
ros2 launch ros2_sim_motion_planner motion_planner.launch.py
```

The library can be tested with the following command:

```bash
ros2 action send_goal /plan_and_execute ros2_sim_msgs/action/PlanAndExecute "{target_pose: {position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {x: 1.0, y: 1.0, z: 1.0, w: 1.0}}, planning_pipeline: 'pilz_industrial_motion_planner', planner_id: 'PTP', timeout: 10.0}"
```

# Contributing

Contributions are welcome! Please open an issue or a pull request if you have any suggestions or improvements.

# License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.