#!/bin/bash

# Function to handle termination signals
terminate() {
    echo "Terminating all processes..."
    kill -TERM "$simulation_pid" 2>/dev/null
    kill -TERM "$pid_controller_pid" 2>/dev/null
    kill -TERM "$control_pid" 2>/dev/null
    kill -TERM "$gateway_pid" 2>/dev/null
    kill -TERM "$planner_pid" 2>/dev/null
}

# Trap termination signals (e.g., Ctrl+C)
trap terminate SIGINT SIGTERM

# Launch the simulation
ros2 launch src/ros2_sim_simulation/launch/simulation.launch.py &
simulation_pid=$!

# Launch the PID controller
ros2 launch src/ros2_sim_pid_controller/launch/pid_controller.launch.py &
pid_controller_pid=$!

# Launch the control bringup
ros2 launch src/ros2_sim_control/bringup/launch/ros2_sim_control.launch.py &
control_pid=$!

# Launch gateway
ros2 launch src/ros2_sim_gateway/launch/ros2_sim_gateway.launch.py &
gateway_pid=$!

# Launch motion planner
ros2 launch src/ros2_sim_motion_planner/launch/motion_planner.launch.py &
planner_pid=$!

# Wait for all processes to complete or be terminated
wait $simulation_pid $pid_controller_pid $control_pid $gateway_pid $planner_pid

echo "All processes have completed."