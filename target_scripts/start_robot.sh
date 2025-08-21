#!/bin/bash
 
# --- Set Environment Variables ---
# These paths are needed for ROS2 to find its libraries and Python packages.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/data/home/qnxuser/opt/ros/humble/lib
export PYTHONPATH=$PYTHONPATH:/data/home/qnxuser/opt/ros/humble/usr/lib/python3.11/site-packages/:/data/home/qnxuser/.local/lib/python3.11/site-packages/
export COLCON_PYTHON_EXECUTABLE=/system/bin/python3
 
# --- Sourcing ROS2 ---
# Source the main ROS2 environment
if [ -f /data/home/qnxuser/opt/ros/humble/setup.bash ]; then
    . /data/home/qnxuser/opt/ros/humble/setup.bash
else
    echo "Error: ROS2 global setup file not found!"
    exit 1
fi
 
# Source your workspace's local setup file to find your custom nodes
if [ -f /data/home/qnxuser/opt/ros/nodes/local_setup.bash ]; then
    . /data/home/qnxuser/opt/ros/nodes/local_setup.bash
fi
 
# --- Starting the ROS2 Nodes ---
echo "Starting Joy Teleop and Arm Controller nodes..."
 
# Run the C++ joystick node in the background
ros2 run joy_teleop_hiddi joy_teleop_node &
 
# Run the Python arm controller node in the background
ros2 run arm_controller arm_controller_node.py &
 
# --- Wait for all background nodes to exit ---
echo "All nodes started. Press Ctrl+C in this terminal to stop both."
wait
 
echo "All nodes have been shut down. Script finished."
 