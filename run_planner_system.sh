#!/bin/bash

# Navigate to workspace
cd /home/kraken/kraken-nano/ROS/ws
source install/setup.bash

echo "Starting Controller in background..."
ros2 run kraken controller &
CONTROLLER_PID=$!

echo "Starting Planner..."
ros2 run kraken planner &
PLANNER_PID=$!

echo "Both nodes started. Controller PID: $CONTROLLER_PID, Planner PID: $PLANNER_PID"
echo "Press Ctrl+C to stop both nodes..."

# Function to cleanup on exit
cleanup() {
    echo "Stopping nodes..."
    kill $CONTROLLER_PID $PLANNER_PID 2>/dev/null
    exit 0
}

# Trap Ctrl+C and call cleanup
trap cleanup SIGINT

# Wait for both processes
wait $CONTROLLER_PID $PLANNER_PID
