#!/bin/bash
set -e

# Source ROS 2 humble first (required for builds and runs)
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Source a workspace overlay if present
if [ -f "/workspaces/xgolite_ws/install/setup.bash" ]; then
  source "/workspaces/xgolite_ws/install/setup.bash"
elif [ -f "/workspaces/xgolite_ws/install/local_setup.bash" ]; then
  source "/workspaces/xgolite_ws/install/local_setup.bash"
elif [ -f "$HOME/workspace/install/setup.bash" ]; then
  source "$HOME/workspace/install/setup.bash"
elif [ -f "$HOME/workspace/install/local_setup.bash" ]; then
  source "$HOME/workspace/install/local_setup.bash"
fi

exec "$@"

