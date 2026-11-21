#!/bin/bash
#
# stop.sh - stop all ROS2 / Gazebo entities started by build2.sh
#
# Safe to run when nothing is up (no-ops). Never starts anything; it only
# tears down. Use build2.sh to start the system again.

set -u

echo "[stop] Stopping build2.sh launcher + ros2 launch ..."
pkill -TERM -f "build2.sh" 2>/dev/null || true
pkill -TERM -f "ros2 launch" 2>/dev/null || true

echo "[stop] Stopping Gazebo (gz sim / gzserver / gzclient) ..."
pkill -TERM -f "gz sim" 2>/dev/null || true
pkill -TERM -f "gzserver" 2>/dev/null || true
pkill -TERM -f "gzclient" 2>/dev/null || true

# Let processes exit gracefully before force-killing survivors.
sleep 3

echo "[stop] Force-killing any survivors ..."
pkill -KILL -f "build2.sh" 2>/dev/null || true
pkill -KILL -f "ros2 launch" 2>/dev/null || true
pkill -KILL -f "gz sim" 2>/dev/null || true
pkill -KILL -f "gzserver" 2>/dev/null || true
pkill -KILL -f "gzclient" 2>/dev/null || true
pkill -KILL -f "rtabmap" 2>/dev/null || true
pkill -KILL -f "rviz2" 2>/dev/null || true
pkill -KILL -f "nav2_" 2>/dev/null || true
pkill -KILL -f "parameter_bridge" 2>/dev/null || true
pkill -KILL -f "ros_gz_bridge" 2>/dev/null || true
pkill -KILL -f "foxglove" 2>/dev/null || true
# Catch-all: any remaining ROS2 node executables launched from /opt/ros
pkill -KILL -f "/opt/ros/jazzy/lib/" 2>/dev/null || true

echo "[stop] Verifying ..."
remaining="$(ps aux | grep -E "build2.sh|ros2 launch|gz sim|gzserver|gzclient|rtabmap|rviz2|nav2_|parameter_bridge|ros_gz_bridge|/opt/ros/jazzy/lib/" | grep -v grep | grep -v 'defunct')"
if [ -n "$remaining" ]; then
  echo "[stop] WARNING: the following are still running:"
  echo "$remaining"
  exit 1
else
  echo "[stop] All ROS2 / Gazebo entities stopped."
  exit 0
fi
