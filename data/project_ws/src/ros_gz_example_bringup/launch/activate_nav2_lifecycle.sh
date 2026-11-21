#!/usr/bin/env bash
# Configure + activate the Nav2 lifecycle nodes reliably.
#
# The nav2 lifecycle_manager (autostart) cannot be used here: it hardcodes
# collision_monitor into its node_names list (nav2 bringup navigation_launch.py)
# and stalls forever waiting for it, since collision_monitor is not launched in
# this setup. So we drive the transitions explicitly.
#
# Key timing insight: a node appears in `ros2 node list` BEFORE it is ready to
# accept lifecycle transitions, so "node exists" != "node configurable". We use
# controller_server as a canary: wait until it actually accepts configure, then
# do the full configure pass (all Nav2 nodes start together). behavior_server
# MUST be active before bt_navigator, otherwise bt_navigator fails to load its
# behavior tree ("Action server spin not available") and NavigateToPose does
# nothing.
set -u

NODES=(controller_server planner_server behavior_server bt_navigator \
       velocity_smoother waypoint_follower smoother_server route_server docking_server)

# Activation order: behavior_server before bt_navigator.
ACTIVATE=(controller_server planner_server behavior_server smoother_server \
          velocity_smoother waypoint_follower route_server docking_server bt_navigator)

state_of() { ros2 lifecycle get "/$1" 2>/dev/null | grep -oE 'unconfigured|inactive|active' | head -1; }

# 1) Wait for the critical nodes to exist (up to 60s).
deadline=$((SECONDS+60))
while [ $SECONDS -lt $deadline ]; do
  c=$(ros2 node list 2>/dev/null | grep -cE '^/(controller_server|planner_server|behavior_server|bt_navigator)$')
  [ "$c" -ge 4 ] && break
  sleep 2
done

# 2) Wait until controller_server (canary) accepts configure, then configure all.
for _ in $(seq 1 25); do
  s=$(state_of controller_server)
  if [ "$s" = "inactive" ] || [ "$s" = "active" ]; then break; fi
  ros2 lifecycle set /controller_server configure >/dev/null 2>&1
  sleep 2
done
for n in "${NODES[@]}"; do ros2 lifecycle set "/$n" configure >/dev/null 2>&1; done
sleep 2
for n in "${NODES[@]}"; do ros2 lifecycle set "/$n" configure >/dev/null 2>&1; done

# 3) Activate in dependency order.
for n in "${ACTIVATE[@]}"; do ros2 lifecycle set "/$n" activate >/dev/null 2>&1; done

# 4) Ensure behavior_server + bt_navigator are active (bounded targeted retry).
for _ in $(seq 1 8); do
  bs=$(state_of behavior_server); bn=$(state_of bt_navigator)
  if [ "$bs" = "active" ] && [ "$bn" = "active" ]; then break; fi
  [ "$bs" != "active" ] && ros2 lifecycle set /behavior_server activate >/dev/null 2>&1
  [ "$bn" != "active" ] && ros2 lifecycle set /bt_navigator activate >/dev/null 2>&1
  sleep 2
done

# 5) Report final states.
echo "Nav2 lifecycle states:"
for n in "${NODES[@]}"; do printf '  %-18s %s\n' "$n" "$(state_of "$n")"; done
