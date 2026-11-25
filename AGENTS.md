# AGENTS.md

## Rules

- Run `xhost +local:` before starting the simulation container to grant X display access.
- Do not run commands on the host that are not related to the container.
- Do not modify files outside this folder.

## Starting the simulation container

The simulation (ROS 2 Jazzy + Gazebo) runs in the `ros2_gazebo` service defined in `compose.sim.yaml`. Start it from the host using the provided script, which grants X display access and starts the container **attached** (no `-d`) so `build2.sh` output is streamed to the agent's console:

```bash
./start_sim.sh
```

To follow logs of an already-running stack:

```bash
docker compose -f compose.sim.yaml logs -f ros2_gazebo
```

Notes:

- The service entrypoint is `/root/data/project_ws/build2.sh`, which builds the workspace and launches the system. See `data/project_ws/AGENTS.md` for what `build2.sh` does and the single-instance rule.
- `stdin_open: true` and `tty: true` keep the entrypoint's output visible and unbuffered.
- Start exactly one instance at a time.

## Persistent named volumes

These named volumes preserve ROS 2 / Gazebo data across `docker compose` recreate/rebuild:

| Volume | Container path | Contents |
|---|---|---|
| `ros2_data` | `/root/.ros` | ROS 2 logs, CLI config |
| `gazebo_data` | `/root/.gz` | Gazebo Sim (Ignition) config, logs, downloaded models/worlds |
| `gazebo_classic_data` | `/root/.gazebo` | Gazebo Classic config/logs/models (if used) |

`./data` is bind-mounted to `/root/data` for live editing of the workspace source.
