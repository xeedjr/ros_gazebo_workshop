# AGENTS.md

## Rules

- Run commands only inside the Docker container.
- Do not run commands on the host that are not related to the container.
- Do not modify files outside this folder.

## Starting the simulation container

The simulation (ROS 2 Jazzy + Gazebo) runs in the `ros2_gazebo` service defined in `compose.sim.yaml`. Start it from the host, **attached** (no `-d`) so `build2.sh` output is streamed to the agent's console:

```bash
docker compose -f compose.sim.yaml up
```

To follow logs of an already-running stack:

```bash
docker compose -f compose.sim.yaml logs -f ros2_gazebo
```

### Web-display variant: `compose.sim.web.yaml`

The same sim + x11 stack is also provided as a separate, self-contained file, `compose.sim.web.yaml` (bundles the GPU `x11`/noVNC service and `ros2_gazebo`). Start it the same way, attached:

```bash
docker compose -f compose.sim.web.yaml up
```

`compose.sim.web.yaml` and `compose.sim.yaml` share the same project name, network, and persistent volumes, so they describe the **same** stack — start only one of the two at a time (single-instance rule).

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

## Display / GUI

The sim provides its own display via the `x11` service (Xorg + nvidia driver, headless, display `:99`, 1920x1080) running an **XFCE desktop**, exposed to the browser via x11vnc + noVNC:

- Web access: `http://localhost:6080/vnc.html` (noVNC shows the full 1920x1080 XFCE desktop, scaled to fit the browser window; direct VNC on `:5900`). The desktop includes a panel, `xfce4-terminal`, and the `thunar` file manager.
- The services use Docker's default bridge network. `DISPLAY=x11:99` is set for `ros2_gazebo` — it reaches the X server in the `x11` container via Docker DNS (TCP, port 6099 inside the x11 container). It is **not** the host's display. Gazebo GUI, RViz2 and rtabmap_viz render there.
- Host ports published by compose: `6080` (web), `5900` (VNC), `8769` (Foxglove bridge). They must be free on the host.
- To change the desktop resolution, edit `Virtual 1920 1080` in `x11/xorg-gpu.conf` and restart the `x11` service.
