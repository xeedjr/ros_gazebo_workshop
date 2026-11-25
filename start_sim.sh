#!/usr/bin/env bash
set -euo pipefail

xhost +local:
docker compose -f compose.sim.yaml up
