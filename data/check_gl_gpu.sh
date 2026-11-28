#!/usr/bin/env bash
# Check which GPU is used for OpenGL rendering inside the container.
#
# Run from the host:
#   docker compose -f compose.sim.yaml exec -it ros2_gazebo bash /root/data/check_gl_gpu.sh
#
# Requires (already in the image): nvidia-smi; glmark2 + an X display for the
# live GL_RENDERER check (headless containers report nvidia-smi only).

set -u

section() { printf '\n== %s ==\n' "$*"; }

section "NVIDIA devices visible in container"
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=index,name,driver_version,compute_mode --format=csv
else
    echo "nvidia-smi NOT found - GPU passthrough may be broken"
fi

section "GL environment"
echo "DISPLAY                    : ${DISPLAY:-<not set>}"
echo "__GLX_VENDOR_LIBRARY_NAME  : ${__GLX_VENDOR_LIBRARY_NAME:-<not set>}"
echo "NVIDIA_VISIBLE_DEVICES     : ${NVIDIA_VISIBLE_DEVICES:-<not set>}"

if [ -n "${DISPLAY:-}" ]; then
    section "OpenGL renderer + live rendering test (glmark2 on $DISPLAY)"
    if command -v glmark2 >/dev/null 2>&1; then
        # a few scenes only: the full suite takes >2 min
        timeout 120 glmark2 --off-screen --size 256x160 -b build -b texture -b shading >/tmp/glmark2.out 2>&1 &
        GLJOB=$!
        # let it start rendering, then see which GPU the process is on
        sleep 4
        GLPID=$(pgrep -x glmark2 2>/dev/null | head -1)
        if command -v nvidia-smi >/dev/null 2>&1; then
            echo "CUDA clients while glmark2 is rendering (pid ${GLPID:-?}):"
            timeout 10 nvidia-smi --query-compute-apps=pid,gpu_bus_id,process_name,used_memory --format=csv 2>/dev/null || true
            # pmon also lists graphics (GLX) clients, but per-process tracking can
            # hang inside containers, so guard it
            if timeout 5 nvidia-smi pmon -c 2 -s p >/tmp/pmon.out 2>&1; then
                cat /tmp/pmon.out
                if [ -n "${GLPID}" ] && awk -v p="${GLPID}" '{for (i = 6; i <= NF; i++) if ($i == p) found = 1} END {exit !found}' /tmp/pmon.out; then
                    echo "-> glmark2 (pid ${GLPID}) is rendering on the NVIDIA GPU(s) listed above"
                else
                    echo "-> per-process mapping unavailable; the GL_RENDERER string below is authoritative"
                fi
            else
                echo "(nvidia-smi pmon not usable in this container)"
            fi
        fi
        wait "$GLJOB"
        echo "--- glmark2 output (OpenGL info + score) ---"
        grep -m1 -A4 'OpenGL Information' /tmp/glmark2.out
        grep -E 'score' /tmp/glmark2.out || tail -n 20 /tmp/glmark2.out
    else
        echo "glmark2 not installed - cannot verify actual rendering"
    fi

    if command -v glxinfo >/dev/null 2>&1; then
        section "OpenGL renderer via glxinfo ($DISPLAY)"
        glxinfo 2>/dev/null | grep -Ei '^\s*(OpenGL (vendor|renderer|version))'
    fi
else
    section "OpenGL renderer"
    echo "No X display (DISPLAY not set): this image's headless EGL path does not"
    echo "expose the NVIDIA ICD, so the GL_RENDERER string cannot be queried."
    echo "OpenGL apps (Gazebo, glmark2) render on the GPU(s) listed by nvidia-smi"
    echo "above - for a single-GPU system that is the GPU used for OpenGL."
    echo "Re-run this script with DISPLAY set (e.g. from the sim container) to get"
    echo "the exact GL_RENDERER string and a live rendering check."
fi
