#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM=99
XORG_LOG="/var/log/Xorg.${DISPLAY_NUM}.log"
export DISPLAY=":${DISPLAY_NUM}"

echo "[x11-start] starting Xorg :${DISPLAY_NUM}"
Xorg :${DISPLAY_NUM} -ac -listen tcp \
    -config /etc/X11/xorg-gpu.conf \
    -logfile "${XORG_LOG}" > "${XORG_LOG}.out" 2>&1 &
XORG_PID=$!

# Wait up to ~30 s for the display socket to appear
for _ in $(seq 1 60); do
    [ -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ] && break
    kill -0 "${XORG_PID}" 2>/dev/null || break
    sleep 0.5
done

if [ ! -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ]; then
    echo "[x11-start] ERROR: Xorg :${DISPLAY_NUM} did not start within 30 s" >&2
    echo "----- ${XORG_LOG} -----" >&2
    cat "${XORG_LOG}" 2>/dev/null >&2 || true
    echo "----- ${XORG_LOG}.out -----" >&2
    cat "${XORG_LOG}.out" 2>/dev/null >&2 || true
    exit 1
fi
echo "[x11-start] Xorg :${DISPLAY_NUM} is up"

echo "[x11-start] starting x11vnc on :5900"
x11vnc -display ":${DISPLAY_NUM}" -forever -shared -nopw -rfbport 5900 \
    -o /var/log/x11vnc.log &

echo "[x11-start] starting XFCE desktop"
dbus-run-session -- startxfce4 > /var/log/xfce.log 2>&1 &

echo "[x11-start] starting websockify on :6080"
exec websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5900
