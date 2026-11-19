#!/usr/bin/env bash
# Start (or attach to) the simulation container.
#   workspace/docker/run.sh            interactive shell (run again for more shells)
#   workspace/docker/run.sh <cmd...>   run a command, e.g.  run.sh build_ws
# The catkin workspace (parent of the "src" folder holding this repository) is
# mounted at /ws. X11 is forwarded for Gazebo/RViz; /dev/dri for Mesa rendering
# (and --gpus all when the NVIDIA container runtime is installed).
#   REBUILD=1 run.sh   rebuild the image (e.g. after editing the Dockerfile)
#   FLAVOR=plain       plain osrf/ros:noetic image, no LARICS (build-only use)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
d="$REPO"; while [ "$d" != "/" ] && [ "$(basename "$d")" != "src" ]; do d="$(dirname "$d")"; done
[ "$(basename "$d")" = "src" ] || { echo "error: this repository must be inside <workspace>/src" >&2; exit 1; }
WS="$(dirname "$d")"
ICUAS_HOME="/ws/${REPO#"$WS"/}"

FLAVOR="${FLAVOR:-larics}"
if [ "$FLAVOR" = larics ]; then IMG=icuas2025-larics; NAME=icuas2025-larics; DF=Dockerfile.larics
else IMG=icuas2025-noetic; NAME=icuas2025; DF=Dockerfile.plain; fi

if [ -n "${REBUILD:-}" ] || ! docker image inspect "$IMG" >/dev/null 2>&1; then
  docker build -t "$IMG" -f "$HERE/$DF" "$HERE"
  docker rm -f "$NAME" >/dev/null 2>&1 || true
fi

xhost +local:root >/dev/null 2>&1 || true

if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  EXTRA=()
  [ -e /dev/dri ] && EXTRA+=(--device /dev/dri)
  docker info 2>/dev/null | grep -qi nvidia && EXTRA+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
  docker run -d --name "$NAME" --net host --ipc host "${EXTRA[@]}" \
    -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -e ICUAS_HOME="$ICUAS_HOME" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$WS":/ws "$IMG" sleep infinity >/dev/null
  echo "container $NAME started (workspace $WS -> /ws)"
fi

if [ $# -eq 0 ]; then
  exec docker exec -it "$NAME" bash
else
  exec docker exec "$NAME" bash -ic "$*"
fi
