#!/bin/bash
# Headless smoke test: start the two-UAV session, take off, explore for N
# minutes, print both UAV states every minute, record a diagnostic bag.
# Honors the same env vars as session.yml (SCENARIO, NUM_UAVS, STRATEGY, ...).
# Run inside the icuas2025-larics container:
#   /ws/sim/icuas2025_larics/run_exploration_test.sh [minutes]   (default 8)
# Output: /ws/sim/icuas2025_larics/results/<timestamp>/{summary.txt,diag.bag}
source "$(dirname "$(readlink -f "$0")")/env.sh"
MIN=${1:-8}
UAVS="red blue"; [ "${NUM_UAVS:-2}" = 3 ] && UAVS="red blue yellow"
HERE="$(dirname "$(readlink -f "$0")")"
OUT="$HERE/results/$(date +%Y%m%d_%H%M%S)"; mkdir -p "$OUT"

stop_all() {
  tmux kill-server 2>/dev/null
  pkill -f rosbag; pkill -f gzserver; pkill -f arducopter; pkill -f roslaunch; pkill -f rosmaster
  sleep 4
}
z_of() { timeout 3 rostopic echo -n1 /$1/odometry/pose/pose/position/z 2>/dev/null | head -1; }
state() {
  local p a
  p=$(timeout 3 rostopic echo -n1 /$1/odometry/pose/pose/position 2>/dev/null | tr -s " " | tr "\n" " " \
      | sed -E "s/x: ([-0-9.]{1,6})[0-9e-]* y: ([-0-9.]{1,6})[0-9e-]* z: ([-0-9.]{1,5})[0-9e-]*.*/(\1,\2,\3)/")
  a=$(timeout 3 rostopic echo -n1 /$1/mavros/state/armed 2>/dev/null | head -1)
  echo "$1 $p armed=$a"
}

stop_all
"$HERE/start.sh" >/dev/null 2>&1
echo "waiting for takeoff..." | tee "$OUT/summary.txt"
for i in $(seq 1 60); do
  up=1; for r in $UAVS; do awk -v a="$(z_of $r)" 'BEGIN{exit !(a>1.2)}' || up=0; done
  [ $up = 1 ] && break; sleep 5
done
if [ $up != 1 ]; then
  for r in $UAVS; do echo "$r z=$(z_of $r)"; done | tee -a "$OUT/summary.txt"
  echo "TAKEOFF FAILED (see: tmux attach -t icuas2025)" | tee -a "$OUT/summary.txt"; exit 1
fi
sleep 40   # planner window waits ~30 s before launching
TOPICS=""; for r in $UAVS; do TOPICS="$TOPICS /$r/odometry /$r/command/trajectory /$r/tracker/input_trajectory /$r/inspection_trajectory /$r/mavros/state /$r/no_go_zones /$r/semantic_location /$r/done_inspection /$r/previous_semantic_points"; done
rosbag record -O "$OUT/diag.bag" $TOPICS __name:=diag_bag >/dev/null 2>&1 &
for r in $UAVS; do rosservice call /$r/pci_initialization_trigger "{}" >/dev/null 2>&1; done
sleep 15
for r in $UAVS; do rosservice call /$r/planner_control_interface/std_srvs/automatic_planning "{}" >/dev/null 2>&1; done
echo "exploration started" | tee -a "$OUT/summary.txt"
for k in $(seq 1 "$MIN"); do
  sleep 60
  line="t+${k}min"; for r in $UAVS; do line="$line | $(state $r)"; done
  echo "$line" | tee -a "$OUT/summary.txt"
  if echo "$line" | grep -q "armed=False"; then echo "CRASH/DISARM detected" | tee -a "$OUT/summary.txt"; break; fi
done
pkill -INT -f "rosbag record"; sleep 4   # SIGINT lets rosbag close the file
tmux capture-pane -p -J -S -200000 -t icuas2025:planner | sed "s/\x1b\[[0-9;]*m//g" > "$OUT/planner.log"
echo "results in $OUT"
