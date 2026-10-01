#!/usr/bin/env bash
# real_robot_test.sh SKETCHDIR [ROUTE]
#   Builds that sketch with the kit simulator (sim/sim.cpp), runs it on every condition in
#   sim/real_robot_conds.txt (the student's real robot, condition by condition, each with the
#   reason from the log of the 19 real runs) and prints a table: condition, score, the first
#   fault and its count n, what the firmware did after a mid-run restart, the path check, and
#   the total.
#
#   ROUTE: 312 (default: the order the student uses), 132, or both.
#   ./sim/real_robot_test.sh ../robot_e33_v2
#   ./sim/real_robot_test.sh ../e33_mission both      (adds a calibrated column: e33_calibrate beside it)
#   CAL=1 ./sim/real_robot_test.sh ../robot_e33_v3    (a calibrated column for an all-in-one sketch too)
#   ONLY=lift,rst-pick1 ./sim/real_robot_test.sh ../robot_e33_v3   (some conditions only)
#   OUT=DIR (where every run's output is kept; default ./rrt_out/SKETCH)  J=N (sim processes, default 4)
#   CONDS=FILE (another condition list)
# Needs g++ and python (3.6 or newer).
set -u
if [ $# -lt 1 ]; then sed -n '2,17p' "$0"; exit 2; fi
K="$(cd "$(dirname "$0")" && pwd)"
PY=python3
"$PY" -c 1 >/dev/null 2>&1 || PY=python
ARGS=("$1" "${2:-312}" --cal "${CAL:-auto}" -j "${J:-4}")
[ -n "${OUT:-}" ] && ARGS+=(--out "$OUT")
[ -n "${ONLY:-}" ] && ARGS+=(--only "$ONLY")
[ -n "${CONDS:-}" ] && ARGS+=(--conds "$CONDS")
exec "$PY" "$K/real_robot_test.py" "${ARGS[@]}"
