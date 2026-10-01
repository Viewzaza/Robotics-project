#!/usr/bin/env bash
# The student's robot in the simulator: the conditions that matter for THIS robot (TT gear motors,
# TB6612 on a 9 V PP3, the left motor weaker, the student's own sensor levels), in one table.
# Every condition runs, for both routes (312 and 132):
#   no-cal : the mission with no calibration (the numbers written in calibration.h)
#   cal    : MODE_CALIBRATE on a fresh EEPROM, then the mission using that EEPROM
# and every mission is run with --path and checked by sim/path_check.py.
#
#   ./sim/student_test.sh                        # the list below (about 2 to 4 minutes)
#   ./sim/student_test.sh "--student --trimL=0.6" "--student-slow --noise=30"   # your own
#   ./sim/student_test.sh "--student-fast => --student-slow"   # calibrate with the left flags,
#                                                # run the missions with the right ones
#   FWROOT=/path/to/tree ./sim/student_test.sh   # the sketches from another tree (it must have
#                                                # e33_mission/ and e33_calibrate/); default: ..
#   DEFS="-DMAX_SP=100" ./sim/student_test.sh    # extra defines for every build
#   JOBS=4 ./sim/student_test.sh                 # conditions run side by side (default 4, 4 runs each)
#   KEEP=dir ./sim/student_test.sh               # keep every run's output in dir
#
# The presets (sim/sim.cpp --help): --student is the best estimate of the robot today,
# --student-slow a tired PP3 and slower motors, --student-fast a fresh PP3 and faster motors.
# Every run places the robot the way the e33 sketches expect: axle over C1 (--startx=0); a
# --startx later in the condition overrides it.
set -u
cd "$(dirname "$0")/.."
FWROOT="${FWROOT:-.}"
DEFS="${DEFS:-}"
JOBS="${JOBS:-4}"
PY="${PYTHON:-python}"
command -v "$PY" >/dev/null 2>&1 || PY=python3

# the path of a sketch as #include sees it from sim/sim.cpp
inc() {
  case "$FWROOT" in
    /*|[A-Za-z]:*) local p="$FWROOT/$1/$1.ino"
                   command -v cygpath >/dev/null 2>&1 && p=$(cygpath -m "$p")
                   printf '"%s"' "$p" ;;
    *)             printf '"../%s/%s/%s.ino"' "$FWROOT" "$1" "$1" ;;
  esac
}
for f in calibration.h controlLibrary.h pidLibrary.h runLog.h; do
  cmp -s "$FWROOT/e33_mission/$f" "$FWROOT/e33_calibrate/$f" ||
    echo "WARNING: e33_mission/$f and e33_calibrate/$f differ (copy one over the other)"
done

BIN=$(mktemp -d)
BUILD="g++ -O2 -std=c++14 -I sim"
$BUILD $DEFS -DROUTE_ORDER=312 -DFIRMWARE_INO="$(inc e33_mission)" -o "$BIN/m312.exe" sim/sim.cpp &
$BUILD $DEFS -DROUTE_ORDER=132 -DFIRMWARE_INO="$(inc e33_mission)" -o "$BIN/m132.exe" sim/sim.cpp &
$BUILD $DEFS -DROBOT_MODE=1 -DFIRMWARE_INO="$(inc e33_calibrate)" -o "$BIN/cal.exe" sim/sim.cpp &
wait
for e in m312 m132 cal; do [ -x "$BIN/$e.exe" ] || { echo "build failed ($e)"; rm -rf "$BIN"; exit 2; }; done

if [ $# -gt 0 ]; then
  CONDS=("$@")
else
  CONDS=(
    "--student"                                   # best estimate of the robot today
    "--student-slow"                              # tired PP3, slower motors
    "--student-fast"                              # fresh PP3, faster motors
    "--student-fast => --student"                 # calibrated on a fresh battery, run later
    "--student-fast => --student-slow"            # ... run on a tired one
    "--student --trimL=0.70"                      # left motor weaker than guessed
    "--student --trimL=0.90"                      # left motor stronger than guessed
    "--student --pivot=-0.8"                      # spins turn about a point behind the axle
    "--student --track=14"                        # a wider chassis (typical 2WD TT kit)
    "--student --noise=25 --pseed=2"              # noisier bar, another physics jitter
    "--student --startx=-3"                       # put down 3 cm behind C1
    "--student --overhang=3"                      # short line stubs past C1 / C4
    "--student-slow --scrub=0.35 --pivotjit=0.6"  # a draggy caster on a tired battery
    "--student-fast --vmax=90"                    # beyond the range: much stronger motors
  )
fi

OUT=$(mktemp -d)
# one mission (route $1, nocal/cal $2, in dir $3, flags $4..) -> its table cell in $3/$2$1.cell
runmission() {
  local r=$1 m=$2 d=$3 ee=""; shift 3
  if [ $m = cal ]; then cp "$d/ee.bin" "$d/ee$r.bin" 2>/dev/null; ee="--eeprom=$d/ee$r.bin"; fi
  "$BIN/m$r.exe" --quiet --path --startx=0 $ee "$@" > "$d/$m$r.txt" 2>/dev/null
  local s=$(sed -n 's/^score \([0-9]\)\/3$/\1/p' "$d/$m$r.txt")
  "$PY" sim/path_check.py --brief < "$d/$m$r.txt" > "$d/$m$r.chk" 2>&1
  local p=$(sed -n 's/^PATH CHECK \([A-Z]*\).*/\1/p' "$d/$m$r.chk")
  case "$p" in PASS) p=ok ;; FAIL) p=FAIL ;; *) p=ERR ;; esac
  printf '%-9s' "${s:-0}/3 $p" > "$d/$m$r.cell"
}
# one condition -> one table row in $OUT/row<i>
runcond() {
  local i=$1 c=$2 cf rf d="$OUT/c$1"
  mkdir -p "$d"
  case "$c" in *"=>"*) cf="${c%%=>*}"; rf="${c#*=>}" ;; *) cf="$c"; rf="$c" ;; esac
  runmission 312 nocal "$d" $rf &
  runmission 132 nocal "$d" $rf &
  "$BIN/cal.exe" --quiet --serial --startx=0 --secs=400 --eeprom="$d/ee.bin" $cf > "$d/cal.txt" 2>&1
  local calst="none"
  grep -q "saved in EEPROM" "$d/cal.txt" && calst="ok"
  grep -q "CALIBRATION STOPPED" "$d/cal.txt" && calst="STOP"
  val() { sed -n "s/^#define CAL_$1 \([0-9]*\).*/\1/p" "$d/cal.txt" | tail -n 1; }
  local v=$(val V_CRUISE_X100) tr=$(val TRIM_R) tl=$(val TRIM_L) dl=$(val DB_LF) dr=$(val DB_RF) t9=$(val T90_L_MS)
  local cruise="-" trim="-" db="-"
  [ -n "$v" ] && cruise=$(awk -v x="$v" 'BEGIN{printf "%.1f", x/100}')
  [ -n "$tr" ] && trim="$tl/$tr"
  [ -n "$dl" ] && db="$dl/$dr"
  runmission 312 cal "$d" $rf &
  runmission 132 cal "$d" $rf &
  wait
  local cells="" x
  for x in nocal312 cal312 nocal132 cal132; do cells="$cells $(cat "$d/$x.cell" 2>/dev/null || printf '%-9s' "0/3 ERR")"; done
  printf "%2d %-44s%s %-5s %-6s %-9s %-6s %s\n" "$((i + 1))" "$c" "$cells" "$calst" "$cruise" "$trim" "$db" "${t9:--}" > "$OUT/row$i"
  # the first failing checks, for the notes under the table
  for f in "$d"/*.chk; do
    grep -q "^PATH CHECK PASS" "$f" || { echo "  [$((i + 1))] $(basename "$f" .chk): $(grep -m 2 "^  FAIL" "$f" | tr -s ' ' | tr '\n' ' ')"; }
  done > "$OUT/why$i"
}

printf "   %-44s %-9s %-9s %-9s %-9s %-5s %-6s %-9s %-6s %s\n" "condition (cal => run)" "312" "312" "132" "132" "cal" "cruise" "trim" "dbF" "t90L"
printf "   %-44s %-9s %-9s %-9s %-9s %-5s %-6s %-9s %-6s %s\n" "" "no-cal" "cal" "no-cal" "cal" "" "cm/s" "L/R" "L/R" "ms"
i=0
for c in "${CONDS[@]}"; do
  runcond $i "$c" &
  i=$((i + 1))
  while [ "$(jobs -rp | wc -l)" -ge "$JOBS" ]; do wait -n 2>/dev/null || sleep 0.2; done
done
wait
n=$i
for ((i = 0; i < n; i++)); do cat "$OUT/row$i"; done
why=$(for ((i = 0; i < n; i++)); do cat "$OUT/why$i"; done)
echo "cells: score/3 and the path check (ok = every count at the right crossing, FAIL = see below)."
echo "cal: the calibration saved (ok) or STOPPED; cruise/trim/dbF/t90L: the CAL VALUES it printed"
echo "     (after a STOP partly the calibration.h defaults: 15.0 cm/s, 850 ms)."
if [ -n "$why" ]; then echo "path check failures (first two lines each; [row] mission):"; printf '%s\n' "$why"; fi
[ -n "${KEEP:-}" ] && { mkdir -p "$KEEP"; cp -r "$OUT"/. "$KEEP"/; }
rm -rf "$OUT" "$BIN"
