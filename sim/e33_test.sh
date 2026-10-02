#!/usr/bin/env bash
# Test the e33 sketches (e33_mission and e33_calibrate) in the simulator: for every condition, run
#   A) the mission with no calibration (defaults in calibration.h), and
#   B) MODE_CALIBRATE on a fresh EEPROM, then the mission using that EEPROM.
#
#   ./sim/e33_test.sh                 # the standard list below
#   ./sim/e33_test.sh "--trimL=0.6 --dbL=50" "--overhang=3"   # your own conditions
#   VERBOSE=1 ./sim/e33_test.sh "--trimL=0.6"                 # show the firmware's trace
#   FW=robot_e33 ./sim/e33_test.sh    # one sketch folder for both (the older all-in-one sketches)
#   (default: the mission from e33_mission, the calibration from e33_calibrate)
#   DEFS="-DROUTE_ORDER=132" ./sim/e33_test.sh   # extra defines for both builds (the other route)
#   GUESS=1 ./sim/e33_test.sh        # no-cal with guessed sensor levels (the standing-still measurement)
# (no-cal = the levels and numbers written in calibration.h: your measured levels, default motors)
#   PATHCHECK=1 ./sim/e33_test.sh     # also check every count of the calibrated mission against
#                                     # the route (sim/path_check.py): a "path" column, and why not
#
# Every run places the robot the way the e33 sketches expect: axle over C1 (--startx=0).
set -u
cd "$(dirname "$0")/.."
BUILD="g++ -O2 -std=c++14 -I sim"
if [ -n "${FW:-}" ]; then MFW="$FW"; CFW="$FW"; else MFW=e33_mission; CFW=e33_calibrate; fi
INO="\"../$MFW/$MFW.ino\""
CINO="\"../$CFW/$CFW.ino\""
# the files both programs share must be the same in both folders
if [ "$MFW" != "$CFW" ]; then
  for f in calibration.h controlLibrary.h pidLibrary.h runLog.h; do
    cmp -s "$MFW/$f" "$CFW/$f" || echo "WARNING: $MFW/$f and $CFW/$f differ (copy one over the other)"
  done
fi
DEFS="${DEFS:-}"
# GUESS=1: the no-cal mission uses guessed levels (250 / 750) instead of the measured ones in
# calibration.h, so it measures the levels itself, standing still
[ "${GUESS:-0}" = "1" ] && GDEFS="-DSIM_GUESS_LEVELS -DCAL_SENSORS_MEASURED=0" || GDEFS=""
$BUILD $DEFS $GDEFS -DFIRMWARE_INO="$INO" -o sim/sim_e33.exe sim/sim.cpp || exit 2
$BUILD $DEFS -DROBOT_MODE=1 -DFIRMWARE_INO="$CINO" -o sim/sim_e33cal.exe sim/sim.cpp || exit 2

if [ $# -gt 0 ]; then
  CONDS=("$@")
else
  CONDS=(
    ""                                              # ideal robot
    "--trimL=0.75"                                  # your robot: left ~75% of right
    "--trimL=0.75 --dbL=45 --dbR=28 --edge=0.8"     # plus dead bands and soft tape edges
    "--trimL=0.75 --trimLr=0.62 --dbL=45 --dbR=28"  # left even weaker going BACKWARD (spins)
    "--trimL=0.70 --vmax=28 --edge=0.8 --noise=25"  # tired battery, noisy sensors
    "--trimL=0.75 --overhang=3 --edge=0.8"          # short lines past C1/C4
    "--trimL=0.80 --cell=22 --edge=0.8"             # smaller field
    "--trimL=0.75 --white=300 --black=650 --edge=0.8"  # weak bar contrast
    "--trimL=0.75 --weakch=2 --edge=0.8"            # one weak sensor
    "--trim=0.80 --edge=0.8"                        # right motor the weak one instead
    "--trimL=0.75 --real"                           # motors with lag, coast, scrub, battery sag
    "--trimL=0.75 --real --tau=150 --pivot=1.5 --scrub=0.35 --sag=0.15"   # ... all of it worse
  )
fi

TMP=$(mktemp -d)
PC="${PATHCHECK:-0}"
if [ "$PC" = "1" ]; then
  printf "%-58s %-10s %-10s %s\n" "condition" "no-cal" "cal+mission" "path"
else
  printf "%-58s %-10s %-10s\n" "condition" "no-cal" "cal+mission"
fi
for c in "${CONDS[@]}"; do
  a=$(./sim/sim_e33.exe --quiet --startx=0 $c 2>/dev/null | sed -n 's/^score \([0-9]\)\/3$/\1/p')
  ee="$TMP/ee.bin"; rm -f "$ee"
  ./sim/sim_e33cal.exe --quiet --startx=0 --secs=400 --eeprom="$ee" $c >/dev/null 2>&1
  if [ "${VERBOSE:-0}" = "1" ]; then
    ./sim/sim_e33.exe --serial --startx=0 --eeprom="$ee" $c 2>&1 | grep -v "^  bar"
  fi
  if [ "$PC" = "1" ]; then
    # the same run with a PATH line at every count (--path only prints)
    out=$(./sim/sim_e33.exe --quiet --path --startx=0 --eeprom="$ee" $c 2>/dev/null)
    b=$(printf '%s\n' "$out" | sed -n 's/^score \([0-9]\)\/3$/\1/p')
    chk=$(printf '%s\n' "$out" | ${PYTHON:-python} sim/path_check.py --brief)
    p=$(printf '%s\n' "$chk" | sed -n 's/^PATH CHECK \([A-Z]*\).*/\1/p')
    printf "%-58s %-10s %-10s %s\n" "${c:-(ideal)}" "${a:-0}/3" "${b:-0}/3" "${p:-ERROR}"
    [ "${p:-}" = "PASS" ] || printf '%s\n' "$chk" | grep "^  FAIL" | head -n 5
  else
    b=$(./sim/sim_e33.exe --quiet --startx=0 --eeprom="$ee" $c 2>/dev/null | sed -n 's/^score \([0-9]\)\/3$/\1/p')
    printf "%-58s %-10s %-10s\n" "${c:-(ideal)}" "${a:-0}/3" "${b:-0}/3"
  fi
done
rm -rf "$TMP"
