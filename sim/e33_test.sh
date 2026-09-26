#!/usr/bin/env bash
# Test robot_e33 in the simulator: for every condition, run
#   A) the mission with no calibration (defaults in calibration.h), and
#   B) MODE_CALIBRATE on a fresh EEPROM, then the mission using that EEPROM.
#
#   ./sim/e33_test.sh                 # the standard list below
#   ./sim/e33_test.sh "--trimL=0.6 --dbL=50" "--overhang=3"   # your own conditions
#   VERBOSE=1 ./sim/e33_test.sh "--trimL=0.6"                 # show the firmware's trace
#
# Every run places the robot the way robot_e33 expects: axle over C1 (--startx=0).
set -u
cd "$(dirname "$0")/.."
BUILD="g++ -O2 -std=c++14 -I sim"
$BUILD -DFIRMWARE_INO='"../robot_e33/robot_e33.ino"' -o sim/sim_e33.exe sim/sim.cpp || exit 2
$BUILD -DROBOT_MODE=1 -DFIRMWARE_INO='"../robot_e33/robot_e33.ino"' -o sim/sim_e33cal.exe sim/sim.cpp || exit 2

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
  )
fi

TMP=$(mktemp -d)
printf "%-58s %-10s %-10s\n" "condition" "no-cal" "cal+mission"
for c in "${CONDS[@]}"; do
  a=$(./sim/sim_e33.exe --quiet --startx=0 $c 2>/dev/null | sed -n 's/^score \([0-9]\)\/3$/\1/p')
  ee="$TMP/ee.bin"; rm -f "$ee"
  ./sim/sim_e33cal.exe --quiet --startx=0 --secs=400 --eeprom="$ee" $c >/dev/null 2>&1
  if [ "${VERBOSE:-0}" = "1" ]; then
    ./sim/sim_e33.exe --serial --startx=0 --eeprom="$ee" $c 2>&1 | grep -v "^  bar"
  fi
  b=$(./sim/sim_e33.exe --quiet --startx=0 --eeprom="$ee" $c 2>/dev/null | sed -n 's/^score \([0-9]\)\/3$/\1/p')
  printf "%-58s %-10s %-10s\n" "${c:-(ideal)}" "${a:-0}/3" "${b:-0}/3"
done
rm -rf "$TMP"
