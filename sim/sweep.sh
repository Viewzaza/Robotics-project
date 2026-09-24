#!/usr/bin/env bash
# Robustness sweep: run the mission across a grid of plausible real-world
# conditions and report how many complete all three placements.
#
#   ./sim/sweep.sh            # full sweep
#   ./sim/sweep.sh quick      # smaller grid
set -u
cd "$(dirname "$0")/.."

SIM=./sim/sim.exe
[ -x "$SIM" ] || SIM=./sim/sim
if [ ! -x "$SIM" ]; then echo "build first: g++ -O2 -std=c++14 -I sim -o sim/sim.exe sim/sim.cpp"; exit 2; fi

if [ "${1:-}" = "quick" ]; then
  TRIMS="0.94 1.00 1.06"; VMAXS="30 40"; CELLS="25"; GRIPS="12"
else
  TRIMS="0.90 0.94 0.97 1.00 1.03 1.06 1.10"   # left/right motor mismatch
  VMAXS="25 30 35 40 50"                        # battery fresh vs sagging
  CELLS="20 25 30"                              # unknown real grid pitch
  GRIPS="11 12 13"                              # gripper reach uncertainty
fi

pass=0; total=0; FAILLOG=$(mktemp)
for t in $TRIMS; do for v in $VMAXS; do for c in $CELLS; do for g in $GRIPS; do
  total=$((total+1))
  score=$("$SIM" --quiet --secs=200 --trim=$t --vmax=$v --cell=$c --grip=$g 2>/dev/null \
          | sed -n 's/^score \([0-9]\)\/3$/\1/p')
  score=${score:-0}
  if [ "$score" = "3" ]; then pass=$((pass+1)); else
    echo "trim=$t vmax=$v cell=$c grip=$g -> $score/3" >> "$FAILLOG"
  fi
done; done; done; done

echo "=============================================="
echo "MISSION COMPLETE IN $pass / $total CONDITIONS"
nf=$(wc -l < "$FAILLOG" 2>/dev/null || echo 0)
if [ "$nf" -gt 0 ]; then
  echo "--- failures (showing up to 25 of $nf) ---"
  head -25 "$FAILLOG" | sed 's/^/  /'
fi
rm -f "$FAILLOG"
echo "=============================================="
[ "$pass" = "$total" ]
