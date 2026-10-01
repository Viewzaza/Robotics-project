#!/usr/bin/env bash
# Your robot's real Serial Monitor printouts (sim/student_frames.txt) played back into
# the REAL firmware, the robot standing still, with a PASS or FAIL for each part:
#   1. the probe (sim/replay_probe.cpp): every frame through scanBar() (ON / OFF levels),
#      errorOfMask(), startMission()'s start test, checkGrid(), rawLook() and lightCheck()
#   2. e33_mission, both routes: does it start on [middle] after 0.2 s with no light
#      warning, and never on [white], [black], [lifted] or [crossing]? (the real
#      startMission(), --rawfile) On a fresh Nano, and after a simulated MODE_CALIBRATE
#      (it keeps the whitest reading of each sensor: your whitest white here)
#   3. MODE_METER: what the meter shows for each part
#   4. MODE_SENSOR_CHECK: the four steps fed with your white, line and black frames:
#      is every sensor OK?
#
#   bash sim/replay_test.sh                      # the frames in sim/student_frames.txt
#   bash sim/replay_test.sh my_printouts.txt     # another file in the same format
#   MARGINS=1 bash sim/replay_test.sh            # also the table of margins per sensor
#   VERBOSE=1 bash sim/replay_test.sh            # also every frame of the probe
#   FWROOT=/path/to/tree bash sim/replay_test.sh # the sketches from another tree
#   DEFS="-DSENS_OFF_PCT=30" ...                 # (only if calibration.h allows it)
# The last line says ALL PASS, or how many checks failed (exit code 1).
set -u
cd "$(dirname "$0")/.."
FRAMES="${1:-sim/student_frames.txt}"
FWROOT="${FWROOT:-.}"
DEFS="${DEFS:-}"
[ -f "$FRAMES" ] || { echo "no file $FRAMES"; exit 2; }

inc() {                                  # the path of a sketch as #include sees it from sim/
  case "$FWROOT" in
    /*|[A-Za-z]:*) local p="$FWROOT/$1/$1.ino"
                   command -v cygpath >/dev/null 2>&1 && p=$(cygpath -m "$p")
                   printf '"%s"' "$p" ;;
    *)             printf '"../%s/%s/%s.ino"' "$FWROOT" "$1" "$1" ;;
  esac
}
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
B="g++ -O2 -std=c++14 -I sim $DEFS"
$B -DROUTE_ORDER=312 -DFIRMWARE_INO="$(inc e33_mission)" -o "$T/m312.exe" sim/sim.cpp &
$B -DROUTE_ORDER=132 -DFIRMWARE_INO="$(inc e33_mission)" -o "$T/m132.exe" sim/sim.cpp &
$B -DFIRMWARE_INO="$(inc e33_mission)" -o "$T/probe.exe" sim/replay_probe.cpp &
$B -DROBOT_MODE=2 -DFIRMWARE_INO="$(inc e33_calibrate)" -o "$T/meter.exe" sim/sim.cpp &
$B -DROBOT_MODE=4 -DFIRMWARE_INO="$(inc e33_calibrate)" -o "$T/sc.exe" sim/sim.cpp &
$B -DROBOT_MODE=1 -DFIRMWARE_INO="$(inc e33_calibrate)" -o "$T/cal.exe" sim/sim.cpp &
wait
for e in m312 m132 probe meter sc cal; do [ -x "$T/$e.exe" ] || { echo "build failed ($e)"; exit 2; }; done

PASS=0; FAIL=0
say() {                                  # say PASS|FAIL "text"
  echo "$1  $2"
  if [ "$1" = PASS ]; then PASS=$((PASS + 1)); else FAIL=$((FAIL + 1)); fi
}

# ---- the parts of the file: part_N.txt holds the frame lines, kind_N the first word ----
tr -d '\r' < "$FRAMES" | awk -v dir="$T" '
  /^[ \t]*\[/ { n++; h = $0; sub(/^[ \t]*\[/, "", h); sub(/\].*$/, "", h)
                k = h; sub(/[ :].*$/, "", k)
                print k > (dir "/kind_" n); print h > (dir "/name_" n); close(dir "/kind_" n); close(dir "/name_" n)
                printf "" > (dir "/part_" n); next }
  /^[ \t]*#/ { next }
  n && /raw [0-9]/ { print >> (dir "/part_" n) }
  n && /^[ \t]*[0-9]+([ \t]+[0-9]+){7}[ \t]*$/ { print >> (dir "/part_" n) }
  END { print n + 0 > (dir "/count") }'
N=$(cat "$T/count")
[ "$N" -gt 0 ] || { echo "no [part] headers in $FRAMES"; exit 2; }
# all frames of one kind, in file order (for the sensor check)
kindframes() { local i; for i in $(seq 1 "$N"); do [ "$(cat "$T/kind_$i")" = "$1" ] && cat "$T/part_$i"; done; }

echo "=== 1. the probe: each frame through the firmware's own functions"
if [ "${MARGINS:-0}" = "1" ]; then "$T/probe.exe" "$FRAMES" --margins > "$T/probe.out" 2>&1
else "$T/probe.exe" "$FRAMES" > "$T/probe.out" 2>&1; fi
if [ "${VERBOSE:-0}" = "1" ]; then cat "$T/probe.out"
else
  sed -n '1,10p' "$T/probe.out"
  grep -E "^(PASS|FAIL)|^      \(with" "$T/probe.out"
  [ "${MARGINS:-0}" = "1" ] && sed -n '/^MARGINS/,/^  A7/p' "$T/probe.out"
fi
PASS=$((PASS + $(grep -c '^PASS' "$T/probe.out")))
FAIL=$((FAIL + $(grep -c '^FAIL' "$T/probe.out")))
grep -q "^probe:" "$T/probe.out" || { echo "FAIL  the probe did not finish"; FAIL=$((FAIL + 1)); }

echo
# white first (0.5 s), then the part, each frame 0.1 s: a start must come 0.2 to 0.4 s
# after the line came (the 3rd or 4th frame of the part), with no light warning, and
# never on anything else. $1: an EEPROM file to start from ("" = a fresh Nano), $2: a label
kindframes white > "$T/wh.txt"; WHITE1=$(head -n 1 "$T/wh.txt")
[ -n "$WHITE1" ] || WHITE1="75 72 66 70 68 69 72 75"
startcheck() {
  local ee="$1" lab="$2" i k name r out f t msg want eeflag
  for i in $(seq 1 "$N"); do
    k=$(cat "$T/kind_$i"); name=$(cat "$T/name_$i")
    [ -s "$T/part_$i" ] || continue
    case "$k" in white|black|middle|lifted|crossing) ;; *) echo "(part [$name]: first word not known, skipped)"; continue ;; esac
    for r in 312 132; do
      { for w in 1 2 3 4 5; do echo "$WHITE1"; done; cat "$T/part_$i"; } > "$T/seq.txt"
      eeflag=""; [ -n "$ee" ] && { cp "$ee" "$T/run.bin"; eeflag="--eeprom=$T/run.bin"; }
      if [ "$k" = middle ]; then
        out=$("$T/m$r.exe" --serial --startx=0 --still=1000 --rawhold=100 --rawfile="$T/seq.txt" $eeflag 2>&1)
        # the frame being read when it said go (frames 1-5 are the white ones)
        f=$(printf '%s\n' "$out" | sed -n 's/.*RAW     frame \([0-9]*\) of.*/\1/p; /^MISSION: go/q' | tail -n 1)
        printf '%s\n' "$out" | grep -q "^MISSION: go" || f=""
        if [ -z "$f" ]; then say FAIL "$lab route $r [$name]: never started"
        else
          t=$((f - 5))
          if [ "$t" -ge 3 ] && [ "$t" -le 4 ]; then say PASS "$lab route $r [$name]: starts on frame $t of the line (0.2 to 0.4 s)"
          else say FAIL "$lab route $r [$name]: started on frame $t of the line (0.1 s each; 3 or 4 expected)"; fi
          if printf '%s\n' "$out" | grep -q "WARNING: not the light"; then
            say FAIL "$lab route $r [$name]: a light warning at the start (the white beside the line is not the calibrated white)"
          else say PASS "$lab route $r [$name]: no light warning"; fi
        fi
      else
        out=$("$T/m$r.exe" --serial --startx=0 --still=1000 --rawhold=1000 --rawfile="$T/seq.txt" $eeflag 2>&1)
        if printf '%s\n' "$out" | grep -q "^MISSION: go"; then say FAIL "$lab route $r [$name]: it STARTED"
        else
          # what the waiting screen said about the last frame
          msg=$(printf '%s\n' "$out" | grep "^  bar " | tail -n 1 | sed 's/.*raw\( [0-9]*\)\{8\}  //')
          want="no line in the middle"; [ "$k" = white ] || want="on a crossing, or lifted"
          case "$msg" in "$want"*) say PASS "$lab route $r [$name]: no start (\"$want\")" ;;
                         *)        say FAIL "$lab route $r [$name]: no start, but it said \"$msg\"" ;; esac
        fi
      fi
    done
  done
}
echo "=== 2. e33_mission's start (startMission), both routes"
echo "--- a fresh Nano (the levels in calibration.h)"
startcheck "" "fresh:"
# MODE_CALIBRATE keeps the whitest reading of each sensor as its white: the simulated
# robot (--student) with your whitest white and your black, calibrated, then your frames
kindframes black > "$T/blk.txt"; [ -s "$T/blk.txt" ] || kindframes crossing > "$T/blk.txt"
if [ -s "$T/wh.txt" ] && [ -s "$T/blk.txt" ]; then
  CH=$(tr -d '\r' < "$T/wh.txt" | sed 's/.*raw //' | awk '{ for (i = 1; i <= 8; i++) if (NR == 1 || $i < w[i]) w[i] = $i }
       END { for (i = 1; i <= 8; i++) printf "%d ", w[i] }')
  CB=$(tr -d '\r' < "$T/blk.txt" | sed 's/.*raw //' | awk '{ for (i = 1; i <= 8; i++) b[i] += $i; n++ }
       END { for (i = 1; i <= 8; i++) printf "%d ", b[i] / n + 0.5 }')
  CHAN=$(awk -v w="$CH" -v b="$CB" 'BEGIN { split(w, W); split(b, B); for (i = 1; i <= 8; i++) printf "--chan=%d:%d:%d ", i - 1, W[i], B[i] }')
  echo "--- after MODE_CALIBRATE (simulated: --student $CHAN)"
  "$T/cal.exe" --quiet --startx=0 --secs=400 --eeprom="$T/cal.bin" --student $CHAN > "$T/cal.out" 2>&1
  if [ -s "$T/cal.bin" ]; then startcheck "$T/cal.bin" "calibrated:"
  else say FAIL "the simulated calibration saved nothing"; fi
fi

echo
echo "=== 3. MODE_METER (what it shows for each part)"
for i in $(seq 1 "$N"); do
  k=$(cat "$T/kind_$i"); name=$(cat "$T/name_$i")
  [ -s "$T/part_$i" ] || continue
  out=$("$T/meter.exe" --serial --quiet --still=10000 --rawhold=1000 --rawfile="$T/part_$i" 2>&1 | grep "^raw ")
  shown=$(printf '%s\n' "$out" | sed 's/^raw\( [0-9]*\)\{8\} | //' | sort | uniq -c | sort -rn | head -n 3 | sed 's/^ *//' | tr '\n' ';')
  bad=0
  case "$k" in
    white)    printf '%s\n' "$out" | grep -qv "| 00000000 error none | line at -$" && bad=1 ;;
    black|crossing) printf '%s\n' "$out" | grep -qv "| 11111111 .*| CROSSING$" && bad=1 ;;
    middle)   printf '%s\n' "$out" | grep -qvE "\| 000(11|01|10)000 error (0|1|-1) \| line at 3\.[0-9]+$" && bad=1 ;;
    lifted)   printf '%s\n' "$out" | grep -qv "| 11111111 " && bad=1 ;;
    *) continue ;;
  esac
  [ -n "$out" ] || bad=1
  if [ $bad = 0 ]; then say PASS "meter [$name]: ${shown%;}"; else say FAIL "meter [$name]: ${shown%;}"; fi
done

echo
echo "=== 4. MODE_SENSOR_CHECK, fed with your frames (one run for each [white] part)"
# The check reads for about 80 s from its first reading (frames of 1 s): step 1 (white) at
# frames 1-2, step 2 (the sweep by hand) 22-36, step 3 (line in the middle) 57, step 4 (bar
# along a line: all black) 78. Step 2 was not in the printouts: it gets the middle, black
# and white frames one after the other (every sensor then sees black once).
kindframes middle > "$T/mid.txt"; kindframes black > "$T/blk.txt"
[ -s "$T/blk.txt" ] || kindframes crossing > "$T/blk.txt"
if [ ! -s "$T/mid.txt" ] || [ ! -s "$T/blk.txt" ]; then
  echo "(needs a [middle] and a [black] part: skipped)"
else
  cyc() { local n=$1 f=$2; while [ "$n" -gt 0 ]; do cat "$f"; n=$((n - $(wc -l < "$f"))); done | head -n "$1"; }
  for i in $(seq 1 "$N"); do
    [ "$(cat "$T/kind_$i")" = white ] && [ -s "$T/part_$i" ] || continue
    name=$(cat "$T/name_$i")
    { cyc 21 "$T/part_$i"; cat "$T/mid.txt" "$T/blk.txt" "$T/part_$i" > "$T/sw.txt"; cyc 35 "$T/sw.txt"
      cyc 21 "$T/mid.txt"; cyc 25 "$T/blk.txt"; } > "$T/sc.txt"
    out=$("$T/sc.exe" --serial --quiet --still=10000 --secs=1000 --rawhold=1000 --rawfile="$T/sc.txt" 2>&1 |
          sed -n '/REPORT BEGIN/,/REPORT END/p')
    [ -n "$out" ] || { say FAIL "sensor check with [$name]: no report"; continue; }
    [ "${VERBOSE:-0}" = "1" ] && printf '%s\n' "$out"
    v=$(printf '%s\n' "$out" | grep "^A[0-7]," | awk -F, '{ printf "%s %s/%s %s %s; ", $1, $2, $5, $9, $10 }')
    nok=$(printf '%s\n' "$out" | grep -c "^A[0-7],.*,yes,OK$")
    if [ "$nok" = 8 ]; then say PASS "sensor check with [$name]: all 8 OK and seen with the levels in use"
    else say FAIL "sensor check with [$name]: only $nok of 8 OK and seen"; fi
    echo "      (white/black, sees_line_now, verdict) $v"
    printf '%s\n' "$out" | grep -q "^black reads HIGH" && say PASS "sensor check with [$name]: black reads HIGH (CAL_LINE_LOW 0)" ||
      say FAIL "sensor check with [$name]: $(printf '%s\n' "$out" | grep '^black reads')"
    s3=$(printf '%s\n' "$out" | grep "^step 3")
    case "$s3" in *good) say PASS "sensor check with [$name]: ${s3}" ;; *) say FAIL "sensor check with [$name]: ${s3}" ;; esac
  done
fi

echo
if [ "$FAIL" = 0 ]; then echo "ALL PASS ($PASS checks)"; exit 0
else echo "$FAIL FAIL, $PASS PASS"; exit 1; fi
