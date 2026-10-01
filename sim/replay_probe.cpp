// sim/replay_probe.cpp: the student's real sensor printouts (sim/student_frames.txt)
// through the REAL firmware functions, one frame at a time, the robot standing still.
// It includes sim/sim.cpp (and so e33_mission.ino with its calibration.h) and calls
// scanBar(), errorOfMask(), checkGrid(), rawLook() and lightCheck() on each frame.
// sim/replay_test.sh builds and runs it; by hand:
//   g++ -O2 -std=c++14 -I sim -DFIRMWARE_INO='"../e33_mission/e33_mission.ino"' \
//       -o probe.exe sim/replay_probe.cpp
//   ./probe.exe sim/student_frames.txt            (a PASS or FAIL line for each check)
//   ./probe.exe sim/student_frames.txt --margins  (and the table of margins per sensor)
// The first word of each [part] header says what the frames must give (see the file).
#define main sim_main
#include "sim.cpp"
#undef main

struct Part { std::string name, kind; std::vector<RawFrame> f; };

static std::vector<Part> readParts(const char* path) {
  std::vector<Part> ps;
  FILE* fp = fopen(path, "r");
  if (!fp) { fprintf(stderr, "probe: cannot open %s\n", path); exit(2); }
  char buf[512]; int ln = 0;
  while (fgets(buf, sizeof buf, fp)) {
    ln++;
    char* s = buf;
    while (*s == ' ' || *s == '\t') s++;
    if (*s == '#') continue;
    if (*s == '[') {
      char* e = strchr(s, ']');
      Part p; p.name.assign(s + 1, e ? e : s + 1);
      size_t k = p.name.find_first_of(" :");
      p.kind = p.name.substr(0, k);
      ps.push_back(p);
      continue;
    }
    char* q = strstr(s, "raw ");
    char* p = q ? q + 4 : s;
    RawFrame fr; fr.line = ln; int got = 0;
    for (; got < 8; got++) {
      char* r; long x = strtol(p, &r, 10);
      if (r == p || x < 0 || x > 1023) break;
      fr.v[got] = (int)x; p = r;
    }
    if (got != 8 || !(*p == 0 || *p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) {
      if (q) fprintf(stderr, "probe: line %d skipped (not 8 readings)\n", ln);
      continue;
    }
    if (ps.empty()) { Part x; x.name = x.kind = "(no header)"; ps.push_back(x); }
    ps.back().f.push_back(fr);
  }
  fclose(fp);
  return ps;
}

static void feed(const RawFrame& fr) {      // every analogRead now returns this frame
  w_raw.assign(1, fr); w_rawT0 = -1; w_rawHoldMs = 1e15;
}
static std::string maskStr(uint8_t m) {
  std::string s; for (int i = 0; i < 8; i++) s += (m & (0x80 >> i)) ? '1' : '0'; return s;
}
/* the bar as the firmware sees it, from "not lit" (ON levels) or from "all lit" (OFF levels) */
static uint8_t scanFrom(uint8_t before) { lineMask = before; return scanBar(); }
/* the same with another SENS_OFF_PCT (only to compare; the firmware uses its own) */
static uint8_t heldMaskAt(const RawFrame& fr, int offPct) {
  uint8_t m = 0;
  for (int i = 0; i < 8; i++) {
    int lo = cal.lo[i], hi = cal.hi[i];
    int off = (hi <= lo + 20) ? 480 : lo + (int)((long)(hi - lo) * offPct / 100);
    int v = cal.lineLow ? 1023 - fr.v[i] : fr.v[i];
    if (v >= off && !(cal.deadMask & (0x80 >> i))) m |= 0x80 >> i;
  }
  return m;
}
/* startMission()'s test of a plain line under the middle (the same expression) */
static bool startRule(uint8_t m) {
  uint8_t b = popcount8(m);
  return b >= 1 && b <= 3 && isRun(m) && abs(errorOfMask(m)) <= 5;
}

static int nPass = 0, nFail = 0;
static void verdict(bool ok, const std::string& part, const char* what, const std::string& why) {
  printf("%s  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", part.c_str(), what, why.empty() ? "" : ": ", why.c_str());
  (ok ? nPass : nFail)++;
}

int main(int argc, char** argv) {
  const char* path = "sim/student_frames.txt";
  bool margins = false;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--margins")) margins = true;
    else path = argv[i];
  }
  w_verbose = false; w_stillSec = 1e12; w_maxSimSec = 1e12;
  for (int i = 0; i < 24; i++) { w_digital[i] = 0; w_pwm[i] = 0; w_servoAngle[i] = 90; }
  buildField();
  std::vector<Part> ps = readParts(path);
  if (ps.empty()) { fprintf(stderr, "probe: no frames\n"); return 2; }
  RawFrame none = {{0, 0, 0, 0, 0, 0, 0, 0}, 0};
  feed(ps[0].f.empty() ? none : ps[0].f[0]);
  beginFnc();                                   // the firmware's own start: calLoad, calApply
  printf("levels: CAL_LINE_LOW %d, SENS_ON_PCT %d, SENS_OFF_PCT %d (source %d)\n",
         (int)cal.lineLow, SENS_ON_PCT, SENS_OFF_PCT, (int)calSource);
  printf("  ch   white  black  contrast   ON   OFF   OFF@30\n");
  for (int i = 0; i < 8; i++) {
    int lo = cal.lo[i], hi = cal.hi[i];
    printf("  A%d  %5d  %5d  %6d    %4d  %4d  %4d\n", i, lo, hi, hi - lo, onLvl[i], offLvl[i],
           lo + (int)((long)(hi - lo) * 30 / 100));
  }

  // per sensor, the worst reading of each kind (raw counts)
  int wMax[8], nMax[8], lMin[8], bMin[8], uMin[8];
  for (int i = 0; i < 8; i++) { wMax[i] = nMax[i] = -1; lMin[i] = bMin[i] = uMin[i] = 9999; }

  for (size_t p = 0; p < ps.size(); p++) {
    Part& P = ps[p];
    if (P.f.empty()) continue;
    const std::string& k = P.kind;
    bool white = k == "white", black = k == "black" || k == "crossing", middle = k == "middle",
         lifted = k == "lifted";
    printf("\n[%s]  %d frames\n", P.name.c_str(), (int)P.f.size());
    printf("  line  raw A0..A7                        fresh    held     held@30  err start grid rawLook\n");
    bool okFresh = true, okHeld = true, okHeld30 = true, okErr = true, okStart = true, okGrid = true,
         okLook = true, okLight = true;
    std::string whyFresh, whyHeld, whyHeld30, whyLook, whyLight;
    for (size_t j = 0; j < P.f.size(); j++) {
      const RawFrame& fr = P.f[j];
      feed(fr);
      uint8_t held = scanFrom(0xFF);
      uint8_t held30 = heldMaskAt(fr, 30);
      uint8_t fresh = scanFrom(0x00);
      int err = errorOfMask(fresh);
      bool st = startRule(fresh);
      lineMask = fresh;
      bool grid = checkGrid();                  // (it scans again: the same frame)
      RawLook r; memset(&r, 0, sizeof r);
      bool look = rawLook(r);
      char buf[64];
      printf("  %4d ", fr.line);
      for (int i = 0; i < 8; i++) printf(" %4d", fr.v[i]);
      printf("  %s %s %s %4d  %-3s  %-3s  ", maskStr(fresh).c_str(), maskStr(held).c_str(),
             maskStr(held30).c_str(), err == 100 ? -99 : err, st ? "yes" : "no", grid ? "yes" : "no");
      if (look) printf("line %s white %d black %d gap %d\n", maskStr(r.mask).c_str(), r.white, r.black, r.gap);
      else printf("no (gap %d%s%s)\n", r.gap, r.wrongSide ? ", wrong side" : "", (r.gap >= 120 && r.white > 600) ? ", lifted" : "");
      snprintf(buf, sizeof buf, "line %d gives %s", fr.line, maskStr(fresh).c_str());
      if (white) {
        if (fresh) { okFresh = false; whyFresh = buf; }
        if (held) { okHeld = false; whyHeld = "line " + std::to_string(fr.line) + ": " + maskStr(held); }
        if (held30) { okHeld30 = false; whyHeld30 = "line " + std::to_string(fr.line) + ": " + maskStr(held30); }
        if (st) okStart = false;
        if (grid) okGrid = false;
        if (look) { okLook = false; whyLook = buf; }
        for (int i = 0; i < 8; i++) if (fr.v[i] > wMax[i]) wMax[i] = fr.v[i];
      } else if (black) {
        if (fresh != 0xFF) { okFresh = false; whyFresh = buf; }
        if (st) okStart = false;
        if (!grid) okGrid = false;
        if (look) { okLook = false; whyLook = buf; }
        for (int i = 0; i < 8; i++) if (fr.v[i] < bMin[i]) bMin[i] = fr.v[i];
      } else if (lifted) {
        if (popcount8(fresh) < 5) { okFresh = false; whyFresh = buf; }
        if (st) okStart = false;
        if (look) { okLook = false; whyLook = buf; }
        for (int i = 0; i < 8; i++) if (fr.v[i] < uMin[i]) uMin[i] = fr.v[i];
      } else if (middle) {
        if (!(fresh && popcount8(fresh) <= 3 && isRun(fresh) && abs(err) <= 1)) { okFresh = false; whyFresh = buf; }
        if (held != fresh) { okHeld = false; whyHeld = "line " + std::to_string(fr.line) + ": " + maskStr(held); }
        if (held30 != fresh) { okHeld30 = false; whyHeld30 = "line " + std::to_string(fr.line) + ": " + maskStr(held30); }
        if (abs(err) > 1) okErr = false;
        if (!st) okStart = false;
        if (grid) okGrid = false;
        if (!look || r.mask != fresh) { okLook = false; whyLook = look ? "another mask " + maskStr(r.mask) : "no line found"; }
        for (int i = 0; i < 8; i++) {
          if (fresh & (0x80 >> i)) { if (fr.v[i] < lMin[i]) lMin[i] = fr.v[i]; }
          else if (fr.v[i] > nMax[i]) nMax[i] = fr.v[i];
        }
        /* lightCheck() as startMission() calls it after the start: the white beside the line */
        lineMask = fresh; scanFrom(fresh);
        /* (a warning flickers the LED for 1 s: that time is how it is seen here) */
        double t0 = w_t_us;
        int d = lightCheck(true);
        bool warned = w_t_us - t0 > 500000.0;
        printf("        lightCheck: %s (the white beside the line is %+d from the calibration's, on average)\n",
               warned ? "WARNING: not the light of the calibration" : "no warning", d);
        if (warned) { okLight = false; whyLight = "line " + std::to_string(fr.line); }
      }
    }
    if (white) {
      verdict(okFresh, P.name, "every sensor reads white (bar 00000000)", whyFresh);
      verdict(okHeld, P.name, "a sensor that saw the line goes off again here (OFF level)", whyHeld);
      printf("      (with SENS_OFF_PCT 30: %s%s)\n", okHeld30 ? "also all off" : "NOT all off, ", whyHeld30.c_str());
      verdict(okStart && okGrid, P.name, "no start and no crossing", "");
      verdict(okLook, P.name, "rawLook finds no line", whyLook);
    } else if (black) {
      verdict(okFresh, P.name, "all 8 read black (bar 11111111)", whyFresh);
      verdict(okGrid, P.name, "checkGrid: a crossing", "");
      verdict(okStart, P.name, "never starts on it", "");
      verdict(okLook, P.name, "rawLook finds no single line", whyLook);
    } else if (lifted) {
      verdict(okFresh, P.name, "5 or more lit (not a plain line)", whyFresh);
      verdict(okStart, P.name, "never starts on it", "");
      verdict(okLook, P.name, "rawLook refuses it (lifted)", whyLook);
    } else if (middle) {
      verdict(okFresh, P.name, "a plain line under the middle (error 0 or +-1)", whyFresh);
      verdict(okHeld, P.name, "the sensors beside the line go off again (OFF level)", whyHeld);
      printf("      (with SENS_OFF_PCT 30: %s%s)\n", okHeld30 ? "also" : "NOT, ", whyHeld30.c_str());
      verdict(okStart, P.name, "the start rule says go", "");
      verdict(okGrid, P.name, "checkGrid: not a crossing", "");
      verdict(okLight, P.name, "lightCheck: no light warning", whyLight);
      verdict(okLook, P.name, "rawLook finds the same line", whyLook);
    } else {
      printf("      (header word \"%s\" is not white/black/middle/lifted/crossing: nothing checked)\n", k.c_str());
    }
  }

  if (margins) {
    printf("\nMARGINS, raw counts and share of each sensor's contrast (black minus white).\n");
    printf("A white sensor must stay below ON (and, after the line left it, below OFF);\n");
    printf("a sensor on the line must be above ON. + = on the safe side.\n");
    printf("  ch  contrast | white max  to ON      to OFF     to OFF@30 | beside line max  to OFF     to OFF@30 | line min  over ON  | black min over ON  | lifted min over ON\n");
    for (int i = 0; i < 8; i++) {
      int lo = cal.lo[i], hi = cal.hi[i], c = hi - lo;
      int on = onLvl[i], off = offLvl[i], off30 = lo + (int)((long)c * 30 / 100);
      auto pc = [&](int d) { return 100.0 * d / c; };
      printf("  A%d  %5d    |", i, c);
      if (wMax[i] >= 0) printf(" %5d  %+5d %4.0f%%  %+5d %4.0f%%  %+5d %4.0f%% |", wMax[i], on - wMax[i], pc(on - wMax[i]),
                              off - wMax[i], pc(off - wMax[i]), off30 - wMax[i], pc(off30 - wMax[i]));
      else printf("%41s|", "-");
      if (nMax[i] >= 0) printf("     %5d       %+5d %4.0f%%  %+5d %4.0f%% |", nMax[i], off - nMax[i], pc(off - nMax[i]),
                              off30 - nMax[i], pc(off30 - nMax[i]));
      else printf("%39s|", "(on the line)");
      if (lMin[i] < 9999) printf(" %5d  %+5d %4.0f%% |", lMin[i], lMin[i] - on, pc(lMin[i] - on));
      else printf("%20s|", "-");
      if (bMin[i] < 9999) printf(" %5d  %+5d %4.0f%% |", bMin[i], bMin[i] - on, pc(bMin[i] - on));
      else printf("%20s|", "-");
      if (uMin[i] < 9999) printf(" %5d  %+5d %4.0f%%\n", uMin[i], uMin[i] - on, pc(uMin[i] - on));
      else printf("%20s\n", "-");
    }
  }
  printf("\nprobe: %d PASS, %d FAIL\n", nPass, nFail);
  fflush(stdout);
  return nFail ? 1 : 0;
}
