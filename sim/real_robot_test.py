#!/usr/bin/env python3
"""real_robot_test.py: run one sketch against the real-robot conditions (sim/real_robot_conds.txt).

Normally started by sim/real_robot_test.sh SKETCHDIR [ROUTE]; see that file and README.md.

  python sim/real_robot_test.py SKETCHDIR [ROUTE] [--conds FILE] [--out DIR] [--cal auto|0|1] [-j N]
                                [--only NAME,NAME] [--keep-exe]

ROUTE: 312 (default, the student's), 132, or both.
Columns of the table:
  score      objects placed (the sim's "score N/3"), uncalibrated: what the student runs
  fault (n)  the first fault the firmware printed and its count (F5 n=9), or, without one:
             "no start" (never drove), "stuck n=K" (stood still without a fault), "time cap n=K"
             (still driving at the sim's time cap), "finished n=42" (counted to the end, but the
             objects are not on their targets), "brown-out n=K" (the supply fell under --bod and
             the run ended there). "-" when it placed all 3, or when it stopped on purpose after
             a restart (the restart column says so). The first fault is shown even when the
             firmware carried on after it (V3: STOP_ON_FAULT 0).
  restart    what the firmware did after each mid-run restart, with the count it was at:
             halted@K (did not drive again), resumed@K (drove on with its count),
             restarted@K (drove again from count 0, as if at the start); "-" = no restart
  path       sim/path_check.py: PASS; n=K = the first count made at a wrong crossing, or a pick or
             place that took nothing; ~n=K = only a pose outside the check's tolerances (for
             example a timed 180 that ends 3 cm to the side); "-" = it only stopped early
  cal, cal fault   the same run after the sketch's own calibration (MODE_CALIBRATE) on that
             condition's robot (e33_mission: e33_calibrate beside it). --cal auto: only when
             there is an e33_calibrate folder beside the sketch.
"""
import argparse, os, re, shutil, subprocess, sys, tempfile, hashlib, threading
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
FINISH = {'312': 42, '132': 33}                  # the last count of each route

# ---------------------------------------------------------------- conditions
def load_conds(path):
    macros, conds = {}, []
    for raw in open(path, encoding='utf-8'):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        m = re.match(r'@(\w+)\s*=\s*(.*)$', line)
        if m:
            macros[m.group(1)] = expand(m.group(2), macros)
            continue
        parts = [p.strip() for p in line.split('|')]
        if len(parts) < 3:
            print('real_robot_test: skipped a bad line in %s: %s' % (path, line), file=sys.stderr)
            continue
        name, routes, flags = parts[0], parts[1], parts[2]
        why = parts[3] if len(parts) > 3 else ''
        conds.append({'name': name, 'routes': [r.strip() for r in routes.split(',') if r.strip()],
                      'flags': expand(flags, macros).split(), 'why': why})
    return conds

def expand(s, macros):
    return re.sub(r'\$(\w+)', lambda m: macros.get(m.group(1), m.group(0)), s)

# flags that are events of the mission run, not the robot: left out of the calibration run
EVENT_FLAGS = ('--reset=', '--bor=', '--resetcause=', '--maxresets=', '--erase=', '--patch=', '--wipe=',
               '--startx=', '--starty=', '--starth=', '--objoff=', '--lift=', '--wshift=', '--bshift=',
               '--wzone=', '--secs=', '--eeprom=')

def robot_flags(flags):
    return [f for f in flags if not f.startswith(EVENT_FLAGS)]

# ---------------------------------------------------------------- the sketch
def sketch_info(sketch_dir):
    sk = os.path.abspath(sketch_dir).rstrip('/\\')
    name = os.path.basename(sk)
    ino = os.path.join(sk, name + '.ino')
    if not os.path.isfile(ino):                  # a copy in a folder of another name: its one .ino
        inos = [f for f in os.listdir(sk) if f.endswith('.ino')] if os.path.isdir(sk) else []
        if len(inos) != 1:
            sys.exit('real_robot_test: no %s (and not exactly one .ino in that folder)' % ino)
        ino = os.path.join(sk, inos[0])
        name = inos[0][:-4]
    src = ''
    for f in sorted(os.listdir(sk)):
        if f.endswith(('.ino', '.h', '.cpp')):
            src += open(os.path.join(sk, f), encoding='utf-8', errors='replace').read() + '\n'
    has_route = 'ROUTE_ORDER' in src
    # .noinit variables the restart must keep (runMark, and V3's keep / gripNow / armNow ...)
    names = re.findall(r'\b(\w+)\s*(?:\[[^\]]*\])?\s+(?:NOINIT|__attribute__\s*\(\(\s*section\s*\(\s*"\.noinit"\s*\)\s*\)\))\s*;', src)
    keep = []
    for n in names:
        if n not in ('resetFlags', 'resetR2') and n not in keep:
            keep.append(n)
    cal_ino = None
    sib = os.path.join(os.path.dirname(sk), 'e33_calibrate', 'e33_calibrate.ino')
    if os.path.isfile(sib):
        cal_ino = ('sibling', sib)
    elif re.search(r'#define\s+MODE_CALIBRATE\b', src):
        cal_ino = ('self', ino)
    return {'dir': sk, 'name': name, 'ino': ino, 'has_route': has_route, 'noinit': keep, 'cal': cal_ino}

def build(sim_dir, ino, out_exe, defs):
    cmd = ['g++', '-O2', '-std=c++14', '-I', sim_dir] + defs + \
          ['-DFIRMWARE_INO="%s"' % ino.replace('\\', '/'), '-o', out_exe, os.path.join(sim_dir, 'sim.cpp')]
    r = subprocess.run(cmd, capture_output=True, text=True, errors='replace')
    if r.returncode != 0:
        sys.stderr.write(r.stderr[-4000:])
        sys.exit('real_robot_test: build failed: %s' % ' '.join(cmd))

# ---------------------------------------------------------------- one run
# A mid-run restart starts a fresh copy of the sim and the old one waits for it, so a run with
# restarts is a chain of processes: each run takes as many of the -j slots as it can have processes.
class Slots:
    def __init__(self, n):
        self.n, self.free, self.cv = n, n, threading.Condition()
    def take(self, k):
        k = min(k, self.n)
        with self.cv:
            while self.free < k:
                self.cv.wait()
            self.free -= k
        return k
    def give(self, k):
        with self.cv:
            self.free += k
            self.cv.notify_all()
SLOTS = Slots(4)

def chain_len(args):
    n = 1 + sum(1 for a in args if a.startswith('--reset='))
    if '--bor=restart' in args:
        m = [int(a.split('=', 1)[1]) for a in args if a.startswith('--maxresets=')]
        n += m[-1] if m else 4
    return n

def run_exe(exe, args, cwd):
    os.makedirs(cwd, exist_ok=True)
    k = SLOTS.take(chain_len(args))
    try:
        r = subprocess.run([exe] + args, capture_output=True, text=True, errors='replace', cwd=cwd)
    finally:
        SLOTS.give(k)
    return r.stdout + r.stderr

POSE_RE = re.compile(r'pose=\(\s*([-\d.]+),\s*([-\d.]+)\)\s*heading=\s*([-\d.]+)deg')
PATH_RE = re.compile(r'^\[\s*([\d.]+)s\] PATH\s+numGride=(-?\d+)\s+axle=\(\s*([-\d.]+),\s*([-\d.]+)\)')

def summarize(out, route, pathcheck=True):
    s = {}
    m = re.search(r'score (\d)/3', out)
    s['score'] = int(m.group(1)) if m else -1
    m = re.search(r'final numGride (-?\d+)', out)
    s['nfinal'] = int(m.group(1)) if m else -1
    m = re.search(r'sim time [\d.]+ s\s+path (\d+) cm', out)
    s['pathcm'] = int(m.group(1)) if m else 0
    lines = out.splitlines()
    # ---- the first fault the firmware printed
    fault = None
    for i, l in enumerate(lines):
        m = re.match(r'^FAULT (\d+): ', l)
        if m:
            n = None
            for k in range(i + 1, min(i + 4, len(lines))):
                mm = re.search(r'\bat n=(-?\d+)', lines[k])
                if mm:
                    n = mm.group(1); break
            fault = 'F%s n=%s' % (m.group(1), n if n is not None else '?')
            break
    halts = re.findall(r'\] HALT    (.*)', out)
    if fault is None:
        if s['pathcm'] < 2 and 'RESET   ' not in out:
            fault = 'no start'
        elif s['score'] == 3 and s['nfinal'] >= 0:
            fault = '-'
        elif s['nfinal'] == FINISH.get(route, -1):
            fault = 'finished n=%d' % s['nfinal']   # counted to the end, but objects not on the targets
        else:
            why = halts[-1] if halts else ''
            if 'brown-out' in why:
                fault = 'brown-out n=%d' % s['nfinal']
            elif 'time cap' in why:
                fault = 'time cap n=%d' % s['nfinal']
            elif 'too many restarts' in why:
                fault = 'restarts n=%d' % s['nfinal']
            else:
                fault = 'stuck n=%d' % s['nfinal']
    s['fault'] = fault
    # ---- restarts: what the firmware did after each one
    events = []
    cur = None
    last_n = 0
    for l in lines:
        pm = PATH_RE.match(l)
        if 'RESET   ' in l and '] RESET' in l:
            if cur is not None:
                events.append(cur)
            cur = {'before': last_n, 'pose': None, 'held': 0, 'moved': False, 'resumed': False}
            continue
        if cur is not None and '] RESUME' in l:
            cur['resumed'] = True
            m = POSE_RE.search(l)
            if m:
                cur['pose'] = (float(m.group(1)), float(m.group(2)))
            continue
        if pm:
            n = int(pm.group(2)); x = float(pm.group(3)); y = float(pm.group(4))
            last_n = n
            if cur is not None and not cur['moved']:
                if cur['pose'] and abs(x - cur['pose'][0]) + abs(y - cur['pose'][1]) > 3.0:
                    cur['moved'] = True          # the count it had when it set off is 'held'
                else:
                    cur['held'] = n
    if cur is not None:
        m = re.search(r'final pose \(([-\d.]+), ([-\d.]+)\)', out)
        if m and cur['pose'] and abs(float(m.group(1)) - cur['pose'][0]) + abs(float(m.group(2)) - cur['pose'][1]) > 3.0:
            cur['moved'] = True
        events.append(cur)
    # One entry per restart, or per run of restarts that came one after the other before the
    # robot drove (a brown-out again in the start-up or in the same servo move: "x2", "x3").
    #   halted     it did not drive again
    #   resumed    it set off with its count kept (restored before driving)
    #   restarted  it set off from count 0, as at the start (the same thing when it was at 0 or 1)
    #   loop       it browned out again and again until the sim stopped the run (--maxresets)
    rs = []
    i = 0
    while i < len(events):
        j = i
        while j + 1 < len(events) and not events[j]['moved']:
            j += 1
        e, before, tries = events[j], events[i]['before'], j - i + 1
        ended_reset = j == len(events) - 1 and halts and ('brown-out' in halts[-1] or 'too many restarts' in halts[-1])
        if not e['resumed'] or (not e['moved'] and ended_reset):
            what = 'loop'
        elif not e['moved']:
            what = 'halted'
        elif before >= 1 and e['held'] >= max(before - 1, 1):
            what = 'resumed'
        else:
            what = 'restarted'
        rs.append('%s@%d%s' % (what, before, ('x%d' % tries) if tries > 1 else ''))
        i = j + 1
    s['restart'] = ','.join(rs) if rs else '-'
    if rs and rs[-1].startswith(('halted', 'loop')) and not re.match(r'F\d', s['fault']):
        s['fault'] = '-'                         # it stopped on purpose after the restart: see that column
    # ---- the path check
    s['path'] = '-'
    if pathcheck:
        r = subprocess.run([sys.executable, os.path.join(HERE, 'path_check.py'), '--brief', '--route', route],
                           input=out, capture_output=True, text=True, errors='replace')
        t = r.stdout
        if 'PATH CHECK PASS' in t:
            s['path'] = 'PASS'
        else:
            hard, soft = [], []
            for m in re.finditer(r'^\s*FAIL n=\s*(\d+)(.*)$', t, re.M):
                # a count at the wrong crossing (the check names where the bar really is), or a pick /
                # place that took nothing or the wrong object: wrong. Otherwise only a pose outside
                # the check's tolerances (for example a timed 180 that ends 3 cm to the side): soft.
                (hard if ('<-' in m.group(2) or 'GRIP' in m.group(2) or 'RELEASE' in m.group(2)) else soft).append(int(m.group(1)))
            s['path'] = ('n=%d' % min(hard)) if hard else (('~n=%d' % min(soft)) if soft else '-')
    return s

# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='real-robot test kit')
    ap.add_argument('sketch')
    ap.add_argument('route', nargs='?', default='312')
    ap.add_argument('--conds', default=os.path.join(HERE, 'real_robot_conds.txt'))
    ap.add_argument('--out', default=None, help='where every run output is kept (default: ./rrt_out/SKETCH)')
    ap.add_argument('--cal', default='auto', choices=['auto', '0', '1'])
    ap.add_argument('-j', type=int, default=int(os.environ.get('J', '4')), help='simulator processes at once (default 4)')
    ap.add_argument('--only', default='', help='only these conditions (comma list)')
    a = ap.parse_args()
    global SLOTS
    SLOTS = Slots(max(1, a.j))

    info = sketch_info(a.sketch)
    routes = ['312', '132'] if a.route == 'both' else [a.route]
    if a.route not in ('312', '132', 'both'):
        sys.exit('real_robot_test: ROUTE is 312, 132 or both')
    if not info['has_route']:
        if '132' in routes:
            print('NOTE: %s has one fixed route (no ROUTE_ORDER): 312 only' % info['name'])
        routes = ['312']
    conds = load_conds(a.conds)
    if a.only:
        want = set(a.only.split(','))
        conds = [c for c in conds if c['name'] in want]
    use_cal = (a.cal == '1' and info['cal']) or (a.cal == 'auto' and info['cal'] and info['cal'][0] == 'sibling')
    out_dir = os.path.abspath(a.out or os.path.join('rrt_out', info['name']))
    os.makedirs(out_dir, exist_ok=True)
    bin_dir = os.path.join(out_dir, 'bin')
    os.makedirs(bin_dir, exist_ok=True)

    defs = []
    if info['noinit'] and info['noinit'] != ['runMark']:
        defs.append('-DSIM_NOINIT_VARS=' + ' '.join('X(%s)' % n for n in info['noinit']))
    print('sketch   %s' % info['ino'])
    print('sim      %s/sim.cpp' % HERE)
    print('restart  keeps .noinit: %s%s' % (', '.join(info['noinit']) or '(none)',
                                             ('  (build: %s)' % defs[0]) if defs else ''))
    exes = {}
    jobs = []
    for r in routes:
        exes[r] = os.path.join(bin_dir, 'mission_%s.exe' % r)
        rd = ['-DROUTE_ORDER=%s' % r]
        jobs.append((info['ino'], exes[r], defs + rd))
        if use_cal:
            exes['cal' + r] = os.path.join(bin_dir, 'cal_%s.exe' % r)
            jobs.append((info['cal'][1], exes['cal' + r], rd + ['-DROBOT_MODE=1']))
    with ThreadPoolExecutor(min(4, len(jobs))) as ex:
        list(ex.map(lambda j: build(HERE, j[0], j[1], j[2]), jobs))
    if use_cal:
        print('cal      %s (MODE_CALIBRATE on each condition\'s robot, then the mission with that EEPROM)' % info['cal'][1])

    work = tempfile.mkdtemp(prefix='rrt_')
    # ---- calibrations (one per robot, shared by the conditions with the same robot flags)
    cal_files = {}
    cal_ok = {}
    if use_cal:
        cal_jobs = {}
        for r in routes:
            for c in conds:
                if r not in c['routes']:
                    continue
                rf = robot_flags(c['flags'])
                key = r + '|' + ' '.join(rf)
                if key not in cal_jobs:
                    h = hashlib.md5(key.encode()).hexdigest()[:10]
                    cal_jobs[key] = (r, rf, os.path.join(out_dir, 'cal_%s_%s.ee' % (r, h)))
        def do_cal(item):
            key, (r, rf, ee) = item
            if os.path.exists(ee):
                os.remove(ee)
            o = run_exe(exes['cal' + r], ['--serial', '--startx=0', '--secs=400', '--eeprom=' + ee] + rf,
                        os.path.join(work, 'c_' + os.path.basename(ee)))
            open(ee + '.txt', 'w', encoding='utf-8').write(o)
            return key, ee, ('saved in EEPROM' in o)
        with ThreadPoolExecutor(a.j) as ex:
            for key, ee, ok in ex.map(do_cal, cal_jobs.items()):
                cal_files[key] = ee
                cal_ok[key] = ok

    # ---- the mission runs
    tasks = []
    for r in routes:
        for c in conds:
            if r in c['routes']:
                tasks.append((r, c, False))
                if use_cal:
                    tasks.append((r, c, True))
    def do_run(t):
        r, c, cal = t
        tag = '%s_%s%s' % (c['name'], r, '_cal' if cal else '')
        cwd = os.path.join(work, tag)
        args = ['--serial', '--path'] + c['flags']
        if cal:
            src = cal_files[r + '|' + ' '.join(robot_flags(c['flags']))]
            ee = os.path.join(work, tag + '.ee')
            os.makedirs(cwd, exist_ok=True)
            if os.path.exists(src):
                shutil.copy(src, ee)
            args.append('--eeprom=' + ee)
        o = run_exe(exes[r], args, cwd)
        open(os.path.join(out_dir, tag + '.txt'), 'w', encoding='utf-8').write(o)
        sm = summarize(o, r)
        if cal and not cal_ok.get(r + '|' + ' '.join(robot_flags(c['flags'])), False):
            sm['fault'] += ' *'                  # the calibration saved nothing: the sketch's defaults
        return t, sm
    with ThreadPoolExecutor(a.j) as ex:
        res = list(ex.map(do_run, tasks))
    shutil.rmtree(work, ignore_errors=True)

    # ---- the table
    by = {}
    for (r, c, cal), s in res:
        by[(r, c['name'], cal)] = s
    for r in routes:
        rows = [c for c in conds if r in c['routes']]
        print()
        print('route %s  (%s, %d conditions)' % (r, info['name'], len(rows)))
        hdr = '%-14s %5s  %-14s %-26s %-6s' % ('condition', 'score', 'fault (n)', 'restart', 'path')
        if use_cal:
            hdr += '  %5s  %-14s' % ('cal', 'cal fault (n)')
        print(hdr)
        print('-' * len(hdr))
        tot = full = ctot = cfull = 0
        for c in rows:
            s = by[(r, c['name'], False)]
            line = '%-14s %5s  %-14s %-26s %-6s' % (c['name'], '%d/3' % s['score'], s['fault'], s['restart'], s['path'])
            tot += max(s['score'], 0); full += s['score'] == 3
            if use_cal:
                k = by[(r, c['name'], True)]
                line += '  %5s  %-14s' % ('%d/3' % k['score'], k['fault'])
                ctot += max(k['score'], 0); cfull += k['score'] == 3
            print(line)
        print('-' * len(hdr))
        t = 'TOTAL route %s: %d/%d objects, %d of %d conditions 3/3' % (r, tot, 3 * len(rows), full, len(rows))
        if use_cal:
            t += ' | calibrated: %d/%d objects, %d of %d 3/3' % (ctot, 3 * len(rows), cfull, len(rows))
        print(t)
    if use_cal and not all(cal_ok.values()):
        print('* the calibration of that robot saved nothing (see cal_*.ee.txt): the run used the defaults')
    print()
    print('outputs: %s  (one .txt per run: the firmware\'s Serial lines and the sim\'s events)' % out_dir)

if __name__ == '__main__':
    main()
