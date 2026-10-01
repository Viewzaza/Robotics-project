#!/bin/sh
# Runs with python3, or with python where python3 is missing or is only the
# Windows Store stub (the line below is shell for sh and a string for python).
''''python3 -c 1 >/dev/null 2>&1 && exec python3 "$0" "$@"; exec python "$0" "$@" # '''
DOC = """path_check.py: prove that a simulated e33 mission follows the route for the
right reason, not only that the objects end on the targets.

Every count of the firmware (numGride) must happen at one crossing of the
field, with the robot heading one way. This script works out where each
count MUST happen from the route alone (the table is not read from the
firmware: see ROUTES below), and checks every count of a sim run against it:

  - the order: 1, 2, 3 ... with none missing, none extra, none taken back,
    and the last one is the finish (42 for route 3-1-2, 33 for 1-3-2);
  - a count on arriving at a crossing: the BAR (9.5 cm ahead of the axle)
    is just past that crossing, on its line, and the heading is the one the
    robot drives in on;
  - the count the case adds after its action (a turn, a pick, a place):
    the axle is where the action leaves it (over the crossing after a turn,
    the jaws' reach back from the end line after a pick or a place) and the
    heading is the one it drives out on. Headings are not wrapped (the sim
    keeps adding them up), so a 180 turned the wrong way is 360 deg off;
  - each pick (GRIP) and place (RELEASE) happens at its crossing, between
    the two counts of that case, with the right object, near the object /
    the target;
  - the sim's score is 3/3.

Usage
  ./sim/path_check.py [--route 132] < output.txt
        output of a mission run with --path (the sim prints a PATH line at
        every change of the count, and a PATHCFG line with the geometry).
        Without PATH lines the COUNT lines of a run without --quiet are used,
        but those only show a count after loop() is done: the count of a
        crossing where a case acts is then never seen (only the one after).
  ./sim/path_check.py --run "<sim flags>" [--exe sim/sim_e33.exe] [--route 132]
        runs the sim itself (adds --path, and --quiet unless --serial).
  --table      print the derived table of counts and exit
  --brief      print only the failures and the summary
  --lenient    a count taken back (a retry at a column end) or credited (a
               missed crossing counted later) is a warning, not a failure
  --save FILE  (with --run) keep the sim's output
  --tol K=V,.. change a tolerance (names in TOL below)

Exit code: 0 = PASS, 1 = FAIL, 2 = no usable input.
"""
import argparse
import math
import os
import re
import shlex
import subprocess
import sys

# ---------------------------------------------------------------- tolerances
# Set from 110 runs that placed all 3 objects (both routes: the e33_test.sh
# list with and without calibration, --real and the harsh --real with
# --pseed=1..6, --startx=-8/0/6); the worst value seen is in [brackets].
# A wrong crossing is a whole cell (25 cm) away and a wrong heading 90 or
# 180 deg, so the limits still tell those apart by far.
TOL = {
    # a count on arriving at a crossing: the bar
    'along_min': -2.0,      # cm, bar past the crossing's centre, at least [+0.1]: it counts once the bar left the tape
    'along_max': 7.0,       # cm, ... at most [+5.0]
    'across': 3.5,          # cm, bar to the side of the line it came along [2.5]
    'head': 20.0,           # deg, heading [10.9]
    # the count after a 90 deg turn: the bar on the line out, just past the crossing
    'axle_turn': 10.0,      # cm, axle from the crossing [7.2]
    'out_side': 3.0,        # cm, bar to the side of the line out, also after a 180 [1.5]
    'out_along_min': 3.0,   # cm, bar along the line out from the crossing, at least [5.5]
    'out_along_max': 15.0,  # cm, ... at most [11.3]
    'head_turn': 60.0,      # deg, heading [47.5] (the bar on the line out is what matters: a spin that
                            # ends skewed leaves the axle off to the side)
    # the count after a pick or a place: where the jaws were over the object / target
    'axle_end': 6.5,        # cm, axle from there [4.3]
    'head_end': 30.0,       # deg, heading after the 180, or after the last place the heading in [18.5]
    'place': 6.0,           # cm, object released from its target: the sim's own score limit [2.6]
    'end_move': 3.0,        # cm, the robot moved after the last count: it must stand still [0.1]
}

# ---------------------------------------------------------------- the route
# The field (sim.cpp buildField): columns C1..C4 at x = 0, c, 2c, 3c; rows
# TOP y = r, MID y = 0, BOT y = -r (c = --cell, 25 cm; r = --row, default c). Objects and targets
# sit objBeyond (5 cm) past the outer lines:
#   o1 top of C1, o2 bottom of C1, o3 top of C4;  x1 bottom of C4, x2 bottom
#   of C3, x3 bottom of C2; o1 goes to x1, o2 to x2, o3 to x3.
# Start: axle over C1 on MID facing east; the bar is past C1, so C1 is not
# counted.
#
# A route is the list of crossings where the robot does something; the
# crossings it drives over on the way are counted too. The robot drives along
# MID or up and down a column, never along TOP or BOT.
#   turn            a 90 deg turn; its direction follows from where it goes next
#   pick  o L|R     keep_item: take object o, then turn 180 LEFT or RIGHT
#   place o L|R     place_item: put object o down, then turn 180
#   place o STOP    the last place: the robot stays there
# The direction of each 180 is part of the route: at a corner (C1 or C4) the
# robot must turn toward the field, or the bar swings over the short line
# past the corner; at C2 BOT the front swings away from C1, where object 2
# still stands.
COLS = {'C1': 0, 'C2': 1, 'C3': 2, 'C4': 3}
ROWS = {'TOP': 1, 'MID': 0, 'BOT': -1}
OBJ_HOME = {'o1': ('C1', 'TOP'), 'o2': ('C1', 'BOT'), 'o3': ('C4', 'TOP')}
TARGET_OF = {'o1': ('x1', 'C4', 'BOT'), 'o2': ('x2', 'C3', 'BOT'), 'o3': ('x3', 'C2', 'BOT')}

ROUTES = {
    # 3, 1, 2: object 3 first, straight along MID from the start
    312: [
        ('C4', 'MID', 'turn'),
        ('C4', 'TOP', 'pick', 'o3', 'L'),
        ('C4', 'MID', 'turn'),
        ('C2', 'MID', 'turn'),
        ('C2', 'BOT', 'place', 'o3', 'L'),
        ('C2', 'MID', 'turn'),
        ('C1', 'MID', 'turn'),
        ('C1', 'TOP', 'pick', 'o1', 'R'),
        ('C1', 'MID', 'turn'),
        ('C4', 'MID', 'turn'),
        ('C4', 'BOT', 'place', 'o1', 'R'),
        ('C4', 'MID', 'turn'),
        ('C1', 'MID', 'turn'),
        ('C1', 'BOT', 'pick', 'o2', 'L'),
        ('C1', 'MID', 'turn'),
        ('C3', 'MID', 'turn'),
        ('C3', 'BOT', 'place', 'o2', 'STOP'),
    ],
    # 1, 3, 2: at "go" it turns onto C1 where it stands (not a count), then
    # object 1; from x1 it drives straight on up C4 to object 3
    132: [
        ('C1', 'TOP', 'pick', 'o1', 'R'),
        ('C1', 'MID', 'turn'),
        ('C4', 'MID', 'turn'),
        ('C4', 'BOT', 'place', 'o1', 'R'),
        ('C4', 'TOP', 'pick', 'o3', 'L'),
        ('C4', 'MID', 'turn'),
        ('C2', 'MID', 'turn'),
        ('C2', 'BOT', 'place', 'o3', 'L'),
        ('C2', 'MID', 'turn'),
        ('C1', 'MID', 'turn'),
        ('C1', 'BOT', 'pick', 'o2', 'L'),
        ('C1', 'MID', 'turn'),
        ('C3', 'MID', 'turn'),
        ('C3', 'BOT', 'place', 'o2', 'STOP'),
    ],
}
HEAD_NAME = {0: 'east', 90: 'north', 180: 'west', 270: 'south'}
COL_NAME = {v: k for k, v in COLS.items()}
ROW_NAME = {v: k for k, v in ROWS.items()}


class RouteError(Exception):
    pass


def unit(deg):
    r = math.radians(deg)
    return math.cos(r), math.sin(r)


def derive(route_id):
    """Walk the route over the grid: the list of every count, in order.
    Each entry: n, kind, col, row (indices), heading (unwrapped deg), and for
    actions the object and the turn."""
    stops = ROUTES[route_id]
    col, row = 0, 0             # C1 MID
    head = 0                    # east; unwrapped: +90 a left turn, -90 a right
    n = 0
    ev = []
    held = None
    for si, st in enumerate(stops):
        tc, tr = COLS[st[0]], ROWS[st[1]]
        dc, dr = tc - col, tr - row
        if (dc != 0) == (dr != 0):
            raise RouteError('stop %d: %s %s is not straight on from %s %s'
                             % (si, st[0], st[1], COL_NAME[col], ROW_NAME[row]))
        if dc != 0 and row != 0:
            raise RouteError('stop %d: drives along %s (only MID is driven along)' % (si, ROW_NAME[row]))
        want = 0 if dc > 0 else 180 if dc < 0 else 90 if dr > 0 else 270
        delta = (want - head) % 360
        if delta != 0:
            if si == 0 and delta in (90, 270):
                # the first stop is not straight ahead: a turn where it stands (route 1-3-2)
                head += 90 if delta == 90 else -90
                ev.append(dict(n=0, kind='start-turn', col=col, row=row, head=head,
                               turn='LEFT' if delta == 90 else 'RIGHT'))
            else:
                raise RouteError('stop %d: the robot does not face %s %s' % (si, st[0], st[1]))
        stepc, stepr = (dc > 0) - (dc < 0), (dr > 0) - (dr < 0)
        while (col, row) != (tc, tr):
            col += stepc
            row += stepr
            n += 1
            if (col, row) != (tc, tr):
                if row != 0:
                    raise RouteError('count %d: drives over %s %s, not a MID crossing'
                                     % (n, COL_NAME[col], ROW_NAME[row]))
                ev.append(dict(n=n, kind='pass', col=col, row=row, head=head))
        act = st[2]
        if act == 'turn':
            if row != 0:
                raise RouteError('stop %d: a 90 deg turn off MID' % si)
            nx = stops[si + 1]
            ndc, ndr = COLS[nx[0]] - col, ROWS[nx[1]] - row
            nwant = 0 if ndc > 0 else 180 if ndc < 0 else 90 if ndr > 0 else 270
            d = (nwant - head) % 360
            if d not in (90, 270):
                raise RouteError('stop %d: the next stop is not a 90 deg turn away' % si)
            ev.append(dict(n=n, kind='turn', col=col, row=row, head=head,
                           turn='LEFT' if d == 90 else 'RIGHT'))
            head += 90 if d == 90 else -90
            n += 1
            ev.append(dict(n=n, kind='after-turn', col=col, row=row, head=head))
        elif act in ('pick', 'place'):
            obj, turn = st[3], st[4]
            if row == 0:
                raise RouteError('stop %d: a %s on MID (only at a column end)' % (si, act))
            if act == 'pick':
                if held or OBJ_HOME[obj] != (st[0], st[1]):
                    raise RouteError('stop %d: %s is not at %s %s, or the gripper is full' % (si, obj, st[0], st[1]))
                held = obj
            else:
                if held != obj or TARGET_OF[obj][1:] != (st[0], st[1]):
                    raise RouteError('stop %d: %s is not held, or its target is not at %s %s' % (si, obj, st[0], st[1]))
                held = None
            ev.append(dict(n=n, kind=act, col=col, row=row, head=head, obj=obj, turn=turn))
            n += 1
            if turn == 'STOP':
                if si != len(stops) - 1:
                    raise RouteError('stop %d: STOP before the end of the route' % si)
                ev.append(dict(n=n, kind='after-stop', col=col, row=row, head=head, back=head))
            else:
                back = head
                head += 180 if turn == 'L' else -180
                ev.append(dict(n=n, kind='after-180', col=col, row=row, head=head, back=back))
        else:
            raise RouteError('stop %d: unknown action %s' % (si, act))
    if held:
        raise RouteError('the route ends holding %s' % held)
    return ev, n


def place_name(e):
    return '%s %s' % (COL_NAME[e['col']], ROW_NAME[e['row']])


def head_name(h):
    return HEAD_NAME.get(h % 360, '%d' % h)


def describe(e, prev_head=None):
    k = e['kind']
    if k == 'pass':
        return 'drive over, heading %s' % head_name(e['head'])
    if k == 'turn':
        return 'turn90 %s (in %s)' % (e['turn'], head_name(e['head']))
    if k in ('pick', 'place'):
        t = {'L': 'LEFT', 'R': 'RIGHT', 'STOP': 'STOP'}[e['turn']]
        what = 'keep_item %s' % e['obj'] if k == 'pick' else 'place_item %s on %s' % (e['obj'], TARGET_OF[e['obj']][0])
        return '%s, %s (in %s)' % (what, t, head_name(e['head']))
    if k == 'after-turn':
        return '  after the turn: out %s' % head_name(e['head'])
    if k == 'after-180':
        return '  after the 180: out %s (%+d deg)' % (head_name(e['head']), e['head'] - e['back'])
    if k == 'after-stop':
        return '  after the last place: stays, facing %s' % head_name(e['head'])
    if k == 'start-turn':
        return 'at go: turn %s where it stands (not a count), now %s' % (e['turn'], head_name(e['head']))
    return k


def print_table(route_id, ev, last):
    print('route %d: %d counts, finished at %d' % (route_id, last, last))
    print('   n  where     heading (unwrapped)  what')
    for e in ev:
        print('  %2d  %-8s  %6d                %s' % (e['n'], place_name(e), e['head'], describe(e)))


# ---------------------------------------------------------------- the sim output
RE_T = r'\[\s*([0-9.]+)s\]\s+'   # (searched for, not anchored: a PATH line can follow a
                                     #  Serial line the firmware had not finished)
RE_PATH = re.compile(RE_T + r'PATH\s+numGride=(-?\d+)\s+axle=\(\s*(-?[0-9.]+),\s*(-?[0-9.]+)\)\s+heading=\s*(-?[0-9.]+)deg'
                     r'(?:\s+retry=(\d+)\s+credit=(\d+))?')
RE_COUNT = re.compile(RE_T + r'COUNT\s+numGride=(-?\d+)\s+pose=\(\s*(-?[0-9.]+),\s*(-?[0-9.]+)\)\s+heading=\s*(-?[0-9.]+)deg')
RE_CFG = re.compile(RE_T + r'PATHCFG\s+(.*)$')
RE_START = re.compile(RE_T + r'START\s+pose=.*sensorAhead=([0-9.]+)cm\s+cell=([0-9.]+)cm')
RE_GRIP = re.compile(RE_T + r'GRIP\s+(o\d) at \((-?[0-9.]+), (-?[0-9.]+)\)')
RE_MISS = re.compile(RE_T + r'GRIP\s+MISSED')
RE_REL = re.compile(RE_T + r'RELEASE (o\d) at \((-?[0-9.]+), (-?[0-9.]+)\)')
RE_HALT = re.compile(RE_T + r'HALT\s+(.*)$')
RE_FINAL = re.compile(r'final numGride (-?\d+)')
RE_FPOSE = re.compile(r'^final pose \((-?[0-9.]+), (-?[0-9.]+)\) heading (-?\d+) deg')
RE_SCORE = re.compile(r'^score (\d)/3')
RE_FAULT = re.compile(r'^FAULT \d+: ')


def parse(text):
    out = dict(cfg={}, path=[], count=[], events=[], halt=[], final=None, fpose=None,
               score=None, faults=[])
    for raw in text.splitlines():
        line = raw.rstrip('\r')
        m = RE_PATH.search(line)
        if m:
            out['path'].append(dict(t=float(m.group(1)), n=int(m.group(2)), x=float(m.group(3)),
                                    y=float(m.group(4)), h=float(m.group(5)),
                                    retry=int(m.group(6) or 0), credit=int(m.group(7) or 0)))
            out['events'].append(('count', len(out['path']) - 1))
            continue
        m = RE_COUNT.search(line)
        if m:
            out['count'].append(dict(t=float(m.group(1)), n=int(m.group(2)), x=float(m.group(3)),
                                     y=float(m.group(4)), h=float(m.group(5)), retry=0, credit=0))
            out['events'].append(('countline', len(out['count']) - 1))
            continue
        m = RE_CFG.search(line)
        if m:
            for kv in m.group(2).split():
                k, _, v = kv.partition('=')
                out['cfg'][k] = float(v)
            continue
        m = RE_START.search(line)
        if m:
            out['cfg'].setdefault('sensorAhead', float(m.group(2)))
            out['cfg'].setdefault('cell', float(m.group(3)))
            continue
        m = RE_GRIP.search(line)
        if m:
            out['events'].append(('grip', dict(t=float(m.group(1)), obj=m.group(2),
                                               x=float(m.group(3)), y=float(m.group(4)))))
            continue
        if RE_MISS.search(line):
            out['events'].append(('miss', line))
            continue
        m = RE_REL.search(line)
        if m:
            out['events'].append(('release', dict(t=float(m.group(1)), obj=m.group(2),
                                                  x=float(m.group(3)), y=float(m.group(4)))))
            continue
        m = RE_HALT.search(line)
        if m:
            out['halt'].append(m.group(2))
            continue
        m = RE_FINAL.search(line)
        if m and line.startswith('sim time'):
            out['final'] = int(m.group(1))
            continue
        m = RE_FPOSE.match(line)
        if m:
            out['fpose'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
            continue
        m = RE_SCORE.match(line)
        if m:
            out['score'] = int(m.group(1))
            continue
        if RE_FAULT.match(line):
            out['faults'].append(line.strip())
    return out


# ---------------------------------------------------------------- the check
class Result:
    def __init__(self, brief):
        self.brief = brief
        self.fails = 0
        self.warns = 0
        self.first = None
        self.lo = {}
        self.hi = {}

    def note(self, key, val):
        """keep the smallest and the largest value of one error"""
        self.lo[key] = min(self.lo.get(key, val), val)
        self.hi[key] = max(self.hi.get(key, val), val)

    def line(self, ok, text, warn=False):
        if not ok:
            self.fails += 1
            if self.first is None:
                self.first = text
        elif warn:
            self.warns += 1
        tag = 'PASS' if ok and not warn else ('WARN' if ok else 'FAIL')
        if not self.brief or tag != 'PASS':
            print('  %s %s' % (tag, text))


def check(p, route_id, tol, brief, lenient):
    cfg = p['cfg']
    c = cfg.get('cell', 25.0)
    rp = cfg.get('row', c)                 # the row pitch (sim --row), MID to TOP and to BOT
    ahead = cfg.get('sensorAhead', 9.5)
    grip = cfg.get('grip', 12.0)
    beyond = cfg.get('objBeyond', 5.0)
    grip_r = cfg.get('gripRadius', 3.5)    # the jaws take an object this close, no nearer limit makes sense [2.9]
    ev, last = derive(route_id)
    mode = 'PATH' if p['path'] else 'COUNT'
    obs = p['path'] if p['path'] else p['count']
    kind_ev = 'count' if p['path'] else 'countline'
    R = Result(brief)
    print('path check: route %d, %d counts to the finish, from %s lines (cell %.1f cm, bar %.1f cm ahead, '
          'jaws %.1f cm)' % (route_id, last, mode, c, ahead, grip))
    if mode == 'COUNT':
        print('  (no PATH lines: run the sim with --path. COUNT lines are printed after loop(), so the count of a\n'
              '   crossing where a case acts is never seen, only the one after the action.)')

    def xy(e):
        return e['col'] * c, e['row'] * rp

    # ---- the observed counts, in order, with the object events between them
    seen = {}               # n -> the observation at the (last) time n was counted
    grips = {}              # n of the count current at the event -> list of events
    rels = {}
    cur = 0 if mode == 'PATH' else None
    retry0 = credit0 = 0
    order_bad = []
    pending = []            # COUNT lines: a pick / place belongs to the count before the next one seen
    skipped = set()         # counts the firmware credited (a missed crossing counted at the next one)
    for kind, item in p['events']:
        if kind == kind_ev:
            o = obs[item]
            n = o['n']
            for pk, pi in pending:
                (grips if pk == 'grip' else rels).setdefault(n - 1, []).append(pi)
            pending = []
            if o['retry'] > retry0:
                order_bad.append((True, 'count %d taken back: line still ahead at the column end, retried (t=%.1f s)'
                                  % (n + 1, o['t'])))
            retry0 = o['retry']
            credited = o['credit'] > credit0
            if credited:
                order_bad.append((True, 'a crossing missed and counted later, at count %d (t=%.1f s)' % (n, o['t'])))
            credit0 = o['credit']
            if cur is not None and n == cur and mode == 'PATH':
                continue                        # only a counter changed
            if cur is not None and n < cur:
                # taken back: the count it goes back to was already checked where it happened
                order_bad.append((True, 'count went back from %d to %d (t=%.1f s)' % (cur, n, o['t'])))
                cur = n
                continue
            if cur is not None and n > cur + 1 and mode == 'PATH':
                gap = list(range(cur + 1, n))
                if credited:
                    skipped.update(gap)
                order_bad.append((True, 'count %s never happened (jumped %d -> %d, t=%.1f s)'
                                  % (','.join(map(str, gap)), cur, n, o['t'])))
            if n > last:
                order_bad.append((False, 'count %d is past the finish (%d)' % (n, last)))
            if n in seen and n > 0:
                seen[n]['again'] = seen[n].get('again', 0) + 1
                o = dict(o, again=seen[n]['again'])
            seen[n] = o
            cur = n
        elif kind in ('grip', 'release') and mode == 'COUNT':
            pending.append((kind, item))
        elif kind == 'grip':
            grips.setdefault(cur, []).append(item)
        elif kind == 'release':
            rels.setdefault(cur, []).append(item)
        elif kind == 'miss':
            order_bad.append((False, 'the gripper closed on nothing: ' + item))
    for pk, pi in pending:
        (grips if pk == 'grip' else rels).setdefault(cur, []).append(pi)

    def nearest(bx, by):
        # where the bar really is: the nearest crossing, if it is near one
        cc, rr = int(math.floor(bx / c + 0.5)), int(math.floor(by / rp + 0.5))
        if cc in COL_NAME and rr in ROW_NAME and math.hypot(bx - cc * c, by - rr * rp) < min(c, rp) / 3:
            return '  <- the bar is at %s %s' % (COL_NAME[cc], ROW_NAME[rr])
        return '  <- the bar is at (%.0f, %.0f), at no crossing' % (bx, by)

    # ---- every count of the table
    hoff = 0                # whole turns the robot's heading is off (a turn the other way round)
    top_n = max(seen) if seen else 0
    for e in ev:
        n, k = e['n'], e['kind']
        if k == 'start-turn':
            continue
        if n > top_n:       # the run stopped before: one line for all of them, below
            continue
        X, Y = xy(e)
        where = '%-7s' % place_name(e)
        o = seen.get(n)
        label = 'n=%2d %s %s' % (n, where, describe(e).strip())
        if o is None:
            if n in skipped and k == 'pass':
                R.line(lenient, '%s: not seen, the firmware counted it at the next crossing' % label, warn=True)
                continue
            if mode != 'COUNT' or k not in ('turn', 'pick', 'place'):
                R.line(False, '%s: never counted' % label)
                continue
            if not brief:
                print('  ---- %s: not visible in COUNT lines' % label)
        else:
            if o.get('again'):
                R.line(lenient, '%s: counted %d more time(s) (a count taken back and done again)'
                       % (label, o['again']), warn=True)
            # the heading, and a whole turn off: a 180 (or a 90) went the other way round
            dh = o['h'] - (e['head'] + hoff)
            turns = int(math.floor(dh / 360.0 + 0.5))
            if turns and abs(dh - 360 * turns) <= 90:
                how = 'a turn before this count went the other way round'
                if k == 'after-180':
                    how = 'the 180 was turned %s, not %s' % (('LEFT', 'RIGHT') if turns > 0 else ('RIGHT', 'LEFT'))
                R.line(False, '%s: the heading is %+d deg off: %s (checked on from the new heading)'
                       % (label, 360 * turns, how))
                hoff += 360 * turns
                dh -= 360 * turns
            bx = o['x'] + ahead * math.cos(math.radians(o['h']))
            by = o['y'] + ahead * math.sin(math.radians(o['h']))
            ux, uy = unit(e['head'])
            along = (bx - X) * ux + (by - Y) * uy
            across = -(bx - X) * uy + (by - Y) * ux
            if k in ('pass', 'turn', 'pick', 'place'):
                # the bar is just past the crossing, on the line it came along
                R.note('along', along)
                R.note('across', across)
                R.note('head', dh)
                ok = (tol['along_min'] <= along <= tol['along_max'] and abs(across) <= tol['across']
                      and abs(dh) <= tol['head'])
                R.line(ok, '%s: bar %+.1f cm past it, %+.1f cm to the side, heading %+.1f deg  (t=%.1f s)%s'
                       % (label, along, across, dh, o['t'], '' if ok else nearest(bx, by)))
            else:
                # after the action: where the axle must be, and the bar over the line it leaves on
                if k == 'after-turn':
                    ex, ey, lim, key, hk = X, Y, tol['axle_turn'], 'axle_turn', 'head_turn'
                else:
                    bx_, by_ = unit(e['back'])
                    back = grip - beyond            # the jaws over the object: the axle this far before the line
                    ex, ey, lim, key, hk = X - back * bx_, Y - back * by_, tol['axle_end'], 'axle_end', 'head_end'
                d = math.hypot(o['x'] - ex, o['y'] - ey)
                R.note(key, d)
                R.note(hk, dh)
                ok = d <= lim and abs(dh) <= tol[hk]
                side = ''
                if k != 'after-stop':
                    R.note('out_side', across)
                    ok = ok and abs(across) <= tol['out_side']
                    side = ', bar %+.1f cm to the side of the line out' % across
                if k == 'after-turn':
                    R.note('out_along', along)
                    ok = ok and tol['out_along_min'] <= along <= tol['out_along_max']
                    side += ' and %.1f cm along it' % along
                hint = ''
                if k == 'after-turn' and abs(abs(dh) - 180) <= 45:
                    hint = '  <- turned %s, not %s' % ('LEFT' if dh > 0 else 'RIGHT', 'RIGHT' if dh > 0 else 'LEFT')
                R.line(ok, '%s: axle %.1f cm from where it must be%s, heading %+.1f deg  (t=%.1f s)%s'
                       % (label, d, side, dh, o['t'], hint))
        # the object moves of this case happen between its two counts
        if k in ('pick', 'place'):
            lst = (grips if k == 'pick' else rels).pop(n, [])
            if k == 'pick':
                hc, hr = OBJ_HOME[e['obj']]
                tx, ty = COLS[hc] * c, ROWS[hr] * rp + (beyond if ROWS[hr] > 0 else -beyond)
                lim, key, what = grip_r, 'grip', 'GRIP'
            else:
                _, tc_, tr_ = TARGET_OF[e['obj']]
                tx, ty = COLS[tc_] * c, ROWS[tr_] * rp - beyond
                lim, key, what = tol['place'], 'place', 'RELEASE'
            if len(lst) != 1 or lst[0]['obj'] != e['obj']:
                R.line(False, 'n=%2d %s %s %s: saw %s' % (n, where, what, e['obj'],
                       ', '.join('%s %s' % (what, g['obj']) for g in lst) or 'nothing'))
            else:
                g = lst[0]
                d = math.hypot(g['x'] - tx, g['y'] - ty)
                R.note(key, d)
                R.line(d <= lim, 'n=%2d %s %s %s at (%.1f, %.1f): %.1f cm from %s  (t=%.1f s)'
                       % (n, where, what, e['obj'], g['x'], g['y'], d,
                          'the object' if k == 'pick' else TARGET_OF[e['obj']][0], g['t']))

    if top_n < last:
        R.line(False, 'counts %d .. %d never happened: the run stopped after count %d' % (top_n + 1, last, top_n))

    # ---- object moves anywhere else, the order, the finish
    for n, lst in sorted(grips.items(), key=lambda a: (a[0] is None, a[0])):
        for g in lst:
            R.line(False, 'GRIP %s during count %s: not a pick crossing' % (g['obj'], n))
    for n, lst in sorted(rels.items(), key=lambda a: (a[0] is None, a[0])):
        for g in lst:
            R.line(False, 'RELEASE %s during count %s: not a place crossing' % (g['obj'], n))
    for retract, text in order_bad:
        R.line(lenient and retract, text, warn=True)
    fin = p['final'] if p['final'] is not None else (max(seen) if seen else None)
    R.line(fin == last, 'final count %s (the finish is %d)%s' % (fin, last,
           '' if not p['halt'] else ', sim: ' + p['halt'][-1]))
    for f in p['faults'][:3]:
        R.line(False, 'firmware: ' + f)
    ee = [e for e in ev if e['kind'] == 'after-stop']
    if p['fpose'] and ee and seen.get(last):
        o = seen[last]
        d = math.hypot(p['fpose'][0] - o['x'], p['fpose'][1] - o['y'])
        R.note('end_move', d)
        R.line(d <= tol['end_move'], 'stood still after the finish: moved %.1f cm after the last count' % d)
    R.line(p['score'] == 3, 'score %s/3' % p['score'])

    def rng(k):
        return ('%+.1f..%+.1f' % (R.lo[k], R.hi[k])) if k in R.lo else '-'
    def top(k):
        return ('%.1f' % R.hi[k]) if k in R.hi else '-'
    print('  range: at a crossing: bar %s cm past it, %s cm to the side, heading %s deg'
          % (rng('along'), rng('across'), rng('head')))
    print('         after a turn: axle %s cm off, bar %s cm along the line out, %s cm to the side, heading %s deg'
          % (top('axle_turn'), rng('out_along'), rng('out_side'), rng('head_turn')))
    print('         after a pick / place: axle %s cm off, heading %s deg | grip %s cm, place %s cm'
          % (top('axle_end'), rng('head_end'), top('grip'), top('place')))
    verdict = 'PASS' if R.fails == 0 else 'FAIL'
    if R.first:
        print('  first failure: ' + R.first)
    print('PATH CHECK %s: route %d, %d failure(s)%s, %d of %d counts seen'
          % (verdict, route_id, R.fails, (', %d warning(s)' % R.warns) if R.warns else '',
             len([n for n in seen if n > 0]), last))
    # one machine-readable line for scripts (sweeps collect the worst errors)
    print('PATHSTAT %s route=%d fails=%d %s' % (verdict, route_id, R.fails,
          ' '.join('%s=%.2f:%.2f' % (k, R.lo[k], R.hi[k]) for k in sorted(R.lo))))
    return R.fails == 0


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=DOC,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--route', type=int, choices=sorted(ROUTES), help='312 (default) or 132; '
                    'the PATHCFG line of the output says which route the firmware was built for')
    ap.add_argument('--run', metavar='FLAGS', help='run the sim with these flags')
    ap.add_argument('--exe', default=os.path.join(here, 'sim_e33.exe'), help='the sim to run (default sim/sim_e33.exe)')
    ap.add_argument('--save', metavar='FILE', help='with --run: keep the output')
    ap.add_argument('--table', action='store_true', help='print the derived table and exit')
    ap.add_argument('--brief', action='store_true', help='only failures and the summary')
    ap.add_argument('--lenient', action='store_true', help='a retry or a credited crossing is only a warning')
    ap.add_argument('--tol', default='', help='K=V,... change tolerances: ' + ', '.join(sorted(TOL)))
    ap.add_argument('file', nargs='?', help='sim output (default: standard input)')
    a = ap.parse_args()

    tol = dict(TOL)
    for kv in filter(None, a.tol.split(',')):
        k, _, v = kv.partition('=')
        if k not in tol:
            ap.error('unknown tolerance %s' % k)
        tol[k] = float(v)

    if a.table:
        for r in ([a.route] if a.route else sorted(ROUTES, reverse=True)):
            ev, last = derive(r)
            print_table(r, ev, last)
        return 0

    if a.run is not None:
        flags = shlex.split(a.run)
        if '--path' not in flags:
            flags.append('--path')
        if '--serial' not in flags and '--quiet' not in flags:
            flags.append('--quiet')
        try:
            text = subprocess.run([os.path.abspath(a.exe)] + flags, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  universal_newlines=True).stdout
        except OSError as e:
            print('path_check: cannot run %s: %s' % (a.exe, e))
            return 2
        if a.save:
            with open(a.save, 'w') as f:
                f.write(text)
    elif a.file:
        with open(a.file) as f:
            text = f.read()
    else:
        text = sys.stdin.read()

    p = parse(text)
    if not p['path'] and not p['count']:
        print('path_check: no PATH or COUNT lines in the input (run the sim with --path)')
        return 2
    route = a.route
    fw = int(p['cfg'].get('route', 0))
    if fw and route and fw != route:
        print('path_check: the output is of route %d, not %d' % (fw, route))
        return 2
    route = route or fw or 312
    return 0 if check(p, route, tol, a.brief, a.lenient) else 1


if __name__ == '__main__':
    sys.exit(main())
