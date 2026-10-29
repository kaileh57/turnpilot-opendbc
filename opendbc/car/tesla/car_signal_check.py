# ruff: noqa: B901 -- the scripts are generators whose steps return their measurements through `yield from`
"""The lamp check (check mode) and the ignition re-check, run in the port through the same request path a drive uses.

TurnPilot, DESIGN phase4/blinker section 7.4 and 7.5. Each is a script (a generator) the port steps once a card cycle:
it yields the side, reason and release mode to request, and reads the lamps, echoes and the car's state off the port.
It never starts in D by itself: in D only from the phone page's "Start check in Drive", with the brake held.
"""
import json
import random

from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.car_signal import NONE, LEFT, RIGHT, other

GearShifter = structs.CarState.GearShifter

CHECK_HOLD_S = 10.      # step 1 and 3: each reason held this long
CHECK_ON_S = 0.5        # lit within this of the first echoed frame
CHECK_GAP_S = 0.2       # longest dark run while held
CHECK_OFF_S = 0.5       # off within this of the CANCEL
CHECK_QUIET_S = 3.      # quiet after a release: the lamp stays off
RELEASE_HOLD_S = 3.     # step 2: held before the NONE alone, and before the silence
NONE_OFF_S = 1.5        # step 2 (a): off within this of a NONE alone
TIMEOUT_OFF_S = 2.8     # step 2 (b): off within this of our last frame (750 ms ownership, one stock period, 1.5 s)
START_HOLD_S = 3.       # preconditions held this long before the check starts
PRESS_WINDOW_S = 10.    # step 5: the driver's own press
VERIFY_HOLD_S = 2.      # the re-check's holds
VERIFY_QUIET_S = 1.
VERIFY_START_S = 1.
ON_REASONS = (8, 9, 0)


class Abort(Exception):
  pass


class Failed(Exception):
  pass


class _Script:
  """Steps a generator: each step sends the time in and gets (side, reason, release mode) out."""
  stage = 'idle'
  why = ''

  def __init__(self, port, rng=None):
    self.port = port
    self.rng = rng if rng is not None else random.SystemRandom()
    self.gen = None
    self.cmd = (NONE, port.on_reason, 'auto')
    self.done = False

  def step(self, t: float):
    if self.done:
      return NONE, self.port.on_reason, 'auto'
    if self.gen is None:
      self.gen = self.run()
      self.cmd = next(self.gen)
    try:
      self.cmd = self.gen.send(t)
    except StopIteration:
      self.done = True
      self.cmd = (NONE, self.port.on_reason, 'auto')
    return self.cmd

  # ------------------------------------------------------------ helpers shared by both scripts

  def lamp(self, side: int) -> bool:
    return self.port.mask.lamp[side]

  def fresh_stock(self, t: float) -> bool:
    return self.port.stock_seen and t - self.port.stock_t < 0.6

  def parked(self) -> bool:
    return self.port.gear == GearShifter.park and self.port.standstill

  def stopped_in_drive(self) -> bool:
    return self.port.gear == GearShifter.drive and self.port.standstill and self.port.brake

  def wait_idle(self, side, reason, release, limit_s=6.):
    """Keep releasing until the port is idle (its mask down), up to limit_s."""
    t0 = None
    while True:
      t = yield (NONE, reason, release)
      t0 = t if t0 is None else t0
      self.guard(t, requesting=False)
      if self.port.phase == 'idle':
        return t
      if t - t0 > limit_s:
        raise Failed("the lamp didn't go off")

  def hold(self, side: int, reason: int, hold_s: float, release: str, gap_s: float = CHECK_GAP_S,
           on_s: float = CHECK_ON_S, other_dark: bool = True, touch_bits: tuple = ()):
    """Request side for hold_s from the first frame, then release. Returns the measurements; raises Failed."""
    p = self.port
    t_start = None
    lit_t = None
    last_lit = None
    max_gap = 0.
    on_under = False
    frame_t = p.frame_t
    while True:
      t = yield (side, reason, release)
      t_start = t if t_start is None else t_start
      self.guard(t, requesting=True)
      if p.phase == 'idle' and t - t_start > 1.:
        raise Failed("the request didn't start")
      if p.fault != 'none':
        raise Failed(f"{p.fault}: {p.fault_why}")
      if p.frame_t != frame_t:
        frame_t = p.frame_t
        if other_dark and self.lamp(other(side)):
          raise Failed("the other side lit")
        for s in touch_bits:
          if p.on_bit[s]:
            raise Abort("signal touched")
        if self.lamp(side):
          if lit_t is None:
            lit_t = frame_t
          if last_lit is not None:
            max_gap = max(max_gap, frame_t - last_lit - 0.1)
          last_lit = frame_t
          on_under = on_under or p.on_bit[side]
        elif lit_t is not None and frame_t - last_lit > gap_s + 0.1:
          raise Failed(f"lamp stopped after {frame_t - lit_t:.1f} s")
      start = p.first_echo_t if p.first_echo_t is not None else p.first_send_t
      if lit_t is None and start is not None and t - start > on_s:
        raise Failed("no lamp")
      if p.first_send_t is not None and t - p.first_send_t >= hold_s:
        break
    latency = lit_t - (p.first_echo_t if p.first_echo_t is not None else p.first_send_t)
    return {'latency': max(latency, 0.), 'gap': max_gap, 'on': on_under}

  def release_off(self, side: int, reason: int, release: str, off_s: float, quiet_s: float):
    """After a hold: release, the lamp off within off_s of the release frame (or of the last frame, when silent),
    then quiet_s with both lamps off. Returns the off latency."""
    p = self.port
    t_rel = None
    off_t = None
    frame_t = p.frame_t
    ref = None
    while True:
      t = yield (NONE, reason, release)
      t_rel = t if t_rel is None else t_rel
      self.guard(t, requesting=False)
      if ref is None:
        if release == 'silent':
          ref = p.last_send_t
        elif release == 'cancel':
          ref = p.cancel_t
        elif release == 'none':
          ref = p.none_sent_t
      if p.frame_t != frame_t:
        frame_t = p.frame_t
        if not self.lamp(side) and off_t is None and ref is not None:
          off_t = frame_t
        elif self.lamp(side):
          off_t = None
      if ref is not None and off_t is None and t - ref > off_s:
        raise Failed("lamp didn't go off" if release != 'silent' and release != 'none' else
                     "car holds the signal after openpilot stops: not safe to command")
      if off_t is not None and p.phase == 'idle':
        break
      if t - t_rel > 8.:
        raise Failed("lamp didn't go off")
    latency = max(off_t - ref, 0.)
    yield from self.quiet(quiet_s)
    return latency

  def quiet(self, quiet_s: float):
    t0 = None
    frame_t = self.port.frame_t
    while True:
      t = yield (NONE, self.port.on_reason, 'auto')
      t0 = t if t0 is None else t0
      self.guard(t, requesting=False)
      if self.port.frame_t != frame_t:
        frame_t = self.port.frame_t
        if self.lamp(LEFT) or self.lamp(RIGHT):
          raise Abort("a lamp lit outside a request")
      if t - t0 >= quiet_s:
        return

  def guard(self, t: float, requesting: bool) -> None:
    raise NotImplementedError


class LampCheck(_Script):
  """Check mode (DESIGN 7.4): ON reason, release gating, right side, a random pattern, the driver's press."""

  def __init__(self, port, rng=None):
    super().__init__(port, rng)
    self.result: dict | None = None    # what card writes to TurnPilotCarSignalsCheck when the check ends
    self.gear_at_start = None
    self.in_drive = False
    self.stops = 0

  def guard(self, t: float, requesting: bool) -> None:
    p = self.port
    if p.enabled:
      raise Abort("engaged")
    if not p.standstill:
      raise Abort("the car moved")
    if p.gear != self.gear_at_start:
      raise Abort("the gear changed")
    if self.in_drive and not p.brake:
      raise Abort("the brake was released")
    if not self.fresh_stock(t):
      raise Abort("no body-control message")

  def run(self):
    p = self.port
    while True:
      self.stage, self.why = 'waiting', ''
      t = yield (NONE, p.on_reason, 'auto')
      # start: in P by itself; in D only from the phone's Start, with the brake held
      held_since = None
      start_in_drive = False
      while True:
        dark = not (self.lamp(LEFT) or self.lamp(RIGHT))
        if p.check_now and self.stopped_in_drive():
          start_in_drive = True
        ok_p = self.parked() and dark and self.fresh_stock(t) and not p.enabled and p.phase == 'idle'
        ok_d = start_in_drive and self.stopped_in_drive() and dark and self.fresh_stock(t) and not p.enabled and p.phase == 'idle'
        if not self.stopped_in_drive():
          start_in_drive = False
        if ok_p or ok_d:
          held_since = t if held_since is None else held_since
          if t - held_since >= START_HOLD_S:
            break
        else:
          held_since = None
        t = yield (NONE, p.on_reason, 'auto')
      p.check_now = False
      self.in_drive = not self.parked()
      self.gear_at_start = p.gear
      carlog.warning({"event": "car signal check started", "gear": "D" if self.in_drive else "P"})
      try:
        record = yield from self.sequence()
      except Abort as e:
        self.stops += 1
        self.stage, self.why = 'aborted', f"check stopped: {e}"
        carlog.warning({"event": "car signal check stopped", "why": str(e)})
        yield from self.cleanup_abort()
        if self.in_drive:
          # in D it never re-arms by itself: another Start press is needed
          while True:
            t = yield (NONE, p.on_reason, 'auto')
            if p.check_now:
              break
          self.in_drive = False
        continue
      except Failed as e:
        self.stage, self.why = 'failed', str(e)
        if not self.in_drive and str(e) == "no lamp" and self.result is None:
          self.why = "no lamp. Car won't signal in Park: set check, restart, and press Start in Drive with the brake held"
        self.result = {'pass': False, 'step': self.step_name, 'why': self.why, **self.measured}
        carlog.error({"event": "car signal check failed", **self.result})
        yield from self.cleanup_fail()
        while True:
          yield (NONE, p.on_reason, 'auto')
      self.stage, self.why = 'passed', "Confirm the outside lamps on the phone"
      self.result = {'pass': True, 'step': 'done', 'why': '', 'record': record, **self.measured}
      carlog.warning({"event": "car signal check passed", **self.result})
      while True:
        yield (NONE, p.on_reason, 'auto')

  def cleanup_abort(self):
    p = self.port
    if p.phase in ('requesting', 'lit', 'release'):
      try:
        yield from self.wait_idle(NONE, p.on_reason, 'auto', limit_s=6.)
      except (Abort, Failed):
        pass

  def cleanup_fail(self):
    """After a failed release step: one fresh request frame, then CANCEL, so the panda's CANCEL window holds."""
    p = self.port
    side = self.cleanup_side
    t = yield (NONE, p.on_reason, 'cancel')
    if not (self.lamp(LEFT) or self.lamp(RIGHT)) and p.phase == 'idle':
      return
    p.cleanup_cancel(side, t)
    t0 = t
    sent = p.frames_sent
    while True:
      t = yield (side, p.reason, 'cancel')
      if p.frames_sent > sent or t - t0 > 2.:
        break
    t1 = None
    while p.phase != 'idle':
      t = yield (NONE, p.on_reason, 'cancel')
      t1 = t if t1 is None else t1
      if t - t1 > 6.:
        self.why += ". Cancel the signal by hand"
        break

  def sequence(self):
    p = self.port
    self.measured: dict = {}
    self.cleanup_side = LEFT
    self.step_name = 'reason'
    self.stage, self.why = 'reason', 'Hands off the signal, stay stopped'
    kept = None
    lat_on, lat_off, gaps = [], [], []
    tried = []
    for reason in ON_REASONS:
      try:
        h = yield from self.hold(LEFT, reason, CHECK_HOLD_S, 'cancel')
        off = yield from self.release_off(LEFT, reason, 'cancel', CHECK_OFF_S, CHECK_QUIET_S)
      except Failed as e:
        tried.append(f"{reason}: {e}")
        yield from self.wait_idle(NONE, reason, 'cancel', limit_s=6.)
        yield from self.quiet(CHECK_QUIET_S)
        continue
      kept = reason
      lat_on.append(h['latency'])
      gaps.append(h['gap'])
      lat_off.append(off)
      self.measured['onUnderLeft'] = h['on']
      break
    if kept is None:
      self.measured['tried'] = tried
      raise Failed("no reason held the lamp: " + "; ".join(tried))
    self.measured['onReason'] = kept
    on_under_left = self.measured['onUnderLeft']
    touch = () if on_under_left else (LEFT,)

    self.step_name = 'release'
    self.stage, self.why = 'release', 'Checking the release'
    yield from self.hold(LEFT, kept, RELEASE_HOLD_S, 'none', touch_bits=touch)
    none_off = yield from self.release_off(LEFT, kept, 'none', NONE_OFF_S, CHECK_QUIET_S)
    yield from self.hold(LEFT, kept, RELEASE_HOLD_S, 'silent', touch_bits=touch)
    timeout_off = yield from self.release_off(LEFT, kept, 'silent', TIMEOUT_OFF_S, CHECK_QUIET_S)
    self.measured.update(noneOffS=round(none_off, 3), timeoutOffS=round(timeout_off, 3))

    self.step_name = 'right'
    self.cleanup_side = RIGHT
    self.stage, self.why = 'right', 'Checking Right Signal'
    h = yield from self.hold(RIGHT, kept, CHECK_HOLD_S, 'cancel', touch_bits=touch)
    off = yield from self.release_off(RIGHT, kept, 'cancel', CHECK_OFF_S, CHECK_QUIET_S)
    lat_on.append(h['latency'])
    gaps.append(h['gap'])
    lat_off.append(off)
    self.measured['onUnderRight'] = h['on']
    touch = tuple(s for s, under in ((LEFT, on_under_left), (RIGHT, h['on'])) if not under)

    self.step_name = 'pattern'
    self.stage, self.why = 'pattern', 'Checking turn signals'
    sides = [LEFT] * 3 + [RIGHT] * 3
    self.rng.shuffle(sides)
    dark_at = self.rng.randrange(len(sides) + 1)
    for i, side in enumerate(sides):
      if i == dark_at:
        yield from self.quiet(3.)
      self.cleanup_side = side
      h = yield from self.hold(side, kept, self.rng.uniform(2., 4.), 'cancel', touch_bits=touch)
      off = yield from self.release_off(side, kept, 'cancel', CHECK_OFF_S, self.rng.uniform(1., 4.))
      lat_on.append(h['latency'])
      gaps.append(h['gap'])
      lat_off.append(off)
    if dark_at == len(sides):
      yield from self.quiet(3.)

    self.step_name = 'driverPress'
    self.stage, self.why = 'driverPress', 'Signal left yourself, then cancel it'
    on_driver = None
    t0 = None
    while True:
      t = yield (NONE, kept, 'auto')
      t0 = t if t0 is None else t0
      if p.enabled or not p.standstill or p.gear != self.gear_at_start:
        break
      if self.lamp(LEFT) and on_driver is not True:
        on_driver = bool(p.on_bit[LEFT])
      if t - t0 >= PRESS_WINDOW_S:
        break
    self.measured['onDriver'] = on_driver
    on_separates = bool(on_driver) and not on_under_left and not self.measured['onUnderRight']
    return {
      'onReason': kept, 'cancelReason': 15,
      'onLatencyS': round(max(lat_on), 3), 'offLatencyS': round(max(lat_off), 3),
      'noneOffS': self.measured['noneOffS'], 'timeoutOffS': self.measured['timeoutOffS'],
      'maxGapS': round(max(gaps), 3), 'gear': 'D' if self.in_drive else 'P', 'onSeparates': on_separates,
    }

  def result_json(self) -> str:
    return json.dumps(self.result) if self.result is not None else ''


class Recheck(_Script):
  """The re-check at every ignition of a car verified in P (DESIGN 7.5): LEFT 2 s, NONE, quiet, RIGHT 2 s, CANCEL."""

  def __init__(self, port, rng=None):
    super().__init__(port, rng)
    self.passed = False
    self.said_wait = False

  def guard(self, t: float, requesting: bool) -> None:
    if not self.parked():
      raise Abort("left Park")
    if not self.fresh_stock(t):
      raise Abort("no body-control message")

  def run(self):
    p = self.port
    while True:
      self.stage, self.why = 'waiting', 'Turn signal re-check: stop in Park to finish it'
      held_since = None
      while True:
        t = yield (NONE, p.on_reason, 'auto')
        dark = not (self.lamp(LEFT) or self.lamp(RIGHT))
        if self.parked() and dark and self.fresh_stock(t) and p.phase == 'idle' and p.fault == 'none':
          held_since = t if held_since is None else held_since
          if t - held_since >= VERIFY_START_S:
            break
        else:
          held_since = None
      touch = (LEFT, RIGHT) if p.on_separates else ()
      try:
        self.stage, self.why = 'left', 'Stay in Park'
        yield from self.hold(LEFT, p.on_reason, VERIFY_HOLD_S, 'none', touch_bits=touch)
        yield from self.release_off(LEFT, p.on_reason, 'none', NONE_OFF_S, VERIFY_QUIET_S)
        self.stage = 'right'
        yield from self.hold(RIGHT, p.on_reason, VERIFY_HOLD_S, 'cancel', touch_bits=touch)
        yield from self.release_off(RIGHT, p.on_reason, 'cancel', CHECK_OFF_S, VERIFY_QUIET_S)
      except Abort as e:
        carlog.warning({"event": "car signal re-check waiting", "why": str(e)})
        if p.phase != 'idle':
          try:
            yield from self.wait_idle(NONE, p.on_reason, 'auto')
          except (Abort, Failed):
            pass
        continue
      except Failed as e:
        self.stage, self.why = 'failed', f"Re-check failed: {e}"
        if p.fault == 'none':
          p._set_fault('recheckFailed', str(e), t, 'auto')
        carlog.error({"event": "car signal re-check failed", "why": str(e), "fault": p.fault})
        while True:
          yield (NONE, p.on_reason, 'auto')
      self.stage, self.why = 'passed', ''
      self.passed = True
      carlog.warning({"event": "car signal re-check passed"})
      while True:
        yield (NONE, p.on_reason, 'auto')
