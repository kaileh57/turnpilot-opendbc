"""The Tesla port working its own turn signal through DAS_bodyControls (0x3E9). TurnPilot, DESIGN phase4/blinker.

One TeslaCarSignal per CarState, shared with the CarController through the CarState object. Each card cycle:

1. CarInterface.update hands it the cycle's CAN packets (`can`): the newest valid stock frame from the Autopilot side
   (src 2), the echo of our last frame on the car side (src 128), and any frame of ours the panda refused (src 192).
2. CarState.update hands it UI_warning's lamps (`lamps`) and publishes the masked blinkers and carState.carSignal.
3. CarController.update hands it the side to signal (`control`) and sends what it returns: at most one copy per stock
   frame, while it requests a side or releases one.

The panda (safety/modes/tesla.h) blocks the stock frame only while openpilot asserts a request, and bounds every frame:
LEFT/RIGHT need engagement (or a standstill in P, or the check's stop in D), a CANCEL only within 3 s of a LEFT/RIGHT.
"""
from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.car_signal import (NONE, LEFT, RIGHT, LAMP_ON_S, LAMP_GAP_S, DRIVER_AFTER_S, LAMP_OFF_S, RELEASE_TAIL_S,
                                    STOCK_SEND_FRESH_S, STOCK_LOST_S, REQUEST_CAP_S, NO_ECHO_S, HAZARD_FRAMES,
                                    BlinkerMask, other)
from opendbc.car.tesla.teslacan import BODY_CONTROLS_ADDR, body_controls_valid, body_controls_request, create_body_controls
from opendbc.car.tesla.values import CANBUS, TeslaFlags

REQ_NONE, REQ_LEFT, REQ_RIGHT, REQ_CANCEL = 0, 1, 2, 3
CANCEL_REASON = 15  # DAS_CANCEL_MANUAL_BLINKER, the only cancel seen putting a lamp out
SRC_ECHO, SRC_REJECTED = 128, 192

CarSignal = structs.CarState.CarSignal


def on_reason_from_flags(flags: int) -> int:
  if flags & TeslaFlags.CAR_SIGNALS_REASON_9:
    return 9
  if flags & TeslaFlags.CAR_SIGNALS_REASON_0:
    return 0
  return 8


class TeslaCarSignal:
  def __init__(self, CP):
    flags = int(CP.flags)
    self.active = bool(flags & TeslaFlags.CAR_SIGNALS)
    self.check_mode = self.active and bool(flags & TeslaFlags.CAR_SIGNALS_CHECK)
    self.on_reason = on_reason_from_flags(flags)
    self.on_separates = bool(flags & TeslaFlags.CAR_SIGNALS_ON_SEPARATES)
    self.recheck = self.active and not self.check_mode and bool(flags & TeslaFlags.CAR_SIGNALS_RECHECK)

    self.mask = BlinkerMask()
    self.on_bit = {LEFT: False, RIGHT: False}
    self.frame_t = 0.     # the last lamp frame's time
    self.now = 0.         # the newest CAN time seen

    # the stock stream
    self.stock: bytes | None = None
    self.stock_t = 0.
    self.stock_new = False
    self.stock_seen = False

    # our frames
    self.sent_recent: list[bytes] = []   # this request's frames, for echo and rejection matching
    self.last_sent: bytes | None = None
    self.last_send_t: float | None = None
    self.none_sent_t: float | None = None
    self.rejected = 0
    self.frames_sent = 0

    # the request
    self.phase = 'idle'  # idle, requesting, lit, release
    self.side = NONE
    self.reason = self.on_reason
    self.dark_start = True
    self.claim_t = 0.
    self.first_send_t: float | None = None
    self.first_echo_t: float | None = None
    self.lit_t = 0.
    self.lit_frame_t = 0.
    self.release_kind = 'none'
    self.release_t = 0.
    self.cancel_t: float | None = None
    self.cancels = 0
    self.none_pending = False
    self.retry_open = False
    self.lit_since_cancel = False
    self.handover = False
    self.release_frames_done = False
    self.release_why = ''
    self.last_request_t: float | None = None

    self.fault = 'none'
    self.fault_why = ''
    self.still_on = False
    self.block = 'none'
    self.want = NONE
    self.driver_other = NONE  # an unmasked other-side lamp that rose apart from ours: the driver's, not hazards
    self.in_phase_run = 0     # consecutive frames with both lamps lit in the same blink phase: hazards

    # the lamp check (check mode) and the ignition re-check (mode on, verified in P)
    from opendbc.car.tesla.car_signal_check import LampCheck, Recheck
    self.check = LampCheck(self) if self.check_mode else None
    self.recheck_seq = Recheck(self) if self.recheck else None
    self.check_now = False  # the phone page's "Start check in Drive", passed in by card
    self.gear = structs.CarState.GearShifter.unknown
    self.standstill = True
    self.brake = False
    self.enabled = False

  # ---------------------------------------------------------------- inputs

  def can(self, can_packets) -> None:
    """The cycle's CAN packets, before the parsers: the stock frame, our echoes and rejections."""
    if not self.active:
      return
    for nanos, frames in can_packets:
      t = nanos * 1e-9
      self.now = max(self.now, t)
      for address, dat, src in frames:
        if address != BODY_CONTROLS_ADDR or len(dat) != 8:
          continue
        dat = bytes(dat)
        if src == CANBUS.autopilot_party:
          if body_controls_valid(dat) and t >= self.stock_t:
            self.stock, self.stock_t, self.stock_new, self.stock_seen = dat, t, True, True
        elif src == SRC_ECHO and dat in self.sent_recent:
          if self.first_echo_t is None and self.phase in ('requesting', 'lit') and body_controls_request(dat)[0] in (REQ_LEFT, REQ_RIGHT):
            self.first_echo_t = t
        elif src == SRC_REJECTED and dat in self.sent_recent:
          self.rejected += 1
          request, reason = body_controls_request(dat)
          stock_age = t - self.stock_t
          carlog.error({"event": "car signal frame refused by the panda", "request": request, "reason": reason,
                        "stock_age_s": round(stock_age, 3), "rejected": self.rejected})

  def lamps(self, new_frame: bool, left_blinking: int, right_blinking: int, left_on: bool, right_on: bool,
            t: float, standstill: bool = True, gear=None, brake: bool = False) -> tuple[bool, bool]:
    """UI_warning's lamps (1 and 2 are a blink's dark and lit phases); returns carState's leftBlinker/rightBlinker."""
    left, right = left_blinking in (1, 2), right_blinking in (1, 2)
    if not self.active:
      return left, right
    self.now = max(self.now, t)
    self.standstill, self.brake = standstill, brake
    if gear is not None:
      self.gear = gear
    if new_frame:
      in_phase = left and right and left_blinking == right_blinking
      self.in_phase_run = self.in_phase_run + 1 if in_phase else 0
      self.on_bit = {LEFT: bool(left_on), RIGHT: bool(right_on)}
      self.frame_t = t
      self.mask.frame(left, right, t)
      self._on_frame(t)
    self._timers(self.now)
    self._update_block()
    return self.mask.blinkers()

  def control(self, want: int, enabled: bool, now_nanos: int) -> list:
    """The side TurnPilot wants signalled (from CarControl's blinkers); returns the frames to send this cycle."""
    if not self.active:
      return []
    t = max(now_nanos * 1e-9, self.now)
    self.enabled = enabled
    release_mode = 'auto'
    if self.check is not None:
      want, reason, release_mode = self.check.step(t)
    elif self.recheck_seq is not None and not self.recheck_seq.passed:
      want, reason, release_mode = self.recheck_seq.step(t)
    else:
      reason = self.on_reason
      if self.fault != 'none' or not enabled:
        want = NONE
    self.want = want
    self._update_block()

    if self.phase == 'idle':
      if want != NONE and self._may_claim():
        self._claim(want, reason, t)
    elif self.phase in ('requesting', 'lit') and want != self.side:
      self._release('auto' if release_mode == 'auto' else release_mode, t, handover=not enabled and self.check is None,
                    why='intent ended')

    frames = []
    if self.stock_new and self.stock is not None and (t - self.stock_t) < STOCK_SEND_FRESH_S:
      dat = self._next_frame(t)
      if dat is not None:
        frames.append((BODY_CONTROLS_ADDR, dat, CANBUS.party))
        self.last_sent = dat
        self.last_send_t = t
        self.sent_recent = (self.sent_recent + [dat])[-16:]
        self.frames_sent += 1
    self.stock_new = False
    self._release_progress(t)
    return frames

  # ---------------------------------------------------------------- the request

  def _may_claim(self) -> bool:
    return (self.stock_seen and self.block == 'none' and not self.mask.any_masked and not self.mask.hazard and
            self.fault == 'none')

  def _claim(self, side: int, reason: int, t: float) -> None:
    self.phase = 'requesting'
    self.side = side
    self.reason = reason
    self.dark_start = not self.mask.lamp[side]
    self.claim_t = t
    self.first_send_t = None
    self.first_echo_t = None
    self.lit_t = 0.
    self.lit_frame_t = t
    self.cancel_t = None
    self.none_sent_t = None
    self.last_request_t = None
    self.cancels = 0
    self.none_pending = False
    self.retry_open = False
    self.release_frames_done = False
    self.handover = False
    self.sent_recent = []
    self.mask.claim(side)

  def cleanup_cancel(self, side: int, t: float) -> None:
    """The check's clean-up after a failed release step: one fresh request frame, then CANCEL (DESIGN 7.4), so the
    panda's CANCEL window holds. Whatever the port was doing is dropped; the mask stays on both sides."""
    reason = self.reason
    self.phase = 'idle'
    self._claim(side, reason, t)
    self.dark_start = True

  def _release(self, mode: str, t: float, handover: bool = False, why: str = '') -> None:
    """End the request. mode: auto (CANCEL then NONE for a dark-start request that saw nothing else light, else NONE),
    none (NONE alone), cancel (CANCEL then NONE), silent (send nothing more: the panda's ownership times out)."""
    if self.phase not in ('requesting', 'lit'):
      return
    both_lit = self.mask.lamp[LEFT] and self.mask.lamp[RIGHT]
    sent_any = self.first_send_t is not None
    if mode == 'auto':
      cancel = self.dark_start and not both_lit and not self.mask.hazard
      if cancel and self.on_separates and self.on_bit[self.side]:
        cancel = False  # the On bit says the lamp is the driver's: leave it to them
      mode = 'cancel' if cancel else 'none'
    if mode == 'cancel' and (both_lit or self.mask.hazard):
      mode = 'none'  # what a CANCEL does to hazards is unknown: never while both lamps are lit
    self.phase = 'release'
    self.release_kind = mode if sent_any else 'silent'
    self.release_t = t
    self.release_why = why
    self.handover = handover
    self.none_pending = self.release_kind in ('none', 'cancel')
    self.lit_since_cancel = False
    self.retry_open = False
    self.release_frames_done = self.release_kind == 'silent'
    if self.release_frames_done:
      self._frames_done(t)

  def _next_frame(self, t: float) -> bytes | None:
    assert self.stock is not None
    if self.phase in ('requesting', 'lit'):
      if self.first_send_t is None:
        self.first_send_t = t
      self.last_request_t = t
      return create_body_controls(self.stock, self.side, self.reason)
    if self.phase == 'release' and not self.release_frames_done:
      _, stock_reason = body_controls_request(self.stock)
      if self.release_kind == 'cancel' and self._cancel_unsafe():
        self.release_kind, self.retry_open, self.none_pending = 'none', False, True  # another lamp lit since: NONE alone
      if self.release_kind == 'cancel' and self.cancels == 0:
        self.cancels = 1
        self.cancel_t = t
        self.lit_since_cancel = True
        return create_body_controls(self.stock, REQ_CANCEL, CANCEL_REASON)
      if self.retry_open:
        self.retry_open = False
        self.cancels += 1
        self.cancel_t = t
        self.none_pending = True
        self.lit_since_cancel = True
        carlog.warning({"event": "car signal CANCEL retried", "side": self.side})
        return create_body_controls(self.stock, REQ_CANCEL, CANCEL_REASON)
      if self.none_pending:
        self.none_pending = False
        self.none_sent_t = t
        return create_body_controls(self.stock, REQ_NONE, stock_reason)
    return None

  def _cancel_unsafe(self) -> bool:
    """A lamp the CANCEL would erase lit since the release was decided: the other side (the driver's), or hazards."""
    m = self.mask
    return m.lamp[other(self.side)] or m.hazard or self.in_phase_run > 0

  def _release_progress(self, t: float) -> None:
    if self.phase != 'release':
      return
    if not self.release_frames_done and not self.none_pending and not self.retry_open:
      if self.release_kind == 'cancel' and self.cancels == 1 and self.cancel_t is not None:
        if t - self.cancel_t >= LAMP_OFF_S:
          # one retry, only while our lamp has read lit on every frame since the CANCEL, and never on a handover
          if self.lit_since_cancel and not self.handover:
            self.retry_open = True
          else:
            self._frames_done(t)
      else:
        self._frames_done(t)
    if self.release_frames_done and not self.mask.masked(self.side):
      self.phase = 'idle'
      self.side = NONE
      self.still_on = self.still_on and self.mask.any_masked

  def _frames_done(self, t: float) -> None:
    self.release_frames_done = True
    lift_after = (self.cancel_t + LAMP_OFF_S) if self.cancel_t is not None else t
    if self.last_request_t is not None:
      # a lamp that answers our last request late (up to LAMP_ON_S, longer with no echo) still falls inside the mask
      lift_after = max(lift_after, self.last_request_t + LAMP_ON_S + NO_ECHO_S)
    self.mask.released(self.side, lift_after)

  # ---------------------------------------------------------------- the lamps

  def _on_frame(self, t: float) -> None:
    m = self.mask
    if self.driver_other != NONE and m.zeros[self.driver_other] >= 2:
      self.driver_other = NONE
    if self.phase == 'release' and self.cancel_t is not None and not m.lamp[self.side]:
      self.lit_since_cancel = False
    if self.phase not in ('requesting', 'lit'):
      return
    s, o = self.side, other(self.side)
    if m.lamp[s] and m.lamp[o]:
      in_on_window = self._in_on_window(t)
      if self.phase == 'requesting' and m.rose(s) and m.rose(o) and self.dark_start and in_on_window:
        self._set_fault('wrongSide', 'both lamps lit for one side', t, 'none')
      elif m.mask[o] == 'pending' and m.rose(o) and not m.rose(s):
        self._set_fault('wrongSide', 'the other side lit', t, 'none')
      elif m.mask[o] is None and m.rose(o) and not m.rose(s):
        self.driver_other = o
        self._release('none', t, why="the driver's other side")
      else:
        self._release('none', t, why='hazards')
      return
    if m.lamp[o]:
      if m.mask[o] == 'pending':
        # ours on the wrong side, or the driver's the other way in the request's first second: released with NONE alone
        # (the check verified a NONE puts our lamp out), never a CANCEL that would erase the driver's signal
        self._set_fault('wrongSide', 'the other side lit', t, 'none')
      else:
        self.driver_other = o
        self._release('none', t, why="the driver's other side")
      return
    if m.lamp[s]:
      self.lit_frame_t = t
      if self.phase == 'requesting' and self.first_send_t is not None:
        self.phase = 'lit'
        self.lit_t = t
    elif self.phase == 'lit' and t - self.lit_frame_t > LAMP_GAP_S:
      self._set_fault('lampLost', 'the lamp went out', t, 'auto')
      return
    if self.phase == 'lit' and t - self.lit_t >= DRIVER_AFTER_S and not m.lamp[o]:
      m.driver_unmask(s)

  def _in_on_window(self, t: float) -> bool:
    start = self.first_echo_t if self.first_echo_t is not None else self.first_send_t
    return start is not None and t - start <= LAMP_ON_S + (0. if self.first_echo_t is not None else NO_ECHO_S)

  def _timers(self, t: float) -> None:
    if self.stock_seen and t - self.stock_t > STOCK_LOST_S and self.fault == 'none':
      self._set_fault('noStockFrame', f'no stock body frame for {t - self.stock_t:.1f} s', t, 'silent')
    if self.phase == 'requesting' and self.fault == 'none':
      late = False
      if self.first_echo_t is not None:
        late = t - self.first_echo_t > LAMP_ON_S
      elif self.first_send_t is not None:
        late = t - self.first_send_t > LAMP_ON_S + NO_ECHO_S
      if late:
        why = 'the panda refused it' if self.rejected > 0 and self.first_echo_t is None else "the lamp didn't light"
        self._set_fault('lampTimeout', why, t, 'auto')
    if self.phase in ('requesting', 'lit') and t - self.claim_t > REQUEST_CAP_S and self.fault == 'none':
      self._set_fault('capped', f'signal on {t - self.claim_t:.0f} s', t, 'auto')
    # rule 8: a masked lamp lit past the release tail is said out loud
    still = False
    if self.phase == 'release' and self.mask.lamp[self.side] and t - self.release_t >= LAMP_OFF_S:
      self.mask.mark_stuck(self.side)  # not out a second after its release: it lifts only after STUCK_OFF_S dark
    if self.phase == 'release' and self.mask.lamp[self.side] and t - self.release_t >= RELEASE_TAIL_S:
      still = True
      if self.dark_start and self.release_kind in ('cancel', 'none', 'silent') and self.fault == 'none':
        self._set_fault('lampStuck', 'the lamp stayed on after the release', t, None)
    for side in (LEFT, RIGHT):
      if self.mask.mask[side] == 'inherited' and self.mask.lamp[side] and t - self.mask.start_t >= RELEASE_TAIL_S:
        still = True
    self.still_on = still

  def _set_fault(self, fault: str, why: str, t: float, release_mode: str | None) -> None:
    if self.fault == 'none':
      self.fault, self.fault_why = fault, why
      carlog.error({"event": "car signal fault", "fault": fault, "why": why, "side": self.side, "rejected": self.rejected})
    if release_mode is not None:
      self._release(release_mode, t, why=fault)

  def _update_block(self) -> None:
    m = self.mask
    left, right = m.lamp[LEFT], m.lamp[RIGHT]
    if self.in_phase_run >= HAZARD_FRAMES:
      self.block = 'hazard'  # both lamps blinking together: the hazard lights
      self.driver_other = NONE
    elif self.driver_other != NONE and m.lamp[self.driver_other]:
      self.block = 'otherSide'
    elif left and right:
      self.block = 'otherSide'  # one overlapping frame is also how a driver switching sides looks
    elif m.hazard:
      self.block = 'hazard'
    elif any(m.mask[s] == 'inherited' and m.lamp[s] for s in (LEFT, RIGHT)):
      self.block = 'inherited'
    elif self.want != NONE and m.lamp[other(self.want)] and m.mask[other(self.want)] is None:
      self.block = 'otherSide'
    else:
      self.block = 'none'

  # ---------------------------------------------------------------- outputs

  @property
  def status(self) -> str:
    if not self.active:
      return 'off'
    if self.check is not None:
      return 'checking'
    if self.fault != 'none':
      return 'failed'
    if self.recheck_seq is not None and not self.recheck_seq.passed:
      return 'verifying'
    if not self.stock_seen:
      return 'waiting'
    return {'idle': 'ready', 'requesting': 'requesting', 'lit': 'lit', 'release': 'releasing'}[self.phase]

  def publish(self, cs) -> None:
    """Fill carState.carSignal."""
    out = cs.carSignal
    out.status = self.status
    out.left = self.side == LEFT and self.phase != 'idle'
    out.right = self.side == RIGHT and self.phase != 'idle'
    out.leftLamp = self.mask.lamp[LEFT]
    out.rightLamp = self.mask.lamp[RIGHT]
    out.fault = self.fault
    out.block = self.block
    out.hazard = self.mask.hazard
    out.stillOn = self.still_on
    out.leftOn = self.on_bit[LEFT]
    out.rightOn = self.on_bit[RIGHT]
    out.rejected = self.rejected
    seq = self.check if self.check is not None else self.recheck_seq
    if seq is not None:
      out.check = seq.stage
      out.checkWhy = seq.why
