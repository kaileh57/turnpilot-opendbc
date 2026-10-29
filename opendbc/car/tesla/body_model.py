"""A model of the Tesla car side for TurnPilot's commanded turn signal (DESIGN phase4/blinker section 9.1).

One model, used by opendbc's unit tests, openpilot's property tests and the BeamNG bridge, so all three exercise the
same behavior:

- the stock DAS: an idle DAS_bodyControls every 502 ms with the measured jitter (to 508 ms), counter + 1 and the Tesla
  checksum, plus early content-change frames (counter + 1, off the cadence) at about the measured rate (38 in 2,717);
- the body controller: a request semantics (level, latched, tap, noneIgnored, or the faulty wrongSide and
  hazardAnswer), an ON latency, the reasons it obeys, and CANCEL clearing every lamp, the driver's latched stalk
  included, as CANCEL did in all 133 Tesla episodes;
- the driver: a latched stalk per side, a stalk touch (a 2 to 6 frame dark run), the hazard button over anything else;
- UI_warning at 10 Hz: lamps blinking 2 (lit) and 1 (dark) in 3 to 4 frame phases, an occasional single 0 frame mid-
  blink, and the On bits following only the driver's stalk ('driver') or any lamp ('any').

It records what caused each lamp (driver, request, hazard), so a test can tell a lamp a person lit from one of ours.
"""
import random

from opendbc.car.tesla.teslacan import BODY_CONTROLS_ADDR, body_controls_checksum, body_controls_request

STOCK_IDLE = bytes.fromhex("00880220000010a6")  # the measured idle frame (2025 HW4 Model 3)
STOCK_PERIOD_S = 0.502
UI_PERIOD_S = 0.1
LEVEL_HOLD_S = 1.2         # a level body keeps a request this long without a new request frame
NONE_IGNORED_HOLD_S = 2.5  # the noneIgnored variant ignores NONE and drops a request only this long after the last one
TAP_FLASHES = 3
NONE, LEFT, RIGHT = 0, 1, 2
REQ_NONE, REQ_LEFT, REQ_RIGHT, REQ_CANCEL = 0, 1, 2, 3
SEMANTICS = ('level', 'latched', 'tap', 'noneIgnored', 'wrongSide', 'hazardAnswer')


_PACKER = None


def _ui_warning(left_blink: int, right_blink: int, left_on: bool, right_on: bool, counter: int) -> bytes:
  """UI_warning (0x311): the driver buckled, the blinker fields; counter and checksum by the party DBC's packer."""
  global _PACKER
  if _PACKER is None:
    from opendbc.can import CANPacker
    _PACKER = CANPacker('tesla_model3_party')
  values = {"buckleStatus": 1, "leftBlinkerBlinking": left_blink, "rightBlinkerBlinking": right_blink,
            "leftBlinkerOn": int(left_on), "rightBlinkerOn": int(right_on), "UI_warningCounter": counter}
  return bytes(_PACKER.make_can_msg("UI_warning", 0, values)[1])


class _Lamp:
  """One side's lamp: blinking 2 (lit) and 1 (dark) in 3 to 4 frame phases while any cause holds it."""

  def __init__(self, rng):
    self.rng = rng
    self.on = False
    self.phase_lit = False
    self.frames_left = 0

  def frame(self, on: bool, glitch: bool) -> int:
    if not on:
      self.on, self.phase_lit, self.frames_left = False, False, 0
      return 0
    if not self.on:
      self.on, self.phase_lit, self.frames_left = True, True, self.rng.choice((3, 4))
    elif self.frames_left <= 0:
      self.phase_lit, self.frames_left = not self.phase_lit, self.rng.choice((3, 4))
    self.frames_left -= 1
    if glitch:
      return 0
    return 2 if self.phase_lit else 1


class BodyModel:
  def __init__(self, semantics: str = 'level', on_latency: float = 0.2, reasons=(0, 8, 9), on_bits: str = 'driver',
               rng: random.Random | None = None, glitch_rate: float = 0.003, early_rate: float = 38 / 2717,
               jitter: bool = True, tap_retap: bool = False):
    assert semantics in SEMANTICS
    self.semantics = semantics
    self.on_latency = on_latency
    self.reasons = set(reasons)
    self.on_bits = on_bits
    self.rng = rng or random.Random(0)
    self.glitch_rate = glitch_rate
    self.early_rate = early_rate
    self.jitter = jitter
    self.tap_retap = tap_retap

    self.t = 0.
    self.next_stock_t = 0.
    self.next_ui_t = 0.
    self.counter = 0
    self.ui_counter = 0
    self.stock = bytearray(STOCK_IDLE)

    # the DAS-side request the body holds, per side: when it was asked, when it lights, until when
    self.das_side = NONE
    self.das_on_t = 0.
    self.das_until = 0.
    self.das_last_t = 0.
    self.tap_until = 0.
    # the driver
    self.stalk = {LEFT: False, RIGHT: False}
    self.touch_frames = {LEFT: 0, RIGHT: 0}
    self.hazard_button = False
    self.lamps = {LEFT: _Lamp(self.rng), RIGHT: _Lamp(self.rng)}
    self.blink = {LEFT: 0, RIGHT: 0}
    self.cause = {LEFT: set(), RIGHT: set()}  # what holds each lamp now: 'driver', 'request', 'hazard'
    self.cancels_received = 0
    self.cancels_erasing_driver = 0
    self.frames_received: list[tuple[float, bytes]] = []

  # -------------------------------------------------------------- the driver

  def driver_stalk(self, side: int, on: bool) -> None:
    for s in (LEFT, RIGHT):
      if s == side:
        self.stalk[s] = on
      elif on:
        self.stalk[s] = False   # the stalk can only be one way

  def driver_touch(self, side: int, frames: int | None = None) -> None:
    """A touch of the stalk against our lamp: a short dark run (2 to 6 frames)."""
    self.touch_frames[side] = frames if frames is not None else self.rng.randint(2, 6)

  def hazards(self, on: bool) -> None:
    self.hazard_button = on

  # -------------------------------------------------------------- the car side receives a 0x3E9

  def receive(self, dat: bytes, t: float) -> None:
    """A DAS_bodyControls reaching the car side (forwarded stock, or ours accepted by the panda)."""
    self.frames_received.append((t, bytes(dat)))
    request, reason = body_controls_request(dat)
    if request == REQ_CANCEL:
      self.cancels_received += 1
      if self.stalk[LEFT] or self.stalk[RIGHT]:
        self.cancels_erasing_driver += 1
      self.stalk = {LEFT: False, RIGHT: False}
      self._das_off()
      return
    if request in (REQ_LEFT, REQ_RIGHT):
      if reason not in self.reasons:
        return
      side = request
      if self.semantics == 'tap':
        if self.das_side != side or (self.tap_retap and t > self.das_last_t + 0.6):
          self.das_side = side
          self.das_on_t = t + self.on_latency
          self.tap_until = self.das_on_t + TAP_FLASHES * 0.7
        self.das_last_t = t
        return
      if self.das_side != side:
        self.das_side = side
        self.das_on_t = t + self.on_latency
      self.das_last_t = t
      return
    # NONE
    if self.semantics in ('level', 'wrongSide', 'hazardAnswer', 'tap'):
      self._das_off()

  def _das_off(self):
    self.das_side = NONE
    self.tap_until = 0.

  def _das_lit_sides(self, t: float) -> set:
    if self.das_side == NONE or t < self.das_on_t:
      return set()
    if self.semantics == 'tap':
      return {self.das_side} if t < self.tap_until else set()
    hold = NONE_IGNORED_HOLD_S if self.semantics == 'noneIgnored' else LEVEL_HOLD_S
    if self.semantics in ('level', 'wrongSide', 'hazardAnswer', 'noneIgnored') and t - self.das_last_t > hold:
      self.das_side = NONE
      return set()
    if self.semantics == 'wrongSide':
      return {RIGHT if self.das_side == LEFT else LEFT}
    if self.semantics == 'hazardAnswer':
      return {LEFT, RIGHT}
    return {self.das_side}

  # -------------------------------------------------------------- time

  def stock_frame(self, early: bool = False) -> bytes:
    self.counter = (self.counter + 1) % 16
    dat = bytearray(self.stock)
    dat[6] = (dat[6] & 0x0F) | (self.counter << 4)
    dat[7] = body_controls_checksum(dat)
    return bytes(dat)

  def tick(self, t: float) -> tuple[list[bytes], tuple | None]:
    """Advance to t. Returns the stock 0x3E9 frames the DAS sent (bus 2) and the UI_warning frame, if one is due:
    (bytes, left_blinking, right_blinking, left_on, right_on)."""
    self.t = t
    stock = []
    while t >= self.next_stock_t - 1e-6:
      stock.append(self.stock_frame())
      gap = STOCK_PERIOD_S + (self.rng.uniform(-0.002, 0.006) if self.jitter else 0.)
      if self.early_rate and self.rng.random() < self.early_rate:
        gap = self.rng.uniform(0.01, 0.4)
      self.next_stock_t += gap
    ui = None
    if t >= self.next_ui_t - 1e-6:
      self.next_ui_t += UI_PERIOD_S
      ui = self._ui_frame(t)
    return stock, ui

  def _ui_frame(self, t: float):
    das = self._das_lit_sides(t)
    on_bit = {}
    for side in (LEFT, RIGHT):
      cause = set()
      if self.hazard_button:
        cause.add('hazard')
      if self.stalk[side]:
        cause.add('driver')
      if side in das:
        cause.add('request')
      self.cause[side] = cause
    for side in (LEFT, RIGHT):
      on = bool(self.cause[side])
      touched = self.touch_frames[side] > 0
      if touched:
        self.touch_frames[side] -= 1
      glitch = on and (touched or (self.glitch_rate and self.rng.random() < self.glitch_rate))
      self.blink[side] = self.lamps[side].frame(on, glitch)
      on_bit[side] = self.stalk[side] if self.on_bits == 'driver' else bool(self.cause[side])
    if self.hazard_button:
      # hazards blink both lamps in phase, over anything else (a touch or a glitch darkens both alike)
      self.blink[RIGHT] = self.blink[LEFT]
    self.ui_counter = (self.ui_counter + 1) % 16
    dat = _ui_warning(self.blink[LEFT], self.blink[RIGHT], on_bit[LEFT], on_bit[RIGHT], self.ui_counter)
    return dat, self.blink[LEFT], self.blink[RIGHT], on_bit[LEFT], on_bit[RIGHT]

  def lit(self, side: int) -> bool:
    return self.blink[side] in (1, 2)

  def caused_by(self, side: int) -> set:
    return set(self.cause[side])


__all__ = ['BodyModel', 'STOCK_IDLE', 'BODY_CONTROLS_ADDR', 'LEFT', 'RIGHT', 'NONE', 'SEMANTICS']
