"""A port that works its car's own turn signal, and the read-back mask that keeps its lamp out of the driver's blinker.

TurnPilot (DESIGN phase4/blinker). A port whose only lamp feedback is the lamp itself (Tesla's UI_warning) reads its own
commanded lamp back as the driver's blinker unless it masks it. Above 20 mph a one-sided driver's blinker puts
DesireHelper in preLaneChange, which starts a lane change on any steering nudge toward it, so a lamp of ours shown as
the driver's is the hazard this module exists for. The rules (DESIGN section 5.2), applied per lamp frame:

1. Our side is masked from the decision to request it until, after its release, it reads 0 on two consecutive frames,
   and not before LAMP_OFF_S after the last CANCEL.
2. The other side is masked from the same moment until our side has been lit DRIVER_AFTER_S with the other side dark.
3. A lamp on a masked other side is ours on the wrong side (the port faults wrongSide), unless both rose together. Once
   lit, that side stays masked after ours lifts, until it reads 0 for STUCK_OFF_S: our side never lit, so its own mask
   lifts on time while the wrong-side lamp may still be lit, and it must never read as the driver's.
4. While any side is masked, a frame with both lamps lit (hazards) publishes both blinkers true until both read 0 on
   two consecutive frames, so DesireHelper never sees one side alone.
5-6. The driver's lamp on the other side once unmasked reads true at once; on our side it is absorbed.
7. A lamp already lit when the port starts is masked (inherited) until it reads 0 twice.
8. A masked lamp lit RELEASE_TAIL_S after its release (or the port's start, or its wrong-side rise) is said out loud
   (stillOn).

Outside any mask the published blinkers are the lamps, exactly as before.
"""
from dataclasses import dataclass

NONE, LEFT, RIGHT = 0, 1, 2
SIDES = (LEFT, RIGHT)

LAMP_ON_S = 1.0         # from the first echoed request frame to the lamp lit
LAMP_GAP_S = 0.35       # longest dark run a held lamp may show
DRIVER_AFTER_S = 1.0    # our lamp lit this long before a lamp on the other side counts as the driver's
LAMP_OFF_S = 1.0        # from a CANCEL to the lamp off; then one retry
RELEASE_TAIL_S = 3.5    # from a release to the lamp off, else lampStuck (or the reminder)
STOCK_SEND_FRESH_S = 0.15  # a copy is only sent of a stock frame this fresh
STOCK_LOST_S = 1.0      # once ready, no stock frame this long is noStockFrame
REQUEST_CAP_S = 300.    # any one continuous request
NO_ECHO_S = 0.6         # with no echo of our request at all, LAMP_ON_S counts from the send plus this
OFF_FRAMES = 2          # consecutive 0 frames that read a lamp off
# a lamp still lit LAMP_OFF_S after its release, lit when the port started, or ours on the wrong side (rule 3) must read
# 0 this long before its mask lifts: a stalk touch darkens a lamp 2 to 6 frames, and a latched lamp of ours coming back
# after one must never read as a new driver's signal
STUCK_OFF_S = 1.0
HAZARD_FRAMES = 3       # consecutive both-lit frames before a hold is called hazards rather than the other side


def other(side: int) -> int:
  return RIGHT if side == LEFT else LEFT


@dataclass(frozen=True)
class CarSignalsConfig:
  """What card resolved for this drive from TurnPilotCarSignals and the verified record (DESIGN 7.2)."""
  mode: str = "off"          # off, check or on
  on_reason: int = 8         # the verified ON reason: 8, 9 or 0
  recheck: bool = False      # the record passed in P: re-check at this ignition before signalling
  on_separates: bool = False  # the check found UI_warning's On bit follows only the driver


class BlinkerMask:
  """Rules 1 to 8 of DESIGN 5.2 on the lamp frames; the port says when it claims, lights and releases."""

  def __init__(self):
    # None, 'ours', 'pending' (the other side, rule 2), 'wrong' (a pending side that lit: ours on the wrong side, rule 3)
    # or 'inherited' (rule 7)
    self.mask = {LEFT: None, RIGHT: None}
    self.lamp = {LEFT: False, RIGHT: False}
    self.prev_lamp = {LEFT: False, RIGHT: False}
    self.zeros = {LEFT: 0, RIGHT: 0}       # consecutive frames reading 0
    self.hazard = False
    self.both_run = 0                      # consecutive frames with both lamps lit
    self.started = False
    self.release_ok = {LEFT: False, RIGHT: False}  # ours: the release is done sending, so the mask may lift
    self.stuck = {LEFT: False, RIGHT: False}       # ours stayed lit past the release tail, or inherited: STUCK_OFF_S
    self.lift_after = {LEFT: 0., RIGHT: 0.}        # ours: not before this time (LAMP_OFF_S after the last CANCEL)
    self.pending_lit = {LEFT: False, RIGHT: False}  # a 'pending' side read lit: ours on the wrong side (rule 3)
    self.since = {LEFT: 0., RIGHT: 0.}             # an inherited or wrong-side lamp: rule 8's clock starts here
    self.start_t = 0.

  @property
  def any_masked(self) -> bool:
    return any(m is not None for m in self.mask.values())

  def masked(self, side: int) -> bool:
    return self.mask[side] is not None

  def claim(self, side: int) -> None:
    """The port decided to request side: both sides are masked from now (rules 1, 2)."""
    self.mask[side] = 'ours'
    self.release_ok[side] = False
    self.stuck[side] = False
    self.lift_after[side] = 0.
    if self.mask[other(side)] is None:
      self.mask[other(side)] = 'pending'
      self.pending_lit[other(side)] = False

  def driver_unmask(self, side: int) -> None:
    """Our side has been lit DRIVER_AFTER_S: the other side is the driver's from now (rule 2)."""
    if self.mask[other(side)] == 'pending':
      self.mask[other(side)] = None
      self.pending_lit[other(side)] = False

  def released(self, side: int, lift_after: float) -> None:
    """The port's release frames are done: ours lifts once the lamp reads off, not before lift_after (rule 1)."""
    self.release_ok[side] = True
    self.lift_after[side] = lift_after

  def mark_stuck(self, side: int) -> None:
    """Our lamp is still lit past its release tail: from now on it lifts only after STUCK_OFF_S off."""
    self.stuck[side] = True

  def frame(self, left: bool, right: bool, t: float) -> None:
    """A new lamp frame (UI_warning at 10 Hz)."""
    if not self.started:
      # rule 7: a lamp lit when the port starts may be ours from before a restart, or the driver's: masked till off
      self.started = True
      self.start_t = t
      for side, lit in ((LEFT, left), (RIGHT, right)):
        if lit:
          self.mask[side] = 'inherited'
          self.stuck[side] = True
          self.since[side] = t
    self.prev_lamp = dict(self.lamp)
    self.lamp = {LEFT: left, RIGHT: right}
    for side in SIDES:
      self.zeros[side] = 0 if self.lamp[side] else self.zeros[side] + 1
    self.both_run = self.both_run + 1 if (left and right) else 0

    # rule 4: hazards under a mask publish both blinkers until both read off
    if left and right and self.any_masked:
      self.hazard = True
    if self.hazard and all(self.zeros[s] >= OFF_FRAMES for s in SIDES):
      self.hazard = False

    # rule 3: a lamp on a pending side is ours on the wrong side
    for side in SIDES:
      if self.mask[side] == 'pending' and self.lamp[side] and not self.pending_lit[side]:
        self.pending_lit[side], self.since[side] = True, t

    # rules 1, 3 and 7: masks lift once the lamp reads off (a stuck, wrong-side or inherited lamp: off for STUCK_OFF_S)
    for side in SIDES:
      off_frames = OFF_FRAMES if not self.stuck[side] else max(OFF_FRAMES, int(round(STUCK_OFF_S / 0.1)) + 1)
      off = self.zeros[side] >= off_frames
      if self.mask[side] in ('inherited', 'wrong') and off:
        self.mask[side], self.stuck[side] = None, False
      elif self.mask[side] == 'ours' and off and self.release_ok[side] and t >= self.lift_after[side]:
        self.mask[side], self.stuck[side] = None, False
        # rule 2: a pending other side stays masked for as long as ours does; rule 3: one that lit (ours on the wrong
        # side) stays masked till it reads off STUCK_OFF_S, however long ours took
        o = other(side)
        if self.mask[o] == 'pending':
          self.mask[o], self.stuck[o] = ('wrong', True) if self.pending_lit[o] else (None, False)
          self.pending_lit[o] = False

  def rose(self, side: int) -> bool:
    return self.lamp[side] and not self.prev_lamp[side]

  def blinkers(self) -> tuple[bool, bool]:
    """The driver's blinkers as carState publishes them."""
    if self.hazard:
      return True, True
    return (self.lamp[LEFT] and not self.masked(LEFT), self.lamp[RIGHT] and not self.masked(RIGHT))
