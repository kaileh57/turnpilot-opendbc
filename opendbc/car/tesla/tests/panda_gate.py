"""PandaGate: the real panda safety code (libsafety) between the Tesla port and BodyModel (DESIGN 9.1).

Stock frames go through the fwd and rx hooks, the port's frames through the tx hook, and the body sees only what is
forwarded or accepted, with controls_allowed, gear, standstill and brake driven by the test. So the freshness rule, the
ownership timeout, the CANCEL bound, NONE's ownership rule and the engagement bound act as on the car.

libsafety is one process-wide library: one PandaGate at a time.

PortHarness runs card cycles at 100 Hz with the port, a BodyModel and a PandaGate, the way card, the panda and the car
meet: stock frames reach the port on src 2 and the car through the panda, the port's frames are echoed on src 128
when accepted and on src 192 when refused, and UI_warning reaches the port at 10 Hz.
"""
import random

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.car_signal import NONE
from opendbc.car.tesla.body_model import BodyModel
from opendbc.car.tesla.teslacan import BODY_CONTROLS_ADDR
from opendbc.car.tesla.values import TeslaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py

GearShifter = structs.CarState.GearShifter
GEAR_CAN = {GearShifter.park: 1, GearShifter.reverse: 2, GearShifter.neutral: 3, GearShifter.drive: 4}


class PandaGate:
  def __init__(self, check: bool = False, longitudinal: bool = False):
    self.safety = libsafety_py.libsafety
    param = int(TeslaSafetyFlags.CAR_SIGNALS)
    if check:
      param |= int(TeslaSafetyFlags.CAR_SIGNALS_CHECK)
    if longitudinal:
      param |= int(TeslaSafetyFlags.LONG_CONTROL)
    self.param = param
    self.reset()
    self.packer = CANPacker('tesla_model3_party')
    self.accepted = 0
    self.refused = 0
    self.forwarded = 0
    self.blocked = 0
    self.sent: list[tuple[float, bytes, bool]] = []

  def reset(self) -> None:
    assert self.safety.set_safety_hooks(structs.CarParams.SafetyModel.tesla, self.param) == 0
    self.safety.init_tests()  # clears the assumed autopark at init

  def set_time(self, t: float) -> None:
    self.safety.set_timer(int(t * 1e6) % 0xFFFFFFFF)

  def _rx(self, name, bus, values):
    addr, dat, _ = self.packer.make_can_msg(name, bus, values)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))

  def car(self, engaged: bool, standstill: bool, gear=GearShifter.drive, brake: bool = False) -> None:
    """The car's state as the panda reads it: gear (DI_systemStatus), standstill (ESP_B), brake (ESP_status), and
    controls_allowed set directly (engagement itself is the panda's other tests)."""
    self._rx("DI_systemStatus", 0, {"DI_gear": GEAR_CAN.get(gear, 0)})
    self._rx("ESP_B", 0, {"ESP_vehicleStandstillSts": 1 if standstill else 0, "ESP_wheelSpeedsQF": 1})
    self._rx("ESP_status", 0, {"ESP_driverBrakeApply": 2 if brake else 1})
    self.safety.set_controls_allowed(engaged)

  def stock(self, dat: bytes) -> bool:
    """A stock 0x3E9 from the Autopilot side: the fwd hook decides first, then the rx hook stores it (fdcan.h order).
    Returns whether it reaches the car."""
    fwd = self.safety.safety_fwd_hook(2, BODY_CONTROLS_ADDR) == 0
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(BODY_CONTROLS_ADDR, 2, dat))
    self.forwarded += fwd
    self.blocked += not fwd
    return fwd

  def send(self, dat: bytes, t: float = 0.) -> bool:
    ok = bool(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(BODY_CONTROLS_ADDR, 0, dat)))
    self.accepted += ok
    self.refused += not ok
    self.sent.append((t, bytes(dat), ok))
    return ok

  @property
  def controls_allowed(self) -> bool:
    return bool(self.safety.get_controls_allowed())


class PortHarness:
  """Card, panda and car at 100 Hz around a TeslaCarSignal (the port), driven by the test."""

  def __init__(self, port, body: BodyModel | None = None, gate: PandaGate | None = None, rng=None, t0: float = 100.):
    self.port = port
    self.rng = rng or random.Random(0)
    self.body = body or BodyModel(rng=self.rng)
    self.gate = gate or PandaGate(check=port.check_mode)
    self.t = t0
    self.body.next_stock_t = t0 + self.rng.uniform(0., 0.5)
    self.body.next_ui_t = t0 + self.rng.uniform(0., 0.1)
    self.pending_echo: list[tuple[bytes, int]] = []
    self.want = NONE
    self.engaged = False
    self.standstill = False
    self.gear = GearShifter.drive
    self.brake = False
    self.ui = (0, 0, False, False)
    self.blinkers: list[tuple[float, bool, bool]] = []
    self.sent: list[tuple[float, bytes, bool]] = []
    self.card_alive = True
    self.cycle_jitter = 0.

  def step(self, n: int = 1) -> None:
    for _ in range(n):
      self._step()

  def run(self, seconds: float) -> None:
    self.step(int(round(seconds / 0.01)))

  def _step(self) -> None:
    self.t += 0.01
    t = self.t
    self.gate.set_time(t)
    self.gate.car(self.engaged, self.standstill, self.gear, self.brake)
    stock, ui = self.body.tick(t)
    nanos = int(t * 1e9)
    frames = [(BODY_CONTROLS_ADDR, dat, src) for dat, src in self.pending_echo]
    self.pending_echo = []
    for dat in stock:
      if self.gate.stock(dat):
        self.body.receive(dat, t)
        frames.append((BODY_CONTROLS_ADDR, dat, 128))
      frames.append((BODY_CONTROLS_ADDR, dat, 2))
    if not self.card_alive:
      return
    self.port.can([(nanos, frames)])
    new = ui is not None
    if new:
      self.ui = ui[1:]
    left, right = self.port.lamps(new, self.ui[0], self.ui[1], self.ui[2], self.ui[3], t,
                                  standstill=self.standstill, gear=self.gear, brake=self.brake)
    self.blinkers.append((t, left, right))
    send_t = t + self.cycle_jitter
    for addr, dat, bus in self.port.control(self.want, self.engaged, int(send_t * 1e9)):
      assert addr == BODY_CONTROLS_ADDR and bus == 0
      ok = self.gate.send(dat, t)
      self.sent.append((t, dat, ok))
      if ok:
        self.body.receive(dat, t)
      self.pending_echo.append((dat, 128 if ok else 192))
