#!/usr/bin/env python3
import random
import unittest
import numpy as np

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.tesla.values import CarControllerParams, TeslaSafetyFlags
from opendbc.car.tesla.carcontroller import get_safety_CP
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.can import CANDefine
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import MAX_WRONG_COUNTERS, away_round, round_speed, speeds_within_delta

MSG_DAS_steeringControl = 0x488
MSG_APS_eacMonitor = 0x27d
MSG_DAS_Control = 0x2b9
MSG_DAS_bodyControls = 0x3E9

# TurnPilot's commanded turn signal: the measured idle stock DAS_bodyControls (2025 HW4 Model 3, src 2)
BODY_IDLE = bytes.fromhex("00880220000010a6")
BODY_NONE, BODY_LEFT, BODY_RIGHT, BODY_CANCEL = 0, 1, 2, 3
BODY_ON_REASONS = (0, 8, 9)
BODY_CANCEL_REASON = 15
# every bit the panda protects: bytes 0, 2, 3, 5; byte 1 bits 3-7; byte 4 bits 0, 6, 7; byte 6 bits 0-3
BODY_PROTECTED_BITS = ([b for byte in (0, 2, 3, 5) for b in range(byte * 8, byte * 8 + 8)] +
                       [8 + i for i in range(3, 8)] + [32, 38, 39] + [48 + i for i in range(4)])


def body_checksum(dat) -> int:
  return (0xE9 + 0x03 + sum(dat[:7])) & 0xFF


def body_frame(stock, request=None, reason=None, counter=None, fix_checksum=True) -> bytearray:
  """A DAS_bodyControls frame from a stock frame, with the request (8|3), reason (33|5) and counter (52|4) replaced."""
  dat = bytearray(stock)
  if request is not None:
    dat[1] = (dat[1] & 0xF8) | request
  if reason is not None:
    dat[4] = (dat[4] & 0xC1) | (reason << 1)
  if counter is not None:
    dat[6] = (dat[6] & 0x0F) | ((counter & 0xF) << 4)
  if fix_checksum:
    dat[7] = body_checksum(dat)
  return dat


def round_angle(apply_angle, can_offset=0):
  apply_angle_can = (apply_angle + 1638.35) / 0.1 + can_offset
  # 0.49999_ == 0.5
  rnd_offset = 1e-5 if apply_angle >= 0 else -1e-5
  return away_round(apply_angle_can + rnd_offset) * 0.1 - 1638.35


class TestTeslaSafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest, common.LongitudinalAccelSafetyTest):

  DBC = "tesla_model3_party"
  SAFETY_MODEL = CarParams.SafetyModel.tesla

  RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor]}
  TX_MSGS = [[MSG_DAS_steeringControl, 0], [MSG_APS_eacMonitor, 0], [MSG_DAS_Control, 0]]

  STANDSTILL_THRESHOLD = 0.1
  GAS_PRESSED_THRESHOLD = 3

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  # Long control limits
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.48
  INACTIVE_ACCEL = 0.0

  cnt_angle_cmd = 0

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    super().setUp()
    self.VM = VehicleModel(get_safety_CP())
    self.define = CANDefine(self.DBC)
    self.acc_states = {d: v for v, d in self.define.dv["DAS_control"]["DAS_accState"].items()}
    self.autopark_states = {d: v for v, d in self.define.dv["DI_state"]["DI_autoparkState"].items()}
    self.active_autopark_states = [self.autopark_states[s] for s in ('ACTIVE', 'COMPLETE', 'SELFPARK_STARTED')]

    self.steer_control_types = {d: v for v, d in self.define.dv["DAS_steeringControl"]["DAS_steeringControlType"].items()}

  def _angle_cmd_msg(self, angle: float, state: bool | int, increment_timer: bool = True, bus: int = 0):
    values = {"DAS_steeringAngleRequest": angle, "DAS_steeringControlType": int(state)}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("DAS_steeringControl", bus, values)

  def _angle_meas_msg(self, angle: float, hands_on_level: int = 0, eac_status: int = 1, eac_error_code: int = 0):
    values = {"EPAS3S_internalSAS": angle, "EPAS3S_handsOnLevel": hands_on_level,
              "EPAS3S_eacStatus": eac_status, "EPAS3S_eacErrorCode": eac_error_code}
    return self.packer.make_can_msg_safety("EPAS3S_sysStatus", 0, values)

  def _user_brake_msg(self, brake, quality_flag: bool = True):
    values = {"ESP_driverBrakeApply": 2 if brake else 1}
    if not quality_flag:
      values["ESP_driverBrakeApply"] = random.choice((0, 3))  # NotInit_orOff, Faulty_SNA
    return self.packer.make_can_msg_safety("ESP_status", 0, values)

  def _speed_msg(self, speed):
    values = {"DI_vehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_safety("DI_speed", 0, values)

  def _speed_msg_2(self, speed, quality_flag=True):
    values = {"ESP_vehicleSpeed": speed * 3.6, "ESP_wheelSpeedsQF": quality_flag}
    return self.packer.make_can_msg_safety("ESP_B", 0, values)

  def _vehicle_moving_msg(self, speed: float, quality_flag=True):
    values = {"ESP_vehicleStandstillSts": 1 if speed <= self.STANDSTILL_THRESHOLD else 0,
              "ESP_wheelSpeedsQF": quality_flag}
    return self.packer.make_can_msg_safety("ESP_B", 0, values)

  def _user_gas_msg(self, gas):
    values = {"DI_accelPedalPos": gas}
    return self.packer.make_can_msg_safety("DI_systemStatus", 0, values)

  def _pcm_status_msg(self, enable, autopark_state=0):
    values = {
      "DI_cruiseState": 2 if enable else 0,
      "DI_autoparkState": autopark_state,
    }
    return self.packer.make_can_msg_safety("DI_state", 0, values)

  def _long_control_msg(self, set_speed, acc_state=0, jerk_limits=(0, 0), accel_limits=(0, 0), aeb_event=0, bus=0):
    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": aeb_event,
      "DAS_jerkMin": jerk_limits[0],
      "DAS_jerkMax": jerk_limits[1],
      "DAS_accelMin": accel_limits[0],
      "DAS_accelMax": accel_limits[1],
    }
    return self.packer.make_can_msg_safety("DAS_control", bus, values)

  def _accel_msg(self, accel: float):
    # For common.LongitudinalAccelSafetyTest
    return self._long_control_msg(10, accel_limits=(accel, max(accel, 0)))

  def test_rx_hook(self):
    # counter check
    for msg_type in ("angle", "long", "speed", "speed_2"):
      # send multiple times to verify counter checks
      for i in range(10):
        if msg_type == "angle":
          msg = self._angle_cmd_msg(0, True, bus=2)
        elif msg_type == "long":
          msg = self._long_control_msg(0, bus=2)
        elif msg_type == "speed":
          msg = self._speed_msg(0)
        elif msg_type == "speed_2":
          msg = self._speed_msg_2(0)

        should_rx = i >= 5
        if not should_rx:
          # mess with checksums
          if msg_type == "angle":
            msg[0].data[3] = 0
          elif msg_type == "long":
            msg[0].data[7] = 0
          elif msg_type == "speed":
            msg[0].data[0] = 0
          elif msg_type == "speed_2":
            msg[0].data[7] = 0

        self.safety.set_controls_allowed(True)
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

      # Send static counters
      for i in range(MAX_WRONG_COUNTERS + 1):
        should_rx = i + 1 < MAX_WRONG_COUNTERS
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

  def test_vehicle_speed_measurements(self):
    # OVERRIDDEN: 79.1667 is the max speed in m/s
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_rx_hook_speed_mismatch(self):
    # TODO: overridden because of custom rounding
    # Tesla relies on speed for lateral limits close to ISO 11270, so it checks two sources
    for speed in np.arange(0, 40, 0.5):
      # match signal rounding on CAN
      speed = away_round(speed / 0.08 * 3.6) * 0.08 / 3.6
      for speed_delta in np.arange(-5, 5, 0.1):
        speed_2 = max(speed + speed_delta, 0)
        speed_2 = away_round(speed_2 * 2 * 3.6) / 2 / 3.6

        # Set controls allowed in between rx since first message can reset it
        self.assertTrue(self._rx(self._speed_msg(speed)))
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._rx(self._speed_msg_2(speed_2)))

        within_delta = speeds_within_delta(speed, speed_2)
        self.assertEqual(self.safety.get_controls_allowed(), within_delta)

    # Test ESP_B quality flag
    for quality_flag in (True, False):
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._rx(self._speed_msg(0)))
      self.assertEqual(quality_flag, self._rx(self._speed_msg_2(0, quality_flag=quality_flag)))
      self.assertEqual(quality_flag, self.safety.get_controls_allowed())

  def test_user_brake_quality_flag(self):
    for quality_flag in (True, False):
      msg = self._user_brake_msg(True, quality_flag=quality_flag)
      self.assertEqual(quality_flag, self._rx(msg))

  def test_steering_wheel_disengage(self):
    # Tesla disengages when the user forcibly overrides the locked-in angle steering control
    # Either when the hands on level is high, or if there is a high angle rate fault
    for hands_on_level in range(4):
      for eac_status in range(8):
        for eac_error_code in range(16):
          self.safety.set_controls_allowed(True)

          should_disengage = hands_on_level >= 3 or (eac_status == 0 and eac_error_code == 9)
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=hands_on_level, eac_status=eac_status,
                                                        eac_error_code=eac_error_code)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertEqual(should_disengage, self.safety.get_steering_disengage_prev())

          # Should not recover
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=1, eac_error_code=0)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertFalse(self.safety.get_steering_disengage_prev())

  def test_autopark_summon_while_enabled(self):
    # We should not respect Autopark that activates while controls are allowed
    self._rx(self._pcm_status_msg(True, 0))

    self._rx(self._pcm_status_msg(True, self.autopark_states["SELFPARK_STARTED"]))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))

    # We should still not respect Autopark if we disengage cruise
    self._rx(self._pcm_status_msg(False, self.autopark_states["SELFPARK_STARTED"]))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
    self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))

  def test_autopark_summon_behavior(self):
    for autopark_state in range(16):
      self._rx(self._pcm_status_msg(False, 0))

      # We shouldn't allow controls if Autopark is an active state
      autopark_active = autopark_state in self.active_autopark_states
      self._rx(self._pcm_status_msg(False, autopark_state))
      self._rx(self._pcm_status_msg(True, autopark_state))
      self.assertNotEqual(autopark_active, self.safety.get_controls_allowed())

      # We should also start blocking all inactive/active openpilot msgs
      self.assertNotEqual(autopark_active, self._tx(self._angle_cmd_msg(0, False)))
      self.assertNotEqual(autopark_active, self._tx(self._angle_cmd_msg(0, True)))
      self.assertNotEqual(autopark_active, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))
      self.assertNotEqual(autopark_active or not self.LONGITUDINAL, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_ON"])))

      # Regain controls when Autopark disables
      self._rx(self._pcm_status_msg(True, 0))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
      self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))
      self.assertEqual(self.LONGITUDINAL, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_ON"])))

  def test_steering_control_type(self):
    # Only angle control is allowed (no LANE_KEEP_ASSIST, EMERGENCY_LANE_KEEP, or FSD)
    self.safety.set_controls_allowed(True)
    for steer_control_type in range(8):
      should_tx = steer_control_type in (self.steer_control_types["NONE"],
                                         self.steer_control_types["ANGLE_CONTROL"])
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, state=steer_control_type)))

  def test_stock_lkas_passthrough(self):
    # TODO: make these generic passthrough tests
    no_lkas_msg = self._angle_cmd_msg(0, state=False)
    no_lkas_msg_cam = self._angle_cmd_msg(0, state=True, bus=2)
    lkas_msg_cam = self._angle_cmd_msg(0, state=self.steer_control_types['LANE_KEEP_ASSIST'], bus=2)

    # stock system sends no LKAS -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_lkas_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_lkas_msg_cam.addr))
    self.assertTrue(self._tx(no_lkas_msg))

    # stock system sends LKAS -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(lkas_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, lkas_msg_cam.addr))
    self.assertFalse(self._tx(no_lkas_msg))

  def test_car_signals_off(self):
    # without CAR_SIGNALS: no 0x3E9 TX, no 0x3E9 RX check, the stock frame always forwarded, and the gear changes nothing
    self.assertEqual(9, self.safety.get_rx_checks_len())
    stock = body_frame(BODY_IDLE, counter=0)
    self.assertTrue(self._rx(common.make_msg(2, MSG_DAS_bodyControls, dat=bytes(stock))))
    for gear in range(8):
      self.assertTrue(self._rx(self.packer.make_can_msg_safety("DI_systemStatus", 0, {"DI_gear": gear})))
      self.assertTrue(self._rx(self._vehicle_moving_msg(0.0)))
      for request in range(4):
        self.safety.set_controls_allowed(True)
        dat = body_frame(stock, request=request, reason=8 if request else 0, counter=1)
        self.assertFalse(self._tx(common.make_msg(0, MSG_DAS_bodyControls, dat=bytes(dat))))
        self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_DAS_bodyControls))

  # Tesla uses get_max_angle_delta_vm and get_max_angle_vm for real lateral accel and jerk limits
  # TODO: integrate this into AngleSteeringSafetyTest
  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match DI_vehicleSpeed rounding on CAN
      speed = round_speed(away_round(speed / 0.08 * 3.6) * 0.08 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        # angle signal can't represent 0, so it biases one unit down
        angle_unit_offset = -1 if sign == -1 else 0

        # at limit (safety tolerance adds 1)
        max_angle = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 1) * sign
        max_angle = np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

        # 1 unit above limit
        max_angle_raw = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 2) * sign
        max_angle = np.clip(max_angle_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self._tx(self._angle_cmd_msg(max_angle, True))

        # at low speeds max angle is above 360, so adding 1 has no effect
        should_tx = abs(max_angle_raw) >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(max_angle, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match DI_vehicleSpeed rounding on CAN
      speed = round_speed(away_round(speed / 0.08 * 3.6) * 0.08 / 3.6)
      for sign in (-1, 1):  # (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        # angle signal can't represent 0, so it biases one unit down
        angle_unit_offset = 1 if sign == -1 else 0

        # Stay within limits
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams), angle_unit_offset) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 1) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


class TeslaCarSignalsMixin:
  """TurnPilot's commanded turn signal (DESIGN section 3.7): the panda's rules for openpilot's DAS_bodyControls.

  Mixed into a Tesla safety test class whose SAFETY_PARAM carries CAR_SIGNALS. Time is set explicitly with set_timer.
  """
  CHECK = False

  def setUp(self):
    super().setUp()
    self.body_counter = 0
    self.body_stock = bytes(BODY_IDLE)

  def _body_rx(self, dat, bus=2):
    return self._rx(common.make_msg(bus, MSG_DAS_bodyControls, dat=bytes(dat)))

  def _stock(self, dat=None, counter=None):
    """The DAS sends its next stock frame (counter +1) on bus 2; returns it."""
    if counter is None:
      counter = self.body_counter
    self.body_counter = (counter + 1) % 16
    stock = body_frame(BODY_IDLE if dat is None else dat, counter=counter)
    self.assertTrue(self._body_rx(stock))
    self.body_stock = bytes(stock)
    return stock

  def _body_tx(self, request, reason=None, stock=None, counter=None, dat=None):
    stock = self.body_stock if stock is None else stock
    if dat is None:
      if reason is None:
        reason = {BODY_LEFT: 8, BODY_RIGHT: 8, BODY_CANCEL: BODY_CANCEL_REASON}.get(request, (stock[4] >> 1) & 0x1F)
      if counter is None:
        counter = ((stock[6] >> 4) + 1) % 16
      dat = body_frame(stock, request=request, reason=reason, counter=counter)
    return self._tx(common.make_msg(0, MSG_DAS_bodyControls, dat=bytes(dat)))

  def _gear_msg(self, gear: int, gas: float = 0):
    return self.packer.make_can_msg_safety("DI_systemStatus", 0, {"DI_gear": gear, "DI_accelPedalPos": gas})

  def _park(self, gear=1, moving=False):
    self.assertTrue(self._rx(self._gear_msg(gear)))
    self.assertTrue(self._rx(self._vehicle_moving_msg(1.0 if moving else 0.0)))

  def _fwd_body(self):
    return self.safety.safety_fwd_hook(2, MSG_DAS_bodyControls)

  def _engaged(self):
    self.safety.set_controls_allowed(True)

  def _reset(self):
    self._reset_safety_hooks()
    self.safety.init_tests()

  def test_car_signals_off(self):
    # OVERRIDDEN: with CAR_SIGNALS the body message is the tenth RX check
    self.assertEqual(10, self.safety.get_rx_checks_len())

  # 1. freshness and storage of the stock frame
  def test_body_no_stock_frame(self):
    self._engaged()
    for request in range(8):
      self.assertFalse(self._body_tx(request, stock=BODY_IDLE))

  def test_body_copy_fresh(self):
    self._engaged()
    self.safety.set_timer(1_000_000)
    self._stock()
    self.assertTrue(self._body_tx(BODY_LEFT))
    self.safety.set_timer(1_000_000 + 249_999)
    self.assertTrue(self._body_tx(BODY_LEFT))
    self.safety.set_timer(1_000_000 + 250_000)
    self.assertFalse(self._body_tx(BODY_LEFT))
    self.safety.set_timer(1_000_000 + 251_000)
    self.assertFalse(self._body_tx(BODY_LEFT))

  def test_body_bad_checksum_not_stored(self):
    self._engaged()
    stock = body_frame(BODY_IDLE, counter=0)
    stock[7] = (stock[7] + 1) & 0xFF
    self.assertFalse(self._body_rx(stock))
    self.assertFalse(self._body_tx(BODY_LEFT, stock=body_frame(stock)))
    # and a later bad one doesn't replace a stored good one
    self.safety.set_timer(10_000)
    good = self._stock(counter=1)
    bad = body_frame(BODY_IDLE, counter=2)
    bad[0] = 0x01
    bad[7] = (body_checksum(bad) + 1) & 0xFF
    self.assertFalse(self._body_rx(bad))
    self.assertFalse(self._body_tx(BODY_LEFT, stock=body_frame(bad)))
    self.assertTrue(self._body_tx(BODY_LEFT, stock=good))

  def test_body_bad_counter_not_stored(self):
    self._engaged()
    for _ in range(MAX_WRONG_COUNTERS + 1):
      self._body_rx(body_frame(BODY_IDLE, counter=3))
    stale = body_frame(BODY_IDLE, counter=3)
    stale[0] = 0x04
    stale = body_frame(stale)
    self.assertFalse(self._body_rx(stale))
    self.assertFalse(self._body_tx(BODY_LEFT, stock=stale))

  # 2. nothing but the request, reason, counter and checksum may change
  def test_body_protected_bits(self):
    self.assertEqual(44, len(BODY_PROTECTED_BITS))
    self._engaged()
    for bit in BODY_PROTECTED_BITS:
      with self.subTest(bit=bit):
        stock = self._stock()
        dat = body_frame(stock, request=BODY_LEFT, reason=8, counter=(stock[6] >> 4) + 1)
        self.assertTrue(self._body_tx(None, dat=dat))
        dat[bit // 8] ^= 1 << (bit % 8)
        dat[7] = body_checksum(dat)
        self.assertFalse(self._body_tx(None, dat=dat))

  def test_body_other_stock_content_copied(self):
    # high beams, wipers and every other stock bit ride through as the DAS sent them
    self._engaged()
    for bit in BODY_PROTECTED_BITS:
      with self.subTest(bit=bit):
        content = bytearray(BODY_IDLE)
        content[bit // 8] ^= 1 << (bit % 8)
        self._stock(dat=content)
        self.assertTrue(self._body_tx(BODY_LEFT))
        self.assertFalse(self._body_tx(BODY_LEFT, stock=BODY_IDLE))

  # 3. counter and checksum
  def test_body_counter(self):
    self._engaged()
    for stock_counter in range(16):
      stock = self._stock(counter=stock_counter)
      for counter in range(16):
        with self.subTest(stock_counter=stock_counter, counter=counter):
          self.assertEqual(counter == (stock_counter + 1) % 16, self._body_tx(BODY_LEFT, stock=stock, counter=counter))

  def test_body_checksum(self):
    self._engaged()
    stock = self._stock()
    dat = body_frame(stock, request=BODY_LEFT, reason=8, counter=(stock[6] >> 4) + 1)
    for delta in (-1, 1):
      bad = bytearray(dat)
      bad[7] = (bad[7] + delta) & 0xFF
      self.assertFalse(self._body_tx(None, dat=bad))
    self.assertTrue(self._body_tx(None, dat=dat))

  # 4. requests and reasons
  def test_body_request_values(self):
    self._engaged()
    for request in range(4, 8):
      self._stock()
      for reason in range(32):
        self.assertFalse(self._body_tx(request, reason=reason))

  def test_body_on_reasons(self):
    for request in (BODY_LEFT, BODY_RIGHT):
      for reason in range(32):
        with self.subTest(request=request, reason=reason):
          self._stock()
          self._engaged()
          self.assertEqual(reason in BODY_ON_REASONS, self._body_tx(request, reason=reason))
          self.safety.set_controls_allowed(False)
          self._park(gear=4, moving=True)
          self.assertFalse(self._body_tx(request, reason=reason))
          self._park(gear=4, moving=False)
          self.assertFalse(self._body_tx(request, reason=reason))

  # 5. NONE ends ownership and needs it
  def test_body_none_needs_ownership(self):
    self._engaged()
    self._stock()
    self.assertFalse(self._body_tx(BODY_NONE))
    self.assertTrue(self._body_tx(BODY_LEFT))
    self._stock()
    self.assertTrue(self._body_tx(BODY_NONE))
    self._stock()
    self.assertFalse(self._body_tx(BODY_NONE))

  def test_body_none_after_ownership_timeout(self):
    self._engaged()
    self.safety.set_timer(1_000_000)
    self._stock()
    self.assertTrue(self._body_tx(BODY_LEFT))
    self.safety.set_timer(1_751_000)
    self._stock()
    self.assertEqual(0, self._fwd_body())
    self.assertTrue(self._body_tx(BODY_NONE))
    self.assertFalse(self._body_tx(BODY_NONE))

  def test_body_none_keeps_stock_reason(self):
    self._engaged()
    stock = self._stock(dat=body_frame(BODY_IDLE, reason=4))
    self.assertTrue(self._body_tx(BODY_LEFT))
    for reason in range(32):
      self.assertEqual(reason == 4, self._body_tx(BODY_NONE, reason=reason, stock=stock))
      if reason == 4:
        self.assertTrue(self._body_tx(BODY_LEFT))

  # 6. CANCEL is bounded to the request it ends
  def test_body_cancel_needs_request(self):
    self._engaged()
    self._stock()
    self.assertFalse(self._body_tx(BODY_CANCEL))
    self.assertTrue(self._body_tx(BODY_LEFT))
    self.assertTrue(self._body_tx(BODY_CANCEL))

  def test_body_cancel_window(self):
    for elapsed, ok in ((0, True), (2_999_999, True), (3_000_000, False), (3_001_000, False)):
      with self.subTest(elapsed=elapsed):
        self._reset()
        self._engaged()
        self.safety.set_timer(1_000_000)
        self._stock()
        self.assertTrue(self._body_tx(BODY_LEFT))
        self.safety.set_timer(1_000_000 + elapsed)
        self._stock()
        self.assertEqual(ok, self._body_tx(BODY_CANCEL))

  def test_body_cancel_count(self):
    self._engaged()
    self._stock()
    self.assertTrue(self._body_tx(BODY_RIGHT))
    self.assertTrue(self._body_tx(BODY_CANCEL))
    self.assertTrue(self._body_tx(BODY_CANCEL))
    self.assertFalse(self._body_tx(BODY_CANCEL))
    # a new request resets the count
    self.assertTrue(self._body_tx(BODY_RIGHT))
    self.assertTrue(self._body_tx(BODY_CANCEL))
    self.assertTrue(self._body_tx(BODY_CANCEL))
    self.assertFalse(self._body_tx(BODY_CANCEL))
    # a rejected request doesn't reset it
    self.assertFalse(self._body_tx(BODY_RIGHT, reason=1))
    self.assertFalse(self._body_tx(BODY_CANCEL))

  def test_body_cancel_reason(self):
    self._engaged()
    self._stock()
    self.assertTrue(self._body_tx(BODY_LEFT))
    for reason in range(32):
      self.assertEqual(reason == BODY_CANCEL_REASON, self._body_tx(BODY_CANCEL, reason=reason))
      self.assertTrue(self._body_tx(BODY_LEFT))

  def test_body_cancel_without_controls(self):
    self._engaged()
    self._stock()
    self.assertTrue(self._body_tx(BODY_LEFT))
    self.safety.set_controls_allowed(False)
    self._park(gear=4, moving=True)
    self.assertFalse(self._body_tx(BODY_LEFT))
    self.assertTrue(self._body_tx(BODY_CANCEL))
    self.assertTrue(self._body_tx(BODY_NONE))

  # 7. the standstill allowances
  def test_body_parked(self):
    for gear in range(8):
      for moving in (False, True):
        for brake in (False, True):
          with self.subTest(gear=gear, moving=moving, brake=brake):
            self._reset()
            self._stock()
            self._park(gear=gear, moving=moving)
            self.assertTrue(self._rx(self._user_brake_msg(brake)))
            self.safety.set_controls_allowed(False)
            parked = gear == 1 and not moving
            stopped_in_d = self.CHECK and gear == 4 and not moving and brake
            self.assertEqual(parked or stopped_in_d, self._body_tx(BODY_LEFT))
            self.assertEqual(parked or stopped_in_d, self._body_tx(BODY_RIGHT))

  def test_body_gear_bits(self):
    # DI_gear is byte 2 bits 5-7; the other bits of byte 2 don't move it
    for low in (0x00, 0x1F, 0x15, 0x0A):
      for gear in range(8):
        with self.subTest(low=low, gear=gear):
          self._reset()
          self._stock()
          values = {"DI_gear": gear, "DI_systemState": low & 0x7, "DI_brakePedalState": (low >> 3) & 0x3}
          self.assertTrue(self._rx(self.packer.make_can_msg_safety("DI_systemStatus", 0, values)))
          self.assertTrue(self._rx(self._vehicle_moving_msg(0.0)))
          self.assertEqual(gear == 1, self._body_tx(BODY_LEFT))

  def test_body_park_then_drive(self):
    self._stock()
    self._park(gear=1)
    self.assertTrue(self._body_tx(BODY_LEFT))
    self._park(gear=4)
    self.assertFalse(self._body_tx(BODY_LEFT))
    self.assertTrue(self._body_tx(BODY_CANCEL))

  # 8. forwarding
  def test_body_forwarding(self):
    self._engaged()
    self.safety.set_timer(1_000_000)
    self._stock()
    self.assertEqual(0, self._fwd_body())
    self.assertTrue(self._body_tx(BODY_LEFT))
    self.assertEqual(-1, self._fwd_body())
    self.safety.set_timer(1_500_000)
    self._stock()
    self.assertTrue(self._body_tx(BODY_CANCEL))
    self.assertEqual(-1, self._fwd_body())
    self.assertTrue(self._body_tx(BODY_NONE))
    self.assertEqual(0, self._fwd_body())

  def test_body_forwarding_timeout(self):
    for elapsed, blocked in ((0, True), (749_999, True), (750_000, False), (751_000, False)):
      with self.subTest(elapsed=elapsed):
        self._reset()
        self._engaged()
        self.safety.set_timer(2_000_000)
        self._stock()
        self.assertTrue(self._body_tx(BODY_LEFT))
        self.safety.set_timer(2_000_000 + elapsed)
        self.assertEqual(-1 if blocked else 0, self._fwd_body())

  def test_body_rejected_request_forwards(self):
    self._stock()
    self.safety.set_controls_allowed(False)
    self._park(gear=4, moving=True)
    self.assertFalse(self._body_tx(BODY_LEFT))
    self.assertEqual(0, self._fwd_body())

  def test_body_autopark_forwards(self):
    self._engaged()
    self._stock()
    self.assertTrue(self._body_tx(BODY_LEFT))
    self.assertEqual(-1, self._fwd_body())
    self._rx(self._pcm_status_msg(False, self.autopark_states["ACTIVE"]))
    self.assertEqual(0, self._fwd_body())
    self.assertFalse(self._body_tx(BODY_LEFT))

  def test_body_other_bus_forwarded(self):
    self._engaged()
    self._stock()
    self.assertTrue(self._body_tx(BODY_LEFT))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, MSG_DAS_bodyControls))
    self.assertEqual(-1, self._fwd_body())

  # 9. observe-only: the body message never touches controls_allowed
  def _feed_rx_checks(self, body=True):
    self.assertTrue(self._rx(self._long_control_msg(0, bus=2)))
    self.assertTrue(self._rx(self._angle_cmd_msg(0, False, increment_timer=False, bus=2)))
    self.assertTrue(self._rx(self._speed_msg(0)))
    self.assertTrue(self._rx(self._speed_msg_2(0)))
    self.assertTrue(self._rx(self._angle_meas_msg(0)))
    self.assertTrue(self._rx(self._user_gas_msg(0)))
    self.assertTrue(self._rx(self._user_brake_msg(False)))
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self._rx(self.packer.make_can_msg_safety("UI_warning", 0, {"buckleStatus": 1})))
    if body:
      self._stock()

  def test_body_observe_only_missing(self):
    self.assertEqual(10, self.safety.get_rx_checks_len())
    self._feed_rx_checks(body=False)
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(10_000_000)
    self._feed_rx_checks(body=False)
    self.safety.safety_tick()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_safety_rx_checks_invalid())
    self.assertTrue(self.safety.safety_config_valid())

  def test_body_observe_only_lagging(self):
    self._feed_rx_checks()
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(10_000_000)
    self._feed_rx_checks(body=False)
    self.safety.safety_tick()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_safety_rx_checks_invalid())

  def test_body_observe_only_bad_checksum(self):
    self._feed_rx_checks()
    self.safety.set_controls_allowed(True)
    bad = body_frame(BODY_IDLE, counter=self.body_counter)
    bad[7] ^= 0xFF
    self.assertFalse(self._body_rx(bad))
    self.assertTrue(self.safety.get_controls_allowed())
    self._feed_rx_checks(body=False)
    self.safety.safety_tick()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_safety_rx_checks_invalid())

  def test_body_observe_only_bad_counters(self):
    self._feed_rx_checks()
    self.safety.set_controls_allowed(True)
    for _ in range(MAX_WRONG_COUNTERS + 2):
      self._body_rx(body_frame(BODY_IDLE, counter=7))
    self.assertTrue(self.safety.get_controls_allowed())
    self._feed_rx_checks(body=False)
    self.safety.safety_tick()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_safety_rx_checks_invalid())

  def test_regular_check_still_disengages(self):
    # beside the observe-only check, a regular one lagging still clears controls_allowed
    self._feed_rx_checks()
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(10_000_000)
    self._feed_rx_checks(body=True)
    self.safety.safety_tick()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(13_000_000)
    self._stock()
    self.safety.safety_tick()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_safety_rx_checks_invalid())

  def test_regular_check_bad_checksum_still_disengages(self):
    self._feed_rx_checks()
    self.safety.set_controls_allowed(True)
    msg = self._angle_cmd_msg(0, False, increment_timer=False, bus=2)
    msg[0].data[3] = (msg[0].data[3] + 1) & 0xFF
    self.assertFalse(self._rx(msg))
    self.assertFalse(self.safety.get_controls_allowed())


class TestTeslaStockSafety(TestTeslaSafetyBase):

  LONGITUDINAL = False

  def test_cancel(self):
    for acc_state in range(16):
      self.safety.set_controls_allowed(True)
      should_tx = acc_state == self.acc_states["ACC_CANCEL_GENERIC_SILENT"]
      self.assertFalse(self._tx(self._long_control_msg(0, acc_state=acc_state, accel_limits=(self.MIN_ACCEL, self.MAX_ACCEL))))
      self.assertEqual(should_tx, self._tx(self._long_control_msg(0, acc_state=acc_state)))

  def test_no_aeb(self):
    for aeb_event in range(4):
      should_tx = aeb_event == 0
      ret = self._tx(self._long_control_msg(10, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"], aeb_event=aeb_event))
      self.assertEqual(ret, should_tx)

  def test_stock_aeb_no_cancel(self):
    # No passthrough logic since we always forward DAS_control,
    # but ensure we can't send cancel cmd while stock AEB is active
    no_aeb_msg = self._long_control_msg(10, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"], aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))


class TestTeslaLongitudinalSafety(TestTeslaSafetyBase):
  SAFETY_PARAM = TeslaSafetyFlags.LONG_CONTROL

  RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor, MSG_DAS_Control)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor, MSG_DAS_Control]}

  def test_no_aeb(self):
    for aeb_event in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, aeb_event=aeb_event)), aeb_event == 0)

  def test_stock_aeb_passthrough(self):
    no_aeb_msg = self._long_control_msg(10, aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))

  def test_prevent_reverse(self):
    # Note: Tesla can reverse while at a standstill if both accel_min and accel_max are negative.
    self.safety.set_controls_allowed(True)

    # accel_min and accel_max are positive
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(1.1, 0.8))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(1.1, 0.8))))

    # accel_min and accel_max are both zero
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(0, 0))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, 0))))

    # accel_min and accel_max have opposing signs
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-0.8, 1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0.8, -1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, -1.3))))

    # accel_min and accel_max are negative
    self.assertFalse(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-1.1, -0.6))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.6, -1.1))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.1, -0.1))))


class TestTeslaCarSignalsSafety(TeslaCarSignalsMixin, TestTeslaStockSafety):
  SAFETY_PARAM = TeslaSafetyFlags.CAR_SIGNALS
  TX_MSGS = TestTeslaStockSafety.TX_MSGS + [[MSG_DAS_bodyControls, 0]]


class TestTeslaCarSignalsLongSafety(TeslaCarSignalsMixin, TestTeslaLongitudinalSafety):
  SAFETY_PARAM = TeslaSafetyFlags.LONG_CONTROL | TeslaSafetyFlags.CAR_SIGNALS
  TX_MSGS = TestTeslaLongitudinalSafety.TX_MSGS + [[MSG_DAS_bodyControls, 0]]


class TestTeslaCarSignalsCheckSafety(TeslaCarSignalsMixin, TestTeslaStockSafety):
  SAFETY_PARAM = TeslaSafetyFlags.CAR_SIGNALS | TeslaSafetyFlags.CAR_SIGNALS_CHECK
  TX_MSGS = TestTeslaStockSafety.TX_MSGS + [[MSG_DAS_bodyControls, 0]]
  CHECK = True


class TestTeslaCarSignalsCheckAloneSafety(TestTeslaStockSafety):
  # the check bit means nothing without CAR_SIGNALS: today's mode exactly
  SAFETY_PARAM = TeslaSafetyFlags.CAR_SIGNALS_CHECK

  def test_no_body_rx_check(self):
    self.assertEqual(9, self.safety.get_rx_checks_len())


class TestTeslaIgnition(common.SafetyTestBase):
  DBC = "tesla_model3_party"
  SAFETY_MODEL = None

  TX_MSGS: list = []

  def _msg(self, counter, state):
    return self.packer.make_can_msg_safety("VCFRONT_LVPowerState", 0,
                                           {"VCFRONT_LVPowerStateCounter": counter,
                                            "VCFRONT_vehiclePowerState": state})

  # VEHICLE_POWER_STATE_DRIVE=3 (counter-gated)
  def test_ignition_on(self):
    for i in range(16):
      self.safety.init_tests()
      self.safety.ignition_can_hook(self._msg(i, 3))
      self.assertFalse(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self._msg((i + 1) % 16, 3))
      self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0, 3))
    self.safety.ignition_can_hook(self._msg(1, 3))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(2, 2))
    self.safety.ignition_can_hook(self._msg(3, 2))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
