from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams


class TeslaCAN:
  def __init__(self, CP, packer):
    self.CP = CP
    self.packer = packer

  def create_steering_control(self, angle, enabled):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,  # ANGLE_CONTROL
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active):
    set_speed = min(max(v_ego + accel, 0) * CV.MS_TO_KPH, 400)

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self):
    values = {
      "APS_eacAllow": 1,
    }

    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)


BODY_CONTROLS_ADDR = 0x3E9


def body_controls_checksum(dat) -> int:
  """The Tesla sum over DAS_bodyControls' address and data bytes 0-6 (the checksum is byte 7)."""
  return ((BODY_CONTROLS_ADDR & 0xFF) + (BODY_CONTROLS_ADDR >> 8) + sum(dat[:7])) & 0xFF


def body_controls_valid(dat) -> bool:
  return len(dat) == 8 and dat[7] == body_controls_checksum(dat)


def body_controls_request(dat) -> tuple[int, int]:
  """DAS_turnIndicatorRequest (8|3) and DAS_turnIndicatorRequestReason (33|5) of a raw frame."""
  return dat[1] & 0x07, (dat[4] >> 1) & 0x1F


def create_body_controls(stock: bytes, request: int, reason: int) -> bytes:
  """openpilot's turn signal: the stock DAS_bodyControls with only the request (8|3), its reason (33|5), the counter
  (stock + 1) and the checksum rewritten, so headlights, wipers, high beams and every bit the DBC doesn't name ride
  through as the DAS sent them. The panda refuses any other difference (safety/modes/tesla.h)."""
  assert len(stock) == 8 and 0 <= request <= 3 and 0 <= reason <= 31
  dat = bytearray(stock)
  dat[1] = (dat[1] & 0xF8) | request
  dat[4] = (dat[4] & 0xC1) | (reason << 1)
  dat[6] = (dat[6] & 0x0F) | ((((stock[6] >> 4) + 1) & 0x0F) << 4)
  dat[7] = body_controls_checksum(dat)
  return bytes(dat)


def tesla_checksum(address: int, sig, d: bytearray) -> int:
  checksum = (address & 0xFF) + ((address >> 8) & 0xFF)
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum += d[i]
  return checksum & 0xFF
