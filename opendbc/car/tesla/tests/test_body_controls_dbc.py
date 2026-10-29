"""DAS_bodyControls (0x3E9) in the party DBC, with the firmware layout (DESIGN phase4/blinker 2.4, 11.2)."""
import random
import unittest

from opendbc.can import CANPacker, CANParser

BODY_CONTROLS_ADDR = 0x3E9
STOCK_IDLE = bytes.fromhex("00880220000010a6")  # the measured idle frame (2025 HW4 Model 3)


def checksum(dat) -> int:
  return (0xE9 + 0x03 + sum(dat[:7])) & 0xFF


class TestDBC(unittest.TestCase):
  def test_idle_frame(self):
    for dbc in ('tesla_model3_party', 'tesla_model3_vehicle'):
      cp = CANParser(dbc, [('DAS_bodyControls', float('nan'))], 2)
      cp.update([(1000, [(BODY_CONTROLS_ADDR, STOCK_IDLE, 2)])])
      vl = cp.vl['DAS_bodyControls']
      self.assertTrue(cp.can_valid)
      self.assertEqual((0, 1, 5, 1, 0, 1), (vl['DAS_turnIndicatorRequest'], vl['DAS_highLowBeamDecision'],
                                            vl['DAS_highLowBeamOffReason'], vl['DAS_overrideWiperSetting'],
                                            vl['DAS_turnIndicatorRequestReason'], vl['DAS_bodyControlsCounter']))

  def test_party_and_vehicle_agree(self):
    rng = random.Random(1)
    parsers = [CANParser(dbc, [('DAS_bodyControls', float('nan'))], 2) for dbc in ('tesla_model3_party', 'tesla_model3_vehicle')]
    for i in range(200):
      dat = bytearray(rng.randbytes(8))
      dat[6] = (dat[6] & 0x0F) | ((i % 16) << 4)
      dat[7] = checksum(dat)
      for cp in parsers:
        cp.update([(1000 + i, [(BODY_CONTROLS_ADDR, bytes(dat), 2)])])
      self.assertEqual(parsers[0].vl['DAS_bodyControls'], parsers[1].vl['DAS_bodyControls'])

  def test_request_and_reason_fields(self):
    packer = CANPacker('tesla_model3_party')
    for request in range(5):
      for reason in (0, 8, 9, 15, 24):
        dat = packer.make_can_msg('DAS_bodyControls', 0, {'DAS_turnIndicatorRequest': request,
                                                          'DAS_turnIndicatorRequestReason': reason})[1]
        self.assertEqual((request, reason), (dat[1] & 0x07, (dat[4] >> 1) & 0x1F))
        self.assertEqual(checksum(dat), dat[7])


if __name__ == "__main__":
  unittest.main()
