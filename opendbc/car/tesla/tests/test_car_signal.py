"""TurnPilot's commanded Tesla turn signal: the port, its mask, the lamp check and the re-check (DESIGN 11.2).

Every port test runs its frames through the real panda safety code (PandaGate over libsafety) into BodyModel, so the
panda's freshness, ownership, CANCEL and engagement rules act as on the car.
"""
import random
import unittest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_signal import (NONE, LEFT, RIGHT, CarSignalsConfig, BlinkerMask, LAMP_ON_S, REQUEST_CAP_S)
from opendbc.car.tesla.body_model import BodyModel, STOCK_IDLE, WrongSideHoldBody
from opendbc.car.tesla.car_signal import TeslaCarSignal, REQ_NONE, REQ_LEFT, REQ_RIGHT, REQ_CANCEL
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.teslacan import (BODY_CONTROLS_ADDR, body_controls_checksum, body_controls_request,
                                        body_controls_valid, create_body_controls)
from opendbc.car.tesla.tests.panda_gate import PandaGate, PortHarness
from opendbc.car.tesla.values import CAR, TeslaFlags, TeslaSafetyFlags

GearShifter = structs.CarState.GearShifter
FREE_BITS = set(range(8, 11)) | set(range(33, 38)) | set(range(52, 56)) | set(range(56, 64))


def make_cp(*flags) -> structs.CarParams:
  CP = structs.CarParams()
  value = int(TeslaFlags.CAR_SIGNALS)
  for f in flags:
    value |= int(f)
  CP.flags = value
  return CP


def harness(*flags, body=None, seed=0, engaged=True, **body_kw) -> PortHarness:
  rng = random.Random(seed)
  port = TeslaCarSignal(make_cp(*flags))
  body = body or BodyModel(rng=rng, glitch_rate=0., **body_kw)
  h = PortHarness(port, body=body, rng=rng)
  h.engaged = engaged
  return h


def requests_sent(h, accepted_only=True):
  return [body_controls_request(d)[0] for _, d, ok in h.sent if ok or not accepted_only]


class TestCreateBodyControls(unittest.TestCase):
  def test_only_turn_fields_change(self):
    # property: no TX alters a non-turn field, and every one passes the panda with its stock frame
    rng = random.Random(2)
    gate = PandaGate()
    gate.safety.set_controls_allowed(True)
    for _ in range(10000):
      stock = bytearray(rng.randbytes(8))
      stock[1] &= 0xF8  # the DAS idles at NONE
      stock[7] = body_controls_checksum(stock)
      request = rng.choice((REQ_LEFT, REQ_RIGHT, REQ_CANCEL))
      reason = {REQ_LEFT: rng.choice((0, 8, 9)), REQ_RIGHT: rng.choice((0, 8, 9)), REQ_CANCEL: 15}[request]
      dat = create_body_controls(bytes(stock), request, reason)
      diff = int.from_bytes(stock, 'little') ^ int.from_bytes(dat, 'little')
      changed = {b for b in range(64) if diff >> b & 1}
      self.assertTrue(changed <= FREE_BITS, (stock.hex(), dat.hex()))
      self.assertEqual(((stock[6] >> 4) + 1) % 16, dat[6] >> 4)
      self.assertTrue(body_controls_valid(dat))
      self.assertEqual((request, reason), body_controls_request(dat))
      gate.reset()
      gate.set_time(1.)
      gate.stock(bytes(stock))
      gate.safety.set_controls_allowed(True)
      if request == REQ_CANCEL:
        self.assertTrue(gate.send(create_body_controls(bytes(stock), REQ_LEFT, 8)))
      self.assertTrue(gate.send(dat), (stock.hex(), dat.hex()))


class TestInterface(unittest.TestCase):
  def _ci(self, mode='on', fingerprint=None):
    fp = fingerprint or gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fp, [], False, False, False, car_signals=CarSignalsConfig(mode=mode))
    return CarInterface(CP)

  def test_get_params_off_unchanged(self):
    fp = gen_empty_fingerprint()
    base = CarInterface.get_params(CAR.TESLA_MODEL_3, fp, [], False, False, False)
    off = CarInterface.get_params(CAR.TESLA_MODEL_3, fp, [], False, False, False, car_signals=CarSignalsConfig())
    self.assertEqual(base.to_bytes(), off.to_bytes())
    self.assertEqual(0, base.safetyConfigs[0].safetyParam)

  def test_get_params_flags(self):
    fp = gen_empty_fingerprint()
    cases = [
      (CarSignalsConfig(mode='check'), TeslaFlags.CAR_SIGNALS | TeslaFlags.CAR_SIGNALS_CHECK,
       TeslaSafetyFlags.CAR_SIGNALS | TeslaSafetyFlags.CAR_SIGNALS_CHECK),
      (CarSignalsConfig(mode='on'), TeslaFlags.CAR_SIGNALS, TeslaSafetyFlags.CAR_SIGNALS),
      (CarSignalsConfig(mode='on', on_reason=9, recheck=True, on_separates=True),
       TeslaFlags.CAR_SIGNALS | TeslaFlags.CAR_SIGNALS_REASON_9 | TeslaFlags.CAR_SIGNALS_RECHECK | TeslaFlags.CAR_SIGNALS_ON_SEPARATES,
       TeslaSafetyFlags.CAR_SIGNALS),
      (CarSignalsConfig(mode='on', on_reason=0), TeslaFlags.CAR_SIGNALS | TeslaFlags.CAR_SIGNALS_REASON_0, TeslaSafetyFlags.CAR_SIGNALS),
      (CarSignalsConfig(mode='bogus'), 0, 0),
    ]
    for config, flags, safety in cases:
      with self.subTest(config=config):
        CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fp, [], False, False, False, car_signals=config)
        self.assertEqual(int(flags), CP.flags & int(flags | TeslaFlags.CAR_SIGNALS | TeslaFlags.CAR_SIGNALS_CHECK))
        self.assertEqual(int(safety), CP.safetyConfigs[0].safetyParam)
    fp[2][BODY_CONTROLS_ADDR] = 8
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fp, [], False, False, False)
    self.assertTrue(CP.flags & TeslaFlags.HAS_BODY_CONTROLS)
    self.assertFalse(CP.flags & TeslaFlags.CAR_SIGNALS)

  def test_stash_and_echoes(self):
    ci = self._ci()
    port = ci.CS.car_signal
    stock1 = create_body_controls(STOCK_IDLE, REQ_NONE, 0)
    stock2 = create_body_controls(stock1, REQ_NONE, 0)
    bad = bytearray(stock2)
    bad[7] ^= 0xFF
    ci.update([(int(1e9), [(BODY_CONTROLS_ADDR, stock1, 2), (BODY_CONTROLS_ADDR, stock2, 2), (BODY_CONTROLS_ADDR, bytes(bad), 2),
                           (BODY_CONTROLS_ADDR, stock1[:7], 2)])])
    self.assertEqual(stock2, port.stock)
    self.assertTrue(port.stock_new)
    ours = create_body_controls(stock2, REQ_LEFT, 8)
    port.sent_recent = [ours]
    port.phase = 'requesting'
    ci.update([(int(1.1e9), [(BODY_CONTROLS_ADDR, stock2, 128)])])
    self.assertIsNone(port.first_echo_t)
    ci.update([(int(1.2e9), [(BODY_CONTROLS_ADDR, ours, 128)])])
    self.assertAlmostEqual(1.2, port.first_echo_t)
    ci.update([(int(1.3e9), [(BODY_CONTROLS_ADDR, ours, 192), (BODY_CONTROLS_ADDR, stock2, 192)])])
    self.assertEqual(1, port.rejected)

  def test_no_stock_frames_keep_can_valid(self):
    # no body message for 30 s never makes canValid false: it is read raw, not through a parser
    for mode in ('off', 'on'):
      ci = self._ci(mode)
      for i in range(3000):
        cs = ci.update([(int(i * 1e7), [])])
      self.assertEqual(ci.update([(int(30.01e9), [])]).canValid, cs.canValid)
      self.assertEqual(mode == 'on', ci.CS.car_signal.active)

  def test_carstate_mask_and_publish(self):
    ci = self._ci()
    packer = CANPacker('tesla_model3_party')
    port = ci.CS.car_signal
    ci.update([(int(0.5e9), [])])  # the parsers register their messages on the first read
    port.stock_seen, port.stock_t, port.now = True, 0.95, 1.
    # a lamp lit when the port starts is inherited: masked until it reads off for STUCK_OFF_S (a touch is 2 to 6 frames)
    frames = [(2, 0), (2, 0)] + [(0, 0)] * 6 + [(2, 0)] + [(0, 0)] * 11 + [(2, 0)]
    for i, (left, right) in enumerate(frames):
      t = 1 + i * 0.1
      ui = packer.make_can_msg('UI_warning', 0, {'leftBlinkerBlinking': left, 'rightBlinkerBlinking': right, 'UI_warningCounter': i % 16})[1]
      port.stock_t = t - 0.05
      cs = ci.update([(int(t * 1e9), [(0x311, ui, 0)])])
      self.assertEqual(i == len(frames) - 1, cs.leftBlinker, i)
      self.assertEqual('inherited' if left and i < 9 else 'none', str(cs.carSignal.block), i)
    self.assertEqual('ready', str(cs.carSignal.status))
    self.assertTrue(cs.carSignal.leftLamp)

  def test_mode_off_is_today(self):
    # outside any mask, over random UI_warning sequences with the port idle, the blinkers are the lamps
    rng = random.Random(3)
    for mode in ('off', 'on'):
      ci = self._ci(mode)
      ci.update([(int(0.5e9), [])])
      packer = CANPacker('tesla_model3_party')
      for i in range(500):
        left, right = rng.choice((0, 1, 2, 3)), rng.choice((0, 1, 2, 3))
        ui = packer.make_can_msg('UI_warning', 0, {'leftBlinkerBlinking': left, 'rightBlinkerBlinking': right, 'UI_warningCounter': i % 16})[1]
        cs = ci.update([(int((1 + i * 0.1) * 1e9), [(0x311, ui, 0)])])
        mask = ci.CS.car_signal.mask
        if mode == 'off' or not (mask.any_masked or mask.hazard):
          self.assertEqual((left in (1, 2), right in (1, 2)), (cs.leftBlinker, cs.rightBlinker))

  def test_empty_can_sends_nothing(self):
    # test_models' empty-CAN rule: with no stock frame the controller sends no 0x3E9, whatever CarControl asks
    for mode in ('on', 'check'):
      ci = self._ci(mode)
      CC = structs.CarControl()
      CC.enabled = True
      for i in range(1000):
        CC.leftBlinker = i % 200 < 100
        ci.update([(int(i * 1e7), [])])
        _, sends = ci.apply(CC.as_reader(), int(i * 1e7))
        self.assertFalse(any(addr == BODY_CONTROLS_ADDR for addr, _, _ in sends))


class TestMask(unittest.TestCase):
  def test_rules(self):
    m = BlinkerMask()
    m.frame(False, False, 0.)
    m.claim(LEFT)
    self.assertEqual('ours', m.mask[LEFT])
    self.assertEqual('pending', m.mask[RIGHT])
    m.frame(True, False, 0.1)
    self.assertEqual((False, False), m.blinkers())
    m.driver_unmask(LEFT)
    self.assertIsNone(m.mask[RIGHT])
    m.frame(True, True, 0.2)
    self.assertEqual((True, True), m.blinkers())  # hazards under a mask: both, never one
    m.frame(False, True, 0.3)
    self.assertEqual((True, True), m.blinkers())
    m.frame(False, False, 0.4)
    m.frame(False, False, 0.5)
    self.assertFalse(m.hazard)
    self.assertEqual('ours', m.mask[LEFT])  # not released yet
    m.released(LEFT, 1.0)
    m.frame(False, False, 0.6)
    self.assertEqual('ours', m.mask[LEFT])  # not before lift_after
    m.frame(True, False, 0.9)
    m.frame(False, False, 1.0)
    self.assertEqual('ours', m.mask[LEFT])  # one 0 frame unmasks nothing
    m.frame(False, False, 1.1)
    self.assertIsNone(m.mask[LEFT])

  def test_wrong_side_lamp_held_till_off(self):
    # a pending side that never lit lifts with ours
    m = BlinkerMask()
    m.frame(False, False, 0.)
    m.claim(LEFT)
    m.released(LEFT, 0.)
    m.frame(False, False, 0.1)
    m.frame(False, False, 0.2)
    self.assertEqual((None, None), (m.mask[LEFT], m.mask[RIGHT]))
    # rule 3: a lamp lit on the pending side is ours on the wrong side. Ours lifting (never lit) doesn't unmask it: it
    # stays masked until it reads off STUCK_OFF_S, never the driver's blinker while it may still be ours
    m.claim(LEFT)
    m.frame(False, True, 0.3)
    m.released(LEFT, 0.5)
    t = 0.4
    for _ in range(10):
      m.frame(False, True, t)
      t += 0.1
    self.assertIsNone(m.mask[LEFT])
    self.assertEqual('wrong', m.mask[RIGHT])
    self.assertEqual((False, False), m.blinkers())
    for lit in [False] * 6 + [True] * 3:  # a stalk touch's dark run, then lit again
      m.frame(False, lit, t)
      t += 0.1
      self.assertEqual((False, False), m.blinkers())
    for _ in range(11):
      m.frame(False, False, t)
      t += 0.1
    self.assertIsNone(m.mask[RIGHT])
    m.frame(False, True, t)
    self.assertEqual((False, True), m.blinkers())  # lit after STUCK_OFF_S dark: the driver's

  def test_dark_phase_counts_as_lit(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    dark_frames = 0
    for _ in range(1000):
      h.step()
      dark_frames += h.body.blink[LEFT] == 1
    self.assertGreater(dark_frames, 200)
    self.assertEqual('lit', h.port.status)
    self.assertEqual('none', h.port.fault)
    self.assertFalse(any(left for _, left, _ in h.blinkers))


class TestPort(unittest.TestCase):
  def test_clean_request(self):
    h = harness()
    h.run(1.)
    self.assertEqual('ready', h.port.status)
    h.want = LEFT
    h.run(4.)
    self.assertEqual('lit', h.port.status)
    self.assertTrue(h.body.lit(LEFT) or h.body.cause[LEFT] == {'request'})
    h.want = NONE
    h.run(4.)
    self.assertEqual('ready', h.port.status)
    reqs = requests_sent(h)
    self.assertEqual([REQ_CANCEL, REQ_NONE], reqs[-2:])
    self.assertTrue(all(r == REQ_LEFT for r in reqs[:-2]))
    self.assertEqual(0, h.gate.refused)
    self.assertFalse(any(left or right for _, left, right in h.blinkers))
    self.assertFalse(h.body.cause[LEFT])

  def test_one_frame_per_stock_frame(self):
    h = harness()
    h.run(1.)
    h.want = RIGHT
    h.run(10.)
    stock_times = [t for t, d in h.body.frames_received if body_controls_request(d)[0] == REQ_NONE]
    ours = [t for t, _, _ in h.sent]
    self.assertGreaterEqual(len(ours), 18)
    for a, b in zip(ours, ours[1:], strict=False):
      self.assertGreater(b - a, 0.3)
    self.assertLessEqual(len([t for t in stock_times if t > ours[0]]), 0)

  def test_no_claim_without_stock(self):
    h = harness()
    h.body.next_stock_t = 1e9
    h.want = LEFT
    h.run(5.)
    self.assertEqual('waiting', h.port.status)
    self.assertEqual([], h.sent)

  def test_lit_start_releases_none(self):
    h = harness()
    h.run(1.)
    h.body.driver_stalk(LEFT, True)
    h.run(1.)
    self.assertTrue(h.blinkers[-1][1])  # the driver's lamp, unmasked
    h.want = LEFT
    h.run(3.)
    self.assertFalse(h.blinkers[-1][1])  # absorbed (rule 6)
    h.want = NONE
    h.run(3.)
    self.assertNotIn(REQ_CANCEL, requests_sent(h))
    self.assertEqual(REQ_NONE, requests_sent(h)[-1])
    self.assertTrue(h.body.stalk[LEFT])  # the driver's signal survives

  def test_driver_other_side_releases_none(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    self.assertIsNone(h.port.mask.mask[RIGHT])  # unmasked after DRIVER_AFTER_S
    h.body.driver_stalk(RIGHT, True)
    h.run(0.3)
    self.assertTrue(h.blinkers[-1][2])
    h.run(2.)
    self.assertNotIn(REQ_CANCEL, requests_sent(h))
    self.assertEqual('otherSide', h.port.block)
    self.assertTrue(h.body.stalk[RIGHT])
    self.assertEqual('none', h.port.fault)

  def test_hazards_release_none_and_publish_both(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    h.body.hazards(True)
    h.run(2.)
    self.assertNotIn(REQ_CANCEL, requests_sent(h))
    self.assertTrue(h.port.mask.hazard)
    self.assertEqual('hazard', h.port.block)
    for _, left, right in h.blinkers[-150:]:
      self.assertEqual(left, right)
    h.body.hazards(False)
    h.want = NONE
    h.run(3.)
    self.assertFalse(h.port.mask.hazard)

  def test_cancel_retry_once_while_lit(self):
    # a body that ignores the first CANCEL: one retry, only while the lamp stayed lit since it
    h = harness(semantics='latched')
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    orig = h.body.receive
    ignored = [0]

    def receive(dat, t):
      if body_controls_request(dat)[0] == REQ_CANCEL and ignored[0] == 0:
        ignored[0] += 1
        return
      orig(dat, t)
    h.body.receive = receive
    h.want = NONE
    h.run(5.)
    self.assertEqual(2, requests_sent(h).count(REQ_CANCEL))
    self.assertEqual('ready', h.port.status)

  def test_no_retry_on_handover(self):
    h = harness(semantics='latched')
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    h.body.receive = (lambda orig: (lambda dat, t: None if body_controls_request(dat)[0] == REQ_CANCEL else orig(dat, t)))(h.body.receive)
    h.engaged = False
    h.want = NONE
    h.run(6.)
    self.assertEqual(1, requests_sent(h).count(REQ_CANCEL))
    self.assertEqual('lampStuck', h.port.fault)
    self.assertTrue(h.port.still_on)

  def test_relit_lamp_never_cancelled(self):
    # after a CANCEL the lamp reads 0 twice then lights again: a new lamp, the driver's, never CANCELed
    h = harness()
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    h.want = NONE
    for _ in range(100):
      h.step()
      if h.port.cancels:
        break
    h.run(0.3)
    h.body.driver_stalk(LEFT, True)
    h.run(4.)
    self.assertEqual(1, requests_sent(h).count(REQ_CANCEL))
    self.assertTrue(h.body.stalk[LEFT])

  def test_on_separates_withholds_cancel(self):
    h = harness(TeslaFlags.CAR_SIGNALS_ON_SEPARATES)
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    h.body.driver_stalk(LEFT, True)  # the driver's press shows in the On bit
    h.run(0.3)
    h.want = NONE
    h.run(3.)
    self.assertNotIn(REQ_CANCEL, requests_sent(h))
    self.assertTrue(h.body.stalk[LEFT])

  def test_lamp_timeout_from_first_echo(self):
    h = harness(on_latency=10.)
    h.run(1.)
    h.want = LEFT
    while h.port.first_echo_t is None:
      h.step()
    echo = h.port.first_echo_t
    while h.port.fault == 'none':
      h.step()
    self.assertEqual('lampTimeout', h.port.fault)
    self.assertAlmostEqual(echo + LAMP_ON_S, h.t, delta=0.03)
    h.run(4.)
    self.assertEqual('failed', h.port.status)
    self.assertEqual([REQ_CANCEL, REQ_NONE], requests_sent(h)[-2:])

  def test_lamp_timeout_when_refused(self):
    h = harness(engaged=False)
    h.run(1.)
    h.port.enabled = True
    h.want = LEFT
    h.port.control = (lambda orig: (lambda want, enabled, n: orig(want, True, n)))(h.port.control)
    h.run(3.)
    self.assertEqual('lampTimeout', h.port.fault)
    self.assertEqual('the panda refused it', h.port.fault_why)
    self.assertGreater(h.port.rejected, 0)

  def test_lamp_lost(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    h.body.driver_touch(LEFT, 6)
    h.run(1.)
    self.assertEqual('lampLost', h.port.fault)

  def test_single_zero_frame_is_not_lost(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    for _ in range(5):
      h.body.driver_touch(LEFT, 1)
      h.run(1.)
    self.assertEqual('none', h.port.fault)

  def test_no_stock_frame(self):
    h = harness()
    h.run(1.)
    h.body.next_stock_t = 1e9
    h.run(1.2)
    self.assertEqual('noStockFrame', h.port.fault)
    self.assertEqual('failed', h.port.status)

  def test_capped(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    h.run(REQUEST_CAP_S + 1.)
    self.assertEqual('capped', h.port.fault)
    h.run(4.)
    self.assertEqual('failed', h.port.status)
    self.assertFalse(h.body.cause[LEFT])
    self.assertEqual([REQ_CANCEL, REQ_NONE], requests_sent(h)[-2:])

  def test_wrong_side_body(self):
    h = harness(semantics='wrongSide')
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    self.assertEqual('wrongSide', h.port.fault)
    self.assertFalse(any(left or right for _, left, right in h.blinkers))
    # NONE alone: a lamp on the other side may be the driver's, never erased by a CANCEL; the NONE puts ours out
    self.assertNotIn(REQ_CANCEL, requests_sent(h))
    self.assertEqual(REQ_NONE, requests_sent(h)[-1])
    self.assertFalse(h.body.cause[RIGHT])

  def test_wrong_side_lamp_outlasting_the_lift_never_reads_as_the_drivers(self):
    # an intermittent wrong side, put out late: NONE ignored (the request dropped 2.5 s after its last frame), obeyed
    # only after 1.2 s (inside the lamp check's own NONE_OFF_S), or after 5 s (past RELEASE_TAIL_S: still on, out loud)
    for none_delay in (None, 1.2, 5.):
      for seed in range(4):
        with self.subTest(none_delay=none_delay, seed=seed):
          body = WrongSideHoldBody(none_delay, rng=random.Random(seed), on_latency=0.2, glitch_rate=0., early_rate=0.)
          h = harness(body=body, seed=seed)
          h.run(1.)
          h.want = LEFT
          for _ in range(250):
            h.step()
            if h.port.fault != 'none':
              break
          h.want = NONE  # the intent drops it with the fault
          still = False
          for _ in range(800):
            h.step()
            still = still or h.port.still_on
          self.assertEqual('wrongSide', h.port.fault)
          self.assertEqual([], [(round(t, 2), left, right) for t, left, right in h.blinkers if left or right][:3])
          self.assertEqual(none_delay == 5., still)
          self.assertFalse(h.port.still_on)
          self.assertFalse(h.port.mask.any_masked)  # lifted once the lamp read off STUCK_OFF_S
          self.assertNotIn(REQ_CANCEL, requests_sent(h))

  def test_driver_other_side_in_first_second_survives(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    while h.port.phase != 'lit':
      h.step()
    h.body.driver_stalk(RIGHT, True)
    h.run(3.)
    self.assertNotIn(REQ_CANCEL, requests_sent(h))
    self.assertTrue(h.body.stalk[RIGHT])

  def test_hazards_before_the_cancel_turn_it_into_none(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    h.want = NONE
    h.step()  # the release is decided; the CANCEL waits for the next stock frame
    h.body.hazards(True)
    h.run(3.)
    self.assertNotIn(REQ_CANCEL, requests_sent(h))

  def test_hazard_answer_body(self):
    h = harness(semantics='hazardAnswer')
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    self.assertEqual('wrongSide', h.port.fault)
    self.assertNotIn(REQ_CANCEL, requests_sent(h))
    for _, left, right in h.blinkers:
      self.assertEqual(left, right)

  def test_driver_other_side_in_first_second_is_wrong_side(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    while h.port.phase != 'lit':
      h.step()
    h.body.driver_stalk(RIGHT, True)
    h.run(0.5)
    self.assertEqual('wrongSide', h.port.fault)

  def test_no_claim_while_blocked(self):
    for setup in ('other', 'hazard', 'inherited'):
      with self.subTest(setup=setup):
        h = harness()
        if setup == 'inherited':
          h.body.driver_stalk(RIGHT, True)
        h.run(1.)
        if setup == 'other':
          h.body.driver_stalk(RIGHT, True)
        elif setup == 'hazard':
          h.body.hazards(True)
        h.run(1.)
        h.want = LEFT
        h.run(3.)
        self.assertEqual([], h.sent)
        self.assertEqual({'other': 'otherSide', 'hazard': 'hazard', 'inherited': 'inherited'}[setup], h.port.block)

  def test_no_side_switch(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    h.want = RIGHT
    h.run(0.2)
    self.assertEqual(LEFT, h.port.side)
    h.run(4.)
    reqs = requests_sent(h)
    i = reqs.index(REQ_RIGHT)
    self.assertEqual([REQ_CANCEL, REQ_NONE], reqs[i - 2:i])

  def test_fault_latches(self):
    h = harness(on_latency=10.)
    h.run(1.)
    h.want = LEFT
    h.run(5.)
    h.body.on_latency = 0.1
    h.want = NONE
    h.run(2.)
    h.want = LEFT
    h.run(5.)
    self.assertEqual('failed', h.port.status)
    self.assertEqual(0, requests_sent(h)[-1])

  def test_disengage_drops_request(self):
    h = harness()
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    h.engaged = False
    h.run(3.)
    self.assertEqual([REQ_CANCEL, REQ_NONE], requests_sent(h)[-2:])
    self.assertFalse(h.body.cause[LEFT])

  def test_card_dies_mid_request(self):
    # the panda's ownership times out and the stock NONE puts a level body's lamp out; a restarted port inherits
    h = harness()
    h.run(1.)
    h.want = LEFT
    h.run(3.)
    h.card_alive = False
    h.run(1.5)
    self.assertFalse(h.body.cause[LEFT])
    h2 = PortHarness(TeslaCarSignal(make_cp()), body=h.body, gate=h.gate, rng=random.Random(4), t0=h.t)
    h2.engaged = True
    h2.run(2.)
    self.assertEqual('ready', h2.port.status)

  def test_random_timing_frames_accepted(self):
    # under random timing, every CANCEL and NONE the port sends is accepted by the panda
    for seed in range(12):
      rng = random.Random(seed)
      h = harness(seed=seed, on_latency=rng.uniform(0.02, 0.45))
      h.body.early_rate = 0.05
      h.run(rng.uniform(0.5, 1.5))
      for _ in range(12):
        h.cycle_jitter = rng.uniform(0., 0.12)
        h.want = rng.choice((NONE, LEFT, RIGHT))
        h.engaged = rng.random() < 0.9
        h.run(rng.uniform(0.2, 6.))
      h.want = NONE
      h.run(6.)
      for _, dat, ok in h.sent:
        if body_controls_request(dat)[0] in (REQ_CANCEL, REQ_NONE):
          self.assertTrue(ok, (seed, dat.hex()))


class TestLampCheck(unittest.TestCase):
  def _check(self, seed=0, gear=GearShifter.park, **body_kw):
    h = harness(TeslaFlags.CAR_SIGNALS_CHECK, seed=seed, engaged=False, **body_kw)
    h.port.check.rng = random.Random(seed)
    h.standstill = True
    h.gear = gear
    h.brake = gear == GearShifter.drive
    return h

  def _run_check(self, h, limit=200.):
    t0 = h.t
    while h.port.check.stage not in ('passed', 'failed') and h.t - t0 < limit:
      h.run(0.5)
    return h.port.check

  def test_level_body_passes(self):
    h = self._check()
    check = self._run_check(h)
    self.assertEqual('passed', check.stage, check.why)
    record = check.result['record']
    self.assertEqual(8, record['onReason'])
    self.assertEqual('P', record['gear'])
    self.assertFalse(record['onSeparates'])
    self.assertLess(record['onLatencyS'], 0.5)
    self.assertEqual(0, h.gate.refused)
    self.assertFalse(any(left or right for _, left, right in h.blinkers))

  def test_reason_9_body(self):
    h = self._check(reasons=(9,))
    check = self._run_check(h)
    self.assertEqual('passed', check.stage, check.why)
    self.assertEqual(9, check.result['record']['onReason'])

  def test_tap_body_fails(self):
    h = self._check(semantics='tap')
    check = self._run_check(h)
    self.assertEqual('failed', check.stage)
    self.assertIn('lamp stopped', check.why)

  def test_latched_body_fails_release(self):
    h = self._check(semantics='latched')
    check = self._run_check(h)
    self.assertEqual('failed', check.stage)
    self.assertEqual('release', check.result['step'])
    self.assertIn('not safe to command', check.why)
    h.run(8.)
    self.assertFalse(h.body.cause[LEFT])  # the clean-up's fresh LEFT then CANCEL put it out
    reqs = [body_controls_request(d)[0] for _, d, _ in h.gate.sent]
    self.assertEqual([REQ_LEFT, REQ_CANCEL], reqs[-3:-1] if reqs[-1] == REQ_NONE else reqs[-2:])
    self.assertTrue(all(ok for _, d, ok in h.gate.sent if body_controls_request(d)[0] == REQ_CANCEL))

  def test_none_ignored_body_fails(self):
    h = self._check(semantics='noneIgnored')
    check = self._run_check(h)
    self.assertEqual('failed', check.stage)
    self.assertEqual('release', check.result['step'])

  def test_faulty_bodies_fail(self):
    for semantics in ('wrongSide', 'hazardAnswer'):
      with self.subTest(semantics=semantics):
        h = self._check(semantics=semantics)
        check = self._run_check(h)
        self.assertEqual('failed', check.stage)

  def test_driver_press_in_pattern_stops_it(self):
    for seed in range(6):
      with self.subTest(seed=seed):
        h = self._check(seed=seed)
        while h.port.check.stage != 'pattern':
          h.run(0.5)
        h.run(random.Random(seed).uniform(0., 12.))
        side = random.Random(seed + 100).choice((LEFT, RIGHT))
        stops = h.port.check.stops
        h.body.driver_stalk(side, True)
        h.run(5.)
        self.assertTrue(h.port.check.stops > stops or h.port.check.stage == 'failed', h.port.check.stage)

  def test_on_bit_press_stops_it(self):
    h = self._check()
    while h.port.check.stage != 'release':
      h.run(0.2)
    h.run(1.)
    h.body.driver_stalk(LEFT, True)
    h.run(1.)
    self.assertIn(h.port.check.stage, ('aborted', 'waiting', 'failed'))

  def test_never_starts_in_drive_without_start(self):
    h = self._check(gear=GearShifter.drive)
    h.run(20.)
    self.assertEqual('waiting', h.port.check.stage)
    self.assertEqual([], h.sent)
    h.port.check_now = True
    h.run(5.)
    self.assertEqual('reason', h.port.check.stage)
    h.standstill = False  # the car moves: stopped, never re-armed in D by itself
    h.run(1.)
    h.standstill = True
    h.run(10.)
    self.assertEqual('aborted', h.port.check.stage)
    n = len(h.sent)
    h.run(10.)
    self.assertEqual(n, len(h.sent))
    h.port.check_now = True
    h.run(5.)
    self.assertNotEqual('aborted', h.port.check.stage)

  def test_drive_check_passes(self):
    h = self._check(gear=GearShifter.drive)
    h.port.check_now = True
    check = self._run_check(h)
    self.assertEqual('passed', check.stage, check.why)
    self.assertEqual('D', check.result['record']['gear'])

  def test_aborts(self):
    for what in ('move', 'gear', 'brake', 'lamp', 'engage'):
      with self.subTest(what=what):
        h = self._check(gear=GearShifter.drive if what == 'brake' else GearShifter.park)
        h.port.check_now = what == 'brake'
        while h.port.check.stage != 'reason':
          h.run(0.2)
        h.run(2.)
        if what == 'move':
          h.standstill = False
        elif what == 'gear':
          h.gear = GearShifter.neutral
        elif what == 'brake':
          h.brake = False
        elif what == 'lamp':
          h.body.driver_stalk(RIGHT, True)
        elif what == 'engage':
          h.engaged = True
          h.port.enabled = True
        h.run(1.)
        self.assertIn(h.port.check.stage, ('aborted', 'failed', 'waiting'))

  def test_parked_rearms(self):
    h = self._check()
    while h.port.check.stage != 'reason':
      h.run(0.2)
    h.run(2.)
    h.gear = GearShifter.drive
    h.run(8.)
    self.assertIn(h.port.check.stage, ('aborted', 'waiting'))
    h.gear = GearShifter.park
    h.run(6.)
    self.assertEqual('reason', h.port.check.stage)

  def test_on_separates(self):
    h = self._check(on_bits='driver')
    t0 = h.t
    while h.port.check.stage != 'driverPress' and h.t - t0 < 200:
      h.run(0.5)
    h.body.driver_stalk(LEFT, True)
    h.run(3.)
    h.body.driver_stalk(LEFT, False)
    check = self._run_check(h)
    self.assertEqual('passed', check.stage)
    self.assertTrue(check.result['record']['onSeparates'])
    h2 = self._check(on_bits='any')
    check = self._run_check(h2)
    self.assertFalse(check.result['record']['onSeparates'])

  def test_only_check_requests(self):
    # in check mode the controller ignores CarControl's blinkers
    h = self._check(gear=GearShifter.drive)
    h.engaged = True
    h.want = LEFT
    h.run(20.)
    self.assertEqual([], h.sent)


class TestRecheck(unittest.TestCase):
  def _h(self, **kw):
    h = harness(TeslaFlags.CAR_SIGNALS_RECHECK, engaged=False, **kw)
    h.standstill, h.gear = True, GearShifter.park
    return h

  def test_passes_in_park(self):
    h = self._h()
    self.assertEqual('verifying', h.port.status)
    h.run(15.)
    self.assertEqual('ready', h.port.status)
    reqs = requests_sent(h)
    self.assertIn(REQ_LEFT, reqs)
    self.assertIn(REQ_RIGHT, reqs)
    self.assertFalse(any(left or right for _, left, right in h.blinkers))

  def test_wrong_side_fails(self):
    h = self._h(semantics='wrongSide')
    h.run(15.)
    self.assertEqual('failed', h.port.status)
    self.assertEqual('wrongSide', h.port.fault)

  def test_waits_out_of_park(self):
    h = self._h()
    h.gear = GearShifter.drive
    h.standstill = False
    h.run(10.)
    self.assertEqual('verifying', h.port.status)
    self.assertEqual([], h.sent)
    h.gear, h.standstill = GearShifter.park, True
    h.run(15.)
    self.assertEqual('ready', h.port.status)

  def test_no_recheck_without_flag(self):
    h = harness(engaged=False)
    h.standstill, h.gear = True, GearShifter.park
    h.run(15.)
    self.assertEqual('ready', h.port.status)
    self.assertEqual([], h.sent)


if __name__ == "__main__":
  unittest.main()
