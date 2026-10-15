"""Contract tests for the bundled probe simulator.

These run the real DeviceClient against the real simulator over loopback, so a
drift between the simulated API and what the UI expects fails here rather than
in the browser.
"""
from pathlib import Path
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import simulator
from device_client import DeviceClient, DeviceError

PORT = 8933
PASSWORD = 'simtest'
HOST = f'127.0.0.1:{PORT}'


class SimulatorContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = simulator.Simulator(PASSWORD, fault_interval=0.0, host_label=HOST)
        cls.sim.start_background()
        cls.server = simulator.ThreadingHTTPServer(
            ('127.0.0.1', PORT), simulator.make_handler(cls.sim))
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()
        time.sleep(0.4)

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()

    def setUp(self):
        self.client = DeviceClient()
        self.client.login(HOST, PASSWORD)
        self.sim.gate_open = False
        self.sim.fault = 'none'
        self.sim.online = True
        self.sim.battery_mv = 7400          # deterministic healthy pack

    def state(self):
        return self.client.robot(HOST)

    # -- identity and session ------------------------------------------------
    def test_identity_marks_simulator(self):
        identity = DeviceClient().identity(HOST)
        self.assertEqual(identity['firmware'], 'rlyoulink')
        self.assertTrue(identity['simulator'])

    def test_wrong_password_rejected(self):
        with self.assertRaises(DeviceError) as caught:
            DeviceClient().login(HOST, 'nope')
        self.assertIn('密码错误', str(caught.exception))

    def test_telemetry_is_normalised(self):
        state = self.state()
        self.assertTrue(state['online'])
        self.assertEqual(state['battery']['condition'], 'normal')
        self.assertEqual(state['camera_stream_url'], f'http://127.0.0.1:{PORT + 1}/stream')
        self.assertEqual(state['camera_capture_url'], f'http://{HOST}/capture')

    # -- the motion gate mirrors the shipped firmware ------------------------
    def test_gate_closed_rejects_without_moving(self):
        before = self.state()['actuators']
        reply = self.client.command(HOST, {'type': 1, 'action': 1, 'mode': 3,
                                           'power': 60, 'strength': 40})
        self.assertEqual(reply['state'], 'queued')
        time.sleep(simulator.ACK_DELAY_MS / 1000 + 0.35)
        state = self.state()
        self.assertEqual(state['command']['result'], 2)          # rejected
        self.assertFalse(state['wifi_control_allowed'])
        self.assertEqual(state['actuators']['left_esc'], before['left_esc'])
        self.assertEqual(state['actuators']['tail1'], before['tail1'])

    def test_gate_open_accepts_and_drives_model(self):
        self.client.simulator(HOST, 'gate', True)
        self.client.command(HOST, {'type': 1, 'action': 1, 'mode': 3,
                                   'power': 60, 'strength': 40})
        time.sleep(simulator.ACK_DELAY_MS / 1000 + 0.35)
        state = self.state()
        self.assertTrue(state['wifi_control_allowed'])
        self.assertEqual(state['command']['result'], 1)          # accepted
        self.assertEqual(state['motion']['action_label'], '前进')
        self.assertEqual(state['actuators']['left_esc'], 60)
        self.assertAlmostEqual(state['actuators']['tail1'], 108.0)

    def test_stop_returns_to_neutral(self):
        self.client.simulator(HOST, 'gate', True)
        self.client.command(HOST, {'type': 1, 'action': 1, 'mode': 3,
                                   'power': 60, 'strength': 40})
        time.sleep(simulator.ACK_DELAY_MS / 1000 + 0.35)
        self.client.command(HOST, {'type': 1, 'action': 0, 'mode': 3,
                                   'power': 0, 'strength': 0})
        time.sleep(simulator.ACK_DELAY_MS / 1000 + 0.35)
        state = self.state()
        self.assertEqual(state['motion']['action_label'], '停止')
        self.assertEqual(state['actuators']['left_esc'], 0)
        self.assertEqual(state['actuators']['right_esc'], 0)
        self.assertAlmostEqual(state['actuators']['tail1'], 90.0)

    def test_turn_moves_escs_differentially(self):
        self.client.simulator(HOST, 'gate', True)
        self.client.command(HOST, {'type': 1, 'action': 4, 'mode': 3,
                                   'power': 50, 'strength': 40})
        time.sleep(simulator.ACK_DELAY_MS / 1000 + 0.35)
        state = self.state()
        self.assertEqual(state['motion']['action_label'], '右转')
        self.assertNotEqual(state['actuators']['left_esc'], state['actuators']['right_esc'])

    def test_pitch_command_applies(self):
        self.client.simulator(HOST, 'gate', True)
        self.client.command(HOST, {'type': 3, 'pitch': 120})
        time.sleep(simulator.ACK_DELAY_MS / 1000 + 0.35)
        self.assertEqual(self.state()['actuators']['pitch'], 120)

    # -- fault injection -----------------------------------------------------
    def test_low_battery_reports_low(self):
        self.sim.fault = 'low'
        self.sim.battery_mv = 6300
        state = self.state()
        self.assertEqual(state['battery']['condition'], 'low')
        # The background fault loop drains the pack while HTTP is in flight.
        self.assertGreaterEqual(state['battery']['voltage'], 6.1)
        self.assertLessEqual(state['battery']['voltage'], 6.3)

    def test_high_battery_reports_high(self):
        self.sim.fault = 'high'
        self.sim.battery_mv = 8600
        self.assertEqual(self.state()['battery']['condition'], 'high')

    def test_unavailable_battery_hides_voltage(self):
        self.sim.fault = 'unavailable'
        state = self.state()
        self.assertEqual(state['battery']['condition'], 'unavailable')
        self.assertIsNone(state['battery']['voltage'])
        self.assertFalse(state['battery']['valid'])

    def test_offline_device_still_answers_identity(self):
        self.sim.online = False
        try:
            self.assertFalse(self.state()['online'])
            with self.assertRaises(DeviceError) as caught:
                self.client.command(HOST, {'type': 1, 'action': 1, 'mode': 3,
                                           'power': 10, 'strength': 10})
            self.assertIn('离线', str(caught.exception))
        finally:
            self.sim.online = True

    def test_link_counters_advance(self):
        first = self.state()['link']['received']
        self.sim.tick()
        self.sim.tick()
        self.assertGreater(self.state()['link']['received'], first)

    def test_simulator_status_reports_gate(self):
        self.assertEqual(self.client.simulator(HOST, 'status')['motion_gate'], 'closed')
        self.client.simulator(HOST, 'gate', True)
        self.assertEqual(self.client.simulator(HOST, 'status')['motion_gate'], 'open')


if __name__ == '__main__':
    unittest.main(verbosity=2)
