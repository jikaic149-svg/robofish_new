"""Offline regressions for the device relay; never contacts a real probe.

The device is faked at the HTTPConnection level so the whole client path is
exercised: login, cookie reuse, 401 re-authentication, telemetry normalisation,
and the streaming-server port for the camera.
"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client
from device_client import DeviceClient, DeviceError

HEALTH = {'firmware': 'rlyoulink', 'version': '3.1.4-robofish',
          'hardware': 'xiaos3-8mb-nrst-d5', 'running_slot': 'ota_0',
          'ota_confirmed': True, 'ssid': 'lab', 'sta_ip': '192.168.1.50',
          'ap_ip': '', 'dap_client': False}

ROBOT = {'online': True, 'ready': True, 'wifi_control_allowed': False,
         'battery_state': 1, 'battery_valid': True, 'battery_voltage': 7.31,
         'battery_age_ms': 120, 'camera': True,
         'sent': 900, 'received': 880, 'invalid': 3, 'errors': 1,
         'round_trip_us': 640, 'clock_offset_us': -12, 'peer_time_us': 123456,
         'command_id': 0, 'command_ack': 0, 'command_result': 0,
         'mode': 3, 'action': 1, 'power': 55, 'failsafe': False,
         'frame_attempts': 901, 'frame_failures': 2,
         'tail1': 90.0, 'tail2': 91.5, 'left_fin': 88.0, 'right_fin': 88.5,
         'left_esc': 42, 'right_esc': 40, 'pitch': 7}


class FakeResponse:
    def __init__(self, status, payload, cookie=None):
        self.status = status
        self._payload = payload
        self._cookie = cookie

    def read(self, _limit=None):
        import json
        return b'' if self._payload is None else json.dumps(self._payload).encode()

    def getheader(self, _name):
        return self._cookie


class FakeConnection:
    """Records requests and replays canned responses."""

    calls = []
    routes = {}
    token = 'session=deadbeef'

    def __init__(self, host, port, timeout=None):
        self.host, self.port = host, port

    def request(self, method, path, body=None, headers=None):
        import json
        FakeConnection.calls.append(
            {'host': self.host, 'port': self.port, 'method': method, 'path': path,
             'cookie': (headers or {}).get('Cookie'),
             'body': json.loads(body) if body else None})

    def getresponse(self):
        call = FakeConnection.calls[-1]
        handler = FakeConnection.routes.get((call['method'], call['path']))
        if handler is None:
            return FakeResponse(404, {'error': 'not_found'})
        return handler(call)

    def close(self):
        pass


def auth_call(call):
    """Routes that require the session cookie."""
    if call['cookie'] != FakeConnection.token:
        return FakeResponse(401, {'error': 'unauthorized'})
    return None


def route(method, path, handler):
    FakeConnection.routes[(method, path)] = handler


class DeviceClientTests(unittest.TestCase):
    def setUp(self):
        FakeConnection.calls = []
        FakeConnection.routes = {}
        self.client = DeviceClient()
        self._real = device_client.http.client.HTTPConnection
        device_client.http.client.HTTPConnection = FakeConnection
        route('GET', '/api/v1/health', lambda _: FakeResponse(200, HEALTH))
        route('POST', '/api/v1/login', lambda c: FakeResponse(
            200, {'authenticated': True},
            cookie='session=deadbeef; HttpOnly; SameSite=Strict; Path=/'
        ) if c['body'].get('password') == 'secret' else FakeResponse(
            401, {'error': 'invalid_password'}))

        def robot(call):
            bad = auth_call(call)
            return bad or FakeResponse(200, ROBOT)
        route('GET', '/api/v1/robot', robot)

    def tearDown(self):
        device_client.http.client.HTTPConnection = self._real

    def test_rejects_non_private_address(self):
        for bad in ('127.0.0.1', '8.8.8.8', '0.0.0.0', '255.255.255.255', 'nope'):
            with self.subTest(host=bad), self.assertRaises(DeviceError):
                self.client.robot(bad)

    def test_robot_requires_login(self):
        with self.assertRaises(DeviceError) as caught:
            self.client.robot('192.168.1.50')
        self.assertIn('尚未登录', str(caught.exception))

    def test_login_stores_cookie_and_robot_normalises(self):
        self.client.login('192.168.1.50', 'secret')
        state = self.client.robot('192.168.1.50')
        self.assertTrue(state['online'])
        self.assertFalse(state['wifi_control_allowed'])
        self.assertEqual(state['camera_stream_url'],
                         'http://192.168.1.50:81/stream')
        self.assertEqual(state['camera_capture_url'],
                         'http://192.168.1.50/capture')
        self.assertEqual(state['battery']['condition'], 'low')
        self.assertEqual(state['battery']['condition_label'], '低压')
        self.assertAlmostEqual(state['battery']['voltage'], 7.31)
        self.assertEqual(state['motion']['mode_label'], '混合')
        self.assertEqual(state['motion']['action_label'], '前进')
        self.assertAlmostEqual(state['actuators']['tail2'], 91.5)
        self.assertEqual(state['actuators']['left_esc'], 42)
        self.assertEqual(state['link']['round_trip_us'], 640)
        # The session cookie must actually be sent on the telemetry read.
        self.assertEqual(FakeConnection.calls[-1]['cookie'], 'session=deadbeef')

    def test_wrong_password_is_reported(self):
        with self.assertRaises(DeviceError) as caught:
            self.client.login('192.168.1.50', 'wrong')
        self.assertIn('密码错误', str(caught.exception))

    def test_reauthenticates_after_session_loss(self):
        self.client.login('192.168.1.50', 'secret', remember=True)
        FakeConnection.token = 'session=rotated'
        route('POST', '/api/v1/login', lambda _: FakeResponse(
            200, {'authenticated': True}, cookie='session=rotated; Path=/'))
        state = self.client.robot('192.168.1.50')
        self.assertTrue(state['online'])
        self.assertEqual(FakeConnection.calls[-1]['cookie'], 'session=rotated')

    def test_missing_password_does_not_retry(self):
        self.client.login('192.168.1.50', 'secret')      # remember=False
        FakeConnection.token = 'session=rotated'
        with self.assertRaises(DeviceError) as caught:
            self.client.robot('192.168.1.50')
        self.assertIn('重新填写', str(caught.exception))

    def test_identity_is_readable_without_login(self):
        identity = self.client.identity('192.168.1.50')
        self.assertEqual(identity['firmware'], 'rlyoulink')
        self.assertEqual(identity['running_slot'], 'ota_0')
        self.assertFalse(identity['authenticated'])

    def test_non_probe_device_is_rejected(self):
        route('GET', '/api/v1/health', lambda _: FakeResponse(
            200, {'firmware': 'something-else'}))
        with self.assertRaises(DeviceError) as caught:
            self.client.identity('192.168.1.50')
        self.assertIn('不是 rlyoulink', str(caught.exception))

    def test_unreachable_device_reports_clearly(self):
        def boom(self, method, path, body=None, headers=None):
            raise OSError('connection refused')
        original = FakeConnection.request
        FakeConnection.request = boom
        try:
            with self.assertRaises(DeviceError) as caught:
                self.client.identity('192.168.1.50')
            self.assertIn('找不到设备', str(caught.exception))
        finally:
            # Restore the class attribute; deleting it would remove the method
            # from every later test.
            FakeConnection.request = original

    def test_battery_unavailable_when_offline(self):
        route('GET', '/api/v1/robot', lambda c: auth_call(c) or FakeResponse(
            200, dict(ROBOT, battery_state=3, battery_valid=False,
                      battery_voltage=None, online=False)))
        self.client.login('192.168.1.50', 'secret')
        state = self.client.robot('192.168.1.50')
        self.assertEqual(state['battery']['condition'], 'unavailable')
        self.assertIsNone(state['battery']['voltage'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
