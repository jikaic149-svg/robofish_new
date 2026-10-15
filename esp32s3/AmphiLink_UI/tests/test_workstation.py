"""Meaningful PC contracts: no management motion, truthful data, simulation safety."""
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from http.server import ThreadingHTTPServer
import http.client

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import credentials
import device_client
import server
from station import Workstation, MAX_RECORDS

GO = {'type': 1, 'action': 1, 'mode': 3, 'power': 35, 'strength': 40}
STOP = {**GO, 'action': 0}


class StationTests(unittest.TestCase):
    def setUp(self):
        self.station = Workstation()
        self.station.configure('simulation')

    def test_simulation_priority_recovery_and_no_implicit_takeover(self):
        s = self.station
        s.command(GO)
        with self.assertRaisesRegex(device_client.DeviceError, '优先级'):
            s.command(GO, 'vision')
        s.simulate({'lora': False})
        self.assertEqual(s.read()['safety_mode'], 'SURFACE')
        self.assertEqual(s.read()['actuators']['left_esc'], 0)
        s.command(GO, 'wifi')
        self.assertEqual(s.owner, 'wifi')
        s.simulate({'lora': True})
        self.assertEqual(s.owner, 'wifi')  # Recovery alone does not replay a goal.
        s.command(GO, 'lora')
        self.assertEqual(s.owner, 'lora')

    def test_goal_expiry_holds_actual_fins_stop_stays_stopped(self):
        s = self.station
        s.command({**GO, 'action': 3})
        before = s.read()['actuators']
        s._advance(s.last_goal + 3)
        after = s.read()
        self.assertEqual(after['safety_mode'], 'SURFACE')
        self.assertEqual(after['actuators']['left_fin'], before['left_fin'])
        self.assertEqual(after['actuators']['left_esc'], 0)
        s.command(STOP)
        s.simulate({'lora': False, 'wifi': False})
        self.assertEqual(s.read()['safety_mode'], 'WAIT_COMMAND')

    def test_spi_start_gate_does_not_stop_healthy_lora(self):
        s = self.station
        s.simulate({'spi': False})
        with self.assertRaisesRegex(device_client.DeviceError, '禁止'):
            s.command(GO)
        s.simulate({'spi': True})
        s.command(GO)
        s.simulate({'spi': False})
        self.assertEqual(s.read()['safety_mode'], 'RUNNING')

    def test_battery_stop_invalidates_goal_and_requires_new_command(self):
        s = self.station
        s.command(GO)
        s.simulate({'battery': 'low'})
        self.assertEqual(s.read()['safety_mode'], 'BATTERY_STOP')
        self.assertEqual(s.read()['actuators']['left_esc'], 0)
        s.simulate({'battery': 'none'})
        self.assertEqual(s.read()['safety_mode'], 'WAIT_COMMAND')
        self.assertEqual(s.read()['actuators']['right_esc'], 0)
        s.command(GO)
        self.assertEqual(s.read()['safety_mode'], 'RUNNING')

    def test_live_commands_never_use_http_motion(self):
        s = self.station
        with patch.object(device_client.CLIENT, '_request', side_effect=AssertionError('unexpected physical request')):
            s.configure('mission')
            with self.assertRaisesRegex(device_client.DeviceError, 'LoRa'):
                s.command(GO)
            self.assertFalse(s.read()['online'])
            self.assertEqual(s.read()['actuators'], {})

    def test_capture_export_has_source_units_gaps_and_formula_guard(self):
        s = self.station
        view = s.read()
        s._append(view)
        view.update(online=False, reason='=UNTRUSTED()')
        s._append(view)
        payload, _ = s.export('json')
        decoded = json.loads(payload)
        self.assertEqual(decoded['time_zone'], 'UTC')
        self.assertEqual(decoded['samples'][0]['source'], 'simulation')
        self.assertIsNone(decoded['samples'][1]['left_esc_pct'])
        self.assertIsNone(decoded['samples'][1]['battery_v'])
        payload, _ = s.export('csv')
        rows = list(csv.DictReader(io.StringIO(payload.decode('utf-8-sig'))))
        self.assertEqual(rows[1]['reason'], "'=UNTRUSTED()")

    def test_recording_blocks_mode_change_and_mixed_data(self):
        s = self.station
        s.capture('start')
        with self.assertRaisesRegex(device_client.DeviceError, '停止数据记录'):
            s.configure('mission')
        s.capture('stop')
        s._append(s.read())
        with patch.object(device_client.CLIENT, 'robot', return_value={'online': False}):
            s.configure('bench', '192.168.4.1')
            with self.assertRaisesRegex(device_client.DeviceError, '记录来源'):
                s.capture('start')

    def test_record_capacity_is_visible(self):
        s = self.station
        s.rows.extend([{'source': 'simulation'}] * MAX_RECORDS)
        s._append(s.read())
        self.assertEqual(s.data()['count'], MAX_RECORDS)
        self.assertEqual(s.data()['dropped'], 1)

    def test_parameter_boundary_rejects_bool_nan_and_out_of_range(self):
        for invalid in [{**GO, 'power': 101}, {**GO, 'power': float('nan')},
                        {**GO, 'power': True}, {**GO, 'type': True}]:
            with self.assertRaises(device_client.DeviceError):
                self.station.command(invalid)


class CredentialTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform == 'win32', 'Windows DPAPI contract')
    def test_current_user_roundtrip_no_plaintext_atomic_forget(self):
        with tempfile.TemporaryDirectory() as directory:
            store = credentials.KeyStore(Path(directory) / 'keys.json')
            key = 'Test-only-key-2026'
            store.save('192.168.4.1', key)
            self.assertNotIn(key, store.path.read_text())
            self.assertTrue(store.has('192.168.4.1'))
            self.assertEqual(store.get('192.168.4.1'), key)
            store.forget('192.168.4.1')
            self.assertFalse(store.has('192.168.4.1'))
            with self.assertRaises(RuntimeError):
                store.save('192.168.4.1', '12345678')


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.old_port, self.old_station = server.PORT, server.STATION
        server.STATION = Workstation()
        self.httpd = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
        server.PORT = self.httpd.server_port
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(2)
        server.PORT, server.STATION = self.old_port, self.old_station

    def call(self, path, body=None, token=True):
        connection = http.client.HTTPConnection('127.0.0.1', self.httpd.server_port, timeout=3)
        headers = {'Origin': f'http://127.0.0.1:{self.httpd.server_port}', 'Content-Type': 'application/json'}
        if token:
            headers['X-AmphiLink-Token'] = server.TOKEN
        connection.request('POST' if body is not None else 'GET', path,
                           json.dumps(body) if body is not None else None, headers)
        response = connection.getresponse()
        data = response.read()
        connection.close()
        return response.status, data

    def test_movement_proxy_disabled_and_csrf_enforced(self):
        self.assertEqual(self.call('/api/device/command', {'host': '192.168.4.1', 'command': GO})[0], 400)
        self.assertEqual(self.call('/api/station/command', {'command': GO}, token=False)[0], 403)

    def test_telemetry_not_blocked_by_hardware_lock(self):
        with server.LOCK:
            self.assertEqual(self.call('/api/station')[0], 200)
            self.assertEqual(self.call('/api/data')[0], 200)
            self.assertEqual(self.call('/api/session')[0], 200)

    def test_vision_model_forwarding_and_folder_link_keep_csrf_checks(self):
        with patch.object(server.VISION,'start',return_value={'running':True}) as start:
            code,_=self.call('/api/vision/start',{'source':2,'model':'VISION/models/fish.pt'})
            self.assertEqual(code,200)
            start.assert_called_once_with(2,server.TOKEN,server.PORT,'VISION/models/fish.pt')
        with patch.object(server.VISION,'open_output',return_value={'opened':True}) as open_folder:
            self.assertEqual(self.call('/api/vision/open-output',{},token=False)[0],403)
            open_folder.assert_not_called()
            self.assertEqual(self.call('/api/vision/open-output',{})[0],200)
            open_folder.assert_called_once()

    def test_simulation_needs_no_script_or_password_and_hardware_denied(self):
        self.assertEqual(self.call('/api/station/mode', {'mode': 'simulation'})[0], 200)
        code, body = self.call('/api/station/command', {'command': GO})
        self.assertEqual(code, 200)
        self.assertTrue(json.loads(body)['simulated'])
        self.assertEqual(self.call('/api/flash', {})[0], 400)
        self.assertEqual(self.call('/api/device/network', {'action': 'scan', 'host': '192.168.4.1'})[0], 400)


if __name__ == '__main__':
    unittest.main()
