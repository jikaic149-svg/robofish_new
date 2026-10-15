"""A fake rlyoulink probe, for driving the host UI with no hardware attached.

It answers the same HTTP surface the real ESP32-S3 probe exposes, so the whole
chain can be exercised end to end: login and session cookie, telemetry, command
queueing with an ACK result, the camera stream, and device identity.

Deliberately dependency-free (standard library only) and loopback-bound.

    python simulator.py --port 8900 --password 12345678

Behaviour worth knowing about:
  * The motion gate starts CLOSED, matching the shipped STM32 firmware where
    `wifiControlAllowed = false`. Commands are accepted and ACKed as rejected.
  * Press the "打开运动闸门" switch in the UI (or POST /__sim/gate) to let
    commands through, which is how the real firmware behaves once the gate is
    enabled.
  * Battery, link counters and the six actuator channels move on their own, and
    every --fault-interval seconds one fault state is cycled so the warning
    paths can be looked at without waiting for real hardware.
"""
from __future__ import annotations

import argparse
import json
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

COOKIE_NAME = 'rlyoulink_session'
DEFAULT_PASSWORD = '12345678'
TELEMETRY_PERIOD_MS = 100
ACK_DELAY_MS = 120
CAMERA_FPS = 8

# Mirrors the firmware's battery condition codes.
BATTERY_NORMAL, BATTERY_LOW, BATTERY_HIGH, BATTERY_UNAVAILABLE = 0, 1, 2, 3
DRIVE_MODES = {0: '待机', 1: '仿生', 2: '螺旋桨', 3: '混合'}
MOTION_ACTIONS = {0: '停止', 1: '前进', 2: '后退', 3: '左转', 4: '右转'}

# Faults cycled by the background model so the UI warning paths get exercised.
FAULTS = ['none', 'none', 'none', 'low', 'none', 'high', 'none', 'unavailable']


class Simulator:
    """Mutating device state; one lock, read by request threads."""

    def __init__(self, password: str, fault_interval: float, host_label: str):
        self.guard = threading.Lock()
        self.password = password
        self.fault_interval = fault_interval
        self.host_label = host_label
        self.started = time.monotonic()
        self.token = 'sim%08x' % random.getrandbits(32)

        self.gate_open = False          # mirrors STM32 wifiControlAllowed
        self.fault = 'none'
        self.online = True
        self.ready = True
        self.camera = True

        self.sent = 0
        self.received = 0
        self.invalid = 0
        self.errors = 0
        self.frame_attempts = 0
        self.frame_failures = 0
        self.round_trip_us = 640
        self.clock_offset_us = -18
        self.peer_a = 0
        self.peer_sequence = 0

        self.mode = 3
        self.action = 0
        self.power = 0
        self.strength = 0
        self.failsafe = False
        self.pitch = 90

        self.tail1 = 90.0
        self.tail2 = 90.0
        self.left_fin = 90.0
        self.right_fin = 90.0
        self.left_esc = 0
        self.right_esc = 0

        self.battery_mv = 7400
        self.battery_age_ms = 0

        self.command_id = 0
        self.command_ack = 0
        self.command_result = 0
        self.command_received_at = 0.0
        self.command_payload = None

    # -- session -----------------------------------------------------------
    def password_ok(self, password):
        return isinstance(password, str) and password == self.password

    # -- background loop ----------------------------------------------------
    def start_background(self):
        """Run the model and ACK timer. Used by main() and by tests."""
        thread = threading.Thread(target=self._run, daemon=True)
        thread.start()
        return thread

    def _run(self):
        while True:
            time.sleep(TELEMETRY_PERIOD_MS / 1000)
            self.tick()
            self.poll_ack()

    # -- command handling --------------------------------------------------
    def accept_command(self, command: dict) -> int:
        with self.guard:
            self.command_id += 1
            self.command_payload = command
            self.command_received_at = time.monotonic()
            self.command_ack = 0
            self.command_result = 0
            return self.command_id

    def poll_ack(self):
        """After ACK_DELAY_MS, publish a result and apply the command if allowed."""
        with self.guard:
            pending = self.command_id and self.command_ack != self.command_id
            if not pending:
                return
            if (time.monotonic() - self.command_received_at) * 1000 < ACK_DELAY_MS:
                return
            self.command_ack = self.command_id
            if not self.gate_open:
                self.command_result = 2          # rejected
                return
            self.command_result = 1              # accepted
            self.apply_command(self.command_payload or {})

    def apply_command(self, command: dict):
        kind = int(command.get('type', 0))
        if kind == 3:                            # pitch
            self.pitch = max(0, min(180, int(command.get('pitch', 90))))
            return
        action = int(command.get('action', 0))
        self.mode = max(0, min(3, int(command.get('mode', 0))))
        self.power = max(0, min(100, int(command.get('power', 0))))
        self.strength = max(0, min(100, int(command.get('strength', 0))))
        self.action = action
        self.failsafe = False
        if action == 0:
            self.power = 0
            self.strength = 0
        # Simple visual model: differential drive on the two ESCs, tail follows
        # the requested action so the meters move in a believable way.
        sign = {1: 1, 2: -1}.get(action, 0)
        turn = {3: -1, 4: 1}.get(action, 0) * self.strength
        base = self.power if sign else 0
        self.left_esc = max(-100, min(100, int(base * sign - turn / 2)))
        self.right_esc = max(-100, min(100, int(base * sign + turn / 2)))
        swing = 18 if action in (1, 2) else 0
        self.tail1 = 90.0 + swing * sign
        self.tail2 = 90.0 + swing * sign
        self.left_fin = 90.0 + turn * 0.12
        self.right_fin = 90.0 + turn * 0.12
        self.telemetry_last_command = time.monotonic()

    # -- background model ---------------------------------------------------
    def tick(self):
        """Advance counters, battery drift and the fault cycle."""
        with self.guard:
            self.sent += random.randint(9, 11)
            self.received += random.randint(9, 11)
            self.frame_attempts += random.randint(9, 11)
            if random.random() < 0.02:
                self.invalid += 1
            if random.random() < 0.01:
                self.frame_failures += 1
                self.errors += 1
            self.round_trip_us = max(180, min(2400, self.round_trip_us + random.randint(-90, 90)))
            self.clock_offset_us = max(-400, min(400, self.clock_offset_us + random.randint(-25, 25)))
            self.battery_age_ms = min(500, self.battery_age_ms + TELEMETRY_PERIOD_MS)

        # Fault cycle and battery level, outside the guard to keep it short.
        if self.fault_interval > 0:
            elapsed = time.monotonic() - self.started
            self.fault = FAULTS[int(elapsed / self.fault_interval) % len(FAULTS)]
        self._apply_fault()

    def _apply_fault(self):
        with self.guard:
            fault = self.fault
            if fault == 'low':
                self.battery_mv = max(6100, self.battery_mv - 55)
            elif fault == 'high':
                self.battery_mv = min(8700, self.battery_mv + 70)
            else:
                # Drift toward a healthy 2S pack.
                self.battery_mv += (7400 - self.battery_mv) * 0.06
                self.battery_mv = int(self.battery_mv)
            self.battery_age_ms = min(self.battery_age_ms, 400)

    # -- views --------------------------------------------------------------
    def battery_state(self):
        if self.fault == 'unavailable' or not self.online:
            return BATTERY_UNAVAILABLE
        if self.fault == 'low' or self.battery_mv < 6600:
            return BATTERY_LOW
        if self.fault == 'high' or self.battery_mv > 8400:
            return BATTERY_HIGH
        return BATTERY_NORMAL

    def health(self):
        with self.guard:
            gate = 'open' if self.gate_open else 'closed'
            return {
                'firmware': 'rlyoulink', 'version': '3.1.4-simulator',
                'hardware': 'xiaos3-8mb-nrst-d5', 'running_slot': 'ota_0',
                'ota_confirmed': True, 'ssid': 'sim-lab', 'sta_ip': self.host_label,
                'ap_ip': '', 'dap_client': False, 'online': self.online,
                'simulator': True, 'motion_gate': gate,
            }

    def robot(self):
        with self.guard:
            unavailable = self.fault == 'unavailable' or not self.online
            return {
                'online': self.online, 'ready': self.ready,
                'wifi_control_allowed': self.gate_open,
                'battery_state': self.battery_state(),
                'battery_valid': not unavailable,
                'battery_voltage': None if unavailable else self.battery_mv / 1000.0,
                'battery_age_ms': 65535 if unavailable else self.battery_age_ms,
                'camera': self.camera and not unavailable,
                'sent': self.sent, 'received': self.received,
                'invalid': self.invalid, 'errors': self.errors,
                'round_trip_us': self.round_trip_us, 'clock_offset_us': self.clock_offset_us,
                'peer_time_us': int((time.monotonic() - self.started) * 1000),
                'command_id': self.command_id, 'command_ack': self.command_ack,
                'command_result': self.command_result,
                'mode': self.mode, 'action': self.action, 'power': self.power,
                'strength': self.strength, 'failsafe': self.failsafe,
                'frame_attempts': self.frame_attempts, 'frame_failures': self.frame_failures,
                'tail1': round(self.tail1, 1), 'tail2': round(self.tail2, 1),
                'left_fin': round(self.left_fin, 1), 'right_fin': round(self.right_fin, 1),
                'left_esc': self.left_esc, 'right_esc': self.right_esc,
                'pitch': self.pitch,
            }


def jpeg_frame(index: int) -> bytes:
    """Tiny solid-colour JPEG, moved along a hue ramp so the stream visibly runs."""
    from io import BytesIO
    try:
        from PIL import Image
    except Exception:                      # pragma: no cover - stdlib fallback
        return b''
    hue = (index * 6) % 256
    image = Image.new('RGB', (320, 240), (hue, 90, 255 - hue))
    buffer = BytesIO()
    image.save(buffer, 'JPEG', quality=70)
    return buffer.getvalue()


def make_handler(sim: Simulator):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'rlyoulink-sim/1.0'

        def log_message(self, *args):
            pass

        # -- helpers --------------------------------------------------------
        def send_json(self, status, payload, cookie=None):
            body = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            if cookie:
                self.send_header('Set-Cookie', cookie)
            self.end_headers()
            self.wfile.write(body)

        def error(self, status, code):
            self.send_json(status, {'error': code})

        def authenticated(self):
            header = self.headers.get('Cookie', '')
            for part in header.split(';'):
                name, _, value = part.strip().partition('=')
                if name == COOKIE_NAME and value == sim.token:
                    return True
            return False

        def read_json(self):
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length < 1024 * 1024:
                return None
            try:
                return json.loads(self.rfile.read(length))
            except ValueError:
                return None

        # -- routes ---------------------------------------------------------
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/v1/health':
                return self.send_json(200, sim.health())
            if path == '/__sim/gate':
                return self.send_json(200, {'motion_gate': 'open' if sim.gate_open else 'closed'})
            if path == '/api/v1/robot':
                if not self.authenticated():
                    return self.error(401, 'unauthorized')
                return self.send_json(200, sim.robot())
            if path == '/api/v1/status':
                if not self.authenticated():
                    return self.error(401, 'unauthorized')
                return self.send_json(200, {'mode': 'sta', 'ssid': 'sim-lab', 'ip': sim.host_label,
                                             'sta_connected': True, 'sta_ip': sim.host_label,
                                             'ap_enabled': False, 'ap_ip': '', 'scanning': False,
                                             'firmware': 'rlyoulink', 'version': '3.1.4-simulator',
                                             'running_slot': 'ota_0', 'ota_confirmed': True,
                                             'dap_client': False})
            if path == '/stream':
                return self.stream()
            if path == '/capture':
                return self.capture()
            return self.error(404, 'not_found')

        def do_POST(self):
            path = urlsplit(self.path).path
            if path == '/api/v1/login':
                body = self.read_json() or {}
                if not sim.password_ok(body.get('password')):
                    return self.error(401, 'invalid_password')
                return self.send_json(
                    200, {'authenticated': True},
                    cookie=f'{COOKIE_NAME}={sim.token}; HttpOnly; SameSite=Strict; Path=/')
            if path == '/__sim/gate':
                body = self.read_json() or {}
                sim.gate_open = bool(body.get('open'))
                return self.send_json(200, {'motion_gate': 'open' if sim.gate_open else 'closed'})
            if path == '/__sim/fault':
                body = self.read_json() or {}
                sim.fault = str(body.get('fault', 'none'))
                return self.send_json(200, {'fault': sim.fault})
            if path == '/__sim/offline':
                body = self.read_json() or {}
                sim.online = not bool(body.get('offline'))
                return self.send_json(200, {'online': sim.online})
            if path == '/api/v1/robot/command':
                if not self.authenticated():
                    return self.error(401, 'unauthorized')
                command = self.read_json()
                if not isinstance(command, dict):
                    return self.error(400, 'invalid_json')
                if not sim.online:
                    return self.error(409, 'controller_offline')
                return self.send_json(202, {'id': sim.accept_command(command), 'state': 'queued'})
            return self.error(404, 'not_found')

        # -- camera ---------------------------------------------------------
        def stream(self):
            if not sim.camera:
                return self.error(503, 'camera_unavailable')
            boundary = 'rlyoulinksim'
            self.send_response(200)
            self.send_header('Content-Type', f'multipart/x-mixed-replace;boundary={boundary}')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            index = 0
            try:
                while True:
                    frame = jpeg_frame(index)
                    if not frame:
                        break
                    self.wfile.write(f'--{boundary}\r\n'.encode())
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                    index += 1
                    time.sleep(1.0 / CAMERA_FPS)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        def capture(self):
            frame = jpeg_frame(random.randint(0, 255))
            if not frame:
                return self.error(503, 'camera_unavailable')
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8900)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--password', default=DEFAULT_PASSWORD)
    parser.add_argument('--fault-interval', type=float, default=12.0,
                        help='秒；每隔这么久切换一次电池/链路故障，0 表示不自动切换')
    args = parser.parse_args()

    sim = Simulator(args.password, args.fault_interval, f'{args.host}:{args.port}')
    sim.start_background()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(sim))
    print(f'仿真探针已启动: http://{args.host}:{args.port}')
    print(f'  管理密码: {args.password}')
    print(f'  运动闸门: {"开启" if sim.gate_open else "关闭（与当前 STM32 固件一致）"}')
    print(f'  故障轮换: {"每 %.0f 秒" % args.fault_interval if args.fault_interval else "关闭"}')
    print('  在界面里点「使用仿真设备」即可连上；Ctrl+C 结束。')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n已停止。')


if __name__ == '__main__':
    main()
