"""Passive discovery and USB enumeration; never claim a DAP interface."""
import json
from pathlib import Path
import socket
import threading
import time
import ipaddress
import http.client
from tooling import OPENOCD_HOME


class ProbeMonitor:
    def __init__(self):
        self.guard = threading.Lock()
        self.enabled = True
        self.generation = 0
        self.wifi = {}
        self.wifi_error = ''
        self.http = {}
        self.watched = {}
        self.usb = {'state': 'unknown', 'detail': 'USB 检测启动中'}

    def start(self):
        for task in (self.discovery, self.enumerate_usb, self.http_monitor):
            threading.Thread(target=task, daemon=True).start()

    def set_enabled(self, enabled):
        with self.guard:
            self.enabled = enabled
            self.generation += 1
            self.http.clear()
            self.watched.clear()

    def discovery(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind(('', 4442))
                while True:
                    data, address = sock.recvfrom(2048)
                    try:
                        packet = json.loads(data)
                        if not isinstance(packet, dict) or packet.get('name') not in ('AmphiLink','rlyoulink'):
                            continue
                    except (ValueError, UnicodeError):
                        continue
                    with self.guard:
                        now = time.time()
                        self.wifi = {ip: stamp for ip, stamp in self.wifi.items() if now-stamp < 60}
                        self.wifi[address[0]] = now
        except OSError as exc:
            self.wifi_error = str(exc)

    def enumerate_usb(self):
        try:
            import usb.core
            import usb.util
            import usb.backend.libusb1
            backend = usb.backend.libusb1.get_backend(
                find_library=lambda _: str(OPENOCD_HOME/'bin/libusb-1.0.dll'))
            if backend is None:
                raise RuntimeError('libusb 不可用')
        except Exception as exc:
            self.usb = {'state': 'unknown', 'detail': 'USB 检测不可用：'+str(exc)}
            return
        while True:
            if not self.enabled:
                time.sleep(2)
                continue
            try:
                found = False
                for dev in usb.core.find(find_all=True, idVendor=0x303A, idProduct=0x83B3, backend=backend):
                    try:
                        found |= usb.util.get_string(dev, dev.iSerialNumber).upper() == 'DCB4D93B2B00'
                    finally:
                        usb.util.dispose_resources(dev)
                self.usb = {'state': 'online' if found else 'offline', 'checked_at': time.time(),
                            'detail': 'USB 枚举 · DCB4D93B2B00' if found else '未枚举到指定 USB 探针'}
            except Exception as exc:
                self.usb = {'state': 'unknown', 'checked_at': time.time(), 'detail': str(exc)}
            time.sleep(2)

    @staticmethod
    def read_health(host):
        connection = http.client.HTTPConnection(host, 80, timeout=1.5)
        try:
            connection.request('GET', '/api/v1/health', headers={'Connection': 'close'})
            response = connection.getresponse()
            payload = response.read(4097)
            if response.status != 200 or len(payload) > 4096:
                raise ValueError('设备状态接口未就绪，请确认探针固件已更新')
            data = json.loads(payload)
            if not isinstance(data, dict) or data.get('firmware') not in ('amphilink-apsta-v2','rlyoulink'):
                raise ValueError('响应设备不是当前 AmphiLink 探针')
            return {'state': 'online', 'checked_at': time.time(), 'detail': 'HTTP 状态应答',
                    'ssid': data.get('ssid', ''), 'ip': host, 'sta_ip': data.get('sta_ip', ''),
                    'dap_client': bool(data.get('dap_client'))}
        finally:
            connection.close()

    def http_monitor(self):
        while True:
            with self.guard:
                now = time.time()
                self.watched = {ip: t for ip, t in self.watched.items() if now-t < 30}
                hosts = list(self.watched)
                hosts += [ip for ip, t in self.wifi.items() if now-t < 60 and ip not in hosts]
                generation = self.generation
                if not self.enabled: hosts = []
            for host in hosts[:4]:
                if generation != self.generation: break
                try:
                    result = self.read_health(host)
                except Exception as exc:
                    result = {'state': 'unknown', 'checked_at': time.time(), 'detail': str(exc)}
                with self.guard:
                    if generation != self.generation: break
                    self.http[host] = result
                    self.http = {ip: value for ip, value in self.http.items() if time.time()-value['checked_at'] < 60}
            time.sleep(2)

    def snapshot(self, host=None):
        with self.guard:
            if host and self.enabled:
                address = ipaddress.IPv4Address(host)
                if not address.is_private or address.is_loopback or address.is_multicast or address.is_unspecified or int(address)==0xffffffff:
                    raise ValueError('请使用局域网探针 IPv4 地址')
                if host in self.watched or len(self.watched) < 4:
                    self.watched[host] = time.time()
            wifi = {ip: stamp for ip, stamp in self.wifi.items() if time.time()-stamp < 60}
            return {'enabled': self.enabled, 'usb': dict(self.usb), 'wifi': wifi,
                    'http': dict(self.http), 'wifi_error': self.wifi_error, 'checked_at': time.time()}
