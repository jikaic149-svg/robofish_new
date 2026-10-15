"""USB maintenance v1: query, enter ROM, or flash the packaged candidate."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

import usb.core
import usb.util
import usb.backend.libusb1
from serial.tools import list_ports

SERIAL = 'DCB4D93B2B00'
MAC = 'dc:b4:d9:3b:2b:00'
ROOT = Path(__file__).resolve().parent
PYTHON = 'D:/ESP32/.espressif/python_env/idf5.3_py3.13_env/Scripts/python.exe'


def device():
    backend = usb.backend.libusb1.get_backend(
        find_library=lambda _: str(ROOT/'openocd-tcp-fix/package/bin/libusb-1.0.dll'))
    for dev in usb.core.find(find_all=True, idVendor=0x303A, idProduct=0x83B3,
                             backend=backend):
        if usb.util.get_string(dev, dev.iSerialNumber).upper() == SERIAL:
            return dev
    raise RuntimeError('Matching USB AmphiLink not found')


def query(dev):
    data = bytes(dev.ctrl_transfer(0xC0, 0x42, 0, 0xA11F, 8, timeout=2000))
    if data != b'ALUP\x01\x01\x00\x00':
        raise RuntimeError(f'Unsupported maintenance response: {data.hex()}')
    return data


def rom_port():
    for port in list_ports.comports():
        serial = (port.serial_number or '').replace(':', '').upper()
        if port.vid == 0x303A and serial == SERIAL:
            return port.device
    return None


def run_tool(port, *args):
    cmd = [PYTHON, '-m', 'esptool', '--chip', 'esp32s3', '--port', port,
           '--before', 'no_reset', '--after', 'no_reset', *args]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    print(result.stdout, end='')
    if result.returncode:
        raise RuntimeError(result.stderr or 'esptool failed')
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--enter', action='store_true')
    action.add_argument('--flash', action='store_true')
    parser.add_argument('--port', help='Already in ROM download mode (initial install)')
    args = parser.parse_args()
    package = ROOT / 'xiao-usb-nrst-d2'
    if args.flash:
        raise RuntimeError('Legacy raw flash is disabled; use rlyou_usb_upgrade.py with a signed .amphipkg')
        manifest = json.loads((package / 'manifest.json').read_text(encoding='utf-8'))
        for name, digest in manifest['sha256'].items():
            if hashlib.sha256((package / name).read_bytes()).hexdigest() != digest:
                raise RuntimeError(f'Firmware hash mismatch: {name}')
    port = args.port
    if not port:
        dev = device()
        try:
            query(dev)
            print('USB maintenance v1 supported')
            if not (args.enter or args.flash):
                return
            dev.ctrl_transfer(0x40, 0x42, 0xB007, 0xA11F, b'', timeout=2000)
        finally:
            usb.util.dispose_resources(dev)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            port = rom_port()
            if port:
                break
            time.sleep(0.25)
        if not port:
            raise RuntimeError('Download port did not appear within 15 seconds')
    print(f'Download port: {port}')
    if not args.flash:
        return
    identity = run_tool(port, '--no-stub', 'read_mac')
    if f'MAC: {MAC}' not in identity:
        raise RuntimeError('Device MAC mismatch; flash cancelled')
    time.sleep(1)  # Windows CDC handle teardown after the identity command.
    run_tool(port, 'write_flash', '--flash_mode', 'dio', '--flash_freq', '80m',
             '--flash_size', '8MB', '0x0', str(package / 'bootloader.bin'),
             '0x20000', str(package / 'AmphiLink.bin'))
    time.sleep(1)
    # Reset the stub into ROM, clear forced download, then use the same S3
    # watchdog startup sequence already used successfully in this workspace.
    code = '''from esptool.targets.esp32s3 import ESP32S3ROM
import serial,sys
esp=ESP32S3ROM(sys.argv[1], baud=115200)
try:
    esp.connect(mode="no_reset", attempts=1)
    if ":".join(f"{b:02x}" for b in esp.read_mac()) != sys.argv[2]:
        raise RuntimeError("MAC mismatch before restart")
    esp.write_reg(esp.RTC_CNTL_OPTION1_REG,0,esp.RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)
    try:
        esp.watchdog_reset()
    except serial.SerialException:
        pass
finally:
    esp._port.close()
'''
    subprocess.run([PYTHON, '-c', code, port, MAC], check=True, timeout=20)
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        try:
            dev = device()
            try:
                query(dev)
            finally:
                usb.util.dispose_resources(dev)
            print('Flash verified; USB maintenance interface returned')
            return
        except (usb.core.USBError, RuntimeError):
            time.sleep(0.5)
    raise RuntimeError('Flash completed, but application USB was not confirmed')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        raise SystemExit(str(exc))
