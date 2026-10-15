"""Read-only USB identity/slot query. Run with workstation Python (pyusb)."""
import json, struct
import upgrade_usb as usb
dev=usb.device()
try:
    data=bytes(dev.ctrl_transfer(0xC0,0x44,0,0xA11F,32,timeout=2000))
    if data[:4]!=b'RLYU':raise RuntimeError('Initial USB migration to rlyoulink is required')
    fields=struct.unpack('<8I',data)
    version=bytes(dev.ctrl_transfer(0xC0,0x45,0,0xA11F,64,timeout=2000))
    print(json.dumps(dict(protocol=fields[1],address=fields[2],size=fields[3],ota_state=fields[4],
        hardware=fields[5],flash_size=fields[6],layout=fields[7],
        version=version[:32].split(b'\0')[0].decode(),project=version[32:].split(b'\0')[0].decode())))
finally:usb.usb.util.dispose_resources(dev)
