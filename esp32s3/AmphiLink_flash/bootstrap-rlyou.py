"""One-time USB migration from the known v2 layout; retains NVS and old app."""
import binascii,hashlib,json,struct,sys,tempfile
from pathlib import Path
from rlyou_usb_upgrade import tool,run,ROOT
from rlyou_release import verify
from esptool.targets.esp32s3 import ESP32S3ROM

port=sys.argv[1]
release=ROOT/'rlyoulink-release'
expected=json.loads((release/'bootstrap-hashes.json').read_text())
for name,digest in expected.items():
    if hashlib.sha256((release/name).read_bytes()).hexdigest()!=digest:raise ValueError('Bootstrap hash mismatch')
verify(release/'rlyoulink-xiaos3-d2-3.0.0.amphipkg')
esp=ESP32S3ROM(port,baud=115200)
try:
    esp.connect(mode='no_reset',attempts=1)
    if ':'.join(f'{x:02x}' for x in esp.read_mac())!='dc:b4:d9:3b:2b:00':raise ValueError('Unexpected device')
finally:esp._port.close()
with tempfile.TemporaryDirectory(prefix='rlyou-bootstrap-') as d:
    d=Path(d);part=d/'part.bin';ota=d/'ota.bin'
    tool(port,'read_flash','0x8000','0x1000',str(part))
    candidate=(release/'partition-table.bin').read_bytes()
    if part.read_bytes()[:len(candidate)]!=candidate:raise ValueError('Partition layout differs; no writes performed')
    tool(port,'read_flash','0x1d000','0x2000',str(ota))
    old_record=struct.pack('<I',1)+b'\xff'*20+struct.pack('<II',2,binascii.crc32(struct.pack('<I',1),0xffffffff)&0xffffffff)
    empty=ota.read_bytes()==b'\xff'*8192
    known_slot0=ota.read_bytes()==old_record+b'\xff'*(8192-len(old_record))
    if not empty and not known_slot0:
        raise ValueError('Bootstrap supports empty v2 metadata or verified slot0 sequence 1 only')
    old=d/'old-header.bin';tool(port,'read_flash','0x20000','0x100',str(old))
    if old.read_bytes()[0]!=0xe9:raise ValueError('No old application to preserve')
    tool(port,'write_flash','0x220000',str(release/'application.bin'))
    tool(port,'write_flash','0x0',str(release/'bootloader.bin'))
    # First create a valid record for the preserved old slot, then select new.
    for index,seq,state in [(0,1,2),(1,2,0)]:
        if index==0 and known_slot0:continue
        record=struct.pack('<I',seq)+b'\xff'*20+struct.pack('<II',state,binascii.crc32(struct.pack('<I',seq),0xffffffff)&0xffffffff)
        path=d/f'select{index}.bin';path.write_bytes(record+b'\xff'*(4096-len(record)))
        tool(port,'write_flash',hex(0x1d000+index*4096),str(path))
run([sys.executable,str(ROOT/'restart-v2-rom.py'),port])
print('Bootstrap written and restarted; verify USB and HTTP boot confirmation')
