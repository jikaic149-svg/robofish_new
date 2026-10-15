"""Verified signed application -> inactive slot -> OTA selection, preserving NVS."""
import json, subprocess, sys, tempfile, struct, binascii, time, os
from pathlib import Path
from rlyou_release import verify, ROOT
from esptool.targets.esp32s3 import ESP32S3ROM
from rlyou_stream import run_stream
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
sys.stderr.reconfigure(encoding='utf-8',errors='replace')
ENV={**os.environ,'PYTHONIOENCODING':'utf-8'}

def run(args):
    p=subprocess.run(args,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=180,env=ENV)
    if p.returncode:raise RuntimeError(p.stdout+'\n'+p.stderr)
    return p.stdout
def query():return json.loads(run(['D:/python/3.13/python.exe',str(ROOT/'rlyou_usb_info.py')]))
def tool(port,*args):
    from serial.tools import list_ports
    for _ in range(40):
        if any(p.device==port for p in list_ports.comports()):break
        time.sleep(.25)
    else:raise RuntimeError('ROM port did not re-enumerate: '+port)
    result=run_stream([sys.executable,'-u','-m','esptool','--chip','esp32s3','--port',port,
        '--before','no_reset','--after','no_reset',*args],
        lambda line:print(line,flush=True),timeout=180,env=ENV)
    read_done=(args[0]=='read_flash' and Path(args[-1]).exists() and
               Path(args[-1]).stat().st_size==int(args[2],0) and
               f'Read {int(args[2],0)} bytes' in result.stdout)
    if result.returncode and not (read_done or (args[0]=='write_flash' and result.stdout.count('Hash of data verified.')==1)):
        raise RuntimeError(result.stdout+'\n'+result.stderr)
    time.sleep(.5)

def upgrade(path):
    m,_,_,app=verify(path)
    info=query()
    if info['hardware']!=0x58334432 or info['flash_size']!=0x800000 or info['layout']!=1 or info['size']!=0x200000:
        raise ValueError('USB hardware/layout mismatch')
    if info['address'] not in (0x20000,0x220000) or info['ota_state']==1:
        raise ValueError('Running slot is unknown or awaiting confirmation')
    target=0x220000 if info['address']==0x20000 else 0x20000
    slot=1 if target==0x220000 else 0
    output=run(['D:/python/3.13/python.exe',str(ROOT/'upgrade_usb.py'),'--enter'])
    import re
    match=re.search(r'Download port: (COM\d+)',output)
    if not match:raise RuntimeError('ROM port did not appear')
    port=match.group(1)
    esp=ESP32S3ROM(port,baud=115200)
    try:
        esp.connect(mode='no_reset',attempts=1)
        if ':'.join(f'{x:02x}' for x in esp.read_mac())!='dc:b4:d9:3b:2b:00':raise ValueError('Unexpected USB probe')
    finally:esp._port.close()
    with tempfile.TemporaryDirectory(prefix='rlyoulink-usb-') as tmp:
        tmp=Path(tmp);part=tmp/'part.bin';metadata=tmp/'ota.bin';image=tmp/'app.bin'
        tool(port,'read_flash','0x8000','0x1000',str(part))
        # Require the installed partition table to match this supported layout.
        sys.path.insert(0,'D:/ESP32/ESP-IDF-v5.3.2/components/partition_table')
        import gen_esp32part
        table=gen_esp32part.PartitionTable.from_binary(part.read_bytes())
        actual={p.name:(p.offset,p.size) for p in table}
        if any(actual.get(k)!=v for k,v in {'ota_0':(0x20000,0x200000),'ota_1':(0x220000,0x200000),'otadata':(0x1d000,0x2000)}.items()):
            raise ValueError('Installed partition table mismatch')
        tool(port,'read_flash','0x1d000','0x2000',str(metadata))
        raw=metadata.read_bytes();valid=[]
        for index in (0,1):
            seq,state,crc=struct.unpack_from('<I20xII',raw,index*4096)
            if seq!=0xffffffff and state not in (3,4) and crc==(binascii.crc32(struct.pack('<I',seq),0xffffffff)&0xffffffff):
                valid.append((seq,index))
        maximum,index=max(valid) if valid else (0,1)
        if valid and (maximum-1)%2!=(0 if info['address']==0x20000 else 1):
            raise ValueError('Boot selection differs from running slot; rescue required')
        seq=maximum+1
        while (seq-1)%2!=slot:seq+=1
        if seq>=0xffffffff:raise ValueError('OTA sequence exhausted')
        selection=struct.pack('<I',seq)+b'\xff'*20+struct.pack('<II',0,binascii.crc32(struct.pack('<I',seq),0xffffffff)&0xffffffff)
        image.write_bytes(app)
        print('Writing inactive application slot',hex(target),flush=True)
        tool(port,'write_flash',hex(target),str(image))
        select=tmp/'selection.bin';select.write_bytes(selection+b'\xff'*(4096-len(selection)))
        print('Image verified; committing OTA boot selection',flush=True)
        tool(port,'write_flash',hex(0x1d000+(1-index)*4096),str(select))
    print('Waiting for confirmed application',flush=True)
    run([sys.executable,str(ROOT/'restart-v2-rom.py'),port])
    for _ in range(25):
        time.sleep(1)
        try:
            result=query()
            if result['address']==target and result['version']==m['version'] and result['ota_state']==2:
                print(json.dumps({'complete':True,'slot':slot,'version':m['version']}));return
        except Exception:pass
    raise RuntimeError('Write completed but new version/boot confirmation not verified')

if __name__=='__main__':upgrade(sys.argv[1])
