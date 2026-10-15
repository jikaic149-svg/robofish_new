"""Build a wired factory image and inspectable A/B manifest; no private keys."""
from pathlib import Path
import argparse,hashlib,json,re,struct

ROOT=Path(__file__).resolve().parents[1]
def layout():
    header=(ROOT/'common/stm32_boot.h').read_text(encoding='utf-8')
    values={}
    for name,value in re.findall(r'^#define (RF_\w+) (?:UINT32_C\()?((?:0x[0-9A-Fa-f]+)|\d+)',header,re.M):
        values[name]=int(value,0)
    return values

def native_crc(data):
    crc=0xFFFFFFFF
    for start in range(0,len(data),4):
        crc ^= int.from_bytes(data[start:start+4].ljust(4,b'\xff'),'little')
        for _ in range(32):
            crc=((crc<<1)^ (0x04C11DB7 if crc&0x80000000 else 0))&0xFFFFFFFF
    return crc

def package(build):
    c=layout(); images=[]
    for name,base,limit in [('RoboFishBoot',c['RF_BOOT_BASE'],c['RF_BOOT_END']-c['RF_BOOT_BASE']),
                            ('RoboFishF405',c['RF_APP_A'],c['RF_APP_MAX_BYTES']),
                            ('RoboFishF405_B',c['RF_APP_B'],c['RF_APP_MAX_BYTES'])]:
        data=(build/(name+'.bin')).read_bytes()
        sp,pc=struct.unpack_from('<II',data)
        if not c['RF_VECTOR_BYTES']<=len(data)<=limit or sp!=c['RF_FAULT_RAM']:
            raise ValueError('Size/stack outside fixed layout: '+name)
        if not pc&1 or not base+c['RF_VECTOR_BYTES']<=(pc&~1)<base+len(data):
            raise ValueError('Reset vector outside linked image: '+name)
        images.append((name,base,data,native_crc(data),hashlib.sha256(data).digest()))
    boot,a,b=images
    header=struct.pack('<8I',c['RF_BOOT_MAGIC'],c['RF_BOOT_FORMAT'],1,0,c['RF_BOOT_NONE'],c['RF_BOOT_STABLE'],0,3)
    body=header+b''.join(struct.pack('<II',len(image[2]),image[3])+image[4] for image in (a,b))
    assert len(body)==112
    record=body+struct.pack('<II',native_crc(body),c['RF_BOOT_COMMIT'])
    factory=bytearray(b'\xff'*(b[1]-c['RF_BOOT_BASE']+len(b[2])))
    for image in images:
        offset=image[1]-c['RF_BOOT_BASE'];factory[offset:offset+len(image[2])]=image[2]
    offset=c['RF_RECORD0']-c['RF_BOOT_BASE'];factory[offset:offset+len(record)]=record
    (build/'RoboFishFactory.bin').write_bytes(factory)
    manifest={'layout':'f405-ab-v1','deployment':'wired factory; application update transport not yet enabled',
              'factory_address':hex(c['RF_BOOT_BASE']),'factory_bytes':len(factory),
              'factory_sha256':hashlib.sha256(factory).hexdigest(),
              'crc':{'poly':'0x04c11db7','init':'0xffffffff','refin':False,'refout':False,'xorout':0,
                     'word_order':'little-endian 32-bit load, MSB first','tail_padding':'0xff'},
              'images':[{'file':name+'.bin','address':hex(base),'bytes':len(data),'crc32':hex(crc),'sha256':sha.hex()}
                        for name,base,data,crc,sha in images]}
    (build/'stm32_ab_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build-dir',type=Path,required=True)
    args=p.parse_args();m=package(args.build_dir)
    print('PASS: vectors, limits, native CRC and factory record; factory bytes:',m['factory_bytes'])
