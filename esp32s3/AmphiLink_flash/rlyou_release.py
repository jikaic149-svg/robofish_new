"""Signed release packages. Run with the existing ESP-IDF Python environment."""
import argparse
import hashlib
import json
import struct
import zipfile
from pathlib import Path
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa, padding

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT/'firmware-v2'
PUBLIC = SOURCE/'components/APP/amphilink/tcp/inc/release_public.pem'
PRIVATE = ROOT/'.release-keys/rlyoulink-private.pem'
HARDWARE = 'xiaos3-8mb-nrst-d5'
LAYOUT = 'amphilink-8mb-v1'
FILES = {'manifest.json', 'manifest.sig', 'application.bin'}

def keys():
    if PRIVATE.exists():
        key = serialization.load_pem_private_key(PRIVATE.read_bytes(), None)
    else:
        PRIVATE.parent.mkdir(exist_ok=True)
        key = rsa.generate_private_key(public_exponent=65537, key_size=3072)
        PRIVATE.write_bytes(key.private_bytes(serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8, serialization.NoEncryption()))
    pub = key.public_key().public_bytes(serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo)
    if PUBLIC.exists() and PUBLIC.read_bytes()!=pub:
        raise ValueError('Existing public key differs; explicit key migration required')
    PUBLIC.write_bytes(pub)
    return key

def verify(path):
    with zipfile.ZipFile(path) as z:
        if len(z.infolist())!=3 or set(z.namelist())!=FILES:
            raise ValueError('Package must contain exactly manifest.json, manifest.sig, application.bin')
        for entry in z.infolist():
            limit=2*1024*1024 if entry.filename=='application.bin' else 2048
            if entry.file_size>limit or entry.flag_bits&1:
                raise ValueError('Invalid package entry')
        raw=z.read('manifest.json');sig=z.read('manifest.sig');app=z.read('application.bin')
    key=serialization.load_pem_public_key(PUBLIC.read_bytes())
    key.verify(sig,raw,padding.PKCS1v15(),hashes.SHA256())
    m=json.loads(raw)
    if (m.get('format')!=1 or m.get('product')!='rlyoulink' or m.get('hardware')!=HARDWARE
        or m.get('layout')!=LAYOUT or m.get('target')!='inactive_ota_slot'
        or m.get('size')!=len(app) or m.get('sha256')!=hashlib.sha256(app).hexdigest()):
        raise ValueError('Incompatible hardware/layout or invalid image hash')
    if len(app)<288 or app[0]!=0xe9 or struct.unpack_from('<H',app,12)[0]!=9:
        raise ValueError('Expected ESP32-S3 application image')
    version=app[48:80].split(b'\0')[0].decode()
    project=app[80:112].split(b'\0')[0].decode()
    if project!='rlyoulink' or version!=m.get('version'):
        raise ValueError('Application descriptor does not match release')
    return m,raw,sig,app

def main():
    p=argparse.ArgumentParser()
    p.add_argument('action',choices=['keys','pack','verify','wire'])
    p.add_argument('path',nargs='?');p.add_argument('--output');p.add_argument('--version')
    args=p.parse_args()
    if args.action=='keys':keys();print('Release key ready; private key excluded from packages');return
    if args.action=='pack':
        key=keys();app=Path(args.path).read_bytes()
        m=dict(format=1,product='rlyoulink',hardware=HARDWARE,layout=LAYOUT,
               target='inactive_ota_slot',version=args.version or app[48:80].split(b'\0')[0].decode(),size=len(app),sha256=hashlib.sha256(app).hexdigest())
        raw=json.dumps(m,sort_keys=True,separators=(',',':')).encode()
        sig=key.sign(raw,padding.PKCS1v15(),hashes.SHA256())
        with zipfile.ZipFile(args.output,'w',zipfile.ZIP_STORED) as z:
            for name,data in [('manifest.json',raw),('manifest.sig',sig),('application.bin',app)]:z.writestr(name,data)
        args.path=args.output
    m,raw,sig,app=verify(args.path)
    if args.action=='wire':
        Path(args.output).write_bytes(b'RLY1'+struct.pack('<II',len(raw),len(sig))+raw+sig+app)
    print(json.dumps(m))

if __name__=='__main__':main()
