"""Package the PC workstation and its explicit runtime/debug assets."""
from pathlib import Path
import argparse,hashlib,json,zipfile

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--motion-model',type=Path,
                    help='Verified current DLL when the running service locks its installed DLL')
args=parser.parse_args()
version=json.loads((root/'build/esp32s3/project_description.json').read_text())['project_version']
files={}
def add(path):files[path.relative_to(root).as_posix()]=path.read_bytes()
ui=root/'esp32s3/AmphiLink_UI'
for path in ui.iterdir():
    if path.is_file() and path.suffix in ('.py','.js','.css','.html','.md','.cmd','.ps1','.ico','.dll'):add(path)
if args.motion_model:
    files['esp32s3/AmphiLink_UI/motion_model.dll']=args.motion_model.read_bytes()
for path in (ui/'tests').glob('*.py'):add(path)
flash=root/'esp32s3/AmphiLink_flash'
for name in ['rlyou_stream.py','rlyou_release.py','rlyou_usb_upgrade.py','rlyou_usb_info.py','upgrade_usb.py','restart-v2-rom.py']:
    add(flash/name)
add(flash/'firmware-v2/components/APP/amphilink/tcp/inc/release_public.pem')
import sys
sys.path.insert(0,str(ui))
import tooling
files['esp32s3/AmphiLink_flash/openocd-tcp-fix/package/bin/libusb-1.0.dll']=(tooling.OPENOCD_HOME/'bin/libusb-1.0.dll').read_bytes()
for path in (root/'VISION').rglob('*'):
    if path.is_file() and path.suffix in ('.py','.json','.pt','.md') and not any(part in ('__pycache__','recordings','training') for part in path.parts):add(path)
for path in (root/'common').iterdir():
    if path.is_file() and path.suffix in ('.h','.md'):add(path)
for folder in ['stm32f405/src','stm32f405/stm32f405','stm32f405/cmake','stm32f405/bootloader']:
    for path in (root/folder).rglob('*'):
        if path.is_file() and path.suffix in ('.c','.cpp','.h','.hpp','.s','.S','.ld','.ioc','.cmake','.md'):add(path)
for name in ['CMakeLists.txt','CMakePresets.json','README.md']:add(root/'stm32f405'/name)
for name in ['RoboFishF405.elf','RoboFishF405_B.elf','RoboFishFactory.elf','RoboFishBoot.elf']:
    add(root/'stm32f405/build/debug'/name)
for name in ['build_pc_motion_model.py','motion_model.cpp','package_workstation.py','verify_ui_feedback.py','verify_vision_toolbar.py','verify_vision_sample.py']:add(root/'tools'/name)
add(root/'.vscode/settings.json')
for path in (root/'tools/debug').glob('*.cfg'):add(path)
for path in (root/'tools/emberprobe').rglob('*'):
    if path.suffix=='.vsix' and path.name!='robofish-workstation-bridge-'+json.loads((root/'tools/emberprobe/bridge/package.json').read_text(encoding='utf8'))['version']+'.vsix':continue
    if path.is_file() and path.suffix in ('.py','.js','.json','.txt','.cmd','.vsix'):add(path)
for path in (root/'.agents/skills').rglob('*'):
    if path.is_file() and path.suffix in ('.js','.json','.md') and path.name!='agent-bridge.json':add(path)
for name in ['HOST_WORKSTATION.md','EMBERPROBE_PARITY.md','workstation-preview.png','vision-layout-preview.png','workstation-validation.json']:
    add(root/'docs'/name)
files['启动上位机.cmd']=b'@echo off\r\ncall "%~dp0esp32s3\\AmphiLink_UI\\start.cmd" %*\r\n'
files['README.md']=(ui/'README.md').read_text(encoding='utf8').replace('../../docs/','docs/').encode('utf8')
manifest={'version':version,'ui_revision':'workstation-4','hardware_tested':False,
          'required_openocd':str(tooling.OPENOCD_HOME),'debugger':'BakeSheep.emberprobe 0.8.3 with browser bridge 0.2.1; three backed-up compatibility hooks',
          'artifacts':[{'path':name,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}
                       for name,data in sorted(files.items())]}
files['manifest.json']=(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n').encode()
out=root/'firmware'/('robofish-workstation-'+version+'.zip')
temporary=out.with_suffix('.zip.tmp')
with zipfile.ZipFile(temporary,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
    for name,data in sorted(files.items()):archive.writestr(name,data)
with zipfile.ZipFile(temporary) as archive:
    assert archive.testzip() is None
    assert not any('.release-keys' in name or name.endswith('.log') or '_edge_profile' in name or 'agent-bridge.json' in name or 'ember-host.json' in name or '__pycache__' in name or 'wireless_secrets' in name for name in archive.namelist())
    for entry in manifest['artifacts']:
        assert hashlib.sha256(archive.read(entry['path'])).hexdigest()==entry['sha256']
temporary.replace(out)
digest=hashlib.sha256(out.read_bytes()).hexdigest()
out.with_suffix(out.suffix+'.sha256').write_text(digest+'  '+out.name+'\n',encoding='ascii')
print(json.dumps({'archive':str(out),'bytes':out.stat().st_size,'files':len(files),'sha256':digest},indent=2))
