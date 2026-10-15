"""Collect already-built public firmware artifacts; never includes release keys."""
from pathlib import Path
import hashlib, json, shutil, zipfile
from datetime import datetime, timezone

root = Path(__file__).resolve().parents[1]
version = json.loads((root/'build/esp32s3/project_description.json').read_text())['project_version']
out = root/'firmware'
out.mkdir(exist_ok=True)
files = []

def copy(source, target):
    source = root/source
    target = out/target
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
    files.append(target)

s3 = Path('build/esp32s3')
flasher = json.loads((root/s3/'flasher_args.json').read_text())
for relative in flasher['flash_files'].values():
    copy(s3/relative, Path('esp32s3')/relative)
for relative in ['flasher_args.json','flash_args','rlyoulink.elf','rlyoulink.map']:
    copy(s3/relative, Path('esp32s3')/relative)
package = 'rlyoulink-xiaos3-d5-'+version+'.amphipkg'
copy(Path('esp32s3/AmphiLink_flash/rlyoulink-release')/version/package,Path('esp32s3')/package)
for configuration in ['release','debug']:
    for image in ['RoboFishF405','RoboFishF405_B','RoboFishBoot']:
        for suffix in ['bin','hex','elf','map']:
            destination = Path('stm32f405')
            if configuration == 'debug': destination /= 'debug'
            filename = image+'.'+suffix
            copy(Path('stm32f405/build')/configuration/filename,destination/filename)
    for filename in ['RoboFishFactory.bin','RoboFishFactory.elf','stm32_ab_manifest.json']:
        destination=Path('stm32f405')/('debug' if configuration=='debug' else '')/filename
        copy(Path('stm32f405/build')/configuration/filename,destination)
copy(Path('stm32f405/stm32f405/openocd.cfg'),Path('stm32f405/openocd.cfg'))
copy(Path('common/README.md'),Path('SPI_PROTOCOL.md'))
copy(Path('common/LORA_PROTOCOL.md'),Path('LORA_PROTOCOL.md'))
copy(Path('README.md'),Path('PROJECT_GUIDE.md'))
copy(Path('docs/DEBUGGING.md'),Path('DEBUGGING.md'))
copy(Path('docs/SAFETY_AND_UPDATE_DESIGN.md'),Path('SAFETY_AND_UPDATE_DESIGN.md'))
copy(Path('docs/HOST_WORKSTATION.md'),Path('HOST_WORKSTATION.md'))
copy(Path('esp32s3/AmphiLink_UI/README.md'),Path('PC_WORKSTATION_README.md'))
copy(Path('validation.json'),Path('validation.json'))
copy(Path('tools/firmware_README.md'),Path('README.md'))
manifest = {
    'version': version,
    'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
    'hardware': {'esp32s3':'xiaos3-8mb-nrst-d5','stm32':'STM32F405, HSE 8MHz'},
    'revision': json.loads((root/'validation.json').read_text(encoding='utf-8'))['revision'],
    'flash': {'esp32s3':flasher['flash_files'],'stm32f405_factory_bin_address':'0x08000000',
              'stm32f405_application_addresses':{'RoboFishF405.bin':'0x08010000','RoboFishF405_B.bin':'0x08080000'}},
    'artifacts': [
        {'path':p.relative_to(out).as_posix(),'bytes':p.stat().st_size,
         'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
        for p in sorted(files)
    ],
}
manifest_path = out/'manifest.json'
manifest_path.write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
files.append(manifest_path)
archive = out/('robofish-firmware-'+version+'.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as bundle:
    for p in sorted(files):
        bundle.write(p,p.relative_to(out).as_posix())
print(json.dumps({'archive':str(archive),'bytes':archive.stat().st_size,
                 'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),
                 'files':len(files)},indent=2))
