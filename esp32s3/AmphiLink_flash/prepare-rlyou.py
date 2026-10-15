from pathlib import Path
import hashlib,json,shutil,subprocess,sys
root=Path(__file__).resolve().parent
build=root.parent.parent/'build/esp32s3'
version=json.loads((build/'project_description.json').read_text())['project_version']
out=root/'rlyoulink-release'/version
out.mkdir(parents=True,exist_ok=True)
for source,name in [('rlyoulink.bin','application.bin'),('bootloader/bootloader.bin','bootloader.bin'),('partition_table/partition-table.bin','partition-table.bin')]:
    shutil.copyfile(build/source,out/name)
subprocess.run([sys.executable,str(root/'rlyou_release.py'),'pack',str(out/'application.bin'),
                '--version',version,'--output',str(out/('rlyoulink-xiaos3-d5-'+version+'.amphipkg'))],check=True)
(out/'bootstrap-hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest()
    for p in out.iterdir() if p.suffix in ('.bin','.amphipkg')},indent=2),encoding='utf-8')
print(out)
