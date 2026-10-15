"""Install the browser task bridge and version-checked EmberProbe compatibility hooks."""
from pathlib import Path
import hashlib,json,os,shutil,subprocess,sys,zipfile
from compatibility import patch

stage=Path(__file__).parent
root=stage.parents[1]
extension=Path.home()/'.vscode/extensions/bakesheep.emberprobe-0.8.3'
license_dir=root/'tools/emberprobe'
if '--bridge-only' not in sys.argv and '--package-only' not in sys.argv:
    manifest=json.loads((extension/'skills/manifest.json').read_text(encoding='utf-8'))
    destination=root/'.agents/skills'
    for entry in manifest['skills']:
        source=extension/'skills'/entry['name'];target=destination/entry['name']
        for relative in entry['required']:
            original=source/relative;copy=target/relative
            if copy.exists() and copy.read_bytes()!=original.read_bytes():
                raise RuntimeError('保留已有修改，未覆盖：'+str(copy))
            copy.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(original,copy)
        (target/'.emberprobe-skill.json').write_text(json.dumps({'name':entry['name'],'version':entry['version']}),encoding='utf-8')
    for name in manifest['shared']:
        original=extension/'skills/_emberprobe'/name;copy=destination/'_emberprobe'/name
        if copy.exists() and copy.read_bytes()!=original.read_bytes():raise RuntimeError('保留已有SDK修改：'+str(copy))
        copy.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(original,copy)
    license_dir=root/'tools/emberprobe';license_dir.mkdir(parents=True,exist_ok=True)
    shutil.copy2(extension/'LICENSE.txt',license_dir/'LICENSE.txt')
    settings=root/'.vscode/settings.json'
    value=json.loads(settings.read_text(encoding='utf-8')) if settings.exists() else {}
    settings.parent.mkdir(parents=True,exist_ok=True)
    value.update({'emberprobe.openocdPath':r'D:\OPENOCD\openocd-custom-20260922\bin\openocd.exe',
                  'emberprobe.gdbPath':r'D:\GCC\ARMGCC\10 2021.10\bin\arm-none-eabi-gdb.exe',
                  'emberprobe.armToolchainPath':r'D:\GCC\ARMGCC\10 2021.10\bin'})
    settings.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    debug=root/'tools/debug';debug.mkdir(exist_ok=True)
    (debug/'robofish-dap-usb.cfg').write_text('source [find interface/cmsis-dap.cfg]\ncmsis-dap backend usb_bulk\ncmsis-dap vid_pid 0x303a 0x83b3\nadapter serial DCB4D93B2B00\n',encoding='utf-8')
    (debug/'robofish-dap-wifi.cfg').write_text('source [find interface/cmsis-dap.cfg]\ncmsis-dap backend tcp\ncmsis-dap tcp host 192.168.4.1\ncmsis-dap tcp port 4441\n',encoding='utf-8')
    (debug/'robofish-f405.cfg').write_text('''source [find target/stm32f4x.cfg]
    # Avoid the vendor script's unconditional 8 MHz speed during reset.
    $_TARGETNAME configure -event reset-start { adapter speed 100 }
    $_TARGETNAME configure -event reset-init {
        mmw 0xE0042008 0x00001800 0
        if {([mrw 0xE0042008] & 0x00001800) != 0x00001800} {
            error "Watchdog debug freeze was not confirmed"
        }
        adapter speed 100
    }
    ''',encoding='utf-8')
    (debug/'robofish-f405-flash.cfg').write_text('source [file join [file dirname [info script]] robofish-f405.cfg]\nreset_config srst_only srst_nogate connect_assert_srst\n',encoding='utf-8')
version=json.loads((stage/'bridge/package.json').read_text(encoding='utf8'))['version']
out=stage/('robofish-workstation-bridge-'+version+'.vsix')
vsix='''<?xml version="1.0" encoding="utf-8"?><PackageManifest Version="2.0.0" xmlns="http://schemas.microsoft.com/developer/vsx-schema/2011"><Metadata><Identity Language="en-US" Id="workstation-bridge" Version="0.1.0" Publisher="robofish"/><DisplayName>RoboFish Workstation Bridge</DisplayName><Description xml:space="preserve">Task mapping to EmberProbe</Description><Properties><Property Id="Microsoft.VisualStudio.Code.Engine" Value="^1.85.0"/><Property Id="Microsoft.VisualStudio.Code.ExtensionDependencies" Value="BakeSheep.emberprobe"/><Property Id="Microsoft.VisualStudio.Code.ExtensionKind" Value="workspace"/></Properties></Metadata><Installation><InstallationTarget Id="Microsoft.VisualStudio.Code"/></Installation><Dependencies/><Assets><Asset Type="Microsoft.VisualStudio.Code.Manifest" Path="extension/package.json" Addressable="true"/></Assets></PackageManifest>'''
vsix=vsix.replace('0.1.0',version)
content='''<?xml version="1.0" encoding="utf-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="json" ContentType="application/json"/><Default Extension="js" ContentType="application/javascript"/><Default Extension="vsixmanifest" ContentType="text/xml"/></Types>'''
with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('extension.vsixmanifest',vsix);z.writestr('[Content_Types].xml',content)
    for name in ['package.json','extension.js','webviews.js']:z.write(stage/'bridge'/name,'extension/'+name)
if out.resolve()!=(license_dir/out.name).resolve():
    shutil.copy2(out,license_dir/out.name)
if '--package-only' in sys.argv:
    print('Built '+str(out));sys.exit(0)
patch(extension,stage/'engine')
scripts=Path(json.loads((root/'.vscode/settings.json').read_text(encoding='utf8'))['emberprobe.openocdPath']).parent.parent/'share/openocd/scripts'
for folder,names in [('interface',['robofish-dap-usb.cfg','robofish-dap-wifi.cfg']),('target',['robofish-f405.cfg','robofish-f405-flash.cfg'])]:
    destination=scripts/folder/'robofish';destination.mkdir(parents=True,exist_ok=True)
    for name in names:shutil.copy2(root/'tools/debug'/name,destination/name)
code=Path(r'D:\Microsoft VS Code')
subprocess.run([os.environ.get('COMSPEC','cmd.exe'),'/d','/c',str(code/'bin/code.cmd'),'--install-extension',str(out),'--force'],check=True,
               creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
print('Installed the browser bridge and backed-up EmberProbe 0.8.3 compatibility hooks. Reload the host. No hardware task executed.')
