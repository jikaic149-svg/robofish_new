"""Keep the test host alive while checking the actual native browser widgets."""
from pathlib import Path
import json
import os
import runpy
import socket
import subprocess
import sys
import time
import urllib.request

root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'esp32s3/AmphiLink_UI'))
import development

existing=development.state()
if (existing.get('status') or {}).get('state') in ('running','paused') or existing.get('sampling',{}).get('running'):
    raise RuntimeError('Stop this offline check: a real debug/sampling session is active')
try:previous_port=development.descriptor()['port']
except development.DevelopmentError:previous_port=None

with socket.socket() as probe:
    probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
environment=os.environ.copy();environment['ROBOFISH_HOST_PORT']=str(port)
os.environ['ROBOFISH_TEST_URL']=f'http://127.0.0.1:{port}'
(root/'build/workstation').mkdir(parents=True,exist_ok=True)
log=(root/'build/workstation/browser-server.log').open('w',encoding='utf8')
service=subprocess.Popen([sys.executable,'-B','-u',str(root/'esp32s3/AmphiLink_UI/server.py')],env=environment,
    cwd=root/'esp32s3/AmphiLink_UI',stdout=log,stderr=log,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))

try:
    workspace=root/f'build/workstation/ember-verification-{os.getpid()}.code-workspace'
    settings=json.loads((root/'.vscode/settings.json').read_text(encoding='utf8'))
    workspace.write_text(json.dumps({'folders':[{'path':'../..'}],
        'settings':{k:v for k,v in settings.items() if k.startswith('emberprobe.')}},ensure_ascii=False),encoding='utf8')
    subprocess.run([os.environ.get('COMSPEC','cmd.exe'),'/d','/c',r'D:\Microsoft VS Code\bin\code.cmd',
        '--new-window',str(workspace)],check=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    for _ in range(120):
        state=development.state()
        if state.get('webReady') and development.descriptor()['port']!=previous_port:break
        time.sleep(.25)
    else:raise RuntimeError('Native browser host did not become ready')
    if state['status']['state']!='none':raise RuntimeError('This verification requires an idle debugger; no active target is modified')
    values={'elf':str(root/'stm32f405/build/debug/RoboFishF405.elf'),
        'mcu':'robofish/robofish-f405.cfg','debugger':'robofish/robofish-dap-usb.cfg',
        'transport':'swd','rtos':'FreeRTOS','adapterSpeedKhz':100}
    development.task('configure',{'values':values})
    symbols=development.task('list-variables')['result']['symbols']
    if not symbols:raise RuntimeError('No native ELF symbols returned')
    print('Native ELF symbol listing returned',len(symbols),'entries',flush=True)
    for _ in range(100):
        try:
            with urllib.request.urlopen(os.environ['ROBOFISH_TEST_URL']+'/api/health',timeout=.5):break
        except OSError:time.sleep(.1)
    runpy.run_path(str(root/'tools/emberprobe/verify_browser.py'),run_name='__main__')
finally:
    service.terminate();service.wait(timeout=5);log.close()
