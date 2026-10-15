"""Authenticated task remapping to the installed EmberProbe extension host."""
import http.client
import json
import os
from pathlib import Path
import subprocess
import time
import ipaddress
import base64
import hashlib
import re
import xml.etree.ElementTree as ET
from urllib.parse import urlencode

from tooling import WORKSPACE
ROOT=WORKSPACE
TASKS={'connect','stop','pause','continue','step-over','step-in','step-out','restart','flash-plan','flash',
       'open-source','open-waveform','open-settings','threads','stack','scopes','locals',
       'list-variables','read-variables','peripherals','peripheral-read','rtos','memory','breakpoint'}
TASKS.update({'initialize','breakpoints','config','configure','probes','capabilities','chip','fault',
 'variable-write','write-permission-reset','peripheral-write','sampling-status','sampling-start','sampling-stop',
 'watch-add','samples','session-select','verify','launch-debug','memory-read','memory-write-plan','memory-write',
 'console-plan','console-run','settings','settings-save'})


class DevelopmentError(RuntimeError):pass


def descriptor():
    pointer=ROOT/'build/workstation/ember-host.json'
    try:
        location=json.loads(pointer.read_text(encoding='utf-8'))['descriptorPath']
        value=json.loads(Path(location).read_text(encoding='utf-8'))
    except (OSError,ValueError,KeyError):
        raise DevelopmentError('EmberProbe宿主未连接，请启动VS Code中的本项目')
    if value.get('host')!='127.0.0.1' or os.path.normcase(str(Path(value.get('workspace','')).resolve()))!=os.path.normcase(str(ROOT.resolve())) or type(value.get('port')) is not int or not 1<=value['port']<=65535:
        raise DevelopmentError('EmberProbe宿主描述无效，未发送任务')
    return value


def request(route,body=None,timeout=4):
    address=descriptor()
    connection=http.client.HTTPConnection('127.0.0.1',address['port'],timeout=timeout)
    try:
        payload=json.dumps(body).encode() if body is not None else None
        connection.request('POST' if body is not None else 'GET',route,payload,
            {'Authorization':'Bearer '+address['token'],'Content-Type':'application/json'})
        response=connection.getresponse();raw=response.read(16*1024*1024+1)
        if len(raw)>16*1024*1024:raise DevelopmentError('EmberProbe响应过大')
        value=json.loads(raw)
        if response.status!=200:raise DevelopmentError(value.get('error','EmberProbe任务失败'))
        return value
    except (OSError,ValueError) as error:
        raise DevelopmentError('EmberProbe连接中断或超时；任务结果未知，请读取状态核对') from error
    finally:connection.close()


def state():
    try:return request('/state')
    except DevelopmentError as error:return {'engine':'EmberProbe','available':False,'status':None,'error':str(error)}


def active():
    current=state()
    return (current.get('status') or {}).get('state') in ('paused','running') or bool(current.get('sampling',{}).get('running'))


def start_host():
    executable=Path(r'D:\Microsoft VS Code\Code.exe')
    if not executable.is_file():raise DevelopmentError('找不到VS Code，请配置本机调试宿主')
    environment=os.environ.copy()
    environment.pop('ELECTRON_RUN_AS_NODE',None)
    workspace=ROOT/'build/workstation/ember-host.code-workspace'
    settings_path=ROOT/'.vscode/settings.json'
    settings=json.loads(settings_path.read_text(encoding='utf8')) if settings_path.is_file() else {}
    saved=json.loads(workspace.read_text(encoding='utf8')) if workspace.is_file() else {}
    host_settings={key:value for key,value in settings.items() if key.startswith('emberprobe.')}
    host_settings.update(saved.get('settings',{}))
    workspace.parent.mkdir(parents=True,exist_ok=True)
    workspace.write_text(json.dumps({'folders':[{'path':'../..'}],
        'settings':host_settings},ensure_ascii=False),encoding='utf8')
    subprocess.Popen([str(executable),'--new-window',str(workspace)],
                     env=environment,
                     creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0),
                     stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)


def task(name,params=None):
    if name not in TASKS:raise DevelopmentError('未映射的开发任务')
    params=params or {}
    if name=='initialize':
        current=state()
        if current.get('available') and current.get('webReady'):return current
        if (current.get('status') or {}).get('state') in ('paused','running'):
            raise DevelopmentError('当前调试宿主需要重载，请先断开调试，避免建立第二条目标连接')
        start_host()
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            time.sleep(.25);current=state()
            if current.get('available') and current.get('webReady'):return current
        raise DevelopmentError(current.get('error') or '网页引擎未就绪，请信任本工程并重载VS Code宿主')
    if name=='connect':
        if active():raise DevelopmentError('已有EmberProbe调试会话，请先断开')
        transport=params.get('transport','usb')
        if transport not in ('usb','wifi'):raise DevelopmentError('无效探针连接方式')
        if params.get('speed',100) not in (100,500,1000):raise DevelopmentError('无效SWD时钟')
        if transport=='wifi':
            try:address=ipaddress.IPv4Address(params.get('host',''))
            except ValueError:raise DevelopmentError('请输入有效探针IPv4地址')
            if not address.is_private or address.is_loopback or address.is_multicast or address.is_unspecified:
                raise DevelopmentError('请使用局域网探针地址')
            config=ROOT/'tools/debug/robofish-dap-wifi.cfg'
            config.parent.mkdir(parents=True,exist_ok=True)
            config.write_text('source [find interface/cmsis-dap.cfg]\ncmsis-dap backend tcp\ncmsis-dap tcp host '+str(address)+'\ncmsis-dap tcp port 4441\n',encoding='utf-8')
        current=state()
        if not current.get('available'):
            start_host()
            deadline=time.monotonic()+12
            while time.monotonic()<deadline:
                time.sleep(.25)
                current=state()
                if current.get('available'):break
            if not current.get('available'):
                raise DevelopmentError(current.get('error') or 'EmberProbe尚未就绪，请完成VS Code中的工作区初始化')
    return request('/task',{'task':name,'params':params},timeout=160)


SOURCE_ROOTS=('stm32f405/src','stm32f405/stm32f405','stm32f405/bootloader','stm32f405/build/deps','common')
SOURCE_SUFFIXES={'.c','.cpp','.h','.hpp','.s','.S','.ld','.cmake','.ioc'}


def source_file(value):
    if not isinstance(value,str) or len(value)>1024:raise DevelopmentError('源码路径无效')
    path=Path(value)
    if not path.is_absolute():path=ROOT/path
    path=path.resolve()
    if path.suffix not in SOURCE_SUFFIXES or not any(path.is_relative_to((ROOT/folder).resolve()) for folder in SOURCE_ROOTS):
        raise DevelopmentError('源码须在当前STM32工程内')
    if not path.is_file() or path.stat().st_size>2*1024*1024:raise DevelopmentError('源码不存在或超过2 MiB')
    raw=path.read_bytes()
    return {'path':path.relative_to(ROOT).as_posix(),'absolute_path':str(path),'text':raw.decode('utf-8',errors='replace'),
            'sha256':hashlib.sha256(raw).hexdigest()}


def assets():
    sources=[]
    for folder in SOURCE_ROOTS:
        if 'build/deps' in folder:continue
        for path in (ROOT/folder).rglob('*'):
            if path.is_file() and path.suffix in SOURCE_SUFFIXES:sources.append(path.relative_to(ROOT).as_posix())
    elfs=[]
    for folder in ['stm32f405/build/debug','stm32f405/build/release','build/workstation/imports']:
        for path in (ROOT/folder).glob('*.elf'):
            elfs.append({'path':path.relative_to(ROOT).as_posix(),'bytes':path.stat().st_size,'mtime':path.stat().st_mtime})
    targets=sorted(path.relative_to(WORKSPACE_OPENOCD/'target').as_posix() for path in (WORKSPACE_OPENOCD/'target').rglob('*.cfg'))
    interfaces=sorted(path.relative_to(WORKSPACE_OPENOCD/'interface').as_posix() for path in (WORKSPACE_OPENOCD/'interface').rglob('*.cfg'))
    return {'sources':sorted(set(sources)),'elfs':elfs,'targets':targets,'interfaces':interfaces,
            'default_elf':'stm32f405/build/debug/RoboFishF405.elf',
            'factory_elf':'stm32f405/build/debug/RoboFishFactory.elf',
            'github':'https://github.com/BakeSheep/EmberProbe-MCU-Flash-Debug','engine_version':'0.8.3'}


from tooling import SCRIPTS as WORKSPACE_OPENOCD


def import_file(name,data):
    if not isinstance(name,str) or not isinstance(data,str):raise DevelopmentError('文件格式无效')
    suffix=Path(name).suffix.lower()
    if suffix not in {'.elf','.svd','.map','.ld','.ioc'}:raise DevelopmentError('仅允许ELF/SVD/MAP/LD/IOC')
    raw=base64.b64decode(data,validate=True)
    if not raw or len(raw)>4*1024*1024:raise DevelopmentError('文件须为1字节～4 MiB')
    if suffix=='.elf' and (len(raw)<52 or raw[:6]!=b'\x7fELF\x01\x01' or int.from_bytes(raw[18:20],'little')!=40):
        raise DevelopmentError('需要ARM小端ELF32文件')
    if suffix=='.svd':
        if b'<!DOCTYPE' in raw.upper() or b'<!ENTITY' in raw.upper():raise DevelopmentError('SVD不接受DTD或实体声明')
        document=ET.fromstring(raw)
        if document.tag.split('}')[-1]!='device':raise DevelopmentError('SVD根节点须为device')
    if suffix in {'.map','.ld','.ioc'}:
        raw.decode('utf-8')
        if b'\0' in raw:raise DevelopmentError('需要文本文件')
    basename=re.sub(r'[^\w.\-]','_',Path(name).name)[:100]
    digest=hashlib.sha256(raw).hexdigest()
    folder=(ROOT/'build/workstation/imports').resolve();folder.mkdir(parents=True,exist_ok=True)
    output=folder/(digest[:12]+'-'+basename)
    if not output.resolve().is_relative_to(folder):raise DevelopmentError('文件路径无效')
    if not output.exists():output.write_bytes(raw)
    return {'path':output.relative_to(ROOT).as_posix(),'absolute_path':str(output),'bytes':len(raw),'sha256':digest}


def web_document(kind,view_id,port):
    result=request('/web/create',{'kind':kind,'id':view_id,'origin':f'http://127.0.0.1:{port}/'},timeout=12)
    html=result['html']
    if not isinstance(html,str) or len(html)>4*1024*1024:raise DevelopmentError('原生网页内容无效')
    html=re.sub(r'<meta\s+http-equiv=["\']Content-Security-Policy["\'][^>]*>','',html,flags=re.I)
    config=json.dumps({'id':result['id'],'kind':kind,'generation':result.get('generation','')},ensure_ascii=True).replace('<','\\u003c')
    bootstrap='<script>window.__RF_WEBVIEW__='+config+';</script><script src="/ember-webview.js"></script>'
    html=html.replace('<head>','<head>'+bootstrap,1) if '<head>' in html else html.replace('<head ', '<head ',1).replace('<meta',bootstrap+'<meta',1)
    html=html.replace('</head>','<link rel="stylesheet" href="/ember-webview.css"></head>',1)
    return html.encode('utf-8'),result['id']


def web_message(view_id,message):
    if not isinstance(view_id,str) or not re.fullmatch(r'(sidebar|chart-\d+)',view_id):raise DevelopmentError('原生网页ID无效')
    if not isinstance(message,dict):raise DevelopmentError('原生操作无效')
    return request('/web/message',{'id':view_id,'message':message},timeout=180)


def web_connection(route,query):
    address=descriptor()
    connection=http.client.HTTPConnection('127.0.0.1',address['port'],timeout=25)
    connection.request('GET','/web/'+route+'?'+urlencode(query),headers={'Authorization':'Bearer '+address['token']})
    response=connection.getresponse()
    if response.status!=200:
        response.read(65536);connection.close();raise DevelopmentError('原生网页通道未就绪')
    return connection,response
