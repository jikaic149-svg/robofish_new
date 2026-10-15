"""Local signed package inspection and USB/Wi-Fi installation."""
import atexit,base64,http.cookiejar,ipaddress,json,secrets,subprocess,tempfile,time,sys,re,os
from pathlib import Path
from urllib.request import Request,build_opener,HTTPCookieProcessor,ProxyHandler

ROOT=Path(os.environ.get('ROBOFISH_FLASH_HOME',Path(__file__).resolve().parent.parent/'AmphiLink_flash'))
sys.path.insert(0,str(ROOT))
from rlyou_stream import run_stream
PYTHON='D:/ESP32/.espressif/python_env/idf5.3_py3.13_env/Scripts/python.exe'
TEMP=tempfile.TemporaryDirectory(prefix='rlyoulink-package-')
atexit.register(TEMP.cleanup)
ticket=None
PACKAGE=Path(TEMP.name)/'release.amphipkg'

def command(*args):
    r=subprocess.run([PYTHON,*map(str,args)],capture_output=True,text=True,encoding='utf-8',errors='replace',
                     timeout=180,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    if r.returncode:raise ValueError((r.stdout+'\n'+r.stderr)[-1800:])
    return r.stdout

def inspect(body):
    global ticket
    ticket=None
    if not body.get('name','').lower().endswith('.amphipkg'):raise ValueError('只接受 .amphipkg 正式升级包')
    data=base64.b64decode(body['data'],validate=True)
    if not 0<len(data)<2200000:raise ValueError('升级包大小无效')
    PACKAGE.write_bytes(data)
    manifest=json.loads(command(ROOT/'rlyou_release.py','verify',PACKAGE))
    ticket=secrets.token_hex(16)
    return {'ticket':ticket,'manifest':manifest}

def install(body,log,progress=lambda *a,**k: None):
    global ticket
    if not ticket or body.get('ticket')!=ticket:raise ValueError('请先选择并校验升级包')
    progress('校验升级包签名与型号')
    manifest=json.loads(command(ROOT/'rlyou_release.py','verify',PACKAGE))
    if body.get('transport')=='usb':
        log('USB 升级：查询当前槽、写备用槽并校验，随后切换启动')
        progress('USB 查询设备、进入下载模式')
        phase=['USB 准备']
        def line(text):
            log(text)
            if text.startswith('Writing inactive application slot'):
                phase[0]='USB 写入备用应用槽';progress(phase[0],0)
            elif text.startswith('Image verified; committing'):
                phase[0]='USB 提交启动选择';progress(phase[0])
            elif text.startswith('Waiting for confirmed application'):
                phase[0]='等待 ESP32 重启并确认新版本';progress(phase[0])
            elif text.startswith('Writing at '):
                match=re.search(r'\((\d+)\s*%\)',text)
                if match:progress(phase[0],int(match[1]))
            elif 'Hash of data verified.' in text:
                progress(phase[0]+' · 写入校验通过')
        result=run_stream([PYTHON,'-u',str(ROOT/'rlyou_usb_upgrade.py'),str(PACKAGE)],line,
                          env={**os.environ,'PYTHONIOENCODING':'utf-8','PYTHONUNBUFFERED':'1'})
        if result.returncode:raise ValueError(result.stdout[-1800:])
        ticket=None
        return {'complete':True,'version':manifest['version']}
    if body.get('transport')!='wifi':raise ValueError('不支持的升级通道')
    host=str(ipaddress.IPv4Address(body.get('host','')))
    address=ipaddress.IPv4Address(host)
    if not address.is_private or address.is_loopback or address.is_multicast or address.is_unspecified:
        raise ValueError('请填写局域网探针地址')
    base='http://'+host
    opener=build_opener(ProxyHandler({}),HTTPCookieProcessor(http.cookiejar.CookieJar()))
    def request(path,data=None,kind='application/json',timeout=5):
        req=Request(base+path,data=data,headers={'Content-Type':kind,'Connection':'close'})
        with opener.open(req,timeout=timeout) as r:return json.load(r)
    progress('连接 ESP32 并验证管理密码')
    health=request('/api/v1/health')
    if health.get('firmware')!='rlyoulink' or health.get('hardware')!=manifest['hardware']:
        raise ValueError('设备需要先通过 USB 安装 rlyoulink 基础版本')
    old_slot=health.get('running_slot')
    request('/api/v1/login',json.dumps({'password':body.get('password','')}).encode())
    wire=Path(TEMP.name)/'upload.bin'
    command(ROOT/'rlyou_release.py','wire',PACKAGE,'--output',wire)
    log('Wi-Fi 升级：设备验签、写备用应用槽，请勿断电')
    # Content-Length avoids chunked transfer, which the firmware does not parse.
    # This percentage measures HTTP data sent, not flash bytes acknowledged.
    raw=wire.read_bytes()
    def chunks():
        progress('Wi-Fi 上传升级包',0,'表示发送进度，尚未确认设备写入')
        for offset in range(0,len(raw),16384):
            yield raw[offset:offset+16384]
            progress('Wi-Fi 上传升级包',min(100,(offset+16384)*100//len(raw)),
                     '表示发送进度，尚未确认设备写入')
        progress('等待设备完成验签、写入与校验')
    req=Request(base+'/api/v1/upgrade',data=chunks(),headers={
        'Content-Type':'application/octet-stream','Content-Length':str(len(raw)),'Connection':'close'})
    with opener.open(req,timeout=120) as response:reply=json.load(response)
    if not reply.get('accepted'):raise ValueError('设备没有确认升级提交')
    log('镜像已提交，等待重启并确认新版本')
    progress('等待 ESP32 重启并确认新版本')
    for _ in range(30):
        time.sleep(1)
        try:
            status=request('/api/v1/health',timeout=1)
            if status.get('version')==manifest['version'] and status.get('running_slot')!=old_slot and status.get('ota_confirmed'):
                ticket=None
                return {'complete':True,'version':manifest['version'],'slot':status['running_slot']}
        except Exception:pass
    raise RuntimeError('镜像已提交，但未确认新版本启动；请检查设备状态，勿视为升级成功')
