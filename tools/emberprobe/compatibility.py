"""Add three compatibility hooks to pinned EmberProbe 0.8.3; no engine logic fork."""
from pathlib import Path
import hashlib,json,sys

def patch(extension,backup_root):
    if json.loads((extension/'package.json').read_text(encoding='utf8'))['version']!='0.8.3':
        raise RuntimeError('仅支持已核对的EmberProbe 0.8.3')
    source=extension/'dist/extension.js'
    raw=source.read_bytes()
    text=raw.decode('utf8')
    marker='// RoboFish workstation compatibility hooks v1'
    if marker in text:
        metadata=json.loads((backup_root/'compatibility.json').read_text(encoding='utf8'))
        if hashlib.sha256(raw).hexdigest()!=metadata['adapted_sha256']:
            raise RuntimeError('已适配引擎又发生变化，请先核对文件')
        return
    changes=[
      ('      openLiveWatchPanel() {','      openLiveWatchPanel(workstationPanel) {'),
      ('        const panel = vscode2.window.createWebviewPanel(\n          "emberprobe.liveWatch",','        const panel = workstationPanel || vscode2.window.createWebviewPanel(\n          "emberprobe.liveWatch",'),
      ('        this._messageListener = webviewView.webview.onDidReceiveMessage(async (message) => {','        this._messageListener = webviewView.webview.onDidReceiveMessage(this._workstationDispatch = async (message) => {'),
      ('\n}\nasync function deactivate() {','\n  '+marker+'\n  return { workstationProvider: provider };\n}\nasync function deactivate() {')]
    for before,after in changes:
        if text.count(before)!=1:raise RuntimeError('EmberProbe源码接口已变化，停止自动适配：'+before[:70])
        text=text.replace(before,after,1)
    backup_root.mkdir(parents=True,exist_ok=True)
    backup=backup_root/'extension-0.8.3.original.js'
    if backup.exists() and backup.read_bytes()!=raw:raise RuntimeError('原生引擎备份不匹配，保留现有文件')
    backup.write_bytes(raw)
    modified=text.encode('utf8')
    source.write_bytes(modified)
    (backup_root/'compatibility.json').write_text(json.dumps({'version':'0.8.3','original_sha256':hashlib.sha256(raw).hexdigest(),'adapted_sha256':hashlib.sha256(modified).hexdigest(),'hooks':['optional virtual chart panel','reuse original sidebar message handler','export active provider'],'engine_behavior_modified':False},indent=2)+'\n',encoding='utf8')

def restore(extension,backup_root):
    source=extension/'dist/extension.js'
    metadata=json.loads((backup_root/'compatibility.json').read_text(encoding='utf8'))
    original=(backup_root/'extension-0.8.3.original.js').read_bytes()
    if hashlib.sha256(original).hexdigest()!=metadata['original_sha256']:
        raise RuntimeError('原版备份校验失败')
    current=hashlib.sha256(source.read_bytes()).hexdigest()
    if current==metadata['original_sha256']:return
    if current!=metadata['adapted_sha256']:raise RuntimeError('引擎已变化，保留现有文件，未覆盖')
    source.write_bytes(original)

if __name__=='__main__':
    root=Path(__file__).resolve().parents[2]
    extension=Path.home()/'.vscode/extensions/bakesheep.emberprobe-0.8.3'
    operation=restore if '--restore' in sys.argv else patch
    operation(extension,root/'tools/emberprobe/engine')
    print('EmberProbe compatibility '+('restored' if '--restore' in sys.argv else 'installed')+'. Reload the host to activate. No hardware task executed.')
