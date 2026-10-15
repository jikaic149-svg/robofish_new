"""Start the local AmphiLink service if needed, then open the UI in the browser.

Designed to be launched by pythonw.exe through start.cmd, so it must never
require a console. Failures surface as a Windows message box with the tail of
server.log, because a silent failure here looks identical to "nothing happened".
"""
from collections import deque
from pathlib import Path
import json
import os
import subprocess
import sys
import time
import urllib.request
import webbrowser

ROOT = Path(__file__).resolve().parent
URL = 'http://127.0.0.1:8765'
LOG = ROOT / 'server.log'
SERVICE_ID = 'amphilink-local'
BOOT_TIMEOUT_SECONDS = 20.0


def alert(message):
    """Report a startup failure without depending on a console."""
    if os.name == 'nt':
        import ctypes
        ctypes.windll.user32.MessageBoxW(None, message, '机器鱼工作站', 0x10)
    print(message, file=sys.stderr)


def probe():
    """Return the running service identity, or None when nothing answers."""
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(URL + '/api/health', timeout=1) as response:
            return json.load(response)
    except Exception:
        return None


def log_tail(lines=8):
    if not LOG.exists():
        return '(server.log 尚未生成)'
    with LOG.open('r', encoding='utf-8', errors='replace') as handle:
        return ''.join(deque(handle, maxlen=lines)).strip() or '(日志为空)'


def spawn():
    LOG.touch(exist_ok=True)
    # sys.executable is pythonw.exe when started from start.cmd, so the child
    # inherits a windowless interpreter. File handles keep it alive after exit.
    with LOG.open('a', encoding='utf-8') as sink:
        return subprocess.Popen(
            [sys.executable, str(ROOT / 'server.py')], cwd=ROOT,
            stdout=sink, stderr=sink,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))


def main():
    open_browser = '--no-browser' not in sys.argv[1:]
    identity = probe()

    if identity is None:
        process = spawn()
        deadline = time.monotonic() + BOOT_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            identity = probe()
            if identity:
                break
            if process.poll() is not None:
                alert('本地服务启动后立即退出。\n\n最后日志：\n' + log_tail())
                return 1
            time.sleep(0.2)
        else:
            alert(f'本地服务在 {BOOT_TIMEOUT_SECONDS:.0f} 秒内没有就绪。\n\n'
                  '可能原因：端口 8765 被占用、缺少依赖模块、或 openocd 目录不完整。\n\n'
                  '最后日志：\n' + log_tail())
            return 1

    if not isinstance(identity,dict) or identity.get('service') != SERVICE_ID:
        alert('端口 8765 已被另一个程序占用，无法启动上位机本地服务。\n\n'
              '请关闭占用该端口的程序后重试。')
        return 1

    if identity.get('revision') != 'workstation-4' or os.path.normcase(identity.get('instance','')) != os.path.normcase(str(ROOT)):
        alert('本机正在运行旧版或其他目录的上位机。\n\n请先保存实验数据并结束旧服务，再重新双击图标。\n当前入口：'+str(ROOT))
        return 1

    if open_browser:
        webbrowser.open(URL)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
