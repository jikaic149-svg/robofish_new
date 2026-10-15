"""Opt-in device keys protected by the current Windows user's DPAPI key."""
import base64
import ctypes
from ctypes import wintypes
import json
import os
from pathlib import Path
import threading


def protect(data, decrypt=False):
    if os.name != 'nt':
        raise RuntimeError('加密保存仅支持Windows；请取消保存密钥，以会话方式登录')
    class Blob(ctypes.Structure):
        _fields_ = [('size', wintypes.DWORD), ('data', ctypes.POINTER(ctypes.c_ubyte))]
    memory = ctypes.create_string_buffer(data)
    incoming = Blob(len(data), ctypes.cast(memory, ctypes.POINTER(ctypes.c_ubyte)))
    outgoing = Blob()
    crypt = ctypes.WinDLL('crypt32', use_last_error=True)
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.LocalFree.argtypes = [ctypes.c_void_p]
    kernel.LocalFree.restype = ctypes.c_void_p
    function = crypt.CryptUnprotectData if decrypt else crypt.CryptProtectData
    function.argtypes = [ctypes.POINTER(Blob), ctypes.c_void_p, ctypes.c_void_p,
                         ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(Blob)]
    function.restype = wintypes.BOOL
    if not function(ctypes.byref(incoming), None, None, None, None, 1, ctypes.byref(outgoing)):
        raise RuntimeError('Windows密钥加解密失败，请重新输入密钥')
    try:
        return ctypes.string_at(outgoing.data, outgoing.size)
    finally:
        kernel.LocalFree(outgoing.data)


class KeyStore:
    def __init__(self, path=None):
        self.path = Path(path) if path else Path(os.environ.get('LOCALAPPDATA', Path.home())) / 'RoboFishWorkstation/device-keys.json'
        self.guard = threading.Lock()

    def _read(self):
        if not self.path.exists():
            return {}
        try:
            value = json.loads(self.path.read_text(encoding='utf-8'))
            if not isinstance(value, dict):
                raise ValueError()
            return value
        except (ValueError, OSError):
            raise RuntimeError('本机密钥文件无法读取，请在设置中清除后重新登录')

    def has(self, host):
        with self.guard:
            return host in self._read()

    def get(self, host):
        with self.guard:
            token = self._read().get(host)
        return protect(base64.b64decode(token, validate=True), decrypt=True).decode() if token else ''

    def save(self, host, password):
        if len(password) < 12 or password.isdigit():
            raise RuntimeError('保存设备密钥至少需要12位且不能全为数字；旧设备可取消保存后登录')
        encrypted = base64.b64encode(protect(password.encode())).decode()
        with self.guard:
            values = self._read()
            values[host] = encrypted
            self._write(values)

    def forget(self, host):
        with self.guard:
            values = self._read()
            values.pop(host, None)
            self._write(values)

    def _write(self, values):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(values), encoding='utf-8')
        temporary.replace(self.path)


KEYS = KeyStore()
