"""Select the Windows inbox WinUSB driver for this AmphiLink only."""
import ctypes as c
from ctypes import wintypes as w
import sys
from pathlib import Path

if "--log" in sys.argv:
    sys.stdout = sys.stderr = open(Path(__file__).with_name("driver-install.log"), "w", encoding="utf-8", buffering=1)

INSTANCE = r"USB\VID_303A&PID_83B3\DCB4D93B2B00"
class GUID(c.Structure):
    _fields_ = [("data", c.c_byte * 16)]
class Device(c.Structure):
    _fields_ = [("cbSize", w.DWORD), ("ClassGuid", GUID), ("DevInst", w.DWORD), ("Reserved", c.c_size_t)]
class Params(c.Structure):
    _fields_ = [("cbSize", w.DWORD), ("Flags", w.DWORD), ("FlagsEx", w.DWORD),
                ("hwndParent", w.HWND), ("InstallMsgHandler", c.c_void_p),
                ("InstallMsgHandlerContext", c.c_void_p), ("FileQueue", c.c_void_p),
                ("ClassInstallReserved", c.c_size_t), ("Reserved", w.DWORD), ("DriverPath", w.WCHAR * 260)]
class Driver(c.Structure):
    _fields_ = [("cbSize", w.DWORD), ("DriverType", w.DWORD), ("Reserved", c.c_size_t),
                ("Description", w.WCHAR * 256), ("MfgName", w.WCHAR * 256),
                ("ProviderName", w.WCHAR * 256), ("DriverDate", w.FILETIME), ("DriverVersion", c.c_ulonglong)]
class Detail(c.Structure):
    _fields_ = [("cbSize", w.DWORD), ("InfDate", w.FILETIME), ("CompatIDsOffset", w.DWORD),
                ("CompatIDsLength", w.DWORD), ("Reserved", c.c_size_t),
                ("SectionName", w.WCHAR * 256), ("InfFileName", w.WCHAR * 260),
                ("DrvDescription", w.WCHAR * 256), ("HardwareID", w.WCHAR * 1)]

api = c.WinDLL("setupapi", use_last_error=True)
def bind(name, args, ret=w.BOOL):
    f = getattr(api, name)
    f.argtypes, f.restype = args, ret
    return f
ptr = c.POINTER
create = bind("SetupDiCreateDeviceInfoList", [c.c_void_p, w.HWND], w.HANDLE)
open_device = bind("SetupDiOpenDeviceInfoW", [w.HANDLE, w.LPCWSTR, w.HWND, w.DWORD, ptr(Device)])
get_params = bind("SetupDiGetDeviceInstallParamsW", [w.HANDLE, ptr(Device), ptr(Params)])
set_params = bind("SetupDiSetDeviceInstallParamsW", [w.HANDLE, ptr(Device), ptr(Params)])
build = bind("SetupDiBuildDriverInfoList", [w.HANDLE, ptr(Device), w.DWORD])
enum = bind("SetupDiEnumDriverInfoW", [w.HANDLE, ptr(Device), w.DWORD, w.DWORD, ptr(Driver)])
detail = bind("SetupDiGetDriverInfoDetailW", [w.HANDLE, ptr(Device), ptr(Driver), c.c_void_p, w.DWORD, ptr(w.DWORD)])
select = bind("SetupDiSetSelectedDriverW", [w.HANDLE, ptr(Device), ptr(Driver)])
destroy = bind("SetupDiDestroyDeviceInfoList", [w.HANDLE])
def check(ok):
    if not ok:
        raise c.WinError(c.get_last_error())

handle = create(None, None)
try:
    dev = Device(); dev.cbSize = c.sizeof(dev)
    check(open_device(handle, INSTANCE, None, 0, c.byref(dev)))
    params = Params(); params.cbSize = c.sizeof(params)
    check(get_params(handle, c.byref(dev), c.byref(params)))
    params.Flags |= 0x10000  # DI_ENUMSINGLEINF: inspect only the signed inbox INF.
    params.FlagsEx |= 0x800  # DI_FLAGSEX_ALLOWEXCLUDEDDRVS: expose inbox WinUSB.
    params.DriverPath = r"C:\Windows\INF\winusb.inf"
    check(set_params(handle, c.byref(dev), c.byref(params)))
    check(build(handle, c.byref(dev), 1))
    selected = None
    for index in range(32):
        drv = Driver(); drv.cbSize = c.sizeof(drv)
        if not enum(handle, c.byref(dev), 1, index, c.byref(drv)):
            if c.get_last_error() == 259:
                break
            check(False)
        buf = c.create_string_buffer(8192)
        info = Detail.from_buffer(buf); info.cbSize = c.sizeof(Detail)
        required = w.DWORD()
        check(detail(handle, c.byref(dev), c.byref(drv), buf, len(buf), c.byref(required)))
        print(index, drv.Description, info.SectionName, info.InfFileName, flush=True)
        if info.SectionName.upper() == "WINUSB":
            selected = Driver.from_buffer_copy(drv)
    if selected is None:
        raise RuntimeError("Inbox WINUSB driver section was not found")
    if "--install" in sys.argv:
        check(select(handle, c.byref(dev), c.byref(selected)))
        newdev = c.WinDLL("newdev", use_last_error=True)
        install = newdev.DiInstallDevice
        install.argtypes = [w.HWND, w.HANDLE, ptr(Device), ptr(Driver), w.DWORD, ptr(w.BOOL)]
        install.restype = w.BOOL
        reboot = w.BOOL()
        check(install(None, handle, c.byref(dev), c.byref(selected), 0, c.byref(reboot)))
        print("WinUSB installed; reboot required:", bool(reboot.value))
finally:
    destroy(handle)
