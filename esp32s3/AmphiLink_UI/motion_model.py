"""Simulation uses the production STM32 allocator, compiled for the PC."""
import ctypes
from pathlib import Path
import subprocess
import sys
import threading


_load_lock=threading.Lock()
_loaded_function=None


def _allocator_sources(root):
    source=root/'stm32f405/src'
    paths=[root/'tools/motion_model.cpp',root/'tools/build_pc_motion_model.py',
           source/'control/l1/Command.h',source/'channel/ActuatorFrame.h',
           source/'channel/MotionTypes.h',source/'algorithm/GaitPhase.h',
           source/'algorithm/PairedLimit.h',
           source/'device/motion/EscCalibration.h']
    for folder in ('control/l0','control/l2','config'):
        paths.extend(path for path in (source/folder).rglob('*')
                     if path.suffix in ('.h','.cpp'))
    return [path for path in paths if path.is_file()]


def _needs_rebuild(library,sources):
    try:
        built_at=library.stat().st_mtime_ns
    except FileNotFoundError:
        return True
    return any(path.stat().st_mtime_ns>built_at for path in sources)


def allocator():
    # Check only before the first load in this process. A running workstation
    # keeps its loaded DLL; updated sources are picked up on its next startup.
    global _loaded_function
    with _load_lock:
        if _loaded_function is not None:
            return _loaded_function
        library_path=Path(__file__).with_name('motion_model.dll')
        root=Path(__file__).resolve().parents[2]
        if _needs_rebuild(library_path,_allocator_sources(root)):
            subprocess.run([sys.executable,'-B','-X','utf8',
                            str(root/'tools/build_pc_motion_model.py')],cwd=root,check=True)
        library=ctypes.CDLL(str(library_path))
        function=library.rf_pc_allocate
        function.argtypes=[ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,
                           ctypes.c_uint,ctypes.POINTER(ctypes.c_float)]
        function.restype=ctypes.c_int
        _loaded_function=function
        return function


def allocate(mode,axes,now_ms):
    values=(ctypes.c_float*6)()
    if not allocator()(mode,axes['surge'],axes['yaw'],axes['heave'],int(now_ms)&0xffffffff,values):
        raise ValueError('无效三轴目标')
    return dict(zip(('tail1','tail2','left_fin','right_fin','left_esc','right_esc'),values))
