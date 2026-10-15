"""Tool selection shared by SWD and USB discovery."""
from functools import lru_cache
import os
from pathlib import Path
import subprocess

OPENOCD_HOME = Path(os.environ.get('ROBOFISH_OPENOCD_HOME', r'D:\OPENOCD\openocd-custom-20260922'))
OPENOCD = OPENOCD_HOME / 'bin/openocd.exe'
SCRIPTS = OPENOCD_HOME / 'share/openocd/scripts'


@lru_cache(maxsize=1)
def info():
    result = {'path': str(OPENOCD), 'scripts': str(SCRIPTS), 'available': OPENOCD.is_file() and SCRIPTS.is_dir()}
    if result['available']:
        try:
            run = subprocess.run([str(OPENOCD), '--version'], capture_output=True, text=True,
                                 timeout=5, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            result['version'] = next((line for line in (run.stdout + run.stderr).splitlines() if 'Open On-Chip' in line), '无法读取版本')
            result['available'] = run.returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            result.update(available=False, version='OpenOCD无法启动')
    return result

_ui_root=Path(__file__).resolve().parent
WORKSPACE=Path(os.environ.get("ROBOFISH_WORKSPACE",_ui_root.parents[1] if _ui_root.parent.name=="esp32s3" else _ui_root.parent))
