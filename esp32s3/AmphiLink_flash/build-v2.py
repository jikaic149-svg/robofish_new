"""Build ESP32-S3 directly from the project, with CMake/Ninja/ESP GCC."""
from pathlib import Path
import os, subprocess, sys

root=Path(__file__).resolve().parent
project=root/'firmware-v2'
workspace=root.parent.parent
build=workspace/'build/esp32s3'
env=os.environ.copy()
env.setdefault('IDF_PATH','D:/ESP32/ESP-IDF-v5.3.2')
env.setdefault('IDF_TOOLS_PATH','D:/ESP32/.espressif')
env.setdefault('IDF_PYTHON_ENV_PATH',str(Path(env['IDF_TOOLS_PATH'])/'python_env/idf5.3_py3.13_env'))
python=Path(env['IDF_PYTHON_ENV_PATH'])/'Scripts/python.exe'
compiler=Path(env['IDF_TOOLS_PATH'])/'tools/xtensa-esp-elf/esp-13.2.0_20240530/xtensa-esp-elf/bin'
env['PATH']=';'.join([str(compiler),str(python.parent),'D:/CMAKE/bin','D:/Ninja','D:/Git/cmd',env['PATH']])
env['IDF_COMPONENT_MANAGER']='0'
build.mkdir(parents=True,exist_ok=True)
commands=[
    ['cmake','-S',str(project),'-B',str(build),'-G','Ninja','-DIDF_TARGET=esp32s3',
     '-DPYTHON='+str(python),'-DCMAKE_MAKE_PROGRAM=D:/Ninja/ninja.exe'],
    ['cmake','--build',str(build),'--parallel','8'],
]
with (build/'build.log').open('w',encoding='utf-8') as log:
    for command in commands:
        result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
        log.write(f'\nProcess exit code: {result.returncode}\n');log.flush()
        if result.returncode:
            print('Build failed; see build/esp32s3/build.log')
            sys.exit(result.returncode)
print('PASS: ESP32-S3 CMake/Ninja/GCC. Outputs: '+str(build))
