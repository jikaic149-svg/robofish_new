"""Compile the production motion allocator for Windows PC simulation."""
from pathlib import Path
import argparse,os,subprocess

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=root/'esp32s3/AmphiLink_UI/motion_model.dll',
                    help='Alternative DLL destination when a running PC service has loaded the default DLL')
args=parser.parse_args()
compiler=Path(os.environ.get('ROBOFISH_HOST_CXX',r'C:\msys64\ucrt64\bin\g++.exe'))
source=root/'stm32f405/src'
files=['control/l0/IntentAdapter.cpp','control/l2/Allocator.cpp',
       'control/l2/tail/Tail.cpp','control/l2/propulsion/Propulsion.cpp',
       'control/l2/pectoral/Pectoral.cpp']
command=[str(compiler),'-std=c++17','-O2','-shared','-static-libgcc','-static-libstdc++',
         '-I'+str(source),'-I'+str(root/'common'),str(root/'tools/motion_model.cpp')]
command += [str(source/name) for name in files]
command += ['-o',str(args.output.resolve())]
subprocess.run(command,env={**os.environ,'PATH':str(compiler.parent)+';'+os.environ['PATH']},check=True)
print('Built PC motion_model.dll from production source. No firmware was modified.')
