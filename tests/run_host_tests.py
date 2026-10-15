"""Run the firmware fault regressions with native GCC and Node; no hardware needed."""
from pathlib import Path
import argparse,os,shutil,subprocess,tempfile

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--compiler',default=os.environ.get('CXX','g++'))
args=parser.parse_args()
compiler=shutil.which(args.compiler) or args.compiler
compiler=Path(compiler).resolve(strict=True)
cc=compiler.with_name(compiler.name.replace('g++','gcc'))
env=os.environ.copy()
env['PATH']=str(compiler.parent)+os.pathsep+env.get('PATH','')
base=['-std=c++17','-O1','-Wall','-Wextra','-Werror','-Icommon']
src='stm32f405/src'
target='stm32f405/stm32f405'
stubs=target+'/tests/stubs'
allocation_sources = [src+'/control/l2/'+name+'.cpp' for name in
                      ['Allocator','tail/Tail','propulsion/Propulsion','pectoral/Pectoral']]
controller_sources = [src+'/task/'+name+'.cpp' for name in
                      ['MotionController','MotionVision','MotionFeedback','MotionCommandInput']]+[
                      src+'/algorithm/PidController.cpp',src+'/control/l0/IntentAdapter.cpp',
                      src+'/control/l0/VisionMotionPolicy.cpp']+allocation_sources
controller_includes = ['tests/control_stubs','tests/host_stubs',stubs,src]
cases=[
 ('motion-allocation',['tests/motion_allocation_test.cpp']+allocation_sources,[src],[]),
 ('motion-command',['tests/motion_command_test.cpp'],[src],[]),
 ('motion-intent',['tests/motion_intent_test.cpp',src+'/control/l0/IntentAdapter.cpp',src+'/control/l0/VisionMotionPolicy.cpp'],[src],[]),
 ('motion-feedback',['tests/motion_feedback_test.cpp']+controller_sources,controller_includes,['-DRF_HAS_NETWORK_LINK=0']),
 ('motion-pipeline',['tests/motion_pipeline_test.cpp']+controller_sources,controller_includes,['-DRF_HAS_NETWORK_LINK=0']),
 ('motion-output',['tests/motion_output_test.cpp',src+'/device/motion/RoboFishDriver.cpp',src+'/device/motion/Propulsion.cpp',src+'/device/motion/TailServos.cpp',src+'/device/motion/PectoralFins.cpp',target+'/bsp/ServoPwm.cpp'],['tests/motion_output_stubs','tests/host_stubs',stubs,target,src],[]),
 ('boot-store',['tests/boot_store_test.cpp',target+'/boot/BootStore.c'],['tests/host_stubs',target],['-DRF_BOOT_HOST_TEST']),
 ('adc',['tests/battery_voltage_test.cpp'],[src],[]),
 ('feedforward',['tests/voltage_feedforward_test.cpp'],[src],[]),
 ('bsp',[target+'/tests/bsp_regression.cpp']+[target+'/bsp/'+name+'.cpp' for name in ['ServoPwm','LoRaUart','RgbStatusStrip']],['tests/host_stubs',stubs,target],[]),
 ('lora',['tests/lora_service_test.cpp',src+'/service/lora/LoRaService.cpp',src+'/service/lora/LoraProtocol.cpp',target+'/bsp/LoRaUart.cpp'],['tests/host_stubs',stubs,target,src],[]),
 ('battery-stop',['tests/battery_stop_test.cpp']+controller_sources,controller_includes,['-DRF_HAS_NETWORK_LINK=0']),
 ('safety-control',['tests/safety_control_test.cpp']+controller_sources,controller_includes,['-DRF_HAS_NETWORK_LINK=0']),
 ('spi-state',['tests/spi_state_test.cpp'],['tests/spi_stubs','tests/host_stubs',target,src,'common'],[]),
]
def run(command):
    subprocess.run([str(arg) for arg in command],cwd=root,env=env,check=True)
with tempfile.TemporaryDirectory(prefix='robofish-host-') as temporary:
    assert Path(temporary).resolve().is_relative_to(Path(tempfile.gettempdir()).resolve())
    for name,sources,includes,flags in cases:
        executable=Path(temporary)/(name+('.exe' if os.name=='nt' else ''))
        run([compiler,*base,*flags,*['-I'+path for path in includes],*sources,'-o',executable])
        factory_images=[root/'stm32f405/build'/configuration/'RoboFishFactory.bin' for configuration in ['debug','release']]
        run([executable,*[path for path in factory_images if name=='boot-store' and path.exists()]])
    executable=Path(temporary)/('spi-protocol.exe' if os.name=='nt' else 'spi-protocol')
    run([cc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-Icommon','tests/spi_protocol_test.c','-o',executable])
    run([executable])
run([os.sys.executable,'tests/control_architecture_test.py'])
run(['node','--check','esp32s3/AmphiLink_flash/firmware-v2/web/robot.js'])
run(['node','tests/robot_web_test.js'])
print(f'PASS: {len(cases)+1} native firmware groups, architecture and webpage checks')
