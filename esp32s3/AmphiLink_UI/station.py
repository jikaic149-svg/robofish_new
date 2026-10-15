"""PC mission state, explicit bench reads, local simulation and bounded capture.

No real motion is sent over the management HTTP/SPI path. The future LoRa
adapter is intentionally unavailable until its sender/uplink contract exists.
"""
from collections import deque
import csv
from datetime import datetime, timezone
import io
import json
import threading
import time

import device_client
from simulator import Simulator
from motion_model import allocate

LEASE_SECONDS = 3.0
MAX_RECORDS = 30000
FIELDS = ('time', 'source', 'valid', 'battery_v', 'power_pct', 'tail1_deg',
          'tail2_deg', 'left_fin_deg', 'right_fin_deg', 'left_esc_pct',
          'right_esc_pct', 'latency_ms', 'owner', 'safety', 'reason')


def empty_state(source, reason):
    return {'online': False, 'ready': False, 'camera': False, 'checked_at': time.time(),
            'telemetry_source': source, 'reason': reason, 'safety_mode': 'UNKNOWN',
            'owner': 'unknown', 'battery': {'valid': False, 'voltage': None,
            'condition': 'unknown', 'condition_label': '未知'}, 'motion': {},
            'actuators': {}, 'link': {}, 'command': {}}


class Workstation:
    def __init__(self):
        self.guard = threading.RLock()
        self.mode = 'mission'
        self.host = ''
        self.sim = Simulator('', 0, 'local-simulation')
        self.links = {'lora': True, 'wifi': True, 'spi': True, 'upgrading': False}
        self.owner = 'none'
        self.safety = 'WAIT_COMMAND'
        self.reason = '等待新的有效运动指令'
        self.last_goal = 0.0
        self.recording = False
        self.rows = deque(maxlen=MAX_RECORDS)
        self.dropped = 0
        self.capture_error = ''
        self.last_sample = 0.0
        self.generation = 0
        self.control_source = "lora"
        self.axes = None
        self.sequences = {}
        self.control_generation=0
        self.vision_sessions={}

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            time.sleep(.2)
            with self.guard:
                if self.mode == 'simulation':
                    self.sim.tick()
                    self._advance(time.monotonic())
                capture = self.recording and time.monotonic() - self.last_sample >= 1
                if capture:
                    self.last_sample = time.monotonic()
                    generation = self.generation
            if capture:
                try:
                    snapshot = self.read()
                    error = ''
                except device_client.DeviceError as exc:
                    snapshot = empty_state('http_spi', str(exc))
                    error = str(exc)
                with self.guard:
                    if self.recording and generation == self.generation:
                        self.capture_error = error
                        self._append(snapshot)

    def configure(self, mode, host=''):
        if mode not in ('mission', 'bench', 'simulation'):
            raise device_client.DeviceError('无效工作模式')
        if mode == 'bench':
            host = device_client.display_host(host)
        with self.guard:
            if self.recording:
                raise device_client.DeviceError('请先停止数据记录，再切换工作模式或设备')
            self.mode, self.host = mode, host
            self.generation += 1
            if mode == 'simulation':
                self.sim = Simulator('', 0, 'local-simulation')
                self.axes=None;self.control_source='lora';self.sequences.clear();self.control_generation+=1;self.vision_sessions.clear()
                self.links = {'lora': True, 'wifi': True, 'spi': True, 'upgrading': False}
                self.owner, self.safety = 'none', 'WAIT_COMMAND'
                self.reason, self.last_goal = '等待新的有效运动指令', 0.0
        return self.read()

    def _neutral(self, surface=False):
        if self.owner!='none' or self.vision_sessions:
            self.control_generation+=1;self.vision_sessions.clear()
        self.axes = None
        self.sim.left_esc = self.sim.right_esc = self.sim.power = 0
        self.sim.action = 0
        if not surface:
            self.sim.tail1 = self.sim.tail2 = self.sim.left_fin = self.sim.right_fin = 90.0
        self.owner = 'none'
        self.sim.failsafe = surface

    def _advance(self, now):
        battery_bad = self.sim.battery_state() != 0
        if battery_bad:
            self.safety, self.reason = 'BATTERY_STOP', '电池异常，已停机；恢复后须新指令'
            self._neutral()
        elif self.safety == 'BATTERY_STOP':
            self.safety, self.reason = 'WAIT_COMMAND', '电池恢复，等待新的运动指令'
        elif self.safety == 'RUNNING':
            owner_link = self.links['lora'] if self.owner in ('lora', 'vision') else self.links['wifi'] and self.links['spi']
            if not owner_link or now - self.last_goal >= LEASE_SECONDS:
                self.safety, self.reason = 'SURFACE', '控制目标失效：双桨中立、舵机保持，等待接管'
                self._neutral(surface=True)

    def simulate(self, changes):
        with self.guard:
            if self.mode != 'simulation':
                raise device_client.DeviceError('仅测试模式可注入故障')
            if not isinstance(changes, dict) or set(changes) - {'lora', 'wifi', 'spi', 'upgrading', 'battery'}:
                raise device_client.DeviceError('无效仿真设置')
            for key in ('lora', 'wifi', 'spi', 'upgrading'):
                if key in changes:
                    if not isinstance(changes[key], bool):
                        raise device_client.DeviceError('链路状态须为布尔值')
                    self.links[key] = changes[key]
            if 'battery' in changes:
                if changes['battery'] not in ('none', 'low', 'high', 'unavailable'):
                    raise device_client.DeviceError('无效电池状态')
                self.sim.fault = changes['battery']
                self.sim.battery_mv = {'none': 7400, 'low': 6400, 'high': 8600, 'unavailable': 0}[self.sim.fault]
            self._advance(time.monotonic())
        return self.read()

    def command(self, command, source='lora'):
        if source not in ('lora', 'wifi', 'vision') or not isinstance(command, dict):
            raise device_client.DeviceError('无效指令来源或格式')
        kind = command.get('type')
        if type(kind) is not int:
            raise device_client.DeviceError('无效指令类型')
        limits = {'action': (0, 4), 'mode': (0, 3), 'power': (0, 100), 'strength': (0, 100)} if kind == 1 else {'pitch': (0, 180)} if kind == 3 else None
        if not limits or any(type(command.get(k)) is not int or not a <= command[k] <= b for k, (a, b) in limits.items()):
            raise device_client.DeviceError('运动指令参数超出范围')
        with self.guard:
            if self.mode != 'simulation':
                raise device_client.DeviceError('LoRa运动通道尚未接入，指令未发送；HTTP/SPI管理链禁止运动')
            now = time.monotonic()
            self._advance(now)
            stop = kind == 1 and command['action'] == 0
            if not stop and self.sim.battery_state() != 0:
                raise device_client.DeviceError('电池异常，禁止运动')
            if source in ('lora', 'vision') and not self.links['lora']:
                raise device_client.DeviceError('LoRa已断开，指令未发送')
            if source == 'wifi' and (not self.links['wifi'] or not self.links['spi']):
                raise device_client.DeviceError('2.4G备用通道不可用')
            priority = {'none': 0, 'vision': 1, 'wifi': 2, 'lora': 3}
            if self.safety == 'RUNNING' and priority[source] < priority[self.owner]:
                raise device_client.DeviceError('更高优先级控制正在生效，本指令未接管')
            if not stop:
                if source == 'wifi' and self.links['lora']:
                    raise device_client.DeviceError('LoRa链路正常，2.4G仅在LoRa失联后接管')
                if self.safety != 'RUNNING' and (self.links['upgrading'] or not self.links['spi']):
                    raise device_client.DeviceError('升级中或SPI断链，禁止从停止状态启动')
                if kind == 3 and self.safety != 'RUNNING':
                    raise device_client.DeviceError('俯仰调整不能启动运动，请先发送新的运动指令')
            self.sim.command_id += 1
            self.sim.command_ack = self.sim.command_id
            self.sim.command_result = 1
            self.sim.apply_command(command)
            if stop:
                self._neutral()
                self.safety, self.reason = ('WAIT_COMMAND', '已全停，新的运动指令才能重新启动') if self.sim.battery_state() == 0 else ('BATTERY_STOP', '电池异常，已全停')
            else:
                self.owner, self.last_goal, self.safety = source, now, 'RUNNING'
                self.reason = '仿真控制生效（目标有效期3秒，不自动续发）'
            return {'id': self.sim.command_id, 'ack': self.sim.command_ack, 'result': 1,
                    'simulated': True, 'state': 'accepted', 'owner': self.owner}

    def _ordered(self, session, sequence):
        if not isinstance(session, str) or not 1 <= len(session) <= 64 or type(sequence) is not int or not 0 <= sequence <= 0xffffffff:
            raise device_client.DeviceError('控制会话或序号无效')
        previous = self.sequences.get(session)
        if previous is not None and not 0 < ((sequence-previous)&0xffffffff) < 0x80000000:
            raise device_client.DeviceError('旧控制请求已拒绝')
        if previous is None and len(self.sequences) >= 8:
            raise device_client.DeviceError('控制会话已满，请重新进入测试模式')

    def _control_route(self, source, stop=False):
        if self.mode != 'simulation':
            raise device_client.DeviceError('LoRa发送端尚未接入，未向实物发送；管理链不代发运动')
        if source in ('lora', 'vision') and not self.links['lora']:
            raise device_client.DeviceError('LoRa断开，指令未发送')
        if source == 'wifi' and (not self.links['wifi'] or not self.links['spi']):
            raise device_client.DeviceError('2.4G备用通道不可用')
        if self.safety == 'RUNNING' and {'vision':1,'wifi':2,'lora':3}[source] < {'vision':1,'wifi':2,'lora':3}[self.owner]:
            raise device_client.DeviceError('更高优先级控制正在生效，未接管')
        if not stop and source == 'wifi' and self.links['lora']:
            raise device_client.DeviceError('LoRa正常，2.4G不能接管')

    def configure_control(self, action, value, session, sequence):
        with self.guard:
            self._ordered(session, sequence)
            if action == 'source':
                if value not in ('lora','wifi','vision'):
                    raise device_client.DeviceError('无效控制来源')
                self.control_source = value
                message = '已选择控制通道；控制权仍由有效指令与优先级决定'
            elif action == 'mode':
                if type(value) is not int or value not in (1,2,3):
                    raise device_client.DeviceError('无效运动模式')
                self._advance(time.monotonic())
                self._control_route(self.control_source)
                self.sim.mode = value
                self._render_axes(time.monotonic())
                message = '模式已独立应用（仿真），没有启动或续期运动'
            else:
                raise device_client.DeviceError('无效配置操作')
            self.sequences[session] = sequence
            return {'mode':self.sim.mode if self.mode=='simulation' else None,
                    'source':self.control_source,'owner':self.owner if self.mode=='simulation' else 'unknown',
                    'sequence':sequence,'message':message,'simulated':self.mode=='simulation'}

    def _render_axes(self, now):
        if self.axes is None or self.safety != 'RUNNING':
            return
        frame = allocate(self.sim.mode,self.axes,now*1000)
        for key,value in frame.items():
            setattr(self.sim,key,value)
        self.sim.pitch = 90-self.axes['heave']*.9
        self.sim.power = abs(self.axes['surge'])
        self.sim.action = 1 if self.axes['surge']>0 else 2 if self.axes['surge']<0 else 3 if self.axes['yaw']<0 else 4 if self.axes['yaw']>0 else 0

    def prepare_vision(self,session,sequence):
        with self.guard:
            self._ordered(session,sequence)
            self._advance(time.monotonic());self._control_route('vision')
            if self.owner=='vision':
                self._neutral();self.safety='WAIT_COMMAND'
            self.sim.mode=3
            self.vision_sessions[session]=self.control_generation
            self.sequences[session]=sequence
            return {'ready':True,'simulated':True,'generation':self.control_generation}

    def target(self, axes, session, sequence, source=None):
        if not isinstance(axes,dict) or set(axes) != {'surge','yaw','heave'} or any(type(value) not in (float,int) or not -100 <= value <= 100 for value in axes.values()):
            raise device_client.DeviceError('三轴目标须为-100～100的有限数值')
        with self.guard:
            self._ordered(session,sequence)
            now=time.monotonic()
            self._advance(now)
            source=source or self.control_source
            if source not in ('lora','wifi','vision'):
                raise device_client.DeviceError('无效控制来源')
            if source=='vision' and self.vision_sessions.get(session)!=self.control_generation:
                raise device_client.DeviceError('视觉任务已退出或被人工接管，需要重新启动循迹')
            stopped=not any(axes.values())
            self._control_route(source,stopped)
            if not stopped:
                if self.sim.battery_state()!=0:
                    raise device_client.DeviceError('电池异常，禁止运动')
                if self.safety!='RUNNING' and (not self.links['spi'] or self.links['upgrading']):
                    raise device_client.DeviceError('SPI断链或升级中，禁止启动')
                if source!='vision' and self.owner!=source:
                    self.control_generation+=1;self.vision_sessions.clear()
                self.axes=dict(axes)
                self.owner,self.last_goal,self.safety=source,now,'RUNNING'
                self.sim.failsafe=False
                self.reason='三轴目标生效（仿真）'
                self._render_axes(now)
            else:
                self._neutral()
                self.safety='BATTERY_STOP' if self.sim.battery_state()!=0 else 'WAIT_COMMAND'
                self.reason='已全停，等待新的有效目标'
            self.sequences[session]=sequence
            self.sim.command_id+=1
            self.sim.command_ack=self.sim.command_id
            self.sim.command_result=1
            return {'id':self.sim.command_id,'sequence':sequence,'axes':dict(axes),
                    'mode':self.sim.mode,'owner':self.owner,'simulated':True}

    def read(self):
        with self.guard:
            mode, host = self.mode, self.host
            if mode == 'simulation':
                self._advance(time.monotonic())
                self._render_axes(time.monotonic())
                raw = self.sim.robot()
                view = device_client.normalize_robot(raw, 'local-simulation')
                view.update(simulated=True, telemetry_source='simulation', safety_mode=self.safety,
                            owner=self.owner, reason=self.reason, links=dict(self.links),
                            camera_stream_url='', camera_capture_url='', camera=True)
                view['control']={'mode':self.sim.mode,'source':self.control_source,'axes':self.axes or {'surge':0,'yaw':0,'heave':0}}
                return view
        if mode == 'bench':
            view = device_client.CLIENT.robot(host)
            view.update(simulated=False, telemetry_source='http_spi', owner='unknown', safety_mode='UNKNOWN',
                        reason='台架遥测；安全原因和LoRa上行尚未由固件上报')
            view['control']={'mode':view.get('motion',{}).get('mode'),'source':self.control_source,'axes':None}
            return view
        view=empty_state('lora', '找不到LoRa设备：PC发送端与状态上行尚未接入')
        view['control']={'mode':None,'source':self.control_source,'axes':None}
        return view

    def _append(self, view):
        act, battery = view.get('actuators', {}), view.get('battery', {})
        valid = bool(view.get('online'))
        row = {'time': datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
               'source': view['telemetry_source'], 'valid': valid,
               'battery_v': battery.get('voltage') if valid and battery.get('valid') else None,
               'power_pct': view.get('motion', {}).get('power') if valid else None,
               'latency_ms': view.get('link', {}).get('round_trip_us', 0) / 1000 if valid else None,
               'owner': view.get('owner', 'unknown'), 'safety': view.get('safety_mode', 'UNKNOWN'),
               'reason': view.get('reason', '')}
        for key, suffix in (('tail1', '_deg'), ('tail2', '_deg'), ('left_fin', '_deg'),
                            ('right_fin', '_deg'), ('left_esc', '_pct'), ('right_esc', '_pct')):
            row[key + suffix] = act.get(key) if valid else None
        if len(self.rows) == MAX_RECORDS:
            self.dropped += 1
        self.rows.append(row)

    def capture(self, action):
        with self.guard:
            if action == 'start':
                if self.mode == 'mission':
                    raise device_client.DeviceError('LoRa上行尚未接入，不能记录实物任务数据')
                source = 'simulation' if self.mode == 'simulation' else 'http_spi'
                if any(row['source'] != source for row in self.rows):
                    raise device_client.DeviceError('记录来源已变化；请先导出并清空旧记录，避免混合仿真和实物数据')
                self.recording, self.last_sample = True, 0.0
                self.generation += 1
            elif action == 'stop':
                self.recording = False
                self.generation += 1
            elif action == 'clear':
                if self.recording:
                    raise device_client.DeviceError('请先停止记录，再清空数据')
                self.rows.clear()
                self.dropped, self.capture_error = 0, ''
            else:
                raise device_client.DeviceError('无效记录操作')
        return self.data()

    def data(self):
        with self.guard:
            return {'recording': self.recording, 'count': len(self.rows), 'dropped': self.dropped,
                    'capacity': MAX_RECORDS, 'sample_interval_ms': 1000, 'error': self.capture_error,
                    'mode': self.mode, 'rows': list(self.rows)[-600:]}

    def export(self, format):
        with self.guard:
            rows = list(self.rows)
            dropped = self.dropped
        if format == 'json':
            return json.dumps({'schema': 1, 'time_zone': 'UTC', 'dropped': dropped,
                               'units': {'battery_v': 'V', '*_deg': 'deg', '*_pct': '%', 'latency_ms': 'ms'},
                               'samples': rows}, ensure_ascii=False, indent=2).encode(), 'application/json'
        if format != 'csv':
            raise device_client.DeviceError('仅支持CSV或JSON导出')
        output = io.StringIO(newline='')
        writer = csv.DictWriter(output, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: "'" + v if isinstance(v, str) and v.startswith(('=', '+', '-', '@')) else v for k, v in row.items()})
        return output.getvalue().encode('utf-8-sig'), 'text/csv; charset=utf-8'


STATION = Workstation()
