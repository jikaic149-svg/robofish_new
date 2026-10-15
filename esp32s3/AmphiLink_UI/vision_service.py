"""Own the existing VISION process and proxy only its local web interface."""
from collections import deque
import http.client
import json
import os
from pathlib import Path
import secrets
import subprocess
import threading
import time

from tooling import WORKSPACE

TASKS={'START','STOP','CLEAR_PATH','MARKER_ROI','HEAD_DIRECTION','POOL_CALIB','TURN_CALIB',
       'RECORD','SNAPSHOT','CLAHE','EXP_DOWN','EXP_UP','DEBUG','BINARY','EXIT'}


class VisionService:
    def __init__(self):
        self.guard=threading.Lock();self.process=None;self.port=None;self.token=None
        self.logs=deque(maxlen=40);self.source='';self.ready=threading.Event()
        self.model=''

    def configuration(self):
        models=[]
        folder=(WORKSPACE/'VISION/models').resolve()
        for path in sorted(folder.rglob('*.pt')):
            if path.is_file() and path.resolve().is_relative_to(folder):
                models.append({'path':path.relative_to(WORKSPACE).as_posix(),'name':path.name,'bytes':path.stat().st_size})
        profiles=[]
        for key,label,name in [('head','鱼头颜色','head_color_profile.local.json'),('tail','鱼尾颜色','marker_profile.local.json'),('pool','场地','pool_calibration.local.json'),('turn','转弯','turn_calibration.local.json')]:
            path=WORKSPACE/'VISION/configs'/name
            item={'key':key,'label':label,'path':'VISION/configs/'+name,'saved':path.is_file()}
            if path.is_file():
                try:
                    if path.stat().st_size>128*1024:raise ValueError('标定文件过大')
                    item['values']=json.loads(path.read_text(encoding='utf8'))
                except (OSError,ValueError) as error:item['error']=str(error)
            profiles.append(item)
        return {'models':models,'default_model':'VISION/models/fish.pt','calibration':profiles,'output':str(WORKSPACE/'build/vision')}

    def model_path(self,value=None):
        if value is None:value='VISION/models/fish.pt'
        if not isinstance(value,str):raise ValueError('请选择工程内的PT模型')
        path=(WORKSPACE/value).resolve()
        if not path.is_relative_to((WORKSPACE/'VISION/models').resolve()) or path.suffix.lower()!='.pt' or not path.is_file():
            raise ValueError('模型须为VISION/models目录内已有的.pt文件')
        return path

    def open_output(self):
        folder=WORKSPACE/'build/vision';folder.mkdir(parents=True,exist_ok=True)
        if not hasattr(os,'startfile'):raise RuntimeError('当前系统不支持直接打开文件夹')
        os.startfile(str(folder))
        return {'opened':True,'path':str(folder)}

    def start(self,source,parent_token,parent_port,model=None):
        model=self.model_path(model)
        with self.guard:
            if type(source) is int and 0<=source<=9:argument=str(source)
            elif source=='sample':
                samples=Path(r'C:\Users\rlyoulo\Desktop\codeFish(1)\codeFish\VISION')
                sample=next(iter(sorted(samples.glob('auv_record_*.mp4'))),None)
                if not sample:raise RuntimeError('原项目录像不存在')
                argument=str(sample)
            else:raise ValueError('无效视觉源')
            if self.process and self.process.poll() is None:
                if argument!=self.source or str(model)!=self.model:raise RuntimeError('请先停止视觉，再应用新的相机或模型参数')
                return self.state()
            python=Path(os.environ.get('ROBOFISH_VISION_PYTHON',r'D:\python\venvs\yolo\ultralytics\8.4.118\Scripts\python.exe'))
            if not python.is_file():raise RuntimeError('找不到视觉Python环境')
            output=WORKSPACE/'build/vision';output.mkdir(parents=True,exist_ok=True)
            self.token=secrets.token_hex(24);self.port=None;self.ready.clear();self.logs.clear();self.source=argument
            self.model=str(model)
            environment={**os.environ,'ROBOFISH_PARENT_TOKEN':parent_token,'ROBOFISH_PARENT_PORT':str(parent_port),
                'ROBOFISH_VISION_TOKEN':self.token,'ROBOFISH_VISION_OUTPUT':str(output),
                'YOLO_CONFIG_DIR':str(output/'ultralytics'),'MPLCONFIGDIR':str(output/'matplotlib'),
                'PYTHONIOENCODING':'utf-8','PYTHONDONTWRITEBYTECODE':'1'}
            command=[str(python),'-B','-u',str(WORKSPACE/'VISION/web_runtime.py'),'--source',argument,
                     '--model',str(model)]
            self.process=subprocess.Popen(command,cwd=WORKSPACE/'VISION',env=environment,
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            threading.Thread(target=self._reader,args=(self.process,),daemon=True).start()
        self.ready.wait(3)
        return self.state()

    def _reader(self,process):
        for line in process.stdout:
            if line.startswith('ROBOFISH_VISION_READY '):
                value=json.loads(line.split(' ',1)[1]);self.port=value['port'];self.ready.set()
            else:self.logs.append(line.rstrip())

    def call(self,path,body=None,binary=False):
        if not self.process or self.process.poll() is not None or not self.port:
            raise RuntimeError('视觉服务未启动或已退出：'+next(iter(reversed(self.logs)),'暂无日志'))
        connection=http.client.HTTPConnection('127.0.0.1',self.port,timeout=2)
        try:
            connection.request('POST' if body is not None else 'GET',path,json.dumps(body) if body is not None else None,
                {'Authorization':'Bearer '+self.token,'Content-Type':'application/json'})
            response=connection.getresponse();data=response.read(4*1024*1024+1)
            if len(data)>4*1024*1024:raise RuntimeError('视觉响应过大')
            if response.status not in (200,202):raise RuntimeError('视觉接口未就绪')
            return data if binary else json.loads(data)
        finally:connection.close()

    def state(self):
        running=bool(self.process and self.process.poll() is None)
        result={'running':running,'ready':False,'source':self.source,'model':self.model,'logs':list(self.logs),'process_id':self.process.pid if running else None}
        if running and self.port:
            try:result.update(self.call('/state'))
            except (RuntimeError,OSError):pass
        return result

    def stream(self,kind):
        if kind not in ('raw','result') or not self.port or not self.process or self.process.poll() is not None:
            raise RuntimeError('视觉流未就绪')
        connection=http.client.HTTPConnection('127.0.0.1',self.port,timeout=3)
        connection.request('GET','/stream/'+kind,headers={'Authorization':'Bearer '+self.token})
        response=connection.getresponse()
        if response.status!=200:
            connection.close();raise RuntimeError('视觉流未就绪')
        return connection,response

    def action(self,body):
        if not isinstance(body,dict) or (body.get('task') not in TASKS and body.get('kind') not in ('down','move','up','right')):
            raise ValueError('未映射的视觉操作')
        return self.call('/action',body)

    def stop(self):
        if self.process and self.process.poll() is None:
            try:self.action({'task':'EXIT'})
            except (OSError,RuntimeError):pass
            try:self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.process.terminate();self.process.wait(timeout=3)
        self.port=None
        return self.state()


VISION=VisionService()
