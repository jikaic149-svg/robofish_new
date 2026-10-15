"""Load the selected PT model on an existing recording, with no device connection."""
from pathlib import Path
import json
import sys
import threading
import time
from http.server import ThreadingHTTPServer
import cv2
import numpy as np

root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'esp32s3/AmphiLink_UI'))
import server
server.STATION.configure('simulation')
httpd=ThreadingHTTPServer(('127.0.0.1',0),server.Handler);server.PORT=httpd.server_port
worker=threading.Thread(target=httpd.serve_forever,daemon=True);worker.start()
try:
    server.VISION.start('sample',server.TOKEN,server.PORT,'VISION/models/fish.pt')
    deadline=time.monotonic()+40
    while time.monotonic()<deadline:
        state=server.VISION.state()
        if state.get('ready') and state.get('result_ready') and state.get('result',{}).get('system',{}).get('yolo_ready'):break
        if not state['running']:raise RuntimeError('Recorded vision exited: '+' '.join(state['logs'][-4:]))
        time.sleep(.25)
    else:raise RuntimeError('Recorded vision/model did not become ready: '+' '.join(state['logs'][-4:]))
    action=server.VISION.action({'task':'BINARY'})
    for _ in range(80):
        state=server.VISION.state()
        if (state.get('last_task') or {}).get('id')==action['id'] and state['last_task']['state']=='processed':break
        time.sleep(.05)
    else:raise RuntimeError('Native display toggle was not processed')
    raw=cv2.imdecode(np.frombuffer(server.VISION.call('/frame/raw',binary=True),np.uint8),cv2.IMREAD_COLOR)
    result=cv2.imdecode(np.frombuffer(server.VISION.call('/frame/result',binary=True),np.uint8),cv2.IMREAD_COLOR)
    assert raw is not None and result is not None
    out=root/'build/workstation';out.mkdir(parents=True,exist_ok=True)
    assert cv2.imwrite(str(out/'vision-native-sample.png'),result)
    report={'passed':True,'source':Path(state['source']).name,'selected_model':'VISION/models/fish.pt','model_ready':True,
            'raw_size':[raw.shape[1],raw.shape[0]],'result_size':[result.shape[1],result.shape[0]],'display_task_processed':True,'hardware_tested':False}
    (out/'vision-sample-validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(report,ensure_ascii=False),flush=True)
finally:
    server.VISION.stop();httpd.shutdown();httpd.server_close();worker.join(3)
