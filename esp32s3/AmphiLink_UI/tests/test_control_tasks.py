"""Exercise independent configuration, three-axis goals and vision exit rules."""
import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from station import Workstation
from device_client import DeviceError
from motion_model import allocate
import development

ZERO={'surge':0,'yaw':0,'heave':0}
GO={'surge':35,'yaw':-20,'heave':100}


class ControlTaskTests(unittest.TestCase):
    def setUp(self):
        self.s=Workstation()
        self.s.configure('simulation')

    def test_mode_is_its_own_task_and_never_starts_or_renews_motion(self):
        self.s.configure_control('mode',2,'manual',1)
        self.assertEqual(self.s.safety,'WAIT_COMMAND')
        self.assertEqual(self.s.last_goal,0)
        self.s.target(GO,'manual',2)
        before=self.s.last_goal
        props=self.s.read()['actuators']
        self.assertNotEqual(props['left_esc'],0)
        self.s.configure_control('mode',1,'manual',3)
        fins=self.s.read()['actuators']
        self.assertEqual(self.s.last_goal,before)
        self.assertEqual(fins['left_esc'],0)
        self.assertEqual(fins['right_esc'],0)
        self.s._advance(before+3)
        self.assertEqual(self.s.safety,'SURFACE')

    def test_three_axes_use_production_allocator_and_release_independently(self):
        self.s.configure_control('mode',2,'manual',1)
        self.s.target(GO,'manual',2)
        act=self.s.read()['actuators']
        self.assertAlmostEqual(act['left_esc'],15)
        self.assertAlmostEqual(act['right_esc'],-55)  # Left yaw request, installed electrical polarity.
        self.assertAlmostEqual(act['left_fin'],132)
        self.assertAlmostEqual(act['right_fin'],48)
        self.s.target({**GO,'yaw':0},'manual',3)
        act=self.s.read()['actuators']
        self.assertAlmostEqual(act['left_esc'],35)
        self.assertAlmostEqual(act['right_esc'],-35)
        self.assertEqual(self.s.axes['heave'],100)

    def test_native_preview_has_no_cross_session_state_and_rejects_tail_reverse(self):
        goal={'surge':60,'yaw':20,'heave':40}
        before=allocate(3,goal,1234)
        allocate(2,{'surge':-100,'yaw':-100,'heave':0},40000)
        self.assertEqual(allocate(3,goal,1234),before)
        self.assertGreater(before['left_esc'],abs(before['right_esc']))
        reverse={'surge':-60,'yaw':20,'heave':100}
        pure_tail=allocate(1,reverse,1234)
        self.assertEqual(tuple(pure_tail.values()),(90,90,90,90,0,0))
        hybrid=allocate(3,reverse,1234)
        self.assertEqual((hybrid['tail1'],hybrid['tail2']),(90,90))
        self.assertLess(hybrid['left_esc'],0)
        self.assertGreater(hybrid['right_esc'],0)

    def test_source_selection_does_not_transfer_ownership(self):
        self.s.target(GO,'manual',1)
        lease=self.s.last_goal
        result=self.s.configure_control('source','wifi','manual',2)
        self.assertEqual(result['source'],'wifi')
        self.assertEqual(result['owner'],'lora')
        self.assertEqual(self.s.last_goal,lease)

    def test_stop_rejects_older_goals_including_uint32_wrap(self):
        self.s.target(GO,'manual',0xfffffffe)
        self.s.target(ZERO,'manual',0xffffffff)
        for sequence in (0xfffffffe,0xffffffff):
            with self.assertRaisesRegex(DeviceError,'旧控制'):
                self.s.target(GO,'manual',sequence)
        self.assertEqual(self.s.safety,'WAIT_COMMAND')
        self.s.target(GO,'manual',0)
        self.assertEqual(self.s.safety,'RUNNING')

    def test_manual_stop_exits_vision_until_explicit_fresh_prepare(self):
        self.s.prepare_vision('vision',1)
        self.s.target(GO,'vision',2,'vision')
        self.s.target(ZERO,'manual',1,'lora')
        with self.assertRaisesRegex(DeviceError,'重新启动'):
            self.s.target(GO,'vision',3,'vision')
        self.assertEqual(self.s.safety,'WAIT_COMMAND')
        self.s.prepare_vision('vision',4)
        self.s.target(GO,'vision',5,'vision')
        self.assertEqual(self.s.owner,'vision')

    def test_degradation_keeps_fins_and_invalidates_vision(self):
        self.s.prepare_vision('vision',1)
        self.s.target(GO,'vision',2,'vision')
        before=self.s.read()['actuators']
        self.s.simulate({'lora':False})
        after=self.s.read()
        self.assertEqual(after['safety_mode'],'SURFACE')
        self.assertEqual(after['actuators']['left_fin'],before['left_fin'])
        self.assertEqual(after['actuators']['left_esc'],0)
        self.s.simulate({'lora':True})
        with self.assertRaisesRegex(DeviceError,'重新启动'):
            self.s.target(GO,'vision',3,'vision')

    def test_live_config_does_not_return_simulated_state_or_send_motion(self):
        self.s.configure('mission')
        result=self.s.configure_control('source','vision','manual',1)
        self.assertIsNone(result['mode'])
        self.assertEqual(result['owner'],'unknown')
        self.assertFalse(result['simulated'])
        with self.assertRaisesRegex(DeviceError,'LoRa'):
            self.s.configure_control('mode',1,'manual',2)
        with self.assertRaisesRegex(DeviceError,'LoRa'):
            self.s.target(GO,'manual',3,'lora')

    def test_invalid_axes_do_not_modify_outputs(self):
        for value in (True,float('nan'),float('inf'),101):
            with self.assertRaises(DeviceError):
                self.s.target({**GO,'surge':value},'manual',1)
        self.assertEqual(self.s.safety,'WAIT_COMMAND')


class DevelopmentBridgeTests(unittest.TestCase):
    def test_real_http_forwarding_uses_private_descriptor_and_bearer(self):
        received=[]
        class Handler(BaseHTTPRequestHandler):
            def do_POST(inner):
                received.append((inner.path,inner.headers.get('Authorization'),json.loads(inner.rfile.read(int(inner.headers['Content-Length'])))))
                payload=json.dumps({'task':'threads','result':{'threads':[{'id':7,'name':'MotionControl'}]}}).encode()
                inner.send_response(200);inner.send_header('Content-Length',str(len(payload)));inner.end_headers();inner.wfile.write(payload)
            def log_message(self,*args):pass
        httpd=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        worker=threading.Thread(target=httpd.serve_forever,daemon=True);worker.start()
        try:
            with tempfile.TemporaryDirectory() as folder:
                root=Path(folder);pointer=root/'build/workstation/ember-host.json';pointer.parent.mkdir(parents=True)
                private=root/'private.json'
                private.write_text(json.dumps({'host':'127.0.0.1','port':httpd.server_port,'workspace':str(root),'token':'test-private-token'}))
                pointer.write_text(json.dumps({'descriptorPath':str(private)}))
                with patch.object(development,'ROOT',root):
                    result=development.task('threads')
                    self.assertEqual(result['result']['threads'][0]['id'],7)
                    with self.assertRaises(development.DevelopmentError):development.task('arbitrary-eval')
                self.assertEqual(received,[('/task','Bearer test-private-token',{'task':'threads','params':{}})])
        finally:httpd.shutdown();httpd.server_close();worker.join(2)


if __name__=='__main__':unittest.main()
