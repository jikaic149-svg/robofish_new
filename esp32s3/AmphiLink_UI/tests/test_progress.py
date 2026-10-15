"""Progress regressions using subprocess fixtures and a local HTTP fixture only."""
import base64
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch
from urllib.request import urlopen

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import server
import probe_upgrade
from progress import Progress, PROGRESS
from rlyou_stream import run_stream


class ProgressTests(unittest.TestCase):
    def test_failure_is_not_complete(self):
        p=Progress();p.begin('esp32');p.stage('upload',100)
        self.assertEqual(p.snapshot()['state'],'active')
        p.finish('verification failed')
        self.assertEqual(p.snapshot()['state'],'failed')
        self.assertIsNone(p.snapshot()['percent'])

    def test_stream_before_process_exit(self):
        observed=[];begin=time.monotonic()
        result=run_stream([sys.executable,'-u','-c',
            "import time;print('Writing at 0x220000... (25 %)');time.sleep(1);print('done')"],
            lambda line:observed.append((line,time.monotonic()-begin)))
        self.assertEqual(result.returncode,0)
        self.assertLess(observed[0][1],observed[-1][1]-.5)

    def test_progress_api_not_blocked_by_debugger_lock(self):
        http=server.ThreadingHTTPServer(('127.0.0.1',0),server.Handler)
        thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
        try:
            with patch.object(server,'PORT',http.server_port),server.LOCK:
                PROGRESS.begin('stm32');PROGRESS.stage('write')
                with urlopen(f'http://127.0.0.1:{http.server_port}/api/progress',timeout=1) as response:
                    self.assertEqual(json.load(response)['phase'],'write')
        finally:http.shutdown();http.server_close();thread.join()

    def test_stm32_verify_failure_stops_before_run(self):
        debugger=server.Debugger();debugger.identity={'flash_kib':128}
        debugger.flash_ranges=[(0x08000000,0x08020000)]
        calls=[]
        def command(text,timeout=5):
            calls.append(text)
            if text.startswith('verify_image'):raise RuntimeError('verify failed')
        debugger.command=command
        with self.assertRaises(RuntimeError):
            debugger.flash(dict(name='test.bin',data=base64.b64encode(b'1234').decode(),
                                address='0x08000000',erase='covered',verify=True,run=True))
        self.assertNotIn('reset run',calls)
        self.assertTrue(any(c.startswith('flash write_image erase') for c in calls))

    def test_wifi_upload_content_length_and_bytes(self):
        from http.server import BaseHTTPRequestHandler,HTTPServer
        raw=b'RLY1'+b'x'*40000
        received=[]
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_POST(self):
                received.append(self.rfile.read(int(self.headers['Content-Length'])))
                self.send_response(200);self.end_headers();self.wfile.write(b'{"accepted":true}')
        http=HTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=http.handle_request);thread.start()
        from urllib.request import Request,build_opener,ProxyHandler
        try:
            req=Request(f'http://127.0.0.1:{http.server_port}/',
                data=(raw[i:i+16384] for i in range(0,len(raw),16384)),
                headers={'Content-Length':str(len(raw))})
            with build_opener(ProxyHandler({})).open(req,timeout=3) as response:
                self.assertTrue(json.load(response)['accepted'])
            self.assertEqual(received,[raw])
        finally:thread.join(timeout=3);http.server_close()


if __name__=='__main__':unittest.main()
